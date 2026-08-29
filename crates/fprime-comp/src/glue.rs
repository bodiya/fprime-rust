//! Command / event / telemetry / parameter / time glue — hand-written
//! equivalents of the autocoded base-class helpers (`regCommands`,
//! `log_<SEVERITY>_<Name>`, `tlmWrite_<Chan>`, `paramGet_*`, `getTime`;
//! analysis: `docs/cpp-analysis/fpp-autocoder.md`).
//!
//! Components embed these blocks as fields and expose the output ports for
//! topology wiring.

use crate::port::{
    CmdRegPort, CmdResponsePort, LogPort, LogTextPort, OutputPort, PrmGetPort, PrmSetPort,
    TimePort, TlmPort,
};
use fprime_config::{FwChanIdType, FwEventIdType, FwIdType, FwOpcodeType, FwPrmIdType};
use fprime_fw::{
    CmdResponse, Endianness, LogBuffer, LogSeverity, ParamBuffer, ParamValid, Serialize,
    SerializeStatus, TextLogString, Time, TlmBuffer, fw_assert,
};
use std::sync::atomic::{AtomicU32, Ordering};

/// C++ `getTime()`: invoke the time port if connected, else return the
/// default (zero) time.
pub fn time_get(port: &OutputPort<dyn TimePort>) -> Time {
    match port.try_get() {
        Some(p) => {
            let mut time = Time::default();
            p.target.invoke(p.port_num, &mut time);
            time
        }
        None => Time::default(),
    }
}

/// Command glue: registration + response output ports.
///
/// Response discipline (documentation, not enforcement — C++ parity):
/// every dispatched command must invoke `cmd_response` **exactly once**
/// (possibly deferred): `FormatError` on argument deserialization failure
/// or residual bytes, `ValidationError` on invalid enum arguments, else the
/// handler's own status.
#[derive(Debug, Default)]
pub struct CmdGlue {
    /// `cmdRegOut` (`Fw.CmdReg`).
    pub cmd_reg_out: OutputPort<dyn CmdRegPort>,
    /// `cmdResponseOut` (`Fw.CmdResponse`).
    pub cmd_response_out: OutputPort<dyn CmdResponsePort>,
}

impl CmdGlue {
    /// New, unconnected.
    pub const fn new() -> Self {
        Self {
            cmd_reg_out: OutputPort::new(),
            cmd_response_out: OutputPort::new(),
        }
    }

    /// C++ `regCommands()`: invokes `cmdRegOut` once per opcode with
    /// `id_base + opcode`. Asserts if the registration port is unconnected
    /// (registration on an unwired port is a topology bug).
    pub fn reg_commands(&self, id_base: FwIdType, opcodes: &[FwOpcodeType]) {
        let p = self.cmd_reg_out.get();
        for &opcode in opcodes {
            p.target.invoke(p.port_num, id_base + opcode);
        }
    }

    /// C++ `cmdResponse_out(...)`: sends the command response (asserts if
    /// unconnected — a dispatched command implies a wired response path).
    pub fn cmd_response(&self, op_code: FwOpcodeType, cmd_seq: u32, response: CmdResponse) {
        let p = self.cmd_response_out.get();
        p.target.invoke(p.port_num, op_code, cmd_seq, response);
    }
}

/// Per-event throttle counter (FPP `throttle N`): the event is emitted
/// while fewer than `limit` emissions have occurred, then suppressed until
/// [`EventThrottle::clear`] (C++ `log_<SEV>_<Name>_ThrottleClear`).
#[derive(Debug)]
pub struct EventThrottle {
    count: AtomicU32,
    limit: u32,
}

impl EventThrottle {
    /// New throttle allowing `limit` emissions.
    pub const fn new(limit: u32) -> Self {
        Self {
            count: AtomicU32::new(0),
            limit,
        }
    }

    /// True (and counts the emission) while under the limit; false once
    /// throttled. C++ parity: `if (m_throttle >= LIMIT) return; m_throttle++`.
    pub fn ok_to_emit(&self) -> bool {
        self.count
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |count| {
                if count >= self.limit {
                    None
                } else {
                    Some(count + 1)
                }
            })
            .is_ok()
    }

    /// Reset the counter — the event flows again.
    pub fn clear(&self) {
        self.count.store(0, Ordering::Relaxed);
    }

    /// Emissions counted so far (saturates at the limit).
    pub fn get_count(&self) -> u32 {
        self.count.load(Ordering::Relaxed)
    }
}

/// Event glue: `logOut` + optional `logTextOut` + the `timeGetOut` port.
#[derive(Debug, Default)]
pub struct EventGlue {
    /// `logOut` (`Fw.Log`) — the binary event path.
    pub log_out: OutputPort<dyn LogPort>,
    /// `logTextOut` (`Fw.LogText`) — optional console text path
    /// (FW_ENABLE_TEXT_LOGGING).
    pub text_log_out: OutputPort<dyn LogTextPort>,
    /// `timeGetOut` (`Fw.Time`).
    pub time_out: OutputPort<dyn TimePort>,
}

impl EventGlue {
    /// New, unconnected.
    pub const fn new() -> Self {
        Self {
            log_out: OutputPort::new(),
            text_log_out: OutputPort::new(),
            time_out: OutputPort::new(),
        }
    }

    /// C++ `getTime()` via this component's time port.
    pub fn time_get(&self) -> Time {
        time_get(&self.time_out)
    }

    /// The autocoded `log_<SEVERITY>_<Name>(...)` body: stamp the time,
    /// serialize the event arguments into a `LogBuffer` via `write_args`
    /// (strings must be truncated by the caller with
    /// `serialize_to_truncated(.., FW_LOG_STRING_MAX_SIZE, ..)` — C++
    /// parity), and invoke `logOut`; when `logTextOut` is connected, also
    /// send the pre-formatted `text` (the caller formats it — the FPP
    /// format string is compile-time knowledge of the component).
    ///
    /// Throttling is the caller's job: guard with
    /// `if self.throttle_x.ok_to_emit() { ... }`.
    ///
    /// Asserts on argument serialization failure (C++ parity: the generated
    /// code FW_ASSERTs each serialize status).
    pub fn log_event<F>(
        &self,
        id_base: FwIdType,
        local_id: FwEventIdType,
        severity: LogSeverity,
        text: &str,
        write_args: F,
    ) where
        F: FnOnce(&mut LogBuffer) -> SerializeStatus,
    {
        let id = id_base + local_id;
        let time = self.time_get();
        if let Some(p) = self.log_out.try_get() {
            let mut args = LogBuffer::new();
            let status = write_args(&mut args);
            fw_assert!(status.is_ok(), status as i32);
            let mut time_tag = time;
            p.target
                .invoke(p.port_num, id, &mut time_tag, severity, &mut args);
        }
        if let Some(p) = self.text_log_out.try_get() {
            // TextLogString truncates silently to FW_LOG_TEXT_BUFFER_SIZE.
            let mut text_buf = TextLogString::from(text);
            let mut time_tag = time;
            p.target
                .invoke(p.port_num, id, &mut time_tag, severity, &mut text_buf);
        }
    }
}

/// Telemetry glue: the `tlmOut` port.
#[derive(Debug, Default)]
pub struct TlmGlue {
    /// `tlmOut` (`Fw.Tlm`).
    pub tlm_out: OutputPort<dyn TlmPort>,
}

impl TlmGlue {
    /// New, unconnected.
    pub const fn new() -> Self {
        Self {
            tlm_out: OutputPort::new(),
        }
    }

    /// The autocoded `tlmWrite_<Chan>(val, timeTag)` body: serialize the
    /// value into a `TlmBuffer` (big-endian) and invoke `tlmOut` with
    /// `id_base + local_id`. No-op when unconnected (C++ parity: guarded by
    /// `isConnected`). Asserts on serialization failure.
    ///
    /// `update on change` suppression is the component's job (compare
    /// before calling).
    pub fn tlm_write(
        &self,
        id_base: FwIdType,
        local_id: FwChanIdType,
        value: &dyn Serialize,
        time_tag: Time,
    ) {
        if let Some(p) = self.tlm_out.try_get() {
            let mut buffer = TlmBuffer::new();
            let status = value.serialize_to(&mut buffer, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
            let mut time_tag = time_tag;
            p.target
                .invoke(p.port_num, id_base + local_id, &mut time_tag, &mut buffer);
        }
    }
}

/// Parameter glue: `prmGetOut` / `prmSetOut` ports.
#[derive(Debug, Default)]
pub struct PrmGlue {
    /// `prmGetOut` (`Fw.PrmGet`).
    pub prm_get_out: OutputPort<dyn PrmGetPort>,
    /// `prmSetOut` (`Fw.PrmSet`).
    pub prm_set_out: OutputPort<dyn PrmSetPort>,
}

impl PrmGlue {
    /// New, unconnected.
    pub const fn new() -> Self {
        Self {
            prm_get_out: OutputPort::new(),
            prm_set_out: OutputPort::new(),
        }
    }

    /// Fetch one parameter (`loadParameters()` body per parameter):
    /// invokes `prmGetOut` with `id_base + local_id`; the component
    /// deserializes `val` on `ParamValid::Valid`/`Default` and falls back
    /// to its FPP default otherwise. Asserts if unconnected (C++
    /// `loadParameters` asserts the port is wired).
    pub fn get_param(
        &self,
        id_base: FwIdType,
        local_id: FwPrmIdType,
        val: &mut ParamBuffer,
    ) -> ParamValid {
        let p = self.prm_get_out.get();
        p.target.invoke(p.port_num, id_base + local_id, val)
    }

    /// Push one parameter to the parameter database (PARAM_SET handler
    /// body). Asserts if unconnected.
    pub fn set_param(&self, id_base: FwIdType, local_id: FwPrmIdType, val: &mut ParamBuffer) {
        let p = self.prm_set_out.get();
        p.target.invoke(p.port_num, id_base + local_id, val);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_config::FwIndexType;
    use fprime_fw::{SerBuf, TimeBase};
    use std::sync::{Arc, Mutex};

    /// (id, time, severity, raw arg bytes)
    type EventRecord = (FwEventIdType, Time, LogSeverity, Vec<u8>);

    #[derive(Default)]
    struct Recorder {
        regs: Mutex<Vec<(FwIndexType, FwOpcodeType)>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: Mutex<Vec<EventRecord>>,
        texts: Mutex<Vec<(FwEventIdType, LogSeverity, String)>>,
        tlm: Mutex<Vec<(FwChanIdType, Time, Vec<u8>)>>,
        prm_sets: Mutex<Vec<(FwPrmIdType, Vec<u8>)>>,
    }

    impl CmdRegPort for Recorder {
        fn invoke(&self, port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push((port_num, op_code));
        }
    }

    impl CmdResponsePort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            op_code: FwOpcodeType,
            cmd_seq: u32,
            response: CmdResponse,
        ) {
            self.responses
                .lock()
                .unwrap()
                .push((op_code, cmd_seq, response));
        }
    }

    impl LogPort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.events
                .lock()
                .unwrap()
                .push((id, *time_tag, severity, args.as_slice().to_vec()));
        }
    }

    impl LogTextPort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            text: &mut TextLogString,
        ) {
            self.texts.lock().unwrap().push((
                id,
                severity,
                text.as_str().unwrap_or_default().to_string(),
            ));
        }
    }

    impl TlmPort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            self.tlm
                .lock()
                .unwrap()
                .push((id, *time_tag, val.as_slice().to_vec()));
        }
    }

    impl PrmGetPort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwPrmIdType,
            val: &mut ParamBuffer,
        ) -> ParamValid {
            // Serve id 0x210 with a u32 value 7; everything else invalid.
            if id == 0x210 {
                val.reset_ser();
                let status = val.serialize_u32_be(7);
                assert!(status.is_ok());
                ParamValid::Valid
            } else {
                ParamValid::Invalid
            }
        }
    }

    impl PrmSetPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) {
            self.prm_sets
                .lock()
                .unwrap()
                .push((id, val.as_slice().to_vec()));
        }
    }

    struct FixedTime;
    impl TimePort for FixedTime {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 100, 2000);
        }
    }

    #[test]
    fn time_get_defaults_when_unconnected() {
        let port: OutputPort<dyn TimePort> = OutputPort::new();
        assert_eq!(time_get(&port), Time::default());
    }

    #[test]
    fn time_get_invokes_connected_port() {
        let port: OutputPort<dyn TimePort> = OutputPort::new();
        port.connect(Arc::new(FixedTime), 0);
        let t = time_get(&port);
        assert_eq!(t.get_seconds(), 100);
        assert_eq!(t.get_useconds(), 2000);
        assert_eq!(t.get_time_base(), TimeBase::TbWorkstationTime);
    }

    #[test]
    fn reg_commands_offsets_each_opcode_by_id_base() {
        let rec = Arc::new(Recorder::default());
        let cmd = CmdGlue::new();
        cmd.cmd_reg_out.connect(rec.clone(), 2);
        cmd.reg_commands(0x100, &[0, 1, 5]);
        assert_eq!(
            *rec.regs.lock().unwrap(),
            vec![(2, 0x100), (2, 0x101), (2, 0x105)]
        );
    }

    #[test]
    fn cmd_response_forwards() {
        let rec = Arc::new(Recorder::default());
        let cmd = CmdGlue::new();
        cmd.cmd_response_out.connect(rec.clone(), 0);
        cmd.cmd_response(0x105, 9, CmdResponse::FormatError);
        assert_eq!(
            *rec.responses.lock().unwrap(),
            vec![(0x105, 9, CmdResponse::FormatError)]
        );
    }

    #[test]
    fn log_event_stamps_time_and_serializes_args() {
        let rec = Arc::new(Recorder::default());
        let evt = EventGlue::new();
        evt.log_out.connect(rec.clone(), 0);
        evt.text_log_out.connect(rec.clone(), 0);
        evt.time_out.connect(Arc::new(FixedTime), 0);
        evt.log_event(0x200, 3, LogSeverity::WarningHi, "value is 42", |buf| {
            buf.serialize_u32_be(42)
        });
        let events = rec.events.lock().unwrap();
        assert_eq!(events.len(), 1);
        let (id, time, severity, args) = &events[0];
        assert_eq!(*id, 0x203);
        assert_eq!(time.get_seconds(), 100);
        assert_eq!(*severity, LogSeverity::WarningHi);
        assert_eq!(args, &[0, 0, 0, 42]);
        let texts = rec.texts.lock().unwrap();
        assert_eq!(texts.len(), 1);
        assert_eq!(
            texts[0],
            (0x203, LogSeverity::WarningHi, "value is 42".into())
        );
    }

    #[test]
    fn log_event_skips_unconnected_ports() {
        let evt = EventGlue::new();
        // Nothing connected: must not assert or invoke anything.
        evt.log_event(0, 0, LogSeverity::ActivityLo, "x", |buf| {
            buf.serialize_u8_be(1)
        });
    }

    #[test]
    fn log_event_text_only_when_binary_unconnected() {
        let rec = Arc::new(Recorder::default());
        let evt = EventGlue::new();
        evt.text_log_out.connect(rec.clone(), 0);
        evt.log_event(0, 1, LogSeverity::Diagnostic, "hello", |buf| {
            buf.serialize_u8_be(1)
        });
        assert!(rec.events.lock().unwrap().is_empty());
        assert_eq!(rec.texts.lock().unwrap().len(), 1);
    }

    #[test]
    fn event_throttle_suppresses_after_limit_until_clear() {
        let throttle = EventThrottle::new(2);
        assert!(throttle.ok_to_emit());
        assert!(throttle.ok_to_emit());
        assert!(!throttle.ok_to_emit());
        assert!(!throttle.ok_to_emit());
        assert_eq!(throttle.get_count(), 2);
        throttle.clear();
        assert!(throttle.ok_to_emit());
        assert_eq!(throttle.get_count(), 1);
    }

    #[test]
    fn event_throttle_zero_limit_never_emits() {
        let throttle = EventThrottle::new(0);
        assert!(!throttle.ok_to_emit());
    }

    #[test]
    fn tlm_write_serializes_value_and_offsets_id() {
        let rec = Arc::new(Recorder::default());
        let tlm = TlmGlue::new();
        tlm.tlm_out.connect(rec.clone(), 1);
        let time = Time::new(TimeBase::TbNone, 0, 5, 6);
        tlm.tlm_write(0x300, 2, &0xAABBu16, time);
        let written = rec.tlm.lock().unwrap();
        assert_eq!(written.len(), 1);
        assert_eq!(written[0].0, 0x302);
        assert_eq!(written[0].1.get_seconds(), 5);
        assert_eq!(written[0].2, vec![0xAA, 0xBB]);
    }

    #[test]
    fn tlm_write_no_op_when_unconnected() {
        let tlm = TlmGlue::new();
        tlm.tlm_write(0, 0, &1u32, Time::default());
    }

    #[test]
    fn prm_glue_round_trips_through_ports() {
        let rec = Arc::new(Recorder::default());
        let prm = PrmGlue::new();
        prm.prm_get_out.connect(rec.clone(), 0);
        prm.prm_set_out.connect(rec.clone(), 0);
        let mut buf = ParamBuffer::new();
        let valid = prm.get_param(0x200, 0x10, &mut buf);
        assert_eq!(valid, ParamValid::Valid);
        assert_eq!(buf.as_slice(), &[0, 0, 0, 7]);
        let valid = prm.get_param(0x200, 0x11, &mut buf);
        assert_eq!(valid, ParamValid::Invalid);
        prm.set_param(0x200, 0x10, &mut buf);
        assert_eq!(
            *rec.prm_sets.lock().unwrap(),
            vec![(0x210, vec![0, 0, 0, 7])]
        );
    }
}
