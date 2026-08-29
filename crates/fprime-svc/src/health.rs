//! # Health — port of `Svc::Health` (queued)
//!
//! C++ sources: `Svc/Health/HealthComponentImpl.{cpp,hpp}`,
//! `Svc/Health/Health.fpp`.
//! Analysis: `docs/cpp-analysis/svc-core.md` (Health section + gotchas).
//!
//! A queued (thread-less) component: async `PingReturn` traffic and async
//! commands only ever execute inside the sync `Run` handler's manual
//! dispatch loop (bounded to `queue_depth` dispatches per rate-group
//! cycle), so all Health logic runs on the rate-group thread.
//!
//! Ping semantics (the documented gotchas):
//! - thresholds fire on exact **equality** (`cycle_count == fatal_cycles`),
//!   FATAL checked BEFORE warn (warn == fatal is legal, produces only the
//!   FATAL), and counters keep incrementing past fatal so each event fires
//!   exactly once per hang;
//! - a wrong-key ping response is a FATAL but does NOT reset the counter —
//!   the entry still marches to `HLTH_PING_LATE`;
//! - the ping key is a single global monotonically incrementing counter.

use fprime_comp::msg;
use fprime_comp::{
    CmdGlue, CmdPort, ComponentDispatch, EventGlue, MsgDispatchStatus, OutputPort, PingPort,
    PortRef, QueueFullPolicy, QueuedBase, SchedPort, TlmGlue, WatchDogPort,
};
use fprime_config::{
    FW_CMD_ARG_BUFFER_MAX_SIZE, FwChanIdType, FwEnumStoreType, FwEventIdType, FwIndexType,
    FwOpcodeType, FwQueuePriorityType, FwSizeType, HEALTH_PING_PORTS,
};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, CmdStringArg, Enabled, Endianness, LinearBuffer, LogSeverity,
    SerBuf, SerBufAny, fw_assert, fw_try,
};
use fprime_os::queue::BlockingType;
use std::sync::{Arc, Mutex};

/// Queue message types (0 is the EXIT sentinel).
const MSG_TYPE_PING_RETURN: FwEnumStoreType = 1;
const MSG_TYPE_CMD_IN: FwEnumStoreType = 2;

/// Queue message size: max over async invocations. The C++ autocoder sizes
/// the message buffer for an async command to hold a FULL `Fw::CmdArgBuffer`
/// (CmdDispatcher forwards an uplinked packet's raw arg bytes unchecked, up
/// to 506): 6 (envelope) + 4 (opCode) + 4 (cmdSeq) + 2 + 506 (length-
/// prefixed `CmdArgBuffer`) = 522. Oversized args then enqueue normally and
/// the handlers' residual-bytes checks answer FORMAT_ERROR (C++ parity).
const MSG_SIZE: usize = 6 + 4 + 4 + 2 + FW_CMD_ARG_BUFFER_MAX_SIZE;

const PING_RETURN_PRIORITY: FwQueuePriorityType = 1;
const CMD_IN_PRIORITY: FwQueuePriorityType = 1;

/// Event string args are FPP `string size 40`.
const EVENT_STRING_SIZE: usize = 40;

/// One ping target (`Svc::Health::PingEntry`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PingEntry {
    /// Cycles before `HLTH_PING_WARN`.
    pub warn_cycles: FwSizeType,
    /// Cycles before the `HLTH_PING_LATE` FATAL.
    pub fatal_cycles: FwSizeType,
    /// Entry name (command lookups compare against this; events truncate
    /// it to the FPP `string size 40`).
    pub name: CmdStringArg,
}

impl PingEntry {
    /// Convenience constructor.
    pub fn new(warn_cycles: FwSizeType, fatal_cycles: FwSizeType, name: &str) -> Self {
        Self {
            warn_cycles,
            fatal_cycles,
            name: CmdStringArg::from(name),
        }
    }
}

/// Per-port tracker (`m_pingTrackerEntries` slots).
#[derive(Debug, Clone, Default)]
struct PingTracker {
    entry: PingEntry,
    cycle_count: FwSizeType,
    key: u32,
    enabled: bool,
}

/// Mutable component state.
struct HealthState {
    trackers: [PingTracker; HEALTH_PING_PORTS],
    num_entries: usize,
    /// `m_key` — global monotonic ping key counter.
    key: u32,
    /// `m_watchDogCode`.
    watchdog_code: u32,
    /// `m_warnings` — `PingLateWarnings` telemetry value.
    warnings: u32,
    /// `m_enabled` — master enable (constructor default ENABLED).
    enabled: Enabled,
    /// Stored at `init`; bounds the per-Run dispatch drain.
    queue_depth: FwSizeType,
}

impl Default for HealthState {
    fn default() -> Self {
        Self {
            trackers: std::array::from_fn(|_| PingTracker::default()),
            num_entries: 0,
            key: 0,
            watchdog_code: 0,
            warnings: 0,
            enabled: Enabled::Enabled,
            queue_depth: 0,
        }
    }
}

/// Deferred sweep emissions (ports are invoked after the state lock is
/// dropped, preserving the C++ per-entry emission order).
enum SweepAction {
    Ping(usize, u32),
    Warn(CmdStringArg, u32),
    Fatal(CmdStringArg),
}

/// `Svc::Health`.
pub struct Health {
    /// Queued core: PassiveBase + message queue (no thread).
    pub queued: QueuedBase,
    /// Command registration + response ports.
    pub cmd: CmdGlue,
    /// Event ports + time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// `PingSend: [25] Svc.Ping` output ports.
    pub ping_send: [OutputPort<dyn PingPort>; HEALTH_PING_PORTS],
    /// `WdogStroke: Svc.WatchDog` output port.
    pub wdog_stroke: OutputPort<dyn WatchDogPort>,
    state: Mutex<HealthState>,
}

impl Health {
    /// Event: `HLTH_PING_WARN(entry)` — WARNING_HI.
    pub const EVENTID_HLTH_PING_WARN: FwEventIdType = 0x0;
    /// Event: `HLTH_PING_LATE(entry)` — FATAL.
    pub const EVENTID_HLTH_PING_LATE: FwEventIdType = 0x1;
    /// Event: `HLTH_PING_WRONG_KEY(entry, badKey)` — FATAL.
    pub const EVENTID_HLTH_PING_WRONG_KEY: FwEventIdType = 0x2;
    /// Event: `HLTH_CHECK_ENABLE(enabled)` — ACTIVITY_HI.
    pub const EVENTID_HLTH_CHECK_ENABLE: FwEventIdType = 0x3;
    /// Event: `HLTH_CHECK_PING(enabled, entry)` — ACTIVITY_HI.
    pub const EVENTID_HLTH_CHECK_PING: FwEventIdType = 0x4;
    /// Event: `HLTH_CHECK_LOOKUP_ERROR(entry)` — WARNING_LO.
    pub const EVENTID_HLTH_CHECK_LOOKUP_ERROR: FwEventIdType = 0x5;
    /// Event: `HLTH_PING_UPDATED(entry, warn, fatal)` — ACTIVITY_HI.
    pub const EVENTID_HLTH_PING_UPDATED: FwEventIdType = 0x6;
    /// Event: `HLTH_PING_INVALID_VALUES(entry, warn, fatal)` — WARNING_HI.
    pub const EVENTID_HLTH_PING_INVALID_VALUES: FwEventIdType = 0x7;

    /// Command: `HLTH_ENABLE(enable: Fw.Enabled)` — async.
    pub const OPCODE_HLTH_ENABLE: FwOpcodeType = 0x0;
    /// Command: `HLTH_PING_ENABLE(entry: string 40, enable: Fw.Enabled)` — async.
    pub const OPCODE_HLTH_PING_ENABLE: FwOpcodeType = 0x1;
    /// Command: `HLTH_CHNG_PING(entry: string 40, warn: U32, fatal: U32)` — async.
    pub const OPCODE_HLTH_CHNG_PING: FwOpcodeType = 0x2;

    /// Telemetry: `PingLateWarnings: U32`.
    pub const CHANID_PING_LATE_WARNINGS: FwChanIdType = 0x0;

    /// Construct (all trackers disabled, checking enabled — C++ ctor).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            queued: QueuedBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            ping_send: [const { OutputPort::new() }; HEALTH_PING_PORTS],
            wdog_stroke: OutputPort::new(),
            state: Mutex::new(HealthState::default()),
        })
    }

    fn id_base(&self) -> u32 {
        self.queued.base.get_id_base()
    }

    /// C++ `init(queueDepth, instance)`: creates the message queue and
    /// stores the depth as the per-Run dispatch bound.
    pub fn init(&self, queue_depth: FwSizeType) {
        self.queued
            .create_queue(queue_depth, MSG_SIZE as FwSizeType);
        self.state.lock().unwrap().queue_depth = queue_depth;
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_HLTH_ENABLE,
                Self::OPCODE_HLTH_PING_ENABLE,
                Self::OPCODE_HLTH_CHNG_PING,
            ],
        );
    }

    /// C++ `setPingEntries(entries, num, watchDogCode)`: asserts
    /// `count <= 25` and `warn <= fatal` per entry; enables the copied
    /// entries with zeroed counters/keys.
    pub fn set_ping_entries(&self, entries: &[PingEntry], watch_dog_code: u32) {
        fw_assert!(entries.len() <= HEALTH_PING_PORTS, entries.len());
        let mut state = self.state.lock().unwrap();
        state.num_entries = entries.len();
        state.watchdog_code = watch_dog_code;
        for (i, entry) in entries.iter().enumerate() {
            fw_assert!(
                entry.warn_cycles <= entry.fatal_cycles,
                entry.warn_cycles,
                entry.fatal_cycles
            );
            state.trackers[i] = PingTracker {
                entry: entry.clone(),
                cycle_count: 0,
                key: 0,
                enabled: true,
            };
        }
    }

    // -- Input-port factories ----------------------------------------------

    /// `Run` — SYNC `Svc.Sched` input (rate-group thread).
    pub fn run_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `PingReturn` — ASYNC `[25] Svc.Ping` input.
    pub fn ping_return_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn PingPort> {
        PortRef::new(Arc::new(PingReturnAdapter { comp: self.clone() }), port_num)
    }

    /// `CmdDisp` — ASYNC command input.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }

    // -- Handlers (all run on the rate-group thread via Run) -----------------

    /// `Run_handler`: drain the message queue (bounded by queue_depth),
    /// sweep the ping table, stroke the watchdog.
    fn run_handler(&self, _port_num: FwIndexType, _context: u32) {
        // Drain own queue: up to queue_depth doDispatch calls, stop at
        // Empty, assert Ok otherwise (C++ parity).
        let queue_depth = self.state.lock().unwrap().queue_depth;
        for _ in 0..queue_depth {
            let status = self.queued.do_dispatch(self, BlockingType::NonBlocking);
            if status == MsgDispatchStatus::Empty {
                break;
            }
            fw_assert!(status == MsgDispatchStatus::Ok, status as i32);
        }

        // Ping sweep under the state lock; port/event emissions deferred
        // until the lock is dropped (order preserved).
        let mut actions: Vec<SweepAction> = Vec::new();
        let watchdog_code = {
            let mut state = self.state.lock().unwrap();
            if state.enabled == Enabled::Enabled {
                for i in 0..state.num_entries {
                    if !state.trackers[i].enabled {
                        continue;
                    }
                    if state.trackers[i].cycle_count == 0 {
                        // Start a ping: key from the global counter.
                        let key = state.key;
                        state.trackers[i].key = key;
                        actions.push(SweepAction::Ping(i, key));
                        state.key = state.key.wrapping_add(1);
                        state.trackers[i].cycle_count += 1;
                    } else {
                        // FATAL first: warn == fatal is legal and produces
                        // only the FATAL. Equality tests fire once each.
                        if state.trackers[i].cycle_count == state.trackers[i].entry.fatal_cycles {
                            let name = state.trackers[i].entry.name.clone();
                            actions.push(SweepAction::Fatal(name));
                        } else if state.trackers[i].cycle_count
                            == state.trackers[i].entry.warn_cycles
                        {
                            state.warnings += 1;
                            let name = state.trackers[i].entry.name.clone();
                            actions.push(SweepAction::Warn(name, state.warnings));
                        }
                        // Counter keeps incrementing past fatal.
                        state.trackers[i].cycle_count += 1;
                    }
                }
            }
            state.watchdog_code
        };

        let id_base = self.id_base();
        for action in actions {
            match action {
                SweepAction::Ping(port, key) => {
                    let p = self.ping_send[port].get();
                    p.target.invoke(p.port_num, key);
                }
                SweepAction::Warn(name, warnings) => {
                    self.evt.log_event(
                        id_base,
                        Self::EVENTID_HLTH_PING_WARN,
                        LogSeverity::WarningHi,
                        &format!("Ping entry {name} late warning"),
                        |buf| name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big),
                    );
                    self.tlm.tlm_write(
                        id_base,
                        Self::CHANID_PING_LATE_WARNINGS,
                        &warnings,
                        self.evt.time_get(),
                    );
                }
                SweepAction::Fatal(name) => {
                    self.evt.log_event(
                        id_base,
                        Self::EVENTID_HLTH_PING_LATE,
                        LogSeverity::Fatal,
                        &format!("Ping entry {name} did not respond"),
                        |buf| name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big),
                    );
                }
            }
        }

        // Always stroke the watchdog when connected.
        if let Some(p) = self.wdog_stroke.try_get() {
            p.target.invoke(p.port_num, watchdog_code);
        }
    }

    /// `PingReturn_handler` (dispatched from the Run drain).
    fn ping_return_handler(&self, port_num: FwIndexType, key: u32) {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < HEALTH_PING_PORTS,
            port_num
        );
        let port = port_num as usize;
        let wrong_key_name = {
            let mut state = self.state.lock().unwrap();
            if key != state.trackers[port].key {
                Some(state.trackers[port].entry.name.clone())
            } else {
                // Reset the counter and clear the key.
                state.trackers[port].cycle_count = 0;
                state.trackers[port].key = 0;
                None
            }
        };
        if let Some(name) = wrong_key_name {
            // Gotcha: the counter is NOT reset — the entry still marches
            // to HLTH_PING_LATE.
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_HLTH_PING_WRONG_KEY,
                LogSeverity::Fatal,
                &format!("Ping entry {name} responded with wrong key 0x{key:x}"),
                |buf| {
                    fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                    buf.serialize_u32_be(key)
                },
            );
        }
    }

    // -- Command handlers ----------------------------------------------------

    /// `findEntry`: name -> tracker index; logs `HLTH_CHECK_LOOKUP_ERROR`
    /// on a miss (C++ parity — the lookup helper itself logs).
    fn find_entry(&self, entry: &CmdStringArg) -> Option<usize> {
        let found = {
            let state = self.state.lock().unwrap();
            (0..state.num_entries).find(|&i| state.trackers[i].entry.name == *entry)
        };
        if found.is_none() {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_HLTH_CHECK_LOOKUP_ERROR,
                LogSeverity::WarningLo,
                &format!("Couldn't find entry {entry}"),
                |buf| entry.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big),
            );
        }
        found
    }

    fn hlth_enable_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut raw = 0u8;
        if !args.deserialize_u8_be(&mut raw).is_ok() || args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let Ok(enable) = Enabled::try_from(raw) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        self.state.lock().unwrap().enabled = enable;
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_HLTH_CHECK_ENABLE,
            LogSeverity::ActivityHi,
            &format!("Health checking set to {}", enabled_str(enable)),
            |buf| buf.serialize_u8_be(enable as u8),
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn hlth_ping_enable_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut entry = CmdStringArg::new();
        let mut raw = 0u8;
        if !args.deserialize(&mut entry, Endianness::Big).is_ok()
            || !args.deserialize_u8_be(&mut raw).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let Ok(enable) = Enabled::try_from(raw) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        let Some(index) = self.find_entry(&entry) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        self.state.lock().unwrap().trackers[index].enabled = enable == Enabled::Enabled;
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_HLTH_CHECK_PING,
            LogSeverity::ActivityHi,
            &format!("Health checking set to {} for {entry}", enabled_str(enable)),
            |buf| {
                fw_try!(buf.serialize_u8_be(enable as u8));
                entry.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn hlth_chng_ping_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut entry = CmdStringArg::new();
        let mut warning_value = 0u32;
        let mut fatal_value = 0u32;
        if !args.deserialize(&mut entry, Endianness::Big).is_ok()
            || !args.deserialize_u32_be(&mut warning_value).is_ok()
            || !args.deserialize_u32_be(&mut fatal_value).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let Some(index) = self.find_entry(&entry) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        if warning_value > fatal_value {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_HLTH_PING_INVALID_VALUES,
                LogSeverity::WarningHi,
                &format!(
                    "Health ping for {entry} invalid values: WARN {warning_value} FATAL {fatal_value}"
                ),
                |buf| {
                    fw_try!(entry.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                    fw_try!(buf.serialize_u32_be(warning_value));
                    buf.serialize_u32_be(fatal_value)
                },
            );
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        {
            let mut state = self.state.lock().unwrap();
            state.trackers[index].entry.warn_cycles = FwSizeType::from(warning_value);
            state.trackers[index].entry.fatal_cycles = FwSizeType::from(fatal_value);
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_HLTH_PING_UPDATED,
            LogSeverity::ActivityHi,
            &format!("Health ping for {entry} changed to WARN {warning_value} FATAL {fatal_value}"),
            |buf| {
                fw_try!(entry.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(warning_value));
                buf.serialize_u32_be(fatal_value)
            },
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }
}

/// Fw.Enabled display text (C++ enum-name formatting).
fn enabled_str(e: Enabled) -> &'static str {
    match e {
        Enabled::Enabled => "ENABLED",
        Enabled::Disabled => "DISABLED",
    }
}

// -- Sync input port (Run) ---------------------------------------------------

impl SchedPort for Health {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        self.run_handler(port_num, context);
    }
}

// -- Async input adapters ----------------------------------------------------

/// `PingReturn` adapter (default/assert policy).
struct PingReturnAdapter {
    comp: Arc<Health>,
}

impl PingPort for PingReturnAdapter {
    fn invoke(&self, port_num: FwIndexType, key: u32) {
        let mut buf = LinearBuffer::<MSG_SIZE>::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_PING_RETURN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(key);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .queued
            .send_message(&buf, PING_RETURN_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// Async command adapter: `[opCode u32][cmdSeq u32][u16 len][args]`.
struct CmdInAdapter {
    comp: Arc<Health>,
}

impl CmdPort for CmdInAdapter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut buf = LinearBuffer::<MSG_SIZE>::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_CMD_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(op_code);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(cmd_seq);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(args, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .queued
            .send_message(&buf, CMD_IN_PRIORITY, QueueFullPolicy::Assert);
    }
}

// -- Dispatch ----------------------------------------------------------------

impl ComponentDispatch for Health {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            MSG_TYPE_PING_RETURN => {
                let mut key = 0u32;
                if !buf.deserialize_u32_be(&mut key).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.ping_return_handler(port_num, key);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_CMD_IN => {
                let mut op_code: FwOpcodeType = 0;
                let mut cmd_seq = 0u32;
                let mut args = CmdArgBuffer::new();
                if !buf.deserialize_u32_be(&mut op_code).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                if !buf.deserialize_u32_be(&mut cmd_seq).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                if !buf.deserialize_buffer(&mut args, Endianness::Big).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                match op_code.wrapping_sub(self.id_base()) {
                    Self::OPCODE_HLTH_ENABLE => {
                        self.hlth_enable_cmd_handler(op_code, cmd_seq, &mut args);
                    }
                    Self::OPCODE_HLTH_PING_ENABLE => {
                        self.hlth_ping_enable_cmd_handler(op_code, cmd_seq, &mut args);
                    }
                    Self::OPCODE_HLTH_CHNG_PING => {
                        self.hlth_chng_ping_cmd_handler(op_code, cmd_seq, &mut args);
                    }
                    _ => self
                        .cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
                }
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort, TlmPort};
    use fprime_fw::{LogBuffer, TextLogString, Time, TlmBuffer};

    const ID_BASE: u32 = 0x600;

    #[derive(Default)]
    struct Ground {
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        texts: Mutex<Vec<(FwEventIdType, String)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        pings: Mutex<Vec<(FwIndexType, u32)>>,
        strokes: Mutex<Vec<u32>>,
    }

    impl LogPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.events
                .lock()
                .unwrap()
                .push((id, severity, args.as_slice().to_vec()));
        }
    }

    impl LogTextPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            text: &mut TextLogString,
        ) {
            self.texts
                .lock()
                .unwrap()
                .push((id, text.as_str().unwrap_or_default().to_string()));
        }
    }

    impl TlmPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            _time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            self.tlm.lock().unwrap().push((id, val.as_slice().to_vec()));
        }
    }

    impl CmdRegPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }

    impl CmdResponsePort for Ground {
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

    impl PingPort for Ground {
        fn invoke(&self, port_num: FwIndexType, key: u32) {
            self.pings.lock().unwrap().push((port_num, key));
        }
    }

    impl WatchDogPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, code: u32) {
            self.strokes.lock().unwrap().push(code);
        }
    }

    const WDOG_CODE: u32 = 0xD06;

    fn build(entries: &[PingEntry], queue_depth: FwSizeType) -> (Arc<Health>, Arc<Ground>) {
        let ground = Arc::new(Ground::default());
        let comp = Health::new("health");
        comp.queued.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.wdog_stroke.connect(ground.clone(), 0);
        for (i, port) in comp.ping_send.iter().take(entries.len()).enumerate() {
            port.connect(ground.clone(), i as FwIndexType);
        }
        comp.init(queue_depth);
        comp.set_ping_entries(entries, WDOG_CODE);
        (comp, ground)
    }

    fn run(comp: &Arc<Health>) {
        let port = comp.run_in(0);
        port.target.invoke(port.port_num, 0);
    }

    fn send_cmd(comp: &Arc<Health>, opcode: FwOpcodeType, seq: u32, arg_bytes: &[u8]) {
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(arg_bytes).is_ok());
        let port = comp.cmd_in(0);
        port.target.invoke(port.port_num, opcode, seq, &mut args);
    }

    /// String-40 command argument wire form: u16 len + bytes.
    fn str_arg(s: &str) -> Vec<u8> {
        let mut v = vec![0, s.len() as u8];
        v.extend_from_slice(s.as_bytes());
        v
    }

    fn event_ids(ground: &Ground) -> Vec<FwEventIdType> {
        ground
            .events
            .lock()
            .unwrap()
            .iter()
            .map(|e| e.0 - ID_BASE)
            .collect()
    }

    #[test]
    fn first_run_pings_all_enabled_entries_with_global_keys() {
        let entries = [
            PingEntry::new(2, 4, "compA"),
            PingEntry::new(2, 4, "compB"),
            PingEntry::new(2, 4, "compC"),
        ];
        let (comp, ground) = build(&entries, 8);
        run(&comp);
        assert_eq!(
            *ground.pings.lock().unwrap(),
            vec![(0, 0), (1, 1), (2, 2)] // global key counter 0,1,2
        );
        assert_eq!(*ground.strokes.lock().unwrap(), vec![WDOG_CODE]);
    }

    #[test]
    fn ping_return_with_correct_key_resets_and_repings() {
        let entries = [PingEntry::new(2, 4, "compA")];
        let (comp, ground) = build(&entries, 8);
        run(&comp); // ping key 0
        let ret = comp.ping_return_in(0);
        ret.target.invoke(ret.port_num, 0); // correct key, queued
        run(&comp); // drain resets counter -> new ping with key 1
        assert_eq!(*ground.pings.lock().unwrap(), vec![(0, 0), (0, 1)]);
        assert!(ground.events.lock().unwrap().is_empty());
    }

    #[test]
    fn warn_then_fatal_fire_on_exact_equality_once_each() {
        let entries = [PingEntry::new(2, 4, "compA")];
        let (comp, ground) = build(&entries, 8);
        // run1: count 0 -> ping, count=1
        // run2: count 1 -> nothing, count=2
        // run3: count==2==warn -> WARN + tlm, count=3
        // run4: count 3 -> nothing, count=4
        // run5: count==4==fatal -> FATAL, count=5
        // run6..: count past fatal -> nothing (events fire exactly once)
        for _ in 0..8 {
            run(&comp);
        }
        assert_eq!(
            event_ids(&ground),
            vec![
                Health::EVENTID_HLTH_PING_WARN,
                Health::EVENTID_HLTH_PING_LATE
            ]
        );
        let events = ground.events.lock().unwrap();
        assert_eq!(events[0].1, LogSeverity::WarningHi);
        assert_eq!(events[1].1, LogSeverity::Fatal);
        // String-40 arg: u16 len + bytes.
        assert_eq!(events[0].2, str_arg("compA"));
        drop(events);
        assert_eq!(
            ground.texts.lock().unwrap()[0].1,
            "Ping entry compA late warning"
        );
        assert_eq!(
            ground.texts.lock().unwrap()[1].1,
            "Ping entry compA did not respond"
        );
        // One PingLateWarnings write, value 1.
        assert_eq!(
            *ground.tlm.lock().unwrap(),
            vec![(
                ID_BASE + Health::CHANID_PING_LATE_WARNINGS,
                vec![0, 0, 0, 1]
            )]
        );
        // Only the initial ping was sent — the entry never recovered.
        assert_eq!(ground.pings.lock().unwrap().len(), 1);
    }

    #[test]
    fn warn_equal_fatal_produces_only_the_fatal() {
        let entries = [PingEntry::new(3, 3, "compA")];
        let (comp, ground) = build(&entries, 8);
        for _ in 0..6 {
            run(&comp);
        }
        assert_eq!(event_ids(&ground), vec![Health::EVENTID_HLTH_PING_LATE]);
    }

    #[test]
    fn wrong_key_is_fatal_and_does_not_reset_the_counter() {
        let entries = [PingEntry::new(2, 3, "compA")];
        let (comp, ground) = build(&entries, 8);
        run(&comp); // ping key 0, count=1
        let ret = comp.ping_return_in(0);
        ret.target.invoke(ret.port_num, 0xBAD); // wrong key
        // run2: drain logs WRONG_KEY FATAL; sweep: count 1 -> count=2
        // run3: count==2==warn -> WARN; count=3
        // run4: count==3==fatal -> LATE FATAL
        for _ in 0..4 {
            run(&comp);
        }
        assert_eq!(
            event_ids(&ground),
            vec![
                Health::EVENTID_HLTH_PING_WRONG_KEY,
                Health::EVENTID_HLTH_PING_WARN,
                Health::EVENTID_HLTH_PING_LATE,
            ]
        );
        let events = ground.events.lock().unwrap();
        assert_eq!(events[0].1, LogSeverity::Fatal);
        let mut expected = str_arg("compA");
        expected.extend_from_slice(&[0x00, 0x00, 0x0B, 0xAD]);
        assert_eq!(events[0].2, expected);
        drop(events);
        assert_eq!(
            ground.texts.lock().unwrap()[0].1,
            "Ping entry compA responded with wrong key 0xbad"
        );
    }

    #[test]
    fn queue_drain_is_bounded_by_queue_depth() {
        let (comp, ground) = build(&[], 8);
        // Shrink the drain bound below the queue capacity to observe it.
        comp.state.lock().unwrap().queue_depth = 2;
        let opcode = ID_BASE + Health::OPCODE_HLTH_ENABLE;
        for seq in 0..4 {
            send_cmd(&comp, opcode, seq, &[Enabled::Enabled as u8]);
        }
        run(&comp);
        assert_eq!(ground.responses.lock().unwrap().len(), 2);
        run(&comp);
        assert_eq!(ground.responses.lock().unwrap().len(), 4);
    }

    #[test]
    fn hlth_enable_disable_stops_pinging_but_still_strokes_watchdog() {
        let entries = [PingEntry::new(2, 4, "compA")];
        let (comp, ground) = build(&entries, 8);
        let opcode = ID_BASE + Health::OPCODE_HLTH_ENABLE;
        send_cmd(&comp, opcode, 1, &[Enabled::Disabled as u8]);
        // The command drains BEFORE the sweep, so no ping at all.
        run(&comp);
        run(&comp);
        assert!(ground.pings.lock().unwrap().is_empty());
        assert_eq!(*ground.strokes.lock().unwrap(), vec![WDOG_CODE, WDOG_CODE]);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(opcode, 1, CmdResponse::Ok)]
        );
        assert_eq!(event_ids(&ground), vec![Health::EVENTID_HLTH_CHECK_ENABLE]);
        let events = ground.events.lock().unwrap();
        assert_eq!(events[0].2, vec![Enabled::Disabled as u8]);
        drop(events);
        assert_eq!(
            ground.texts.lock().unwrap()[0].1,
            "Health checking set to DISABLED"
        );
    }

    #[test]
    fn hlth_enable_rejects_invalid_enum_and_residual_bytes() {
        let (comp, ground) = build(&[], 8);
        let opcode = ID_BASE + Health::OPCODE_HLTH_ENABLE;
        send_cmd(&comp, opcode, 1, &[7]); // invalid Enabled value
        send_cmd(&comp, opcode, 2, &[1, 9]); // residual byte
        send_cmd(&comp, opcode, 3, &[]); // short
        run(&comp);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (opcode, 1, CmdResponse::ValidationError),
                (opcode, 2, CmdResponse::FormatError),
                (opcode, 3, CmdResponse::FormatError),
            ]
        );
    }

    /// Regression: a command carrying a FULL `CmdArgBuffer` (506 bytes —
    /// what CmdDispatcher can forward from a malformed uplink) must fit the
    /// queue message (no assert on the dispatching thread) and answer
    /// FORMAT_ERROR from the handler's residual-bytes check, C++ parity
    /// (the autocoded queue message is sized for a full arg buffer).
    #[test]
    fn full_cmd_arg_buffer_enqueues_and_answers_format_error() {
        let (comp, ground) = build(&[], 8);
        let opcode = ID_BASE + Health::OPCODE_HLTH_ENABLE;
        let args = [0xABu8; FW_CMD_ARG_BUFFER_MAX_SIZE];
        send_cmd(&comp, opcode, 9, &args); // must not panic
        run(&comp);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(opcode, 9, CmdResponse::FormatError)]
        );
    }

    #[test]
    fn hlth_ping_enable_disables_one_entry() {
        let entries = [PingEntry::new(2, 4, "compA"), PingEntry::new(2, 4, "compB")];
        let (comp, ground) = build(&entries, 8);
        let opcode = ID_BASE + Health::OPCODE_HLTH_PING_ENABLE;
        let mut arg = str_arg("compB");
        arg.push(Enabled::Disabled as u8);
        send_cmd(&comp, opcode, 5, &arg);
        run(&comp);
        // compB skipped in the sweep after the queued command applied.
        assert_eq!(*ground.pings.lock().unwrap(), vec![(0, 0)]);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(opcode, 5, CmdResponse::Ok)]
        );
        assert_eq!(event_ids(&ground), vec![Health::EVENTID_HLTH_CHECK_PING]);
        // Args: [enabled u8][string 40].
        let mut expected = vec![Enabled::Disabled as u8];
        expected.extend_from_slice(&str_arg("compB"));
        assert_eq!(ground.events.lock().unwrap()[0].2, expected);
        assert_eq!(
            ground.texts.lock().unwrap()[0].1,
            "Health checking set to DISABLED for compB"
        );
    }

    #[test]
    fn hlth_ping_enable_unknown_entry_logs_lookup_error() {
        let entries = [PingEntry::new(2, 4, "compA")];
        let (comp, ground) = build(&entries, 8);
        let opcode = ID_BASE + Health::OPCODE_HLTH_PING_ENABLE;
        let mut arg = str_arg("nobody");
        arg.push(Enabled::Enabled as u8);
        send_cmd(&comp, opcode, 6, &arg);
        run(&comp);
        assert_eq!(
            ground.responses.lock().unwrap()[0],
            (opcode, 6, CmdResponse::ValidationError)
        );
        assert_eq!(
            event_ids(&ground),
            vec![Health::EVENTID_HLTH_CHECK_LOOKUP_ERROR]
        );
        assert_eq!(ground.events.lock().unwrap()[0].1, LogSeverity::WarningLo);
        assert_eq!(
            ground.texts.lock().unwrap()[0].1,
            "Couldn't find entry nobody"
        );
    }

    #[test]
    fn hlth_chng_ping_updates_thresholds() {
        let entries = [PingEntry::new(5, 9, "compA")];
        let (comp, ground) = build(&entries, 8);
        let opcode = ID_BASE + Health::OPCODE_HLTH_CHNG_PING;
        let mut arg = str_arg("compA");
        arg.extend_from_slice(&1u32.to_be_bytes());
        arg.extend_from_slice(&2u32.to_be_bytes());
        send_cmd(&comp, opcode, 7, &arg);
        // run1: command applies (warn=1, fatal=2), then ping (count=1)
        // run2: count==1==warn -> WARN, count=2
        // run3: count==2==fatal -> FATAL
        for _ in 0..3 {
            run(&comp);
        }
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(opcode, 7, CmdResponse::Ok)]
        );
        assert_eq!(
            event_ids(&ground),
            vec![
                Health::EVENTID_HLTH_PING_UPDATED,
                Health::EVENTID_HLTH_PING_WARN,
                Health::EVENTID_HLTH_PING_LATE,
            ]
        );
        let mut expected = str_arg("compA");
        expected.extend_from_slice(&1u32.to_be_bytes());
        expected.extend_from_slice(&2u32.to_be_bytes());
        assert_eq!(ground.events.lock().unwrap()[0].2, expected);
        assert_eq!(
            ground.texts.lock().unwrap()[0].1,
            "Health ping for compA changed to WARN 1 FATAL 2"
        );
    }

    #[test]
    fn hlth_chng_ping_warn_above_fatal_is_a_validation_error() {
        let entries = [PingEntry::new(2, 4, "compA")];
        let (comp, ground) = build(&entries, 8);
        let opcode = ID_BASE + Health::OPCODE_HLTH_CHNG_PING;
        let mut arg = str_arg("compA");
        arg.extend_from_slice(&5u32.to_be_bytes());
        arg.extend_from_slice(&3u32.to_be_bytes());
        send_cmd(&comp, opcode, 8, &arg);
        comp.state.lock().unwrap().enabled = Enabled::Disabled; // quiet sweep
        run(&comp);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(opcode, 8, CmdResponse::ValidationError)]
        );
        assert_eq!(
            event_ids(&ground),
            vec![Health::EVENTID_HLTH_PING_INVALID_VALUES]
        );
        assert_eq!(
            ground.texts.lock().unwrap()[0].1,
            "Health ping for compA invalid values: WARN 5 FATAL 3"
        );
        // Thresholds unchanged.
        let state = comp.state.lock().unwrap();
        assert_eq!(state.trackers[0].entry.warn_cycles, 2);
        assert_eq!(state.trackers[0].entry.fatal_cycles, 4);
    }

    #[test]
    fn unknown_opcode_answers_invalid_opcode() {
        let (comp, ground) = build(&[], 8);
        send_cmd(&comp, ID_BASE + 0x99, 1, &[]);
        run(&comp);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(ID_BASE + 0x99, 1, CmdResponse::InvalidOpcode)]
        );
    }

    #[test]
    fn reg_commands_registers_all_three_opcodes() {
        let (comp, ground) = build(&[], 8);
        comp.reg_commands();
        assert_eq!(
            *ground.regs.lock().unwrap(),
            vec![ID_BASE, ID_BASE + 1, ID_BASE + 2]
        );
    }

    #[test]
    #[should_panic]
    fn set_ping_entries_asserts_warn_above_fatal() {
        let comp = Health::new("healthBad");
        comp.init(4);
        comp.set_ping_entries(&[PingEntry::new(5, 3, "compA")], 0);
    }

    #[test]
    #[should_panic]
    fn set_ping_entries_asserts_too_many_entries() {
        let comp = Health::new("healthBig");
        comp.init(4);
        let entries = vec![PingEntry::new(1, 2, "x"); HEALTH_PING_PORTS + 1];
        comp.set_ping_entries(&entries, 0);
    }
}
