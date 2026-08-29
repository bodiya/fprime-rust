//! # PassiveRateGroup — port of `Svc::PassiveRateGroup` (passive)
//!
//! C++ sources: `Svc/PassiveRateGroup/PassiveRateGroup.{cpp,hpp,fpp}`,
//! `default/config/PassiveRateGroupCfg.hpp`.
//! Analysis: `docs/cpp-analysis/svc-core.md` (PassiveRateGroup section).
//!
//! Sync `CycleIn` handler (runs on the caller's thread, possibly ISR):
//! invokes each connected member `Sched` port with its configured context,
//! measuring per-port and total cycle times. High-water marks
//! (`m_maxTime`, per-port HWMs) are relaxed atomics updated with a
//! CAS-max (`fetch_max`), matching the C++ lock-free `compare_exchange`
//! loops, so the `CLEAR_STATISTICS` command can clear them without a lock.
//!
//! Telemetry channels (auto ids in FPP declaration order):
//! `MaxCycleTime=0` (on change), `CycleTime=1`, `CycleCount=2`,
//! `PortCycleTime=3`, `PortCycleTimeHWM=4` (on change).

use fprime_comp::{
    CmdGlue, CmdPort, CyclePort, OutputPort, PassiveBase, PortRef, SchedPort, TimePort, TlmGlue,
    glue,
};
use fprime_config::{FwChanIdType, FwIndexType, FwOpcodeType, RATE_GROUP_MEMBER_OUT_PORTS};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, Endianness, SerBuf, SerBufAny, Serialize, SerializeStatus,
    fw_assert, fw_try,
};
use fprime_os::RawTime;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

/// `PassiveRateGroupCfg::PortCycleTime` — enable per-port cycle-time
/// measurement/telemetry (C++ default `true`).
pub const PORT_CYCLE_TIME: bool = true;

/// FPP `array CycleTime = [10] U32 default 0` — the telemetry value type
/// of `PortCycleTime` / `PortCycleTimeHWM`. Serializes as 10 raw
/// big-endian `U32`s (40 bytes, no length prefix).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CycleTimes(pub [u32; RATE_GROUP_MEMBER_OUT_PORTS]);

impl Serialize for CycleTimes {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        for v in &self.0 {
            fw_try!(buf.serialize_u32(*v, e));
        }
        SerializeStatus::Ok
    }

    fn serialized_size(&self) -> usize {
        RATE_GROUP_MEMBER_OUT_PORTS * 4
    }
}

/// Mutable (mutex-guarded) state; the high-water marks live outside as
/// atomics (see module docs).
#[derive(Default)]
struct RateGroupState {
    contexts: [u32; RATE_GROUP_MEMBER_OUT_PORTS],
    /// 0 means unconfigured (handler asserts).
    num_contexts: usize,
    /// `m_cycles` — running total, NOT cleared by `CLEAR_STATISTICS`.
    cycles: u32,
    /// Last emitted `MaxCycleTime` (`update on change`).
    prev_max_tlm: Option<u32>,
    /// Last emitted `PortCycleTimeHWM` (`update on change`).
    prev_hwm_tlm: Option<[u32; RATE_GROUP_MEMBER_OUT_PORTS]>,
}

/// `Svc::PassiveRateGroup`.
pub struct PassiveRateGroup {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Command registration + response ports.
    pub cmd: CmdGlue,
    /// Telemetry port (`Tlm`).
    pub tlm: TlmGlue,
    /// `Time` — time get port for telemetry stamps.
    pub time_out: OutputPort<dyn TimePort>,
    /// `RateGroupMemberOut: [10] Svc.Sched`.
    pub rate_group_member_out: [OutputPort<dyn SchedPort>; RATE_GROUP_MEMBER_OUT_PORTS],
    /// `m_maxTime` — max total cycle time in usec (relaxed atomic).
    max_time: AtomicU32,
    /// `m_portCycleTimeHWMUsec` — per-port HWMs (relaxed atomics).
    port_cycle_time_hwm: [AtomicU32; RATE_GROUP_MEMBER_OUT_PORTS],
    state: Mutex<RateGroupState>,
}

impl PassiveRateGroup {
    /// Command: `CLEAR_STATISTICS` — sync, auto opcode 0.
    pub const OPCODE_CLEAR_STATISTICS: FwOpcodeType = 0;
    /// Telemetry: `MaxCycleTime: U32` — on change, "{} us".
    pub const CHANID_MAX_CYCLE_TIME: FwChanIdType = 0;
    /// Telemetry: `CycleTime: U32` — "{} us".
    pub const CHANID_CYCLE_TIME: FwChanIdType = 1;
    /// Telemetry: `CycleCount: U32`.
    pub const CHANID_CYCLE_COUNT: FwChanIdType = 2;
    /// Telemetry: `PortCycleTime: CycleTime` (array of 10 U32).
    pub const CHANID_PORT_CYCLE_TIME: FwChanIdType = 3;
    /// Telemetry: `PortCycleTimeHWM: CycleTime` — on change.
    pub const CHANID_PORT_CYCLE_TIME_HWM: FwChanIdType = 4;

    /// Construct.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            cmd: CmdGlue::new(),
            tlm: TlmGlue::new(),
            time_out: OutputPort::new(),
            rate_group_member_out: [const { OutputPort::new() }; RATE_GROUP_MEMBER_OUT_PORTS],
            max_time: AtomicU32::new(0),
            port_cycle_time_hwm: [const { AtomicU32::new(0) }; RATE_GROUP_MEMBER_OUT_PORTS],
            state: Mutex::new(RateGroupState::default()),
        })
    }

    fn id_base(&self) -> u32 {
        self.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd
            .reg_commands(self.id_base(), &[Self::OPCODE_CLEAR_STATISTICS]);
    }

    /// C++ `configure(ContextArray)` — the `rawTimeSource` parameter is
    /// not ported (the Rust OSAL has a single RawTime source).
    pub fn configure(&self, contexts: [u32; RATE_GROUP_MEMBER_OUT_PORTS]) {
        let mut state = self.state.lock().unwrap();
        state.contexts = contexts;
        state.num_contexts = RATE_GROUP_MEMBER_OUT_PORTS;
    }

    // -- Input-port factories ----------------------------------------------

    /// `CycleIn` — SYNC `Svc.Cycle` input.
    pub fn cycle_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CyclePort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `CmdDisp` — SYNC command input (`CLEAR_STATISTICS` is a sync
    /// command; it runs on the dispatcher's thread).
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Handlers ------------------------------------------------------------

    /// `CycleIn_handler` — caller's thread.
    fn cycle_in_handler(&self, _port_num: FwIndexType, cycle_start: &RawTime) {
        let contexts = {
            let state = self.state.lock().unwrap();
            // C++ parity: FW_ASSERT(m_numContexts != 0).
            fw_assert!(state.num_contexts != 0);
            state.contexts
        };

        let mut port_times = CycleTimes::default();
        for (port, context) in contexts.iter().enumerate() {
            if let Some(p) = self.rate_group_member_out[port].try_get() {
                let mut port_start = RawTime::new();
                if PORT_CYCLE_TIME {
                    let _ = port_start.now();
                }
                p.target.invoke(p.port_num, *context);
                if PORT_CYCLE_TIME {
                    let mut port_end = RawTime::new();
                    let _ = port_end.now();
                    let mut port_time: u32 = 0;
                    let _ = port_end.get_diff_usec(&port_start, &mut port_time);
                    port_times.0[port] = port_time;
                    // Lock-free CAS-max HWM update (C++ compare_exchange loop).
                    self.port_cycle_time_hwm[port].fetch_max(port_time, Ordering::Relaxed);
                }
            }
        }

        let mut end_time = RawTime::new();
        let _ = end_time.now();
        let mut cycle_time: u32 = 0;
        // Only possible error is overflow, capped in get_diff_usec (C++ void).
        let _ = end_time.get_diff_usec(cycle_start, &mut cycle_time);

        // Lock-free CAS-max on the total-cycle high-water mark.
        self.max_time.fetch_max(cycle_time, Ordering::Relaxed);
        let max_time = self.max_time.load(Ordering::Relaxed);

        let hwm_snapshot = CycleTimes(std::array::from_fn(|i| {
            self.port_cycle_time_hwm[i].load(Ordering::Relaxed)
        }));

        // Cycle counter + on-change decisions under the state lock.
        let (cycles, emit_max, emit_hwm) = {
            let mut state = self.state.lock().unwrap();
            state.cycles += 1;
            let emit_max = if state.prev_max_tlm != Some(max_time) {
                state.prev_max_tlm = Some(max_time);
                true
            } else {
                false
            };
            let emit_hwm = if state.prev_hwm_tlm != Some(hwm_snapshot.0) {
                state.prev_hwm_tlm = Some(hwm_snapshot.0);
                true
            } else {
                false
            };
            (state.cycles, emit_max, emit_hwm)
        };

        // Telemetry, in the C++ handler's write order.
        let time_tag = glue::time_get(&self.time_out);
        let id_base = self.id_base();
        if PORT_CYCLE_TIME {
            self.tlm
                .tlm_write(id_base, Self::CHANID_PORT_CYCLE_TIME, &port_times, time_tag);
            if emit_hwm {
                self.tlm.tlm_write(
                    id_base,
                    Self::CHANID_PORT_CYCLE_TIME_HWM,
                    &hwm_snapshot,
                    time_tag,
                );
            }
        }
        if emit_max {
            self.tlm
                .tlm_write(id_base, Self::CHANID_MAX_CYCLE_TIME, &max_time, time_tag);
        }
        self.tlm
            .tlm_write(id_base, Self::CHANID_CYCLE_TIME, &cycle_time, time_tag);
        self.tlm
            .tlm_write(id_base, Self::CHANID_CYCLE_COUNT, &cycles, time_tag);
    }

    /// `CLEAR_STATISTICS_cmdHandler`: zero the max cycle time and the
    /// per-port HWMs (lock-free); `m_cycles` is intentionally NOT cleared.
    fn clear_statistics_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        // C++ parity (FW_CMD_CHECK_RESIDUAL): zero-arg command rejects
        // residual bytes with FormatError.
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        self.max_time.store(0, Ordering::Relaxed);
        for hwm in &self.port_cycle_time_hwm {
            hwm.store(0, Ordering::Relaxed);
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }
}

impl CyclePort for PassiveRateGroup {
    fn invoke(&self, port_num: FwIndexType, cycle_start: &RawTime) {
        self.cycle_in_handler(port_num, cycle_start);
    }
}

impl CmdPort for PassiveRateGroup {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_CLEAR_STATISTICS => {
                self.clear_statistics_cmd_handler(op_code, cmd_seq, args);
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, TlmPort};
    use fprime_fw::{Time, TlmBuffer};

    const ID_BASE: u32 = 0x500;

    #[derive(Default)]
    struct Ground {
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        sched: Mutex<Vec<(FwIndexType, u32)>>,
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
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

    impl SchedPort for Ground {
        fn invoke(&self, port_num: FwIndexType, context: u32) {
            self.sched.lock().unwrap().push((port_num, context));
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

    fn contexts() -> [u32; RATE_GROUP_MEMBER_OUT_PORTS] {
        std::array::from_fn(|i| 200 + i as u32)
    }

    fn build(connect_members: usize) -> (Arc<PassiveRateGroup>, Arc<Ground>) {
        let ground = Arc::new(Ground::default());
        let comp = PassiveRateGroup::new("prg");
        comp.base.set_id_base(ID_BASE);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        for (i, port) in comp
            .rate_group_member_out
            .iter()
            .take(connect_members)
            .enumerate()
        {
            port.connect(ground.clone(), i as FwIndexType);
        }
        comp.configure(contexts());
        (comp, ground)
    }

    fn cycle(comp: &Arc<PassiveRateGroup>) {
        let port = comp.cycle_in(0);
        port.target
            .invoke(port.port_num, &RawTime::from_parts(0, 0));
    }

    fn send_cmd(comp: &Arc<PassiveRateGroup>, opcode: FwOpcodeType, seq: u32, arg_bytes: &[u8]) {
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(arg_bytes).is_ok());
        let port = comp.cmd_in(0);
        port.target.invoke(port.port_num, opcode, seq, &mut args);
    }

    fn values_for(ground: &Ground, chan: FwChanIdType) -> Vec<Vec<u8>> {
        ground
            .tlm
            .lock()
            .unwrap()
            .iter()
            .filter(|(id, _)| *id == ID_BASE + chan)
            .map(|(_, v)| v.clone())
            .collect()
    }

    #[test]
    fn members_invoked_in_order_with_contexts() {
        let (comp, ground) = build(3);
        cycle(&comp);
        assert_eq!(
            *ground.sched.lock().unwrap(),
            vec![(0, 200), (1, 201), (2, 202)]
        );
    }

    #[test]
    fn telemetry_channels_and_declaration_order_ids() {
        let (comp, ground) = build(2);
        cycle(&comp);
        // First cycle: every channel emits (on-change channels have no
        // previous value). Write order per the C++ handler.
        let ids: Vec<FwChanIdType> = ground
            .tlm
            .lock()
            .unwrap()
            .iter()
            .map(|(id, _)| id - ID_BASE)
            .collect();
        assert_eq!(
            ids,
            vec![
                PassiveRateGroup::CHANID_PORT_CYCLE_TIME,
                PassiveRateGroup::CHANID_PORT_CYCLE_TIME_HWM,
                PassiveRateGroup::CHANID_MAX_CYCLE_TIME,
                PassiveRateGroup::CHANID_CYCLE_TIME,
                PassiveRateGroup::CHANID_CYCLE_COUNT,
            ]
        );
        // Array channels serialize as 10 raw U32s (40 bytes, no prefix).
        let port_times = values_for(&ground, PassiveRateGroup::CHANID_PORT_CYCLE_TIME);
        assert_eq!(port_times[0].len(), 40);
    }

    #[test]
    fn cycle_count_increments_and_cycle_time_always_emits() {
        let (comp, ground) = build(1);
        for _ in 0..3 {
            cycle(&comp);
        }
        let counts = values_for(&ground, PassiveRateGroup::CHANID_CYCLE_COUNT);
        assert_eq!(
            counts,
            vec![vec![0, 0, 0, 1], vec![0, 0, 0, 2], vec![0, 0, 0, 3]]
        );
        // CycleTime is not on-change: one emission per cycle.
        assert_eq!(
            values_for(&ground, PassiveRateGroup::CHANID_CYCLE_TIME).len(),
            3
        );
    }

    #[test]
    fn max_cycle_time_on_change_emits_strictly_increasing_values() {
        let (comp, ground) = build(1);
        for _ in 0..10 {
            cycle(&comp);
        }
        let values: Vec<u32> = values_for(&ground, PassiveRateGroup::CHANID_MAX_CYCLE_TIME)
            .iter()
            .map(|v| u32::from_be_bytes(v.as_slice().try_into().unwrap()))
            .collect();
        assert!(!values.is_empty());
        assert!(values.windows(2).all(|w| w[0] < w[1]));
    }

    #[test]
    fn hwms_never_decrease() {
        let (comp, _ground) = build(2);
        let mut last_max = 0;
        let mut last_hwm = [0u32; RATE_GROUP_MEMBER_OUT_PORTS];
        for _ in 0..5 {
            cycle(&comp);
            let max = comp.max_time.load(Ordering::Relaxed);
            assert!(max >= last_max);
            last_max = max;
            for (i, hwm) in comp.port_cycle_time_hwm.iter().enumerate() {
                let v = hwm.load(Ordering::Relaxed);
                assert!(v >= last_hwm[i]);
                last_hwm[i] = v;
            }
        }
    }

    #[test]
    fn unconnected_ports_are_not_invoked_and_report_zero_port_time() {
        let (comp, ground) = build(1);
        cycle(&comp);
        assert_eq!(ground.sched.lock().unwrap().len(), 1);
        let port_times = values_for(&ground, PassiveRateGroup::CHANID_PORT_CYCLE_TIME);
        // Ports 1..9 unconnected: their slots stay 0.
        assert!(port_times[0][4..].iter().all(|b| *b == 0));
    }

    #[test]
    fn reg_commands_registers_clear_statistics() {
        let (comp, ground) = build(0);
        comp.reg_commands();
        assert_eq!(
            *ground.regs.lock().unwrap(),
            vec![ID_BASE + PassiveRateGroup::OPCODE_CLEAR_STATISTICS]
        );
    }

    #[test]
    fn clear_statistics_zeroes_hwms_but_not_cycles() {
        let (comp, ground) = build(2);
        for _ in 0..3 {
            cycle(&comp);
        }
        let opcode = ID_BASE + PassiveRateGroup::OPCODE_CLEAR_STATISTICS;
        send_cmd(&comp, opcode, 9, &[]);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(opcode, 9, CmdResponse::Ok)]
        );
        assert_eq!(comp.max_time.load(Ordering::Relaxed), 0);
        for hwm in &comp.port_cycle_time_hwm {
            assert_eq!(hwm.load(Ordering::Relaxed), 0);
        }
        // Cycle count continues (running total not cleared).
        cycle(&comp);
        let counts = values_for(&ground, PassiveRateGroup::CHANID_CYCLE_COUNT);
        assert_eq!(counts.last().unwrap(), &vec![0, 0, 0, 4]);
    }

    #[test]
    fn clear_statistics_rejects_residual_bytes() {
        let (comp, ground) = build(0);
        let opcode = ID_BASE + PassiveRateGroup::OPCODE_CLEAR_STATISTICS;
        send_cmd(&comp, opcode, 1, &[0xAA]);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(opcode, 1, CmdResponse::FormatError)]
        );
    }

    #[test]
    fn unknown_opcode_answers_invalid_opcode() {
        let (comp, ground) = build(0);
        send_cmd(&comp, ID_BASE + 0x77, 2, &[]);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(ID_BASE + 0x77, 2, CmdResponse::InvalidOpcode)]
        );
    }

    #[test]
    #[should_panic]
    fn unconfigured_cycle_asserts() {
        let comp = PassiveRateGroup::new("prgUnconf");
        let port = comp.cycle_in(0);
        port.target
            .invoke(port.port_num, &RawTime::from_parts(0, 0));
    }
}
