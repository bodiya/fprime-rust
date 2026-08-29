//! # ActiveRateGroup — port of `Svc::ActiveRateGroup` (active)
//!
//! C++ sources: `Svc/ActiveRateGroup/ActiveRateGroup.{cpp,fpp}`,
//! `default/config/ActiveRateGroupCfg.hpp`.
//! Analysis: `docs/cpp-analysis/svc-core.md` (ActiveRateGroup section).
//!
//! Receives ticks on the async `CycleIn` port (`drop` queue-full policy)
//! and fans them out to up to [`RATE_GROUP_MEMBER_OUT_PORTS`] member
//! `Sched` ports with per-port context values, recording cycle-time
//! telemetry and detecting cycle slips.
//!
//! Slip detection (C++ parity, the documented gotcha): the `CycleIn`
//! adapter sets the `cycle_started` flag on the SENDER thread *before*
//! enqueueing (`CycleIn_preMsgHook`); the handler clears it at start — if
//! the flag is set again by the time member execution finishes, the next
//! tick arrived mid-cycle and the cycle slipped. The flag is a relaxed
//! atomic (the C++ field is a racy plain bool). The overrun event uses a
//! manual count-up/count-down throttle
//! ([`fprime_config::active_rate_group::OVERRUN_THROTTLE`]), distinct from
//! FPP event throttling.

use fprime_comp::{
    ActiveBase, ActiveComponent, ComponentDispatch, CyclePort, EventGlue, EventThrottle,
    MsgDispatchStatus, OutputPort, PingPort, QueueFullPolicy, SchedPort, TlmGlue,
    async_input_port_adapter, component_msg_types, msg,
};
use fprime_config::{
    FwChanIdType, FwEnumStoreType, FwEventIdType, FwIndexType, FwQueuePriorityType,
    RATE_GROUP_MEMBER_OUT_PORTS, active_rate_group::OVERRUN_THROTTLE,
};
use fprime_fw::{LogSeverity, SerBuf, SerBufAny, fw_assert};
use fprime_os::RawTime;
use fprime_os::rawtime::Status as RawTimeStatus;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

component_msg_types! {
    /// Queue message types — the C++ generated `<Comp>_MSG_TYPE` set
    /// (0 is the EXIT sentinel, so these start at 1).
    impl ActiveRateGroup {
        /// `CycleIn` async input port.
        MSG_TYPE_CYCLE_IN,
        /// `PingIn` async input port.
        MSG_TYPE_PING_IN,
    }
}

/// Queue sizing: message size = max over async invocations
/// (CycleIn: 6-byte envelope + 8-byte RawTime).
const MSG_SIZE: usize = 32;

/// Queue priorities (FPP declares none; both async ports share one so
/// dispatch is pure FIFO).
const CYCLE_IN_PRIORITY: FwQueuePriorityType = 1;
const PING_IN_PRIORITY: FwQueuePriorityType = 1;

impl ActiveRateGroup {
    /// Event: `RateGroupStarted` — DIAGNOSTIC, logged from the preamble.
    pub const EVENTID_RATE_GROUP_STARTED: FwEventIdType = 0;
    /// Event: `RateGroupCycleSlip(cycle: U32)` — WARNING_HI (manual throttle).
    pub const EVENTID_RATE_GROUP_CYCLE_SLIP: FwEventIdType = 1;
    /// Event: `RateGroupTimeGetError(status: Os.RawTimeStatus)` — WARNING_HI, throttle 5.
    pub const EVENTID_RATE_GROUP_TIME_GET_ERROR: FwEventIdType = 2;
    /// FPP `throttle 5` on `RateGroupTimeGetError`.
    pub const TIME_GET_ERROR_THROTTLE: u32 = 5;
    /// Telemetry: `RgMaxTime: U32` — update on change, "{} us".
    pub const CHANID_RG_MAX_TIME: FwChanIdType = 0;
    /// Telemetry: `RgCycleSlips: U32` — update on change.
    pub const CHANID_RG_CYCLE_SLIPS: FwChanIdType = 1;
}

/// Mutable state — only ever touched from the component thread (handlers)
/// or at init (`configure`).
#[derive(Default)]
struct RateGroupState {
    /// Per-port context values (`m_contexts`).
    contexts: [u32; RATE_GROUP_MEMBER_OUT_PORTS],
    /// `m_numContexts` — 0 means unconfigured (handler asserts).
    num_contexts: usize,
    /// `m_cycles` — cycle counter, slip event argument.
    cycles: u32,
    /// `m_maxTime` — high-water cycle time in usec.
    max_time: u32,
    /// `m_overrunThrottle` — manual count-up/count-down slip throttle.
    overrun_throttle: u32,
    /// `m_cycleSlips`.
    cycle_slips: u32,
    /// Last emitted `RgMaxTime` value (`update on change`).
    prev_max_tlm: Option<u32>,
    /// Last emitted `RgCycleSlips` value (`update on change`).
    prev_slips_tlm: Option<u32>,
}

/// `Svc::ActiveRateGroup`.
pub struct ActiveRateGroup {
    /// Active core: PassiveBase + queue + task.
    pub active: ActiveBase,
    /// Event ports (`Log`/`LogText`) + `Time` port.
    pub evt: EventGlue,
    /// Telemetry port (`Tlm`).
    pub tlm: TlmGlue,
    /// `RateGroupMemberOut: [10] Svc.Sched` output ports.
    pub rate_group_member_out: [OutputPort<dyn SchedPort>; RATE_GROUP_MEMBER_OUT_PORTS],
    /// `PingOut: Svc.Ping` output port.
    pub ping_out: OutputPort<dyn PingPort>,
    /// FPP `throttle 5` counter for `RateGroupTimeGetError`.
    time_get_error_throttle: EventThrottle,
    /// `m_cycleStarted` — set by the CycleIn adapter on the SENDER thread
    /// before enqueue, cleared at handler start (relaxed; C++ racy bool).
    cycle_started: AtomicBool,
    state: Mutex<RateGroupState>,
}

impl ActiveRateGroup {
    /// Construct. Call [`configure`](Self::configure), wire ports, then
    /// `create_queue` + `start` per the topology order.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            rate_group_member_out: [const { OutputPort::new() }; RATE_GROUP_MEMBER_OUT_PORTS],
            ping_out: OutputPort::new(),
            time_get_error_throttle: EventThrottle::new(Self::TIME_GET_ERROR_THROTTLE),
            cycle_started: AtomicBool::new(false),
            state: Mutex::new(RateGroupState::default()),
        })
    }

    fn id_base(&self) -> u32 {
        self.active.queued.base.get_id_base()
    }

    /// C++ `configure(ContextArray)`: copy the per-port context values.
    /// Must be called before the first cycle (the handler asserts).
    pub fn configure(&self, contexts: [u32; RATE_GROUP_MEMBER_OUT_PORTS]) {
        let mut state = self.state.lock().unwrap();
        state.contexts = contexts;
        state.num_contexts = RATE_GROUP_MEMBER_OUT_PORTS;
    }

    /// Create the message queue (`init` equivalent).
    pub fn init(&self, queue_depth: fprime_config::FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, MSG_SIZE as fprime_config::FwSizeType);
    }

    // -- Handlers (component thread) ---------------------------------------

    /// `CycleIn_handler`.
    fn cycle_in_handler(&self, _port_num: FwIndexType, cycle_start: &RawTime) {
        // C++ parity: FW_ASSERT(m_numContexts != 0) — configure() missing.
        let contexts = {
            let state = self.state.lock().unwrap();
            fw_assert!(state.num_contexts != 0);
            state.contexts
        };

        self.cycle_started.store(false, Ordering::Relaxed);

        // Invoke members of the rate group (ports invoked outside the
        // state lock, per convention; C++ holds no lock here either).
        for (port, context) in contexts.iter().enumerate() {
            if let Some(p) = self.rate_group_member_out[port].try_get() {
                p.target.invoke(p.port_num, *context);
            }
        }

        // Grab timer for endTime of cycle.
        let mut end_time = RawTime::new();
        let time_status = end_time.now();
        if time_status != RawTimeStatus::OpOk && self.time_get_error_throttle.ok_to_emit() {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_RATE_GROUP_TIME_GET_ERROR,
                LogSeverity::WarningHi,
                &format!(
                    "Rate group failed to read cycle end time with status {}",
                    time_status as i32
                ),
                // Os.RawTimeStatus is an FPP `enum : U8`.
                |buf| buf.serialize_u8_be(time_status as u8),
            );
        }

        // Cycle execution time; the only error is overflow, which
        // get_diff_usec already caps at u32::MAX (C++ casts to void).
        let mut cycle_time: u32 = 0;
        let _ = end_time.get_diff_usec(cycle_start, &mut cycle_time);

        // State updates + telemetry decisions under the lock; emissions
        // after it is dropped.
        let mut emit_max: Option<u32> = None;
        let mut emit_slips: Option<u32> = None;
        let mut slip_event_cycle: Option<u32> = None;
        {
            let mut state = self.state.lock().unwrap();
            if cycle_time > state.max_time {
                state.max_time = cycle_time;
            }
            // tlmWrite_RgMaxTime — update on change.
            if state.prev_max_tlm != Some(state.max_time) {
                state.prev_max_tlm = Some(state.max_time);
                emit_max = Some(state.max_time);
            }
            // Cycle slip: the next tick was enqueued while members ran.
            if self.cycle_started.load(Ordering::Relaxed) {
                state.cycle_slips += 1;
                if state.overrun_throttle < OVERRUN_THROTTLE {
                    slip_event_cycle = Some(state.cycles);
                    state.overrun_throttle += 1;
                }
                // tlmWrite_RgCycleSlips — update on change.
                if state.prev_slips_tlm != Some(state.cycle_slips) {
                    state.prev_slips_tlm = Some(state.cycle_slips);
                    emit_slips = Some(state.cycle_slips);
                }
            } else if state.overrun_throttle > 0 {
                // Clean cycle: decrement the manual throttle.
                state.overrun_throttle -= 1;
            }
            state.cycles += 1;
        }

        if let Some(max) = emit_max {
            self.tlm.tlm_write(
                self.id_base(),
                Self::CHANID_RG_MAX_TIME,
                &max,
                self.evt.time_get(),
            );
        }
        if let Some(cycle) = slip_event_cycle {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_RATE_GROUP_CYCLE_SLIP,
                LogSeverity::WarningHi,
                &format!("Rate group cycle slipped on cycle {cycle}"),
                |buf| buf.serialize_u32_be(cycle),
            );
        }
        if let Some(slips) = emit_slips {
            self.tlm.tlm_write(
                self.id_base(),
                Self::CHANID_RG_CYCLE_SLIPS,
                &slips,
                self.evt.time_get(),
            );
        }
    }

    /// `PingIn_handler`: echo the key back to Health.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }
}

// -- Async input adapters (generated by the codegen layer) -------------------

async_input_port_adapter! {
    /// `CycleIn` — ASYNC `Svc.Cycle` input with the `drop` queue-full policy.
    component: ActiveRateGroup;
    adapter: CycleInAdapter;
    port: CyclePort;
    input: pub cycle_in;
    deserialize: cycle_in_deserialize;
    handler: cycle_in_handler;
    base: active.queued;
    msg_type: ActiveRateGroup::MSG_TYPE_CYCLE_IN;
    msg_size: MSG_SIZE;
    priority: CYCLE_IN_PRIORITY;
    queue_full: QueueFullPolicy::Drop;
    // RawTime wire format: [sec u32][nsec u32] BE (8 bytes).
    args { ref cycle_start: RawTime }
    pre_msg_hook |comp, _port_num| {
        // `CycleIn_preMsgHook`: runs on the SENDER thread BEFORE enqueue.
        comp.cycle_started.store(true, Ordering::Relaxed);
    }
}

async_input_port_adapter! {
    /// `PingIn` — ASYNC `Svc.Ping` input (default/`assert` policy).
    component: ActiveRateGroup;
    adapter: PingInAdapter;
    port: PingPort;
    input: pub ping_in;
    deserialize: ping_in_deserialize;
    handler: ping_in_handler;
    base: active.queued;
    msg_type: ActiveRateGroup::MSG_TYPE_PING_IN;
    msg_size: MSG_SIZE;
    priority: PING_IN_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val key: u32 }
}

// -- Dispatch ----------------------------------------------------------------

impl ComponentDispatch for ActiveRateGroup {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        // The arg codecs come from the same `args { .. }` declarations as
        // the adapters above, so the read side cannot drift from the write
        // side.
        match msg_type {
            Self::MSG_TYPE_CYCLE_IN => match Self::cycle_in_deserialize(buf) {
                Some((cycle_start,)) => {
                    self.cycle_in_handler(port_num, &cycle_start);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_PING_IN => match Self::ping_in_deserialize(buf) {
                Some((key,)) => {
                    self.ping_in_handler(port_num, key);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            _ => MsgDispatchStatus::Error,
        }
    }

    /// C++ `preamble()`: first thing on the component thread.
    fn preamble(&self) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_RATE_GROUP_STARTED,
            LogSeverity::Diagnostic,
            "Rate group started.",
            |_buf| fprime_fw::SerializeStatus::Ok,
        );
    }
}

impl ActiveComponent for ActiveRateGroup {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{LogPort, LogTextPort, TlmPort};
    use fprime_fw::{LogBuffer, TextLogString, Time, TlmBuffer};
    use fprime_os::queue::BlockingType;
    use fprime_os::task::{Status as TaskStatus, TASK_DEFAULT};

    const ID_BASE: u32 = 0x400;

    #[derive(Default)]
    struct Ground {
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        texts: Mutex<Vec<(FwEventIdType, String)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        sched: Mutex<Vec<(FwIndexType, u32)>>,
        pings: Mutex<Vec<u32>>,
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

    impl SchedPort for Ground {
        fn invoke(&self, port_num: FwIndexType, context: u32) {
            self.sched.lock().unwrap().push((port_num, context));
        }
    }

    impl PingPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.pings.lock().unwrap().push(key);
        }
    }

    fn contexts() -> [u32; RATE_GROUP_MEMBER_OUT_PORTS] {
        std::array::from_fn(|i| 100 + i as u32)
    }

    fn build(connect_members: usize) -> (Arc<ActiveRateGroup>, Arc<Ground>) {
        let ground = Arc::new(Ground::default());
        let comp = ActiveRateGroup::new("rg1");
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.ping_out.connect(ground.clone(), 0);
        for (i, port) in comp
            .rate_group_member_out
            .iter()
            .take(connect_members)
            .enumerate()
        {
            port.connect(ground.clone(), i as FwIndexType);
        }
        comp.configure(contexts());
        comp.init(16);
        (comp, ground)
    }

    /// Drive one queued message through dispatch on this thread.
    fn dispatch_one(comp: &Arc<ActiveRateGroup>) {
        let status = comp
            .active
            .queued
            .do_dispatch(&**comp, BlockingType::NonBlocking);
        assert_eq!(status, MsgDispatchStatus::Ok);
    }

    #[test]
    fn cycle_envelope_bytes_are_byte_exact() {
        let (comp, _ground) = build(0);
        let cycle = comp.cycle_in(2);
        let raw = RawTime::from_parts(0x01020304, 0x0A0B0C0D);
        cycle.target.invoke(cycle.port_num, &raw);
        let mut dest = [0u8; MSG_SIZE];
        let mut size = 0;
        let mut priority = 0;
        let status = comp.active.queued.queue().receive(
            &mut dest,
            BlockingType::NonBlocking,
            &mut size,
            &mut priority,
        );
        assert_eq!(status, fprime_os::queue::Status::OpOk);
        assert_eq!(
            &dest[..size as usize],
            &[
                0x00, 0x00, 0x00, 0x01, // msg_type = CYCLE_IN (i32 BE)
                0x00, 0x02, // port_num = 2 (i16 BE)
                0x01, 0x02, 0x03, 0x04, // RawTime seconds (u32 BE)
                0x0A, 0x0B, 0x0C, 0x0D, // RawTime nanoseconds (u32 BE)
            ]
        );
        assert_eq!(priority, CYCLE_IN_PRIORITY);
    }

    #[test]
    fn cycle_invokes_connected_members_with_contexts() {
        let (comp, ground) = build(3);
        let cycle = comp.cycle_in(0);
        cycle
            .target
            .invoke(cycle.port_num, &RawTime::from_parts(1, 0));
        dispatch_one(&comp);
        assert_eq!(
            *ground.sched.lock().unwrap(),
            vec![(0, 100), (1, 101), (2, 102)]
        );
        // RgMaxTime emitted on the first cycle (on-change, no previous).
        let tlm = ground.tlm.lock().unwrap();
        assert_eq!(tlm.len(), 1);
        assert_eq!(tlm[0].0, ID_BASE + ActiveRateGroup::CHANID_RG_MAX_TIME);
    }

    #[test]
    fn rg_max_time_tlm_values_are_strictly_increasing() {
        // On-change: RgMaxTime is written every cycle but only emitted
        // when the high-water value changes -> emitted values must be
        // strictly increasing.
        let (comp, ground) = build(1);
        let cycle = comp.cycle_in(0);
        for _ in 0..10 {
            cycle
                .target
                .invoke(cycle.port_num, &RawTime::from_parts(0, 0));
            dispatch_one(&comp);
        }
        let tlm = ground.tlm.lock().unwrap();
        let max_values: Vec<u32> = tlm
            .iter()
            .filter(|(id, _)| *id == ID_BASE + ActiveRateGroup::CHANID_RG_MAX_TIME)
            .map(|(_, bytes)| u32::from_be_bytes(bytes.as_slice().try_into().unwrap()))
            .collect();
        assert!(!max_values.is_empty());
        assert!(max_values.windows(2).all(|w| w[0] < w[1]));
    }

    /// A member that re-invokes CycleIn from inside the cycle — the next
    /// tick arrives on the "sender thread" while members are running,
    /// which must be detected as a cycle slip via the atomic flag.
    struct SlippingMember {
        comp: Mutex<Option<Arc<ActiveRateGroup>>>,
        raw: RawTime,
    }

    impl SchedPort for SlippingMember {
        fn invoke(&self, _port_num: FwIndexType, _context: u32) {
            if let Some(comp) = self.comp.lock().unwrap().as_ref() {
                let cycle = comp.cycle_in(0);
                cycle.target.invoke(cycle.port_num, &self.raw);
            }
        }
    }

    fn build_slipping() -> (Arc<ActiveRateGroup>, Arc<Ground>, Arc<SlippingMember>) {
        let ground = Arc::new(Ground::default());
        let comp = ActiveRateGroup::new("rgSlip");
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        let member = Arc::new(SlippingMember {
            comp: Mutex::new(None),
            raw: RawTime::from_parts(0, 0),
        });
        comp.rate_group_member_out[0].connect(member.clone(), 0);
        comp.configure(contexts());
        comp.init(64);
        *member.comp.lock().unwrap() = Some(comp.clone());
        (comp, ground, member)
    }

    #[test]
    fn cycle_slip_detected_via_sender_thread_flag() {
        let (comp, ground, member) = build_slipping();
        let cycle = comp.cycle_in(0);
        cycle
            .target
            .invoke(cycle.port_num, &RawTime::from_parts(0, 0));
        dispatch_one(&comp); // member re-enqueued a tick mid-cycle -> slip
        *member.comp.lock().unwrap() = None; // stop re-triggering

        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0].0,
            ID_BASE + ActiveRateGroup::EVENTID_RATE_GROUP_CYCLE_SLIP
        );
        assert_eq!(events[0].1, LogSeverity::WarningHi);
        assert_eq!(events[0].2, vec![0, 0, 0, 0]); // slipped on cycle 0
        drop(events);
        assert_eq!(
            ground.texts.lock().unwrap()[0].1,
            "Rate group cycle slipped on cycle 0"
        );
        // RgCycleSlips tlm emitted with value 1.
        let tlm = ground.tlm.lock().unwrap();
        let slips: Vec<Vec<u8>> = tlm
            .iter()
            .filter(|(id, _)| *id == ID_BASE + ActiveRateGroup::CHANID_RG_CYCLE_SLIPS)
            .map(|(_, v)| v.clone())
            .collect();
        assert_eq!(slips, vec![vec![0, 0, 0, 1]]);
    }

    #[test]
    fn overrun_throttle_counts_up_and_down() {
        let (comp, ground, member) = build_slipping();
        let cycle = comp.cycle_in(0);
        // 8 slipping cycles: the member re-enqueues each time, so after the
        // first external tick every dispatch slips. Only the first 5 emit
        // the event (manual throttle at OVERRUN_THROTTLE=5).
        cycle
            .target
            .invoke(cycle.port_num, &RawTime::from_parts(0, 0));
        for _ in 0..8 {
            dispatch_one(&comp);
        }
        // Stop slipping; drain the one queued tick plus clean ticks to
        // count the throttle back down (one decrement per clean cycle).
        *member.comp.lock().unwrap() = None;
        dispatch_one(&comp); // queued tick, clean now
        let slip_id = ID_BASE + ActiveRateGroup::EVENTID_RATE_GROUP_CYCLE_SLIP;
        let count_events = |g: &Ground| {
            g.events
                .lock()
                .unwrap()
                .iter()
                .filter(|e| e.0 == slip_id)
                .count()
        };
        assert_eq!(count_events(&ground), 5);

        // 4 clean cycles decrement the throttle 5 -> 1; the next slip may
        // emit again.
        for _ in 0..4 {
            cycle
                .target
                .invoke(cycle.port_num, &RawTime::from_parts(0, 0));
            dispatch_one(&comp);
        }
        *member.comp.lock().unwrap() = Some(comp.clone());
        cycle
            .target
            .invoke(cycle.port_num, &RawTime::from_parts(0, 0));
        dispatch_one(&comp);
        *member.comp.lock().unwrap() = None;
        dispatch_one(&comp); // drain re-enqueued tick
        assert_eq!(count_events(&ground), 6);
    }

    #[test]
    fn drop_policy_discards_when_queue_full() {
        let small = ActiveRateGroup::new("rgSmall");
        small.configure(contexts());
        small.active.queued.create_queue(2, MSG_SIZE as u64);
        let cycle = small.cycle_in(0);
        for _ in 0..5 {
            cycle
                .target
                .invoke(cycle.port_num, &RawTime::from_parts(0, 0));
        }
        assert_eq!(small.active.queued.get_num_msgs_dropped(), 3);
    }

    #[test]
    fn ping_is_echoed_from_component_thread() {
        let (comp, ground) = build(1);
        let ping = comp.ping_in(0);
        ping.target.invoke(ping.port_num, 0xFEED);
        dispatch_one(&comp);
        assert_eq!(*ground.pings.lock().unwrap(), vec![0xFEED]);
    }

    #[test]
    #[should_panic]
    fn unconfigured_cycle_asserts() {
        let ground = Arc::new(Ground::default());
        let comp = ActiveRateGroup::new("rgUnconf");
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.init(4);
        let cycle = comp.cycle_in(0);
        cycle
            .target
            .invoke(cycle.port_num, &RawTime::from_parts(0, 0));
        let _ = comp
            .active
            .queued
            .do_dispatch(&*comp, BlockingType::NonBlocking);
    }

    #[test]
    fn full_lifecycle_preamble_logs_rate_group_started() {
        let (comp, ground) = build(2);
        comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);
        let cycle = comp.cycle_in(0);
        cycle
            .target
            .invoke(cycle.port_num, &RawTime::from_parts(5, 0));
        comp.active.exit();
        assert_eq!(comp.active.join(), TaskStatus::OpOk);
        let events = ground.events.lock().unwrap();
        assert_eq!(
            events[0],
            (
                ID_BASE + ActiveRateGroup::EVENTID_RATE_GROUP_STARTED,
                LogSeverity::Diagnostic,
                vec![]
            )
        );
        drop(events);
        assert_eq!(
            ground.texts.lock().unwrap()[0],
            (
                ID_BASE + ActiveRateGroup::EVENTID_RATE_GROUP_STARTED,
                "Rate group started.".to_string()
            )
        );
        // Members ran once each.
        assert_eq!(*ground.sched.lock().unwrap(), vec![(0, 100), (1, 101)]);
    }
}
