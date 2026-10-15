//! # ActivePhaser — port of `Svc::ActivePhaser` (active)
//!
//! C++ sources: `Svc/ActivePhaser/ActivePhaser.{cpp,hpp,fpp}`,
//! `Svc/ActivePhaser/docs/sdd.md`.
//!
//! A phased rate group: each phaser cycle is [`ActivePhaser::configure`]d
//! as `cycle_ticks` ticks of the `CycleIn` input, and every registered
//! `PhaserMemberOut` call ([`ActivePhaser::register_phased`]) owns a window
//! `[start, start + length)` of ticks in it. Calls run in registration
//! order, one at a time: a call starts once the cycle reaches its start
//! tick and the previous call has finished, and it "finishes" on the next
//! tick the component thread processes after it returned. A call that took
//! longer than its `length` is reported with `MissedDeadline` (throttled
//! to 5) and the next call is held back one tick, so a late child cannot
//! shorten its successor's window.
//!
//! The context each call receives is either SEQUENTIAL — the registration
//! index of that entry among the entries on the same port (0, 1, ...) —
//! or COUNT, when registered with a `user_context` window: the number of
//! phaser cycles elapsed, modulo `user_context / cycle_ticks`.
//!
//! ## Threading and dropped cycles (C++ parity)
//!
//! `CycleIn` is async with the `drop` policy and the queue depth must be
//! exactly one ([`ActivePhaser::init`] asserts it — "to prevent a rush to
//! catch up"). The `CycleIn` handler (component thread) only counts the
//! tick and posts the internal `Tick` message (also `drop`); `Tick` does
//! the scheduling and invokes the children on the component thread. While
//! a child runs, the arriving cycles still count ticks, but their `Tick`s
//! are dropped silently — no event, no channel (the SDD's "Dropped
//! Cycles") — so the next `Tick` sees the advanced count. A dropped cycle
//! leaves the phase base permanently lagged. All tick arithmetic is
//! wrapping `U32`, so the counter rolls over cleanly.
//!
//! The tick counter is a relaxed atomic (C++ guards `m_ticks` with its own
//! `Os::Mutex`; an atomic read-modify-write is the same contract). The
//! schedule table is touched only by the component thread after
//! registration; its mutex is dropped before a child is invoked.

use alloc::format;
use fprime_comp::{
    ActiveBase, ActiveComponent, ComponentDispatch, CyclePort, EventGlue, EventThrottle,
    MsgDispatchStatus, OutputPort, QueueFullPolicy, SchedPort, TlmGlue, async_input_port_adapter,
    component_msg_types, msg,
};
use fprime_config::{
    FwEnumStoreType, FwEventIdType, FwIndexType, FwQueuePriorityType, FwSizeType,
    RATE_GROUP_MEMBER_OUT_PORTS,
};
use fprime_fw::sync::{Arc, AtomicU32, Mutex, Ordering};
use fprime_fw::{LinearBuffer, LogBuffer, LogSeverity, SerBuf, SerBufAny, fw_assert, fw_try};
use fprime_os::RawTime;

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel): async input ports,
    /// then internal ports, in FPP declaration order.
    impl ActivePhaser {
        /// `CycleIn` async input port.
        MSG_TYPE_CYCLE_IN,
        /// `Tick` internal port.
        MSG_TYPE_INT_TICK,
    }
}

/// Queue message size: the `CycleIn` message, 6 (envelope) + 8 (RawTime).
const MSG_SIZE: usize = 16;

/// FPP declares no priorities: FIFO.
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

/// The one queue depth the component accepts.
pub const QUEUE_DEPTH: FwSizeType = 1;

/// How a phased call's context value is produced (C++
/// `PhaserContextType`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PhaserContextType {
    /// The entry's registration index among the entries on its port.
    #[default]
    Sequential,
    /// Phaser cycles elapsed, modulo the entry's cycle ratio.
    Count,
}

/// Result of finishing the running child (C++ `FinishStatus`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FinishStatus {
    /// Improper finish call: child not running, no child, etc.
    Unknown,
    /// Child finished on time.
    OnTime,
    /// Child finished late.
    Late,
}

/// One registered phased call (C++ `PhaserStateEntry`).
#[derive(Debug, Clone, Copy, Default)]
struct PhaserStateEntry {
    port: FwIndexType,
    start: u32,
    length: u32,
    context: u32,
    context_type: PhaserContextType,
    started: bool,
}

/// The schedule and its cursor (C++ `PhaserStateTable` plus the cycle
/// bookkeeping members).
struct PhaserState {
    /// C++ `m_cycle`: ticks per phaser cycle (0 = not configured).
    cycle: u32,
    /// C++ `m_ticks_rollover`: a common multiple of the COUNT ratios.
    ticks_rollover: u32,
    /// C++ `m_last_start_ticks`.
    last_start_ticks: u32,
    /// C++ `m_last_cycle_ticks`.
    last_cycle_ticks: u32,
    /// C++ `m_cycle_count`.
    cycle_count: u32,
    /// Number of registered entries.
    used: u32,
    /// The current entry index.
    current: u32,
    /// The entries, in registration order.
    entries: [PhaserStateEntry; ActivePhaser::MAX_CHILDREN as usize],
}

impl PhaserState {
    /// C++ `timeInCycle(full_ticks)`.
    fn time_in_cycle(&self, full_ticks: u32) -> u32 {
        full_ticks.wrapping_sub(self.last_cycle_ticks)
    }
}

/// `MissedDeadline` arguments.
struct MissedDeadline {
    port: FwIndexType,
    start: u32,
    length: u32,
    ticks: u32,
}

/// `Svc::ActivePhaser`.
pub struct ActivePhaser {
    /// Active core: PassiveBase + queue + task.
    pub active: ActiveBase,
    /// `logOut` / `logTextOut` / `timeCaller`.
    pub evt: EventGlue,
    /// `tlmOut` (declared by the FPP model; no channels).
    pub tlm: TlmGlue,
    /// `PhaserMemberOut: [ActiveRateGroupOutputPorts] Svc.Sched`.
    pub phaser_member_out: [OutputPort<dyn SchedPort>; RATE_GROUP_MEMBER_OUT_PORTS],
    /// FPP `throttle 5` on `MissedDeadline`.
    missed_deadline_throttle: EventThrottle,
    /// C++ `m_ticks` (its own `m_lock` in C++); starts at `0xFFFFFFFF` so
    /// the first cycle is tick 0.
    ticks: AtomicU32,
    /// The schedule (component thread after registration).
    state: Mutex<PhaserState>,
}

impl ActivePhaser {
    /// Maximum number of registered calls (C++ `MAX_CHILDREN`).
    pub const MAX_CHILDREN: u32 = 100;
    /// "Not specified" for `start` and `user_context` (C++ `DONT_CARE`).
    pub const DONT_CARE: u32 = 0xFFFF_FFFF;
    /// Number of `PhaserMemberOut` ports (C++ `CONNECTION_COUNT_MAX`).
    pub const CONNECTION_COUNT_MAX: FwIndexType = RATE_GROUP_MEMBER_OUT_PORTS as FwIndexType;

    /// `MissedDeadline(p: FwIndexType, start: U32, length: U32, ticks: U32)`
    /// — WARNING_HI, id 0, throttle 5.
    pub const EVENTID_MISSED_DEADLINE: FwEventIdType = 0;
    /// FPP `throttle 5` on `MissedDeadline`.
    pub const MISSED_DEADLINE_THROTTLE: u32 = 5;

    /// Construct (C++ constructor: empty table, ticks at `0xFFFFFFFF`,
    /// rollover 1).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            phaser_member_out: [const { OutputPort::new() }; RATE_GROUP_MEMBER_OUT_PORTS],
            missed_deadline_throttle: EventThrottle::new(Self::MISSED_DEADLINE_THROTTLE),
            ticks: AtomicU32::new(0xFFFF_FFFF),
            state: Mutex::new(PhaserState {
                cycle: 0,
                // "Start at 1. Will be multiplied by each context to find
                // some common multiple."
                ticks_rollover: 1,
                last_start_ticks: 0,
                last_cycle_ticks: 0,
                cycle_count: 0,
                used: 0,
                current: 0,
                entries: [PhaserStateEntry::default(); Self::MAX_CHILDREN as usize],
            }),
        })
    }

    /// C++ `init(queueDepth, instance)`: create the queue; asserts the
    /// depth is exactly one.
    pub fn init(&self, queue_depth: FwSizeType) {
        // Dependent on a queue depth of one to prevent a rush to catch up.
        fw_assert!(queue_depth == QUEUE_DEPTH, queue_depth as i32);
        self.active
            .queued
            .create_queue(QUEUE_DEPTH, MSG_SIZE as FwSizeType);
    }

    /// C++ `configure(cycle_ticks)`: ticks per phaser cycle (non-zero).
    pub fn configure(&self, cycle_ticks: u32) {
        fw_assert!(cycle_ticks != 0);
        self.state.lock().cycle = cycle_ticks;
    }

    /// C++ `register_phased(port, length, start = DONT_CARE, userContext =
    /// DONT_CARE)`: append a call of `PhaserMemberOut[port]` owning `length`
    /// ticks from `start` ([`DONT_CARE`](Self::DONT_CARE): right after the
    /// previous entry, or 0 for the first). With a `user_context` (in
    /// ticks, larger than the cycle) the call's context counts phaser
    /// cycles modulo `user_context / cycle_ticks`; with `DONT_CARE` it is
    /// the entry's index among the port's entries.
    ///
    /// Asserts (topology errors, C++ parity): not configured, table full,
    /// windows out of registration order or overlapping, port out of range
    /// or not connected, window outside the cycle, `user_context` not
    /// larger than the cycle, rollover overflow.
    pub fn register_phased(&self, port: FwIndexType, length: u32, start: u32, user_context: u32) {
        let mut state = self.state.lock();
        let st = &mut *state;
        fw_assert!(st.cycle != 0);
        fw_assert!(st.used < Self::MAX_CHILDREN, st.used, Self::MAX_CHILDREN);
        let mut start = start;
        if st.used > 0 {
            let previous = st.entries[(st.used - 1) as usize];
            // Must start after the previous entry.
            fw_assert!(
                previous.start + previous.length <= start,
                st.used,
                previous.start,
                start
            );
            fw_assert!(previous.start < start, st.used, previous.start, start);
            // DONT_CARE: right after the previous entry.
            if start == Self::DONT_CARE {
                start = previous.start + previous.length;
            }
        }
        // The first entry's DONT_CARE start is 0.
        if start == Self::DONT_CARE {
            start = 0;
        }

        fw_assert!((0..Self::CONNECTION_COUNT_MAX).contains(&port), port);
        fw_assert!(self.phaser_member_out[port as usize].is_connected(), port);
        fw_assert!(length <= st.cycle, length, st.cycle);
        fw_assert!(start <= st.cycle - length, start, length, st.cycle);
        fw_assert!(user_context > st.cycle, user_context, st.cycle);

        let (context, context_type) = if user_context != Self::DONT_CARE {
            // COUNT: the ratio of the user window to the phaser cycle.
            let ratio = user_context / st.cycle;
            // Check for overflow before multiplying.
            fw_assert!(u32::MAX / st.ticks_rollover >= ratio);
            st.ticks_rollover *= ratio;
            (ratio, PhaserContextType::Count)
        } else {
            (Self::next_context(st, port), PhaserContextType::Sequential)
        };
        st.entries[st.used as usize] = PhaserStateEntry {
            port,
            start,
            length,
            context,
            context_type,
            started: false,
        };
        st.used += 1;
    }

    /// C++ `getNextContext(port)`: one more than the context of the port's
    /// last registered entry (a COUNT entry's ratio counts too, as in C++).
    fn next_context(st: &PhaserState, port: FwIndexType) -> u32 {
        let mut context = 0;
        for entry in &st.entries[..st.used as usize] {
            if entry.port == port {
                context = entry.context + 1;
            }
        }
        context
    }

    fn id_base(&self) -> u32 {
        self.active.queued.base.get_id_base()
    }

    // -- Handlers (component thread) ---------------------------------------

    /// `CycleIn_handler`: count the tick, post `Tick`.
    fn cycle_in_handler(&self, _port_num: FwIndexType, _cycle_start: &RawTime) {
        self.ticks.fetch_add(1, Ordering::Relaxed);
        self.tick_internal_interface_invoke();
    }

    /// `Tick_internalInterfaceInvoke` — `drop` policy: a full queue drops
    /// the tick silently (counted in the base's dropped-message count).
    fn tick_internal_interface_invoke(&self) {
        let mut buf = LinearBuffer::<MSG_SIZE>::new();
        // The fprime-comp dispatch contract reads a port number after every
        // message type, internal ports included (port 0).
        let status = msg::write_envelope_header(&mut buf, Self::MSG_TYPE_INT_TICK, 0);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .active
            .queued
            .send_message(&buf, QUEUE_PRIORITY, QueueFullPolicy::Drop);
    }

    /// `Tick_internalInterfaceHandler`: roll the cycle over when it is
    /// done, finish the running child, start the next one.
    fn tick_internal_interface_handler(&self) {
        let mut missed: Option<MissedDeadline> = None;
        let mut invoke: Option<(FwIndexType, u32)> = None;
        {
            let mut state = self.state.lock();
            let st = &mut *state;
            fw_assert!(st.current <= st.used, st.current, st.used);
            let full_ticks = self.ticks.load(Ordering::Relaxed);

            // If the cycle is over (and every child ran), restart it.
            if st.time_in_cycle(full_ticks) >= st.cycle && st.current == st.used {
                st.last_cycle_ticks = full_ticks;
                // Cycle count modulo a common multiple of every ratio.
                fw_assert!(st.ticks_rollover != 0);
                st.cycle_count = st.cycle_count.wrapping_add(1) % st.ticks_rollover;
                st.current = 0;
            }
            // Run the next child unless the finishing one was late.
            if Self::finish_child(st, full_ticks, &mut missed) != FinishStatus::Late {
                invoke = Self::start_child(st, full_ticks);
            }
        }
        if let Some(m) = missed {
            if self.missed_deadline_throttle.ok_to_emit() {
                self.evt.log_event(
                    self.id_base(),
                    Self::EVENTID_MISSED_DEADLINE,
                    LogSeverity::WarningHi,
                    &format!(
                        "Port {} schedule at {} for {} ticks was long by {} ticks",
                        m.port, m.start, m.length, m.ticks
                    ),
                    |buf: &mut LogBuffer| {
                        fw_try!(buf.serialize_i16_be(m.port));
                        fw_try!(buf.serialize_u32_be(m.start));
                        fw_try!(buf.serialize_u32_be(m.length));
                        buf.serialize_u32_be(m.ticks)
                    },
                );
            }
        }
        if let Some((port, context)) = invoke {
            let p = self.phaser_member_out[port as usize].get();
            p.target.invoke(p.port_num, context);
        }
    }

    /// C++ `finishChild(full_ticks)`. The `MissedDeadline` event is handed
    /// back to be logged after the state lock is released.
    fn finish_child(
        st: &mut PhaserState,
        full_ticks: u32,
        missed: &mut Option<MissedDeadline>,
    ) -> FinishStatus {
        // Guard against finishing improperly.
        if st.current >= st.used || !st.entries[st.current as usize].started {
            return FinishStatus::Unknown;
        }
        let last_start_ticks = st.last_start_ticks;
        let used = st.used;
        let index = (st.current % used) as usize;
        let entry = &mut st.entries[index];
        let execution_time = full_ticks.wrapping_sub(last_start_ticks);
        let expected_time = entry.length;
        entry.started = false;
        let entry = *entry;
        st.current = if st.current == used {
            used
        } else {
            st.current + 1
        };
        // A deadline violation reports this child LATE.
        if execution_time > expected_time {
            *missed = Some(MissedDeadline {
                port: entry.port,
                start: entry.start,
                length: entry.length,
                ticks: execution_time - expected_time,
            });
            return FinishStatus::Late;
        }
        FinishStatus::OnTime
    }

    /// C++ `startChild(full_ticks)`: returns the port call to make.
    fn start_child(st: &mut PhaserState, full_ticks: u32) -> Option<(FwIndexType, u32)> {
        if st.current >= st.used
            || st.entries[st.current as usize].start > st.time_in_cycle(full_ticks)
            || st.entries[st.current as usize].started
        {
            return None;
        }
        let cycle_count = st.cycle_count;
        let index = (st.current % st.used) as usize;
        let entry = &mut st.entries[index];
        let mut context = entry.context;
        if entry.context_type != PhaserContextType::Sequential {
            fw_assert!(entry.context != 0, entry.port);
            context = cycle_count % entry.context;
        }
        entry.started = true;
        let port = entry.port;
        st.last_start_ticks = full_ticks;
        Some((port, context))
    }
}

async_input_port_adapter! {
    /// `CycleIn` — ASYNC `Svc.Cycle` input, `drop` policy.
    component: ActivePhaser;
    adapter: CycleInAdapter;
    port: CyclePort;
    input: pub cycle_in;
    deserialize: cycle_in_deserialize;
    handler: cycle_in_handler;
    base: active.queued;
    msg_type: ActivePhaser::MSG_TYPE_CYCLE_IN;
    msg_size: MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Drop;
    args { ref cycle_start: RawTime }
}

impl ComponentDispatch for ActivePhaser {
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
            Self::MSG_TYPE_CYCLE_IN => match Self::cycle_in_deserialize(buf) {
                Some((cycle_start,)) => {
                    self.cycle_in_handler(port_num, &cycle_start);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_INT_TICK => {
                self.tick_internal_interface_handler();
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for ActivePhaser {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    //! Port of `Svc/ActivePhaser/test/ut`: the `FauxPhaser` model, the
    //! tester and all eight tests (Nominal.SingleChild /
    //! MultipleChildren / UnrulyChildren, Lethargic.SingleChild /
    //! MultipleChildren / MultipleNonInitialChildren, Stress.YiWan /
    //! Rollover), plus the registration asserts and the async path.

    use super::*;
    use fprime_comp::LogPort;
    use fprime_fw::Time;
    use fprime_os::queue::BlockingType;
    use std::collections::{BTreeMap, VecDeque};
    use std::sync::Mutex as StdMutex;
    use std::vec::Vec;

    const DONT_CARE: u32 = ActivePhaser::DONT_CARE;
    const MAX_CYCLES: u64 = 100;

    /// Upstream `TestChild`.
    #[derive(Debug, Clone, Copy)]
    struct TestChild {
        port: FwIndexType,
        /// Execution time bound for a child task.
        length: u32,
        /// Actual execution time configured by the test case.
        actual_length: u32,
        actual_start: u32,
        start: u32,
        context: u32,
        /// Ticks the child has been running.
        runtime: u32,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum FauxState {
        Running,
        Idle,
        EndOfCycle,
        Starting,
    }

    /// Upstream `FauxPhaser`: the expected behavior, tick by tick.
    struct FauxPhaser {
        children: Vec<TestChild>,
        /// Index into `children`; `children.len()` is `end()`.
        current: usize,
        state: FauxState,
    }

    impl FauxPhaser {
        fn register_child(&mut self, mut child: TestChild) {
            child.runtime = 0;
            self.children.push(child);
            self.current = 0;
            self.state = FauxState::EndOfCycle;
        }

        fn run(&mut self, tick_of_cycle: u32, cycle_length: u32) -> FauxState {
            let end = self.children.len();
            if self.current == end && tick_of_cycle >= cycle_length {
                // End of cycle: reset.
                self.current = 0;
                self.children[0].runtime = 0;
                self.state = FauxState::EndOfCycle;
            } else if self.current == end
                || tick_of_cycle < self.children[self.current].actual_start
            {
                self.state = FauxState::Idle;
            } else if self.children[self.current].runtime
                == self.children[self.current].actual_length
                && matches!(self.state, FauxState::Running | FauxState::Starting)
            {
                // Runtime expired: on to the next child.
                let child = self.children[self.current];
                let finished_on_time = child.length == child.actual_length;
                self.current += 1;
                if self.current < end {
                    self.children[self.current].runtime = 0;
                }
                // A late child forces an idle tick before the next starts.
                self.state = if finished_on_time {
                    FauxState::Starting
                } else {
                    FauxState::Idle
                };
            } else if matches!(self.state, FauxState::Running | FauxState::Starting) {
                self.state = FauxState::Running;
            } else if self.state == FauxState::EndOfCycle
                || tick_of_cycle >= self.children[self.current].actual_start
            {
                self.children[self.current].runtime = 0;
                self.state = FauxState::Starting;
            } else {
                panic!("FauxPhaser reached an impossible state");
            }
            if self.current < end {
                self.children[self.current].runtime += 1;
            }
            self.state
        }

        fn active(&self) -> Option<&TestChild> {
            self.children.get(self.current)
        }
    }

    #[derive(Debug, Clone, Copy)]
    struct CallContext {
        port: FwIndexType,
        context: u32,
    }

    /// The tester's state, shared with the `PhaserMemberOut` handler.
    struct Harness {
        mock: FauxPhaser,
        calls: BTreeMap<FwIndexType, VecDeque<CallContext>>,
        ticks: u32,
        cycle: u32,
        nominal: bool,
    }

    /// The `PhaserMemberOut` receiver (upstream
    /// `from_PhaserMemberOut_handler`) and the event sink.
    struct Sink {
        harness: Arc<StdMutex<Harness>>,
        events: StdMutex<Vec<(FwEventIdType, Vec<u8>)>>,
    }

    impl SchedPort for Sink {
        fn invoke(&self, port_num: FwIndexType, context: u32) {
            let mut h = self.harness.lock().unwrap();
            let active = *h
                .mock
                .active()
                .expect("component run when none should be active");
            let tick = h.ticks % h.cycle;
            h.calls.entry(port_num).or_default().push_back(CallContext {
                port: port_num,
                context,
            });
            assert_eq!(active.port, port_num, "child attached to invalid port");
            if h.nominal {
                assert_eq!(active.actual_start, tick);
            } else {
                assert!(active.actual_start <= tick);
            }
        }
    }

    impl LogPort for Sink {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.events
                .lock()
                .unwrap()
                .push((id, args.as_slice().to_vec()));
        }
    }

    /// Upstream `ActivePhaserTester`.
    struct Tester {
        comp: Arc<ActivePhaser>,
        sink: Arc<Sink>,
        harness: Arc<StdMutex<Harness>>,
        children: Vec<TestChild>,
        start_counter: u32,
    }

    impl Tester {
        fn new() -> Self {
            let harness = Arc::new(StdMutex::new(Harness {
                mock: FauxPhaser {
                    children: Vec::new(),
                    current: 0,
                    state: FauxState::EndOfCycle,
                },
                calls: BTreeMap::new(),
                ticks: 0,
                cycle: 0,
                nominal: true,
            }));
            let sink = Arc::new(Sink {
                harness: harness.clone(),
                events: StdMutex::new(Vec::new()),
            });
            let comp = ActivePhaser::new("ActivePhaser");
            for (i, port) in comp.phaser_member_out.iter().enumerate() {
                port.connect(sink.clone(), i as FwIndexType);
            }
            comp.evt.log_out.connect(sink.clone(), 0);
            comp.init(QUEUE_DEPTH);
            Self {
                comp,
                sink,
                harness,
                children: Vec::new(),
                start_counter: 0,
            }
        }

        fn setup(&mut self, cycle: u32, nominal: bool) {
            {
                let mut h = self.harness.lock().unwrap();
                h.cycle = cycle;
                h.ticks = 0;
                h.nominal = nominal;
            }
            self.comp.configure(cycle);
        }

        fn create_child(
            &mut self,
            port: FwIndexType,
            length: u32,
            start: u32,
            context: u32,
            lethargy: u32,
        ) {
            let actual_start = if start != DONT_CARE {
                start
            } else {
                self.start_counter
            };
            let child = TestChild {
                port,
                length,
                actual_length: length + lethargy,
                actual_start,
                start,
                context,
                runtime: 0,
            };
            self.start_counter = child.actual_start + child.length;
            self.children.push(child);
            self.harness.lock().unwrap().mock.register_child(child);
            self.comp
                .register_phased(child.port, child.length, child.start, child.context);
        }

        fn child(&mut self, port: FwIndexType, length: u32) {
            self.create_child(port, length, DONT_CARE, DONT_CARE, 0);
        }

        /// Upstream `new_cycle`: one tick of both the component and the
        /// model; true at the end of a cycle.
        fn new_cycle(&mut self, cycle_number: u64) -> bool {
            let response = {
                let mut h = self.harness.lock().unwrap();
                let (ticks, cycle) = (h.ticks, h.cycle);
                h.mock.run(ticks, cycle)
            };
            // Calling the handler directly bypasses the CycleIn dispatch
            // (as upstream does); it posts the Tick message.
            self.comp.cycle_in_handler(0, &RawTime::new());
            match response {
                FauxState::EndOfCycle => {
                    self.check_cycle(cycle_number);
                    self.harness.lock().unwrap().ticks = 0;
                    return true;
                }
                FauxState::Idle | FauxState::Starting => {
                    let status = self
                        .comp
                        .active
                        .queued
                        .do_dispatch(&*self.comp, BlockingType::NonBlocking);
                    assert_eq!(status, MsgDispatchStatus::Ok);
                    self.harness.lock().unwrap().ticks += 1;
                }
                FauxState::Running => {
                    self.harness.lock().unwrap().ticks += 1;
                }
            }
            false
        }

        /// Upstream `check_cycle`: every child was called once, with the
        /// right context.
        fn check_cycle(&mut self, outer: u64) {
            let mut h = self.harness.lock().unwrap();
            let cycle = h.cycle;
            let mut counts: BTreeMap<FwIndexType, u32> = BTreeMap::new();
            for child in &self.children {
                let call = h
                    .calls
                    .get_mut(&child.port)
                    .and_then(VecDeque::pop_front)
                    .unwrap_or_else(|| {
                        panic!(
                            "child on port {} at {} not called in cycle {outer}",
                            child.port, child.actual_start
                        )
                    });
                assert_eq!(call.port, child.port, "unexpected port on call context");
                if child.context != DONT_CARE {
                    assert_eq!(
                        u64::from(call.context),
                        outer % u64::from(child.context / cycle),
                        "subdividing context bad for port {} and outer loop {outer}",
                        child.port
                    );
                } else {
                    let count = counts.entry(child.port).or_insert(0);
                    assert_eq!(*count, call.context, "sequential context bad");
                    *count += 1;
                }
            }
        }

        fn run_cycle(&mut self, i: u64) {
            while !self.new_cycle(i) {}
        }

        fn missed_deadlines(&self) -> Vec<(i16, u32, u32, u32)> {
            self.sink
                .events
                .lock()
                .unwrap()
                .iter()
                .filter(|(id, _)| *id == ActivePhaser::EVENTID_MISSED_DEADLINE)
                .map(|(_, a)| {
                    let u = |o: usize| u32::from_be_bytes([a[o], a[o + 1], a[o + 2], a[o + 3]]);
                    (i16::from_be_bytes([a[0], a[1]]), u(2), u(6), u(10))
                })
                .collect()
        }

        fn clear_events(&self) {
            self.sink.events.lock().unwrap().clear();
        }

        /// Upstream `test_lethargic_children(adjust_initial, cycles,
        /// tick_start)`. Like upstream, `cycles` is accepted but the loop
        /// runs `MAX_CYCLES` cycles.
        fn lethargic_children(&mut self, adjust_initial: bool, _cycles: u64, tick_start: u32) {
            self.setup(100, false);
            let mut throttle: i32 = 5;
            self.create_child(
                0,
                5,
                if adjust_initial { 2 } else { DONT_CARE },
                DONT_CARE,
                5,
            );
            self.create_child(0, 5, 8, DONT_CARE, 1);
            self.create_child(1, 5, DONT_CARE, 1000, 5);
            self.create_child(0, 1, 23, DONT_CARE, 0);
            self.create_child(2, 1, DONT_CARE, DONT_CARE, 1);
            self.create_child(5, 30, 25, 2000, 80);

            // Fast-forward time.
            if tick_start != 0xFFFF_FFFF {
                let cycle = self.harness.lock().unwrap().cycle;
                let tick_start = tick_start / cycle * cycle;
                // Right before the last start.
                self.comp.ticks.store(tick_start - 1, Ordering::Relaxed);
                let mut st = self.comp.state.lock();
                st.last_cycle_ticks = tick_start - cycle;
                st.current = st.used;
                st.cycle_count = st.cycle_count.wrapping_sub(1);
            }
            for i in 0..MAX_CYCLES {
                self.run_cycle(i);
                if throttle > 0 {
                    let first = if adjust_initial { 2 } else { 0 };
                    assert_eq!(
                        self.missed_deadlines(),
                        vec![
                            (0, first, 5, 5),
                            (0, 8, 5, 1),
                            (1, 13, 5, 5),
                            (2, 24, 1, 1),
                            (5, 25, 30, 80),
                        ]
                    );
                    self.clear_events();
                    throttle -= 5;
                }
            }
        }
    }

    #[test]
    fn nominal_single_child() {
        let mut t = Tester::new();
        t.setup(100, true);
        t.child(0, 5);
        for i in 0..MAX_CYCLES {
            t.run_cycle(i);
            assert!(t.missed_deadlines().is_empty());
        }
    }

    #[test]
    fn nominal_multiple_children() {
        let mut t = Tester::new();
        t.setup(100, true);
        t.child(0, 5);
        t.child(0, 5);
        t.child(1, 5);
        for i in 0..MAX_CYCLES {
            t.run_cycle(i);
            assert!(t.missed_deadlines().is_empty());
        }
    }

    #[test]
    fn nominal_unruly_children() {
        let mut t = Tester::new();
        t.setup(100, true);
        t.child(0, 5);
        t.create_child(0, 5, 8, DONT_CARE, 0);
        t.create_child(1, 5, DONT_CARE, 1000, 0);
        t.create_child(0, 1, 23, DONT_CARE, 0);
        t.child(2, 1);
        t.create_child(5, 30, 25, 2000, 0);
        for i in 0..MAX_CYCLES {
            t.run_cycle(i);
            assert!(t.missed_deadlines().is_empty());
        }
    }

    #[test]
    fn lethargic_single_child() {
        let mut t = Tester::new();
        t.setup(100, false);
        let mut throttle = 5;
        t.create_child(0, 5, DONT_CARE, DONT_CARE, 5);
        for i in 0..MAX_CYCLES {
            t.run_cycle(i);
            // Checked as long as the event throttle lets it through.
            if throttle > 0 {
                assert_eq!(t.missed_deadlines(), vec![(0, 0, 5, 5)]);
                t.clear_events();
                throttle -= 1;
            }
            t.harness.lock().unwrap().ticks = 0;
        }
        // Throttled after five.
        assert!(t.missed_deadlines().is_empty());
    }

    #[test]
    fn lethargic_multiple_children() {
        Tester::new().lethargic_children(false, MAX_CYCLES, 0xFFFF_FFFF);
    }

    #[test]
    fn lethargic_multiple_non_initial_children() {
        Tester::new().lethargic_children(true, MAX_CYCLES, 0xFFFF_FFFF);
    }

    #[test]
    fn stress_yi_wan() {
        Tester::new().lethargic_children(false, 10_000, 0xFFFF_FFFF);
    }

    #[test]
    fn stress_rollover() {
        let cycle_start: u32 = 4_294_960_000;
        let mut t = Tester::new();
        t.lethargic_children(true, 1000, cycle_start);
        assert!(
            t.comp.ticks.load(Ordering::Relaxed) < cycle_start,
            "component rollover failed"
        );
    }

    #[test]
    fn registration_contexts_and_starts() {
        let t = Tester::new();
        t.comp.configure(100);
        t.comp.register_phased(0, 5, DONT_CARE, DONT_CARE);
        t.comp.register_phased(0, 5, DONT_CARE, DONT_CARE);
        t.comp.register_phased(1, 10, 40, 1000);
        t.comp.register_phased(1, 10, DONT_CARE, 300);
        t.comp.register_phased(0, 1, DONT_CARE, DONT_CARE);
        let st = t.comp.state.lock();
        let summary: Vec<_> = st.entries[..st.used as usize]
            .iter()
            .map(|e| (e.port, e.start, e.length, e.context, e.context_type))
            .collect();
        use PhaserContextType::{Count, Sequential};
        assert_eq!(
            summary,
            vec![
                (0, 0, 5, 0, Sequential),
                (0, 5, 5, 1, Sequential),
                (1, 40, 10, 10, Count),
                (1, 50, 10, 3, Count),
                (0, 60, 1, 2, Sequential),
            ]
        );
        assert_eq!(st.ticks_rollover, 30);
    }

    #[test]
    fn async_cycle_drives_the_schedule() {
        // Through the real CycleIn port: CycleIn and Tick share the
        // one-deep queue, so each cycle needs two dispatches.
        let t = Tester::new();
        {
            let mut h = t.harness.lock().unwrap();
            h.cycle = 4;
            h.nominal = true;
            let child = TestChild {
                port: 3,
                length: 1,
                actual_length: 1,
                actual_start: 0,
                start: 0,
                context: DONT_CARE,
                runtime: 0,
            };
            h.mock.register_child(child);
        }
        t.comp.configure(4);
        t.comp.register_phased(3, 1, 0, DONT_CARE);
        let cycle = t.comp.cycle_in(0);
        cycle.target.invoke(cycle.port_num, &RawTime::new());
        // A second cycle while the first is queued is dropped.
        cycle.target.invoke(cycle.port_num, &RawTime::new());
        assert_eq!(t.comp.active.queued.get_num_msgs_dropped(), 1);
        for _ in 0..2 {
            let status = t
                .comp
                .active
                .queued
                .do_dispatch(&*t.comp, BlockingType::NonBlocking);
            assert_eq!(status, MsgDispatchStatus::Ok);
        }
        let calls = &t.harness.lock().unwrap().calls;
        assert_eq!(calls[&3].len(), 1);
        assert_eq!(calls[&3][0].context, 0);
    }

    #[test]
    #[should_panic]
    fn init_requires_a_queue_depth_of_one() {
        let comp = ActivePhaser::new("phaser");
        comp.init(2);
    }

    #[test]
    #[should_panic]
    fn register_before_configure_asserts() {
        let t = Tester::new();
        t.comp.register_phased(0, 1, DONT_CARE, DONT_CARE);
    }

    #[test]
    #[should_panic]
    fn overlapping_window_asserts() {
        let t = Tester::new();
        t.comp.configure(100);
        t.comp.register_phased(0, 10, 0, DONT_CARE);
        t.comp.register_phased(1, 10, 5, DONT_CARE);
    }

    #[test]
    #[should_panic]
    fn window_past_the_cycle_asserts() {
        let t = Tester::new();
        t.comp.configure(100);
        t.comp.register_phased(0, 10, 95, DONT_CARE);
    }

    #[test]
    #[should_panic]
    fn unconnected_port_asserts() {
        let comp = ActivePhaser::new("phaser");
        comp.configure(100);
        comp.register_phased(0, 1, DONT_CARE, DONT_CARE);
    }

    #[test]
    #[should_panic]
    fn user_context_within_the_cycle_asserts() {
        let t = Tester::new();
        t.comp.configure(100);
        t.comp.register_phased(0, 1, DONT_CARE, 100);
    }
}
