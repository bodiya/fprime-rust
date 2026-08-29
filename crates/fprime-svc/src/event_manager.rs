//! # EventManager — port of `Svc::EventManager` (active; fork rename of
//! `ActiveLogger`)
//!
//! C++ sources: `Svc/EventManager/EventManager.{cpp,hpp,fpp}`,
//! `Svc/Types/EventSeverityFilter/EventSeverityFilter.{cpp,hpp}`,
//! `default/config/EventManagerCfg.hpp`.
//! Analysis: `docs/cpp-analysis/svc-core.md` (EventManager section + gotchas).
//!
//! Receives events on the **sync** `LogRecv` port (producer thread), applies
//! the severity filter and the mutex-guarded ID filter, re-queues surviving
//! events through the internal async `loqQueue` port (FPP `drop` policy) to
//! its own thread, serializes them as `FW_PACKET_LOG` com packets to
//! `PktSend`, and announces FATALs on `FatalAnnounce` — on the **caller's**
//! thread, after the enqueue.
//!
//! Ported quirks (each covered by a unit test):
//! - [`EventManagerEnabled`] `{ENABLED=0, DISABLED=1}` is REVERSED vs
//!   `Fw::Enabled {Disabled=0, Enabled=1}`.
//! - FATAL bypasses both the severity and the ID filter.
//! - `SET_EVENT_FILTER` (opcode 0) is a SYNC command: the command input
//!   adapter runs the handler directly on the dispatcher's thread.
//!   `SET_ID_FILTER` (opcode 2) and `DUMP_FILTER_STATE` (opcode 3) are
//!   async. There is deliberately no opcode 1.
//! - Serialize failures and (in C++) invalid severities fall back to
//!   `Fw::Logger`, not to events.
//! - `EventsDropped` telemetry (id 0, on change) surfaces the component
//!   base `m_msgsDropped` counter fed by the `drop`-policy queue sends.

use fprime_comp::msg;
use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, ComPort, ComponentDispatch, EventGlue,
    FatalEventPort, LogPort, MsgDispatchStatus, OutputPort, PingPort, PortRef, QueueFullPolicy,
    SchedPort, TlmGlue,
};
use fprime_config::event_manager::{
    FILTER_ACTIVITY_HI_DEFAULT, FILTER_ACTIVITY_LO_DEFAULT, FILTER_COMMAND_DEFAULT,
    FILTER_DIAGNOSTIC_DEFAULT, FILTER_WARNING_HI_DEFAULT, FILTER_WARNING_LO_DEFAULT,
    ID_FILTER_SIZE,
};
use fprime_config::{
    FwChanIdType, FwEnumStoreType, FwEventIdType, FwIndexType, FwOpcodeType, FwQueuePriorityType,
    FwSizeType,
};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, ComBuffer, Endianness, LinearBuffer, LogBuffer, LogPacket,
    LogSeverity, SerBuf, SerBufAny, Serialize, SerializeStatus, Time, fw_assert, fw_log,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// FPP types.
// ---------------------------------------------------------------------------

/// `Svc::EventManager_FilterSeverity` (U8) — severity index for filter
/// commands. Same order as [`EventSeverityFilter`] indices; FATAL is
/// deliberately absent (never filterable).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterSeverity {
    /// Filter WARNING_HI events.
    WarningHi = 0,
    /// Filter WARNING_LO events.
    WarningLo = 1,
    /// Filter COMMAND events.
    Command = 2,
    /// Filter ACTIVITY_HI events.
    ActivityHi = 3,
    /// Filter ACTIVITY_LO events.
    ActivityLo = 4,
    /// Filter DIAGNOSTIC events.
    Diagnostic = 5,
}

impl FilterSeverity {
    /// Number of filterable severity levels (`NUM_CONSTANTS`).
    pub const NUM_CONSTANTS: usize = 6;

    /// Dictionary name (used in event text).
    pub fn name(self) -> &'static str {
        match self {
            FilterSeverity::WarningHi => "WARNING_HI",
            FilterSeverity::WarningLo => "WARNING_LO",
            FilterSeverity::Command => "COMMAND",
            FilterSeverity::ActivityHi => "ACTIVITY_HI",
            FilterSeverity::ActivityLo => "ACTIVITY_LO",
            FilterSeverity::Diagnostic => "DIAGNOSTIC",
        }
    }

    /// Filter index -> variant (`EventSeverityFilter::fromIndex` shape).
    pub fn from_index(index: usize) -> Option<Self> {
        match index {
            0 => Some(FilterSeverity::WarningHi),
            1 => Some(FilterSeverity::WarningLo),
            2 => Some(FilterSeverity::Command),
            3 => Some(FilterSeverity::ActivityHi),
            4 => Some(FilterSeverity::ActivityLo),
            5 => Some(FilterSeverity::Diagnostic),
            _ => None,
        }
    }

    /// The `Fw::LogSeverity` this filter index covers.
    pub fn to_log_severity(self) -> LogSeverity {
        match self {
            FilterSeverity::WarningHi => LogSeverity::WarningHi,
            FilterSeverity::WarningLo => LogSeverity::WarningLo,
            FilterSeverity::Command => LogSeverity::Command,
            FilterSeverity::ActivityHi => LogSeverity::ActivityHi,
            FilterSeverity::ActivityLo => LogSeverity::ActivityLo,
            FilterSeverity::Diagnostic => LogSeverity::Diagnostic,
        }
    }
}

impl TryFrom<u8> for FilterSeverity {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, ()> {
        Self::from_index(value as usize).ok_or(())
    }
}

impl Serialize for FilterSeverity {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        buf.serialize_u8(*self as u8, e)
    }
    fn serialized_size(&self) -> usize {
        1
    }
}

/// `Svc::EventManager_Enabled` (U8). **REVERSED** vs `Fw::Enabled`
/// (`{Disabled=0, Enabled=1}`) — confusing them silently inverts filter
/// commands (gotcha; ported faithfully).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventManagerEnabled {
    /// Enabled state (0!).
    Enabled = 0,
    /// Disabled state (1!).
    Disabled = 1,
}

impl TryFrom<u8> for EventManagerEnabled {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, ()> {
        match value {
            0 => Ok(EventManagerEnabled::Enabled),
            1 => Ok(EventManagerEnabled::Disabled),
            _ => Err(()),
        }
    }
}

// ---------------------------------------------------------------------------
// EventSeverityFilter (Svc/Types/EventSeverityFilter).
// ---------------------------------------------------------------------------

/// Port of `Svc::EventSeverityFilter`: per-severity enabled state, FATAL
/// never filterable. The C++ bool array is read/written from multiple
/// threads without a lock (benign race by design); relaxed atomics keep
/// that lock-free shape in safe Rust (see CONVENTIONS: racy plain fields
/// become relaxed atomics). Shared with `PassiveTextLogger`.
#[derive(Debug)]
pub struct EventSeverityFilter {
    enabled: [AtomicBool; Self::NUM_FILTER_LEVELS],
}

impl EventSeverityFilter {
    /// Number of filterable severity levels (excludes FATAL).
    pub const NUM_FILTER_LEVELS: usize = 6;

    /// All levels enabled (events pass through), as the C++ constructor.
    pub fn new() -> Self {
        Self {
            enabled: std::array::from_fn(|_| AtomicBool::new(true)),
        }
    }

    fn index(severity: LogSeverity) -> Option<usize> {
        match severity {
            LogSeverity::WarningHi => Some(0),
            LogSeverity::WarningLo => Some(1),
            LogSeverity::Command => Some(2),
            LogSeverity::ActivityHi => Some(3),
            LogSeverity::ActivityLo => Some(4),
            LogSeverity::Diagnostic => Some(5),
            LogSeverity::Fatal => None,
        }
    }

    /// Set the filter state for a severity level (FATAL is ignored).
    pub fn set_filter(&self, severity: LogSeverity, enabled: bool) {
        if let Some(i) = Self::index(severity) {
            self.enabled[i].store(enabled, Ordering::Relaxed);
        }
    }

    /// True if an event of `severity` should be dropped (FATAL: never).
    pub fn is_filtered(&self, severity: LogSeverity) -> bool {
        match Self::index(severity) {
            Some(i) => !self.enabled[i].load(Ordering::Relaxed),
            None => false,
        }
    }

    /// True if events of `severity` pass through (FATAL: always).
    pub fn is_enabled(&self, severity: LogSeverity) -> bool {
        match Self::index(severity) {
            Some(i) => self.enabled[i].load(Ordering::Relaxed),
            None => true,
        }
    }
}

impl Default for EventSeverityFilter {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// ID filter set: Fw::ArraySet<FwEventIdType, 25> behind its own mutex.
// ---------------------------------------------------------------------------

/// Fixed-capacity set with C++ `ArraySet` semantics: inserting an existing
/// element succeeds; a full set fails only for new elements.
struct IdFilterSet {
    ids: Vec<FwEventIdType>, // capacity ID_FILTER_SIZE, never grows past it
}

impl IdFilterSet {
    fn new() -> Self {
        Self {
            ids: Vec::with_capacity(ID_FILTER_SIZE),
        }
    }
    fn insert(&mut self, id: FwEventIdType) -> bool {
        if self.ids.contains(&id) {
            return true;
        }
        if self.ids.len() >= ID_FILTER_SIZE {
            return false;
        }
        self.ids.push(id);
        true
    }
    fn remove(&mut self, id: FwEventIdType) -> bool {
        match self.ids.iter().position(|&x| x == id) {
            Some(pos) => {
                self.ids.remove(pos);
                true
            }
            None => false,
        }
    }
    fn contains(&self, id: FwEventIdType) -> bool {
        self.ids.contains(&id)
    }
    /// Snapshot into a fixed array (no allocation), C++ DUMP pattern.
    fn snapshot(&self) -> ([FwEventIdType; ID_FILTER_SIZE], usize) {
        let mut out = [0; ID_FILTER_SIZE];
        let count = self.ids.len();
        out[..count].copy_from_slice(&self.ids);
        (out, count)
    }
}

// ---------------------------------------------------------------------------
// Dictionary constants.
// ---------------------------------------------------------------------------

/// Queue message type for the internal `loqQueue` port (FPP `drop`).
pub const MSG_TYPE_LOQ_QUEUE: FwEnumStoreType = 1;
/// Queue message type for the async `run` (Sched) input (FPP `drop`).
pub const MSG_TYPE_RUN: FwEnumStoreType = 2;
/// Queue message type for the async `pingIn` input.
pub const MSG_TYPE_PING_IN: FwEnumStoreType = 3;
/// Queue message type for the async commands (opcodes 2 and 3).
pub const MSG_TYPE_CMD: FwEnumStoreType = 4;

/// Queue message size: max over async invocations. The largest is the
/// internal `loqQueue` message: 6 (envelope) + 4 (id) + 11 (Time) +
/// 1 (severity) + 2 + 506 (length-prefixed `LogBuffer`) = 530. (The async
/// command envelope is 6 + 4 + 4 + 2 + 506 = 522.)
pub const QUEUE_MSG_SIZE: usize = 530;

const QUEUE_PRIORITY: FwQueuePriorityType = 1;

impl EventManager {
    /// `SET_EVENT_FILTER` — SYNC command (runs on the dispatcher thread).
    pub const OPCODE_SET_EVENT_FILTER: FwOpcodeType = 0;
    /// `SET_ID_FILTER` — async. NOTE: there is no opcode 1 (deliberate gap;
    /// `SET_EVENT_REPORT_FILTER` was removed upstream).
    pub const OPCODE_SET_ID_FILTER: FwOpcodeType = 2;
    /// `DUMP_FILTER_STATE` — async.
    pub const OPCODE_DUMP_FILTER_STATE: FwOpcodeType = 3;

    /// `SEVERITY_FILTER_STATE` (ACTIVITY_LO).
    pub const EVENTID_SEVERITY_FILTER_STATE: FwEventIdType = 0;
    /// `ID_FILTER_ENABLED` (ACTIVITY_HI).
    pub const EVENTID_ID_FILTER_ENABLED: FwEventIdType = 1;
    /// `ID_FILTER_LIST_FULL` (WARNING_LO).
    pub const EVENTID_ID_FILTER_LIST_FULL: FwEventIdType = 2;
    /// `ID_FILTER_REMOVED` (ACTIVITY_HI).
    pub const EVENTID_ID_FILTER_REMOVED: FwEventIdType = 3;
    /// `ID_FILTER_NOT_FOUND` (WARNING_LO).
    pub const EVENTID_ID_FILTER_NOT_FOUND: FwEventIdType = 4;

    /// `EventsDropped` telemetry channel (FwSizeType, on change).
    pub const CHANID_EVENTS_DROPPED: FwChanIdType = 0;
}

// ---------------------------------------------------------------------------
// The component.
// ---------------------------------------------------------------------------

/// `Svc::EventManager` — active event logger/downlinker.
pub struct EventManager {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Command registration/response glue.
    pub cmd: CmdGlue,
    /// Own event ports (binary + text) and time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// `PktSend`: `Fw.Com` out (serialized `FW_PACKET_LOG` packets).
    pub pkt_send: OutputPort<dyn ComPort>,
    /// `FatalAnnounce`: `Svc.FatalEvent` out.
    pub fatal_announce: OutputPort<dyn FatalEventPort>,
    /// `pingOut`: `Svc.Ping` out.
    pub ping_out: OutputPort<dyn PingPort>,
    /// Severity filter — lock-free (C++ has no lock here either).
    severity_filter: EventSeverityFilter,
    /// ID filter behind its own dedicated mutex (C++ `m_idFilterLock`):
    /// mutated by commands on the component thread, read by the sync
    /// `LogRecv` on every producer thread.
    id_filter: Mutex<IdFilterSet>,
    /// `update on change` cache for `EventsDropped`.
    last_dropped: Mutex<Option<FwSizeType>>,
}

impl EventManager {
    /// Construct with the severity-filter defaults from
    /// `EventManagerCfg.hpp` (DIAGNOSTIC filtered, everything else passes).
    pub fn new(name: &str) -> Arc<Self> {
        let severity_filter = EventSeverityFilter::new();
        severity_filter.set_filter(LogSeverity::WarningHi, FILTER_WARNING_HI_DEFAULT);
        severity_filter.set_filter(LogSeverity::WarningLo, FILTER_WARNING_LO_DEFAULT);
        severity_filter.set_filter(LogSeverity::Command, FILTER_COMMAND_DEFAULT);
        severity_filter.set_filter(LogSeverity::ActivityHi, FILTER_ACTIVITY_HI_DEFAULT);
        severity_filter.set_filter(LogSeverity::ActivityLo, FILTER_ACTIVITY_LO_DEFAULT);
        severity_filter.set_filter(LogSeverity::Diagnostic, FILTER_DIAGNOSTIC_DEFAULT);
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            pkt_send: OutputPort::new(),
            fatal_announce: OutputPort::new(),
            ping_out: OutputPort::new(),
            severity_filter,
            id_filter: Mutex::new(IdFilterSet::new()),
            last_dropped: Mutex::new(None),
        })
    }

    /// Create the message queue (topology phase; C++ `init(queueDepth)`).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    fn id_base(&self) -> u32 {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_SET_EVENT_FILTER,
                Self::OPCODE_SET_ID_FILTER,
                Self::OPCODE_DUMP_FILTER_STATE,
            ],
        );
    }

    // -- Input-port factories -----------------------------------------------

    /// `LogRecv` — SYNC `Fw.Log` input; filtering runs on the caller's
    /// thread.
    pub fn log_recv_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn LogPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `run` — ASYNC `Svc.Sched` input with the `drop` queue-full policy.
    pub fn run_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(Arc::new(RunInAdapter { comp: self.clone() }), port_num)
    }

    /// `pingIn` — ASYNC `Svc.Ping` input.
    pub fn ping_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn PingPort> {
        PortRef::new(Arc::new(PingInAdapter { comp: self.clone() }), port_num)
    }

    /// `CmdDisp` — command-dispatch input. `SET_EVENT_FILTER` executes
    /// synchronously on the calling thread; the other opcodes enqueue.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }

    // -- Handlers -----------------------------------------------------------

    /// `LogRecv_handler` — SYNC, caller thread.
    ///
    /// C++ first rejects invalid severity values (they can arrive from a
    /// hub bridging another address space) with an `Fw::Logger` message.
    /// The Rust `LogSeverity` enum cannot hold an invalid value — the
    /// rejection happens at whatever boundary deserialized it — so that
    /// branch is unrepresentable here.
    fn log_recv_handler(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        time_tag: &Time,
        severity: LogSeverity,
        args: &LogBuffer,
    ) {
        // Severity filter (FATAL always passes through).
        if self.severity_filter.is_filtered(severity) {
            return;
        }
        // ID filter (lock scope = the find only, C++ parity).
        let id_filtered = self.id_filter.lock().unwrap().contains(id);
        if id_filtered && severity != LogSeverity::Fatal {
            return;
        }
        // Enqueue to the component thread; `drop` policy counts overflow
        // in the base m_msgsDropped counter.
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_LOQ_QUEUE, 0);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(id);
        fw_assert!(status.is_ok(), status as i32);
        let status = time_tag.serialize_to(&mut buf, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = severity.serialize_to(&mut buf, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(args, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .active
            .queued
            .send_message(&buf, QUEUE_PRIORITY, QueueFullPolicy::Drop);

        // Announce the FATAL on the caller thread, AFTER the enqueue
        // (C++ parity).
        if severity == LogSeverity::Fatal {
            if let Some(p) = self.fatal_announce.try_get() {
                p.target.invoke(p.port_num, id);
            }
        }
    }

    /// `loqQueue_internalInterfaceHandler` — component thread: serialize
    /// the LogPacket and send it out `PktSend`.
    fn loq_queue_handler(
        &self,
        id: FwEventIdType,
        time_tag: Time,
        _severity: LogSeverity,
        args: &LogBuffer,
    ) {
        let mut packet = LogPacket::new();
        packet.set_id(id);
        packet.set_time_tag(time_tag);
        packet.set_log_buffer(args);
        let mut com_buffer = ComBuffer::new();
        let stat = packet.serialize_to(&mut com_buffer, Endianness::Big);
        // A maximum-size LogBuffer plus the packet header exceeds the com
        // buffer capacity: drop with an Fw::Logger message, not an event
        // or an assert (C++ parity).
        if !stat.is_ok() {
            fw_log!(
                "[ERROR] EventManager: dropping event 0x{:x} (serialize status {})\n",
                id,
                stat as i32
            );
            return;
        }
        if let Some(p) = self.pkt_send.try_get() {
            p.target.invoke(p.port_num, &mut com_buffer, 0);
        }
    }

    /// `run_handler` — component thread; `EventsDropped` on-change.
    fn run_handler(&self, _port_num: FwIndexType, _context: u32) {
        let dropped = self.active.queued.get_num_msgs_dropped();
        let changed = {
            let mut last = self.last_dropped.lock().unwrap();
            if *last == Some(dropped) {
                false
            } else {
                *last = Some(dropped);
                true
            }
        };
        if changed {
            self.tlm.tlm_write(
                self.id_base(),
                Self::CHANID_EVENTS_DROPPED,
                &dropped,
                self.evt.time_get(),
            );
        }
    }

    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }

    // -- Command handlers ---------------------------------------------------

    /// `SET_EVENT_FILTER` — SYNC (dispatcher thread).
    fn set_event_filter_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut level_raw = 0u8;
        if !args.deserialize_u8_be(&mut level_raw).is_ok() {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let Ok(filter_level) = FilterSeverity::try_from(level_raw) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        let mut enable_raw = 0u8;
        if !args.deserialize_u8_be(&mut enable_raw).is_ok() {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let Ok(filter_enable) = EventManagerEnabled::try_from(enable_raw) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        self.severity_filter.set_filter(
            filter_level.to_log_severity(),
            filter_enable == EventManagerEnabled::Enabled,
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `SET_ID_FILTER` — async (component thread).
    fn set_id_filter_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut id: FwEventIdType = 0;
        if !args.deserialize_u32_be(&mut id).is_ok() {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let mut enable_raw = 0u8;
        if !args.deserialize_u8_be(&mut enable_raw).is_ok() {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let Ok(id_enabled) = EventManagerEnabled::try_from(enable_raw) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let id_base = self.id_base();
        if id_enabled == EventManagerEnabled::Enabled {
            // Add the ID.
            let inserted = self.id_filter.lock().unwrap().insert(id);
            if inserted {
                // C++ order: response first, then the event.
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
                self.evt.log_event(
                    id_base,
                    Self::EVENTID_ID_FILTER_ENABLED,
                    LogSeverity::ActivityHi,
                    &format!("ID {id} is filtered."),
                    |buf| buf.serialize_u32_be(id),
                );
            } else {
                self.evt.log_event(
                    id_base,
                    Self::EVENTID_ID_FILTER_LIST_FULL,
                    LogSeverity::WarningLo,
                    &format!("ID filter list is full. Cannot filter {id} ."),
                    |buf| buf.serialize_u32_be(id),
                );
                self.cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            }
        } else {
            // Remove the ID.
            let removed = self.id_filter.lock().unwrap().remove(id);
            if removed {
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
                self.evt.log_event(
                    id_base,
                    Self::EVENTID_ID_FILTER_REMOVED,
                    LogSeverity::ActivityHi,
                    &format!("ID filter ID {id} removed."),
                    |buf| buf.serialize_u32_be(id),
                );
            } else {
                self.evt.log_event(
                    id_base,
                    Self::EVENTID_ID_FILTER_NOT_FOUND,
                    LogSeverity::WarningLo,
                    &format!("ID filter ID {id} not found."),
                    |buf| buf.serialize_u32_be(id),
                );
                self.cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            }
        }
    }

    /// `DUMP_FILTER_STATE` — async (component thread).
    fn dump_filter_state_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let id_base = self.id_base();
        // Severity filter states.
        for index in 0..FilterSeverity::NUM_CONSTANTS {
            // from_index cannot fail for 0..NUM_CONSTANTS (C++ FW_ASSERT).
            let Some(filter_state) = FilterSeverity::from_index(index) else {
                fw_assert!(false, index as i32);
                return;
            };
            let enabled = self
                .severity_filter
                .is_enabled(filter_state.to_log_severity());
            self.evt.log_event(
                id_base,
                Self::EVENTID_SEVERITY_FILTER_STATE,
                LogSeverity::ActivityLo,
                &format!("{} filter state. {}", filter_state.name(), enabled),
                |buf| {
                    let status = filter_state.serialize_to(buf, Endianness::Big);
                    if !status.is_ok() {
                        return status;
                    }
                    buf.serialize_bool_be(enabled)
                },
            );
        }
        // Snapshot the ID filter under the lock; log after release (C++
        // parity: LogRecv is sync and takes the same lock).
        let (ids, count) = self.id_filter.lock().unwrap().snapshot();
        for &id in &ids[..count] {
            self.evt.log_event(
                id_base,
                Self::EVENTID_ID_FILTER_ENABLED,
                LogSeverity::ActivityHi,
                &format!("ID {id} is filtered."),
                |buf| buf.serialize_u32_be(id),
            );
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }
}

// ---------------------------------------------------------------------------
// Sync input port: LogRecv implemented directly on the component.
// ---------------------------------------------------------------------------

impl LogPort for EventManager {
    fn invoke(
        &self,
        port_num: FwIndexType,
        id: FwEventIdType,
        time_tag: &mut Time,
        severity: LogSeverity,
        args: &mut LogBuffer,
    ) {
        self.log_recv_handler(port_num, id, time_tag, severity, args);
    }
}

// ---------------------------------------------------------------------------
// Async input adapters.
// ---------------------------------------------------------------------------

type MsgBuffer = LinearBuffer<QUEUE_MSG_SIZE>;

struct RunInAdapter {
    comp: Arc<EventManager>,
}

impl SchedPort for RunInAdapter {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_RUN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(context);
        fw_assert!(status.is_ok(), status as i32);
        // FPP `drop` policy on the run port.
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, QUEUE_PRIORITY, QueueFullPolicy::Drop);
    }
}

struct PingInAdapter {
    comp: Arc<EventManager>,
}

impl PingPort for PingInAdapter {
    fn invoke(&self, port_num: FwIndexType, key: u32) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_PING_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(key);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, QUEUE_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// Command-dispatch adapter: SYNC commands execute on the calling thread,
/// async commands enqueue, unknown opcodes answer InvalidOpcode
/// immediately — exactly what the generated `CmdDisp` handler does.
struct CmdInAdapter {
    comp: Arc<EventManager>,
}

impl CmdPort for CmdInAdapter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.comp.id_base()) {
            EventManager::OPCODE_SET_EVENT_FILTER => {
                // SYNC command: run the handler on the caller's thread.
                self.comp
                    .set_event_filter_cmd_handler(op_code, cmd_seq, args);
            }
            EventManager::OPCODE_SET_ID_FILTER | EventManager::OPCODE_DUMP_FILTER_STATE => {
                let mut buf = MsgBuffer::new();
                let status = msg::write_envelope_header(&mut buf, MSG_TYPE_CMD, port_num);
                fw_assert!(status.is_ok(), status as i32);
                let status = buf.serialize_u32_be(op_code);
                fw_assert!(status.is_ok(), status as i32);
                let status = buf.serialize_u32_be(cmd_seq);
                fw_assert!(status.is_ok(), status as i32);
                let status = buf.serialize_buffer(args, Endianness::Big);
                fw_assert!(status.is_ok(), status as i32);
                let _ = self.comp.active.queued.send_message(
                    &buf,
                    QUEUE_PRIORITY,
                    QueueFullPolicy::Assert,
                );
            }
            _ => {
                self.comp
                    .cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Dispatch.
// ---------------------------------------------------------------------------

impl ComponentDispatch for EventManager {
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
            MSG_TYPE_LOQ_QUEUE => {
                let mut id: FwEventIdType = 0;
                let mut time_tag = Time::default();
                let mut severity = LogSeverity::Fatal;
                let mut args = LogBuffer::new();
                if !buf.deserialize_u32_be(&mut id).is_ok()
                    || !buf.deserialize(&mut time_tag, Endianness::Big).is_ok()
                    || !buf.deserialize(&mut severity, Endianness::Big).is_ok()
                    || !buf.deserialize_buffer(&mut args, Endianness::Big).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                self.loq_queue_handler(id, time_tag, severity, &args);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_RUN => {
                let mut context = 0u32;
                if !buf.deserialize_u32_be(&mut context).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.run_handler(port_num, context);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_PING_IN => {
                let mut key = 0u32;
                if !buf.deserialize_u32_be(&mut key).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.ping_in_handler(port_num, key);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_CMD => {
                let mut op_code: FwOpcodeType = 0;
                let mut cmd_seq = 0u32;
                let mut args = CmdArgBuffer::new();
                if !buf.deserialize_u32_be(&mut op_code).is_ok()
                    || !buf.deserialize_u32_be(&mut cmd_seq).is_ok()
                    || !buf.deserialize_buffer(&mut args, Endianness::Big).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                match op_code.wrapping_sub(self.id_base()) {
                    EventManager::OPCODE_SET_ID_FILTER => {
                        self.set_id_filter_cmd_handler(op_code, cmd_seq, &mut args)
                    }
                    EventManager::OPCODE_DUMP_FILTER_STATE => {
                        self.dump_filter_state_cmd_handler(op_code, cmd_seq, &mut args)
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

impl ActiveComponent for EventManager {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogTextPort, TimePort, TlmPort};
    use fprime_fw::{TextLogString, TimeBase, TlmBuffer};

    const ID_BASE: u32 = 0x200;

    #[derive(Default)]
    struct GroundStub {
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        text_events: Mutex<Vec<(FwEventIdType, String)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        packets: Mutex<Vec<(Vec<u8>, u32)>>,
        fatals: Mutex<Vec<FwEventIdType>>,
        ping_keys: Mutex<Vec<u32>>,
    }

    impl CmdRegPort for GroundStub {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }
    impl CmdResponsePort for GroundStub {
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
    impl LogPort for GroundStub {
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
    impl LogTextPort for GroundStub {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            text: &mut TextLogString,
        ) {
            self.text_events
                .lock()
                .unwrap()
                .push((id, text.as_str().unwrap_or_default().to_string()));
        }
    }
    impl TlmPort for GroundStub {
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
    impl ComPort for GroundStub {
        fn invoke(&self, _port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
            self.packets
                .lock()
                .unwrap()
                .push((data.as_slice().to_vec(), context));
        }
    }
    impl FatalEventPort for GroundStub {
        fn invoke(&self, _port_num: FwIndexType, id: FwEventIdType) {
            self.fatals.lock().unwrap().push(id);
        }
    }
    impl PingPort for GroundStub {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.ping_keys.lock().unwrap().push(key);
        }
    }

    struct TimeStub;
    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 100, 42);
        }
    }

    fn build() -> (Arc<EventManager>, Arc<GroundStub>) {
        let ground = Arc::new(GroundStub::default());
        let comp = EventManager::new("eventManager");
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.evt.time_out.connect(Arc::new(TimeStub), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.pkt_send.connect(ground.clone(), 0);
        comp.fatal_announce.connect(ground.clone(), 0);
        comp.ping_out.connect(ground.clone(), 0);
        comp.init(16);
        (comp, ground)
    }

    fn send_event(
        comp: &Arc<EventManager>,
        id: FwEventIdType,
        severity: LogSeverity,
        arg_bytes: &[u8],
    ) {
        let mut args = LogBuffer::new();
        assert!(args.set_buff(arg_bytes).is_ok());
        let mut time = Time::new(TimeBase::TbWorkstationTime, 3, 500, 600);
        let p = comp.log_recv_in(0);
        p.target
            .invoke(p.port_num, id, &mut time, severity, &mut args);
    }

    fn drain(comp: &Arc<EventManager>) {
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
    }

    fn send_cmd(comp: &Arc<EventManager>, opcode: FwOpcodeType, seq: u32, arg_bytes: &[u8]) {
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(arg_bytes).is_ok());
        let p = comp.cmd_in(0);
        p.target.invoke(p.port_num, opcode, seq, &mut args);
        drain(comp);
    }

    // -- happy path: byte-exact log packet ----------------------------------

    #[test]
    fn event_produces_byte_exact_log_packet() {
        let (comp, ground) = build();
        send_event(&comp, 0x1234, LogSeverity::ActivityHi, &[0xAB, 0xCD]);
        drain(&comp);
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 1);
        assert_eq!(
            packets[0].0,
            vec![
                0x00, 0x02, // FW_PACKET_LOG descriptor
                0x00, 0x00, 0x12, 0x34, // event id
                0x00, 0x02, // time base = TB_WORKSTATION_TIME
                0x03, // time context
                0x00, 0x00, 0x01, 0xF4, // seconds = 500
                0x00, 0x00, 0x02, 0x58, // useconds = 600
                0xAB, 0xCD, // raw args, OMIT_LENGTH
            ]
        );
        assert_eq!(packets[0].1, 0); // PktSend context 0
    }

    // -- loqQueue envelope byte-exactness ------------------------------------

    #[test]
    fn loq_queue_envelope_bytes_are_byte_exact() {
        let (comp, _ground) = build();
        send_event(&comp, 0x0102_0304, LogSeverity::WarningHi, &[0x55]);
        let mut dest = [0u8; QUEUE_MSG_SIZE];
        let mut size: FwSizeType = 0;
        let mut priority: FwQueuePriorityType = 0;
        let status = comp.active.queued.queue().receive(
            &mut dest,
            fprime_os::queue::BlockingType::NonBlocking,
            &mut size,
            &mut priority,
        );
        assert_eq!(status, fprime_os::queue::Status::OpOk);
        assert_eq!(
            &dest[..size as usize],
            &[
                0x00, 0x00, 0x00, 0x01, // msg_type = MSG_TYPE_LOQ_QUEUE
                0x00, 0x00, // port_num = 0 (internal port)
                0x01, 0x02, 0x03, 0x04, // event id
                0x00, 0x02, 0x03, // time base, context
                0x00, 0x00, 0x01, 0xF4, // seconds
                0x00, 0x00, 0x02, 0x58, // useconds
                0x02, // severity = WARNING_HI
                0x00, 0x01, // LogBuffer length prefix
                0x55, // arg byte
            ]
        );
    }

    // -- severity filtering ---------------------------------------------------

    #[test]
    fn diagnostic_filtered_by_default_others_pass() {
        let (comp, ground) = build();
        send_event(&comp, 1, LogSeverity::Diagnostic, &[]);
        send_event(&comp, 2, LogSeverity::WarningHi, &[]);
        send_event(&comp, 3, LogSeverity::ActivityLo, &[]);
        drain(&comp);
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 2);
        // ids 2 and 3 made it through.
        assert_eq!(&packets[0].0[2..6], &[0, 0, 0, 2]);
        assert_eq!(&packets[1].0[2..6], &[0, 0, 0, 3]);
    }

    #[test]
    fn set_event_filter_is_sync_and_enabled_sense_is_reversed() {
        let (comp, ground) = build();
        // Disable WARNING_LO: filterLevel=1, filterEnabled=DISABLED=1.
        // No dispatch — the command must execute synchronously.
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(&[1, 1]).is_ok());
        let p = comp.cmd_in(0);
        p.target.invoke(p.port_num, ID_BASE, 7, &mut args);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(ID_BASE, 7, CmdResponse::Ok)]
        );
        send_event(&comp, 10, LogSeverity::WarningLo, &[]);
        drain(&comp);
        assert!(ground.packets.lock().unwrap().is_empty());

        // Enable DIAGNOSTIC: filterLevel=5, filterEnabled=ENABLED=0.
        send_cmd(&comp, ID_BASE, 8, &[5, 0]);
        send_event(&comp, 11, LogSeverity::Diagnostic, &[]);
        drain(&comp);
        assert_eq!(ground.packets.lock().unwrap().len(), 1);
    }

    #[test]
    fn set_event_filter_error_paths() {
        let (comp, ground) = build();
        send_cmd(&comp, ID_BASE, 1, &[6, 0]); // invalid FilterSeverity
        send_cmd(&comp, ID_BASE, 2, &[0, 2]); // invalid Enabled
        send_cmd(&comp, ID_BASE, 3, &[0]); // short args
        send_cmd(&comp, ID_BASE, 4, &[0, 0, 9]); // residual byte
        send_cmd(&comp, ID_BASE + 1, 5, &[]); // opcode 1 gap -> invalid
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (ID_BASE, 1, CmdResponse::ValidationError),
                (ID_BASE, 2, CmdResponse::ValidationError),
                (ID_BASE, 3, CmdResponse::FormatError),
                (ID_BASE, 4, CmdResponse::FormatError),
                (ID_BASE + 1, 5, CmdResponse::InvalidOpcode),
            ]
        );
    }

    // -- FATAL bypass + announce ----------------------------------------------

    #[test]
    fn fatal_bypasses_all_filters_and_announces_on_caller_thread() {
        let (comp, ground) = build();
        // Disable every severity and filter the FATAL's id too.
        for level in 0..6u8 {
            send_cmd(&comp, ID_BASE, level as u32, &[level, 1]);
        }
        send_cmd(
            &comp,
            ID_BASE + EventManager::OPCODE_SET_ID_FILTER,
            10,
            &[0, 0, 0, 42, 0], // ID 42, ENABLED (add)
        );
        // Announce happens before any dispatch (caller thread, post-enqueue).
        send_event(&comp, 42, LogSeverity::Fatal, &[]);
        assert_eq!(*ground.fatals.lock().unwrap(), vec![42]);
        drain(&comp);
        // The packet went out despite severity + id filters.
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 1);
        assert_eq!(&packets[0].0[2..6], &[0, 0, 0, 42]);

        // A non-FATAL with the same filtered id is dropped.
        drop(packets);
        send_event(&comp, 42, LogSeverity::WarningHi, &[]);
        drain(&comp);
        assert_eq!(ground.packets.lock().unwrap().len(), 1);
    }

    // -- ID filter commands ----------------------------------------------------

    #[test]
    fn id_filter_add_remove_and_errors() {
        let (comp, ground) = build();
        let opcode = ID_BASE + EventManager::OPCODE_SET_ID_FILTER;
        // Add 7 -> OK + ID_FILTER_ENABLED.
        send_cmd(&comp, opcode, 1, &[0, 0, 0, 7, 0]);
        // Event with id 7 dropped, id 8 passes.
        send_event(&comp, 7, LogSeverity::WarningHi, &[]);
        send_event(&comp, 8, LogSeverity::WarningHi, &[]);
        drain(&comp);
        assert_eq!(ground.packets.lock().unwrap().len(), 1);
        // Remove 7 -> OK + ID_FILTER_REMOVED; id 7 passes again.
        send_cmd(&comp, opcode, 2, &[0, 0, 0, 7, 1]);
        send_event(&comp, 7, LogSeverity::WarningHi, &[]);
        drain(&comp);
        assert_eq!(ground.packets.lock().unwrap().len(), 2);
        // Remove missing -> ID_FILTER_NOT_FOUND + EXECUTION_ERROR.
        send_cmd(&comp, opcode, 3, &[0, 0, 0, 99, 1]);
        assert_eq!(
            ground.responses.lock().unwrap().last().unwrap(),
            &(opcode, 3, CmdResponse::ExecutionError)
        );
        let events = ground.events.lock().unwrap();
        assert_eq!(
            events[0].0,
            ID_BASE + EventManager::EVENTID_ID_FILTER_ENABLED
        );
        assert_eq!(events[0].1, LogSeverity::ActivityHi);
        assert_eq!(events[0].2, vec![0, 0, 0, 7]);
        assert_eq!(
            events[1].0,
            ID_BASE + EventManager::EVENTID_ID_FILTER_REMOVED
        );
        assert_eq!(
            events[2].0,
            ID_BASE + EventManager::EVENTID_ID_FILTER_NOT_FOUND
        );
        assert_eq!(events[2].1, LogSeverity::WarningLo);
        assert_eq!(events[2].2, vec![0, 0, 0, 99]);
    }

    #[test]
    fn id_filter_full_list_reports_and_readd_is_ok() {
        let (comp, ground) = build();
        let opcode = ID_BASE + EventManager::OPCODE_SET_ID_FILTER;
        for i in 0..ID_FILTER_SIZE as u32 {
            send_cmd(
                &comp,
                opcode,
                i,
                &i.to_be_bytes()
                    .iter()
                    .chain(&[0u8])
                    .copied()
                    .collect::<Vec<u8>>(),
            );
        }
        // 26th new id -> full.
        send_cmd(&comp, opcode, 100, &[0, 0, 1, 0, 0]);
        assert_eq!(
            ground.responses.lock().unwrap().last().unwrap(),
            &(opcode, 100, CmdResponse::ExecutionError)
        );
        assert_eq!(
            ground.events.lock().unwrap().last().unwrap().0,
            ID_BASE + EventManager::EVENTID_ID_FILTER_LIST_FULL
        );
        // Re-adding an existing id succeeds (ArraySet parity).
        send_cmd(&comp, opcode, 101, &[0, 0, 0, 3, 0]);
        assert_eq!(
            ground.responses.lock().unwrap().last().unwrap(),
            &(opcode, 101, CmdResponse::Ok)
        );
    }

    // -- DUMP_FILTER_STATE -----------------------------------------------------

    #[test]
    fn dump_filter_state_reports_severities_and_ids() {
        let (comp, ground) = build();
        let set_id = ID_BASE + EventManager::OPCODE_SET_ID_FILTER;
        send_cmd(&comp, set_id, 1, &[0, 0, 0, 9, 0]);
        ground.events.lock().unwrap().clear();
        ground.responses.lock().unwrap().clear();

        send_cmd(
            &comp,
            ID_BASE + EventManager::OPCODE_DUMP_FILTER_STATE,
            2,
            &[],
        );
        let events = ground.events.lock().unwrap();
        // 6 severity states + 1 filtered id.
        assert_eq!(events.len(), 7);
        for (i, event) in events[..6].iter().enumerate() {
            assert_eq!(
                event.0,
                ID_BASE + EventManager::EVENTID_SEVERITY_FILTER_STATE
            );
            assert_eq!(event.1, LogSeverity::ActivityLo);
            // args = [severity u8][enabled bool byte]; DIAGNOSTIC (5)
            // defaults to disabled.
            let expected_enabled = if i == 5 { 0x00 } else { 0xFF };
            assert_eq!(event.2, vec![i as u8, expected_enabled]);
        }
        assert_eq!(
            events[6].0,
            ID_BASE + EventManager::EVENTID_ID_FILTER_ENABLED
        );
        assert_eq!(events[6].2, vec![0, 0, 0, 9]);
        drop(events);
        assert_eq!(
            ground.responses.lock().unwrap().last().unwrap(),
            &(
                ID_BASE + EventManager::OPCODE_DUMP_FILTER_STATE,
                2,
                CmdResponse::Ok
            )
        );
    }

    // -- oversized event args --------------------------------------------------

    #[test]
    fn oversized_log_packet_is_dropped_not_asserted() {
        let (comp, ground) = build();
        // 506-byte LogBuffer -> LogPacket = 2+4+11+506 = 523 > 512.
        let big = [0u8; 506];
        send_event(&comp, 5, LogSeverity::WarningHi, &big);
        drain(&comp);
        assert!(ground.packets.lock().unwrap().is_empty());
    }

    // -- dropped-event telemetry ----------------------------------------------

    #[test]
    fn events_dropped_telemetry_counts_queue_overflow_on_change() {
        let ground = Arc::new(GroundStub::default());
        let comp = EventManager::new("tinyEventManager");
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.evt.time_out.connect(Arc::new(TimeStub), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.pkt_send.connect(ground.clone(), 0);
        comp.init(2);

        // First run: writes 0 (first write always emitted).
        let run = comp.run_in(0);
        run.target.invoke(run.port_num, 0);
        drain(&comp);
        // Second run with no change: suppressed.
        run.target.invoke(run.port_num, 0);
        drain(&comp);
        assert_eq!(
            *ground.tlm.lock().unwrap(),
            vec![(
                ID_BASE + EventManager::CHANID_EVENTS_DROPPED,
                vec![0, 0, 0, 0, 0, 0, 0, 0]
            )]
        );

        // Fill the depth-2 queue; the third event drops silently.
        for i in 0..3 {
            send_event(&comp, i, LogSeverity::WarningHi, &[]);
        }
        run.target.invoke(run.port_num, 0); // this run msg ALSO drops (queue full)
        drain(&comp);
        run.target.invoke(run.port_num, 0);
        drain(&comp);
        let tlm = ground.tlm.lock().unwrap();
        // Dropped count = 2 (one event + one run tick).
        assert_eq!(
            tlm.last().unwrap(),
            &(
                ID_BASE + EventManager::CHANID_EVENTS_DROPPED,
                vec![0, 0, 0, 0, 0, 0, 0, 2]
            )
        );
    }

    // -- ping ------------------------------------------------------------------

    #[test]
    fn ping_in_returns_key_on_ping_out() {
        let (comp, ground) = build();
        let ping = comp.ping_in(0);
        ping.target.invoke(ping.port_num, 0xBEEF);
        drain(&comp);
        assert_eq!(*ground.ping_keys.lock().unwrap(), vec![0xBEEF]);
    }

    // -- registration + active lifecycle ---------------------------------------

    #[test]
    fn reg_commands_registers_the_three_opcodes() {
        let (comp, ground) = build();
        comp.reg_commands();
        assert_eq!(
            *ground.regs.lock().unwrap(),
            vec![ID_BASE, ID_BASE + 2, ID_BASE + 3]
        );
    }

    #[test]
    fn active_lifecycle_end_to_end() {
        let (comp, ground) = build();
        comp.active.start(
            &comp,
            100,
            fprime_os::task::TASK_DEFAULT,
            fprime_os::task::TASK_DEFAULT,
        );
        send_event(&comp, 77, LogSeverity::ActivityHi, &[1]);
        comp.active.exit();
        assert_eq!(comp.active.join(), fprime_os::task::Status::OpOk);
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 1);
        assert_eq!(&packets[0].0[2..6], &[0, 0, 0, 77]);
    }
}
