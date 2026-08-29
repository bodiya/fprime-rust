//! # CmdDispatcher — port of `Svc::CommandDispatcher` (active)
//!
//! C++ sources: `Svc/CmdDispatcher/CommandDispatcherImpl.{cpp,hpp}`,
//! `Svc/CmdDispatcher/CmdDispatcher.fpp`,
//! `default/config/CommandDispatcherImplCfg.hpp`.
//! Analysis: `docs/cpp-analysis/svc-core.md` (CmdDispatcher section + gotchas).
//!
//! Receives serialized command packets (`Fw.Com`) from sequencers/uplink,
//! decodes the opcode, looks it up in the registration table populated via
//! the guarded `compCmdReg` ports, forwards the argument buffer to the
//! owning component with an internal sequence number, tracks in-flight
//! commands, and forwards completion status back to the originating caller
//! port — with the caller's **context** in the `cmdSeq` argument position
//! (C++ parity, see `seqCmdStatus` note in the analysis).
//!
//! Ported quirks (each covered by a unit test):
//! - `m_seq` increments after every `seqCmdBuff` message **including** the
//!   invalid-opcode path, but NOT on the malformed-packet or
//!   tracker-full early returns.
//! - A registered opcode whose `compCmdSend` port is unconnected is
//!   reported as `InvalidCommand`/`INVALID_OPCODE`.
//! - A command is tracked only when the caller's `seqCmdStatus` port is
//!   connected.
//! - Re-registration to the same port is a DIAGNOSTIC event; to a
//!   different port it is an `FW_ASSERT` (panic).
//! - `seqCmdBuff` uses the `hook` queue-full policy: the overflow hook
//!   parses the opcode (config permitting), emits the throttled (5)
//!   `CommandDroppedQueueOverflow` event, and counts the drop.

use fprime_comp::msg;
use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, CmdRegPort, CmdResponsePort, ComPort,
    ComponentDispatch, EventGlue, EventThrottle, MsgDispatchStatus, PingPort, PortRef,
    QueueFullPolicy, SchedPort, TlmGlue,
};
use fprime_config::cmd_dispatcher::{
    DISPATCH_TABLE_SIZE, INCLUDE_COMMAND_OPCODES_IN_EVENTS, SEQUENCER_TABLE_SIZE,
};
use fprime_config::{
    CMD_DISPATCHER_COMMAND_PORTS, CMD_DISPATCHER_SEQUENCE_PORTS, FwChanIdType, FwEnumStoreType,
    FwEventIdType, FwIndexType, FwOpcodeType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::{
    CmdArgBuffer, CmdPacket, CmdResponse, CmdStringArg, ComBuffer, DeserialStatus, Deserialize,
    Endianness, LinearBuffer, LogSeverity, SerBuf, SerBufAny, Serialize, fw_assert,
};
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Dictionary constants (from CmdDispatcher.fpp).
// ---------------------------------------------------------------------------

/// Queue message type for the async `compCmdStat` input.
pub const MSG_TYPE_COMP_CMD_STAT: FwEnumStoreType = 1;
/// Queue message type for the async `seqCmdBuff` input (hook policy).
pub const MSG_TYPE_SEQ_CMD_BUFF: FwEnumStoreType = 2;
/// Queue message type for the async `pingIn` input.
pub const MSG_TYPE_PING_IN: FwEnumStoreType = 3;
/// Queue message type for the async `run` (Sched) input.
pub const MSG_TYPE_RUN: FwEnumStoreType = 4;
/// Queue message type for the async command-dispatch (`CmdDisp`) input.
pub const MSG_TYPE_CMD: FwEnumStoreType = 5;

/// Queue message size: max over async invocations. The largest is
/// `seqCmdBuff`: 6 (envelope) + 2 + 512 (length-prefixed `ComBuffer`) +
/// 4 (context u32) = 524. (The command envelope is 6 + 4 + 4 + 2 + 506 =
/// 522; all others are ≤ 15.)
pub const QUEUE_MSG_SIZE: usize = 524;

/// All async inputs share one queue priority (no FPP `priority`
/// qualifiers on this component) so dispatch is pure FIFO.
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

impl CmdDispatcher {
    /// `CMD_NO_OP` opcode (component-relative).
    pub const OPCODE_CMD_NO_OP: FwOpcodeType = 0;
    /// `CMD_NO_OP_STRING` opcode (string size 40).
    pub const OPCODE_CMD_NO_OP_STRING: FwOpcodeType = 1;
    /// `CMD_TEST_CMD_1` opcode (I32, F32, U8).
    pub const OPCODE_CMD_TEST_CMD_1: FwOpcodeType = 2;
    /// `CMD_CLEAR_TRACKING` opcode.
    pub const OPCODE_CMD_CLEAR_TRACKING: FwOpcodeType = 3;

    /// `OpCodeRegistered` (DIAGNOSTIC).
    pub const EVENTID_OP_CODE_REGISTERED: FwEventIdType = 0;
    /// `OpCodeDispatched` (COMMAND).
    pub const EVENTID_OP_CODE_DISPATCHED: FwEventIdType = 1;
    /// `OpCodeCompleted` (COMMAND).
    pub const EVENTID_OP_CODE_COMPLETED: FwEventIdType = 2;
    /// `OpCodeError` (COMMAND).
    pub const EVENTID_OP_CODE_ERROR: FwEventIdType = 3;
    /// `MalformedCommand` (WARNING_HI).
    pub const EVENTID_MALFORMED_COMMAND: FwEventIdType = 4;
    /// `InvalidCommand` (WARNING_HI).
    pub const EVENTID_INVALID_COMMAND: FwEventIdType = 5;
    /// `TooManyCommands` (WARNING_HI).
    pub const EVENTID_TOO_MANY_COMMANDS: FwEventIdType = 6;
    /// `NoOpReceived` (ACTIVITY_HI).
    pub const EVENTID_NO_OP_RECEIVED: FwEventIdType = 7;
    /// `NoOpStringReceived` (ACTIVITY_HI).
    pub const EVENTID_NO_OP_STRING_RECEIVED: FwEventIdType = 8;
    /// `TestCmd1Args` (ACTIVITY_HI).
    pub const EVENTID_TEST_CMD_1_ARGS: FwEventIdType = 9;
    /// `OpCodeReregistered` (DIAGNOSTIC).
    pub const EVENTID_OP_CODE_REREGISTERED: FwEventIdType = 10;
    /// `CommandDroppedQueueOverflow` (WARNING_HI, throttle 5).
    pub const EVENTID_COMMAND_DROPPED_QUEUE_OVERFLOW: FwEventIdType = 11;
    /// FPP `throttle 5` on `CommandDroppedQueueOverflow`.
    pub const COMMAND_DROPPED_THROTTLE: u32 = 5;

    /// `CommandsDispatched` telemetry channel (U32, on change).
    pub const CHANID_COMMANDS_DISPATCHED: FwChanIdType = 0;
    /// `CommandErrors` telemetry channel (U32, on change).
    pub const CHANID_COMMAND_ERRORS: FwChanIdType = 1;
    /// `CommandsDropped` telemetry channel (U32, on change).
    pub const CHANID_COMMANDS_DROPPED: FwChanIdType = 2;
}

/// `CmdDispatcherCfg::getEventOpcode`: opcode as-is when config includes
/// opcodes in events, else the maximum `FwOpcodeType` value.
const fn get_event_opcode(opcode: FwOpcodeType) -> FwOpcodeType {
    if INCLUDE_COMMAND_OPCODES_IN_EVENTS {
        opcode
    } else {
        FwOpcodeType::MAX
    }
}

// ---------------------------------------------------------------------------
// Sequence tracker: Fw::ArrayMap<U32, SequenceTrackerEntry, 25>.
// ---------------------------------------------------------------------------

/// One in-flight command (C++ `SequenceTrackerEntry`).
#[derive(Debug, Clone, Copy)]
struct SequenceTrackerEntry {
    op_code: FwOpcodeType,
    context: u32,
    caller_port: FwIndexType,
}

/// Fixed-capacity array map keyed by the dispatcher sequence number.
/// C++ `Fw::ArrayMap` semantics: insert overwrites an existing key,
/// otherwise takes the first free slot, and fails only when full.
struct SequenceTracker {
    entries: [Option<(u32, SequenceTrackerEntry)>; SEQUENCER_TABLE_SIZE],
}

impl SequenceTracker {
    const fn new() -> Self {
        Self {
            entries: [None; SEQUENCER_TABLE_SIZE],
        }
    }

    /// True on success (existing key overwritten or free slot used).
    fn insert(&mut self, key: u32, entry: SequenceTrackerEntry) -> bool {
        // Existing key: overwrite (ArrayMap parity).
        for slot in self.entries.iter_mut().flatten() {
            if slot.0 == key {
                slot.1 = entry;
                return true;
            }
        }
        for slot in self.entries.iter_mut() {
            if slot.is_none() {
                *slot = Some((key, entry));
                return true;
            }
        }
        false
    }

    fn remove(&mut self, key: u32) -> Option<SequenceTrackerEntry> {
        for slot in self.entries.iter_mut() {
            if let Some((k, entry)) = slot {
                if *k == key {
                    let entry = *entry;
                    *slot = None;
                    return Some(entry);
                }
            }
        }
        None
    }

    fn clear(&mut self) {
        self.entries = [None; SEQUENCER_TABLE_SIZE];
    }
}

// ---------------------------------------------------------------------------
// Component state.
// ---------------------------------------------------------------------------

/// Mutable dispatcher state behind the component mutex (the C++ guarded-port
/// mutex generalized).
struct CmdDispatcherState {
    /// Dispatch table: opcode -> `compCmdSend` port index. C++ uses
    /// `Fw::RedBlackTreeMap<FwOpcodeType, FwIndexType, 150>`; a `BTreeMap`
    /// capped at [`DISPATCH_TABLE_SIZE`] reproduces it (allocation happens
    /// only during init-time registration).
    entry_table: BTreeMap<FwOpcodeType, FwIndexType>,
    sequence_tracker: SequenceTracker,
    /// Internal sequence number (`m_seq`), starts at 0.
    seq: u32,
    num_cmds_dispatched: u32,
    num_cmd_errors: u32,
    num_cmds_dropped: u32,
    // `update on change` caches for the three telemetry channels (the C++
    // autocoded tlmWrite_* keeps these in the component base).
    last_dispatched: Option<u32>,
    last_errors: Option<u32>,
    last_dropped: Option<u32>,
}

impl CmdDispatcherState {
    fn new() -> Self {
        Self {
            entry_table: BTreeMap::new(),
            sequence_tracker: SequenceTracker::new(),
            seq: 0,
            num_cmds_dispatched: 0,
            num_cmd_errors: 0,
            num_cmds_dropped: 0,
            last_dispatched: None,
            last_errors: None,
            last_dropped: None,
        }
    }
}

// ---------------------------------------------------------------------------
// The component.
// ---------------------------------------------------------------------------

/// `Svc::CommandDispatcher` — active command dispatcher.
pub struct CmdDispatcher {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Own command registration/response glue (`CmdReg`/`CmdStatus`).
    pub cmd: CmdGlue,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// `compCmdSend`: \[30\] `Fw.Cmd` out.
    pub comp_cmd_send: [fprime_comp::OutputPort<dyn CmdPort>; CMD_DISPATCHER_COMMAND_PORTS],
    /// `seqCmdStatus`: \[5\] `Fw.CmdResponse` out.
    pub seq_cmd_status:
        [fprime_comp::OutputPort<dyn CmdResponsePort>; CMD_DISPATCHER_SEQUENCE_PORTS],
    /// `pingOut`: `Svc.Ping` out.
    pub ping_out: fprime_comp::OutputPort<dyn PingPort>,
    /// Throttle (5) for `CommandDroppedQueueOverflow`.
    dropped_throttle: EventThrottle,
    state: Mutex<CmdDispatcherState>,
}

impl CmdDispatcher {
    /// Construct (topology phase 1). Follow with `set_id_base`, wiring,
    /// [`Self::init`], [`Self::reg_commands`], and `active.start`.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            comp_cmd_send: std::array::from_fn(|_| fprime_comp::OutputPort::new()),
            seq_cmd_status: std::array::from_fn(|_| fprime_comp::OutputPort::new()),
            ping_out: fprime_comp::OutputPort::new(),
            dropped_throttle: EventThrottle::new(Self::COMMAND_DROPPED_THROTTLE),
            state: Mutex::new(CmdDispatcherState::new()),
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

    /// C++ `regCommands()`: register this component's own opcodes.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_CMD_NO_OP,
                Self::OPCODE_CMD_NO_OP_STRING,
                Self::OPCODE_CMD_TEST_CMD_1,
                Self::OPCODE_CMD_CLEAR_TRACKING,
            ],
        );
    }

    // -- Input-port factories (topology wiring surface) ---------------------

    /// `compCmdReg` — GUARDED `Fw.CmdReg` input, ports 0..30. Runs on the
    /// caller's thread under the component mutex.
    pub fn comp_cmd_reg_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdRegPort> {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < CMD_DISPATCHER_COMMAND_PORTS,
            port_num as i32
        );
        PortRef::new(self.clone(), port_num)
    }

    /// `compCmdStat` — ASYNC `Fw.CmdResponse` input.
    pub fn comp_cmd_stat_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn CmdResponsePort> {
        PortRef::new(
            Arc::new(CompCmdStatAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `seqCmdBuff` — ASYNC `Fw.Com` input with the `hook` overflow policy,
    /// ports 0..5.
    pub fn seq_cmd_buff_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComPort> {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < CMD_DISPATCHER_SEQUENCE_PORTS,
            port_num as i32
        );
        PortRef::new(Arc::new(SeqCmdBuffAdapter { comp: self.clone() }), port_num)
    }

    /// `pingIn` — ASYNC `Svc.Ping` input.
    pub fn ping_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn PingPort> {
        PortRef::new(Arc::new(PingInAdapter { comp: self.clone() }), port_num)
    }

    /// `run` — ASYNC `Svc.Sched` input (telemetry emission).
    pub fn run_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(Arc::new(RunInAdapter { comp: self.clone() }), port_num)
    }

    /// `CmdDisp` — ASYNC command-dispatch input for this component's own
    /// commands.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }

    // -- Handlers -----------------------------------------------------------

    /// `compCmdReg_handler` — GUARDED (caller thread). Holds the component
    /// mutex across the event emission (C++ guarded ports hold the mutex
    /// for the whole handler).
    fn comp_cmd_reg_handler(&self, port_num: FwIndexType, op_code: FwOpcodeType) {
        let state = &mut *self.state.lock().unwrap();
        if let Some(&existing_port) = state.entry_table.get(&op_code) {
            // Re-registration must target the same port (C++ FW_ASSERT).
            fw_assert!(existing_port == port_num, op_code as i32);
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_OP_CODE_REREGISTERED,
                LogSeverity::Diagnostic,
                &format!(
                    "Opcode 0x{:X} is already registered to port {}",
                    get_event_opcode(op_code),
                    port_num
                ),
                |buf| {
                    let status = buf.serialize_u32_be(get_event_opcode(op_code));
                    if !status.is_ok() {
                        return status;
                    }
                    buf.serialize_i32_be(port_num as i32)
                },
            );
        } else {
            // Slot reported = table size before insert (C++ parity).
            let slot = state.entry_table.len() as i32;
            // C++ asserts on a full table (RedBlackTreeMap insert failure).
            fw_assert!(
                state.entry_table.len() < DISPATCH_TABLE_SIZE,
                op_code as i32
            );
            state.entry_table.insert(op_code, port_num);
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_OP_CODE_REGISTERED,
                LogSeverity::Diagnostic,
                &format!(
                    "Opcode 0x{:X} registered to port {} slot {}",
                    get_event_opcode(op_code),
                    port_num,
                    slot
                ),
                |buf| {
                    let status = buf.serialize_u32_be(get_event_opcode(op_code));
                    if !status.is_ok() {
                        return status;
                    }
                    let status = buf.serialize_i32_be(port_num as i32);
                    if !status.is_ok() {
                        return status;
                    }
                    buf.serialize_i32_be(slot)
                },
            );
        }
    }

    /// Reply on `seqCmdStatus[port]` if that port is connected.
    fn seq_status_reply(
        &self,
        caller_port: FwIndexType,
        op_code: FwOpcodeType,
        context: u32,
        response: CmdResponse,
    ) {
        if let Some(p) = self.seq_cmd_status[caller_port as usize].try_get() {
            // NOTE: the context value travels in the cmdSeq position — the
            // caller gets its context back, not the dispatcher's sequence
            // number (CommandDispatcherImpl.cpp parity).
            p.target.invoke(p.port_num, op_code, context, response);
        }
    }

    /// `seqCmdBuff_handler` — component thread.
    fn seq_cmd_buff_handler(&self, port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
        let mut cmd_pkt = CmdPacket::new();
        let stat = cmd_pkt.deserialize_from(data, Endianness::Big);
        if !stat.is_ok() {
            // C++ casts the SerializeStatus ordinal to Fw::DeserialStatus;
            // only the Deser* ordinals (3..=6) are reachable here.
            let ser_err =
                DeserialStatus::try_from(stat as u8).unwrap_or(DeserialStatus::FormatError);
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_MALFORMED_COMMAND,
                LogSeverity::WarningHi,
                &format!("Received malformed command packet. Status: {ser_err:?}"),
                |buf| ser_err.serialize_to(buf, Endianness::Big),
            );
            // Early return: m_seq is NOT incremented on this path.
            self.seq_status_reply(
                port_num,
                cmd_pkt.get_opcode(),
                context,
                CmdResponse::ValidationError,
            );
            return;
        }

        let op_code = cmd_pkt.get_opcode();
        let entry_port = self
            .state
            .lock()
            .unwrap()
            .entry_table
            .get(&op_code)
            .copied();

        let connected_target =
            entry_port.filter(|&p| self.comp_cmd_send[p as usize].is_connected());

        if let Some(entry_port) = connected_target {
            let seq = self.state.lock().unwrap().seq;
            // Track the command only if the caller's status port is
            // connected (C++ parity).
            if self.seq_cmd_status[port_num as usize].is_connected() {
                let inserted = self.state.lock().unwrap().sequence_tracker.insert(
                    seq,
                    SequenceTrackerEntry {
                        op_code,
                        context,
                        caller_port: port_num,
                    },
                );
                if !inserted {
                    self.evt.log_event(
                        self.id_base(),
                        Self::EVENTID_TOO_MANY_COMMANDS,
                        LogSeverity::WarningHi,
                        &format!(
                            "Too many outstanding commands. opcode=0x{:X}",
                            get_event_opcode(op_code)
                        ),
                        |buf| buf.serialize_u32_be(get_event_opcode(op_code)),
                    );
                    self.seq_status_reply(port_num, op_code, context, CmdResponse::ExecutionError);
                    // Early return: m_seq is NOT incremented on this path.
                    return;
                }
            }
            // Dispatch: forwards the packet's argument bytes with the
            // dispatcher's own sequence number.
            let mut args = cmd_pkt.get_arg_buffer().clone();
            let p = self.comp_cmd_send[entry_port as usize].get();
            p.target.invoke(p.port_num, op_code, seq, &mut args);
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_OP_CODE_DISPATCHED,
                LogSeverity::Command,
                &format!(
                    "Opcode 0x{:X} dispatched to port {}",
                    get_event_opcode(op_code),
                    entry_port
                ),
                |buf| {
                    let status = buf.serialize_u32_be(get_event_opcode(op_code));
                    if !status.is_ok() {
                        return status;
                    }
                    buf.serialize_i32_be(entry_port as i32)
                },
            );
            self.state.lock().unwrap().num_cmds_dispatched += 1;
        } else {
            // Unknown opcode OR unconnected target port (the checks are
            // ANDed in C++, so both fail the same way).
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_INVALID_COMMAND,
                LogSeverity::WarningHi,
                &format!("Invalid opcode 0x{:X} received", get_event_opcode(op_code)),
                |buf| buf.serialize_u32_be(get_event_opcode(op_code)),
            );
            self.state.lock().unwrap().num_cmd_errors += 1;
            self.seq_status_reply(port_num, op_code, context, CmdResponse::InvalidOpcode);
        }

        // m_seq increments even on the invalid-opcode path (C++ parity).
        let mut state = self.state.lock().unwrap();
        state.seq = state.seq.wrapping_add(1);
    }

    /// `compCmdStat_handler` — component thread.
    fn comp_cmd_stat_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        if response == CmdResponse::Ok {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_OP_CODE_COMPLETED,
                LogSeverity::Command,
                &format!("Opcode 0x{:X} completed", get_event_opcode(op_code)),
                |buf| buf.serialize_u32_be(get_event_opcode(op_code)),
            );
        } else {
            self.state.lock().unwrap().num_cmd_errors += 1;
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_OP_CODE_ERROR,
                LogSeverity::Command,
                &format!(
                    "Opcode 0x{:X} completed with error {:?}",
                    get_event_opcode(op_code),
                    response
                ),
                |buf| {
                    let status = buf.serialize_u32_be(get_event_opcode(op_code));
                    if !status.is_ok() {
                        return status;
                    }
                    response.serialize_to(buf, Endianness::Big)
                },
            );
        }
        // Look for the command source.
        let tracked = self.state.lock().unwrap().sequence_tracker.remove(cmd_seq);
        if let Some(tracked) = tracked {
            fw_assert!(op_code == tracked.op_code, op_code as i32);
            fw_assert!(
                (tracked.caller_port as usize) < CMD_DISPATCHER_SEQUENCE_PORTS,
                tracked.caller_port as i32
            );
            // Context in the cmdSeq slot (C++ parity).
            self.seq_status_reply(tracked.caller_port, op_code, tracked.context, response);
        }
    }

    /// `run_handler` — component thread; writes the three on-change
    /// telemetry channels in the C++ order (Dropped, Errors, Dispatched).
    fn run_handler(&self, _port_num: FwIndexType, _context: u32) {
        fn changed(last: &mut Option<u32>, value: u32) -> Option<u32> {
            if *last == Some(value) {
                None
            } else {
                *last = Some(value);
                Some(value)
            }
        }
        let (dropped, errors, dispatched) = {
            let state = &mut *self.state.lock().unwrap();
            (
                changed(&mut state.last_dropped, state.num_cmds_dropped),
                changed(&mut state.last_errors, state.num_cmd_errors),
                changed(&mut state.last_dispatched, state.num_cmds_dispatched),
            )
        };
        let id_base = self.id_base();
        if let Some(v) = dropped {
            self.tlm.tlm_write(
                id_base,
                Self::CHANID_COMMANDS_DROPPED,
                &v,
                self.evt.time_get(),
            );
        }
        if let Some(v) = errors {
            self.tlm.tlm_write(
                id_base,
                Self::CHANID_COMMAND_ERRORS,
                &v,
                self.evt.time_get(),
            );
        }
        if let Some(v) = dispatched {
            self.tlm.tlm_write(
                id_base,
                Self::CHANID_COMMANDS_DISPATCHED,
                &v,
                self.evt.time_get(),
            );
        }
    }

    /// `pingIn_handler` — component thread.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }

    /// `seqCmdBuff_overflowHook` — runs on the SENDER thread when the
    /// queue is full.
    fn seq_cmd_buff_overflow_hook(
        &self,
        _port_num: FwIndexType,
        data: &mut ComBuffer,
        context: u32,
    ) {
        // 0 = reserved opcode (C++ parity for an unparseable packet).
        let mut opcode: FwOpcodeType = 0;
        if INCLUDE_COMMAND_OPCODES_IN_EVENTS {
            let mut cmd_pkt = CmdPacket::new();
            let stat = cmd_pkt.deserialize_from(data, Endianness::Big);
            if stat.is_ok() {
                opcode = cmd_pkt.get_opcode();
            }
        }
        if self.dropped_throttle.ok_to_emit() {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_COMMAND_DROPPED_QUEUE_OVERFLOW,
                LogSeverity::WarningHi,
                &format!(
                    "Opcode 0x{:X} was dropped due to buffer overflow and not processed. Context {}",
                    get_event_opcode(opcode),
                    context
                ),
                |buf| {
                    let status = buf.serialize_u32_be(get_event_opcode(opcode));
                    if !status.is_ok() {
                        return status;
                    }
                    buf.serialize_u32_be(context)
                },
            );
        }
        self.state.lock().unwrap().num_cmds_dropped += 1;
    }

    // -- Own command handlers (component thread) ----------------------------

    fn cmd_no_op_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_NO_OP_RECEIVED,
            LogSeverity::ActivityHi,
            "Received a NO-OP command",
            |_buf| fprime_fw::SerializeStatus::Ok,
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn cmd_no_op_string_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut arg1 = CmdStringArg::new();
        if !args.deserialize(&mut arg1, Endianness::Big).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let text = String::from_utf8_lossy(arg1.as_bytes()).into_owned();
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_NO_OP_STRING_RECEIVED,
            LogSeverity::ActivityHi,
            &format!("Received a NO-OP string={text}"),
            // Event arg is `string size 40` — CmdStringArg's own u16-len
            // serialization is already capped at 40.
            |buf| arg1.serialize_to(buf, Endianness::Big),
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn cmd_test_cmd_1_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut arg1: i32 = 0;
        let mut arg2: f32 = 0.0;
        let mut arg3: u8 = 0;
        if !args.deserialize_i32_be(&mut arg1).is_ok()
            || !args.deserialize_f32_be(&mut arg2).is_ok()
            || !args.deserialize_u8_be(&mut arg3).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_TEST_CMD_1_ARGS,
            LogSeverity::ActivityHi,
            &format!("TEST_CMD_1 args: I32: {arg1}, F32: {arg2}, U8: {arg3}"),
            |buf| {
                let status = buf.serialize_i32_be(arg1);
                if !status.is_ok() {
                    return status;
                }
                let status = buf.serialize_f32_be(arg2);
                if !status.is_ok() {
                    return status;
                }
                buf.serialize_u8_be(arg3)
            },
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn cmd_clear_tracking_handler(
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
        self.state.lock().unwrap().sequence_tracker.clear();
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }
}

// ---------------------------------------------------------------------------
// Sync/guarded input port: compCmdReg implemented directly on the component.
// ---------------------------------------------------------------------------

impl CmdRegPort for CmdDispatcher {
    fn invoke(&self, port_num: FwIndexType, op_code: FwOpcodeType) {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < CMD_DISPATCHER_COMMAND_PORTS,
            port_num as i32
        );
        self.comp_cmd_reg_handler(port_num, op_code);
    }
}

// ---------------------------------------------------------------------------
// Async input adapters.
// ---------------------------------------------------------------------------

type MsgBuffer = LinearBuffer<QUEUE_MSG_SIZE>;

struct CompCmdStatAdapter {
    comp: Arc<CmdDispatcher>,
}

impl CmdResponsePort for CompCmdStatAdapter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_COMP_CMD_STAT, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(op_code);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(cmd_seq);
        fw_assert!(status.is_ok(), status as i32);
        let status = response.serialize_to(&mut buf, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, QUEUE_PRIORITY, QueueFullPolicy::Assert);
    }
}

struct SeqCmdBuffAdapter {
    comp: Arc<CmdDispatcher>,
}

impl ComPort for SeqCmdBuffAdapter {
    fn invoke(&self, port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_SEQ_CMD_BUFF, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(data, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(context);
        fw_assert!(status.is_ok(), status as i32);
        // `hook` policy: on Full the adapter calls the overflow hook with
        // the original args (C++ seqCmdBuff_overflowHook parity).
        let send_status =
            self.comp
                .active
                .queued
                .send_message(&buf, QUEUE_PRIORITY, QueueFullPolicy::Hook);
        if send_status == fprime_os::queue::Status::Full {
            self.comp
                .seq_cmd_buff_overflow_hook(port_num, data, context);
        }
    }
}

struct PingInAdapter {
    comp: Arc<CmdDispatcher>,
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

struct RunInAdapter {
    comp: Arc<CmdDispatcher>,
}

impl SchedPort for RunInAdapter {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_RUN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(context);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, QUEUE_PRIORITY, QueueFullPolicy::Assert);
    }
}

struct CmdInAdapter {
    comp: Arc<CmdDispatcher>,
}

impl CmdPort for CmdInAdapter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_CMD, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(op_code);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(cmd_seq);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(args, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, QUEUE_PRIORITY, QueueFullPolicy::Assert);
    }
}

// ---------------------------------------------------------------------------
// Dispatch (the hand-written doDispatch switch).
// ---------------------------------------------------------------------------

impl ComponentDispatch for CmdDispatcher {
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
            MSG_TYPE_COMP_CMD_STAT => {
                let mut op_code: FwOpcodeType = 0;
                let mut cmd_seq = 0u32;
                let mut response = CmdResponse::Ok;
                if !buf.deserialize_u32_be(&mut op_code).is_ok()
                    || !buf.deserialize_u32_be(&mut cmd_seq).is_ok()
                    || !buf.deserialize(&mut response, Endianness::Big).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                self.comp_cmd_stat_handler(port_num, op_code, cmd_seq, response);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_SEQ_CMD_BUFF => {
                fw_assert!(
                    port_num >= 0 && (port_num as usize) < CMD_DISPATCHER_SEQUENCE_PORTS,
                    port_num as i32
                );
                let mut data = ComBuffer::new();
                let mut context = 0u32;
                if !buf.deserialize_buffer(&mut data, Endianness::Big).is_ok()
                    || !buf.deserialize_u32_be(&mut context).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                self.seq_cmd_buff_handler(port_num, &mut data, context);
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
            MSG_TYPE_RUN => {
                let mut context = 0u32;
                if !buf.deserialize_u32_be(&mut context).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.run_handler(port_num, context);
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
                    Self::OPCODE_CMD_NO_OP => self.cmd_no_op_handler(op_code, cmd_seq, &mut args),
                    Self::OPCODE_CMD_NO_OP_STRING => {
                        self.cmd_no_op_string_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_CMD_TEST_CMD_1 => {
                        self.cmd_test_cmd_1_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_CMD_CLEAR_TRACKING => {
                        self.cmd_clear_tracking_handler(op_code, cmd_seq, &mut args)
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

impl ActiveComponent for CmdDispatcher {
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
    use fprime_comp::{LogPort, LogTextPort, TimePort, TlmPort};
    use fprime_fw::{LogBuffer, TextLogString, Time, TimeBase, TlmBuffer};

    const ID_BASE: u32 = 0x100;

    /// Ground stub: records events, telemetry, own-command responses and
    /// registrations.
    #[derive(Default)]
    struct GroundStub {
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        text_events: Mutex<Vec<(FwEventIdType, String)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
    }

    impl fprime_comp::CmdRegPort for GroundStub {
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

    struct TimeStub;
    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 100, 42);
        }
    }

    /// A recorded dispatch: (port_num, opcode, seq, arg bytes).
    type DispatchRecord = (FwIndexType, FwOpcodeType, u32, Vec<u8>);

    /// Command target stub: records dispatches.
    #[derive(Default)]
    struct TargetStub {
        cmds: Mutex<Vec<DispatchRecord>>,
    }
    impl CmdPort for TargetStub {
        fn invoke(
            &self,
            port_num: FwIndexType,
            op_code: FwOpcodeType,
            cmd_seq: u32,
            args: &mut CmdArgBuffer,
        ) {
            self.cmds
                .lock()
                .unwrap()
                .push((port_num, op_code, cmd_seq, args.as_slice().to_vec()));
        }
    }

    /// Sequencer stub: records forwarded responses.
    #[derive(Default)]
    struct SeqStub {
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
    }
    impl CmdResponsePort for SeqStub {
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

    #[derive(Default)]
    struct PingStub {
        keys: Mutex<Vec<u32>>,
    }
    impl PingPort for PingStub {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.keys.lock().unwrap().push(key);
        }
    }

    fn build() -> (Arc<CmdDispatcher>, Arc<GroundStub>) {
        let ground = Arc::new(GroundStub::default());
        let comp = CmdDispatcher::new("cmdDisp");
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.evt.time_out.connect(Arc::new(TimeStub), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.init(16);
        (comp, ground)
    }

    /// Build a serialized command packet: [u16 0][opcode u32][raw args].
    fn cmd_packet(opcode: FwOpcodeType, args: &[u8]) -> ComBuffer {
        let mut buf = ComBuffer::new();
        assert!(buf.serialize_u16_be(0).is_ok());
        assert!(buf.serialize_u32_be(opcode).is_ok());
        assert!(
            buf.serialize_bytes(args, fprime_fw::LengthMode::OmitLength, Endianness::Big)
                .is_ok()
        );
        buf
    }

    /// Push a command packet in through seqCmdBuff and dispatch it
    /// synchronously.
    fn send_seq_cmd(
        comp: &Arc<CmdDispatcher>,
        port: FwIndexType,
        packet: &mut ComBuffer,
        ctx: u32,
    ) {
        let p = comp.seq_cmd_buff_in(port);
        p.target.invoke(p.port_num, packet, ctx);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
    }

    fn register(comp: &Arc<CmdDispatcher>, port: FwIndexType, opcode: FwOpcodeType) {
        let reg = comp.comp_cmd_reg_in(port);
        reg.target.invoke(reg.port_num, opcode);
    }

    // -- registration -------------------------------------------------------

    #[test]
    fn registration_emits_diagnostic_with_insertion_slot() {
        let (comp, ground) = build();
        register(&comp, 3, 0x500);
        register(&comp, 7, 0x600);
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 2);
        // OpCodeRegistered(opcode u32, port i32, slot i32)
        assert_eq!(
            events[0].0,
            ID_BASE + CmdDispatcher::EVENTID_OP_CODE_REGISTERED
        );
        assert_eq!(events[0].1, LogSeverity::Diagnostic);
        assert_eq!(
            events[0].2,
            vec![0, 0, 0x05, 0, 0, 0, 0, 3, 0, 0, 0, 0] // op, port 3, slot 0
        );
        assert_eq!(
            events[1].2,
            vec![0, 0, 0x06, 0, 0, 0, 0, 7, 0, 0, 0, 1] // op, port 7, slot 1
        );
    }

    #[test]
    fn reregistration_same_port_is_diagnostic() {
        let (comp, ground) = build();
        register(&comp, 3, 0x500);
        register(&comp, 3, 0x500);
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 2);
        assert_eq!(
            events[1].0,
            ID_BASE + CmdDispatcher::EVENTID_OP_CODE_REREGISTERED
        );
        assert_eq!(events[1].1, LogSeverity::Diagnostic);
        assert_eq!(events[1].2, vec![0, 0, 0x05, 0, 0, 0, 0, 3]);
    }

    #[test]
    #[should_panic]
    fn reregistration_different_port_asserts() {
        let (comp, _ground) = build();
        register(&comp, 3, 0x500);
        register(&comp, 4, 0x500);
    }

    // -- envelope byte-exactness --------------------------------------------

    #[test]
    fn seq_cmd_buff_envelope_bytes_are_byte_exact() {
        let (comp, _ground) = build();
        let mut packet = cmd_packet(0x0102_0304, &[0xAA]);
        let p = comp.seq_cmd_buff_in(2);
        p.target.invoke(p.port_num, &mut packet, 0x0BAD_F00D);

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
                0x00, 0x00, 0x00, 0x02, // msg_type = MSG_TYPE_SEQ_CMD_BUFF
                0x00, 0x02, // port_num = 2
                0x00, 0x07, // ComBuffer length prefix (2 + 4 + 1)
                0x00, 0x00, // packet descriptor
                0x01, 0x02, 0x03, 0x04, // opcode
                0xAA, // raw arg byte
                0x0B, 0xAD, 0xF0, 0x0D, // context
            ]
        );
        assert_eq!(priority, QUEUE_PRIORITY);
    }

    // -- dispatch happy path -------------------------------------------------

    #[test]
    fn dispatch_and_completion_forwards_context_in_cmd_seq_position() {
        let (comp, ground) = build();
        let target = Arc::new(TargetStub::default());
        let seq_stub = Arc::new(SeqStub::default());
        comp.comp_cmd_send[4].connect(target.clone(), 9);
        comp.seq_cmd_status[1].connect(seq_stub.clone(), 0);
        register(&comp, 4, 0x500);

        let mut packet = cmd_packet(0x500, &[1, 2, 3]);
        send_seq_cmd(&comp, 1, &mut packet, 0xC0FFEE);

        // Dispatched with the dispatcher's own sequence number (0).
        assert_eq!(
            *target.cmds.lock().unwrap(),
            vec![(9, 0x500, 0, vec![1, 2, 3])]
        );
        // OpCodeDispatched(op, port) after registration event.
        {
            let events = ground.events.lock().unwrap();
            let last = events.last().unwrap();
            assert_eq!(last.0, ID_BASE + CmdDispatcher::EVENTID_OP_CODE_DISPATCHED);
            assert_eq!(last.1, LogSeverity::Command);
            assert_eq!(last.2, vec![0, 0, 0x05, 0, 0, 0, 0, 4]);
        }

        // Completion: OK response keyed by seq 0.
        let stat = comp.comp_cmd_stat_in(0);
        stat.target.invoke(stat.port_num, 0x500, 0, CmdResponse::Ok);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());

        // The caller gets its CONTEXT back in the cmdSeq slot, not seq 0.
        assert_eq!(
            *seq_stub.responses.lock().unwrap(),
            vec![(0x500, 0xC0FFEE, CmdResponse::Ok)]
        );
        let events = ground.events.lock().unwrap();
        let last = events.last().unwrap();
        assert_eq!(last.0, ID_BASE + CmdDispatcher::EVENTID_OP_CODE_COMPLETED);
        assert_eq!(last.2, vec![0, 0, 0x05, 0]);
    }

    #[test]
    fn error_completion_counts_error_and_forwards_response() {
        let (comp, ground) = build();
        let target = Arc::new(TargetStub::default());
        let seq_stub = Arc::new(SeqStub::default());
        comp.comp_cmd_send[0].connect(target.clone(), 0);
        comp.seq_cmd_status[0].connect(seq_stub.clone(), 0);
        register(&comp, 0, 0x500);

        let mut packet = cmd_packet(0x500, &[]);
        send_seq_cmd(&comp, 0, &mut packet, 7);
        let stat = comp.comp_cmd_stat_in(0);
        stat.target
            .invoke(stat.port_num, 0x500, 0, CmdResponse::ExecutionError);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());

        assert_eq!(
            *seq_stub.responses.lock().unwrap(),
            vec![(0x500, 7, CmdResponse::ExecutionError)]
        );
        // OpCodeError(op, response) COMMAND severity.
        let events = ground.events.lock().unwrap();
        let last = events.last().unwrap();
        assert_eq!(last.0, ID_BASE + CmdDispatcher::EVENTID_OP_CODE_ERROR);
        assert_eq!(last.1, LogSeverity::Command);
        assert_eq!(last.2, vec![0, 0, 0x05, 0, 4]); // ExecutionError = 4
        drop(events);

        // Errors telemetry reflects the error.
        let run = comp.run_in(0);
        run.target.invoke(run.port_num, 0);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
        let tlm = ground.tlm.lock().unwrap();
        assert!(tlm.contains(&(
            ID_BASE + CmdDispatcher::CHANID_COMMAND_ERRORS,
            vec![0, 0, 0, 1]
        )));
    }

    // -- gotcha: malformed packet --------------------------------------------

    #[test]
    fn malformed_packet_replies_validation_error_and_does_not_bump_seq() {
        let (comp, ground) = build();
        let target = Arc::new(TargetStub::default());
        let seq_stub = Arc::new(SeqStub::default());
        comp.comp_cmd_send[0].connect(target.clone(), 0);
        comp.seq_cmd_status[0].connect(seq_stub.clone(), 0);
        register(&comp, 0, 0x500);

        // Wrong descriptor (1 != FW_PACKET_COMMAND) -> DeserTypeMismatch.
        let mut bad = ComBuffer::new();
        assert!(bad.serialize_u16_be(1).is_ok());
        assert!(bad.serialize_u32_be(0x500).is_ok());
        send_seq_cmd(&comp, 0, &mut bad, 5);

        {
            let events = ground.events.lock().unwrap();
            let last = events.last().unwrap();
            assert_eq!(last.0, ID_BASE + CmdDispatcher::EVENTID_MALFORMED_COMMAND);
            assert_eq!(last.1, LogSeverity::WarningHi);
            assert_eq!(last.2, vec![6]); // DeserialStatus::TypeMismatch
        }
        assert_eq!(
            *seq_stub.responses.lock().unwrap(),
            vec![(0, 5, CmdResponse::ValidationError)] // opcode 0: parse never got there
        );

        // seq unchanged: the next dispatched command still carries seq 0.
        let mut good = cmd_packet(0x500, &[]);
        send_seq_cmd(&comp, 0, &mut good, 6);
        assert_eq!(*target.cmds.lock().unwrap(), vec![(0, 0x500, 0, vec![])]);
    }

    // -- gotcha: invalid opcode still bumps seq ------------------------------

    #[test]
    fn invalid_opcode_replies_invalid_opcode_and_bumps_seq() {
        let (comp, ground) = build();
        let target = Arc::new(TargetStub::default());
        let seq_stub = Arc::new(SeqStub::default());
        comp.comp_cmd_send[0].connect(target.clone(), 0);
        comp.seq_cmd_status[0].connect(seq_stub.clone(), 0);
        register(&comp, 0, 0x500);

        let mut unknown = cmd_packet(0x999, &[]);
        send_seq_cmd(&comp, 0, &mut unknown, 1);
        {
            let events = ground.events.lock().unwrap();
            let last = events.last().unwrap();
            assert_eq!(last.0, ID_BASE + CmdDispatcher::EVENTID_INVALID_COMMAND);
            assert_eq!(last.2, vec![0, 0, 0x09, 0x99]);
        }
        assert_eq!(
            *seq_stub.responses.lock().unwrap(),
            vec![(0x999, 1, CmdResponse::InvalidOpcode)]
        );

        // seq DID increment: next dispatch carries seq 1.
        let mut good = cmd_packet(0x500, &[]);
        send_seq_cmd(&comp, 0, &mut good, 2);
        assert_eq!(*target.cmds.lock().unwrap(), vec![(0, 0x500, 1, vec![])]);
    }

    // -- gotcha: registered but unconnected target ---------------------------

    #[test]
    fn registered_but_unconnected_target_is_invalid_command() {
        let (comp, ground) = build();
        let seq_stub = Arc::new(SeqStub::default());
        comp.seq_cmd_status[0].connect(seq_stub.clone(), 0);
        register(&comp, 12, 0x500); // compCmdSend[12] never connected

        let mut packet = cmd_packet(0x500, &[]);
        send_seq_cmd(&comp, 0, &mut packet, 3);
        assert_eq!(
            ground.events.lock().unwrap().last().unwrap().0,
            ID_BASE + CmdDispatcher::EVENTID_INVALID_COMMAND
        );
        assert_eq!(
            *seq_stub.responses.lock().unwrap(),
            vec![(0x500, 3, CmdResponse::InvalidOpcode)]
        );
    }

    // -- gotcha: no tracking when caller's status port unconnected -----------

    #[test]
    fn unconnected_caller_status_port_skips_tracking_but_dispatches() {
        let (comp, _ground) = build();
        let target = Arc::new(TargetStub::default());
        let seq_stub = Arc::new(SeqStub::default());
        comp.comp_cmd_send[0].connect(target.clone(), 0);
        // seq_cmd_status[2] (the caller) left unconnected; [0] connected to
        // prove nothing is forwarded anywhere.
        comp.seq_cmd_status[0].connect(seq_stub.clone(), 0);
        register(&comp, 0, 0x500);

        let mut packet = cmd_packet(0x500, &[]);
        send_seq_cmd(&comp, 2, &mut packet, 3);
        assert_eq!(target.cmds.lock().unwrap().len(), 1);

        // Completion finds no tracking entry -> no forward, no assert.
        let stat = comp.comp_cmd_stat_in(0);
        stat.target.invoke(stat.port_num, 0x500, 0, CmdResponse::Ok);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
        assert!(seq_stub.responses.lock().unwrap().is_empty());
    }

    // -- gotcha: tracker full -------------------------------------------------

    #[test]
    fn tracker_full_replies_execution_error_and_does_not_bump_seq() {
        let (comp, ground) = build();
        let target = Arc::new(TargetStub::default());
        let seq_stub = Arc::new(SeqStub::default());
        comp.comp_cmd_send[0].connect(target.clone(), 0);
        comp.seq_cmd_status[0].connect(seq_stub.clone(), 0);
        register(&comp, 0, 0x500);

        // Fill all 25 tracker slots (each dispatch tracks + bumps seq).
        for i in 0..SEQUENCER_TABLE_SIZE as u32 {
            let mut packet = cmd_packet(0x500, &[]);
            send_seq_cmd(&comp, 0, &mut packet, i);
        }
        assert_eq!(target.cmds.lock().unwrap().len(), SEQUENCER_TABLE_SIZE);

        // 26th: TooManyCommands + EXECUTION_ERROR, not dispatched.
        let mut packet = cmd_packet(0x500, &[]);
        send_seq_cmd(&comp, 0, &mut packet, 99);
        assert_eq!(target.cmds.lock().unwrap().len(), SEQUENCER_TABLE_SIZE);
        {
            let events = ground.events.lock().unwrap();
            let last = events.last().unwrap();
            assert_eq!(last.0, ID_BASE + CmdDispatcher::EVENTID_TOO_MANY_COMMANDS);
            assert_eq!(last.1, LogSeverity::WarningHi);
        }
        assert_eq!(
            seq_stub.responses.lock().unwrap().last().unwrap(),
            &(0x500, 99, CmdResponse::ExecutionError)
        );

        // seq did NOT increment on the full-tracker path: complete one
        // command to free a slot, then the next dispatch reuses seq 25.
        let stat = comp.comp_cmd_stat_in(0);
        stat.target.invoke(stat.port_num, 0x500, 0, CmdResponse::Ok);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
        let mut packet = cmd_packet(0x500, &[]);
        send_seq_cmd(&comp, 0, &mut packet, 100);
        assert_eq!(
            target.cmds.lock().unwrap().last().unwrap(),
            &(0, 0x500, SEQUENCER_TABLE_SIZE as u32, vec![])
        );
    }

    // -- own commands ---------------------------------------------------------

    fn send_own_cmd(comp: &Arc<CmdDispatcher>, opcode: FwOpcodeType, seq: u32, args: &[u8]) {
        let mut buf = CmdArgBuffer::new();
        assert!(buf.set_buff(args).is_ok());
        let p = comp.cmd_in(0);
        p.target.invoke(p.port_num, opcode, seq, &mut buf);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
    }

    #[test]
    fn own_commands_emit_events_and_respond_ok() {
        let (comp, ground) = build();
        comp.reg_commands();
        assert_eq!(
            *ground.regs.lock().unwrap(),
            vec![ID_BASE, ID_BASE + 1, ID_BASE + 2, ID_BASE + 3]
        );

        // NO_OP
        send_own_cmd(&comp, ID_BASE, 1, &[]);
        // NO_OP_STRING("hi") — string arg = u16 len + bytes.
        send_own_cmd(&comp, ID_BASE + 1, 2, &[0, 2, b'h', b'i']);
        // TEST_CMD_1(-1, 2.0f, 3)
        let mut args = vec![0xFF, 0xFF, 0xFF, 0xFF];
        args.extend_from_slice(&2.0f32.to_be_bytes());
        args.push(3);
        send_own_cmd(&comp, ID_BASE + 2, 3, &args);
        // CLEAR_TRACKING
        send_own_cmd(&comp, ID_BASE + 3, 4, &[]);

        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (ID_BASE, 1, CmdResponse::Ok),
                (ID_BASE + 1, 2, CmdResponse::Ok),
                (ID_BASE + 2, 3, CmdResponse::Ok),
                (ID_BASE + 3, 4, CmdResponse::Ok),
            ]
        );
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 3); // CLEAR_TRACKING emits no event
        assert_eq!(events[0].0, ID_BASE + CmdDispatcher::EVENTID_NO_OP_RECEIVED);
        assert_eq!(events[0].1, LogSeverity::ActivityHi);
        assert_eq!(events[0].2, Vec::<u8>::new());
        assert_eq!(
            events[1].0,
            ID_BASE + CmdDispatcher::EVENTID_NO_OP_STRING_RECEIVED
        );
        assert_eq!(events[1].2, vec![0, 2, b'h', b'i']);
        assert_eq!(
            events[2].0,
            ID_BASE + CmdDispatcher::EVENTID_TEST_CMD_1_ARGS
        );
        let mut expected = vec![0xFF, 0xFF, 0xFF, 0xFF];
        expected.extend_from_slice(&2.0f32.to_be_bytes());
        expected.push(3);
        assert_eq!(events[2].2, expected);
        let texts = ground.text_events.lock().unwrap();
        assert_eq!(texts[1].1, "Received a NO-OP string=hi");
        assert_eq!(texts[2].1, "TEST_CMD_1 args: I32: -1, F32: 2, U8: 3");
    }

    #[test]
    fn own_command_error_paths() {
        let (comp, ground) = build();
        // Residual bytes on NO_OP -> FormatError.
        send_own_cmd(&comp, ID_BASE, 1, &[9]);
        // Short TEST_CMD_1 args -> FormatError.
        send_own_cmd(&comp, ID_BASE + 2, 2, &[0, 0]);
        // Unknown local opcode -> InvalidOpcode.
        send_own_cmd(&comp, ID_BASE + 0x42, 3, &[]);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (ID_BASE, 1, CmdResponse::FormatError),
                (ID_BASE + 2, 2, CmdResponse::FormatError),
                (ID_BASE + 0x42, 3, CmdResponse::InvalidOpcode),
            ]
        );
    }

    #[test]
    fn clear_tracking_drops_pending_commands() {
        let (comp, _ground) = build();
        let target = Arc::new(TargetStub::default());
        let seq_stub = Arc::new(SeqStub::default());
        comp.comp_cmd_send[0].connect(target.clone(), 0);
        comp.seq_cmd_status[0].connect(seq_stub.clone(), 0);
        register(&comp, 0, 0x500);

        let mut packet = cmd_packet(0x500, &[]);
        send_seq_cmd(&comp, 0, &mut packet, 11);
        send_own_cmd(&comp, ID_BASE + 3, 1, &[]); // CLEAR_TRACKING

        // Completion after clear: entry gone, nothing forwarded.
        let stat = comp.comp_cmd_stat_in(0);
        stat.target.invoke(stat.port_num, 0x500, 0, CmdResponse::Ok);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
        assert!(seq_stub.responses.lock().unwrap().is_empty());
    }

    // -- overflow hook --------------------------------------------------------

    #[test]
    fn queue_overflow_hook_parses_opcode_throttles_and_counts() {
        let (comp, ground) = build();
        // Tiny queue to force overflow: rebuild with depth 2.
        let comp2 = CmdDispatcher::new("tinyDisp");
        comp2.active.queued.base.set_id_base(ID_BASE);
        comp2.evt.log_out.connect(ground.clone(), 0);
        comp2.evt.time_out.connect(Arc::new(TimeStub), 0);
        comp2.tlm.tlm_out.connect(ground.clone(), 0);
        comp2.init(2);
        drop(comp);

        let p = comp2.seq_cmd_buff_in(0);
        // 2 fit; 8 more overflow (> throttle 5).
        for i in 0..10u32 {
            let mut packet = cmd_packet(0x500 + i, &[]);
            p.target.invoke(p.port_num, &mut packet, i);
        }
        let events = ground.events.lock().unwrap();
        let dropped: Vec<_> = events
            .iter()
            .filter(|e| e.0 == ID_BASE + CmdDispatcher::EVENTID_COMMAND_DROPPED_QUEUE_OVERFLOW)
            .collect();
        // Throttle 5: only the first five overflow events emitted.
        assert_eq!(dropped.len(), 5);
        // First overflow was context 2 with opcode 0x502 (parsed by the hook).
        assert_eq!(dropped[0].1, LogSeverity::WarningHi);
        assert_eq!(dropped[0].2, vec![0, 0, 0x05, 0x02, 0, 0, 0, 2]);
        drop(events);

        // All 8 drops counted regardless of throttling. Drain the two
        // queued commands first — `run` uses the assert policy, so it must
        // not be invoked against a still-full queue.
        let _ = comp2
            .active
            .queued
            .dispatch_available_messages(comp2.as_ref());
        let run = comp2.run_in(0);
        run.target.invoke(run.port_num, 0);
        let _ = comp2
            .active
            .queued
            .dispatch_available_messages(comp2.as_ref());
        let tlm = ground.tlm.lock().unwrap();
        assert!(tlm.contains(&(
            ID_BASE + CmdDispatcher::CHANID_COMMANDS_DROPPED,
            vec![0, 0, 0, 8]
        )));
    }

    // -- telemetry on-change ---------------------------------------------------

    #[test]
    fn run_telemetry_is_on_change() {
        let (comp, ground) = build();
        let run = comp.run_in(0);
        run.target.invoke(run.port_num, 0);
        run.target.invoke(run.port_num, 0);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
        // First run writes all three zeros; second run writes nothing.
        let tlm = ground.tlm.lock().unwrap();
        assert_eq!(
            *tlm,
            vec![
                (
                    ID_BASE + CmdDispatcher::CHANID_COMMANDS_DROPPED,
                    vec![0, 0, 0, 0]
                ),
                (
                    ID_BASE + CmdDispatcher::CHANID_COMMAND_ERRORS,
                    vec![0, 0, 0, 0]
                ),
                (
                    ID_BASE + CmdDispatcher::CHANID_COMMANDS_DISPATCHED,
                    vec![0, 0, 0, 0]
                ),
            ]
        );
    }

    // -- ping -----------------------------------------------------------------

    #[test]
    fn ping_in_returns_key_on_ping_out() {
        let (comp, _ground) = build();
        let ping_stub = Arc::new(PingStub::default());
        comp.ping_out.connect(ping_stub.clone(), 0);
        let ping = comp.ping_in(0);
        ping.target.invoke(ping.port_num, 0xFEED);
        let _ = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
        assert_eq!(*ping_stub.keys.lock().unwrap(), vec![0xFEED]);
    }

    // -- active lifecycle -----------------------------------------------------

    #[test]
    fn active_lifecycle_end_to_end() {
        let (comp, ground) = build();
        let target = Arc::new(TargetStub::default());
        let seq_stub = Arc::new(SeqStub::default());
        comp.comp_cmd_send[0].connect(target.clone(), 0);
        comp.seq_cmd_status[0].connect(seq_stub.clone(), 0);
        register(&comp, 0, 0x500);
        comp.active.start(
            &comp,
            100,
            fprime_os::task::TASK_DEFAULT,
            fprime_os::task::TASK_DEFAULT,
        );

        let mut packet = cmd_packet(0x500, &[7]);
        let p = comp.seq_cmd_buff_in(0);
        p.target.invoke(p.port_num, &mut packet, 55);
        let stat = comp.comp_cmd_stat_in(0);
        stat.target.invoke(stat.port_num, 0x500, 0, CmdResponse::Ok);

        comp.active.exit();
        assert_eq!(comp.active.join(), fprime_os::task::Status::OpOk);

        assert_eq!(*target.cmds.lock().unwrap(), vec![(0, 0x500, 0, vec![7])]);
        assert_eq!(
            *seq_stub.responses.lock().unwrap(),
            vec![(0x500, 55, CmdResponse::Ok)]
        );
        assert!(
            ground
                .events
                .lock()
                .unwrap()
                .iter()
                .any(|e| e.0 == ID_BASE + CmdDispatcher::EVENTID_OP_CODE_COMPLETED)
        );
    }
}
