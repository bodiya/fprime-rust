//! # SeqDispatcher — port of `Svc::SeqDispatcher` (active)
//!
//! C++ sources: `Svc/SeqDispatcher/SeqDispatcher.{cpp,hpp,fpp}`,
//! `SeqDispatcherCommands.fppi`, `SeqDispatcherEvents.fppi`,
//! `SeqDispatcherTelemetry.fppi`, `Svc/Seq/Seq.fpp`,
//! `Svc/SeqDispatcher/docs/sdd.md`.
//!
//! Routes sequence-run requests (the `RUN` / `RUN_ARGS` commands and the
//! `seqRunIn` port) to the first available of
//! [`SEQ_DISPATCHER_SEQUENCER_PORTS`] sequencers — any component with the
//! `Svc.CmdSeqIn` / `Svc.CmdSeqCancel` / `Fw.CmdResponse` sequencer
//! interface, i.e. [`CmdSequencer`](crate::cmd_sequencer) or
//! `FpySequencer`. Index `i` of `seqRunOut` / `seqCancelOut` / `seqDoneIn`
//! / `seqStartIn` is one sequencer (FPP `match` clauses): wire
//! `seqRunOut[i] -> seqRunIn`, `seqCancelOut[i] -> seqCancelIn`,
//! `seqDone -> seqDoneIn[i]` and `seqStartOut -> seqStartIn[i]`.
//!
//! A `BLOCK` run answers its command when the sequencer reports done on
//! `seqDoneIn` (with the sequencer's response); a `NO_BLOCK` run, and a
//! `seqRunIn` request, are answered/forgotten at once. `seqStartIn`
//! notices sequences the dispatcher did not start (a ground command sent
//! straight to a sequencer) and takes the sequencer out of the pool until
//! it reports done.
//!
//! Ported quirks:
//! - `InvalidSequencer` is declared but never emitted.
//! - `errorCount` counts only `EXECUTION_ERROR` responses of `BLOCK` runs.
//! - `CANCEL_ALL` cancels every running sequencer, including the one
//!   running the sequence that issued it.
//! - Cancels leave the entry as is; it is cleared when the sequencer
//!   reports done.
//!
//! Threading: active; every input (ports and commands) is async, so all
//! handlers run on the component thread. The state mutex is the workspace
//! convention for component state; it is held across the port calls as
//! the C++ handlers run them (component thread only).

use alloc::format;
use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, CmdResponsePort, ComponentDispatch, EventGlue,
    MsgDispatchStatus, OutputPort, QueueFullPolicy, TlmGlue, async_input_port_adapter,
    component_msg_types, msg,
};
use fprime_config::{
    FW_LOG_STRING_MAX_SIZE, FwChanIdType, FwEnumStoreType, FwEventIdType, FwIdType, FwIndexType,
    FwOpcodeType, FwQueuePriorityType, FwSizeType, SEQ_DISPATCHER_SEQUENCER_PORTS,
};
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, CmdStringArg, Endianness, FileNameString, FwDefaultString,
    LogBuffer, LogSeverity, SerBuf, SerBufAny, fpp_enum, fw_assert, fw_try,
};

use crate::seq::{BlockState, CmdSeqCancelPort, CmdSeqInPort, SeqArgs};

fpp_enum! {
    /// `SeqDispatcher.CmdSequencerState` — what the dispatcher believes a
    /// sequencer is doing.
    pub enum CmdSequencerState : u8 {
        /// Ready for a sequence.
        Available = 0,
        /// Running a sequence whose `RUN` command awaits its response.
        RunningSequenceBlock = 1,
        /// Running a sequence nobody waits for.
        RunningSequenceNoBlock = 2,
    }
    default Available
}

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel): async input ports in
    /// FPP declaration order, then the async commands (one message type).
    impl SeqDispatcher {
        /// `seqRunIn` async input port.
        MSG_TYPE_SEQ_RUN_IN,
        /// `seqDoneIn` async input port.
        MSG_TYPE_SEQ_DONE_IN,
        /// `seqStartIn` async input port.
        MSG_TYPE_SEQ_START_IN,
        /// `cmdIn` (async commands).
        MSG_TYPE_CMD_IN,
    }
}

/// Queue message size: the maximum over every async invocation. The
/// command envelope is the largest: 6 (type and port), 4 (opcode), 4
/// (cmdSeq), 2 + 506 (length-prefixed `CmdArgBuffer`) = 522. `seqRunIn` /
/// `seqStartIn` need 6 + (2 + 240) + 20 = 268, `seqDoneIn` 15.
pub const QUEUE_MSG_SIZE: usize = 522;

/// FPP declares no priorities: FIFO.
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

/// The file name an idle entry shows (C++ `"<no seq>"`).
const NO_SEQ: &str = "<no seq>";

impl SeqDispatcher {
    /// `RUN(fileName: string size 240, block: Svc.BlockState)` — async,
    /// opcode 0.
    pub const OPCODE_RUN: FwOpcodeType = 0;
    /// `RUN_ARGS(fileName, block, buffer: Svc.SeqArgs)` — async, opcode 1.
    pub const OPCODE_RUN_ARGS: FwOpcodeType = 1;
    /// `LOG_STATUS()` — async, opcode 2.
    pub const OPCODE_LOG_STATUS: FwOpcodeType = 2;
    /// `CANCEL_NAME(fileName: string size 240)` — async, opcode 3.
    pub const OPCODE_CANCEL_NAME: FwOpcodeType = 3;
    /// `CANCEL_ALL()` — async, opcode 4.
    pub const OPCODE_CANCEL_ALL: FwOpcodeType = 4;

    /// `InvalidSequencer(idx: U16)` — WARNING_HI (never emitted, C++ parity).
    pub const EVENTID_INVALID_SEQUENCER: FwEventIdType = 0;
    /// `NoAvailableSequencers()` — WARNING_HI.
    pub const EVENTID_NO_AVAILABLE_SEQUENCERS: FwEventIdType = 1;
    /// `UnknownSequenceFinished(idx: U16)` — WARNING_LO.
    pub const EVENTID_UNKNOWN_SEQUENCE_FINISHED: FwEventIdType = 2;
    /// `ConflictingSequenceStarted(idx: U16, newSequence: string size 240,
    /// sequenceInInternalState: string size 240)` — WARNING_HI.
    pub const EVENTID_CONFLICTING_SEQUENCE_STARTED: FwEventIdType = 3;
    /// `UnexpectedSequenceStarted(idx: U16, newSequence: string size 240)`
    /// — WARNING_LO.
    pub const EVENTID_UNEXPECTED_SEQUENCE_STARTED: FwEventIdType = 4;
    /// `LogSequencerStatus(idx: U16, state: CmdSequencerState, filename:
    /// string size 240)` — ACTIVITY_LO.
    pub const EVENTID_LOG_SEQUENCER_STATUS: FwEventIdType = 5;
    /// `SequenceCanceled(idx: U16, filename: string size 240)` —
    /// ACTIVITY_HI.
    pub const EVENTID_SEQUENCE_CANCELED: FwEventIdType = 6;
    /// `CancelSequenceNotFound(filename: string size 240)` — WARNING_LO.
    pub const EVENTID_CANCEL_SEQUENCE_NOT_FOUND: FwEventIdType = 7;

    /// `dispatchedCount: U32`.
    pub const CHANID_DISPATCHED_COUNT: FwChanIdType = 0;
    /// `errorCount: U32`.
    pub const CHANID_ERROR_COUNT: FwChanIdType = 1;
    /// `sequencersAvailable: U32`.
    pub const CHANID_SEQUENCERS_AVAILABLE: FwChanIdType = 2;
    /// `canceledCount: U32`.
    pub const CHANID_CANCELED_COUNT: FwChanIdType = 3;
}

/// One sequencer's entry (C++ `DispatchEntry`).
#[derive(Clone)]
struct DispatchEntry {
    /// Opcode of the `BLOCK` command awaiting the response.
    op_code: FwOpcodeType,
    /// Its command sequence number.
    cmd_seq: u32,
    /// The sequencer's state.
    state: CmdSequencerState,
    /// The sequence it is running (an `Fw::String`).
    sequence_running: FwDefaultString,
}

/// Component state.
struct DispatcherState {
    /// Sequences dispatched (successful or otherwise).
    dispatched_count: u32,
    /// `EXECUTION_ERROR`s of dispatched `BLOCK` sequences.
    error_count: u32,
    /// Sequencers in state `Available`.
    sequencers_available: u32,
    /// Sequences canceled by `CANCEL_NAME` / `CANCEL_ALL`.
    canceled_count: u32,
    /// C++ `m_entryTable`.
    entries: [DispatchEntry; SEQ_DISPATCHER_SEQUENCER_PORTS],
}

/// `Svc::SeqDispatcher`.
pub struct SeqDispatcher {
    /// Active core: PassiveBase + queue + task.
    pub active: ActiveBase,
    /// `cmdRegOut` / `cmdResponseOut`.
    pub cmd: CmdGlue,
    /// `logOut` / `logTextOut` / `timeCaller`.
    pub evt: EventGlue,
    /// `tlmOut`.
    pub tlm: TlmGlue,
    /// `seqRunOut: [SeqDispatcherSequencerPorts] Svc.CmdSeqIn`.
    pub seq_run_out: [OutputPort<dyn CmdSeqInPort>; SEQ_DISPATCHER_SEQUENCER_PORTS],
    /// `seqCancelOut: [SeqDispatcherSequencerPorts] Svc.CmdSeqCancel`.
    pub seq_cancel_out: [OutputPort<dyn CmdSeqCancelPort>; SEQ_DISPATCHER_SEQUENCER_PORTS],
    state: Mutex<DispatcherState>,
}

impl SeqDispatcher {
    /// Construct: every sequencer available, running `"<no seq>"`.
    pub fn new(name: &str) -> Arc<Self> {
        let idle = DispatchEntry {
            op_code: 0,
            cmd_seq: 0,
            state: CmdSequencerState::Available,
            sequence_running: FwDefaultString::from(NO_SEQ),
        };
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            seq_run_out: [const { OutputPort::new() }; SEQ_DISPATCHER_SEQUENCER_PORTS],
            seq_cancel_out: [const { OutputPort::new() }; SEQ_DISPATCHER_SEQUENCER_PORTS],
            state: Mutex::new(DispatcherState {
                dispatched_count: 0,
                error_count: 0,
                sequencers_available: SEQ_DISPATCHER_SEQUENCER_PORTS as u32,
                canceled_count: 0,
                entries: core::array::from_fn(|_| idle.clone()),
            }),
        })
    }

    /// Create the message queue (C++ `init(queueDepth, instance)`).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_RUN,
                Self::OPCODE_RUN_ARGS,
                Self::OPCODE_LOG_STATUS,
                Self::OPCODE_CANCEL_NAME,
                Self::OPCODE_CANCEL_ALL,
            ],
        );
    }

    // -- Telemetry / events ---------------------------------------------------

    fn tlm_write(&self, id: FwChanIdType, value: u32) {
        self.tlm
            .tlm_write(self.id_base(), id, &value, self.evt.time_get());
    }

    fn log_idx(&self, id: FwEventIdType, severity: LogSeverity, text: &str, idx: u16) {
        self.evt
            .log_event(self.id_base(), id, severity, text, |buf: &mut LogBuffer| {
                buf.serialize_u16_be(idx)
            });
    }

    fn log_idx_name(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        text: &str,
        idx: u16,
        name: &FwDefaultString,
    ) {
        self.evt
            .log_event(self.id_base(), id, severity, text, |buf: &mut LogBuffer| {
                fw_try!(buf.serialize_u16_be(idx));
                name.serialize_to_truncated(buf, FW_LOG_STRING_MAX_SIZE, Endianness::Big)
            });
    }

    // -- Internals ------------------------------------------------------------

    /// C++ `getNextAvailableSequencerIdx()`: the first connected, available
    /// sequencer.
    fn next_available_sequencer_idx(&self, st: &DispatcherState) -> Option<usize> {
        (0..SEQ_DISPATCHER_SEQUENCER_PORTS).find(|&i| {
            self.seq_run_out[i].is_connected()
                && st.entries[i].state == CmdSequencerState::Available
        })
    }

    /// C++ `runSequence(idx, fileName, block, args)`.
    fn run_sequence(
        &self,
        st: &mut DispatcherState,
        idx: usize,
        file_name: &[u8],
        block: BlockState,
        args: &SeqArgs,
    ) {
        // Internal use only: the caller guarantees a valid, connected,
        // available sequencer.
        fw_assert!(idx < SEQ_DISPATCHER_SEQUENCER_PORTS, idx);
        fw_assert!(self.seq_run_out[idx].is_connected());
        fw_assert!(
            st.entries[idx].state == CmdSequencerState::Available,
            st.entries[idx].state.as_repr()
        );
        st.entries[idx].state = if block == BlockState::NoBlock {
            CmdSequencerState::RunningSequenceNoBlock
        } else {
            CmdSequencerState::RunningSequenceBlock
        };
        st.sequencers_available = st.sequencers_available.wrapping_sub(1);
        self.tlm_write(Self::CHANID_SEQUENCERS_AVAILABLE, st.sequencers_available);
        st.entries[idx].sequence_running.set_bytes(file_name);
        st.dispatched_count = st.dispatched_count.wrapping_add(1);
        self.tlm_write(Self::CHANID_DISPATCHED_COUNT, st.dispatched_count);
        let mut filename = FileNameString::new();
        filename.set_bytes(st.entries[idx].sequence_running.as_bytes());
        let p = self.seq_run_out[idx].get();
        p.target.invoke(p.port_num, &filename, args);
    }

    // -- Port handlers (component thread) ------------------------------------

    /// `seqStartIn_handler`: a sequencer started a sequence.
    fn seq_start_in_handler(
        &self,
        port_num: FwIndexType,
        file_name: &FileNameString,
        _args: &SeqArgs,
    ) {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < SEQ_DISPATCHER_SEQUENCER_PORTS,
            port_num
        );
        let idx = port_num as usize;
        let mut st = self.state.lock();
        let entry_state = st.entries[idx].state;
        if entry_state == CmdSequencerState::RunningSequenceBlock
            || entry_state == CmdSequencerState::RunningSequenceNoBlock
        {
            // We knew it was running: check it runs what we think.
            if st.entries[idx].sequence_running.as_bytes() != file_name.as_bytes() {
                let mut new_sequence = FwDefaultString::new();
                new_sequence.set_bytes(file_name.as_bytes());
                let old = st.entries[idx].sequence_running.clone();
                self.evt.log_event(
                    self.id_base(),
                    Self::EVENTID_CONFLICTING_SEQUENCE_STARTED,
                    LogSeverity::WarningHi,
                    &format!(
                        "Sequencer {idx} started a sequence {new_sequence} while still running {old}"
                    ),
                    |buf: &mut LogBuffer| {
                        fw_try!(buf.serialize_u16_be(idx as u16));
                        fw_try!(new_sequence.serialize_to_truncated(
                            buf,
                            FW_LOG_STRING_MAX_SIZE,
                            Endianness::Big
                        ));
                        old.serialize_to_truncated(buf, FW_LOG_STRING_MAX_SIZE, Endianness::Big)
                    },
                );
                st.entries[idx].sequence_running = new_sequence;
            }
        } else {
            // Ground commanded that sequencer directly: warn, and track it.
            let mut new_sequence = FwDefaultString::new();
            new_sequence.set_bytes(file_name.as_bytes());
            self.log_idx_name(
                Self::EVENTID_UNEXPECTED_SEQUENCE_STARTED,
                LogSeverity::WarningLo,
                &format!(
                    "Sequencer {idx} was externally commanded to start a sequence {new_sequence}"
                ),
                idx as u16,
                &new_sequence,
            );
            st.entries[idx].state = CmdSequencerState::RunningSequenceNoBlock;
            st.entries[idx].sequence_running = new_sequence;
            st.sequencers_available = st.sequencers_available.wrapping_sub(1);
            self.tlm_write(Self::CHANID_SEQUENCERS_AVAILABLE, st.sequencers_available);
        }
    }

    /// `seqDoneIn_handler`: a sequencer finished a sequence.
    fn seq_done_in_handler(
        &self,
        port_num: FwIndexType,
        _op_code: FwOpcodeType,
        _cmd_seq: u32,
        response: CmdResponse,
    ) {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < SEQ_DISPATCHER_SEQUENCER_PORTS,
            port_num
        );
        let idx = port_num as usize;
        let mut st = self.state.lock();
        let entry_state = st.entries[idx].state;
        if entry_state != CmdSequencerState::RunningSequenceBlock
            && entry_state != CmdSequencerState::RunningSequenceNoBlock
        {
            // A sequence we never heard of finished; the sequencer is
            // already counted available, so just reset the entry.
            self.log_idx(
                Self::EVENTID_UNKNOWN_SEQUENCE_FINISHED,
                LogSeverity::WarningLo,
                &format!(
                    "Sequencer {idx} completed a sequence with no matching start notification"
                ),
                idx as u16,
            );
            st.entries[idx].state = CmdSequencerState::Available;
            st.entries[idx].sequence_running.set(NO_SEQ);
            return;
        }
        if entry_state == CmdSequencerState::RunningSequenceBlock {
            // Answer the blocked RUN command with the sequence's result.
            self.cmd
                .cmd_response(st.entries[idx].op_code, st.entries[idx].cmd_seq, response);
            if response == CmdResponse::ExecutionError {
                st.error_count = st.error_count.wrapping_add(1);
                self.tlm_write(Self::CHANID_ERROR_COUNT, st.error_count);
            }
        }
        // Any response means the sequencer is free again.
        st.entries[idx].state = CmdSequencerState::Available;
        st.entries[idx].sequence_running.set(NO_SEQ);
        st.sequencers_available = st.sequencers_available.wrapping_add(1);
        self.tlm_write(Self::CHANID_SEQUENCERS_AVAILABLE, st.sequencers_available);
    }

    /// `seqRunIn_handler`: run a sequence on the first available sequencer,
    /// non-blocking.
    fn seq_run_in_handler(
        &self,
        _port_num: FwIndexType,
        file_name: &FileNameString,
        args: &SeqArgs,
    ) {
        let mut st = self.state.lock();
        match self.next_available_sequencer_idx(&st) {
            None => self.log_no_available_sequencers(),
            Some(idx) => self.run_sequence(
                &mut st,
                idx,
                file_name.as_bytes(),
                BlockState::NoBlock,
                args,
            ),
        }
    }

    fn log_no_available_sequencers(&self) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_NO_AVAILABLE_SEQUENCERS,
            LogSeverity::WarningHi,
            "No available cmd sequencers to dispatch a sequence to",
            |_buf| fprime_fw::SerializeStatus::Ok,
        );
    }

    // -- Command handlers (component thread) ---------------------------------

    /// `RUN_ARGS_cmdHandler` (`RUN` delegates here with empty arguments).
    fn run_args_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        file_name: &CmdStringArg,
        block: BlockState,
        args: &SeqArgs,
    ) {
        let mut st = self.state.lock();
        let Some(idx) = self.next_available_sequencer_idx(&st) else {
            self.log_no_available_sequencers();
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        };
        self.run_sequence(&mut st, idx, file_name.as_bytes(), block, args);
        if block == BlockState::NoBlock {
            // Return at once.
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
        } else {
            // Answer when the sequencer reports done.
            st.entries[idx].op_code = op_code;
            st.entries[idx].cmd_seq = cmd_seq;
        }
    }

    /// `LOG_STATUS_cmdHandler`: one event per sequencer.
    fn log_status_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        let st = self.state.lock();
        for (idx, entry) in st.entries.iter().enumerate() {
            let state = entry.state;
            let name = &entry.sequence_running;
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_LOG_SEQUENCER_STATUS,
                LogSeverity::ActivityLo,
                &format!("Sequencer {idx} with state {state:?} is running file {name}"),
                |buf: &mut LogBuffer| {
                    fw_try!(buf.serialize_u16_be(idx as u16));
                    fw_try!(buf.serialize_u8_be(state.as_repr()));
                    name.serialize_to_truncated(buf, FW_LOG_STRING_MAX_SIZE, Endianness::Big)
                },
            );
        }
        drop(st);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// Cancel sequencer `idx` (shared by both cancel commands): the entry
    /// is cleared later, by `seqDoneIn`.
    fn cancel_sequencer(&self, st: &mut DispatcherState, idx: usize) {
        let p = self.seq_cancel_out[idx].get();
        p.target.invoke(p.port_num);
        let name = st.entries[idx].sequence_running.clone();
        self.log_idx_name(
            Self::EVENTID_SEQUENCE_CANCELED,
            LogSeverity::ActivityHi,
            &format!("Sequencer {idx} canceled running sequence {name}"),
            idx as u16,
            &name,
        );
        st.canceled_count = st.canceled_count.wrapping_add(1);
        self.tlm_write(Self::CHANID_CANCELED_COUNT, st.canceled_count);
    }

    /// `CANCEL_NAME_cmdHandler`: cancel every running sequencer whose
    /// sequence has this name.
    fn cancel_name_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        file_name: &CmdStringArg,
    ) {
        let mut st = self.state.lock();
        let mut canceled = false;
        for idx in 0..SEQ_DISPATCHER_SEQUENCER_PORTS {
            let running = st.entries[idx].state != CmdSequencerState::Available;
            if running
                && st.entries[idx].sequence_running.as_bytes() == file_name.as_bytes()
                && self.seq_cancel_out[idx].is_connected()
            {
                self.cancel_sequencer(&mut st, idx);
                canceled = true;
            }
        }
        drop(st);
        if canceled {
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
        } else {
            let mut name = FwDefaultString::new();
            name.set_bytes(file_name.as_bytes());
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_CANCEL_SEQUENCE_NOT_FOUND,
                LogSeverity::WarningLo,
                &format!("No running sequence matched file {name} to cancel"),
                |buf: &mut LogBuffer| {
                    name.serialize_to_truncated(buf, FW_LOG_STRING_MAX_SIZE, Endianness::Big)
                },
            );
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
        }
    }

    /// `CANCEL_ALL_cmdHandler`: cancel every running sequencer — the
    /// caller's own included.
    fn cancel_all_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        let mut st = self.state.lock();
        for idx in 0..SEQ_DISPATCHER_SEQUENCER_PORTS {
            let running = st.entries[idx].state != CmdSequencerState::Available;
            if running && self.seq_cancel_out[idx].is_connected() {
                self.cancel_sequencer(&mut st, idx);
            }
        }
        drop(st);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// The generated `cmdIn` dispatch: argument decoding with the
    /// `FormatError` (short buffer or residual bytes) / `ValidationError`
    /// (undeclared enum value) / `InvalidOpcode` responses.
    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let format_error = || {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError)
        };
        match op_code.wrapping_sub(self.id_base()) {
            local @ (Self::OPCODE_RUN | Self::OPCODE_RUN_ARGS) => {
                let mut file_name = CmdStringArg::new();
                if !args.deserialize(&mut file_name, Endianness::Big).is_ok() {
                    return format_error();
                }
                let mut raw_block = 0u8;
                if !args.deserialize_u8_be(&mut raw_block).is_ok() {
                    return format_error();
                }
                let Ok(block) = BlockState::try_from(raw_block) else {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
                    return;
                };
                // RUN delegates to RUN_ARGS with empty arguments
                // (C++ `Svc::SeqArgs emptyArgs{0, 0}`).
                let mut seq_args = SeqArgs::default();
                if local == Self::OPCODE_RUN_ARGS
                    && !args.deserialize(&mut seq_args, Endianness::Big).is_ok()
                {
                    return format_error();
                }
                if args.deserialize_size_left() != 0 {
                    return format_error();
                }
                self.run_args_cmd_handler(op_code, cmd_seq, &file_name, block, &seq_args);
            }
            Self::OPCODE_CANCEL_NAME => {
                let mut file_name = CmdStringArg::new();
                if !args.deserialize(&mut file_name, Endianness::Big).is_ok()
                    || args.deserialize_size_left() != 0
                {
                    return format_error();
                }
                self.cancel_name_cmd_handler(op_code, cmd_seq, &file_name);
            }
            local @ (Self::OPCODE_LOG_STATUS | Self::OPCODE_CANCEL_ALL) => {
                if args.deserialize_size_left() != 0 {
                    return format_error();
                }
                if local == Self::OPCODE_LOG_STATUS {
                    self.log_status_cmd_handler(op_code, cmd_seq);
                } else {
                    self.cancel_all_cmd_handler(op_code, cmd_seq);
                }
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }
}

async_input_port_adapter! {
    /// `seqRunIn` — ASYNC `Svc.CmdSeqIn` input: run a sequence (non-blocking).
    component: SeqDispatcher;
    adapter: SeqRunInAdapter;
    port: CmdSeqInPort;
    input: pub seq_run_in;
    deserialize: seq_run_in_deserialize;
    handler: seq_run_in_handler;
    base: active.queued;
    msg_type: SeqDispatcher::MSG_TYPE_SEQ_RUN_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { ref filename: FileNameString, ref args: SeqArgs }
}

async_input_port_adapter! {
    /// `seqDoneIn` — ASYNC `Fw.CmdResponse` input, one per sequencer.
    component: SeqDispatcher;
    adapter: SeqDoneInAdapter;
    port: CmdResponsePort;
    input: pub seq_done_in;
    deserialize: seq_done_in_deserialize;
    handler: seq_done_in_handler;
    base: active.queued;
    msg_type: SeqDispatcher::MSG_TYPE_SEQ_DONE_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, val response: CmdResponse }
}

async_input_port_adapter! {
    /// `seqStartIn` — ASYNC `Svc.CmdSeqIn` input, one per sequencer.
    component: SeqDispatcher;
    adapter: SeqStartInAdapter;
    port: CmdSeqInPort;
    input: pub seq_start_in;
    deserialize: seq_start_in_deserialize;
    handler: seq_start_in_handler;
    base: active.queued;
    msg_type: SeqDispatcher::MSG_TYPE_SEQ_START_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { ref filename: FileNameString, ref args: SeqArgs }
}

async_input_port_adapter! {
    /// `cmdIn` — ASYNC `Fw.Cmd` input (every command is `async`).
    component: SeqDispatcher;
    adapter: CmdInAdapter;
    port: CmdPort;
    input: pub cmd_in;
    deserialize: cmd_in_deserialize;
    handler: cmd_in_handler;
    base: active.queued;
    msg_type: SeqDispatcher::MSG_TYPE_CMD_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, buf args: CmdArgBuffer }
}

impl ComponentDispatch for SeqDispatcher {
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
            Self::MSG_TYPE_SEQ_RUN_IN => match Self::seq_run_in_deserialize(buf) {
                Some((filename, args)) => {
                    self.seq_run_in_handler(port_num, &filename, &args);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_SEQ_DONE_IN => match Self::seq_done_in_deserialize(buf) {
                Some((op_code, cmd_seq, response)) => {
                    self.seq_done_in_handler(port_num, op_code, cmd_seq, response);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_SEQ_START_IN => match Self::seq_start_in_deserialize(buf) {
                Some((filename, args)) => {
                    self.seq_start_in_handler(port_num, &filename, &args);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_CMD_IN => match Self::cmd_in_deserialize(buf) {
                Some((op_code, cmd_seq, mut args)) => {
                    self.cmd_in_handler(port_num, op_code, cmd_seq, &mut args);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for SeqDispatcher {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    //! Port of `Svc/SeqDispatcher/test/ut` (all ten tests) plus the
    //! start-notification paths, argument errors and wire checks.

    use super::*;
    use fprime_comp::{CmdRegPort, LogPort, TlmPort};
    use fprime_fw::{SerializeStatus, Time, TlmBuffer};
    use fprime_os::queue::BlockingType;
    use std::sync::Mutex as StdMutex;
    use std::vec::Vec;

    const N: usize = SEQ_DISPATCHER_SEQUENCER_PORTS;
    const ID_BASE: FwIdType = 0;

    #[derive(Default)]
    struct Ground {
        regs: StdMutex<Vec<FwOpcodeType>>,
        responses: StdMutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: StdMutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        tlm: StdMutex<Vec<(FwChanIdType, u32)>>,
        /// `seqRunOut` calls: (port, file name, args).
        runs: StdMutex<Vec<(FwIndexType, String, SeqArgs)>>,
        /// `seqCancelOut` calls: port.
        cancels: StdMutex<Vec<FwIndexType>>,
    }

    use std::string::{String, ToString};

    impl CmdRegPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }

    impl CmdResponsePort for Ground {
        fn invoke(&self, _: FwIndexType, op_code: FwOpcodeType, cmd_seq: u32, r: CmdResponse) {
            self.responses.lock().unwrap().push((op_code, cmd_seq, r));
        }
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

    impl TlmPort for Ground {
        fn invoke(&self, _: FwIndexType, id: FwChanIdType, _: &mut Time, val: &mut TlmBuffer) {
            let bytes = val.as_slice();
            let value = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
            self.tlm.lock().unwrap().push((id, value));
        }
    }

    impl CmdSeqInPort for Ground {
        fn invoke(&self, port_num: FwIndexType, filename: &FileNameString, args: &SeqArgs) {
            self.runs.lock().unwrap().push((
                port_num,
                filename.as_str().unwrap().to_string(),
                *args,
            ));
        }
    }

    impl CmdSeqCancelPort for Ground {
        fn invoke(&self, port_num: FwIndexType) {
            self.cancels.lock().unwrap().push(port_num);
        }
    }

    struct Tester {
        comp: Arc<SeqDispatcher>,
        ground: Arc<Ground>,
    }

    impl Tester {
        fn new() -> Self {
            let ground = Arc::new(Ground::default());
            let comp = SeqDispatcher::new("SeqDispatcher");
            comp.active.queued.base.set_id_base(ID_BASE);
            comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
            comp.cmd.cmd_response_out.connect(ground.clone(), 0);
            comp.evt.log_out.connect(ground.clone(), 0);
            comp.tlm.tlm_out.connect(ground.clone(), 0);
            for i in 0..N {
                comp.seq_run_out[i].connect(ground.clone(), i as FwIndexType);
                comp.seq_cancel_out[i].connect(ground.clone(), i as FwIndexType);
            }
            comp.init(10);
            Self { comp, ground }
        }

        fn do_dispatch(&self) {
            let status = self
                .comp
                .active
                .queued
                .do_dispatch(&*self.comp, BlockingType::NonBlocking);
            assert_eq!(status, MsgDispatchStatus::Ok);
        }

        fn clear_history(&self) {
            let g = &self.ground;
            g.responses.lock().unwrap().clear();
            g.events.lock().unwrap().clear();
            g.tlm.lock().unwrap().clear();
            g.runs.lock().unwrap().clear();
            g.cancels.lock().unwrap().clear();
        }

        fn send_cmd(&self, opcode: FwOpcodeType, args: &mut CmdArgBuffer) {
            let port = self.comp.cmd_in(0);
            port.target.invoke(port.port_num, ID_BASE + opcode, 0, args);
            self.do_dispatch();
        }

        fn run_args_buffer(file: &str, block: BlockState, args: Option<&SeqArgs>) -> CmdArgBuffer {
            let mut buf = CmdArgBuffer::new();
            let name = CmdStringArg::from(file);
            assert_eq!(buf.serialize(&name, Endianness::Big), SerializeStatus::Ok);
            assert_eq!(buf.serialize_u8_be(block.as_repr()), SerializeStatus::Ok);
            if let Some(args) = args {
                assert_eq!(buf.serialize(args, Endianness::Big), SerializeStatus::Ok);
            }
            buf
        }

        /// `sendCmd_RUN` + `doDispatch`.
        fn run(&self, file: &str, block: BlockState) {
            let mut buf = Self::run_args_buffer(file, block, None);
            self.send_cmd(SeqDispatcher::OPCODE_RUN, &mut buf);
        }

        /// `sendCmd_RUN_ARGS` + `doDispatch`.
        fn run_args(&self, file: &str, block: BlockState, args: &SeqArgs) {
            let mut buf = Self::run_args_buffer(file, block, Some(args));
            self.send_cmd(SeqDispatcher::OPCODE_RUN_ARGS, &mut buf);
        }

        fn cancel_name(&self, file: &str) {
            let mut buf = CmdArgBuffer::new();
            let _ = buf.serialize(&CmdStringArg::from(file), Endianness::Big);
            self.send_cmd(SeqDispatcher::OPCODE_CANCEL_NAME, &mut buf);
        }

        fn no_arg_cmd(&self, opcode: FwOpcodeType) {
            self.send_cmd(opcode, &mut CmdArgBuffer::new());
        }

        /// `invoke_to_seqDoneIn` + `doDispatch`.
        fn seq_done(&self, port: FwIndexType, response: CmdResponse) {
            let p = self.comp.seq_done_in(port);
            p.target.invoke(p.port_num, 0, 0, response);
            self.do_dispatch();
        }

        /// `invoke_to_seqStartIn` + `doDispatch`.
        fn seq_start(&self, port: FwIndexType, file: &str) {
            let p = self.comp.seq_start_in(port);
            p.target
                .invoke(p.port_num, &FileNameString::from(file), &SeqArgs::default());
            self.do_dispatch();
        }

        fn responses(&self) -> Vec<(FwOpcodeType, u32, CmdResponse)> {
            self.ground.responses.lock().unwrap().clone()
        }

        fn event_ids(&self) -> Vec<FwEventIdType> {
            self.ground
                .events
                .lock()
                .unwrap()
                .iter()
                .map(|e| e.0)
                .collect()
        }

        fn events_of(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
            self.ground
                .events
                .lock()
                .unwrap()
                .iter()
                .filter(|e| e.0 == id)
                .map(|e| e.2.clone())
                .collect()
        }

        fn tlm_of(&self, id: FwChanIdType) -> Vec<u32> {
            self.ground
                .tlm
                .lock()
                .unwrap()
                .iter()
                .filter(|t| t.0 == id)
                .map(|t| t.1)
                .collect()
        }

        fn runs(&self) -> Vec<(FwIndexType, String, SeqArgs)> {
            self.ground.runs.lock().unwrap().clone()
        }

        fn cancels(&self) -> Vec<FwIndexType> {
            self.ground.cancels.lock().unwrap().clone()
        }
    }

    /// `[idx u16][u16 len][name]`.
    fn idx_name(idx: u16, name: &str) -> Vec<u8> {
        let mut out = idx.to_be_bytes().to_vec();
        out.extend((name.len() as u16).to_be_bytes());
        out.extend(name.as_bytes());
        out
    }

    fn empty_args() -> SeqArgs {
        SeqArgs::default()
    }

    #[test]
    fn test_dispatch() {
        let t = Tester::new();
        // Fill every sequencer with a blocking sequence.
        for _ in 0..N {
            t.run("test", BlockState::Block);
            // No response: blocking.
            assert!(t.responses().is_empty());
            assert!(t.event_ids().is_empty());
        }
        assert_eq!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE)[N - 1],
            0
        );
        t.clear_history();

        // All busy.
        t.run("test", BlockState::Block);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN, 0, CmdResponse::ExecutionError)]
        );
        t.clear_history();

        t.seq_done(0, CmdResponse::Ok);
        assert!(t.event_ids().is_empty());
        // The blocked RUN gets its response now.
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN, 0, CmdResponse::Ok)]
        );
        t.clear_history();

        // Non-blocking returns at once.
        t.run("test", BlockState::NoBlock);
        assert!(t.event_ids().is_empty());
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN, 0, CmdResponse::Ok)]
        );
        t.clear_history();

        // A blocking sequence that errors answers with the error.
        t.seq_done(1, CmdResponse::ExecutionError);
        assert!(t.event_ids().is_empty());
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN, 0, CmdResponse::ExecutionError)]
        );
        assert_eq!(t.tlm_of(SeqDispatcher::CHANID_ERROR_COUNT), vec![1]);
    }

    #[test]
    fn test_log_status() {
        let t = Tester::new();
        t.run("test", BlockState::Block);
        t.clear_history();
        t.no_arg_cmd(SeqDispatcher::OPCODE_LOG_STATUS);
        assert_eq!(t.event_ids().len(), N);
        let statuses = t.events_of(SeqDispatcher::EVENTID_LOG_SEQUENCER_STATUS);
        let mut first = 0u16.to_be_bytes().to_vec();
        first.push(CmdSequencerState::RunningSequenceBlock.as_repr());
        first.extend(4u16.to_be_bytes());
        first.extend(b"test");
        assert_eq!(statuses[0], first);
        let mut second = 1u16.to_be_bytes().to_vec();
        second.push(CmdSequencerState::Available.as_repr());
        second.extend(8u16.to_be_bytes());
        second.extend(b"<no seq>");
        assert_eq!(statuses[1], second);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_LOG_STATUS, 0, CmdResponse::Ok)]
        );
    }

    #[test]
    fn test_run_args_with_valid_arguments() {
        let t = Tester::new();
        let args = empty_args();
        t.run_args("test", BlockState::NoBlock, &args);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN_ARGS, 0, CmdResponse::Ok)]
        );
        assert_eq!(t.runs(), vec![(0, "test".to_string(), args)]);
        assert_eq!(t.tlm_of(SeqDispatcher::CHANID_DISPATCHED_COUNT), vec![1]);
        assert_eq!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE),
            vec![N as u32 - 1]
        );
    }

    #[test]
    fn test_run_args_with_max_sized_arguments() {
        let t = Tester::new();
        let mut args = SeqArgs {
            size: crate::seq::SEQUENCE_ARGUMENTS_MAX_SIZE as FwSizeType,
            ..SeqArgs::default()
        };
        for (i, b) in args.buffer.elements_mut().iter_mut().enumerate() {
            *b = (i % 256) as u8;
        }
        t.run_args("test", BlockState::NoBlock, &args);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN_ARGS, 0, CmdResponse::Ok)]
        );
        assert_eq!(t.runs(), vec![(0, "test".to_string(), args)]);
        assert_eq!(t.tlm_of(SeqDispatcher::CHANID_DISPATCHED_COUNT), vec![1]);
    }

    #[test]
    fn test_run_args_no_sequencers_available() {
        let t = Tester::new();
        for _ in 0..N {
            t.run_args("test", BlockState::Block, &empty_args());
            assert!(t.responses().is_empty());
        }
        assert_eq!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE)[N - 1],
            0
        );
        t.clear_history();
        t.run_args("test", BlockState::Block, &empty_args());
        assert_eq!(
            t.responses(),
            vec![(
                SeqDispatcher::OPCODE_RUN_ARGS,
                0,
                CmdResponse::ExecutionError
            )]
        );
        assert_eq!(
            t.event_ids(),
            vec![SeqDispatcher::EVENTID_NO_AVAILABLE_SEQUENCERS]
        );
        assert!(t.runs().is_empty());
    }

    #[test]
    fn test_run_args_blocking_vs_non_blocking() {
        let t = Tester::new();
        let args = empty_args();
        t.run_args("nonblocking", BlockState::NoBlock, &args);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN_ARGS, 0, CmdResponse::Ok)]
        );
        assert_eq!(t.runs().len(), 1);
        t.clear_history();
        t.seq_done(0, CmdResponse::Ok);
        t.clear_history();

        t.run_args("blocking", BlockState::Block, &args);
        assert!(t.responses().is_empty());
        assert_eq!(t.runs(), vec![(0, "blocking".to_string(), args)]);
        t.seq_done(0, CmdResponse::Ok);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN_ARGS, 0, CmdResponse::Ok)]
        );

        t.clear_history();
        t.run_args("blocking_error", BlockState::Block, &args);
        assert!(t.responses().is_empty());
        t.seq_done(0, CmdResponse::ExecutionError);
        assert_eq!(
            t.responses(),
            vec![(
                SeqDispatcher::OPCODE_RUN_ARGS,
                0,
                CmdResponse::ExecutionError
            )]
        );
        assert_eq!(t.tlm_of(SeqDispatcher::CHANID_ERROR_COUNT), vec![1]);
    }

    #[test]
    fn test_cancel_name() {
        let t = Tester::new();
        t.run_args("test", BlockState::NoBlock, &empty_args());
        assert_eq!(t.runs().len(), 1);
        assert_eq!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE),
            vec![N as u32 - 1]
        );
        t.clear_history();

        t.cancel_name("test");
        assert_eq!(t.cancels(), vec![0]);
        assert_eq!(
            t.events_of(SeqDispatcher::EVENTID_SEQUENCE_CANCELED),
            vec![idx_name(0, "test")]
        );
        assert_eq!(t.tlm_of(SeqDispatcher::CHANID_CANCELED_COUNT), vec![1]);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_CANCEL_NAME, 0, CmdResponse::Ok)]
        );
        t.clear_history();

        // The canceled sequencer reports done: state cleared.
        t.seq_done(0, CmdResponse::ExecutionError);
        assert_eq!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE),
            vec![N as u32]
        );
        // NO_BLOCK: nobody to answer, no error counted.
        assert!(t.responses().is_empty());
        assert!(t.tlm_of(SeqDispatcher::CHANID_ERROR_COUNT).is_empty());
    }

    #[test]
    fn test_cancel_name_not_found() {
        let t = Tester::new();
        t.cancel_name("does_not_exist");
        assert!(t.cancels().is_empty());
        assert_eq!(
            t.events_of(SeqDispatcher::EVENTID_CANCEL_SEQUENCE_NOT_FOUND),
            vec![idx_name(14, "does_not_exist")[2..].to_vec()]
        );
        assert_eq!(
            t.responses(),
            vec![(
                SeqDispatcher::OPCODE_CANCEL_NAME,
                0,
                CmdResponse::ExecutionError
            )]
        );
    }

    #[test]
    fn test_cancel_all() {
        let t = Tester::new();
        for _ in 0..N {
            t.run_args("test", BlockState::NoBlock, &empty_args());
        }
        assert_eq!(t.runs().len(), N);
        assert_eq!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE)[N - 1],
            0
        );
        t.clear_history();

        t.no_arg_cmd(SeqDispatcher::OPCODE_CANCEL_ALL);
        assert_eq!(t.cancels(), (0..N as FwIndexType).collect::<Vec<_>>());
        assert_eq!(
            t.events_of(SeqDispatcher::EVENTID_SEQUENCE_CANCELED).len(),
            N
        );
        let canceled = t.tlm_of(SeqDispatcher::CHANID_CANCELED_COUNT);
        assert_eq!(canceled.len(), N);
        assert_eq!(canceled[N - 1], N as u32);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_CANCEL_ALL, 0, CmdResponse::Ok)]
        );
        t.clear_history();

        for i in 0..N {
            t.seq_done(i as FwIndexType, CmdResponse::ExecutionError);
        }
        assert_eq!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE)[N - 1],
            N as u32
        );
    }

    #[test]
    fn test_cancel_all_none_running() {
        let t = Tester::new();
        t.no_arg_cmd(SeqDispatcher::OPCODE_CANCEL_ALL);
        assert!(t.cancels().is_empty());
        assert!(
            t.events_of(SeqDispatcher::EVENTID_SEQUENCE_CANCELED)
                .is_empty()
        );
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_CANCEL_ALL, 0, CmdResponse::Ok)]
        );
    }

    #[test]
    fn seq_run_in_dispatches_non_blocking() {
        let t = Tester::new();
        let p = t.comp.seq_run_in(0);
        let mut args = SeqArgs {
            size: 1,
            ..SeqArgs::default()
        };
        args.buffer[0] = 0xAB;
        p.target
            .invoke(p.port_num, &FileNameString::from("port.bin"), &args);
        t.do_dispatch();
        assert_eq!(t.runs(), vec![(0, "port.bin".to_string(), args)]);
        assert!(t.responses().is_empty());
        // Done with a NO_BLOCK entry: no response.
        t.seq_done(0, CmdResponse::Ok);
        assert!(t.responses().is_empty());

        // No sequencer free: event only.
        t.run("a", BlockState::NoBlock);
        t.run("b", BlockState::NoBlock);
        t.clear_history();
        p.target
            .invoke(p.port_num, &FileNameString::from("c"), &SeqArgs::default());
        t.do_dispatch();
        assert_eq!(
            t.event_ids(),
            vec![SeqDispatcher::EVENTID_NO_AVAILABLE_SEQUENCERS]
        );
        assert!(t.runs().is_empty());
    }

    #[test]
    fn unconnected_sequencers_are_skipped() {
        let ground = Arc::new(Ground::default());
        let comp = SeqDispatcher::new("SeqDispatcher");
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        // Only sequencer 1 is wired.
        comp.seq_run_out[1].connect(ground.clone(), 1);
        comp.init(4);
        let t = Tester { comp, ground };
        t.run("x", BlockState::NoBlock);
        assert_eq!(t.runs(), vec![(1, "x".to_string(), SeqArgs::default())]);
        t.clear_history();
        t.run("y", BlockState::NoBlock);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN, 0, CmdResponse::ExecutionError)]
        );
        // CANCEL_NAME needs a connected cancel port.
        t.clear_history();
        t.cancel_name("x");
        assert_eq!(
            t.responses(),
            vec![(
                SeqDispatcher::OPCODE_CANCEL_NAME,
                0,
                CmdResponse::ExecutionError
            )]
        );
    }

    #[test]
    fn unexpected_and_conflicting_starts() {
        let t = Tester::new();
        // Ground started sequencer 1 directly.
        t.seq_start(1, "ground.bin");
        assert_eq!(
            t.events_of(SeqDispatcher::EVENTID_UNEXPECTED_SEQUENCE_STARTED),
            vec![idx_name(1, "ground.bin")]
        );
        assert_eq!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE),
            vec![N as u32 - 1]
        );
        t.clear_history();
        // A start of what we dispatched is silent...
        t.run("mine.bin", BlockState::NoBlock);
        assert_eq!(t.runs()[0].0, 0);
        t.clear_history();
        t.seq_start(0, "mine.bin");
        assert!(t.event_ids().is_empty());
        // ...a different file is a conflict, and the entry follows it.
        t.seq_start(0, "other.bin");
        let mut conflict = idx_name(0, "other.bin");
        conflict.extend(8u16.to_be_bytes());
        conflict.extend(b"mine.bin");
        assert_eq!(
            t.events_of(SeqDispatcher::EVENTID_CONFLICTING_SEQUENCE_STARTED),
            vec![conflict]
        );
        t.clear_history();
        t.cancel_name("other.bin");
        assert_eq!(t.cancels(), vec![0]);
    }

    #[test]
    fn unknown_sequence_finished() {
        let t = Tester::new();
        t.seq_done(1, CmdResponse::Ok);
        assert_eq!(
            t.events_of(SeqDispatcher::EVENTID_UNKNOWN_SEQUENCE_FINISHED),
            vec![1u16.to_be_bytes().to_vec()]
        );
        // Already counted available: no change, no telemetry.
        assert!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE)
                .is_empty()
        );
        assert!(t.responses().is_empty());
    }

    #[test]
    fn available_count_tracks_external_starts() {
        let t = Tester::new();
        // Both sequencers started externally.
        t.seq_start(0, "a");
        t.seq_start(1, "b");
        assert_eq!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE),
            vec![1, 0]
        );
        // Done twice on the same index: the second is "unknown".
        t.seq_done(0, CmdResponse::Ok);
        t.seq_done(0, CmdResponse::Ok);
        assert_eq!(
            t.events_of(SeqDispatcher::EVENTID_UNKNOWN_SEQUENCE_FINISHED)
                .len(),
            1
        );
        assert_eq!(
            t.tlm_of(SeqDispatcher::CHANID_SEQUENCERS_AVAILABLE),
            vec![1, 0, 1]
        );
    }

    #[test]
    fn command_argument_errors() {
        let t = Tester::new();
        // Undeclared BlockState.
        let mut buf = CmdArgBuffer::new();
        let _ = buf.serialize(&CmdStringArg::from("f"), Endianness::Big);
        let _ = buf.serialize_u8_be(7);
        t.send_cmd(SeqDispatcher::OPCODE_RUN, &mut buf);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN, 0, CmdResponse::ValidationError)]
        );
        t.clear_history();
        // RUN_ARGS without its SeqArgs.
        let mut buf = Tester::run_args_buffer("f", BlockState::NoBlock, None);
        t.send_cmd(SeqDispatcher::OPCODE_RUN_ARGS, &mut buf);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN_ARGS, 0, CmdResponse::FormatError)]
        );
        t.clear_history();
        // RUN with residual bytes.
        let mut buf = Tester::run_args_buffer("f", BlockState::NoBlock, Some(&empty_args()));
        t.send_cmd(SeqDispatcher::OPCODE_RUN, &mut buf);
        assert_eq!(
            t.responses(),
            vec![(SeqDispatcher::OPCODE_RUN, 0, CmdResponse::FormatError)]
        );
        t.clear_history();
        // LOG_STATUS with an argument.
        let mut buf = CmdArgBuffer::new();
        let _ = buf.serialize_u8_be(1);
        t.send_cmd(SeqDispatcher::OPCODE_LOG_STATUS, &mut buf);
        assert_eq!(
            t.responses(),
            vec![(
                SeqDispatcher::OPCODE_LOG_STATUS,
                0,
                CmdResponse::FormatError
            )]
        );
        t.clear_history();
        // Unknown opcode.
        t.send_cmd(5, &mut CmdArgBuffer::new());
        assert_eq!(t.responses(), vec![(5, 0, CmdResponse::InvalidOpcode)]);
        assert!(t.runs().is_empty());
    }

    #[test]
    fn reg_commands_registers_all_five() {
        let t = Tester::new();
        t.comp.active.queued.base.set_id_base(0x1000);
        t.comp.reg_commands();
        assert_eq!(
            *t.ground.regs.lock().unwrap(),
            vec![0x1000, 0x1001, 0x1002, 0x1003, 0x1004]
        );
    }

    #[test]
    fn seq_start_envelope_is_byte_exact() {
        let t = Tester::new();
        let p = t.comp.seq_start_in(1);
        let args = SeqArgs {
            size: 2,
            ..SeqArgs::default()
        };
        p.target
            .invoke(p.port_num, &FileNameString::from("ab"), &args);
        let mut dest = [0u8; QUEUE_MSG_SIZE];
        let mut size = 0;
        let mut priority = 0;
        let status = t.comp.active.queued.queue().receive(
            &mut dest,
            BlockingType::NonBlocking,
            &mut size,
            &mut priority,
        );
        assert_eq!(status, fprime_os::queue::Status::OpOk);
        let mut expected = vec![0, 0, 0, 3, 0, 1, 0, 2, b'a', b'b'];
        expected.extend(2u64.to_be_bytes());
        expected.extend([0u8; crate::seq::SEQUENCE_ARGUMENTS_MAX_SIZE]);
        assert_eq!(&dest[..size as usize], &expected[..]);
    }
}
