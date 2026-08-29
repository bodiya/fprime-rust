//! # Svc::FileDownlink — file send component (ACTIVE)
//!
//! Port of `Svc/FileDownlink/{FileDownlink,File,Warnings}.cpp`,
//! `FileDownlink.fpp` and `Svc/FileDownlinkPorts/FileDownlinkPorts.fpp` per
//! `docs/cpp-analysis/file-services.md` (§ "Svc::FileDownlink" + the
//! FileDownlink gotcha list).
//!
//! Requests arrive from the `SendFile`/`SendPartial` commands or the GUARDED
//! `SendFile` port and are queued as [`FileEntry`] records. A `Run` (Sched)
//! tick pops one and drives a five-state machine
//! (`IDLE → DOWNLINK/WAIT/CANCEL → COOLDOWN → IDLE`) that sends **exactly
//! one packet** and then blocks in `WAIT` until that same buffer comes back
//! on `bufferReturn` — a strict one-packet credit.
//!
//! Ported quirks (all from the analysis gotcha list, all covered by tests):
//!
//! - there is **no** WAIT timeout: `m_curTimer` accumulates forever and
//!   nothing ever compares it (`m_timeout` is declared and never read), so a
//!   hung consumer stalls the component permanently. Do not invent one;
//! - the guarded `SendFile` port answers `STATUS_ERROR` (never
//!   `STATUS_BUSY`) with `context = U32::MAX` when the internal queue is
//!   full; `STATUS_BUSY` is produced by no code path;
//! - a successful enqueue produces **no** command response — the response is
//!   deferred until the transfer finishes. A queue-send failure answers
//!   `EXECUTION_ERROR` immediately, a filename overflow `VALIDATION_ERROR`;
//! - [`COMMAND_FAILURES_DISABLED`] defaults to `true`, so open failures,
//!   zero-size files, a start offset past EOF and send-data failures all
//!   report `Fw::CmdResponse::OK` while still emitting their warning event.
//!   A cancel also finishes with `STATUS_OK`;
//! - `send_data_packet` marks `last_completed_type = T_DATA` BEFORE the read
//!   and send, so a read failure on the final chunk leaves the state marked
//!   "data complete"; the failure path calls `enter_cooldown` (resetting it
//!   to `T_NONE`) and returns WITHOUT entering `WAIT`;
//! - `send_cancel_packet` takes a SECOND buffer id, so the file-packet
//!   buffer sent just before becomes stale and its return is ignored;
//! - a return arriving in `DOWNLINK` mode with a matching context is a hard
//!   assert (after the stale/IDLE filter);
//! - the `Warnings` telemetry counter is bumped only by the four `Warnings::`
//!   helpers (file open, file read, zero size, source out of sandbox) — the
//!   partial/send-data/overflow events do NOT bump it.
//!
//! Buffer bookkeeping: C++ owns two static 512-byte arenas and wraps
//! `Fw::Buffer` views around them, identifying the in-flight buffer by its
//! context word (`context + 1 == m_lastBufferId`). Rust `Buffer`s own their
//! storage and move through the ports, so the arenas become a two-slot
//! storage pool that recycles whatever comes back on `bufferReturn`,
//! including the buffers whose *return* is otherwise ignored. The context
//! bookkeeping — one id per `get_buffer`, a second one for a CANCEL packet —
//! is reproduced exactly, because it is what rejects stale returns.

use fprime_comp::escrow::BufferEscrow;
use fprime_comp::{
    ActiveBase, ActiveComponent, BufferSendPort, CmdGlue, CmdPort, ComponentDispatch, EventGlue,
    MsgDispatchStatus, OutputPort, PingPort, PortRef, QueueFullPolicy, SchedPort, TlmGlue, msg,
};
use fprime_config::{
    FW_CMD_ARG_BUFFER_MAX_SIZE, FW_COM_BUFFER_MAX_SIZE, FwChanIdType, FwEnumStoreType,
    FwEventIdType, FwIdType, FwIndexType, FwOpcodeType, FwPacketDescriptorType,
    FwQueuePriorityType, FwSizeType,
};
use fprime_fw::file_packet::{
    CancelPacket, DATA_PACKET_HEADER_SIZE, DataPacket, EndPacket, FilePacket, FilePacketType,
    StartPacket,
};
use fprime_fw::{
    Buffer, BufferStorage, CmdArgBuffer, CmdResponse, CmdStringArg, ComPacketType, Endianness,
    FileNameString, FwString, LinearBuffer, LogSeverity, LogStringArg, SerBuf, SerBufAny,
    SerializeStatus, fpp_enum, fpp_struct, fw_assert, fw_try,
};
use fprime_os::file::{Mode as FileMode, SeekType, Status as FileStatus, WaitType};
use fprime_utils::cfdp::Checksum;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use crate::file_uplink::SandboxedFile;

// ---------------------------------------------------------------------------
// Config constants (C++ default/config/FileDownlinkCfg.hpp + FppConstants)
// ---------------------------------------------------------------------------

/// `FILEDOWNLINK_INTERNAL_BUFFER_SIZE` = `FW_FILE_BUFFER_MAX_SIZE` =
/// `FW_COM_BUFFER_MAX_SIZE` = 512.
pub const INTERNAL_BUFFER_SIZE: usize = FW_COM_BUFFER_MAX_SIZE;

/// Maximum file bytes per DATA packet: 512 - 11 (`DataPacket::HEADERSIZE`)
/// - 2 (`FwPacketDescriptorType`) = 499.
pub const MAX_DATA_SIZE: usize =
    INTERNAL_BUFFER_SIZE - DATA_PACKET_HEADER_SIZE - size_of::<FwPacketDescriptorType>();

/// `FileDownCompletePorts` — the `FileComplete` output-port array size.
pub const FILE_DOWN_COMPLETE_PORTS: usize = 1;

/// `FILEDOWNLINK_COMMAND_FAILURES_DISABLED` — TRUE by default, so failures
/// report success on the command/port response while still warning.
pub const COMMAND_FAILURES_DISABLED: bool = true;

/// The two static packet arenas (`FILE_PACKET`, `CANCEL_PACKET`).
const PACKET_ARENA_COUNT: usize = 2;

// ---------------------------------------------------------------------------
// Svc.FileDownlinkPorts types
// ---------------------------------------------------------------------------

fpp_enum! {
    /// `Svc.SendFileStatus` (`enum SendFileStatus : U8`).
    pub enum SendFileStatus : u8 {
        /// The request completed (or was canceled) successfully.
        StatusOk = 0,
        /// The request failed.
        StatusError = 1,
        /// The request was invalid (zero-size file, offset past EOF).
        StatusInvalid = 2,
        /// Declared by FPP but produced by no code path.
        StatusBusy = 3,
    }
    default StatusOk
}

fpp_struct! {
    /// `Svc.SendFileResponse` — 5 bytes on the wire: `[u8 status][u32 context]`.
    #[derive(Clone, Copy, Eq)]
    pub struct SendFileResponse {
        /// The outcome of the request.
        status: SendFileStatus { get_status, set_status },
        /// The caller's context id (`U32::MAX` for command-sourced entries).
        context: u32 { get_context, set_context },
    }
}

/// `string size 100` — the `SendFileRequest` port's filename arguments.
pub type SendFileNameArg = FwString<100>;

/// `Svc.SendFileRequest` — the GUARDED request port; a `length` of 0 means
/// "to the end of the file".
pub trait SendFileRequestPort: Send + Sync {
    /// Request a downlink; the response is immediate (accepted/rejected),
    /// completion is reported later on `SendFileComplete`.
    fn invoke(
        &self,
        port_num: FwIndexType,
        source_file_name: &SendFileNameArg,
        dest_file_name: &SendFileNameArg,
        offset: u32,
        length: u32,
    ) -> SendFileResponse;
}

/// `Svc.SendFileComplete` — final outcome of a port-sourced request.
pub trait SendFileCompletePort: Send + Sync {
    /// Report the completed request.
    fn invoke(&self, port_num: FwIndexType, resp: SendFileResponse);
}

// ---------------------------------------------------------------------------
// Dictionary constants
// ---------------------------------------------------------------------------

const MSG_TYPE_RUN: FwEnumStoreType = 1;
const MSG_TYPE_BUFFER_RETURN: FwEnumStoreType = 2;
const MSG_TYPE_PING_IN: FwEnumStoreType = 3;
const MSG_TYPE_CMD_IN: FwEnumStoreType = 4;

/// Queue message size = the command envelope, the largest async invocation:
/// 6 (envelope) + 4 (opcode) + 4 (cmdSeq) + 2 + 506 (nested `CmdArgBuffer`)
/// = 522.
pub const QUEUE_MSG_SIZE: FwSizeType =
    (msg::ENVELOPE_HEADER_SIZE + 4 + 4 + 2 + FW_CMD_ARG_BUFFER_MAX_SIZE) as FwSizeType;

const PORT_PRIORITY: FwQueuePriorityType = 1;

/// Event string arguments are `string size 100`.
const EVENT_STRING_SIZE: usize = 100;

type MsgBuffer = LinearBuffer<{ QUEUE_MSG_SIZE as usize }>;

/// `FileDownlink::Mode`.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Nothing in flight; a `Run` tick may pop a queued request.
    Idle = 0,
    /// A packet is being produced.
    Downlink = 1,
    /// A cancel was requested; the next packet is a CANCEL packet.
    Cancel = 2,
    /// Waiting for the in-flight buffer to come back.
    Wait = 3,
    /// Post-transfer cooldown before returning to IDLE.
    Cooldown = 4,
}

/// Where a queued request came from (`FileDownlink::CallerSource`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CallerSource {
    /// A `SendFile`/`SendPartial` command; the response is a `cmdResponse`.
    Command = 0,
    /// The guarded `SendFile` port; the response is a `FileComplete` invoke.
    Port = 1,
}

/// One queued downlink request (`FileDownlink::FileEntry`). C++ memcpy's the
/// POD into an `Os::Queue`; here it is a typed element in a bounded queue
/// (the queue is component-internal, so no wire format is at stake).
#[derive(Debug, Clone)]
pub struct FileEntry {
    /// Source (on-board) file name.
    pub src_filename: FileNameString,
    /// Destination (ground) file name.
    pub dest_filename: FileNameString,
    /// Start offset (0 for `SendFile`).
    pub offset: u32,
    /// Byte count (0 = to the end of the file).
    pub length: u32,
    /// Request origin.
    pub source: CallerSource,
    /// Opcode to respond to (command-sourced only).
    pub op_code: FwOpcodeType,
    /// Command sequence number (command-sourced only).
    pub cmd_seq: u32,
    /// Port context id, or `U32::MAX` for command-sourced entries.
    pub context: u32,
}

impl Default for FileEntry {
    fn default() -> Self {
        Self {
            src_filename: FileNameString::new(),
            dest_filename: FileNameString::new(),
            offset: 0,
            length: 0,
            source: CallerSource::Command,
            op_code: 0,
            cmd_seq: 0,
            context: 0,
        }
    }
}

/// `FileDownlink::File` — the file currently being sent.
struct DownlinkFile {
    source_name: LogStringArg,
    dest_name: LogStringArg,
    size: u32,
    os_file: SandboxedFile,
    checksum: Checksum,
}

impl DownlinkFile {
    fn new() -> Self {
        Self {
            source_name: LogStringArg::new(),
            dest_name: LogStringArg::new(),
            size: 0,
            os_file: SandboxedFile::new(),
            checksum: Checksum::new(),
        }
    }

    /// C++ `FileDownlink::File::open`: names and checksum are set first, then
    /// the sandboxed open (which is where path validation happens), then the
    /// size query — a size that does not round-trip through `U32` is
    /// `BAD_SIZE`, capping downlink at 4 GiB.
    fn open(&mut self, source: &FileNameString, dest: &FileNameString) -> FileStatus {
        self.source_name.set_bytes(source.as_bytes());
        self.dest_name.set_bytes(dest.as_bytes());
        self.checksum = Checksum::new();
        let path = match source.as_str() {
            Some(p) => p,
            // Rust file APIs take &str; a non-UTF-8 path cannot be opened.
            None => return FileStatus::OtherError,
        };
        let status = self.os_file.open(path, FileMode::OpenRead);
        if status != FileStatus::OpOk {
            return status;
        }
        let mut file_size: FwSizeType = 0;
        let status = self.os_file.size(&mut file_size);
        if status != FileStatus::OpOk {
            self.os_file.close();
            return FileStatus::BadSize;
        }
        if file_size > FwSizeType::from(u32::MAX) {
            self.os_file.close();
            return FileStatus::BadSize;
        }
        self.size = file_size as u32;
        FileStatus::OpOk
    }

    /// C++ `FileDownlink::File::read`: absolute seek, exact-size read (a
    /// short read is `BAD_SIZE`), then the checksum update.
    fn read(&mut self, data: &mut [u8], byte_offset: u32) -> FileStatus {
        let status = self
            .os_file
            .seek(i64::from(byte_offset), SeekType::Absolute);
        if status != FileStatus::OpOk {
            return status;
        }
        let requested = data.len();
        let mut read: FwSizeType = 0;
        let status = self.os_file.read(data, &mut read, WaitType::Wait);
        if status != FileStatus::OpOk {
            return status;
        }
        if read as usize != requested {
            return FileStatus::BadSize;
        }
        self.checksum.update(data, byte_offset);
        FileStatus::OpOk
    }
}

/// Mutable component state (everything except [`Mode`], which has its own
/// mutex exactly as the C++ `m_mode` does).
struct DownlinkState {
    configured: bool,
    cooldown: u32,
    cycle_time: u32,
    file_queue: VecDeque<FileEntry>,
    file_queue_depth: usize,
    cntx_id: u32,
    cur_entry: FileEntry,
    file: DownlinkFile,
    sequence_index: u32,
    cur_timer: u32,
    byte_offset: u32,
    end_offset: u32,
    last_completed_type: FilePacketType,
    last_buffer_id: u32,
    cur_context: u32,
    /// The two packet arenas, as recycled owned storage.
    free_storage: Vec<BufferStorage>,
    files_sent: u32,
    packets_sent: u32,
    warnings: u32,
}

impl DownlinkState {
    fn new() -> Self {
        let mut free_storage = Vec::with_capacity(PACKET_ARENA_COUNT);
        for _ in 0..PACKET_ARENA_COUNT {
            free_storage.push(new_storage());
        }
        Self {
            configured: false,
            cooldown: 0,
            cycle_time: 0,
            file_queue: VecDeque::new(),
            file_queue_depth: 0,
            cntx_id: 0,
            cur_entry: FileEntry::default(),
            file: DownlinkFile::new(),
            sequence_index: 0,
            cur_timer: 0,
            byte_offset: 0,
            end_offset: 0,
            last_completed_type: FilePacketType::None,
            last_buffer_id: 0,
            cur_context: 0,
            free_storage,
            files_sent: 0,
            packets_sent: 0,
            warnings: 0,
        }
    }

    fn pop_storage(&mut self) -> BufferStorage {
        self.free_storage.pop().unwrap_or_else(new_storage)
    }

    fn push_storage(&mut self, storage: BufferStorage) {
        if storage.len() == INTERNAL_BUFFER_SIZE && self.free_storage.len() < PACKET_ARENA_COUNT {
            self.free_storage.push(storage);
        }
    }
}

fn new_storage() -> BufferStorage {
    vec![0u8; INTERNAL_BUFFER_SIZE].into_boxed_slice()
}

/// `Svc::FileDownlink` — the active file-send component.
pub struct FileDownlink {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Command registration/response ports.
    pub cmd: CmdGlue,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// `bufferSendOut` — carries each file packet out.
    pub buffer_send_out: OutputPort<dyn BufferSendPort>,
    /// `FileComplete` — final response for port-sourced requests.
    pub file_complete: [OutputPort<dyn SendFileCompletePort>; FILE_DOWN_COMPLETE_PORTS],
    /// `pingOut` — echoes the ping key.
    pub ping_out: OutputPort<dyn PingPort>,
    /// Escrow for the async buffer-carrying `bufferReturn` port.
    escrow: BufferEscrow,
    /// C++ `m_mode` with its own mutex (the guarded port and the component
    /// thread must see the same serialization).
    mode: Mutex<Mode>,
    state: Mutex<DownlinkState>,
}

impl FileDownlink {
    // -- Commands ----------------------------------------------------------

    /// `SendFile(sourceFileName, destFileName)` — opcode 0x00.
    pub const OPCODE_SEND_FILE: FwOpcodeType = 0x00;
    /// `Cancel` — opcode 0x01.
    pub const OPCODE_CANCEL: FwOpcodeType = 0x01;
    /// `SendPartial(sourceFileName, destFileName, startOffset, length)` —
    /// opcode 0x02.
    pub const OPCODE_SEND_PARTIAL: FwOpcodeType = 0x02;

    // -- Events ------------------------------------------------------------

    /// `FileOpenError(fileName)` — WARNING_HI, id 0x00.
    pub const EVENTID_FILE_OPEN_ERROR: FwEventIdType = 0x00;
    /// `FileReadError(fileName, status: I32)` — WARNING_HI, id 0x01.
    pub const EVENTID_FILE_READ_ERROR: FwEventIdType = 0x01;
    /// `FileSent(sourceFileName, destFileName)` — ACTIVITY_HI, id 0x02.
    pub const EVENTID_FILE_SENT: FwEventIdType = 0x02;
    /// `DownlinkCanceled(sourceFileName, destFileName)` — ACTIVITY_HI,
    /// id 0x03. (0x04 is an FPP gap.)
    pub const EVENTID_DOWNLINK_CANCELED: FwEventIdType = 0x03;
    /// `DownlinkPartialWarning(startOffset, length, filesize, source, dest)`
    /// — WARNING_LO, id 0x05.
    pub const EVENTID_DOWNLINK_PARTIAL_WARNING: FwEventIdType = 0x05;
    /// `DownlinkPartialFail(source, dest, startOffset, filesize)` —
    /// WARNING_HI, id 0x06.
    pub const EVENTID_DOWNLINK_PARTIAL_FAIL: FwEventIdType = 0x06;
    /// `SendDataFail(sourceFileName, byteOffset)` — WARNING_HI, id 0x07.
    pub const EVENTID_SEND_DATA_FAIL: FwEventIdType = 0x07;
    /// `SendStarted(fileSize, sourceFileName, destFileName)` — ACTIVITY_HI,
    /// id 0x08.
    pub const EVENTID_SEND_STARTED: FwEventIdType = 0x08;
    /// `DownlinkZeroSizeFile(sourceFileName)` — WARNING_HI, id 0x09.
    pub const EVENTID_DOWNLINK_ZERO_SIZE_FILE: FwEventIdType = 0x09;
    /// `FilenameSourceOverflow` — WARNING_HI, id 0x10.
    pub const EVENTID_FILENAME_SOURCE_OVERFLOW: FwEventIdType = 0x10;
    /// `FilenameDestinationOverflow` — WARNING_HI, id 0x11.
    pub const EVENTID_FILENAME_DESTINATION_OVERFLOW: FwEventIdType = 0x11;
    /// `SourceOutOfSandbox(fileName)` — WARNING_HI, id 0x12.
    pub const EVENTID_SOURCE_OUT_OF_SANDBOX: FwEventIdType = 0x12;

    // -- Telemetry ---------------------------------------------------------

    /// `FilesSent: U32` — id 0x00.
    pub const CHANID_FILES_SENT: FwChanIdType = 0x00;
    /// `PacketsSent: U32` — id 0x01.
    pub const CHANID_PACKETS_SENT: FwChanIdType = 0x01;
    /// `Warnings: U32` — id 0x02.
    pub const CHANID_WARNINGS: FwChanIdType = 0x02;

    /// Construct the component (topology step 1).
    #[must_use]
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            buffer_send_out: OutputPort::new(),
            file_complete: std::array::from_fn(|_| OutputPort::new()),
            ping_out: OutputPort::new(),
            escrow: BufferEscrow::new(),
            mode: Mutex::new(Mode::Idle),
            state: Mutex::new(DownlinkState::new()),
        })
    }

    /// C++ `configure(cooldown, cycleTime, fileQueueDepth)` plus the message
    /// queue creation. Both timers are in the same (caller-chosen) units;
    /// the stock subtopology uses 1000 ms for each.
    pub fn configure(&self, cooldown: u32, cycle_time: u32, file_queue_depth: usize) {
        let mut state = self.state.lock().unwrap();
        state.cooldown = cooldown;
        state.cycle_time = cycle_time;
        state.file_queue_depth = file_queue_depth;
        state.file_queue = VecDeque::with_capacity(file_queue_depth);
        state.configured = true;
    }

    /// C++ `configure(directory)`: restrict the source sandbox. Optional —
    /// the default sandbox is `/` (fail open).
    pub fn configure_sandbox(&self, directory: &str) {
        self.state.lock().unwrap().file.os_file.configure(directory);
    }

    /// Create the message queue.
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active.queued.create_queue(queue_depth, QUEUE_MSG_SIZE);
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_SEND_FILE,
                Self::OPCODE_CANCEL,
                Self::OPCODE_SEND_PARTIAL,
            ],
        );
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    /// The current mode (C++ `m_mode.get()`, which locks).
    #[must_use]
    pub fn mode(&self) -> Mode {
        *self.mode.lock().unwrap()
    }

    fn set_mode(&self, mode: Mode) {
        *self.mode.lock().unwrap() = mode;
    }

    // -- Input-port factories ---------------------------------------------

    /// `Run` — ASYNC `Svc.Sched` input.
    pub fn run_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(Arc::new(RunInAdapter { comp: self.clone() }), port_num)
    }

    /// `bufferReturn` — ASYNC `Fw.BufferSend` input (the flow-control
    /// credit).
    pub fn buffer_return_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn BufferSendPort> {
        PortRef::new(
            Arc::new(BufferReturnAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `pingIn` — ASYNC `Svc.Ping` input.
    pub fn ping_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn PingPort> {
        PortRef::new(Arc::new(PingInAdapter { comp: self.clone() }), port_num)
    }

    /// `cmdIn` — ASYNC `Fw.Cmd` input.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }

    /// `SendFile` — GUARDED `Svc.SendFileRequest` input: the component
    /// implements the port trait and the handler runs on the CALLER's thread.
    pub fn send_file_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn SendFileRequestPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Handlers ----------------------------------------------------------

    /// `pingIn_handler`: echo the key.
    fn ping_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }

    /// `Run_handler`: pop a request in IDLE, tick the cooldown, accumulate
    /// the (never-read) WAIT timer.
    fn run_handler(&self, _port_num: FwIndexType, _context: u32) {
        match self.mode() {
            Mode::Idle => {
                let entry = {
                    let mut state = self.state.lock().unwrap();
                    match state.file_queue.pop_front() {
                        Some(entry) => {
                            state.cur_entry = entry.clone();
                            Some(entry)
                        }
                        None => None,
                    }
                };
                if let Some(entry) = entry {
                    let mut state = self.state.lock().unwrap();
                    self.send_file(
                        &mut state,
                        &entry.src_filename,
                        &entry.dest_filename,
                        entry.offset,
                        entry.length,
                    );
                }
            }
            Mode::Cooldown => {
                let mut state = self.state.lock().unwrap();
                if state.cur_timer >= state.cooldown {
                    state.cur_timer = 0;
                    self.set_mode(Mode::Idle);
                } else {
                    state.cur_timer += state.cycle_time;
                }
            }
            Mode::Wait => {
                // No timeout exists: m_curTimer accumulates and is never read.
                let mut state = self.state.lock().unwrap();
                state.cur_timer += state.cycle_time;
            }
            Mode::Downlink | Mode::Cancel => {}
        }
    }

    /// `bufferReturn_handler`: the one-packet credit coming back.
    fn buffer_return_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        let context = buffer.context();
        let mut state = self.state.lock().unwrap();
        // Rust owns the storage, so recycle it even when the RETURN itself
        // is ignored (C++ points at static memory and can just drop it).
        state.push_storage(buffer.into_storage());

        let mode = self.mode();
        if state.last_buffer_id != context.wrapping_add(1) || mode == Mode::Idle {
            return; // stale (old, timed out, or both)
        }
        fw_assert!(mode == Mode::Wait || mode == Mode::Cancel, mode as i32);
        if state.last_completed_type == FilePacketType::End
            || state.last_completed_type == FilePacketType::Cancel
        {
            let cancel = state.last_completed_type == FilePacketType::Cancel;
            self.finish_helper(&mut state, cancel);
            return;
        } else if mode == Mode::Wait {
            self.set_mode(Mode::Downlink);
        }
        self.downlink_packet(&mut state);
    }

    /// C++ `sendFile`: open, validate, then START + WAIT.
    fn send_file(
        &self,
        state: &mut DownlinkState,
        source: &FileNameString,
        dest: &FileNameString,
        start_offset: u32,
        length: u32,
    ) {
        let status = state.file.open(source, dest);
        if status != FileStatus::OpOk {
            self.set_mode(Mode::Idle);
            if status == FileStatus::OutsideSandbox {
                self.warning_source_out_of_sandbox(state);
            } else {
                self.warning_file_open_error(state);
            }
            self.send_response(state, failure_status(SendFileStatus::StatusError));
            return;
        }
        let file_size = state.file.size;

        if file_size == 0 {
            state.file.os_file.close();
            self.set_mode(Mode::Idle);
            self.warning_zero_size(state);
            self.send_response(state, failure_status(SendFileStatus::StatusInvalid));
            return;
        }

        let mut length = length;
        if start_offset >= file_size {
            self.enter_cooldown(state);
            self.log_downlink_partial_fail(state, start_offset, file_size);
            self.send_response(state, failure_status(SendFileStatus::StatusInvalid));
            return;
        } else if length > file_size - start_offset {
            self.log_downlink_partial_warning(state, start_offset, length, file_size);
            length = file_size - start_offset;
        }

        self.get_buffer(state);
        self.send_start_packet(state);
        self.set_mode(Mode::Wait);
        state.sequence_index = 1;
        state.cur_timer = 0;
        state.byte_offset = start_offset;
        state.last_completed_type = FilePacketType::Start;

        // A zero length means "read to the end of the file".
        if length > 0 {
            self.log_send_started(state, length);
            state.end_offset = start_offset + length;
        } else {
            self.log_send_started(state, file_size - start_offset);
            state.end_offset = file_size;
        }
    }

    /// C++ `downlinkPacket`.
    fn downlink_packet(&self, state: &mut DownlinkState) {
        fw_assert!(
            state.last_completed_type != FilePacketType::None,
            state.last_completed_type.as_repr() as i32
        );
        let mode = self.mode();
        fw_assert!(mode == Mode::Cancel || mode == Mode::Downlink, mode as i32);
        if mode == Mode::Cancel && state.last_completed_type == FilePacketType::Start {
            self.send_cancel_packet(state);
            state.last_completed_type = FilePacketType::Cancel;
        } else if mode == Mode::Downlink && state.last_completed_type == FilePacketType::Start {
            let status = self.send_data_packet(state);
            if status != FileStatus::OpOk {
                self.log_send_data_fail(state);
                self.enter_cooldown(state);
                self.send_response(state, failure_status(SendFileStatus::StatusError));
                return; // do NOT go to WAIT
            }
        } else if state.last_completed_type == FilePacketType::Data {
            self.send_end_packet(state);
            state.last_completed_type = FilePacketType::End;
        }
        self.set_mode(Mode::Wait);
        state.cur_timer = 0;
    }

    /// C++ `finishHelper`: a cancel still reports `STATUS_OK`.
    fn finish_helper(&self, state: &mut DownlinkState, cancel: bool) {
        if !cancel {
            state.files_sent += 1;
            let count = state.files_sent;
            self.tlm_write(Self::CHANID_FILES_SENT, count);
            self.log_two_names(
                Self::EVENTID_FILE_SENT,
                LogSeverity::ActivityHi,
                state,
                "Sent file",
            );
        } else {
            self.log_two_names(
                Self::EVENTID_DOWNLINK_CANCELED,
                LogSeverity::ActivityHi,
                state,
                "Canceled downlink of file",
            );
        }
        self.enter_cooldown(state);
        self.send_response(state, SendFileStatus::StatusOk);
    }

    fn enter_cooldown(&self, state: &mut DownlinkState) {
        state.file.os_file.close();
        self.set_mode(Mode::Cooldown);
        state.last_completed_type = FilePacketType::None;
        state.cur_timer = 0;
    }

    /// C++ `getBuffer(m_buffer, FILE_PACKET)`: assign the current buffer id
    /// and bump the counter. The storage itself is claimed at send time.
    fn get_buffer(&self, state: &mut DownlinkState) {
        state.cur_context = state.last_buffer_id;
        state.last_buffer_id = state.last_buffer_id.wrapping_add(1);
    }

    /// C++ `sendStartPacket`: `StartPacket::initialize` forces sequence 0.
    fn send_start_packet(&self, state: &mut DownlinkState) {
        let source = state.file.source_name.clone();
        let dest = state.file.dest_name.clone();
        let packet = FilePacket::Start(StartPacket::initialize(
            state.file.size,
            source.as_bytes(),
            dest.as_bytes(),
        ));
        let context = state.cur_context;
        self.send_file_packet(state, &packet, context);
    }

    /// C++ `sendEndPacket`: reuses the current buffer (no new id).
    fn send_end_packet(&self, state: &mut DownlinkState) {
        let packet = FilePacket::End(EndPacket::initialize(
            state.sequence_index,
            state.file.checksum.get_value(),
        ));
        let context = state.cur_context;
        self.send_file_packet(state, &packet, context);
    }

    /// C++ `sendCancelPacket`: takes a SECOND buffer id from the CANCEL
    /// arena, which makes the file-packet buffer sent just before stale.
    fn send_cancel_packet(&self, state: &mut DownlinkState) {
        let packet = FilePacket::Cancel(CancelPacket::initialize(state.sequence_index));
        let context = state.last_buffer_id;
        state.last_buffer_id = state.last_buffer_id.wrapping_add(1);
        self.send_file_packet(state, &packet, context);
    }

    /// C++ `sendDataPacket`.
    fn send_data_packet(&self, state: &mut DownlinkState) -> FileStatus {
        // The caller maintains byte_offset < end_offset.
        if state.byte_offset >= state.end_offset {
            return FileStatus::InvalidArgument;
        }
        let remaining = state.end_offset - state.byte_offset;
        let data_size = core::cmp::min(remaining as usize, MAX_DATA_SIZE);
        let byte_offset = state.byte_offset;
        // GOTCHA: the "last data packet" mark is set BEFORE the read/send.
        if data_size as u32 + byte_offset == state.end_offset {
            state.last_completed_type = FilePacketType::Data;
        }

        let mut data = [0u8; MAX_DATA_SIZE];
        let status = state.file.read(&mut data[..data_size], byte_offset);
        if status != FileStatus::OpOk {
            self.warning_file_read(state, status);
            return status;
        }

        let packet = FilePacket::Data(DataPacket::initialize(
            state.sequence_index,
            byte_offset,
            data_size as u16,
            &data[..data_size],
        ));
        state.sequence_index += 1;
        let context = state.cur_context;
        self.send_file_packet(state, &packet, context);
        state.byte_offset += data_size as u32;
        FileStatus::OpOk
    }

    /// C++ `sendFilePacket`: `[u16 FW_PACKET_FILE][file packet]`, buffer
    /// size = `packet.bufferSize() + 2`.
    fn send_file_packet(&self, state: &mut DownlinkState, packet: &FilePacket<'_>, context: u32) {
        let buffer_size = packet.buffer_size() + size_of::<FwPacketDescriptorType>();
        let storage = state.pop_storage();
        fw_assert!(storage.len() >= buffer_size, storage.len() as i32);
        let mut buffer = Buffer::from_storage(storage, context);
        {
            let mut ser = buffer.get_serializer();
            let status = ser.serialize_u16_be(ComPacketType::FwPacketFile.as_repr());
            fw_assert!(status.is_ok(), status as i32);
            let status = packet.serialize_to(&mut ser);
            fw_assert!(status.is_ok(), status as i32);
        }
        buffer.set_size(buffer_size);
        let p = self.buffer_send_out.get();
        p.target.invoke(p.port_num, buffer);
        state.packets_sent += 1;
        let count = state.packets_sent;
        self.tlm_write(Self::CHANID_PACKETS_SENT, count);
    }

    /// C++ `sendResponse`.
    fn send_response(&self, state: &mut DownlinkState, status: SendFileStatus) {
        match state.cur_entry.source {
            CallerSource::Command => {
                self.cmd.cmd_response(
                    state.cur_entry.op_code,
                    state.cur_entry.cmd_seq,
                    status_to_cmd_resp(status),
                );
            }
            CallerSource::Port => {
                let resp = SendFileResponse::new(status, state.cur_entry.context);
                for port in &self.file_complete {
                    if let Some(p) = port.try_get() {
                        p.target.invoke(p.port_num, resp);
                    }
                }
            }
        }
    }

    /// Enqueue a request; false when the bounded queue is full (the C++
    /// non-blocking `Os::Queue::send` failure).
    fn enqueue(&self, state: &mut DownlinkState, entry: FileEntry) -> bool {
        if state.file_queue.len() >= state.file_queue_depth {
            return false;
        }
        state.file_queue.push_back(entry);
        true
    }

    // -- Command handlers (component thread) -------------------------------

    fn send_file_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut source = CmdStringArg::new();
        let mut dest = CmdStringArg::new();
        if !args.deserialize(&mut source, Endianness::Big).is_ok()
            || !args.deserialize(&mut dest, Endianness::Big).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        self.queue_command_request(op_code, cmd_seq, &source, &dest, 0, 0);
    }

    fn send_partial_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut source = CmdStringArg::new();
        let mut dest = CmdStringArg::new();
        let mut start_offset = 0u32;
        let mut length = 0u32;
        if !args.deserialize(&mut source, Endianness::Big).is_ok()
            || !args.deserialize(&mut dest, Endianness::Big).is_ok()
            || !args.deserialize_u32_be(&mut start_offset).is_ok()
            || !args.deserialize_u32_be(&mut length).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        self.queue_command_request(op_code, cmd_seq, &source, &dest, start_offset, length);
    }

    /// The shared tail of `SendFile_cmdHandler` / `SendPartial_cmdHandler`:
    /// NO response on a successful enqueue (it is deferred to completion).
    fn queue_command_request(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        source: &CmdStringArg,
        dest: &CmdStringArg,
        offset: u32,
        length: u32,
    ) {
        // C++ guards against a filename longer than Fw::FileNameString's
        // capacity. A CmdStringArg holds at most FW_CMD_STRING_MAX_SIZE
        // bytes, so this is unreachable in the stock configuration — kept
        // for parity with the C++ control flow.
        if source.len() >= FileNameString::max_length() {
            self.log_filename_source_overflow();
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        if dest.len() >= FileNameString::max_length() {
            self.log_filename_destination_overflow();
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        let mut entry = FileEntry {
            offset,
            length,
            source: CallerSource::Command,
            op_code,
            cmd_seq,
            context: u32::MAX,
            ..FileEntry::default()
        };
        entry.src_filename.set_bytes(source.as_bytes());
        entry.dest_filename.set_bytes(dest.as_bytes());
        let queued = {
            let mut state = self.state.lock().unwrap();
            self.enqueue(&mut state, entry)
        };
        if !queued {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
        }
        // On success the response is deferred until the downlink finishes.
    }

    fn cancel_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let mode = self.mode();
        if mode == Mode::Downlink || mode == Mode::Wait {
            self.set_mode(Mode::Cancel);
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `SendFile_handler` — the GUARDED port, on the CALLER's thread.
    fn send_file_port_handler(
        &self,
        _port_num: FwIndexType,
        source_file_name: &SendFileNameArg,
        dest_file_name: &SendFileNameArg,
        offset: u32,
        length: u32,
    ) -> SendFileResponse {
        let context = {
            let mut state = self.state.lock().unwrap();
            let context = state.cntx_id;
            state.cntx_id = state.cntx_id.wrapping_add(1);
            context
        };
        // Unreachable with the stock 100-byte port strings; parity only.
        if source_file_name.len() >= FileNameString::max_length() {
            self.log_filename_source_overflow();
            return SendFileResponse::new(SendFileStatus::StatusError, u32::MAX);
        }
        if dest_file_name.len() >= FileNameString::max_length() {
            self.log_filename_destination_overflow();
            return SendFileResponse::new(SendFileStatus::StatusError, u32::MAX);
        }
        let mut entry = FileEntry {
            offset,
            length,
            source: CallerSource::Port,
            op_code: 0,
            cmd_seq: 0,
            context,
            ..FileEntry::default()
        };
        entry.src_filename.set_bytes(source_file_name.as_bytes());
        entry.dest_filename.set_bytes(dest_file_name.as_bytes());
        let queued = {
            let mut state = self.state.lock().unwrap();
            self.enqueue(&mut state, entry)
        };
        if !queued {
            // A full queue is STATUS_ERROR (never STATUS_BUSY).
            return SendFileResponse::new(SendFileStatus::StatusError, u32::MAX);
        }
        SendFileResponse::new(SendFileStatus::StatusOk, context)
    }

    // -- Events / telemetry -------------------------------------------------

    fn tlm_write(&self, chan: FwChanIdType, value: u32) {
        self.tlm
            .tlm_write(self.id_base(), chan, &value, self.evt.time_get());
    }

    /// C++ `Warnings::warning()` — only the four `Warnings::` helpers bump
    /// this counter.
    fn warning(&self, state: &mut DownlinkState) {
        state.warnings += 1;
        let count = state.warnings;
        self.tlm_write(Self::CHANID_WARNINGS, count);
    }

    fn log_one_name(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        name: &LogStringArg,
        text: &str,
    ) {
        self.evt.log_event(
            self.id_base(),
            id,
            severity,
            &format!("{text} {name}"),
            |buf| name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big),
        );
    }

    fn log_two_names(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        state: &DownlinkState,
        text: &str,
    ) {
        let source = state.file.source_name.clone();
        let dest = state.file.dest_name.clone();
        self.evt.log_event(
            self.id_base(),
            id,
            severity,
            &format!("{text} {source} to file {dest}"),
            |buf| {
                fw_try!(source.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                dest.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn warning_file_open_error(&self, state: &mut DownlinkState) {
        let name = state.file.source_name.clone();
        self.log_one_name(
            Self::EVENTID_FILE_OPEN_ERROR,
            LogSeverity::WarningHi,
            &name,
            "Could not open file",
        );
        self.warning(state);
    }

    fn warning_source_out_of_sandbox(&self, state: &mut DownlinkState) {
        let name = state.file.source_name.clone();
        self.log_one_name(
            Self::EVENTID_SOURCE_OUT_OF_SANDBOX,
            LogSeverity::WarningHi,
            &name,
            "Source file is outside the configured read sandbox:",
        );
        self.warning(state);
    }

    fn warning_zero_size(&self, state: &mut DownlinkState) {
        let name = state.file.source_name.clone();
        self.log_one_name(
            Self::EVENTID_DOWNLINK_ZERO_SIZE_FILE,
            LogSeverity::WarningHi,
            &name,
            "Downlink stopped due to zero-size:",
        );
        self.warning(state);
    }

    fn warning_file_read(&self, state: &mut DownlinkState, status: FileStatus) {
        let name = state.file.source_name.clone();
        let code = status as i32;
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_READ_ERROR,
            LogSeverity::WarningHi,
            &format!("Could not read file {name} with status {code}"),
            |buf| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_i32_be(code)
            },
        );
        self.warning(state);
    }

    fn log_send_started(&self, state: &DownlinkState, length: u32) {
        let source = state.file.source_name.clone();
        let dest = state.file.dest_name.clone();
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_SEND_STARTED,
            LogSeverity::ActivityHi,
            &format!("Downlink of {length} bytes started from {source} to {dest}"),
            |buf| {
                fw_try!(buf.serialize_u32_be(length));
                fw_try!(source.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                dest.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_downlink_partial_warning(
        &self,
        state: &DownlinkState,
        start_offset: u32,
        length: u32,
        file_size: u32,
    ) {
        let source = state.file.source_name.clone();
        let dest = state.file.dest_name.clone();
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_DOWNLINK_PARTIAL_WARNING,
            LogSeverity::WarningLo,
            &format!(
                "Offset {start_offset} plus length {length} is greater than source size {file_size} for partial downlink of file {source} to file {dest}. "
            ),
            |buf| {
                fw_try!(buf.serialize_u32_be(start_offset));
                fw_try!(buf.serialize_u32_be(length));
                fw_try!(buf.serialize_u32_be(file_size));
                fw_try!(source.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                dest.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_downlink_partial_fail(&self, state: &DownlinkState, start_offset: u32, file_size: u32) {
        let source = state.file.source_name.clone();
        let dest = state.file.dest_name.clone();
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_DOWNLINK_PARTIAL_FAIL,
            LogSeverity::WarningHi,
            &format!(
                "Error occurred during partial downlink of file {source} to file {dest}. Offset {start_offset} greater than or equal to source filesize {file_size}."
            ),
            |buf| {
                fw_try!(source.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(dest.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(start_offset));
                buf.serialize_u32_be(file_size)
            },
        );
    }

    fn log_send_data_fail(&self, state: &DownlinkState) {
        let source = state.file.source_name.clone();
        let byte_offset = state.byte_offset;
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_SEND_DATA_FAIL,
            LogSeverity::WarningHi,
            &format!("Failed to send data packet from file {source} at byte offset {byte_offset}."),
            |buf| {
                fw_try!(source.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u32_be(byte_offset)
            },
        );
    }

    fn log_filename_source_overflow(&self) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILENAME_SOURCE_OVERFLOW,
            LogSeverity::WarningHi,
            "Commanded source filename too long",
            |_buf| SerializeStatus::Ok,
        );
    }

    fn log_filename_destination_overflow(&self) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILENAME_DESTINATION_OVERFLOW,
            LogSeverity::WarningHi,
            "Commanded destination filename too long",
            |_buf| SerializeStatus::Ok,
        );
    }
}

/// C++ `statusToCmdResp`.
fn status_to_cmd_resp(status: SendFileStatus) -> CmdResponse {
    match status {
        SendFileStatus::StatusOk => CmdResponse::Ok,
        SendFileStatus::StatusError => CmdResponse::ExecutionError,
        SendFileStatus::StatusInvalid => CmdResponse::ValidationError,
        SendFileStatus::StatusBusy => CmdResponse::Busy,
    }
}

/// `FILEDOWNLINK_COMMAND_FAILURES_DISABLED ? STATUS_OK : failure`.
fn failure_status(failure: SendFileStatus) -> SendFileStatus {
    if COMMAND_FAILURES_DISABLED {
        SendFileStatus::StatusOk
    } else {
        failure
    }
}

// -- Guarded sync port, implemented directly on the component ---------------

impl SendFileRequestPort for FileDownlink {
    fn invoke(
        &self,
        port_num: FwIndexType,
        source_file_name: &SendFileNameArg,
        dest_file_name: &SendFileNameArg,
        offset: u32,
        length: u32,
    ) -> SendFileResponse {
        self.send_file_port_handler(port_num, source_file_name, dest_file_name, offset, length)
    }
}

// -- Async input adapters ---------------------------------------------------

struct RunInAdapter {
    comp: Arc<FileDownlink>,
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
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// `bufferReturn` adapter: the owned `Buffer` rides through the escrow.
struct BufferReturnAdapter {
    comp: Arc<FileDownlink>,
}

impl BufferSendPort for BufferReturnAdapter {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        let token = self.comp.escrow.deposit(buffer);
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_BUFFER_RETURN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u64_be(token);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

struct PingInAdapter {
    comp: Arc<FileDownlink>,
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
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

struct CmdInAdapter {
    comp: Arc<FileDownlink>,
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
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

// -- Dispatch ---------------------------------------------------------------

impl ComponentDispatch for FileDownlink {
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
            MSG_TYPE_RUN => {
                let mut context = 0u32;
                if !buf.deserialize_u32_be(&mut context).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.run_handler(port_num, context);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_BUFFER_RETURN => {
                let mut token = 0u64;
                if !buf.deserialize_u64_be(&mut token).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                let buffer = self.escrow.claim(token);
                self.buffer_return_handler(port_num, buffer);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_PING_IN => {
                let mut key = 0u32;
                if !buf.deserialize_u32_be(&mut key).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.ping_handler(port_num, key);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_CMD_IN => {
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
                    Self::OPCODE_SEND_FILE => {
                        self.send_file_cmd_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_CANCEL => self.cancel_cmd_handler(op_code, cmd_seq, &mut args),
                    Self::OPCODE_SEND_PARTIAL => {
                        self.send_partial_cmd_handler(op_code, cmd_seq, &mut args)
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

    /// C++ `preamble()` asserts the component was configured.
    fn preamble(&self) {
        fw_assert!(self.state.lock().unwrap().configured);
    }
}

impl ActiveComponent for FileDownlink {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::file_uplink::FileUplink;
    use fprime_comp::queued::MsgDispatchStatus as DispatchStatus;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, TlmPort};
    use fprime_fw::{LogBuffer, Serialize, Time, TlmBuffer};
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    const ID_BASE: FwIdType = 0x0500_1000;

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    fn temp_dir(tag: &str) -> PathBuf {
        let mut path = std::env::temp_dir();
        // Command string arguments are Fw::CmdStringArg (40 bytes), so the
        // paths used in these tests must stay short.
        let _ = tag;
        path.push(format!(
            "fpr{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&path).unwrap();
        path
    }

    fn write_file(dir: &std::path::Path, name: &str, content: &[u8]) -> String {
        let path = dir.join(name);
        std::fs::write(&path, content).unwrap();
        path.to_str().unwrap().to_string()
    }

    #[derive(Default)]
    struct Ground {
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        regs: Mutex<Vec<FwOpcodeType>>,
        sent: Mutex<Vec<Buffer>>,
        completions: Mutex<Vec<SendFileResponse>>,
        pings: Mutex<Vec<u32>>,
    }

    impl Ground {
        fn event_ids(&self) -> Vec<FwEventIdType> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .map(|e| e.0 - ID_BASE)
                .collect()
        }
        fn events_of(&self, id: FwEventIdType) -> Vec<(LogSeverity, Vec<u8>)> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .filter(|e| e.0 == ID_BASE + id)
                .map(|e| (e.1, e.2.clone()))
                .collect()
        }
        fn last_tlm(&self, chan: FwChanIdType) -> Option<u32> {
            self.tlm
                .lock()
                .unwrap()
                .iter()
                .rfind(|e| e.0 == ID_BASE + chan)
                .map(|e| u32::from_be_bytes([e.1[0], e.1[1], e.1[2], e.1[3]]))
        }
        fn take_sent(&self) -> Buffer {
            self.sent.lock().unwrap().remove(0)
        }
        fn sent_count(&self) -> usize {
            self.sent.lock().unwrap().len()
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

    impl CmdRegPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }

    impl BufferSendPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.sent.lock().unwrap().push(buffer);
        }
    }

    impl SendFileCompletePort for Ground {
        fn invoke(&self, _port_num: FwIndexType, resp: SendFileResponse) {
            self.completions.lock().unwrap().push(resp);
        }
    }

    impl PingPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.pings.lock().unwrap().push(key);
        }
    }

    fn setup(cooldown: u32, depth: usize) -> (Arc<FileDownlink>, Arc<Ground>) {
        let comp = FileDownlink::new("fileDownlink");
        let ground = Arc::new(Ground::default());
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.buffer_send_out.connect(ground.clone(), 0);
        comp.file_complete[0].connect(ground.clone(), 0);
        comp.ping_out.connect(ground.clone(), 0);
        comp.configure(cooldown, 1000, depth);
        comp.init(64);
        (comp, ground)
    }

    fn dispatch(comp: &Arc<FileDownlink>) {
        let status = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
        assert!(status == DispatchStatus::Ok || status == DispatchStatus::Empty);
    }

    fn tick(comp: &Arc<FileDownlink>) {
        let p = comp.run_in(0);
        p.target.invoke(p.port_num, 0);
        dispatch(comp);
    }

    fn return_buffer(comp: &Arc<FileDownlink>, buffer: Buffer) {
        let p = comp.buffer_return_in(0);
        p.target.invoke(p.port_num, buffer);
        dispatch(comp);
    }

    fn send_cmd(
        comp: &Arc<FileDownlink>,
        local_opcode: FwOpcodeType,
        cmd_seq: u32,
        build: impl FnOnce(&mut CmdArgBuffer),
    ) {
        let mut args = CmdArgBuffer::new();
        build(&mut args);
        let p = comp.cmd_in(0);
        p.target
            .invoke(p.port_num, ID_BASE + local_opcode, cmd_seq, &mut args);
        dispatch(comp);
    }

    fn send_file_cmd(comp: &Arc<FileDownlink>, cmd_seq: u32, source: &str, dest: &str) {
        send_cmd(comp, FileDownlink::OPCODE_SEND_FILE, cmd_seq, |args| {
            let src = CmdStringArg::from(source);
            let dst = CmdStringArg::from(dest);
            assert_eq!(src.serialize_to(args, Endianness::Big), SerializeStatus::Ok);
            assert_eq!(dst.serialize_to(args, Endianness::Big), SerializeStatus::Ok);
        });
    }

    /// Drive the whole transfer: tick to start it, then keep returning each
    /// buffer until the component is back in COOLDOWN/IDLE. Returns the raw
    /// bytes of every packet in order.
    fn run_transfer(comp: &Arc<FileDownlink>, ground: &Arc<Ground>) -> Vec<Vec<u8>> {
        let mut packets = Vec::new();
        tick(comp);
        while ground.sent_count() > 0 {
            let buffer = ground.take_sent();
            packets.push(buffer.data().to_vec());
            return_buffer(comp, buffer);
        }
        packets
    }

    // ---- packet sequence --------------------------------------------------

    #[test]
    fn downlink_produces_the_exact_packet_sequence() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("seq");
        let source = write_file(&dir, "h.bin", b"hello world");
        send_file_cmd(&comp, 7, &source, "g/h.bin");
        // No response yet: it is deferred to completion.
        assert!(ground.responses.lock().unwrap().is_empty());

        let packets = run_transfer(&comp, &ground);
        assert_eq!(packets.len(), 3);

        // START: [0003][00][seq 0][fileSize 11][srcLen src][dstLen dst]
        let mut expected = vec![0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00];
        expected.extend_from_slice(&11u32.to_be_bytes());
        expected.push(source.len() as u8);
        expected.extend_from_slice(source.as_bytes());
        expected.push(7);
        expected.extend_from_slice(b"g/h.bin");
        assert_eq!(packets[0], expected);

        // DATA: [0003][01][seq 1][offset 0][size 11]["hello world"]
        assert_eq!(
            packets[1],
            vec![
                0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, b'h',
                b'e', b'l', b'l', b'o', b' ', b'w', b'o', b'r', b'l', b'd',
            ]
        );

        // END: [0003][02][seq 2][checksum]
        let mut checksum = Checksum::new();
        checksum.update(b"hello world", 0);
        let mut end = vec![0x00, 0x03, 0x02, 0x00, 0x00, 0x00, 0x02];
        end.extend_from_slice(&checksum.get_value().to_be_bytes());
        assert_eq!(packets[2], end);

        assert_eq!(ground.last_tlm(FileDownlink::CHANID_FILES_SENT), Some(1));
        assert_eq!(ground.last_tlm(FileDownlink::CHANID_PACKETS_SENT), Some(3));
        assert_eq!(
            ground.responses.lock().unwrap().as_slice(),
            &[(ID_BASE + FileDownlink::OPCODE_SEND_FILE, 7, CmdResponse::Ok)]
        );
        assert_eq!(
            ground.event_ids(),
            vec![
                FileDownlink::EVENTID_SEND_STARTED,
                FileDownlink::EVENTID_FILE_SENT
            ]
        );
    }

    #[test]
    fn a_1000_byte_file_chunks_as_499_499_2() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("chunk");
        let content: Vec<u8> = (0u8..=255).cycle().take(1000).collect();
        let source = write_file(&dir, "big.bin", &content);
        send_file_cmd(&comp, 1, &source, "g.bin");
        let packets = run_transfer(&comp, &ground);
        // START + 3 DATA + END
        assert_eq!(packets.len(), 5);
        let sizes: Vec<usize> = packets[1..4]
            .iter()
            .map(|p| u16::from_be_bytes([p[11], p[12]]) as usize)
            .collect();
        assert_eq!(sizes, vec![MAX_DATA_SIZE, MAX_DATA_SIZE, 2]);
        // Sequence indices: START 0, DATA 1..3, END 4.
        let seqs: Vec<u32> = packets
            .iter()
            .map(|p| u32::from_be_bytes([p[3], p[4], p[5], p[6]]))
            .collect();
        assert_eq!(seqs, vec![0, 1, 2, 3, 4]);
        assert_eq!(ground.last_tlm(FileDownlink::CHANID_PACKETS_SENT), Some(5));
    }

    #[test]
    fn one_packet_is_in_flight_at_a_time() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("credit");
        let source = write_file(&dir, "f.bin", &vec![0xABu8; 1200]);
        send_file_cmd(&comp, 1, &source, "g.bin");
        tick(&comp);
        assert_eq!(ground.sent_count(), 1);
        assert_eq!(comp.mode(), Mode::Wait);
        // Extra ticks in WAIT produce nothing (there is NO timeout).
        for _ in 0..50 {
            tick(&comp);
        }
        assert_eq!(ground.sent_count(), 1);
        assert_eq!(comp.mode(), Mode::Wait);
        // The credit returns and exactly one more packet goes out.
        let buffer = ground.take_sent();
        return_buffer(&comp, buffer);
        assert_eq!(ground.sent_count(), 1);
    }

    // ---- end-to-end round trip -------------------------------------------

    #[test]
    fn downlinked_packets_uplink_back_to_a_byte_identical_file() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("roundtrip");
        let content: Vec<u8> = (0u8..=255).cycle().take(2345).collect();
        let source = write_file(&dir, "s.bin", &content);
        let dest = dir.join("u.bin");
        let dest_str = dest.to_str().unwrap().to_string();

        send_file_cmd(&comp, 3, &source, &dest_str);
        let packets = run_transfer(&comp, &ground);
        // START + 5 DATA (499*4 + 349) + END
        assert_eq!(packets.len(), 7);

        // Feed the very same bytes through FileUplink.
        let uplink = FileUplink::new("fileUplink");
        let sink = Arc::new(UplinkSink::default());
        uplink.active.queued.base.set_id_base(0x0500_0000);
        uplink.evt.log_out.connect(sink.clone(), 0);
        uplink.buffer_send_out.connect(sink.clone(), 0);
        uplink.init(64);
        for packet in &packets {
            let mut storage = vec![0u8; packet.len()].into_boxed_slice();
            storage.copy_from_slice(packet);
            let mut buffer = Buffer::from_storage(storage, 0);
            buffer.set_size(packet.len());
            let p = uplink.buffer_send_in(0);
            p.target.invoke(p.port_num, buffer);
            let _ = uplink
                .active
                .queued
                .dispatch_available_messages(uplink.as_ref());
        }

        assert_eq!(std::fs::read(&dest).unwrap(), content);
        // FileReceived (id 2) and nothing else — no warnings on either side.
        assert_eq!(
            *sink.event_ids.lock().unwrap(),
            vec![FileUplink::EVENTID_FILE_RECEIVED]
        );
        assert_eq!(sink.returned.load(Ordering::SeqCst), packets.len() as u32);
    }

    #[derive(Default)]
    struct UplinkSink {
        event_ids: Mutex<Vec<FwEventIdType>>,
        returned: AtomicU32,
    }

    impl LogPort for UplinkSink {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            _args: &mut LogBuffer,
        ) {
            self.event_ids.lock().unwrap().push(id - 0x0500_0000);
        }
    }

    impl BufferSendPort for UplinkSink {
        fn invoke(&self, _port_num: FwIndexType, _buffer: Buffer) {
            self.returned.fetch_add(1, Ordering::SeqCst);
        }
    }

    // ---- command handling -------------------------------------------------

    #[test]
    fn reg_commands_registers_the_three_opcodes() {
        let (comp, ground) = setup(0, 4);
        comp.reg_commands();
        assert_eq!(
            ground.regs.lock().unwrap().as_slice(),
            &[ID_BASE, ID_BASE + 1, ID_BASE + 2]
        );
    }

    #[test]
    fn residual_command_bytes_are_a_format_error() {
        let (comp, ground) = setup(0, 4);
        send_cmd(&comp, FileDownlink::OPCODE_SEND_FILE, 1, |args| {
            let s = CmdStringArg::from("a");
            let _ = s.serialize_to(args, Endianness::Big);
            let _ = s.serialize_to(args, Endianness::Big);
            let _ = args.serialize_u8(0xFF, Endianness::Big);
        });
        assert_eq!(
            ground.responses.lock().unwrap().as_slice(),
            &[(ID_BASE, 1, CmdResponse::FormatError)]
        );
    }

    #[test]
    fn short_command_args_are_a_format_error() {
        let (comp, ground) = setup(0, 4);
        send_cmd(&comp, FileDownlink::OPCODE_SEND_PARTIAL, 2, |args| {
            let s = CmdStringArg::from("a");
            let _ = s.serialize_to(args, Endianness::Big);
            let _ = s.serialize_to(args, Endianness::Big);
            let _ = args.serialize_u32_be(0);
            // `length` missing
        });
        assert_eq!(
            ground.responses.lock().unwrap().as_slice(),
            &[(ID_BASE + 2, 2, CmdResponse::FormatError)]
        );
    }

    #[test]
    fn unknown_opcode_answers_invalid_opcode() {
        let (comp, ground) = setup(0, 4);
        send_cmd(&comp, 0x7F, 5, |_args| {});
        assert_eq!(
            ground.responses.lock().unwrap().as_slice(),
            &[(ID_BASE + 0x7F, 5, CmdResponse::InvalidOpcode)]
        );
    }

    #[test]
    fn a_full_request_queue_answers_execution_error_immediately() {
        let (comp, ground) = setup(0, 1);
        let dir = temp_dir("full");
        let source = write_file(&dir, "f.bin", b"x");
        send_file_cmd(&comp, 1, &source, "g1");
        send_file_cmd(&comp, 2, &source, "g2");
        assert_eq!(
            ground.responses.lock().unwrap().as_slice(),
            &[(ID_BASE, 2, CmdResponse::ExecutionError)]
        );
    }

    #[test]
    fn send_partial_downlinks_a_window() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("partial");
        let content: Vec<u8> = (0u8..100).collect();
        let source = write_file(&dir, "p.bin", &content);
        send_cmd(&comp, FileDownlink::OPCODE_SEND_PARTIAL, 9, |args| {
            let src = CmdStringArg::from(source.as_str());
            let dst = CmdStringArg::from("g.bin");
            let _ = src.serialize_to(args, Endianness::Big);
            let _ = dst.serialize_to(args, Endianness::Big);
            let _ = args.serialize_u32_be(10);
            let _ = args.serialize_u32_be(20);
        });
        let packets = run_transfer(&comp, &ground);
        assert_eq!(packets.len(), 3);
        // DATA offset 10, size 20, payload = content[10..30]
        assert_eq!(&packets[1][7..11], &10u32.to_be_bytes());
        assert_eq!(&packets[1][11..13], &20u16.to_be_bytes());
        assert_eq!(&packets[1][13..], &content[10..30]);
        // SendStarted reports the requested length, not the file size.
        let started = ground.events_of(FileDownlink::EVENTID_SEND_STARTED);
        assert_eq!(&started[0].1[..4], &20u32.to_be_bytes());
    }

    #[test]
    fn a_length_past_eof_is_clamped_with_a_warning() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("clamp");
        let source = write_file(&dir, "c.bin", &[1u8; 10]);
        send_cmd(&comp, FileDownlink::OPCODE_SEND_PARTIAL, 1, |args| {
            let src = CmdStringArg::from(source.as_str());
            let dst = CmdStringArg::from("g");
            let _ = src.serialize_to(args, Endianness::Big);
            let _ = dst.serialize_to(args, Endianness::Big);
            let _ = args.serialize_u32_be(4);
            let _ = args.serialize_u32_be(999);
        });
        let packets = run_transfer(&comp, &ground);
        let warnings = ground.events_of(FileDownlink::EVENTID_DOWNLINK_PARTIAL_WARNING);
        assert_eq!(warnings.len(), 1);
        assert_eq!(warnings[0].0, LogSeverity::WarningLo);
        // [startOffset 4][length 999][filesize 10]...
        assert_eq!(
            &warnings[0].1[..12],
            &[0, 0, 0, 4, 0, 0, 3, 231, 0, 0, 0, 10]
        );
        // The clamped window is 6 bytes.
        assert_eq!(u16::from_be_bytes([packets[1][11], packets[1][12]]), 6);
        // The partial warning does NOT bump the Warnings counter.
        assert_eq!(ground.last_tlm(FileDownlink::CHANID_WARNINGS), None);
    }

    #[test]
    fn a_start_offset_past_eof_fails_and_enters_cooldown() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("past");
        let source = write_file(&dir, "s.bin", &[1u8; 4]);
        send_cmd(&comp, FileDownlink::OPCODE_SEND_PARTIAL, 4, |args| {
            let src = CmdStringArg::from(source.as_str());
            let dst = CmdStringArg::from("g");
            let _ = src.serialize_to(args, Endianness::Big);
            let _ = dst.serialize_to(args, Endianness::Big);
            let _ = args.serialize_u32_be(99);
            let _ = args.serialize_u32_be(0);
        });
        tick(&comp);
        assert_eq!(ground.sent_count(), 0);
        assert_eq!(comp.mode(), Mode::Cooldown);
        assert_eq!(
            ground
                .events_of(FileDownlink::EVENTID_DOWNLINK_PARTIAL_FAIL)
                .len(),
            1
        );
        // COMMAND_FAILURES_DISABLED: the response is OK, not VALIDATION_ERROR.
        assert_eq!(
            ground.responses.lock().unwrap().as_slice(),
            &[(ID_BASE + 2, 4, CmdResponse::Ok)]
        );
    }

    #[test]
    fn a_missing_source_file_warns_but_answers_ok() {
        let (comp, ground) = setup(0, 4);
        send_file_cmd(&comp, 1, "/no/such/file.bin", "g");
        tick(&comp);
        assert_eq!(
            ground
                .events_of(FileDownlink::EVENTID_FILE_OPEN_ERROR)
                .len(),
            1
        );
        assert_eq!(ground.last_tlm(FileDownlink::CHANID_WARNINGS), Some(1));
        assert_eq!(comp.mode(), Mode::Idle);
        assert_eq!(
            ground.responses.lock().unwrap().as_slice(),
            &[(ID_BASE, 1, CmdResponse::Ok)]
        );
    }

    #[test]
    fn a_source_outside_the_sandbox_warns_with_its_own_event() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("sandbox");
        let source = write_file(&dir, "in.bin", b"data");
        let outside = temp_dir("outside");
        let outside_file = write_file(&outside, "out.bin", b"data");
        comp.configure_sandbox(dir.to_str().unwrap());

        send_file_cmd(&comp, 1, &outside_file, "g");
        tick(&comp);
        assert_eq!(
            ground
                .events_of(FileDownlink::EVENTID_SOURCE_OUT_OF_SANDBOX)
                .len(),
            1
        );
        assert!(
            ground
                .events_of(FileDownlink::EVENTID_FILE_OPEN_ERROR)
                .is_empty()
        );
        // A source inside the sandbox still works.
        send_file_cmd(&comp, 2, &source, "g");
        let packets = run_transfer(&comp, &ground);
        assert_eq!(packets.len(), 3);
    }

    #[test]
    fn a_zero_size_file_warns_and_answers_ok() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("zero");
        let source = write_file(&dir, "e.bin", b"");
        send_file_cmd(&comp, 6, &source, "g");
        tick(&comp);
        assert_eq!(
            ground
                .events_of(FileDownlink::EVENTID_DOWNLINK_ZERO_SIZE_FILE)
                .len(),
            1
        );
        assert_eq!(ground.last_tlm(FileDownlink::CHANID_WARNINGS), Some(1));
        assert_eq!(comp.mode(), Mode::Idle);
        assert_eq!(
            ground.responses.lock().unwrap().as_slice(),
            &[(ID_BASE, 6, CmdResponse::Ok)]
        );
    }

    // ---- cancel -----------------------------------------------------------

    #[test]
    fn cancel_sends_a_cancel_packet_and_finishes_ok() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("cancel");
        let source = write_file(&dir, "c.bin", &vec![7u8; 1500]);
        send_file_cmd(&comp, 2, &source, "g");
        tick(&comp);
        let start_buffer = ground.take_sent();

        // Cancel while WAITing.
        send_cmd(&comp, FileDownlink::OPCODE_CANCEL, 3, |_args| {});
        assert_eq!(comp.mode(), Mode::Cancel);
        assert_eq!(
            ground.responses.lock().unwrap()[0],
            (ID_BASE + 1, 3, CmdResponse::Ok)
        );

        // The START buffer comes back -> a CANCEL packet goes out.
        return_buffer(&comp, start_buffer);
        assert_eq!(ground.sent_count(), 1);
        let cancel_buffer = ground.take_sent();
        let bytes = cancel_buffer.data().to_vec();
        assert_eq!(bytes, vec![0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x01]);

        // Its return completes the transfer with STATUS_OK.
        return_buffer(&comp, cancel_buffer);
        assert_eq!(
            ground
                .events_of(FileDownlink::EVENTID_DOWNLINK_CANCELED)
                .len(),
            1
        );
        assert!(ground.events_of(FileDownlink::EVENTID_FILE_SENT).is_empty());
        assert_eq!(ground.last_tlm(FileDownlink::CHANID_FILES_SENT), None);
        assert_eq!(
            ground.responses.lock().unwrap()[1],
            (ID_BASE, 2, CmdResponse::Ok)
        );
    }

    #[test]
    fn cancel_in_idle_is_a_no_op_that_still_answers_ok() {
        let (comp, ground) = setup(0, 4);
        send_cmd(&comp, FileDownlink::OPCODE_CANCEL, 1, |_args| {});
        assert_eq!(comp.mode(), Mode::Idle);
        assert_eq!(
            ground.responses.lock().unwrap().as_slice(),
            &[(ID_BASE + 1, 1, CmdResponse::Ok)]
        );
    }

    #[test]
    fn the_buffer_sent_before_a_cancel_packet_becomes_stale() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("stale");
        let source = write_file(&dir, "s.bin", &vec![3u8; 1500]);
        send_file_cmd(&comp, 1, &source, "g");
        tick(&comp);
        let start_buffer = ground.take_sent();
        send_cmd(&comp, FileDownlink::OPCODE_CANCEL, 2, |_args| {});
        return_buffer(&comp, start_buffer);
        let cancel_buffer = ground.take_sent();

        // A duplicate return of a now-stale context must be ignored.
        let mut stale = Buffer::from_storage(vec![0u8; INTERNAL_BUFFER_SIZE].into_boxed_slice(), 0);
        stale.set_size(8);
        return_buffer(&comp, stale);
        assert_eq!(ground.sent_count(), 0);
        assert!(
            ground
                .events_of(FileDownlink::EVENTID_DOWNLINK_CANCELED)
                .is_empty()
        );

        // The real cancel buffer still completes the transfer.
        return_buffer(&comp, cancel_buffer);
        assert_eq!(
            ground
                .events_of(FileDownlink::EVENTID_DOWNLINK_CANCELED)
                .len(),
            1
        );
    }

    #[test]
    fn a_return_in_idle_mode_is_ignored() {
        let (comp, ground) = setup(0, 4);
        let mut buffer =
            Buffer::from_storage(vec![0u8; INTERNAL_BUFFER_SIZE].into_boxed_slice(), 0);
        buffer.set_size(8);
        return_buffer(&comp, buffer);
        assert_eq!(ground.sent_count(), 0);
        assert_eq!(comp.mode(), Mode::Idle);
    }

    // ---- cooldown ---------------------------------------------------------

    #[test]
    fn cooldown_counts_cycles_before_returning_to_idle() {
        let (comp, ground) = setup(2000, 4);
        let dir = temp_dir("cool");
        let source = write_file(&dir, "c.bin", b"abc");
        send_file_cmd(&comp, 1, &source, "g");
        run_transfer(&comp, &ground);
        assert_eq!(comp.mode(), Mode::Cooldown);
        // cycleTime = 1000, cooldown = 2000: three ticks to reach IDLE.
        tick(&comp);
        assert_eq!(comp.mode(), Mode::Cooldown);
        tick(&comp);
        assert_eq!(comp.mode(), Mode::Cooldown);
        tick(&comp);
        assert_eq!(comp.mode(), Mode::Idle);
    }

    // ---- guarded SendFile port -------------------------------------------

    #[test]
    fn the_guarded_port_hands_back_incrementing_contexts() {
        let (comp, _ground) = setup(0, 4);
        let dir = temp_dir("port");
        let source = write_file(&dir, "p.bin", b"abc");
        let p = comp.send_file_in(0);
        for expected in 0..3u32 {
            let resp = p.target.invoke(
                p.port_num,
                &SendFileNameArg::from(source.as_str()),
                &SendFileNameArg::from("g"),
                0,
                0,
            );
            assert_eq!(*resp.get_status(), SendFileStatus::StatusOk);
            assert_eq!(*resp.get_context(), expected);
        }
        // The fourth request overflows the depth-4... queue after 3? No: the
        // depth is 4, so the fourth still fits and the fifth fails.
        let resp = p.target.invoke(
            p.port_num,
            &SendFileNameArg::from(source.as_str()),
            &SendFileNameArg::from("g"),
            0,
            0,
        );
        assert_eq!(*resp.get_status(), SendFileStatus::StatusOk);
        let resp = p.target.invoke(
            p.port_num,
            &SendFileNameArg::from(source.as_str()),
            &SendFileNameArg::from("g"),
            0,
            0,
        );
        // A full queue is STATUS_ERROR with the U32::MAX context sentinel.
        assert_eq!(*resp.get_status(), SendFileStatus::StatusError);
        assert_eq!(*resp.get_context(), u32::MAX);
    }

    #[test]
    fn a_port_sourced_transfer_reports_on_file_complete() {
        let (comp, ground) = setup(0, 4);
        let dir = temp_dir("complete");
        let source = write_file(&dir, "p.bin", b"abcd");
        let p = comp.send_file_in(0);
        let resp = p.target.invoke(
            p.port_num,
            &SendFileNameArg::from(source.as_str()),
            &SendFileNameArg::from("g"),
            0,
            0,
        );
        assert_eq!(*resp.get_context(), 0);
        run_transfer(&comp, &ground);
        let completions = ground.completions.lock().unwrap();
        assert_eq!(completions.len(), 1);
        assert_eq!(*completions[0].get_status(), SendFileStatus::StatusOk);
        assert_eq!(*completions[0].get_context(), 0);
        // No command response: this request came from the port.
        assert!(ground.responses.lock().unwrap().is_empty());
    }

    // ---- wire formats -----------------------------------------------------

    #[test]
    fn send_file_response_is_five_bytes() {
        let resp = SendFileResponse::new(SendFileStatus::StatusInvalid, 0x0102_0304);
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(
            resp.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0x02, 0x01, 0x02, 0x03, 0x04]);
        assert_eq!(SendFileResponse::SERIALIZED_SIZE, 5);
    }

    #[test]
    fn status_to_cmd_resp_maps_every_variant() {
        assert_eq!(
            status_to_cmd_resp(SendFileStatus::StatusOk),
            CmdResponse::Ok
        );
        assert_eq!(
            status_to_cmd_resp(SendFileStatus::StatusError),
            CmdResponse::ExecutionError
        );
        assert_eq!(
            status_to_cmd_resp(SendFileStatus::StatusInvalid),
            CmdResponse::ValidationError
        );
        assert_eq!(
            status_to_cmd_resp(SendFileStatus::StatusBusy),
            CmdResponse::Busy
        );
    }

    #[test]
    fn command_failures_are_disabled_by_default() {
        const { assert!(COMMAND_FAILURES_DISABLED) };
        assert_eq!(
            failure_status(SendFileStatus::StatusError),
            SendFileStatus::StatusOk
        );
    }

    #[test]
    fn queue_message_size_fits_a_full_command_argument_buffer() {
        assert_eq!(QUEUE_MSG_SIZE, 522);
        assert_eq!(MAX_DATA_SIZE, 499);
    }

    #[test]
    fn ping_is_echoed() {
        let (comp, ground) = setup(0, 4);
        let p = comp.ping_in(0);
        p.target.invoke(p.port_num, 42);
        dispatch(&comp);
        assert_eq!(ground.pings.lock().unwrap().as_slice(), &[42]);
    }
}
