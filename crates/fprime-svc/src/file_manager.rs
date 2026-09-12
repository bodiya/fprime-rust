//! # Svc::FileManager — on-board filesystem commands (ACTIVE)
//!
//! Port of `Svc/FileManager/FileManager.cpp`, `FileManager.fpp`,
//! `Commands.fppi`, `Events.fppi` and `Telemetry.fppi` per
//! `docs/cpp-analysis/file-services.md` (§ "Svc::FileManager" + its gotcha
//! list).
//!
//! Eight filesystem commands run to completion on the component thread; two
//! (`ListDirectory` and `GenerateDp`) are rate-group paced. `schedIn` is
//! SYNC — it runs on the rate group's thread and only performs an
//! `AtomicBool` compare-exchange before hopping onto the component thread
//! through the `internal port run drop` message, so a lagging component
//! thread simply skips ticks instead of flooding its queue.
//!
//! **Opcode 0x04 is a deliberate gap** (the historic `ShellCommand` was
//! removed upstream); event ids 0x04, 0x07 and 0x0D are gaps too. There is
//! no shell/exec command in this version and none is added here.
//!
//! Ported quirks (all covered by tests):
//!
//! - `RemoveFile` with `ignoreErrors = true` takes an EARLY return that
//!   increments `Errors` and responds `OK`, skipping `emitTelemetry` and
//!   `sendCommandResponse` — a suppressed removal counts as an error but
//!   never as a `CommandsExecuted`;
//! - `CalculateCrc` (0x08) and `GenerateDp` (0x09) do NOT touch the
//!   `CommandsExecuted`/`Errors` channels at all;
//! - `GenerateDp` always responds `Fw::CmdResponse::OK`, including for BUSY,
//!   unconnected DP ports and every failure stage — only the warning event
//!   distinguishes them;
//! - a second `ListDirectory` while one is in progress is rejected with
//!   `ListDirectoryError(dirName, Os::Directory::OTHER_ERROR = 10)` — a
//!   *Directory* status — while `emitTelemetry` receives a *FileSystem*
//!   status;
//! - the listing formats the full path as `"%s/%s"` even when the directory
//!   name already ends in `/`, producing a double slash.
//!
//! ## Data products
//!
//! `GenerateDp` (0x09) packages a byte range of a file into
//! `FileDpContainer` data products through the `Fw.DataProductSync`
//! ports (`productGetOut` / `productSendOut`). Each container holds one
//! chunk: a `FileChunkHeaderRecord` (`[id][FileChunkHeader]`) followed by a
//! `FileChunkDataRecord` (`[id][count][bytes]`), so ground tools can
//! reassemble the file from any number of containers. `IMMEDIATE` mode
//! emits the whole range inside the command handler; `PACED` mode emits
//! `CHUNKS_PER_RATE_TICK` chunks per `schedIn` tick and defers the command
//! response until the range is exhausted or a failure ends the run. With
//! the DP ports unconnected the command takes the C++ path verbatim:
//! `GenerateDpBufferFailed` and `OK`.
//!
//! Not ported: the newer upstream `resolveInSandbox` step (this port's
//! `FileManager` has no sandbox yet; see `docs/ROADMAP.md`).

use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, ComponentDispatch, EventGlue, MsgDispatchStatus,
    OutputPort, PingPort, PortRef, QueueFullPolicy, SchedPort, TlmGlue, msg,
};
use fprime_config::{
    FILE_NAME_STRING_SIZE, FW_CMD_ARG_BUFFER_MAX_SIZE, FW_LOG_STRING_MAX_SIZE, FwChanIdType,
    FwDpIdType, FwDpPriorityType, FwEnumStoreType, FwEventIdType, FwIdType, FwIndexType,
    FwOpcodeType, FwQueuePriorityType, FwSignedSizeType, FwSizeStoreType, FwSizeType,
};
use fprime_fw::dp::{DpContainer, DpGetPort, DpSendPort};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, CmdStringArg, Endianness, ExtBuf, FileNameString,
    FwDefaultString, LengthMode, LinearBuffer, LogSeverity, LogStringArg, SerBuf, SerBufAny,
    Serialize, SerializeStatus, Success, fpp_enum, fpp_struct, fw_assert, fw_try,
};
use fprime_os::directory::{OpenMode, Status as DirStatus};
use fprime_os::file::{Mode as FileMode, SeekType, Status as FileStatus, WaitType};
use fprime_os::{Directory, File, filesystem};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Config (C++ default/config/FileManagerConfig.hpp + FileManagerCfg.fpp)
// ---------------------------------------------------------------------------

/// `FileManagerConfig::FILES_PER_RATE_TICK` — directory entries listed per
/// rate tick.
pub const FILES_PER_RATE_TICK: u32 = 1;
/// `FileManagerConfig::GENERATE_DP_MAX_CHUNK_SIZE`.
pub const GENERATE_DP_MAX_CHUNK_SIZE: u32 = 1024;
/// `FileManagerConfig::CHUNKS_PER_RATE_TICK`.
pub const CHUNKS_PER_RATE_TICK: u32 = 1;
/// `FileManagerCfg::DEFAULT_DP_PRIORITY` — the container priority used when
/// a `GenerateDp` command passes `priority = 0`.
pub const DEFAULT_DP_PRIORITY: FwDpPriorityType = 10;

// ---------------------------------------------------------------------------
// Data products (FileManager.fpp `product container` / `product record`)
// ---------------------------------------------------------------------------

/// `product container FileDpContainer id 0` (relative to the base id).
pub const CONTAINER_ID_FILE_DP: FwDpIdType = 0;
/// `product record FileChunkHeaderRecord: FileChunkHeader id 0` (relative to
/// the base id).
pub const RECORD_ID_FILE_CHUNK_HEADER: FwDpIdType = 0;
/// `product record FileChunkDataRecord: U8 array id 1` (relative to the base
/// id).
pub const RECORD_ID_FILE_CHUNK_DATA: FwDpIdType = 1;
/// Autocoded `SIZE_OF_FileChunkHeaderRecord_RECORD`: the record id plus the
/// MAXIMUM serialized [`FileChunkHeader`] (the string at full capacity).
pub const SIZE_OF_FILE_CHUNK_HEADER_RECORD: FwSizeType =
    (size_of::<FwDpIdType>() + FileChunkHeader::SERIALIZED_SIZE) as FwSizeType;

/// Autocoded `SIZE_OF_FileChunkDataRecord_RECORD(n)`: the record id, the
/// element count (`FwSizeStoreType`) and `n` bytes.
#[must_use]
pub const fn size_of_file_chunk_data_record(elements: FwSizeType) -> FwSizeType {
    (size_of::<FwDpIdType>() + size_of::<FwSizeStoreType>()) as FwSizeType + elements
}

fpp_struct! {
    /// `Svc.FileManager.FileChunkHeader` — metadata for one chunk of a file
    /// data product. Each instance is followed by a `FileChunkDataRecord`
    /// carrying the chunk's bytes.
    #[derive(Clone)]
    pub struct FileChunkHeader {
        /// The name of the source file (`string size FileNameStringSize`).
        file_name: FileNameString,
        /// The offset of this chunk within the source file.
        offset: u64,
        /// The number of data bytes in this chunk.
        data_size: u32,
    }
}

fpp_enum! {
    /// `Svc.FileManager.GenerateDpStage` — where a data-product generation
    /// failed. FPP declares no representation type, so it is `I32`
    /// (`FwEnumStoreType`) on the wire.
    pub enum GenerateDpStage : i32 {
        /// Opening the source file.
        Open = 0,
        /// Querying the size of the source file.
        Size = 1,
        /// Seeking to the requested begin offset.
        Seek = 2,
        /// Reading a chunk from the source file.
        Read = 3,
        /// Serializing a chunk into the container.
        Serialize = 4,
        /// A request arrived while another was in progress.
        Busy = 5,
    }
    default Open
}

fpp_enum! {
    /// `Svc.FileManager.GenerateDpMode` — how chunks are emitted. `I32` on
    /// the wire (no FPP representation type).
    pub enum GenerateDpMode : i32 {
        /// One chunk per rate-group tick.
        Paced = 0,
        /// All chunks in the command handler.
        Immediate = 1,
    }
    default Paced
}

fpp_enum! {
    /// `Fw.StringFormatStatus` — the shadow enum for `Fw::FormatStatus`.
    /// Declared here because `fprime-fw`'s `enums` module (owned by an
    /// earlier wave) does not carry it; move it there when that crate is
    /// next touched.
    pub enum StringFormatStatus : u8 {
        /// Format worked.
        Success = 0,
        /// Format overflowed the destination.
        Overflowed = 1,
        /// The format string was invalid.
        InvalidFormatString = 2,
        /// `FwSizeType` overflowed `size_t`.
        SizeOverflow = 3,
        /// An underlying call returned an error.
        OtherError = 4,
    }
    default Success
}

// ---------------------------------------------------------------------------
// Dictionary constants
// ---------------------------------------------------------------------------

const MSG_TYPE_PING_IN: FwEnumStoreType = 1;
const MSG_TYPE_CMD_IN: FwEnumStoreType = 2;
/// The `internal port run drop` message (schedIn hops onto the component
/// thread through it).
const MSG_TYPE_RUN_INTERNAL: FwEnumStoreType = 3;

/// Queue message size = the command envelope, the largest async invocation:
/// 6 + 4 (opcode) + 4 (cmdSeq) + 2 + 506 (nested `CmdArgBuffer`) = 522.
pub const QUEUE_MSG_SIZE: FwSizeType =
    (msg::ENVELOPE_HEADER_SIZE + 4 + 4 + 2 + FW_CMD_ARG_BUFFER_MAX_SIZE) as FwSizeType;

const PORT_PRIORITY: FwQueuePriorityType = 1;

/// Event string arguments are `string size FileNameStringSize` (240), but
/// the generated `log_*` methods serialize through `Fw::LogStringArg`
/// (`StringTemplate<FW_LOG_STRING_MAX_SIZE>`), so the effective on-wire cap
/// is `min(declared size, FW_LOG_STRING_MAX_SIZE)` = 200.
const EVENT_STRING_SIZE: usize = if FILE_NAME_STRING_SIZE < FW_LOG_STRING_MAX_SIZE {
    FILE_NAME_STRING_SIZE
} else {
    FW_LOG_STRING_MAX_SIZE
};

type MsgBuffer = LinearBuffer<{ QUEUE_MSG_SIZE as usize }>;

/// `FileManager::ListDirectoryState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ListDirectoryState {
    Idle,
    ListingInProgress,
}

/// `FileManager::GenerateDpState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GenerateDpState {
    /// Not currently generating a data product.
    Idle,
    /// A `GenerateDp` range is being emitted (paced or immediate).
    InProgress,
}

/// Mutable component state.
struct ManagerState {
    command_count: u32,
    error_count: u32,
    list_state: ListDirectoryState,
    current_dir: Directory,
    current_dir_name: CmdStringArg,
    total_entries: u32,
    current_op_code: FwOpcodeType,
    current_cmd_seq: u32,
    // -- GenerateDp (C++ m_dp*) --------------------------------------------
    dp_state: GenerateDpState,
    dp_file: File,
    dp_file_name: FileNameString,
    dp_file_size: FwSizeType,
    dp_offset: u64,
    dp_chunk_size: u32,
    dp_end_offset: u64,
    dp_priority: FwDpPriorityType,
    dp_chunk_count: u32,
    dp_op_code: FwOpcodeType,
    dp_cmd_seq: u32,
    /// C++ `m_dpBuffer[GENERATE_DP_MAX_CHUNK_SIZE]` — allocated once at
    /// construction, reused for every chunk.
    dp_buffer: Box<[u8]>,
}

impl ManagerState {
    fn new() -> Self {
        Self {
            command_count: 0,
            error_count: 0,
            list_state: ListDirectoryState::Idle,
            current_dir: Directory::new(),
            current_dir_name: CmdStringArg::new(),
            total_entries: 0,
            current_op_code: 0,
            current_cmd_seq: 0,
            dp_state: GenerateDpState::Idle,
            dp_file: File::new(),
            dp_file_name: FileNameString::new(),
            dp_file_size: 0,
            dp_offset: 0,
            dp_chunk_size: 0,
            dp_end_offset: 0,
            dp_priority: DEFAULT_DP_PRIORITY,
            dp_chunk_count: 0,
            dp_op_code: 0,
            dp_cmd_seq: 0,
            dp_buffer: vec![0u8; GENERATE_DP_MAX_CHUNK_SIZE as usize].into_boxed_slice(),
        }
    }
}

/// `Svc::FileManager` — the active filesystem-command component.
pub struct FileManager {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Command registration/response ports.
    pub cmd: CmdGlue,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// `pingOut` — echoes the ping key.
    pub ping_out: OutputPort<dyn PingPort>,
    /// `productGetOut` — SYNC `Fw.DpGet`: request a `FileDpContainer` buffer.
    pub product_get_out: OutputPort<dyn DpGetPort>,
    /// `productSendOut` — `Fw.DpSend`: hand a filled container to the
    /// data-product manager.
    pub product_send_out: OutputPort<dyn DpSendPort>,
    /// C++ `std::atomic<bool> m_runQueued`: the gate that keeps a lagging
    /// component thread from being flooded with rate ticks.
    run_queued: AtomicBool,
    state: Mutex<ManagerState>,
}

impl FileManager {
    // -- Commands ----------------------------------------------------------

    /// `CreateDirectory(dirName)` — opcode 0x00.
    pub const OPCODE_CREATE_DIRECTORY: FwOpcodeType = 0x00;
    /// `MoveFile(sourceFileName, destFileName)` — opcode 0x01.
    pub const OPCODE_MOVE_FILE: FwOpcodeType = 0x01;
    /// `RemoveDirectory(dirName)` — opcode 0x02.
    pub const OPCODE_REMOVE_DIRECTORY: FwOpcodeType = 0x02;
    /// `RemoveFile(fileName, ignoreErrors)` — opcode 0x03.
    /// (0x04 is the deliberate `ShellCommand` gap.)
    pub const OPCODE_REMOVE_FILE: FwOpcodeType = 0x03;
    /// `AppendFile(source, target)` — opcode 0x05.
    pub const OPCODE_APPEND_FILE: FwOpcodeType = 0x05;
    /// `FileSize(fileName)` — opcode 0x06.
    pub const OPCODE_FILE_SIZE: FwOpcodeType = 0x06;
    /// `ListDirectory(dirName)` — opcode 0x07 (rate-group paced).
    pub const OPCODE_LIST_DIRECTORY: FwOpcodeType = 0x07;
    /// `CalculateCrc(filename)` — opcode 0x08.
    pub const OPCODE_CALCULATE_CRC: FwOpcodeType = 0x08;
    /// `GenerateDp(fileName, chunkSize, beginOffset, endOffset, priority,
    /// mode)` — opcode 0x09.
    pub const OPCODE_GENERATE_DP: FwOpcodeType = 0x09;

    // -- Events ------------------------------------------------------------

    /// `DirectoryCreateError(dirName, status)` — WARNING_HI, id 0x00.
    pub const EVENTID_DIRECTORY_CREATE_ERROR: FwEventIdType = 0x00;
    /// `DirectoryRemoveError(dirName, status)` — WARNING_HI, id 0x01.
    pub const EVENTID_DIRECTORY_REMOVE_ERROR: FwEventIdType = 0x01;
    /// `FileMoveError(source, dest, status)` — WARNING_HI, id 0x02.
    pub const EVENTID_FILE_MOVE_ERROR: FwEventIdType = 0x02;
    /// `FileRemoveError(fileName, status)` — WARNING_HI, id 0x03.
    pub const EVENTID_FILE_REMOVE_ERROR: FwEventIdType = 0x03;
    /// `AppendFileFailed(source, target, status)` — WARNING_HI, id 0x05.
    pub const EVENTID_APPEND_FILE_FAILED: FwEventIdType = 0x05;
    /// `AppendFileSucceeded(source, target)` — ACTIVITY_HI, id 0x06.
    pub const EVENTID_APPEND_FILE_SUCCEEDED: FwEventIdType = 0x06;
    /// `CreateDirectorySucceeded(dirName)` — ACTIVITY_HI, id 0x08.
    pub const EVENTID_CREATE_DIRECTORY_SUCCEEDED: FwEventIdType = 0x08;
    /// `RemoveDirectorySucceeded(dirName)` — ACTIVITY_HI, id 0x09.
    pub const EVENTID_REMOVE_DIRECTORY_SUCCEEDED: FwEventIdType = 0x09;
    /// `MoveFileSucceeded(source, dest)` — ACTIVITY_HI, id 0x0A.
    pub const EVENTID_MOVE_FILE_SUCCEEDED: FwEventIdType = 0x0A;
    /// `RemoveFileSucceeded(fileName)` — ACTIVITY_HI, id 0x0B.
    pub const EVENTID_REMOVE_FILE_SUCCEEDED: FwEventIdType = 0x0B;
    /// `AppendFileStarted(source, target)` — ACTIVITY_HI, id 0x0C.
    pub const EVENTID_APPEND_FILE_STARTED: FwEventIdType = 0x0C;
    /// `CreateDirectoryStarted(dirName)` — ACTIVITY_HI, id 0x0E.
    pub const EVENTID_CREATE_DIRECTORY_STARTED: FwEventIdType = 0x0E;
    /// `RemoveDirectoryStarted(dirName)` — ACTIVITY_HI, id 0x0F.
    pub const EVENTID_REMOVE_DIRECTORY_STARTED: FwEventIdType = 0x0F;
    /// `MoveFileStarted(source, dest)` — ACTIVITY_HI, id 0x10.
    pub const EVENTID_MOVE_FILE_STARTED: FwEventIdType = 0x10;
    /// `RemoveFileStarted(fileName)` — ACTIVITY_HI, id 0x11.
    pub const EVENTID_REMOVE_FILE_STARTED: FwEventIdType = 0x11;
    /// `FileSizeSucceeded(fileName, size: FwSizeType)` — ACTIVITY_HI,
    /// id 0x12.
    pub const EVENTID_FILE_SIZE_SUCCEEDED: FwEventIdType = 0x12;
    /// `FileSizeError(fileName, status)` — WARNING_HI, id 0x13.
    pub const EVENTID_FILE_SIZE_ERROR: FwEventIdType = 0x13;
    /// `FileSizeStarted(fileName)` — ACTIVITY_HI, id 0x14.
    pub const EVENTID_FILE_SIZE_STARTED: FwEventIdType = 0x14;
    /// `ListDirectoryStarted(dirName)` — ACTIVITY_HI, id 0x15.
    pub const EVENTID_LIST_DIRECTORY_STARTED: FwEventIdType = 0x15;
    /// `ListDirectorySucceeded(dirName, fileCount: U32)` — ACTIVITY_HI,
    /// id 0x16.
    pub const EVENTID_LIST_DIRECTORY_SUCCEEDED: FwEventIdType = 0x16;
    /// `ListDirectoryError(dirName, status)` — WARNING_HI, id 0x17.
    pub const EVENTID_LIST_DIRECTORY_ERROR: FwEventIdType = 0x17;
    /// `DirectoryListing(dirName, fileName, fileSize: FwSizeType)` —
    /// ACTIVITY_HI, id 0x18.
    pub const EVENTID_DIRECTORY_LISTING: FwEventIdType = 0x18;
    /// `DirectoryListingSubdir(dirName, subdirName)` — ACTIVITY_HI,
    /// id 0x19.
    pub const EVENTID_DIRECTORY_LISTING_SUBDIR: FwEventIdType = 0x19;
    /// `CalculateCrcStarted(fileName)` — ACTIVITY_HI, id 0x1A.
    pub const EVENTID_CALCULATE_CRC_STARTED: FwEventIdType = 0x1A;
    /// `CalculateCrcFailed(fileName, status)` — WARNING_HI, id 0x1B.
    pub const EVENTID_CALCULATE_CRC_FAILED: FwEventIdType = 0x1B;
    /// `CalculateCrcSucceeded(fileName, crc: U32)` — ACTIVITY_HI, id 0x1C.
    pub const EVENTID_CALCULATE_CRC_SUCCEEDED: FwEventIdType = 0x1C;
    /// `FileNameFormatError(fileName, status: Fw.StringFormatStatus)` —
    /// WARNING_HI, id 0x1D.
    pub const EVENTID_FILE_NAME_FORMAT_ERROR: FwEventIdType = 0x1D;
    /// `GenerateDpStarted(fileName, bytesToWrite: U64)` — ACTIVITY_HI,
    /// id 0x1E.
    pub const EVENTID_GENERATE_DP_STARTED: FwEventIdType = 0x1E;
    /// `GenerateDpComplete(fileName, chunks: U32)` — ACTIVITY_HI, id 0x1F.
    pub const EVENTID_GENERATE_DP_COMPLETE: FwEventIdType = 0x1F;
    /// `GenerateDpFailed(fileName, stage: GenerateDpStage, status)` —
    /// WARNING_HI, id 0x20.
    pub const EVENTID_GENERATE_DP_FAILED: FwEventIdType = 0x20;
    /// `GenerateDpBufferFailed(fileName)` — WARNING_HI, id 0x21.
    pub const EVENTID_GENERATE_DP_BUFFER_FAILED: FwEventIdType = 0x21;
    /// `GenerateDpInvalidRange(fileName, beginOffset, endOffset, fileSize)`
    /// — WARNING_HI, id 0x22.
    pub const EVENTID_GENERATE_DP_INVALID_RANGE: FwEventIdType = 0x22;

    // -- Telemetry ---------------------------------------------------------

    /// `CommandsExecuted: U32` — id 0x00.
    pub const CHANID_COMMANDS_EXECUTED: FwChanIdType = 0x00;
    /// `Errors: U32` — id 0x01.
    pub const CHANID_ERRORS: FwChanIdType = 0x01;

    /// Construct the component (topology step 1).
    #[must_use]
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            ping_out: OutputPort::new(),
            product_get_out: OutputPort::new(),
            product_send_out: OutputPort::new(),
            run_queued: AtomicBool::new(false),
            state: Mutex::new(ManagerState::new()),
        })
    }

    /// Create the message queue.
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active.queued.create_queue(queue_depth, QUEUE_MSG_SIZE);
    }

    /// C++ `regCommands()` — opcode 0x04 is a deliberate gap.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_CREATE_DIRECTORY,
                Self::OPCODE_MOVE_FILE,
                Self::OPCODE_REMOVE_DIRECTORY,
                Self::OPCODE_REMOVE_FILE,
                Self::OPCODE_APPEND_FILE,
                Self::OPCODE_FILE_SIZE,
                Self::OPCODE_LIST_DIRECTORY,
                Self::OPCODE_CALCULATE_CRC,
                Self::OPCODE_GENERATE_DP,
            ],
        );
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    // -- Input-port factories ---------------------------------------------

    /// `pingIn` — ASYNC `Svc.Ping` input.
    pub fn ping_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn PingPort> {
        PortRef::new(Arc::new(PingInAdapter { comp: self.clone() }), port_num)
    }

    /// `cmdIn` — ASYNC `Fw.Cmd` input.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }

    /// `schedIn` — SYNC `Svc.Sched` input: the component implements the port
    /// trait and the handler runs on the RATE GROUP's thread.
    pub fn sched_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Handlers ----------------------------------------------------------

    /// `pingIn_handler`: echo the key.
    fn ping_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }

    /// `schedIn_handler` (CALLER's thread): gate on `m_runQueued` and hop to
    /// the component thread via the internal `run` message, whose queue-full
    /// policy is `drop` — a missed tick is simply skipped.
    fn sched_handler(&self, _port_num: FwIndexType, _context: u32) {
        if self
            .run_queued
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .is_ok()
        {
            self.run_internal_interface_invoke();
        }
    }

    fn run_internal_interface_invoke(&self) {
        let mut buf = MsgBuffer::new();
        // C++ internal-interface messages carry no port number; the
        // fprime-comp dispatch contract reads one for every non-EXIT
        // message, so a 0 is written here (internal only, never hubbed).
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_RUN_INTERNAL, 0);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Drop);
    }

    /// `run_internalInterfaceHandler` (component thread): data-product
    /// pacing then listing pacing, in that order, in the SAME tick.
    fn run_internal_handler(&self) {
        fw_assert!(self.run_queued.load(Ordering::SeqCst));
        self.run_queued.store(false, Ordering::SeqCst);
        // Data product generation is paced the same way as directory listing.
        {
            let mut state = self.state.lock().unwrap();
            if state.dp_state == GenerateDpState::InProgress {
                self.process_dp_chunks(&mut state, CHUNKS_PER_RATE_TICK);
            }
        }
        self.process_listing_tick();
    }

    /// One rate tick of the directory listing state machine.
    fn process_listing_tick(&self) {
        let mut state = self.state.lock().unwrap();
        if state.list_state != ListDirectoryState::ListingInProgress {
            return;
        }
        for _ in 0..FILES_PER_RATE_TICK {
            let mut filename = FileNameString::new();
            let status = state.current_dir.read(&mut filename);
            match status {
                DirStatus::NoMoreFiles => {
                    state.current_dir.close();
                    state.list_state = ListDirectoryState::Idle;
                    let dir_name = state.current_dir_name.clone();
                    let entries = state.total_entries;
                    self.log_list_directory_succeeded(&dir_name, entries);
                    self.emit_telemetry(&mut state, filesystem::Status::OpOk);
                    self.send_command_response(
                        state.current_op_code,
                        state.current_cmd_seq,
                        filesystem::Status::OpOk,
                    );
                    break;
                }
                DirStatus::OpOk => {
                    let dir_name = state.current_dir_name.clone();
                    let (full_path, format_status) = format_full_path(&dir_name, &filename);
                    if format_status != StringFormatStatus::Success {
                        self.log_file_name_format_error(&filename, format_status);
                    } else {
                        let path_str = full_path.as_str().unwrap_or("");
                        match filesystem::get_path_type(path_str) {
                            filesystem::PathType::File => {
                                let mut size: FwSizeType = 0;
                                let size_status = filesystem::get_file_size(path_str, &mut size);
                                let reported = if size_status == filesystem::Status::OpOk {
                                    size
                                } else {
                                    0
                                };
                                self.log_directory_listing(&dir_name, &filename, reported);
                            }
                            filesystem::PathType::Directory => {
                                self.log_directory_listing_subdir(&dir_name, &filename);
                            }
                            _ => self.log_directory_listing(&dir_name, &filename, 0),
                        }
                    }
                    state.total_entries += 1;
                }
                other => {
                    state.current_dir.close();
                    state.list_state = ListDirectoryState::Idle;
                    let dir_name = state.current_dir_name.clone();
                    self.log_list_directory_error(&dir_name, other as u32);
                    self.emit_telemetry(&mut state, filesystem::Status::OtherError);
                    self.send_command_response(
                        state.current_op_code,
                        state.current_cmd_seq,
                        filesystem::Status::OtherError,
                    );
                    break;
                }
            }
        }
    }

    // -- Command handlers --------------------------------------------------

    fn create_directory_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let dir_name = match self.read_one_string(op_code, cmd_seq, args) {
            Some(v) => v,
            None => return,
        };
        let log_name = to_log_string(&dir_name);
        self.log_one_name(
            Self::EVENTID_CREATE_DIRECTORY_STARTED,
            LogSeverity::ActivityHi,
            &log_name,
            "Creating directory",
        );
        let status = filesystem::create_directory(dir_name.as_str().unwrap_or(""), true);
        if status != filesystem::Status::OpOk {
            self.log_name_and_status(
                Self::EVENTID_DIRECTORY_CREATE_ERROR,
                LogSeverity::WarningHi,
                &log_name,
                status as u32,
                "Could not create directory",
            );
        } else {
            self.log_one_name(
                Self::EVENTID_CREATE_DIRECTORY_SUCCEEDED,
                LogSeverity::ActivityHi,
                &log_name,
                "Created directory",
            );
        }
        self.emit_telemetry_locked(status);
        self.send_command_response(op_code, cmd_seq, status);
    }

    fn remove_file_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut file_name = CmdStringArg::new();
        let mut ignore_errors = false;
        if !args.deserialize(&mut file_name, Endianness::Big).is_ok()
            || !args.deserialize_bool_be(&mut ignore_errors).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let log_name = to_log_string(&file_name);
        self.log_one_name(
            Self::EVENTID_REMOVE_FILE_STARTED,
            LogSeverity::ActivityHi,
            &log_name,
            "Removing file",
        );
        let status = filesystem::remove_file(file_name.as_str().unwrap_or(""));
        if status != filesystem::Status::OpOk {
            self.log_name_and_status(
                Self::EVENTID_FILE_REMOVE_ERROR,
                LogSeverity::WarningHi,
                &log_name,
                status as u32,
                "Could not remove file",
            );
            if ignore_errors {
                // GOTCHA: early return — counts as an error but never as a
                // CommandsExecuted, and responds OK.
                let mut state = self.state.lock().unwrap();
                state.error_count += 1;
                let count = state.error_count;
                drop(state);
                self.tlm_write(Self::CHANID_ERRORS, count);
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
                return;
            }
        } else {
            self.log_one_name(
                Self::EVENTID_REMOVE_FILE_SUCCEEDED,
                LogSeverity::ActivityHi,
                &log_name,
                "Removed file",
            );
        }
        self.emit_telemetry_locked(status);
        self.send_command_response(op_code, cmd_seq, status);
    }

    fn move_file_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let (source, dest) = match self.read_two_strings(op_code, cmd_seq, args) {
            Some(v) => v,
            None => return,
        };
        let log_source = to_log_string(&source);
        let log_dest = to_log_string(&dest);
        self.log_two_names(
            Self::EVENTID_MOVE_FILE_STARTED,
            LogSeverity::ActivityHi,
            &log_source,
            &log_dest,
            "Moving file",
        );
        let status =
            filesystem::move_file(source.as_str().unwrap_or(""), dest.as_str().unwrap_or(""));
        if status != filesystem::Status::OpOk {
            self.log_two_names_and_status(
                Self::EVENTID_FILE_MOVE_ERROR,
                LogSeverity::WarningHi,
                &log_source,
                &log_dest,
                status as u32,
                "Could not move file",
            );
        } else {
            self.log_two_names(
                Self::EVENTID_MOVE_FILE_SUCCEEDED,
                LogSeverity::ActivityHi,
                &log_source,
                &log_dest,
                "Moved file",
            );
        }
        self.emit_telemetry_locked(status);
        self.send_command_response(op_code, cmd_seq, status);
    }

    fn remove_directory_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let dir_name = match self.read_one_string(op_code, cmd_seq, args) {
            Some(v) => v,
            None => return,
        };
        let log_name = to_log_string(&dir_name);
        self.log_one_name(
            Self::EVENTID_REMOVE_DIRECTORY_STARTED,
            LogSeverity::ActivityHi,
            &log_name,
            "Removing directory",
        );
        let status = filesystem::remove_directory(dir_name.as_str().unwrap_or(""));
        if status != filesystem::Status::OpOk {
            self.log_name_and_status(
                Self::EVENTID_DIRECTORY_REMOVE_ERROR,
                LogSeverity::WarningHi,
                &log_name,
                status as u32,
                "Could not remove directory",
            );
        } else {
            self.log_one_name(
                Self::EVENTID_REMOVE_DIRECTORY_SUCCEEDED,
                LogSeverity::ActivityHi,
                &log_name,
                "Removed directory",
            );
        }
        self.emit_telemetry_locked(status);
        self.send_command_response(op_code, cmd_seq, status);
    }

    fn append_file_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let (source, target) = match self.read_two_strings(op_code, cmd_seq, args) {
            Some(v) => v,
            None => return,
        };
        let log_source = to_log_string(&source);
        let log_target = to_log_string(&target);
        self.log_two_names(
            Self::EVENTID_APPEND_FILE_STARTED,
            LogSeverity::ActivityHi,
            &log_source,
            &log_target,
            "Appending file",
        );
        let status = filesystem::append_file(
            source.as_str().unwrap_or(""),
            target.as_str().unwrap_or(""),
            true,
        );
        if status != filesystem::Status::OpOk {
            self.log_two_names_and_status(
                Self::EVENTID_APPEND_FILE_FAILED,
                LogSeverity::WarningHi,
                &log_source,
                &log_target,
                status as u32,
                "Could not append file",
            );
        } else {
            self.log_two_names(
                Self::EVENTID_APPEND_FILE_SUCCEEDED,
                LogSeverity::ActivityHi,
                &log_source,
                &log_target,
                "Appended file",
            );
        }
        self.emit_telemetry_locked(status);
        self.send_command_response(op_code, cmd_seq, status);
    }

    fn file_size_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let file_name = match self.read_one_string(op_code, cmd_seq, args) {
            Some(v) => v,
            None => return,
        };
        let log_name = to_log_string(&file_name);
        self.log_one_name(
            Self::EVENTID_FILE_SIZE_STARTED,
            LogSeverity::ActivityHi,
            &log_name,
            "Getting size of file",
        );
        let mut size: FwSizeType = 0;
        let status = filesystem::get_file_size(file_name.as_str().unwrap_or(""), &mut size);
        if status != filesystem::Status::OpOk {
            self.log_name_and_status(
                Self::EVENTID_FILE_SIZE_ERROR,
                LogSeverity::WarningHi,
                &log_name,
                status as u32,
                "Could not get size of file",
            );
        } else {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_FILE_SIZE_SUCCEEDED,
                LogSeverity::ActivityHi,
                &format!("File {log_name} size is {size}"),
                |buf| {
                    fw_try!(log_name.serialize_to_truncated(
                        buf,
                        EVENT_STRING_SIZE,
                        Endianness::Big
                    ));
                    buf.serialize_u64_be(size)
                },
            );
        }
        self.emit_telemetry_locked(status);
        self.send_command_response(op_code, cmd_seq, status);
    }

    /// `ListDirectory` — starts the rate-group-paced listing; NO response is
    /// sent until the listing completes.
    fn list_directory_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let dir_name = match self.read_one_string(op_code, cmd_seq, args) {
            Some(v) => v,
            None => return,
        };
        let mut state = self.state.lock().unwrap();
        if state.list_state == ListDirectoryState::ListingInProgress {
            // GOTCHA: the status arg is a DIRECTORY status here while
            // emitTelemetry gets a FILESYSTEM status.
            self.log_list_directory_error(&dir_name, DirStatus::OtherError as u32);
            self.emit_telemetry(&mut state, filesystem::Status::OtherError);
            self.send_command_response(op_code, cmd_seq, filesystem::Status::OtherError);
            return;
        }
        self.log_list_directory_started(&dir_name);
        let status = state
            .current_dir
            .open(dir_name.as_str().unwrap_or(""), OpenMode::Read);
        if status != DirStatus::OpOk {
            self.log_list_directory_error(&dir_name, status as u32);
            self.emit_telemetry(&mut state, filesystem::Status::OtherError);
            self.send_command_response(op_code, cmd_seq, filesystem::Status::OtherError);
            return;
        }
        state.list_state = ListDirectoryState::ListingInProgress;
        state.current_dir_name = dir_name;
        state.current_op_code = op_code;
        state.current_cmd_seq = cmd_seq;
        state.total_entries = 0;
    }

    /// `CalculateCrc` — does NOT touch the CommandsExecuted/Errors channels.
    fn calculate_crc_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let file_name = match self.read_one_string(op_code, cmd_seq, args) {
            Some(v) => v,
            None => return,
        };
        let log_name = to_log_string(&file_name);
        self.log_one_name(
            Self::EVENTID_CALCULATE_CRC_STARTED,
            LogSeverity::ActivityHi,
            &log_name,
            "Calculating CRC of file",
        );
        let mut file = File::new();
        let mut crc_value: u32 = 0;
        let mut status = file.open(file_name.as_str().unwrap_or(""), FileMode::OpenRead);
        if status == FileStatus::OpOk {
            status = file.calculate_crc(&mut crc_value);
        }
        if status == FileStatus::OpOk {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_CALCULATE_CRC_SUCCEEDED,
                LogSeverity::ActivityHi,
                &format!("CRC of file {log_name} is 0x{crc_value:x}"),
                |buf| {
                    fw_try!(log_name.serialize_to_truncated(
                        buf,
                        EVENT_STRING_SIZE,
                        Endianness::Big
                    ));
                    buf.serialize_u32_be(crc_value)
                },
            );
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
        } else {
            self.log_name_and_status(
                Self::EVENTID_CALCULATE_CRC_FAILED,
                LogSeverity::WarningHi,
                &log_name,
                status as u32,
                "Could not calculate CRC of file",
            );
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
        }
        file.close();
    }

    /// `GenerateDp(fileName, chunkSize, beginOffset, endOffset, priority,
    /// mode)` — package `[beginOffset, endOffset)` of a file into
    /// `FileDpContainer` data products, one chunk per container. Every
    /// failure emits a WARNING_HI event and still answers `OK` (C++ parity:
    /// a bad file name or a transient resource problem must not stop a
    /// whole sequence).
    fn generate_dp_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut file_name = CmdStringArg::new();
        let mut chunk_size = 0u32;
        let mut begin_offset = 0u64;
        let mut end_offset = 0u64;
        let mut priority = 0u32;
        let mut mode_repr: i32 = 0;
        if !args.deserialize(&mut file_name, Endianness::Big).is_ok()
            || !args.deserialize_u32_be(&mut chunk_size).is_ok()
            || !args.deserialize_u64_be(&mut begin_offset).is_ok()
            || !args.deserialize_u64_be(&mut end_offset).is_ok()
            || !args.deserialize_u32_be(&mut priority).is_ok()
            || !args.deserialize_i32_be(&mut mode_repr).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let Ok(mode) = GenerateDpMode::try_from(mode_repr) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        let log_name = to_log_string(&file_name);
        let mut state = self.state.lock().unwrap();

        // Reject a second request while one is already running.
        if state.dp_state != GenerateDpState::Idle {
            self.log_generate_dp_failed(&log_name, GenerateDpStage::Busy, 0);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            return;
        }

        // Data products must be available.
        if !self.product_get_out.is_connected() || !self.product_send_out.is_connected() {
            self.log_generate_dp_buffer_failed(&log_name);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            return;
        }

        // Clamp the requested chunk size to the configured read buffer.
        let effective_chunk_size = if chunk_size == 0 || chunk_size > GENERATE_DP_MAX_CHUNK_SIZE {
            GENERATE_DP_MAX_CHUNK_SIZE
        } else {
            chunk_size
        };

        let path = file_name.as_str().unwrap_or_default();
        let status = state.dp_file.open(path, FileMode::OpenRead);
        if status != FileStatus::OpOk {
            self.log_generate_dp_failed(&log_name, GenerateDpStage::Open, status as u32);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            return;
        }

        let mut file_size: FwSizeType = 0;
        let status = state.dp_file.size(&mut file_size);
        if status != FileStatus::OpOk {
            state.dp_file.close();
            self.log_generate_dp_failed(&log_name, GenerateDpStage::Size, status as u32);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            return;
        }

        // An end offset of zero, or one past the end of the file, means the
        // end of the file. Ranges let an operator retransmit part of a file
        // or spread the downlink over several commands.
        let mut effective_end = end_offset;
        if effective_end == 0 || effective_end > file_size {
            effective_end = file_size;
        }

        let empty_file = file_size == 0;
        let bad_range = begin_offset > file_size || (!empty_file && begin_offset >= effective_end);
        if bad_range {
            state.dp_file.close();
            self.log_generate_dp_invalid_range(&log_name, begin_offset, end_offset, file_size);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            return;
        }

        // Position the file at the start of the requested range.
        if begin_offset > 0 {
            let status = state
                .dp_file
                .seek(begin_offset as FwSignedSizeType, SeekType::Absolute);
            if status != FileStatus::OpOk {
                state.dp_file.close();
                self.log_generate_dp_failed(&log_name, GenerateDpStage::Seek, status as u32);
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
                return;
            }
        }

        state.dp_file_name.set(path);
        state.dp_file_size = file_size;
        state.dp_offset = begin_offset;
        state.dp_end_offset = effective_end;
        state.dp_chunk_size = effective_chunk_size;
        state.dp_chunk_count = 0;
        state.dp_op_code = op_code;
        state.dp_cmd_seq = cmd_seq;
        // A priority of zero reverts to the configured default.
        state.dp_priority = if priority == 0 {
            DEFAULT_DP_PRIORITY
        } else {
            priority
        };
        state.dp_state = GenerateDpState::InProgress;

        // Report the number of bytes that will be written: the requested
        // range rather than the size of the whole file.
        self.log_generate_dp_started(&log_name, state.dp_end_offset - state.dp_offset);

        // An empty range produces no chunks, so complete immediately.
        if state.dp_offset >= state.dp_end_offset {
            self.log_generate_dp_complete(&log_name, state.dp_chunk_count);
            self.finish_dp_generation(&mut state);
            return;
        }

        // In immediate mode the whole range is emitted here, so a project
        // that wants the file out quickly is not limited by the rate group.
        // In paced mode the rate group meters the work out and the response
        // is deferred.
        if mode == GenerateDpMode::Immediate {
            self.process_dp_chunks(&mut state, 0);
        }
    }

    /// C++ `processDpChunks(chunkLimit)`: emit up to `chunk_limit` chunks
    /// (0 = the whole remaining range), one container per chunk.
    fn process_dp_chunks(&self, state: &mut ManagerState, chunk_limit: u32) {
        let log_name = log_string_of(&state.dp_file_name);
        // A limit of zero means emit the whole remaining range in this call.
        let paced = chunk_limit > 0;
        let mut chunk = 0u32;
        while !paced || chunk < chunk_limit {
            // Bytes remaining in the requested range. The loop returns as
            // soon as the range is exhausted, so this is always non-zero.
            let remaining = state.dp_end_offset - state.dp_offset;
            let requested_size = remaining.min(u64::from(state.dp_chunk_size)) as usize;

            // The file size is known, so a short read means the file changed
            // underneath us.
            let mut read_size = requested_size as FwSizeType;
            let status = state.dp_file.read(
                &mut state.dp_buffer[..requested_size],
                &mut read_size,
                WaitType::Wait,
            );
            if status != FileStatus::OpOk || read_size as usize != requested_size {
                self.log_generate_dp_failed(&log_name, GenerateDpStage::Read, status as u32);
                self.finish_dp_generation(state);
                return;
            }

            // Request a container large enough for this chunk's header and
            // data (autocoded `dpGet_FileDpContainer`: the PACKET size that
            // holds `dp_size` bytes of records).
            let dp_size =
                SIZE_OF_FILE_CHUNK_HEADER_RECORD + size_of_file_chunk_data_record(read_size);
            let container_id = self.id_base() + CONTAINER_ID_FILE_DP;
            let mut buffer = Buffer::empty();
            let dp_status = {
                let p = self.product_get_out.get();
                p.target.invoke(
                    p.port_num,
                    container_id,
                    DpContainer::packet_size_for_data_size(dp_size),
                    &mut buffer,
                )
            };
            if dp_status != Success::Success {
                self.log_generate_dp_buffer_failed(&log_name);
                self.finish_dp_generation(state);
                return;
            }
            let mut container = DpContainer::with_buffer(container_id, buffer);
            container.set_priority(state.dp_priority);
            container.set_time_tag(self.evt.time_get());

            // Each chunk is a metadata record followed by a data record, so
            // that ground tools can reassemble the file from any number of
            // containers.
            let header = FileChunkHeader::new(
                state.dp_file_name.clone(),
                state.dp_offset,
                read_size as u32,
            );
            let (serialize_status, written) = {
                let mut ser = container.data_serializer();
                let status = Self::serialize_chunk_records(
                    &mut ser,
                    self.id_base(),
                    &header,
                    &state.dp_buffer[..requested_size],
                );
                (status, ser.ser_loc())
            };
            if !serialize_status.is_ok() {
                self.log_generate_dp_failed(
                    &log_name,
                    GenerateDpStage::Serialize,
                    serialize_status as u32,
                );
                self.finish_dp_generation(state);
                return;
            }
            container.set_data_size(written as FwSizeType);

            // Autocoded `dpSend`: finalize the header (which re-hashes it)
            // and hand the buffer to the data-product manager. The DATA hash
            // is `Svc::DpWriter`'s job (C++ parity).
            container.serialize_header();
            let buffer = container.take_buffer();
            {
                let p = self.product_send_out.get();
                p.target.invoke(p.port_num, container_id, buffer);
            }

            state.dp_offset += read_size;
            state.dp_chunk_count += 1;

            // Last chunk of the requested range.
            if state.dp_offset >= state.dp_end_offset {
                self.log_generate_dp_complete(&log_name, state.dp_chunk_count);
                self.finish_dp_generation(state);
                return;
            }
            chunk += 1;
        }
    }

    /// Autocoded `serializeRecord_FileChunkHeaderRecord` followed by
    /// `serializeRecord_FileChunkDataRecord`: `[id u32][FileChunkHeader]`
    /// then `[id u32][count FwSizeStoreType][bytes]`, ids absolute (base id
    /// + record id).
    fn serialize_chunk_records(
        ser: &mut ExtBuf<'_>,
        id_base: FwIdType,
        header: &FileChunkHeader,
        data: &[u8],
    ) -> SerializeStatus {
        fw_try!(ser.serialize_u32_be(id_base + RECORD_ID_FILE_CHUNK_HEADER));
        fw_try!(header.serialize_to(ser, Endianness::Big));
        fw_try!(ser.serialize_u32_be(id_base + RECORD_ID_FILE_CHUNK_DATA));
        fw_try!(ser.serialize_size(data.len() as FwSizeType, Endianness::Big));
        ser.serialize_bytes(data, LengthMode::OmitLength, Endianness::Big)
    }

    /// C++ `finishDpGeneration`: close the file, go idle and answer the
    /// (possibly deferred) command — always `OK`, since failures were
    /// already reported by their warning event.
    fn finish_dp_generation(&self, state: &mut ManagerState) {
        state.dp_file.close();
        state.dp_state = GenerateDpState::Idle;
        state.dp_offset = 0;
        state.dp_end_offset = 0;
        state.dp_file_size = 0;
        self.cmd
            .cmd_response(state.dp_op_code, state.dp_cmd_seq, CmdResponse::Ok);
    }

    // -- Argument helpers ---------------------------------------------------

    /// Deserialize one string argument, answering `FormatError` exactly once
    /// on a short read or residual bytes.
    fn read_one_string(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) -> Option<CmdStringArg> {
        let mut value = CmdStringArg::new();
        if !args.deserialize(&mut value, Endianness::Big).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return None;
        }
        Some(value)
    }

    fn read_two_strings(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) -> Option<(CmdStringArg, CmdStringArg)> {
        let mut first = CmdStringArg::new();
        let mut second = CmdStringArg::new();
        if !args.deserialize(&mut first, Endianness::Big).is_ok()
            || !args.deserialize(&mut second, Endianness::Big).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return None;
        }
        Some((first, second))
    }

    // -- Telemetry / response helpers --------------------------------------

    fn tlm_write(&self, chan: FwChanIdType, value: u32) {
        self.tlm
            .tlm_write(self.id_base(), chan, &value, self.evt.time_get());
    }

    /// C++ `emitTelemetry(status)`.
    fn emit_telemetry(&self, state: &mut ManagerState, status: filesystem::Status) {
        if status == filesystem::Status::OpOk {
            state.command_count += 1;
            let count = state.command_count;
            self.tlm_write(Self::CHANID_COMMANDS_EXECUTED, count);
        } else {
            state.error_count += 1;
            let count = state.error_count;
            self.tlm_write(Self::CHANID_ERRORS, count);
        }
    }

    fn emit_telemetry_locked(&self, status: filesystem::Status) {
        let mut state = self.state.lock().unwrap();
        self.emit_telemetry(&mut state, status);
    }

    /// C++ `sendCommandResponse(op, seq, status)`.
    fn send_command_response(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        status: filesystem::Status,
    ) {
        let response = if status == filesystem::Status::OpOk {
            CmdResponse::Ok
        } else {
            CmdResponse::ExecutionError
        };
        self.cmd.cmd_response(op_code, cmd_seq, response);
    }

    // -- Event helpers ------------------------------------------------------

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

    fn log_name_and_status(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        name: &LogStringArg,
        status: u32,
        text: &str,
    ) {
        self.evt.log_event(
            self.id_base(),
            id,
            severity,
            &format!("{text} {name}: status {status}"),
            |buf| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u32_be(status)
            },
        );
    }

    fn log_two_names(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        first: &LogStringArg,
        second: &LogStringArg,
        text: &str,
    ) {
        self.evt.log_event(
            self.id_base(),
            id,
            severity,
            &format!("{text} {first} to {second}"),
            |buf| {
                fw_try!(first.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                second.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_two_names_and_status(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        first: &LogStringArg,
        second: &LogStringArg,
        status: u32,
        text: &str,
    ) {
        self.evt.log_event(
            self.id_base(),
            id,
            severity,
            &format!("{text} {first} to {second}: status {status}"),
            |buf| {
                fw_try!(first.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(second.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u32_be(status)
            },
        );
    }

    fn log_list_directory_started(&self, dir_name: &CmdStringArg) {
        let name = to_log_string(dir_name);
        self.log_one_name(
            Self::EVENTID_LIST_DIRECTORY_STARTED,
            LogSeverity::ActivityHi,
            &name,
            "Listing directory",
        );
    }

    fn log_list_directory_error(&self, dir_name: &CmdStringArg, status: u32) {
        let name = to_log_string(dir_name);
        self.log_name_and_status(
            Self::EVENTID_LIST_DIRECTORY_ERROR,
            LogSeverity::WarningHi,
            &name,
            status,
            "Could not list directory",
        );
    }

    fn log_list_directory_succeeded(&self, dir_name: &CmdStringArg, file_count: u32) {
        let name = to_log_string(dir_name);
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_LIST_DIRECTORY_SUCCEEDED,
            LogSeverity::ActivityHi,
            &format!("Listed {file_count} entries in directory {name}"),
            |buf| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u32_be(file_count)
            },
        );
    }

    /// `GenerateDpStarted(fileName, bytesToWrite: U64)` — ACTIVITY_HI.
    fn log_generate_dp_started(&self, name: &LogStringArg, bytes_to_write: u64) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_GENERATE_DP_STARTED,
            LogSeverity::ActivityHi,
            &format!("Generating data products for file {name}: {bytes_to_write} bytes"),
            |buf| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u64_be(bytes_to_write)
            },
        );
    }

    /// `GenerateDpComplete(fileName, chunks: U32)` — ACTIVITY_HI.
    fn log_generate_dp_complete(&self, name: &LogStringArg, chunks: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_GENERATE_DP_COMPLETE,
            LogSeverity::ActivityHi,
            &format!("Generated {chunks} data product chunks for file {name}"),
            |buf| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u32_be(chunks)
            },
        );
    }

    /// `GenerateDpFailed(fileName, stage: GenerateDpStage, status: U32)` —
    /// WARNING_HI.
    fn log_generate_dp_failed(&self, name: &LogStringArg, stage: GenerateDpStage, status: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_GENERATE_DP_FAILED,
            LogSeverity::WarningHi,
            &format!(
                "Data product generation for file {name} failed at stage {}: status {status}",
                stage.as_repr()
            ),
            |buf| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(stage.serialize_to(buf, Endianness::Big));
                buf.serialize_u32_be(status)
            },
        );
    }

    /// `GenerateDpBufferFailed(fileName)` — WARNING_HI.
    fn log_generate_dp_buffer_failed(&self, name: &LogStringArg) {
        self.log_one_name(
            Self::EVENTID_GENERATE_DP_BUFFER_FAILED,
            LogSeverity::WarningHi,
            name,
            "Could not get a data product buffer for file",
        );
    }

    /// `GenerateDpInvalidRange(fileName, beginOffset: U64, endOffset: U64,
    /// fileSize: U64)` — WARNING_HI.
    fn log_generate_dp_invalid_range(
        &self,
        name: &LogStringArg,
        begin_offset: u64,
        end_offset: u64,
        file_size: u64,
    ) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_GENERATE_DP_INVALID_RANGE,
            LogSeverity::WarningHi,
            &format!(
                "Invalid range [{begin_offset}, {end_offset}) for file {name} of size {file_size}"
            ),
            |buf| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u64_be(begin_offset));
                fw_try!(buf.serialize_u64_be(end_offset));
                buf.serialize_u64_be(file_size)
            },
        );
    }

    fn log_directory_listing(
        &self,
        dir_name: &CmdStringArg,
        file_name: &FileNameString,
        file_size: FwSizeType,
    ) {
        let dir = to_log_string(dir_name);
        let mut file = LogStringArg::new();
        file.set_bytes(file_name.as_bytes());
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_DIRECTORY_LISTING,
            LogSeverity::ActivityHi,
            &format!("Directory {dir} entry {file} size {file_size}"),
            |buf| {
                fw_try!(dir.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(file.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u64_be(file_size)
            },
        );
    }

    fn log_directory_listing_subdir(&self, dir_name: &CmdStringArg, sub_dir: &FileNameString) {
        let dir = to_log_string(dir_name);
        let mut sub = LogStringArg::new();
        sub.set_bytes(sub_dir.as_bytes());
        self.log_two_names(
            Self::EVENTID_DIRECTORY_LISTING_SUBDIR,
            LogSeverity::ActivityHi,
            &dir,
            &sub,
            "Directory",
        );
    }

    fn log_file_name_format_error(&self, file_name: &FileNameString, status: StringFormatStatus) {
        let mut name = LogStringArg::new();
        name.set_bytes(file_name.as_bytes());
        let code = status.as_repr();
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_NAME_FORMAT_ERROR,
            LogSeverity::WarningHi,
            &format!("Could not format the path of {name}: status {code}"),
            |buf| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u8(code, Endianness::Big)
            },
        );
    }
}

/// C++ constructs a `Fw::LogStringArg` from the command string.
fn to_log_string(value: &CmdStringArg) -> LogStringArg {
    let mut out = LogStringArg::new();
    out.set_bytes(value.as_bytes());
    out
}

/// C++ constructs a `Fw::LogStringArg` from the stored `Fw::String` file
/// name (`GenerateDp` events after the command handler returned).
fn log_string_of(value: &FileNameString) -> LogStringArg {
    let mut out = LogStringArg::new();
    out.set_bytes(value.as_bytes());
    out
}

/// C++ `fullPath.format("%s/%s", dirName, filename)` into an `Fw::String`
/// (capacity `FW_FIXED_LENGTH_STRING_SIZE` = 256). The `/` is appended even
/// when the directory name already ends with one (a double slash — harmless
/// for `stat`, reproduced for byte-identical event text).
fn format_full_path(
    dir_name: &CmdStringArg,
    file_name: &FileNameString,
) -> (FwDefaultString, StringFormatStatus) {
    let needed = dir_name.len() + 1 + file_name.len();
    let mut out = FwDefaultString::new();
    if needed > FwDefaultString::max_length() {
        return (out, StringFormatStatus::Overflowed);
    }
    let mut bytes = Vec::with_capacity(needed);
    bytes.extend_from_slice(dir_name.as_bytes());
    bytes.push(b'/');
    bytes.extend_from_slice(file_name.as_bytes());
    out.set_bytes(&bytes);
    (out, StringFormatStatus::Success)
}

// -- Sync port, implemented directly on the component ------------------------

impl SchedPort for FileManager {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        self.sched_handler(port_num, context);
    }
}

// -- Async input adapters ---------------------------------------------------

struct PingInAdapter {
    comp: Arc<FileManager>,
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
    comp: Arc<FileManager>,
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

impl ComponentDispatch for FileManager {
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
            MSG_TYPE_PING_IN => {
                let mut key = 0u32;
                if !buf.deserialize_u32_be(&mut key).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.ping_handler(port_num, key);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_RUN_INTERNAL => {
                self.run_internal_handler();
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
                    Self::OPCODE_CREATE_DIRECTORY => {
                        self.create_directory_cmd_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_MOVE_FILE => {
                        self.move_file_cmd_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_REMOVE_DIRECTORY => {
                        self.remove_directory_cmd_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_REMOVE_FILE => {
                        self.remove_file_cmd_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_APPEND_FILE => {
                        self.append_file_cmd_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_FILE_SIZE => {
                        self.file_size_cmd_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_LIST_DIRECTORY => {
                        self.list_directory_cmd_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_CALCULATE_CRC => {
                        self.calculate_crc_cmd_handler(op_code, cmd_seq, &mut args)
                    }
                    Self::OPCODE_GENERATE_DP => {
                        self.generate_dp_cmd_handler(op_code, cmd_seq, &mut args)
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

impl ActiveComponent for FileManager {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::queued::MsgDispatchStatus as DispatchStatus;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, TlmPort};
    use fprime_fw::{LogBuffer, Serialize, SerializeStatus, Time, TlmBuffer};
    use std::path::PathBuf;
    use std::sync::atomic::AtomicU32;

    const ID_BASE: FwIdType = 0x0500_2000;

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    /// Command string arguments are `Fw::CmdStringArg` (40 bytes), so test
    /// paths must stay short.
    fn temp_dir() -> PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "fpm{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&path).unwrap();
        path
    }

    fn path_str(path: &std::path::Path) -> String {
        path.to_str().unwrap().to_string()
    }

    #[derive(Default)]
    struct Ground {
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        regs: Mutex<Vec<FwOpcodeType>>,
        pings: Mutex<Vec<u32>>,
        /// `(id, packet bytes)` of every container sent on `productSendOut`.
        dp_sent: Mutex<Vec<(FwDpIdType, Vec<u8>)>>,
        /// `(id, requested packet size)` of every `productGetOut` call.
        dp_gets: Mutex<Vec<(FwDpIdType, FwSizeType)>>,
        /// Number of `productGetOut` calls to satisfy before failing
        /// (`None` = never fail).
        dp_fail_after: Mutex<Option<usize>>,
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
        fn responses(&self) -> Vec<(FwOpcodeType, u32, CmdResponse)> {
            self.responses.lock().unwrap().clone()
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

    impl PingPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.pings.lock().unwrap().push(key);
        }
    }

    impl DpGetPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwDpIdType,
            data_size: FwSizeType,
            buffer: &mut Buffer,
        ) -> Success {
            let mut gets = self.dp_gets.lock().unwrap();
            gets.push((id, data_size));
            if self
                .dp_fail_after
                .lock()
                .unwrap()
                .is_some_and(|limit| gets.len() > limit)
            {
                return Success::Failure;
            }
            *buffer = Buffer::allocate(data_size as usize);
            Success::Success
        }
    }

    impl DpSendPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, id: FwDpIdType, buffer: Buffer) {
            self.dp_sent
                .lock()
                .unwrap()
                .push((id, buffer.data().to_vec()));
        }
    }

    fn setup() -> (Arc<FileManager>, Arc<Ground>) {
        let comp = FileManager::new("fileManager");
        let ground = Arc::new(Ground::default());
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.ping_out.connect(ground.clone(), 0);
        comp.init(64);
        (comp, ground)
    }

    /// `setup()` plus the two data-product ports wired to the ground stub.
    fn setup_with_dp() -> (Arc<FileManager>, Arc<Ground>) {
        let (comp, ground) = setup();
        comp.product_get_out.connect(ground.clone(), 0);
        comp.product_send_out.connect(ground.clone(), 0);
        (comp, ground)
    }

    fn dispatch(comp: &Arc<FileManager>) {
        let status = comp
            .active
            .queued
            .dispatch_available_messages(comp.as_ref());
        assert!(status == DispatchStatus::Ok || status == DispatchStatus::Empty);
    }

    fn send_cmd(
        comp: &Arc<FileManager>,
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

    fn one_string(value: &str) -> impl FnOnce(&mut CmdArgBuffer) + '_ {
        move |args: &mut CmdArgBuffer| {
            let s = CmdStringArg::from(value);
            assert_eq!(s.serialize_to(args, Endianness::Big), SerializeStatus::Ok);
        }
    }

    fn two_strings<'a>(a: &'a str, b: &'a str) -> impl FnOnce(&mut CmdArgBuffer) + 'a {
        move |args: &mut CmdArgBuffer| {
            let first = CmdStringArg::from(a);
            let second = CmdStringArg::from(b);
            assert_eq!(
                first.serialize_to(args, Endianness::Big),
                SerializeStatus::Ok
            );
            assert_eq!(
                second.serialize_to(args, Endianness::Big),
                SerializeStatus::Ok
            );
        }
    }

    fn sched_tick(comp: &Arc<FileManager>) {
        let p = comp.sched_in(0);
        p.target.invoke(p.port_num, 0);
        dispatch(comp);
    }

    // ---- registration -----------------------------------------------------

    #[test]
    fn reg_commands_registers_nine_opcodes_with_the_0x04_gap() {
        let (comp, ground) = setup();
        comp.reg_commands();
        let expected: Vec<FwOpcodeType> = [0x00, 0x01, 0x02, 0x03, 0x05, 0x06, 0x07, 0x08, 0x09]
            .iter()
            .map(|op| ID_BASE + op)
            .collect();
        assert_eq!(ground.regs.lock().unwrap().as_slice(), expected.as_slice());
    }

    // ---- CreateDirectory / RemoveDirectory --------------------------------

    #[test]
    fn create_directory_succeeds_and_counts_a_command() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let target = path_str(&dir.join("sub"));
        send_cmd(
            &comp,
            FileManager::OPCODE_CREATE_DIRECTORY,
            1,
            one_string(&target),
        );
        assert!(std::path::Path::new(&target).is_dir());
        assert_eq!(
            ground.event_ids(),
            vec![
                FileManager::EVENTID_CREATE_DIRECTORY_STARTED,
                FileManager::EVENTID_CREATE_DIRECTORY_SUCCEEDED
            ]
        );
        assert_eq!(
            ground.last_tlm(FileManager::CHANID_COMMANDS_EXECUTED),
            Some(1)
        );
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), None);
        assert_eq!(ground.responses(), vec![(ID_BASE, 1, CmdResponse::Ok)]);
    }

    #[test]
    fn create_directory_on_an_existing_directory_is_an_error() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let target = path_str(&dir);
        send_cmd(
            &comp,
            FileManager::OPCODE_CREATE_DIRECTORY,
            2,
            one_string(&target),
        );
        let errors = ground.events_of(FileManager::EVENTID_DIRECTORY_CREATE_ERROR);
        assert_eq!(errors.len(), 1);
        assert_eq!(errors[0].0, LogSeverity::WarningHi);
        // The last four bytes are the FileSystem status as U32.
        let args = &errors[0].1;
        assert_eq!(
            &args[args.len() - 4..],
            &(filesystem::Status::AlreadyExists as u32).to_be_bytes()
        );
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), Some(1));
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE, 2, CmdResponse::ExecutionError)]
        );
    }

    #[test]
    fn remove_directory_succeeds() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let target = path_str(&dir.join("gone"));
        std::fs::create_dir(&target).unwrap();
        send_cmd(
            &comp,
            FileManager::OPCODE_REMOVE_DIRECTORY,
            1,
            one_string(&target),
        );
        assert!(!std::path::Path::new(&target).exists());
        assert_eq!(
            ground.event_ids(),
            vec![
                FileManager::EVENTID_REMOVE_DIRECTORY_STARTED,
                FileManager::EVENTID_REMOVE_DIRECTORY_SUCCEEDED
            ]
        );
        assert_eq!(ground.responses(), vec![(ID_BASE + 2, 1, CmdResponse::Ok)]);
    }

    #[test]
    fn remove_directory_error_is_reported() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let missing = path_str(&dir.join("nope"));
        send_cmd(
            &comp,
            FileManager::OPCODE_REMOVE_DIRECTORY,
            1,
            one_string(&missing),
        );
        assert_eq!(
            ground
                .events_of(FileManager::EVENTID_DIRECTORY_REMOVE_ERROR)
                .len(),
            1
        );
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), Some(1));
    }

    // ---- RemoveFile -------------------------------------------------------

    #[test]
    fn remove_file_succeeds() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let file = dir.join("f.bin");
        std::fs::write(&file, b"x").unwrap();
        let target = path_str(&file);
        send_cmd(&comp, FileManager::OPCODE_REMOVE_FILE, 1, |args| {
            let s = CmdStringArg::from(target.as_str());
            let _ = s.serialize_to(args, Endianness::Big);
            let _ = args.serialize_bool_be(false);
        });
        assert!(!file.exists());
        assert_eq!(
            ground.event_ids(),
            vec![
                FileManager::EVENTID_REMOVE_FILE_STARTED,
                FileManager::EVENTID_REMOVE_FILE_SUCCEEDED
            ]
        );
        assert_eq!(
            ground.last_tlm(FileManager::CHANID_COMMANDS_EXECUTED),
            Some(1)
        );
    }

    #[test]
    fn remove_file_error_without_ignore_answers_execution_error() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let target = path_str(&dir.join("missing.bin"));
        send_cmd(&comp, FileManager::OPCODE_REMOVE_FILE, 3, |args| {
            let s = CmdStringArg::from(target.as_str());
            let _ = s.serialize_to(args, Endianness::Big);
            let _ = args.serialize_bool_be(false);
        });
        assert_eq!(
            ground
                .events_of(FileManager::EVENTID_FILE_REMOVE_ERROR)
                .len(),
            1
        );
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), Some(1));
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE + 3, 3, CmdResponse::ExecutionError)]
        );
    }

    /// GOTCHA: `ignoreErrors = true` takes an early return that counts the
    /// error but answers OK and never touches CommandsExecuted.
    #[test]
    fn remove_file_with_ignore_errors_counts_an_error_and_answers_ok() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let target = path_str(&dir.join("missing.bin"));
        send_cmd(&comp, FileManager::OPCODE_REMOVE_FILE, 4, |args| {
            let s = CmdStringArg::from(target.as_str());
            let _ = s.serialize_to(args, Endianness::Big);
            let _ = args.serialize_bool_be(true);
        });
        assert_eq!(
            ground
                .events_of(FileManager::EVENTID_FILE_REMOVE_ERROR)
                .len(),
            1
        );
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), Some(1));
        assert_eq!(ground.last_tlm(FileManager::CHANID_COMMANDS_EXECUTED), None);
        assert_eq!(ground.responses(), vec![(ID_BASE + 3, 4, CmdResponse::Ok)]);
    }

    #[test]
    fn remove_file_rejects_a_non_boolean_ignore_flag() {
        let (comp, ground) = setup();
        send_cmd(&comp, FileManager::OPCODE_REMOVE_FILE, 5, |args| {
            let s = CmdStringArg::from("/tmp/x");
            let _ = s.serialize_to(args, Endianness::Big);
            // Strict bool decode: only 0xFF/0x00 are valid.
            let _ = args.serialize_u8(0x01, Endianness::Big);
        });
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE + 3, 5, CmdResponse::FormatError)]
        );
    }

    // ---- MoveFile / AppendFile / FileSize ---------------------------------

    #[test]
    fn move_file_moves_the_content() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let src = dir.join("a.bin");
        let dst = dir.join("b.bin");
        std::fs::write(&src, b"payload").unwrap();
        send_cmd(
            &comp,
            FileManager::OPCODE_MOVE_FILE,
            1,
            two_strings(&path_str(&src), &path_str(&dst)),
        );
        assert!(!src.exists());
        assert_eq!(std::fs::read(&dst).unwrap(), b"payload");
        assert_eq!(
            ground.event_ids(),
            vec![
                FileManager::EVENTID_MOVE_FILE_STARTED,
                FileManager::EVENTID_MOVE_FILE_SUCCEEDED
            ]
        );
    }

    #[test]
    fn move_file_error_is_reported() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        send_cmd(
            &comp,
            FileManager::OPCODE_MOVE_FILE,
            1,
            two_strings(&path_str(&dir.join("none")), &path_str(&dir.join("x"))),
        );
        assert_eq!(
            ground.events_of(FileManager::EVENTID_FILE_MOVE_ERROR).len(),
            1
        );
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), Some(1));
    }

    #[test]
    fn append_file_concatenates() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let src = dir.join("s.bin");
        let dst = dir.join("d.bin");
        std::fs::write(&src, b"BBB").unwrap();
        std::fs::write(&dst, b"AAA").unwrap();
        send_cmd(
            &comp,
            FileManager::OPCODE_APPEND_FILE,
            1,
            two_strings(&path_str(&src), &path_str(&dst)),
        );
        assert_eq!(std::fs::read(&dst).unwrap(), b"AAABBB");
        assert_eq!(
            ground.event_ids(),
            vec![
                FileManager::EVENTID_APPEND_FILE_STARTED,
                FileManager::EVENTID_APPEND_FILE_SUCCEEDED
            ]
        );
        assert_eq!(
            ground.last_tlm(FileManager::CHANID_COMMANDS_EXECUTED),
            Some(1)
        );
    }

    #[test]
    fn file_size_reports_a_u64_size() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let file = dir.join("f.bin");
        std::fs::write(&file, vec![0u8; 300]).unwrap();
        send_cmd(
            &comp,
            FileManager::OPCODE_FILE_SIZE,
            1,
            one_string(&path_str(&file)),
        );
        let events = ground.events_of(FileManager::EVENTID_FILE_SIZE_SUCCEEDED);
        assert_eq!(events.len(), 1);
        let args = &events[0].1;
        assert_eq!(&args[args.len() - 8..], &300u64.to_be_bytes());
        assert_eq!(ground.responses(), vec![(ID_BASE + 6, 1, CmdResponse::Ok)]);
    }

    #[test]
    fn file_size_error_is_reported() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        send_cmd(
            &comp,
            FileManager::OPCODE_FILE_SIZE,
            2,
            one_string(&path_str(&dir.join("nope"))),
        );
        assert_eq!(
            ground.events_of(FileManager::EVENTID_FILE_SIZE_ERROR).len(),
            1
        );
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), Some(1));
    }

    // ---- CalculateCrc -----------------------------------------------------

    /// GOTCHA: CalculateCrc never touches CommandsExecuted/Errors.
    #[test]
    fn calculate_crc_succeeds_without_touching_the_counters() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        let file = dir.join("c.bin");
        std::fs::write(&file, b"123456789").unwrap();
        send_cmd(
            &comp,
            FileManager::OPCODE_CALCULATE_CRC,
            1,
            one_string(&path_str(&file)),
        );
        let events = ground.events_of(FileManager::EVENTID_CALCULATE_CRC_SUCCEEDED);
        assert_eq!(events.len(), 1);
        // Os::File::calculateCrc returns the UN-complemented register:
        // !crc32("123456789") = !0xCBF43926.
        let args = &events[0].1;
        assert_eq!(&args[args.len() - 4..], &(!0xCBF4_3926u32).to_be_bytes());
        assert_eq!(ground.responses(), vec![(ID_BASE + 8, 1, CmdResponse::Ok)]);
        assert_eq!(ground.last_tlm(FileManager::CHANID_COMMANDS_EXECUTED), None);
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), None);
    }

    #[test]
    fn calculate_crc_failure_answers_execution_error_without_counters() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        send_cmd(
            &comp,
            FileManager::OPCODE_CALCULATE_CRC,
            2,
            one_string(&path_str(&dir.join("missing"))),
        );
        assert_eq!(
            ground
                .events_of(FileManager::EVENTID_CALCULATE_CRC_FAILED)
                .len(),
            1
        );
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE + 8, 2, CmdResponse::ExecutionError)]
        );
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), None);
    }

    // ---- ListDirectory ----------------------------------------------------

    #[test]
    fn a_listed_entry_name_is_clipped_to_the_log_string_cap() {
        // C++ parity: the event args are declared `string size
        // FileNameStringSize` (240) but the generated `log_*` method carries
        // them as `Fw::LogStringArg` = `StringTemplate<FW_LOG_STRING_MAX_SIZE>`,
        // so anything past 200 bytes never reaches the wire.
        assert_eq!(EVENT_STRING_SIZE, FW_LOG_STRING_MAX_SIZE);
        const { assert!(EVENT_STRING_SIZE < FILE_NAME_STRING_SIZE) };

        let (comp, ground) = setup();
        let dir = temp_dir();
        let long_name = format!("{}.bin", "z".repeat(216));
        assert_eq!(long_name.len(), 220);
        std::fs::write(dir.join(&long_name), b"12345").unwrap();

        send_cmd(
            &comp,
            FileManager::OPCODE_LIST_DIRECTORY,
            9,
            one_string(&path_str(&dir)),
        );
        sched_tick(&comp);

        let listed = ground.events_of(FileManager::EVENTID_DIRECTORY_LISTING);
        assert_eq!(listed.len(), 1);
        let args = &listed[0].1;
        // dir: 2-byte length then bytes; the entry name follows.
        let dir_len = u16::from_be_bytes([args[0], args[1]]) as usize;
        let file_off = 2 + dir_len;
        assert_eq!(
            &args[file_off..file_off + 2],
            &(FW_LOG_STRING_MAX_SIZE as u16).to_be_bytes()
        );
        assert_eq!(
            &args[file_off + 2..file_off + 2 + FW_LOG_STRING_MAX_SIZE],
            &long_name.as_bytes()[..FW_LOG_STRING_MAX_SIZE]
        );
        // ... then the u64 size and nothing more.
        assert_eq!(args.len(), file_off + 2 + FW_LOG_STRING_MAX_SIZE + 8);
    }

    #[test]
    fn list_directory_is_paced_one_entry_per_tick() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        std::fs::write(dir.join("a.bin"), b"12345").unwrap();
        std::fs::write(dir.join("b.bin"), b"1").unwrap();
        std::fs::create_dir(dir.join("sub")).unwrap();

        send_cmd(
            &comp,
            FileManager::OPCODE_LIST_DIRECTORY,
            9,
            one_string(&path_str(&dir)),
        );
        // Started, but no response yet.
        assert_eq!(
            ground.event_ids(),
            vec![FileManager::EVENTID_LIST_DIRECTORY_STARTED]
        );
        assert!(ground.responses().is_empty());

        // FILES_PER_RATE_TICK = 1, so one entry per tick.
        for expected in 1..=3 {
            sched_tick(&comp);
            let listed = ground
                .events_of(FileManager::EVENTID_DIRECTORY_LISTING)
                .len()
                + ground
                    .events_of(FileManager::EVENTID_DIRECTORY_LISTING_SUBDIR)
                    .len();
            assert_eq!(listed, expected);
            assert!(ground.responses().is_empty());
        }
        // The next tick hits NO_MORE_FILES and completes the command.
        sched_tick(&comp);
        let done = ground.events_of(FileManager::EVENTID_LIST_DIRECTORY_SUCCEEDED);
        assert_eq!(done.len(), 1);
        assert_eq!(&done[0].1[done[0].1.len() - 4..], &3u32.to_be_bytes());
        assert_eq!(ground.responses(), vec![(ID_BASE + 7, 9, CmdResponse::Ok)]);
        assert_eq!(
            ground.last_tlm(FileManager::CHANID_COMMANDS_EXECUTED),
            Some(1)
        );
        // Two files and one subdirectory.
        assert_eq!(
            ground
                .events_of(FileManager::EVENTID_DIRECTORY_LISTING)
                .len(),
            2
        );
        assert_eq!(
            ground
                .events_of(FileManager::EVENTID_DIRECTORY_LISTING_SUBDIR)
                .len(),
            1
        );
        // Sizes are reported as FwSizeType (u64) and 5 must appear.
        let sizes: Vec<u64> = ground
            .events_of(FileManager::EVENTID_DIRECTORY_LISTING)
            .iter()
            .map(|e| {
                let a = &e.1;
                u64::from_be_bytes(a[a.len() - 8..].try_into().unwrap())
            })
            .collect();
        assert!(sizes.contains(&5) && sizes.contains(&1), "{sizes:?}");
    }

    #[test]
    fn a_second_list_directory_is_rejected_with_a_directory_status() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        std::fs::write(dir.join("a.bin"), b"x").unwrap();
        let target = path_str(&dir);
        send_cmd(
            &comp,
            FileManager::OPCODE_LIST_DIRECTORY,
            1,
            one_string(&target),
        );
        send_cmd(
            &comp,
            FileManager::OPCODE_LIST_DIRECTORY,
            2,
            one_string(&target),
        );
        let errors = ground.events_of(FileManager::EVENTID_LIST_DIRECTORY_ERROR);
        assert_eq!(errors.len(), 1);
        // Os::Directory::OTHER_ERROR = 10 (a DIRECTORY status, not a
        // FileSystem one).
        assert_eq!(
            &errors[0].1[errors[0].1.len() - 4..],
            &(DirStatus::OtherError as u32).to_be_bytes()
        );
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE + 7, 2, CmdResponse::ExecutionError)]
        );
        // emitTelemetry got a FileSystem status, so Errors moved.
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), Some(1));
    }

    #[test]
    fn list_directory_open_failure_answers_immediately() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        send_cmd(
            &comp,
            FileManager::OPCODE_LIST_DIRECTORY,
            1,
            one_string(&path_str(&dir.join("missing"))),
        );
        assert_eq!(
            ground
                .events_of(FileManager::EVENTID_LIST_DIRECTORY_ERROR)
                .len(),
            1
        );
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE + 7, 1, CmdResponse::ExecutionError)]
        );
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), Some(1));
    }

    #[test]
    fn sched_ticks_are_gated_by_the_run_queued_flag() {
        let (comp, ground) = setup();
        let dir = temp_dir();
        for i in 0..3 {
            std::fs::write(dir.join(format!("f{i}.bin")), b"x").unwrap();
        }
        send_cmd(
            &comp,
            FileManager::OPCODE_LIST_DIRECTORY,
            1,
            one_string(&path_str(&dir)),
        );
        // Two schedIn calls before any dispatch enqueue only ONE internal
        // message (the atomic gate), so only one entry is listed.
        let p = comp.sched_in(0);
        p.target.invoke(p.port_num, 0);
        p.target.invoke(p.port_num, 0);
        dispatch(&comp);
        assert_eq!(
            ground
                .events_of(FileManager::EVENTID_DIRECTORY_LISTING)
                .len(),
            1
        );
        // The gate reopened after the run handler executed.
        sched_tick(&comp);
        assert_eq!(
            ground
                .events_of(FileManager::EVENTID_DIRECTORY_LISTING)
                .len(),
            2
        );
    }

    #[test]
    fn a_sched_tick_with_no_listing_in_progress_is_harmless() {
        let (comp, ground) = setup();
        sched_tick(&comp);
        sched_tick(&comp);
        assert!(ground.event_ids().is_empty());
    }

    // ---- GenerateDp -------------------------------------------------------

    #[test]
    fn generate_dp_reports_a_buffer_failure_when_the_dp_ports_are_unconnected() {
        let (comp, ground) = setup();
        send_cmd(&comp, FileManager::OPCODE_GENERATE_DP, 1, |args| {
            let s = CmdStringArg::from("/tmp/x.bin");
            let _ = s.serialize_to(args, Endianness::Big);
            let _ = args.serialize_u32_be(64);
            let _ = args.serialize_u64_be(0);
            let _ = args.serialize_u64_be(0);
            let _ = args.serialize_u32_be(0);
            let _ = args.serialize_i32_be(GenerateDpMode::Immediate.as_repr());
        });
        assert_eq!(
            ground.event_ids(),
            vec![FileManager::EVENTID_GENERATE_DP_BUFFER_FAILED]
        );
        assert_eq!(ground.responses(), vec![(ID_BASE + 9, 1, CmdResponse::Ok)]);
        assert_eq!(ground.last_tlm(FileManager::CHANID_COMMANDS_EXECUTED), None);
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), None);
    }

    // ---- GenerateDp: data products ------------------------------------------

    /// 100 bytes of deterministic content (the upstream unit test's file).
    fn dp_source_file(dir: &std::path::Path) -> (String, Vec<u8>) {
        let content: Vec<u8> = b"0123456789".repeat(10);
        let path = dir.join("dp.bin");
        std::fs::write(&path, &content).unwrap();
        (path_str(&path), content)
    }

    /// Send `GenerateDp(path, chunk_size, range.0, range.1, priority, mode)`.
    fn generate_dp(
        comp: &Arc<FileManager>,
        cmd_seq: u32,
        path: &str,
        chunk_size: u32,
        range: (u64, u64),
        priority: u32,
        mode: GenerateDpMode,
    ) {
        let (begin, end) = range;
        send_cmd(comp, FileManager::OPCODE_GENERATE_DP, cmd_seq, |args| {
            let s = CmdStringArg::from(path);
            assert_eq!(s.serialize_to(args, Endianness::Big), SerializeStatus::Ok);
            assert!(args.serialize_u32_be(chunk_size).is_ok());
            assert!(args.serialize_u64_be(begin).is_ok());
            assert!(args.serialize_u64_be(end).is_ok());
            assert!(args.serialize_u32_be(priority).is_ok());
            assert!(args.serialize_i32_be(mode.as_repr()).is_ok());
        });
    }

    /// The exact record bytes one chunk container must carry:
    /// `[hdr id][u16 len][name][u64 offset][u32 size][data id][u16 n][bytes]`.
    fn expected_chunk_records(path: &str, offset: u64, data: &[u8]) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&(ID_BASE + RECORD_ID_FILE_CHUNK_HEADER).to_be_bytes());
        v.extend_from_slice(&(path.len() as u16).to_be_bytes());
        v.extend_from_slice(path.as_bytes());
        v.extend_from_slice(&offset.to_be_bytes());
        v.extend_from_slice(&(data.len() as u32).to_be_bytes());
        v.extend_from_slice(&(ID_BASE + RECORD_ID_FILE_CHUNK_DATA).to_be_bytes());
        v.extend_from_slice(&(data.len() as u16).to_be_bytes());
        v.extend_from_slice(data);
        v
    }

    /// Parse a sent packet back into a container (header + valid data).
    fn container_of(packet: &[u8]) -> DpContainer {
        let buffer = Buffer::from_storage(packet.to_vec().into_boxed_slice(), 0);
        let mut container = DpContainer::with_buffer(0, buffer);
        assert_eq!(container.deserialize_header(), SerializeStatus::Ok);
        container
    }

    fn name_arg(name: &str) -> Vec<u8> {
        let mut v = (name.len() as u16).to_be_bytes().to_vec();
        v.extend_from_slice(name.as_bytes());
        v
    }

    #[test]
    fn generate_dp_immediate_emits_one_container_per_chunk_with_byte_exact_records() {
        let (comp, ground) = setup_with_dp();
        let dir = temp_dir();
        let (path, content) = dp_source_file(&dir);

        generate_dp(&comp, 7, &path, 40, (0, 0), 3, GenerateDpMode::Immediate);

        // Three chunks: 40 + 40 + 20.
        let sent = ground.dp_sent.lock().unwrap().clone();
        assert_eq!(sent.len(), 3);
        for (i, (id, packet)) in sent.iter().enumerate() {
            assert_eq!(*id, ID_BASE + CONTAINER_ID_FILE_DP);
            let container = container_of(packet);
            assert_eq!(container.id(), ID_BASE + CONTAINER_ID_FILE_DP);
            assert_eq!(container.priority(), 3);
            let offset = (i * 40) as u64;
            let end = ((i + 1) * 40).min(content.len());
            assert_eq!(
                container.data(),
                &expected_chunk_records(&path, offset, &content[i * 40..end])[..],
                "chunk {i}"
            );
        }
        // Every buffer request asked for the packet holding the MAXIMUM
        // header record plus this chunk's data record (autocoder sizing).
        let gets = ground.dp_gets.lock().unwrap().clone();
        assert_eq!(gets.len(), 3);
        assert_eq!(
            gets[0].1,
            DpContainer::packet_size_for_data_size(
                SIZE_OF_FILE_CHUNK_HEADER_RECORD + size_of_file_chunk_data_record(40)
            )
        );
        assert_eq!(
            gets[2].1,
            DpContainer::packet_size_for_data_size(
                SIZE_OF_FILE_CHUNK_HEADER_RECORD + size_of_file_chunk_data_record(20)
            )
        );

        // Started(name, 100) then Complete(name, 3); one OK response; the
        // command counters are untouched.
        assert_eq!(
            ground.event_ids(),
            vec![
                FileManager::EVENTID_GENERATE_DP_STARTED,
                FileManager::EVENTID_GENERATE_DP_COMPLETE
            ]
        );
        let started = ground.events_of(FileManager::EVENTID_GENERATE_DP_STARTED);
        let mut expected = name_arg(&path);
        expected.extend_from_slice(&100u64.to_be_bytes());
        assert_eq!(started[0], (LogSeverity::ActivityHi, expected));
        let complete = ground.events_of(FileManager::EVENTID_GENERATE_DP_COMPLETE);
        let mut expected = name_arg(&path);
        expected.extend_from_slice(&3u32.to_be_bytes());
        assert_eq!(complete[0], (LogSeverity::ActivityHi, expected));
        assert_eq!(ground.responses(), vec![(ID_BASE + 9, 7, CmdResponse::Ok)]);
        assert_eq!(ground.last_tlm(FileManager::CHANID_COMMANDS_EXECUTED), None);
        assert_eq!(ground.last_tlm(FileManager::CHANID_ERRORS), None);
    }

    #[test]
    fn generate_dp_paced_emits_one_chunk_per_tick_and_defers_the_response() {
        let (comp, ground) = setup_with_dp();
        let dir = temp_dir();
        let (path, content) = dp_source_file(&dir);

        generate_dp(&comp, 1, &path, 40, (0, 0), 0, GenerateDpMode::Paced);
        // Nothing emitted yet, and no response.
        assert_eq!(
            ground.event_ids(),
            vec![FileManager::EVENTID_GENERATE_DP_STARTED]
        );
        assert!(ground.dp_sent.lock().unwrap().is_empty());
        assert!(ground.responses().is_empty());

        sched_tick(&comp);
        assert_eq!(ground.dp_sent.lock().unwrap().len(), 1);
        assert!(ground.responses().is_empty());
        sched_tick(&comp);
        assert_eq!(ground.dp_sent.lock().unwrap().len(), 2);
        assert!(ground.responses().is_empty());
        sched_tick(&comp);
        let sent = ground.dp_sent.lock().unwrap().clone();
        assert_eq!(sent.len(), 3);
        assert_eq!(
            container_of(&sent[2].1).data(),
            &expected_chunk_records(&path, 80, &content[80..])[..]
        );
        assert_eq!(
            ground.event_ids(),
            vec![
                FileManager::EVENTID_GENERATE_DP_STARTED,
                FileManager::EVENTID_GENERATE_DP_COMPLETE
            ]
        );
        assert_eq!(ground.responses(), vec![(ID_BASE + 9, 1, CmdResponse::Ok)]);

        // Further ticks do nothing.
        sched_tick(&comp);
        assert_eq!(ground.dp_sent.lock().unwrap().len(), 3);
        assert_eq!(ground.responses().len(), 1);
    }

    #[test]
    fn generate_dp_rejects_a_second_request_while_one_is_in_progress() {
        let (comp, ground) = setup_with_dp();
        let dir = temp_dir();
        let (path, _) = dp_source_file(&dir);

        generate_dp(&comp, 1, &path, 40, (0, 0), 0, GenerateDpMode::Paced);
        generate_dp(&comp, 2, &path, 40, (0, 0), 0, GenerateDpMode::Immediate);

        // The second answers OK immediately with Failed(BUSY, 0)...
        assert_eq!(ground.responses(), vec![(ID_BASE + 9, 2, CmdResponse::Ok)]);
        let failed = ground.events_of(FileManager::EVENTID_GENERATE_DP_FAILED);
        let mut expected = name_arg(&path);
        expected.extend_from_slice(&GenerateDpStage::Busy.as_repr().to_be_bytes());
        expected.extend_from_slice(&0u32.to_be_bytes());
        assert_eq!(failed, vec![(LogSeverity::WarningHi, expected)]);
        // ...and the first keeps going.
        sched_tick(&comp);
        sched_tick(&comp);
        sched_tick(&comp);
        assert_eq!(ground.dp_sent.lock().unwrap().len(), 3);
        assert_eq!(
            ground.responses(),
            vec![
                (ID_BASE + 9, 2, CmdResponse::Ok),
                (ID_BASE + 9, 1, CmdResponse::Ok)
            ]
        );
    }

    #[test]
    fn generate_dp_clamps_the_chunk_size_and_treats_end_zero_as_end_of_file() {
        let (comp, ground) = setup_with_dp();
        let dir = temp_dir();
        let (path, content) = dp_source_file(&dir);

        // chunkSize 0 -> GENERATE_DP_MAX_CHUNK_SIZE (1024): one chunk.
        generate_dp(&comp, 1, &path, 0, (0, 0), 0, GenerateDpMode::Immediate);
        let sent = ground.dp_sent.lock().unwrap().clone();
        assert_eq!(sent.len(), 1);
        assert_eq!(
            container_of(&sent[0].1).data(),
            &expected_chunk_records(&path, 0, &content)[..]
        );

        // chunkSize above the maximum is clamped the same way; an end offset
        // past the file is the end of the file.
        generate_dp(
            &comp,
            2,
            &path,
            5000,
            (0, 999),
            0,
            GenerateDpMode::Immediate,
        );
        let sent = ground.dp_sent.lock().unwrap().clone();
        assert_eq!(sent.len(), 2);
        assert_eq!(
            container_of(&sent[1].1).data(),
            &expected_chunk_records(&path, 0, &content)[..]
        );
    }

    #[test]
    fn generate_dp_honors_a_partial_range() {
        let (comp, ground) = setup_with_dp();
        let dir = temp_dir();
        let (path, content) = dp_source_file(&dir);

        generate_dp(&comp, 1, &path, 100, (10, 50), 0, GenerateDpMode::Immediate);
        let sent = ground.dp_sent.lock().unwrap().clone();
        assert_eq!(sent.len(), 1);
        assert_eq!(
            container_of(&sent[0].1).data(),
            &expected_chunk_records(&path, 10, &content[10..50])[..]
        );
        // Started reports the RANGE, not the file size.
        let started = ground.events_of(FileManager::EVENTID_GENERATE_DP_STARTED);
        let mut expected = name_arg(&path);
        expected.extend_from_slice(&40u64.to_be_bytes());
        assert_eq!(started[0].1, expected);
    }

    #[test]
    fn generate_dp_reports_an_invalid_range() {
        let (comp, ground) = setup_with_dp();
        let dir = temp_dir();
        let (path, _) = dp_source_file(&dir);

        // begin past the end of the file.
        generate_dp(&comp, 1, &path, 0, (200, 0), 0, GenerateDpMode::Immediate);
        // begin == end (an empty range in a non-empty file).
        generate_dp(&comp, 2, &path, 0, (50, 50), 0, GenerateDpMode::Immediate);
        // begin > end.
        generate_dp(&comp, 3, &path, 0, (60, 50), 0, GenerateDpMode::Immediate);

        let invalid = ground.events_of(FileManager::EVENTID_GENERATE_DP_INVALID_RANGE);
        assert_eq!(invalid.len(), 3);
        let mut expected = name_arg(&path);
        expected.extend_from_slice(&200u64.to_be_bytes());
        expected.extend_from_slice(&0u64.to_be_bytes());
        expected.extend_from_slice(&100u64.to_be_bytes());
        assert_eq!(invalid[0], (LogSeverity::WarningHi, expected));
        assert!(ground.dp_sent.lock().unwrap().is_empty());
        assert!(ground.dp_gets.lock().unwrap().is_empty());
        assert_eq!(
            ground.responses(),
            vec![
                (ID_BASE + 9, 1, CmdResponse::Ok),
                (ID_BASE + 9, 2, CmdResponse::Ok),
                (ID_BASE + 9, 3, CmdResponse::Ok)
            ]
        );
        // The component is idle again: a valid request works.
        generate_dp(&comp, 4, &path, 0, (0, 0), 0, GenerateDpMode::Immediate);
        assert_eq!(ground.dp_sent.lock().unwrap().len(), 1);
    }

    #[test]
    fn generate_dp_reports_an_open_failure_with_the_stage() {
        let (comp, ground) = setup_with_dp();
        let dir = temp_dir();
        let missing = path_str(&dir.join("missing.bin"));

        generate_dp(&comp, 1, &missing, 0, (0, 0), 0, GenerateDpMode::Immediate);

        let failed = ground.events_of(FileManager::EVENTID_GENERATE_DP_FAILED);
        assert_eq!(failed.len(), 1);
        let mut expected = name_arg(&missing);
        expected.extend_from_slice(&GenerateDpStage::Open.as_repr().to_be_bytes());
        expected.extend_from_slice(&(FileStatus::DoesntExist as u32).to_be_bytes());
        assert_eq!(failed[0], (LogSeverity::WarningHi, expected));
        assert_eq!(ground.responses(), vec![(ID_BASE + 9, 1, CmdResponse::Ok)]);
        assert!(ground.dp_gets.lock().unwrap().is_empty());
    }

    #[test]
    fn generate_dp_buffer_failure_mid_transfer_still_answers_ok_and_goes_idle() {
        let (comp, ground) = setup_with_dp();
        let dir = temp_dir();
        let (path, _) = dp_source_file(&dir);
        *ground.dp_fail_after.lock().unwrap() = Some(1);

        generate_dp(&comp, 1, &path, 40, (0, 0), 0, GenerateDpMode::Immediate);

        assert_eq!(ground.dp_sent.lock().unwrap().len(), 1);
        assert_eq!(
            ground.event_ids(),
            vec![
                FileManager::EVENTID_GENERATE_DP_STARTED,
                FileManager::EVENTID_GENERATE_DP_BUFFER_FAILED
            ]
        );
        assert_eq!(ground.responses(), vec![(ID_BASE + 9, 1, CmdResponse::Ok)]);

        // Idle again: the next request is not BUSY.
        *ground.dp_fail_after.lock().unwrap() = None;
        generate_dp(&comp, 2, &path, 40, (0, 0), 0, GenerateDpMode::Immediate);
        assert_eq!(ground.dp_sent.lock().unwrap().len(), 4);
        assert!(
            ground
                .events_of(FileManager::EVENTID_GENERATE_DP_FAILED)
                .is_empty()
        );
    }

    #[test]
    fn generate_dp_priority_zero_uses_the_configured_default() {
        let (comp, ground) = setup_with_dp();
        let dir = temp_dir();
        let (path, _) = dp_source_file(&dir);

        generate_dp(&comp, 1, &path, 0, (0, 0), 0, GenerateDpMode::Immediate);
        generate_dp(&comp, 2, &path, 0, (0, 0), 42, GenerateDpMode::Immediate);

        let sent = ground.dp_sent.lock().unwrap().clone();
        assert_eq!(container_of(&sent[0].1).priority(), DEFAULT_DP_PRIORITY);
        assert_eq!(container_of(&sent[1].1).priority(), 42);
    }

    #[test]
    fn generate_dp_of_an_empty_file_completes_with_zero_chunks() {
        let (comp, ground) = setup_with_dp();
        let dir = temp_dir();
        let path = path_str(&dir.join("empty.bin"));
        std::fs::write(&path, b"").unwrap();

        generate_dp(&comp, 1, &path, 0, (0, 0), 0, GenerateDpMode::Paced);

        assert_eq!(
            ground.event_ids(),
            vec![
                FileManager::EVENTID_GENERATE_DP_STARTED,
                FileManager::EVENTID_GENERATE_DP_COMPLETE
            ]
        );
        let complete = ground.events_of(FileManager::EVENTID_GENERATE_DP_COMPLETE);
        let mut expected = name_arg(&path);
        expected.extend_from_slice(&0u32.to_be_bytes());
        assert_eq!(complete[0].1, expected);
        assert!(ground.dp_sent.lock().unwrap().is_empty());
        assert_eq!(ground.responses(), vec![(ID_BASE + 9, 1, CmdResponse::Ok)]);
    }

    #[test]
    fn generate_dp_record_size_constants_match_the_autocoder() {
        // id (4) + string (2 + 240) + offset (8) + dataSize (4).
        assert_eq!(FileChunkHeader::SERIALIZED_SIZE, 2 + 240 + 8 + 4);
        assert_eq!(SIZE_OF_FILE_CHUNK_HEADER_RECORD, 4 + 254);
        // id (4) + count (2) + n.
        assert_eq!(size_of_file_chunk_data_record(0), 6);
        assert_eq!(size_of_file_chunk_data_record(1024), 1030);
    }

    #[test]
    fn generate_dp_rejects_an_invalid_mode_with_validation_error() {
        let (comp, ground) = setup();
        send_cmd(&comp, FileManager::OPCODE_GENERATE_DP, 2, |args| {
            let s = CmdStringArg::from("/tmp/x.bin");
            let _ = s.serialize_to(args, Endianness::Big);
            let _ = args.serialize_u32_be(0);
            let _ = args.serialize_u64_be(0);
            let _ = args.serialize_u64_be(0);
            let _ = args.serialize_u32_be(0);
            let _ = args.serialize_i32_be(7);
        });
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE + 9, 2, CmdResponse::ValidationError)]
        );
    }

    // ---- argument discipline ----------------------------------------------

    #[test]
    fn residual_argument_bytes_are_a_format_error() {
        let (comp, ground) = setup();
        send_cmd(&comp, FileManager::OPCODE_CREATE_DIRECTORY, 1, |args| {
            let s = CmdStringArg::from("/tmp/x");
            let _ = s.serialize_to(args, Endianness::Big);
            let _ = args.serialize_u8(0x00, Endianness::Big);
        });
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE, 1, CmdResponse::FormatError)]
        );
    }

    #[test]
    fn missing_arguments_are_a_format_error() {
        let (comp, ground) = setup();
        send_cmd(
            &comp,
            FileManager::OPCODE_MOVE_FILE,
            1,
            one_string("/tmp/a"),
        );
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE + 1, 1, CmdResponse::FormatError)]
        );
    }

    #[test]
    fn the_removed_shell_command_opcode_is_an_invalid_opcode() {
        let (comp, ground) = setup();
        // 0x04 is the deliberate gap left by the removed ShellCommand.
        send_cmd(&comp, 0x04, 1, |_args| {});
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE + 4, 1, CmdResponse::InvalidOpcode)]
        );
    }

    #[test]
    fn unknown_opcode_answers_invalid_opcode() {
        let (comp, ground) = setup();
        send_cmd(&comp, 0x55, 2, |_args| {});
        assert_eq!(
            ground.responses(),
            vec![(ID_BASE + 0x55, 2, CmdResponse::InvalidOpcode)]
        );
    }

    // ---- misc -------------------------------------------------------------

    #[test]
    fn ping_is_echoed() {
        let (comp, ground) = setup();
        let p = comp.ping_in(0);
        p.target.invoke(p.port_num, 0x1234);
        dispatch(&comp);
        assert_eq!(ground.pings.lock().unwrap().as_slice(), &[0x1234]);
    }

    #[test]
    fn full_path_formatting_reproduces_the_double_slash() {
        let dir = CmdStringArg::from("/data/");
        let file = FileNameString::from("x.bin");
        let (path, status) = format_full_path(&dir, &file);
        assert_eq!(status, StringFormatStatus::Success);
        assert_eq!(path.as_str(), Some("/data//x.bin"));
    }

    #[test]
    fn full_path_formatting_reports_overflow() {
        let dir = CmdStringArg::from("d".repeat(40).as_str());
        let file = FileNameString::from("f".repeat(240).as_str());
        let (path, status) = format_full_path(&dir, &file);
        assert_eq!(status, StringFormatStatus::Overflowed);
        assert!(path.is_empty());
    }

    #[test]
    fn generate_dp_enums_serialize_at_the_i32_width() {
        assert_eq!(GenerateDpStage::SERIALIZED_SIZE, 4);
        assert_eq!(GenerateDpMode::SERIALIZED_SIZE, 4);
        assert_eq!(GenerateDpStage::Busy.as_repr(), 5);
        assert_eq!(StringFormatStatus::SERIALIZED_SIZE, 1);
    }

    #[test]
    fn queue_message_size_fits_a_full_command_argument_buffer() {
        assert_eq!(QUEUE_MSG_SIZE, 522);
    }
}
