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
//! ## Gap: data products
//!
//! `GenerateDp` (0x09) needs `Fw/Dp` and the `Fw.DataProductSync` ports,
//! which are not ported yet (`fprime-fw::dp` and `Svc::DpManager` belong to
//! another wave). The command is implemented up to the point the C++ itself
//! reaches when the DP ports are unconnected: arguments are validated, then
//! `GenerateDpBufferFailed` is emitted and the command answers `OK` — a
//! behavior the C++ produces verbatim in that configuration. The paced
//! chunking loop is deliberately not stubbed; wire it up when data products
//! land.

use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, ComponentDispatch, EventGlue, MsgDispatchStatus,
    OutputPort, PingPort, PortRef, QueueFullPolicy, SchedPort, TlmGlue, msg,
};
use fprime_config::{
    FILE_NAME_STRING_SIZE, FW_CMD_ARG_BUFFER_MAX_SIZE, FW_LOG_STRING_MAX_SIZE, FwChanIdType,
    FwEnumStoreType, FwEventIdType, FwIdType, FwIndexType, FwOpcodeType, FwQueuePriorityType,
    FwSizeType,
};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, CmdStringArg, Endianness, FileNameString, FwDefaultString,
    LinearBuffer, LogSeverity, LogStringArg, SerBuf, SerBufAny, fpp_enum, fw_assert, fw_try,
};
use fprime_os::directory::{OpenMode, Status as DirStatus};
use fprime_os::file::{Mode as FileMode, Status as FileStatus};
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
/// `FileManagerCfg::DEFAULT_DP_PRIORITY`.
pub const DEFAULT_DP_PRIORITY: u32 = 10;

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
        // Data-product pacing would run here; see the module header gap note.
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

    /// `GenerateDp` — see the module header: the DP ports do not exist yet,
    /// so this takes the C++ "productGetOut not connected" path.
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
        if GenerateDpMode::try_from(mode_repr).is_err() {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        let log_name = to_log_string(&file_name);
        self.log_one_name(
            Self::EVENTID_GENERATE_DP_BUFFER_FAILED,
            LogSeverity::WarningHi,
            &log_name,
            "Could not get a data product buffer for file",
        );
        // GenerateDp always responds OK, including for every failure path.
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
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
    fn generate_dp_reports_a_buffer_failure_and_always_answers_ok() {
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
