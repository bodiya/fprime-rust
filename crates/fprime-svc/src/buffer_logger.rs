//! # Svc::BufferLogger — log buffers and com packets to files (active component)
//!
//! C++ sources: `Svc/BufferLogger/BufferLogger.{cpp,hpp,fpp}`,
//! `Svc/BufferLogger/BufferLoggerFile.cpp`,
//! `Svc/BufferLogger/{Commands,Events,Telemetry}.fppi`; SDD
//! `Svc/BufferLogger/docs/sdd.md`; tests `Svc/BufferLogger/test/ut/`
//! (`Logging`, `Errors`, `Health` and `LogNoInit`, ported below).
//!
//! Every `Fw::Buffer` arriving on the async `bufferSendIn` port and every
//! `Fw::ComBuffer` arriving on `comIn` is appended to the current log file as
//! a big-endian size field of `size_of_size` bytes followed by the data;
//! `bufferSendIn` buffers are then forwarded on `bufferSendOut` (whatever
//! the logging state), com buffers are not forwarded. Files are named
//! `<prefix><baseName><suffix>` for the first file after `BL_OpenFile` and
//! `<prefix><baseName><counter><suffix>` after that; a file is closed and the
//! next one opened when a record would push it past `max_file_size`.
//! Closing a file writes `<file>.CRC32` (its CRC-32, 4 big-endian bytes)
//! through [`ValidatedFile`] from a hash computed incrementally as the data
//! was written.
//!
//! C++-parity notes:
//! - File access is plain `Os::File` (no sandbox), opened `OPEN_WRITE`:
//!   created if missing, NOT truncated — re-opening the name of a longer
//!   existing file overwrites its head and leaves its tail.
//! - `BL_OpenFile` closes the current file (with `BL_LogFileClosed`), sets
//!   the base name, resets the counter to 0 and opens the file at once.
//! - Before a base name is set (or before [`BufferLogger::init_log`] when
//!   it never was) every record emits `BL_NoLogFileOpenInitError`; open,
//!   write and name errors are not throttled either — every failing record
//!   emits its event.
//! - A failed size-field write skips the data; the file stays open.
//! - A size that does not fit in `size_of_size` bytes is rejected with
//!   `BL_LogFileWriteError(INVALID_ARGUMENT, 0, size)` instead of logging
//!   its low bytes.
//! - `BL_SetLogging(LOGGING_OFF)` closes the file; `BL_FlushFile` is a
//!   no-op that always answers `OK` (unbuffered file I/O).
//! - Upstream declares but never emits `BL_Activated`, `BL_Deactivated` and
//!   the `BufferLogger_NumLoggedBuffers` channel; neither does this port
//!   (their ids are below for the dictionary). `schedIn` does nothing.
//! - Dropping the component closes the open file and writes its hash file
//!   without `BL_LogFileClosed` (the C++ `File` destructor), but still
//!   reports a hash-file failure with `BL_LogFileValidationError`, as the
//!   destructor's `close()` does.

use fprime_comp::escrow::BufferEscrow;
use fprime_comp::{
    ActiveBase, ActiveComponent, BufferSendPort, CmdGlue, CmdPort, ComPort, ComponentDispatch,
    EventGlue, MsgDispatchStatus, OutputPort, PingPort, QueueFullPolicy, SchedPort, TlmGlue,
    async_input_port_adapter, component_msg_types, msg,
};
use fprime_config::{
    FW_LOG_STRING_MAX_SIZE, FwChanIdType, FwEnumStoreType, FwEventIdType, FwIndexType,
    FwOpcodeType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, CmdStringArg, ComBuffer, Endianness, FwDefaultString,
    LinearBuffer, LogBuffer, LogSeverity, SerBuf, SerBufAny, SerializeStatus, StringFormatStatus,
    fpp_enum, fw_assert, fw_try,
};
use fprime_os::file::{self, File, Mode, WaitType};
use fprime_utils::hash::Hash;
use fprime_utils::validate_file::{self, ValidatedFile};

fpp_enum! {
    /// `BufferLogger.LogState` (C++ `BufferLogger_LogState`) — the volatile
    /// logging state set by `BL_SetLogging`.
    pub enum LogState : u8 {
        /// Logging is on (the state at construction).
        LoggingOn = 0,
        /// Logging is off: nothing is written, `bufferSendIn` still
        /// forwards.
        LoggingOff = 1,
    }
    default LoggingOn
}

/// Queue message size: the largest async invocation.
///
/// `comIn` = 6 (envelope) + 2 + 512 (nested `ComBuffer`) + 4 (context) =
/// 524; a command message is 6 + 4 + 4 + 2 + 506 = 522; `bufferSendIn` is
/// 6 + 8 (the escrow token, in the slot C++ uses for the buffer).
pub const QUEUE_MSG_SIZE: usize = 524;

/// Queue priority for every async input (no FPP `priority` qualifier).
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

/// The event `file` arguments are `string size 256`, but the generated
/// `log_*` methods serialize them truncated to `FW_LOG_STRING_MAX_SIZE`.
const EVENT_STRING_SIZE: usize = FW_LOG_STRING_MAX_SIZE;

/// The widest size field: `sizeof(FwSizeType)`.
const MAX_SIZE_OF_SIZE: u8 = size_of::<FwSizeType>() as u8;

/// C++ `BufferLogger::File::Mode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FileMode {
    /// No file is open.
    Closed,
    /// A file is open.
    Open,
}

/// C++ `BufferLogger::File` — the log file and its naming state. Its
/// methods live on [`BufferLogger`], which emits the events (C++
/// `m_bufferLogger`).
struct LogFile {
    /// C++ `m_prefix`.
    prefix: FwDefaultString,
    /// C++ `m_suffix`.
    suffix: FwDefaultString,
    /// C++ `m_baseName`.
    base_name: FwDefaultString,
    /// C++ `m_fileCounter`: the counter appended to names after the first.
    file_counter: FwSizeType,
    /// C++ `m_maxSize`.
    max_size: FwSizeType,
    /// C++ `m_sizeOfSize`: bytes in each record's size field.
    size_of_size: u8,
    /// C++ `m_name`: the currently open (or last) file name.
    name: FwDefaultString,
    /// C++ `m_mode`.
    mode: FileMode,
    /// C++ `m_osFile`.
    os_file: File,
    /// C++ `m_bytesWritten`.
    bytes_written: FwSizeType,
    /// C++ `m_hash`: running hash of the bytes written to the open file.
    hash: Hash,
}

impl LogFile {
    fn new() -> Self {
        Self {
            prefix: FwDefaultString::new(),
            suffix: FwDefaultString::new(),
            base_name: FwDefaultString::new(),
            file_counter: 0,
            max_size: 0,
            size_of_size: 0,
            name: FwDefaultString::new(),
            mode: FileMode::Closed,
            os_file: File::new(),
            bytes_written: 0,
            hash: Hash::new(),
        }
    }
}

/// Component state. Every input is async, so this is touched only by the
/// component thread (and by [`BufferLogger::init_log`] before it starts).
struct BufferLoggerState {
    /// C++ `m_state`.
    log_state: LogState,
    /// C++ `m_file`.
    file: LogFile,
}

/// `Svc::BufferLogger`.
pub struct BufferLogger {
    /// Active core: PassiveBase + queue + task.
    pub active: ActiveBase,
    /// Command registration + response ports (`cmdRegOut`, `cmdResponseOut`).
    pub cmd: CmdGlue,
    /// Event ports (`eventOut`, `eventOutText`) and the `timeCaller` port.
    pub evt: EventGlue,
    /// `tlmOut` (upstream never writes its channel).
    pub tlm: TlmGlue,
    /// `bufferSendOut: Fw.BufferSend` — every `bufferSendIn` buffer.
    pub buffer_send_out: OutputPort<dyn BufferSendPort>,
    /// `pingOut: Svc.Ping`.
    pub ping_out: OutputPort<dyn PingPort>,
    /// Escrow for the async `bufferSendIn` buffers.
    escrow: BufferEscrow,
    state: Mutex<BufferLoggerState>,
}

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel), in the C++ generated
    /// order (commands share [`BufferLogger::MSG_TYPE_CMD_IN`]; their
    /// opcode selects the handler).
    impl BufferLogger {
        /// `bufferSendIn` async input port.
        MSG_TYPE_BUFFER_SEND_IN,
        /// `comIn` async input port.
        MSG_TYPE_COM_IN,
        /// `pingIn` async input port.
        MSG_TYPE_PING_IN,
        /// `schedIn` async input port.
        MSG_TYPE_SCHED_IN,
        /// `cmdIn` async commands.
        MSG_TYPE_CMD_IN,
    }
}

impl BufferLogger {
    /// `BL_OpenFile(file: string size 40)` — opcode 0x00, async.
    pub const OPCODE_BL_OPENFILE: FwOpcodeType = 0x00;
    /// `BL_CloseFile` — opcode 0x01, async.
    pub const OPCODE_BL_CLOSEFILE: FwOpcodeType = 0x01;
    /// `BL_SetLogging(state: LogState)` — opcode 0x02, async.
    pub const OPCODE_BL_SETLOGGING: FwOpcodeType = 0x02;
    /// `BL_FlushFile` — opcode 0x03, async.
    pub const OPCODE_BL_FLUSHFILE: FwOpcodeType = 0x03;

    /// `BL_LogFileClosed(file: string size 256)` — DIAGNOSTIC.
    pub const EVENTID_BL_LOGFILECLOSED: FwEventIdType = 0x00;
    /// `BL_LogFileOpenError(errornum: U32, file)` — WARNING_HI.
    pub const EVENTID_BL_LOGFILEOPENERROR: FwEventIdType = 0x01;
    /// `BL_LogFileValidationError(validationFile, status: U32)` —
    /// WARNING_HI.
    pub const EVENTID_BL_LOGFILEVALIDATIONERROR: FwEventIdType = 0x02;
    /// `BL_LogFileWriteError(errornum, bytesWritten, bytesToWrite: U32,
    /// file)` — WARNING_HI.
    pub const EVENTID_BL_LOGFILEWRITEERROR: FwEventIdType = 0x03;
    /// `BL_Activated` — ACTIVITY_LO (declared, never emitted upstream).
    pub const EVENTID_BL_ACTIVATED: FwEventIdType = 0x04;
    /// `BL_Deactivated` — ACTIVITY_LO (declared, never emitted upstream).
    pub const EVENTID_BL_DEACTIVATED: FwEventIdType = 0x05;
    /// `BL_NoLogFileOpenInitError` — WARNING_HI.
    pub const EVENTID_BL_NOLOGFILEOPENINITERROR: FwEventIdType = 0x06;
    /// `BL_LogFileNameError(status: Fw.StringFormatStatus)` — WARNING_HI.
    pub const EVENTID_BL_LOGFILENAMEERROR: FwEventIdType = 0x07;

    /// `BufferLogger_NumLoggedBuffers: U32` (declared, never written
    /// upstream).
    pub const CHANID_BUFFERLOGGER_NUMLOGGEDBUFFERS: FwChanIdType = 0x00;

    /// Construct the component (C++ constructor: logging on, no file, no
    /// log parameters — every record reports `BL_NoLogFileOpenInitError`
    /// until [`init_log`](Self::init_log) and `BL_OpenFile`).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            buffer_send_out: OutputPort::new(),
            ping_out: OutputPort::new(),
            escrow: BufferEscrow::new(),
            state: Mutex::new(BufferLoggerState {
                log_state: LogState::LoggingOn,
                file: LogFile::new(),
            }),
        })
    }

    /// C++ `initLog(prefix, suffix, maxFileSize, sizeOfSize)`: set the log
    /// file parameters. Asserts, as C++ does, that no file is open, that
    /// `size_of_size` is at most `sizeof(FwSizeType)` and that
    /// `max_file_size > size_of_size`. Prefix and suffix are truncated to
    /// the string capacity (C++ `Fw::String` assignment).
    pub fn init_log(
        &self,
        log_file_prefix: &str,
        log_file_suffix: &str,
        max_file_size: FwSizeType,
        size_of_size: u8,
    ) {
        let mut state = self.state.lock();
        let file = &mut state.file;
        fw_assert!(file.mode == FileMode::Closed);
        file.prefix.set(log_file_prefix);
        file.suffix.set(log_file_suffix);
        file.max_size = max_file_size;
        file.size_of_size = size_of_size;
        fw_assert!(size_of_size <= MAX_SIZE_OF_SIZE, size_of_size as i32);
        fw_assert!(
            max_file_size > FwSizeType::from(size_of_size),
            max_file_size as i32
        );
    }

    fn id_base(&self) -> u32 {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_BL_OPENFILE,
                Self::OPCODE_BL_CLOSEFILE,
                Self::OPCODE_BL_SETLOGGING,
                Self::OPCODE_BL_FLUSHFILE,
            ],
        );
    }

    /// Create the component queue (topology phase 4).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    /// Name of the currently open (or last) log file.
    #[must_use]
    pub fn file_name(&self) -> FwDefaultString {
        self.state.lock().file.name.clone()
    }

    /// Whether a log file is open.
    #[must_use]
    pub fn is_file_open(&self) -> bool {
        self.state.lock().file.mode == FileMode::Open
    }

    /// The current logging state.
    #[must_use]
    pub fn log_state(&self) -> LogState {
        self.state.lock().log_state
    }

    // -- Handlers (component thread) -----------------------------------------

    /// `bufferSendIn_handler`: log the buffer (when on), then forward it.
    fn buffer_send_in_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        {
            let mut state = self.state.lock();
            if state.log_state == LogState::LoggingOn {
                // C++ parity: File::logBuffer asserts data != nullptr, i.e.
                // a default-constructed (storage-less) buffer.
                fw_assert!(buffer.capacity() > 0);
                self.log_buffer(&mut state.file, buffer.data());
            }
        }
        let p = self.buffer_send_out.get();
        p.target.invoke(p.port_num, buffer);
    }

    /// `comIn_handler`: log the packet (when on); it is not forwarded.
    fn com_in_handler(&self, _port_num: FwIndexType, data: &mut ComBuffer, _context: u32) {
        let mut state = self.state.lock();
        if state.log_state == LogState::LoggingOn {
            self.log_buffer(&mut state.file, data.as_slice());
        }
    }

    /// `pingIn_handler` — echo the key.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }

    /// `schedIn_handler` — empty upstream (`// TODO`).
    fn sched_in_handler(&self, _port_num: FwIndexType, _context: u32) {}

    /// `cmdIn` dispatch.
    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_BL_OPENFILE => self.bl_open_file_cmd_handler(op_code, cmd_seq, args),
            Self::OPCODE_BL_CLOSEFILE => self.bl_close_file_cmd_handler(op_code, cmd_seq, args),
            Self::OPCODE_BL_SETLOGGING => self.bl_set_logging_cmd_handler(op_code, cmd_seq, args),
            Self::OPCODE_BL_FLUSHFILE => self.bl_flush_file_cmd_handler(op_code, cmd_seq, args),
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }

    /// `BL_OpenFile_cmdHandler`: set the base name (closing the current
    /// file), reset the counter, open; always `OK`.
    fn bl_open_file_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut file_arg = CmdStringArg::new();
        if !args.deserialize(&mut file_arg, Endianness::Big).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        {
            let mut state = self.state.lock();
            self.set_base_name(&mut state.file, file_arg.as_bytes());
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `BL_CloseFile_cmdHandler`: close (if open); always `OK`.
    fn bl_close_file_cmd_handler(
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
        {
            let mut state = self.state.lock();
            self.close_and_emit_event(&mut state.file);
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `BL_SetLogging_cmdHandler`: set the state; `LOGGING_OFF` closes the
    /// file. No `BL_Activated`/`BL_Deactivated` (C++ parity).
    fn bl_set_logging_cmd_handler(
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
        // Workspace convention: an undeclared enum value answers
        // VALIDATION_ERROR (the generated C++ folds it into FORMAT_ERROR).
        let Ok(log_state) = LogState::try_from(raw) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        {
            let mut state = self.state.lock();
            state.log_state = log_state;
            if log_state == LogState::LoggingOff {
                self.close_and_emit_event(&mut state.file);
            }
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `BL_FlushFile_cmdHandler`: `File::flush()` always succeeds
    /// (unbuffered file I/O), so this always answers `OK`.
    fn bl_flush_file_cmd_handler(
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
        let status = Self::flush();
        let response = if status {
            CmdResponse::Ok
        } else {
            CmdResponse::ExecutionError
        };
        self.cmd.cmd_response(op_code, cmd_seq, response);
    }

    // -- File plumbing (C++ BufferLogger::File) ------------------------------

    /// C++ `File::setBaseName`.
    fn set_base_name(&self, file: &mut LogFile, base_name: &[u8]) {
        if file.mode == FileMode::Open {
            self.close_and_emit_event(file);
        }
        file.base_name.set_bytes(base_name);
        file.file_counter = 0;
        self.open(file);
    }

    /// C++ `File::logBuffer`.
    fn log_buffer(&self, file: &mut LogFile, data: &[u8]) {
        let size = data.len() as FwSizeType;
        // Close the file if it would grow too big.
        if file.mode == FileMode::Open {
            let projected = file
                .bytes_written
                .wrapping_add(FwSizeType::from(file.size_of_size))
                .wrapping_add(size);
            if projected > file.max_size {
                self.close_and_emit_event(file);
            }
        }
        if file.mode == FileMode::Closed {
            self.open(file);
        }
        if file.mode == FileMode::Open {
            let _ = self.write_buffer(file, data);
        }
    }

    /// C++ `File::closeAndEmitEvent`.
    fn close_and_emit_event(&self, file: &mut LogFile) {
        if file.mode == FileMode::Open {
            self.close(file);
            self.log_bl_log_file_closed(&file.name);
        }
    }

    /// C++ `File::flush`: unbuffered I/O needs no flush.
    fn flush() -> bool {
        true
    }

    /// C++ `File::open`.
    fn open(&self, file: &mut LogFile) {
        fw_assert!(file.mode == FileMode::Closed);

        // Check that a base name was set and initLog was called.
        if file.base_name.is_empty()
            || file.size_of_size > MAX_SIZE_OF_SIZE
            || file.max_size <= FwSizeType::from(file.size_of_size)
        {
            self.log_bl_no_log_file_open_init_error();
            return;
        }

        let status = Self::format_name(file);
        if status != StringFormatStatus::Success {
            self.log_bl_log_file_name_error(status);
            return;
        }

        // The OSAL takes `&str`; a non-UTF-8 name (bytes from the
        // BL_OpenFile argument) cannot be opened and reports the open
        // failure of the empty path.
        let status = file
            .os_file
            .open(file.name.as_str().unwrap_or_default(), Mode::OpenWrite);
        if status == file::Status::OpOk {
            file.file_counter = file.file_counter.wrapping_add(1);
            file.bytes_written = 0;
            file.hash.init();
            file.mode = FileMode::Open;
        } else {
            self.log_bl_log_file_open_error(status as u32, &file.name);
        }
    }

    /// The C++ `m_name.format("%s%s%s" | "%s%s%" PRI_FwSizeType "%s", ...)`:
    /// `Overflowed` (with the name truncated, as `vsnprintf` leaves it) when
    /// the result does not fit the string.
    fn format_name(file: &mut LogFile) -> StringFormatStatus {
        let mut counter = FwDefaultString::new();
        if file.file_counter != 0 {
            counter.format(format_args!("{}", file.file_counter));
        }
        let total = file.prefix.len() + file.base_name.len() + counter.len() + file.suffix.len();
        file.name.set_bytes(file.prefix.as_bytes());
        file.name.append_bytes(file.base_name.as_bytes());
        file.name.append_bytes(counter.as_bytes());
        file.name.append_bytes(file.suffix.as_bytes());
        if total > FwDefaultString::max_length() {
            StringFormatStatus::Overflowed
        } else {
            StringFormatStatus::Success
        }
    }

    /// C++ `File::writeBuffer`: the size field, then (only if it was
    /// written) the data.
    fn write_buffer(&self, file: &mut LogFile, data: &[u8]) -> bool {
        self.write_size(file, data.len() as FwSizeType) && self.write_bytes(file, data)
    }

    /// C++ `File::writeSize`: `size_of_size` big-endian bytes.
    fn write_size(&self, file: &mut LogFile, size: FwSizeType) -> bool {
        fw_assert!(file.size_of_size <= MAX_SIZE_OF_SIZE);
        // Reject sizes that do not fit; writing the low bytes would corrupt
        // the log.
        if file.size_of_size < MAX_SIZE_OF_SIZE && (size >> (8 * u32::from(file.size_of_size))) != 0
        {
            self.log_bl_log_file_write_error(
                file::Status::InvalidArgument as u32,
                0,
                size as u32,
                &file.name,
            );
            return false;
        }
        let bytes = size.to_be_bytes();
        let field = &bytes[bytes.len() - usize::from(file.size_of_size)..];
        let mut size_buffer = [0u8; MAX_SIZE_OF_SIZE as usize];
        size_buffer[..field.len()].copy_from_slice(field);
        self.write_bytes(file, &size_buffer[..field.len()])
    }

    /// C++ `File::writeBytes`: a successful write counts the bytes and
    /// feeds the running hash; a failure emits `BL_LogFileWriteError`.
    fn write_bytes(&self, file: &mut LogFile, data: &[u8]) -> bool {
        let length = data.len() as FwSizeType;
        let mut size = length;
        // C++ parity: the two-argument Os::File::write is WAIT.
        let status = file.os_file.write(data, &mut size, WaitType::Wait);
        if status == file::Status::OpOk && size == length {
            file.bytes_written = file.bytes_written.wrapping_add(length);
            file.hash.update(data);
            true
        } else {
            self.log_bl_log_file_write_error(status as u32, size as u32, length as u32, &file.name);
            false
        }
    }

    /// C++ `File::writeHashFile`: the incrementally computed hash, written
    /// through `Os::ValidatedFile`.
    fn write_hash_file(&self, file: &mut LogFile) {
        let hash_buffer = file.hash.finalize_buffer();
        let mut validated_file = ValidatedFile::new(file.name.as_str().unwrap_or_default());
        let status = validated_file.create_hash_file_from(&hash_buffer);
        if status != validate_file::Status::ValidationOk {
            self.log_bl_log_file_validation_error(
                validated_file.get_hash_file_name(),
                status as u32,
            );
        }
    }

    /// C++ `File::close`.
    fn close(&self, file: &mut LogFile) {
        if file.mode == FileMode::Open {
            file.os_file.close();
            self.write_hash_file(file);
            file.mode = FileMode::Closed;
        }
    }

    // -- Events ---------------------------------------------------------------

    fn log_bl_log_file_closed(&self, file_name: &FwDefaultString) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BL_LOGFILECLOSED,
            LogSeverity::Diagnostic,
            &format!("File {file_name} closed"),
            |buf: &mut LogBuffer| {
                file_name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_bl_log_file_open_error(&self, errornum: u32, file_name: &FwDefaultString) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BL_LOGFILEOPENERROR,
            LogSeverity::WarningHi,
            &format!("Error {errornum} opening file {file_name}"),
            |buf: &mut LogBuffer| {
                fw_try!(buf.serialize_u32_be(errornum));
                file_name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_bl_log_file_validation_error(&self, validation_file: &FwDefaultString, status: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BL_LOGFILEVALIDATIONERROR,
            LogSeverity::WarningHi,
            &format!("Failed creating validation file {validation_file} with status {status}"),
            |buf: &mut LogBuffer| {
                fw_try!(validation_file.serialize_to_truncated(
                    buf,
                    EVENT_STRING_SIZE,
                    Endianness::Big
                ));
                buf.serialize_u32_be(status)
            },
        );
    }

    fn log_bl_log_file_write_error(
        &self,
        errornum: u32,
        bytes_written: u32,
        bytes_to_write: u32,
        file_name: &FwDefaultString,
    ) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BL_LOGFILEWRITEERROR,
            LogSeverity::WarningHi,
            &format!(
                "Error {errornum} while writing {bytes_written} of {bytes_to_write} bytes to {file_name}"
            ),
            |buf: &mut LogBuffer| {
                fw_try!(buf.serialize_u32_be(errornum));
                fw_try!(buf.serialize_u32_be(bytes_written));
                fw_try!(buf.serialize_u32_be(bytes_to_write));
                file_name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_bl_no_log_file_open_init_error(&self) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BL_NOLOGFILEOPENINITERROR,
            LogSeverity::WarningHi,
            "No log file open command",
            |_buf: &mut LogBuffer| SerializeStatus::Ok,
        );
    }

    fn log_bl_log_file_name_error(&self, status: StringFormatStatus) {
        let name = match status {
            StringFormatStatus::Success => "SUCCESS",
            StringFormatStatus::Overflowed => "OVERFLOWED",
            StringFormatStatus::InvalidFormatString => "INVALID_FORMAT_STRING",
            StringFormatStatus::SizeOverflow => "SIZE_OVERFLOW",
            StringFormatStatus::OtherError => "OTHER_ERROR",
        };
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BL_LOGFILENAMEERROR,
            LogSeverity::WarningHi,
            &format!("Error {name} formatting log file name"),
            |buf: &mut LogBuffer| buf.serialize_u8_be(status as u8),
        );
    }
}

impl Drop for BufferLogger {
    /// C++ `File::~File` -> `close()`: close the file and write its hash
    /// file, with no `BL_LogFileClosed` (a hash-file failure is still
    /// reported, as the destructor's `close()` does).
    fn drop(&mut self) {
        let mut file = core::mem::replace(&mut self.state.get_mut().file, LogFile::new());
        self.close(&mut file);
    }
}

// -- Async input adapters ----------------------------------------------------

/// `bufferSendIn` adapter: the buffer is deposited in the escrow and the
/// token rides in the queue message where C++ serializes the buffer.
struct BufferSendInAdapter {
    comp: Arc<BufferLogger>,
}

impl BufferSendPort for BufferSendInAdapter {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        let token = self.comp.escrow.deposit(buffer);
        let mut msg = LinearBuffer::<QUEUE_MSG_SIZE>::new();
        let status =
            msg::write_envelope_header(&mut msg, BufferLogger::MSG_TYPE_BUFFER_SEND_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = msg.serialize_u64_be(token);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&msg, QUEUE_PRIORITY, QueueFullPolicy::Assert);
    }
}

impl BufferLogger {
    /// `bufferSendIn` — ASYNC `Fw.BufferSend` input: a buffer to log and
    /// forward.
    pub fn buffer_send_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> fprime_comp::PortRef<dyn BufferSendPort> {
        fprime_comp::PortRef::new(
            Arc::new(BufferSendInAdapter {
                comp: Arc::clone(self),
            }),
            port_num,
        )
    }
}

async_input_port_adapter! {
    /// `comIn` — ASYNC `Fw.Com` input: a packet to log.
    component: BufferLogger;
    adapter: ComInAdapter;
    port: ComPort;
    input: pub com_in;
    deserialize: com_in_deserialize;
    handler: com_in_handler;
    base: active.queued;
    msg_type: BufferLogger::MSG_TYPE_COM_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { buf data: ComBuffer, val context: u32 }
}

async_input_port_adapter! {
    /// `pingIn` — ASYNC `Svc.Ping` input with the `drop` policy.
    component: BufferLogger;
    adapter: PingInAdapter;
    port: PingPort;
    input: pub ping_in;
    deserialize: ping_in_deserialize;
    handler: ping_in_handler;
    base: active.queued;
    msg_type: BufferLogger::MSG_TYPE_PING_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Drop;
    args { val key: u32 }
}

async_input_port_adapter! {
    /// `schedIn` — ASYNC `Svc.Sched` input (its handler is empty).
    component: BufferLogger;
    adapter: SchedInAdapter;
    port: SchedPort;
    input: pub sched_in;
    deserialize: sched_in_deserialize;
    handler: sched_in_handler;
    base: active.queued;
    msg_type: BufferLogger::MSG_TYPE_SCHED_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val context: u32 }
}

async_input_port_adapter! {
    /// `cmdIn` — ASYNC `Fw.Cmd` input for the four commands.
    component: BufferLogger;
    adapter: CmdInAdapter;
    port: CmdPort;
    input: pub cmd_in;
    deserialize: cmd_in_deserialize;
    handler: cmd_in_handler;
    base: active.queued;
    msg_type: BufferLogger::MSG_TYPE_CMD_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, buf args: CmdArgBuffer }
}

// -- Dispatch (the generated `doDispatch` switch) -----------------------------

impl ComponentDispatch for BufferLogger {
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
            Self::MSG_TYPE_BUFFER_SEND_IN => {
                let mut token = 0u64;
                if !buf.deserialize_u64_be(&mut token).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                let buffer = self.escrow.claim(token);
                self.buffer_send_in_handler(port_num, buffer);
                MsgDispatchStatus::Ok
            }
            Self::MSG_TYPE_COM_IN => match Self::com_in_deserialize(buf) {
                Some((mut data, context)) => {
                    self.com_in_handler(port_num, &mut data, context);
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
            Self::MSG_TYPE_SCHED_IN => match Self::sched_in_deserialize(buf) {
                Some((context,)) => {
                    self.sched_in_handler(port_num, context);
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

impl ActiveComponent for BufferLogger {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests;
