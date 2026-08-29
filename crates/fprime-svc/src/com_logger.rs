//! # Svc::ComLogger — rotating `.com` file logger (active component)
//!
//! C++ sources: `Svc/ComLogger/ComLogger.{cpp,hpp,fpp}`,
//! `Svc/ComLogger/{Commands,Events}.fppi`, `Os/ValidateFileCommon.cpp`.
//! Analysis: `docs/cpp-analysis/svc-misc.md` (ComLogger section + gotchas).
//!
//! Every `Fw::ComBuffer` arriving on the async `comIn` port is appended to
//! `<prefix>_<timeBase>_<seconds>_<useconds:06>.com`. With
//! `store_buffer_length` (the default) each record is a big-endian `u16`
//! length followed by exactly that many raw buffer bytes; otherwise the raw
//! bytes are written back to back with no framing at all.
//!
//! A new file is started when the projected size (`byte_count + size`, plus
//! the 2 prefix bytes when storing lengths) is strictly greater than
//! `max_file_size`. Closing a file — by rotation, by the `CloseFile`
//! command, or by dropping the component — writes a `.CRC32` sidecar next to
//! it containing the file's CRC-32 as 4 BIG-ENDIAN bytes (this is the
//! `Os::ValidateFile` encoding; `Utils::CRCChecker`'s identically-named
//! sidecar is native-endian and is NOT what this component writes).
//!
//! C++-parity notes carried over from the analysis:
//! - `comIn`, `pingIn` and `CloseFile` are ALL async, so every piece of file
//!   state lives on the component thread; no extra locking is needed.
//! - `FileOpenError` and `FileWriteError` each have an independent one-shot
//!   latch (not an FPP throttle): the event is emitted once and re-armed
//!   only by a subsequent success.
//! - A write failure neither closes nor rotates the file; if the length
//!   prefix fails to write the payload is skipped entirely.
//! - A component built without a file prefix stays uninitialized: every
//!   `comIn` drops its buffer and emits the throttled (5)
//!   `FileNotInitialized` event.
//! - The destructor closes the file and writes the sidecar but deliberately
//!   emits NO `FileClosed` event.

use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, ComPort, ComponentDispatch, EventGlue,
    EventThrottle, MsgDispatchStatus, OutputPort, PingPort, QueueFullPolicy,
    async_input_port_adapter, component_msg_types, msg,
};
use fprime_config::{
    FW_LOG_STRING_MAX_SIZE, FwEnumStoreType, FwEventIdType, FwIndexType, FwOpcodeType,
    FwQueuePriorityType, FwSizeType, FwTimeBaseStoreType,
};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, ComBuffer, Endianness, FileNameString, LogBuffer, LogSeverity,
    SerBuf, SerBufAny, SerializeStatus, fw_assert, fw_try,
};
use fprime_os::file::{self, File, Mode, WaitType};
use fprime_os::filesystem;
use fprime_utils::hash::{HASH_EXTENSION_STRING, Hash, HashBuffer};
use std::sync::{Arc, Mutex};

/// Queue message size: the largest async invocation.
///
/// `comIn` = 6 (envelope) + 2 + 512 (nested `ComBuffer`) + 4 (context) =
/// 524; a command message is 6 + 4 + 4 + 2 + 506 = 522.
pub const QUEUE_MSG_SIZE: usize = 524;

/// Queue priority for every async input (no FPP `priority` qualifier).
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

/// `Os::ValidateFile::VFILE_HASH_CHUNK_SIZE` — the file is hashed in
/// 256-byte chunks.
pub const VFILE_HASH_CHUNK_SIZE: usize = 256;

/// Event `file` arguments are `string size 240`, but the generated
/// `log_*` methods serialize through `Fw::LogStringArg`, so the effective
/// on-wire cap is `FW_LOG_STRING_MAX_SIZE`.
const EVENT_STRING_SIZE: usize = FW_LOG_STRING_MAX_SIZE;

/// Port of `Os::ValidateFile::Status` — the ordinals carried by the
/// `FileValidationError` event's `status` argument.
#[must_use]
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidateStatus {
    /// The validation of the file passed / the sidecar was written.
    ValidationOk = 0,
    /// The validation of the file did not pass.
    ValidationFail = 1,
    /// File doesn't exist (for read).
    FileDoesntExist = 2,
    /// No permission to read/write the file.
    FileNoPermission = 3,
    /// Invalid size parameter on the file.
    FileBadSize = 4,
    /// Validation file doesn't exist (for read).
    ValidationFileDoesntExist = 5,
    /// No permission to read/write the validation file.
    ValidationFileNoPermission = 6,
    /// Invalid size parameter on the validation file.
    ValidationFileBadSize = 7,
    /// No space left on the device for writing.
    NoSpace = 8,
    /// Catch-all for other errors.
    OtherError = 9,
}

/// Which file a [`file::Status`] came from, for [`translate_status`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StatusFileType {
    /// The `.com` file being hashed.
    File,
    /// The `.CRC32` sidecar.
    HashFile,
}

/// Port of `Os::translateStatus` (ValidateFileCommon.cpp).
fn translate_status(status: file::Status, file_type: StatusFileType) -> ValidateStatus {
    match file_type {
        StatusFileType::File => match status {
            file::Status::OpOk => ValidateStatus::ValidationOk,
            file::Status::DoesntExist => ValidateStatus::FileDoesntExist,
            file::Status::NoSpace => ValidateStatus::NoSpace,
            file::Status::NoPermission => ValidateStatus::FileNoPermission,
            file::Status::BadSize => ValidateStatus::FileBadSize,
            _ => ValidateStatus::OtherError,
        },
        StatusFileType::HashFile => match status {
            file::Status::OpOk => ValidateStatus::ValidationOk,
            file::Status::DoesntExist => ValidateStatus::ValidationFileDoesntExist,
            file::Status::NoSpace => ValidateStatus::NoSpace,
            file::Status::NoPermission => ValidateStatus::ValidationFileNoPermission,
            file::Status::BadSize => ValidateStatus::ValidationFileBadSize,
            _ => ValidateStatus::OtherError,
        },
    }
}

/// Port of `Os::computeHash` — CRC-32 of `file_name`, read in
/// [`VFILE_HASH_CHUNK_SIZE`] chunks.
fn compute_hash(file_name: &str, hash_buffer: &mut HashBuffer) -> file::Status {
    let mut file = File::new();
    let status = file.open(file_name, Mode::OpenRead);
    if status != file::Status::OpOk {
        return status;
    }
    let mut file_size: FwSizeType = 0;
    if filesystem::get_file_size(file_name, &mut file_size) != filesystem::Status::OpOk {
        return file::Status::BadSize;
    }
    // C++ parity: the iteration bound is computed from the size at open, so a
    // file that grows while being hashed is a BAD_SIZE error, not a hang.
    let max_itr = file_size / (VFILE_HASH_CHUNK_SIZE as FwSizeType) + 1;

    let mut hash = Hash::new();
    hash.init();
    let mut buffer = [0u8; VFILE_HASH_CHUNK_SIZE];
    let mut size: FwSizeType = 0;
    let mut cnt: FwSizeType = 0;
    while cnt <= max_itr {
        size = buffer.len() as FwSizeType;
        let status = file.read(&mut buffer, &mut size, WaitType::NoWait);
        if status != file::Status::OpOk {
            return status;
        }
        if size == 0 {
            break;
        }
        hash.update(&buffer[..size as usize]);
        cnt += 1;
    }
    file.close();
    if size != 0 {
        return file::Status::BadSize;
    }
    *hash_buffer = hash.finalize_buffer();
    file::Status::OpOk
}

/// Port of `Os::writeHash` — the 4-byte big-endian digest.
fn write_hash(hash_file_name: &str, hash_buffer: &HashBuffer) -> file::Status {
    let mut hash_file = File::new();
    let status = hash_file.open(hash_file_name, Mode::OpenWrite);
    if status != file::Status::OpOk {
        return status;
    }
    let digest_len = hash_buffer.as_slice().len() as FwSizeType;
    let mut size: FwSizeType = digest_len;
    let status = hash_file.write(hash_buffer.as_slice(), &mut size, WaitType::NoWait);
    if status != file::Status::OpOk {
        return status;
    }
    if size != digest_len {
        return file::Status::BadSize;
    }
    hash_file.close();
    file::Status::OpOk
}

/// Port of `Os::ValidateFile::createValidation(fileName, hashFileName)`:
/// hash `file_name` and store the digest in `hash_file_name`.
pub fn create_validation(file_name: &str, hash_file_name: &str) -> ValidateStatus {
    let mut hash_buffer = HashBuffer::new();
    let status = compute_hash(file_name, &mut hash_buffer);
    if status != file::Status::OpOk {
        return translate_status(status, StatusFileType::File);
    }
    let status = write_hash(hash_file_name, &hash_buffer);
    if status != file::Status::OpOk {
        return translate_status(status, StatusFileType::HashFile);
    }
    ValidateStatus::ValidationOk
}

/// C++ `ComLogger::FileMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FileMode {
    /// No file is open; the next record opens one.
    Closed,
    /// A file is open and accepting records.
    Open,
}

/// Component state. Every input is async, so this is touched only by the
/// component thread (the `Mutex` provides interior mutability, not
/// cross-thread arbitration).
struct ComLoggerState {
    /// C++ `m_filePrefix`.
    file_prefix: FileNameString,
    /// C++ `m_maxFileSize`.
    max_file_size: u32,
    /// C++ `m_storeBufferLength`.
    store_buffer_length: bool,
    /// C++ `m_initialized`.
    initialized: bool,
    /// C++ `m_fileMode`.
    file_mode: FileMode,
    /// C++ `m_fileName`.
    file_name: FileNameString,
    /// C++ `m_hashFileName`.
    hash_file_name: FileNameString,
    /// C++ `m_byteCount` — successfully written bytes in the open file.
    byte_count: u32,
    /// C++ `m_writeErrorOccurred` one-shot latch.
    write_error_occurred: bool,
    /// C++ `m_openErrorOccurred` one-shot latch.
    open_error_occurred: bool,
    /// The open `.com` file.
    file: File,
}

/// `Svc::ComLogger`.
pub struct ComLogger {
    /// Active core: PassiveBase + queue + task.
    pub active: ActiveBase,
    /// Command registration + response ports (`cmdRegOut`, `cmdResponseOut`).
    pub cmd: CmdGlue,
    /// Event ports (`logOut`, `LogText`) and the `timeCaller` port.
    pub evt: EventGlue,
    /// `pingOut: Svc.Ping`.
    pub ping_out: OutputPort<dyn PingPort>,
    /// FPP `throttle 5` on `FileNotInitialized`.
    file_not_initialized_throttle: EventThrottle,
    state: Mutex<ComLoggerState>,
}

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel).
    impl ComLogger {
        /// `comIn` async input port.
        MSG_TYPE_COM_IN,
        /// `pingIn` async input port.
        MSG_TYPE_PING_IN,
        /// `cmdIn` async command port.
        MSG_TYPE_CMD_IN,
    }
}

impl ComLogger {
    /// Command `CloseFile` — opcode 0x00, async, no arguments. This is the
    /// component's ONLY command.
    pub const OPCODE_CLOSE_FILE: FwOpcodeType = 0x00;

    /// `FileOpenError(errornum: U32, file: string size 240)` — WARNING_HI.
    pub const EVENTID_FILE_OPEN_ERROR: FwEventIdType = 0x00;
    /// `FileWriteError(errornum: U32, bytesWritten: U32, bytesToWrite: U32,
    /// file: string size 240)` — WARNING_HI.
    pub const EVENTID_FILE_WRITE_ERROR: FwEventIdType = 0x01;
    /// `FileValidationError(validationFile, file, status: U32)` —
    /// WARNING_LO.
    pub const EVENTID_FILE_VALIDATION_ERROR: FwEventIdType = 0x02;
    /// `FileClosed(file: string size 240)` — DIAGNOSTIC.
    pub const EVENTID_FILE_CLOSED: FwEventIdType = 0x03;
    /// `FileNotInitialized` — WARNING_LO, `throttle 5`.
    pub const EVENTID_FILE_NOT_INITIALIZED: FwEventIdType = 0x04;
    /// FPP `throttle 5` on `FileNotInitialized`.
    pub const FILE_NOT_INITIALIZED_THROTTLE: u32 = 5;

    /// Construct an UNINITIALIZED logger (C++ `ComLogger(compName)`): every
    /// `comIn` drops its buffer and emits the throttled
    /// `FileNotInitialized` event until [`ComLogger::init_log_file`] is
    /// called.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            ping_out: OutputPort::new(),
            file_not_initialized_throttle: EventThrottle::new(Self::FILE_NOT_INITIALIZED_THROTTLE),
            state: Mutex::new(ComLoggerState {
                file_prefix: FileNameString::new(),
                max_file_size: 0,
                store_buffer_length: false,
                initialized: false,
                file_mode: FileMode::Closed,
                file_name: FileNameString::new(),
                hash_file_name: FileNameString::new(),
                byte_count: 0,
                write_error_occurred: false,
                open_error_occurred: false,
                file: File::new(),
            }),
        })
    }

    /// Construct and initialize in one step (C++
    /// `ComLogger(compName, incomingFilePrefix, maxFileSize,
    /// storeBufferLength)`).
    pub fn with_log_file(
        name: &str,
        file_prefix: &str,
        max_file_size: u32,
        store_buffer_length: bool,
    ) -> Arc<Self> {
        let comp = Self::new(name);
        comp.init_log_file(file_prefix, max_file_size, store_buffer_length);
        comp
    }

    /// C++ `init_log_file`. Asserts, as C++ does, that a length-prefixing
    /// logger has room for at least the prefix, and that the prefix fits the
    /// file-name string.
    pub fn init_log_file(&self, file_prefix: &str, max_file_size: u32, store_buffer_length: bool) {
        if store_buffer_length {
            fw_assert!(max_file_size > 2, max_file_size as i32);
        }
        // C++ parity: the m_filePrefix.format() status is asserted, i.e. an
        // over-long prefix is a configuration error, not a truncation.
        fw_assert!(
            file_prefix.len() <= FileNameString::max_length(),
            file_prefix.len() as i32
        );
        let mut state = self.state.lock().unwrap();
        state.max_file_size = max_file_size;
        state.store_buffer_length = store_buffer_length;
        state.file_prefix.set(file_prefix);
        state.initialized = true;
    }

    fn id_base(&self) -> u32 {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd
            .reg_commands(self.id_base(), &[Self::OPCODE_CLOSE_FILE]);
    }

    /// Create the component queue (topology phase 4).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    /// Name of the currently open (or last closed) `.com` file.
    #[must_use]
    pub fn file_name(&self) -> FileNameString {
        self.state.lock().unwrap().file_name.clone()
    }

    /// Whether a file is currently open.
    #[must_use]
    pub fn is_file_open(&self) -> bool {
        self.state.lock().unwrap().file_mode == FileMode::Open
    }

    // -- Handlers (component thread) -----------------------------------------

    /// `comIn_handler`.
    fn com_in_handler(&self, port_num: FwIndexType, data: &mut ComBuffer, _context: u32) {
        // C++ parity: the port is a single-element array and the ComLogger
        // only writes 16-bit record sizes.
        fw_assert!(port_num == 0, port_num as i32);
        let size_native = data.get_size();
        fw_assert!(size_native < 65536, size_native as i32);
        let size = (size_native & 0xFFFF) as u16;

        let mut state = self.state.lock().unwrap();
        // Close the file if this record would make it too big.
        if state.file_mode == FileMode::Open {
            let mut projected = state.byte_count.wrapping_add(u32::from(size));
            if state.store_buffer_length {
                projected = projected.wrapping_add(2);
            }
            if projected > state.max_file_size {
                self.close_file(&mut state);
            }
        }
        if state.file_mode == FileMode::Closed {
            self.open_file(&mut state);
        }
        if state.file_mode == FileMode::Open {
            self.write_com_buffer_to_file(&mut state, data, size);
        }
    }

    /// `pingIn_handler` — echo the key.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        if let Some(p) = self.ping_out.try_get() {
            p.target.invoke(p.port_num, key);
        }
    }

    /// `cmdIn` dispatch (only `CloseFile` exists).
    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_CLOSE_FILE => self.close_file_cmd_handler(op_code, cmd_seq, args),
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }

    /// `CloseFile_cmdHandler`: closes the file (if any) and always answers
    /// `Ok`, even when nothing was open.
    fn close_file_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        // C++ parity (FW_CMD_CHECK_RESIDUAL): a zero-argument command
        // rejects residual bytes.
        if args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        {
            let mut state = self.state.lock().unwrap();
            self.close_file(&mut state);
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    // -- File plumbing -------------------------------------------------------

    /// C++ `openFile()`.
    fn open_file(&self, state: &mut ComLoggerState) {
        fw_assert!(state.file_mode == FileMode::Closed);

        if !state.initialized {
            if self.file_not_initialized_throttle.ok_to_emit() {
                self.log_file_not_initialized();
            }
            return;
        }

        let timestamp = self.evt.time_get();
        let name = format!(
            "{}_{}_{}_{:06}.com",
            state.file_prefix,
            timestamp.get_time_base() as FwTimeBaseStoreType,
            timestamp.get_seconds(),
            timestamp.get_useconds()
        );
        let hash_name = format!("{name}{HASH_EXTENSION_STRING}");
        // C++ parity: both format() statuses are asserted.
        fw_assert!(
            name.len() <= FileNameString::max_length(),
            name.len() as i32
        );
        fw_assert!(
            hash_name.len() <= FileNameString::max_length(),
            hash_name.len() as i32
        );
        state.file_name.set(&name);
        state.hash_file_name.set(&hash_name);

        let status = state.file.open(&name, Mode::OpenWrite);
        if status != file::Status::OpOk {
            if !state.open_error_occurred {
                // Throttled by hand: an unthrottled open error can drive a
                // positive feedback loop through the event path.
                self.log_file_open_error(status as u32, &state.file_name);
            }
            state.open_error_occurred = true;
        } else {
            state.open_error_occurred = false;
            state.byte_count = 0;
            state.file_mode = FileMode::Open;
        }
    }

    /// C++ `closeFile()` — close, write the sidecar, emit `FileClosed`.
    fn close_file(&self, state: &mut ComLoggerState) {
        if state.file_mode != FileMode::Open {
            return;
        }
        state.file.close();
        self.write_hash_file(state);
        state.file_mode = FileMode::Closed;
        self.log_file_closed(&state.file_name);
    }

    /// C++ `writeComBufferToFile`.
    fn write_com_buffer_to_file(&self, state: &mut ComLoggerState, data: &ComBuffer, size: u16) {
        if state.store_buffer_length {
            let prefix = size.to_be_bytes();
            if self.write_to_file(state, &prefix) {
                state.byte_count = state.byte_count.wrapping_add(prefix.len() as u32);
            } else {
                // Gotcha: the payload is skipped entirely when the length
                // prefix could not be written.
                return;
            }
        }
        if self.write_to_file(state, &data.bytes()[..usize::from(size)]) {
            state.byte_count = state.byte_count.wrapping_add(u32::from(size));
        }
    }

    /// C++ `writeToFile` — a failure latches the write-error event and
    /// leaves the file open (no rotation, no close).
    fn write_to_file(&self, state: &mut ComLoggerState, data: &[u8]) -> bool {
        let length = data.len() as FwSizeType;
        let mut size = length;
        let status = state.file.write(data, &mut size, WaitType::Wait);
        if status != file::Status::OpOk || size != length {
            if !state.write_error_occurred {
                self.log_file_write_error(
                    status as u32,
                    size as u32,
                    length as u32,
                    &state.file_name,
                );
            }
            state.write_error_occurred = true;
            return false;
        }
        state.write_error_occurred = false;
        true
    }

    /// C++ `writeHashFile()`.
    fn write_hash_file(&self, state: &mut ComLoggerState) {
        let status = create_validation(
            state.file_name.as_str().unwrap_or_default(),
            state.hash_file_name.as_str().unwrap_or_default(),
        );
        if status != ValidateStatus::ValidationOk {
            self.log_file_validation_error(&state.hash_file_name, &state.file_name, status as u32);
        }
    }

    // -- Events ---------------------------------------------------------------

    fn log_file_open_error(&self, errornum: u32, file_name: &FileNameString) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_OPEN_ERROR,
            LogSeverity::WarningHi,
            &format!("Error {errornum} opening file {file_name}"),
            |buf: &mut LogBuffer| {
                fw_try!(buf.serialize_u32_be(errornum));
                file_name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_file_write_error(
        &self,
        errornum: u32,
        bytes_written: u32,
        bytes_to_write: u32,
        file_name: &FileNameString,
    ) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_WRITE_ERROR,
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

    fn log_file_validation_error(
        &self,
        validation_file: &FileNameString,
        file_name: &FileNameString,
        status: u32,
    ) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_VALIDATION_ERROR,
            LogSeverity::WarningLo,
            &format!(
                "The ComLogger failed to create a validation file {validation_file} for {file_name} with error {status}."
            ),
            |buf: &mut LogBuffer| {
                fw_try!(validation_file.serialize_to_truncated(
                    buf,
                    EVENT_STRING_SIZE,
                    Endianness::Big
                ));
                fw_try!(file_name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u32_be(status)
            },
        );
    }

    fn log_file_closed(&self, file_name: &FileNameString) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_CLOSED,
            LogSeverity::Diagnostic,
            &format!("File {file_name} closed successfully."),
            |buf: &mut LogBuffer| {
                file_name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_file_not_initialized(&self) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_NOT_INITIALIZED,
            LogSeverity::WarningLo,
            "Could not open ComLogger file. File not initialized",
            |_buf: &mut LogBuffer| SerializeStatus::Ok,
        );
    }
}

impl Drop for ComLogger {
    /// C++ destructor: close the file and write the sidecar, but emit NO
    /// `FileClosed` event (the C++ comment calls out the
    /// virtual-call-during-destruction hazard).
    fn drop(&mut self) {
        let Ok(state) = self.state.get_mut() else {
            return;
        };
        if state.file_mode == FileMode::Open {
            state.file.close();
            let _ = create_validation(
                state.file_name.as_str().unwrap_or_default(),
                state.hash_file_name.as_str().unwrap_or_default(),
            );
            state.file_mode = FileMode::Closed;
        }
    }
}

// -- Async input adapters ----------------------------------------------------

async_input_port_adapter! {
    /// `comIn` — ASYNC `Fw.Com` input: one com buffer to log.
    component: ComLogger;
    adapter: ComInAdapter;
    port: ComPort;
    input: pub com_in;
    deserialize: com_in_deserialize;
    handler: com_in_handler;
    base: active.queued;
    msg_type: ComLogger::MSG_TYPE_COM_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { buf data: ComBuffer, val context: u32 }
}

async_input_port_adapter! {
    /// `pingIn` — ASYNC `Svc.Ping` input.
    component: ComLogger;
    adapter: PingInAdapter;
    port: PingPort;
    input: pub ping_in;
    deserialize: ping_in_deserialize;
    handler: ping_in_handler;
    base: active.queued;
    msg_type: ComLogger::MSG_TYPE_PING_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val key: u32 }
}

async_input_port_adapter! {
    /// `cmdIn` — ASYNC `Fw.Cmd` input for `CloseFile`.
    component: ComLogger;
    adapter: CmdInAdapter;
    port: CmdPort;
    input: pub cmd_in;
    deserialize: cmd_in_deserialize;
    handler: cmd_in_handler;
    base: active.queued;
    msg_type: ComLogger::MSG_TYPE_CMD_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, buf args: CmdArgBuffer }
}

// -- Dispatch (the generated `doDispatch` switch) -----------------------------

impl ComponentDispatch for ComLogger {
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

impl ActiveComponent for ComLogger {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort, TimePort};
    use fprime_fw::{TextLogString, Time, TimeBase};
    use fprime_os::task::TASK_DEFAULT;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    const ID_BASE: u32 = 0x3300;

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    fn temp_dir(tag: &str) -> PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "fprime_rust_comlogger_{tag}_{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&path).unwrap();
        path
    }

    /// (id, severity, raw arg bytes)
    type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

    #[derive(Default)]
    struct Ground {
        events: Mutex<Vec<EventRecord>>,
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
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
            _id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            _text: &mut TextLogString,
        ) {
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

    impl PingPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.pings.lock().unwrap().push(key);
        }
    }

    /// Time source whose seconds advance on every call, so successive file
    /// names differ (the C++ names are timestamped at open).
    #[derive(Default)]
    struct TickingTime {
        seconds: AtomicU32,
    }

    impl TimePort for TickingTime {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            let seconds = self.seconds.fetch_add(1, Ordering::SeqCst);
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 100 + seconds, 42);
        }
    }

    fn build(comp: Arc<ComLogger>) -> (Arc<ComLogger>, Arc<Ground>) {
        let ground = Arc::new(Ground::default());
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.evt
            .time_out
            .connect(Arc::new(TickingTime::default()), 0);
        comp.ping_out.connect(ground.clone(), 0);
        comp.init(16);
        (comp, ground)
    }

    fn com_buffer(bytes: &[u8]) -> ComBuffer {
        let mut buf = ComBuffer::new();
        assert!(buf.set_buff(bytes).is_ok());
        buf
    }

    fn send_com(comp: &Arc<ComLogger>, bytes: &[u8]) {
        let mut data = com_buffer(bytes);
        let port = comp.com_in(0);
        port.target.invoke(port.port_num, &mut data, 0);
    }

    fn send_close_cmd(comp: &Arc<ComLogger>, seq: u32, arg_bytes: &[u8]) {
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(arg_bytes).is_ok());
        let port = comp.cmd_in(0);
        port.target.invoke(
            port.port_num,
            ID_BASE + ComLogger::OPCODE_CLOSE_FILE,
            seq,
            &mut args,
        );
    }

    fn run(comp: &Arc<ComLogger>) {
        comp.active.start(comp, 100, TASK_DEFAULT, TASK_DEFAULT);
        comp.active.exit();
        assert_eq!(
            comp.active.join(),
            fprime_os::task::Status::OpOk,
            "component thread joined"
        );
    }

    fn com_files(dir: &PathBuf) -> Vec<String> {
        let mut names: Vec<String> = std::fs::read_dir(dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().to_string())
            .filter(|n| n.ends_with(".com"))
            .collect();
        names.sort();
        names
    }

    #[test]
    fn records_are_length_prefixed_big_endian_and_the_name_is_timestamped() {
        let dir = temp_dir("records");
        let prefix = dir.join("log").to_str().unwrap().to_string();
        let (comp, ground) = build(ComLogger::with_log_file("comLog", &prefix, 1024, true));

        send_com(&comp, &[0xAA, 0xBB, 0xCC]);
        send_com(&comp, &[0x01]);
        send_close_cmd(&comp, 7, &[]);
        run(&comp);

        // File name: <prefix>_<timeBase>_<sec>_<usec:06>.com; the first
        // getTime() call happens at open.
        let expected_name = format!("{prefix}_2_100_000042.com");
        assert_eq!(
            std::fs::read(&expected_name).unwrap(),
            vec![0x00, 0x03, 0xAA, 0xBB, 0xCC, 0x00, 0x01, 0x01]
        );

        // Sidecar: 4 BIG-ENDIAN bytes of the CRC-32 of the .com file.
        let crc = Hash::hash_u32(&[0x00, 0x03, 0xAA, 0xBB, 0xCC, 0x00, 0x01, 0x01]);
        assert_eq!(
            std::fs::read(format!("{expected_name}.CRC32")).unwrap(),
            crc.to_be_bytes().to_vec()
        );

        // FileClosed (DIAGNOSTIC) carries the file name, and the command
        // answered exactly once.
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].0, ID_BASE + ComLogger::EVENTID_FILE_CLOSED);
        assert_eq!(events[0].1, LogSeverity::Diagnostic);
        let mut expected_args = vec![0x00, expected_name.len() as u8];
        expected_args.extend_from_slice(expected_name.as_bytes());
        assert_eq!(events[0].2, expected_args);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(ID_BASE + ComLogger::OPCODE_CLOSE_FILE, 7, CmdResponse::Ok)]
        );
    }

    #[test]
    fn without_store_buffer_length_records_are_raw_bytes() {
        let dir = temp_dir("raw");
        let prefix = dir.join("raw").to_str().unwrap().to_string();
        let (comp, _ground) = build(ComLogger::with_log_file("comLog", &prefix, 1024, false));
        send_com(&comp, &[1, 2, 3]);
        send_com(&comp, &[4, 5]);
        run(&comp);
        let name = format!("{prefix}_2_100_000042.com");
        assert_eq!(std::fs::read(name).unwrap(), vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn rotation_uses_strict_greater_than_including_the_length_prefix() {
        let dir = temp_dir("rotate");
        let prefix = dir.join("rot").to_str().unwrap().to_string();
        // maxFileSize 10: two 5-byte records (2 + 3) fit exactly; the third
        // projects 15 > 10 and rotates.
        let (comp, _ground) = build(ComLogger::with_log_file("comLog", &prefix, 10, true));
        send_com(&comp, &[1, 2, 3]);
        send_com(&comp, &[4, 5, 6]);
        send_com(&comp, &[7, 8, 9]);
        run(&comp);

        // Two files: the first holds exactly maxFileSize bytes, the second
        // starts with the record that would have overflowed it. (The second
        // file's timestamp depends on how many getTime() calls the event
        // path made, so the names are discovered, not predicted.)
        let files = com_files(&dir);
        assert_eq!(files.len(), 2, "one rotation: {files:?}");
        let first = dir.join(&files[0]).to_str().unwrap().to_string();
        let second = dir.join(&files[1]).to_str().unwrap().to_string();
        assert_eq!(first, format!("{prefix}_2_100_000042.com"));
        assert_eq!(
            std::fs::read(&first).unwrap(),
            vec![0, 3, 1, 2, 3, 0, 3, 4, 5, 6]
        );
        assert_eq!(std::fs::read(&second).unwrap(), vec![0, 3, 7, 8, 9]);
        // The rotated-away file gets its sidecar immediately; the still-open
        // one gets it when the component is dropped.
        assert!(std::fs::metadata(format!("{first}.CRC32")).is_ok());
    }

    #[test]
    fn dropping_the_component_closes_and_hashes_without_an_event() {
        let dir = temp_dir("drop");
        let prefix = dir.join("drp").to_str().unwrap().to_string();
        let (comp, ground) = build(ComLogger::with_log_file("comLog", &prefix, 1024, true));
        send_com(&comp, &[9, 9]);
        run(&comp);
        let name = format!("{prefix}_2_100_000042.com");
        assert!(comp.is_file_open());
        drop(comp);

        assert_eq!(std::fs::read(&name).unwrap(), vec![0, 2, 9, 9]);
        let crc = Hash::hash_u32(&[0, 2, 9, 9]);
        assert_eq!(
            std::fs::read(format!("{name}.CRC32")).unwrap(),
            crc.to_be_bytes().to_vec()
        );
        // Gotcha: the destructor path emits NO FileClosed event.
        assert!(ground.events.lock().unwrap().is_empty());
    }

    #[test]
    fn uninitialized_logger_drops_buffers_with_a_throttled_event() {
        let (comp, ground) = build(ComLogger::new("comLog"));
        for _ in 0..8 {
            send_com(&comp, &[1]);
        }
        run(&comp);
        let events = ground.events.lock().unwrap();
        // FPP throttle 5: exactly five events for eight dropped buffers.
        assert_eq!(
            events.len(),
            ComLogger::FILE_NOT_INITIALIZED_THROTTLE as usize
        );
        for event in events.iter() {
            assert_eq!(event.0, ID_BASE + ComLogger::EVENTID_FILE_NOT_INITIALIZED);
            assert_eq!(event.1, LogSeverity::WarningLo);
            assert!(event.2.is_empty(), "the event has no arguments");
        }
        assert!(!comp.is_file_open());
    }

    #[test]
    fn open_error_is_emitted_once_and_the_record_is_dropped() {
        let dir = temp_dir("openerr");
        // A prefix pointing into a non-existent directory makes every open
        // fail.
        let prefix = dir.join("missing").join("x").to_str().unwrap().to_string();
        let (comp, ground) = build(ComLogger::with_log_file("comLog", &prefix, 1024, true));
        send_com(&comp, &[1]);
        send_com(&comp, &[2]);
        send_com(&comp, &[3]);
        run(&comp);
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 1, "one-shot latch: {events:?}");
        assert_eq!(events[0].0, ID_BASE + ComLogger::EVENTID_FILE_OPEN_ERROR);
        assert_eq!(events[0].1, LogSeverity::WarningHi);
        // [errornum u32][u16 len][name bytes]
        assert_eq!(
            events[0].2[..4],
            (file::Status::DoesntExist as u32).to_be_bytes()
        );
        assert!(!comp.is_file_open());
    }

    #[test]
    fn close_file_command_on_a_closed_logger_still_answers_ok() {
        let (comp, ground) = build(ComLogger::new("comLog"));
        comp.reg_commands();
        send_close_cmd(&comp, 1, &[]);
        send_close_cmd(&comp, 2, &[0xFF]); // residual byte
        run(&comp);
        assert_eq!(
            *ground.regs.lock().unwrap(),
            vec![ID_BASE + ComLogger::OPCODE_CLOSE_FILE]
        );
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (ID_BASE + ComLogger::OPCODE_CLOSE_FILE, 1, CmdResponse::Ok),
                (
                    ID_BASE + ComLogger::OPCODE_CLOSE_FILE,
                    2,
                    CmdResponse::FormatError
                ),
            ]
        );
        // No file was ever opened, so no FileClosed event.
        assert!(ground.events.lock().unwrap().is_empty());
    }

    #[test]
    fn unknown_opcode_answers_invalid_opcode() {
        let (comp, ground) = build(ComLogger::new("comLog"));
        let mut args = CmdArgBuffer::new();
        let port = comp.cmd_in(0);
        port.target
            .invoke(port.port_num, ID_BASE + 0x42, 3, &mut args);
        run(&comp);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(ID_BASE + 0x42, 3, CmdResponse::InvalidOpcode)]
        );
    }

    #[test]
    fn ping_is_echoed() {
        let (comp, ground) = build(ComLogger::new("comLog"));
        let port = comp.ping_in(0);
        port.target.invoke(port.port_num, 0xFEED);
        run(&comp);
        assert_eq!(*ground.pings.lock().unwrap(), vec![0xFEED]);
    }

    #[test]
    fn com_in_envelope_bytes_are_byte_exact() {
        let (comp, _ground) = build(ComLogger::new("comLog"));
        send_com(&comp, &[0xDE, 0xAD]);
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
            dest[..size as usize].to_vec(),
            vec![
                0x00, 0x00, 0x00, 0x01, // msg_type = MSG_TYPE_COM_IN
                0x00, 0x00, // port_num
                0x00, 0x02, // nested ComBuffer length
                0xDE, 0xAD, // the com buffer
                0x00, 0x00, 0x00, 0x00, // context
            ]
        );
        assert_eq!(priority, QUEUE_PRIORITY);
    }

    #[test]
    fn create_validation_writes_a_big_endian_sidecar() {
        let dir = temp_dir("validate");
        let path = dir.join("data.bin");
        std::fs::write(&path, b"123456789").unwrap();
        let name = path.to_str().unwrap();
        let hash_name = format!("{name}.CRC32");
        assert_eq!(
            create_validation(name, &hash_name),
            ValidateStatus::ValidationOk
        );
        // The canonical CRC-32 test vector, big-endian.
        assert_eq!(
            std::fs::read(&hash_name).unwrap(),
            vec![0xCB, 0xF4, 0x39, 0x26]
        );
    }

    #[test]
    fn create_validation_on_a_missing_file_reports_the_file_status() {
        let dir = temp_dir("validate_missing");
        let name = dir.join("nope.bin").to_str().unwrap().to_string();
        assert_eq!(
            create_validation(&name, &format!("{name}.CRC32")),
            ValidateStatus::FileDoesntExist
        );
    }

    #[test]
    fn validation_error_is_reported_as_an_event() {
        // Close a file whose sidecar cannot be written: make the hash file
        // name point into a directory that does not exist by removing the
        // working directory after the file was opened.
        let dir = temp_dir("valerr");
        let prefix = dir.join("v").to_str().unwrap().to_string();
        let (comp, ground) = build(ComLogger::with_log_file("comLog", &prefix, 1024, true));
        send_com(&comp, &[1, 2]);
        comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);
        comp.active.exit();
        assert_eq!(comp.active.join(), fprime_os::task::Status::OpOk);
        // Delete the .com file behind the component's back: the hash of a
        // missing file fails, which is the FileValidationError path.
        std::fs::remove_file(format!("{prefix}_2_100_000042.com")).unwrap();
        send_close_cmd(&comp, 1, &[]);
        run(&comp);

        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 2, "validation error then FileClosed");
        assert_eq!(
            events[0].0,
            ID_BASE + ComLogger::EVENTID_FILE_VALIDATION_ERROR
        );
        assert_eq!(events[0].1, LogSeverity::WarningLo);
        // Trailing argument is the ValidateFile status ordinal.
        let args = &events[0].2;
        assert_eq!(
            args[args.len() - 4..],
            (ValidateStatus::FileDoesntExist as u32).to_be_bytes()
        );
        assert_eq!(events[1].0, ID_BASE + ComLogger::EVENTID_FILE_CLOSED);
    }

    #[test]
    fn dictionary_ids_match_the_fpp_model() {
        assert_eq!(ComLogger::OPCODE_CLOSE_FILE, 0x00);
        assert_eq!(ComLogger::EVENTID_FILE_OPEN_ERROR, 0x00);
        assert_eq!(ComLogger::EVENTID_FILE_WRITE_ERROR, 0x01);
        assert_eq!(ComLogger::EVENTID_FILE_VALIDATION_ERROR, 0x02);
        assert_eq!(ComLogger::EVENTID_FILE_CLOSED, 0x03);
        assert_eq!(ComLogger::EVENTID_FILE_NOT_INITIALIZED, 0x04);
        assert_eq!(ComLogger::MSG_TYPE_COM_IN, 1);
        assert_eq!(ComLogger::MSG_TYPE_PING_IN, 2);
        assert_eq!(ComLogger::MSG_TYPE_CMD_IN, 3);
    }
}
