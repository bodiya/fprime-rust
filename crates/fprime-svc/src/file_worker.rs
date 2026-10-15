//! # Svc::FileWorker — off-thread file reads, writes and CRC checks (active component)
//!
//! C++ sources: `Svc/FileWorker/FileWorker.{cpp,hpp,fpp}`,
//! `Svc/FileWorker/FileWorkerTypes.hpp`,
//! `Svc/Ports/FilePorts/FileWorkerPorts.fpp`; SDD `Svc/FileWorker/docs/sdd.md`;
//! tests `Svc/FileWorker/test/ut/FileWorker{,Err}Tester.cpp` (ported in
//! `file_worker/tests.rs`).
//!
//! A client hands the worker a request on an async port and is told the
//! outcome on the matching `*DoneOut` port:
//! - `readIn(path, buffer)`: check `path` against its `<path>.CRC32` sidecar
//!   (`Utils::verify_checksum`), then fill the buffer with the first
//!   `buffer.size()` bytes of the file; `readDoneOut(status, size)`.
//! - `writeIn(path, buffer, offsetBytes, append)`: write the buffer from
//!   `offsetBytes` on to `path` (appending, or over the start of the file),
//!   then write the sidecar (extending the stored CRC when appending);
//!   `writeDoneOut(status, bytesWritten)`.
//! - `verifyIn(path, crc)`: check `path` against its sidecar AND against
//!   `crc`; `verifyDoneOut(status, fileSize)`.
//! - `cancelIn` (guarded, caller's thread): abort the read or write in
//!   progress at its next chunk boundary.
//!
//! File I/O goes through `fprime_os` `File` / `filesystem` in chunks of
//! [`FileWorker::configure`]'s size (default 4096), each chunk checked for
//! cancellation and against a timeout of one second per chunk.
//!
//! ## `Fw::Buffer` ownership (deviation)
//!
//! Upstream passes `ref buffer: Fw.Buffer` on the async `readIn`/`writeIn`
//! ports: the queue message carries the buffer's pointer, the worker fills
//! (or reads) the client's memory, and the client learns the outcome on
//! `Svc.SignalDone(status, sizeBytes)`, which has no buffer. The Rust
//! `Buffer` owns its storage, so it MOVES in on `readIn`/`writeIn` (through
//! the escrow, like every async buffer port here) and moves back to the
//! client as an extra argument of the done port —
//! [`SignalDonePort::invoke`]`(status, size_bytes, buffer)`. Every
//! `readDoneOut`/`writeDoneOut` call returns the request's buffer, whatever
//! the status (including invalid input); `verifyDoneOut` has none and
//! passes [`Buffer::empty`]. The port list, the call order and the
//! statuses are upstream's.
//!
//! ## C++-parity notes
//! - Input validation (nasa/fprime#5803 and the input-validation work after
//!   it): an empty path, a path that leaves no room for `.CRC32` in a
//!   `FileNameString` (longer than 234 bytes), an invalid buffer, or an
//!   offset past the end of the buffer is answered `INVALID_INPUT` with an
//!   `InvalidInput(handler, issue)` event, never an assert — so the CRC
//!   helpers' `FAILED_FILE_NAME_TOO_LONG` is never reached from here.
//! - `NOT_IDLE` (and `NotInIdle`) can only be seen when the state is
//!   changed behind the handlers' back: requests are serialized on the
//!   component thread and every handler returns to `IDLE`.
//! - Writes open the file `OPEN_WRITE` (created if missing, NOT truncated)
//!   or `OPEN_APPEND`; a non-append write over a longer file leaves its
//!   tail while the sidecar holds the CRC of the written bytes only.
//! - A zero-length write (`offsetBytes == buffer size`) succeeds without
//!   opening the file or touching the sidecar.
//! - `readDoneOut` reports the buffer size only for `DONE_READ`, 0
//!   otherwise; `writeDoneOut` reports `size - offsetBytes` for
//!   `DONE_WRITE`, 0 for `FAILED_TO_WRITE`. A failed sidecar write does not
//!   change the write status.
//! - A cancel or timeout is checked AFTER each chunk transfer and the chunk
//!   is not counted, so a write cancelled after its first chunk reports 0
//!   bytes written although the chunk is on disk.
//! - `ReadCompleted` is emitted after every read attempt, failed or not.
//! - `verifyIn` runs all three checks and reports the last failure; when
//!   the sidecar check fails before computing the CRC, the computed CRC is
//!   0, so an expected `crc` of 0 raises no `CrcVerificationError`.
//! - Appending reads the stored CRC from the sidecar to extend it; a
//!   sidecar shorter than 4 bytes is an `FW_ASSERT` (upstream
//!   `Utils::Hash::setHashValue`), not an event.
//! - The elapsed time is computed as upstream's
//!   `seconds * 1000000 + useconds` in 32-bit arithmetic, which wraps after
//!   about 71 minutes; `Fw::Time::sub` asserts if the time base changes or
//!   time runs backwards during a transfer.
//! - Seven warnings use FPP `throttle 2 every {seconds=10}`: after two
//!   emissions the event is suppressed until 10 s (of the component's time
//!   source) have passed since the first of them.
//! - `cmdIn` exists but the component has no commands: an opcode below the
//!   id base asserts, any other is ignored (the generated `cmdIn`
//!   dispatcher); `tlmOut`, `prmGetOut` and `prmSetOut` are never used.

use fprime_comp::escrow::BufferEscrow;
use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, ComponentDispatch, EventGlue, MsgDispatchStatus,
    OutputPort, PortRef, PrmGlue, QueueFullPolicy, TlmGlue, async_input_port_adapter,
    component_msg_types, input_port_adapter, msg,
};
use fprime_config::{
    FW_LOG_STRING_MAX_SIZE, FwEnumStoreType, FwEventIdType, FwIndexType, FwOpcodeType,
    FwQueuePriorityType, FwSizeType,
};
use fprime_fw::sync::{Arc, AtomicBool, AtomicU32, AtomicU64, Mutex, Ordering};
use fprime_fw::{
    Buffer, CmdArgBuffer, Endianness, FileNameString, FwString, LinearBuffer, LogBuffer,
    LogSeverity, SerBuf, SerBufAny, SerializeStatus, StringFormatStatus, TextLogString, Time,
    TimeInterval, fw_assert, fw_try,
};
use fprime_os::file::{self, File, Mode, WaitType};
use fprime_os::filesystem;
use fprime_utils::crc_checker::{self, CrcStat};
use fprime_utils::hash::{HASH_DIGEST_LENGTH, Hash, HashBuffer};

// ---------------------------------------------------------------------------
// Svc/Ports/FilePorts/FileWorkerPorts.fpp
// ---------------------------------------------------------------------------

/// `Svc.FileRead` — `port FileRead(path: string size FileNameStringSize,
/// ref buffer: Fw.Buffer)`. The buffer moves to the worker and comes back
/// on [`SignalDonePort`] (see the module docs).
pub trait FileReadPort: Send + Sync {
    /// Request a read of `path` into `buffer`.
    fn invoke(&self, port_num: FwIndexType, path: &FileNameString, buffer: Buffer);
}

/// `Svc.FileWrite` — `port FileWrite(path, ref buffer: Fw.Buffer,
/// offsetBytes: FwSizeType, append: bool)`. The buffer moves to the worker
/// and comes back on [`SignalDonePort`].
pub trait FileWritePort: Send + Sync {
    /// Request a write of `buffer[offset_bytes..]` to `path`.
    fn invoke(
        &self,
        port_num: FwIndexType,
        path: &FileNameString,
        buffer: Buffer,
        offset_bytes: FwSizeType,
        append: bool,
    );
}

/// `Svc.SignalDone` — `port SignalDone(status: U32, sizeBytes:
/// FwSizeType)`, plus the request's buffer handed back to the client
/// (deviation, see the module docs; [`Buffer::empty`] for a verify).
pub trait SignalDonePort: Send + Sync {
    /// Report a finished request.
    fn invoke(&self, port_num: FwIndexType, status: u32, size_bytes: FwSizeType, buffer: Buffer);
}

/// `Svc.CancelStatus` — `port CancelStatus()`.
pub trait CancelStatusPort: Send + Sync {
    /// Cancel the operation in progress.
    fn invoke(&self, port_num: FwIndexType);
}

/// `Svc.VerifyStatus` — `port VerifyStatus(path: string size
/// FileNameStringSize, crc: U32)`.
pub trait VerifyStatusPort: Send + Sync {
    /// Request a verification of `path` against `crc`.
    fn invoke(&self, port_num: FwIndexType, path: &FileNameString, crc: u32);
}

// ---------------------------------------------------------------------------
// FileWorkerTypes.hpp
// ---------------------------------------------------------------------------

/// C++ `Svc::FileWorkerState` (exact discriminants; reported as the
/// `NotInIdle` argument).
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileWorkerState {
    /// `FW_STATE_IDLE` — ready for a request.
    Idle = 0,
    /// `FW_STATE_WRITING`.
    Writing = 1,
    /// `FW_STATE_READING`.
    Reading = 2,
}

impl FileWorkerState {
    fn from_u32(raw: u32) -> Self {
        match raw {
            0 => Self::Idle,
            1 => Self::Writing,
            _ => Self::Reading,
        }
    }
}

/// C++ `Svc::FileWorkerStatus` (exact discriminants) — the `status`
/// argument of the done ports.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileWorkerStatus {
    /// `FW_STATUS_NOT_IDLE`.
    NotIdle = 0,
    /// `FW_STATUS_FILE_NOT_OPEN` (unused upstream).
    FileNotOpen = 1,
    /// `FW_STATUS_FAILED_TO_OPEN`.
    FailedToOpen = 2,
    /// `FW_STATUS_FAILED_TO_WRITE`.
    FailedToWrite = 3,
    /// `FW_STATUS_FAILED_TO_READ`.
    FailedToRead = 4,
    /// `FW_STATUS_FAILED_TO_START` (unused upstream).
    FailedToStart = 5,
    /// `FW_STATUS_CANCELLED_WRITE` (unused upstream).
    CancelledWrite = 6,
    /// `FW_STATUS_FAILED_CRC`.
    FailedCrc = 7,
    /// `FW_STATUS_FAILED_FILE_SIZE`.
    FailedFileSize = 8,
    /// `FW_STATUS_FAILED_FILE_SIZE_CONV` (unused upstream).
    FailedFileSizeConv = 9,
    /// `FW_STATUS_FAILED_GET_BUF` (unused upstream).
    FailedGetBuf = 10,
    /// `FW_STATUS_START_WRITE` (unused upstream).
    StartWrite = 11,
    /// `FW_STATUS_DONE_WRITE`.
    DoneWrite = 12,
    /// `FW_STATUS_START_READ` (unused upstream).
    StartRead = 13,
    /// `FW_STATUS_DONE_READ`.
    DoneRead = 14,
    /// `FW_STATUS_DONE` — a passed verification.
    Done = 15,
    /// `FW_STATUS_INVALID_INPUT`.
    InvalidInput = 16,
}

/// C++ `Svc::FileWorkerReadStatus` (exact discriminants).
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileWorkerReadStatus {
    /// `FW_READ_DONE`.
    Done = 0,
    /// `FW_READ_ERROR`.
    Error = 1,
    /// `FW_READ_ABORT`.
    Abort = 2,
    /// `FW_READ_TIMEOUT`.
    Timeout = 3,
    /// `FW_READ_UNKNOWN` — the read loop ran out of iterations.
    Unknown = 4,
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

/// Queue message size: the largest async invocation.
///
/// `writeIn` = 6 (envelope) + 2 + 240 (`FileNameString`) + 8 (escrow token,
/// in the slot C++ uses for the buffer) + 8 (`offsetBytes`) + 1 (`append`)
/// = 265; `readIn` = 6 + 242 + 8 = 256; `verifyIn` = 6 + 242 + 4 = 252.
pub const QUEUE_MSG_SIZE: usize = 265;

/// Queue priority for every async input (no FPP `priority` qualifier).
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

/// Event file-name arguments are `string size FileNameStringSize` (240),
/// serialized truncated to `FW_LOG_STRING_MAX_SIZE`.
const EVENT_FILE_NAME_SIZE: usize = if FW_LOG_STRING_MAX_SIZE < FileNameString::CAPACITY {
    FW_LOG_STRING_MAX_SIZE
} else {
    FileNameString::CAPACITY
};

/// FPP `throttle 2` on the seven throttled warnings.
const THROTTLE_LIMIT: u32 = 2;
/// FPP `every {seconds=10}` on the seven throttled warnings.
const THROTTLE_SECONDS: u32 = 10;

/// Index of each throttled event in [`FileWorker`]'s throttle table.
#[derive(Clone, Copy)]
enum Throttled {
    NotInIdle,
    CrcFailed,
    ReadFailedFileSize,
    OpenFileError,
    WriteFileError,
    WriteValidationOpenError,
    WriteValidationReadError,
}

const NUM_THROTTLED: usize = 7;

/// The generated per-event `m_<Name>Throttle` counter and
/// `m_<Name>ThrottleTime` stamp.
#[derive(Debug, Clone, Copy, Default)]
struct TimedThrottle {
    count: u32,
    time: Time,
}

/// `Svc::FileWorker`.
pub struct FileWorker {
    /// Active core: PassiveBase + queue + task.
    pub active: ActiveBase,
    /// `cmdRegOut` / `cmdResponseOut` (the component has no commands).
    pub cmd: CmdGlue,
    /// Event ports (`logOut`, `logTextOut`) and the `timeCaller` port, which
    /// also times transfers.
    pub evt: EventGlue,
    /// `tlmOut` (no channels).
    pub tlm: TlmGlue,
    /// `prmGetOut` / `prmSetOut` (no parameters).
    pub prm: PrmGlue,
    /// `writeDoneOut: Svc.SignalDone`.
    pub write_done_out: OutputPort<dyn SignalDonePort>,
    /// `readDoneOut: Svc.SignalDone`.
    pub read_done_out: OutputPort<dyn SignalDonePort>,
    /// `verifyDoneOut: Svc.SignalDone`.
    pub verify_done_out: OutputPort<dyn SignalDonePort>,
    /// Escrow for the async `readIn` / `writeIn` buffers.
    escrow: BufferEscrow,
    /// C++ `m_state` ([`FileWorkerState`]); only the component thread
    /// changes it in flight.
    state: AtomicU32,
    /// C++ `std::atomic<bool> m_abort` (relaxed, as upstream).
    abort: AtomicBool,
    /// C++ `m_chunkSize`.
    chunk_size: AtomicU64,
    /// The guarded-port mutex (C++ `m_guardedPortMutex`), taken by
    /// `cancelIn`.
    guard: Mutex<()>,
    /// The `throttle 2 every {seconds=10}` state (C++ `m_eventLock` + the
    /// generated counters).
    throttles: Mutex<[TimedThrottle; NUM_THROTTLED]>,
}

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel), in the C++ generated
    /// order.
    impl FileWorker {
        /// `readIn` async input port.
        MSG_TYPE_READ_IN,
        /// `verifyIn` async input port.
        MSG_TYPE_VERIFY_IN,
        /// `writeIn` async input port.
        MSG_TYPE_WRITE_IN,
    }
}

impl FileWorker {
    /// C++ `BLOCK_SIZE_BYTES`: the default chunk size.
    pub const BLOCK_SIZE_BYTES: u64 = 4096;
    /// C++ `TIMEOUT_MS`: the timeout per chunk — in MICROseconds despite
    /// the name (one second).
    pub const TIMEOUT_MS: u32 = 1_000_000;
    /// C++ `MAX_LOOP_ITERATIONS`: extra write-loop iterations allowed for
    /// short writes.
    pub const MAX_LOOP_ITERATIONS: u32 = 32;

    /// `NotInIdle(currState: U32)` — WARNING_HI, throttled.
    pub const EVENTID_NOTINIDLE: FwEventIdType = 0x00;
    /// `CrcFailed(crcStat: U32)` — WARNING_HI, throttled.
    pub const EVENTID_CRCFAILED: FwEventIdType = 0x01;
    /// `CrcVerificationError(crcExp: U32, crcCalculated: U32)` —
    /// WARNING_LO.
    pub const EVENTID_CRCVERIFICATIONERROR: FwEventIdType = 0x02;
    /// `ReadFailedFileSize(fsStat: U32)` — WARNING_HI, throttled.
    pub const EVENTID_READFAILEDFILESIZE: FwEventIdType = 0x03;
    /// `OpenFileError(fileName, fsStat: U32)` — WARNING_HI, throttled.
    pub const EVENTID_OPENFILEERROR: FwEventIdType = 0x04;
    /// `ReadBegin(fileSize, fileName)` — ACTIVITY_LO.
    pub const EVENTID_READBEGIN: FwEventIdType = 0x05;
    /// `ReadCompleted(fileSize, fileName)` — ACTIVITY_LO.
    pub const EVENTID_READCOMPLETED: FwEventIdType = 0x06;
    /// `ReadError(bytesRead, readSize, fileName)` — WARNING_HI.
    pub const EVENTID_READERROR: FwEventIdType = 0x07;
    /// `ReadAborted(bytesRead, readSize, fileName)` — WARNING_LO.
    pub const EVENTID_READABORTED: FwEventIdType = 0x08;
    /// `ReadTimeout(bytesRead, readSize, fileName, timeout: U64)` —
    /// WARNING_HI.
    pub const EVENTID_READTIMEOUT: FwEventIdType = 0x09;
    /// `WriteBegin(writeSize, fileName)` — ACTIVITY_LO.
    pub const EVENTID_WRITEBEGIN: FwEventIdType = 0x0A;
    /// `WriteCompleted(writeSize, fileName)` — ACTIVITY_LO.
    pub const EVENTID_WRITECOMPLETED: FwEventIdType = 0x0B;
    /// `WriteFileError(bytesWritten, writeSize, fileName, status: I32)` —
    /// WARNING_HI, throttled.
    pub const EVENTID_WRITEFILEERROR: FwEventIdType = 0x0C;
    /// `WriteValidationOpenError(hashFileName, status: I32)` — WARNING_HI,
    /// throttled.
    pub const EVENTID_WRITEVALIDATIONOPENERROR: FwEventIdType = 0x0D;
    /// `WriteValidationReadError(hashFileName, status: I32)` — WARNING_HI,
    /// throttled.
    pub const EVENTID_WRITEVALIDATIONREADERROR: FwEventIdType = 0x0E;
    /// `WriteValidationError(hashFileName, bytesWritten, hashSize)` —
    /// WARNING_LO.
    pub const EVENTID_WRITEVALIDATIONERROR: FwEventIdType = 0x0F;
    /// `WriteTimeout(bytesWritten, writeSize, fileName, timeout: U64)` —
    /// WARNING_HI.
    pub const EVENTID_WRITETIMEOUT: FwEventIdType = 0x10;
    /// `WriteAborted(bytesWritten, writeSize, fileName)` — WARNING_LO.
    pub const EVENTID_WRITEABORTED: FwEventIdType = 0x11;
    /// `InvalidInput(handler: string size 16, issue: string size 32)` —
    /// WARNING_HI.
    pub const EVENTID_INVALIDINPUT: FwEventIdType = 0x12;
    /// FPP `throttle 2` (all seven throttled events).
    pub const EVENTID_THROTTLE: u32 = THROTTLE_LIMIT;

    /// Construct the component: idle, not aborting, chunk size
    /// [`BLOCK_SIZE_BYTES`](Self::BLOCK_SIZE_BYTES).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            prm: PrmGlue::new(),
            write_done_out: OutputPort::new(),
            read_done_out: OutputPort::new(),
            verify_done_out: OutputPort::new(),
            escrow: BufferEscrow::new(),
            state: AtomicU32::new(FileWorkerState::Idle as u32),
            abort: AtomicBool::new(false),
            chunk_size: AtomicU64::new(Self::BLOCK_SIZE_BYTES),
            guard: Mutex::new(()),
            throttles: Mutex::new([TimedThrottle::default(); NUM_THROTTLED]),
        })
    }

    /// C++ `configure(chunkSize)`: the file I/O chunk size (optional;
    /// timeouts scale with the chunk count). Asserts `chunk_size > 0`.
    pub fn configure(&self, chunk_size: u64) {
        fw_assert!(chunk_size > 0);
        self.chunk_size.store(chunk_size, Ordering::Relaxed);
    }

    /// The component has no commands; kept for topology uniformity (the
    /// generated `regCommands()` registers nothing).
    pub fn reg_commands(&self) {}

    /// Create the component queue (topology phase 4).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    /// The current state.
    #[must_use]
    pub fn state(&self) -> FileWorkerState {
        FileWorkerState::from_u32(self.state.load(Ordering::Relaxed))
    }

    fn set_state(&self, state: FileWorkerState) {
        self.state.store(state as u32, Ordering::Relaxed);
    }

    fn id_base(&self) -> u32 {
        self.active.queued.base.get_id_base()
    }

    fn chunk_size(&self) -> u64 {
        let chunk_size = self.chunk_size.load(Ordering::Relaxed);
        fw_assert!(chunk_size > 0);
        chunk_size
    }

    /// `ceil(size / chunk) * TIMEOUT_MS` and the chunk count.
    fn chunks_and_timeout(&self, size: FwSizeType) -> (FwSizeType, u64) {
        let chunk_size = self.chunk_size();
        let mut num_chunks = size / chunk_size;
        if !size.is_multiple_of(chunk_size) {
            num_chunks += 1;
        }
        (
            num_chunks,
            num_chunks.wrapping_mul(u64::from(Self::TIMEOUT_MS)),
        )
    }

    // -- Done ports -----------------------------------------------------------

    fn read_done(&self, status: FileWorkerStatus, size: FwSizeType, buffer: Buffer) {
        let p = self.read_done_out.get();
        p.target.invoke(p.port_num, status as u32, size, buffer);
    }

    fn write_done(&self, status: FileWorkerStatus, size: FwSizeType, buffer: Buffer) {
        let p = self.write_done_out.get();
        p.target.invoke(p.port_num, status as u32, size, buffer);
    }

    fn verify_done(&self, status: FileWorkerStatus, size: FwSizeType) {
        let p = self.verify_done_out.get();
        p.target
            .invoke(p.port_num, status as u32, size, Buffer::empty());
    }

    // -- Handlers ---------------------------------------------------------------

    /// `cancelIn_handler` (guarded, caller's thread).
    fn cancel_in_handler(&self, _port_num: FwIndexType) {
        let _guard = self.guard.lock();
        self.abort.store(true, Ordering::Relaxed);
    }

    /// The generated synchronous `cmdIn_handlerBase` of a component with no
    /// commands: assert the opcode is not below the id base, then ignore it.
    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        _cmd_seq: u32,
        _args: &mut CmdArgBuffer,
    ) {
        let id_base = self.id_base();
        fw_assert!(op_code >= id_base, op_code as i32, id_base as i32);
    }

    /// `readIn_handler`.
    fn read_in_handler(&self, _port_num: FwIndexType, path: &FileNameString, mut buffer: Buffer) {
        if path.is_empty() {
            self.log_invalid_input("readIn", "empty path");
            self.read_done(FileWorkerStatus::InvalidInput, 0, buffer);
            return;
        }
        // The path must leave room for the hash-file extension appended by
        // the CRC helpers.
        if !Self::path_fits_with_hash_extension(path) {
            self.log_invalid_input("readIn", "path too long");
            self.read_done(FileWorkerStatus::InvalidInput, 0, buffer);
            return;
        }
        if !buffer.is_valid() {
            self.log_invalid_input("readIn", "invalid buffer");
            self.read_done(FileWorkerStatus::InvalidInput, 0, buffer);
            return;
        }

        let file_name = path.as_str().unwrap_or_default();
        let mut file_size: FwSizeType = 0;

        let state = self.state.load(Ordering::Relaxed);
        if state != FileWorkerState::Idle as u32 {
            self.log_not_in_idle(state);
            self.read_done(FileWorkerStatus::NotIdle, 0, buffer);
            return;
        }

        // A new request overrides any leftover abort.
        self.abort.store(false, Ordering::Relaxed);
        self.set_state(FileWorkerState::Reading);

        let mut crc_from_file = 0u32;
        let mut crc_calculated = 0u32;
        let crc_stat =
            crc_checker::verify_checksum(file_name, &mut crc_from_file, &mut crc_calculated);
        if crc_stat != CrcStat::PassedFileCrcCheck {
            self.log_crc_failed(crc_stat as u32);
            self.read_done(FileWorkerStatus::FailedCrc, 0, buffer);
            self.set_state(FileWorkerState::Idle);
            return;
        }

        // The file may change between the CRC check and here.
        let fs_stat = filesystem::get_file_size(file_name, &mut file_size);
        if fs_stat != filesystem::Status::OpOk {
            self.log_read_failed_file_size(fs_stat as u32);
            self.read_done(FileWorkerStatus::FailedFileSize, 0, buffer);
            self.set_state(FileWorkerState::Idle);
            return;
        }

        let worker_stat = self.read_buffer_from_file(&mut buffer, file_name);

        // 0 bytes on a failed or aborted read, so the done signal does not
        // imply success.
        let size = if worker_stat == FileWorkerStatus::DoneRead {
            buffer.size() as FwSizeType
        } else {
            0
        };
        self.read_done(worker_stat, size, buffer);
        self.set_state(FileWorkerState::Idle);
    }

    /// `verifyIn_handler` (does not look at or change the state).
    fn verify_in_handler(&self, _port_num: FwIndexType, path: &FileNameString, crc: u32) {
        if path.is_empty() {
            self.log_invalid_input("verifyIn", "empty path");
            self.verify_done(FileWorkerStatus::InvalidInput, 0);
            return;
        }
        if !Self::path_fits_with_hash_extension(path) {
            self.log_invalid_input("verifyIn", "path too long");
            self.verify_done(FileWorkerStatus::InvalidInput, 0);
            return;
        }

        let file_name = path.as_str().unwrap_or_default();
        let mut file_size: FwSizeType = 0;
        let mut worker_stat = FileWorkerStatus::Done;

        let mut crc_from_file = 0u32;
        let mut crc_calculated = 0u32;
        let crc_stat =
            crc_checker::verify_checksum(file_name, &mut crc_from_file, &mut crc_calculated);
        if crc_stat != CrcStat::PassedFileCrcCheck {
            self.log_crc_failed(crc_stat as u32);
            worker_stat = FileWorkerStatus::FailedCrc;
        }

        if crc != crc_calculated {
            worker_stat = FileWorkerStatus::FailedCrc;
            self.log_crc_verification_error(crc, crc_calculated);
        }

        let fs_stat = filesystem::get_file_size(file_name, &mut file_size);
        if fs_stat != filesystem::Status::OpOk {
            self.log_read_failed_file_size(fs_stat as u32);
            worker_stat = FileWorkerStatus::FailedFileSize;
        }

        self.verify_done(worker_stat, file_size);
    }

    /// `writeIn_handler`.
    fn write_in_handler(
        &self,
        _port_num: FwIndexType,
        path: &FileNameString,
        buffer: Buffer,
        offset_bytes: FwSizeType,
        append: bool,
    ) {
        if path.is_empty() {
            self.log_invalid_input("writeIn", "empty path");
            self.write_done(FileWorkerStatus::InvalidInput, 0, buffer);
            return;
        }
        if !Self::path_fits_with_hash_extension(path) {
            self.log_invalid_input("writeIn", "path too long");
            self.write_done(FileWorkerStatus::InvalidInput, 0, buffer);
            return;
        }
        if !buffer.is_valid() {
            self.log_invalid_input("writeIn", "invalid buffer");
            self.write_done(FileWorkerStatus::InvalidInput, 0, buffer);
            return;
        }
        if offset_bytes > buffer.size() as FwSizeType {
            self.log_invalid_input("writeIn", "invalid offset");
            self.write_done(FileWorkerStatus::InvalidInput, 0, buffer);
            return;
        }

        let state = self.state.load(Ordering::Relaxed);
        if state != FileWorkerState::Idle as u32 {
            self.log_not_in_idle(state);
            self.write_done(FileWorkerStatus::NotIdle, 0, buffer);
            return;
        }

        self.set_state(FileWorkerState::Writing);
        // A new request overrides any leftover abort.
        self.abort.store(false, Ordering::Relaxed);

        // C++ parity: upstream re-checks the length against
        // FileNameStringSize when copying the name into a local array. The
        // hash-extension check above already bounds it at 234, so this
        // branch cannot be taken; it is kept for fidelity.
        if path.len() >= FileNameString::CAPACITY {
            self.log_invalid_input("writeIn", "path too long");
            self.write_done(FileWorkerStatus::InvalidInput, 0, buffer);
            self.set_state(FileWorkerState::Idle);
            return;
        }
        let file_name = path.as_str().unwrap_or_default();

        let is_write = self.write_buffer_to_file(&buffer, file_name, offset_bytes, append);
        if is_write {
            self.write_buffer_hash_to_file(&buffer, file_name, offset_bytes, append);
        }

        // Report the actual outcome: bytes actually written (excluding the
        // skipped offset), 0 on failure.
        let (status, written) = if is_write {
            (
                FileWorkerStatus::DoneWrite,
                buffer.size() as FwSizeType - offset_bytes,
            )
        } else {
            (FileWorkerStatus::FailedToWrite, 0)
        };
        self.write_done(status, written, buffer);
        self.set_state(FileWorkerState::Idle);
    }

    // -- Helpers ----------------------------------------------------------------

    /// C++ `pathFitsWithHashExtension`: `path` plus `.CRC32` fits in a
    /// `FileNameString`.
    fn path_fits_with_hash_extension(path: &FileNameString) -> bool {
        path.len() as FwSizeType + Hash::get_file_extension_length()
            <= FileNameString::CAPACITY as FwSizeType
    }

    /// C++ `readBufferFromFile`.
    fn read_buffer_from_file(&self, buffer: &mut Buffer, file_name: &str) -> FileWorkerStatus {
        // C++ parity: FW_ASSERT(buffer.getData() != nullptr).
        fw_assert!(buffer.capacity() > 0);

        let mut file = File::new();
        let file_stat = file.open(file_name, Mode::OpenRead);
        if file_stat != file::Status::OpOk {
            self.log_open_file_error(file_name, file_stat as u32);
            return FileWorkerStatus::FailedToOpen;
        }

        let read_size = buffer.size() as FwSizeType;
        self.log_read_begin(read_size, file_name);
        let read_stat = self.read_file(buffer, read_size, &mut file, file_name);
        // C++ parity: emitted whatever the outcome.
        self.log_read_completed(read_size, file_name);
        file.close();

        if read_stat == FileWorkerReadStatus::Done {
            FileWorkerStatus::DoneRead
        } else {
            FileWorkerStatus::FailedToRead
        }
    }

    /// C++ `readFile`: run the read loop and report its outcome.
    fn read_file(
        &self,
        buffer: &mut Buffer,
        size: FwSizeType,
        file: &mut File,
        file_name: &str,
    ) -> FileWorkerReadStatus {
        fw_assert!(buffer.capacity() > 0);
        fw_assert!(size > 0);

        if !file.is_open() {
            return FileWorkerReadStatus::Error;
        }

        let mut bytes_read: FwSizeType = 0;
        let read_stat = self.read_file_bytes(buffer, size, file, &mut bytes_read);
        match read_stat {
            FileWorkerReadStatus::Done => {}
            FileWorkerReadStatus::Error | FileWorkerReadStatus::Unknown => {
                // Unknown: the loop ran out of iterations.
                self.log_read_error(bytes_read, size, file_name);
            }
            FileWorkerReadStatus::Abort => {
                self.log_read_aborted(bytes_read, size, file_name);
            }
            FileWorkerReadStatus::Timeout => {
                let (_, timeout) = self.chunks_and_timeout(size);
                self.log_read_timeout(bytes_read, size, file_name, timeout);
            }
        }
        read_stat
    }

    /// C++ `readFileBytes`: read `size` bytes into the buffer in chunks,
    /// checking cancellation and the timeout after each chunk.
    ///
    /// Deviation: upstream reads through the raw pointer and never checks
    /// `size` against the buffer (every caller passes the buffer's size);
    /// a larger `size` is an `fw_assert` here instead of an overrun.
    fn read_file_bytes(
        &self,
        buffer: &mut Buffer,
        size: FwSizeType,
        file: &mut File,
        bytes_read: &mut FwSizeType,
    ) -> FileWorkerReadStatus {
        fw_assert!(buffer.capacity() > 0);
        fw_assert!(size > 0);
        fw_assert!(size <= buffer.size() as FwSizeType, size as i32);

        let chunk_size = self.chunk_size();
        let (num_chunks, timeout) = self.chunks_and_timeout(size);

        *bytes_read = 0;
        let start = self.evt.time_get();
        let data = buffer.data_mut();

        for _ in 0..num_chunks {
            let read_amt = (size - *bytes_read).min(chunk_size);
            let begin = *bytes_read as usize;
            let mut read_amt_actual = read_amt;
            // C++ parity: the two-argument Os::File::read is WAIT.
            let ret = file.read(
                &mut data[begin..begin + read_amt as usize],
                &mut read_amt_actual,
                WaitType::Wait,
            );
            if ret != file::Status::OpOk || read_amt != read_amt_actual {
                // Count what was actually transferred; a short read stays
                // an error (FileWorker reads a fixed, caller-chosen size).
                *bytes_read += read_amt_actual;
                return FileWorkerReadStatus::Error;
            }

            if self.abort.load(Ordering::Relaxed) {
                return FileWorkerReadStatus::Abort;
            }

            if timeout > 0 && self.elapsed_us(&start) >= timeout {
                return FileWorkerReadStatus::Timeout;
            }

            *bytes_read += read_amt;
            if *bytes_read >= size {
                return FileWorkerReadStatus::Done;
            }
        }

        FileWorkerReadStatus::Unknown
    }

    /// Microseconds since `start` on the component's time source, computed
    /// as upstream does: `Fw::Time::sub` (asserting on a base change or
    /// time running backwards), then `seconds * 1000000 + useconds` in
    /// 32-bit arithmetic (it wraps after ~71 minutes) widened to `U64`.
    fn elapsed_us(&self, start: &Time) -> u64 {
        let now = self.evt.time_get();
        let diff = Time::sub(&now, start);
        u64::from(
            diff.get_seconds()
                .wrapping_mul(1_000_000)
                .wrapping_add(diff.get_useconds()),
        )
    }

    /// C++ `getHash`: extend the CRC stored in an existing sidecar with
    /// `data`, or hash `data` alone when there is no sidecar.
    fn get_hash(&self, hash_file_name: &str, hash_buffer: &mut HashBuffer, data: &[u8]) -> bool {
        fw_assert!(!data.is_empty());

        let mut file = File::new();
        let stat = file.open(hash_file_name, Mode::OpenRead);
        if stat == file::Status::OpOk {
            let mut hash_value = [0u8; HASH_DIGEST_LENGTH];
            let mut hash_size = hash_value.len() as FwSizeType;
            // C++ parity: the two-argument Os::File::read is WAIT.
            let read_stat = file.read(&mut hash_value, &mut hash_size, WaitType::Wait);
            if read_stat != file::Status::OpOk {
                self.log_write_validation_read_error(hash_file_name, read_stat as i32);
                return false;
            }
            // C++ parity: Utils::Hash::setHashValue(HashBuffer&) deserializes
            // a big-endian U32 and FW_ASSERTs that it can, so a sidecar
            // shorter than 4 bytes is an assertion, not an event.
            let mut stored = HashBuffer::from_bytes(&hash_value[..hash_size as usize]);
            let mut hash = Hash::new();
            hash.set_hash_value_buffer(&mut stored);
            hash.update(data);
            *hash_buffer = hash.finalize_buffer();
        } else if stat == file::Status::DoesntExist {
            *hash_buffer = Hash::hash(data);
        } else {
            self.log_write_validation_open_error(hash_file_name, stat as i32);
            return false;
        }
        true
    }

    /// C++ `writeBufferToFile`: write `buffer[offset..]` (no-op success
    /// when that is empty).
    fn write_buffer_to_file(
        &self,
        buffer: &Buffer,
        file_name: &str,
        offset: FwSizeType,
        append: bool,
    ) -> bool {
        // C++ parity: FW_ASSERT(buffer.getData() != nullptr).
        fw_assert!(buffer.capacity() > 0);
        let data = buffer.data();
        fw_assert!(offset <= data.len() as FwSizeType);
        let data_from_offset = &data[offset as usize..];
        let size = data_from_offset.len() as FwSizeType;

        // A zero-length write is a successful no-op: do not open (and so
        // create) the file.
        if size == 0 {
            self.log_write_completed(size, file_name);
            return true;
        }

        let mut file = File::new();
        let stat = if append {
            file.open(file_name, Mode::OpenAppend)
        } else {
            file.open(file_name, Mode::OpenWrite)
        };
        if stat != file::Status::OpOk {
            self.log_open_file_error(file_name, stat as u32);
            return false;
        }

        self.log_write_begin(size, file_name);
        let written_size = self.write_to_file(data_from_offset, &mut file, file_name);
        if written_size != size {
            return false;
        }

        self.log_write_completed(size, file_name);
        true
    }

    /// C++ `writeBufferHashToFile`: write `<file>.CRC32` for the written
    /// bytes (extending the stored CRC when appending).
    fn write_buffer_hash_to_file(
        &self,
        buffer: &Buffer,
        file_name: &str,
        offset: FwSizeType,
        append: bool,
    ) {
        fw_assert!(buffer.capacity() > 0);

        // Same capacity as the CRC helpers, so read, verify and write agree
        // on the longest legal path.
        let mut hash_file_name = FileNameString::new();
        let status = Hash::add_file_extension(file_name.as_bytes(), &mut hash_file_name);
        if status != StringFormatStatus::Success {
            // writeIn rejects such paths up front; report rather than
            // assert if one gets here.
            self.log_invalid_input("writeIn", "path too long");
            return;
        }
        let hash_file_name = hash_file_name.as_str().unwrap_or_default();

        let data = buffer.data();
        fw_assert!(offset <= data.len() as FwSizeType);
        let data_from_offset = &data[offset as usize..];

        // A zero-length write changed nothing, so the hash must not change
        // either.
        if data_from_offset.is_empty() {
            return;
        }

        let mut hash_buffer = HashBuffer::new();
        if !append {
            hash_buffer = Hash::hash(data_from_offset);
        } else if !self.get_hash(hash_file_name, &mut hash_buffer, data_from_offset) {
            return;
        }

        let mut file = File::new();
        let stat = file.open(hash_file_name, Mode::OpenWrite);
        if stat != file::Status::OpOk {
            self.log_open_file_error(hash_file_name, stat as u32);
            return;
        }

        let digest = hash_buffer.as_slice();
        let written_size = self.write_to_file(digest, &mut file, hash_file_name);
        let hash_size = digest.len() as FwSizeType;
        if written_size != hash_size {
            self.log_write_validation_error(hash_file_name, written_size, hash_size);
        }
    }

    /// C++ `writeToFile`: write `data` in chunks, checking cancellation and
    /// the timeout after each chunk; returns the bytes counted as written.
    fn write_to_file(&self, data: &[u8], file: &mut File, file_name: &str) -> FwSizeType {
        fw_assert!(!data.is_empty());
        fw_assert!(file.is_open());

        let size = data.len() as FwSizeType;
        let chunk_size = self.chunk_size();
        let (num_chunks, timeout) = self.chunks_and_timeout(size);

        // Legal short writes make progress but consume an iteration, so
        // allow extra iterations beyond the chunk count.
        let max_iterations = num_chunks + FwSizeType::from(Self::MAX_LOOP_ITERATIONS);
        let mut bytes_written: FwSizeType = 0;
        let start = self.evt.time_get();
        let mut i: FwSizeType = 0;
        while i < max_iterations && bytes_written < size {
            let begin = bytes_written as usize;
            let mut write_amt = (size - bytes_written).min(chunk_size);
            let end = begin + write_amt as usize;
            // C++ parity: the two-argument Os::File::write is WAIT.
            let ret = file.write(&data[begin..end], &mut write_amt, WaitType::Wait);

            if ret != file::Status::OpOk || write_amt == 0 {
                self.log_write_file_error(bytes_written, size, file_name, ret as i32);
                break;
            }

            if self.abort.load(Ordering::Relaxed) {
                self.log_write_aborted(bytes_written, size, file_name);
                break;
            }

            if timeout > 0 && self.elapsed_us(&start) >= timeout {
                self.log_write_timeout(bytes_written, size, file_name, timeout);
                break;
            }

            bytes_written += write_amt;
            i += 1;
        }

        bytes_written
    }

    // -- Events -----------------------------------------------------------------

    /// The generated throttle check for `throttle 2 every {seconds=10}`:
    /// once the limit is reached, the count resets only when 10 s have
    /// passed since the first emission of the window.
    fn throttle_ok(&self, which: Throttled, now: &Time) -> bool {
        let mut throttles = self.throttles.lock();
        let throttle = &mut throttles[which as usize];
        if throttle.count >= THROTTLE_LIMIT {
            if TimeInterval::between(&throttle.time, now) >= TimeInterval::new(THROTTLE_SECONDS, 0)
            {
                throttle.count = 0;
            } else {
                return false;
            }
        }
        if throttle.count == 0 {
            throttle.time = *now;
        }
        throttle.count += 1;
        true
    }

    /// Emit a throttled event: one time read stamps the event and drives
    /// the throttle, as in the generated code.
    fn log_throttled<F>(
        &self,
        which: Throttled,
        local_id: FwEventIdType,
        severity: LogSeverity,
        text: &str,
        write_args: F,
    ) where
        F: FnOnce(&mut LogBuffer) -> SerializeStatus,
    {
        let time = self.evt.time_get();
        if !self.throttle_ok(which, &time) {
            return;
        }
        let id = self.id_base() + local_id;
        if let Some(p) = self.evt.log_out.try_get() {
            let mut args = LogBuffer::new();
            let status = write_args(&mut args);
            fw_assert!(status.is_ok(), status as i32);
            let mut time_tag = time;
            p.target
                .invoke(p.port_num, id, &mut time_tag, severity, &mut args);
        }
        if let Some(p) = self.evt.text_log_out.try_get() {
            let mut text_buf = TextLogString::from(text);
            let mut time_tag = time;
            p.target
                .invoke(p.port_num, id, &mut time_tag, severity, &mut text_buf);
        }
    }

    fn log(
        &self,
        local_id: FwEventIdType,
        severity: LogSeverity,
        text: &str,
        write_args: impl FnOnce(&mut LogBuffer) -> SerializeStatus,
    ) {
        self.evt
            .log_event(self.id_base(), local_id, severity, text, write_args);
    }

    fn log_not_in_idle(&self, curr_state: u32) {
        self.log_throttled(
            Throttled::NotInIdle,
            Self::EVENTID_NOTINIDLE,
            LogSeverity::WarningHi,
            &format!("Not in IDLE state, currently in state: {curr_state}"),
            |buf| buf.serialize_u32_be(curr_state),
        );
    }

    fn log_crc_failed(&self, crc_stat: u32) {
        self.log_throttled(
            Throttled::CrcFailed,
            Self::EVENTID_CRCFAILED,
            LogSeverity::WarningHi,
            &format!("Failed CRC check with {crc_stat} status"),
            |buf| buf.serialize_u32_be(crc_stat),
        );
    }

    fn log_crc_verification_error(&self, crc_exp: u32, crc_calculated: u32) {
        self.log(
            Self::EVENTID_CRCVERIFICATIONERROR,
            LogSeverity::WarningLo,
            &format!("Failed to verify file. Expected CRC {crc_exp}. Actual CRC {crc_calculated}"),
            |buf| {
                fw_try!(buf.serialize_u32_be(crc_exp));
                buf.serialize_u32_be(crc_calculated)
            },
        );
    }

    fn log_read_failed_file_size(&self, fs_stat: u32) {
        self.log_throttled(
            Throttled::ReadFailedFileSize,
            Self::EVENTID_READFAILEDFILESIZE,
            LogSeverity::WarningHi,
            &format!("Failed to get filesize with stat {fs_stat} in read handler"),
            |buf| buf.serialize_u32_be(fs_stat),
        );
    }

    fn log_open_file_error(&self, file_name: &str, fs_stat: u32) {
        self.log_throttled(
            Throttled::OpenFileError,
            Self::EVENTID_OPENFILEERROR,
            LogSeverity::WarningHi,
            &format!("Failed to open file {file_name} with stat {fs_stat}"),
            |buf| {
                fw_try!(file_name_arg(buf, file_name));
                buf.serialize_u32_be(fs_stat)
            },
        );
    }

    fn log_size_and_name(
        &self,
        local_id: FwEventIdType,
        severity: LogSeverity,
        text: &str,
        size: FwSizeType,
        file_name: &str,
    ) {
        self.log(local_id, severity, text, |buf| {
            fw_try!(buf.serialize_u64_be(size));
            file_name_arg(buf, file_name)
        });
    }

    fn log_read_begin(&self, file_size: FwSizeType, file_name: &str) {
        self.log_size_and_name(
            Self::EVENTID_READBEGIN,
            LogSeverity::ActivityLo,
            &format!("Reading {file_size} bytes from {file_name}"),
            file_size,
            file_name,
        );
    }

    fn log_read_completed(&self, file_size: FwSizeType, file_name: &str) {
        self.log_size_and_name(
            Self::EVENTID_READCOMPLETED,
            LogSeverity::ActivityLo,
            &format!("Finished reading {file_size} bytes from {file_name}"),
            file_size,
            file_name,
        );
    }

    fn log_write_begin(&self, write_size: FwSizeType, file_name: &str) {
        self.log_size_and_name(
            Self::EVENTID_WRITEBEGIN,
            LogSeverity::ActivityLo,
            &format!("Beginning write of size {write_size} to {file_name}"),
            write_size,
            file_name,
        );
    }

    fn log_write_completed(&self, write_size: FwSizeType, file_name: &str) {
        self.log_size_and_name(
            Self::EVENTID_WRITECOMPLETED,
            LogSeverity::ActivityLo,
            &format!("Completed write of size {write_size} to {file_name}"),
            write_size,
            file_name,
        );
    }

    fn log_two_sizes_and_name(
        &self,
        local_id: FwEventIdType,
        severity: LogSeverity,
        text: &str,
        done: FwSizeType,
        size: FwSizeType,
        file_name: &str,
    ) {
        self.log(local_id, severity, text, |buf| {
            fw_try!(buf.serialize_u64_be(done));
            fw_try!(buf.serialize_u64_be(size));
            file_name_arg(buf, file_name)
        });
    }

    fn log_read_error(&self, bytes_read: FwSizeType, read_size: FwSizeType, file_name: &str) {
        self.log_two_sizes_and_name(
            Self::EVENTID_READERROR,
            LogSeverity::WarningHi,
            &format!("Failed after {bytes_read} of {read_size} bytes read to {file_name}"),
            bytes_read,
            read_size,
            file_name,
        );
    }

    fn log_read_aborted(&self, bytes_read: FwSizeType, read_size: FwSizeType, file_name: &str) {
        self.log_two_sizes_and_name(
            Self::EVENTID_READABORTED,
            LogSeverity::WarningLo,
            &format!("Aborted after {bytes_read} of {read_size} bytes read to {file_name}"),
            bytes_read,
            read_size,
            file_name,
        );
    }

    fn log_write_aborted(
        &self,
        bytes_written: FwSizeType,
        write_size: FwSizeType,
        file_name: &str,
    ) {
        self.log_two_sizes_and_name(
            Self::EVENTID_WRITEABORTED,
            LogSeverity::WarningLo,
            &format!("Aborted after {bytes_written} of {write_size} bytes written to {file_name}"),
            bytes_written,
            write_size,
            file_name,
        );
    }

    fn log_read_timeout(
        &self,
        bytes_read: FwSizeType,
        read_size: FwSizeType,
        file_name: &str,
        timeout: u64,
    ) {
        self.log(
            Self::EVENTID_READTIMEOUT,
            LogSeverity::WarningHi,
            &format!(
                "Failed after {bytes_read} of {read_size} bytes read to {file_name} after exceeding timeout of {timeout} microseconds"
            ),
            |buf| {
                fw_try!(buf.serialize_u64_be(bytes_read));
                fw_try!(buf.serialize_u64_be(read_size));
                fw_try!(file_name_arg(buf, file_name));
                buf.serialize_u64_be(timeout)
            },
        );
    }

    fn log_write_timeout(
        &self,
        bytes_written: FwSizeType,
        write_size: FwSizeType,
        file_name: &str,
        timeout: u64,
    ) {
        self.log(
            Self::EVENTID_WRITETIMEOUT,
            LogSeverity::WarningHi,
            &format!(
                "Failed after {bytes_written} of {write_size} bytes written to {file_name} after exceeding timeout of {timeout} microseconds"
            ),
            |buf| {
                fw_try!(buf.serialize_u64_be(bytes_written));
                fw_try!(buf.serialize_u64_be(write_size));
                fw_try!(file_name_arg(buf, file_name));
                buf.serialize_u64_be(timeout)
            },
        );
    }

    fn log_write_file_error(
        &self,
        bytes_written: FwSizeType,
        write_size: FwSizeType,
        file_name: &str,
        status: i32,
    ) {
        self.log_throttled(
            Throttled::WriteFileError,
            Self::EVENTID_WRITEFILEERROR,
            LogSeverity::WarningHi,
            &format!(
                "Failed after {bytes_written} of {write_size} bytes written to {file_name} with write status {status}"
            ),
            |buf| {
                fw_try!(buf.serialize_u64_be(bytes_written));
                fw_try!(buf.serialize_u64_be(write_size));
                fw_try!(file_name_arg(buf, file_name));
                buf.serialize_i32_be(status)
            },
        );
    }

    fn log_write_validation_open_error(&self, hash_file_name: &str, status: i32) {
        self.log_throttled(
            Throttled::WriteValidationOpenError,
            Self::EVENTID_WRITEVALIDATIONOPENERROR,
            LogSeverity::WarningHi,
            &format!("Failed to open validation file {hash_file_name} with status {status}"),
            |buf| {
                fw_try!(file_name_arg(buf, hash_file_name));
                buf.serialize_i32_be(status)
            },
        );
    }

    fn log_write_validation_read_error(&self, hash_file_name: &str, status: i32) {
        self.log_throttled(
            Throttled::WriteValidationReadError,
            Self::EVENTID_WRITEVALIDATIONREADERROR,
            LogSeverity::WarningHi,
            &format!("Failed to read validation file {hash_file_name} with status {status}"),
            |buf| {
                fw_try!(file_name_arg(buf, hash_file_name));
                buf.serialize_i32_be(status)
            },
        );
    }

    fn log_write_validation_error(
        &self,
        hash_file_name: &str,
        bytes_written: FwSizeType,
        hash_size: FwSizeType,
    ) {
        self.log(
            Self::EVENTID_WRITEVALIDATIONERROR,
            LogSeverity::WarningLo,
            &format!(
                "Failed to create hash file {hash_file_name}. Wrote {bytes_written} bytes when expected to write {hash_size} bytes to hash file"
            ),
            |buf| {
                fw_try!(file_name_arg(buf, hash_file_name));
                fw_try!(buf.serialize_u64_be(bytes_written));
                buf.serialize_u64_be(hash_size)
            },
        );
    }

    fn log_invalid_input(&self, handler: &str, issue: &str) {
        self.log(
            Self::EVENTID_INVALIDINPUT,
            LogSeverity::WarningHi,
            &format!("Invalid input in {handler} handler: {issue}"),
            |buf| {
                fw_try!(FwString::<16>::from(handler).serialize_to_truncated(
                    buf,
                    16,
                    Endianness::Big
                ));
                FwString::<32>::from(issue).serialize_to_truncated(buf, 32, Endianness::Big)
            },
        );
    }
}

/// Serialize a file-name event argument (`string size FileNameStringSize`)
/// as the generated code does: through `Fw::LogStringArg`, truncated to
/// `min(FW_LOG_STRING_MAX_SIZE, FileNameStringSize)`.
fn file_name_arg(buf: &mut LogBuffer, file_name: &str) -> SerializeStatus {
    FileNameString::from(file_name).serialize_to_truncated(
        buf,
        EVENT_FILE_NAME_SIZE,
        Endianness::Big,
    )
}

// -- Input adapters ------------------------------------------------------------

/// `readIn` adapter: the buffer is deposited in the escrow and the token
/// rides in the queue message where C++ serializes the buffer.
struct ReadInAdapter {
    comp: Arc<FileWorker>,
}

impl FileReadPort for ReadInAdapter {
    fn invoke(&self, port_num: FwIndexType, path: &FileNameString, buffer: Buffer) {
        let token = self.comp.escrow.deposit(buffer);
        let mut msg = LinearBuffer::<QUEUE_MSG_SIZE>::new();
        let status = msg::write_envelope_header(&mut msg, FileWorker::MSG_TYPE_READ_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = msg.serialize(path, Endianness::Big);
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

/// `writeIn` adapter (escrowed buffer, as `readIn`).
struct WriteInAdapter {
    comp: Arc<FileWorker>,
}

impl FileWritePort for WriteInAdapter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        path: &FileNameString,
        buffer: Buffer,
        offset_bytes: FwSizeType,
        append: bool,
    ) {
        let token = self.comp.escrow.deposit(buffer);
        let mut msg = LinearBuffer::<QUEUE_MSG_SIZE>::new();
        let status = msg::write_envelope_header(&mut msg, FileWorker::MSG_TYPE_WRITE_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = msg.serialize(path, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = msg.serialize_u64_be(token);
        fw_assert!(status.is_ok(), status as i32);
        let status = msg.serialize_u64_be(offset_bytes);
        fw_assert!(status.is_ok(), status as i32);
        let status = msg.serialize_bool(append, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&msg, QUEUE_PRIORITY, QueueFullPolicy::Assert);
    }
}

impl FileWorker {
    /// `readIn` — ASYNC `Svc.FileRead` input.
    pub fn read_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn FileReadPort> {
        PortRef::new(
            Arc::new(ReadInAdapter {
                comp: Arc::clone(self),
            }),
            port_num,
        )
    }

    /// `writeIn` — ASYNC `Svc.FileWrite` input.
    pub fn write_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn FileWritePort> {
        PortRef::new(
            Arc::new(WriteInAdapter {
                comp: Arc::clone(self),
            }),
            port_num,
        )
    }
}

async_input_port_adapter! {
    /// `verifyIn` — ASYNC `Svc.VerifyStatus` input.
    component: FileWorker;
    adapter: VerifyInAdapter;
    port: VerifyStatusPort;
    input: pub verify_in;
    deserialize: verify_in_deserialize;
    handler: verify_in_handler;
    base: active.queued;
    msg_type: FileWorker::MSG_TYPE_VERIFY_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { ref path: FileNameString, val crc: u32 }
}

input_port_adapter! {
    /// `cancelIn` — GUARDED `Svc.CancelStatus` input (caller's thread).
    component: FileWorker;
    adapter: CancelInAdapter;
    port: CancelStatusPort;
    input: pub cancel_in;
    handler: cancel_in_handler;
    args {}
}

input_port_adapter! {
    /// `cmdIn` — SYNC `Fw.Cmd` input (no commands: see the module docs).
    component: FileWorker;
    adapter: CmdInAdapter;
    port: CmdPort;
    input: pub cmd_in;
    handler: cmd_in_handler;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, mut args: CmdArgBuffer }
}

// -- Dispatch (the generated `doDispatch` switch) -------------------------------

impl ComponentDispatch for FileWorker {
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
            Self::MSG_TYPE_READ_IN => {
                let mut path = FileNameString::new();
                let mut token = 0u64;
                if !buf.deserialize(&mut path, Endianness::Big).is_ok()
                    || !buf.deserialize_u64_be(&mut token).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                let buffer = self.escrow.claim(token);
                self.read_in_handler(port_num, &path, buffer);
                MsgDispatchStatus::Ok
            }
            Self::MSG_TYPE_VERIFY_IN => match Self::verify_in_deserialize(buf) {
                Some((path, crc)) => {
                    self.verify_in_handler(port_num, &path, crc);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_WRITE_IN => {
                let mut path = FileNameString::new();
                let mut token = 0u64;
                let mut offset_bytes: FwSizeType = 0;
                let mut append = false;
                if !buf.deserialize(&mut path, Endianness::Big).is_ok()
                    || !buf.deserialize_u64_be(&mut token).is_ok()
                    || !buf.deserialize_u64_be(&mut offset_bytes).is_ok()
                    || !buf.deserialize_bool(&mut append, Endianness::Big).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                let buffer = self.escrow.claim(token);
                self.write_in_handler(port_num, &path, buffer, offset_bytes, append);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for FileWorker {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests;
