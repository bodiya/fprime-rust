//! `Svc::DpCatalog` — the data-product catalog and downlink driver (ACTIVE).
//!
//! Port of `Svc/DpCatalog/DpCatalog.{fpp,hpp,cpp}` and
//! `default/config/DpCatalogCfg.hpp`; analysis:
//! `docs/cpp-analysis/data-products.md` (the four `Svc::DpCatalog`
//! sections).
//!
//! `BUILD_CATALOG` scans the configured directories for `.fdp` files,
//! validates each header (hash, deserialization, declared data size and the
//! canonical file name), merges the transmit state recorded in the state
//! file and inserts the survivors into a priority-ordered catalog.
//! `START_XMIT_CATALOG` walks that catalog highest-priority-first, handing
//! one file at a time to `Svc::FileDownlink` over `Svc.SendFileRequest` and
//! advancing on each `Svc.SendFileComplete`.
//!
//! # Deviations from C++
//!
//! * The C++ catalog is a `Fw::RedBlackTreeSet<DpStateEntry, 127>`. Here it
//!   is a sorted, fixed-capacity `Vec` with binary-search insert/find/remove
//!   — the externally visible behavior (smallest entry first, duplicate
//!   detection, insert failure when full) and the ORDER are identical.
//!   [`compare_entries`] is the C++ `DpStateEntry::compareEntries`, and like
//!   it deliberately EXCLUDES `dir`.
//! * The C++ `MemAllocator` sizes only the state-file tracking array; here
//!   that array is a `Box<[DpStateFileEntry]>` sized at
//!   [`DpCatalog::configure`], and [`DpCatalog::configure_with_slots`]
//!   represents the short-allocation path (a smaller `m_numDpSlots`).
//! * `Svc.SendFileRequest` / `Svc.SendFileComplete` / `Svc.SendFileStatus`
//!   come from [`crate::file_downlink`], and `Svc.DpWritten` from
//!   [`crate::dp_writer`], where their emitters live.

use crate::dp_writer::{DP_EXT, DpWrittenPort, format_dp_file_name};
use crate::file_downlink::{
    SendFileCompletePort, SendFileNameArg, SendFileRequestPort, SendFileResponse, SendFileStatus,
};
use crate::file_manager::StringFormatStatus;
use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, ComponentDispatch, EventGlue, EventThrottle,
    MsgDispatchStatus, OutputPort, PingPort, PortRef, QueueFullPolicy, TlmGlue, msg,
};
use fprime_config::{
    FILE_NAME_STRING_SIZE, FW_CMD_ARG_BUFFER_MAX_SIZE, FW_LOG_STRING_MAX_SIZE, FwChanIdType,
    FwDpIdType, FwDpPriorityType, FwEnumStoreType, FwEventIdType, FwIdType, FwIndexType,
    FwOpcodeType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::dp::{DpContainer, DpState};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, Endianness, ExtBuf, FileNameString, FwDefaultString,
    LinearBuffer, LogSeverity, SerBuf, SerBufAny, Success, Wait, fpp_enum, fpp_struct, fw_assert,
    fw_try,
};
use fprime_os::directory::{OpenMode, Status as DirStatus};
use fprime_os::file::{Mode, OverwriteType, Status as FileStatus, WaitType};
use fprime_os::{Directory, File, filesystem};
use std::cmp::Ordering as CmpOrdering;
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Configuration (default/config/DpCatalogCfg.hpp)
// ---------------------------------------------------------------------------

/// `DP_MAX_DIRECTORIES` — the number of managed data-product directories.
pub const DP_MAX_DIRECTORIES: usize = 2;

/// `DP_MAX_FILES` — the catalog capacity.
pub const DP_MAX_FILES: usize = 127;

/// `DIRECTORY_DELIMITER`.
const DIRECTORY_DELIMITER: char = '/';

// ---------------------------------------------------------------------------
// FPP types
// ---------------------------------------------------------------------------

fpp_enum! {
    /// `Svc.DpHdrField` — the header field a validation error refers to.
    pub enum DpHdrField : u8 {
        /// The packet descriptor.
        Descriptor = 0,
        /// The container id.
        Id = 1,
        /// The priority.
        Priority = 2,
        /// The header CRC.
        Crc = 3,
    }
    default Descriptor
}

fpp_struct! {
    /// `Svc.DpRecord` — data-product metadata; 29 bytes big-endian in
    /// declaration order.
    #[derive(Clone, Copy)]
    pub struct DpRecord {
        /// The data-product id.
        id: FwDpIdType { get_id, set_id },
        /// Generation time, seconds.
        t_sec: u32 { get_t_sec, set_t_sec },
        /// Generation time, microseconds.
        t_sub: u32 { get_t_sub, set_t_sub },
        /// Downlink priority (lower value = higher priority).
        priority: u32 { get_priority, set_priority },
        /// Overall size of the data product in bytes.
        size: u64 { get_size, set_size },
        /// Number of blocks transmitted.
        blocks: u32 { get_blocks, set_blocks },
        /// Transmission state.
        state: DpState { get_state, set_state },
    }
}

/// A catalog entry: the directory index plus the product metadata
/// (C++ `DpCatalog::DpStateEntry`).
#[derive(Debug, Clone, Copy, Default)]
pub struct DpStateEntry {
    /// Index into the configured directories.
    pub dir: FwIndexType,
    /// The product metadata.
    pub record: DpRecord,
}

/// C++ `DpStateEntry::compareEntries`: priority, then generation time, then
/// id — all "lower sorts first" — and `dir` is deliberately NOT part of the
/// comparison, so the same product in two managed directories compares
/// EQUAL (and the second copy is treated as a duplicate).
pub fn compare_entries(left: &DpStateEntry, right: &DpStateEntry) -> CmpOrdering {
    left.record
        .priority
        .cmp(&right.record.priority)
        .then(left.record.t_sec.cmp(&right.record.t_sec))
        .then(left.record.t_sub.cmp(&right.record.t_sub))
        .then(left.record.id.cmp(&right.record.id))
}

impl PartialEq for DpStateEntry {
    fn eq(&self, other: &Self) -> bool {
        compare_entries(self, other) == CmpOrdering::Equal
    }
}

impl Eq for DpStateEntry {}

impl PartialOrd for DpStateEntry {
    fn partial_cmp(&self, other: &Self) -> Option<CmpOrdering> {
        Some(self.cmp(other))
    }
}

impl Ord for DpStateEntry {
    fn cmp(&self, other: &Self) -> CmpOrdering {
        compare_entries(self, other)
    }
}

/// One line of the state file plus its bookkeeping flags
/// (C++ `DpDstateFileEntry`).
#[derive(Debug, Clone, Copy, Default)]
struct DpStateFileEntry {
    used: bool,
    visited: bool,
    entry: DpStateEntry,
}

/// The sorted, fixed-capacity catalog standing in for
/// `Fw::RedBlackTreeSet<DpStateEntry, DP_MAX_FILES>`.
#[derive(Debug, Default)]
struct Catalog {
    entries: Vec<DpStateEntry>,
}

impl Catalog {
    /// Allocate the whole capacity up front (no steady-state allocation).
    fn new() -> Self {
        Self {
            entries: Vec::with_capacity(DP_MAX_FILES),
        }
    }

    fn clear(&mut self) {
        self.entries.clear();
    }

    fn len(&self) -> usize {
        self.entries.len()
    }

    /// C++ `find(entry) == Fw::Success::SUCCESS`.
    fn find(&self, entry: &DpStateEntry) -> bool {
        self.entries
            .binary_search_by(|probe| compare_entries(probe, entry))
            .is_ok()
    }

    /// C++ `insert`: an existing equal entry is updated in place; a new
    /// entry fails only when the set is full.
    fn insert(&mut self, entry: DpStateEntry) -> bool {
        match self
            .entries
            .binary_search_by(|probe| compare_entries(probe, &entry))
        {
            Ok(index) => {
                self.entries[index] = entry;
                true
            }
            Err(index) => {
                if self.entries.len() >= DP_MAX_FILES {
                    return false;
                }
                self.entries.insert(index, entry);
                true
            }
        }
    }

    /// C++ `remove`.
    fn remove(&mut self, entry: &DpStateEntry) -> bool {
        match self
            .entries
            .binary_search_by(|probe| compare_entries(probe, entry))
        {
            Ok(index) => {
                self.entries.remove(index);
                true
            }
            Err(_) => false,
        }
    }

    /// C++ `*begin()`: the highest-priority (smallest) entry.
    fn first(&self) -> Option<DpStateEntry> {
        self.entries.first().copied()
    }
}

/// C++ `DpCatalog::ProcessFileStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProcessFileStatus {
    /// The file was added to the catalog.
    Success,
    /// The file could not be processed; continue with the next one.
    Failed,
    /// The catalog is full; stop processing files.
    Quit,
}

// ---------------------------------------------------------------------------
// Dictionary constants
// ---------------------------------------------------------------------------

/// Queue message types, numbered from 1 in FPP declaration order
/// (`pingIn`, `fileDone`, `addToCat`, command input).
const MSG_TYPE_PING_IN: FwEnumStoreType = 1;
const MSG_TYPE_FILE_DONE: FwEnumStoreType = 2;
const MSG_TYPE_ADD_TO_CAT: FwEnumStoreType = 3;
const MSG_TYPE_CMD_IN: FwEnumStoreType = 4;

/// Queue message size = the largest async invocation, the command envelope:
/// 6 + 4 + 4 + 2 + 506 = 522 (`addToCat` needs 6 + 242 + 4 + 8 = 260).
pub const QUEUE_MSG_SIZE: FwSizeType =
    (msg::ENVELOPE_HEADER_SIZE + 4 + 4 + 2 + FW_CMD_ARG_BUFFER_MAX_SIZE) as FwSizeType;

/// FPP declares no `priority` qualifier on any async input.
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

/// One state-file record: `sizeof(FwIndexType)` + `DpRecord::SERIALIZED_SIZE`
/// = 31 bytes with the default config, derived from the config alias so an
/// `FwIndexType` override still round-trips.
pub const STATE_FILE_RECORD_SIZE: usize = size_of::<FwIndexType>() + DpRecord::SERIALIZED_SIZE;

/// The number of declared event ids (0..=49), used to size the throttle
/// table.
const NUM_EVENT_IDS: usize = 50;

/// The event ids declared `throttle 10` in the FPP model.
const THROTTLED_EVENTS: [FwEventIdType; 16] = [
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 41, 42, 47, 49,
];

/// The `throttle 10` limit.
const THROTTLE_10: u32 = 10;

type MsgBuffer = LinearBuffer<{ QUEUE_MSG_SIZE as usize }>;

// ---------------------------------------------------------------------------
// Component state
// ---------------------------------------------------------------------------

/// All DpCatalog state. Every input port and command is `async`, so this
/// lives entirely on the component thread; the mutex is the Rust shape of
/// "single-threaded component state", not an extra C++ lock.
struct CatalogState {
    /// `m_initialized`.
    initialized: bool,
    /// `m_dpCatalog`.
    catalog: Catalog,
    /// `m_currentXmitEntry` / `m_hasCurrentXmit`.
    current_xmit_entry: DpStateEntry,
    has_current_xmit: bool,
    /// `m_numDpSlots`.
    num_dp_slots: usize,
    /// `m_directories` / `m_numDirectories`.
    directories: [FileNameString; DP_MAX_DIRECTORIES],
    num_directories: usize,
    /// `m_stateFile`, `m_stateFileData`, `m_stateFileEntries`.
    state_file: FileNameString,
    state_file_data: Box<[DpStateFileEntry]>,
    state_file_entries: usize,
    /// `m_catalogBuilt`.
    catalog_built: bool,
    /// `m_xmitInProgress` and friends.
    xmit_in_progress: bool,
    curr_xmit_file_name: FileNameString,
    xmit_cmd_wait: bool,
    xmit_bytes: u64,
    xmit_op_code: FwOpcodeType,
    xmit_cmd_seq: u32,
    /// `m_pendingFiles` / `m_pendingDpBytes`.
    pending_files: u32,
    pending_dp_bytes: u64,
    /// `m_remainActive`.
    remain_active: bool,
}

impl CatalogState {
    fn new() -> Self {
        Self {
            initialized: false,
            catalog: Catalog::new(),
            current_xmit_entry: DpStateEntry::default(),
            has_current_xmit: false,
            num_dp_slots: 0,
            directories: std::array::from_fn(|_| FileNameString::new()),
            num_directories: 0,
            state_file: FileNameString::new(),
            state_file_data: Box::new([]),
            state_file_entries: 0,
            catalog_built: false,
            xmit_in_progress: false,
            curr_xmit_file_name: FileNameString::new(),
            xmit_cmd_wait: false,
            xmit_bytes: 0,
            xmit_op_code: 0,
            xmit_cmd_seq: 0,
            pending_files: 0,
            pending_dp_bytes: 0,
            remain_active: false,
        }
    }
}

/// `Svc::DpCatalog` — active data-product catalog.
pub struct DpCatalog {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Command registration/response glue (`CmdReg`/`CmdStatus`).
    pub cmd: CmdGlue,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// Telemetry port (`Tlm`).
    pub tlm: TlmGlue,
    /// `pingOut`: `Svc.Ping` out.
    pub ping_out: OutputPort<dyn PingPort>,
    /// `fileOut`: `Svc.SendFileRequest` out (a `Svc::FileDownlink`).
    pub file_out: OutputPort<dyn SendFileRequestPort>,
    /// Per-event-id throttles; only the ids in [`THROTTLED_EVENTS`] consult
    /// them. DpCatalog has NO `CLEAR_EVENT_THROTTLE` command, so once a
    /// throttle fills it stays filled (C++ parity).
    throttles: [EventThrottle; NUM_EVENT_IDS],
    state: Mutex<CatalogState>,
}

impl DpCatalog {
    // -- Commands (FPP explicit opcodes) -----------------------------------

    /// `BUILD_CATALOG` — opcode 0, no arguments. Blocks the component
    /// thread for the whole directory scan (C++ documents this).
    pub const OPCODE_BUILD_CATALOG: FwOpcodeType = 0;
    /// `START_XMIT_CATALOG(wait: Fw.Wait, remainActive: bool)` — opcode 1.
    pub const OPCODE_START_XMIT_CATALOG: FwOpcodeType = 1;
    /// `STOP_XMIT_CATALOG` — opcode 2.
    pub const OPCODE_STOP_XMIT_CATALOG: FwOpcodeType = 2;
    /// `CLEAR_CATALOG` — opcode 3.
    pub const OPCODE_CLEAR_CATALOG: FwOpcodeType = 3;

    // -- Events (FPP EXPLICIT ids; there is no 6..9 and no 33) -------------

    /// `DirectoryOpenError(loc, stat)` — WARNING_HI, id 0.
    pub const EVENTID_DIRECTORY_OPEN_ERROR: FwEventIdType = 0;
    /// `ProcessingDirectory(directory)` — ACTIVITY_LO, id 1.
    pub const EVENTID_PROCESSING_DIRECTORY: FwEventIdType = 1;
    /// `ProcessingFile(file)` — ACTIVITY_LO, id 2.
    pub const EVENTID_PROCESSING_FILE: FwEventIdType = 2;
    /// `ProcessingDirectoryComplete(loc, total, pending, pending_bytes)` —
    /// ACTIVITY_HI, id 3.
    pub const EVENTID_PROCESSING_DIRECTORY_COMPLETE: FwEventIdType = 3;
    /// `CatalogBuildComplete()` — ACTIVITY_HI, id 4.
    pub const EVENTID_CATALOG_BUILD_COMPLETE: FwEventIdType = 4;
    /// `DirectoryNotManaged(file)` — WARNING_HI, id 5.
    pub const EVENTID_DIRECTORY_NOT_MANAGED: FwEventIdType = 5;
    /// `CatalogXmitStarted()` — ACTIVITY_HI, id 10. Declared by the FPP
    /// model but NEVER emitted by the implementation (kept for dictionary
    /// compatibility).
    pub const EVENTID_CATALOG_XMIT_STARTED: FwEventIdType = 10;
    /// `CatalogXmitStopped(bytes)` — ACTIVITY_HI, id 11.
    pub const EVENTID_CATALOG_XMIT_STOPPED: FwEventIdType = 11;
    /// `CatalogXmitCompleted(bytes)` — ACTIVITY_HI, id 12.
    pub const EVENTID_CATALOG_XMIT_COMPLETED: FwEventIdType = 12;
    /// `SendingProduct(file, bytes, prio)` — ACTIVITY_LO, id 13.
    pub const EVENTID_SENDING_PRODUCT: FwEventIdType = 13;
    /// `ProductComplete(file, pending, pending_bytes)` — ACTIVITY_LO, id 14.
    pub const EVENTID_PRODUCT_COMPLETE: FwEventIdType = 14;
    /// `ComponentNotInitialized()` — WARNING_HI, id 20, `throttle 10`.
    pub const EVENTID_COMPONENT_NOT_INITIALIZED: FwEventIdType = 20;
    /// `ComponentNoMemory()` — WARNING_HI, id 21, `throttle 10`.
    pub const EVENTID_COMPONENT_NO_MEMORY: FwEventIdType = 21;
    /// `CatalogFull(dir)` — WARNING_HI, id 22, `throttle 10`.
    pub const EVENTID_CATALOG_FULL: FwEventIdType = 22;
    /// `FileOpenError(loc, stat)` — WARNING_HI, id 23, `throttle 10`.
    pub const EVENTID_FILE_OPEN_ERROR: FwEventIdType = 23;
    /// `FileReadError(file, stat)` — WARNING_HI, id 24, `throttle 10`.
    pub const EVENTID_FILE_READ_ERROR: FwEventIdType = 24;
    /// `FileHdrError(file, field, exp, act)` — WARNING_HI, id 25.
    pub const EVENTID_FILE_HDR_ERROR: FwEventIdType = 25;
    /// `FileHdrDesError(file, stat)` — WARNING_HI, id 26, `throttle 10`.
    pub const EVENTID_FILE_HDR_DES_ERROR: FwEventIdType = 26;
    /// `DpInsertError(dp)` — WARNING_HI, id 27, `throttle 10`.
    pub const EVENTID_DP_INSERT_ERROR: FwEventIdType = 27;
    /// `DpDuplicate(dp)` — DIAGNOSTIC, id 28. Declared but NEVER emitted.
    pub const EVENTID_DP_DUPLICATE: FwEventIdType = 28;
    /// `DpCatalogFull(dp)` — WARNING_HI, id 29, `throttle 10`.
    pub const EVENTID_DP_CATALOG_FULL: FwEventIdType = 29;
    /// `DpXmitInProgress()` — WARNING_LO, id 30, `throttle 10`.
    pub const EVENTID_DP_XMIT_IN_PROGRESS: FwEventIdType = 30;
    /// `FileSizeError(file, stat)` — WARNING_HI, id 31, `throttle 10`.
    pub const EVENTID_FILE_SIZE_ERROR: FwEventIdType = 31;
    /// `NoDpMemory()` — WARNING_HI, id 32.
    pub const EVENTID_NO_DP_MEMORY: FwEventIdType = 32;
    /// `XmitNotActive()` — WARNING_LO, id 34.
    pub const EVENTID_XMIT_NOT_ACTIVE: FwEventIdType = 34;
    /// `StateFileOpenError(file, stat)` — WARNING_HI, id 35.
    pub const EVENTID_STATE_FILE_OPEN_ERROR: FwEventIdType = 35;
    /// `StateFileReadError(file, stat, offset)` — WARNING_HI, id 36.
    pub const EVENTID_STATE_FILE_READ_ERROR: FwEventIdType = 36;
    /// `StateFileTruncated(file, offset, size)` — WARNING_HI, id 37.
    pub const EVENTID_STATE_FILE_TRUNCATED: FwEventIdType = 37;
    /// `NoStateFileSpecified()` — WARNING_LO, id 38.
    pub const EVENTID_NO_STATE_FILE_SPECIFIED: FwEventIdType = 38;
    /// `StateFileWriteError(file, stat)` — WARNING_HI, id 39.
    pub const EVENTID_STATE_FILE_WRITE_ERROR: FwEventIdType = 39;
    /// `NoStateFile(file)` — WARNING_LO, id 40.
    pub const EVENTID_NO_STATE_FILE: FwEventIdType = 40;
    /// `DpFileXmitError(file, stat)` — WARNING_HI, id 41, `throttle 10`.
    pub const EVENTID_DP_FILE_XMIT_ERROR: FwEventIdType = 41;
    /// `DpFileSendError(file, stat)` — WARNING_HI, id 42, `throttle 10`.
    pub const EVENTID_DP_FILE_SEND_ERROR: FwEventIdType = 42;
    /// `DpFileAdded(file)` — ACTIVITY_HI, id 43.
    pub const EVENTID_DP_FILE_ADDED: FwEventIdType = 43;
    /// `NotLoaded(file)` — ACTIVITY_HI, id 44.
    pub const EVENTID_NOT_LOADED: FwEventIdType = 44;
    /// `DpFileSkipped(file)` — ACTIVITY_HI, id 45.
    pub const EVENTID_DP_FILE_SKIPPED: FwEventIdType = 45;
    /// `XmitUnbuiltCatalog()` — WARNING_HI, id 46.
    pub const EVENTID_XMIT_UNBUILT_CATALOG: FwEventIdType = 46;
    /// `InvalidFileName(file, expected)` — WARNING_HI, id 47, `throttle 10`.
    pub const EVENTID_INVALID_FILE_NAME: FwEventIdType = 47;
    /// `FileCorruptedDataError(file, stat)` — WARNING_HI, id 48.
    pub const EVENTID_FILE_CORRUPTED_DATA_ERROR: FwEventIdType = 48;
    /// `FileNameFormatError(file, status)` — WARNING_HI, id 49,
    /// `throttle 10`.
    pub const EVENTID_FILE_NAME_FORMAT_ERROR: FwEventIdType = 49;

    // -- Telemetry ---------------------------------------------------------

    /// `CatalogDps: U32` — id 0. Declared by the FPP model but NEVER
    /// written by the implementation (dead channel, kept for the
    /// dictionary).
    pub const CHANID_CATALOG_DPS: FwChanIdType = 0;
    /// `DpsSent: U32` — id 1. Likewise declared and never written.
    pub const CHANID_DPS_SENT: FwChanIdType = 1;

    /// Construct (topology phase 1).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            ping_out: OutputPort::new(),
            file_out: OutputPort::new(),
            throttles: std::array::from_fn(|_| EventThrottle::new(THROTTLE_10)),
            state: Mutex::new(CatalogState::new()),
        })
    }

    /// Create the message queue (topology "configure" step).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active.queued.create_queue(queue_depth, QUEUE_MSG_SIZE);
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_BUILD_CATALOG,
                Self::OPCODE_START_XMIT_CATALOG,
                Self::OPCODE_STOP_XMIT_CATALOG,
                Self::OPCODE_CLEAR_CATALOG,
            ],
        );
    }

    /// C++ `configure(directories, stateFile, memId, allocator)` with the
    /// full [`DP_MAX_FILES`] slot count.
    ///
    /// `state_file` may be empty, meaning "do not track transmit state".
    /// `fw_assert`s that at most [`DP_MAX_DIRECTORIES`] directories are
    /// given (C++ FW_ASSERT).
    pub fn configure(&self, directories: &[FileNameString], state_file: &FileNameString) {
        self.configure_with_slots(directories, state_file, DP_MAX_FILES as FwSizeType);
    }

    /// The short-allocation form of [`Self::configure`]: `requested_slots`
    /// stands in for the memory the C++ `MemAllocator` actually returned.
    /// Zero slots reproduce the "no memory" path (`ComponentNoMemory` /
    /// `NoDpMemory`); more than [`DP_MAX_FILES`] is capped, exactly as the
    /// C++ `min(allocatedSlots, DP_MAX_FILES)` does.
    pub fn configure_with_slots(
        &self,
        directories: &[FileNameString],
        state_file: &FileNameString,
        requested_slots: FwSizeType,
    ) {
        fw_assert!(
            directories.len() <= DP_MAX_DIRECTORIES,
            directories.len() as i32
        );
        let mut state = self.state.lock().unwrap();
        state.state_file = state_file.clone();
        let slots = (requested_slots as usize).min(DP_MAX_FILES);
        if slots > 0 {
            state.num_dp_slots = slots;
            state.state_file_data = vec![DpStateFileEntry::default(); slots].into_boxed_slice();
            Self::reset_catalog(&mut state);
        } else {
            // C++: not enough memory -> zero slots, detected later.
            state.num_dp_slots = 0;
            state.state_file_data = Box::new([]);
        }
        for (index, dir) in directories.iter().enumerate() {
            state.directories[index] = dir.clone();
        }
        state.num_directories = directories.len();
        state.initialized = true;
    }

    /// C++ `shutdown()`: release the state-file storage (the C++
    /// `MemAllocator::deallocate`).
    pub fn shutdown(&self) {
        let mut state = self.state.lock().unwrap();
        state.state_file_data = Box::new([]);
        state.num_dp_slots = 0;
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    /// The number of products currently in the catalog (the C++
    /// `m_dpCatalog.getSize()`); useful for topology-level checks.
    pub fn catalog_size(&self) -> usize {
        self.state.lock().unwrap().catalog.len()
    }

    // -- Input-port factories ---------------------------------------------

    /// `pingIn` — ASYNC `Svc.Ping` input.
    pub fn ping_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn PingPort> {
        PortRef::new(Arc::new(PingInAdapter { comp: self.clone() }), port_num)
    }

    /// `fileDone` — ASYNC `Svc.SendFileComplete` input.
    pub fn file_done(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SendFileCompletePort> {
        PortRef::new(Arc::new(FileDoneAdapter { comp: self.clone() }), port_num)
    }

    /// `addToCat` — ASYNC `Svc.DpWritten` input (from `Svc::DpWriter`).
    pub fn add_to_cat(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn DpWrittenPort> {
        PortRef::new(Arc::new(AddToCatAdapter { comp: self.clone() }), port_num)
    }

    /// `CmdDisp` — ASYNC `Fw.Cmd` input.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }
}

// ---------------------------------------------------------------------------
// Catalog logic
// ---------------------------------------------------------------------------

impl DpCatalog {
    /// C++ `resetCatalog`.
    fn reset_catalog(st: &mut CatalogState) {
        st.catalog.clear();
        st.has_current_xmit = false;
        st.pending_files = 0;
        st.pending_dp_bytes = 0;
        st.catalog_built = false;
    }

    /// C++ `resetStateFileData`.
    fn reset_state_file_data(st: &mut CatalogState) {
        for slot in st.state_file_data.iter_mut() {
            *slot = DpStateFileEntry::default();
        }
        st.state_file_entries = 0;
    }

    /// C++ `checkInit`.
    fn check_init(&self, st: &CatalogState) -> bool {
        if !st.initialized {
            self.log_simple(
                Self::EVENTID_COMPONENT_NOT_INITIALIZED,
                LogSeverity::WarningHi,
                "DpCatalog not initialized!",
            );
            false
        } else if st.num_dp_slots == 0 {
            self.log_simple(
                Self::EVENTID_COMPONENT_NO_MEMORY,
                LogSeverity::WarningHi,
                "DpCatalog couldn't get memory",
            );
            false
        } else {
            true
        }
    }

    /// C++ `doCatalogBuild`.
    fn do_catalog_build(&self, st: &mut CatalogState) -> CmdResponse {
        if !self.check_init(st) {
            return CmdResponse::ExecutionError;
        }
        if st.num_dp_slots == 0 {
            self.log_no_dp_memory();
            return CmdResponse::ExecutionError;
        }
        if st.xmit_in_progress {
            self.log_dp_xmit_in_progress();
            return CmdResponse::ExecutionError;
        }
        Self::reset_state_file_data(st);
        // Proceeding on a failed load would later overwrite the state file
        // and destroy the transmit state it records.
        let response = self.load_state_file(st);
        if response != CmdResponse::Ok {
            Self::reset_state_file_data(st);
            return response;
        }
        Self::reset_catalog(st);
        let response = self.fill_catalog(st);
        if response != CmdResponse::Ok {
            Self::reset_catalog(st);
            Self::reset_state_file_data(st);
            return response;
        }
        self.prune_and_write_state_file(st);
        self.log_simple(
            Self::EVENTID_CATALOG_BUILD_COMPLETE,
            LogSeverity::ActivityHi,
            "Catalog build complete",
        );
        st.catalog_built = true;
        CmdResponse::Ok
    }

    /// C++ `loadStateFile`.
    fn load_state_file(&self, st: &mut CatalogState) -> CmdResponse {
        fw_assert!(!st.state_file_data.is_empty());
        if st.state_file.is_empty() {
            self.log_simple(
                Self::EVENTID_NO_STATE_FILE_SPECIFIED,
                LogSeverity::WarningLo,
                "No specified state file",
            );
            return CmdResponse::Ok;
        }
        let state_file = st.state_file.clone();
        let path = state_file.as_str().unwrap_or_default();
        let mut file = File::new();
        let status = file.open(path, Mode::OpenRead);
        if status == FileStatus::DoesntExist {
            // Expected on first boot; not an error.
            self.log_string(
                Self::EVENTID_NO_STATE_FILE,
                LogSeverity::WarningLo,
                &format!("State file {state_file} doesn't exist"),
                &state_file,
            );
            return CmdResponse::Ok;
        }
        if status != FileStatus::OpOk {
            self.log_state_file_open_error(&state_file, status);
            return CmdResponse::ExecutionError;
        }

        let mut file_loc: i32 = 0;
        st.state_file_entries = 0;
        for entry in 0..st.num_dp_slots {
            let mut bytes = [0u8; STATE_FILE_RECORD_SIZE];
            let mut size = STATE_FILE_RECORD_SIZE as FwSizeType;
            let status = file.read(&mut bytes, &mut size, WaitType::Wait);
            if status != FileStatus::OpOk {
                self.log_string_two_i32(
                    Self::EVENTID_STATE_FILE_READ_ERROR,
                    LogSeverity::WarningHi,
                    &format!(
                        "Error reading state file {state_file}, stat {}, offset: {file_loc}",
                        status as i32
                    ),
                    &state_file,
                    status as i32,
                    file_loc,
                );
                file.close();
                return CmdResponse::ExecutionError;
            }
            if size == 0 {
                // EOF.
                break;
            }
            if size != STATE_FILE_RECORD_SIZE as FwSizeType {
                // Keep what was read.
                self.log_string_two_i32(
                    Self::EVENTID_STATE_FILE_TRUNCATED,
                    LogSeverity::WarningHi,
                    &format!(
                        "Truncated state file {state_file} size. offset: {file_loc} size: {size}"
                    ),
                    &state_file,
                    file_loc,
                    size as i32,
                );
                file.close();
                return CmdResponse::Ok;
            }
            let mut de = ExtBuf::with_len(&mut bytes, STATE_FILE_RECORD_SIZE);
            let mut dir: FwIndexType = 0;
            let mut status = de.deserialize(&mut dir, Endianness::Big);
            if status.is_ok() {
                status =
                    de.deserialize(&mut st.state_file_data[entry].entry.record, Endianness::Big);
            }
            if !status.is_ok() {
                self.log_string_i32(
                    Self::EVENTID_FILE_CORRUPTED_DATA_ERROR,
                    LogSeverity::WarningHi,
                    &format!(
                        "DP file {state_file} contains malformed data (status {})",
                        status as i32
                    ),
                    &state_file,
                    status as i32,
                );
                file.close();
                return CmdResponse::ExecutionError;
            }
            st.state_file_data[entry].entry.dir = dir;
            st.state_file_data[entry].used = true;
            st.state_file_data[entry].visited = false;
            file_loc += STATE_FILE_RECORD_SIZE as i32;
            st.state_file_entries += 1;
        }
        file.close();
        CmdResponse::Ok
    }

    /// C++ `getFileState`: merge the transmit state recorded in the state
    /// file into `entry`. The match requires the SAME directory index plus
    /// entry equality (priority/tSec/tSub/id).
    fn get_file_state(st: &mut CatalogState, entry: &mut DpStateEntry) {
        for line in 0..st.state_file_entries {
            let stored = st.state_file_data[line].entry;
            if stored.dir == entry.dir && stored == *entry {
                entry.record.state = stored.record.state;
                entry.record.blocks = stored.record.blocks;
                st.state_file_data[line].visited = true;
                return;
            }
        }
    }

    /// C++ `pruneAndWriteStateFile`: truncate the file and write back only
    /// the used AND visited lines.
    fn prune_and_write_state_file(&self, st: &mut CatalogState) {
        fw_assert!(!st.state_file_data.is_empty());
        let state_file = st.state_file.clone();
        let mut file = File::new();
        let status = file.open_with_overwrite(
            state_file.as_str().unwrap_or_default(),
            Mode::OpenCreate,
            OverwriteType::Overwrite,
        );
        if status != FileStatus::OpOk {
            self.log_state_file_open_error(&state_file, status);
            return;
        }
        for slot in 0..st.state_file_data.len() {
            let line = st.state_file_data[slot];
            if line.used && line.visited {
                let bytes = serialize_state_file_record(&line.entry);
                let mut size = bytes.len() as FwSizeType;
                let status = file.write(&bytes, &mut size, WaitType::Wait);
                if status != FileStatus::OpOk {
                    self.log_state_file_write_error(&state_file, status);
                    file.close();
                    return;
                }
            }
        }
        file.close();
    }

    /// C++ `appendFileState`: append ONE record for a transmitted product.
    /// Note that, unlike `loadStateFile`, this does not check for an empty
    /// state-file name (C++ parity).
    fn append_file_state(&self, st: &CatalogState, entry: &DpStateEntry) {
        fw_assert!(!st.state_file_data.is_empty());
        fw_assert!(
            (entry.dir as usize) < st.num_directories,
            entry.dir as i32,
            st.num_directories as i32
        );
        let state_file = st.state_file.clone();
        let mut file = File::new();
        let status = file.open(state_file.as_str().unwrap_or_default(), Mode::OpenAppend);
        if status != FileStatus::OpOk {
            self.log_state_file_open_error(&state_file, status);
            return;
        }
        let bytes = serialize_state_file_record(entry);
        let mut size = bytes.len() as FwSizeType;
        let status = file.write(&bytes, &mut size, WaitType::Wait);
        if status != FileStatus::OpOk {
            file.close();
            self.log_state_file_write_error(&state_file, status);
            return;
        }
        file.close();
    }

    /// C++ `fillBinaryTree`: scan every managed directory.
    fn fill_catalog(&self, st: &mut CatalogState) -> CmdResponse {
        let mut total_files: usize = 0;
        let num_directories = st.num_directories.min(DP_MAX_DIRECTORIES);
        for dir in 0..num_directories {
            let dir_name = st.directories[dir].clone();
            self.log_string(
                Self::EVENTID_PROCESSING_DIRECTORY,
                LogSeverity::ActivityLo,
                &format!("Processing directory {dir_name}"),
                &dir_name,
            );
            let mut files_processed: usize = 0;

            let mut dp_dir = Directory::new();
            let status = dp_dir.open(dir_name.as_str().unwrap_or_default(), OpenMode::Read);
            if status != DirStatus::OpOk {
                self.log_directory_open_error(&dir_name, status);
                return CmdResponse::ExecutionError;
            }
            let mut file_count: FwSizeType = 0;
            let status = dp_dir.get_file_count(&mut file_count);
            if status != DirStatus::OpOk {
                self.log_directory_open_error(&dir_name, status);
                return CmdResponse::ExecutionError;
            }

            for _entry in 0..file_count {
                let mut file_name = FileNameString::new();
                let status = dp_dir.read(&mut file_name);
                if status == DirStatus::NoMoreFiles {
                    break;
                }
                if status != DirStatus::OpOk {
                    self.log_directory_open_error(&dir_name, status);
                    return CmdResponse::ExecutionError;
                }
                // The DP extension must be the FINAL suffix; other files are
                // skipped without consuming a catalog slot.
                let name = file_name.as_str().unwrap_or_default().to_string();
                if !name.ends_with(DP_EXT) {
                    continue;
                }
                // The free-slot check happens BEFORE the name is built, so a
                // full catalog breaks out mid-directory.
                if total_files + files_processed == st.num_dp_slots {
                    break;
                }
                let (full_file, format_status) =
                    format_full_path(dir_name.as_str().unwrap_or_default(), &name);
                if format_status != StringFormatStatus::Success {
                    self.log_file_name_format_error(&to_file_name(&name), format_status);
                    continue;
                }
                match self.process_file(st, &full_file, dir) {
                    ProcessFileStatus::Quit => break,
                    ProcessFileStatus::Success => files_processed += 1,
                    ProcessFileStatus::Failed => {}
                }
            }

            total_files += files_processed;
            self.log_processing_directory_complete(
                &dir_name,
                total_files as u32,
                st.pending_files,
                st.pending_dp_bytes,
            );
            if total_files == st.num_dp_slots {
                self.log_string(
                    Self::EVENTID_CATALOG_FULL,
                    LogSeverity::WarningHi,
                    &format!("DpCatalog full during directory {dir_name}"),
                    &dir_name,
                );
                break;
            }
        }
        CmdResponse::Ok
    }

    /// C++ `determineDirectory`: the index of the managed directory holding
    /// `full_file`, or [`DP_MAX_DIRECTORIES`] when it is not managed.
    fn determine_directory(st: &CatalogState, full_file: &str) -> usize {
        fw_assert!(
            st.num_directories <= DP_MAX_DIRECTORIES,
            st.num_directories as i32
        );
        let loc = match full_file.rfind(DIRECTORY_DELIMITER) {
            Some(loc) => loc,
            None => return DP_MAX_DIRECTORIES,
        };
        for dir in 0..st.num_directories {
            let dir_bytes = st.directories[dir].as_bytes();
            if dir_bytes.len() == loc && dir_bytes == &full_file.as_bytes()[..loc] {
                return dir;
            }
        }
        DP_MAX_DIRECTORIES
    }

    /// C++ `processFile`: validate one `.fdp` file and add it to the
    /// catalog.
    fn process_file(
        &self,
        st: &mut CatalogState,
        full_file: &FwDefaultString,
        dir: usize,
    ) -> ProcessFileStatus {
        fw_assert!(dir < DP_MAX_DIRECTORIES, dir as i32);
        let path = full_file.as_str().unwrap_or_default().to_string();
        let event_name = to_file_name(&path);
        self.log_string(
            Self::EVENTID_PROCESSING_FILE,
            LogSeverity::ActivityLo,
            &format!("Processing file {event_name}"),
            &event_name,
        );

        let mut file_size: FwSizeType = 0;
        let size_status = filesystem::get_file_size(&path, &mut file_size);
        if size_status != filesystem::Status::OpOk {
            self.log_string_i32(
                Self::EVENTID_FILE_SIZE_ERROR,
                LogSeverity::WarningHi,
                &format!(
                    "Error getting file {event_name} size. stat: {}",
                    size_status as i32
                ),
                &event_name,
                size_status as i32,
            );
            return ProcessFileStatus::Failed;
        }
        if (file_size as usize) < DpContainer::MIN_PACKET_SIZE {
            self.log_file_read_error(&event_name, FileStatus::BadSize as i32);
            return ProcessFileStatus::Failed;
        }

        let mut dp_file = File::new();
        let status = dp_file.open(&path, Mode::OpenRead);
        if status != FileStatus::OpOk {
            self.log_string_i32(
                Self::EVENTID_FILE_OPEN_ERROR,
                LogSeverity::WarningHi,
                &format!(
                    "Unable to open DP file {event_name} status {}",
                    status as i32
                ),
                &event_name,
                status as i32,
            );
            return ProcessFileStatus::Failed;
        }
        let mut header = [0u8; DpContainer::MIN_PACKET_SIZE];
        let mut size = DpContainer::MIN_PACKET_SIZE as FwSizeType;
        let status = dp_file.read(&mut header, &mut size, WaitType::Wait);
        if status != FileStatus::OpOk {
            self.log_file_read_error(&event_name, status as i32);
            dp_file.close();
            return ProcessFileStatus::Failed;
        }
        if size != DpContainer::MIN_PACKET_SIZE as FwSizeType {
            self.log_file_read_error(&event_name, FileStatus::BadSize as i32);
            dp_file.close();
            return ProcessFileStatus::Failed;
        }
        dp_file.close();

        let mut header_buffer = Buffer::allocate(DpContainer::MIN_PACKET_SIZE);
        header_buffer.data_mut().copy_from_slice(&header);
        let mut container = DpContainer::new();
        container.set_buffer(header_buffer);

        let (hash_status, stored, computed) = container.check_header_hash();
        if hash_status != Success::Success {
            // C++ parity: the arguments are (exp = computed, act = stored),
            // the reverse of what the names suggest.
            self.log_file_hdr_error(&event_name, DpHdrField::Crc, computed, stored);
            return ProcessFileStatus::Failed;
        }
        let des_status = container.deserialize_header();
        if !des_status.is_ok() {
            self.log_string_i32(
                Self::EVENTID_FILE_HDR_DES_ERROR,
                LogSeverity::WarningHi,
                &format!(
                    "Error deserializing DP {event_name} header stat: {}",
                    des_status as i32
                ),
                &event_name,
                des_status as i32,
            );
            return ProcessFileStatus::Failed;
        }
        let expected_data_size = file_size - DpContainer::MIN_PACKET_SIZE as FwSizeType;
        if container.data_size() != expected_data_size {
            self.log_file_read_error(&event_name, FileStatus::BadSize as i32);
            return ProcessFileStatus::Failed;
        }

        let time_tag = container.time_tag();
        let (canonical, format_status) = format_dp_file_name(
            st.directories[dir].as_str().unwrap_or_default(),
            container.id(),
            time_tag.get_seconds(),
            time_tag.get_useconds(),
        );
        if format_status != StringFormatStatus::Success {
            self.log_file_name_format_error(&event_name, format_status);
            return ProcessFileStatus::Failed;
        }
        if canonical.as_bytes() != full_file.as_bytes() {
            self.log_invalid_file_name(&event_name, &canonical);
            return ProcessFileStatus::Failed;
        }
        if container.state() == DpState::Transmitted {
            self.log_string(
                Self::EVENTID_DP_FILE_SKIPPED,
                LogSeverity::ActivityHi,
                &format!("Already Transmitted DP file {event_name} not added"),
                &event_name,
            );
            return ProcessFileStatus::Failed;
        }

        let mut entry = DpStateEntry {
            dir: dir as FwIndexType,
            record: DpRecord {
                id: container.id(),
                t_sec: time_tag.get_seconds(),
                t_sub: time_tag.get_useconds(),
                priority: container.priority(),
                size: file_size,
                blocks: 0,
                state: container.state(),
            },
        };
        Self::get_file_state(st, &mut entry);

        // A duplicate insert would update the tree in place; skipping it
        // keeps the pending counters from double-counting.
        if st.catalog.find(&entry) {
            self.log_string(
                Self::EVENTID_DP_FILE_SKIPPED,
                LogSeverity::ActivityHi,
                &format!("Already Transmitted DP file {event_name} not added"),
                &event_name,
            );
            return ProcessFileStatus::Failed;
        }
        if !st.catalog.insert(entry) {
            self.log_record(
                Self::EVENTID_DP_INSERT_ERROR,
                LogSeverity::WarningHi,
                "Error deserializing DP",
                &entry.record,
            );
            return ProcessFileStatus::Quit;
        }
        st.pending_files += 1;
        st.pending_dp_bytes += entry.record.size;
        if st.pending_files as usize > st.num_dp_slots {
            self.log_record(
                Self::EVENTID_DP_CATALOG_FULL,
                LogSeverity::WarningHi,
                "Catalog full trying to insert DP",
                &entry.record,
            );
            return ProcessFileStatus::Quit;
        }
        self.log_string(
            Self::EVENTID_DP_FILE_ADDED,
            LogSeverity::ActivityHi,
            &format!("DP file {canonical} added at runtime"),
            &canonical,
        );
        ProcessFileStatus::Success
    }

    /// C++ `doCatalogXmit`.
    fn do_catalog_xmit(&self, st: &mut CatalogState) -> CmdResponse {
        if !self.check_init(st) {
            return CmdResponse::ExecutionError;
        }
        if st.num_dp_slots == 0 {
            self.log_no_dp_memory();
            return CmdResponse::ExecutionError;
        }
        if st.xmit_in_progress {
            self.log_dp_xmit_in_progress();
            return CmdResponse::ExecutionError;
        }
        if !st.catalog_built {
            self.log_simple(
                Self::EVENTID_XMIT_UNBUILT_CATALOG,
                LogSeverity::WarningHi,
                "Cannot Transmit a Catalog before Building",
            );
            return CmdResponse::ExecutionError;
        }
        st.xmit_bytes = 0;
        st.xmit_in_progress = true;
        self.send_next_entry(st);
        CmdResponse::Ok
    }

    /// C++ `sendNextEntry`: hand the highest-priority entry to FileDownlink.
    fn send_next_entry(&self, st: &mut CatalogState) {
        // How STOP_XMIT_CATALOG breaks the chain.
        if !st.xmit_in_progress {
            return;
        }
        let entry = match st.catalog.first() {
            Some(entry) => entry,
            None => {
                st.xmit_in_progress = false;
                self.log_u64(
                    Self::EVENTID_CATALOG_XMIT_COMPLETED,
                    LogSeverity::ActivityHi,
                    &format!(
                        "Catalog transmission completed.  {} bytes transmitted.",
                        st.xmit_bytes
                    ),
                    st.xmit_bytes,
                );
                self.dispatch_waited_response(st, CmdResponse::Ok);
                return;
            }
        };
        st.current_xmit_entry = entry;
        st.has_current_xmit = true;

        let (file_name, format_status) = format_dp_file_name(
            st.directories[entry.dir as usize]
                .as_str()
                .unwrap_or_default(),
            entry.record.id,
            entry.record.t_sec,
            entry.record.t_sub,
        );
        st.curr_xmit_file_name = file_name;
        if format_status != StringFormatStatus::Success {
            let name = st.curr_xmit_file_name.clone();
            self.log_file_name_format_error(&name, format_status);
            // No send is in flight, so no fileDone will arrive.
            st.has_current_xmit = false;
            st.xmit_in_progress = false;
            self.dispatch_waited_response(st, CmdResponse::ExecutionError);
            return;
        }
        let name = st.curr_xmit_file_name.clone();
        self.log_sending_product(&name, entry.record.size as u32, entry.record.priority);

        // The port's string type is `string size 100`: long DP paths are
        // truncated at the port boundary (C++ parity).
        let mut port_name = SendFileNameArg::new();
        port_name.set_bytes(name.as_bytes());
        let port = self.file_out.get();
        let response = port
            .target
            .invoke(port.port_num, &port_name, &port_name, 0, 0);
        if *response.get_status() != SendFileStatus::StatusOk {
            self.log_send_file_status(
                Self::EVENTID_DP_FILE_SEND_ERROR,
                &format!(
                    "Error sending DP file {name}, stat {}. Halting xmit.",
                    response.get_status().as_repr()
                ),
                &name,
                *response.get_status(),
            );
            // A rejected send produces no fileDone callback.
            st.has_current_xmit = false;
            st.xmit_in_progress = false;
            self.dispatch_waited_response(st, CmdResponse::ExecutionError);
        }
    }

    /// C++ `dispatchWaitedResponse`: answer a waiting `START_XMIT_CATALOG`
    /// exactly once.
    fn dispatch_waited_response(&self, st: &mut CatalogState, response: CmdResponse) {
        if st.xmit_cmd_wait {
            self.cmd
                .cmd_response(st.xmit_op_code, st.xmit_cmd_seq, response);
            st.xmit_cmd_wait = false;
            st.xmit_op_code = 0;
            st.xmit_cmd_seq = 0;
        }
    }
}

/// Serialize one state-file record: `[FwIndexType dir][DpRecord]`, 31 bytes
/// big-endian with the default config.
fn serialize_state_file_record(entry: &DpStateEntry) -> [u8; STATE_FILE_RECORD_SIZE] {
    let mut bytes = [0u8; STATE_FILE_RECORD_SIZE];
    {
        let mut ser = ExtBuf::new(&mut bytes);
        let status = ser.serialize(&entry.dir, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = ser.serialize(&entry.record, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
    }
    bytes
}

/// C++ `fullFile.format("%s/%s", dirName, fileName)` into an `Fw::String`
/// (capacity `FW_FIXED_LENGTH_STRING_SIZE` = 256).
fn format_full_path(dir_name: &str, file_name: &str) -> (FwDefaultString, StringFormatStatus) {
    let formatted = format!("{dir_name}/{file_name}");
    let mut out = FwDefaultString::new();
    if formatted.len() > FwDefaultString::max_length() {
        return (out, StringFormatStatus::Overflowed);
    }
    out.set(&formatted);
    (out, StringFormatStatus::Success)
}

/// Event string arguments are `string size FileNameStringSize`.
fn to_file_name(value: &str) -> FileNameString {
    let mut out = FileNameString::new();
    out.set(value);
    out
}

// ---------------------------------------------------------------------------
// Port and command handlers
// ---------------------------------------------------------------------------

impl DpCatalog {
    /// `pingIn_handler`: immediate echo.
    fn ping_handler(&self, _port_num: FwIndexType, key: u32) {
        let port = self.ping_out.get();
        port.target.invoke(port.port_num, key);
    }

    /// `fileDone_handler`: account for a completed downlink and start the
    /// next one.
    fn file_done_handler(&self, _port_num: FwIndexType, resp: SendFileResponse) {
        let st = &mut *self.state.lock().unwrap();
        if *resp.get_status() != SendFileStatus::StatusOk {
            let name = st.curr_xmit_file_name.clone();
            self.log_send_file_status(
                Self::EVENTID_DP_FILE_XMIT_ERROR,
                &format!(
                    "Error transmitting DP file {name}, stat {}. Halting xmit.",
                    resp.get_status().as_repr()
                ),
                &name,
                *resp.get_status(),
            );
            st.xmit_in_progress = false;
            self.dispatch_waited_response(st, CmdResponse::ExecutionError);
            return;
        }
        // The catalog was cleared while this file was in flight.
        if !st.catalog_built {
            st.has_current_xmit = false;
            st.xmit_in_progress = false;
            self.dispatch_waited_response(st, CmdResponse::ExecutionError);
            return;
        }
        fw_assert!(st.has_current_xmit);

        let size = st.current_xmit_entry.record.size;
        st.pending_dp_bytes -= size;
        st.pending_files -= 1;
        let name = st.curr_xmit_file_name.clone();
        self.log_product_complete(&name, st.pending_files, st.pending_dp_bytes);

        st.current_xmit_entry.record.state = DpState::Transmitted;
        let entry = st.current_xmit_entry;
        self.append_file_state(st, &entry);
        st.xmit_bytes += size;
        let removed = st.catalog.remove(&entry);
        fw_assert!(removed);
        st.has_current_xmit = false;

        self.send_next_entry(st);
    }

    /// `addToCat_handler`: a product written at runtime. The `priority` and
    /// `size` arguments are IGNORED — everything is re-read from the file's
    /// own header (C++ parity).
    fn add_to_cat_handler(
        &self,
        _port_num: FwIndexType,
        file_name: &FileNameString,
        _priority: FwDpPriorityType,
        _size: FwSizeType,
    ) {
        let st = &mut *self.state.lock().unwrap();
        if !self.check_init(st) {
            return;
        }
        if st.num_dp_slots == 0 {
            self.log_no_dp_memory();
            return;
        }
        if !st.catalog_built {
            self.log_string(
                Self::EVENTID_NOT_LOADED,
                LogSeverity::ActivityHi,
                &format!("Not adding file {file_name} now; Catalog not yet loaded"),
                file_name,
            );
            return;
        }
        let path = file_name.as_str().unwrap_or_default().to_string();
        let dir = Self::determine_directory(st, &path);
        if dir == DP_MAX_DIRECTORIES {
            self.log_string(
                Self::EVENTID_DIRECTORY_NOT_MANAGED,
                LogSeverity::WarningHi,
                &format!("Unable to add file {file_name}; directory not managed"),
                file_name,
            );
            return;
        }
        let mut full_file = FwDefaultString::new();
        full_file.set(&path);
        if self.process_file(st, &full_file, dir) == ProcessFileStatus::Success {
            // If the catalog already finished, only resume when asked to.
            if !st.xmit_in_progress && st.remain_active {
                st.xmit_in_progress = true;
                self.send_next_entry(st);
            }
            self.prune_and_write_state_file(st);
        }
    }

    /// `BUILD_CATALOG` handler.
    fn build_catalog_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        let response = {
            let st = &mut *self.state.lock().unwrap();
            self.do_catalog_build(st)
        };
        self.cmd.cmd_response(op_code, cmd_seq, response);
    }

    /// `START_XMIT_CATALOG` handler: the waited response is armed BEFORE
    /// the transmit starts, so an empty catalog (which completes inline)
    /// still answers.
    fn start_xmit_catalog_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        wait: Wait,
        remain_active: bool,
    ) {
        let st = &mut *self.state.lock().unwrap();
        st.remain_active = remain_active;
        if wait == Wait::Wait {
            st.xmit_cmd_wait = true;
            st.xmit_op_code = op_code;
            st.xmit_cmd_seq = cmd_seq;
        }
        let response = self.do_catalog_xmit(st);
        if response != CmdResponse::Ok {
            st.xmit_cmd_wait = false;
            st.xmit_op_code = 0;
            st.xmit_cmd_seq = 0;
            self.cmd.cmd_response(op_code, cmd_seq, response);
        } else if wait == Wait::NoWait {
            self.cmd.cmd_response(op_code, cmd_seq, response);
        }
    }

    /// `STOP_XMIT_CATALOG` handler: stopping while idle is benign. The
    /// in-flight FileDownlink transfer is NOT cancelled; its `fileDone`
    /// still does the accounting and then finds `xmit_in_progress` false.
    fn stop_xmit_catalog_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        let st = &mut *self.state.lock().unwrap();
        if !st.xmit_in_progress {
            self.log_simple(
                Self::EVENTID_XMIT_NOT_ACTIVE,
                LogSeverity::WarningLo,
                "DpCatalog transmit not active",
            );
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
        } else {
            self.log_u64(
                Self::EVENTID_CATALOG_XMIT_STOPPED,
                LogSeverity::ActivityHi,
                &format!(
                    "Catalog transmission stopped. {} bytes transmitted.",
                    st.xmit_bytes
                ),
                st.xmit_bytes,
            );
            st.xmit_in_progress = false;
            self.dispatch_waited_response(st, CmdResponse::Ok);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
        }
    }

    /// `CLEAR_CATALOG` handler: unconditional — it does NOT check or clear
    /// `xmit_in_progress` / `xmit_cmd_wait` (C++ parity), so a later
    /// `fileDone` takes the "catalog not built" path.
    fn clear_catalog_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        {
            let st = &mut *self.state.lock().unwrap();
            Self::reset_catalog(st);
            Self::reset_state_file_data(st);
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// Command dispatch on the local opcode (exactly-once response
    /// discipline: FormatError on deserialization failure or residual
    /// bytes, ValidationError on an invalid `Fw.Wait`).
    fn handle_command(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_BUILD_CATALOG => {
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.build_catalog_cmd_handler(op_code, cmd_seq);
            }
            Self::OPCODE_START_XMIT_CATALOG => {
                let mut wait_raw = 0u8;
                if !args.deserialize_u8_be(&mut wait_raw).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                let wait = match Wait::try_from(wait_raw) {
                    Ok(wait) => wait,
                    Err(_) => {
                        self.cmd
                            .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
                        return;
                    }
                };
                let mut remain_active = false;
                if !args.deserialize_bool_be(&mut remain_active).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.start_xmit_catalog_cmd_handler(op_code, cmd_seq, wait, remain_active);
            }
            Self::OPCODE_STOP_XMIT_CATALOG => {
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.stop_xmit_catalog_cmd_handler(op_code, cmd_seq);
            }
            Self::OPCODE_CLEAR_CATALOG => {
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.clear_catalog_cmd_handler(op_code, cmd_seq);
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }

    // -- Events ------------------------------------------------------------

    /// Only the ids in [`THROTTLED_EVENTS`] are `throttle 10`; there is no
    /// `CLEAR_EVENT_THROTTLE` command on this component.
    fn ok_to_emit(&self, id: FwEventIdType) -> bool {
        if THROTTLED_EVENTS.contains(&id) {
            self.throttles[id as usize].ok_to_emit()
        } else {
            true
        }
    }

    fn log_simple(&self, id: FwEventIdType, severity: LogSeverity, text: &str) {
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt
            .log_event(self.id_base(), id, severity, text, |_buf| {
                fprime_fw::SerializeStatus::Ok
            });
    }

    fn log_string(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        text: &str,
        value: &FileNameString,
    ) {
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt
            .log_event(self.id_base(), id, severity, text, |buf| {
                value.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            });
    }

    fn log_string_i32(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        text: &str,
        value: &FileNameString,
        stat: i32,
    ) {
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt
            .log_event(self.id_base(), id, severity, text, |buf| {
                fw_try!(value.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_i32_be(stat)
            });
    }

    fn log_string_two_i32(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        text: &str,
        value: &FileNameString,
        first: i32,
        second: i32,
    ) {
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt
            .log_event(self.id_base(), id, severity, text, |buf| {
                fw_try!(value.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_i32_be(first));
                buf.serialize_i32_be(second)
            });
    }

    fn log_record(&self, id: FwEventIdType, severity: LogSeverity, text: &str, record: &DpRecord) {
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt
            .log_event(self.id_base(), id, severity, text, |buf| {
                buf.serialize(record, Endianness::Big)
            });
    }

    fn log_u64(&self, id: FwEventIdType, severity: LogSeverity, text: &str, value: u64) {
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt
            .log_event(self.id_base(), id, severity, text, |buf| {
                buf.serialize_u64_be(value)
            });
    }

    fn log_no_dp_memory(&self) {
        self.log_simple(
            Self::EVENTID_NO_DP_MEMORY,
            LogSeverity::WarningHi,
            "No memory for DP",
        );
    }

    fn log_dp_xmit_in_progress(&self) {
        self.log_simple(
            Self::EVENTID_DP_XMIT_IN_PROGRESS,
            LogSeverity::WarningLo,
            "Cannot build new catalog while DPs are being transmitted",
        );
    }

    fn log_directory_open_error(&self, dir: &FileNameString, status: DirStatus) {
        self.log_string_i32(
            Self::EVENTID_DIRECTORY_OPEN_ERROR,
            LogSeverity::WarningHi,
            &format!("Unable to process directory {dir} status {}", status as i32),
            dir,
            status as i32,
        );
    }

    fn log_state_file_open_error(&self, file: &FileNameString, status: FileStatus) {
        self.log_string_i32(
            Self::EVENTID_STATE_FILE_OPEN_ERROR,
            LogSeverity::WarningHi,
            &format!("Error opening state file {file}, stat: {}", status as i32),
            file,
            status as i32,
        );
    }

    fn log_state_file_write_error(&self, file: &FileNameString, status: FileStatus) {
        self.log_string_i32(
            Self::EVENTID_STATE_FILE_WRITE_ERROR,
            LogSeverity::WarningHi,
            &format!("Error writing state file {file}, stat {}", status as i32),
            file,
            status as i32,
        );
    }

    fn log_file_read_error(&self, file: &FileNameString, stat: i32) {
        self.log_string_i32(
            Self::EVENTID_FILE_READ_ERROR,
            LogSeverity::WarningHi,
            &format!("Error reading DP file {file} status {stat}"),
            file,
            stat,
        );
    }

    fn log_file_hdr_error(
        &self,
        file: &FileNameString,
        field: DpHdrField,
        expected: u32,
        actual: u32,
    ) {
        let id = Self::EVENTID_FILE_HDR_ERROR;
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            id,
            LogSeverity::WarningHi,
            &format!(
                "Error reading DP {file} header {} field. Expected: {expected} Actual: {actual}",
                field.as_repr()
            ),
            |buf| {
                fw_try!(file.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize(&field, Endianness::Big));
                fw_try!(buf.serialize_u32_be(expected));
                buf.serialize_u32_be(actual)
            },
        );
    }

    fn log_file_name_format_error(&self, file: &FileNameString, status: StringFormatStatus) {
        let id = Self::EVENTID_FILE_NAME_FORMAT_ERROR;
        if !self.ok_to_emit(id) {
            return;
        }
        let code = status.as_repr();
        self.evt.log_event(
            self.id_base(),
            id,
            LogSeverity::WarningHi,
            &format!("Failed to format DP file name for {file} with status {code}"),
            |buf| {
                fw_try!(file.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u8_be(code)
            },
        );
    }

    fn log_invalid_file_name(&self, file: &FileNameString, expected: &FileNameString) {
        let id = Self::EVENTID_INVALID_FILE_NAME;
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            id,
            LogSeverity::WarningHi,
            &format!("Invalid DP file name {file}. Expected {expected}"),
            |buf| {
                fw_try!(file.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                expected.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_processing_directory_complete(
        &self,
        dir: &FileNameString,
        total: u32,
        pending: u32,
        pending_bytes: u64,
    ) {
        let id = Self::EVENTID_PROCESSING_DIRECTORY_COMPLETE;
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            id,
            LogSeverity::ActivityHi,
            &format!(
                "Completed processing directory {dir}. Total products: {total} \
                 Pending products: {pending} Pending bytes: {pending_bytes}"
            ),
            |buf| {
                fw_try!(dir.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(total));
                fw_try!(buf.serialize_u32_be(pending));
                buf.serialize_u64_be(pending_bytes)
            },
        );
    }

    fn log_sending_product(&self, file: &FileNameString, bytes: u32, prio: u32) {
        let id = Self::EVENTID_SENDING_PRODUCT;
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            id,
            LogSeverity::ActivityLo,
            &format!("Sending product {file} of size {bytes} priority {prio}"),
            |buf| {
                fw_try!(file.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(bytes));
                buf.serialize_u32_be(prio)
            },
        );
    }

    fn log_product_complete(&self, file: &FileNameString, pending: u32, pending_bytes: u64) {
        let id = Self::EVENTID_PRODUCT_COMPLETE;
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            id,
            LogSeverity::ActivityLo,
            &format!(
                "Product {file} complete. Pending products: {pending} \
                 Pending bytes: {pending_bytes}"
            ),
            |buf| {
                fw_try!(file.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(pending));
                buf.serialize_u64_be(pending_bytes)
            },
        );
    }

    fn log_send_file_status(
        &self,
        id: FwEventIdType,
        text: &str,
        file: &FileNameString,
        status: SendFileStatus,
    ) {
        if !self.ok_to_emit(id) {
            return;
        }
        self.evt
            .log_event(self.id_base(), id, LogSeverity::WarningHi, text, |buf| {
                fw_try!(file.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize(&status, Endianness::Big)
            });
    }
}

// -- Async input adapters ---------------------------------------------------

/// `pingIn` adapter.
struct PingInAdapter {
    comp: Arc<DpCatalog>,
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

/// `fileDone` adapter: `[msg_type][port_num][SendFileResponse (5 B)]`.
struct FileDoneAdapter {
    comp: Arc<DpCatalog>,
}

impl SendFileCompletePort for FileDoneAdapter {
    fn invoke(&self, port_num: FwIndexType, resp: SendFileResponse) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_FILE_DONE, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize(&resp, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// `addToCat` adapter:
/// `[msg_type][port_num][fileName u16+bytes][priority u32][size u64]`.
struct AddToCatAdapter {
    comp: Arc<DpCatalog>,
}

impl DpWrittenPort for AddToCatAdapter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        file_name: &FileNameString,
        priority: FwDpPriorityType,
        size: FwSizeType,
    ) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_ADD_TO_CAT, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize(file_name, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(priority);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u64_be(size);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// `CmdDisp` adapter.
struct CmdInAdapter {
    comp: Arc<DpCatalog>,
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

impl ComponentDispatch for DpCatalog {
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
            MSG_TYPE_FILE_DONE => {
                let mut resp = SendFileResponse::default();
                if !buf.deserialize(&mut resp, Endianness::Big).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.file_done_handler(port_num, resp);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_ADD_TO_CAT => {
                let mut file_name = FileNameString::new();
                let mut priority: FwDpPriorityType = 0;
                let mut size: FwSizeType = 0;
                if !buf.deserialize(&mut file_name, Endianness::Big).is_ok()
                    || !buf.deserialize_u32_be(&mut priority).is_ok()
                    || !buf.deserialize_u64_be(&mut size).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                self.add_to_cat_handler(port_num, &file_name, priority, size);
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
                self.handle_command(op_code, cmd_seq, &mut args);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for DpCatalog {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort, TimePort, TlmPort};
    use fprime_fw::{LogBuffer, TextLogString, Time, TimeBase, TlmBuffer};
    use std::path::{Path, PathBuf};
    use std::sync::atomic::{AtomicU32, Ordering};

    const ID_BASE: FwIdType = 0x4000;

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    fn temp_dir(tag: &str) -> PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "fprime_rust_dpcat_{tag}_{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&path).unwrap();
        path
    }

    /// Write a well-formed `.fdp` file into `dir` and return its path.
    fn write_dp_file(
        dir: &Path,
        id: FwDpIdType,
        priority: FwDpPriorityType,
        seconds: u32,
        useconds: u32,
        state: DpState,
        data: &[u8],
    ) -> String {
        let mut container = DpContainer::with_buffer(
            id,
            Buffer::allocate(DpContainer::MIN_PACKET_SIZE + data.len()),
        );
        container.set_priority(priority);
        container.set_time_tag(Time::new(TimeBase::TbWorkstationTime, 0, seconds, useconds));
        container.set_dp_state(state);
        container.data_region_mut()[..data.len()].copy_from_slice(data);
        container.set_data_size(data.len() as FwSizeType);
        container.serialize_header();
        container.update_data_hash();
        let (name, status) = format_dp_file_name(dir.to_str().unwrap(), id, seconds, useconds);
        assert_eq!(status, StringFormatStatus::Success);
        let path = name.as_str().unwrap().to_string();
        std::fs::write(&path, container.buffer().data()).unwrap();
        path
    }

    #[derive(Default)]
    struct GroundStub {
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        pings: Mutex<Vec<u32>>,
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
            _id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            _text: &mut TextLogString,
        ) {
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

    impl PingPort for GroundStub {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.pings.lock().unwrap().push(key);
        }
    }

    struct TimeStub;

    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 100, 42);
        }
    }

    /// `Svc::FileDownlink` stub: records the requests and answers with a
    /// configurable status.
    #[derive(Default)]
    struct DownlinkStub {
        requests: Mutex<Vec<String>>,
        status: Mutex<Option<SendFileStatus>>,
    }

    impl SendFileRequestPort for DownlinkStub {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            source_file_name: &SendFileNameArg,
            _dest_file_name: &SendFileNameArg,
            _offset: u32,
            _length: u32,
        ) -> SendFileResponse {
            self.requests
                .lock()
                .unwrap()
                .push(source_file_name.as_str().unwrap_or_default().to_string());
            let status = self
                .status
                .lock()
                .unwrap()
                .unwrap_or(SendFileStatus::StatusOk);
            SendFileResponse::new(status, 0)
        }
    }

    struct Harness {
        comp: Arc<DpCatalog>,
        ground: Arc<GroundStub>,
        downlink: Arc<DownlinkStub>,
        dirs: Vec<PathBuf>,
        state_file: PathBuf,
    }

    impl Harness {
        fn new(tag: &str, num_dirs: usize) -> Self {
            let dirs: Vec<PathBuf> = (0..num_dirs)
                .map(|i| temp_dir(&format!("{tag}_{i}")))
                .collect();
            let state_file = dirs[0].join("dp_state.dat");
            Self::build(&dirs, &state_file, DP_MAX_FILES as FwSizeType, true)
        }

        fn build(dirs: &[PathBuf], state_file: &Path, slots: FwSizeType, configure: bool) -> Self {
            let comp = DpCatalog::new("dpCatalog");
            let ground = Arc::new(GroundStub::default());
            let downlink = Arc::new(DownlinkStub::default());
            comp.active.queued.base.set_id_base(ID_BASE);
            comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
            comp.cmd.cmd_response_out.connect(ground.clone(), 0);
            comp.evt.log_out.connect(ground.clone(), 0);
            comp.evt.text_log_out.connect(ground.clone(), 0);
            comp.evt.time_out.connect(Arc::new(TimeStub), 0);
            comp.tlm.tlm_out.connect(ground.clone(), 0);
            comp.ping_out.connect(ground.clone(), 0);
            comp.file_out.connect(downlink.clone(), 0);
            comp.init(32);
            if configure {
                let names: Vec<FileNameString> = dirs
                    .iter()
                    .map(|d| to_file_name(d.to_str().unwrap()))
                    .collect();
                comp.configure_with_slots(
                    &names,
                    &to_file_name(state_file.to_str().unwrap()),
                    slots,
                );
            }
            Self {
                comp,
                ground,
                downlink,
                dirs: dirs.to_vec(),
                state_file: state_file.to_path_buf(),
            }
        }

        fn drain(&self) {
            let _ = self
                .comp
                .active
                .queued
                .dispatch_available_messages(self.comp.as_ref());
        }

        fn send_cmd(&self, op_code: FwOpcodeType, cmd_seq: u32, arg_bytes: &[u8]) {
            let mut args = CmdArgBuffer::new();
            assert!(args.set_buff(arg_bytes).is_ok());
            let port = self.comp.cmd_in(0);
            port.target
                .invoke(port.port_num, op_code, cmd_seq, &mut args);
            self.drain();
        }

        fn build_catalog(&self) {
            self.send_cmd(ID_BASE + DpCatalog::OPCODE_BUILD_CATALOG, 1, &[]);
        }

        fn start_xmit(&self, wait: Wait, remain_active: bool) {
            self.send_cmd(
                ID_BASE + DpCatalog::OPCODE_START_XMIT_CATALOG,
                2,
                &[wait.as_repr(), if remain_active { 0xFF } else { 0x00 }],
            );
        }

        fn file_done(&self, status: SendFileStatus) {
            let port = self.comp.file_done(0);
            port.target
                .invoke(port.port_num, SendFileResponse::new(status, 0));
            self.drain();
        }

        fn event_ids(&self) -> Vec<FwEventIdType> {
            self.ground
                .events
                .lock()
                .unwrap()
                .iter()
                .map(|e| e.0 - ID_BASE)
                .collect()
        }

        fn events_with_id(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
            self.ground
                .events
                .lock()
                .unwrap()
                .iter()
                .filter(|e| e.0 == ID_BASE + id)
                .map(|e| e.2.clone())
                .collect()
        }

        fn responses(&self) -> Vec<(FwOpcodeType, u32, CmdResponse)> {
            self.ground.responses.lock().unwrap().clone()
        }

        fn requests(&self) -> Vec<String> {
            self.downlink.requests.lock().unwrap().clone()
        }
    }

    #[test]
    fn an_event_directory_name_is_clipped_to_the_log_string_cap() {
        // C++ parity: the arg is declared `string size FileNameStringSize`
        // (240) but the generated `log_*` method carries it as
        // `Fw::LogStringArg` = `StringTemplate<FW_LOG_STRING_MAX_SIZE>`, so
        // anything past 200 bytes never reaches the wire.
        let base = temp_dir("longdir");
        let pad = 210usize.saturating_sub(base.to_str().unwrap().len() + 1);
        assert!(pad > 0);
        let dir = base.join("d".repeat(pad));
        std::fs::create_dir_all(&dir).unwrap();
        let full = dir.to_str().unwrap().to_string();
        assert!(full.len() > FW_LOG_STRING_MAX_SIZE);
        assert!(full.len() <= FILE_NAME_STRING_SIZE);

        let state_file = base.join("dp_state.dat");
        let h = Harness::build(&[dir], &state_file, DP_MAX_FILES as FwSizeType, true);
        h.build_catalog();

        let args = h.events_with_id(DpCatalog::EVENTID_PROCESSING_DIRECTORY);
        assert_eq!(args.len(), 1);
        assert_eq!(
            &args[0][..2],
            &(FW_LOG_STRING_MAX_SIZE as u16).to_be_bytes()
        );
        assert_eq!(args[0].len(), 2 + FW_LOG_STRING_MAX_SIZE);
        assert_eq!(&args[0][2..], &full.as_bytes()[..FW_LOG_STRING_MAX_SIZE]);
    }

    // -- Wire formats ------------------------------------------------------

    #[test]
    fn dp_record_serializes_to_twenty_nine_bytes() {
        assert_eq!(DpRecord::SERIALIZED_SIZE, 29);
        let record = DpRecord {
            id: 0x0102_0304,
            t_sec: 0x0506_0708,
            t_sub: 0x090A_0B0C,
            priority: 0x0D0E_0F10,
            size: 0x1112_1314_1516_1718,
            blocks: 0x191A_1B1C,
            state: DpState::Transmitted,
        };
        let mut bytes = [0u8; 29];
        let mut ser = ExtBuf::new(&mut bytes);
        assert!(ser.serialize(&record, Endianness::Big).is_ok());
        assert_eq!(ser.get_size(), 29);
        assert_eq!(
            bytes,
            [
                0x01, 0x02, 0x03, 0x04, // id
                0x05, 0x06, 0x07, 0x08, // tSec
                0x09, 0x0A, 0x0B, 0x0C, // tSub
                0x0D, 0x0E, 0x0F, 0x10, // priority
                0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, // size
                0x19, 0x1A, 0x1B, 0x1C, // blocks
                0x02, // state = TRANSMITTED
            ]
        );
    }

    #[test]
    fn state_file_records_are_thirty_one_bytes() {
        assert_eq!(STATE_FILE_RECORD_SIZE, 31);
        let entry = DpStateEntry {
            dir: 1,
            record: DpRecord {
                id: 7,
                t_sec: 2,
                t_sub: 3,
                priority: 4,
                size: 65,
                blocks: 0,
                state: DpState::Untransmitted,
            },
        };
        let bytes = serialize_state_file_record(&entry);
        assert_eq!(
            bytes,
            [
                0x00, 0x01, // dir (FwIndexType = I16)
                0x00, 0x00, 0x00, 0x07, // id
                0x00, 0x00, 0x00, 0x02, // tSec
                0x00, 0x00, 0x00, 0x03, // tSub
                0x00, 0x00, 0x00, 0x04, // priority
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, // size = 65
                0x00, 0x00, 0x00, 0x00, // blocks
                0x00, // state = UNTRANSMITTED
            ]
        );
    }

    #[test]
    fn dictionary_ids_match_the_fpp_model() {
        assert_eq!(DpCatalog::OPCODE_BUILD_CATALOG, 0);
        assert_eq!(DpCatalog::OPCODE_START_XMIT_CATALOG, 1);
        assert_eq!(DpCatalog::OPCODE_STOP_XMIT_CATALOG, 2);
        assert_eq!(DpCatalog::OPCODE_CLEAR_CATALOG, 3);
        // Explicit ids with gaps at 6..9 and 33.
        assert_eq!(DpCatalog::EVENTID_DIRECTORY_OPEN_ERROR, 0);
        assert_eq!(DpCatalog::EVENTID_DIRECTORY_NOT_MANAGED, 5);
        assert_eq!(DpCatalog::EVENTID_CATALOG_XMIT_STARTED, 10);
        assert_eq!(DpCatalog::EVENTID_COMPONENT_NOT_INITIALIZED, 20);
        assert_eq!(DpCatalog::EVENTID_NO_DP_MEMORY, 32);
        assert_eq!(DpCatalog::EVENTID_XMIT_NOT_ACTIVE, 34);
        assert_eq!(DpCatalog::EVENTID_FILE_NAME_FORMAT_ERROR, 49);
        assert_eq!(DpCatalog::CHANID_CATALOG_DPS, 0);
        assert_eq!(DpCatalog::CHANID_DPS_SENT, 1);
        assert_eq!(DpHdrField::Crc.as_repr(), 3);
        assert_eq!(QUEUE_MSG_SIZE, 522);

        let h = Harness::new("dict", 1);
        h.comp.reg_commands();
        assert_eq!(
            *h.ground.regs.lock().unwrap(),
            vec![ID_BASE, ID_BASE + 1, ID_BASE + 2, ID_BASE + 3]
        );
    }

    // -- Ordering ----------------------------------------------------------

    fn entry(dir: FwIndexType, priority: u32, t_sec: u32, t_sub: u32, id: u32) -> DpStateEntry {
        DpStateEntry {
            dir,
            record: DpRecord {
                id,
                t_sec,
                t_sub,
                priority,
                size: 65,
                blocks: 0,
                state: DpState::Untransmitted,
            },
        }
    }

    #[test]
    fn entries_order_by_priority_then_time_then_id() {
        let a = entry(0, 1, 10, 0, 5);
        let b = entry(0, 2, 0, 0, 0);
        let c = entry(0, 1, 20, 0, 0);
        let d = entry(0, 1, 10, 5, 0);
        let e = entry(0, 1, 10, 0, 9);
        assert!(a < b); // lower priority value first
        assert!(a < c); // older time first
        assert!(a < d); // older subsecond first
        assert!(a < e); // lower id first
        // `dir` is NOT part of the ordering or of equality.
        assert_eq!(entry(0, 1, 10, 0, 5), entry(1, 1, 10, 0, 5));
    }

    #[test]
    fn the_catalog_keeps_the_highest_priority_entry_first() {
        let mut catalog = Catalog::new();
        assert!(catalog.insert(entry(0, 5, 0, 0, 1)));
        assert!(catalog.insert(entry(0, 1, 9, 0, 2)));
        assert!(catalog.insert(entry(0, 1, 3, 0, 3)));
        assert_eq!(catalog.len(), 3);
        assert_eq!(catalog.first().unwrap().record.id, 3);
        assert!(catalog.find(&entry(0, 1, 9, 0, 2)));
        assert!(catalog.remove(&entry(0, 1, 3, 0, 3)));
        assert_eq!(catalog.first().unwrap().record.id, 2);
        assert!(!catalog.remove(&entry(0, 1, 3, 0, 3)));
    }

    #[test]
    fn the_catalog_rejects_an_insert_past_its_capacity() {
        let mut catalog = Catalog::new();
        for id in 0..DP_MAX_FILES as u32 {
            assert!(catalog.insert(entry(0, 0, 0, 0, id)));
        }
        assert!(!catalog.insert(entry(0, 0, 0, 0, DP_MAX_FILES as u32)));
        // An equal entry updates in place instead of failing.
        assert!(catalog.insert(entry(0, 0, 0, 0, 0)));
    }

    // -- Catalog build -----------------------------------------------------

    #[test]
    fn build_catalog_orders_products_by_priority_and_time() {
        let h = Harness::new("order", 1);
        let dir = h.dirs[0].clone();
        // Deliberately written out of order.
        write_dp_file(&dir, 3, 5, 100, 0, DpState::Untransmitted, &[]);
        write_dp_file(&dir, 1, 1, 200, 0, DpState::Untransmitted, &[1]);
        write_dp_file(&dir, 2, 1, 100, 0, DpState::Untransmitted, &[1, 2]);
        h.build_catalog();
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_BUILD_CATALOG,
                1,
                CmdResponse::Ok
            )]
        );
        assert_eq!(h.comp.catalog_size(), 3);
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_CATALOG_BUILD_COMPLETE)
        );

        h.start_xmit(Wait::NoWait, false);
        // Priority 1 @ t=100 first, then priority 1 @ t=200, then priority 5.
        let expected: Vec<String> = vec![2, 1, 3]
            .into_iter()
            .map(|id| {
                let secs = if id == 1 { 200 } else { 100 };
                format_dp_file_name(dir.to_str().unwrap(), id, secs, 0)
                    .0
                    .as_str()
                    .unwrap()
                    .to_string()
            })
            .collect();
        h.file_done(SendFileStatus::StatusOk);
        h.file_done(SendFileStatus::StatusOk);
        h.file_done(SendFileStatus::StatusOk);
        assert_eq!(h.requests(), expected);
        assert_eq!(h.comp.catalog_size(), 0);
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_CATALOG_XMIT_COMPLETED)
        );
    }

    #[test]
    fn non_dp_files_and_bad_products_are_skipped() {
        let h = Harness::new("skip", 1);
        let dir = h.dirs[0].clone();
        // A good product.
        write_dp_file(&dir, 1, 0, 1, 1, DpState::Untransmitted, &[]);
        // Not a .fdp file: ignored entirely.
        std::fs::write(dir.join("notes.txt"), b"hello").unwrap();
        // A .fdp file whose name does not match its header.
        let good = write_dp_file(&dir, 2, 0, 2, 2, DpState::Untransmitted, &[]);
        std::fs::rename(&good, dir.join("Dp_00000002_00000009_00000002.fdp")).unwrap();
        // A .fdp file that is too short.
        std::fs::write(dir.join("Dp_00000003_00000003_00000003.fdp"), [0u8; 10]).unwrap();
        // A .fdp file whose header hash is broken.
        let broken = write_dp_file(&dir, 4, 0, 4, 4, DpState::Untransmitted, &[]);
        let mut bytes = std::fs::read(&broken).unwrap();
        bytes[2] ^= 0xFF;
        std::fs::write(&broken, &bytes).unwrap();
        // An already-transmitted product.
        write_dp_file(&dir, 5, 0, 5, 5, DpState::Transmitted, &[]);

        h.build_catalog();
        assert_eq!(h.comp.catalog_size(), 1);
        let ids = h.event_ids();
        assert!(ids.contains(&DpCatalog::EVENTID_INVALID_FILE_NAME));
        assert!(ids.contains(&DpCatalog::EVENTID_FILE_READ_ERROR));
        assert!(ids.contains(&DpCatalog::EVENTID_FILE_HDR_ERROR));
        assert!(ids.contains(&DpCatalog::EVENTID_DP_FILE_SKIPPED));
        // FileHdrError reports (field=CRC, exp=COMPUTED, act=STORED) — the
        // C++ argument order, which is the reverse of the names.
        let args = h.events_with_id(DpCatalog::EVENTID_FILE_HDR_ERROR);
        assert_eq!(args.len(), 1);
        let field_offset = args[0].len() - 9;
        assert_eq!(args[0][field_offset], DpHdrField::Crc.as_repr());
        let computed = &args[0][field_offset + 1..field_offset + 5];
        let stored = &args[0][field_offset + 5..];
        assert_ne!(computed, stored);
        let mut container = DpContainer::new();
        let mut buffer = Buffer::allocate(DpContainer::MIN_PACKET_SIZE);
        buffer.data_mut().copy_from_slice(&bytes[..65]);
        container.set_buffer(buffer);
        let (_, stored_hash, computed_hash) = container.check_header_hash();
        assert_eq!(computed, computed_hash.to_be_bytes());
        assert_eq!(stored, stored_hash.to_be_bytes());
    }

    #[test]
    fn the_same_product_in_two_directories_is_a_duplicate() {
        let h = Harness::new("dup", 2);
        // Identical priority/time/id in both managed directories: the second
        // compares EQUAL (dir is excluded) and is skipped.
        write_dp_file(&h.dirs[0], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        write_dp_file(&h.dirs[1], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        h.build_catalog();
        assert_eq!(h.comp.catalog_size(), 1);
        assert_eq!(
            h.events_with_id(DpCatalog::EVENTID_DP_FILE_SKIPPED).len(),
            1
        );
    }

    #[test]
    fn a_full_catalog_stops_the_scan() {
        let dirs = vec![temp_dir("full")];
        let state_file = dirs[0].join("state.dat");
        let h = Harness::build(&dirs, &state_file, 2, true);
        for id in 1..=4u32 {
            write_dp_file(&dirs[0], id, id, id, 0, DpState::Untransmitted, &[]);
        }
        h.build_catalog();
        assert_eq!(h.comp.catalog_size(), 2);
        assert!(h.event_ids().contains(&DpCatalog::EVENTID_CATALOG_FULL));
    }

    #[test]
    fn an_unopenable_directory_aborts_the_whole_build() {
        let dirs = vec![PathBuf::from("/nonexistent-dp-catalog-dir")];
        let state_file = temp_dir("dirfail").join("state.dat");
        let h = Harness::build(&dirs, &state_file, DP_MAX_FILES as FwSizeType, true);
        h.build_catalog();
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_DIRECTORY_OPEN_ERROR)
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_BUILD_CATALOG,
                1,
                CmdResponse::ExecutionError
            )]
        );
    }

    // -- State file --------------------------------------------------------

    #[test]
    fn transmitting_appends_state_and_a_rebuild_merges_it() {
        let h = Harness::new("state", 1);
        let dir = h.dirs[0].clone();
        write_dp_file(&dir, 1, 0, 1, 1, DpState::Untransmitted, &[]);
        write_dp_file(&dir, 2, 1, 2, 2, DpState::Untransmitted, &[]);
        h.build_catalog();
        // A missing state file is a WARNING_LO NoStateFile, not an error.
        assert!(h.event_ids().contains(&DpCatalog::EVENTID_NO_STATE_FILE));
        h.start_xmit(Wait::NoWait, false);
        h.file_done(SendFileStatus::StatusOk);

        // One appended TRANSMITTED record for the first product.
        let bytes = std::fs::read(&h.state_file).unwrap();
        assert_eq!(bytes.len(), STATE_FILE_RECORD_SIZE);
        assert_eq!(&bytes[..2], &[0x00, 0x00]); // dir 0
        assert_eq!(&bytes[2..6], &1u32.to_be_bytes()); // id 1
        assert_eq!(
            bytes[STATE_FILE_RECORD_SIZE - 1],
            DpState::Transmitted.as_repr()
        );

        // Finish the second product so the transmit session completes.
        h.file_done(SendFileStatus::StatusOk);
        assert_eq!(
            std::fs::read(&h.state_file).unwrap().len(),
            2 * STATE_FILE_RECORD_SIZE
        );

        // A rebuild reads that state back and merges it into the entry.
        // C++ parity: the FILE header is never rewritten, so the product is
        // NOT skipped (only a header whose own DpState is TRANSMITTED is);
        // the state-file line is matched, marked visited, and survives the
        // prune.
        h.ground.events.lock().unwrap().clear();
        h.send_cmd(ID_BASE + DpCatalog::OPCODE_BUILD_CATALOG, 9, &[]);
        assert_eq!(h.comp.catalog_size(), 2);
        assert!(
            h.events_with_id(DpCatalog::EVENTID_DP_FILE_SKIPPED)
                .is_empty()
        );
        let bytes = std::fs::read(&h.state_file).unwrap();
        assert_eq!(bytes.len(), 2 * STATE_FILE_RECORD_SIZE);
        assert_eq!(
            bytes[STATE_FILE_RECORD_SIZE - 1],
            DpState::Transmitted.as_repr()
        );
        {
            let st = h.comp.state.lock().unwrap();
            let first = st.catalog.first().unwrap();
            assert_eq!(first.record.id, 1);
            assert_eq!(first.record.state, DpState::Transmitted);
        }
    }

    #[test]
    fn a_truncated_state_file_keeps_what_was_read() {
        let h = Harness::new("trunc", 1);
        write_dp_file(&h.dirs[0], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        std::fs::write(&h.state_file, [0u8; 5]).unwrap();
        h.build_catalog();
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_STATE_FILE_TRUNCATED)
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_BUILD_CATALOG,
                1,
                CmdResponse::Ok
            )]
        );
        assert_eq!(h.comp.catalog_size(), 1);
    }

    #[test]
    fn an_empty_state_file_name_is_reported_once() {
        let dirs = vec![temp_dir("nostate")];
        let h = Harness::build(&dirs, Path::new(""), DP_MAX_FILES as FwSizeType, true);
        h.build_catalog();
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_NO_STATE_FILE_SPECIFIED)
        );
    }

    // -- Transmit flow -----------------------------------------------------

    #[test]
    fn an_empty_catalog_answers_a_waiting_start_command_immediately() {
        let h = Harness::new("empty", 1);
        h.build_catalog();
        h.ground.responses.lock().unwrap().clear();
        h.start_xmit(Wait::Wait, false);
        // Completed inline inside doCatalogXmit; exactly one response.
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_START_XMIT_CATALOG,
                2,
                CmdResponse::Ok
            )]
        );
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_CATALOG_XMIT_COMPLETED)
        );
    }

    #[test]
    fn a_waiting_start_command_answers_when_the_last_product_completes() {
        let h = Harness::new("wait", 1);
        write_dp_file(&h.dirs[0], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        h.build_catalog();
        h.ground.responses.lock().unwrap().clear();
        h.start_xmit(Wait::Wait, false);
        assert!(h.responses().is_empty());
        h.file_done(SendFileStatus::StatusOk);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_START_XMIT_CATALOG,
                2,
                CmdResponse::Ok
            )]
        );
        // dispatchWaitedResponse disarms: no duplicate response later.
        h.send_cmd(ID_BASE + DpCatalog::OPCODE_STOP_XMIT_CATALOG, 5, &[]);
        assert_eq!(h.responses().len(), 2);
    }

    #[test]
    fn transmitting_without_a_built_catalog_is_rejected() {
        let h = Harness::new("unbuilt", 1);
        h.start_xmit(Wait::NoWait, false);
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_XMIT_UNBUILT_CATALOG)
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_START_XMIT_CATALOG,
                2,
                CmdResponse::ExecutionError
            )]
        );
    }

    #[test]
    fn a_rejected_send_aborts_the_transmit() {
        let h = Harness::new("reject", 1);
        write_dp_file(&h.dirs[0], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        h.build_catalog();
        *h.downlink.status.lock().unwrap() = Some(SendFileStatus::StatusBusy);
        h.ground.responses.lock().unwrap().clear();
        h.start_xmit(Wait::Wait, false);
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_DP_FILE_SEND_ERROR)
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_START_XMIT_CATALOG,
                2,
                CmdResponse::ExecutionError
            )]
        );
    }

    #[test]
    fn a_failed_downlink_halts_the_transmit() {
        let h = Harness::new("xmiterr", 1);
        write_dp_file(&h.dirs[0], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        write_dp_file(&h.dirs[0], 2, 1, 2, 2, DpState::Untransmitted, &[]);
        h.build_catalog();
        h.start_xmit(Wait::NoWait, false);
        h.file_done(SendFileStatus::StatusError);
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_DP_FILE_XMIT_ERROR)
        );
        // Only the first file was ever requested.
        assert_eq!(h.requests().len(), 1);
        assert_eq!(h.comp.catalog_size(), 2);
    }

    #[test]
    fn stop_is_benign_when_idle_and_breaks_the_chain_when_active() {
        let h = Harness::new("stop", 1);
        h.send_cmd(ID_BASE + DpCatalog::OPCODE_STOP_XMIT_CATALOG, 3, &[]);
        assert!(h.event_ids().contains(&DpCatalog::EVENTID_XMIT_NOT_ACTIVE));
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_STOP_XMIT_CATALOG,
                3,
                CmdResponse::Ok
            )]
        );

        write_dp_file(&h.dirs[0], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        write_dp_file(&h.dirs[0], 2, 1, 2, 2, DpState::Untransmitted, &[]);
        h.build_catalog();
        h.start_xmit(Wait::NoWait, false);
        h.ground.events.lock().unwrap().clear();
        h.send_cmd(ID_BASE + DpCatalog::OPCODE_STOP_XMIT_CATALOG, 4, &[]);
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_CATALOG_XMIT_STOPPED)
        );
        // The in-flight transfer is not cancelled: its fileDone still does
        // the accounting, but no further file is requested.
        h.file_done(SendFileStatus::StatusOk);
        assert_eq!(h.requests().len(), 1);
        assert_eq!(h.comp.catalog_size(), 1);
    }

    #[test]
    fn clear_catalog_leaves_a_later_file_done_to_unwind_the_transmit() {
        let h = Harness::new("clear", 1);
        write_dp_file(&h.dirs[0], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        h.build_catalog();
        h.ground.responses.lock().unwrap().clear();
        h.start_xmit(Wait::Wait, false);
        h.send_cmd(ID_BASE + DpCatalog::OPCODE_CLEAR_CATALOG, 6, &[]);
        assert_eq!(h.comp.catalog_size(), 0);
        // CLEAR_CATALOG does not touch the xmit flags; the in-flight
        // fileDone takes the "catalog not built" path and answers the
        // waiting START with EXECUTION_ERROR.
        h.file_done(SendFileStatus::StatusOk);
        assert_eq!(
            h.responses(),
            vec![
                (
                    ID_BASE + DpCatalog::OPCODE_CLEAR_CATALOG,
                    6,
                    CmdResponse::Ok
                ),
                (
                    ID_BASE + DpCatalog::OPCODE_START_XMIT_CATALOG,
                    2,
                    CmdResponse::ExecutionError
                ),
            ]
        );
    }

    #[test]
    fn building_while_transmitting_is_rejected() {
        let h = Harness::new("busy", 1);
        write_dp_file(&h.dirs[0], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        h.build_catalog();
        h.start_xmit(Wait::NoWait, false);
        h.ground.responses.lock().unwrap().clear();
        h.send_cmd(ID_BASE + DpCatalog::OPCODE_BUILD_CATALOG, 7, &[]);
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_DP_XMIT_IN_PROGRESS)
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_BUILD_CATALOG,
                7,
                CmdResponse::ExecutionError
            )]
        );
    }

    // -- addToCat ----------------------------------------------------------

    #[test]
    fn add_to_cat_needs_a_built_catalog_and_a_managed_directory() {
        let h = Harness::new("addcat", 1);
        let path = write_dp_file(&h.dirs[0], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        let port = h.comp.add_to_cat(0);

        // Catalog not built yet.
        port.target
            .invoke(port.port_num, &to_file_name(&path), 0, 65);
        h.drain();
        assert!(h.event_ids().contains(&DpCatalog::EVENTID_NOT_LOADED));

        h.build_catalog();
        h.ground.events.lock().unwrap().clear();

        // A file outside the managed directories.
        port.target.invoke(
            port.port_num,
            &to_file_name("/elsewhere/Dp_1_2_3.fdp"),
            0,
            65,
        );
        h.drain();
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_DIRECTORY_NOT_MANAGED)
        );
    }

    #[test]
    fn add_to_cat_resumes_the_transmit_only_when_remain_active() {
        let h = Harness::new("remain", 1);
        h.build_catalog();
        h.start_xmit(Wait::NoWait, true); // remainActive = true
        assert!(h.requests().is_empty());

        let path = write_dp_file(&h.dirs[0], 1, 0, 1, 1, DpState::Untransmitted, &[]);
        let port = h.comp.add_to_cat(0);
        // The priority and size arguments are ignored: everything is
        // re-read from the header.
        port.target
            .invoke(port.port_num, &to_file_name(&path), 0xDEAD, 0xBEEF);
        h.drain();
        assert_eq!(h.requests(), vec![path]);
    }

    // -- Configuration and commands ---------------------------------------

    #[test]
    fn an_unconfigured_component_reports_not_initialized() {
        let dirs = vec![temp_dir("noinit")];
        let state_file = dirs[0].join("state.dat");
        let h = Harness::build(&dirs, &state_file, DP_MAX_FILES as FwSizeType, false);
        h.build_catalog();
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_COMPONENT_NOT_INITIALIZED)
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_BUILD_CATALOG,
                1,
                CmdResponse::ExecutionError
            )]
        );
    }

    #[test]
    fn a_zero_slot_allocation_reports_no_memory() {
        let dirs = vec![temp_dir("nomem")];
        let state_file = dirs[0].join("state.dat");
        let h = Harness::build(&dirs, &state_file, 0, true);
        h.build_catalog();
        assert!(
            h.event_ids()
                .contains(&DpCatalog::EVENTID_COMPONENT_NO_MEMORY)
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + DpCatalog::OPCODE_BUILD_CATALOG,
                1,
                CmdResponse::ExecutionError
            )]
        );
    }

    #[test]
    fn command_argument_errors_are_reported_exactly_once() {
        let h = Harness::new("args", 1);
        // Invalid Fw.Wait value.
        h.send_cmd(
            ID_BASE + DpCatalog::OPCODE_START_XMIT_CATALOG,
            1,
            &[0x07, 0x00],
        );
        // Missing the bool argument.
        h.send_cmd(ID_BASE + DpCatalog::OPCODE_START_XMIT_CATALOG, 2, &[0x00]);
        // Residual bytes.
        h.send_cmd(ID_BASE + DpCatalog::OPCODE_CLEAR_CATALOG, 3, &[0x00]);
        // Unknown opcode.
        h.send_cmd(ID_BASE + 0x40, 4, &[]);
        assert_eq!(
            h.responses(),
            vec![
                (
                    ID_BASE + DpCatalog::OPCODE_START_XMIT_CATALOG,
                    1,
                    CmdResponse::ValidationError
                ),
                (
                    ID_BASE + DpCatalog::OPCODE_START_XMIT_CATALOG,
                    2,
                    CmdResponse::FormatError
                ),
                (
                    ID_BASE + DpCatalog::OPCODE_CLEAR_CATALOG,
                    3,
                    CmdResponse::FormatError
                ),
                (ID_BASE + 0x40, 4, CmdResponse::InvalidOpcode),
            ]
        );
    }

    #[test]
    fn ping_is_echoed() {
        let h = Harness::new("ping", 1);
        let port = h.comp.ping_in(0);
        port.target.invoke(port.port_num, 0x1234);
        h.drain();
        assert_eq!(*h.ground.pings.lock().unwrap(), vec![0x1234]);
    }

    #[test]
    fn the_add_to_cat_envelope_is_byte_exact() {
        let h = Harness::new("envelope", 1);
        let port = h.comp.add_to_cat(2);
        port.target
            .invoke(port.port_num, &to_file_name("ab"), 0x0102_0304, 0x11);
        let mut dest = [0u8; QUEUE_MSG_SIZE as usize];
        let mut size: FwSizeType = 0;
        let mut priority: FwQueuePriorityType = 0;
        let status = h.comp.active.queued.queue().receive(
            &mut dest,
            fprime_os::queue::BlockingType::NonBlocking,
            &mut size,
            &mut priority,
        );
        assert_eq!(status, fprime_os::queue::Status::OpOk);
        assert_eq!(
            &dest[..size as usize],
            &[
                0x00, 0x00, 0x00, 0x03, // msg_type = ADD_TO_CAT
                0x00, 0x02, // port_num
                0x00, 0x02, b'a', b'b', // fileName
                0x01, 0x02, 0x03, 0x04, // priority
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, // size
            ]
        );
    }
}
