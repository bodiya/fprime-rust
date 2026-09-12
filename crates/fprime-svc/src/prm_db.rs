//! # PrmDb — port of `Svc::PrmDbImpl` (ACTIVE)
//!
//! C++ sources: `Svc/PrmDb/PrmDbImpl.{cpp,hpp}`, `Svc/PrmDb/PrmDb.fpp`,
//! `Svc/PrmDb/PrmDbCmdDict.fppi`, `Svc/PrmDb/PrmDbEventDict.fppi`,
//! `default/config/PrmDbImplCfg.hpp`.
//! Analysis: `docs/cpp-analysis/utils-misc.md` (the `Svc::PrmDb` section,
//! the "PrmDb parameter file" wire format, and the PrmDb gotchas).
//!
//! The parameter database: a bounded, insertion-ordered map of
//! `FwPrmIdType -> Fw::ParamBuffer` that components read at
//! `loadParameters()` time through the guarded `getPrm` port, that ground
//! updates through the async `setPrm` port, and that is persisted to /
//! restored from a single file.
//!
//! Structure ported from C++:
//!
//! - **Double buffered stores.** Two [`PrmDbStore`]s (active + staging) of
//!   [`NUM_DB_ENTRIES`] entries each. C++ swaps two `PrmDbStore*` under the
//!   component lock; Rust swaps the two owned stores with
//!   [`std::mem::swap`] under the same lock — same observable behavior.
//!   Insertion order is load bearing: it is the record order of the saved
//!   file.
//! - **Three-state file-load machine** [`PrmDbFileLoadState`]
//!   (`Idle` / `LoadingFileUpdates` / `FileUpdatesStaged`) gating the three
//!   commands and `setPrm`.
//! - **Ports.** `getPrm` is GUARDED (runs on the caller's thread, takes the
//!   component mutex); `setPrm`, `pingIn` and all three commands are ASYNC
//!   (dispatched on the component thread).
//! - **`readParamFile()`** is the boot-time load: it runs *before* the task
//!   starts (topology "configure" phase), like the C++ method that
//!   documents "assumed to run at initialization time".
//!
//! Ported quirks (each covered by a unit test):
//!
//! - The file CRC is the **un-complemented** CRC-32 register
//!   (`!standard_crc32`), big-endian, covering every byte from offset 4 to
//!   EOF; save writes a `0xFFFFFFFF` placeholder, streams the records, then
//!   seeks back to offset 0 to overwrite it and restores the position.
//! - A load reads at most [`NUM_DB_ENTRIES`] records and silently ignores
//!   the rest; a clean EOF is a *successful* delimiter read of size 0 (a
//!   failed read that also yields 0 is an error, not EOF).
//! - Any dropped record (database full) fails the whole load with `Error`
//!   *after* processing the remaining records.
//! - `PRM_SAVE_FILE` / `PRM_LOAD_FILE` answer `BUSY` when the state is not
//!   `Idle`; `PRM_COMMIT_STAGED` answers `VALIDATION_ERROR` when the state
//!   is not `FileUpdatesStaged`; each also emits
//!   `PrmDbFileLoadInvalidAction`.
//! - `PRM_LOAD_FILE` with `MERGE` copies active -> staging first (emitting
//!   `PrmDbCopyAllComplete`); with `RESET` it clears staging.
//! - Saving opens the file `OPEN_WRITE` **without** truncation (C++
//!   `O_WRONLY | O_CREAT`), so saving a shorter image over a longer file
//!   leaves the tail bytes in place — and the next load fails its CRC
//!   check, because the CRC covers everything up to the real EOF.

use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, ComponentDispatch, EventGlue, EventThrottle,
    MsgDispatchStatus, OutputPort, PingPort, PrmGetPort, PrmSetPort, QueueFullPolicy,
    async_input_port_adapter, component_msg_types, input_port_adapter, msg,
};
use fprime_config::{
    FW_PARAM_BUFFER_MAX_SIZE, FwEnumStoreType, FwEventIdType, FwIdType, FwIndexType, FwOpcodeType,
    FwPrmIdType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, CmdStringArg, Endianness, FileNameString, FwDefaultString,
    LogSeverity, LogStringArg, ParamBuffer, ParamValid, SerBuf, SerBufAny, Serialize,
    SerializeStatus, fpp_enum, fw_assert,
};
use fprime_os::File;
use fprime_os::file::{Mode, OverwriteType, SeekType, Status as FileStatus, WaitType};
use fprime_os::file_path_utils::{PathStatus, check_containment, resolve_from_cwd};
use fprime_utils::Hash;
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Configuration (default/config/PrmDbImplCfg.hpp).
// ---------------------------------------------------------------------------

/// `PRMDB_NUM_DB_ENTRIES`: entries per database (active and staging each).
pub const NUM_DB_ENTRIES: usize = 25;

/// `PRMDB_ENTRY_DELIMITER`: byte preceding every record in the file.
pub const ENTRY_DELIMITER: u8 = 0xA5;

/// Smallest legal `recordSize` field: just the parameter id.
pub const MIN_RECORD_SIZE: u32 = 4;

/// Largest legal `recordSize` field: `FW_PARAM_BUFFER_MAX_SIZE` + the id.
pub const MAX_RECORD_SIZE: u32 = (FW_PARAM_BUFFER_MAX_SIZE + 4) as u32;

/// Queue message size: the max over all async invocations. The command
/// envelope is the largest: 6 (envelope) + 4 (opcode) + 4 (cmdSeq) +
/// 2 + 506 (length-prefixed `CmdArgBuffer`) = 522. (`setPrm` needs
/// 6 + 4 + 2 + 506 = 518; `pingIn` needs 10.)
pub const QUEUE_MSG_SIZE: usize = 522;

/// All async inputs share one queue priority (no FPP `priority`
/// qualifiers on this component), so dispatch is pure FIFO.
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

/// FPP default string size, the max length the autocoder passes when
/// serializing an unsized `string` event argument.
const EVENT_STRING_SIZE: usize = 80;

// ---------------------------------------------------------------------------
// FPP types (PrmDb.fpp / PrmDbCmdDict.fppi / PrmDbEventDict.fppi).
// ---------------------------------------------------------------------------

fpp_enum! {
    /// Which of the two databases an operation targets (`PrmDb.PrmDbType`).
    pub enum PrmDbType : u8 {
        /// The database `getPrm` reads.
        DbActive = 0,
        /// The database a commanded file load writes.
        DbStaging = 1,
    }
    default DbActive
}

fpp_enum! {
    /// State of parameter DB file load operations
    /// (`PrmDb.PrmDbFileLoadState`).
    pub enum PrmDbFileLoadState : u8 {
        /// No file load in progress; `setPrm` and every command are legal.
        Idle = 0,
        /// A `PRM_LOAD_FILE` is being processed.
        LoadingFileUpdates = 1,
        /// A file load succeeded; staging awaits `PRM_COMMIT_STAGED`.
        FileUpdatesStaged = 2,
    }
    default Idle
}

fpp_enum! {
    /// Parameter file read error stage (`PrmDb.PrmReadError`).
    pub enum PrmReadError : u8 {
        /// Opening the file failed.
        Open = 0,
        /// Reading the record delimiter failed.
        Delimiter = 1,
        /// The delimiter read returned the wrong number of bytes.
        DelimiterSize = 2,
        /// The delimiter byte was not [`ENTRY_DELIMITER`].
        DelimiterValue = 3,
        /// Reading the record size failed.
        RecordSize = 4,
        /// The record-size read returned the wrong number of bytes.
        RecordSizeSize = 5,
        /// The record size was out of range.
        RecordSizeValue = 6,
        /// Reading the parameter id failed.
        ParameterId = 7,
        /// The parameter-id read returned the wrong number of bytes.
        ParameterIdSize = 8,
        /// Reading the parameter value failed.
        ParameterValue = 9,
        /// The parameter-value read returned the wrong number of bytes.
        ParameterValueSize = 10,
        /// Reading the stored CRC failed.
        Crc = 11,
        /// The stored-CRC read returned the wrong number of bytes.
        CrcSize = 12,
        /// Computing the CRC over the file body failed.
        CrcBuffer = 13,
        /// Seeking back past the CRC failed.
        SeekZero = 14,
    }
    default Open
}

fpp_enum! {
    /// Parameter file write error stage (`PrmDb.PrmWriteError`).
    pub enum PrmWriteError : u8 {
        /// Opening the file failed.
        Open = 0,
        /// Writing the record delimiter failed.
        Delimiter = 1,
        /// The delimiter write reported the wrong number of bytes.
        DelimiterSize = 2,
        /// Writing the record size failed.
        RecordSize = 3,
        /// The record-size write reported the wrong number of bytes.
        RecordSizeSize = 4,
        /// Writing the parameter id failed.
        ParameterId = 5,
        /// The parameter-id write reported the wrong number of bytes.
        ParameterIdSize = 6,
        /// Writing the parameter value failed.
        ParameterValue = 7,
        /// The parameter-value write reported the wrong number of bytes.
        ParameterValueSize = 8,
        /// Writing the placeholder CRC failed.
        CrcPlace = 9,
        /// Writing the real CRC failed.
        CrcReal = 10,
        /// Reading back the current file position failed.
        CurrPosition = 11,
        /// Seeking to offset 0 for the real CRC failed.
        SeekZero = 12,
        /// Seeking back to the saved position failed.
        SeekPosition = 13,
    }
    default Open
}

fpp_enum! {
    /// `PRM_LOAD_FILE` merge behavior (`PrmDb.Merge`).
    pub enum Merge : u8 {
        /// Copy the active database into staging before loading the file.
        Merge = 0,
        /// Clear staging before loading the file.
        Reset = 1,
    }
    default Merge
}

fpp_enum! {
    /// Action rejected by the file-load state machine
    /// (`PrmDb.PrmLoadAction`).
    pub enum PrmLoadAction : u8 {
        /// A `setPrm` port invocation.
        SetParameter = 0,
        /// The `PRM_SAVE_FILE` command.
        SaveFileCommand = 1,
        /// The `PRM_LOAD_FILE` command.
        LoadFileCommand = 2,
        /// The `PRM_COMMIT_STAGED` command.
        CommitStagedCommand = 3,
    }
    default SetParameter
}

/// Result of an individual parameter update or add
/// (C++ `PrmDbImpl::PrmUpdateType`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrmUpdateType {
    /// No slots available to add a new parameter.
    NoSlots,
    /// The parameter was added to the database.
    ParamAdded,
    /// The parameter was already present and its value was replaced.
    ParamUpdated,
}

/// Result of a parameter file load (C++ `PrmDbImpl::PrmLoadStatus`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrmLoadStatus {
    /// The file was read and applied.
    Success,
    /// The file was rejected; the caller reports it.
    Error,
}

// ---------------------------------------------------------------------------
// The bounded, insertion-ordered store (C++ Fw::ArrayMap<..., 25>).
// ---------------------------------------------------------------------------

/// One parameter database: `Fw::ArrayMap<FwPrmIdType, Fw::ParamBuffer,
/// PRMDB_NUM_DB_ENTRIES>`.
///
/// Insertion ordered and bounded: `insert` replaces the value of an
/// existing key in place (keeping its position), otherwise appends while
/// there is room. Nothing is ever removed — the store is only cleared
/// wholesale — so the iteration order is exactly the order the C++ ArrayMap
/// saves records in.
/// The backing `Vec` is allocated once at construction with the full
/// capacity and never grows.
#[derive(Debug)]
pub struct PrmDbStore {
    entries: Vec<(FwPrmIdType, ParamBuffer)>,
}

impl PrmDbStore {
    /// Allocate an empty store with room for [`NUM_DB_ENTRIES`] entries.
    fn new() -> Self {
        Self {
            entries: Vec::with_capacity(NUM_DB_ENTRIES),
        }
    }

    /// Look up a parameter value (C++ `ArrayMap::find`).
    fn find(&self, id: FwPrmIdType) -> Option<&ParamBuffer> {
        self.entries
            .iter()
            .find(|(key, _)| *key == id)
            .map(|(_, value)| value)
    }

    /// Insert or overwrite. Returns `false` when the store is full and the
    /// key is new (C++ `Fw::Success::FAILURE`).
    fn insert(&mut self, id: FwPrmIdType, value: &ParamBuffer) -> bool {
        for (key, slot) in self.entries.iter_mut() {
            if *key == id {
                slot.clone_from(value);
                return true;
            }
        }
        if self.entries.len() < NUM_DB_ENTRIES {
            self.entries.push((id, value.clone()));
            true
        } else {
            false
        }
    }

    /// Drop every entry, keeping the allocation.
    fn clear(&mut self) {
        self.entries.clear();
    }

    /// Deep copy (C++ `PrmDbStore` copy assignment in `dbCopy`).
    fn copy_from(&mut self, src: &PrmDbStore) {
        self.entries.clear();
        self.entries.extend(src.entries.iter().cloned());
    }

    /// Number of stored parameters.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// True when no parameters are stored.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Iterate `(id, value)` in insertion (file save) order.
    pub fn iter(&self) -> impl Iterator<Item = (FwPrmIdType, &ParamBuffer)> {
        self.entries.iter().map(|(id, value)| (*id, value))
    }
}

// ---------------------------------------------------------------------------
// Component state.
// ---------------------------------------------------------------------------

/// Mutable component state behind the component mutex. This one mutex is
/// both the C++ guarded-port mutex (`getPrm`) and the explicit
/// `lock()`/`unLock()` bracket the C++ implementation puts around every DB
/// read, mutation and the active/staging swap.
struct PrmDbState {
    /// The database `getPrm` reads; the one `PRM_SAVE_FILE` writes.
    active: PrmDbStore,
    /// The database a commanded load fills, promoted by `PRM_COMMIT_STAGED`.
    staging: PrmDbStore,
    /// File-load state machine (`m_state`).
    load_state: PrmDbFileLoadState,
    /// Parameter file name (`m_fileName`, set by [`PrmDb::configure`]).
    file_name: FwDefaultString,
    /// Sandbox directory for commanded loads (`m_sandboxDir`); empty means
    /// unrestricted, matching the C++ legacy behavior.
    sandbox_dir: FwDefaultString,
}

impl PrmDbState {
    fn new() -> Self {
        Self {
            active: PrmDbStore::new(),
            staging: PrmDbStore::new(),
            load_state: PrmDbFileLoadState::Idle,
            file_name: FwDefaultString::new(),
            sandbox_dir: FwDefaultString::new(),
        }
    }

    /// C++ `getDbPtr`.
    fn db_mut(&mut self, db_type: PrmDbType) -> &mut PrmDbStore {
        match db_type {
            PrmDbType::DbActive => &mut self.active,
            PrmDbType::DbStaging => &mut self.staging,
        }
    }

    /// C++ `dbCopy` (without the event, which the caller emits outside the
    /// lock).
    fn copy_db(&mut self, dest: PrmDbType, src: PrmDbType) {
        if dest == src {
            return;
        }
        let Self {
            active, staging, ..
        } = self;
        match dest {
            PrmDbType::DbActive => active.copy_from(staging),
            PrmDbType::DbStaging => staging.copy_from(active),
        }
    }
}

/// C++ `getDbString`.
const fn db_string(db_type: PrmDbType) -> &'static str {
    match db_type {
        PrmDbType::DbActive => "ACTIVE",
        PrmDbType::DbStaging => "STAGING",
    }
}

// ---------------------------------------------------------------------------
// The component.
// ---------------------------------------------------------------------------

/// `Svc::PrmDbImpl` — the active parameter database component.
pub struct PrmDb {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Command registration/response glue (`CmdReg`/`CmdStatus`).
    pub cmd: CmdGlue,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// `pingOut`: `Svc.Ping` out.
    pub ping_out: OutputPort<dyn PingPort>,
    /// FPP `throttle 5` on `PrmIdNotFound`.
    not_found_throttle: EventThrottle,
    state: Mutex<PrmDbState>,
}

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel).
    impl PrmDb {
        /// `setPrm` async input port.
        MSG_TYPE_SET_PRM,
        /// `pingIn` async input port.
        MSG_TYPE_PING_IN,
        /// `CmdDisp` async command input port.
        MSG_TYPE_CMD,
    }
}

impl PrmDb {
    /// `PRM_SAVE_FILE` opcode (component-relative).
    pub const OPCODE_PRM_SAVE_FILE: FwOpcodeType = 0x00;
    /// `PRM_LOAD_FILE(fileName, merge)` opcode.
    pub const OPCODE_PRM_LOAD_FILE: FwOpcodeType = 0x01;
    /// `PRM_COMMIT_STAGED` opcode.
    pub const OPCODE_PRM_COMMIT_STAGED: FwOpcodeType = 0x02;

    /// `PrmIdNotFound(Id)` (WARNING_LO, throttle 5).
    pub const EVENTID_PRM_ID_NOT_FOUND: FwEventIdType = 0;
    /// `PrmIdUpdated(Id)` (ACTIVITY_HI).
    pub const EVENTID_PRM_ID_UPDATED: FwEventIdType = 1;
    /// `PrmDbFull(Id)` (WARNING_HI).
    pub const EVENTID_PRM_DB_FULL: FwEventIdType = 2;
    /// `PrmIdAdded(Id)` (ACTIVITY_HI).
    pub const EVENTID_PRM_ID_ADDED: FwEventIdType = 3;
    /// `PrmFileWriteError(stage, record, error)` (WARNING_HI).
    pub const EVENTID_PRM_FILE_WRITE_ERROR: FwEventIdType = 4;
    /// `PrmFileSaveComplete(records)` (ACTIVITY_HI).
    pub const EVENTID_PRM_FILE_SAVE_COMPLETE: FwEventIdType = 5;
    /// `PrmFileReadError(stage, record, error)` (WARNING_HI).
    pub const EVENTID_PRM_FILE_READ_ERROR: FwEventIdType = 6;
    /// `PrmFileLoadComplete(db, total, added, updated)` (ACTIVITY_HI).
    pub const EVENTID_PRM_FILE_LOAD_COMPLETE: FwEventIdType = 7;
    /// `PrmDbCommitComplete()` (ACTIVITY_HI).
    pub const EVENTID_PRM_DB_COMMIT_COMPLETE: FwEventIdType = 8;
    /// `PrmDbCopyAllComplete(src, dest)` (ACTIVITY_HI).
    pub const EVENTID_PRM_DB_COPY_ALL_COMPLETE: FwEventIdType = 9;
    /// `PrmDbFileLoadFailed()` (WARNING_HI).
    pub const EVENTID_PRM_DB_FILE_LOAD_FAILED: FwEventIdType = 10;
    /// `PrmDbFileLoadInvalidAction(currentState, attemptedAction)`
    /// (WARNING_LO).
    pub const EVENTID_PRM_DB_FILE_LOAD_INVALID_ACTION: FwEventIdType = 11;
    /// `PrmFileBadCrc(readCrc, compCrc)` (WARNING_HI).
    pub const EVENTID_PRM_FILE_BAD_CRC: FwEventIdType = 12;

    /// FPP `throttle 5` on `PrmIdNotFound`.
    pub const PRM_ID_NOT_FOUND_THROTTLE: u32 = 5;

    /// Construct (topology phase 1). Both databases start empty, like the
    /// C++ constructor. Follow with `set_id_base`, wiring, [`Self::init`],
    /// [`Self::configure`], [`Self::read_param_file`],
    /// [`Self::reg_commands`], and `active.start`.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            ping_out: OutputPort::new(),
            not_found_throttle: EventThrottle::new(Self::PRM_ID_NOT_FOUND_THROTTLE),
            state: Mutex::new(PrmDbState::new()),
        })
    }

    /// Create the message queue (topology phase; C++ `init(queueDepth)`).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    /// C++ `configure(file)`: store the parameter file name used by
    /// `PRM_SAVE_FILE` and [`Self::read_param_file`].
    pub fn configure(&self, file: &str) {
        // C++ FW_ASSERT(file != nullptr); an empty name is caught by the
        // length assert in the save/load paths.
        self.state.lock().unwrap().file_name.set(file);
    }

    /// C++ `configureLoadSandbox(directory)`: restrict the directory a
    /// ground-commanded `PRM_LOAD_FILE` may read from. Never calling this
    /// leaves commanded loads unrestricted (C++ legacy behavior).
    pub fn configure_load_sandbox(&self, directory: &str) {
        // C++ FW_ASSERT(resolveStatus == VALID): an unresolvable directory
        // is a programmer error. Should an assert hook let execution
        // continue, the sandbox is left unchanged rather than widened.
        let mut resolved = FileNameString::new();
        if resolve_from_cwd(directory, &mut resolved) != PathStatus::Valid {
            fw_assert!(false);
            return;
        }
        if !resolved.as_bytes().ends_with(b"/") {
            resolved.append("/");
        }
        self.state
            .lock()
            .unwrap()
            .sandbox_dir
            .set(resolved.as_str().unwrap_or_default());
    }

    /// C++ `readParamFile()`: the boot-time load of the configured file
    /// into the ACTIVE database.
    ///
    /// Assumed to run at initialization time, before the component task is
    /// started — no other thread may touch the databases while it runs
    /// (C++ makes the same assumption and asserts the state is `Idle`).
    pub fn read_param_file(&self) {
        let file_name = {
            let state = &mut *self.state.lock().unwrap();
            // C++ FW_ASSERT(m_state == IDLE).
            fw_assert!(state.load_state == PrmDbFileLoadState::Idle);
            state.active.clear();
            state.staging.clear();
            state.file_name.clone()
        };
        let _ =
            self.read_param_file_impl(file_name.as_str().unwrap_or_default(), PrmDbType::DbActive);
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_PRM_SAVE_FILE,
                Self::OPCODE_PRM_LOAD_FILE,
                Self::OPCODE_PRM_COMMIT_STAGED,
            ],
        );
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    fn load_state(&self) -> PrmDbFileLoadState {
        self.state.lock().unwrap().load_state
    }

    // -- Handlers ----------------------------------------------------------

    /// `getPrm_handler` — GUARDED: runs on the CALLER's thread under the
    /// component mutex. It ALWAYS searches the active database.
    ///
    /// Deviation from C++, deliberate: the C++ generated guarded port holds
    /// the component mutex across the whole handler, event included; here
    /// the lock is dropped before the event is emitted, per the workspace
    /// convention that output ports are invoked outside the state lock. The
    /// observable behavior is identical (nothing else can enter the miss
    /// path for the same id and change the outcome).
    fn get_prm_handler(
        &self,
        _port_num: FwIndexType,
        id: FwPrmIdType,
        val: &mut ParamBuffer,
    ) -> ParamValid {
        let found = self.state.lock().unwrap().active.find(id).cloned();
        match found {
            Some(value) => {
                // C++ ArrayMap::find copies the stored buffer into `val`
                // (cursors included) and leaves it untouched on a miss.
                *val = value;
                ParamValid::Valid
            }
            None => {
                if self.not_found_throttle.ok_to_emit() {
                    self.evt.log_event(
                        self.id_base(),
                        Self::EVENTID_PRM_ID_NOT_FOUND,
                        LogSeverity::WarningLo,
                        &format!("Parameter ID 0x{id:x} not found"),
                        |buf| buf.serialize_u32_be(id),
                    );
                }
                ParamValid::Invalid
            }
        }
    }

    /// `setPrm_handler` — component thread. Updates the ACTIVE database.
    fn set_prm_handler(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) {
        // Reject parameter updates during non-idle file load states.
        let load_state = self.load_state();
        if load_state != PrmDbFileLoadState::Idle {
            self.log_file_load_invalid_action(load_state, PrmLoadAction::SetParameter);
            return;
        }

        match self.update_add_prm(id, val, PrmDbType::DbActive) {
            PrmUpdateType::ParamUpdated => self.evt.log_event(
                self.id_base(),
                Self::EVENTID_PRM_ID_UPDATED,
                LogSeverity::ActivityHi,
                &format!("Parameter ID 0x{id:x} updated"),
                |buf| buf.serialize_u32_be(id),
            ),
            PrmUpdateType::NoSlots => self.log_prm_db_full(id),
            PrmUpdateType::ParamAdded => self.evt.log_event(
                self.id_base(),
                Self::EVENTID_PRM_ID_ADDED,
                LogSeverity::ActivityHi,
                &format!("Parameter ID 0x{id:x} added"),
                |buf| buf.serialize_u32_be(id),
            ),
        }
    }

    /// `pingIn_handler` — component thread.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }

    /// C++ `updateAddPrmImpl`: the underlying add-or-update, bracketed by
    /// the component lock.
    fn update_add_prm(
        &self,
        id: FwPrmIdType,
        val: &ParamBuffer,
        db_type: PrmDbType,
    ) -> PrmUpdateType {
        let state = &mut *self.state.lock().unwrap();
        let db = state.db_mut(db_type);
        let prev_size = db.len();
        if !db.insert(id, val) {
            return PrmUpdateType::NoSlots;
        }
        if prev_size < db.len() {
            PrmUpdateType::ParamAdded
        } else {
            PrmUpdateType::ParamUpdated
        }
    }

    // -- Commands (component thread) ---------------------------------------

    /// `PRM_SAVE_FILE`: write the ACTIVE database to the configured file.
    fn prm_save_file_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        let load_state = self.load_state();
        if load_state != PrmDbFileLoadState::Idle {
            self.log_file_load_invalid_action(load_state, PrmLoadAction::SaveFileCommand);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Busy);
            return;
        }

        let file_name = self.state.lock().unwrap().file_name.clone();
        let path = file_name.as_str().unwrap_or_default();
        // C++ FW_ASSERT(this->m_fileName.length() > 0).
        fw_assert!(!path.is_empty());

        let mut param_file = File::new();
        // C++ parity: OPEN_WRITE is O_WRONLY|O_CREAT — it does NOT
        // truncate. A shorter image saved over a longer file leaves the
        // tail bytes behind (and the next load then fails its CRC check).
        let stat = param_file.open_with_overwrite(path, Mode::OpenWrite, OverwriteType::Overwrite);
        if stat != FileStatus::OpOk {
            self.fail_save(op_code, cmd_seq, PrmWriteError::Open, 0, stat as i32);
            return;
        }

        // Placeholder for the CRC, overwritten at the end. C++ checks only
        // the status here, not the written size.
        let placeholder = 0xFFFF_FFFFu32.to_be_bytes();
        let mut write_size: FwSizeType = 0;
        let stat = param_file.write(&placeholder, &mut write_size, WaitType::Wait);
        if stat != FileStatus::OpOk {
            self.fail_save(op_code, cmd_seq, PrmWriteError::CrcPlace, 0, stat as i32);
            return;
        }

        let mut hash = Hash::new();
        let num_records = match self.write_records(&mut param_file, &mut hash) {
            Ok(num_records) => num_records,
            Err((stage, record, error)) => {
                self.fail_save(op_code, cmd_seq, stage, record, error);
                return;
            }
        };

        // Save the current position so the file is left where the record
        // stream ended.
        let mut curr_pos: FwSizeType = 0;
        let stat = param_file.position(&mut curr_pos);
        if stat != FileStatus::OpOk {
            self.fail_save(
                op_code,
                cmd_seq,
                PrmWriteError::CurrPosition,
                0,
                stat as i32,
            );
            return;
        }
        let stat = param_file.seek(0, SeekType::Absolute);
        if stat != FileStatus::OpOk {
            self.fail_save(op_code, cmd_seq, PrmWriteError::SeekZero, 0, stat as i32);
            return;
        }
        // C++ `crc.finalize(crcFinal); crcFinal = ~crcFinal;` — the file
        // stores the UN-complemented CRC-32 register, and Hash::finalize
        // returns the complemented (standard) value.
        let crc_final = !hash.finalize();
        let stat = param_file.write(&crc_final.to_be_bytes(), &mut write_size, WaitType::Wait);
        if stat != FileStatus::OpOk {
            self.fail_save(op_code, cmd_seq, PrmWriteError::CrcReal, 0, stat as i32);
            return;
        }
        let stat = param_file.seek(curr_pos as i64, SeekType::Absolute);
        if stat != FileStatus::OpOk {
            self.fail_save(
                op_code,
                cmd_seq,
                PrmWriteError::SeekPosition,
                0,
                stat as i32,
            );
            return;
        }

        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_PRM_FILE_SAVE_COMPLETE,
            LogSeverity::ActivityHi,
            &format!("Parameter file save completed. Wrote {num_records} records."),
            |buf| buf.serialize_u32_be(num_records),
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// Stream every ACTIVE record into `param_file`, accumulating `hash`.
    ///
    /// The component lock is held for the whole traversal, exactly as the
    /// C++ `lock()` / `unLock()` bracket around the save loop; every event
    /// is emitted by the caller after the lock is released.
    #[allow(clippy::type_complexity)]
    fn write_records(
        &self,
        param_file: &mut File,
        hash: &mut Hash,
    ) -> Result<u32, (PrmWriteError, i32, i32)> {
        let state = self.state.lock().unwrap();
        let mut num_records: u32 = 0;
        for (id, value) in state.active.iter() {
            let record = num_records as i32;

            // Delimiter.
            let delim = [ENTRY_DELIMITER];
            write_field(
                param_file,
                &delim,
                record,
                PrmWriteError::Delimiter,
                PrmWriteError::DelimiterSize,
            )?;
            hash.update(&delim);

            // Record size = id field + value. F Prime U32 serialization is
            // big endian (locked down by the literal-byte tests).
            let record_size = (4 + value.get_size()) as u32;
            let record_size_bytes = record_size.to_be_bytes();
            write_field(
                param_file,
                &record_size_bytes,
                record,
                PrmWriteError::RecordSize,
                PrmWriteError::RecordSizeSize,
            )?;
            hash.update(&record_size_bytes);

            // Parameter id.
            let id_bytes = id.to_be_bytes();
            write_field(
                param_file,
                &id_bytes,
                record,
                PrmWriteError::ParameterId,
                PrmWriteError::ParameterIdSize,
            )?;
            hash.update(&id_bytes);

            // Serialized parameter value (opaque bytes).
            let value_bytes = value.as_slice();
            write_field(
                param_file,
                value_bytes,
                record,
                PrmWriteError::ParameterValue,
                PrmWriteError::ParameterValueSize,
            )?;
            hash.update(value_bytes);

            num_records += 1;
        }
        Ok(num_records)
    }

    /// `PRM_LOAD_FILE(fileName, merge)`: load a file into STAGING.
    fn prm_load_file_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        file_name: &CmdStringArg,
        merge: Merge,
    ) {
        let load_state = self.load_state();
        if load_state != PrmDbFileLoadState::Idle {
            self.log_file_load_invalid_action(load_state, PrmLoadAction::LoadFileCommand);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Busy);
            return;
        }

        self.state.lock().unwrap().load_state = PrmDbFileLoadState::LoadingFileUpdates;

        if merge == Merge::Merge {
            // Copy active into staging so the file merges into the current
            // parameter set.
            self.db_copy(PrmDbType::DbStaging, PrmDbType::DbActive);
        } else {
            self.state.lock().unwrap().staging.clear();
        }

        // readParamFileImpl emits the per-stage EVRs on failure and the
        // completion EVR on success.
        let status =
            self.read_param_file_impl(file_name.as_str().unwrap_or_default(), PrmDbType::DbStaging);

        if status == PrmLoadStatus::Success {
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            self.state.lock().unwrap().load_state = PrmDbFileLoadState::FileUpdatesStaged;
        } else {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_PRM_DB_FILE_LOAD_FAILED,
                LogSeverity::WarningHi,
                "Parameter file load failed. Clearing staging database and abandoning parameter file load.",
                |_buf| SerializeStatus::Ok,
            );
            {
                let state = &mut *self.state.lock().unwrap();
                state.staging.clear();
                state.load_state = PrmDbFileLoadState::Idle;
            }
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
        }
    }

    /// `PRM_COMMIT_STAGED`: promote STAGING to ACTIVE.
    fn prm_commit_staged_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        let load_state = self.load_state();
        if load_state != PrmDbFileLoadState::FileUpdatesStaged {
            self.log_file_load_invalid_action(load_state, PrmLoadAction::CommitStagedCommand);
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }

        {
            // The swap happens under the component lock so the guarded
            // getPrm never observes a half-swapped pair (C++ swaps the two
            // pointers inside lock()/unLock()).
            let state = &mut *self.state.lock().unwrap();
            std::mem::swap(&mut state.active, &mut state.staging);
            state.staging.clear();
            state.load_state = PrmDbFileLoadState::Idle;
        }

        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_PRM_DB_COMMIT_COMPLETE,
            LogSeverity::ActivityHi,
            "Parameter DB commit complete, staged updates are now active.",
            |_buf| SerializeStatus::Ok,
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    // -- File load ---------------------------------------------------------

    /// C++ `readParamFileImpl`: pick the file object (sandboxed for
    /// commanded/staging loads when a sandbox directory is configured) and
    /// run the read.
    fn read_param_file_impl(&self, file_name: &str, db_type: PrmDbType) -> PrmLoadStatus {
        // C++ FW_ASSERT(fileName.length() > 0).
        fw_assert!(!file_name.is_empty());

        let sandbox_dir = self.state.lock().unwrap().sandbox_dir.clone();
        let sandbox_dir = sandbox_dir.as_str().unwrap_or_default();

        let mut param_file = File::new();
        // Commanded loads (staging) may carry a ground-supplied path;
        // restrict them to the configured sandbox directory to prevent path
        // traversal (the C++ `Os::SandboxedFile` check, using the
        // `fprime-os` `FilePathUtils` port).
        let open_status = if db_type == PrmDbType::DbStaging && !sandbox_dir.is_empty() {
            let mut resolved = FileNameString::new();
            let contained = resolve_from_cwd(file_name, &mut resolved) == PathStatus::Valid
                && resolved
                    .as_str()
                    .is_some_and(|r| check_containment(r, sandbox_dir) == PathStatus::Valid);
            if contained {
                param_file.open(resolved.as_str().unwrap_or_default(), Mode::OpenRead)
            } else {
                FileStatus::OutsideSandbox
            }
        } else {
            param_file.open(file_name, Mode::OpenRead)
        };

        self.read_param_file_work(&mut param_file, open_status, db_type)
    }

    /// C++ `readParamFileWork`: validate the CRC, then stream records into
    /// `db_type`.
    fn read_param_file_work(
        &self,
        param_file: &mut File,
        open_status: FileStatus,
        db_type: PrmDbType,
    ) -> PrmLoadStatus {
        if open_status != FileStatus::OpOk {
            self.log_read_error(PrmReadError::Open, 0, open_status as i32);
            return PrmLoadStatus::Error;
        }

        // === CRC =============================================================
        let mut crc_bytes = [0u8; 4];
        let mut read_size: FwSizeType = 0;
        let stat = param_file.read(&mut crc_bytes, &mut read_size, WaitType::Wait);
        if stat != FileStatus::OpOk {
            self.log_read_error(PrmReadError::Crc, 0, stat as i32);
            return PrmLoadStatus::Error;
        }
        if read_size != 4 {
            self.log_read_error(PrmReadError::CrcSize, 0, read_size as i32);
            return PrmLoadStatus::Error;
        }
        let file_crc = u32::from_be_bytes(crc_bytes);

        // calculateCrc runs from the CURRENT position (offset 4) to EOF and
        // returns the un-complemented register — the same form the file
        // stores.
        let mut crc: u32 = 0;
        let stat = param_file.calculate_crc(&mut crc);
        if stat != FileStatus::OpOk {
            self.log_read_error(PrmReadError::CrcBuffer, 0, stat as i32);
            return PrmLoadStatus::Error;
        }
        if file_crc != crc {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_PRM_FILE_BAD_CRC,
                LogSeverity::WarningHi,
                &format!("Parameter file failed CRC. Read: 0x{file_crc:x} Computed: 0x{crc:x}"),
                |buf| {
                    let status = buf.serialize_u32_be(file_crc);
                    if !status.is_ok() {
                        return status;
                    }
                    buf.serialize_u32_be(crc)
                },
            );
            return PrmLoadStatus::Error;
        }

        // Seek back to just after the CRC.
        let stat = param_file.seek(4, SeekType::Absolute);
        if stat != FileStatus::OpOk {
            self.log_read_error(PrmReadError::SeekZero, 0, stat as i32);
            return PrmLoadStatus::Error;
        }

        // === Records =========================================================
        let mut record_num_total: u32 = 0;
        let mut record_num_added: u32 = 0;
        let mut record_num_updated: u32 = 0;
        let mut record_num_dropped: u32 = 0;
        let mut value_bytes = [0u8; FW_PARAM_BUFFER_MAX_SIZE];

        // At most NUM_DB_ENTRIES records are read; any further records in
        // the file are silently ignored (C++ parity).
        for _entry in 0..NUM_DB_ENTRIES {
            let record = record_num_total as i32;

            // Delimiter. A SUCCESSFUL read of size 0 is a clean EOF; a
            // failed read that also yields 0 must not be mistaken for one.
            let mut delimiter = [0u8; 1];
            let stat = param_file.read(&mut delimiter, &mut read_size, WaitType::Wait);
            if stat == FileStatus::OpOk && read_size == 0 {
                break;
            }
            if stat != FileStatus::OpOk {
                self.log_read_error(PrmReadError::Delimiter, record, stat as i32);
                return PrmLoadStatus::Error;
            }
            if read_size != 1 {
                self.log_read_error(PrmReadError::DelimiterSize, record, read_size as i32);
                return PrmLoadStatus::Error;
            }
            if delimiter[0] != ENTRY_DELIMITER {
                self.log_read_error(PrmReadError::DelimiterValue, record, delimiter[0] as i32);
                return PrmLoadStatus::Error;
            }

            // Record size.
            let mut record_size_bytes = [0u8; 4];
            let stat = param_file.read(&mut record_size_bytes, &mut read_size, WaitType::Wait);
            if stat != FileStatus::OpOk {
                self.log_read_error(PrmReadError::RecordSize, record, stat as i32);
                return PrmLoadStatus::Error;
            }
            if read_size != 4 {
                self.log_read_error(PrmReadError::RecordSizeSize, record, read_size as i32);
                return PrmLoadStatus::Error;
            }
            let record_size = u32::from_be_bytes(record_size_bytes);
            // Sanity check: no larger than a parameter buffer plus the id,
            // no smaller than the id alone.
            if !(MIN_RECORD_SIZE..=MAX_RECORD_SIZE).contains(&record_size) {
                self.log_read_error(PrmReadError::RecordSizeValue, record, record_size as i32);
                return PrmLoadStatus::Error;
            }

            // Parameter id.
            let mut id_bytes = [0u8; 4];
            let stat = param_file.read(&mut id_bytes, &mut read_size, WaitType::Wait);
            if stat != FileStatus::OpOk {
                self.log_read_error(PrmReadError::ParameterId, record, stat as i32);
                return PrmLoadStatus::Error;
            }
            if read_size != 4 {
                self.log_read_error(PrmReadError::ParameterIdSize, record, read_size as i32);
                return PrmLoadStatus::Error;
            }
            let parameter_id: FwPrmIdType = u32::from_be_bytes(id_bytes);

            // Parameter value.
            let value_len = (record_size - 4) as usize;
            let stat = param_file.read(
                &mut value_bytes[..value_len],
                &mut read_size,
                WaitType::Wait,
            );
            if stat != FileStatus::OpOk {
                self.log_read_error(PrmReadError::ParameterValue, record, stat as i32);
                return PrmLoadStatus::Error;
            }
            if read_size as usize != value_len {
                self.log_read_error(PrmReadError::ParameterValueSize, record, read_size as i32);
                return PrmLoadStatus::Error;
            }
            let mut tmp_param_buffer = ParamBuffer::new();
            let status = tmp_param_buffer.set_buff(&value_bytes[..value_len]);
            // The length was range-checked against FW_PARAM_BUFFER_MAX_SIZE
            // above, so this cannot fail.
            fw_assert!(status.is_ok(), status as i32);

            match self.update_add_prm(parameter_id, &tmp_param_buffer, db_type) {
                PrmUpdateType::ParamAdded => record_num_added += 1,
                PrmUpdateType::ParamUpdated => record_num_updated += 1,
                PrmUpdateType::NoSlots => {
                    self.log_prm_db_full(parameter_id);
                    record_num_dropped += 1;
                }
            }
            record_num_total += 1;
        }

        // Dropped records mean the database does not reflect the file.
        if record_num_dropped > 0 {
            return PrmLoadStatus::Error;
        }

        let db_string = db_string(db_type);
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_PRM_FILE_LOAD_COMPLETE,
            LogSeverity::ActivityHi,
            &format!(
                "Parameter file load completed. Database: {db_string}, Records: {record_num_total} ({record_num_added} added and {record_num_updated} updated)."
            ),
            |buf| {
                let name = LogStringArg::from(db_string);
                let status = name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big);
                if !status.is_ok() {
                    return status;
                }
                let status = buf.serialize_u32_be(record_num_total);
                if !status.is_ok() {
                    return status;
                }
                let status = buf.serialize_u32_be(record_num_added);
                if !status.is_ok() {
                    return status;
                }
                buf.serialize_u32_be(record_num_updated)
            },
        );
        PrmLoadStatus::Success
    }

    /// C++ `dbCopy`: deep copy one database into the other and report it.
    fn db_copy(&self, dest: PrmDbType, src: PrmDbType) {
        self.state.lock().unwrap().copy_db(dest, src);
        let src_string = db_string(src);
        let dest_string = db_string(dest);
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_PRM_DB_COPY_ALL_COMPLETE,
            LogSeverity::ActivityHi,
            &format!(
                "All parameters copied. Source database: {src_string}, Destination database: {dest_string}."
            ),
            |buf| {
                let src_arg = LogStringArg::from(src_string);
                let dest_arg = LogStringArg::from(dest_string);
                let status =
                    src_arg.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big);
                if !status.is_ok() {
                    return status;
                }
                dest_arg.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    // -- Event helpers -----------------------------------------------------

    fn log_prm_db_full(&self, id: FwPrmIdType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_PRM_DB_FULL,
            LogSeverity::WarningHi,
            &format!("Parameter DB full when adding ID 0x{id:x} "),
            |buf| buf.serialize_u32_be(id),
        );
    }

    fn log_file_load_invalid_action(
        &self,
        current_state: PrmDbFileLoadState,
        attempted_action: PrmLoadAction,
    ) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_PRM_DB_FILE_LOAD_INVALID_ACTION,
            LogSeverity::WarningLo,
            &format!(
                "Invalid action during parameter file load. Current state: {current_state:?}, Action (Invalid for current state): {attempted_action:?}."
            ),
            |buf| {
                let status = current_state.serialize_to(buf, Endianness::Big);
                if !status.is_ok() {
                    return status;
                }
                attempted_action.serialize_to(buf, Endianness::Big)
            },
        );
    }

    fn log_read_error(&self, stage: PrmReadError, record: i32, error: i32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_PRM_FILE_READ_ERROR,
            LogSeverity::WarningHi,
            &format!(
                "Parameter file read failed in stage {stage:?} with record {record} and error {error}"
            ),
            |buf| {
                let status = stage.serialize_to(buf, Endianness::Big);
                if !status.is_ok() {
                    return status;
                }
                let status = buf.serialize_i32_be(record);
                if !status.is_ok() {
                    return status;
                }
                buf.serialize_i32_be(error)
            },
        );
    }

    fn log_write_error(&self, stage: PrmWriteError, record: i32, error: i32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_PRM_FILE_WRITE_ERROR,
            LogSeverity::WarningHi,
            &format!(
                "Parameter write failed in stage {stage:?} with record {record} and error {error}"
            ),
            |buf| {
                let status = stage.serialize_to(buf, Endianness::Big);
                if !status.is_ok() {
                    return status;
                }
                let status = buf.serialize_i32_be(record);
                if !status.is_ok() {
                    return status;
                }
                buf.serialize_i32_be(error)
            },
        );
    }

    /// Event + `EXECUTION_ERROR` response, the shared tail of every
    /// `PRM_SAVE_FILE` failure path.
    fn fail_save(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        stage: PrmWriteError,
        record: i32,
        error: i32,
    ) {
        self.log_write_error(stage, record, error);
        self.cmd
            .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
    }

    // -- Command dispatch --------------------------------------------------

    /// `CmdDisp` handler: dispatch on the local opcode with the
    /// exactly-once response discipline (`FormatError` on a failed
    /// argument deserialization or residual bytes, `ValidationError` on an
    /// invalid enum argument, `InvalidOpcode` for anything unknown).
    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_PRM_SAVE_FILE => {
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.prm_save_file_cmd_handler(op_code, cmd_seq);
            }
            Self::OPCODE_PRM_LOAD_FILE => {
                let mut file_name = CmdStringArg::new();
                let mut merge_raw = 0u8;
                if !args.deserialize(&mut file_name, Endianness::Big).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                if !args.deserialize_u8_be(&mut merge_raw).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                let merge = match Merge::try_from(merge_raw) {
                    Ok(merge) => merge,
                    Err(_) => {
                        self.cmd
                            .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
                        return;
                    }
                };
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.prm_load_file_cmd_handler(op_code, cmd_seq, &file_name, merge);
            }
            Self::OPCODE_PRM_COMMIT_STAGED => {
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.prm_commit_staged_cmd_handler(op_code, cmd_seq);
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }
}

/// Write one file field, mapping the two C++ failure checks (status, then
/// short write) onto their distinct `PrmWriteError` stages.
fn write_field(
    param_file: &mut File,
    bytes: &[u8],
    record: i32,
    error_stage: PrmWriteError,
    size_stage: PrmWriteError,
) -> Result<(), (PrmWriteError, i32, i32)> {
    let mut write_size: FwSizeType = 0;
    let stat = param_file.write(bytes, &mut write_size, WaitType::Wait);
    if stat != FileStatus::OpOk {
        return Err((error_stage, record, stat as i32));
    }
    if write_size as usize != bytes.len() {
        return Err((size_stage, record, write_size as i32));
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Ports.
// ---------------------------------------------------------------------------

input_port_adapter! {
    /// `getPrm` — GUARDED `Fw.PrmGet` input: read a parameter from the
    /// ACTIVE database on the caller's thread.
    component: PrmDb;
    adapter: GetPrmAdapter;
    port: PrmGetPort;
    input: pub get_prm;
    handler: get_prm_handler;
    returns: ParamValid;
    args { val id: FwPrmIdType, buf val: ParamBuffer }
}

async_input_port_adapter! {
    /// `setPrm` — ASYNC `Fw.PrmSet` input: add or update a parameter in the
    /// ACTIVE database.
    component: PrmDb;
    adapter: SetPrmAdapter;
    port: PrmSetPort;
    input: pub set_prm;
    deserialize: set_prm_deserialize;
    handler: set_prm_handler;
    base: active.queued;
    msg_type: PrmDb::MSG_TYPE_SET_PRM;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val id: FwPrmIdType, buf val: ParamBuffer }
}

async_input_port_adapter! {
    /// `pingIn` — ASYNC `Svc.Ping` input: liveness check answered on
    /// `pingOut`.
    component: PrmDb;
    adapter: PingInAdapter;
    port: PingPort;
    input: pub ping_in;
    deserialize: ping_in_deserialize;
    handler: ping_in_handler;
    base: active.queued;
    msg_type: PrmDb::MSG_TYPE_PING_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val key: u32 }
}

async_input_port_adapter! {
    /// `CmdDisp` — ASYNC `Fw.Cmd` input for this component's own commands.
    component: PrmDb;
    adapter: CmdInAdapter;
    port: CmdPort;
    input: pub cmd_in;
    deserialize: cmd_in_deserialize;
    handler: cmd_in_handler;
    base: active.queued;
    msg_type: PrmDb::MSG_TYPE_CMD;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, buf args: CmdArgBuffer }
}

// ---------------------------------------------------------------------------
// Dispatch (the hand-written doDispatch switch).
// ---------------------------------------------------------------------------

impl ComponentDispatch for PrmDb {
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
            Self::MSG_TYPE_SET_PRM => match Self::set_prm_deserialize(buf) {
                Some((id, mut val)) => {
                    self.set_prm_handler(port_num, id, &mut val);
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
            Self::MSG_TYPE_CMD => match Self::cmd_in_deserialize(buf) {
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

impl ActiveComponent for PrmDb {
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
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort, TimePort};
    use fprime_config::FwIdType;
    use fprime_fw::{LogBuffer, TextLogString, Time, TimeBase};
    use fprime_os::task::{Status as TaskStatus, TASK_DEFAULT};
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    const ID_BASE: FwIdType = 0x400;

    // -- Ground stub -------------------------------------------------------

    /// (id, severity, raw arg bytes)
    type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

    #[derive(Default)]
    struct GroundStub {
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: Mutex<Vec<EventRecord>>,
        text_events: Mutex<Vec<String>>,
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
            text: &mut TextLogString,
        ) {
            self.text_events
                .lock()
                .unwrap()
                .push(text.as_str().unwrap_or_default().to_string());
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

    impl GroundStub {
        fn event_ids(&self) -> Vec<FwEventIdType> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .map(|(id, _, _)| *id - ID_BASE)
                .collect()
        }

        fn events_with_id(&self, local_id: FwEventIdType) -> Vec<EventRecord> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .filter(|(id, _, _)| *id == ID_BASE + local_id)
                .cloned()
                .collect()
        }

        fn last_response(&self) -> (FwOpcodeType, u32, CmdResponse) {
            *self.responses.lock().unwrap().last().expect("a response")
        }
    }

    // -- Temp files --------------------------------------------------------

    static TEMP_COUNTER: AtomicU32 = AtomicU32::new(0);

    /// A per-test scratch directory under the system temp dir, removed with
    /// everything in it on drop.
    struct TempDir {
        path: PathBuf,
    }

    impl TempDir {
        fn new(tag: &str) -> Self {
            let unique = TEMP_COUNTER.fetch_add(1, Ordering::Relaxed);
            let mut path = std::env::temp_dir();
            // Names are kept SHORT on purpose: `PRM_LOAD_FILE` carries the
            // file name in a `Fw::CmdStringArg` (40 bytes, C++ parity), so a
            // long scratch path would make the command itself unusable.
            let _ = tag;
            path.push(format!("fpdb{}_{unique}", std::process::id()));
            let _ = std::fs::remove_dir_all(&path);
            std::fs::create_dir_all(&path).expect("temp dir");
            Self { path }
        }

        fn file(&self, name: &str) -> String {
            self.path
                .join(name)
                .to_str()
                .expect("utf-8 temp path")
                .to_string()
        }

        fn dir(&self) -> String {
            self.path.to_str().expect("utf-8 temp path").to_string()
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.path);
        }
    }

    // -- Helpers -----------------------------------------------------------

    fn build(name: &str) -> (Arc<PrmDb>, Arc<GroundStub>) {
        let ground = Arc::new(GroundStub::default());
        let comp = PrmDb::new(name);
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.evt.time_out.connect(Arc::new(TimeStub), 0);
        comp.ping_out.connect(ground.clone(), 0);
        comp.init(16);
        (comp, ground)
    }

    fn param(bytes: &[u8]) -> ParamBuffer {
        let mut buffer = ParamBuffer::new();
        assert!(buffer.set_buff(bytes).is_ok());
        buffer
    }

    /// Add a parameter through the `setPrm` handler (what the dispatch loop
    /// calls).
    fn set_prm(comp: &Arc<PrmDb>, id: FwPrmIdType, bytes: &[u8]) {
        let mut value = param(bytes);
        comp.set_prm_handler(0, id, &mut value);
    }

    /// Read a parameter through the guarded `getPrm` port.
    fn get_prm(comp: &Arc<PrmDb>, id: FwPrmIdType) -> (ParamValid, Vec<u8>) {
        let port = comp.get_prm(0);
        let mut value = ParamBuffer::new();
        let valid = port.target.invoke(port.port_num, id, &mut value);
        (valid, value.as_slice().to_vec())
    }

    /// Run a command through the command handler (what the dispatch loop
    /// calls), with raw argument bytes.
    fn run_cmd(comp: &Arc<PrmDb>, local_opcode: FwOpcodeType, cmd_seq: u32, arg_bytes: &[u8]) {
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(arg_bytes).is_ok());
        comp.cmd_in_handler(0, ID_BASE + local_opcode, cmd_seq, &mut args);
    }

    /// `PRM_LOAD_FILE` argument bytes: `[u16 len][name][u8 merge]`.
    fn load_file_args(name: &str, merge: Merge) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&(name.len() as u16).to_be_bytes());
        bytes.extend_from_slice(name.as_bytes());
        bytes.push(merge.as_repr());
        bytes
    }

    fn db_contents(store: &PrmDbStore) -> Vec<(FwPrmIdType, Vec<u8>)> {
        store
            .iter()
            .map(|(id, value)| (id, value.as_slice().to_vec()))
            .collect()
    }

    /// One on-disk record: `[0xA5][u32 size][u32 id][value]`.
    fn record(id: FwPrmIdType, value: &[u8]) -> Vec<u8> {
        let mut bytes = vec![ENTRY_DELIMITER];
        bytes.extend_from_slice(&((4 + value.len()) as u32).to_be_bytes());
        bytes.extend_from_slice(&id.to_be_bytes());
        bytes.extend_from_slice(value);
        bytes
    }

    /// Prepend the file CRC: the UN-complemented CRC-32 register over the
    /// body, big-endian.
    fn with_crc(body: &[u8]) -> Vec<u8> {
        let mut bytes = (!Hash::hash_u32(body)).to_be_bytes().to_vec();
        bytes.extend_from_slice(body);
        bytes
    }

    // -- File format -------------------------------------------------------

    /// Saving two parameters produces the exact documented bytes:
    /// `[u32 CRC]` then `[0xA5][u32 recordSize][u32 id][value]` per record,
    /// in insertion order.
    #[test]
    fn save_file_bytes_are_byte_exact() {
        let dir = TempDir::new("save_bytes");
        let path = dir.file("prm.dat");
        let (comp, ground) = build("prmDbSaveBytes");
        comp.configure(&path);

        set_prm(&comp, 0x1122_3344, &[0xDE, 0xAD]);
        set_prm(&comp, 5, &[1, 2, 3, 4]);
        run_cmd(&comp, PrmDb::OPCODE_PRM_SAVE_FILE, 1, &[]);

        assert_eq!(
            ground.last_response(),
            (ID_BASE + PrmDb::OPCODE_PRM_SAVE_FILE, 1, CmdResponse::Ok)
        );

        let bytes = std::fs::read(&path).expect("saved file");
        #[rustfmt::skip]
        let expected_body: Vec<u8> = vec![
            // record 0: id 0x11223344, value DE AD
            0xA5,
            0x00, 0x00, 0x00, 0x06, // recordSize = 4 + 2
            0x11, 0x22, 0x33, 0x44, // id
            0xDE, 0xAD,             // value
            // record 1: id 5, value 01 02 03 04
            0xA5,
            0x00, 0x00, 0x00, 0x08, // recordSize = 4 + 4
            0x00, 0x00, 0x00, 0x05, // id
            0x01, 0x02, 0x03, 0x04, // value
        ];
        assert_eq!(&bytes[4..], &expected_body[..]);
        // The stored CRC is the un-complemented CRC-32 register over every
        // byte after offset 4 (hand-checked literal + the rule that
        // produced it).
        assert_eq!(&bytes[..4], &[0xB1, 0xBF, 0x45, 0x28]);
        assert_eq!(
            &bytes[..4],
            &(!Hash::hash_u32(&expected_body)).to_be_bytes()
        );

        // PrmFileSaveComplete carries the record count.
        let saved = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_SAVE_COMPLETE);
        assert_eq!(saved.len(), 1);
        assert_eq!(saved[0].1, LogSeverity::ActivityHi);
        assert_eq!(saved[0].2, vec![0, 0, 0, 2]);
    }

    /// Save then reload: the reloaded database is identical, entry for
    /// entry, in the same order.
    #[test]
    fn save_then_reload_restores_identical_store() {
        let dir = TempDir::new("round_trip");
        let path = dir.file("prm.dat");

        let (writer, _wground) = build("prmDbWriter");
        writer.configure(&path);
        set_prm(&writer, 7, &[0x00, 0x01]);
        set_prm(&writer, 9, &[0xAA]);
        set_prm(&writer, 3, &[]);
        run_cmd(&writer, PrmDb::OPCODE_PRM_SAVE_FILE, 1, &[]);

        let (reader, ground) = build("prmDbReader");
        reader.configure(&path);
        reader.read_param_file();

        assert_eq!(
            db_contents(&reader.state.lock().unwrap().active),
            db_contents(&writer.state.lock().unwrap().active)
        );
        assert_eq!(
            db_contents(&reader.state.lock().unwrap().active),
            vec![
                (7, vec![0x00, 0x01]),
                (9, vec![0xAA]),
                (3, Vec::<u8>::new()),
            ]
        );
        // A zero-length value is legal (recordSize == 4).
        assert_eq!(get_prm(&reader, 3), (ParamValid::Valid, Vec::new()));

        // PrmFileLoadComplete: db string, total, added, updated.
        let complete = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_LOAD_COMPLETE);
        assert_eq!(complete.len(), 1);
        let mut expected = vec![0x00, 0x06];
        expected.extend_from_slice(b"ACTIVE");
        expected.extend_from_slice(&3u32.to_be_bytes());
        expected.extend_from_slice(&3u32.to_be_bytes());
        expected.extend_from_slice(&0u32.to_be_bytes());
        assert_eq!(complete[0].2, expected);
    }

    /// An empty database saves as just the CRC of nothing, and reloads.
    #[test]
    fn empty_database_saves_and_reloads() {
        let dir = TempDir::new("empty");
        let path = dir.file("prm.dat");
        let (comp, ground) = build("prmDbEmpty");
        comp.configure(&path);
        run_cmd(&comp, PrmDb::OPCODE_PRM_SAVE_FILE, 1, &[]);
        assert_eq!(ground.last_response().2, CmdResponse::Ok);

        // The CRC register starts at 0xFFFFFFFF and is never updated, and
        // the file body is empty.
        assert_eq!(std::fs::read(&path).unwrap(), vec![0xFF, 0xFF, 0xFF, 0xFF]);

        comp.read_param_file();
        assert!(comp.state.lock().unwrap().active.is_empty());
        let complete = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_LOAD_COMPLETE);
        assert_eq!(complete.len(), 1);
    }

    #[test]
    fn load_rejects_crc_mismatch() {
        let dir = TempDir::new("bad_crc");
        let path = dir.file("prm.dat");
        let body = record(1, &[0xAB]);
        let mut bytes = with_crc(&body);
        bytes[3] ^= 0xFF; // corrupt the stored CRC
        std::fs::write(&path, &bytes).unwrap();

        let (comp, ground) = build("prmDbBadCrc");
        comp.configure(&path);
        comp.read_param_file();

        let bad = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_BAD_CRC);
        assert_eq!(bad.len(), 1);
        assert_eq!(bad[0].1, LogSeverity::WarningHi);
        let read_crc = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let computed = !Hash::hash_u32(&body);
        let mut expected = read_crc.to_be_bytes().to_vec();
        expected.extend_from_slice(&computed.to_be_bytes());
        assert_eq!(bad[0].2, expected);
        assert!(comp.state.lock().unwrap().active.is_empty());
    }

    #[test]
    fn load_rejects_bad_delimiter() {
        let dir = TempDir::new("bad_delim");
        let path = dir.file("prm.dat");
        let mut body = record(1, &[0xAB]);
        body[0] = 0x5A; // wrong delimiter
        std::fs::write(&path, with_crc(&body)).unwrap();

        let (comp, ground) = build("prmDbBadDelim");
        comp.configure(&path);
        comp.read_param_file();

        let errors = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_READ_ERROR);
        assert_eq!(errors.len(), 1);
        assert_eq!(errors[0].1, LogSeverity::WarningHi);
        // [stage u8][record i32][error i32]
        let mut expected = vec![PrmReadError::DelimiterValue.as_repr()];
        expected.extend_from_slice(&0i32.to_be_bytes());
        expected.extend_from_slice(&0x5Ai32.to_be_bytes());
        assert_eq!(errors[0].2, expected);
        assert!(comp.state.lock().unwrap().active.is_empty());
    }

    #[test]
    fn load_rejects_oversized_record() {
        let dir = TempDir::new("big_record");
        let path = dir.file("prm.dat");
        let mut body = vec![ENTRY_DELIMITER];
        // recordSize = FW_PARAM_BUFFER_MAX_SIZE + 4 + 1 = 511 (one past the
        // maximum), with no payload behind it.
        body.extend_from_slice(&(MAX_RECORD_SIZE + 1).to_be_bytes());
        std::fs::write(&path, with_crc(&body)).unwrap();

        let (comp, ground) = build("prmDbBigRecord");
        comp.configure(&path);
        comp.read_param_file();

        let errors = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_READ_ERROR);
        assert_eq!(errors.len(), 1);
        let mut expected = vec![PrmReadError::RecordSizeValue.as_repr()];
        expected.extend_from_slice(&0i32.to_be_bytes());
        expected.extend_from_slice(&(MAX_RECORD_SIZE as i32 + 1).to_be_bytes());
        assert_eq!(errors[0].2, expected);
    }

    #[test]
    fn load_rejects_undersized_record() {
        let dir = TempDir::new("small_record");
        let path = dir.file("prm.dat");
        let mut body = vec![ENTRY_DELIMITER];
        body.extend_from_slice(&3u32.to_be_bytes()); // < sizeof(FwPrmIdType)
        std::fs::write(&path, with_crc(&body)).unwrap();

        let (comp, ground) = build("prmDbSmallRecord");
        comp.configure(&path);
        comp.read_param_file();

        let errors = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_READ_ERROR);
        assert_eq!(errors.len(), 1);
        assert_eq!(errors[0].2[0], PrmReadError::RecordSizeValue.as_repr());
    }

    /// A file whose last record is cut short fails with the stage that ran
    /// out of bytes — not with a clean EOF.
    #[test]
    fn load_rejects_truncated_file() {
        let dir = TempDir::new("truncated");

        // (a) truncated inside the parameter value.
        let path = dir.file("value.dat");
        let mut body = record(1, &[0xAA]);
        body.extend_from_slice(&record(2, &[0xBB, 0xCC])[..10]); // 1 of 2 value bytes
        std::fs::write(&path, with_crc(&body)).unwrap();
        let (comp, ground) = build("prmDbTruncValue");
        comp.configure(&path);
        comp.read_param_file();
        let errors = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_READ_ERROR);
        assert_eq!(errors.len(), 1);
        let mut expected = vec![PrmReadError::ParameterValueSize.as_repr()];
        expected.extend_from_slice(&1i32.to_be_bytes()); // second record
        expected.extend_from_slice(&1i32.to_be_bytes()); // read 1 of 2 bytes
        assert_eq!(errors[0].2, expected);
        // The records read before the truncation are still applied; the
        // load as a whole reports failure.
        assert_eq!(comp.state.lock().unwrap().active.len(), 1);

        // (b) truncated inside the record-size field.
        let path = dir.file("size.dat");
        let mut body = record(1, &[0xAA]);
        body.extend_from_slice(&[ENTRY_DELIMITER, 0x00, 0x00]);
        std::fs::write(&path, with_crc(&body)).unwrap();
        let (comp, ground) = build("prmDbTruncSize");
        comp.configure(&path);
        comp.read_param_file();
        let errors = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_READ_ERROR);
        assert_eq!(errors.len(), 1);
        assert_eq!(errors[0].2[0], PrmReadError::RecordSizeSize.as_repr());

        // (c) a file too short to even hold the CRC.
        let path = dir.file("crc.dat");
        std::fs::write(&path, [0x01, 0x02]).unwrap();
        let (comp, ground) = build("prmDbTruncCrc");
        comp.configure(&path);
        comp.read_param_file();
        let errors = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_READ_ERROR);
        assert_eq!(errors.len(), 1);
        let mut expected = vec![PrmReadError::CrcSize.as_repr()];
        expected.extend_from_slice(&0i32.to_be_bytes());
        expected.extend_from_slice(&2i32.to_be_bytes());
        assert_eq!(errors[0].2, expected);
    }

    #[test]
    fn load_reports_open_failure() {
        let dir = TempDir::new("missing");
        let path = dir.file("does-not-exist.dat");
        let (comp, ground) = build("prmDbMissing");
        comp.configure(&path);
        comp.read_param_file();

        let errors = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_READ_ERROR);
        assert_eq!(errors.len(), 1);
        let mut expected = vec![PrmReadError::Open.as_repr()];
        expected.extend_from_slice(&0i32.to_be_bytes());
        expected.extend_from_slice(&(FileStatus::DoesntExist as i32).to_be_bytes());
        assert_eq!(errors[0].2, expected);
        assert!(comp.state.lock().unwrap().active.is_empty());
    }

    /// Records past `PRMDB_NUM_DB_ENTRIES` are silently ignored — the load
    /// still succeeds.
    #[test]
    fn load_ignores_records_beyond_capacity() {
        let dir = TempDir::new("overflow_file");
        let path = dir.file("prm.dat");
        let mut body = Vec::new();
        for id in 0..(NUM_DB_ENTRIES as u32 + 3) {
            body.extend_from_slice(&record(id, &[id as u8]));
        }
        std::fs::write(&path, with_crc(&body)).unwrap();

        let (comp, ground) = build("prmDbOverflowFile");
        comp.configure(&path);
        comp.read_param_file();

        assert_eq!(comp.state.lock().unwrap().active.len(), NUM_DB_ENTRIES);
        // Success: no PrmDbFull, one PrmFileLoadComplete with 25 records.
        assert!(ground.events_with_id(PrmDb::EVENTID_PRM_DB_FULL).is_empty());
        let complete = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_LOAD_COMPLETE);
        assert_eq!(complete.len(), 1);
        assert_eq!(
            &complete[0].2[8..12],
            &(NUM_DB_ENTRIES as u32).to_be_bytes()
        );
    }

    /// C++ parity quirk: `OPEN_WRITE` does not truncate, so a shorter image
    /// leaves the previous file's tail in place — and the CRC (which covers
    /// everything to EOF) then rejects the file on load.
    #[test]
    fn save_over_longer_file_leaves_residue_and_fails_crc() {
        let dir = TempDir::new("no_truncate");
        let path = dir.file("prm.dat");
        std::fs::write(&path, vec![0x77u8; 200]).unwrap();

        let (comp, ground) = build("prmDbNoTruncate");
        comp.configure(&path);
        set_prm(&comp, 1, &[0xAB]);
        run_cmd(&comp, PrmDb::OPCODE_PRM_SAVE_FILE, 1, &[]);
        assert_eq!(ground.last_response().2, CmdResponse::Ok);

        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(bytes.len(), 200);
        assert_eq!(&bytes[4..14], &record(1, &[0xAB])[..]);
        assert!(bytes[14..].iter().all(|b| *b == 0x77));

        comp.read_param_file();
        assert_eq!(
            ground.events_with_id(PrmDb::EVENTID_PRM_FILE_BAD_CRC).len(),
            1
        );
    }

    // -- getPrm / setPrm ---------------------------------------------------

    #[test]
    fn get_prm_returns_valid_value_for_known_id() {
        let (comp, ground) = build("prmDbGet");
        set_prm(&comp, 42, &[1, 2, 3]);
        assert_eq!(get_prm(&comp, 42), (ParamValid::Valid, vec![1, 2, 3]));
        assert!(
            ground
                .events_with_id(PrmDb::EVENTID_PRM_ID_NOT_FOUND)
                .is_empty()
        );
    }

    /// A miss returns INVALID, emits PrmIdNotFound, and the event is
    /// throttled after 5 emissions.
    #[test]
    fn get_prm_miss_is_invalid_and_throttled() {
        let (comp, ground) = build("prmDbMiss");
        for _ in 0..8 {
            assert_eq!(get_prm(&comp, 0xABCD).0, ParamValid::Invalid);
        }
        let misses = ground.events_with_id(PrmDb::EVENTID_PRM_ID_NOT_FOUND);
        assert_eq!(misses.len(), PrmDb::PRM_ID_NOT_FOUND_THROTTLE as usize);
        assert_eq!(misses[0].1, LogSeverity::WarningLo);
        assert_eq!(misses[0].2, 0xABCDu32.to_be_bytes().to_vec());
    }

    /// A miss must leave the caller's buffer untouched (C++ ArrayMap::find
    /// only writes on success).
    #[test]
    fn get_prm_miss_leaves_buffer_untouched() {
        let (comp, _ground) = build("prmDbMissBuffer");
        let port = comp.get_prm(0);
        let mut value = param(&[9, 9, 9]);
        let valid = port.target.invoke(port.port_num, 1, &mut value);
        assert_eq!(valid, ParamValid::Invalid);
        assert_eq!(value.as_slice(), &[9, 9, 9]);
    }

    #[test]
    fn set_prm_adds_then_updates() {
        let (comp, ground) = build("prmDbSet");
        set_prm(&comp, 1, &[0xAA]);
        set_prm(&comp, 1, &[0xBB, 0xCC]);
        assert_eq!(
            ground.event_ids(),
            vec![PrmDb::EVENTID_PRM_ID_ADDED, PrmDb::EVENTID_PRM_ID_UPDATED]
        );
        // The update replaces the value in place, keeping the position.
        assert_eq!(
            db_contents(&comp.state.lock().unwrap().active),
            vec![(1, vec![0xBB, 0xCC])]
        );
        assert_eq!(get_prm(&comp, 1), (ParamValid::Valid, vec![0xBB, 0xCC]));
    }

    #[test]
    fn set_prm_reports_db_full() {
        let (comp, ground) = build("prmDbFull");
        for id in 0..NUM_DB_ENTRIES as u32 {
            set_prm(&comp, id, &[id as u8]);
        }
        set_prm(&comp, 999, &[0xFF]);
        let full = ground.events_with_id(PrmDb::EVENTID_PRM_DB_FULL);
        assert_eq!(full.len(), 1);
        assert_eq!(full[0].1, LogSeverity::WarningHi);
        assert_eq!(full[0].2, 999u32.to_be_bytes().to_vec());
        assert_eq!(comp.state.lock().unwrap().active.len(), NUM_DB_ENTRIES);
        // A full database still updates an EXISTING id.
        set_prm(&comp, 0, &[0x11]);
        assert_eq!(get_prm(&comp, 0), (ParamValid::Valid, vec![0x11]));
    }

    /// Dropping a record during a load fails the whole load, after the rest
    /// of the file has been processed.
    #[test]
    fn load_with_dropped_record_fails() {
        let dir = TempDir::new("load_full");
        let path = dir.file("extra.dat");
        // The file holds one id that is NOT in the (already full) database
        // plus one that is.
        let mut body = record(1000, &[0x01]);
        body.extend_from_slice(&record(0, &[0x02]));
        std::fs::write(&path, with_crc(&body)).unwrap();

        let (comp, ground) = build("prmDbLoadFull");
        for id in 0..NUM_DB_ENTRIES as u32 {
            set_prm(&comp, id, &[id as u8]);
        }
        run_cmd(
            &comp,
            PrmDb::OPCODE_PRM_LOAD_FILE,
            7,
            &load_file_args(&path, Merge::Merge),
        );

        assert_eq!(
            ground.last_response(),
            (
                ID_BASE + PrmDb::OPCODE_PRM_LOAD_FILE,
                7,
                CmdResponse::ExecutionError
            )
        );
        assert_eq!(ground.events_with_id(PrmDb::EVENTID_PRM_DB_FULL).len(), 1);
        assert_eq!(
            ground
                .events_with_id(PrmDb::EVENTID_PRM_DB_FILE_LOAD_FAILED)
                .len(),
            1
        );
        // No completion event, staging cleared, state back to Idle.
        assert!(
            ground
                .events_with_id(PrmDb::EVENTID_PRM_FILE_LOAD_COMPLETE)
                .is_empty()
        );
        let state = comp.state.lock().unwrap();
        assert!(state.staging.is_empty());
        assert_eq!(state.load_state, PrmDbFileLoadState::Idle);
        // The second record WAS processed before the load failed.
        assert_eq!(state.active.len(), NUM_DB_ENTRIES);
    }

    // -- State machine -----------------------------------------------------

    /// Write a small file and run a successful `PRM_LOAD_FILE`.
    fn stage_a_load(comp: &Arc<PrmDb>, path: &str, merge: Merge, cmd_seq: u32) {
        let mut body = record(100, &[0x0A]);
        body.extend_from_slice(&record(101, &[0x0B]));
        std::fs::write(path, with_crc(&body)).unwrap();
        run_cmd(
            comp,
            PrmDb::OPCODE_PRM_LOAD_FILE,
            cmd_seq,
            &load_file_args(path, merge),
        );
    }

    #[test]
    fn load_then_commit_swaps_databases() {
        let dir = TempDir::new("commit");
        let path = dir.file("upd.dat");
        let (comp, ground) = build("prmDbCommit");
        set_prm(&comp, 1, &[0x01]);

        stage_a_load(&comp, &path, Merge::Reset, 1);
        assert_eq!(
            ground.last_response(),
            (ID_BASE + PrmDb::OPCODE_PRM_LOAD_FILE, 1, CmdResponse::Ok)
        );
        {
            let state = comp.state.lock().unwrap();
            assert_eq!(state.load_state, PrmDbFileLoadState::FileUpdatesStaged);
            // RESET: staging holds ONLY the file contents.
            assert_eq!(
                db_contents(&state.staging),
                vec![(100, vec![0x0A]), (101, vec![0x0B])]
            );
            // The active database is untouched until the commit.
            assert_eq!(db_contents(&state.active), vec![(1, vec![0x01])]);
        }
        // getPrm still reads the OLD active database.
        assert_eq!(get_prm(&comp, 100).0, ParamValid::Invalid);

        run_cmd(&comp, PrmDb::OPCODE_PRM_COMMIT_STAGED, 2, &[]);
        assert_eq!(
            ground.last_response(),
            (
                ID_BASE + PrmDb::OPCODE_PRM_COMMIT_STAGED,
                2,
                CmdResponse::Ok
            )
        );
        assert_eq!(
            ground
                .events_with_id(PrmDb::EVENTID_PRM_DB_COMMIT_COMPLETE)
                .len(),
            1
        );
        let state = comp.state.lock().unwrap();
        assert_eq!(state.load_state, PrmDbFileLoadState::Idle);
        assert_eq!(
            db_contents(&state.active),
            vec![(100, vec![0x0A]), (101, vec![0x0B])]
        );
        assert!(state.staging.is_empty());
    }

    #[test]
    fn load_with_merge_copies_active_into_staging_first() {
        let dir = TempDir::new("merge");
        let path = dir.file("upd.dat");
        let (comp, ground) = build("prmDbMerge");
        set_prm(&comp, 1, &[0x01]);
        set_prm(&comp, 100, &[0xFF]); // overwritten by the file

        stage_a_load(&comp, &path, Merge::Merge, 1);

        // PrmDbCopyAllComplete(src=ACTIVE, dest=STAGING).
        let copies = ground.events_with_id(PrmDb::EVENTID_PRM_DB_COPY_ALL_COMPLETE);
        assert_eq!(copies.len(), 1);
        let mut expected = vec![0x00, 0x06];
        expected.extend_from_slice(b"ACTIVE");
        expected.extend_from_slice(&[0x00, 0x07]);
        expected.extend_from_slice(b"STAGING");
        assert_eq!(copies[0].2, expected);

        let state = comp.state.lock().unwrap();
        assert_eq!(
            db_contents(&state.staging),
            vec![(1, vec![0x01]), (100, vec![0x0A]), (101, vec![0x0B])]
        );
        drop(state);

        // The load event reports 1 updated (id 100) and 1 added (id 101).
        let complete = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_LOAD_COMPLETE);
        assert_eq!(&complete[0].2[9..13], &2u32.to_be_bytes()); // total
        assert_eq!(&complete[0].2[13..17], &1u32.to_be_bytes()); // added
        assert_eq!(&complete[0].2[17..21], &1u32.to_be_bytes()); // updated
    }

    #[test]
    fn commands_are_busy_while_updates_are_staged() {
        let dir = TempDir::new("busy");
        let path = dir.file("upd.dat");
        let (comp, ground) = build("prmDbBusy");
        comp.configure(&dir.file("prm.dat"));
        stage_a_load(&comp, &path, Merge::Reset, 1);

        // PRM_SAVE_FILE -> BUSY + invalid-action event.
        run_cmd(&comp, PrmDb::OPCODE_PRM_SAVE_FILE, 2, &[]);
        assert_eq!(
            ground.last_response(),
            (ID_BASE + PrmDb::OPCODE_PRM_SAVE_FILE, 2, CmdResponse::Busy)
        );
        // PRM_LOAD_FILE -> BUSY.
        run_cmd(
            &comp,
            PrmDb::OPCODE_PRM_LOAD_FILE,
            3,
            &load_file_args(&path, Merge::Reset),
        );
        assert_eq!(
            ground.last_response(),
            (ID_BASE + PrmDb::OPCODE_PRM_LOAD_FILE, 3, CmdResponse::Busy)
        );
        // setPrm -> rejected, database unchanged.
        set_prm(&comp, 55, &[0x01]);
        assert_eq!(get_prm(&comp, 55).0, ParamValid::Invalid);

        let invalid = ground.events_with_id(PrmDb::EVENTID_PRM_DB_FILE_LOAD_INVALID_ACTION);
        assert_eq!(invalid.len(), 3);
        assert_eq!(invalid[0].1, LogSeverity::WarningLo);
        assert_eq!(
            invalid[0].2,
            vec![
                PrmDbFileLoadState::FileUpdatesStaged.as_repr(),
                PrmLoadAction::SaveFileCommand.as_repr(),
            ]
        );
        assert_eq!(invalid[1].2[1], PrmLoadAction::LoadFileCommand.as_repr());
        assert_eq!(invalid[2].2[1], PrmLoadAction::SetParameter.as_repr());
        // No parameter file was written.
        assert!(!std::path::Path::new(&dir.file("prm.dat")).exists());
    }

    #[test]
    fn commit_staged_requires_staged_state() {
        let (comp, ground) = build("prmDbCommitIdle");
        run_cmd(&comp, PrmDb::OPCODE_PRM_COMMIT_STAGED, 1, &[]);
        assert_eq!(
            ground.last_response(),
            (
                ID_BASE + PrmDb::OPCODE_PRM_COMMIT_STAGED,
                1,
                CmdResponse::ValidationError
            )
        );
        let invalid = ground.events_with_id(PrmDb::EVENTID_PRM_DB_FILE_LOAD_INVALID_ACTION);
        assert_eq!(
            invalid[0].2,
            vec![
                PrmDbFileLoadState::Idle.as_repr(),
                PrmLoadAction::CommitStagedCommand.as_repr(),
            ]
        );
    }

    /// A failed commanded load clears staging and returns to Idle, leaving
    /// the active database usable.
    #[test]
    fn failed_load_clears_staging_and_returns_to_idle() {
        let dir = TempDir::new("failed_load");
        let path = dir.file("bad.dat");
        std::fs::write(&path, [0x00, 0x00, 0x00, 0x00, 0xA5]).unwrap(); // bad CRC

        let (comp, ground) = build("prmDbFailedLoad");
        set_prm(&comp, 1, &[0x01]);
        run_cmd(
            &comp,
            PrmDb::OPCODE_PRM_LOAD_FILE,
            4,
            &load_file_args(&path, Merge::Merge),
        );

        assert_eq!(ground.last_response().2, CmdResponse::ExecutionError);
        assert_eq!(
            ground
                .events_with_id(PrmDb::EVENTID_PRM_DB_FILE_LOAD_FAILED)
                .len(),
            1
        );
        let state = comp.state.lock().unwrap();
        assert_eq!(state.load_state, PrmDbFileLoadState::Idle);
        assert!(state.staging.is_empty());
        assert_eq!(db_contents(&state.active), vec![(1, vec![0x01])]);
    }

    // -- Command argument handling -----------------------------------------

    #[test]
    fn command_argument_errors_respond_exactly_once() {
        let (comp, ground) = build("prmDbCmdArgs");
        comp.configure("/nonexistent-dir/prm.dat");

        // Residual bytes on a no-argument command.
        run_cmd(&comp, PrmDb::OPCODE_PRM_SAVE_FILE, 1, &[0x00]);
        run_cmd(&comp, PrmDb::OPCODE_PRM_COMMIT_STAGED, 2, &[0x00]);
        // Short PRM_LOAD_FILE arguments (string only, no merge byte).
        let mut short = 4u16.to_be_bytes().to_vec();
        short.extend_from_slice(b"file");
        run_cmd(&comp, PrmDb::OPCODE_PRM_LOAD_FILE, 3, &short);
        // Invalid Merge enum value.
        let mut bad_enum = load_file_args("file", Merge::Merge);
        *bad_enum.last_mut().unwrap() = 7;
        run_cmd(&comp, PrmDb::OPCODE_PRM_LOAD_FILE, 4, &bad_enum);
        // Residual bytes after a valid argument list.
        let mut trailing = load_file_args("file", Merge::Reset);
        trailing.push(0x99);
        run_cmd(&comp, PrmDb::OPCODE_PRM_LOAD_FILE, 5, &trailing);
        // Unknown opcode.
        run_cmd(&comp, 0x7F, 6, &[]);

        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (
                    ID_BASE + PrmDb::OPCODE_PRM_SAVE_FILE,
                    1,
                    CmdResponse::FormatError
                ),
                (
                    ID_BASE + PrmDb::OPCODE_PRM_COMMIT_STAGED,
                    2,
                    CmdResponse::FormatError
                ),
                (
                    ID_BASE + PrmDb::OPCODE_PRM_LOAD_FILE,
                    3,
                    CmdResponse::FormatError
                ),
                (
                    ID_BASE + PrmDb::OPCODE_PRM_LOAD_FILE,
                    4,
                    CmdResponse::ValidationError
                ),
                (
                    ID_BASE + PrmDb::OPCODE_PRM_LOAD_FILE,
                    5,
                    CmdResponse::FormatError
                ),
                (ID_BASE + 0x7F, 6, CmdResponse::InvalidOpcode),
            ]
        );
        // None of the rejected commands touched the state machine.
        assert_eq!(comp.load_state(), PrmDbFileLoadState::Idle);
    }

    #[test]
    fn save_file_open_failure_reports_execution_error() {
        let dir = TempDir::new("open_fail");
        // A directory is not a writable file.
        let (comp, ground) = build("prmDbOpenFail");
        comp.configure(&dir.dir());
        run_cmd(&comp, PrmDb::OPCODE_PRM_SAVE_FILE, 1, &[]);
        assert_eq!(ground.last_response().2, CmdResponse::ExecutionError);
        let errors = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_WRITE_ERROR);
        assert_eq!(errors.len(), 1);
        assert_eq!(errors[0].1, LogSeverity::WarningHi);
        assert_eq!(errors[0].2[0], PrmWriteError::Open.as_repr());
    }

    // -- Sandbox -----------------------------------------------------------

    #[test]
    fn sandbox_rejects_paths_outside_the_allowed_directory() {
        let dir = TempDir::new("sandbox");
        let inside = dir.file("inside.dat");
        let body = record(1, &[0x01]);
        std::fs::write(&inside, with_crc(&body)).unwrap();

        let outside_dir = TempDir::new("sandbox_out");
        let outside = outside_dir.file("outside.dat");
        std::fs::write(&outside, with_crc(&body)).unwrap();

        let (comp, ground) = build("prmDbSandbox");
        comp.configure_load_sandbox(&dir.dir());

        // Inside: loads fine.
        run_cmd(
            &comp,
            PrmDb::OPCODE_PRM_LOAD_FILE,
            1,
            &load_file_args(&inside, Merge::Reset),
        );
        assert_eq!(ground.last_response().2, CmdResponse::Ok);
        run_cmd(&comp, PrmDb::OPCODE_PRM_COMMIT_STAGED, 2, &[]);

        // Outside: rejected at open with OUTSIDE_SANDBOX.
        run_cmd(
            &comp,
            PrmDb::OPCODE_PRM_LOAD_FILE,
            3,
            &load_file_args(&outside, Merge::Reset),
        );
        assert_eq!(ground.last_response().2, CmdResponse::ExecutionError);
        let errors = ground.events_with_id(PrmDb::EVENTID_PRM_FILE_READ_ERROR);
        assert_eq!(errors.len(), 1);
        let mut expected = vec![PrmReadError::Open.as_repr()];
        expected.extend_from_slice(&0i32.to_be_bytes());
        expected.extend_from_slice(&(FileStatus::OutsideSandbox as i32).to_be_bytes());
        assert_eq!(errors[0].2, expected);

        // Traversal out of the sandbox is rejected too.
        let traversal = format!("{}/../{}", dir.dir(), "escape.dat");
        run_cmd(
            &comp,
            PrmDb::OPCODE_PRM_LOAD_FILE,
            4,
            &load_file_args(&traversal, Merge::Reset),
        );
        assert_eq!(ground.last_response().2, CmdResponse::ExecutionError);
    }

    /// The boot-time load is NOT sandboxed (it uses the configured file
    /// name, not a ground-supplied one).
    #[test]
    fn sandbox_does_not_restrict_the_boot_load() {
        let dir = TempDir::new("sandbox_boot");
        let other = TempDir::new("sandbox_boot_other");
        let path = other.file("prm.dat");
        std::fs::write(&path, with_crc(&record(1, &[0x01]))).unwrap();

        let (comp, _ground) = build("prmDbSandboxBoot");
        comp.configure(&path);
        comp.configure_load_sandbox(&dir.dir());
        comp.read_param_file();
        assert_eq!(get_prm(&comp, 1), (ParamValid::Valid, vec![0x01]));
    }

    // -- Queue / dispatch --------------------------------------------------

    /// The async `setPrm` envelope is byte exact:
    /// `[msg_type i32][port_num i16][id u32][u16 len][value]`.
    #[test]
    fn set_prm_envelope_bytes_are_byte_exact() {
        let (comp, _ground) = build("prmDbEnvelope");
        let port = comp.set_prm(2);
        let mut value = param(&[0xAA, 0xBB]);
        port.target.invoke(port.port_num, 0x0102_0304, &mut value);

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
                0x00, 0x00, 0x00, 0x01, // msg_type = MSG_TYPE_SET_PRM
                0x00, 0x02, // port_num
                0x01, 0x02, 0x03, 0x04, // id
                0x00, 0x02, // ParamBuffer length prefix
                0xAA, 0xBB, // value
            ]
        );
        assert_eq!(priority, QUEUE_PRIORITY);
    }

    /// The queue message must fit the largest async invocation: a command
    /// carrying a full 506-byte argument buffer.
    #[test]
    fn queue_message_size_fits_a_full_command() {
        let (comp, _ground) = build("prmDbMsgSize");
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(&[0x5Au8; 506]).is_ok());
        let port = comp.cmd_in(0);
        port.target.invoke(port.port_num, ID_BASE, 1, &mut args);

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
        assert_eq!(size as usize, QUEUE_MSG_SIZE);
    }

    /// End to end over the real queue and task: registration, async
    /// setPrm, a save command, a ping, then exit/join.
    #[test]
    fn full_lifecycle_end_to_end() {
        let dir = TempDir::new("lifecycle");
        let path = dir.file("prm.dat");
        let (comp, ground) = build("prmDbLifecycle");
        comp.configure(&path);
        comp.reg_commands();
        assert_eq!(
            *ground.regs.lock().unwrap(),
            vec![
                ID_BASE + PrmDb::OPCODE_PRM_SAVE_FILE,
                ID_BASE + PrmDb::OPCODE_PRM_LOAD_FILE,
                ID_BASE + PrmDb::OPCODE_PRM_COMMIT_STAGED,
            ]
        );

        comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);

        let set_port = comp.set_prm(0);
        let mut value = param(&[0x12, 0x34]);
        set_port.target.invoke(set_port.port_num, 77, &mut value);

        let cmd_port = comp.cmd_in(0);
        let mut args = CmdArgBuffer::new();
        cmd_port.target.invoke(
            cmd_port.port_num,
            ID_BASE + PrmDb::OPCODE_PRM_SAVE_FILE,
            9,
            &mut args,
        );

        let ping_port = comp.ping_in(0);
        ping_port.target.invoke(ping_port.port_num, 0xFEED);

        comp.active.exit();
        assert_eq!(comp.active.join(), TaskStatus::OpOk);

        assert_eq!(*ground.pings.lock().unwrap(), vec![0xFEED]);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(ID_BASE + PrmDb::OPCODE_PRM_SAVE_FILE, 9, CmdResponse::Ok)]
        );
        assert_eq!(
            ground.event_ids(),
            vec![
                PrmDb::EVENTID_PRM_ID_ADDED,
                PrmDb::EVENTID_PRM_FILE_SAVE_COMPLETE
            ]
        );
        // The file the component thread wrote reloads into an identical DB.
        let mut expected = 4u32.to_be_bytes().to_vec();
        expected.extend_from_slice(&record(77, &[0x12, 0x34]));
        assert_eq!(std::fs::read(&path).unwrap()[4..], expected[4..]);
        assert!(!ground.text_events.lock().unwrap().is_empty());
    }

    /// A truncated queue message is a dispatch error, not a panic.
    #[test]
    fn dispatch_rejects_malformed_messages() {
        let (comp, _ground) = build("prmDbBadMsg");
        let mut msg = fprime_fw::LinearBuffer::<QUEUE_MSG_SIZE>::new();
        assert!(msg::write_envelope_header(&mut msg, PrmDb::MSG_TYPE_SET_PRM, 0).is_ok());
        // No id/value follow.
        let mut header = fprime_fw::LinearBuffer::<QUEUE_MSG_SIZE>::new();
        assert!(header.set_buff(&msg.as_slice()[4..]).is_ok());
        assert_eq!(
            comp.dispatch_message(PrmDb::MSG_TYPE_SET_PRM, &mut header),
            MsgDispatchStatus::Error
        );

        let mut unknown = fprime_fw::LinearBuffer::<QUEUE_MSG_SIZE>::new();
        assert!(unknown.set_buff(&[0x00, 0x00]).is_ok());
        assert_eq!(
            comp.dispatch_message(99, &mut unknown),
            MsgDispatchStatus::Error
        );
    }

    /// C++ parity limitation: `PRM_LOAD_FILE` carries the file name in a
    /// `Fw::CmdStringArg` (`FW_CMD_STRING_MAX_SIZE` = 40), even though the
    /// FPP model declares `string size FileNameStringSize`. A longer name
    /// fails to deserialize and is answered with FormatError.
    #[test]
    fn load_file_name_longer_than_cmd_string_is_format_error() {
        let (comp, ground) = build("prmDbLongName");
        let long_name = "/".to_string() + &"a".repeat(40);
        assert!(long_name.len() > 40);
        run_cmd(
            &comp,
            PrmDb::OPCODE_PRM_LOAD_FILE,
            1,
            &load_file_args(&long_name, Merge::Reset),
        );
        assert_eq!(ground.last_response().2, CmdResponse::FormatError);
        assert_eq!(comp.load_state(), PrmDbFileLoadState::Idle);

        // Exactly 40 characters still deserializes (and then fails to open).
        let max_name = "/".to_string() + &"a".repeat(39);
        run_cmd(
            &comp,
            PrmDb::OPCODE_PRM_LOAD_FILE,
            2,
            &load_file_args(&max_name, Merge::Reset),
        );
        assert_eq!(ground.last_response().2, CmdResponse::ExecutionError);
        assert_eq!(
            ground.events_with_id(PrmDb::EVENTID_PRM_FILE_READ_ERROR)[0].2[0],
            PrmReadError::Open.as_repr()
        );
    }
}
