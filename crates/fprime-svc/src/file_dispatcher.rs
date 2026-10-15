//! # Svc::FileDispatcher — route announced files by extension (active component)
//!
//! C++ sources: `Svc/FileDispatcher/FileDispatcher.{cpp,hpp,fpp}`,
//! `default/config/FileDispatcherCfg.fpp`; SDD
//! `Svc/FileDispatcher/docs/sdd.md`; tests
//! `Svc/FileDispatcher/test/ut/FileDispatcherTester.cpp` (ported below).
//!
//! A file name arriving on the async `fileAnnounceRecv` port (typically from
//! `Svc::FileUplink`'s `fileAnnounce`) is matched against a configured table
//! of file extensions; every ENABLED entry whose extension ends the name
//! sends the name out on that entry's `fileDispatch` port (an
//! `Svc.FileDispatch` port, e.g. `Svc::CmdSequencer`'s `seqDispatchIn`). The
//! `ENABLE_DISPATCH` command enables or disables every entry routed to a
//! port.
//!
//! C++-parity notes:
//! - The table walk does not stop at the first match: the same file can be
//!   dispatched on several ports (one `FileDispatched` event each), and two
//!   entries routed to the same port dispatch it twice.
//! - The match is "the LAST occurrence of the extension is at the end of the
//!   name" (`Fw::StringUtils::substring_find_last`), i.e. `ends_with` for a
//!   non-empty extension (empty extensions are rejected by
//!   [`FileDispatcher::configure`]).
//! - The name is passed to each `fileDispatch` port by reference (C++
//!   `ref`): a synchronous handler that edits it changes the name logged in
//!   `FileDispatched` and the name later table entries match against.
//! - A matching entry whose port is not connected emits the throttled (5)
//!   `FileDispatchPortNotConnected` warning instead of asserting.
//! - `ENABLE_DISPATCH` always answers `OK` and emits `FileDispatchState`,
//!   even when no entry is routed to the port.
//!
//! The component has no file I/O of its own; it is `std`-only with the
//! other file services because its port types (`Svc.FileAnnounce`,
//! `Svc.FileDispatch`) are declared by `std`-only modules
//! ([`crate::file_uplink`], [`crate::cmd_sequencer`]).

use crate::cmd_sequencer::FileDispatchPort;
use crate::file_uplink::FileAnnouncePort;
use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, ComponentDispatch, EventGlue, EventThrottle,
    MsgDispatchStatus, OutputPort, PingPort, QueueFullPolicy, async_input_port_adapter,
    component_msg_types, msg,
};
use fprime_config::file_dispatcher::{FILE_DISPATCHER_MAX_TABLE_SIZE, MAX_FILE_DISPATCH_PORTS};
use fprime_config::{
    FW_LOG_STRING_MAX_SIZE, FwEnumStoreType, FwEventIdType, FwIndexType, FwOpcodeType,
    FwQueuePriorityType, FwSizeType,
};
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, Enabled, Endianness, FileNameString, FwDefaultString, LogBuffer,
    LogSeverity, SerBuf, SerBufAny, fw_assert, fw_try,
};

/// Upstream's `Svc::FileDispatcherCfg` module (`FileDispatcherCfg.fpp`):
/// the project-extensible dispatch-port enum.
pub mod file_dispatcher_cfg {
    use fprime_fw::fpp_enum;

    fpp_enum! {
        /// `FileDispatcherCfg.FileDispatchPort` — maps file types to
        /// `fileDispatch` port numbers. Projects add their own types before
        /// the sentinel (raising
        /// `fprime_config::file_dispatcher::MAX_FILE_DISPATCH_PORTS` with
        /// it).
        pub enum FileDispatchPort : u8 {
            /// Sequence file port.
            SequenceFilePort = 0,
            /// Bundle file port.
            BundleFilePort = 1,
            /// User file port (make up your own file types starting here).
            UserFilePort = 2,
            /// Maximum number of file dispatch ports (the sentinel; not a
            /// valid table entry).
            MaxFileDispatchPorts = 3,
        }
        default SequenceFilePort
    }

    impl FileDispatchPort {
        /// The C++ enumerator name (for event text).
        #[must_use]
        pub fn name(self) -> &'static str {
            match self {
                Self::SequenceFilePort => "SEQUENCE_FILE_PORT",
                Self::BundleFilePort => "BUNDLE_FILE_PORT",
                Self::UserFilePort => "USER_FILE_PORT",
                Self::MaxFileDispatchPorts => "MAX_FILE_DISPATCH_PORTS",
            }
        }
    }
}

use file_dispatcher_cfg::FileDispatchPort as DispatchPortType;

// The sentinel and the configured port-array size must agree.
const _: () = assert!(DispatchPortType::MaxFileDispatchPorts as usize == MAX_FILE_DISPATCH_PORTS);

/// One dispatch-table row (C++ `FileDispatcherEntry`).
#[derive(Debug, Clone, Default)]
pub struct FileDispatcherEntry {
    /// File extension (suffix) to match; must be non-empty.
    pub file_ext: FwDefaultString,
    /// Port to dispatch to; must be below the sentinel.
    pub port: DispatchPortType,
    /// Whether dispatching is enabled for this entry.
    pub enabled: bool,
}

/// The dispatch table (C++ `FileDispatcherTable`): the first `num_entries`
/// rows of `entries` are used.
#[derive(Debug, Clone)]
pub struct FileDispatcherTable {
    /// The rows.
    pub entries: [FileDispatcherEntry; FILE_DISPATCHER_MAX_TABLE_SIZE],
    /// Number of rows in use (at most `FILE_DISPATCHER_MAX_TABLE_SIZE`).
    pub num_entries: FwSizeType,
}

impl Default for FileDispatcherTable {
    fn default() -> Self {
        Self::new()
    }
}

impl FileDispatcherTable {
    /// An empty table: no rows in use, every row disabled.
    #[must_use]
    pub fn new() -> Self {
        Self {
            entries: core::array::from_fn(|_| FileDispatcherEntry::default()),
            num_entries: 0,
        }
    }
}

/// Queue message size: the largest async invocation.
///
/// `fileAnnounceRecv` = 6 (envelope) + 2 + 240 (`FileNameString`) = 248; a
/// command message is 6 + 4 + 4 + 2 + 506 = 522.
pub const QUEUE_MSG_SIZE: usize = 522;

/// Queue priority for every async input (no FPP `priority` qualifier).
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

/// `Svc::FileDispatcher`.
pub struct FileDispatcher {
    /// Active core: PassiveBase + queue + task.
    pub active: ActiveBase,
    /// Command registration + response ports (`cmdRegOut`, `cmdResponseOut`).
    pub cmd: CmdGlue,
    /// Event ports (`logOut`, `logTextOut`) and the `timeCaller` port.
    pub evt: EventGlue,
    /// `fileDispatch: [MAX_FILE_DISPATCH_PORTS] Svc.FileDispatch`.
    pub file_dispatch: [OutputPort<dyn FileDispatchPort>; MAX_FILE_DISPATCH_PORTS],
    /// `pingOut: Svc.Ping`.
    pub ping_out: OutputPort<dyn PingPort>,
    /// FPP `throttle 5` on `FileDispatchPortNotConnected`.
    port_not_connected_throttle: EventThrottle,
    /// C++ `m_dispatchTable`.
    table: Mutex<FileDispatcherTable>,
}

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel), in the C++ generated
    /// order.
    impl FileDispatcher {
        /// `fileAnnounceRecv` async input port.
        MSG_TYPE_FILE_ANNOUNCE_RECV,
        /// `pingIn` async input port.
        MSG_TYPE_PING_IN,
        /// `ENABLE_DISPATCH` async command.
        MSG_TYPE_CMD_ENABLE_DISPATCH,
    }
}

impl FileDispatcher {
    /// Command `ENABLE_DISPATCH(file_type, enable)` — opcode 0x00, async.
    pub const OPCODE_ENABLE_DISPATCH: FwOpcodeType = 0x00;

    /// `FileDispatchState(file_type, enabled)` — ACTIVITY_HI.
    pub const EVENTID_FILE_DISPATCH_STATE: FwEventIdType = 0x00;
    /// `FileDispatched(file_name: string size 240, file_type)` — ACTIVITY_HI.
    pub const EVENTID_FILE_DISPATCHED: FwEventIdType = 0x01;
    /// `FileDispatchPortNotConnected(file_name, file_type)` — WARNING_LO,
    /// `throttle 5`.
    pub const EVENTID_FILE_DISPATCH_PORT_NOT_CONNECTED: FwEventIdType = 0x02;
    /// FPP `throttle 5` on `FileDispatchPortNotConnected`.
    pub const FILE_DISPATCH_PORT_NOT_CONNECTED_THROTTLE: u32 = 5;

    /// Construct the component with an empty table (C++ constructor: every
    /// row disabled, `numEntries = 0`).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            file_dispatch: core::array::from_fn(|_| OutputPort::new()),
            ping_out: OutputPort::new(),
            port_not_connected_throttle: EventThrottle::new(
                Self::FILE_DISPATCH_PORT_NOT_CONNECTED_THROTTLE,
            ),
            table: Mutex::new(FileDispatcherTable::new()),
        })
    }

    /// C++ `configure(table)`: copy the first `num_entries` rows. Asserts,
    /// as C++ does, that the row count fits, that every port is below the
    /// sentinel and that every extension is non-empty.
    pub fn configure(&self, table: &FileDispatcherTable) {
        fw_assert!(
            table.num_entries <= FILE_DISPATCHER_MAX_TABLE_SIZE as FwSizeType,
            table.num_entries as i32
        );
        let mut current = self.table.lock();
        for (dest, entry) in current
            .entries
            .iter_mut()
            .zip(table.entries.iter())
            .take(table.num_entries as usize)
        {
            fw_assert!(
                (entry.port as usize) < MAX_FILE_DISPATCH_PORTS,
                entry.port as i32
            );
            fw_assert!(!entry.file_ext.is_empty(), entry.file_ext.len() as i32);
            dest.port = entry.port;
            dest.file_ext = entry.file_ext.clone();
            dest.enabled = entry.enabled;
        }
        current.num_entries = table.num_entries;
    }

    /// A copy of the current dispatch table.
    #[must_use]
    pub fn table(&self) -> FileDispatcherTable {
        self.table.lock().clone()
    }

    fn id_base(&self) -> u32 {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd
            .reg_commands(self.id_base(), &[Self::OPCODE_ENABLE_DISPATCH]);
    }

    /// Create the component queue (topology phase 4).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    // -- Handlers (component thread) -----------------------------------------

    /// `fileAnnounceRecv_handler`.
    fn file_announce_recv_handler(&self, _port_num: FwIndexType, file_name: &mut FileNameString) {
        // The table is copied out so no lock is held across the output
        // ports (a synchronous downstream handler may call back in).
        let table = self.table.lock().clone();
        fw_assert!(
            table.num_entries <= FILE_DISPATCHER_MAX_TABLE_SIZE as FwSizeType,
            table.num_entries as i32
        );
        for entry in table.entries.iter().take(table.num_entries as usize) {
            if !entry.enabled {
                continue;
            }
            // C++ parity: substring_find_last(name, ext) == len(name) -
            // len(ext), which for the non-empty extensions configure()
            // admits is exactly a suffix match.
            if !file_name.as_bytes().ends_with(entry.file_ext.as_bytes()) {
                continue;
            }
            let port = &self.file_dispatch[entry.port as usize];
            match port.try_get() {
                Some(p) => {
                    p.target.invoke(p.port_num, file_name);
                    self.log_file_dispatched(file_name, entry.port);
                }
                None => {
                    if self.port_not_connected_throttle.ok_to_emit() {
                        self.log_file_dispatch_port_not_connected(file_name, entry.port);
                    }
                }
            }
        }
    }

    /// `pingIn_handler` — echo the key.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }

    /// `cmdIn` dispatch (only `ENABLE_DISPATCH` exists).
    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_ENABLE_DISPATCH => {
                self.enable_dispatch_cmd_handler(op_code, cmd_seq, args)
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }

    /// `ENABLE_DISPATCH_cmdHandler`: set `enabled` on every row routed to
    /// `file_type`, then always `FileDispatchState` + `OK`.
    fn enable_dispatch_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut raw_type = 0u8;
        let mut raw_enable = 0u8;
        if !args.deserialize_u8_be(&mut raw_type).is_ok()
            || !args.deserialize_u8_be(&mut raw_enable).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        // Workspace convention: an undeclared enum value answers
        // VALIDATION_ERROR (the generated C++ folds it into FORMAT_ERROR).
        // The sentinel MAX_FILE_DISPATCH_PORTS is a declared value, so it
        // is accepted as in C++ (and matches no row).
        let (Ok(file_type), Ok(enable)) = (
            DispatchPortType::try_from(raw_type),
            Enabled::try_from(raw_enable),
        ) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        {
            let mut table = self.table.lock();
            fw_assert!(
                table.num_entries <= FILE_DISPATCHER_MAX_TABLE_SIZE as FwSizeType,
                table.num_entries as i32
            );
            let count = table.num_entries as usize;
            for entry in table.entries.iter_mut().take(count) {
                if entry.port == file_type {
                    entry.enabled = enable == Enabled::Enabled;
                }
            }
        }
        self.log_file_dispatch_state(file_type, enable);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    // -- Events ---------------------------------------------------------------

    fn log_file_dispatch_state(&self, file_type: DispatchPortType, enabled: Enabled) {
        let enabled_text = match enabled {
            Enabled::Enabled => "ENABLED",
            Enabled::Disabled => "DISABLED",
        };
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_DISPATCH_STATE,
            LogSeverity::ActivityHi,
            &format!(
                "File dispatch {} state changed: to {enabled_text}",
                file_type.name()
            ),
            |buf: &mut LogBuffer| {
                fw_try!(buf.serialize_u8_be(file_type as u8));
                buf.serialize_u8_be(enabled as u8)
            },
        );
    }

    fn log_file_dispatched(&self, file_name: &FileNameString, file_type: DispatchPortType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_DISPATCHED,
            LogSeverity::ActivityHi,
            &format!("File {file_name} dispatched to {}", file_type.name()),
            |buf: &mut LogBuffer| {
                fw_try!(file_name.serialize_to_truncated(
                    buf,
                    FW_LOG_STRING_MAX_SIZE,
                    Endianness::Big
                ));
                buf.serialize_u8_be(file_type as u8)
            },
        );
    }

    fn log_file_dispatch_port_not_connected(
        &self,
        file_name: &FileNameString,
        file_type: DispatchPortType,
    ) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_FILE_DISPATCH_PORT_NOT_CONNECTED,
            LogSeverity::WarningLo,
            &format!(
                "File {file_name} matched {} but the dispatch port is not connected",
                file_type.name()
            ),
            |buf: &mut LogBuffer| {
                fw_try!(file_name.serialize_to_truncated(
                    buf,
                    FW_LOG_STRING_MAX_SIZE,
                    Endianness::Big
                ));
                buf.serialize_u8_be(file_type as u8)
            },
        );
    }
}

// -- Async input adapters ----------------------------------------------------

async_input_port_adapter! {
    /// `fileAnnounceRecv` — ASYNC `Svc.FileAnnounce` input (C++ `ref` file
    /// name: copied into the message, never written back).
    component: FileDispatcher;
    adapter: FileAnnounceRecvAdapter;
    port: FileAnnouncePort;
    input: pub file_announce_recv;
    deserialize: file_announce_recv_deserialize;
    handler: file_announce_recv_handler;
    base: active.queued;
    msg_type: FileDispatcher::MSG_TYPE_FILE_ANNOUNCE_RECV;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { mut file_name: FileNameString }
}

async_input_port_adapter! {
    /// `pingIn` — ASYNC `Svc.Ping` input with the `drop` policy.
    component: FileDispatcher;
    adapter: PingInAdapter;
    port: PingPort;
    input: pub ping_in;
    deserialize: ping_in_deserialize;
    handler: ping_in_handler;
    base: active.queued;
    msg_type: FileDispatcher::MSG_TYPE_PING_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Drop;
    args { val key: u32 }
}

async_input_port_adapter! {
    /// `cmdIn` — ASYNC `Fw.Cmd` input for `ENABLE_DISPATCH`.
    component: FileDispatcher;
    adapter: CmdInAdapter;
    port: CmdPort;
    input: pub cmd_in;
    deserialize: cmd_in_deserialize;
    handler: cmd_in_handler;
    base: active.queued;
    msg_type: FileDispatcher::MSG_TYPE_CMD_ENABLE_DISPATCH;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, buf args: CmdArgBuffer }
}

// -- Dispatch (the generated `doDispatch` switch) -----------------------------

impl ComponentDispatch for FileDispatcher {
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
            Self::MSG_TYPE_FILE_ANNOUNCE_RECV => match Self::file_announce_recv_deserialize(buf) {
                Some((mut file_name,)) => {
                    self.file_announce_recv_handler(port_num, &mut file_name);
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
            Self::MSG_TYPE_CMD_ENABLE_DISPATCH => match Self::cmd_in_deserialize(buf) {
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

impl ActiveComponent for FileDispatcher {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort};
    use fprime_fw::{TextLogString, Time};
    use fprime_os::queue::BlockingType;
    use std::sync::Mutex as StdMutex;

    const ID_BASE: u32 = 0x4400;

    /// (id, severity, raw arg bytes)
    type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

    #[derive(Default)]
    struct Ground {
        events: StdMutex<Vec<EventRecord>>,
        text: StdMutex<Vec<String>>,
        regs: StdMutex<Vec<FwOpcodeType>>,
        responses: StdMutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        pings: StdMutex<Vec<u32>>,
    }

    impl Ground {
        fn clear(&self) {
            self.events.lock().unwrap().clear();
            self.text.lock().unwrap().clear();
            self.responses.lock().unwrap().clear();
            self.pings.lock().unwrap().clear();
        }
        fn events_with(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .filter(|e| e.0 == ID_BASE + id)
                .map(|e| e.2.clone())
                .collect()
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

    impl LogTextPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            _id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            text: &mut TextLogString,
        ) {
            self.text
                .lock()
                .unwrap()
                .push(text.as_str().unwrap_or_default().to_string());
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

    /// Records `fileDispatch` calls as (output port index, file name).
    struct DispatchSink {
        index: usize,
        calls: Arc<StdMutex<Vec<(usize, String)>>>,
    }

    impl FileDispatchPort for DispatchSink {
        fn invoke(&self, _port_num: FwIndexType, file_name: &mut FileNameString) {
            self.calls
                .lock()
                .unwrap()
                .push((self.index, file_name.as_str().unwrap().to_string()));
        }
    }

    struct Harness {
        comp: Arc<FileDispatcher>,
        ground: Arc<Ground>,
        dispatched: Arc<StdMutex<Vec<(usize, String)>>>,
    }

    impl Harness {
        /// Upstream `connectPorts()`: every `fileDispatch` port connected.
        fn new() -> Self {
            Self::with_connected(&[0, 1, 2])
        }

        fn with_connected(ports: &[usize]) -> Self {
            let comp = FileDispatcher::new("FileDispatcher");
            let ground = Arc::new(Ground::default());
            let dispatched = Arc::new(StdMutex::new(Vec::new()));
            comp.active.queued.base.set_id_base(ID_BASE);
            comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
            comp.cmd.cmd_response_out.connect(ground.clone(), 0);
            comp.evt.log_out.connect(ground.clone(), 0);
            comp.evt.text_log_out.connect(ground.clone(), 0);
            comp.ping_out.connect(ground.clone(), 0);
            for &index in ports {
                comp.file_dispatch[index].connect(
                    Arc::new(DispatchSink {
                        index,
                        calls: dispatched.clone(),
                    }),
                    0,
                );
            }
            comp.init(10);
            Self {
                comp,
                ground,
                dispatched,
            }
        }

        fn clear_history(&self) {
            self.ground.clear();
            self.dispatched.lock().unwrap().clear();
        }

        fn do_dispatch(&self) {
            assert_eq!(
                self.comp
                    .active
                    .queued
                    .do_dispatch(self.comp.as_ref(), BlockingType::NonBlocking),
                MsgDispatchStatus::Ok
            );
        }

        fn announce(&self, name: &str) {
            let mut file_name = FileNameString::from(name);
            let port = self.comp.file_announce_recv(0);
            port.target.invoke(port.port_num, &mut file_name);
            self.do_dispatch();
        }

        fn send_cmd(&self, cmd_seq: u32, arg_bytes: &[u8]) {
            let mut args = CmdArgBuffer::new();
            assert!(args.set_buff(arg_bytes).is_ok());
            let port = self.comp.cmd_in(0);
            port.target.invoke(
                port.port_num,
                ID_BASE + FileDispatcher::OPCODE_ENABLE_DISPATCH,
                cmd_seq,
                &mut args,
            );
            self.do_dispatch();
        }

        /// Upstream `populateTable`: ten rows `.file<i>` routed to port
        /// `i % MAX_FILE_DISPATCH_PORTS`.
        fn populate_table(&self, enabled: bool) {
            let mut table = FileDispatcherTable::new();
            table.num_entries = FILE_DISPATCHER_MAX_TABLE_SIZE as FwSizeType;
            for (i, entry) in table.entries.iter_mut().enumerate() {
                entry.enabled = enabled;
                entry.file_ext.set(&format!(".file{i}"));
                entry.port = port_for(i);
            }
            self.comp.configure(&table);
        }
    }

    fn port_for(i: usize) -> DispatchPortType {
        DispatchPortType::try_from((i % MAX_FILE_DISPATCH_PORTS) as u8).unwrap()
    }

    /// The `FileDispatched` argument bytes for (name, port).
    fn dispatched_args(name: &str, port: DispatchPortType) -> Vec<u8> {
        let mut bytes = (name.len() as u16).to_be_bytes().to_vec();
        bytes.extend_from_slice(name.as_bytes());
        bytes.push(port as u8);
        bytes
    }

    /// Port of `dispatchTest`.
    #[test]
    fn dispatch_test() {
        let h = Harness::new();
        h.populate_table(true);
        for i in 0..FILE_DISPATCHER_MAX_TABLE_SIZE {
            h.clear_history();
            let name = format!("afile.file{}", i % MAX_FILE_DISPATCH_PORTS);
            h.announce(&name);
            let events = h
                .ground
                .events_with(FileDispatcher::EVENTID_FILE_DISPATCHED);
            assert_eq!(events, vec![dispatched_args(&name, port_for(i))]);
            assert_eq!(
                *h.dispatched.lock().unwrap(),
                vec![(i % MAX_FILE_DISPATCH_PORTS, name.clone())]
            );
        }
    }

    /// Port of `dispatchMaxLengthNameTest`.
    #[test]
    fn dispatch_max_length_name_test() {
        let h = Harness::new();
        h.populate_table(true);
        let ext = ".file0";
        let name = format!(
            "{}{ext}",
            "a".repeat(FileNameString::max_length() - ext.len())
        );
        assert_eq!(name.len(), FileNameString::max_length());
        h.announce(&name);
        // The full-length name round-trips through the port without
        // truncation.
        assert_eq!(*h.dispatched.lock().unwrap(), vec![(0, name.clone())]);
        // The event string is cut at FW_LOG_STRING_MAX_SIZE (200).
        let events = h
            .ground
            .events_with(FileDispatcher::EVENTID_FILE_DISPATCHED);
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0],
            dispatched_args(
                &name[..FW_LOG_STRING_MAX_SIZE],
                DispatchPortType::SequenceFilePort
            )
        );
    }

    /// Port of `dispatchAllDisabledTest`.
    #[test]
    fn dispatch_all_disabled_test() {
        let h = Harness::new();
        h.populate_table(false);
        for i in 0..FILE_DISPATCHER_MAX_TABLE_SIZE {
            h.clear_history();
            h.announce(&format!("afile.file{}", i % MAX_FILE_DISPATCH_PORTS));
            assert!(h.ground.events.lock().unwrap().is_empty());
            assert!(h.dispatched.lock().unwrap().is_empty());
        }
    }

    fn enable_dispatch_all(h: &Harness, enable: Enabled) {
        for i in 0..MAX_FILE_DISPATCH_PORTS {
            h.clear_history();
            h.send_cmd(0x10, &[port_for(i) as u8, enable as u8]);
            assert_eq!(
                h.ground
                    .events_with(FileDispatcher::EVENTID_FILE_DISPATCH_STATE),
                vec![vec![port_for(i) as u8, enable as u8]]
            );
            assert_eq!(
                *h.ground.responses.lock().unwrap(),
                vec![(
                    ID_BASE + FileDispatcher::OPCODE_ENABLE_DISPATCH,
                    0x10,
                    CmdResponse::Ok
                )]
            );
        }
    }

    /// Port of `dispatchAllCmdDisabledTest`.
    #[test]
    fn dispatch_all_cmd_disabled_test() {
        let h = Harness::new();
        h.populate_table(true);
        enable_dispatch_all(&h, Enabled::Disabled);
        for i in 0..FILE_DISPATCHER_MAX_TABLE_SIZE {
            h.clear_history();
            h.announce(&format!("afile.file{}", i % MAX_FILE_DISPATCH_PORTS));
            assert!(h.ground.events.lock().unwrap().is_empty());
            assert!(h.dispatched.lock().unwrap().is_empty());
        }
    }

    /// Port of `dispatchAllCmdEnabledTest`.
    #[test]
    fn dispatch_all_cmd_enabled_test() {
        let h = Harness::new();
        h.populate_table(false);
        enable_dispatch_all(&h, Enabled::Enabled);
        for i in 0..FILE_DISPATCHER_MAX_TABLE_SIZE {
            h.clear_history();
            let name = format!("afile.file{}", i % MAX_FILE_DISPATCH_PORTS);
            h.announce(&name);
            assert_eq!(
                h.ground
                    .events_with(FileDispatcher::EVENTID_FILE_DISPATCHED),
                vec![dispatched_args(&name, port_for(i))]
            );
            assert_eq!(h.dispatched.lock().unwrap().len(), 1);
        }
    }

    /// Port of `dispatchPingTest`.
    #[test]
    fn dispatch_ping_test() {
        let h = Harness::new();
        let port = h.comp.ping_in(0);
        port.target.invoke(port.port_num, 0x1234);
        h.do_dispatch();
        assert_eq!(*h.ground.pings.lock().unwrap(), vec![0x1234]);
    }

    #[test]
    fn every_matching_row_dispatches_including_duplicates() {
        let h = Harness::new();
        let mut table = FileDispatcherTable::new();
        table.num_entries = 4;
        let rows = [
            (".seq", DispatchPortType::SequenceFilePort),
            ("seq", DispatchPortType::UserFilePort),
            (".bin", DispatchPortType::BundleFilePort),
            (".seq", DispatchPortType::SequenceFilePort),
        ];
        for (entry, (ext, port)) in table.entries.iter_mut().zip(rows) {
            entry.file_ext.set(ext);
            entry.port = port;
            entry.enabled = true;
        }
        h.comp.configure(&table);
        h.announce("dir/run.seq");
        assert_eq!(
            *h.dispatched.lock().unwrap(),
            vec![
                (0, "dir/run.seq".to_string()),
                (2, "dir/run.seq".to_string()),
                (0, "dir/run.seq".to_string()),
            ]
        );
        assert_eq!(
            h.ground
                .events_with(FileDispatcher::EVENTID_FILE_DISPATCHED)
                .len(),
            3
        );
    }

    #[test]
    fn the_extension_must_end_the_name() {
        let h = Harness::new();
        h.populate_table(true);
        // ".file1" occurs, but not at the end; a name shorter than every
        // extension matches nothing either.
        h.announce("a.file1.txt");
        h.announce("x");
        h.announce("");
        assert!(h.dispatched.lock().unwrap().is_empty());
        assert!(h.ground.events.lock().unwrap().is_empty());
        // The last occurrence is the one at the end.
        h.announce("a.file1.file1");
        assert_eq!(
            *h.dispatched.lock().unwrap(),
            vec![(1, "a.file1.file1".to_string())]
        );
    }

    #[test]
    fn unconnected_port_warns_with_a_throttle_of_five() {
        let h = Harness::with_connected(&[0, 2]);
        h.populate_table(true);
        for _ in 0..7 {
            h.announce("b.file1");
        }
        assert!(h.dispatched.lock().unwrap().is_empty());
        let warnings = h
            .ground
            .events_with(FileDispatcher::EVENTID_FILE_DISPATCH_PORT_NOT_CONNECTED);
        assert_eq!(
            warnings.len(),
            FileDispatcher::FILE_DISPATCH_PORT_NOT_CONNECTED_THROTTLE as usize
        );
        assert_eq!(
            warnings[0],
            dispatched_args("b.file1", DispatchPortType::BundleFilePort)
        );
        assert_eq!(h.ground.events.lock().unwrap()[0].1, LogSeverity::WarningLo);
    }

    #[test]
    fn enable_dispatch_answers_ok_for_an_unrouted_port_and_checks_arguments() {
        let h = Harness::new();
        h.comp.reg_commands();
        assert_eq!(
            *h.ground.regs.lock().unwrap(),
            vec![ID_BASE + FileDispatcher::OPCODE_ENABLE_DISPATCH]
        );
        // Empty table, and the sentinel itself: still OK + the event.
        h.send_cmd(1, &[DispatchPortType::MaxFileDispatchPorts as u8, 1]);
        h.send_cmd(2, &[DispatchPortType::UserFilePort as u8]); // short
        h.send_cmd(3, &[0, 1, 0]); // residual byte
        h.send_cmd(4, &[9, 1]); // undeclared file type
        h.send_cmd(5, &[0, 2]); // undeclared Enabled value
        let opcode = ID_BASE + FileDispatcher::OPCODE_ENABLE_DISPATCH;
        assert_eq!(
            *h.ground.responses.lock().unwrap(),
            vec![
                (opcode, 1, CmdResponse::Ok),
                (opcode, 2, CmdResponse::FormatError),
                (opcode, 3, CmdResponse::FormatError),
                (opcode, 4, CmdResponse::ValidationError),
                (opcode, 5, CmdResponse::ValidationError),
            ]
        );
        assert_eq!(
            h.ground
                .events_with(FileDispatcher::EVENTID_FILE_DISPATCH_STATE),
            vec![vec![3, 1]]
        );
        assert_eq!(
            h.ground.text.lock().unwrap()[0],
            "File dispatch MAX_FILE_DISPATCH_PORTS state changed: to ENABLED"
        );
    }

    #[test]
    fn enable_dispatch_only_touches_rows_of_that_port() {
        let h = Harness::new();
        h.populate_table(true);
        h.send_cmd(1, &[DispatchPortType::BundleFilePort as u8, 0]);
        let table = h.comp.table();
        for (i, entry) in table.entries.iter().enumerate() {
            assert_eq!(entry.enabled, i % MAX_FILE_DISPATCH_PORTS != 1, "row {i}");
        }
    }

    #[test]
    fn configure_copies_only_the_rows_in_use() {
        let h = Harness::new();
        let mut table = FileDispatcherTable::new();
        table.num_entries = 1;
        table.entries[0].file_ext.set(".a");
        table.entries[0].enabled = true;
        table.entries[1].file_ext.set(".b");
        table.entries[1].enabled = true;
        h.comp.configure(&table);
        let copy = h.comp.table();
        assert_eq!(copy.num_entries, 1);
        assert_eq!(copy.entries[0].file_ext.as_bytes(), b".a");
        assert!(copy.entries[1].file_ext.is_empty());
        assert!(!copy.entries[1].enabled);
    }

    #[test]
    #[should_panic]
    fn configure_asserts_on_too_many_rows() {
        let h = Harness::new();
        let mut table = FileDispatcherTable::new();
        table.num_entries = FILE_DISPATCHER_MAX_TABLE_SIZE as FwSizeType + 1;
        h.comp.configure(&table);
    }

    #[test]
    #[should_panic]
    fn configure_asserts_on_the_sentinel_port() {
        let h = Harness::new();
        let mut table = FileDispatcherTable::new();
        table.num_entries = 1;
        table.entries[0].file_ext.set(".x");
        table.entries[0].port = DispatchPortType::MaxFileDispatchPorts;
        h.comp.configure(&table);
    }

    #[test]
    #[should_panic]
    fn configure_asserts_on_an_empty_extension() {
        let h = Harness::new();
        let mut table = FileDispatcherTable::new();
        table.num_entries = 1;
        h.comp.configure(&table);
    }

    #[test]
    fn a_synchronous_handler_can_rewrite_the_name_for_later_rows() {
        // C++ parity: the name goes out by reference.
        struct Renamer;
        impl FileDispatchPort for Renamer {
            fn invoke(&self, _port_num: FwIndexType, file_name: &mut FileNameString) {
                file_name.set("renamed.bin");
            }
        }
        let comp = FileDispatcher::new("FileDispatcher");
        let ground = Arc::new(Ground::default());
        let calls = Arc::new(StdMutex::new(Vec::new()));
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.file_dispatch[0].connect(Arc::new(Renamer), 0);
        comp.file_dispatch[1].connect(
            Arc::new(DispatchSink {
                index: 1,
                calls: calls.clone(),
            }),
            0,
        );
        comp.init(4);
        let mut table = FileDispatcherTable::new();
        table.num_entries = 2;
        table.entries[0].file_ext.set(".seq");
        table.entries[0].enabled = true;
        table.entries[1].file_ext.set(".bin");
        table.entries[1].port = DispatchPortType::BundleFilePort;
        table.entries[1].enabled = true;
        comp.configure(&table);
        let mut name = FileNameString::from("a.seq");
        let port = comp.file_announce_recv(0);
        port.target.invoke(port.port_num, &mut name);
        assert_eq!(
            comp.active
                .queued
                .do_dispatch(comp.as_ref(), BlockingType::NonBlocking),
            MsgDispatchStatus::Ok
        );
        assert_eq!(*calls.lock().unwrap(), vec![(1, "renamed.bin".to_string())]);
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 2);
        assert_eq!(
            events[0].2,
            dispatched_args("renamed.bin", DispatchPortType::SequenceFilePort)
        );
    }

    #[test]
    fn file_announce_envelope_bytes_are_byte_exact() {
        let h = Harness::new();
        let mut name = FileNameString::from("ab");
        let port = h.comp.file_announce_recv(0);
        port.target.invoke(port.port_num, &mut name);
        let mut dest = [0u8; QUEUE_MSG_SIZE];
        let mut size: FwSizeType = 0;
        let mut priority: FwQueuePriorityType = 0;
        let status = h.comp.active.queued.queue().receive(
            &mut dest,
            BlockingType::NonBlocking,
            &mut size,
            &mut priority,
        );
        assert_eq!(status, fprime_os::queue::Status::OpOk);
        assert_eq!(
            dest[..size as usize].to_vec(),
            vec![0, 0, 0, 1, 0, 0, 0, 2, b'a', b'b']
        );
    }

    #[test]
    fn dictionary_ids_match_the_fpp_model() {
        assert_eq!(FileDispatcher::OPCODE_ENABLE_DISPATCH, 0x00);
        assert_eq!(FileDispatcher::EVENTID_FILE_DISPATCH_STATE, 0x00);
        assert_eq!(FileDispatcher::EVENTID_FILE_DISPATCHED, 0x01);
        assert_eq!(
            FileDispatcher::EVENTID_FILE_DISPATCH_PORT_NOT_CONNECTED,
            0x02
        );
        assert_eq!(FileDispatcher::FILE_DISPATCH_PORT_NOT_CONNECTED_THROTTLE, 5);
        assert_eq!(FileDispatcher::MSG_TYPE_FILE_ANNOUNCE_RECV, 1);
        assert_eq!(FileDispatcher::MSG_TYPE_PING_IN, 2);
        assert_eq!(FileDispatcher::MSG_TYPE_CMD_ENABLE_DISPATCH, 3);
        assert_eq!(DispatchPortType::SequenceFilePort as u8, 0);
        assert_eq!(DispatchPortType::BundleFilePort as u8, 1);
        assert_eq!(DispatchPortType::UserFilePort as u8, 2);
        assert_eq!(DispatchPortType::MaxFileDispatchPorts as u8, 3);
        assert_eq!(DispatchPortType::SERIALIZED_SIZE, 1);
    }
}
