//! # Version — port of `Svc::Version` (passive, guarded)
//!
//! C++ sources: `Svc/Version/Version.{cpp,hpp,fpp}`,
//! `Svc/Ports/VersionPorts/VersionPorts.fpp` (`Svc.Version` port,
//! `Svc.VersionStatus`), `default/config/VersionCfg.fpp`
//! (`Svc.VersionCfg.VersionEnum`), `Svc/Version/docs/sdd.md`.
//!
//! Reports the framework, project and library versions as events and
//! telemetry — at startup ([`Version::start`]) and on the `VERSION`
//! command — and keeps a table of custom versions other components set and
//! get through the `setVersion` / `getVersion` ports (hardware or FPGA
//! versions, say). Custom versions always produce a `CustomVersions` event
//! when set; their `CustomVersionNN` channels are written only while
//! verbosity is enabled ([`Version::config`] or the `ENABLE` command).
//!
//! ## The version strings
//!
//! Upstream's build generates `versions/version.hpp` (`git describe` of the
//! F Prime and project checkouts, one `"<lib> @ <version>"` per library).
//! Here they come from [`fprime_config::version`] — by default
//! [`FRAMEWORK_VERSION`] is the pinned upstream F Prime commit this port
//! tracks (the short hash in `FPRIME_UPSTREAM_COMMIT`), [`PROJECT_VERSION`]
//! is the workspace crate version, and [`LIBRARY_VERSIONS`] is empty — the
//! same three values the ground dictionary's `metadata` carries. A
//! deployment can override them per instance with
//! [`Version::with_versions`].
//!
//! ## Ported quirks (each covered by a test)
//!
//! - The custom-element counter is a `U8` bumped on every `setVersion`, so
//!   it wraps: the 256th set leaves it at 0 and that set emits nothing.
//! - `VERSION CUSTOM` (and `ALL`) emit custom versions only while
//!   verbosity is enabled — even their events — although `setVersion`
//!   emits the event regardless.
//! - A slot whose value is the literal `"no_ver"` (the initial value) is
//!   never reported.
//! - Library versions beyond the tenth get an event but no channel.
//!
//! Threading: both ports and both commands are guarded by the one
//! component mutex, held across the event and telemetry calls as in C++.
//! [`Version::config`] and [`Version::start`] are init-time calls.

use alloc::format;
use fprime_comp::{CmdGlue, CmdPort, EventGlue, PassiveBase, PortRef, TlmGlue, input_port_adapter};
use fprime_config::{FwChanIdType, FwEventIdType, FwIdType, FwIndexType, FwOpcodeType};
use fprime_fw::sync::{Arc, Mutex, MutexGuard};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, Endianness, FwDefaultString, FwString, LogBuffer, LogSeverity,
    LogStringArg, SerBuf, TlmString, fpp_enum, fpp_struct, fw_assert, fw_try,
};

pub use fprime_config::version::{
    FRAMEWORK_VERSION, LIBRARY_VERSIONS, NUM_VERSION_ENUMS, PROJECT_VERSION,
};

// ---------------------------------------------------------------------------
// Types (VersionPorts.fpp, VersionCfg.fpp, Version.fpp)
// ---------------------------------------------------------------------------

fpp_enum! {
    /// `Svc.VersionStatus` — status of a version get/set.
    pub enum VersionStatus : u8 {
        /// Version was good.
        Ok = 0,
        /// Failure to get version.
        Failure = 1,
    }
    default Ok
}

fpp_enum! {
    /// `Svc.VersionCfg.VersionEnum` (`VersionCfg.fpp`): the custom-version
    /// slots, defined per project.
    pub enum VersionEnum : u32 {
        /// Entry 0.
        ProjectVersion00 = 0,
        /// Entry 1.
        ProjectVersion01 = 1,
        /// Entry 2.
        ProjectVersion02 = 2,
        /// Entry 3.
        ProjectVersion03 = 3,
        /// Entry 4.
        ProjectVersion04 = 4,
        /// Entry 5.
        ProjectVersion05 = 5,
        /// Entry 6.
        ProjectVersion06 = 6,
        /// Entry 7.
        ProjectVersion07 = 7,
        /// Entry 8.
        ProjectVersion08 = 8,
        /// Entry 9.
        ProjectVersion09 = 9,
    }
    default ProjectVersion00
}

// The slot count is configured in `fprime-config`; keep the two in step.
const _: () = assert!(VersionEnum::NUM_CONSTANTS == NUM_VERSION_ENUMS);

fpp_enum! {
    /// `Svc.VersionEnabled` — custom-version telemetry verbosity.
    pub enum VersionEnabled : u8 {
        /// Verbosity disabled.
        Disabled = 0,
        /// Verbosity enabled.
        Enabled = 1,
    }
    default Disabled
}

fpp_enum! {
    /// `Svc.VersionType` — which versions the `VERSION` command reports.
    pub enum VersionType : u8 {
        /// Project version.
        Project = 0,
        /// Framework version.
        Framework = 1,
        /// Library versions.
        Library = 2,
        /// Custom versions.
        Custom = 3,
        /// All of the above.
        All = 4,
    }
    default Project
}

/// `version_value: string size 80` of [`CustomVersionDb`].
pub type CustomVersionString = FwString<80>;

fpp_struct! {
    /// `Svc.CustomVersionDb` — one custom version, the value of the
    /// `CustomVersionNN` channels. Wire format: `[enum u32][u16 len + up to
    /// 80 bytes][status u8]`.
    #[derive(Clone)]
    pub struct CustomVersionDb {
        /// Enumeration/name of the custom version.
        version_enum: VersionEnum { get_version_enum, set_version_enum },
        /// The custom version string.
        version_value: CustomVersionString { get_version_value, set_version_value },
        /// Status of the custom version.
        version_status: VersionStatus { get_version_status, set_version_status },
    }
}

/// `Svc.Version` — set or get a custom version. `version_string` and
/// `status` are in/out (`ref`): a setter passes them in, a getter receives
/// them. The FPP `string` (no size) is the default-size `Fw::String`.
pub trait VersionPort: Send + Sync {
    /// Invoke the port.
    fn invoke(
        &self,
        port_num: FwIndexType,
        version_id: VersionEnum,
        version_string: &mut FwDefaultString,
        status: &mut VersionStatus,
    );
}

// ---------------------------------------------------------------------------
// The component
// ---------------------------------------------------------------------------

/// Number of `LibraryVersionNN` / `CustomVersionNN` channels (C++
/// `VersionSlot`, `VER_SLOT_00..09`).
pub const NUM_VERSION_SLOTS: usize = 10;

/// `string size 40` of the version events and channels.
const VERSION_STRING_SIZE: usize = 40;

/// The initial value of every custom slot; never reported.
const NO_VERSION: &str = "no_ver";

impl Version {
    /// `ENABLE(enable: VersionEnabled)` — guarded, opcode 0.
    pub const OPCODE_ENABLE: FwOpcodeType = 0;
    /// `VERSION(version_type: VersionType)` — guarded, opcode 1.
    pub const OPCODE_VERSION: FwOpcodeType = 1;

    /// `FrameworkVersion(version: string size 40)` — ACTIVITY_LO.
    pub const EVENTID_FRAMEWORK_VERSION: FwEventIdType = 0;
    /// `ProjectVersion(version: string size 40)` — ACTIVITY_LO.
    pub const EVENTID_PROJECT_VERSION: FwEventIdType = 1;
    /// `LibraryVersions(version: string size 40)` — ACTIVITY_LO.
    pub const EVENTID_LIBRARY_VERSIONS: FwEventIdType = 2;
    /// `CustomVersions(version_enum, version_value: string size 40)` —
    /// ACTIVITY_LO.
    pub const EVENTID_CUSTOM_VERSIONS: FwEventIdType = 3;

    /// `FrameworkVersion: string size 40`.
    pub const CHANID_FRAMEWORK_VERSION: FwChanIdType = 0;
    /// `ProjectVersion: string size 40`.
    pub const CHANID_PROJECT_VERSION: FwChanIdType = 1;
    /// `CustomVersion01: CustomVersionDb` (id 2) — `CustomVersion10` is id
    /// 11; slot `n` is `CHANID_CUSTOM_VERSION_01 + n`.
    pub const CHANID_CUSTOM_VERSION_01: FwChanIdType = 2;
    /// `CustomVersion02`.
    pub const CHANID_CUSTOM_VERSION_02: FwChanIdType = 3;
    /// `CustomVersion03`.
    pub const CHANID_CUSTOM_VERSION_03: FwChanIdType = 4;
    /// `CustomVersion04`.
    pub const CHANID_CUSTOM_VERSION_04: FwChanIdType = 5;
    /// `CustomVersion05`.
    pub const CHANID_CUSTOM_VERSION_05: FwChanIdType = 6;
    /// `CustomVersion06`.
    pub const CHANID_CUSTOM_VERSION_06: FwChanIdType = 7;
    /// `CustomVersion07`.
    pub const CHANID_CUSTOM_VERSION_07: FwChanIdType = 8;
    /// `CustomVersion08`.
    pub const CHANID_CUSTOM_VERSION_08: FwChanIdType = 9;
    /// `CustomVersion09`.
    pub const CHANID_CUSTOM_VERSION_09: FwChanIdType = 10;
    /// `CustomVersion10`.
    pub const CHANID_CUSTOM_VERSION_10: FwChanIdType = 11;
    /// `LibraryVersion01: string size 40` (id 12) — `LibraryVersion10` is
    /// id 21; library `n` is `CHANID_LIBRARY_VERSION_01 + n`.
    pub const CHANID_LIBRARY_VERSION_01: FwChanIdType = 12;
    /// `LibraryVersion02`.
    pub const CHANID_LIBRARY_VERSION_02: FwChanIdType = 13;
    /// `LibraryVersion03`.
    pub const CHANID_LIBRARY_VERSION_03: FwChanIdType = 14;
    /// `LibraryVersion04`.
    pub const CHANID_LIBRARY_VERSION_04: FwChanIdType = 15;
    /// `LibraryVersion05`.
    pub const CHANID_LIBRARY_VERSION_05: FwChanIdType = 16;
    /// `LibraryVersion06`.
    pub const CHANID_LIBRARY_VERSION_06: FwChanIdType = 17;
    /// `LibraryVersion07`.
    pub const CHANID_LIBRARY_VERSION_07: FwChanIdType = 18;
    /// `LibraryVersion08`.
    pub const CHANID_LIBRARY_VERSION_08: FwChanIdType = 19;
    /// `LibraryVersion09`.
    pub const CHANID_LIBRARY_VERSION_09: FwChanIdType = 20;
    /// `LibraryVersion10`.
    pub const CHANID_LIBRARY_VERSION_10: FwChanIdType = 21;
}

/// Guarded state.
struct VersionState {
    /// C++ `m_enable`: write the custom-version channels.
    enable: bool,
    /// C++ `m_num_custom_elements` (a `U8`: wraps).
    num_custom_elements: u8,
    /// C++ `verId_db`.
    db: [CustomVersionDb; NUM_VERSION_ENUMS],
}

/// `Svc::Version`.
pub struct Version {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `cmdRegOut` / `cmdResponseOut`.
    pub cmd: CmdGlue,
    /// `logOut` / `logTextOut` / `timeCaller`.
    pub evt: EventGlue,
    /// `tlmOut`.
    pub tlm: TlmGlue,
    /// `Project::Version::FRAMEWORK_VERSION`.
    framework_version: &'static str,
    /// `Project::Version::PROJECT_VERSION`.
    project_version: &'static str,
    /// `Project::Version::LIBRARY_VERSIONS`.
    library_versions: &'static [&'static str],
    /// Guarded state; the mutex is the guarded-port mutex.
    state: Mutex<VersionState>,
}

impl Version {
    /// Construct with the configured versions ([`FRAMEWORK_VERSION`],
    /// [`PROJECT_VERSION`], [`LIBRARY_VERSIONS`]).
    pub fn new(name: &str) -> Arc<Self> {
        Self::with_versions(name, FRAMEWORK_VERSION, PROJECT_VERSION, LIBRARY_VERSIONS)
    }

    /// Construct with explicit versions — the values upstream's build bakes
    /// into `versions/version.hpp`. Every custom slot starts as `"no_ver"`
    /// with status `Failure`; verbosity starts disabled.
    pub fn with_versions(
        name: &str,
        framework: &'static str,
        project: &'static str,
        libraries: &'static [&'static str],
    ) -> Arc<Self> {
        let initial = CustomVersionDb::new(
            // C++: "setVersion_enum is by default set to the first enum
            // value, so not setting it here".
            VersionEnum::default(),
            CustomVersionString::from(NO_VERSION),
            VersionStatus::Failure,
        );
        Arc::new(Self {
            base: PassiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            framework_version: framework,
            project_version: project,
            library_versions: libraries,
            state: Mutex::new(VersionState {
                enable: false,
                num_custom_elements: 0,
                db: core::array::from_fn(|_| initial.clone()),
            }),
        })
    }

    fn id_base(&self) -> FwIdType {
        self.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd
            .reg_commands(self.id_base(), &[Self::OPCODE_ENABLE, Self::OPCODE_VERSION]);
    }

    /// C++ `config(enable)`: custom-version telemetry verbosity.
    pub fn config(&self, enable: bool) {
        self.state.lock().enable = enable;
    }

    /// C++ `start()`: the startup framework, project and library version
    /// events and telemetry (SVC-VERSION-001..003). Call during start
    /// tasks.
    pub fn start(&self) {
        self.fw_version_tlm();
        self.project_version_tlm();
        self.library_version_tlm();
    }

    // -- Input-port factories ------------------------------------------------

    /// `cmdIn` — GUARDED `Fw.Cmd` input (both commands are `guarded`).
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Event / telemetry helpers -------------------------------------------

    fn log_version(&self, id: FwEventIdType, label: &str, version: &str) {
        let arg = LogStringArg::from(version);
        self.evt.log_event(
            self.id_base(),
            id,
            LogSeverity::ActivityLo,
            &format!("{label}: [{arg}]"),
            |buf: &mut LogBuffer| {
                arg.serialize_to_truncated(buf, VERSION_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn tlm_write_string(&self, id: FwChanIdType, value: &str) {
        let value = TlmString::from(value);
        self.tlm
            .tlm_write(self.id_base(), id, &value, self.evt.time_get());
    }

    /// C++ `fwVersion_tlm()`.
    fn fw_version_tlm(&self) {
        self.log_version(
            Self::EVENTID_FRAMEWORK_VERSION,
            "Framework Version",
            self.framework_version,
        );
        self.tlm_write_string(Self::CHANID_FRAMEWORK_VERSION, self.framework_version);
    }

    /// C++ `projectVersion_tlm()`.
    fn project_version_tlm(&self) {
        self.log_version(
            Self::EVENTID_PROJECT_VERSION,
            "Project Version",
            self.project_version,
        );
        self.tlm_write_string(Self::CHANID_PROJECT_VERSION, self.project_version);
    }

    /// C++ `libraryVersion_tlm()`: an event per library, a channel for the
    /// first ten ("design agreed to only provide 10 TLM channels").
    fn library_version_tlm(&self) {
        for (i, &library) in self.library_versions.iter().enumerate() {
            self.log_version(Self::EVENTID_LIBRARY_VERSIONS, "Library Versions", library);
            if i < NUM_VERSION_SLOTS {
                self.tlm_write_string(Self::CHANID_LIBRARY_VERSION_01 + i as FwChanIdType, library);
            }
        }
    }

    /// C++ `customVersion_tlm_all()`: nothing at all unless verbosity is
    /// enabled and some custom version was ever set.
    fn custom_version_tlm_all(&self, state: &VersionState) {
        let mut slot = 0;
        while state.enable && state.num_custom_elements != 0 && slot < NUM_VERSION_ENUMS {
            self.custom_version_tlm(state, slot);
            slot += 1;
        }
    }

    /// C++ `customVersion_tlm(slot)`: the event for a set slot, plus its
    /// channel while verbosity is enabled.
    fn custom_version_tlm(&self, state: &VersionState, slot: usize) {
        let entry = &state.db[slot];
        if entry.version_value != NO_VERSION && state.num_custom_elements > 0 {
            let version_enum = entry.version_enum;
            let value = &entry.version_value;
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_CUSTOM_VERSIONS,
                LogSeverity::ActivityLo,
                &format!("Custom Versions: [{version_enum:?}] [{value}]"),
                |buf: &mut LogBuffer| {
                    fw_try!(buf.serialize_u32_be(version_enum.as_repr()));
                    value.serialize_to_truncated(buf, VERSION_STRING_SIZE, Endianness::Big)
                },
            );
            if state.enable {
                // C++ switch over VER_SLOT_00..09 with FW_ASSERT(false) in
                // the default: there are only ten custom channels.
                fw_assert!(slot < NUM_VERSION_SLOTS, slot);
                self.tlm.tlm_write(
                    self.id_base(),
                    Self::CHANID_CUSTOM_VERSION_01 + slot as FwChanIdType,
                    entry,
                    self.evt.time_get(),
                );
            }
        }
    }

    // -- Handlers (guarded) -------------------------------------------------

    fn lock(&self) -> MutexGuard<'_, VersionState> {
        self.state.lock()
    }

    /// `getVersion_handler`: copy the slot out.
    fn get_version_handler(
        &self,
        _port_num: FwIndexType,
        version_id: VersionEnum,
        version_string: &mut FwDefaultString,
        status: &mut VersionStatus,
    ) {
        let slot = version_id.as_repr() as usize;
        fw_assert!(slot < NUM_VERSION_ENUMS, slot);
        let state = self.lock();
        version_string.set_bytes(state.db[slot].version_value.as_bytes());
        *status = state.db[slot].version_status;
    }

    /// `setVersion_handler`: store the slot, count it, report it.
    fn set_version_handler(
        &self,
        _port_num: FwIndexType,
        version_id: VersionEnum,
        version_string: &mut FwDefaultString,
        status: &mut VersionStatus,
    ) {
        let slot = version_id.as_repr() as usize;
        fw_assert!(slot < NUM_VERSION_ENUMS, slot);
        let mut state = self.lock();
        let entry = &mut state.db[slot];
        entry.version_enum = version_id;
        // `string size 80`: truncates a longer value.
        entry.version_value.set_bytes(version_string.as_bytes());
        entry.version_status = *status;
        // C++ parity: `U8 m_num_custom_elements++` wraps at 256.
        state.num_custom_elements = state.num_custom_elements.wrapping_add(1);
        self.custom_version_tlm(&state, slot);
    }

    /// `ENABLE_cmdHandler`.
    fn enable_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, enable: VersionEnabled) {
        self.lock().enable = enable == VersionEnabled::Enabled;
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `VERSION_cmdHandler`.
    fn version_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, version_type: VersionType) {
        {
            let state = self.lock();
            match version_type {
                VersionType::Project => self.project_version_tlm(),
                VersionType::Framework => self.fw_version_tlm(),
                VersionType::Library => self.library_version_tlm(),
                VersionType::Custom => self.custom_version_tlm_all(&state),
                VersionType::All => {
                    self.project_version_tlm();
                    self.fw_version_tlm();
                    self.library_version_tlm();
                    self.custom_version_tlm_all(&state);
                }
            }
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// The generated `cmdIn` dispatch: decode the one `U8` enum argument,
    /// `FormatError` on a short or over-long argument buffer,
    /// `ValidationError` on an undeclared enum value, `InvalidOpcode` on an
    /// unknown opcode.
    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let local = op_code.wrapping_sub(self.id_base());
        if local != Self::OPCODE_ENABLE && local != Self::OPCODE_VERSION {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode);
            return;
        }
        let mut raw = 0u8;
        if !args.deserialize_u8_be(&mut raw).is_ok() || args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        if local == Self::OPCODE_ENABLE {
            match VersionEnabled::try_from(raw) {
                Ok(enable) => self.enable_cmd_handler(op_code, cmd_seq, enable),
                Err(_) => self
                    .cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError),
            }
        } else {
            match VersionType::try_from(raw) {
                Ok(version_type) => self.version_cmd_handler(op_code, cmd_seq, version_type),
                Err(_) => self
                    .cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError),
            }
        }
    }
}

impl CmdPort for Version {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        self.cmd_in_handler(port_num, op_code, cmd_seq, args);
    }
}

input_port_adapter! {
    /// `getVersion` — GUARDED `Svc.Version` input: read a custom version.
    component: Version;
    adapter: GetVersionAdapter;
    port: VersionPort;
    input: pub get_version;
    handler: get_version_handler;
    args { val version_id: VersionEnum, mut version_string: FwDefaultString, mut status: VersionStatus }
}

input_port_adapter! {
    /// `setVersion` — GUARDED `Svc.Version` input: set a custom version.
    component: Version;
    adapter: SetVersionAdapter;
    port: VersionPort;
    input: pub set_version;
    handler: set_version_handler;
    args { val version_id: VersionEnum, mut version_string: FwDefaultString, mut status: VersionStatus }
}

#[cfg(test)]
mod tests {
    //! Ports of `Svc/Version/test/ut` (`test_startup`, `test_commands` =
    //! `test_enable` + `test_versions`, `test_ports` = `test_setVer` +
    //! `test_getVer`), with upstream's test `versions/version.hpp`
    //! (twelve libraries `"blahN @ blahN"`), plus the ported quirks.

    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, TlmPort};
    use fprime_fw::{SerializeStatus, Time, TlmBuffer};
    use std::string::{String, ToString};
    use std::sync::Mutex as StdMutex;
    use std::vec::Vec;

    /// Upstream `test/ut/version.cpp`.
    const TEST_FRAMEWORK_VERSION: &str = "v3.4.3-107-gf80da30ba";
    const TEST_PROJECT_VERSION: &str = "v3.4.4-107-gf80da30ba";
    const TEST_LIBRARY_VERSIONS: &[&str] = &[
        "blah0 @ blah0",
        "blah1 @ blah1",
        "blah2 @ blah2",
        "blah3 @ blah3",
        "blah4 @ blah4",
        "blah5 @ blah5",
        "blah6 @ blah6",
        "blah7 @ blah7",
        "blah8 @ blah8",
        "blah9 @ blah9",
        "blah10 @ blah10",
        "blah11 @ blah11",
    ];

    const ID_BASE: FwIdType = 0;

    #[derive(Default)]
    struct Ground {
        regs: StdMutex<Vec<FwOpcodeType>>,
        responses: StdMutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: StdMutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        tlm: StdMutex<Vec<(FwChanIdType, Vec<u8>)>>,
    }

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
            self.tlm.lock().unwrap().push((id, val.as_slice().to_vec()));
        }
    }

    impl Ground {
        fn clear(&self) {
            self.responses.lock().unwrap().clear();
            self.events.lock().unwrap().clear();
            self.tlm.lock().unwrap().clear();
        }

        fn events_of(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
            let events = self.events.lock().unwrap();
            events
                .iter()
                .filter(|(i, _, _)| *i == id)
                .map(|(_, _, a)| a.clone())
                .collect()
        }

        fn tlm_of(&self, id: FwChanIdType) -> Vec<Vec<u8>> {
            let tlm = self.tlm.lock().unwrap();
            tlm.iter()
                .filter(|(i, _)| *i == id)
                .map(|(_, v)| v.clone())
                .collect()
        }

        fn last_response(&self) -> (FwOpcodeType, u32, CmdResponse) {
            *self
                .responses
                .lock()
                .unwrap()
                .last()
                .expect("a command response")
        }
    }

    /// `[u16 len][bytes]`.
    fn string_bytes(s: &str) -> Vec<u8> {
        let mut out = (s.len() as u16).to_be_bytes().to_vec();
        out.extend_from_slice(s.as_bytes());
        out
    }

    /// A `CustomVersions` event's arguments.
    fn custom_event(slot: VersionEnum, value: &str) -> Vec<u8> {
        let mut out = slot.as_repr().to_be_bytes().to_vec();
        out.extend(string_bytes(value));
        out
    }

    /// A `CustomVersionDb` channel value.
    fn custom_db(slot: VersionEnum, value: &str, status: VersionStatus) -> Vec<u8> {
        let mut out = custom_event(slot, value);
        out.push(status.as_repr());
        out
    }

    fn build() -> (Arc<Version>, Arc<Ground>) {
        let ground = Arc::new(Ground::default());
        let comp = Version::with_versions(
            "Version",
            TEST_FRAMEWORK_VERSION,
            TEST_PROJECT_VERSION,
            TEST_LIBRARY_VERSIONS,
        );
        comp.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        (comp, ground)
    }

    fn send_cmd(comp: &Arc<Version>, opcode: FwOpcodeType, cmd_seq: u32, arg: u8) {
        let mut args = CmdArgBuffer::new();
        assert_eq!(args.serialize_u8_be(arg), SerializeStatus::Ok);
        let port = comp.cmd_in(0);
        port.target
            .invoke(port.port_num, ID_BASE + opcode, cmd_seq, &mut args);
    }

    fn set_version(comp: &Arc<Version>, id: VersionEnum, value: &str, status: VersionStatus) {
        let port = comp.set_version(0);
        let mut value = FwDefaultString::from(value);
        let mut status = status;
        port.target
            .invoke(port.port_num, id, &mut value, &mut status);
    }

    fn get_version(comp: &Arc<Version>, id: VersionEnum) -> (String, VersionStatus) {
        let port = comp.get_version(0);
        let mut value = FwDefaultString::new();
        let mut status = VersionStatus::Ok;
        port.target
            .invoke(port.port_num, id, &mut value, &mut status);
        (value.as_str().unwrap().to_string(), status)
    }

    /// Upstream `test_setVer(is_enabled)`: slots 0-2 OK, 3-6 FAILURE, 7-9
    /// OK, values `ver_N`.
    fn test_set_ver(comp: &Arc<Version>, ground: &Ground, is_enabled: bool) {
        ground.clear();
        for (i, &slot) in VersionEnum::VALUES.iter().enumerate() {
            let status = if (3..=6).contains(&i) {
                VersionStatus::Failure
            } else {
                VersionStatus::Ok
            };
            let value = format!("ver_{i}");
            set_version(comp, slot, &value, status);
            if is_enabled {
                let events = ground.events_of(Version::EVENTID_CUSTOM_VERSIONS);
                assert_eq!(events.len(), i + 1);
                assert_eq!(events[i], custom_event(slot, &value));
                let chan = Version::CHANID_CUSTOM_VERSION_01 + i as FwChanIdType;
                assert_eq!(ground.tlm_of(chan), vec![custom_db(slot, &value, status)]);
            }
        }
    }

    fn assert_no_custom_tlm(ground: &Ground) {
        for i in 0..NUM_VERSION_SLOTS {
            let chan = Version::CHANID_CUSTOM_VERSION_01 + i as FwChanIdType;
            assert!(ground.tlm_of(chan).is_empty(), "CustomVersion{:02}", i + 1);
        }
    }

    #[test]
    fn startup() {
        // SVC-VERSION-001..003.
        let (comp, ground) = build();
        comp.config(true);
        assert!(ground.tlm_of(Version::CHANID_FRAMEWORK_VERSION).is_empty());
        comp.start();
        assert_eq!(
            ground.tlm_of(Version::CHANID_FRAMEWORK_VERSION),
            vec![string_bytes(TEST_FRAMEWORK_VERSION)]
        );
        assert_eq!(
            ground.tlm_of(Version::CHANID_PROJECT_VERSION),
            vec![string_bytes(TEST_PROJECT_VERSION)]
        );
        assert_eq!(
            ground.events_of(Version::EVENTID_FRAMEWORK_VERSION),
            vec![string_bytes(TEST_FRAMEWORK_VERSION)]
        );
        assert_eq!(
            ground.events_of(Version::EVENTID_PROJECT_VERSION),
            vec![string_bytes(TEST_PROJECT_VERSION)]
        );
        let libraries = ground.events_of(Version::EVENTID_LIBRARY_VERSIONS);
        assert_eq!(libraries.len(), 12);
        assert_eq!(libraries[0], string_bytes("blah0 @ blah0"));
        let severities: Vec<LogSeverity> =
            ground.events.lock().unwrap().iter().map(|e| e.1).collect();
        assert!(severities.iter().all(|s| *s == LogSeverity::ActivityLo));
        // Startup does not report custom versions.
        assert!(
            ground
                .events_of(Version::EVENTID_CUSTOM_VERSIONS)
                .is_empty()
        );
    }

    #[test]
    fn reg_commands_registers_both_opcodes() {
        let (comp, ground) = build();
        comp.base.set_id_base(0x0100_7000);
        comp.reg_commands();
        assert_eq!(*ground.regs.lock().unwrap(), vec![0x0100_7000, 0x0100_7001]);
    }

    /// Upstream `test_enable`.
    fn test_enable(comp: &Arc<Version>, ground: &Ground) {
        send_cmd(
            comp,
            Version::OPCODE_ENABLE,
            9,
            VersionEnabled::Disabled.as_repr(),
        );
        assert_eq!(ground.last_response(), (0, 9, CmdResponse::Ok));
        test_set_ver(comp, ground, false);
        // Verbosity disabled: events still, no channels.
        assert_eq!(ground.events_of(Version::EVENTID_CUSTOM_VERSIONS).len(), 10);
        assert_no_custom_tlm(ground);

        send_cmd(
            comp,
            Version::OPCODE_ENABLE,
            9,
            VersionEnabled::Enabled.as_repr(),
        );
        assert_eq!(ground.last_response(), (0, 9, CmdResponse::Ok));
        test_set_ver(comp, ground, true);
        assert_eq!(ground.events_of(Version::EVENTID_CUSTOM_VERSIONS).len(), 10);
        for i in 0..NUM_VERSION_SLOTS {
            let chan = Version::CHANID_CUSTOM_VERSION_01 + i as FwChanIdType;
            assert_eq!(ground.tlm_of(chan).len(), 1);
        }
    }

    /// Upstream `test_versions`.
    fn test_versions(comp: &Arc<Version>, ground: &Ground) {
        let cmd_seq = 9;
        ground.clear();
        send_cmd(
            comp,
            Version::OPCODE_VERSION,
            cmd_seq,
            VersionType::Framework.as_repr(),
        );
        assert_eq!(ground.last_response(), (1, 9, CmdResponse::Ok));
        assert_eq!(
            ground.tlm_of(Version::CHANID_FRAMEWORK_VERSION),
            vec![string_bytes(TEST_FRAMEWORK_VERSION)]
        );
        assert_eq!(
            ground.events_of(Version::EVENTID_FRAMEWORK_VERSION),
            vec![string_bytes(TEST_FRAMEWORK_VERSION)]
        );

        ground.clear();
        send_cmd(
            comp,
            Version::OPCODE_VERSION,
            cmd_seq,
            VersionType::Project.as_repr(),
        );
        assert_eq!(ground.last_response(), (1, 9, CmdResponse::Ok));
        assert_eq!(
            ground.tlm_of(Version::CHANID_PROJECT_VERSION),
            vec![string_bytes(TEST_PROJECT_VERSION)]
        );
        assert_eq!(
            ground.events_of(Version::EVENTID_PROJECT_VERSION),
            vec![string_bytes(TEST_PROJECT_VERSION)]
        );

        ground.clear();
        send_cmd(
            comp,
            Version::OPCODE_VERSION,
            cmd_seq,
            VersionType::Library.as_repr(),
        );
        assert_eq!(ground.last_response(), (1, 9, CmdResponse::Ok));
        let events = ground.events_of(Version::EVENTID_LIBRARY_VERSIONS);
        assert_eq!(events.len(), 12);
        for (i, library) in TEST_LIBRARY_VERSIONS.iter().enumerate() {
            assert_eq!(events[i], string_bytes(library));
            if i < NUM_VERSION_SLOTS {
                let chan = Version::CHANID_LIBRARY_VERSION_01 + i as FwChanIdType;
                assert_eq!(ground.tlm_of(chan), vec![string_bytes(library)]);
            }
        }
        // Libraries 11 and 12: events only.
        assert_eq!(ground.tlm.lock().unwrap().len(), NUM_VERSION_SLOTS);

        ground.clear();
        test_set_ver(comp, ground, false);
        ground.clear();
        send_cmd(
            comp,
            Version::OPCODE_VERSION,
            cmd_seq,
            VersionType::Custom.as_repr(),
        );
        assert_eq!(ground.last_response(), (1, 9, CmdResponse::Ok));
        let events = ground.events_of(Version::EVENTID_CUSTOM_VERSIONS);
        assert_eq!(events.len(), 10);
        for (slot, status) in [
            (VersionEnum::ProjectVersion02, VersionStatus::Ok),
            (VersionEnum::ProjectVersion03, VersionStatus::Failure),
            (VersionEnum::ProjectVersion06, VersionStatus::Failure),
            (VersionEnum::ProjectVersion09, VersionStatus::Ok),
        ] {
            let i = slot.as_repr() as usize;
            let value = format!("ver_{i}");
            assert_eq!(events[i], custom_event(slot, &value));
            let chan = Version::CHANID_CUSTOM_VERSION_01 + i as FwChanIdType;
            assert_eq!(ground.tlm_of(chan), vec![custom_db(slot, &value, status)]);
        }

        ground.clear();
        send_cmd(
            comp,
            Version::OPCODE_VERSION,
            cmd_seq,
            VersionType::All.as_repr(),
        );
        assert_eq!(ground.last_response(), (1, 9, CmdResponse::Ok));
        assert_eq!(
            ground.tlm_of(Version::CHANID_FRAMEWORK_VERSION),
            vec![string_bytes(TEST_FRAMEWORK_VERSION)]
        );
        assert_eq!(
            ground.tlm_of(Version::CHANID_PROJECT_VERSION),
            vec![string_bytes(TEST_PROJECT_VERSION)]
        );
        assert_eq!(
            ground.events_of(Version::EVENTID_FRAMEWORK_VERSION).len(),
            1
        );
        assert_eq!(ground.events_of(Version::EVENTID_PROJECT_VERSION).len(), 1);
        assert_eq!(ground.events_of(Version::EVENTID_CUSTOM_VERSIONS).len(), 10);
        assert_eq!(
            ground.events_of(Version::EVENTID_LIBRARY_VERSIONS).len(),
            12
        );
    }

    #[test]
    fn commands() {
        // SVC-VERSION-004..009, 013.
        let (comp, ground) = build();
        test_enable(&comp, &ground);
        test_versions(&comp, &ground);
    }

    #[test]
    fn ports() {
        // SVC-VERSION-014/015.
        let (comp, ground) = build();
        send_cmd(
            &comp,
            Version::OPCODE_ENABLE,
            9,
            VersionEnabled::Enabled.as_repr(),
        );
        test_set_ver(&comp, &ground, true);
        for (slot, status) in [
            (VersionEnum::ProjectVersion02, VersionStatus::Ok),
            (VersionEnum::ProjectVersion03, VersionStatus::Failure),
            (VersionEnum::ProjectVersion06, VersionStatus::Failure),
            (VersionEnum::ProjectVersion09, VersionStatus::Ok),
            (VersionEnum::ProjectVersion05, VersionStatus::Failure),
        ] {
            let expected = format!("ver_{}", slot.as_repr());
            assert_eq!(get_version(&comp, slot), (expected, status));
        }
    }

    #[test]
    fn unset_slots_read_back_no_ver_failure() {
        let (comp, _ground) = build();
        assert_eq!(
            get_version(&comp, VersionEnum::ProjectVersion07),
            ("no_ver".to_string(), VersionStatus::Failure)
        );
    }

    #[test]
    fn custom_command_is_silent_while_verbosity_is_disabled() {
        // Quirk: customVersion_tlm_all loops only while m_enable — even the
        // events are suppressed, although setVersion logs them regardless.
        let (comp, ground) = build();
        set_version(
            &comp,
            VersionEnum::ProjectVersion01,
            "fpga-1",
            VersionStatus::Ok,
        );
        assert_eq!(ground.events_of(Version::EVENTID_CUSTOM_VERSIONS).len(), 1);
        ground.clear();
        send_cmd(
            &comp,
            Version::OPCODE_VERSION,
            1,
            VersionType::Custom.as_repr(),
        );
        assert_eq!(ground.last_response(), (1, 1, CmdResponse::Ok));
        assert!(ground.events.lock().unwrap().is_empty());
        assert!(ground.tlm.lock().unwrap().is_empty());
        // ...and so does a fresh component with verbosity on but nothing set.
        let (comp, ground) = build();
        comp.config(true);
        send_cmd(
            &comp,
            Version::OPCODE_VERSION,
            2,
            VersionType::Custom.as_repr(),
        );
        assert!(ground.events.lock().unwrap().is_empty());
    }

    #[test]
    fn custom_command_reports_only_set_slots() {
        let (comp, ground) = build();
        comp.config(true);
        set_version(
            &comp,
            VersionEnum::ProjectVersion04,
            "hw-B",
            VersionStatus::Ok,
        );
        ground.clear();
        send_cmd(
            &comp,
            Version::OPCODE_VERSION,
            3,
            VersionType::Custom.as_repr(),
        );
        assert_eq!(
            ground.events_of(Version::EVENTID_CUSTOM_VERSIONS),
            vec![custom_event(VersionEnum::ProjectVersion04, "hw-B")]
        );
        assert_eq!(
            ground.tlm_of(Version::CHANID_CUSTOM_VERSION_05),
            vec![custom_db(
                VersionEnum::ProjectVersion04,
                "hw-B",
                VersionStatus::Ok
            )]
        );
        assert_eq!(ground.tlm.lock().unwrap().len(), 1);
    }

    #[test]
    fn custom_element_counter_wraps_at_256() {
        // Quirk: `U8 m_num_custom_elements` — the 256th set wraps it to 0,
        // and that set reports nothing.
        let (comp, ground) = build();
        comp.config(true);
        for _ in 0..255 {
            set_version(&comp, VersionEnum::ProjectVersion00, "v", VersionStatus::Ok);
        }
        assert_eq!(
            ground.events_of(Version::EVENTID_CUSTOM_VERSIONS).len(),
            255
        );
        ground.clear();
        set_version(&comp, VersionEnum::ProjectVersion00, "v", VersionStatus::Ok);
        assert!(ground.events.lock().unwrap().is_empty());
        assert!(ground.tlm.lock().unwrap().is_empty());
        // The value was still stored.
        assert_eq!(
            get_version(&comp, VersionEnum::ProjectVersion00),
            ("v".to_string(), VersionStatus::Ok)
        );
        set_version(&comp, VersionEnum::ProjectVersion00, "w", VersionStatus::Ok);
        assert_eq!(ground.events_of(Version::EVENTID_CUSTOM_VERSIONS).len(), 1);
    }

    #[test]
    fn long_strings_are_truncated_per_declared_size() {
        let long: String = "0123456789".repeat(10); // 100 chars
        let (comp, ground) = build();
        comp.config(true);
        set_version(
            &comp,
            VersionEnum::ProjectVersion00,
            &long,
            VersionStatus::Ok,
        );
        // Event argument: string size 40; channel struct member: size 80.
        assert_eq!(
            ground.events_of(Version::EVENTID_CUSTOM_VERSIONS),
            vec![custom_event(VersionEnum::ProjectVersion00, &long[..40])]
        );
        assert_eq!(
            ground.tlm_of(Version::CHANID_CUSTOM_VERSION_01),
            vec![custom_db(
                VersionEnum::ProjectVersion00,
                &long[..80],
                VersionStatus::Ok
            )]
        );
        assert_eq!(
            get_version(&comp, VersionEnum::ProjectVersion00).0,
            long[..80]
        );

        let leaked: &'static str = std::boxed::Box::leak(long.clone().into_boxed_str());
        let ground = Arc::new(Ground::default());
        let comp = Version::with_versions("v", leaked, leaked, &[]);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.start();
        assert_eq!(
            ground.events_of(Version::EVENTID_FRAMEWORK_VERSION),
            vec![string_bytes(&long[..40])]
        );
        assert_eq!(
            ground.tlm_of(Version::CHANID_FRAMEWORK_VERSION),
            vec![string_bytes(&long[..40])]
        );
    }

    #[test]
    fn command_argument_errors() {
        let (comp, ground) = build();
        // Undeclared enum values.
        send_cmd(&comp, Version::OPCODE_ENABLE, 1, 2);
        assert_eq!(ground.last_response(), (0, 1, CmdResponse::ValidationError));
        send_cmd(&comp, Version::OPCODE_VERSION, 2, 5);
        assert_eq!(ground.last_response(), (1, 2, CmdResponse::ValidationError));
        // Missing and residual bytes.
        let port = comp.cmd_in(0);
        let mut empty = CmdArgBuffer::new();
        port.target.invoke(0, Version::OPCODE_ENABLE, 3, &mut empty);
        assert_eq!(ground.last_response(), (0, 3, CmdResponse::FormatError));
        let mut long = CmdArgBuffer::new();
        let _ = long.serialize_u8_be(1);
        let _ = long.serialize_u8_be(1);
        port.target.invoke(0, Version::OPCODE_VERSION, 4, &mut long);
        assert_eq!(ground.last_response(), (1, 4, CmdResponse::FormatError));
        // Unknown opcode.
        let mut args = CmdArgBuffer::new();
        port.target.invoke(0, 7, 5, &mut args);
        assert_eq!(ground.last_response(), (7, 5, CmdResponse::InvalidOpcode));
        assert!(ground.events.lock().unwrap().is_empty());
    }

    #[test]
    fn default_versions_describe_this_port() {
        assert_eq!(FRAMEWORK_VERSION.len(), 7);
        assert!(FRAMEWORK_VERSION.bytes().all(|b| b.is_ascii_hexdigit()));
        assert_eq!(PROJECT_VERSION, env!("CARGO_PKG_VERSION"));
        assert!(LIBRARY_VERSIONS.is_empty());
        let (ground, comp) = (Arc::new(Ground::default()), Version::new("version"));
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.start();
        assert_eq!(
            ground.events_of(Version::EVENTID_FRAMEWORK_VERSION),
            vec![string_bytes(FRAMEWORK_VERSION)]
        );
        assert_eq!(ground.events.lock().unwrap().len(), 2);
    }

    #[test]
    fn wire_types() {
        assert_eq!(VersionEnum::SERIALIZED_SIZE, 4);
        assert_eq!(VersionStatus::SERIALIZED_SIZE, 1);
        assert_eq!(VersionEnabled::SERIALIZED_SIZE, 1);
        assert_eq!(VersionType::All.as_repr(), 4);
        assert_eq!(CustomVersionDb::SERIALIZED_SIZE, 4 + 2 + 80 + 1);
        let db = CustomVersionDb::new(
            VersionEnum::ProjectVersion03,
            CustomVersionString::from("ab"),
            VersionStatus::Failure,
        );
        let mut buf = fprime_fw::LinearBuffer::<32>::new();
        assert_eq!(buf.serialize(&db, Endianness::Big), SerializeStatus::Ok);
        assert_eq!(buf.as_slice(), &[0, 0, 0, 3, 0, 2, b'a', b'b', 1]);
    }
}
