//! # Svc::TlmPacketizer — pre-laid-out telemetry packets (active component)
//!
//! C++ sources: `Svc/TlmPacketizer/TlmPacketizer.{cpp,hpp,fpp}`,
//! `Svc/TlmPacketizer/TlmPacketizerTypes.hpp`,
//! `Svc/TlmPacketizer/config/TlmPacketizerConfig/TlmPacketizerCfg.{fpp,hpp}`,
//! `Svc/Types/TlmPacketizerTypes/TlmPacketizerTypes.fpp`,
//! `Svc/Ports/TlmPacketizerPorts/TlmPacketizerPorts.fpp`.
//! Analysis: `docs/cpp-analysis/svc-misc.md` (TlmPacketizer section + gotchas).
//!
//! The alternative to `TlmChan`: instead of per-channel packets, a
//! compile-time packet table fixes the byte layout of every packet once
//! ([`TlmPacketizer::set_packet_list`]), each incoming channel value is
//! copied into the pre-computed offset of every packet that contains it, and
//! each `Run` tick emits the packets whose (section, group) rate/enable logic
//! says they are due.
//!
//! Packet wire layout (big-endian, fixed length for the life of the packet):
//!
//! ```text
//! [0..2)   FwPacketDescriptorType u16 = 0x0004 (FW_PACKET_PACKETIZED_TLM)
//! [2..4)   FwTlmPacketizeIdType   u16 = the packet id from the table
//! [4..15)  Fw::Time (11 bytes)        = rewritten in place on every send
//! [15..N)  the raw channel values, each at its table-assigned offset
//! ```
//!
//! There are no channel ids, no per-channel times, no length prefixes and no
//! channel count on the wire — ground decodes from the dictionary alone.
//!
//! C++-parity notes carried over from the analysis:
//! - `set_packet_list`'s `start_level` parameter is accepted and NEVER used.
//! - `prev_sent_counter` starts at `u32::MAX` and freezes there, so the first
//!   data arrival satisfies both MIN and MAX immediately.
//! - The counter increment happens AFTER the enable/silence/never-updated
//!   gate, so a disabled or SILENCED group's counter freezes.
//! - `SEND_PKT` emits `PacketSent` at command time (not at emission), marks
//!   only the requested section `REQUESTED`, and marks the shared fill buffer
//!   updated — which makes the OTHER sections see new data too.
//! - The port-connection check runs BEFORE the `REQUESTED` bypass, so a
//!   requested packet on an unconnected port keeps its `REQUESTED` flag
//!   forever.
//! - `controlIn` (the port) sets the section-enable state WITHOUT writing the
//!   `SectionEnabled` channel; the `ENABLE_SECTION` command does write it.
//! - `missingChannel`'s 25-slot table never resets: after 25 distinct unknown
//!   ids no further `NoChan` events are ever emitted.
//! - An oversized `TlmRecv` value is rejected with the throttled (10)
//!   `OversizedChannel` WARNING_HI, not asserted (hub guard).
//!
//! ## Locking
//!
//! `m_lock` (the C++ `Os::Mutex` guarding the fill buffers) is [`Self::lock`]
//! here and is taken exactly where C++ takes it: once per matching packet in
//! `TlmRecv`/`TlmGet`, and twice per packet in `Run`. `hasValue` lives inside
//! that mutex (it is written under the lock in C++, after the copy). The
//! channel table itself is written only by `set_packet_list` and read
//! lock-free in C++; here it sits behind an `RwLock` so concurrent producers
//! still do not serialize against each other. The `[section][group]`
//! configuration and the packet flags live on the component thread only
//! (`Run`, `controlIn`, the commands) and are held in a plain mutex.

use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, ComPort, ComponentDispatch, EventGlue,
    EventThrottle, MsgDispatchStatus, OutputPort, PingPort, PrmGlue, QueueFullPolicy, SchedPort,
    TlmGlue, TlmPort, async_input_port_adapter, component_msg_types, input_port_adapter, msg,
};
use fprime_config::{
    FW_COM_BUFFER_MAX_SIZE, FwChanIdType, FwEnumStoreType, FwEventIdType, FwIdType, FwIndexType,
    FwOpcodeType, FwPrmIdType, FwQueuePriorityType, FwSizeType, FwTlmPacketizeIdType,
};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, ComBuffer, ComPacketType, Deserialize, Enabled, Endianness, ExtBuf,
    LogBuffer, LogSeverity, ParamBuffer, ParamValid, SerBuf, SerBufAny, Serialize, SerializeStatus,
    Time, TlmBuffer, TlmValid, fpp_array, fpp_enum, fw_assert, fw_try,
};
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, RwLock};

use crate::tlm_chan::TlmGetPort;

// ---------------------------------------------------------------------------
// Configuration constants (TlmPacketizerCfg.hpp / TlmPacketizerCfg.fpp).
//
// They live here rather than in fprime-config because that crate has no
// tlm_packetizer submodule (it is owned by another wave); flagged for
// migration, exactly as PassiveTextLoggerCfg was.
// ---------------------------------------------------------------------------

/// `MAX_PACKETIZER_PACKETS` — packet-table capacity.
pub const MAX_PACKETIZER_PACKETS: usize = 50;
/// `MAX_PACKETIZER_CHANNELS` — distinct channels the table can hold.
pub const MAX_PACKETIZER_CHANNELS: usize = 200;
/// `TLMPACKETIZER_MAX_MISSING_TLM_CHECK` — missing-channel report slots.
pub const TLMPACKETIZER_MAX_MISSING_TLM_CHECK: usize = 25;
/// `MAX_CONFIGURABLE_TLMPACKETIZER_GROUP` — the greatest packet group/level.
pub const MAX_CONFIGURABLE_TLMPACKETIZER_GROUP: FwChanIdType = 3;
/// `NUM_CONFIGURABLE_TLMPACKETIZER_GROUPS` = max group + 1.
pub const NUM_CONFIGURABLE_TLMPACKETIZER_GROUPS: usize =
    MAX_CONFIGURABLE_TLMPACKETIZER_GROUP as usize + 1;
/// `TelemetrySection.NUM_SECTIONS` — configured resampling sections.
pub const NUM_SECTIONS: usize = 2;
/// `TELEMETRY_SEND_PORTS` — size of the `PktSend` output port array.
pub const TELEMETRY_SEND_PORTS: usize = 2;
/// `TELEMETRY_SEND_PORT_MAPPING` — `[section][group]` -> `PktSend` index.
pub const TELEMETRY_SEND_PORT_MAPPING: [[FwIndexType; NUM_CONFIGURABLE_TLMPACKETIZER_GROUPS];
    NUM_SECTIONS] = [[0, 0, 0, 0], [1, 1, 1, 1]];

/// Queue message size: the largest async invocation is a command
/// (6 envelope + 4 opcode + 4 cmdSeq + 2 + 506 arg buffer).
pub const QUEUE_MSG_SIZE: usize = 522;

/// Queue priority for every async input (no FPP `priority` qualifier).
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

/// Byte offset of the `Fw::Time` field inside a packetized-telemetry packet:
/// after the descriptor (u16) and the packet id (u16).
const PACKET_TIME_OFFSET: usize = 4;

/// `sizeof(FwPacketDescriptorType) + Fw::Time::SERIALIZED_SIZE +
/// sizeof(FwTlmPacketizeIdType)` — the fixed packet header length.
pub const PACKET_HEADER_SIZE: FwSizeType = 2 + 11 + 2;

// ---------------------------------------------------------------------------
// FPP data types.
// ---------------------------------------------------------------------------

fpp_enum! {
    /// `Svc::TelemetrySection` — the configurable resampling dimension.
    ///
    /// `NumSections` is a declared enum constant in FPP (a counter), so it is
    /// a VALID value on the wire; the handlers bounds-check against it
    /// separately, exactly as the C++ `section < NUM_SECTIONS` tests do.
    pub enum TelemetrySection : i32 {
        /// Realtime telemetry downlink through the communication stack.
        Realtime = 0,
        /// Recorded telemetry stored on disk for later retrieval.
        Recorded = 1,
        /// Counter constant; not a usable section.
        NumSections = 2,
    }
    default Realtime
}

fpp_enum! {
    /// `Svc::RateLogic` — per-(section, group) send logic.
    pub enum RateLogic : i32 {
        /// No logic applied: never sends and freezes the counter.
        Silenced = 0,
        /// Send every MAX ticks between sends.
        EveryMax = 1,
        /// Send on updates after MIN ticks since the last send.
        OnChangeMin = 2,
        /// Both the MIN-on-change and the MAX-interval rules.
        OnChangeMinOrEveryMax = 3,
    }
    default Silenced
}

/// `TlmPacketizer.GroupConfig` — 14 big-endian bytes.
///
/// `Default` is the FPP `DEFAULT_GROUP_CONFIG` constant, which makes
/// [`SectionConfigs::default()`] equal to `TELEMETRY_SECTION_DEFAULTS`.
///
/// Hand-written rather than `fpp_struct!`-generated because `Fw::Enabled`
/// lives in `fprime-fw` without an `FppSized` impl and the orphan rule
/// forbids adding one here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GroupConfig {
    /// Enable / disable telemetry output for this group.
    pub enabled: Enabled,
    /// Force output even when the group or section is disabled.
    pub force_enabled: Enabled,
    /// Rate logic selector.
    pub rate_logic: RateLogic,
    /// Minimum sched ticks between sends when using ON_CHANGE logic.
    pub min: u32,
    /// Maximum sched ticks between sends when using EVERY_MAX logic.
    pub max: u32,
}

impl GroupConfig {
    /// On-wire size: `1 + 1 + 4 + 4 + 4`.
    pub const SERIALIZED_SIZE: usize = 14;

    /// All-member constructor (the C++ generated full constructor).
    #[must_use]
    pub const fn new(
        enabled: Enabled,
        force_enabled: Enabled,
        rate_logic: RateLogic,
        min: u32,
        max: u32,
    ) -> Self {
        Self {
            enabled,
            force_enabled,
            rate_logic,
            min,
            max,
        }
    }
}

impl Default for GroupConfig {
    /// FPP `DEFAULT_GROUP_CONFIG`.
    fn default() -> Self {
        Self::new(
            Enabled::Enabled,
            Enabled::Disabled,
            RateLogic::OnChangeMin,
            0,
            0,
        )
    }
}

impl Serialize for GroupConfig {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        fw_try!(self.enabled.serialize_to(buf, e));
        fw_try!(self.force_enabled.serialize_to(buf, e));
        fw_try!(self.rate_logic.serialize_to(buf, e));
        fw_try!(buf.serialize_u32(self.min, e));
        buf.serialize_u32(self.max, e)
    }

    fn serialized_size(&self) -> usize {
        Self::SERIALIZED_SIZE
    }
}

impl Deserialize for GroupConfig {
    /// Commit-on-success, like the generated `fpp_struct!` code.
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let mut tmp = Self::default();
        fw_try!(tmp.enabled.deserialize_from(buf, e));
        fw_try!(tmp.force_enabled.deserialize_from(buf, e));
        fw_try!(tmp.rate_logic.deserialize_from(buf, e));
        fw_try!(buf.deserialize_u32(&mut tmp.min, e));
        fw_try!(buf.deserialize_u32(&mut tmp.max, e));
        *self = tmp;
        SerializeStatus::Ok
    }
}

impl fprime_fw::fpp::FppSized for GroupConfig {
    const SERIALIZED_SIZE: usize = Self::SERIALIZED_SIZE;
}

fpp_array! {
    /// `TlmPacketizer.GroupConfigs` — one [`GroupConfig`] per group
    /// (56 bytes).
    #[derive(Clone, Copy, Eq)]
    pub array GroupConfigs = [GroupConfig; NUM_CONFIGURABLE_TLMPACKETIZER_GROUPS]
}

fpp_array! {
    /// `TlmPacketizer.SectionConfigs` — one [`GroupConfigs`] per section
    /// (112 bytes). The default is `TELEMETRY_SECTION_DEFAULTS`.
    #[derive(Clone, Copy, Eq)]
    pub array SectionConfigs = [GroupConfigs; NUM_SECTIONS]
}

/// `TlmPacketizer.SectionEnabled` — one `Fw.Enabled` per section (2 bytes).
///
/// The default is `TELEMETRY_SECTION_ENABLED_DEFAULTS` (all ENABLED).
/// Hand-written for the same reason as [`GroupConfig`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SectionEnabled(pub [Enabled; NUM_SECTIONS]);

impl SectionEnabled {
    /// Number of elements.
    pub const SIZE: usize = NUM_SECTIONS;
    /// On-wire size: one byte per section.
    pub const SERIALIZED_SIZE: usize = NUM_SECTIONS;
}

impl Default for SectionEnabled {
    fn default() -> Self {
        Self([Enabled::Enabled; NUM_SECTIONS])
    }
}

impl std::ops::Index<usize> for SectionEnabled {
    type Output = Enabled;
    fn index(&self, index: usize) -> &Enabled {
        &self.0[index]
    }
}

impl std::ops::IndexMut<usize> for SectionEnabled {
    fn index_mut(&mut self, index: usize) -> &mut Enabled {
        &mut self.0[index]
    }
}

impl Serialize for SectionEnabled {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        for value in &self.0 {
            fw_try!(value.serialize_to(buf, e));
        }
        SerializeStatus::Ok
    }

    fn serialized_size(&self) -> usize {
        Self::SERIALIZED_SIZE
    }
}

impl Deserialize for SectionEnabled {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let mut tmp = Self::default();
        for value in &mut tmp.0 {
            fw_try!(value.deserialize_from(buf, e));
        }
        *self = tmp;
        SerializeStatus::Ok
    }
}

// ---------------------------------------------------------------------------
// Packet table (TlmPacketizerTypes.hpp).
// ---------------------------------------------------------------------------

/// `Svc::TlmPacketizerChannelEntry` — one channel in a packet definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TlmPacketizerChannelEntry {
    /// Channel id.
    pub id: FwChanIdType,
    /// MAXIMUM serialized size of the channel, in bytes.
    pub size: FwSizeType,
}

impl TlmPacketizerChannelEntry {
    /// Construct a table entry.
    #[must_use]
    pub const fn new(id: FwChanIdType, size: FwSizeType) -> Self {
        Self { id, size }
    }
}

/// `Svc::TlmPacketizerPacket` — one packet definition.
///
/// The C++ `numEntries` member is carried by the `channels` slice.
#[derive(Debug, Clone, Copy)]
pub struct TlmPacketizerPacket<'a> {
    /// Channels in this packet, in wire order.
    pub channels: &'a [TlmPacketizerChannelEntry],
    /// Packet id (goes on the wire at offset 2).
    pub id: FwTlmPacketizeIdType,
    /// Packet level, i.e. its group; must be
    /// `<= MAX_CONFIGURABLE_TLMPACKETIZER_GROUP`.
    pub level: FwChanIdType,
}

impl<'a> TlmPacketizerPacket<'a> {
    /// Construct a packet definition.
    #[must_use]
    pub const fn new(
        channels: &'a [TlmPacketizerChannelEntry],
        id: FwTlmPacketizeIdType,
        level: FwChanIdType,
    ) -> Self {
        Self {
            channels,
            id,
            level,
        }
    }
}

/// C++ `IGNORE_OMIT_LIST` — the sentinel that disables ignore-list handling.
pub const IGNORE_OMIT_LIST: &[TlmPacketizerChannelEntry] = &[];

// ---------------------------------------------------------------------------
// Ports declared by Svc/Ports/TlmPacketizerPorts (absent from fprime-comp).
// ---------------------------------------------------------------------------

/// `Svc.EnableSection` port: enable/disable a telemetry section.
pub trait EnableSectionPort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType, section: TelemetrySection, enabled: Enabled);
}

/// `Svc.ConfigureGroupRate` port: set a (section, group)'s rate logic.
pub trait ConfigureGroupRatePort: Send + Sync {
    /// Invoke the port.
    fn invoke(
        &self,
        port_num: FwIndexType,
        section: TelemetrySection,
        tlm_group: FwChanIdType,
        rate_logic: RateLogic,
        min_delta: u32,
        max_delta: u32,
    );
}

// ---------------------------------------------------------------------------
// Internal state.
// ---------------------------------------------------------------------------

/// C++ `UpdateFlag`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum UpdateFlag {
    /// Packet has never been updated (no data).
    NeverUpdated = 0,
    /// Packet has been sent and holds old data.
    Past = 1,
    /// Packet has been updated since the last send.
    New = 2,
    /// Packet was requested: bypasses all rate and enable checks.
    Requested = 3,
}

/// C++ `PktSendCounters`.
#[derive(Debug, Clone, Copy)]
struct PktSendCounters {
    /// Ticks since the last send. Starts at `u32::MAX` (C++: "prevent
    /// start up spam") and stops incrementing there.
    prev_sent_counter: u32,
    /// Update state machine.
    update_flag: UpdateFlag,
}

impl Default for PktSendCounters {
    fn default() -> Self {
        Self {
            prev_sent_counter: u32::MAX,
            update_flag: UpdateFlag::NeverUpdated,
        }
    }
}

/// C++ `BufferEntry` — one pre-laid-out packet.
#[derive(Debug, Clone)]
struct BufferEntry {
    /// The packet bytes; length fixed at configuration time.
    buffer: ComBuffer,
    /// Time of the most recent channel write into this packet.
    latest_time: Time,
    /// Packet id.
    id: FwChanIdType,
    /// Packet level (group).
    level: FwChanIdType,
    /// Whether any channel updated this packet since the last `Run`.
    updated: bool,
}

impl Default for BufferEntry {
    fn default() -> Self {
        Self {
            buffer: ComBuffer::new(),
            latest_time: Time::ZERO,
            id: 0,
            level: 0,
            updated: false,
        }
    }
}

/// C++ `TlmEntry` (minus `hasValue`, which lives under the packet mutex).
#[derive(Debug, Clone)]
struct TlmEntry {
    /// Channel id.
    id: FwChanIdType,
    /// Byte offset of this channel in each packet; `-1` = not in the packet.
    packet_offset: [i64; MAX_PACKETIZER_PACKETS],
    /// Maximum serialized size of the channel.
    channel_size: FwSizeType,
    /// Channel is deliberately not packetized (no `NoChan` warning).
    ignored: bool,
}

impl Default for TlmEntry {
    fn default() -> Self {
        Self {
            id: 0,
            packet_offset: [-1; MAX_PACKETIZER_PACKETS],
            channel_size: 0,
            ignored: false,
        }
    }
}

/// The configured channel table: written once by `set_packet_list`, read by
/// every `TlmRecv`/`TlmGet`.
struct ChannelTable {
    /// Channel entries, indexed by [`ChannelTable::indices`].
    channels: Vec<TlmEntry>,
    /// Channel id -> index into `channels` (C++ `RedBlackTreeMap`; only
    /// exact-match lookups are performed, so a `BTreeMap` is equivalent).
    indices: BTreeMap<FwChanIdType, usize>,
    /// Number of configured packets.
    num_packets: usize,
    /// C++ `m_configured`.
    configured: bool,
}

/// Everything guarded by the C++ `m_lock`.
struct PacketState {
    /// C++ `m_fillBuffers`.
    fill_buffers: Vec<BufferEntry>,
    /// C++ `TlmEntry::hasValue`, parallel to `ChannelTable::channels`.
    has_value: Vec<bool>,
}

/// Component-thread-only configuration state (`Run`, `controlIn`, commands).
struct CtrlState {
    /// C++ `m_sectionEnabled`.
    section_enabled: SectionEnabled,
    /// C++ `m_groupConfigs`.
    group_configs: SectionConfigs,
    /// C++ `m_packetFlags[section][packet]`.
    packet_flags: [[PktSendCounters; MAX_PACKETIZER_PACKETS]; NUM_SECTIONS],
}

/// C++ `MissingTlmChan`.
#[derive(Debug, Clone, Copy, Default)]
struct MissingTlmChan {
    /// The unknown channel id that claimed this slot.
    id: FwChanIdType,
    /// Slot is in use (and its event has been emitted).
    checked: bool,
}

// ---------------------------------------------------------------------------
// The component.
// ---------------------------------------------------------------------------

/// `Svc::TlmPacketizer`.
pub struct TlmPacketizer {
    /// Active core: PassiveBase + queue + task.
    pub active: ActiveBase,
    /// Command registration + response ports.
    pub cmd: CmdGlue,
    /// Event ports (`eventOut`, `textEventOut`) and `timeGetOut`.
    pub evt: EventGlue,
    /// Telemetry port (`tlmOut`).
    pub tlm: TlmGlue,
    /// Parameter ports (`paramGetOut`, `paramSetOut`).
    pub prm: PrmGlue,
    /// `PktSend: [TELEMETRY_SEND_PORTS] Fw.Com`, ordered by section/group
    /// through [`TELEMETRY_SEND_PORT_MAPPING`].
    pub pkt_send: [OutputPort<dyn ComPort>; TELEMETRY_SEND_PORTS],
    /// `pingOut: Svc.Ping`.
    pub ping_out: OutputPort<dyn PingPort>,
    /// FPP `throttle 10` on `OversizedChannel`.
    oversized_channel_throttle: EventThrottle,
    /// The configured packet/channel table.
    table: RwLock<ChannelTable>,
    /// C++ `m_lock` — the packet fill buffers and `hasValue`.
    lock: Mutex<PacketState>,
    /// Section/group configuration and per-packet flags.
    ctrl: Mutex<CtrlState>,
    /// C++ `m_missTlmCheck`.
    missing: Mutex<[MissingTlmChan; TLMPACKETIZER_MAX_MISSING_TLM_CHECK]>,
}

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel), in FPP declaration
    /// order.
    impl TlmPacketizer {
        /// `controlIn` async input port.
        MSG_TYPE_CONTROL_IN,
        /// `pingIn` async input port.
        MSG_TYPE_PING_IN,
        /// `Run` async input port.
        MSG_TYPE_RUN,
        /// `configureSectionGroupRate` async input port.
        MSG_TYPE_CONFIGURE_SECTION_GROUP_RATE,
        /// `cmdIn` async command port.
        MSG_TYPE_CMD_IN,
    }
}

impl TlmPacketizer {
    /// Command `SET_LEVEL(level: FwChanIdType)` — opcode 0.
    pub const OPCODE_SET_LEVEL: FwOpcodeType = 0;
    /// Command `SEND_PKT(id: U32, section: TelemetrySection)` — opcode 1.
    pub const OPCODE_SEND_PKT: FwOpcodeType = 1;
    /// Command `ENABLE_SECTION(section, enable: Fw.Enabled)` — opcode 2.
    pub const OPCODE_ENABLE_SECTION: FwOpcodeType = 2;
    /// Command `ENABLE_GROUP(section, tlmGroup, enable)` — opcode 3.
    pub const OPCODE_ENABLE_GROUP: FwOpcodeType = 3;
    /// Command `FORCE_GROUP(section, tlmGroup, enable)` — opcode 4.
    pub const OPCODE_FORCE_GROUP: FwOpcodeType = 4;
    /// Command `CONFIGURE_GROUP_RATES(section, tlmGroup, rateLogic,
    /// minDelta, maxDelta)` — opcode 5.
    pub const OPCODE_CONFIGURE_GROUP_RATES: FwOpcodeType = 5;

    /// `NoChan(Id: FwChanIdType)` — WARNING_LO, id 0.
    pub const EVENTID_NO_CHAN: FwEventIdType = 0;
    /// `LevelSet(level: FwChanIdType)` — ACTIVITY_HI, id 1.
    pub const EVENTID_LEVEL_SET: FwEventIdType = 1;
    /// `MaxLevelExceed(level, max)` — WARNING_LO, id 2.
    pub const EVENTID_MAX_LEVEL_EXCEED: FwEventIdType = 2;
    /// `PacketSent(packetId: U32)` — ACTIVITY_LO, id 3.
    pub const EVENTID_PACKET_SENT: FwEventIdType = 3;
    /// `PacketNotFound(packetId: U32)` — WARNING_LO, id 4.
    pub const EVENTID_PACKET_NOT_FOUND: FwEventIdType = 4;
    /// `SectionUnconfigurable(section, enable)` — WARNING_LO, id 5.
    pub const EVENTID_SECTION_UNCONFIGURABLE: FwEventIdType = 5;
    /// `OversizedChannel(Id, valSize, expected)` — WARNING_HI, id 6,
    /// `throttle 10`.
    pub const EVENTID_OVERSIZED_CHANNEL: FwEventIdType = 6;
    /// FPP `throttle 10` on `OversizedChannel`.
    pub const OVERSIZED_CHANNEL_THROTTLE: u32 = 10;

    /// Telemetry `GroupConfigs: SectionConfigs` — channel id 0 (112 bytes).
    pub const CHANID_GROUP_CONFIGS: FwChanIdType = 0;
    /// Telemetry `SectionEnabled: SectionEnabled` — channel id 1 (2 bytes).
    pub const CHANID_SECTION_ENABLED: FwChanIdType = 1;

    /// External parameter `SECTION_ENABLED` — local id 0.
    pub const PARAMID_SECTION_ENABLED: FwPrmIdType = 0;
    /// External parameter `SECTION_CONFIGS` — local id 1.
    pub const PARAMID_SECTION_CONFIGS: FwPrmIdType = 1;

    /// Construct.
    pub fn new(name: &str) -> Arc<Self> {
        let mut fill_buffers = Vec::with_capacity(MAX_PACKETIZER_PACKETS);
        fill_buffers.resize_with(MAX_PACKETIZER_PACKETS, BufferEntry::default);
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            prm: PrmGlue::new(),
            pkt_send: [const { OutputPort::new() }; TELEMETRY_SEND_PORTS],
            ping_out: OutputPort::new(),
            oversized_channel_throttle: EventThrottle::new(Self::OVERSIZED_CHANNEL_THROTTLE),
            table: RwLock::new(ChannelTable {
                channels: Vec::with_capacity(MAX_PACKETIZER_CHANNELS),
                indices: BTreeMap::new(),
                num_packets: 0,
                configured: false,
            }),
            lock: Mutex::new(PacketState {
                fill_buffers,
                has_value: vec![false; MAX_PACKETIZER_CHANNELS],
            }),
            ctrl: Mutex::new(CtrlState {
                section_enabled: SectionEnabled::default(),
                group_configs: SectionConfigs::default(),
                packet_flags: [[PktSendCounters::default(); MAX_PACKETIZER_PACKETS]; NUM_SECTIONS],
            }),
            missing: Mutex::new([MissingTlmChan::default(); TLMPACKETIZER_MAX_MISSING_TLM_CHECK]),
        })
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_SET_LEVEL,
                Self::OPCODE_SEND_PKT,
                Self::OPCODE_ENABLE_SECTION,
                Self::OPCODE_ENABLE_GROUP,
                Self::OPCODE_FORCE_GROUP,
                Self::OPCODE_CONFIGURE_GROUP_RATES,
            ],
        );
    }

    /// Create the component queue (topology phase 4).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    // -- Configuration --------------------------------------------------------

    /// C++ `setPacketList(packetList, ignoreList, startLevel)`.
    ///
    /// Lays out every packet, builds the channel index and pre-serializes
    /// each packet's descriptor and id. Re-entrant: calling it again
    /// rebuilds the table from scratch.
    ///
    /// `start_level` is accepted and **never used** — C++ parity; only
    /// `SET_LEVEL` ever changes the group enables.
    ///
    /// Asserts (all of them C++ `FW_ASSERT`s, i.e. configuration errors):
    /// more than [`MAX_PACKETIZER_PACKETS`] packets, more than
    /// [`MAX_PACKETIZER_CHANNELS`] distinct channels, the same channel id
    /// declared with two different sizes, a packet longer than
    /// `FW_COM_BUFFER_MAX_SIZE`, a level above
    /// [`MAX_CONFIGURABLE_TLMPACKETIZER_GROUP`], and a channel that appears
    /// both in a packet and in the ignore list.
    pub fn set_packet_list(
        &self,
        packet_list: &[TlmPacketizerPacket<'_>],
        ignore_list: &[TlmPacketizerChannelEntry],
        _start_level: FwChanIdType,
    ) {
        fw_assert!(
            packet_list.len() <= MAX_PACKETIZER_PACKETS,
            packet_list.len() as i32
        );

        let mut table = self.table.write().unwrap();
        let mut state = self.lock.lock().unwrap();

        // Reset key data members in case of re-entrant calls.
        table.channels.clear();
        table.indices.clear();
        table.configured = false;
        for has_value in state.has_value.iter_mut() {
            *has_value = false;
        }

        let mut max_level: FwChanIdType = 0;
        for (pkt_entry, packet) in packet_list.iter().enumerate() {
            let mut packet_len: FwSizeType = PACKET_HEADER_SIZE;
            for channel in packet.channels {
                let entry_index = match table.indices.get(&channel.id) {
                    Some(&index) => {
                        // A channel id may repeat across packets, but its
                        // declared size must match: a conflicting size would
                        // corrupt the offsets computed from the earlier
                        // definition.
                        fw_assert!(
                            table.channels[index].channel_size == channel.size,
                            channel.id as i32,
                            channel.size as i32,
                            table.channels[index].channel_size as i32
                        );
                        index
                    }
                    None => {
                        let index = table.channels.len();
                        fw_assert!(index < MAX_PACKETIZER_CHANNELS, index as i32);
                        table.channels.push(TlmEntry {
                            id: channel.id,
                            packet_offset: [-1; MAX_PACKETIZER_PACKETS],
                            channel_size: channel.size,
                            ignored: false,
                        });
                        table.indices.insert(channel.id, index);
                        index
                    }
                };
                let entry = &mut table.channels[entry_index];
                entry.ignored = false;
                entry.channel_size = channel.size;
                fw_assert!(packet_len <= i64::MAX as FwSizeType, packet_len as i32);
                entry.packet_offset[pkt_entry] = packet_len as i64;
                packet_len += entry.channel_size;
            }

            fw_assert!(
                packet_len <= FW_COM_BUFFER_MAX_SIZE as FwSizeType,
                packet_len as i32,
                pkt_entry as i32
            );

            let fill = &mut state.fill_buffers[pkt_entry];
            let len = packet_len as usize;
            // Clear the packet body: unwritten channel bytes are zeros.
            fill.buffer.bytes_mut()[..len].fill(0);
            fill.buffer.reset_ser();
            // The descriptor and the packet id never change, so serialize
            // them once here.
            let status = fill
                .buffer
                .serialize_u16_be(ComPacketType::FwPacketPacketizedTlm as u16);
            fw_assert!(status.is_ok(), status as i32);
            let status = fill.buffer.serialize_u16_be(packet.id);
            fw_assert!(status.is_ok(), status as i32);
            let status = fill.buffer.set_buff_len(len);
            fw_assert!(status.is_ok(), status as i32);
            fill.id = FwChanIdType::from(packet.id);
            fill.level = packet.level;
            if packet.level > max_level {
                max_level = packet.level;
            }
        }
        fw_assert!(
            max_level <= MAX_CONFIGURABLE_TLMPACKETIZER_GROUP,
            max_level as i32
        );

        // Ignore list: channels deliberately not packetized.
        for channel in ignore_list {
            let entry_index = match table.indices.get(&channel.id) {
                Some(&index) => {
                    // Gotcha: a channel in BOTH a packet and the ignore list
                    // is a configuration error (packets are processed first,
                    // so `ignored` is still false here).
                    fw_assert!(table.channels[index].ignored, channel.id as i32);
                    index
                }
                None => {
                    let index = table.channels.len();
                    fw_assert!(index < MAX_PACKETIZER_CHANNELS, index as i32);
                    table.channels.push(TlmEntry {
                        id: channel.id,
                        ..TlmEntry::default()
                    });
                    table.indices.insert(channel.id, index);
                    index
                }
            };
            let entry = &mut table.channels[entry_index];
            entry.ignored = true;
            entry.channel_size = channel.size;
        }

        table.num_packets = packet_list.len();
        table.configured = true;
    }

    /// C++ generated `loadParameters()` for the two external parameters.
    ///
    /// A no-op when `paramGetOut` is unconnected (a deployment without a
    /// parameter database keeps the FPP defaults).
    pub fn load_parameters(&self) {
        if !self.prm.prm_get_out.is_connected() {
            return;
        }
        for local_id in [Self::PARAMID_SECTION_ENABLED, Self::PARAMID_SECTION_CONFIGS] {
            let mut buf = ParamBuffer::new();
            let valid = self.prm.get_param(self.id_base(), local_id, &mut buf);
            let _ = self.deserialize_param(self.id_base(), local_id, valid, &mut buf);
        }
    }

    /// `Fw::ParamExternalDelegate::serializeParam` — write the parameter
    /// identified by `local_id` into `buf`.
    ///
    /// An unknown `local_id` is a `FW_ASSERT` (C++ parity).
    pub fn serialize_param(
        &self,
        _base_id: FwPrmIdType,
        local_id: FwPrmIdType,
        buf: &mut dyn SerBufAny,
    ) -> SerializeStatus {
        let ctrl = self.ctrl.lock().unwrap();
        match local_id {
            Self::PARAMID_SECTION_ENABLED => {
                ctrl.section_enabled.serialize_to(buf, Endianness::Big)
            }
            Self::PARAMID_SECTION_CONFIGS => ctrl.group_configs.serialize_to(buf, Endianness::Big),
            _ => {
                fw_assert!(false, local_id as i32);
                SerializeStatus::FormatError
            }
        }
    }

    /// `Fw::ParamExternalDelegate::deserializeParam` — load the parameter
    /// identified by `local_id` from `buf`.
    ///
    /// A not-OK `prm_stat` returns `DeserTypeMismatch` WITHOUT touching the
    /// stored value (C++ parity); an unknown `local_id` is a `FW_ASSERT`.
    pub fn deserialize_param(
        &self,
        _base_id: FwPrmIdType,
        local_id: FwPrmIdType,
        prm_stat: ParamValid,
        buf: &mut dyn SerBufAny,
    ) -> SerializeStatus {
        if prm_stat.is_ok() {
            let mut ctrl = self.ctrl.lock().unwrap();
            match local_id {
                Self::PARAMID_SECTION_ENABLED => {
                    return ctrl.section_enabled.deserialize_from(buf, Endianness::Big);
                }
                Self::PARAMID_SECTION_CONFIGS => {
                    return ctrl.group_configs.deserialize_from(buf, Endianness::Big);
                }
                _ => {
                    fw_assert!(false, local_id as i32);
                }
            }
        }
        SerializeStatus::DeserTypeMismatch
    }

    /// C++ `sectionGroupToPort` — `[section][group]` -> `PktSend` index.
    fn section_group_to_port(section: usize, group: usize) -> FwIndexType {
        fw_assert!(group < NUM_CONFIGURABLE_TLMPACKETIZER_GROUPS, group as i32);
        fw_assert!(section < NUM_SECTIONS, section as i32);
        let out_index = TELEMETRY_SEND_PORT_MAPPING[section][group];
        fw_assert!(
            (out_index as usize) < TELEMETRY_SEND_PORTS,
            out_index as i32
        );
        out_index
    }

    // -- Handlers -------------------------------------------------------------

    /// `TlmRecv_handler` — SYNC: runs on the producer's thread.
    fn tlm_recv_handler(
        &self,
        _port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) {
        let table = self.table.read().unwrap();
        fw_assert!(table.configured);

        let Some(&entry_index) = table.indices.get(&id) else {
            drop(table);
            self.missing_channel(id);
            return;
        };
        let entry = &table.channels[entry_index];
        if entry.ignored {
            return;
        }
        let value_size = val.get_size() as FwSizeType;
        if value_size > entry.channel_size {
            // Hub guard: values may arrive from another address space, so
            // reject rather than assert.
            let expected = entry.channel_size;
            drop(table);
            if self.oversized_channel_throttle.ok_to_emit() {
                self.log_oversized_channel(id, value_size, expected);
            }
            return;
        }

        let size = val.get_size();
        for pkt in 0..MAX_PACKETIZER_PACKETS {
            let offset = entry.packet_offset[pkt];
            if offset == -1 {
                continue;
            }
            // C++ parity: the lock is taken and released ONCE PER PACKET, so
            // a concurrent Run can interleave between packets.
            let mut state = self.lock.lock().unwrap();
            {
                let fill = &mut state.fill_buffers[pkt];
                fill.updated = true;
                fill.latest_time = *time_tag;
                let start = offset as usize;
                fill.buffer.bytes_mut()[start..start + size].copy_from_slice(val.as_slice());
            }
            // Written under the lock AFTER the copy, so TlmGet can never see
            // a VALID-but-empty entry.
            state.has_value[entry_index] = true;
        }
    }

    /// `TlmGet_handler` — SYNC: runs on the caller's thread.
    fn tlm_get_handler(
        &self,
        _port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) -> TlmValid {
        let table = self.table.read().unwrap();
        fw_assert!(table.configured);

        let Some(&entry_index) = table.indices.get(&id) else {
            drop(table);
            self.missing_channel(id);
            val.reset_ser();
            return TlmValid::Invalid;
        };
        let entry = &table.channels[entry_index];
        if entry.ignored {
            val.reset_ser();
            return TlmValid::Invalid;
        }
        {
            let state = self.lock.lock().unwrap();
            if !state.has_value[entry_index] {
                drop(state);
                val.reset_ser();
                return TlmValid::Invalid;
            }
        }
        fw_assert!(
            entry.channel_size <= val.capacity() as FwSizeType,
            entry.channel_size as i32,
            val.capacity() as i32
        );

        for pkt in 0..MAX_PACKETIZER_PACKETS {
            let offset = entry.packet_offset[pkt];
            if offset == -1 {
                continue;
            }
            let state = self.lock.lock().unwrap();
            let fill = &state.fill_buffers[pkt];
            *time_tag = fill.latest_time;
            let start = offset as usize;
            let size = entry.channel_size as usize;
            // The value is padded to the table's MAX channel size, so the
            // tail may hold junk from a previously longer value.
            let status = val.set_buff(&fill.buffer.bytes()[start..start + size]);
            fw_assert!(status.is_ok(), status as i32);
            return TlmValid::Valid;
        }

        // Not ignored, so it must live in some packet: a coding error.
        fw_assert!(false, entry.id as i32);
        val.reset_ser();
        TlmValid::Invalid
    }

    /// `Run_handler` — component thread.
    fn run_handler(&self, _port_num: FwIndexType, _context: u32) {
        let num_packets = {
            let table = self.table.read().unwrap();
            fw_assert!(table.configured);
            table.num_packets
        };

        for pkt in 0..num_packets {
            let mut section_needs_send = [false; NUM_SECTIONS];
            let mut any_section_needs_send = false;

            // Lock only to capture the update status and reset the flag.
            let (is_new_data, entry_group) = {
                let mut state = self.lock.lock().unwrap();
                let fill = &mut state.fill_buffers[pkt];
                let captured = (fill.updated, fill.level);
                fill.updated = false;
                captured
            };
            let group = entry_group as usize;

            // Sends to make once the flags are settled, one slot per
            // section: `Some((port index, context))`. A fixed array keeps the
            // tick allocation-free.
            let mut sends: [Option<(FwIndexType, u32)>; NUM_SECTIONS] = [None; NUM_SECTIONS];
            {
                let mut ctrl = self.ctrl.lock().unwrap();
                #[allow(clippy::needless_range_loop)]
                for section in 0..NUM_SECTIONS {
                    // Keep a REQUESTED marking so it bypasses the checks.
                    if is_new_data
                        && ctrl.packet_flags[section][pkt].update_flag != UpdateFlag::Requested
                    {
                        ctrl.packet_flags[section][pkt].update_flag = UpdateFlag::New;
                    }

                    let out_index = Self::section_group_to_port(section, group);
                    // Gotcha: the connection check precedes the REQUESTED
                    // bypass, so a requested packet on an unconnected port
                    // keeps its flag forever.
                    if !self.pkt_send[out_index as usize].is_connected() {
                        continue;
                    }

                    let config = ctrl.group_configs[section][group];
                    let section_enabled = ctrl.section_enabled[section];
                    let flags = &mut ctrl.packet_flags[section][pkt];

                    if flags.update_flag == UpdateFlag::Requested {
                        section_needs_send[section] = true;
                    } else {
                        if !((config.enabled == Enabled::Enabled
                            && section_enabled == Enabled::Enabled)
                            || config.force_enabled == Enabled::Enabled)
                        {
                            continue;
                        }
                        if config.rate_logic == RateLogic::Silenced {
                            continue;
                        }
                        if flags.update_flag == UpdateFlag::NeverUpdated {
                            continue; // avoid sending "no data"
                        }
                    }

                    // The increment is AFTER the gate: a disabled or silenced
                    // group's counter freezes. C++ writes this as an explicit
                    // `< U32_MAX` guard; a saturating add is the same thing.
                    flags.prev_sent_counter = flags.prev_sent_counter.saturating_add(1);

                    if flags.update_flag == UpdateFlag::New
                        && config.rate_logic != RateLogic::EveryMax
                        && flags.prev_sent_counter >= config.min
                    {
                        section_needs_send[section] = true;
                    }
                    if config.rate_logic != RateLogic::OnChangeMin
                        && flags.prev_sent_counter >= config.max
                    {
                        section_needs_send[section] = true;
                    }

                    if section_needs_send[section] {
                        any_section_needs_send = true;
                        sends[section] = Some((out_index, flags.prev_sent_counter));
                        // C++ resets these right after the port call; nothing
                        // reads them in between, so they are updated here and
                        // the ports are invoked outside the lock.
                        flags.prev_sent_counter = 0;
                        flags.update_flag = UpdateFlag::Past;
                    }
                }
            }

            if !any_section_needs_send {
                continue;
            }

            let send_buffer = {
                let state = self.lock.lock().unwrap();
                state.fill_buffers[pkt].clone()
            };
            let mut buffer = send_buffer.buffer;
            {
                // Rewrite the 11 time bytes in place; the header is not
                // re-serialized (C++ ExternalSerializeBuffer over &buf[4]).
                let slice = &mut buffer.bytes_mut()[PACKET_TIME_OFFSET..PACKET_TIME_OFFSET + 11];
                let mut ext = ExtBuf::new(slice);
                let status = send_buffer
                    .latest_time
                    .serialize_to(&mut ext, Endianness::Big);
                fw_assert!(status.is_ok(), status as i32);
            }
            for (out_index, context) in sends.into_iter().flatten() {
                let port = self.pkt_send[out_index as usize].get();
                port.target.invoke(port.port_num, &mut buffer, context);
            }
        }
    }

    /// `controlIn_handler` — component thread.
    ///
    /// Gotcha: unlike `ENABLE_SECTION`, this does NOT write the
    /// `SectionEnabled` telemetry channel.
    fn control_in_handler(
        &self,
        _port_num: FwIndexType,
        section: TelemetrySection,
        enabled: Enabled,
    ) {
        if (section.as_repr() as usize) < NUM_SECTIONS {
            let mut ctrl = self.ctrl.lock().unwrap();
            ctrl.section_enabled[section.as_repr() as usize] = enabled;
        } else {
            self.log_section_unconfigurable(section, enabled);
        }
    }

    /// `configureSectionGroupRate_handler` — component thread. Bad port
    /// arguments are a crash here (C++ uses `FW_ASSERT`s, not status
    /// returns, on this path).
    fn configure_section_group_rate_handler(
        &self,
        _port_num: FwIndexType,
        section: TelemetrySection,
        tlm_group: FwChanIdType,
        rate_logic: RateLogic,
        min_delta: u32,
        max_delta: u32,
    ) {
        self.configure_section_group_rate(section, tlm_group, rate_logic, min_delta, max_delta);
    }

    /// `pingIn_handler` — echo the key on port 0.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        if let Some(p) = self.ping_out.try_get() {
            p.target.invoke(p.port_num, key);
        }
    }

    /// C++ `configureSectionGroupRate` helper (shared by the port handler
    /// and the `CONFIGURE_GROUP_RATES` command).
    fn configure_section_group_rate(
        &self,
        section: TelemetrySection,
        tlm_group: FwChanIdType,
        rate_logic: RateLogic,
        min_delta: u32,
        max_delta: u32,
    ) {
        let section_index = section.as_repr();
        fw_assert!(
            section_index >= 0 && (section_index as usize) < NUM_SECTIONS,
            section_index
        );
        fw_assert!(
            tlm_group <= MAX_CONFIGURABLE_TLMPACKETIZER_GROUP,
            tlm_group as i32
        );
        let configs = {
            let mut ctrl = self.ctrl.lock().unwrap();
            let config = &mut ctrl.group_configs[section_index as usize][tlm_group as usize];
            config.rate_logic = rate_logic;
            config.min = min_delta;
            config.max = max_delta;
            ctrl.group_configs
        };
        self.tlm_write_group_configs(&configs);
    }

    /// C++ `missingChannel(id)`: report an unknown channel once, using a
    /// 25-slot table that never resets.
    fn missing_channel(&self, id: FwChanIdType) {
        let emit = {
            let mut slots = self.missing.lock().unwrap();
            let mut emit = false;
            for slot in slots.iter_mut() {
                if slot.checked && slot.id == id {
                    break;
                } else if !slot.checked {
                    slot.checked = true;
                    slot.id = id;
                    emit = true;
                    break;
                }
            }
            emit
        };
        if emit {
            self.log_no_chan(id);
        }
    }

    // -- Command handlers -----------------------------------------------------

    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_SET_LEVEL => self.set_level_cmd_handler(op_code, cmd_seq, args),
            Self::OPCODE_SEND_PKT => self.send_pkt_cmd_handler(op_code, cmd_seq, args),
            Self::OPCODE_ENABLE_SECTION => self.enable_section_cmd_handler(op_code, cmd_seq, args),
            Self::OPCODE_ENABLE_GROUP => self.group_cmd_handler(op_code, cmd_seq, args, false),
            Self::OPCODE_FORCE_GROUP => self.group_cmd_handler(op_code, cmd_seq, args, true),
            Self::OPCODE_CONFIGURE_GROUP_RATES => {
                self.configure_group_rates_cmd_handler(op_code, cmd_seq, args);
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }

    /// `SET_LEVEL(level)`: enable groups `<= level`, disable the rest, in
    /// EVERY section.
    fn set_level_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut level: FwChanIdType = 0;
        if !args.deserialize_u32_be(&mut level).is_ok() || args.deserialize_size_left() != 0 {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        if level > MAX_CONFIGURABLE_TLMPACKETIZER_GROUP {
            self.log_max_level_exceed(level, MAX_CONFIGURABLE_TLMPACKETIZER_GROUP);
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        let configs = {
            let mut ctrl = self.ctrl.lock().unwrap();
            for section in 0..NUM_SECTIONS {
                for group in 0..NUM_CONFIGURABLE_TLMPACKETIZER_GROUPS {
                    ctrl.group_configs[section][group].enabled = if (group as FwChanIdType) <= level
                    {
                        Enabled::Enabled
                    } else {
                        Enabled::Disabled
                    };
                }
            }
            ctrl.group_configs
        };
        self.tlm_write_group_configs(&configs);
        self.log_level_set(level);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `SEND_PKT(id, section)`: mark one packet REQUESTED for one section.
    fn send_pkt_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        let mut id: u32 = 0;
        let mut raw_section: i32 = 0;
        if !args.deserialize_u32_be(&mut id).is_ok()
            || !args.deserialize_i32_be(&mut raw_section).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let Ok(section) = TelemetrySection::try_from(raw_section) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        let section_index = section.as_repr();
        if section_index < 0 || (section_index as usize) >= NUM_SECTIONS {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }

        let now = self.evt.time_get();
        let num_packets = self.table.read().unwrap().num_packets;
        let mut found = None;
        {
            let mut state = self.lock.lock().unwrap();
            for pkt in 0..num_packets {
                if state.fill_buffers[pkt].id == id {
                    // Gotcha: this also marks the SHARED fill buffer updated,
                    // so the other sections see the packet as new data too.
                    state.fill_buffers[pkt].updated = true;
                    state.fill_buffers[pkt].latest_time = now;
                    found = Some(pkt);
                    break;
                }
            }
        }
        match found {
            Some(pkt) => {
                {
                    let mut ctrl = self.ctrl.lock().unwrap();
                    ctrl.packet_flags[section_index as usize][pkt].update_flag =
                        UpdateFlag::Requested;
                }
                // Emitted at COMMAND time, before the packet is actually sent.
                self.log_packet_sent(id);
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            }
            None => {
                self.log_packet_not_found(id);
                self.cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            }
        }
    }

    /// `ENABLE_SECTION(section, enable)`.
    fn enable_section_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut raw_section: i32 = 0;
        let mut raw_enable: u8 = 0;
        if !args.deserialize_i32_be(&mut raw_section).is_ok()
            || !args.deserialize_u8_be(&mut raw_enable).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let (Ok(section), Ok(enable)) = (
            TelemetrySection::try_from(raw_section),
            Enabled::try_from(raw_enable),
        ) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        let section_index = section.as_repr();
        if section_index < 0 || (section_index as usize) >= NUM_SECTIONS {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        let section_enabled = {
            let mut ctrl = self.ctrl.lock().unwrap();
            ctrl.section_enabled[section_index as usize] = enable;
            ctrl.section_enabled
        };
        self.tlm.tlm_write(
            self.id_base(),
            Self::CHANID_SECTION_ENABLED,
            &section_enabled,
            self.evt.time_get(),
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `ENABLE_GROUP` / `FORCE_GROUP` (identical apart from the field set).
    fn group_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
        force: bool,
    ) {
        let mut raw_section: i32 = 0;
        let mut tlm_group: FwChanIdType = 0;
        let mut raw_enable: u8 = 0;
        if !args.deserialize_i32_be(&mut raw_section).is_ok()
            || !args.deserialize_u32_be(&mut tlm_group).is_ok()
            || !args.deserialize_u8_be(&mut raw_enable).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let (Ok(section), Ok(enable)) = (
            TelemetrySection::try_from(raw_section),
            Enabled::try_from(raw_enable),
        ) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        let section_index = section.as_repr();
        if section_index < 0
            || (section_index as usize) >= NUM_SECTIONS
            || tlm_group > MAX_CONFIGURABLE_TLMPACKETIZER_GROUP
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        let configs = {
            let mut ctrl = self.ctrl.lock().unwrap();
            let config = &mut ctrl.group_configs[section_index as usize][tlm_group as usize];
            if force {
                config.force_enabled = enable;
            } else {
                config.enabled = enable;
            }
            ctrl.group_configs
        };
        self.tlm_write_group_configs(&configs);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `CONFIGURE_GROUP_RATES(section, tlmGroup, rateLogic, min, max)`.
    fn configure_group_rates_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut raw_section: i32 = 0;
        let mut tlm_group: FwChanIdType = 0;
        let mut raw_logic: i32 = 0;
        let mut min_delta: u32 = 0;
        let mut max_delta: u32 = 0;
        if !args.deserialize_i32_be(&mut raw_section).is_ok()
            || !args.deserialize_u32_be(&mut tlm_group).is_ok()
            || !args.deserialize_i32_be(&mut raw_logic).is_ok()
            || !args.deserialize_u32_be(&mut min_delta).is_ok()
            || !args.deserialize_u32_be(&mut max_delta).is_ok()
            || args.deserialize_size_left() != 0
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
            return;
        }
        let (Ok(section), Ok(rate_logic)) = (
            TelemetrySection::try_from(raw_section),
            RateLogic::try_from(raw_logic),
        ) else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        };
        let section_index = section.as_repr();
        if section_index < 0
            || (section_index as usize) >= NUM_SECTIONS
            || tlm_group > MAX_CONFIGURABLE_TLMPACKETIZER_GROUP
        {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        self.configure_section_group_rate(section, tlm_group, rate_logic, min_delta, max_delta);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    // -- Telemetry / events ---------------------------------------------------

    fn tlm_write_group_configs(&self, configs: &SectionConfigs) {
        self.tlm.tlm_write(
            self.id_base(),
            Self::CHANID_GROUP_CONFIGS,
            configs,
            self.evt.time_get(),
        );
    }

    fn log_no_chan(&self, id: FwChanIdType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_NO_CHAN,
            LogSeverity::WarningLo,
            &format!("Telemetry ID 0x{id:x} not packetized"),
            |buf: &mut LogBuffer| buf.serialize_u32_be(id),
        );
    }

    fn log_level_set(&self, level: FwChanIdType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_LEVEL_SET,
            LogSeverity::ActivityHi,
            &format!("Telemetry send level to {level}"),
            |buf: &mut LogBuffer| buf.serialize_u32_be(level),
        );
    }

    fn log_max_level_exceed(&self, level: FwChanIdType, max: FwChanIdType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_MAX_LEVEL_EXCEED,
            LogSeverity::WarningLo,
            &format!("Requested send level {level} higher than max packet level of {max}"),
            |buf: &mut LogBuffer| {
                fw_try!(buf.serialize_u32_be(level));
                buf.serialize_u32_be(max)
            },
        );
    }

    fn log_packet_sent(&self, packet_id: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_PACKET_SENT,
            LogSeverity::ActivityLo,
            &format!("Sent packet ID {packet_id}"),
            |buf: &mut LogBuffer| buf.serialize_u32_be(packet_id),
        );
    }

    fn log_packet_not_found(&self, packet_id: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_PACKET_NOT_FOUND,
            LogSeverity::WarningLo,
            &format!("Could not find packet ID {packet_id}"),
            |buf: &mut LogBuffer| buf.serialize_u32_be(packet_id),
        );
    }

    fn log_section_unconfigurable(&self, section: TelemetrySection, enable: Enabled) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_SECTION_UNCONFIGURABLE,
            LogSeverity::WarningLo,
            &format!("Section {section:?} is unconfigurable and cannot be set to {enable:?}"),
            |buf: &mut LogBuffer| {
                fw_try!(section.serialize_to(buf, Endianness::Big));
                enable.serialize_to(buf, Endianness::Big)
            },
        );
    }

    fn log_oversized_channel(&self, id: FwChanIdType, val_size: FwSizeType, expected: FwSizeType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_OVERSIZED_CHANNEL,
            LogSeverity::WarningHi,
            &format!(
                "Telemetry ID 0x{id:x} update of size {val_size} exceeds configured size {expected}"
            ),
            |buf: &mut LogBuffer| {
                fw_try!(buf.serialize_u32_be(id));
                fw_try!(buf.serialize_u64_be(val_size));
                buf.serialize_u64_be(expected)
            },
        );
    }
}

// -- Sync input adapters -----------------------------------------------------

input_port_adapter! {
    /// `TlmRecv` — SYNC `Fw.Tlm` input: store one channel value.
    component: TlmPacketizer;
    adapter: TlmRecvAdapter;
    port: TlmPort;
    input: pub tlm_recv_in;
    handler: tlm_recv_handler;
    args { val id: FwChanIdType, mut time_tag: Time, mut val: TlmBuffer }
}

input_port_adapter! {
    /// `TlmGet` — SYNC `Fw.TlmGet` input: read one stored channel value.
    component: TlmPacketizer;
    adapter: TlmGetAdapter;
    port: TlmGetPort;
    input: pub tlm_get_in;
    handler: tlm_get_handler;
    returns: TlmValid;
    args { val id: FwChanIdType, mut time_tag: Time, mut val: TlmBuffer }
}

// -- Async input adapters ----------------------------------------------------

async_input_port_adapter! {
    /// `controlIn` — ASYNC `Svc.EnableSection` input.
    component: TlmPacketizer;
    adapter: ControlInAdapter;
    port: EnableSectionPort;
    input: pub control_in;
    deserialize: control_in_deserialize;
    handler: control_in_handler;
    base: active.queued;
    msg_type: TlmPacketizer::MSG_TYPE_CONTROL_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val section: TelemetrySection, val enabled: Enabled }
}

async_input_port_adapter! {
    /// `pingIn` — ASYNC `Svc.Ping` input.
    component: TlmPacketizer;
    adapter: PingInAdapter;
    port: PingPort;
    input: pub ping_in;
    deserialize: ping_in_deserialize;
    handler: ping_in_handler;
    base: active.queued;
    msg_type: TlmPacketizer::MSG_TYPE_PING_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val key: u32 }
}

async_input_port_adapter! {
    /// `Run` — ASYNC `Svc.Sched` input: the packet send cycle.
    component: TlmPacketizer;
    adapter: RunAdapter;
    port: SchedPort;
    input: pub run_in;
    deserialize: run_deserialize;
    handler: run_handler;
    base: active.queued;
    msg_type: TlmPacketizer::MSG_TYPE_RUN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val context: u32 }
}

async_input_port_adapter! {
    /// `configureSectionGroupRate` — ASYNC `Svc.ConfigureGroupRate` input.
    component: TlmPacketizer;
    adapter: ConfigureSectionGroupRateAdapter;
    port: ConfigureGroupRatePort;
    input: pub configure_section_group_rate_in;
    deserialize: configure_section_group_rate_deserialize;
    handler: configure_section_group_rate_handler;
    base: active.queued;
    msg_type: TlmPacketizer::MSG_TYPE_CONFIGURE_SECTION_GROUP_RATE;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args {
        val section: TelemetrySection,
        val tlm_group: FwChanIdType,
        val rate_logic: RateLogic,
        val min_delta: u32,
        val max_delta: u32
    }
}

async_input_port_adapter! {
    /// `cmdIn` — ASYNC `Fw.Cmd` input for this component's six commands.
    component: TlmPacketizer;
    adapter: CmdInAdapter;
    port: CmdPort;
    input: pub cmd_in;
    deserialize: cmd_in_deserialize;
    handler: cmd_in_handler;
    base: active.queued;
    msg_type: TlmPacketizer::MSG_TYPE_CMD_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, buf args: CmdArgBuffer }
}

// -- Dispatch (the generated `doDispatch` switch) -----------------------------

impl ComponentDispatch for TlmPacketizer {
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
            Self::MSG_TYPE_CONTROL_IN => match Self::control_in_deserialize(buf) {
                Some((section, enabled)) => {
                    self.control_in_handler(port_num, section, enabled);
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
            Self::MSG_TYPE_RUN => match Self::run_deserialize(buf) {
                Some((context,)) => {
                    self.run_handler(port_num, context);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_CONFIGURE_SECTION_GROUP_RATE => {
                match Self::configure_section_group_rate_deserialize(buf) {
                    Some((section, tlm_group, rate_logic, min_delta, max_delta)) => {
                        self.configure_section_group_rate_handler(
                            port_num, section, tlm_group, rate_logic, min_delta, max_delta,
                        );
                        MsgDispatchStatus::Ok
                    }
                    None => MsgDispatchStatus::Error,
                }
            }
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

impl ActiveComponent for TlmPacketizer {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort, TimePort};
    use fprime_fw::{TextLogString, TimeBase};
    use fprime_os::task::{Status as TaskStatus, TASK_DEFAULT};

    const ID_BASE: u32 = 0x7000;

    /// Two-packet table: packet 4 (level 1) holds channels 10 (U32) and 11
    /// (U16); packet 8 (level 2) holds channel 10 again plus channel 12 (U8).
    const PKT4_CHANNELS: [TlmPacketizerChannelEntry; 2] = [
        TlmPacketizerChannelEntry::new(10, 4),
        TlmPacketizerChannelEntry::new(11, 2),
    ];
    const PKT8_CHANNELS: [TlmPacketizerChannelEntry; 2] = [
        TlmPacketizerChannelEntry::new(10, 4),
        TlmPacketizerChannelEntry::new(12, 1),
    ];
    const IGNORED: [TlmPacketizerChannelEntry; 1] = [TlmPacketizerChannelEntry::new(99, 4)];

    fn packet_list() -> Vec<TlmPacketizerPacket<'static>> {
        vec![
            TlmPacketizerPacket::new(&PKT4_CHANNELS, 4, 1),
            TlmPacketizerPacket::new(&PKT8_CHANNELS, 8, 2),
        ]
    }

    /// (port index, packet bytes, context)
    type PacketRecord = (FwIndexType, Vec<u8>, u32);
    /// (id, severity, raw arg bytes)
    type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

    #[derive(Default)]
    struct Ground {
        packets: Mutex<Vec<PacketRecord>>,
        events: Mutex<Vec<EventRecord>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        pings: Mutex<Vec<u32>>,
    }

    impl ComPort for Ground {
        fn invoke(&self, port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
            self.packets
                .lock()
                .unwrap()
                .push((port_num, data.as_slice().to_vec(), context));
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
            _text: &mut TextLogString,
        ) {
        }
    }

    impl fprime_comp::TlmPort for Ground {
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

    /// Fixed time source: 0x11223344 seconds, 0x00051615 useconds
    /// (333333 decimal), TB_WORKSTATION_TIME (2), context 0.
    struct TimeStub;
    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 0x1122_3344, 333_333);
        }
    }

    /// Expected 11 time bytes for [`TimeStub`].
    const TIME_BYTES: [u8; 11] = [
        0x00, 0x02, // time base = TB_WORKSTATION_TIME
        0x00, // context
        0x11, 0x22, 0x33, 0x44, // seconds
        0x00, 0x05, 0x16, 0x15, // useconds = 333333
    ];

    /// Wire both `PktSend` ports unless `ports` says otherwise.
    fn build(connect_ports: &[bool; TELEMETRY_SEND_PORTS]) -> (Arc<TlmPacketizer>, Arc<Ground>) {
        let ground = Arc::new(Ground::default());
        let comp = TlmPacketizer::new("tlmPack");
        comp.active.queued.base.set_id_base(ID_BASE);
        for (index, connect) in connect_ports.iter().enumerate() {
            if *connect {
                comp.pkt_send[index].connect(ground.clone(), index as FwIndexType);
            }
        }
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.evt.time_out.connect(Arc::new(TimeStub), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.ping_out.connect(ground.clone(), 0);
        comp.init(32);
        (comp, ground)
    }

    fn configured(
        connect_ports: &[bool; TELEMETRY_SEND_PORTS],
    ) -> (Arc<TlmPacketizer>, Arc<Ground>) {
        let (comp, ground) = build(connect_ports);
        comp.set_packet_list(&packet_list(), &IGNORED, 0);
        (comp, ground)
    }

    fn recv(comp: &Arc<TlmPacketizer>, id: FwChanIdType, value: &[u8]) {
        let mut time = Time::new(TimeBase::TbWorkstationTime, 0, 0x1122_3344, 333_333);
        let mut buf = TlmBuffer::new();
        assert!(buf.set_buff(value).is_ok());
        let port = comp.tlm_recv_in(0);
        port.target.invoke(port.port_num, id, &mut time, &mut buf);
    }

    /// Run one `Run` tick synchronously (the handler is what the component
    /// thread would call).
    fn run_tick(comp: &Arc<TlmPacketizer>) {
        comp.run_handler(0, 0);
    }

    fn send_cmd(comp: &Arc<TlmPacketizer>, opcode: FwOpcodeType, seq: u32, arg_bytes: &[u8]) {
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(arg_bytes).is_ok());
        comp.cmd_in_handler(0, ID_BASE + opcode, seq, &mut args);
    }

    // -- Packet table ---------------------------------------------------------

    #[test]
    fn packetized_packet_bytes_are_byte_exact() {
        let (comp, ground) = configured(&[true, true]);
        // Packet 4 = 15 header + 4 + 2 = 21 bytes; packet 8 = 15 + 4 + 1 = 20.
        recv(&comp, 10, &[0xDE, 0xAD, 0xBE, 0xEF]);
        recv(&comp, 11, &[0x12, 0x34]);
        recv(&comp, 12, &[0x7F]);
        run_tick(&comp);

        let packets = ground.packets.lock().unwrap();
        // Both sections are ENABLED by default, so each packet is emitted
        // once per section: section 0 -> port 0, section 1 -> port 1. The
        // packet loop is the outer one.
        assert_eq!(packets.len(), 4, "two packets x two sections");

        let mut expected4 = vec![0x00, 0x04, 0x00, 0x04];
        expected4.extend_from_slice(&TIME_BYTES);
        expected4.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34]);
        assert_eq!(packets[0].0, 0, "section 0 -> port 0");
        assert_eq!(packets[0].1, expected4);
        // prevSentCounter saturated at u32::MAX and is not incremented.
        assert_eq!(packets[0].2, u32::MAX);
        assert_eq!(packets[1].0, 1, "section 1 -> port 1");
        assert_eq!(packets[1].1, expected4);

        let mut expected8 = vec![0x00, 0x04, 0x00, 0x08];
        expected8.extend_from_slice(&TIME_BYTES);
        expected8.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF, 0x7F]);
        assert_eq!(packets[2].0, 0);
        assert_eq!(packets[2].1, expected8);
        assert_eq!(packets[3].0, 1);
        assert_eq!(packets[3].1, expected8);
    }

    #[test]
    fn unwritten_channels_stay_zero_and_the_time_is_rewritten_in_place() {
        let (comp, ground) = configured(&[true, true]);
        // Only channel 11 arrives: channel 10's four bytes stay zero.
        recv(&comp, 11, &[0xAB, 0xCD]);
        run_tick(&comp);
        let packets = ground.packets.lock().unwrap();
        let mut expected = vec![0x00, 0x04, 0x00, 0x04];
        expected.extend_from_slice(&TIME_BYTES);
        expected.extend_from_slice(&[0x00, 0x00, 0x00, 0x00, 0xAB, 0xCD]);
        assert_eq!(packets[0].1, expected);
    }

    #[test]
    fn a_channel_in_two_packets_is_copied_into_both() {
        let (comp, ground) = configured(&[true, true]);
        recv(&comp, 10, &[1, 2, 3, 4]);
        run_tick(&comp);
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 4, "two packets x two sections");
        assert_eq!(&packets[0].1[15..19], &[1, 2, 3, 4], "packet id 4");
        assert_eq!(&packets[2].1[15..19], &[1, 2, 3, 4], "packet id 8");
    }

    #[test]
    fn missing_channel_warns_once_per_id_and_stops_after_the_table_fills() {
        let (comp, ground) = configured(&[true, true]);
        recv(&comp, 0xAA, &[1]);
        recv(&comp, 0xAA, &[1]); // same id: no second event
        recv(&comp, 0xBB, &[1]);
        {
            let events = ground.events.lock().unwrap();
            assert_eq!(events.len(), 2);
            assert_eq!(events[0].0, ID_BASE + TlmPacketizer::EVENTID_NO_CHAN);
            assert_eq!(events[0].1, LogSeverity::WarningLo);
            assert_eq!(events[0].2, 0xAAu32.to_be_bytes().to_vec());
        }
        // Fill the remaining 23 slots, then confirm the table never resets.
        for id in 0..(TLMPACKETIZER_MAX_MISSING_TLM_CHECK as FwChanIdType - 2) {
            recv(&comp, 1000 + id, &[1]);
        }
        assert_eq!(
            ground.events.lock().unwrap().len(),
            TLMPACKETIZER_MAX_MISSING_TLM_CHECK
        );
        recv(&comp, 0xCCCC, &[1]);
        assert_eq!(
            ground.events.lock().unwrap().len(),
            TLMPACKETIZER_MAX_MISSING_TLM_CHECK,
            "no NoChan events once all 25 slots are used"
        );
    }

    #[test]
    fn ignored_channels_are_silently_dropped() {
        let (comp, ground) = configured(&[true, true]);
        recv(&comp, 99, &[1, 2, 3, 4]);
        assert!(ground.events.lock().unwrap().is_empty());
        run_tick(&comp);
        assert!(
            ground.packets.lock().unwrap().is_empty(),
            "an ignored channel does not mark any packet updated"
        );
    }

    #[test]
    fn oversized_values_are_rejected_with_a_throttled_warning() {
        let (comp, ground) = configured(&[true, true]);
        for _ in 0..(TlmPacketizer::OVERSIZED_CHANNEL_THROTTLE + 3) {
            recv(&comp, 11, &[1, 2, 3, 4, 5]); // channel 11 is 2 bytes
        }
        let events = ground.events.lock().unwrap();
        assert_eq!(
            events.len(),
            TlmPacketizer::OVERSIZED_CHANNEL_THROTTLE as usize
        );
        assert_eq!(
            events[0].0,
            ID_BASE + TlmPacketizer::EVENTID_OVERSIZED_CHANNEL
        );
        assert_eq!(events[0].1, LogSeverity::WarningHi);
        let mut expected = 11u32.to_be_bytes().to_vec();
        expected.extend_from_slice(&5u64.to_be_bytes());
        expected.extend_from_slice(&2u64.to_be_bytes());
        assert_eq!(events[0].2, expected);
        drop(events);
        run_tick(&comp);
        assert!(ground.packets.lock().unwrap().is_empty());
    }

    #[test]
    fn tlm_get_returns_the_padded_value_and_the_packet_time() {
        let (comp, _ground) = configured(&[true, true]);
        let mut time = Time::ZERO;
        let mut val = TlmBuffer::new();
        let port = comp.tlm_get_in(0);

        // Never written: INVALID with an empty buffer.
        assert_eq!(
            port.target.invoke(port.port_num, 10, &mut time, &mut val),
            TlmValid::Invalid
        );
        assert_eq!(val.get_size(), 0);

        recv(&comp, 10, &[9, 8, 7, 6]);
        assert_eq!(
            port.target.invoke(port.port_num, 10, &mut time, &mut val),
            TlmValid::Valid
        );
        assert_eq!(val.as_slice(), &[9, 8, 7, 6]);
        assert_eq!(time.get_seconds(), 0x1122_3344);

        // Ignored and unknown channels are INVALID.
        assert_eq!(
            port.target.invoke(port.port_num, 99, &mut time, &mut val),
            TlmValid::Invalid
        );
        assert_eq!(
            port.target
                .invoke(port.port_num, 0xDEAD, &mut time, &mut val),
            TlmValid::Invalid
        );
    }

    #[test]
    fn tlm_get_pads_a_short_value_to_the_declared_channel_size() {
        // Gotcha: the returned buffer is the table's MAX size, so a shorter
        // value keeps whatever was in the packet's tail (zeros here).
        let (comp, _ground) = configured(&[true, true]);
        recv(&comp, 10, &[0xAA, 0xBB]);
        let mut time = Time::ZERO;
        let mut val = TlmBuffer::new();
        let port = comp.tlm_get_in(0);
        assert_eq!(
            port.target.invoke(port.port_num, 10, &mut time, &mut val),
            TlmValid::Valid
        );
        assert_eq!(val.as_slice(), &[0xAA, 0xBB, 0x00, 0x00]);
    }

    #[test]
    fn set_packet_list_is_reentrant() {
        let (comp, ground) = configured(&[true, true]);
        recv(&comp, 10, &[1, 1, 1, 1]);
        // Reconfigure with a single one-channel packet.
        const ONLY: [TlmPacketizerChannelEntry; 1] = [TlmPacketizerChannelEntry::new(11, 2)];
        comp.set_packet_list(
            &[TlmPacketizerPacket::new(&ONLY, 7, 0)],
            IGNORE_OMIT_LIST,
            0,
        );
        // Channel 10 is no longer packetized.
        recv(&comp, 10, &[2, 2, 2, 2]);
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].0, ID_BASE + TlmPacketizer::EVENTID_NO_CHAN);
        drop(events);

        ground.packets.lock().unwrap().clear();
        recv(&comp, 11, &[0x55, 0x66]);
        run_tick(&comp);
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 2, "level 0: section 0 + section 1");
        let mut expected = vec![0x00, 0x04, 0x00, 0x07];
        expected.extend_from_slice(&TIME_BYTES);
        expected.extend_from_slice(&[0x55, 0x66]);
        assert_eq!(packets[0].1, expected);
    }

    // -- Rate logic -----------------------------------------------------------

    #[test]
    fn a_packet_without_data_is_never_sent() {
        let (comp, ground) = configured(&[true, true]);
        run_tick(&comp);
        run_tick(&comp);
        assert!(
            ground.packets.lock().unwrap().is_empty(),
            "NEVER_UPDATED packets are skipped"
        );
    }

    #[test]
    fn on_change_min_only_sends_on_new_data() {
        let (comp, ground) = configured(&[true, true]);
        recv(&comp, 11, &[1, 2]);
        run_tick(&comp);
        // Packet id 4 only (channel 11 is not in packet id 8), once per
        // section.
        assert_eq!(ground.packets.lock().unwrap().len(), 2);
        // No new data: ON_CHANGE_MIN (the default) does not resend.
        run_tick(&comp);
        assert_eq!(ground.packets.lock().unwrap().len(), 2);
        recv(&comp, 11, &[3, 4]);
        run_tick(&comp);
        assert_eq!(ground.packets.lock().unwrap().len(), 4);
        // The second send reports the ticks since the previous one.
        assert_eq!(ground.packets.lock().unwrap()[2].2, 2);
    }

    #[test]
    fn every_max_sends_on_the_max_interval_without_new_data() {
        let (comp, ground) = configured(&[true, true]);
        // Section 0, group 1 (packet 4): EVERY_MAX with max = 3.
        let mut args = Vec::new();
        args.extend_from_slice(&0i32.to_be_bytes());
        args.extend_from_slice(&1u32.to_be_bytes());
        args.extend_from_slice(&(RateLogic::EveryMax.as_repr()).to_be_bytes());
        args.extend_from_slice(&0u32.to_be_bytes());
        args.extend_from_slice(&3u32.to_be_bytes());
        send_cmd(&comp, TlmPacketizer::OPCODE_CONFIGURE_GROUP_RATES, 1, &args);

        recv(&comp, 11, &[1, 2]);
        run_tick(&comp); // counter saturated at u32::MAX >= 3 -> sends
        assert_eq!(ground.packets.lock().unwrap()[0].2, u32::MAX);
        ground.packets.lock().unwrap().clear();

        run_tick(&comp); // counter 1
        run_tick(&comp); // counter 2
        assert!(ground.packets.lock().unwrap().is_empty());
        run_tick(&comp); // counter 3 == max -> sends
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].2, 3);
    }

    #[test]
    fn silenced_groups_never_send_and_freeze_the_counter() {
        let (comp, ground) = configured(&[true, true]);
        let mut args = Vec::new();
        args.extend_from_slice(&0i32.to_be_bytes());
        args.extend_from_slice(&1u32.to_be_bytes());
        args.extend_from_slice(&(RateLogic::Silenced.as_repr()).to_be_bytes());
        args.extend_from_slice(&0u32.to_be_bytes());
        args.extend_from_slice(&0u32.to_be_bytes());
        send_cmd(&comp, TlmPacketizer::OPCODE_CONFIGURE_GROUP_RATES, 1, &args);
        recv(&comp, 11, &[1, 2]);
        run_tick(&comp);
        run_tick(&comp);
        assert!(
            ground
                .packets
                .lock()
                .unwrap()
                .iter()
                .all(|(port, _, _)| *port != 0),
            "the silenced section/group never sends on port 0"
        );
    }

    #[test]
    fn a_disabled_section_stops_sends_and_force_enable_overrides_it() {
        let (comp, ground) = configured(&[true, true]);
        // Disable section 0 (REALTIME).
        let mut args = 0i32.to_be_bytes().to_vec();
        args.push(Enabled::Disabled as u8);
        send_cmd(&comp, TlmPacketizer::OPCODE_ENABLE_SECTION, 1, &args);

        recv(&comp, 11, &[1, 2]);
        run_tick(&comp);
        assert!(
            ground
                .packets
                .lock()
                .unwrap()
                .iter()
                .all(|(port, _, _)| *port != 0)
        );

        // FORCE_GROUP(section 0, group 1, ENABLED) overrides the section.
        let mut args = 0i32.to_be_bytes().to_vec();
        args.extend_from_slice(&1u32.to_be_bytes());
        args.push(Enabled::Enabled as u8);
        send_cmd(&comp, TlmPacketizer::OPCODE_FORCE_GROUP, 2, &args);
        recv(&comp, 11, &[3, 4]);
        run_tick(&comp);
        assert!(
            ground
                .packets
                .lock()
                .unwrap()
                .iter()
                .any(|(port, _, _)| *port == 0)
        );
    }

    #[test]
    fn unconnected_ports_are_skipped() {
        // Only port 0 is wired: the level-2 packet (section 1 -> port 1) is
        // never emitted.
        let (comp, ground) = configured(&[true, false]);
        recv(&comp, 10, &[1, 2, 3, 4]);
        run_tick(&comp);
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 2, "both packets go out on port 0");
        assert!(packets.iter().all(|(port, _, _)| *port == 0));
    }

    // -- Commands -------------------------------------------------------------

    #[test]
    fn set_level_enables_groups_up_to_the_level() {
        let (comp, ground) = configured(&[true, true]);
        comp.reg_commands();
        assert_eq!(
            *ground.regs.lock().unwrap(),
            vec![
                ID_BASE + TlmPacketizer::OPCODE_SET_LEVEL,
                ID_BASE + TlmPacketizer::OPCODE_SEND_PKT,
                ID_BASE + TlmPacketizer::OPCODE_ENABLE_SECTION,
                ID_BASE + TlmPacketizer::OPCODE_ENABLE_GROUP,
                ID_BASE + TlmPacketizer::OPCODE_FORCE_GROUP,
                ID_BASE + TlmPacketizer::OPCODE_CONFIGURE_GROUP_RATES,
            ]
        );

        // Level 1: groups 0 and 1 enabled, 2 and 3 disabled -> the level-2
        // packet (id 8) stops being sent.
        send_cmd(
            &comp,
            TlmPacketizer::OPCODE_SET_LEVEL,
            1,
            &1u32.to_be_bytes(),
        );
        recv(&comp, 10, &[1, 2, 3, 4]);
        run_tick(&comp);
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 2, "packet id 4 only, once per section");
        assert_eq!(&packets[0].1[2..4], &[0x00, 0x04], "only packet id 4");
        assert_eq!(&packets[1].1[2..4], &[0x00, 0x04]);
        drop(packets);

        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].0, ID_BASE + TlmPacketizer::EVENTID_LEVEL_SET);
        assert_eq!(events[0].1, LogSeverity::ActivityHi);
        assert_eq!(events[0].2, 1u32.to_be_bytes().to_vec());
        drop(events);

        // GroupConfigs telemetry: 112 bytes, groups 0/1 ENABLED, 2/3 DISABLED.
        let tlm = ground.tlm.lock().unwrap();
        assert_eq!(tlm.len(), 1);
        assert_eq!(tlm[0].0, ID_BASE + TlmPacketizer::CHANID_GROUP_CONFIGS);
        assert_eq!(tlm[0].1.len(), 112);
        assert_eq!(tlm[0].1[0], Enabled::Enabled as u8); // section 0, group 0
        assert_eq!(
            tlm[0].1[GroupConfig::SERIALIZED_SIZE],
            Enabled::Enabled as u8
        );
        assert_eq!(
            tlm[0].1[2 * GroupConfig::SERIALIZED_SIZE],
            Enabled::Disabled as u8
        );
        assert_eq!(
            tlm[0].1[3 * GroupConfig::SERIALIZED_SIZE],
            Enabled::Disabled as u8
        );
    }

    #[test]
    fn set_level_above_the_max_is_a_validation_error() {
        let (comp, ground) = configured(&[true, true]);
        send_cmd(
            &comp,
            TlmPacketizer::OPCODE_SET_LEVEL,
            1,
            &4u32.to_be_bytes(),
        );
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0].0,
            ID_BASE + TlmPacketizer::EVENTID_MAX_LEVEL_EXCEED
        );
        let mut expected = 4u32.to_be_bytes().to_vec();
        expected.extend_from_slice(&MAX_CONFIGURABLE_TLMPACKETIZER_GROUP.to_be_bytes());
        assert_eq!(events[0].2, expected);
        drop(events);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(
                ID_BASE + TlmPacketizer::OPCODE_SET_LEVEL,
                1,
                CmdResponse::ValidationError
            )]
        );
    }

    #[test]
    fn send_pkt_requests_one_section_and_marks_the_shared_buffer_updated() {
        let (comp, ground) = configured(&[true, true]);
        // Silence every group so only the REQUESTED bypass can send.
        for section in 0..NUM_SECTIONS {
            for group in 0..NUM_CONFIGURABLE_TLMPACKETIZER_GROUPS {
                let mut args = (section as i32).to_be_bytes().to_vec();
                args.extend_from_slice(&(group as u32).to_be_bytes());
                args.extend_from_slice(&RateLogic::Silenced.as_repr().to_be_bytes());
                args.extend_from_slice(&0u32.to_be_bytes());
                args.extend_from_slice(&0u32.to_be_bytes());
                send_cmd(&comp, TlmPacketizer::OPCODE_CONFIGURE_GROUP_RATES, 1, &args);
            }
        }
        ground.events.lock().unwrap().clear();

        // Request packet id 4 on section 0.
        let mut args = 4u32.to_be_bytes().to_vec();
        args.extend_from_slice(&0i32.to_be_bytes());
        send_cmd(&comp, TlmPacketizer::OPCODE_SEND_PKT, 9, &args);

        // PacketSent is emitted at COMMAND time, before any Run tick.
        {
            let events = ground.events.lock().unwrap();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].0, ID_BASE + TlmPacketizer::EVENTID_PACKET_SENT);
            assert_eq!(events[0].1, LogSeverity::ActivityLo);
            assert_eq!(events[0].2, 4u32.to_be_bytes().to_vec());
        }
        assert!(ground.packets.lock().unwrap().is_empty());

        run_tick(&comp);
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 1, "only the requested section sends");
        assert_eq!(packets[0].0, 0);
        assert_eq!(&packets[0].1[2..4], &[0x00, 0x04]);
        drop(packets);
        assert_eq!(
            ground.responses.lock().unwrap().last().copied(),
            Some((ID_BASE + TlmPacketizer::OPCODE_SEND_PKT, 9, CmdResponse::Ok))
        );
    }

    #[test]
    fn send_pkt_for_an_unknown_packet_is_a_validation_error() {
        let (comp, ground) = configured(&[true, true]);
        let mut args = 77u32.to_be_bytes().to_vec();
        args.extend_from_slice(&0i32.to_be_bytes());
        send_cmd(&comp, TlmPacketizer::OPCODE_SEND_PKT, 3, &args);
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0].0,
            ID_BASE + TlmPacketizer::EVENTID_PACKET_NOT_FOUND
        );
        assert_eq!(events[0].2, 77u32.to_be_bytes().to_vec());
        drop(events);
        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![(
                ID_BASE + TlmPacketizer::OPCODE_SEND_PKT,
                3,
                CmdResponse::ValidationError
            )]
        );
    }

    #[test]
    fn enable_section_writes_the_section_enabled_channel() {
        let (comp, ground) = configured(&[true, true]);
        let mut args = 1i32.to_be_bytes().to_vec();
        args.push(Enabled::Disabled as u8);
        send_cmd(&comp, TlmPacketizer::OPCODE_ENABLE_SECTION, 4, &args);
        let tlm = ground.tlm.lock().unwrap();
        assert_eq!(tlm.len(), 1);
        assert_eq!(tlm[0].0, ID_BASE + TlmPacketizer::CHANID_SECTION_ENABLED);
        assert_eq!(
            tlm[0].1,
            vec![Enabled::Enabled as u8, Enabled::Disabled as u8]
        );
    }

    #[test]
    fn control_in_port_sets_the_state_without_writing_telemetry() {
        let (comp, ground) = configured(&[true, true]);
        comp.control_in_handler(0, TelemetrySection::Recorded, Enabled::Disabled);
        // Gotcha: no SectionEnabled telemetry write on this path.
        assert!(ground.tlm.lock().unwrap().is_empty());
        assert!(ground.events.lock().unwrap().is_empty());

        // Out of range (NUM_SECTIONS is a valid enum constant) -> event.
        comp.control_in_handler(0, TelemetrySection::NumSections, Enabled::Enabled);
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0].0,
            ID_BASE + TlmPacketizer::EVENTID_SECTION_UNCONFIGURABLE
        );
        let mut expected = 2i32.to_be_bytes().to_vec();
        expected.push(Enabled::Enabled as u8);
        assert_eq!(events[0].2, expected);
    }

    #[test]
    fn command_status_branches() {
        let (comp, ground) = configured(&[true, true]);
        let set_level = ID_BASE + TlmPacketizer::OPCODE_SET_LEVEL;
        send_cmd(&comp, TlmPacketizer::OPCODE_SET_LEVEL, 1, &[0, 0]); // short
        send_cmd(&comp, TlmPacketizer::OPCODE_SET_LEVEL, 2, &[0, 0, 0, 1, 9]); // residual
        // SEND_PKT with an undeclared TelemetrySection value.
        let mut args = 4u32.to_be_bytes().to_vec();
        args.extend_from_slice(&9i32.to_be_bytes());
        send_cmd(&comp, TlmPacketizer::OPCODE_SEND_PKT, 3, &args);
        // SEND_PKT with the NUM_SECTIONS counter constant (valid enum, out of
        // range section).
        let mut args = 4u32.to_be_bytes().to_vec();
        args.extend_from_slice(&2i32.to_be_bytes());
        send_cmd(&comp, TlmPacketizer::OPCODE_SEND_PKT, 4, &args);
        // ENABLE_GROUP with a group above the max.
        let mut args = 0i32.to_be_bytes().to_vec();
        args.extend_from_slice(&4u32.to_be_bytes());
        args.push(Enabled::Enabled as u8);
        send_cmd(&comp, TlmPacketizer::OPCODE_ENABLE_GROUP, 5, &args);
        // ENABLE_SECTION with an undeclared Fw.Enabled value.
        let mut args = 0i32.to_be_bytes().to_vec();
        args.push(0x07);
        send_cmd(&comp, TlmPacketizer::OPCODE_ENABLE_SECTION, 6, &args);
        // CONFIGURE_GROUP_RATES with an undeclared RateLogic value.
        let mut args = 0i32.to_be_bytes().to_vec();
        args.extend_from_slice(&0u32.to_be_bytes());
        args.extend_from_slice(&9i32.to_be_bytes());
        args.extend_from_slice(&0u32.to_be_bytes());
        args.extend_from_slice(&0u32.to_be_bytes());
        send_cmd(&comp, TlmPacketizer::OPCODE_CONFIGURE_GROUP_RATES, 7, &args);
        // Unknown opcode.
        send_cmd(&comp, 0x40, 8, &[]);

        assert_eq!(
            *ground.responses.lock().unwrap(),
            vec![
                (set_level, 1, CmdResponse::FormatError),
                (set_level, 2, CmdResponse::FormatError),
                (
                    ID_BASE + TlmPacketizer::OPCODE_SEND_PKT,
                    3,
                    CmdResponse::ValidationError
                ),
                (
                    ID_BASE + TlmPacketizer::OPCODE_SEND_PKT,
                    4,
                    CmdResponse::ValidationError
                ),
                (
                    ID_BASE + TlmPacketizer::OPCODE_ENABLE_GROUP,
                    5,
                    CmdResponse::ValidationError
                ),
                (
                    ID_BASE + TlmPacketizer::OPCODE_ENABLE_SECTION,
                    6,
                    CmdResponse::ValidationError
                ),
                (
                    ID_BASE + TlmPacketizer::OPCODE_CONFIGURE_GROUP_RATES,
                    7,
                    CmdResponse::ValidationError
                ),
                (ID_BASE + 0x40, 8, CmdResponse::InvalidOpcode),
            ]
        );
    }

    // -- Wire formats / parameters -------------------------------------------

    #[test]
    fn section_configs_defaults_serialize_to_112_bytes() {
        let configs = SectionConfigs::default();
        let mut buf = fprime_fw::LinearBuffer::<128>::new();
        assert!(configs.serialize_to(&mut buf, Endianness::Big).is_ok());
        assert_eq!(buf.as_slice().len(), 112);
        // One GroupConfig = ENABLED, DISABLED, ON_CHANGE_MIN, 0, 0.
        assert_eq!(
            &buf.as_slice()[..GroupConfig::SERIALIZED_SIZE],
            &[
                0x01, // enabled = ENABLED
                0x00, // forceEnabled = DISABLED
                0x00, 0x00, 0x00, 0x02, // rateLogic = ON_CHANGE_MIN (I32)
                0x00, 0x00, 0x00, 0x00, // min
                0x00, 0x00, 0x00, 0x00, // max
            ]
        );
        // Every one of the 8 slots is the same default.
        for slot in 0..(NUM_SECTIONS * NUM_CONFIGURABLE_TLMPACKETIZER_GROUPS) {
            let start = slot * GroupConfig::SERIALIZED_SIZE;
            assert_eq!(
                &buf.as_slice()[start..start + GroupConfig::SERIALIZED_SIZE],
                &buf.as_slice()[..GroupConfig::SERIALIZED_SIZE]
            );
        }
        assert_eq!(SectionEnabled::default().serialized_size(), 2);
    }

    #[test]
    fn external_parameters_round_trip() {
        let (comp, _ground) = configured(&[true, true]);
        // Serialize the defaults out.
        let mut buf = ParamBuffer::new();
        assert!(
            comp.serialize_param(ID_BASE, TlmPacketizer::PARAMID_SECTION_CONFIGS, &mut buf)
                .is_ok()
        );
        assert_eq!(buf.get_size(), 112);

        // Load a modified value back in.
        let mut configs = SectionConfigs::default();
        configs[0][0].max = 7;
        configs[0][0].rate_logic = RateLogic::EveryMax;
        let mut in_buf = ParamBuffer::new();
        assert!(configs.serialize_to(&mut in_buf, Endianness::Big).is_ok());
        assert!(
            comp.deserialize_param(
                ID_BASE,
                TlmPacketizer::PARAMID_SECTION_CONFIGS,
                ParamValid::Valid,
                &mut in_buf
            )
            .is_ok()
        );
        assert_eq!(comp.ctrl.lock().unwrap().group_configs, configs);

        // A not-OK parameter status leaves the value alone.
        let mut other = SectionConfigs::default();
        other[1][1].min = 42;
        let mut in_buf = ParamBuffer::new();
        assert!(other.serialize_to(&mut in_buf, Endianness::Big).is_ok());
        assert_eq!(
            comp.deserialize_param(
                ID_BASE,
                TlmPacketizer::PARAMID_SECTION_CONFIGS,
                ParamValid::Invalid,
                &mut in_buf
            ),
            SerializeStatus::DeserTypeMismatch
        );
        assert_eq!(comp.ctrl.lock().unwrap().group_configs, configs);

        // SECTION_ENABLED is 2 bytes.
        let mut buf = ParamBuffer::new();
        assert!(
            comp.serialize_param(ID_BASE, TlmPacketizer::PARAMID_SECTION_ENABLED, &mut buf)
                .is_ok()
        );
        assert_eq!(buf.as_slice(), &[0x01, 0x01]);
    }

    // -- Threading / envelopes -----------------------------------------------

    #[test]
    fn run_envelope_bytes_are_byte_exact() {
        let (comp, _ground) = configured(&[true, true]);
        let run = comp.run_in(2);
        run.target.invoke(run.port_num, 0x0A0B_0C0D);
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
                0x00, 0x00, 0x00, 0x03, // msg_type = MSG_TYPE_RUN
                0x00, 0x02, // port_num
                0x0A, 0x0B, 0x0C, 0x0D, // context
            ]
        );
    }

    #[test]
    fn control_in_envelope_bytes_are_byte_exact() {
        let (comp, _ground) = configured(&[true, true]);
        let control = comp.control_in(0);
        control.target.invoke(
            control.port_num,
            TelemetrySection::Recorded,
            Enabled::Disabled,
        );
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
                0x00, 0x00, 0x00, 0x01, // msg_type = MSG_TYPE_CONTROL_IN
                0x00, 0x00, // port_num
                0x00, 0x00, 0x00, 0x01, // section = RECORDED (I32)
                0x00, // enabled = DISABLED
            ]
        );
    }

    #[test]
    fn end_to_end_on_the_component_thread() {
        let (comp, ground) = configured(&[true, true]);
        comp.reg_commands();
        comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);

        // Sync port on this thread; async traffic through the queue.
        recv(&comp, 10, &[4, 3, 2, 1]);
        let ping = comp.ping_in(0);
        ping.target.invoke(ping.port_num, 0xC0DE);
        let rate = comp.configure_section_group_rate_in(0);
        rate.target.invoke(
            rate.port_num,
            TelemetrySection::Realtime,
            1,
            RateLogic::OnChangeMinOrEveryMax,
            0,
            2,
        );
        let run = comp.run_in(0);
        run.target.invoke(run.port_num, 0);

        comp.active.exit();
        assert_eq!(comp.active.join(), TaskStatus::OpOk);

        assert_eq!(*ground.pings.lock().unwrap(), vec![0xC0DE]);
        let packets = ground.packets.lock().unwrap();
        assert_eq!(packets.len(), 4, "two packets x two sections");
        assert_eq!(&packets[0].1[15..19], &[4, 3, 2, 1]);
    }

    #[test]
    fn a_requested_packet_on_an_unconnected_port_keeps_its_flag_forever() {
        // Gotcha: the isConnected check precedes the REQUESTED bypass, so the
        // flag is never cleared for that section — the packet goes out on the
        // first tick after the port is wired, even though the group is
        // silenced.
        let (comp, ground) = configured(&[true, false]);
        for group in 0..NUM_CONFIGURABLE_TLMPACKETIZER_GROUPS {
            let mut args = 1i32.to_be_bytes().to_vec();
            args.extend_from_slice(&(group as u32).to_be_bytes());
            args.extend_from_slice(&RateLogic::Silenced.as_repr().to_be_bytes());
            args.extend_from_slice(&0u32.to_be_bytes());
            args.extend_from_slice(&0u32.to_be_bytes());
            send_cmd(&comp, TlmPacketizer::OPCODE_CONFIGURE_GROUP_RATES, 1, &args);
        }
        // Request packet 4 on section 1, whose port is not connected.
        let mut args = 4u32.to_be_bytes().to_vec();
        args.extend_from_slice(&1i32.to_be_bytes());
        send_cmd(&comp, TlmPacketizer::OPCODE_SEND_PKT, 2, &args);
        run_tick(&comp);
        assert!(
            ground
                .packets
                .lock()
                .unwrap()
                .iter()
                .all(|(port, _, _)| *port != 1)
        );

        comp.pkt_send[1].connect(ground.clone(), 1);
        run_tick(&comp);
        assert!(
            ground
                .packets
                .lock()
                .unwrap()
                .iter()
                .any(|(port, id_bytes, _)| *port == 1 && id_bytes[2..4] == [0x00, 0x04]),
            "the retained REQUESTED flag still bypasses the SILENCED group"
        );
    }

    #[test]
    #[should_panic]
    fn conflicting_channel_sizes_across_packets_assert() {
        const A: [TlmPacketizerChannelEntry; 1] = [TlmPacketizerChannelEntry::new(5, 4)];
        const B: [TlmPacketizerChannelEntry; 1] = [TlmPacketizerChannelEntry::new(5, 2)];
        let (comp, _ground) = build(&[true, true]);
        comp.set_packet_list(
            &[
                TlmPacketizerPacket::new(&A, 1, 0),
                TlmPacketizerPacket::new(&B, 2, 0),
            ],
            IGNORE_OMIT_LIST,
            0,
        );
    }

    #[test]
    #[should_panic]
    fn a_channel_in_a_packet_and_the_ignore_list_asserts() {
        const A: [TlmPacketizerChannelEntry; 1] = [TlmPacketizerChannelEntry::new(5, 4)];
        let (comp, _ground) = build(&[true, true]);
        comp.set_packet_list(&[TlmPacketizerPacket::new(&A, 1, 0)], &A, 0);
    }

    #[test]
    #[should_panic]
    fn a_packet_longer_than_the_com_buffer_asserts() {
        const BIG: [TlmPacketizerChannelEntry; 1] = [TlmPacketizerChannelEntry::new(5, 600)];
        let (comp, _ground) = build(&[true, true]);
        comp.set_packet_list(&[TlmPacketizerPacket::new(&BIG, 1, 0)], IGNORE_OMIT_LIST, 0);
    }

    #[test]
    #[should_panic]
    fn a_packet_level_above_the_max_group_asserts() {
        const A: [TlmPacketizerChannelEntry; 1] = [TlmPacketizerChannelEntry::new(5, 4)];
        let (comp, _ground) = build(&[true, true]);
        comp.set_packet_list(&[TlmPacketizerPacket::new(&A, 1, 4)], IGNORE_OMIT_LIST, 0);
    }

    #[test]
    fn dictionary_ids_match_the_fpp_model() {
        assert_eq!(TlmPacketizer::OPCODE_SET_LEVEL, 0);
        assert_eq!(TlmPacketizer::OPCODE_SEND_PKT, 1);
        assert_eq!(TlmPacketizer::OPCODE_ENABLE_SECTION, 2);
        assert_eq!(TlmPacketizer::OPCODE_ENABLE_GROUP, 3);
        assert_eq!(TlmPacketizer::OPCODE_FORCE_GROUP, 4);
        assert_eq!(TlmPacketizer::OPCODE_CONFIGURE_GROUP_RATES, 5);
        assert_eq!(TlmPacketizer::EVENTID_NO_CHAN, 0);
        assert_eq!(TlmPacketizer::EVENTID_LEVEL_SET, 1);
        assert_eq!(TlmPacketizer::EVENTID_MAX_LEVEL_EXCEED, 2);
        assert_eq!(TlmPacketizer::EVENTID_PACKET_SENT, 3);
        assert_eq!(TlmPacketizer::EVENTID_PACKET_NOT_FOUND, 4);
        assert_eq!(TlmPacketizer::EVENTID_SECTION_UNCONFIGURABLE, 5);
        assert_eq!(TlmPacketizer::EVENTID_OVERSIZED_CHANNEL, 6);
        assert_eq!(TlmPacketizer::CHANID_GROUP_CONFIGS, 0);
        assert_eq!(TlmPacketizer::CHANID_SECTION_ENABLED, 1);
        assert_eq!(TlmPacketizer::MSG_TYPE_CONTROL_IN, 1);
        assert_eq!(TlmPacketizer::MSG_TYPE_PING_IN, 2);
        assert_eq!(TlmPacketizer::MSG_TYPE_RUN, 3);
        assert_eq!(TlmPacketizer::MSG_TYPE_CONFIGURE_SECTION_GROUP_RATE, 4);
        assert_eq!(TlmPacketizer::MSG_TYPE_CMD_IN, 5);
        assert_eq!(
            TelemetrySection::NumSections.as_repr() as usize,
            NUM_SECTIONS
        );
        assert_eq!(PACKET_HEADER_SIZE, 15);
    }
}
