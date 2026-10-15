//! F Prime project-configurable type aliases and constants.
//!
//! Rust port of the C++ `default/config/` layer (`FpConfig.fpp`,
//! `FpConstants.fpp`, `AcConstants.fpp`, `ComCfg.fpp`, and the
//! per-component `*Cfg.hpp` headers). See `docs/cpp-analysis/fw-types.md`.
//!
//! Everything here is `pub type` / `pub const` so a project can fork this
//! crate to retune the framework, exactly as C++ projects override
//! `default/config`. The type-alias names keep their exact F Prime spelling
//! for grep-compatibility with the C++ tree.

//!
//! The crate is unconditionally `#![no_std]` (and needs no `alloc`): it is
//! nothing but type aliases and constants, so it has no `std` feature to
//! turn off (ROADMAP item 13; see ARCHITECTURE.md "no_std").

// The Fw* alias names are kept verbatim from the C++ tree.
#![allow(non_camel_case_types)]
#![no_std]

// ---------------------------------------------------------------------------
// Type aliases (unix platform defaults, from PlatformTypes.fpp + FpConfig.fpp)
// ---------------------------------------------------------------------------

/// Unsigned size type (`FwSizeType`); must be unsigned with at least u32 range.
pub type FwSizeType = u64;
/// Signed size type (`FwSignedSizeType`); same width as `FwSizeType`.
pub type FwSignedSizeType = i64;
/// Port-index type (`FwIndexType`). SIGNED by design; -1 = unset.
pub type FwIndexType = i16;
/// Argument type for `fw_assert!` arguments (`FwAssertArgType`).
pub type FwAssertArgType = i32;
/// Base ID type for opcodes / channels / events / parameters (`FwIdType`).
pub type FwIdType = u32;
/// Command opcode type.
pub type FwOpcodeType = FwIdType;
/// Telemetry channel ID type.
pub type FwChanIdType = FwIdType;
/// Event ID type.
pub type FwEventIdType = FwIdType;
/// Parameter ID type.
pub type FwPrmIdType = FwIdType;
/// Data product ID type.
pub type FwDpIdType = FwIdType;
/// Data product priority type.
pub type FwDpPriorityType = u32;
/// Serialization width of plain (non-FPP) enums.
pub type FwEnumStoreType = i32;
/// On-wire length-prefix type (dictionary type; U16 in the analyzed tree).
pub type FwSizeStoreType = u16;
/// Legacy alias of [`FwSizeStoreType`] (`FwBuffSizeType` in C++).
pub type FwBuffSizeType = FwSizeStoreType;
/// Packet descriptor width (from `ComCfg.fpp`). NOTE: U16 in this codebase
/// (legacy fprime used U32).
pub type FwPacketDescriptorType = u16;
/// On-wire storage type for `TimeBase`.
pub type FwTimeBaseStoreType = u16;
/// On-wire storage type for the time context byte.
pub type FwTimeContextStoreType = u8;
/// Packetized-telemetry packet ID type.
pub type FwTlmPacketizeIdType = u16;
/// Trace ID type.
pub type FwTraceIdType = u32;
/// Queue priority type.
pub type FwQueuePriorityType = u8;
/// Task priority type.
pub type FwTaskPriorityType = u8;
/// Task ID type.
pub type FwTaskIdType = i32;

// Static invariants mirrored from Fw/FPrimeBasicTypes.hpp. Some are
// tautological under the default aliases but bind when a project retunes
// this crate, so they stay.
#[allow(clippy::absurd_extreme_comparisons)]
const _: () = {
    assert!(FwSizeType::MIN == 0, "FwSizeType must be unsigned");
    assert!(FwSizeType::MAX >= u32::MAX as FwSizeType);
    assert!(FwIndexType::MIN < 0, "FwIndexType must be signed");
    assert!(size_of::<FwSizeType>() == size_of::<FwSignedSizeType>());
    assert!(
        FwSizeType::MAX >= FwSizeStoreType::MAX as FwSizeType,
        "FwSizeType range must be a superset of FwSizeStoreType"
    );
};

// ---------------------------------------------------------------------------
// Constants (FpConstants.fpp / AcConstants.fpp defaults)
// ---------------------------------------------------------------------------

/// Maximum size of a com buffer (`FW_COM_BUFFER_MAX_SIZE`).
pub const FW_COM_BUFFER_MAX_SIZE: usize = 512;
/// Maximum size of a file-packet buffer (`FW_FILE_BUFFER_MAX_SIZE`); a file
/// packet travels in one com buffer, so it defaults to
/// [`FW_COM_BUFFER_MAX_SIZE`].
pub const FW_FILE_BUFFER_MAX_SIZE: usize = FW_COM_BUFFER_MAX_SIZE;
/// Command argument buffer: 512 - sizeof(FwOpcodeType)=4 - sizeof(FwPacketDescriptorType)=2.
pub const FW_CMD_ARG_BUFFER_MAX_SIZE: usize = FW_COM_BUFFER_MAX_SIZE - 4 - 2;
/// Event log argument buffer size.
pub const FW_LOG_BUFFER_MAX_SIZE: usize = FW_COM_BUFFER_MAX_SIZE - 4 - 2;
/// Telemetry value buffer size.
pub const FW_TLM_BUFFER_MAX_SIZE: usize = FW_COM_BUFFER_MAX_SIZE - 4 - 2;
/// Parameter value buffer size.
pub const FW_PARAM_BUFFER_MAX_SIZE: usize = FW_COM_BUFFER_MAX_SIZE - 4 - 2;
/// Maximum size of a command string argument.
pub const FW_CMD_STRING_MAX_SIZE: usize = 40;
/// Maximum size of an event log string argument.
pub const FW_LOG_STRING_MAX_SIZE: usize = 200;
/// Maximum size of a telemetry string argument.
pub const FW_TLM_STRING_MAX_SIZE: usize = 40;
/// Maximum size of a parameter string argument.
pub const FW_PARAM_STRING_MAX_SIZE: usize = 40;
/// Size of the text-log string buffer.
pub const FW_LOG_TEXT_BUFFER_SIZE: usize = 256;
/// Size of `Fw::String` (`FwDefaultString`).
pub const FW_FIXED_LENGTH_STRING_SIZE: usize = 256;
/// Size of object-name buffers.
pub const FW_OBJ_NAME_BUFFER_SIZE: usize = 80;
/// Size of queue-name buffers.
pub const FW_QUEUE_NAME_BUFFER_SIZE: usize = 80;
/// Size of task-name buffers.
pub const FW_TASK_NAME_BUFFER_SIZE: usize = 80;
/// On-wire byte value for `true` (dictionary constant).
pub const FW_SERIALIZE_TRUE_VALUE: u8 = 0xFF;
/// On-wire byte value for `false` (dictionary constant).
pub const FW_SERIALIZE_FALSE_VALUE: u8 = 0x00;
/// Maximum length of a formatted assert message.
pub const FW_ASSERT_TEXT_SIZE: usize = 256;
/// Maximum size of an internal-interface string argument
/// (`FW_INTERNAL_INTERFACE_STRING_MAX_SIZE` in FpConstants.fpp).
pub const FW_INTERNAL_INTERFACE_STRING_MAX_SIZE: usize = 256;
/// Capacity of `Fw::SmSignalBuffer`, the data buffer carried by a signal to
/// an external state machine (`FW_SM_SIGNAL_BUFFER_MAX_SIZE` in
/// FpConstants.fpp).
pub const FW_SM_SIGNAL_BUFFER_MAX_SIZE: usize = 128;
/// Maximum length of a file-name string (`FileNameStringSize` in AcConstants.fpp).
pub const FILE_NAME_STRING_SIZE: usize = 240;
/// Time-context "don't care" value used by sequences (`FW_CONTEXT_DONT_CARE`).
pub const FW_CONTEXT_DONT_CARE: u8 = 0xFF;
/// Capacity of `Fw::StatementArgBuffer`, the argument buffer of one Fpy
/// sequence statement (`FW_STATEMENT_ARG_BUFFER_MAX_SIZE` in
/// FpConstants.fpp, defined as [`FW_CMD_ARG_BUFFER_MAX_SIZE`]).
pub const FW_STATEMENT_ARG_BUFFER_MAX_SIZE: usize = FW_CMD_ARG_BUFFER_MAX_SIZE;
/// Size in bytes of the argument blob in a `Svc::SeqArgs`
/// (`SequenceArgumentsMaxSize` in AcConstants.fpp). Upstream keeps it small
/// on purpose, so a `SeqArgs` is easy to fill from fprime-gds.
pub const SEQUENCE_ARGUMENTS_MAX_SIZE: usize = 12;

// ---------------------------------------------------------------------------
// Port-count constants (AcConstants.fpp defaults)
// ---------------------------------------------------------------------------

/// Number of component command/registration ports on the CmdDispatcher
/// (`CmdDispatcherComponentCommandPorts`).
pub const CMD_DISPATCHER_COMMAND_PORTS: usize = 30;
/// Number of sequence-source ports on the CmdDispatcher (`CmdDispatcherSequencePorts`).
pub const CMD_DISPATCHER_SEQUENCE_PORTS: usize = 5;
/// Number of rate-group member output ports (`ActiveRateGroupOutputPorts` /
/// `PassiveRateGroupOutputPorts`).
pub const RATE_GROUP_MEMBER_OUT_PORTS: usize = 10;
/// Number of cycle output ports on the RateGroupDriver (`RateGroupDriverRateGroupPorts`).
pub const RATE_GROUP_DRIVER_CYCLE_PORTS: usize = 3;
/// Number of ping ports on the Health component (`HealthPingPorts`).
pub const HEALTH_PING_PORTS: usize = 25;
/// Number of `FileComplete` output ports on FileDownlink (`FileDownCompletePorts`).
pub const FILE_DOWN_COMPLETE_PORTS: usize = 1;

// ---------------------------------------------------------------------------
// Per-component configuration (default/config/*Cfg.hpp equivalents)
// ---------------------------------------------------------------------------

/// `Svc::CmdDispatcher` configuration (`CommandDispatcherImplCfg.hpp`).
pub mod cmd_dispatcher {
    /// Size of the opcode dispatch table (`CMD_DISPATCHER_DISPATCH_TABLE_SIZE`).
    pub const DISPATCH_TABLE_SIZE: usize = 150;
    /// Size of the in-progress command table (`CMD_DISPATCHER_SEQUENCER_TABLE_SIZE`).
    pub const SEQUENCER_TABLE_SIZE: usize = 25;
    /// Include command opcodes in events when true; when false, opcode fields
    /// in events are set to the maximum `FwOpcodeType` value.
    pub const INCLUDE_COMMAND_OPCODES_IN_EVENTS: bool = true;
}

/// `Svc::EventManager` configuration (`EventManagerCfg.hpp`).
///
/// Severity filter defaults: `true` = events pass through, `false` = filtered.
pub mod event_manager {
    /// WARNING_HI events pass by default.
    pub const FILTER_WARNING_HI_DEFAULT: bool = true;
    /// WARNING_LO events pass by default.
    pub const FILTER_WARNING_LO_DEFAULT: bool = true;
    /// COMMAND events pass by default.
    pub const FILTER_COMMAND_DEFAULT: bool = true;
    /// ACTIVITY_HI events pass by default.
    pub const FILTER_ACTIVITY_HI_DEFAULT: bool = true;
    /// ACTIVITY_LO events pass by default.
    pub const FILTER_ACTIVITY_LO_DEFAULT: bool = true;
    /// DIAGNOSTIC events are filtered out by default.
    pub const FILTER_DIAGNOSTIC_DEFAULT: bool = false;
    /// Size of the event ID filter table (`TELEM_ID_FILTER_SIZE`).
    pub const ID_FILTER_SIZE: usize = 25;
}

/// `Svc::ActiveTextLogger` configuration (`ActiveTextLoggerCfg.hpp`).
pub mod active_text_logger {
    /// Size of the configure-time event-ID filter
    /// (`ACTIVE_TEXT_LOGGER_ID_FILTER_SIZE`).
    pub const ID_FILTER_SIZE: usize = 25;
    /// WARNING_HI events pass by default.
    pub const FILTER_WARNING_HI_DEFAULT: bool = true;
    /// WARNING_LO events pass by default.
    pub const FILTER_WARNING_LO_DEFAULT: bool = true;
    /// COMMAND events pass by default.
    pub const FILTER_COMMAND_DEFAULT: bool = true;
    /// ACTIVITY_HI events pass by default.
    pub const FILTER_ACTIVITY_HI_DEFAULT: bool = true;
    /// ACTIVITY_LO events pass by default.
    pub const FILTER_ACTIVITY_LO_DEFAULT: bool = true;
    /// DIAGNOSTIC events are filtered out by default.
    pub const FILTER_DIAGNOSTIC_DEFAULT: bool = false;
}

/// `Svc::TlmChan` configuration (`TlmChanImplCfg.hpp`).
pub mod tlm_chan {
    /// Number of slots in the hash table (`TLMCHAN_NUM_TLM_HASH_SLOTS`).
    pub const NUM_TLM_HASH_SLOTS: usize = 15;
    /// Modulo value of the hashing function (`TLMCHAN_HASH_MOD_VALUE`).
    pub const HASH_MOD_VALUE: u32 = 99;
    /// Buckets assignable to a hash slot; must be >= number of telemetry
    /// channels in the system (`TLMCHAN_HASH_BUCKETS`).
    pub const HASH_BUCKETS: usize = 500;
    /// Maximum updated entries `run` serializes per invocation
    /// (`TLMCHAN_MAX_ENTRIES_PER_RUN`); default equals [`HASH_BUCKETS`] so
    /// the cap is a no-op.
    pub const MAX_ENTRIES_PER_RUN: usize = HASH_BUCKETS;
}

/// `Svc::ActiveRateGroup` configuration (`ActiveRateGroupCfg.hpp`).
pub mod active_rate_group {
    /// Number of overruns allowed before overrun events are throttled
    /// (`ACTIVE_RATE_GROUP_OVERRUN_THROTTLE`).
    pub const OVERRUN_THROTTLE: u32 = 5;
}

/// `Svc::BufferManager` configuration (`BufferManagerComponentImplCfg.hpp`).
pub mod buffer_manager {
    /// Maximum number of buffer bins (`BUFFERMGR_MAX_NUM_BINS`).
    pub const MAX_NUM_BINS: usize = 10;
}

/// `Svc::ComQueue` port counts (AcConstants.fpp).
pub mod com_queue {
    /// Number of com-buffer input queues (`ComQueueComPorts`).
    pub const COM_PORTS: usize = 2;
    /// Number of buffer input queues (`ComQueueBufferPorts`).
    pub const BUFFER_PORTS: usize = 1;
}

/// `Svc::FileDownlink` configuration (`FileDownlinkCfg.hpp`).
pub mod file_downlink {
    /// Size of the internal file-packet buffer
    /// (`FILEDOWNLINK_INTERNAL_BUFFER_SIZE`).
    pub const INTERNAL_BUFFER_SIZE: usize = super::FW_FILE_BUFFER_MAX_SIZE;
    /// When true, failures report success on the command/port response while
    /// still emitting their warning events
    /// (`FILEDOWNLINK_COMMAND_FAILURES_DISABLED`).
    pub const COMMAND_FAILURES_DISABLED: bool = true;
}

/// `Svc::FileManager` configuration (`FileManagerConfig.hpp` +
/// `FileManagerCfg.fpp`).
pub mod file_manager {
    use super::FwDpPriorityType;

    /// Directory entries listed per rate tick
    /// (`FileManagerConfig::FILES_PER_RATE_TICK`).
    pub const FILES_PER_RATE_TICK: u32 = 1;
    /// Maximum file bytes per `GenerateDp` chunk
    /// (`FileManagerConfig::GENERATE_DP_MAX_CHUNK_SIZE`).
    pub const GENERATE_DP_MAX_CHUNK_SIZE: u32 = 1024;
    /// `GenerateDp` chunks emitted per rate tick in paced mode
    /// (`FileManagerConfig::CHUNKS_PER_RATE_TICK`).
    pub const CHUNKS_PER_RATE_TICK: u32 = 1;
    /// Container priority used when a `GenerateDp` command passes
    /// `priority = 0` (`FileManagerCfg::DEFAULT_DP_PRIORITY`).
    pub const DEFAULT_DP_PRIORITY: FwDpPriorityType = 10;
}

/// `Svc::GenericHub` port-array sizes (`GenericHubCfg.fpp`). Both hubs of a
/// pair must agree: hub A's inputs are hub B's outputs and vice versa.
pub mod generic_hub {
    /// `GenericHubCfg.NumSerialInputPorts` — size of `serialIn`.
    pub const NUM_SERIAL_INPUT_PORTS: usize = 10;
    /// `GenericHubCfg.NumBufferInputPorts` — size of `bufferIn` /
    /// `bufferInReturn`.
    pub const NUM_BUFFER_INPUT_PORTS: usize = 10;
    /// `GenericHubCfg.NumSerialOutputPorts` — size of `serialOut`.
    pub const NUM_SERIAL_OUTPUT_PORTS: usize = 10;
    /// `GenericHubCfg.NumBufferOutputPorts` — size of `bufferOut` /
    /// `bufferOutReturn`.
    pub const NUM_BUFFER_OUTPUT_PORTS: usize = 10;
}

/// `Utils::CRCChecker` configuration (`CRCCheckerConfig.hpp`).
pub mod crc_checker {
    /// Block size for streaming a file through the CRC
    /// (`CONFIG_CRC_FILE_READ_BLOCK`).
    pub const FILE_READ_BLOCK: usize = 2048;
}

/// The `DpCompression` subtopology configuration
/// (`Svc/Subtopologies/DpCompression/DpCompressionConfig/DpCompressionConfig.fpp`):
/// the base id of `dpCompressProc` / `dpZLibCompressor` /
/// `dpZLibCompressorBufferManager` (+0x0000 / +0x1000 / +0x2000) and the two
/// bins of that buffer manager. Only meaningful with `fprime-svc`'s `zlib`
/// feature; the Rust Ref, like upstream's, does not instantiate the
/// subtopology.
pub mod dp_compression {
    use super::{FwIdType, FwSizeType};

    /// `DpCompressionConfig.BASE_ID`.
    pub const BASE_ID: FwIdType = 0x0410_0000;
    /// `ZLibBuffMgr.compressionBufferStoreSize` — scratch output buffers
    /// (at least the `DpCompressProc` `CHUNK_SIZE`).
    pub const COMPRESSION_BUFFER_STORE_SIZE: FwSizeType = 32 * 1024;
    /// `ZLibBuffMgr.compressionBufferStoreCount`.
    pub const COMPRESSION_BUFFER_STORE_COUNT: u16 = 2;
    /// `ZLibBuffMgr.allocBufferStoreSize` — zlib working memory (at least
    /// the `DpZLibCompressor` `ZLibBufferSize` parameter).
    pub const ALLOC_BUFFER_STORE_SIZE: FwSizeType = 269_000;
    /// `ZLibBuffMgr.allocBufferStoreCount`.
    pub const ALLOC_BUFFER_STORE_COUNT: u16 = 2;
    /// `ZLibBuffMgr.dpZLibCompressorBufferManagerId`.
    pub const BUFFER_MANAGER_ID: u16 = 301;
}

/// `Svc::FpySequencer` configuration (`FpySequencerCfg.fpp`, FPP module
/// `Svc.Fpy`).
pub mod fpy_sequencer {
    use super::FwIndexType;

    /// The maximum number of arguments a sequence can have
    /// (`Fpy.MAX_SEQUENCE_ARG_COUNT`; must fit a U8).
    pub const MAX_SEQUENCE_ARG_COUNT: usize = 16;
    /// The maximum number of statements a sequence can have
    /// (`Fpy.MAX_SEQUENCE_STATEMENT_COUNT`; must fit a U16).
    pub const MAX_SEQUENCE_STATEMENT_COUNT: usize = 2048;
    /// The maximum number of bytes in the sequence stack
    /// (`Fpy.MAX_STACK_SIZE`; must fit `Fpy.StackSizeType`, a U32).
    pub const MAX_STACK_SIZE: usize = 65535;
    /// The maximum number of bytes in a directive's constant byte array
    /// (`Fpy.MAX_DIRECTIVE_SIZE`: `CONST_CMD` arguments, `PUSH_VAL`).
    pub const MAX_DIRECTIVE_SIZE: usize = 2048;
    /// The default value of the `SEQ_BASE_DIR` parameter
    /// (`Fpy.DEFAULT_SEQ_BASE_DIR`).
    pub const DEFAULT_SEQ_BASE_DIR: &str = "";
    /// `Fpy.SerialPortIndex.MAX_SERIAL_PORTS`: the size of the
    /// `serialOut` port array. Projects rename the `EXAMPLE_PORT_<n>`
    /// members of `Fpy.SerialPortIndex`; this sentinel must keep its name.
    pub const MAX_SERIAL_PORTS: FwIndexType = 5;
}

/// SDLS configuration: `Svc/Ccsds/Types/config/SdlsKeyConfig/SdlsKeyCfg.fpp`
/// and `Svc/Ccsds/SdlsSaRouter/config/SdlsSaRouterConfig/SdlsSaRouterCfg.fpp`
/// (FPP module `SdlsCfg`).
pub mod sdls {
    use super::FwIndexType;

    /// Maximum size in bytes of an SDLS key held in an `SdlsKeyBuffer`
    /// (`SdlsCfg.MAX_SDLS_KEY_SIZE`).
    pub const MAX_SDLS_KEY_SIZE: usize = 32;

    /// Number of downstream crypto component ports on the `SdlsSaRouter`
    /// (`SdlsCfg.SaRouterPortCount`).
    pub const SA_ROUTER_PORT_COUNT: usize = 2;

    /// Number of entries in the SA-to-port routing map
    /// (`SdlsCfg.SaRouterMapEntryCount`).
    pub const SA_ROUTER_MAP_ENTRY_COUNT: usize = 2;

    /// Maximum number of processed data buffers outstanding — sent
    /// downstream, not yet returned (`SdlsCfg.SaRouterMaxOutstandingBuffers`).
    pub const SA_ROUTER_MAX_OUTSTANDING_BUFFERS: usize = 4;

    /// `SdlsCfg.SaRouterPorts.UNCONNECTED`: the router port standard
    /// topologies leave unconnected.
    pub const SA_ROUTER_PORT_UNCONNECTED: FwIndexType = 0;

    /// `SdlsCfg.SaRouterPorts.PLAINTEXT`: the router port the default
    /// clear-text crypto components are connected to.
    pub const SA_ROUTER_PORT_PLAINTEXT: FwIndexType = 1;

    /// The compile-time default SA-to-port map (`SdlsCfg.SaMap`), as
    /// `(securityAssociationIndex, portIndex)` pairs: SA 1 to the
    /// plain-text port, SA 0 (reserved by CCSDS 355.0-B-2 for Extended
    /// Procedures) to the unconnected port.
    pub const SA_MAP: [(u16, FwIndexType); SA_ROUTER_MAP_ENTRY_COUNT] = [
        (1, SA_ROUTER_PORT_PLAINTEXT),
        (0, SA_ROUTER_PORT_UNCONNECTED),
    ];
}

/// `Os::Generic::LocklessPriorityQueue` configuration
/// (`default/config/LocklessQueueCfg.hpp`).
pub mod lockless_queue {
    use super::FwSizeType;

    /// Unsigned integral type of the lockless queue slot state-tag word
    /// (`LocklessStateTagType`): the low two bits hold the slot state, the
    /// rest the ABA epoch tag (62 tag bits for `u64`, 30 for `u32`, 14 for
    /// `u16`, 6 for `u8`). The target must have native atomic
    /// compare-and-swap of this width; a width it lacks fails to compile
    /// (the C++ `static_assert`s on `ATOMIC_*_LOCK_FREE`).
    ///
    /// Deviation: upstream defaults to `U64` and leaves narrowing to the
    /// project on targets without lock-free 64-bit atomics; here the
    /// default narrows to `u32` automatically on such targets (thumbv7em,
    /// for example), so the default configuration builds everywhere.
    #[cfg(target_has_atomic = "64")]
    pub type LocklessStateTagType = u64;
    /// See the `target_has_atomic = "64"` definition (this is the narrowed
    /// default for targets without 64-bit atomics).
    #[cfg(not(target_has_atomic = "64"))]
    pub type LocklessStateTagType = u32;

    /// Backoff (microseconds) between bounded scans on the BLOCKING
    /// send/receive paths (`LOCKLESS_QUEUE_BLOCKING_BACKOFF_US`). Must be
    /// greater than 0: a zero backoff could livelock on a strict-priority
    /// scheduler.
    pub const LOCKLESS_QUEUE_BLOCKING_BACKOFF_US: u32 = 100;

    /// Alignment (bytes) of each queue slot
    /// (`LOCKLESS_QUEUE_SLOT_ALIGNMENT`): the target's cache-line size
    /// avoids false sharing; a smaller power of two saves memory on
    /// constrained targets. Must be a power of two from 1 to 4096.
    pub const LOCKLESS_QUEUE_SLOT_ALIGNMENT: FwSizeType = 64;

    /// Retry passes through the slot array before a NONBLOCKING operation
    /// returns FULL/EMPTY (`LOCKLESS_QUEUE_MAX_RETRY_PASSES`); bounds a
    /// non-blocking call at `depth * passes` slot visits. At least 1.
    pub const LOCKLESS_QUEUE_MAX_RETRY_PASSES: FwSizeType = 4;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derived_buffer_sizes_match_cpp_formula() {
        // 512 - sizeof(FwOpcodeType) - sizeof(FwPacketDescriptorType)
        let derived = FW_COM_BUFFER_MAX_SIZE
            - size_of::<FwOpcodeType>()
            - size_of::<FwPacketDescriptorType>();
        assert_eq!(derived, 506);
        assert_eq!(FW_CMD_ARG_BUFFER_MAX_SIZE, 506);
        assert_eq!(FW_LOG_BUFFER_MAX_SIZE, 506);
        assert_eq!(FW_TLM_BUFFER_MAX_SIZE, 506);
        assert_eq!(FW_PARAM_BUFFER_MAX_SIZE, 506);
    }

    #[test]
    fn type_alias_widths_match_defaults() {
        assert_eq!(size_of::<FwSizeType>(), 8);
        assert_eq!(size_of::<FwSignedSizeType>(), 8);
        assert_eq!(size_of::<FwIndexType>(), 2);
        assert_eq!(size_of::<FwAssertArgType>(), 4);
        assert_eq!(size_of::<FwIdType>(), 4);
        assert_eq!(size_of::<FwEnumStoreType>(), 4);
        assert_eq!(size_of::<FwSizeStoreType>(), 2);
        assert_eq!(size_of::<FwPacketDescriptorType>(), 2);
        assert_eq!(size_of::<FwTimeBaseStoreType>(), 2);
        assert_eq!(size_of::<FwTimeContextStoreType>(), 1);
    }

    #[test]
    // regression pins on configuration constants — constant by definition
    #[allow(clippy::assertions_on_constants)]
    fn per_component_config_defaults() {
        assert_eq!(cmd_dispatcher::DISPATCH_TABLE_SIZE, 150);
        assert_eq!(cmd_dispatcher::SEQUENCER_TABLE_SIZE, 25);
        assert!(cmd_dispatcher::INCLUDE_COMMAND_OPCODES_IN_EVENTS);
        assert!(event_manager::FILTER_WARNING_HI_DEFAULT);
        assert!(event_manager::FILTER_ACTIVITY_LO_DEFAULT);
        assert!(!event_manager::FILTER_DIAGNOSTIC_DEFAULT);
        assert_eq!(event_manager::ID_FILTER_SIZE, 25);
        assert_eq!(tlm_chan::NUM_TLM_HASH_SLOTS, 15);
        assert_eq!(tlm_chan::HASH_MOD_VALUE, 99);
        assert_eq!(tlm_chan::HASH_BUCKETS, 500);
        assert_eq!(tlm_chan::MAX_ENTRIES_PER_RUN, 500);
        assert_eq!(active_rate_group::OVERRUN_THROTTLE, 5);
        assert_eq!(buffer_manager::MAX_NUM_BINS, 10);
        assert_eq!(com_queue::COM_PORTS, 2);
        assert_eq!(com_queue::BUFFER_PORTS, 1);
        assert_eq!(CMD_DISPATCHER_COMMAND_PORTS, 30);
        assert_eq!(CMD_DISPATCHER_SEQUENCE_PORTS, 5);
        assert_eq!(RATE_GROUP_MEMBER_OUT_PORTS, 10);
        assert_eq!(RATE_GROUP_DRIVER_CYCLE_PORTS, 3);
        assert_eq!(HEALTH_PING_PORTS, 25);
        assert_eq!(FILE_DOWN_COMPLETE_PORTS, 1);
        assert_eq!(FW_FILE_BUFFER_MAX_SIZE, 512);
        assert_eq!(file_downlink::INTERNAL_BUFFER_SIZE, 512);
        assert!(file_downlink::COMMAND_FAILURES_DISABLED);
        assert_eq!(file_manager::FILES_PER_RATE_TICK, 1);
        assert_eq!(file_manager::GENERATE_DP_MAX_CHUNK_SIZE, 1024);
        assert_eq!(file_manager::CHUNKS_PER_RATE_TICK, 1);
        assert_eq!(file_manager::DEFAULT_DP_PRIORITY, 10);
        assert_eq!(crc_checker::FILE_READ_BLOCK, 2048);
        assert_eq!(sdls::MAX_SDLS_KEY_SIZE, 32);
        assert_eq!(sdls::SA_ROUTER_PORT_COUNT, 2);
        assert_eq!(sdls::SA_ROUTER_MAP_ENTRY_COUNT, 2);
        assert_eq!(sdls::SA_ROUTER_MAX_OUTSTANDING_BUFFERS, 4);
        assert_eq!(sdls::SA_MAP, [(1, 1), (0, 0)]);
    }

    #[test]
    fn lockless_queue_defaults_match_cpp() {
        assert_eq!(lockless_queue::LOCKLESS_QUEUE_BLOCKING_BACKOFF_US, 100);
        assert_eq!(lockless_queue::LOCKLESS_QUEUE_SLOT_ALIGNMENT, 64);
        assert_eq!(lockless_queue::LOCKLESS_QUEUE_MAX_RETRY_PASSES, 4);
        // Upstream default U64 on hosts with 64-bit atomics.
        #[cfg(target_has_atomic = "64")]
        assert_eq!(size_of::<lockless_queue::LocklessStateTagType>(), 8);
    }
}
