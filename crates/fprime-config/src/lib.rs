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

// The Fw* alias names are kept verbatim from the C++ tree.
#![allow(non_camel_case_types)]

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
/// Maximum length of a file-name string (`FileNameStringSize` in AcConstants.fpp).
pub const FILE_NAME_STRING_SIZE: usize = 240;
/// Time-context "don't care" value used by sequences (`FW_CONTEXT_DONT_CARE`).
pub const FW_CONTEXT_DONT_CARE: u8 = 0xFF;

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
    }
}
