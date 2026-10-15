//! The upstream pin and the static id comparison (ROADMAP item 17).
//!
//! The Ref's ground dictionary (`dictionary/RefTopologyDictionary.json`) is
//! generated from the upstream FPP model at the commit named in
//! `FPRIME_UPSTREAM_COMMIT`; the Rust components carry their command, event,
//! channel, parameter, record and container ids as hand-written
//! `OPCODE_*`/`EVENTID_*`/`CHANID_*`/`PARAMID_*`/... constants. These tests
//! fail when the two drift apart:
//!
//! - `upstream_pin`: the dictionary records the pinned commit;
//! - `rust_ids_match_the_dictionary`: every dictionary entry of every
//!   instance has a Rust constant with the same id (instance base id from
//!   `topology.rs` plus the component-relative constant), and every Rust
//!   constant listed here names a dictionary entry. A dictionary regenerated
//!   from a newer upstream that adds, removes or renumbers anything fails
//!   here until the Rust side (and this table) catches up.
//!
//! The table below lists each instance's constants by their FPP names.

use std::collections::{BTreeMap, BTreeSet};

use fprime_fpp::json::Json;
use fprime_ref::topology::*;

/// The dictionary sections this test compares, with each entry's id key.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Kind {
    Command,
    Event,
    Channel,
    Parameter,
    Record,
    Container,
}

impl Kind {
    const ALL: [Kind; 6] = [
        Kind::Command,
        Kind::Event,
        Kind::Channel,
        Kind::Parameter,
        Kind::Record,
        Kind::Container,
    ];

    fn section(self) -> &'static str {
        match self {
            Kind::Command => "commands",
            Kind::Event => "events",
            Kind::Channel => "telemetryChannels",
            Kind::Parameter => "parameters",
            Kind::Record => "records",
            Kind::Container => "containers",
        }
    }

    fn id_key(self) -> &'static str {
        match self {
            Kind::Command => "opcode",
            _ => "id",
        }
    }
}

/// One instance of the Rust Ref: its FPP name, base id, and the
/// (kind, FPP name, component-relative id) of every Rust constant.
struct Instance {
    name: &'static str,
    base: u32,
    entries: Vec<(Kind, &'static str, u64)>,
}

fn instance(name: &'static str, base: u32, entries: Vec<(Kind, &'static str, u64)>) -> Instance {
    Instance {
        name,
        base,
        entries,
    }
}

macro_rules! entries {
    ($($kind:ident $name:literal = $value:expr),* $(,)?) => {
        vec![$((Kind::$kind, $name, u64::from($value))),*]
    };
}

/// Every Ref instance that has dictionary entries. Instances without any
/// (`textLogger`, `fatalHandler`, `comStub`, `tmFramer`, `posixTime`,
/// `rateGroupDriverComp`, `linuxTimer`, `comDriver`, `comSplitter`) need
/// no row; a new instance with entries fails `rust_ids_match_the_dictionary`
/// until it gets one.
fn rust_instances() -> Vec<Instance> {
    vec![
        instance(
            "cmdDisp",
            CMD_DISPATCHER_BASE_ID,
            entries![
                Command "CMD_NO_OP" = fprime_svc::cmd_dispatcher::CmdDispatcher::OPCODE_CMD_NO_OP,
                Command "CMD_NO_OP_STRING" = fprime_svc::cmd_dispatcher::CmdDispatcher::OPCODE_CMD_NO_OP_STRING,
                Command "CMD_TEST_CMD_1" = fprime_svc::cmd_dispatcher::CmdDispatcher::OPCODE_CMD_TEST_CMD_1,
                Command "CMD_CLEAR_TRACKING" = fprime_svc::cmd_dispatcher::CmdDispatcher::OPCODE_CMD_CLEAR_TRACKING,
                Event "OpCodeRegistered" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_OP_CODE_REGISTERED,
                Event "OpCodeDispatched" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_OP_CODE_DISPATCHED,
                Event "OpCodeCompleted" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_OP_CODE_COMPLETED,
                Event "OpCodeError" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_OP_CODE_ERROR,
                Event "MalformedCommand" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_MALFORMED_COMMAND,
                Event "InvalidCommand" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_INVALID_COMMAND,
                Event "TooManyCommands" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_TOO_MANY_COMMANDS,
                Event "NoOpReceived" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_NO_OP_RECEIVED,
                Event "NoOpStringReceived" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_NO_OP_STRING_RECEIVED,
                Event "TestCmd1Args" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_TEST_CMD_1_ARGS,
                Event "OpCodeReregistered" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_OP_CODE_REREGISTERED,
                Event "CommandDroppedQueueOverflow" = fprime_svc::cmd_dispatcher::CmdDispatcher::EVENTID_COMMAND_DROPPED_QUEUE_OVERFLOW,
                Channel "CommandsDispatched" = fprime_svc::cmd_dispatcher::CmdDispatcher::CHANID_COMMANDS_DISPATCHED,
                Channel "CommandErrors" = fprime_svc::cmd_dispatcher::CmdDispatcher::CHANID_COMMAND_ERRORS,
                Channel "CommandsDropped" = fprime_svc::cmd_dispatcher::CmdDispatcher::CHANID_COMMANDS_DROPPED,
            ],
        ),
        instance(
            "events",
            EVENT_MANAGER_BASE_ID,
            entries![
                Command "SET_EVENT_FILTER" = fprime_svc::event_manager::EventManager::OPCODE_SET_EVENT_FILTER,
                Command "SET_ID_FILTER" = fprime_svc::event_manager::EventManager::OPCODE_SET_ID_FILTER,
                Command "DUMP_FILTER_STATE" = fprime_svc::event_manager::EventManager::OPCODE_DUMP_FILTER_STATE,
                Event "SEVERITY_FILTER_STATE" = fprime_svc::event_manager::EventManager::EVENTID_SEVERITY_FILTER_STATE,
                Event "ID_FILTER_ENABLED" = fprime_svc::event_manager::EventManager::EVENTID_ID_FILTER_ENABLED,
                Event "ID_FILTER_LIST_FULL" = fprime_svc::event_manager::EventManager::EVENTID_ID_FILTER_LIST_FULL,
                Event "ID_FILTER_REMOVED" = fprime_svc::event_manager::EventManager::EVENTID_ID_FILTER_REMOVED,
                Event "ID_FILTER_NOT_FOUND" = fprime_svc::event_manager::EventManager::EVENTID_ID_FILTER_NOT_FOUND,
                Channel "EventsDropped" = fprime_svc::event_manager::EventManager::CHANID_EVENTS_DROPPED,
            ],
        ),
        instance(
            "health",
            HEALTH_BASE_ID,
            entries![
                Event "HLTH_PING_WARN" = fprime_svc::health::Health::EVENTID_HLTH_PING_WARN,
                Event "HLTH_PING_LATE" = fprime_svc::health::Health::EVENTID_HLTH_PING_LATE,
                Event "HLTH_PING_WRONG_KEY" = fprime_svc::health::Health::EVENTID_HLTH_PING_WRONG_KEY,
                Event "HLTH_CHECK_ENABLE" = fprime_svc::health::Health::EVENTID_HLTH_CHECK_ENABLE,
                Event "HLTH_CHECK_PING" = fprime_svc::health::Health::EVENTID_HLTH_CHECK_PING,
                Event "HLTH_CHECK_LOOKUP_ERROR" = fprime_svc::health::Health::EVENTID_HLTH_CHECK_LOOKUP_ERROR,
                Event "HLTH_PING_UPDATED" = fprime_svc::health::Health::EVENTID_HLTH_PING_UPDATED,
                Event "HLTH_PING_INVALID_VALUES" = fprime_svc::health::Health::EVENTID_HLTH_PING_INVALID_VALUES,
                Command "HLTH_ENABLE" = fprime_svc::health::Health::OPCODE_HLTH_ENABLE,
                Command "HLTH_PING_ENABLE" = fprime_svc::health::Health::OPCODE_HLTH_PING_ENABLE,
                Command "HLTH_CHNG_PING" = fprime_svc::health::Health::OPCODE_HLTH_CHNG_PING,
                Channel "PingLateWarnings" = fprime_svc::health::Health::CHANID_PING_LATE_WARNINGS,
            ],
        ),
        instance(
            "tlmSend",
            TLM_CHAN_BASE_ID,
            entries![
                Event "TlmChanEpochProcessingCapReached" = fprime_svc::tlm_chan::TlmChan::EVENTID_TLM_CHAN_EPOCH_PROCESSING_CAP_REACHED,
                Event "TlmChanBucketPoolExhausted" = fprime_svc::tlm_chan::TlmChan::EVENTID_TLM_CHAN_BUCKET_POOL_EXHAUSTED,
            ],
        ),
        instance(
            "fatalAdapter",
            FATAL_ADAPTER_BASE_ID,
            entries![
                Event "AF_ASSERT_0" = fprime_svc::assert_fatal_adapter::AssertFatalAdapter::EVENTID_AF_ASSERT_0,
                Event "AF_ASSERT_1" = fprime_svc::assert_fatal_adapter::AssertFatalAdapter::EVENTID_AF_ASSERT_1,
                Event "AF_ASSERT_2" = fprime_svc::assert_fatal_adapter::AssertFatalAdapter::EVENTID_AF_ASSERT_2,
                Event "AF_ASSERT_3" = fprime_svc::assert_fatal_adapter::AssertFatalAdapter::EVENTID_AF_ASSERT_3,
                Event "AF_ASSERT_4" = fprime_svc::assert_fatal_adapter::AssertFatalAdapter::EVENTID_AF_ASSERT_4,
                Event "AF_ASSERT_5" = fprime_svc::assert_fatal_adapter::AssertFatalAdapter::EVENTID_AF_ASSERT_5,
                Event "AF_ASSERT_6" = fprime_svc::assert_fatal_adapter::AssertFatalAdapter::EVENTID_AF_ASSERT_6,
                Event "AF_UNEXPECTED_ASSERT" = fprime_svc::assert_fatal_adapter::AssertFatalAdapter::EVENTID_AF_UNEXPECTED_ASSERT,
            ],
        ),
        instance(
            "version",
            VERSION_BASE_ID,
            entries![
                Command "ENABLE" = fprime_svc::version::Version::OPCODE_ENABLE,
                Command "VERSION" = fprime_svc::version::Version::OPCODE_VERSION,
                Event "FrameworkVersion" = fprime_svc::version::Version::EVENTID_FRAMEWORK_VERSION,
                Event "ProjectVersion" = fprime_svc::version::Version::EVENTID_PROJECT_VERSION,
                Event "LibraryVersions" = fprime_svc::version::Version::EVENTID_LIBRARY_VERSIONS,
                Event "CustomVersions" = fprime_svc::version::Version::EVENTID_CUSTOM_VERSIONS,
                Channel "FrameworkVersion" = fprime_svc::version::Version::CHANID_FRAMEWORK_VERSION,
                Channel "ProjectVersion" = fprime_svc::version::Version::CHANID_PROJECT_VERSION,
                Channel "CustomVersion01" = fprime_svc::version::Version::CHANID_CUSTOM_VERSION_01,
                Channel "CustomVersion02" = fprime_svc::version::Version::CHANID_CUSTOM_VERSION_02,
                Channel "CustomVersion03" = fprime_svc::version::Version::CHANID_CUSTOM_VERSION_03,
                Channel "CustomVersion04" = fprime_svc::version::Version::CHANID_CUSTOM_VERSION_04,
                Channel "CustomVersion05" = fprime_svc::version::Version::CHANID_CUSTOM_VERSION_05,
                Channel "CustomVersion06" = fprime_svc::version::Version::CHANID_CUSTOM_VERSION_06,
                Channel "CustomVersion07" = fprime_svc::version::Version::CHANID_CUSTOM_VERSION_07,
                Channel "CustomVersion08" = fprime_svc::version::Version::CHANID_CUSTOM_VERSION_08,
                Channel "CustomVersion09" = fprime_svc::version::Version::CHANID_CUSTOM_VERSION_09,
                Channel "CustomVersion10" = fprime_svc::version::Version::CHANID_CUSTOM_VERSION_10,
                Channel "LibraryVersion01" = fprime_svc::version::Version::CHANID_LIBRARY_VERSION_01,
                Channel "LibraryVersion02" = fprime_svc::version::Version::CHANID_LIBRARY_VERSION_02,
                Channel "LibraryVersion03" = fprime_svc::version::Version::CHANID_LIBRARY_VERSION_03,
                Channel "LibraryVersion04" = fprime_svc::version::Version::CHANID_LIBRARY_VERSION_04,
                Channel "LibraryVersion05" = fprime_svc::version::Version::CHANID_LIBRARY_VERSION_05,
                Channel "LibraryVersion06" = fprime_svc::version::Version::CHANID_LIBRARY_VERSION_06,
                Channel "LibraryVersion07" = fprime_svc::version::Version::CHANID_LIBRARY_VERSION_07,
                Channel "LibraryVersion08" = fprime_svc::version::Version::CHANID_LIBRARY_VERSION_08,
                Channel "LibraryVersion09" = fprime_svc::version::Version::CHANID_LIBRARY_VERSION_09,
                Channel "LibraryVersion10" = fprime_svc::version::Version::CHANID_LIBRARY_VERSION_10,
            ],
        ),
        instance(
            "comQueue",
            COM_QUEUE_BASE_ID,
            entries![
                Command "FLUSH_QUEUE" = fprime_svc::com_queue::OPCODE_FLUSH_QUEUE,
                Command "FLUSH_ALL_QUEUES" = fprime_svc::com_queue::OPCODE_FLUSH_ALL_QUEUES,
                Command "SET_QUEUE_PRIORITY" = fprime_svc::com_queue::OPCODE_SET_QUEUE_PRIORITY,
                Event "QueueOverflow" = fprime_svc::com_queue::EVENTID_QUEUE_OVERFLOW,
                Event "QueuePriorityChanged" = fprime_svc::com_queue::EVENTID_QUEUE_PRIORITY_CHANGED,
                Channel "comQueueDepth" = fprime_svc::com_queue::CHANID_COM_QUEUE_DEPTH,
                Channel "buffQueueDepth" = fprime_svc::com_queue::CHANID_BUFF_QUEUE_DEPTH,
            ],
        ),
        instance(
            "frameAccumulator",
            FRAME_ACCUMULATOR_BASE_ID,
            entries![
                Event "NoBufferAvailable" = fprime_svc::frame_accumulator::EVENTID_NO_BUFFER_AVAILABLE,
                Event "FrameDetectionSizeError" = fprime_svc::frame_accumulator::EVENTID_FRAME_DETECTION_SIZE_ERROR,
                Event "FrameDetectionValidFrameDropped" = fprime_svc::frame_accumulator::EVENTID_FRAME_DETECTION_VALID_FRAME_DROPPED,
            ],
        ),
        instance(
            "bufferManager",
            BUFFER_MANAGER_BASE_ID,
            entries![
                Event "NoBuffsAvailable" = fprime_svc::buffer_manager::BufferManager::EVENTID_NO_BUFFS_AVAILABLE,
                Event "NullEmptyBuffer" = fprime_svc::buffer_manager::BufferManager::EVENTID_NULL_EMPTY_BUFFER,
                Channel "TotalBuffs" = fprime_svc::buffer_manager::BufferManager::CHANID_TOTAL_BUFFS,
                Channel "CurrBuffs" = fprime_svc::buffer_manager::BufferManager::CHANID_CURR_BUFFS,
                Channel "HiBuffs" = fprime_svc::buffer_manager::BufferManager::CHANID_HI_BUFFS,
                Channel "NoBuffs" = fprime_svc::buffer_manager::BufferManager::CHANID_NO_BUFFS,
                Channel "EmptyBuffs" = fprime_svc::buffer_manager::BufferManager::CHANID_EMPTY_BUFFS,
            ],
        ),
        instance(
            "deframer",
            DEFRAMER_BASE_ID,
            entries![
                Event "InvalidBufferReceived" = fprime_svc::fprime_deframer::EVENTID_INVALID_BUFFER_RECEIVED,
                Event "InvalidStartWord" = fprime_svc::fprime_deframer::EVENTID_INVALID_START_WORD,
                Event "InvalidLengthReceived" = fprime_svc::fprime_deframer::EVENTID_INVALID_LENGTH_RECEIVED,
                Event "InvalidChecksum" = fprime_svc::fprime_deframer::EVENTID_INVALID_CHECKSUM,
                Event "PayloadTooShort" = fprime_svc::fprime_deframer::EVENTID_PAYLOAD_TOO_SHORT,
            ],
        ),
        instance(
            "framer",
            FRAMER_BASE_ID,
            entries![
                Event "NoBufferAvailable" = fprime_svc::fprime_framer::EVENTID_NO_BUFFER_AVAILABLE,
            ],
        ),
        instance(
            "fprimeRouter",
            ROUTER_BASE_ID,
            entries![
                Event "SerializationError" = fprime_svc::fprime_router::EVENTID_SERIALIZATION_ERROR,
                Event "DeserializationError" = fprime_svc::fprime_router::EVENTID_DESERIALIZATION_ERROR,
                Event "FileOutContextTableFull" = fprime_svc::fprime_router::EVENTID_FILE_OUT_CONTEXT_TABLE_FULL,
                Event "UnknownDataOutContextTableFull" = fprime_svc::fprime_router::EVENTID_UNKNOWN_DATA_OUT_CONTEXT_TABLE_FULL,
                Event "BufferContextNotFound" = fprime_svc::fprime_router::EVENTID_BUFFER_CONTEXT_NOT_FOUND,
            ],
        ),
        instance(
            "spacePacketFramer",
            SPACE_PACKET_FRAMER_BASE_ID,
            entries![
                Event "NoBufferAvailable" = fprime_svc::ccsds::space_packet_framer::EVENTID_NO_BUFFER_AVAILABLE,
            ],
        ),
        // ComAggregator declares no commands, events, telemetry or
        // parameters: the row pins its instance id and asserts that the
        // dictionary carries no entries for it.
        instance("aggregator", AGGREGATOR_BASE_ID, entries![]),
        instance(
            "apidManager",
            APID_MANAGER_BASE_ID,
            entries![
                Event "UnexpectedSequenceCount" = fprime_svc::ccsds::apid_manager::ApidManager::EVENTID_UNEXPECTED_SEQUENCE_COUNT,
            ],
        ),
        instance(
            "tcDeframer",
            TC_DEFRAMER_BASE_ID,
            entries![
                Event "InvalidPacket" = fprime_svc::ccsds::tc_deframer::EVENTID_INVALID_PACKET,
                Event "InvalidSpacecraftId" = fprime_svc::ccsds::tc_deframer::EVENTID_INVALID_SPACECRAFT_ID,
                Event "InvalidFrameLength" = fprime_svc::ccsds::tc_deframer::EVENTID_INVALID_FRAME_LENGTH,
                Event "InvalidVcId" = fprime_svc::ccsds::tc_deframer::EVENTID_INVALID_VC_ID,
                Event "InvalidCrc" = fprime_svc::ccsds::tc_deframer::EVENTID_INVALID_CRC,
            ],
        ),
        instance(
            "spacePacketDeframer",
            SPACE_PACKET_DEFRAMER_BASE_ID,
            entries![
                Event "InvalidPacket" = fprime_svc::ccsds::space_packet_deframer::EVENTID_INVALID_PACKET,
                Event "InvalidLength" = fprime_svc::ccsds::space_packet_deframer::EVENTID_INVALID_LENGTH,
            ],
        ),
        instance(
            "dpCat",
            DP_CATALOG_BASE_ID,
            entries![
                Command "BUILD_CATALOG" = fprime_svc::dp_catalog::DpCatalog::OPCODE_BUILD_CATALOG,
                Command "START_XMIT_CATALOG" = fprime_svc::dp_catalog::DpCatalog::OPCODE_START_XMIT_CATALOG,
                Command "STOP_XMIT_CATALOG" = fprime_svc::dp_catalog::DpCatalog::OPCODE_STOP_XMIT_CATALOG,
                Command "CLEAR_CATALOG" = fprime_svc::dp_catalog::DpCatalog::OPCODE_CLEAR_CATALOG,
                Event "DirectoryOpenError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_DIRECTORY_OPEN_ERROR,
                Event "ProcessingDirectory" = fprime_svc::dp_catalog::DpCatalog::EVENTID_PROCESSING_DIRECTORY,
                Event "ProcessingFile" = fprime_svc::dp_catalog::DpCatalog::EVENTID_PROCESSING_FILE,
                Event "ProcessingDirectoryComplete" = fprime_svc::dp_catalog::DpCatalog::EVENTID_PROCESSING_DIRECTORY_COMPLETE,
                Event "CatalogBuildComplete" = fprime_svc::dp_catalog::DpCatalog::EVENTID_CATALOG_BUILD_COMPLETE,
                Event "DirectoryNotManaged" = fprime_svc::dp_catalog::DpCatalog::EVENTID_DIRECTORY_NOT_MANAGED,
                Event "CatalogXmitStarted" = fprime_svc::dp_catalog::DpCatalog::EVENTID_CATALOG_XMIT_STARTED,
                Event "CatalogXmitStopped" = fprime_svc::dp_catalog::DpCatalog::EVENTID_CATALOG_XMIT_STOPPED,
                Event "CatalogXmitCompleted" = fprime_svc::dp_catalog::DpCatalog::EVENTID_CATALOG_XMIT_COMPLETED,
                Event "SendingProduct" = fprime_svc::dp_catalog::DpCatalog::EVENTID_SENDING_PRODUCT,
                Event "ProductComplete" = fprime_svc::dp_catalog::DpCatalog::EVENTID_PRODUCT_COMPLETE,
                Event "ComponentNotInitialized" = fprime_svc::dp_catalog::DpCatalog::EVENTID_COMPONENT_NOT_INITIALIZED,
                Event "ComponentNoMemory" = fprime_svc::dp_catalog::DpCatalog::EVENTID_COMPONENT_NO_MEMORY,
                Event "CatalogFull" = fprime_svc::dp_catalog::DpCatalog::EVENTID_CATALOG_FULL,
                Event "FileOpenError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_FILE_OPEN_ERROR,
                Event "FileReadError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_FILE_READ_ERROR,
                Event "FileHdrError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_FILE_HDR_ERROR,
                Event "FileHdrDesError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_FILE_HDR_DES_ERROR,
                Event "DpInsertError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_DP_INSERT_ERROR,
                Event "DpDuplicate" = fprime_svc::dp_catalog::DpCatalog::EVENTID_DP_DUPLICATE,
                Event "DpCatalogFull" = fprime_svc::dp_catalog::DpCatalog::EVENTID_DP_CATALOG_FULL,
                Event "DpXmitInProgress" = fprime_svc::dp_catalog::DpCatalog::EVENTID_DP_XMIT_IN_PROGRESS,
                Event "FileSizeError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_FILE_SIZE_ERROR,
                Event "NoDpMemory" = fprime_svc::dp_catalog::DpCatalog::EVENTID_NO_DP_MEMORY,
                Event "XmitNotActive" = fprime_svc::dp_catalog::DpCatalog::EVENTID_XMIT_NOT_ACTIVE,
                Event "StateFileOpenError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_STATE_FILE_OPEN_ERROR,
                Event "StateFileReadError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_STATE_FILE_READ_ERROR,
                Event "StateFileTruncated" = fprime_svc::dp_catalog::DpCatalog::EVENTID_STATE_FILE_TRUNCATED,
                Event "NoStateFileSpecified" = fprime_svc::dp_catalog::DpCatalog::EVENTID_NO_STATE_FILE_SPECIFIED,
                Event "StateFileWriteError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_STATE_FILE_WRITE_ERROR,
                Event "NoStateFile" = fprime_svc::dp_catalog::DpCatalog::EVENTID_NO_STATE_FILE,
                Event "DpFileXmitError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_DP_FILE_XMIT_ERROR,
                Event "DpFileSendError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_DP_FILE_SEND_ERROR,
                Event "DpFileAdded" = fprime_svc::dp_catalog::DpCatalog::EVENTID_DP_FILE_ADDED,
                Event "NotLoaded" = fprime_svc::dp_catalog::DpCatalog::EVENTID_NOT_LOADED,
                Event "DpFileSkipped" = fprime_svc::dp_catalog::DpCatalog::EVENTID_DP_FILE_SKIPPED,
                Event "XmitUnbuiltCatalog" = fprime_svc::dp_catalog::DpCatalog::EVENTID_XMIT_UNBUILT_CATALOG,
                Event "InvalidFileName" = fprime_svc::dp_catalog::DpCatalog::EVENTID_INVALID_FILE_NAME,
                Event "FileCorruptedDataError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_FILE_CORRUPTED_DATA_ERROR,
                Event "FileNameFormatError" = fprime_svc::dp_catalog::DpCatalog::EVENTID_FILE_NAME_FORMAT_ERROR,
                Channel "CatalogDps" = fprime_svc::dp_catalog::DpCatalog::CHANID_CATALOG_DPS,
                Channel "DpsSent" = fprime_svc::dp_catalog::DpCatalog::CHANID_DPS_SENT,
            ],
        ),
        instance(
            "dpMgr",
            DP_MANAGER_BASE_ID,
            entries![
                Command "CLEAR_EVENT_THROTTLE" = fprime_svc::dp_manager::DpManager::OPCODE_CLEAR_EVENT_THROTTLE,
                Event "BufferAllocationFailed" = fprime_svc::dp_manager::DpManager::EVENTID_BUFFER_ALLOCATION_FAILED,
                Channel "NumSuccessfulAllocations" = fprime_svc::dp_manager::DpManager::CHANID_NUM_SUCCESSFUL_ALLOCATIONS,
                Channel "NumFailedAllocations" = fprime_svc::dp_manager::DpManager::CHANID_NUM_FAILED_ALLOCATIONS,
                Channel "NumDataProducts" = fprime_svc::dp_manager::DpManager::CHANID_NUM_DATA_PRODUCTS,
                Channel "NumBytes" = fprime_svc::dp_manager::DpManager::CHANID_NUM_BYTES,
            ],
        ),
        instance(
            "dpWriter",
            DP_WRITER_BASE_ID,
            entries![
                Command "CLEAR_EVENT_THROTTLE" = fprime_svc::dp_writer::DpWriter::OPCODE_CLEAR_EVENT_THROTTLE,
                Event "InvalidBuffer" = fprime_svc::dp_writer::DpWriter::EVENTID_INVALID_BUFFER,
                Event "BufferTooSmallForPacket" = fprime_svc::dp_writer::DpWriter::EVENTID_BUFFER_TOO_SMALL_FOR_PACKET,
                Event "InvalidHeaderHash" = fprime_svc::dp_writer::DpWriter::EVENTID_INVALID_HEADER_HASH,
                Event "InvalidHeader" = fprime_svc::dp_writer::DpWriter::EVENTID_INVALID_HEADER,
                Event "BufferTooSmallForData" = fprime_svc::dp_writer::DpWriter::EVENTID_BUFFER_TOO_SMALL_FOR_DATA,
                Event "FileNameFormatError" = fprime_svc::dp_writer::DpWriter::EVENTID_FILE_NAME_FORMAT_ERROR,
                Event "FileOpenError" = fprime_svc::dp_writer::DpWriter::EVENTID_FILE_OPEN_ERROR,
                Event "FileWriteError" = fprime_svc::dp_writer::DpWriter::EVENTID_FILE_WRITE_ERROR,
                Event "FileWritten" = fprime_svc::dp_writer::DpWriter::EVENTID_FILE_WRITTEN,
                Channel "NumBuffersReceived" = fprime_svc::dp_writer::DpWriter::CHANID_NUM_BUFFERS_RECEIVED,
                Channel "NumBytesWritten" = fprime_svc::dp_writer::DpWriter::CHANID_NUM_BYTES_WRITTEN,
                Channel "NumSuccessfulWrites" = fprime_svc::dp_writer::DpWriter::CHANID_NUM_SUCCESSFUL_WRITES,
                Channel "NumFailedWrites" = fprime_svc::dp_writer::DpWriter::CHANID_NUM_FAILED_WRITES,
                Channel "NumErrors" = fprime_svc::dp_writer::DpWriter::CHANID_NUM_ERRORS,
            ],
        ),
        instance(
            "dpBufferManager",
            DP_BUFFER_MANAGER_BASE_ID,
            entries![
                Event "NoBuffsAvailable" = fprime_svc::buffer_manager::BufferManager::EVENTID_NO_BUFFS_AVAILABLE,
                Event "NullEmptyBuffer" = fprime_svc::buffer_manager::BufferManager::EVENTID_NULL_EMPTY_BUFFER,
                Channel "TotalBuffs" = fprime_svc::buffer_manager::BufferManager::CHANID_TOTAL_BUFFS,
                Channel "CurrBuffs" = fprime_svc::buffer_manager::BufferManager::CHANID_CURR_BUFFS,
                Channel "HiBuffs" = fprime_svc::buffer_manager::BufferManager::CHANID_HI_BUFFS,
                Channel "NoBuffs" = fprime_svc::buffer_manager::BufferManager::CHANID_NO_BUFFS,
                Channel "EmptyBuffs" = fprime_svc::buffer_manager::BufferManager::CHANID_EMPTY_BUFFS,
            ],
        ),
        instance(
            "dpBufferAccumulator",
            DP_BUFFER_ACCUMULATOR_BASE_ID,
            entries![
                Command "BA_SetMode" = fprime_svc::buffer_accumulator::BufferAccumulator::OPCODE_BA_SETMODE,
                Command "BA_DrainBuffers" = fprime_svc::buffer_accumulator::BufferAccumulator::OPCODE_BA_DRAINBUFFERS,
                Event "BA_BufferAccepted" = fprime_svc::buffer_accumulator::BufferAccumulator::EVENTID_BA_BUFFERACCEPTED,
                Event "BA_QueueFull" = fprime_svc::buffer_accumulator::BufferAccumulator::EVENTID_BA_QUEUEFULL,
                Event "BA_StillDraining" = fprime_svc::buffer_accumulator::BufferAccumulator::EVENTID_BA_STILLDRAINING,
                Event "BA_AlreadyDraining" = fprime_svc::buffer_accumulator::BufferAccumulator::EVENTID_BA_ALREADYDRAINING,
                Event "BA_DrainStalled" = fprime_svc::buffer_accumulator::BufferAccumulator::EVENTID_BA_DRAINSTALLED,
                Event "BA_PartialDrainDone" = fprime_svc::buffer_accumulator::BufferAccumulator::EVENTID_BA_PARTIALDRAINDONE,
                Event "BA_NonBlockDrain" = fprime_svc::buffer_accumulator::BufferAccumulator::EVENTID_BA_NONBLOCKDRAIN,
                Channel "BA_NumQueuedBuffers" = fprime_svc::buffer_accumulator::BufferAccumulator::CHANID_BA_NUMQUEUEDBUFFERS,
            ],
        ),
        instance(
            "fileUplink",
            FILE_UPLINK_BASE_ID,
            entries![
                Event "BadChecksum" = fprime_svc::file_uplink::FileUplink::EVENTID_BAD_CHECKSUM,
                Event "FileOpenError" = fprime_svc::file_uplink::FileUplink::EVENTID_FILE_OPEN_ERROR,
                Event "FileReceived" = fprime_svc::file_uplink::FileUplink::EVENTID_FILE_RECEIVED,
                Event "FileWriteError" = fprime_svc::file_uplink::FileUplink::EVENTID_FILE_WRITE_ERROR,
                Event "InvalidReceiveMode" = fprime_svc::file_uplink::FileUplink::EVENTID_INVALID_RECEIVE_MODE,
                Event "PacketOutOfBounds" = fprime_svc::file_uplink::FileUplink::EVENTID_PACKET_OUT_OF_BOUNDS,
                Event "PacketOutOfOrder" = fprime_svc::file_uplink::FileUplink::EVENTID_PACKET_OUT_OF_ORDER,
                Event "PacketDuplicate" = fprime_svc::file_uplink::FileUplink::EVENTID_PACKET_DUPLICATE,
                Event "UplinkCanceled" = fprime_svc::file_uplink::FileUplink::EVENTID_UPLINK_CANCELED,
                Event "DecodeError" = fprime_svc::file_uplink::FileUplink::EVENTID_DECODE_ERROR,
                Event "InvalidPacketReceived" = fprime_svc::file_uplink::FileUplink::EVENTID_INVALID_PACKET_RECEIVED,
                Channel "FilesReceived" = fprime_svc::file_uplink::FileUplink::CHANID_FILES_RECEIVED,
                Channel "PacketsReceived" = fprime_svc::file_uplink::FileUplink::CHANID_PACKETS_RECEIVED,
                Channel "Warnings" = fprime_svc::file_uplink::FileUplink::CHANID_WARNINGS,
                Channel "FilesReceivedFailed" = fprime_svc::file_uplink::FileUplink::CHANID_FILES_RECEIVED_FAILED,
            ],
        ),
        instance(
            "fileDownlink",
            FILE_DOWNLINK_BASE_ID,
            entries![
                Command "SendFile" = fprime_svc::file_downlink::FileDownlink::OPCODE_SEND_FILE,
                Command "Cancel" = fprime_svc::file_downlink::FileDownlink::OPCODE_CANCEL,
                Command "SendPartial" = fprime_svc::file_downlink::FileDownlink::OPCODE_SEND_PARTIAL,
                Event "FileOpenError" = fprime_svc::file_downlink::FileDownlink::EVENTID_FILE_OPEN_ERROR,
                Event "FileReadError" = fprime_svc::file_downlink::FileDownlink::EVENTID_FILE_READ_ERROR,
                Event "FileSent" = fprime_svc::file_downlink::FileDownlink::EVENTID_FILE_SENT,
                Event "DownlinkCanceled" = fprime_svc::file_downlink::FileDownlink::EVENTID_DOWNLINK_CANCELED,
                Event "DownlinkPartialWarning" = fprime_svc::file_downlink::FileDownlink::EVENTID_DOWNLINK_PARTIAL_WARNING,
                Event "DownlinkPartialFail" = fprime_svc::file_downlink::FileDownlink::EVENTID_DOWNLINK_PARTIAL_FAIL,
                Event "SendDataFail" = fprime_svc::file_downlink::FileDownlink::EVENTID_SEND_DATA_FAIL,
                Event "SendStarted" = fprime_svc::file_downlink::FileDownlink::EVENTID_SEND_STARTED,
                Event "DownlinkZeroSizeFile" = fprime_svc::file_downlink::FileDownlink::EVENTID_DOWNLINK_ZERO_SIZE_FILE,
                Event "FilenameSourceOverflow" = fprime_svc::file_downlink::FileDownlink::EVENTID_FILENAME_SOURCE_OVERFLOW,
                Event "FilenameDestinationOverflow" = fprime_svc::file_downlink::FileDownlink::EVENTID_FILENAME_DESTINATION_OVERFLOW,
                Event "SourceOutOfSandbox" = fprime_svc::file_downlink::FileDownlink::EVENTID_SOURCE_OUT_OF_SANDBOX,
                Channel "FilesSent" = fprime_svc::file_downlink::FileDownlink::CHANID_FILES_SENT,
                Channel "PacketsSent" = fprime_svc::file_downlink::FileDownlink::CHANID_PACKETS_SENT,
                Channel "Warnings" = fprime_svc::file_downlink::FileDownlink::CHANID_WARNINGS,
            ],
        ),
        instance(
            "fileManager",
            FILE_MANAGER_BASE_ID,
            entries![
                Container "FileDpContainer" = fprime_svc::file_manager::CONTAINER_ID_FILE_DP,
                Record "FileChunkHeaderRecord" = fprime_svc::file_manager::RECORD_ID_FILE_CHUNK_HEADER,
                Record "FileChunkDataRecord" = fprime_svc::file_manager::RECORD_ID_FILE_CHUNK_DATA,
                Command "CreateDirectory" = fprime_svc::file_manager::FileManager::OPCODE_CREATE_DIRECTORY,
                Command "MoveFile" = fprime_svc::file_manager::FileManager::OPCODE_MOVE_FILE,
                Command "RemoveDirectory" = fprime_svc::file_manager::FileManager::OPCODE_REMOVE_DIRECTORY,
                Command "RemoveFile" = fprime_svc::file_manager::FileManager::OPCODE_REMOVE_FILE,
                Command "AppendFile" = fprime_svc::file_manager::FileManager::OPCODE_APPEND_FILE,
                Command "FileSize" = fprime_svc::file_manager::FileManager::OPCODE_FILE_SIZE,
                Command "ListDirectory" = fprime_svc::file_manager::FileManager::OPCODE_LIST_DIRECTORY,
                Command "CalculateCrc" = fprime_svc::file_manager::FileManager::OPCODE_CALCULATE_CRC,
                Command "GenerateDp" = fprime_svc::file_manager::FileManager::OPCODE_GENERATE_DP,
                Event "DirectoryCreateError" = fprime_svc::file_manager::FileManager::EVENTID_DIRECTORY_CREATE_ERROR,
                Event "DirectoryRemoveError" = fprime_svc::file_manager::FileManager::EVENTID_DIRECTORY_REMOVE_ERROR,
                Event "FileMoveError" = fprime_svc::file_manager::FileManager::EVENTID_FILE_MOVE_ERROR,
                Event "FileRemoveError" = fprime_svc::file_manager::FileManager::EVENTID_FILE_REMOVE_ERROR,
                Event "AppendFileFailed" = fprime_svc::file_manager::FileManager::EVENTID_APPEND_FILE_FAILED,
                Event "AppendFileSucceeded" = fprime_svc::file_manager::FileManager::EVENTID_APPEND_FILE_SUCCEEDED,
                Event "CreateDirectorySucceeded" = fprime_svc::file_manager::FileManager::EVENTID_CREATE_DIRECTORY_SUCCEEDED,
                Event "RemoveDirectorySucceeded" = fprime_svc::file_manager::FileManager::EVENTID_REMOVE_DIRECTORY_SUCCEEDED,
                Event "MoveFileSucceeded" = fprime_svc::file_manager::FileManager::EVENTID_MOVE_FILE_SUCCEEDED,
                Event "RemoveFileSucceeded" = fprime_svc::file_manager::FileManager::EVENTID_REMOVE_FILE_SUCCEEDED,
                Event "AppendFileStarted" = fprime_svc::file_manager::FileManager::EVENTID_APPEND_FILE_STARTED,
                Event "CreateDirectoryStarted" = fprime_svc::file_manager::FileManager::EVENTID_CREATE_DIRECTORY_STARTED,
                Event "RemoveDirectoryStarted" = fprime_svc::file_manager::FileManager::EVENTID_REMOVE_DIRECTORY_STARTED,
                Event "MoveFileStarted" = fprime_svc::file_manager::FileManager::EVENTID_MOVE_FILE_STARTED,
                Event "RemoveFileStarted" = fprime_svc::file_manager::FileManager::EVENTID_REMOVE_FILE_STARTED,
                Event "FileSizeSucceeded" = fprime_svc::file_manager::FileManager::EVENTID_FILE_SIZE_SUCCEEDED,
                Event "FileSizeError" = fprime_svc::file_manager::FileManager::EVENTID_FILE_SIZE_ERROR,
                Event "FileSizeStarted" = fprime_svc::file_manager::FileManager::EVENTID_FILE_SIZE_STARTED,
                Event "ListDirectoryStarted" = fprime_svc::file_manager::FileManager::EVENTID_LIST_DIRECTORY_STARTED,
                Event "ListDirectorySucceeded" = fprime_svc::file_manager::FileManager::EVENTID_LIST_DIRECTORY_SUCCEEDED,
                Event "ListDirectoryError" = fprime_svc::file_manager::FileManager::EVENTID_LIST_DIRECTORY_ERROR,
                Event "DirectoryListing" = fprime_svc::file_manager::FileManager::EVENTID_DIRECTORY_LISTING,
                Event "DirectoryListingSubdir" = fprime_svc::file_manager::FileManager::EVENTID_DIRECTORY_LISTING_SUBDIR,
                Event "CalculateCrcStarted" = fprime_svc::file_manager::FileManager::EVENTID_CALCULATE_CRC_STARTED,
                Event "CalculateCrcFailed" = fprime_svc::file_manager::FileManager::EVENTID_CALCULATE_CRC_FAILED,
                Event "CalculateCrcSucceeded" = fprime_svc::file_manager::FileManager::EVENTID_CALCULATE_CRC_SUCCEEDED,
                Event "FileNameFormatError" = fprime_svc::file_manager::FileManager::EVENTID_FILE_NAME_FORMAT_ERROR,
                Event "GenerateDpStarted" = fprime_svc::file_manager::FileManager::EVENTID_GENERATE_DP_STARTED,
                Event "GenerateDpComplete" = fprime_svc::file_manager::FileManager::EVENTID_GENERATE_DP_COMPLETE,
                Event "GenerateDpFailed" = fprime_svc::file_manager::FileManager::EVENTID_GENERATE_DP_FAILED,
                Event "GenerateDpBufferFailed" = fprime_svc::file_manager::FileManager::EVENTID_GENERATE_DP_BUFFER_FAILED,
                Event "GenerateDpInvalidRange" = fprime_svc::file_manager::FileManager::EVENTID_GENERATE_DP_INVALID_RANGE,
                Event "PathOutsideSandbox" = fprime_svc::file_manager::FileManager::EVENTID_PATH_OUTSIDE_SANDBOX,
                Channel "CommandsExecuted" = fprime_svc::file_manager::FileManager::CHANID_COMMANDS_EXECUTED,
                Channel "Errors" = fprime_svc::file_manager::FileManager::CHANID_ERRORS,
            ],
        ),
        instance(
            "prmDb",
            PRM_DB_BASE_ID,
            entries![
                Command "PRM_SAVE_FILE" = fprime_svc::prm_db::PrmDb::OPCODE_PRM_SAVE_FILE,
                Command "PRM_LOAD_FILE" = fprime_svc::prm_db::PrmDb::OPCODE_PRM_LOAD_FILE,
                Command "PRM_COMMIT_STAGED" = fprime_svc::prm_db::PrmDb::OPCODE_PRM_COMMIT_STAGED,
                Event "PrmIdNotFound" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_ID_NOT_FOUND,
                Event "PrmIdUpdated" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_ID_UPDATED,
                Event "PrmDbFull" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_DB_FULL,
                Event "PrmIdAdded" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_ID_ADDED,
                Event "PrmFileWriteError" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_FILE_WRITE_ERROR,
                Event "PrmFileSaveComplete" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_FILE_SAVE_COMPLETE,
                Event "PrmFileReadError" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_FILE_READ_ERROR,
                Event "PrmFileLoadComplete" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_FILE_LOAD_COMPLETE,
                Event "PrmDbCommitComplete" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_DB_COMMIT_COMPLETE,
                Event "PrmDbCopyAllComplete" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_DB_COPY_ALL_COMPLETE,
                Event "PrmDbFileLoadFailed" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_DB_FILE_LOAD_FAILED,
                Event "PrmDbFileLoadInvalidAction" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_DB_FILE_LOAD_INVALID_ACTION,
                Event "PrmFileBadCrc" = fprime_svc::prm_db::PrmDb::EVENTID_PRM_FILE_BAD_CRC,
            ],
        ),
        instance(
            "rateGroup1Comp",
            RATE_GROUP_1_BASE_ID,
            entries![
                Event "RateGroupStarted" = fprime_svc::active_rate_group::ActiveRateGroup::EVENTID_RATE_GROUP_STARTED,
                Event "RateGroupCycleSlip" = fprime_svc::active_rate_group::ActiveRateGroup::EVENTID_RATE_GROUP_CYCLE_SLIP,
                Event "RateGroupTimeGetError" = fprime_svc::active_rate_group::ActiveRateGroup::EVENTID_RATE_GROUP_TIME_GET_ERROR,
                Channel "RgMaxTime" = fprime_svc::active_rate_group::ActiveRateGroup::CHANID_RG_MAX_TIME,
                Channel "RgCycleSlips" = fprime_svc::active_rate_group::ActiveRateGroup::CHANID_RG_CYCLE_SLIPS,
            ],
        ),
        instance(
            "rateGroup2Comp",
            RATE_GROUP_2_BASE_ID,
            entries![
                Event "RateGroupStarted" = fprime_svc::active_rate_group::ActiveRateGroup::EVENTID_RATE_GROUP_STARTED,
                Event "RateGroupCycleSlip" = fprime_svc::active_rate_group::ActiveRateGroup::EVENTID_RATE_GROUP_CYCLE_SLIP,
                Event "RateGroupTimeGetError" = fprime_svc::active_rate_group::ActiveRateGroup::EVENTID_RATE_GROUP_TIME_GET_ERROR,
                Channel "RgMaxTime" = fprime_svc::active_rate_group::ActiveRateGroup::CHANID_RG_MAX_TIME,
                Channel "RgCycleSlips" = fprime_svc::active_rate_group::ActiveRateGroup::CHANID_RG_CYCLE_SLIPS,
            ],
        ),
        instance(
            "rateGroup3Comp",
            RATE_GROUP_3_BASE_ID,
            entries![
                Event "RateGroupStarted" = fprime_svc::active_rate_group::ActiveRateGroup::EVENTID_RATE_GROUP_STARTED,
                Event "RateGroupCycleSlip" = fprime_svc::active_rate_group::ActiveRateGroup::EVENTID_RATE_GROUP_CYCLE_SLIP,
                Event "RateGroupTimeGetError" = fprime_svc::active_rate_group::ActiveRateGroup::EVENTID_RATE_GROUP_TIME_GET_ERROR,
                Channel "RgMaxTime" = fprime_svc::active_rate_group::ActiveRateGroup::CHANID_RG_MAX_TIME,
                Channel "RgCycleSlips" = fprime_svc::active_rate_group::ActiveRateGroup::CHANID_RG_CYCLE_SLIPS,
            ],
        ),
        instance(
            "cmdSeq",
            CMD_SEQUENCER_BASE_ID,
            entries![
                Command "CS_RUN" = fprime_svc::cmd_sequencer::CmdSequencer::OPCODE_CS_RUN,
                Command "CS_VALIDATE" = fprime_svc::cmd_sequencer::CmdSequencer::OPCODE_CS_VALIDATE,
                Command "CS_CANCEL" = fprime_svc::cmd_sequencer::CmdSequencer::OPCODE_CS_CANCEL,
                Command "CS_START" = fprime_svc::cmd_sequencer::CmdSequencer::OPCODE_CS_START,
                Command "CS_STEP" = fprime_svc::cmd_sequencer::CmdSequencer::OPCODE_CS_STEP,
                Command "CS_AUTO" = fprime_svc::cmd_sequencer::CmdSequencer::OPCODE_CS_AUTO,
                Command "CS_MANUAL" = fprime_svc::cmd_sequencer::CmdSequencer::OPCODE_CS_MANUAL,
                Command "CS_JOIN_WAIT" = fprime_svc::cmd_sequencer::CmdSequencer::OPCODE_CS_JOIN_WAIT,
                Event "CS_SequenceLoaded" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_SEQUENCE_LOADED,
                Event "CS_SequenceCanceled" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_SEQUENCE_CANCELED,
                Event "CS_FileReadError" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_FILE_READ_ERROR,
                Event "CS_FileInvalid" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_FILE_INVALID,
                Event "CS_RecordInvalid" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_RECORD_INVALID,
                Event "CS_FileSizeError" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_FILE_SIZE_ERROR,
                Event "CS_FileNotFound" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_FILE_NOT_FOUND,
                Event "CS_FileCrcFailure" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_FILE_CRC_FAILURE,
                Event "CS_CommandComplete" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_COMMAND_COMPLETE,
                Event "CS_SequenceComplete" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_SEQUENCE_COMPLETE,
                Event "CS_CommandError" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_COMMAND_ERROR,
                Event "CS_InvalidMode" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_INVALID_MODE,
                Event "CS_RecordMismatch" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_RECORD_MISMATCH,
                Event "CS_TimeBaseMismatch" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_TIME_BASE_MISMATCH,
                Event "CS_TimeContextMismatch" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_TIME_CONTEXT_MISMATCH,
                Event "CS_PortSequenceStarted" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_PORT_SEQUENCE_STARTED,
                Event "CS_UnexpectedCompletion" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_UNEXPECTED_COMPLETION,
                Event "CS_ModeSwitched" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_MODE_SWITCHED,
                Event "CS_NoSequenceActive" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE,
                Event "CS_SequenceValid" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_SEQUENCE_VALID,
                Event "CS_SequenceTimeout" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_SEQUENCE_TIMEOUT,
                Event "CS_CmdStepped" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_CMD_STEPPED,
                Event "CS_CmdStarted" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_CMD_STARTED,
                Event "CS_JoinWaiting" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_JOIN_WAITING,
                Event "CS_JoinWaitingNotComplete" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_JOIN_WAITING_NOT_COMPLETE,
                Event "CS_NoRecords" = fprime_svc::cmd_sequencer::CmdSequencer::EVENTID_CS_NO_RECORDS,
                Channel "CS_LoadCommands" = fprime_svc::cmd_sequencer::CmdSequencer::CHANID_CS_LOAD_COMMANDS,
                Channel "CS_CancelCommands" = fprime_svc::cmd_sequencer::CmdSequencer::CHANID_CS_CANCEL_COMMANDS,
                Channel "CS_Errors" = fprime_svc::cmd_sequencer::CmdSequencer::CHANID_CS_ERRORS,
                Channel "CS_CommandsExecuted" = fprime_svc::cmd_sequencer::CmdSequencer::CHANID_CS_COMMANDS_EXECUTED,
                Channel "CS_SequencesCompleted" = fprime_svc::cmd_sequencer::CmdSequencer::CHANID_CS_SEQUENCES_COMPLETED,
                Channel "CS_CurrentSequence" = fprime_svc::cmd_sequencer::CmdSequencer::CHANID_CS_CURRENT_SEQUENCE,
            ],
        ),
        instance(
            "signalGen",
            SIGNAL_GEN_BASE_ID,
            entries![
                Container "DataContainer" = fprime_ref::signal_gen::CONTAINER_ID_DATA,
                Record "DataRecord" = fprime_ref::signal_gen::RECORD_ID_DATA,
                Command "Settings" = fprime_ref::signal_gen::OPCODE_SETTINGS,
                Command "Toggle" = fprime_ref::signal_gen::OPCODE_TOGGLE,
                Command "Skip" = fprime_ref::signal_gen::OPCODE_SKIP,
                Command "Dp" = fprime_ref::signal_gen::OPCODE_DP,
                Command "AMPLITUDE_PRM_SET" = fprime_ref::signal_gen::OPCODE_AMPLITUDE_PARAM_SET,
                Command "AMPLITUDE_PRM_SAVE" = fprime_ref::signal_gen::OPCODE_AMPLITUDE_PARAM_SAVE,
                Parameter "Amplitude" = fprime_ref::signal_gen::PARAMID_AMPLITUDE,
                Event "SettingsChanged" = fprime_ref::signal_gen::EVENTID_SETTINGS_CHANGED,
                Event "Toggled" = fprime_ref::signal_gen::EVENTID_TOGGLED,
                Event "SampleSkipped" = fprime_ref::signal_gen::EVENTID_SAMPLE_SKIPPED,
                Event "DpSent" = fprime_ref::signal_gen::EVENTID_DP_SENT,
                Event "DpBufferFailed" = fprime_ref::signal_gen::EVENTID_DP_BUFFER_FAILED,
                Event "AmplitudeUpdated" = fprime_ref::signal_gen::EVENTID_AMPLITUDE_UPDATED,
                Channel "SignalValue" = fprime_ref::signal_gen::CHANID_SIGNAL_VALUE,
                Channel "SignalType" = fprime_ref::signal_gen::CHANID_SIGNAL_TYPE,
            ],
        ),
        instance(
            "systemResources",
            SYSTEM_RESOURCES_BASE_ID,
            entries![
                Command "ENABLE" = fprime_svc::system_resources::SystemResources::OPCODE_ENABLE,
                Channel "MEMORY_TOTAL" = fprime_svc::system_resources::SystemResources::CHANID_MEMORY_TOTAL,
                Channel "MEMORY_USED" = fprime_svc::system_resources::SystemResources::CHANID_MEMORY_USED,
                Channel "NON_VOLATILE_TOTAL" = fprime_svc::system_resources::SystemResources::CHANID_NON_VOLATILE_TOTAL,
                Channel "NON_VOLATILE_FREE" = fprime_svc::system_resources::SystemResources::CHANID_NON_VOLATILE_FREE,
                Channel "CPU" = fprime_svc::system_resources::SystemResources::CHANID_CPU,
                Channel "CPU_00" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00,
                Channel "CPU_01" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 1,
                Channel "CPU_02" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 2,
                Channel "CPU_03" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 3,
                Channel "CPU_04" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 4,
                Channel "CPU_05" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 5,
                Channel "CPU_06" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 6,
                Channel "CPU_07" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 7,
                Channel "CPU_08" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 8,
                Channel "CPU_09" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 9,
                Channel "CPU_10" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 10,
                Channel "CPU_11" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 11,
                Channel "CPU_12" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 12,
                Channel "CPU_13" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 13,
                Channel "CPU_14" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 14,
                Channel "CPU_15" = fprime_svc::system_resources::SystemResources::CHANID_CPU_00 + 15,
            ],
        ),
        instance(
            "comLog",
            COM_LOGGER_BASE_ID,
            entries![
                Command "CloseFile" = fprime_svc::com_logger::ComLogger::OPCODE_CLOSE_FILE,
                Event "FileOpenError" = fprime_svc::com_logger::ComLogger::EVENTID_FILE_OPEN_ERROR,
                Event "FileWriteError" = fprime_svc::com_logger::ComLogger::EVENTID_FILE_WRITE_ERROR,
                Event "FileValidationError" = fprime_svc::com_logger::ComLogger::EVENTID_FILE_VALIDATION_ERROR,
                Event "FileClosed" = fprime_svc::com_logger::ComLogger::EVENTID_FILE_CLOSED,
                Event "FileNotInitialized" = fprime_svc::com_logger::ComLogger::EVENTID_FILE_NOT_INITIALIZED,
            ],
        ),
    ]
}

fn repo_root() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn dictionary() -> Json {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("dictionary/RefTopologyDictionary.json");
    let text = std::fs::read_to_string(&path).expect("read the Ref dictionary");
    Json::parse(&text).expect("parse the Ref dictionary")
}

/// The full hash on the only non-comment line of `FPRIME_UPSTREAM_COMMIT`.
fn pinned_commit() -> String {
    let text = std::fs::read_to_string(repo_root().join("FPRIME_UPSTREAM_COMMIT"))
        .expect("read FPRIME_UPSTREAM_COMMIT");
    let lines: Vec<&str> = text
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .collect();
    assert_eq!(lines.len(), 1, "FPRIME_UPSTREAM_COMMIT: one hash line");
    lines[0].to_string()
}

#[test]
fn upstream_pin() {
    let pin = pinned_commit();
    assert!(
        pin.len() == 40
            && pin
                .bytes()
                .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase()),
        "FPRIME_UPSTREAM_COMMIT must hold a full lower-case commit hash, not {pin:?}"
    );
    let dict = dictionary();
    let version = dict
        .get("metadata")
        .and_then(|m| m.get("frameworkVersion"))
        .and_then(Json::as_str)
        .expect("metadata.frameworkVersion");
    assert_eq!(
        version,
        &pin[..7],
        "the dictionary was generated from {version}, but the port is pinned to {pin}: \
         regenerate it with crates/fprime-ref/fpp/generate-dictionary.sh"
    );
}

/// `Svc::Version` reports the same framework/project/library versions the
/// dictionary's metadata records (the defaults in
/// `fprime_config::version`).
#[test]
fn version_strings_match_the_dictionary_metadata() {
    let dict = dictionary();
    let metadata = dict.get("metadata").expect("metadata");
    let field = |name: &str| {
        metadata
            .get(name)
            .and_then(Json::as_str)
            .unwrap_or_else(|| panic!("metadata.{name}"))
            .to_string()
    };
    assert_eq!(
        fprime_svc::version::FRAMEWORK_VERSION,
        &pinned_commit()[..7]
    );
    assert_eq!(
        fprime_svc::version::FRAMEWORK_VERSION,
        field("frameworkVersion")
    );
    assert_eq!(
        fprime_svc::version::PROJECT_VERSION,
        field("projectVersion")
    );
    let libraries: Vec<String> = metadata
        .get("libraryVersions")
        .and_then(Json::as_arr)
        .expect("metadata.libraryVersions")
        .iter()
        .map(|l| l.as_str().expect("library version string").to_string())
        .collect();
    assert_eq!(fprime_svc::version::LIBRARY_VERSIONS, libraries.as_slice());
}

#[test]
fn rust_ids_match_the_dictionary() {
    let dict = dictionary();
    // (kind, instance, name) -> id, from the dictionary.
    let mut in_dictionary: BTreeMap<(Kind, String, String), i128> = BTreeMap::new();
    for kind in Kind::ALL {
        let section = dict
            .get(kind.section())
            .and_then(Json::as_arr)
            .unwrap_or_else(|| panic!("dictionary section {}", kind.section()));
        for entry in section {
            let qualified = entry.get("name").and_then(Json::as_str).expect("name");
            let id = match entry.get(kind.id_key()) {
                Some(Json::Int(id)) => *id,
                other => panic!("{qualified}: {} is {other:?}", kind.id_key()),
            };
            let mut parts = qualified.splitn(3, '.');
            let (Some("Ref"), Some(instance), Some(name)) =
                (parts.next(), parts.next(), parts.next())
            else {
                panic!("unexpected dictionary name {qualified}");
            };
            in_dictionary.insert((kind, instance.to_string(), name.to_string()), id);
        }
    }

    let instances = rust_instances();
    let mut problems = Vec::new();
    let mut seen = BTreeSet::new();
    let mut names = BTreeSet::new();
    for inst in &instances {
        assert!(
            names.insert(inst.name),
            "instance {} listed twice",
            inst.name
        );
        for &(kind, name, local) in &inst.entries {
            let key = (kind, inst.name.to_string(), name.to_string());
            if !seen.insert(key.clone()) {
                problems.push(format!("{kind:?} Ref.{}.{name} listed twice", inst.name));
                continue;
            }
            let want = i128::from(inst.base) + i128::from(local);
            match in_dictionary.get(&key) {
                Some(&id) if id == want => {}
                Some(&id) => problems.push(format!(
                    "{kind:?} Ref.{}.{name}: Rust id {want:#x} (base {:#x} + {local:#x}), dictionary {id:#x}",
                    inst.name, inst.base
                )),
                None => problems.push(format!(
                    "{kind:?} Ref.{}.{name}: Rust constant {local:#x} has no dictionary entry",
                    inst.name
                )),
            }
        }
    }
    for ((kind, instance, name), id) in &in_dictionary {
        if !names.contains(instance.as_str()) {
            problems.push(format!(
                "{kind:?} Ref.{instance}.{name} ({id:#x}): instance {instance} has no row in rust_instances()"
            ));
        } else if !seen.contains(&(*kind, instance.clone(), name.clone())) {
            problems.push(format!(
                "{kind:?} Ref.{instance}.{name} ({id:#x}): no Rust constant listed"
            ));
        }
    }
    assert!(
        problems.is_empty(),
        "the Rust ids and the dictionary disagree ({} problems):\n  {}",
        problems.len(),
        problems.join("\n  ")
    );
}
