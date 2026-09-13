//! # RefTopology — the reference deployment topology
//!
//! Rust port of `TestDeploymentsProject/Ref/Top/RefTopology.cpp` plus the
//! `CdhCore` / `ComFprime` / `FileHandling` / `DataProducts` /
//! `ComLoggerTee` subtopologies it composes (analysis:
//! `docs/cpp-analysis/ref-topology.md`; comms wiring mirrors
//! `Svc/Subtopologies/ComFprime/ComFprime.fpp` exactly, the file /
//! data-product / logger wiring mirrors `FileHandling.fpp`,
//! `DataProducts.fpp` and `Ref/Top/topology.fpp`).
//!
//! ## Instance / base-ID map (0xDSSCCxxx convention, instances 0x1000 apart)
//!
//! | Instance            | Component              | Base ID       | Window        |
//! |---------------------|------------------------|---------------|---------------|
//! | `cmdDisp`           | `Svc.CommandDispatcher`| `0x0100_0000` | CdhCore       |
//! | `events`            | `Svc.EventManager`     | `0x0100_1000` | CdhCore       |
//! | `health`            | `Svc.Health`           | `0x0100_2000` | CdhCore       |
//! | `textLogger`        | `Svc.PassiveTextLogger`| `0x0100_3000` | CdhCore       |
//! | `fatalHandler`      | `Svc.FatalHandler`     | `0x0100_4000` | CdhCore       |
//! | `tlmSend`           | `Svc.TlmChan`          | `0x0100_5000` | CdhCore       |
//! | `comQueue`          | `Svc.ComQueue`         | `0x0200_0000` | ComFprime     |
//! | `frameAccumulator`  | `Svc.FrameAccumulator` | `0x0200_1000` | ComFprime     |
//! | `bufferManager`     | `Svc.BufferManager`    | `0x0200_2000` | ComFprime     |
//! | `deframer`          | `Svc.FprimeDeframer`   | `0x0200_3000` | ComFprime     |
//! | `framer`            | `Svc.FprimeFramer`     | `0x0200_4000` | ComFprime     |
//! | `fprimeRouter`      | `Svc.FprimeRouter`     | `0x0200_5000` | ComFprime     |
//! | `comStub`           | `Svc.ComStub`          | `0x0200_6000` | ComFprime     |
//! | `dpCat`             | `Svc.DpCatalog`        | `0x0400_0000` | DataProducts  |
//! | `dpMgr`             | `Svc.DpManager`        | `0x0400_1000` | DataProducts  |
//! | `dpWriter`          | `Svc.DpWriter`         | `0x0400_2000` | DataProducts  |
//! | `dpBufferManager`   | `Svc.BufferManager`    | `0x0400_3000` | DataProducts  |
//! | `fileUplink`        | `Svc.FileUplink`       | `0x0500_0000` | FileHandling  |
//! | `fileDownlink`      | `Svc.FileDownlink`     | `0x0500_1000` | FileHandling  |
//! | `fileManager`       | `Svc.FileManager`      | `0x0500_2000` | FileHandling  |
//! | `prmDb`             | `Svc.PrmDb`            | `0x0500_3000` | FileHandling  |
//! | `rateGroup1Comp`    | `Svc.ActiveRateGroup`  | `0x1000_1000` | Ref (main)    |
//! | `rateGroup2Comp`    | `Svc.ActiveRateGroup`  | `0x1000_2000` | Ref (main)    |
//! | `rateGroup3Comp`    | `Svc.ActiveRateGroup`  | `0x1000_3000` | Ref (main)    |
//! | `cmdSeq`            | `Svc.CmdSequencer`     | `0x1000_6000` | Ref (main)    |
//! | `signalGen`         | `Ref.SignalGen`        | `0x1001_1000` | Ref (SG1 slot)|
//! | `posixTime`         | `Svc.PosixTime`        | `0x1002_0000` | Ref (main)    |
//! | `rateGroupDriverComp`| `Svc.RateGroupDriver` | `0x1002_1000` | Ref (main)    |
//! | `systemResources`   | `Svc.SystemResources`  | `0x1002_3000` | Ref (main)    |
//! | `linuxTimer`        | `Svc.LinuxTimer`       | `0x1002_4000` | Ref (main)    |
//! | `comDriver`         | `Drv.TcpClient`        | `0x1002_5000` | Ref (main)    |
//! | `comLog`            | `Svc.ComLogger`        | `0x1050_0000` | ComLoggerTee  |
//! | `comSplitter`       | `Svc.ComSplitter`      | `0x1050_0100` | ComLoggerTee  |
//!
//! (Base IDs are the C++ ones: `CdhCore` 0x01000000, `ComFprime`
//! 0x02000000 — this deployment frames with `ComFprime`, not `ComCcsds` —
//! `DataProducts` 0x04000000, `FileHandling` 0x05000000, `ComLoggerTee`
//! 0x10500000, main-topology instances per `Ref/Top/instances.fpp`.)
//!
//! ## Command-port index map (cmdDisp `compCmdReg`/`compCmdSend`, matched)
//!
//! 0 `cmdDisp` (self), 1 `events`, 2 `health`, 3 `comQueue`, 4 `signalGen`,
//! 5 `fileDownlink`, 6 `fileManager`, 7 `prmDb`, 8 `cmdSeq`, 9 `dpMgr`,
//! 10 `dpWriter`, 11 `dpCat`, 12 `systemResources`, 13 `comLog`.
//! (`fileUplink` declares no commands, as in C++.)
//!
//! ## Command-source index map (cmdDisp `seqCmdBuff`/`seqCmdStatus`, matched)
//!
//! 0 `fprimeRouter` (the uplink), 1 `cmdSeq` (the sequencer).
//!
//! ## Health ping index map (`pingSend[i]`/`pingReturn[i]`, WARN 3 FATAL 5)
//!
//! 0 `rateGroup1Comp`, 1 `rateGroup2Comp`, 2 `rateGroup3Comp`,
//! 3 `cmdDisp`, 4 `events`, 5 `tlmSend`, 6 `fileUplink`, 7 `fileDownlink`,
//! 8 `fileManager`, 9 `prmDb`, 10 `cmdSeq`, 11 `dpCat`, 12 `comLog`.
//!
//! ## Deviations from the C++ topology (each deliberate)
//!
//! - **No FileHandling `BufferManager`.** Neither `Svc::FileUplink` nor
//!   `Svc::FileDownlink` has a buffer-allocation port in C++ or in this
//!   port: uplink file buffers are allocated by `frameAccumulator` from the
//!   comms `bufferManager` and returned through
//!   `fileUplink.bufferSendOut -> fprimeRouter.fileBufferReturnIn`, and
//!   `FileDownlink` owns its two-slot packet pool. A separate
//!   file-handling `BufferManager` instance would therefore have no
//!   consumer, so none is created; the data-product chain does get its own
//!   (`dpBufferManager`), because `dpMgr.bufferGetOut` really allocates.
//! - **No `Svc::BufferAccumulator`** between `dpMgr` and `dpWriter` (not
//!   ported): `dpMgr.productSendOut[0]` feeds `dpWriter.bufferSendIn`
//!   directly and `dpWriter.deallocBufferSendOut` returns straight to
//!   `dpBufferManager`. Same ownership cycle, one hop shorter.
//! - **`ComSplitter` lives in this crate** ([`crate::com_splitter`]) since
//!   `fprime-svc` is owned by another wave; it is a faithful port of
//!   `Svc/ComSplitter`.
//! - `signalGen` implements only the synchronous data-product request kind
//!   (see [`crate::signal_gen`]).
//! - `comLog`'s `.com` file is closed (and its `.CRC32` sidecar written) by
//!   the `CLOSE_FILE` command or by the component's destructor. In C++ the
//!   destructor runs at process exit; here the port graph is a cycle of
//!   `Arc`s, so `Drop` does not run at teardown and the sidecar is only
//!   written on command. The log bytes themselves are on disk either way
//!   (every record is written straight through, no buffering).
//!
//! The comms chain is always wired internally (as the C++ Ref topology
//! wires ComFprime); only the TCP driver is optional (`-a`/`-p`). Without a
//! driver, `comStub`'s driver-side ports stay unconnected — nothing invokes
//! them because ComQueue starts WAITING and is only primed by a driver
//! `ready` — and downlink packets accumulate/overflow in ComQueue exactly
//! as C++ does when comms are down. The in-process integration tests wire a
//! loopback driver onto those same ports.
//!
//! Everything the deployment writes lives under one data directory
//! ([`TopologyConfig::data_dir`], `--data-dir`, default
//! `<tmp>/fprime-ref-<pid>`): the parameter database, uplinked files,
//! data-product files and catalog, and the `.com` logs.

use std::sync::Arc;

use fprime_comp::PortRef;
use fprime_config::{FwIdType, FwSizeType, FwTaskPriorityType};
use fprime_fw::{TimeInterval, fw_log};
use fprime_os::task::TASK_DEFAULT;

use fprime_drv::socket_helper::SocketIpStatus;
use fprime_drv::tcp_client::TcpClient;
use fprime_fw::FileNameString;
use fprime_svc::active_rate_group::ActiveRateGroup;
use fprime_svc::buffer_manager::{BufferBin, BufferManager};
use fprime_svc::cmd_dispatcher::CmdDispatcher;
use fprime_svc::cmd_sequencer::CmdSequencer;
use fprime_svc::com_logger::ComLogger;
use fprime_svc::com_queue::{ComQueue, QueueConfigurationTable};
use fprime_svc::com_stub::ComStub;
use fprime_svc::dp_catalog::DpCatalog;
use fprime_svc::dp_manager::DpManager;
use fprime_svc::dp_writer::DpWriter;
use fprime_svc::event_manager::EventManager;
use fprime_svc::fatal_handler::FatalHandler;
use fprime_svc::file_downlink::FileDownlink;
use fprime_svc::file_manager::FileManager;
use fprime_svc::file_uplink::FileUplink;
use fprime_svc::fprime_deframer::FprimeDeframer;
use fprime_svc::fprime_framer::FprimeFramer;
use fprime_svc::fprime_router::FprimeRouter;
use fprime_svc::frame_accumulator::{FprimeFrameDetector, FrameAccumulator};
use fprime_svc::health::{Health, PingEntry};
use fprime_svc::linux_timer::LinuxTimer;
use fprime_svc::passive_text_logger::PassiveTextLogger;
use fprime_svc::posix_time::PosixTime;
use fprime_svc::prm_db::PrmDb;
use fprime_svc::rate_group_driver::{Divider, RateGroupDriver};
use fprime_svc::system_resources::SystemResources;
use fprime_svc::tlm_chan::TlmChan;

use crate::com_splitter::ComSplitter;
use crate::shims::{DrvToSvcDataShim, DrvToSvcReadyShim, SvcToDrvSendShim};
use crate::signal_gen::SignalGen;

// ---------------------------------------------------------------------------
// Base IDs (see the instance map above)
// ---------------------------------------------------------------------------

/// `cmdDisp` base ID.
pub const CMD_DISPATCHER_BASE_ID: FwIdType = 0x0100_0000;
/// `events` base ID.
pub const EVENT_MANAGER_BASE_ID: FwIdType = 0x0100_1000;
/// `health` base ID.
pub const HEALTH_BASE_ID: FwIdType = 0x0100_2000;
/// `textLogger` base ID.
pub const TEXT_LOGGER_BASE_ID: FwIdType = 0x0100_3000;
/// `fatalHandler` base ID.
pub const FATAL_HANDLER_BASE_ID: FwIdType = 0x0100_4000;
/// `tlmSend` base ID.
pub const TLM_CHAN_BASE_ID: FwIdType = 0x0100_5000;
/// `comQueue` base ID.
pub const COM_QUEUE_BASE_ID: FwIdType = 0x0200_0000;
/// `frameAccumulator` base ID.
pub const FRAME_ACCUMULATOR_BASE_ID: FwIdType = 0x0200_1000;
/// `bufferManager` base ID.
pub const BUFFER_MANAGER_BASE_ID: FwIdType = 0x0200_2000;
/// `deframer` base ID.
pub const DEFRAMER_BASE_ID: FwIdType = 0x0200_3000;
/// `framer` base ID.
pub const FRAMER_BASE_ID: FwIdType = 0x0200_4000;
/// `fprimeRouter` base ID.
pub const ROUTER_BASE_ID: FwIdType = 0x0200_5000;
/// `comStub` base ID.
pub const COM_STUB_BASE_ID: FwIdType = 0x0200_6000;
/// `dpCat` base ID (DataProducts window).
pub const DP_CATALOG_BASE_ID: FwIdType = 0x0400_0000;
/// `dpMgr` base ID.
pub const DP_MANAGER_BASE_ID: FwIdType = 0x0400_1000;
/// `dpWriter` base ID.
pub const DP_WRITER_BASE_ID: FwIdType = 0x0400_2000;
/// `dpBufferManager` base ID.
pub const DP_BUFFER_MANAGER_BASE_ID: FwIdType = 0x0400_3000;
/// `fileUplink` base ID (FileHandling window).
pub const FILE_UPLINK_BASE_ID: FwIdType = 0x0500_0000;
/// `fileDownlink` base ID.
pub const FILE_DOWNLINK_BASE_ID: FwIdType = 0x0500_1000;
/// `fileManager` base ID.
pub const FILE_MANAGER_BASE_ID: FwIdType = 0x0500_2000;
/// `prmDb` base ID.
pub const PRM_DB_BASE_ID: FwIdType = 0x0500_3000;
/// `rateGroup1Comp` (1 Hz) base ID.
pub const RATE_GROUP_1_BASE_ID: FwIdType = 0x1000_1000;
/// `rateGroup2Comp` (0.5 Hz) base ID.
pub const RATE_GROUP_2_BASE_ID: FwIdType = 0x1000_2000;
/// `rateGroup3Comp` (0.25 Hz) base ID.
pub const RATE_GROUP_3_BASE_ID: FwIdType = 0x1000_3000;
/// `cmdSeq` base ID.
pub const CMD_SEQUENCER_BASE_ID: FwIdType = 0x1000_6000;
/// `signalGen` base ID (the C++ SG1 slot).
pub const SIGNAL_GEN_BASE_ID: FwIdType = 0x1001_1000;
/// `posixTime` base ID.
pub const POSIX_TIME_BASE_ID: FwIdType = 0x1002_0000;
/// `rateGroupDriverComp` base ID.
pub const RATE_GROUP_DRIVER_BASE_ID: FwIdType = 0x1002_1000;
/// `systemResources` base ID.
pub const SYSTEM_RESOURCES_BASE_ID: FwIdType = 0x1002_3000;
/// `linuxTimer` base ID.
pub const LINUX_TIMER_BASE_ID: FwIdType = 0x1002_4000;
/// `comDriver` (TcpClient) base ID.
pub const COM_DRIVER_BASE_ID: FwIdType = 0x1002_5000;
/// `comLog` base ID (ComLoggerTee window).
pub const COM_LOGGER_BASE_ID: FwIdType = 0x1050_0000;
/// `comSplitter` base ID (ComLoggerTee `BASE_ID + 0x0100`).
pub const COM_SPLITTER_BASE_ID: FwIdType = 0x1050_0100;

// ---------------------------------------------------------------------------
// Resource configuration (Ref `instances.fpp` / CdhCore / ComFprime values)
// ---------------------------------------------------------------------------

/// Default queue depth (Ref `Default.QUEUE_SIZE`).
const QUEUE_DEPTH: FwSizeType = 10;
/// EventManager queue depth (bursty producers).
const EVENTS_QUEUE_DEPTH: FwSizeType = 25;
/// Health queue depth. CdhCore declares `$health queue size 25`; this
/// deployment pings 13 instances per `Run`, and every reply lands in this
/// queue through an ASSERT-on-full async port (C++ parity), so the queue
/// is sized to hold several cycles' worth of replies rather than two.
const HEALTH_QUEUE_DEPTH: FwSizeType = 100;
/// ComQueue message-queue depth (ComFprime `QueueSizes.comQueue`).
const COM_QUEUE_DEPTH: FwSizeType = 50;
/// ComLogger message-queue depth. `ComLoggerTeeConfig.QueueSizes.comLog`
/// is 10; this deployment tees the WHOLE event stream into the logger and
/// `Fw.Com` async inputs assert on a full queue (C++ parity — an
/// `async input port` with no `drop` qualifier), so the queue is sized
/// like the ComQueue events queue instead. A logger that stalls longer
/// than 200 queued packets (a full or hung disk) still fails hard, exactly
/// as the C++ component does.
const COM_LOGGER_QUEUE_DEPTH: FwSizeType = 200;

/// Task priorities (best-effort no-ops on std; recorded for parity).
const RG1_PRIORITY: FwTaskPriorityType = 43;
const RG2_PRIORITY: FwTaskPriorityType = 42;
const RG3_PRIORITY: FwTaskPriorityType = 41;
const CMD_DISP_PRIORITY: FwTaskPriorityType = 35;
const COM_QUEUE_PRIORITY: FwTaskPriorityType = 29;
/// `DataProductsConfig.Priorities.dpCat`.
const DP_CATALOG_PRIORITY: FwTaskPriorityType = 24;
/// `FileHandlingConfig.Priorities.fileUplink`.
const FILE_UPLINK_PRIORITY: FwTaskPriorityType = 24;
const EVENTS_PRIORITY: FwTaskPriorityType = 23;
/// `FileHandlingConfig.Priorities.fileDownlink` / `DataProducts.dpMgr`.
const FILE_DOWNLINK_PRIORITY: FwTaskPriorityType = 23;
const DP_MANAGER_PRIORITY: FwTaskPriorityType = 23;
const TLM_PRIORITY: FwTaskPriorityType = 22;
/// `FileHandlingConfig.Priorities.fileManager` / `DataProducts.dpWriter`.
const FILE_MANAGER_PRIORITY: FwTaskPriorityType = 22;
const DP_WRITER_PRIORITY: FwTaskPriorityType = 22;
/// `FileHandlingConfig.Priorities.prmDb`.
const PRM_DB_PRIORITY: FwTaskPriorityType = 21;
/// `Ref/Top/instances.fpp`: `cmdSeq ... priority 20`.
const CMD_SEQ_PRIORITY: FwTaskPriorityType = 20;
/// `ComLoggerTeeConfig.Priorities.comLog`.
const COM_LOGGER_PRIORITY: FwTaskPriorityType = 18;

/// ComQueue queue-configuration-table indices and depths (ComFprime
/// config): EVENTS depth 200 pri 0, TELEMETRY depth 500 pri 2, FILE depth
/// 100 pri 1 — every entry needs depth > 0 (configure asserts).
///
/// The FILE entry is a **buffer** queue: table indices run
/// `[0, COM_PORT_COUNT)` for the `Fw.Com` packet queues and then
/// `COM_PORT_COUNT + <buffer queue index>`, so `fileDownlink`'s buffer
/// queue (port index 0) is table entry 2. The enum count leaking into the
/// table indexing is C++ behavior, reproduced here.
const EVENTS_QUEUE_INDEX: usize = 0;
const TELEMETRY_QUEUE_INDEX: usize = 1;
const FILE_QUEUE_INDEX: usize = 2;
/// `fileDownlink`'s port index within ComQueue's buffer-queue port array.
const FILE_BUFFER_QUEUE_PORT: fprime_config::FwIndexType = 0;

/// BufferManager: one 2048-byte x 20 bin (ComFprime `commsBuffSize`-style)
/// covering framer frames (512-payload + 12 overhead), accumulator frame
/// extraction, and TCP receive allocations.
const COMMS_BUFFER_SIZE: FwSizeType = 2048;
const COMMS_BUFFER_COUNT: u16 = 20;
/// BufferManager manager id (ComFprime `commsBuffMgrId`).
const COMMS_BUFFER_MGR_ID: u16 = 200;
/// FrameAccumulator ring size (task/ComFprime `frameAccumulatorSize`).
const FRAME_ACCUMULATOR_RING_SIZE: usize = 2048;
/// TCP receive allocation size.
const TCP_BUFFER_SIZE: FwSizeType = 1024;

/// Health ping thresholds (`PingEntries` WARN=3 FATAL=5 across Ref).
const HEALTH_WARN_CYCLES: FwSizeType = 3;
const HEALTH_FATAL_CYCLES: FwSizeType = 5;
/// CdhCore `HEALTH_WATCHDOG_CODE`.
const HEALTH_WATCHDOG_CODE: u32 = 0x123;

/// Data-product BufferManager bin (`DataProductsConfig.BuffMgr`:
/// `dpBufferStoreSize` x `dpBufferStoreCount`, manager id
/// `dpBufferManagerId`).
const DP_BUFFER_SIZE: FwSizeType = 10_000;
const DP_BUFFER_COUNT: u16 = 10;
const DP_BUFFER_MGR_ID: u16 = 300;

/// `FileHandlingConfig.DownlinkConfig` (cooldown/cycleTime in ms,
/// fileQueueDepth in entries).
const FILE_DOWNLINK_COOLDOWN: u32 = 1000;
const FILE_DOWNLINK_CYCLE_TIME: u32 = 1000;
const FILE_DOWNLINK_QUEUE_DEPTH: usize = 10;

/// `RefTopology.cpp`: `cmdSeq.allocateBuffer(0, mallocator, 5 * 1024)`.
const CMD_SEQ_ALLOCATOR_ID: fprime_config::FwEnumStoreType = 0;
const CMD_SEQ_BUFFER_BYTES: usize = 5 * 1024;

/// `ComLogger` rotation size and the `[u16 length][packet]` record mode
/// (`Svc/ComLogger` defaults used by the ComLoggerTee subtopology).
const COM_LOGGER_MAX_FILE_SIZE: u32 = 64 * 1024;
const COM_LOGGER_STORE_BUFFER_LENGTH: bool = true;

/// `comSplitter.comOut` index map: 0 keeps the normal downlink, 1 tees to
/// `comLog` (C++ `ComLoggerTee`).
const COM_SPLITTER_DOWNLINK_PORT: usize = 0;
const COM_SPLITTER_LOGGER_PORT: usize = 1;

/// Sub-paths under the deployment data directory (C++
/// `FileHandlingConfig::Paths` / `DataProductsConfig::Paths`, made
/// relative to the configured directory instead of the CWD).
const PRM_DB_FILE: &str = "PrmDb.dat";
const UPLINK_SUBDIR: &str = "uplink";
const DP_SUBDIR: &str = "DpCat";
const DP_STATE_FILE: &str = "DpState.dat";
const COM_LOGGER_PREFIX: &str = "comlog";

// ---------------------------------------------------------------------------
// TopologyState equivalent
// ---------------------------------------------------------------------------

/// CLI-derived deployment state (C++ `Ref::TopologyState`).
#[derive(Debug, Clone, Default)]
pub struct TopologyConfig {
    /// GDS hostname (dotted-quad IPv4). `None` disables comms.
    pub hostname: Option<String>,
    /// GDS TCP port. `0` disables comms (C++ parity: both must be given).
    pub port: u16,
    /// Directory every file-writing component works under (`--data-dir`):
    /// the parameter database, uplinked files, data products and their
    /// catalog, and the `.com` logs. `None` selects
    /// `<temp dir>/fprime-ref-<pid>`.
    ///
    /// The C++ deployment hard-codes `"PrmDb.dat"`, `"./DpCat"` and
    /// `"/tmp/uplink/"`; one configurable root keeps a test (or a second
    /// instance) from colliding with those fixed paths.
    pub data_dir: Option<String>,
}

impl TopologyConfig {
    /// Comms are enabled only when hostname AND a nonzero port are given
    /// (C++ `hostname != nullptr && port != 0`).
    fn comms_enabled(&self) -> Option<(&str, u16)> {
        match (&self.hostname, self.port) {
            (Some(hostname), port) if port != 0 => Some((hostname.as_str(), port)),
            _ => None,
        }
    }

    /// The resolved data directory (no trailing separator).
    #[must_use]
    pub fn resolved_data_dir(&self) -> String {
        let dir = match &self.data_dir {
            Some(dir) => dir.clone(),
            None => {
                let mut path = std::env::temp_dir();
                path.push(format!("fprime-ref-{}", std::process::id()));
                path.to_string_lossy().into_owned()
            }
        };
        // Strip trailing separators so the derived paths join cleanly;
        // `/` (and `///`) stay the root rather than becoming empty.
        let trimmed = dir.trim_end_matches('/');
        if trimmed.is_empty() {
            "/".to_string()
        } else {
            trimmed.to_string()
        }
    }
}

/// Join a directory and a child name with exactly one separator.
fn join(dir: &str, child: &str) -> String {
    if dir.ends_with('/') {
        format!("{dir}{child}")
    } else {
        format!("{dir}/{child}")
    }
}

/// The paths derived from [`TopologyConfig::resolved_data_dir`], created
/// (as directories) during the configure phase — the Rust stand-in for the
/// C++ `Paths` config modules plus `configComponents`'
/// `Os::FileSystem::createDirectory(dpDir)`.
#[derive(Debug, Clone)]
pub struct DataPaths {
    /// The root data directory.
    pub root: String,
    /// `prmDb` parameter file.
    pub prm_db_file: String,
    /// `fileUplink` sandbox directory (C++ `"/tmp/uplink/"`).
    pub uplink_dir: String,
    /// `dpWriter` file prefix / `dpCat` managed directory.
    pub dp_dir: String,
    /// `dpCat` state file.
    pub dp_state_file: String,
    /// `comLog` file prefix.
    pub com_log_prefix: String,
}

impl DataPaths {
    /// Derive every path from a data directory. A root of `/` stays `/`
    /// (rather than collapsing to an empty prefix), and no path ever ends
    /// up with a doubled separator.
    #[must_use]
    pub fn new(root: &str) -> DataPaths {
        let trimmed = root.trim_end_matches('/');
        let root = if trimmed.is_empty() {
            "/".to_string()
        } else {
            trimmed.to_string()
        };
        let dp_dir = join(&root, DP_SUBDIR);
        DataPaths {
            prm_db_file: join(&root, PRM_DB_FILE),
            uplink_dir: join(&root, UPLINK_SUBDIR),
            dp_state_file: join(&dp_dir, DP_STATE_FILE),
            com_log_prefix: join(&root, COM_LOGGER_PREFIX),
            dp_dir,
            root,
        }
    }

    /// Create the directories the components expect to exist. Failures are
    /// reported and left to the components (which answer with their own
    /// file-error events), exactly as an unwritable C++ `dpDir` would.
    fn create_directories(&self) {
        for dir in [&self.root, &self.uplink_dir, &self.dp_dir] {
            let status = fprime_os::filesystem::create_directory(dir, false);
            if status != fprime_os::filesystem::Status::OpOk {
                fw_log!("Could not create data directory {} ({:?})\n", dir, status);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// The topology
// ---------------------------------------------------------------------------

/// All instances of the reference deployment, wired and running after
/// [`RefTopology::setup`].
pub struct RefTopology {
    /// `posixTime` — `Svc.PosixTime` time source.
    pub posix_time: Arc<PosixTime>,
    /// `linuxTimer` — blocking 1 Hz cycle source.
    pub linux_timer: Arc<LinuxTimer>,
    /// `rateGroupDriverComp` — divides ticks by {1, 2, 4}.
    pub rate_group_driver: Arc<RateGroupDriver>,
    /// `rateGroup1Comp` — 1 Hz rate group.
    pub rate_group_1: Arc<ActiveRateGroup>,
    /// `rateGroup2Comp` — 0.5 Hz rate group.
    pub rate_group_2: Arc<ActiveRateGroup>,
    /// `rateGroup3Comp` — 0.25 Hz rate group.
    pub rate_group_3: Arc<ActiveRateGroup>,
    /// `cmdDisp` — command dispatcher.
    pub cmd_dispatcher: Arc<CmdDispatcher>,
    /// `events` — event manager.
    pub event_manager: Arc<EventManager>,
    /// `tlmSend` — telemetry channel store/downlinker.
    pub tlm_chan: Arc<TlmChan>,
    /// `health` — ping-based health monitor.
    pub health: Arc<Health>,
    /// `fatalHandler` — FATAL announce sink.
    pub fatal_handler: Arc<FatalHandler>,
    /// `textLogger` — console text-event sink.
    pub text_logger: Arc<PassiveTextLogger>,
    /// `bufferManager` — comms buffer pool.
    pub buffer_manager: Arc<BufferManager>,
    /// `comQueue` — prioritized downlink queue.
    pub com_queue: Arc<ComQueue>,
    /// `framer` — F Prime frame encoder.
    pub framer: Arc<FprimeFramer>,
    /// `deframer` — F Prime frame validator/decoder.
    pub deframer: Arc<FprimeDeframer>,
    /// `frameAccumulator` — byte-stream to frame accumulator.
    pub frame_accumulator: Arc<FrameAccumulator>,
    /// `fprimeRouter` — uplink packet router.
    pub router: Arc<FprimeRouter>,
    /// `comStub` — byte-stream com adapter.
    pub com_stub: Arc<ComStub>,
    /// `signalGen` — the demo component.
    pub signal_gen: Arc<SignalGen>,
    /// `cmdSeq` — command sequencer (command source index 1).
    pub cmd_sequencer: Arc<CmdSequencer>,
    /// `systemResources` — CPU/memory telemetry on rate group 1.
    pub system_resources: Arc<SystemResources>,
    /// `prmDb` — parameter database.
    pub prm_db: Arc<PrmDb>,
    /// `fileUplink` — uplinked file receiver.
    pub file_uplink: Arc<FileUplink>,
    /// `fileDownlink` — file downlink into ComQueue's buffer queue.
    pub file_downlink: Arc<FileDownlink>,
    /// `fileManager` — file-system command component.
    pub file_manager: Arc<FileManager>,
    /// `dpBufferManager` — data-product buffer pool.
    pub dp_buffer_manager: Arc<BufferManager>,
    /// `dpMgr` — data-product manager.
    pub dp_manager: Arc<DpManager>,
    /// `dpWriter` — data-product file writer.
    pub dp_writer: Arc<DpWriter>,
    /// `dpCat` — data-product catalog / downlink scheduler.
    pub dp_catalog: Arc<DpCatalog>,
    /// `comSplitter` — event-stream tee (`ComLoggerTee`).
    pub com_splitter: Arc<ComSplitter>,
    /// `comLog` — `.com` file logger on the teed event stream.
    pub com_logger: Arc<ComLogger>,
    /// `comDriver` — TCP client, present only with `-a`/`-p`.
    pub tcp_client: Option<Arc<TcpClient>>,
    /// The resolved data-directory paths (see [`DataPaths`]).
    pub paths: DataPaths,
}

impl RefTopology {
    /// C++ `setupTopology`: construct, set id bases, wire, configure, init
    /// queues, register commands, start tasks (+ the TCP driver threads
    /// when comms are configured).
    pub fn setup(config: &TopologyConfig) -> RefTopology {
        // ------------------------------------------------------------------
        // Phase 1: construct instances (C++ initComponents object part).
        // ------------------------------------------------------------------
        let posix_time = PosixTime::new("posixTime");
        let linux_timer = LinuxTimer::new("linuxTimer");
        let rate_group_driver = RateGroupDriver::new("rateGroupDriverComp");
        let rate_group_1 = ActiveRateGroup::new("rateGroup1Comp");
        let rate_group_2 = ActiveRateGroup::new("rateGroup2Comp");
        let rate_group_3 = ActiveRateGroup::new("rateGroup3Comp");
        let cmd_dispatcher = CmdDispatcher::new("cmdDisp");
        let event_manager = EventManager::new("events");
        let tlm_chan = TlmChan::new("tlmSend");
        let health = Health::new("health");
        let fatal_handler = FatalHandler::new("fatalHandler");
        let text_logger = PassiveTextLogger::new("textLogger");
        let buffer_manager = BufferManager::new("bufferManager");
        let com_queue = ComQueue::new("comQueue");
        let framer = FprimeFramer::new("framer");
        let deframer = FprimeDeframer::new("deframer");
        let frame_accumulator = FrameAccumulator::new("frameAccumulator");
        let router = FprimeRouter::new("fprimeRouter");
        let com_stub = ComStub::new("comStub");
        let signal_gen = SignalGen::new("signalGen");
        let cmd_sequencer = CmdSequencer::new("cmdSeq");
        let system_resources = SystemResources::new("systemResources");
        let prm_db = PrmDb::new("prmDb");
        let file_uplink = FileUplink::new("fileUplink");
        let file_downlink = FileDownlink::new("fileDownlink");
        let file_manager = FileManager::new("fileManager");
        let dp_buffer_manager = BufferManager::new("dpBufferManager");
        let dp_manager = DpManager::new("dpMgr");
        let dp_writer = DpWriter::new("dpWriter");
        let dp_catalog = DpCatalog::new("dpCat");
        let com_splitter = ComSplitter::new("comSplitter");
        let com_logger = ComLogger::new("comLog");
        let tcp_client = config.comms_enabled().map(|_| TcpClient::new("comDriver"));
        let paths = DataPaths::new(&config.resolved_data_dir());

        // ------------------------------------------------------------------
        // Phase 2: setBaseIds.
        // ------------------------------------------------------------------
        posix_time.base.set_id_base(POSIX_TIME_BASE_ID);
        linux_timer.base.set_id_base(LINUX_TIMER_BASE_ID);
        rate_group_driver
            .base
            .set_id_base(RATE_GROUP_DRIVER_BASE_ID);
        rate_group_1
            .active
            .queued
            .base
            .set_id_base(RATE_GROUP_1_BASE_ID);
        rate_group_2
            .active
            .queued
            .base
            .set_id_base(RATE_GROUP_2_BASE_ID);
        rate_group_3
            .active
            .queued
            .base
            .set_id_base(RATE_GROUP_3_BASE_ID);
        cmd_dispatcher
            .active
            .queued
            .base
            .set_id_base(CMD_DISPATCHER_BASE_ID);
        event_manager
            .active
            .queued
            .base
            .set_id_base(EVENT_MANAGER_BASE_ID);
        tlm_chan.active.queued.base.set_id_base(TLM_CHAN_BASE_ID);
        health.queued.base.set_id_base(HEALTH_BASE_ID);
        fatal_handler.base.set_id_base(FATAL_HANDLER_BASE_ID);
        text_logger.base.set_id_base(TEXT_LOGGER_BASE_ID);
        buffer_manager.base.set_id_base(BUFFER_MANAGER_BASE_ID);
        com_queue.active.queued.base.set_id_base(COM_QUEUE_BASE_ID);
        framer.base.set_id_base(FRAMER_BASE_ID);
        deframer.base.set_id_base(DEFRAMER_BASE_ID);
        frame_accumulator
            .base
            .set_id_base(FRAME_ACCUMULATOR_BASE_ID);
        router.base.set_id_base(ROUTER_BASE_ID);
        com_stub.base.set_id_base(COM_STUB_BASE_ID);
        signal_gen.queued.base.set_id_base(SIGNAL_GEN_BASE_ID);
        cmd_sequencer
            .active
            .queued
            .base
            .set_id_base(CMD_SEQUENCER_BASE_ID);
        system_resources.base.set_id_base(SYSTEM_RESOURCES_BASE_ID);
        prm_db.active.queued.base.set_id_base(PRM_DB_BASE_ID);
        file_uplink
            .active
            .queued
            .base
            .set_id_base(FILE_UPLINK_BASE_ID);
        file_downlink
            .active
            .queued
            .base
            .set_id_base(FILE_DOWNLINK_BASE_ID);
        file_manager
            .active
            .queued
            .base
            .set_id_base(FILE_MANAGER_BASE_ID);
        dp_buffer_manager
            .base
            .set_id_base(DP_BUFFER_MANAGER_BASE_ID);
        dp_manager
            .active
            .queued
            .base
            .set_id_base(DP_MANAGER_BASE_ID);
        dp_writer.active.queued.base.set_id_base(DP_WRITER_BASE_ID);
        dp_catalog
            .active
            .queued
            .base
            .set_id_base(DP_CATALOG_BASE_ID);
        com_splitter.base.set_id_base(COM_SPLITTER_BASE_ID);
        com_logger
            .active
            .queued
            .base
            .set_id_base(COM_LOGGER_BASE_ID);
        if let Some(tcp) = &tcp_client {
            tcp.base.set_id_base(COM_DRIVER_BASE_ID);
        }

        // ------------------------------------------------------------------
        // Phase 3: connectComponents.
        // ------------------------------------------------------------------

        // -- Command pattern (matched compCmdReg/compCmdSend indices) -------
        // 0: cmdDisp (self), 1: events, 2: health, 3: comQueue, 4: signalGen,
        // 5: fileDownlink, 6: fileManager, 7: prmDb, 8: cmdSeq, 9: dpMgr,
        // 10: dpWriter, 11: dpCat, 12: systemResources, 13: comLog.
        cmd_dispatcher
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(0));
        cmd_dispatcher.comp_cmd_send[0].connect_to(cmd_dispatcher.cmd_in(0));
        cmd_dispatcher
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(0));

        event_manager
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(1));
        cmd_dispatcher.comp_cmd_send[1].connect_to(event_manager.cmd_in(0));
        event_manager
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(1));

        health
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(2));
        cmd_dispatcher.comp_cmd_send[2].connect_to(health.cmd_in(0));
        health
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(2));

        com_queue
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(3));
        cmd_dispatcher.comp_cmd_send[3].connect_to(com_queue.cmd_in(0));
        com_queue
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(3));

        signal_gen
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(4));
        cmd_dispatcher.comp_cmd_send[4].connect_to(signal_gen.cmd_in(0));
        signal_gen
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(4));

        file_downlink
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(5));
        cmd_dispatcher.comp_cmd_send[5].connect_to(file_downlink.cmd_in(0));
        file_downlink
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(5));

        file_manager
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(6));
        cmd_dispatcher.comp_cmd_send[6].connect_to(file_manager.cmd_in(0));
        file_manager
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(6));

        prm_db
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(7));
        cmd_dispatcher.comp_cmd_send[7].connect_to(prm_db.cmd_in(0));
        prm_db
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(7));

        cmd_sequencer
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(8));
        cmd_dispatcher.comp_cmd_send[8].connect_to(cmd_sequencer.cmd_in(0));
        cmd_sequencer
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(8));

        dp_manager
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(9));
        cmd_dispatcher.comp_cmd_send[9].connect_to(dp_manager.cmd_in(0));
        dp_manager
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(9));

        dp_writer
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(10));
        cmd_dispatcher.comp_cmd_send[10].connect_to(dp_writer.cmd_in(0));
        dp_writer
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(10));

        dp_catalog
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(11));
        cmd_dispatcher.comp_cmd_send[11].connect_to(dp_catalog.cmd_in(0));
        dp_catalog
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(11));

        system_resources
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(12));
        cmd_dispatcher.comp_cmd_send[12].connect_to(system_resources.cmd_in(0));
        system_resources
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(12));

        com_logger
            .cmd
            .cmd_reg_out
            .connect_to(cmd_dispatcher.comp_cmd_reg_in(13));
        cmd_dispatcher.comp_cmd_send[13].connect_to(com_logger.cmd_in(0));
        com_logger
            .cmd
            .cmd_response_out
            .connect_to(cmd_dispatcher.comp_cmd_stat_in(13));

        // -- Event pattern: logOut -> events.LogRecv, textLogOut ->
        //    textLogger, timeGetOut -> posixTime (every EventGlue holder,
        //    including events' own logOut — C++ pattern-wires the self-loop).
        let event_glues = [
            &cmd_dispatcher.evt,
            &event_manager.evt,
            &tlm_chan.evt,
            &health.evt,
            &buffer_manager.evt,
            &com_queue.evt,
            &framer.evt,
            &deframer.evt,
            &frame_accumulator.evt,
            &router.evt,
            &rate_group_1.evt,
            &rate_group_2.evt,
            &rate_group_3.evt,
            &signal_gen.evt,
            &cmd_sequencer.evt,
            &system_resources.evt,
            &prm_db.evt,
            &file_uplink.evt,
            &file_downlink.evt,
            &file_manager.evt,
            &dp_buffer_manager.evt,
            &dp_manager.evt,
            &dp_writer.evt,
            &dp_catalog.evt,
            &com_logger.evt,
        ];
        for evt in event_glues {
            evt.log_out.connect_to(event_manager.log_recv_in(0));
            evt.text_log_out.connect_to(text_logger.text_logger_in(0));
            evt.time_out.connect_to(posix_time.time_get_port_in(0));
        }

        // -- Telemetry pattern: tlmOut -> tlmSend.TlmRecv.
        let tlm_glues = [
            &cmd_dispatcher.tlm,
            &event_manager.tlm,
            &health.tlm,
            &buffer_manager.tlm,
            &com_queue.tlm,
            &rate_group_1.tlm,
            &rate_group_2.tlm,
            &rate_group_3.tlm,
            &signal_gen.tlm,
            &cmd_sequencer.tlm,
            &system_resources.tlm,
            &file_uplink.tlm,
            &file_downlink.tlm,
            &file_manager.tlm,
            &dp_buffer_manager.tlm,
            &dp_manager.tlm,
            &dp_writer.tlm,
            &dp_catalog.tlm,
        ];
        for tlm in tlm_glues {
            tlm.tlm_out.connect_to(tlm_chan.tlm_recv_in(0));
        }

        // -- Health ping pattern (index map in the module header). ----------
        health.ping_send[0].connect_to(rate_group_1.ping_in(0));
        rate_group_1.ping_out.connect_to(health.ping_return_in(0));
        health.ping_send[1].connect_to(rate_group_2.ping_in(0));
        rate_group_2.ping_out.connect_to(health.ping_return_in(1));
        health.ping_send[2].connect_to(rate_group_3.ping_in(0));
        rate_group_3.ping_out.connect_to(health.ping_return_in(2));
        health.ping_send[3].connect_to(cmd_dispatcher.ping_in(0));
        cmd_dispatcher.ping_out.connect_to(health.ping_return_in(3));
        health.ping_send[4].connect_to(event_manager.ping_in(0));
        event_manager.ping_out.connect_to(health.ping_return_in(4));
        health.ping_send[5].connect_to(tlm_chan.ping_in(0));
        tlm_chan.ping_out.connect_to(health.ping_return_in(5));
        health.ping_send[6].connect_to(file_uplink.ping_in(0));
        file_uplink.ping_out.connect_to(health.ping_return_in(6));
        health.ping_send[7].connect_to(file_downlink.ping_in(0));
        file_downlink.ping_out.connect_to(health.ping_return_in(7));
        health.ping_send[8].connect_to(file_manager.ping_in(0));
        file_manager.ping_out.connect_to(health.ping_return_in(8));
        health.ping_send[9].connect_to(prm_db.ping_in(0));
        prm_db.ping_out.connect_to(health.ping_return_in(9));
        health.ping_send[10].connect_to(cmd_sequencer.ping_in(0));
        cmd_sequencer.ping_out.connect_to(health.ping_return_in(10));
        health.ping_send[11].connect_to(dp_catalog.ping_in(0));
        dp_catalog.ping_out.connect_to(health.ping_return_in(11));
        health.ping_send[12].connect_to(com_logger.ping_in(0));
        com_logger.ping_out.connect_to(health.ping_return_in(12));

        // -- Rate groups. ---------------------------------------------------
        linux_timer
            .cycle_out
            .connect_to(rate_group_driver.cycle_in(0));
        rate_group_driver.cycle_out[0].connect_to(rate_group_1.cycle_in(0));
        rate_group_driver.cycle_out[1].connect_to(rate_group_2.cycle_in(0));
        rate_group_driver.cycle_out[2].connect_to(rate_group_3.cycle_in(0));

        // RG1 (1 Hz): signalGen.schedIn, tlmSend.Run, cmdDisp.run,
        // comQueue.run, fileDownlink.Run, systemResources.run — the same
        // members the C++ Ref puts on rate group 1.
        rate_group_1.rate_group_member_out[0].connect_to(signal_gen.sched_in(0));
        rate_group_1.rate_group_member_out[1].connect_to(tlm_chan.run_in(0));
        rate_group_1.rate_group_member_out[2].connect_to(cmd_dispatcher.run_in(0));
        rate_group_1.rate_group_member_out[3].connect_to(com_queue.run_in(0));
        rate_group_1.rate_group_member_out[4].connect_to(file_downlink.run_in(0));
        rate_group_1.rate_group_member_out[5].connect_to(system_resources.run_in(0));
        // RG2 (0.5 Hz): events.run, cmdSeq.schedIn, fileManager.schedIn
        // (C++ drives the last two from rate group 2).
        rate_group_2.rate_group_member_out[0].connect_to(event_manager.run_in(0));
        rate_group_2.rate_group_member_out[1].connect_to(cmd_sequencer.sched_in(0));
        rate_group_2.rate_group_member_out[2].connect_to(file_manager.sched_in(0));
        // RG3 (0.25 Hz): health.Run + bufferManager.schedIn (C++ Ref wires
        // the comms BufferManager telemetry sweep on RG3 too) + the three
        // DataProducts telemetry sweeps.
        rate_group_3.rate_group_member_out[0].connect_to(health.run_in(0));
        rate_group_3.rate_group_member_out[1].connect_to(buffer_manager.sched_in(0));
        rate_group_3.rate_group_member_out[2].connect_to(dp_buffer_manager.sched_in(0));
        rate_group_3.rate_group_member_out[3].connect_to(dp_writer.sched_in(0));
        rate_group_3.rate_group_member_out[4].connect_to(dp_manager.sched_in(0));

        // -- Downlink chain (ComFprime.fpp `Downlink` + `ComStub` blocks). --
        // Event stream: events.PktSend -> comSplitter -> {comQueue, comLog}
        // (C++ `ComLoggerTee`: comSplitter.comOut -> comLog.comIn).
        event_manager.pkt_send.connect_to(com_splitter.com_in(0));
        com_splitter.com_out[COM_SPLITTER_DOWNLINK_PORT]
            .connect_to(com_queue.com_packet_queue_in(EVENTS_QUEUE_INDEX as i16));
        com_splitter.com_out[COM_SPLITTER_LOGGER_PORT].connect_to(com_logger.com_in(0));
        tlm_chan
            .pkt_send
            .connect_to(com_queue.com_packet_queue_in(TELEMETRY_QUEUE_INDEX as i16));
        // File downlink rides ComQueue's BUFFER queue (index 0 in the
        // buffer-port space, table entry COM_PORT_COUNT + 0).
        file_downlink
            .buffer_send_out
            .connect_to(com_queue.buffer_queue_in(FILE_BUFFER_QUEUE_PORT));
        com_queue.buffer_return_out[FILE_BUFFER_QUEUE_PORT as usize]
            .connect_to(file_downlink.buffer_return_in(0));
        com_queue.data_out.connect_to(framer.data_in(0));
        framer
            .data_return_out
            .connect_to(com_queue.data_return_in(0));
        framer
            .buffer_allocate
            .connect_to(buffer_manager.buffer_get_callee_in(0));
        framer
            .buffer_deallocate
            .connect_to(buffer_manager.buffer_send_in(0));
        framer.com_status_out.connect_to(com_queue.com_status_in(0));
        framer.data_out.connect_to(com_stub.data_in(0));
        com_stub
            .data_return_out
            .connect_to(framer.data_return_in(0));
        com_stub.com_status_out.connect_to(framer.com_status_in(0));

        // -- Uplink chain (ComFprime.fpp `Uplink` + `ComStub` blocks). ------
        com_stub.data_out.connect_to(frame_accumulator.data_in(0));
        frame_accumulator
            .data_return_out
            .connect_to(com_stub.data_return_in(0));
        frame_accumulator
            .buffer_allocate
            .connect_to(buffer_manager.buffer_get_callee_in(0));
        frame_accumulator
            .buffer_deallocate
            .connect_to(buffer_manager.buffer_send_in(0));
        frame_accumulator.data_out.connect_to(deframer.data_in(0));
        deframer
            .data_return_out
            .connect_to(frame_accumulator.data_return_in(0));
        deframer.data_out.connect_to(router.data_in(0));
        router
            .data_return_out
            .connect_to(deframer.data_return_in(0));
        // Router <-> dispatcher: seqCmdBuff/seqCmdStatus matched at index 0.
        router
            .command_out
            .connect_to(cmd_dispatcher.seq_cmd_buff_in(0));
        cmd_dispatcher.seq_cmd_status[0].connect_to(router.cmd_response_in(0));
        // Router <-> FileUplink (ComFprime `fileUplinkOut` /
        // `fileUplinkReturnIn`): the uplinked file buffer is owned by the
        // comms BufferManager and returns through the router.
        router.file_out.connect_to(file_uplink.buffer_send_in(0));
        file_uplink
            .buffer_send_out
            .connect_to(router.file_buffer_return_in(0));
        // router.unknownDataOut stays unconnected (C++ Ref leaves it so);
        // the router guards it with is_connected. fileUplink.fileAnnounce
        // is likewise unwired, as in the C++ FileHandling subtopology.

        // -- Command sequencer (Ref `ComCcsds_CdhCore` block). -------------
        // cmdSeq is the SECOND command source: seqCmdBuff/seqCmdStatus are
        // matched port arrays, so it must use the same index on both.
        cmd_sequencer
            .com_cmd_out
            .connect_to(cmd_dispatcher.seq_cmd_buff_in(1));
        cmd_dispatcher.seq_cmd_status[1].connect_to(cmd_sequencer.cmd_response_in(0));
        // cmdSeq.seqStartOut / seqDone stay unconnected (C++ Ref parity);
        // both are guarded by is_connected on every path a sequence run
        // can take from a command.

        // -- Data products (DataProducts.fpp `DataProducts` block). --------
        // Two producers, on consecutive dpMgr port indices (FPP auto-numbers
        // `X.productGetOut -> DataProducts.Subtopology.productGetIn`):
        // 0 signalGen, 1 fileManager (`GenerateDp` chunk containers).
        signal_gen
            .product_get_out
            .connect_to(dp_manager.product_get_in(0));
        signal_gen
            .product_send_out
            .connect_to(dp_manager.product_send_in(0));
        file_manager
            .product_get_out
            .connect_to(dp_manager.product_get_in(1));
        file_manager
            .product_send_out
            .connect_to(dp_manager.product_send_in(1));
        // dpMgr answers on the same index it was asked on, so every producer
        // index needs its own allocator/writer connection.
        for i in 0..2 {
            dp_manager.buffer_get_out[i].connect_to(dp_buffer_manager.buffer_get_callee_in(0));
            // C++ routes this through Svc.BufferAccumulator (not ported);
            // the ownership cycle is otherwise identical.
            dp_manager.product_send_out[i].connect_to(dp_writer.buffer_send_in(0));
        }
        dp_writer
            .dealloc_buffer_send_out
            .connect_to(dp_buffer_manager.buffer_send_in(0));
        dp_writer
            .dp_written_out
            .connect_to(dp_catalog.add_to_cat(0));
        // dpCat <-> fileDownlink (Ref `FileHandling_DataProducts` block).
        dp_catalog
            .file_out
            .connect_to(file_downlink.send_file_in(0));
        file_downlink.file_complete[0].connect_to(dp_catalog.file_done(0));

        // -- Parameters (`param connections instance FileHandling.prmDb`). -
        signal_gen.prm.prm_get_out.connect_to(prm_db.get_prm(0));
        signal_gen.prm.prm_set_out.connect_to(prm_db.set_prm(0));

        // -- Fatal chain. ---------------------------------------------------
        event_manager
            .fatal_announce
            .connect_to(fatal_handler.fatal_receive_in(0));

        // -- TCP driver (optional) + byte-stream shims. ---------------------
        if let Some(tcp) = &tcp_client {
            tcp.allocate_out
                .connect_to(buffer_manager.buffer_get_callee_in(0));
            tcp.deallocate_out
                .connect_to(buffer_manager.buffer_send_in(0));
            tcp.recv_out.connect(
                Arc::new(DrvToSvcDataShim {
                    target: com_stub.drv_receive_in(0),
                }),
                0,
            );
            tcp.ready_out.connect(
                Arc::new(DrvToSvcReadyShim {
                    target: com_stub.drv_connected(0),
                }),
                0,
            );
            com_stub.drv_send_out.connect(
                Arc::new(SvcToDrvSendShim {
                    target: tcp.send_in(0),
                }),
                0,
            );
            // Same framework trait on both sides — no shim needed.
            com_stub
                .drv_receive_return_out
                .connect_to(tcp.recv_return_in(0));
        }

        // ------------------------------------------------------------------
        // Phase 4: configComponents (must precede regCommands, C++ parity).
        // ------------------------------------------------------------------
        rate_group_driver.configure(&[Divider::new(1, 0), Divider::new(2, 0), Divider::new(4, 0)]);
        rate_group_1.configure([0; 10]);
        rate_group_2.configure([0; 10]);
        rate_group_3.configure([0; 10]);
        buffer_manager.setup(
            COMMS_BUFFER_MGR_ID,
            &[BufferBin {
                buffer_size: COMMS_BUFFER_SIZE,
                num_buffers: COMMS_BUFFER_COUNT,
            }],
        );
        frame_accumulator.configure(
            Box::new(FprimeFrameDetector::new()),
            FRAME_ACCUMULATOR_RING_SIZE,
        );
        let mut queue_table = QueueConfigurationTable::default();
        queue_table.entries[EVENTS_QUEUE_INDEX].depth = 200;
        queue_table.entries[EVENTS_QUEUE_INDEX].priority = 0;
        queue_table.entries[TELEMETRY_QUEUE_INDEX].depth = 500;
        queue_table.entries[TELEMETRY_QUEUE_INDEX].priority = 2;
        queue_table.entries[FILE_QUEUE_INDEX].depth = 100;
        queue_table.entries[FILE_QUEUE_INDEX].priority = 1;
        com_queue.configure(&queue_table);
        health.set_ping_entries(
            &[
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "rateGroup1Comp"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "rateGroup2Comp"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "rateGroup3Comp"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "cmdDisp"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "events"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "tlmSend"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "fileUplink"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "fileDownlink"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "fileManager"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "prmDb"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "cmdSeq"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "dpCat"),
                PingEntry::new(HEALTH_WARN_CYCLES, HEALTH_FATAL_CYCLES, "comLog"),
            ],
            HEALTH_WATCHDOG_CODE,
        );

        // -- Data directory and the components that write into it. ---------
        paths.create_directories();
        prm_db.configure(&paths.prm_db_file);
        // Commanded PRM_LOAD_FILE reads are confined to the data directory.
        prm_db.configure_load_sandbox(&paths.root);
        // C++ `FileHandling::fileUplink.configure("/tmp/uplink/")`.
        file_uplink.configure(&paths.uplink_dir);
        file_downlink.configure(
            FILE_DOWNLINK_COOLDOWN,
            FILE_DOWNLINK_CYCLE_TIME,
            FILE_DOWNLINK_QUEUE_DEPTH,
        );
        file_downlink.configure_sandbox(&paths.root);
        dp_buffer_manager.setup(
            DP_BUFFER_MGR_ID,
            &[BufferBin {
                buffer_size: DP_BUFFER_SIZE,
                num_buffers: DP_BUFFER_COUNT,
            }],
        );
        dp_writer.configure(&paths.dp_dir);
        dp_catalog.configure(
            &[FileNameString::from(paths.dp_dir.as_str())],
            &FileNameString::from(paths.dp_state_file.as_str()),
        );
        com_logger.init_log_file(
            &paths.com_log_prefix,
            COM_LOGGER_MAX_FILE_SIZE,
            COM_LOGGER_STORE_BUFFER_LENGTH,
        );
        // C++ `cmdSeq.allocateBuffer(0, mallocator, 5 * 1024)`; released in
        // teardown after every task has been joined.
        cmd_sequencer.allocate_buffer(CMD_SEQ_ALLOCATOR_ID, CMD_SEQ_BUFFER_BYTES);
        if let (Some(tcp), Some((hostname, port))) = (&tcp_client, config.comms_enabled()) {
            let status = tcp.configure(hostname, port, TCP_BUFFER_SIZE, true);
            // C++ parity: configure failures are init-time fatal.
            fprime_fw::fw_assert!(status == SocketIpStatus::Success, status as i32);
        }

        // ------------------------------------------------------------------
        // Phase 5: init message queues (C++ initComponents queue part).
        // ------------------------------------------------------------------
        rate_group_1.init(QUEUE_DEPTH);
        rate_group_2.init(QUEUE_DEPTH);
        rate_group_3.init(QUEUE_DEPTH);
        cmd_dispatcher.init(2 * QUEUE_DEPTH);
        event_manager.init(EVENTS_QUEUE_DEPTH);
        tlm_chan
            .active
            .queued
            .create_queue(QUEUE_DEPTH, fprime_svc::tlm_chan::QUEUE_MESSAGE_SIZE);
        health.init(HEALTH_QUEUE_DEPTH);
        com_queue
            .active
            .queued
            .create_queue(COM_QUEUE_DEPTH, fprime_svc::com_queue::MSG_SIZE);
        signal_gen.init(QUEUE_DEPTH);
        cmd_sequencer.init(QUEUE_DEPTH);
        prm_db.init(QUEUE_DEPTH);
        file_uplink.init(QUEUE_DEPTH);
        file_downlink.init(QUEUE_DEPTH);
        file_manager.init(QUEUE_DEPTH);
        dp_manager.init(QUEUE_DEPTH);
        dp_writer.init(QUEUE_DEPTH);
        dp_catalog.init(QUEUE_DEPTH);
        com_logger.init(COM_LOGGER_QUEUE_DEPTH);

        // ------------------------------------------------------------------
        // Phase 6: regCommands (after setBaseIds + config, before tasks —
        // registration is synchronous through the guarded compCmdReg port).
        // ------------------------------------------------------------------
        cmd_dispatcher.reg_commands();
        event_manager.reg_commands();
        health.reg_commands();
        com_queue.reg_commands();
        signal_gen.reg_commands();
        file_downlink.reg_commands();
        file_manager.reg_commands();
        prm_db.reg_commands();
        cmd_sequencer.reg_commands();
        dp_manager.reg_commands();
        dp_writer.reg_commands();
        dp_catalog.reg_commands();
        system_resources.reg_commands();
        com_logger.reg_commands();

        // ------------------------------------------------------------------
        // Phase 7a: readParameters — load the parameter file into prmDb
        // BEFORE any component reads a parameter (C++ FileHandling
        // `phase readParameters`). Runs on this thread, before tasks start.
        // ------------------------------------------------------------------
        prm_db.read_param_file();

        // ------------------------------------------------------------------
        // Phase 7b: loadParameters — every component with parameters pulls
        // its values through prmGetOut (falling back to the FPP defaults).
        // ------------------------------------------------------------------
        signal_gen.load_parameters();

        // ------------------------------------------------------------------
        // Phase 8: startTasks (+ hand-started driver threads, C++ parity).
        // ------------------------------------------------------------------
        rate_group_1
            .active
            .start(&rate_group_1, RG1_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        rate_group_2
            .active
            .start(&rate_group_2, RG2_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        rate_group_3
            .active
            .start(&rate_group_3, RG3_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        cmd_dispatcher.active.start(
            &cmd_dispatcher,
            CMD_DISP_PRIORITY,
            TASK_DEFAULT,
            TASK_DEFAULT,
        );
        event_manager
            .active
            .start(&event_manager, EVENTS_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        tlm_chan
            .active
            .start(&tlm_chan, TLM_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        com_queue
            .active
            .start(&com_queue, COM_QUEUE_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        file_uplink.active.start(
            &file_uplink,
            FILE_UPLINK_PRIORITY,
            TASK_DEFAULT,
            TASK_DEFAULT,
        );
        file_downlink.active.start(
            &file_downlink,
            FILE_DOWNLINK_PRIORITY,
            TASK_DEFAULT,
            TASK_DEFAULT,
        );
        file_manager.active.start(
            &file_manager,
            FILE_MANAGER_PRIORITY,
            TASK_DEFAULT,
            TASK_DEFAULT,
        );
        prm_db
            .active
            .start(&prm_db, PRM_DB_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        cmd_sequencer
            .active
            .start(&cmd_sequencer, CMD_SEQ_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        dp_manager
            .active
            .start(&dp_manager, DP_MANAGER_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        dp_writer
            .active
            .start(&dp_writer, DP_WRITER_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        dp_catalog
            .active
            .start(&dp_catalog, DP_CATALOG_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        com_logger
            .active
            .start(&com_logger, COM_LOGGER_PRIORITY, TASK_DEFAULT, TASK_DEFAULT);
        if let Some(tcp) = &tcp_client {
            tcp.start();
        }

        RefTopology {
            posix_time,
            linux_timer,
            rate_group_driver,
            rate_group_1,
            rate_group_2,
            rate_group_3,
            cmd_dispatcher,
            event_manager,
            tlm_chan,
            health,
            fatal_handler,
            text_logger,
            buffer_manager,
            com_queue,
            framer,
            deframer,
            frame_accumulator,
            router,
            com_stub,
            signal_gen,
            cmd_sequencer,
            system_resources,
            prm_db,
            file_uplink,
            file_downlink,
            file_manager,
            dp_buffer_manager,
            dp_manager,
            dp_writer,
            dp_catalog,
            com_splitter,
            com_logger,
            tcp_client,
            paths,
        }
    }

    /// C++ `startRateGroups(interval)`: BLOCKS the calling thread inside
    /// the timer loop until [`request_stop`](Self::request_stop).
    pub fn start_rate_loop(&self, interval: TimeInterval) {
        self.linux_timer.start_timer(interval);
    }

    /// C++ `stopRateGroups()`: makes a blocked
    /// [`start_rate_loop`](Self::start_rate_loop) return. Callable from any
    /// thread.
    pub fn request_stop(&self) {
        self.linux_timer.quit();
    }

    /// C++ `teardownTopology`: stopTasks (exit) in instance order, then
    /// freeThreads (join), then the hand-owned driver stop/join, then
    /// component cleanup. Call after the rate loop has returned.
    pub fn teardown(self) {
        // stopTasks: send EXIT to every active component first...
        self.rate_group_1.active.exit();
        self.rate_group_2.active.exit();
        self.rate_group_3.active.exit();
        self.cmd_dispatcher.active.exit();
        self.event_manager.active.exit();
        self.tlm_chan.active.exit();
        self.com_queue.active.exit();
        self.file_uplink.active.exit();
        self.file_downlink.active.exit();
        self.file_manager.active.exit();
        self.prm_db.active.exit();
        self.cmd_sequencer.active.exit();
        self.dp_manager.active.exit();
        self.dp_writer.active.exit();
        self.dp_catalog.active.exit();
        self.com_logger.active.exit();
        // ...freeThreads: then join them all, same order.
        let _ = self.rate_group_1.active.join();
        let _ = self.rate_group_2.active.join();
        let _ = self.rate_group_3.active.join();
        let _ = self.cmd_dispatcher.active.join();
        let _ = self.event_manager.active.join();
        let _ = self.tlm_chan.active.join();
        let _ = self.com_queue.active.join();
        let _ = self.file_uplink.active.join();
        let _ = self.file_downlink.active.join();
        let _ = self.file_manager.active.join();
        let _ = self.prm_db.active.join();
        let _ = self.cmd_sequencer.active.join();
        let _ = self.dp_manager.active.join();
        let _ = self.dp_writer.active.join();
        let _ = self.dp_catalog.active.join();
        let _ = self.com_logger.active.join();
        // Driver-owned threads are stopped/joined by hand AFTER the
        // framework tasks (C++ parity).
        if let Some(tcp) = &self.tcp_client {
            tcp.stop();
            let _ = tcp.join();
        }
        // Resource deallocation (C++ `cmdSeq.deallocateBuffer(mallocator)`)
        // — strictly after every task has been joined.
        self.cmd_sequencer.deallocate_buffer();
        // tearDownComponents phase (C++ DataProducts: dpCat.shutdown() then
        // dpBufferManager.cleanup()).
        self.dp_catalog.shutdown();
        self.dp_buffer_manager.cleanup();
        self.buffer_manager.cleanup();
        fw_log!("Ref topology torn down\n");
    }

    /// Test/support helper: one manual timer tick (drives the rate-group
    /// driver exactly like a 1 Hz cycle without the blocking loop).
    pub fn tick(&self) {
        self.linux_timer.tick();
    }

    /// Test/support helper: the `cycleIn` port of the rate-group driver,
    /// for driving cycles with an explicit `RawTime`.
    pub fn rate_group_cycle_in(&self) -> PortRef<dyn fprime_comp::CyclePort> {
        self.rate_group_driver.cycle_in(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every derived path hangs off the configured root with exactly one
    /// separator, whatever trailing slashes the CLI supplied.
    #[test]
    fn data_paths_join_the_root_exactly_once() {
        for root in ["/data/ref", "/data/ref/", "/data/ref///"] {
            let paths = DataPaths::new(root);
            assert_eq!(paths.root, "/data/ref");
            assert_eq!(paths.prm_db_file, "/data/ref/PrmDb.dat");
            assert_eq!(paths.uplink_dir, "/data/ref/uplink");
            assert_eq!(paths.dp_dir, "/data/ref/DpCat");
            assert_eq!(paths.dp_state_file, "/data/ref/DpCat/DpState.dat");
            assert_eq!(paths.com_log_prefix, "/data/ref/comlog");
        }
    }

    /// A root of `/` stays the filesystem root rather than collapsing to an
    /// empty prefix (which would hand components relative paths).
    #[test]
    fn root_directory_stays_absolute() {
        let paths = DataPaths::new("/");
        assert_eq!(paths.root, "/");
        assert_eq!(paths.prm_db_file, "/PrmDb.dat");
        assert_eq!(paths.dp_state_file, "/DpCat/DpState.dat");
    }

    /// With no `--data-dir`, the deployment works under a per-process
    /// directory in the system temp directory.
    #[test]
    fn default_data_dir_is_process_private() {
        let config = TopologyConfig::default();
        let resolved = config.resolved_data_dir();
        assert!(resolved.ends_with(&format!("fprime-ref-{}", std::process::id())));
        assert!(!resolved.ends_with('/'));
        // An explicit directory wins.
        let config = TopologyConfig {
            data_dir: Some("/tmp/somewhere".to_string()),
            ..TopologyConfig::default()
        };
        assert_eq!(config.resolved_data_dir(), "/tmp/somewhere");
    }

    /// Comms stay optional: without BOTH `-a` and `-p` the deployment runs
    /// standalone (C++ `hostname != nullptr && port != 0`).
    #[test]
    fn comms_require_both_hostname_and_port() {
        assert!(TopologyConfig::default().comms_enabled().is_none());
        let host_only = TopologyConfig {
            hostname: Some("127.0.0.1".to_string()),
            ..TopologyConfig::default()
        };
        assert!(host_only.comms_enabled().is_none());
        let port_only = TopologyConfig {
            port: 50000,
            ..TopologyConfig::default()
        };
        assert!(port_only.comms_enabled().is_none());
        let both = TopologyConfig {
            hostname: Some("127.0.0.1".to_string()),
            port: 50000,
            ..TopologyConfig::default()
        };
        assert_eq!(both.comms_enabled(), Some(("127.0.0.1", 50000)));
    }
}
