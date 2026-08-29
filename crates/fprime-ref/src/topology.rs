//! # RefTopology — the reference deployment topology
//!
//! Rust port of `TestDeploymentsProject/Ref/Top/RefTopology.cpp` plus the
//! `CdhCore` / `ComFprime` subtopologies it composes (analysis:
//! `docs/cpp-analysis/ref-topology.md`; comms wiring mirrors
//! `Svc/Subtopologies/ComFprime/ComFprime.fpp` exactly).
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
//! | `rateGroup1Comp`    | `Svc.ActiveRateGroup`  | `0x1000_1000` | Ref (main)    |
//! | `rateGroup2Comp`    | `Svc.ActiveRateGroup`  | `0x1000_2000` | Ref (main)    |
//! | `rateGroup3Comp`    | `Svc.ActiveRateGroup`  | `0x1000_3000` | Ref (main)    |
//! | `signalGen`         | `Ref.SignalGen`        | `0x1001_1000` | Ref (SG1 slot)|
//! | `posixTime`         | `Svc.PosixTime`        | `0x1002_0000` | Ref (main)    |
//! | `rateGroupDriverComp`| `Svc.RateGroupDriver` | `0x1002_1000` | Ref (main)    |
//! | `linuxTimer`        | `Svc.LinuxTimer`       | `0x1002_4000` | Ref (main)    |
//! | `comDriver`         | `Drv.TcpClient`        | `0x1002_5000` | Ref (main)    |
//!
//! ## Command-port index map (cmdDisp `compCmdReg`/`compCmdSend`, matched)
//!
//! 0 `cmdDisp` (self), 1 `events`, 2 `health`, 3 `comQueue`, 4 `signalGen`.
//!
//! ## Health ping index map (`pingSend[i]`/`pingReturn[i]`, WARN 3 FATAL 5)
//!
//! 0 `rateGroup1Comp`, 1 `rateGroup2Comp`, 2 `rateGroup3Comp`,
//! 3 `cmdDisp`, 4 `events`, 5 `tlmSend`.
//!
//! The comms chain is always wired internally (as the C++ Ref topology
//! wires ComFprime); only the TCP driver is optional (`-a`/`-p`). Without a
//! driver, `comStub`'s driver-side ports stay unconnected — nothing invokes
//! them because ComQueue starts WAITING and is only primed by a driver
//! `ready` — and downlink packets accumulate/overflow in ComQueue exactly
//! as C++ does when comms are down. The in-process integration tests wire a
//! loopback driver onto those same ports.

use std::sync::Arc;

use fprime_comp::PortRef;
use fprime_config::{FwIdType, FwSizeType, FwTaskPriorityType};
use fprime_fw::{TimeInterval, fw_log};
use fprime_os::task::TASK_DEFAULT;

use fprime_drv::socket_helper::SocketIpStatus;
use fprime_drv::tcp_client::TcpClient;
use fprime_svc::active_rate_group::ActiveRateGroup;
use fprime_svc::buffer_manager::{BufferBin, BufferManager};
use fprime_svc::cmd_dispatcher::CmdDispatcher;
use fprime_svc::com_queue::{ComQueue, QueueConfigurationTable};
use fprime_svc::com_stub::ComStub;
use fprime_svc::event_manager::EventManager;
use fprime_svc::fatal_handler::FatalHandler;
use fprime_svc::fprime_deframer::FprimeDeframer;
use fprime_svc::fprime_framer::FprimeFramer;
use fprime_svc::fprime_router::FprimeRouter;
use fprime_svc::frame_accumulator::{FprimeFrameDetector, FrameAccumulator};
use fprime_svc::health::{Health, PingEntry};
use fprime_svc::linux_timer::LinuxTimer;
use fprime_svc::passive_text_logger::PassiveTextLogger;
use fprime_svc::posix_time::PosixTime;
use fprime_svc::rate_group_driver::{Divider, RateGroupDriver};
use fprime_svc::tlm_chan::TlmChan;

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
/// `rateGroup1Comp` (1 Hz) base ID.
pub const RATE_GROUP_1_BASE_ID: FwIdType = 0x1000_1000;
/// `rateGroup2Comp` (0.5 Hz) base ID.
pub const RATE_GROUP_2_BASE_ID: FwIdType = 0x1000_2000;
/// `rateGroup3Comp` (0.25 Hz) base ID.
pub const RATE_GROUP_3_BASE_ID: FwIdType = 0x1000_3000;
/// `signalGen` base ID (the C++ SG1 slot).
pub const SIGNAL_GEN_BASE_ID: FwIdType = 0x1001_1000;
/// `posixTime` base ID.
pub const POSIX_TIME_BASE_ID: FwIdType = 0x1002_0000;
/// `rateGroupDriverComp` base ID.
pub const RATE_GROUP_DRIVER_BASE_ID: FwIdType = 0x1002_1000;
/// `linuxTimer` base ID.
pub const LINUX_TIMER_BASE_ID: FwIdType = 0x1002_4000;
/// `comDriver` (TcpClient) base ID.
pub const COM_DRIVER_BASE_ID: FwIdType = 0x1002_5000;

// ---------------------------------------------------------------------------
// Resource configuration (Ref `instances.fpp` / CdhCore / ComFprime values)
// ---------------------------------------------------------------------------

/// Default queue depth (Ref `Default.QUEUE_SIZE`).
const QUEUE_DEPTH: FwSizeType = 10;
/// EventManager queue depth (bursty producers).
const EVENTS_QUEUE_DEPTH: FwSizeType = 25;
/// Health queue depth (CdhCore `$health queue size 25`).
const HEALTH_QUEUE_DEPTH: FwSizeType = 25;
/// ComQueue message-queue depth (ComFprime `QueueSizes.comQueue`).
const COM_QUEUE_DEPTH: FwSizeType = 50;

/// Task priorities (best-effort no-ops on std; recorded for parity).
const RG1_PRIORITY: FwTaskPriorityType = 43;
const RG2_PRIORITY: FwTaskPriorityType = 42;
const RG3_PRIORITY: FwTaskPriorityType = 41;
const CMD_DISP_PRIORITY: FwTaskPriorityType = 35;
const COM_QUEUE_PRIORITY: FwTaskPriorityType = 29;
const EVENTS_PRIORITY: FwTaskPriorityType = 23;
const TLM_PRIORITY: FwTaskPriorityType = 22;

/// ComQueue downlink queues (ComFprime config): EVENTS depth 200 pri 0,
/// TELEMETRY depth 500 pri 2, FILE (buffer queue, unused here) depth 100
/// pri 1 — every entry needs depth > 0 (configure asserts).
const EVENTS_QUEUE_INDEX: usize = 0;
const TELEMETRY_QUEUE_INDEX: usize = 1;
const FILE_QUEUE_INDEX: usize = 2;

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
    /// `comDriver` — TCP client, present only with `-a`/`-p`.
    pub tcp_client: Option<Arc<TcpClient>>,
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
        let tcp_client = config.comms_enabled().map(|_| TcpClient::new("comDriver"));

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
        if let Some(tcp) = &tcp_client {
            tcp.base.set_id_base(COM_DRIVER_BASE_ID);
        }

        // ------------------------------------------------------------------
        // Phase 3: connectComponents.
        // ------------------------------------------------------------------

        // -- Command pattern (matched compCmdReg/compCmdSend indices) -------
        // 0: cmdDisp (self), 1: events, 2: health, 3: comQueue, 4: signalGen
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

        // -- Rate groups. ---------------------------------------------------
        linux_timer
            .cycle_out
            .connect_to(rate_group_driver.cycle_in(0));
        rate_group_driver.cycle_out[0].connect_to(rate_group_1.cycle_in(0));
        rate_group_driver.cycle_out[1].connect_to(rate_group_2.cycle_in(0));
        rate_group_driver.cycle_out[2].connect_to(rate_group_3.cycle_in(0));

        // RG1 (1 Hz): signalGen.schedIn, tlmSend.Run, cmdDisp.run,
        // comQueue.run.
        rate_group_1.rate_group_member_out[0].connect_to(signal_gen.sched_in(0));
        rate_group_1.rate_group_member_out[1].connect_to(tlm_chan.run_in(0));
        rate_group_1.rate_group_member_out[2].connect_to(cmd_dispatcher.run_in(0));
        rate_group_1.rate_group_member_out[3].connect_to(com_queue.run_in(0));
        // RG2 (0.5 Hz): events.run.
        rate_group_2.rate_group_member_out[0].connect_to(event_manager.run_in(0));
        // RG3 (0.25 Hz): health.Run + bufferManager.schedIn (C++ Ref wires
        // the comms BufferManager telemetry sweep on RG3 too).
        rate_group_3.rate_group_member_out[0].connect_to(health.run_in(0));
        rate_group_3.rate_group_member_out[1].connect_to(buffer_manager.sched_in(0));

        // -- Downlink chain (ComFprime.fpp `Downlink` + `ComStub` blocks). --
        event_manager
            .pkt_send
            .connect_to(com_queue.com_packet_queue_in(EVENTS_QUEUE_INDEX as i16));
        tlm_chan
            .pkt_send
            .connect_to(com_queue.com_packet_queue_in(TELEMETRY_QUEUE_INDEX as i16));
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
        // router.fileOut / unknownDataOut stay unconnected (no file uplink
        // in phase 1); the router guards both with is_connected.

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
            ],
            HEALTH_WATCHDOG_CODE,
        );
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

        // ------------------------------------------------------------------
        // Phase 6: regCommands (after setBaseIds + config, before tasks —
        // registration is synchronous through the guarded compCmdReg port).
        // ------------------------------------------------------------------
        cmd_dispatcher.reg_commands();
        event_manager.reg_commands();
        health.reg_commands();
        com_queue.reg_commands();
        signal_gen.reg_commands();

        // (Phase 7, loadParameters: no parameter components in this
        // deployment.)

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
            tcp_client,
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
        // ...freeThreads: then join them all, same order.
        let _ = self.rate_group_1.active.join();
        let _ = self.rate_group_2.active.join();
        let _ = self.rate_group_3.active.join();
        let _ = self.cmd_dispatcher.active.join();
        let _ = self.event_manager.active.join();
        let _ = self.tlm_chan.active.join();
        let _ = self.com_queue.active.join();
        // Driver-owned threads are stopped/joined by hand AFTER the
        // framework tasks (C++ parity).
        if let Some(tcp) = &self.tcp_client {
            tcp.stop();
            let _ = tcp.join();
        }
        // tearDownComponents phase.
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
