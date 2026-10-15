//! A `#![no_std]` + `alloc` deployment built from `fprime-svc` components
//! (ROADMAP "no_std beyond the core").
//!
//! This example is built as a library (`crate-type = ["lib"]` in
//! Cargo.toml), so it needs no `main`, allocator or panic handler of its
//! own. Every `--all-targets` build compiles it, and the bare-metal check
//! builds it for a target that has no `std` at all:
//!
//! ```text
//! cargo build -p fprime-svc --no-default-features --features spin \
//!     --target thumbv7em-none-eabihf --example no_std_deployment
//! ```
//!
//! [`Deployment`] is a small file-free F Prime topology — the CdhCore and
//! ComFprime parts of the Ref, driven by a superloop instead of tasks:
//!
//! ```text
//! tick -> rateGroupDriver -> rateGroup -> { tlmSend.Run, cmdDisp.Run, comQueue.run }
//! uplink:   radio -> comStub -> frameAccumulator -> deframer -> router -> cmdDisp
//! commands: cmdDisp <-> { cmdDisp, events, comQueue } (CmdReg / Cmd / CmdStatus)
//! events:   every component -> events.LogRecv -> comQueue[0]
//! tlm:      every component -> tlmSend.TlmRecv -> comQueue[1]
//! downlink: comQueue -> framer -> comStub -> radio
//! time:     every component -> posixTime (Platform clock, zero without one)
//! ```
//!
//! The active components (`CmdDispatcher`, `EventManager`, `TlmChan`,
//! `ComQueue`) are never started: [`Deployment::step`] drains their queues
//! non-blockingly, the way a bare-metal superloop runs F Prime's
//! `FPRIME_USE_BAREMETAL_SCHEDULER` deployments. Nothing here needs a
//! `fprime_os::platform::Platform`; with one registered, `posixTime` and
//! the rate group's cycle timing read its clock.

#![no_std]

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec::Vec;
use fprime_comp::fw::sync::{Arc, Mutex};
use fprime_comp::os::RawTime;
use fprime_comp::os::queue::BlockingType;
use fprime_comp::{ComponentDispatch, MsgDispatchStatus, QueuedBase};
use fprime_config::{FwIndexType, FwSizeType};
use fprime_fw::{Buffer, Endianness, LinearBuffer, SerBuf};
use fprime_svc::buffer_manager::{BufferBin, BufferManager};
use fprime_svc::cmd_dispatcher::CmdDispatcher;
use fprime_svc::com_queue::{ComQueue, MSG_SIZE as COM_QUEUE_MSG_SIZE, QueueConfigurationTable};
use fprime_svc::com_stub::{ByteStreamSendPort, ByteStreamStatus, ComStub};
use fprime_svc::event_manager::EventManager;
use fprime_svc::fprime_deframer::FprimeDeframer;
use fprime_svc::fprime_framer::{FprimeFramer, START_WORD};
use fprime_svc::fprime_router::FprimeRouter;
use fprime_svc::frame_accumulator::{FprimeFrameDetector, FrameAccumulator};
use fprime_svc::passive_rate_group::PassiveRateGroup;
use fprime_svc::posix_time::PosixTime;
use fprime_svc::rate_group_driver::{Divider, RateGroupDriver};
use fprime_svc::tlm_chan::{QUEUE_MESSAGE_SIZE as TLM_CHAN_MSG_SIZE, TlmChan};
use fprime_utils::Hash;

/// Base id of `cmdDisp` (the Ref's).
pub const CMD_DISP_BASE_ID: u32 = 0x0100_0000;
/// Base id of `events`.
pub const EVENTS_BASE_ID: u32 = 0x0100_1000;
/// Base id of `tlmSend`.
pub const TLM_SEND_BASE_ID: u32 = 0x0100_2000;
/// Base id of `comQueue`.
pub const COM_QUEUE_BASE_ID: u32 = 0x0200_0000;
/// Base id of `rateGroup`.
pub const RATE_GROUP_BASE_ID: u32 = 0x0300_0000;

const QUEUE_DEPTH: FwSizeType = 32;
const COMMS_BUFFER_SIZE: FwSizeType = 2048;
const COMMS_BUFFER_COUNT: u16 = 8;
const EVENTS_QUEUE_INDEX: usize = 0;
const TELEMETRY_QUEUE_INDEX: usize = 1;

/// The byte-stream driver: records every downlinked frame. A real target
/// would write them to a UART or radio.
#[derive(Default)]
pub struct Radio {
    frames: Mutex<Vec<Vec<u8>>>,
}

impl ByteStreamSendPort for Radio {
    fn invoke(&self, _port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus {
        self.frames.lock().push(buffer.data().to_vec());
        ByteStreamStatus::OpOk
    }
}

/// The file-free deployment.
pub struct Deployment {
    /// `rateGroupDriver`.
    pub rate_group_driver: Arc<RateGroupDriver>,
    /// `rateGroup` (passive).
    pub rate_group: Arc<PassiveRateGroup>,
    /// `cmdDisp`.
    pub cmd_disp: Arc<CmdDispatcher>,
    /// `events`.
    pub events: Arc<EventManager>,
    /// `tlmSend`.
    pub tlm_send: Arc<TlmChan>,
    /// `comQueue`.
    pub com_queue: Arc<ComQueue>,
    /// `framer`.
    pub framer: Arc<FprimeFramer>,
    /// `deframer`.
    pub deframer: Arc<FprimeDeframer>,
    /// `frameAccumulator`.
    pub frame_accumulator: Arc<FrameAccumulator>,
    /// `fprimeRouter`.
    pub router: Arc<FprimeRouter>,
    /// `comStub`.
    pub com_stub: Arc<ComStub>,
    /// `commsBufferManager`.
    pub buffer_manager: Arc<BufferManager>,
    /// `posixTime`.
    pub posix_time: Arc<PosixTime>,
    /// The driver.
    pub radio: Arc<Radio>,
}

impl Deployment {
    /// Construct, wire, configure and register commands (the Ref's
    /// `setup` phases, without starting any task).
    pub fn new() -> Self {
        let d = Self {
            rate_group_driver: RateGroupDriver::new("rateGroupDriver"),
            rate_group: PassiveRateGroup::new("rateGroup"),
            cmd_disp: CmdDispatcher::new("cmdDisp"),
            events: EventManager::new("events"),
            tlm_send: TlmChan::new("tlmSend"),
            com_queue: ComQueue::new("comQueue"),
            framer: FprimeFramer::new("framer"),
            deframer: FprimeDeframer::new("deframer"),
            frame_accumulator: FrameAccumulator::new("frameAccumulator"),
            router: FprimeRouter::new("fprimeRouter"),
            com_stub: ComStub::new("comStub"),
            buffer_manager: BufferManager::new("commsBufferManager"),
            posix_time: PosixTime::new("posixTime"),
            radio: Arc::new(Radio::default()),
        };
        d.cmd_disp.active.queued.base.set_id_base(CMD_DISP_BASE_ID);
        d.events.active.queued.base.set_id_base(EVENTS_BASE_ID);
        d.tlm_send.active.queued.base.set_id_base(TLM_SEND_BASE_ID);
        d.com_queue
            .active
            .queued
            .base
            .set_id_base(COM_QUEUE_BASE_ID);
        d.rate_group.base.set_id_base(RATE_GROUP_BASE_ID);
        d.wire();
        d.configure();
        d
    }

    fn wire(&self) {
        // Rate groups.
        self.rate_group_driver.cycle_out[0].connect_to(self.rate_group.cycle_in(0));
        self.rate_group.rate_group_member_out[0].connect_to(self.tlm_send.run_in(0));
        self.rate_group.rate_group_member_out[1].connect_to(self.cmd_disp.run_in(0));
        self.rate_group.rate_group_member_out[2].connect_to(self.com_queue.run_in(0));

        // Commands: each commandable component on its own dispatcher port.
        let commandable = [
            (&self.cmd_disp.cmd, self.cmd_disp.cmd_in(0)),
            (&self.events.cmd, self.events.cmd_in(0)),
            (&self.com_queue.cmd, self.com_queue.cmd_in(0)),
            (&self.rate_group.cmd, self.rate_group.cmd_in(0)),
        ];
        for (port, (glue, cmd_in)) in commandable.into_iter().enumerate() {
            let port = port as FwIndexType;
            glue.cmd_reg_out
                .connect_to(self.cmd_disp.comp_cmd_reg_in(port));
            self.cmd_disp.comp_cmd_send[port as usize].connect_to(cmd_in);
            glue.cmd_response_out
                .connect_to(self.cmd_disp.comp_cmd_stat_in(port));
        }

        // Events, telemetry and time.
        for evt in [
            &self.cmd_disp.evt,
            &self.events.evt,
            &self.tlm_send.evt,
            &self.com_queue.evt,
            &self.framer.evt,
            &self.deframer.evt,
            &self.frame_accumulator.evt,
            &self.router.evt,
            &self.buffer_manager.evt,
        ] {
            evt.log_out.connect_to(self.events.log_recv_in(0));
            evt.time_out.connect_to(self.posix_time.time_get_port_in(0));
        }
        for tlm in [
            &self.cmd_disp.tlm,
            &self.events.tlm,
            &self.com_queue.tlm,
            &self.rate_group.tlm,
            &self.buffer_manager.tlm,
        ] {
            tlm.tlm_out.connect_to(self.tlm_send.tlm_recv_in(0));
        }
        self.rate_group
            .time_out
            .connect_to(self.posix_time.time_get_port_in(0));

        // Downlink: events and telemetry -> comQueue -> framer -> comStub.
        self.events.pkt_send.connect_to(
            self.com_queue
                .com_packet_queue_in(EVENTS_QUEUE_INDEX as FwIndexType),
        );
        self.tlm_send.pkt_send.connect_to(
            self.com_queue
                .com_packet_queue_in(TELEMETRY_QUEUE_INDEX as FwIndexType),
        );
        self.com_queue.data_out.connect_to(self.framer.data_in(0));
        self.framer
            .data_return_out
            .connect_to(self.com_queue.data_return_in(0));
        self.framer
            .buffer_allocate
            .connect_to(self.buffer_manager.buffer_get_callee_in(0));
        self.framer
            .buffer_deallocate
            .connect_to(self.buffer_manager.buffer_send_in(0));
        self.framer
            .com_status_out
            .connect_to(self.com_queue.com_status_in(0));
        self.framer.data_out.connect_to(self.com_stub.data_in(0));
        self.com_stub
            .data_return_out
            .connect_to(self.framer.data_return_in(0));
        self.com_stub
            .com_status_out
            .connect_to(self.framer.com_status_in(0));
        self.com_stub.drv_send_out.connect(self.radio.clone(), 0);

        // Uplink: comStub -> frameAccumulator -> deframer -> router -> cmdDisp.
        self.com_stub
            .data_out
            .connect_to(self.frame_accumulator.data_in(0));
        self.frame_accumulator
            .data_return_out
            .connect_to(self.com_stub.data_return_in(0));
        self.frame_accumulator
            .buffer_allocate
            .connect_to(self.buffer_manager.buffer_get_callee_in(0));
        self.frame_accumulator
            .buffer_deallocate
            .connect_to(self.buffer_manager.buffer_send_in(0));
        self.frame_accumulator
            .data_out
            .connect_to(self.deframer.data_in(0));
        self.deframer
            .data_return_out
            .connect_to(self.frame_accumulator.data_return_in(0));
        self.deframer.data_out.connect_to(self.router.data_in(0));
        self.router
            .data_return_out
            .connect_to(self.deframer.data_return_in(0));
        self.router
            .command_out
            .connect_to(self.cmd_disp.seq_cmd_buff_in(0));
        self.cmd_disp.seq_cmd_status[0].connect_to(self.router.cmd_response_in(0));
        // Received driver buffers go back to the pool they came from.
        self.com_stub
            .drv_receive_return_out
            .connect_to(self.buffer_manager.buffer_send_in(0));
    }

    fn configure(&self) {
        self.rate_group_driver.configure(&[
            Divider::new(1, 0),
            Divider::new(0, 0),
            Divider::new(0, 0),
        ]);
        self.rate_group.configure([0; 10]);
        self.buffer_manager.setup(
            0,
            &[BufferBin {
                buffer_size: COMMS_BUFFER_SIZE,
                num_buffers: COMMS_BUFFER_COUNT,
            }],
        );
        self.frame_accumulator
            .configure(Box::new(FprimeFrameDetector::new()), 2048);
        let mut table = QueueConfigurationTable::default();
        for entry in &mut table.entries {
            entry.depth = 4;
        }
        table.entries[EVENTS_QUEUE_INDEX].depth = 16;
        table.entries[EVENTS_QUEUE_INDEX].priority = 0;
        table.entries[TELEMETRY_QUEUE_INDEX].depth = 16;
        table.entries[TELEMETRY_QUEUE_INDEX].priority = 1;
        self.com_queue.configure(&table);

        self.cmd_disp.init(QUEUE_DEPTH);
        self.events.init(QUEUE_DEPTH);
        self.tlm_send
            .active
            .queued
            .create_queue(QUEUE_DEPTH, TLM_CHAN_MSG_SIZE);
        self.com_queue
            .active
            .queued
            .create_queue(QUEUE_DEPTH, COM_QUEUE_MSG_SIZE);

        self.cmd_disp.reg_commands();
        self.events.reg_commands();
        self.com_queue.reg_commands();
        self.rate_group.reg_commands();

        // The driver is up: comStub reports ready, which lets comQueue send.
        let ready = self.com_stub.drv_connected(0);
        ready.target.invoke(ready.port_num);
    }

    /// Dispatch every queued message of every active component, until all
    /// queues are empty (handlers enqueue into each other).
    pub fn drain(&self) {
        fn drain_one<C: ComponentDispatch>(queued: &QueuedBase, comp: &C) -> bool {
            let mut any = false;
            while queued.do_dispatch(comp, BlockingType::NonBlocking) == MsgDispatchStatus::Ok {
                any = true;
            }
            any
        }
        loop {
            let mut any = drain_one(&self.cmd_disp.active.queued, &*self.cmd_disp);
            any |= drain_one(&self.events.active.queued, &*self.events);
            any |= drain_one(&self.tlm_send.active.queued, &*self.tlm_send);
            any |= drain_one(&self.com_queue.active.queued, &*self.com_queue);
            if !any {
                return;
            }
        }
    }

    /// One superloop pass: a rate-group tick, then drain the queues.
    pub fn step(&self) {
        let tick = self.rate_group_driver.cycle_in(0);
        tick.target.invoke(tick.port_num, &RawTime::new());
        self.drain();
    }

    /// Hand bytes read from the wire to `comStub`, in a pool buffer (what a
    /// driver's receive interrupt or poll does).
    pub fn uplink(&self, bytes: &[u8]) {
        let get = self.buffer_manager.buffer_get_callee_in(0);
        let mut buffer = get.target.invoke(get.port_num, bytes.len() as FwSizeType);
        buffer.data_mut()[..bytes.len()].copy_from_slice(bytes);
        buffer.set_size(bytes.len());
        let recv = self.com_stub.drv_receive_in(0);
        recv.target
            .invoke(recv.port_num, buffer, ByteStreamStatus::OpOk);
    }

    /// Take the frames downlinked so far.
    pub fn take_downlink(&self) -> Vec<Vec<u8>> {
        core::mem::take(&mut *self.radio.frames.lock())
    }
}

impl Default for Deployment {
    fn default() -> Self {
        Self::new()
    }
}

/// An F Prime uplink frame (`FprimeFramer` format: start word, payload
/// size, payload, CRC-32) carrying a command packet.
pub fn command_frame(opcode: u32) -> Vec<u8> {
    let mut payload = LinearBuffer::<6>::new();
    let _ = payload.serialize_u16(0, Endianness::Big); // FW_PACKET_COMMAND
    let _ = payload.serialize_u32(opcode, Endianness::Big);
    let payload = payload.as_slice();
    let mut frame = Vec::new();
    frame.extend_from_slice(&START_WORD.to_be_bytes());
    frame.extend_from_slice(&(payload.len() as u32).to_be_bytes());
    frame.extend_from_slice(payload);
    let mut hash = Hash::new();
    hash.update(&frame);
    frame.extend_from_slice(&hash.finalize().to_be_bytes());
    frame
}

/// A downlinked packet: `(packet type, event or channel id)` of each
/// frame's first record (`FW_PACKET_TELEM` = 1, `FW_PACKET_LOG` = 2).
pub fn packet_ids(frames: &[Vec<u8>]) -> Vec<(u16, u32)> {
    frames
        .iter()
        .filter(|f| f.len() >= 8 + 6 + 4 && f[..4] == START_WORD.to_be_bytes())
        .map(|f| {
            (
                u16::from_be_bytes([f[8], f[9]]),
                u32::from_be_bytes([f[10], f[11], f[12], f[13]]),
            )
        })
        .collect()
}

/// A short scenario: one tick, a `CMD_NO_OP` uplinked and dispatched,
/// another tick to downlink the telemetry; returns what went down.
pub fn run_scenario() -> Vec<(u16, u32)> {
    let deployment = Deployment::new();
    deployment.step();
    deployment.uplink(&command_frame(
        CMD_DISP_BASE_ID + CmdDispatcher::OPCODE_CMD_NO_OP,
    ));
    deployment.drain();
    deployment.step();
    packet_ids(&deployment.take_downlink())
}
