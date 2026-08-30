//! Shared in-process harness for the `fprime-ref` integration tests.
//!
//! The topology runs WITHOUT TCP: a loopback driver implements the
//! `fprime_svc::com_stub` byte-stream traits, capturing downlink frames and
//! letting a test inject uplink bytes on the same ports the real
//! `Drv::TcpClient` would drive. Rate groups are ticked manually through
//! the rate-group driver, so nothing depends on wall-clock timing.
//!
//! Wire-format ground truth exercised end to end:
//! frame  = `[0xdeadbeef u32][length u32][payload][crc32 u32]` (BE),
//! command payload = `[0x0000 u16][opcode u32][args]`,
//! file    payload = `[0x0003 u16][file packet]`,
//! log packet      = `[0x0002 u16][id u32][time 11B][raw args]`,
//! tlm packet      = `[0x0001 u16]` + N x `[id u32][time 11B][raw value]`.

// Each integration-test binary uses a subset of this module.
#![allow(dead_code)]

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use fprime_config::FwIndexType;
use fprime_fw::Buffer;
use fprime_os::RawTime;
use fprime_ref::topology::{RefTopology, TopologyConfig};
use fprime_svc::com_stub::{ByteStreamSendPort, ByteStreamStatus};
use fprime_utils::Hash;

/// Bound on every wait in the integration tests. Generous on purpose: the
/// suite runs several topologies (each with a dozen threads) in parallel,
/// so the deadline exists to fail a broken test, not to time anything.
pub const DEADLINE: Duration = Duration::from_secs(30);

/// Period between manual rate-group cycles in [`Harness::tick_until`].
pub const TICK_PERIOD: Duration = Duration::from_millis(20);

/// A private data directory per topology instance: every file-writing
/// component (`prmDb`, `dpWriter`, `comLog`, `fileUplink`) works under it,
/// so concurrently running tests never share a file. Kept SHORT because
/// command string arguments are `Fw::CmdStringArg` (40 bytes).
pub fn scratch_data_dir() -> String {
    static COUNTER: AtomicU32 = AtomicU32::new(0);
    let mut path = std::env::temp_dir();
    path.push(format!(
        "fpref{}_{}",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::SeqCst)
    ));
    path.to_string_lossy().into_owned()
}

// ---------------------------------------------------------------------------
// LoopbackDriver — the com_stub-facing byte-stream driver stub
// ---------------------------------------------------------------------------

/// Captures every frame ComStub sends and accepts returned receive
/// buffers. The test injects received bytes by invoking
/// `com_stub.drv_receive_in` directly (the delivery the driver would make).
#[derive(Default)]
pub struct LoopbackDriver {
    /// Downlink frames, one `Vec<u8>` per ComStub send.
    pub sent_frames: Mutex<Vec<Vec<u8>>>,
    /// Receive buffers returned to the driver (uplink ownership chain).
    pub returned_buffers: Mutex<Vec<Vec<u8>>>,
}

impl ByteStreamSendPort for LoopbackDriver {
    fn invoke(&self, _port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus {
        self.sent_frames
            .lock()
            .unwrap()
            .push(buffer.data().to_vec());
        ByteStreamStatus::OpOk
    }
}

impl fprime_comp::BufferSendPort for LoopbackDriver {
    fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
        self.returned_buffers
            .lock()
            .unwrap()
            .push(buffer.data().to_vec());
    }
}

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// A live topology plus its loopback driver.
pub struct Harness {
    /// The deployment under test.
    pub topology: RefTopology,
    /// The captured driver side of `comStub`.
    pub loopback: Arc<LoopbackDriver>,
}

impl Harness {
    /// Build the no-comms topology in a private data directory, wire the
    /// loopback onto ComStub's driver-side ports, neutralize the
    /// FatalHandler exit action (a test must never `exit(1)` the whole
    /// binary), and prime the comStatus chain via the driver-ready signal
    /// (ComQueue starts WAITING).
    pub fn up() -> Harness {
        Harness::up_in(&scratch_data_dir())
    }

    /// [`Harness::up`] with an explicit data directory.
    pub fn up_in(data_dir: &str) -> Harness {
        let topology = RefTopology::setup(&TopologyConfig {
            data_dir: Some(data_dir.to_string()),
            ..TopologyConfig::default()
        });
        topology
            .fatal_handler
            .set_exit_action(Box::new(|id| panic!("unexpected FATAL 0x{id:x}")));
        let loopback = Arc::new(LoopbackDriver::default());
        topology.com_stub.drv_send_out.connect(loopback.clone(), 0);
        topology
            .com_stub
            .drv_receive_return_out
            .connect(loopback.clone(), 0);
        // LoopbackDriver "connects": prime the comStatus chain.
        let ready = topology.com_stub.drv_connected(0);
        ready.target.invoke(ready.port_num);
        Harness { topology, loopback }
    }

    /// The deployment data directory.
    pub fn data_dir(&self) -> &str {
        &self.topology.paths.root
    }

    /// Inject bytes as if the driver had received them off the wire.
    pub fn inject(&self, bytes: &[u8]) {
        let mut buffer = Buffer::allocate(bytes.len());
        buffer.data_mut().copy_from_slice(bytes);
        buffer.set_size(bytes.len());
        let recv = self.topology.com_stub.drv_receive_in(0);
        recv.target
            .invoke(recv.port_num, buffer, ByteStreamStatus::OpOk);
    }

    /// One manual 1 Hz cycle into the rate-group driver.
    pub fn tick(&self) {
        let mut now = RawTime::new();
        let _ = now.now();
        let cycle = self.topology.rate_group_cycle_in();
        cycle.target.invoke(cycle.port_num, &now);
    }

    /// Every downlinked frame captured so far.
    pub fn frames(&self) -> Vec<Vec<u8>> {
        self.loopback.sent_frames.lock().unwrap().clone()
    }

    /// Snapshot of every LOG-packet (id, arg-bytes) pair downlinked so far.
    pub fn log_events(&self) -> Vec<(u32, Vec<u8>)> {
        self.frames()
            .iter()
            .filter_map(|frame| parse_log_packet(&deframe(frame)))
            .collect()
    }

    /// Whether an event id has been downlinked.
    pub fn saw_event(&self, id: u32) -> bool {
        self.log_events().iter().any(|(seen, _)| *seen == id)
    }

    /// The argument bytes of the first occurrence of an event id.
    pub fn event_args(&self, id: u32) -> Option<Vec<u8>> {
        self.log_events()
            .into_iter()
            .find(|(seen, _)| *seen == id)
            .map(|(_, args)| args)
    }

    /// Poll until `cond` holds, ticking the rate groups each round so that
    /// queued components drain. Returns false on timeout.
    ///
    /// The tick period is deliberately not zero: cycles drive fan-outs
    /// (rate-group members, health pings) whose async input ports assert on
    /// a full queue, exactly as the C++ ones do, so a tick storm faster
    /// than the components drain is a self-inflicted overrun rather than a
    /// test of anything. 20 ms is still 50x the deployment's real 1 Hz.
    pub fn tick_until(&self, mut cond: impl FnMut() -> bool) -> bool {
        let end = Instant::now() + DEADLINE;
        loop {
            if cond() {
                return true;
            }
            if Instant::now() >= end {
                return false;
            }
            self.tick();
            std::thread::sleep(TICK_PERIOD);
        }
    }
}

/// Poll `cond` every 10 ms until it holds or `deadline` elapses.
pub fn wait_until(deadline: Duration, mut cond: impl FnMut() -> bool) -> bool {
    let end = Instant::now() + deadline;
    loop {
        if cond() {
            return true;
        }
        if Instant::now() >= end {
            return false;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

// ---------------------------------------------------------------------------
// Literal frame building / parsing
// ---------------------------------------------------------------------------

/// `[0xdeadbeef][len][payload][crc32]`, CRC over header + payload.
pub fn build_frame(payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(payload.len() + 12);
    frame.extend_from_slice(&0xdead_beef_u32.to_be_bytes());
    frame.extend_from_slice(&(payload.len() as u32).to_be_bytes());
    frame.extend_from_slice(payload);
    let crc = Hash::hash_u32(&frame);
    frame.extend_from_slice(&crc.to_be_bytes());
    frame
}

/// Command payload: `[descriptor 0x0000 u16][opcode u32][args]`.
pub fn build_command_frame(opcode: u32, args: &[u8]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(6 + args.len());
    payload.extend_from_slice(&0x0000_u16.to_be_bytes());
    payload.extend_from_slice(&opcode.to_be_bytes());
    payload.extend_from_slice(args);
    build_frame(&payload)
}

/// File payload: `[descriptor 0x0003 u16][file packet bytes]`.
pub fn build_file_frame(file_packet: &[u8]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(2 + file_packet.len());
    payload.extend_from_slice(&0x0003_u16.to_be_bytes());
    payload.extend_from_slice(file_packet);
    build_frame(&payload)
}

/// A `Fw::CmdStringArg` command argument: `[u16 len][bytes]`.
pub fn cmd_string_arg(value: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(2 + value.len());
    out.extend_from_slice(&(value.len() as u16).to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    out
}

/// Literal frame parse: validate start word, length, and CRC; return the
/// payload.
pub fn deframe(frame: &[u8]) -> Vec<u8> {
    assert!(frame.len() >= 12, "frame too short: {}", frame.len());
    assert_eq!(&frame[0..4], &0xdead_beef_u32.to_be_bytes(), "start word");
    let len = u32::from_be_bytes([frame[4], frame[5], frame[6], frame[7]]) as usize;
    assert_eq!(frame.len(), len + 12, "frame length field");
    let crc = u32::from_be_bytes([
        frame[len + 8],
        frame[len + 9],
        frame[len + 10],
        frame[len + 11],
    ]);
    assert_eq!(crc, Hash::hash_u32(&frame[..len + 8]), "frame CRC");
    frame[8..8 + len].to_vec()
}

/// LOG packet: `[0x0002 u16][id u32][time 11B][raw args]` -> (id, args).
pub fn parse_log_packet(payload: &[u8]) -> Option<(u32, Vec<u8>)> {
    if payload.len() < 17 || payload[0..2] != [0x00, 0x02] {
        return None;
    }
    let id = u32::from_be_bytes([payload[2], payload[3], payload[4], payload[5]]);
    Some((id, payload[17..].to_vec()))
}
