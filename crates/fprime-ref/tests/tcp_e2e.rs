//! Full TCP end-to-end test: the topology runs with a real `Drv::TcpClient`
//! pointed at a `std::net::TcpListener` the TEST owns (the test plays the
//! GDS server). The test accepts the connection, sends a framed NO_OP
//! command over the socket, and verifies the corresponding event frames
//! come back over the same socket. Ephemeral ports, bounded deadlines.

use std::io::{ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

use fprime_ref::topology::{CMD_DISPATCHER_BASE_ID, RefTopology, TopologyConfig};
use fprime_svc::cmd_dispatcher::CmdDispatcher;
use fprime_utils::Hash;

mod common;

use common::scratch_data_dir;

const DEADLINE: Duration = Duration::from_secs(15);

// ---------------------------------------------------------------------------
// Frame helpers (literal, self-contained)
// ---------------------------------------------------------------------------

/// `[0xdeadbeef][len][payload][crc32]`, CRC over header + payload.
fn build_command_frame(opcode: u32, args: &[u8]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(6 + args.len());
    payload.extend_from_slice(&0x0000_u16.to_be_bytes()); // FwPacketCommand
    payload.extend_from_slice(&opcode.to_be_bytes());
    payload.extend_from_slice(args);
    let mut frame = Vec::with_capacity(payload.len() + 12);
    frame.extend_from_slice(&0xdead_beef_u32.to_be_bytes());
    frame.extend_from_slice(&(payload.len() as u32).to_be_bytes());
    frame.extend_from_slice(&payload);
    let crc = Hash::hash_u32(&frame);
    frame.extend_from_slice(&crc.to_be_bytes());
    frame
}

/// Incremental downlink-stream parser: splits accumulated bytes into
/// validated frames and returns their payloads.
struct FrameStream {
    acc: Vec<u8>,
}

impl FrameStream {
    fn new() -> Self {
        Self { acc: Vec::new() }
    }

    fn feed(&mut self, bytes: &[u8]) -> Vec<Vec<u8>> {
        self.acc.extend_from_slice(bytes);
        let mut payloads = Vec::new();
        loop {
            if self.acc.len() < 12 {
                break;
            }
            assert_eq!(
                &self.acc[0..4],
                &0xdead_beef_u32.to_be_bytes(),
                "downlink stream lost frame sync"
            );
            let len =
                u32::from_be_bytes([self.acc[4], self.acc[5], self.acc[6], self.acc[7]]) as usize;
            if self.acc.len() < len + 12 {
                break;
            }
            let crc = u32::from_be_bytes([
                self.acc[len + 8],
                self.acc[len + 9],
                self.acc[len + 10],
                self.acc[len + 11],
            ]);
            assert_eq!(crc, Hash::hash_u32(&self.acc[..len + 8]), "frame CRC");
            payloads.push(self.acc[8..8 + len].to_vec());
            self.acc.drain(..len + 12);
        }
        payloads
    }
}

/// LOG packet payload -> (event id, arg bytes).
fn parse_log_packet(payload: &[u8]) -> Option<(u32, Vec<u8>)> {
    if payload.len() < 17 || payload[0..2] != [0x00, 0x02] {
        return None;
    }
    let id = u32::from_be_bytes([payload[2], payload[3], payload[4], payload[5]]);
    Some((id, payload[17..].to_vec()))
}

/// Accept with a bounded deadline (TcpListener has no native timeout).
fn accept_with_deadline(listener: &TcpListener, deadline: Duration) -> TcpStream {
    listener.set_nonblocking(true).unwrap();
    let end = Instant::now() + deadline;
    loop {
        match listener.accept() {
            Ok((stream, _addr)) => {
                stream.set_nonblocking(false).unwrap();
                return stream;
            }
            Err(e) if e.kind() == ErrorKind::WouldBlock => {
                assert!(Instant::now() < end, "TcpClient never connected");
                std::thread::sleep(Duration::from_millis(10));
            }
            Err(e) => panic!("accept failed: {e}"),
        }
    }
}

// ---------------------------------------------------------------------------
// The test
// ---------------------------------------------------------------------------

/// GDS-side session: connect, uplink NO_OP over the socket, downlink the
/// dispatch/execution event frames back over the socket.
#[test]
fn tcp_no_op_round_trip() {
    // The test owns the server socket; the topology's TcpClient dials it.
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral");
    let port = listener.local_addr().unwrap().port();

    let topology = RefTopology::setup(&TopologyConfig {
        hostname: Some("127.0.0.1".to_string()),
        port,
        data_dir: Some(scratch_data_dir()),
    });
    topology
        .fatal_handler
        .set_exit_action(Box::new(|id| panic!("unexpected FATAL 0x{id:x}")));

    let mut stream = accept_with_deadline(&listener, DEADLINE);
    stream
        .set_read_timeout(Some(Duration::from_millis(50)))
        .unwrap();

    // Uplink a NO_OP through the socket.
    let no_op = CMD_DISPATCHER_BASE_ID + CmdDispatcher::OPCODE_CMD_NO_OP;
    stream.write_all(&build_command_frame(no_op, &[])).unwrap();
    stream.flush().unwrap();

    // Downlink: read frames until the three expected event packets arrive.
    let dispatched = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_DISPATCHED;
    let no_op_received = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_NO_OP_RECEIVED;
    let completed = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_COMPLETED;
    let mut frames = FrameStream::new();
    let mut events: Vec<(u32, Vec<u8>)> = Vec::new();
    let mut chunk = [0u8; 4096];
    let end = Instant::now() + DEADLINE;
    while !(events.iter().any(|(id, _)| *id == dispatched)
        && events.iter().any(|(id, _)| *id == no_op_received)
        && events.iter().any(|(id, _)| *id == completed))
    {
        assert!(
            Instant::now() < end,
            "timed out waiting for NO_OP event frames; got {events:?}"
        );
        match stream.read(&mut chunk) {
            Ok(0) => panic!("downlink socket closed early"),
            Ok(n) => {
                for payload in frames.feed(&chunk[..n]) {
                    if let Some(event) = parse_log_packet(&payload) {
                        events.push(event);
                    }
                }
            }
            Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {}
            Err(e) => panic!("downlink read failed: {e}"),
        }
    }

    // The dispatched/completed events name the NO_OP opcode.
    let (_, args) = events.iter().find(|(id, _)| *id == dispatched).unwrap();
    assert_eq!(args[0..4], no_op.to_be_bytes());
    let (_, args) = events.iter().find(|(id, _)| *id == completed).unwrap();
    assert_eq!(args[..], no_op.to_be_bytes());

    topology.teardown();
}
