//! In-process integration test of the full reference topology WITHOUT TCP:
//! a loopback driver implements the `fprime_svc::com_stub` byte-stream
//! traits, capturing downlink frames and letting the test inject uplink
//! bytes. Rate groups are ticked manually through the rate-group driver.
//!
//! Wire-format ground truth exercised end to end:
//! frame  = `[0xdeadbeef u32][length u32][payload][crc32 u32]` (BE),
//! command payload = `[0x0000 u16][opcode u32][args]`,
//! log packet      = `[0x0002 u16][id u32][time 11B][raw args]`,
//! tlm packet      = `[0x0001 u16]` + N x `[id u32][time 11B][raw value]`.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use fprime_config::FwIndexType;
use fprime_fw::Buffer;
use fprime_os::RawTime;
use fprime_ref::signal_gen;
use fprime_ref::topology::{
    BUFFER_MANAGER_BASE_ID, CMD_DISPATCHER_BASE_ID, COM_QUEUE_BASE_ID, EVENT_MANAGER_BASE_ID,
    HEALTH_BASE_ID, RATE_GROUP_1_BASE_ID, RATE_GROUP_2_BASE_ID, RATE_GROUP_3_BASE_ID, RefTopology,
    SIGNAL_GEN_BASE_ID, TopologyConfig,
};
use fprime_svc::cmd_dispatcher::CmdDispatcher;
use fprime_svc::com_stub::{ByteStreamSendPort, ByteStreamStatus};
use fprime_utils::Hash;

// ---------------------------------------------------------------------------
// LoopbackDriver — the com_stub-facing byte-stream driver stub
// ---------------------------------------------------------------------------

/// Captures every frame ComStub sends and accepts returned receive
/// buffers. The test injects received bytes by invoking
/// `com_stub.drv_receive_in` directly (the delivery the driver would make).
#[derive(Default)]
struct LoopbackDriver {
    /// Downlink frames, one `Vec<u8>` per ComStub send.
    sent_frames: Mutex<Vec<Vec<u8>>>,
    /// Receive buffers returned to the driver (uplink ownership chain).
    returned_buffers: Mutex<Vec<Vec<u8>>>,
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

struct Harness {
    topology: RefTopology,
    loopback: Arc<LoopbackDriver>,
}

impl Harness {
    /// Build the no-comms topology, wire the loopback onto ComStub's
    /// driver-side ports, neutralize the FatalHandler exit action (a test
    /// must never `exit(1)` the whole binary), and prime the comStatus
    /// chain via the driver-ready signal (ComQueue starts WAITING).
    fn up() -> Harness {
        let topology = RefTopology::setup(&TopologyConfig::default());
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

    /// Inject bytes as if the driver had received them off the wire.
    fn inject(&self, bytes: &[u8]) {
        let mut buffer = Buffer::allocate(bytes.len());
        buffer.data_mut().copy_from_slice(bytes);
        buffer.set_size(bytes.len());
        let recv = self.topology.com_stub.drv_receive_in(0);
        recv.target
            .invoke(recv.port_num, buffer, ByteStreamStatus::OpOk);
    }

    /// One manual 1 Hz cycle into the rate-group driver.
    fn tick(&self) {
        let mut now = RawTime::new();
        let _ = now.now();
        let cycle = self.topology.rate_group_cycle_in();
        cycle.target.invoke(cycle.port_num, &now);
    }

    /// Snapshot of every LOG-packet (id, arg-bytes) pair downlinked so far.
    fn log_events(&self) -> Vec<(u32, Vec<u8>)> {
        self.loopback
            .sent_frames
            .lock()
            .unwrap()
            .iter()
            .filter_map(|frame| parse_log_packet(&deframe(frame)))
            .collect()
    }

    /// Snapshot of every telemetry (channel id, value-bytes) pair
    /// downlinked so far.
    fn tlm_values(&self) -> Vec<(u32, Vec<u8>)> {
        self.loopback
            .sent_frames
            .lock()
            .unwrap()
            .iter()
            .flat_map(|frame| parse_tlm_packet(&deframe(frame)))
            .collect()
    }
}

/// Poll `cond` every 10 ms until it holds or `deadline` elapses.
fn wait_until(deadline: Duration, mut cond: impl FnMut() -> bool) -> bool {
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

const DEADLINE: Duration = Duration::from_secs(10);

// ---------------------------------------------------------------------------
// Literal frame building / parsing
// ---------------------------------------------------------------------------

/// `[0xdeadbeef][len][payload][crc32]`, CRC over header + payload.
fn build_frame(payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(payload.len() + 12);
    frame.extend_from_slice(&0xdead_beef_u32.to_be_bytes());
    frame.extend_from_slice(&(payload.len() as u32).to_be_bytes());
    frame.extend_from_slice(payload);
    let crc = Hash::hash_u32(&frame);
    frame.extend_from_slice(&crc.to_be_bytes());
    frame
}

/// Command payload: `[descriptor 0x0000 u16][opcode u32][args]`.
fn build_command_frame(opcode: u32, args: &[u8]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(6 + args.len());
    payload.extend_from_slice(&0x0000_u16.to_be_bytes());
    payload.extend_from_slice(&opcode.to_be_bytes());
    payload.extend_from_slice(args);
    build_frame(&payload)
}

/// Literal frame parse: validate start word, length, and CRC; return the
/// payload.
fn deframe(frame: &[u8]) -> Vec<u8> {
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
fn parse_log_packet(payload: &[u8]) -> Option<(u32, Vec<u8>)> {
    if payload.len() < 17 || payload[0..2] != [0x00, 0x02] {
        return None;
    }
    let id = u32::from_be_bytes([payload[2], payload[3], payload[4], payload[5]]);
    Some((id, payload[17..].to_vec()))
}

/// Value size (bytes) per telemetry channel id — the test's mini
/// dictionary (TLM packets carry no per-entry length, C++ parity).
fn tlm_value_size(id: u32) -> usize {
    match id {
        _ if id == SIGNAL_GEN_BASE_ID + signal_gen::CHANID_SIGNAL_VALUE => 4, // F32
        _ if id == SIGNAL_GEN_BASE_ID + signal_gen::CHANID_SIGNAL_TYPE => 1,  // U8
        _ if id == CMD_DISPATCHER_BASE_ID + CmdDispatcher::CHANID_COMMANDS_DISPATCHED => 4,
        _ if id == CMD_DISPATCHER_BASE_ID + CmdDispatcher::CHANID_COMMAND_ERRORS => 4,
        _ if id == CMD_DISPATCHER_BASE_ID + CmdDispatcher::CHANID_COMMANDS_DROPPED => 4,
        _ if id == EVENT_MANAGER_BASE_ID => 8, // EventsDropped: FwSizeType
        _ if id == COM_QUEUE_BASE_ID => 8,     // ComQueueDepth [2] U32
        _ if id == COM_QUEUE_BASE_ID + 1 => 4, // BuffQueueDepth [1] U32
        _ if id == HEALTH_BASE_ID => 4,        // PingLateWarnings U32
        _ if (BUFFER_MANAGER_BASE_ID..BUFFER_MANAGER_BASE_ID + 5).contains(&id) => 4,
        _ if id == RATE_GROUP_1_BASE_ID || id == RATE_GROUP_1_BASE_ID + 1 => 4,
        _ if id == RATE_GROUP_2_BASE_ID || id == RATE_GROUP_2_BASE_ID + 1 => 4,
        _ if id == RATE_GROUP_3_BASE_ID || id == RATE_GROUP_3_BASE_ID + 1 => 4,
        _ => panic!("telemetry channel 0x{id:x} missing from the test dictionary"),
    }
}

/// TLM packet: `[0x0001 u16]` + N x `[id u32][time 11B][value]` ->
/// [(id, value bytes)]. Entries have no length prefix; sizes come from the
/// dictionary above.
fn parse_tlm_packet(payload: &[u8]) -> Vec<(u32, Vec<u8>)> {
    if payload.len() < 2 || payload[0..2] != [0x00, 0x01] {
        return Vec::new();
    }
    let mut entries = Vec::new();
    let mut at = 2;
    while at < payload.len() {
        assert!(payload.len() >= at + 15, "truncated tlm entry header");
        let id = u32::from_be_bytes([
            payload[at],
            payload[at + 1],
            payload[at + 2],
            payload[at + 3],
        ]);
        let size = tlm_value_size(id);
        let start = at + 4 + 11; // id + time tag
        assert!(payload.len() >= start + size, "truncated tlm value");
        entries.push((id, payload[start..start + size].to_vec()));
        at = start + size;
    }
    entries
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// NO_OP command round trip: framed uplink in, LOG-packet downlink frames
/// out (OpCodeDispatched, NoOpReceived, OpCodeCompleted — the completed
/// event IS the observable command response flow).
#[test]
fn no_op_command_round_trip() {
    let harness = Harness::up();
    let no_op = CMD_DISPATCHER_BASE_ID + CmdDispatcher::OPCODE_CMD_NO_OP;
    harness.inject(&build_command_frame(no_op, &[]));

    let dispatched = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_DISPATCHED;
    let no_op_received = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_NO_OP_RECEIVED;
    let completed = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_COMPLETED;
    assert!(
        wait_until(DEADLINE, || {
            let ids: Vec<u32> = harness.log_events().iter().map(|(id, _)| *id).collect();
            [dispatched, no_op_received, completed]
                .iter()
                .all(|want| ids.contains(want))
        }),
        "expected NO_OP event packets; got {:?}",
        harness.log_events()
    );

    // Event argument bytes: OpCodeDispatched(opcode u32, port i32) and
    // OpCodeCompleted(opcode u32) carry the absolute opcode.
    let events = harness.log_events();
    let (_, dispatched_args) = events.iter().find(|(id, _)| *id == dispatched).unwrap();
    assert_eq!(dispatched_args[0..4], no_op.to_be_bytes());
    let (_, completed_args) = events.iter().find(|(id, _)| *id == completed).unwrap();
    assert_eq!(completed_args[..], no_op.to_be_bytes());

    // The uplink ownership chain returned the injected buffer to the
    // driver.
    assert_eq!(harness.loopback.returned_buffers.lock().unwrap().len(), 1);
    harness.topology.teardown();
}

/// TOGGLE + manual rate-group ticks: SignalValue telemetry frames appear
/// on the downlink (queued-component drain + TlmChan run + comms chain).
#[test]
fn telemetry_downlinks_after_ticks() {
    let harness = Harness::up();
    let toggle = SIGNAL_GEN_BASE_ID + signal_gen::OPCODE_TOGGLE;
    harness.inject(&build_command_frame(toggle, &[]));

    // Tick until the SignalValue channel shows up in a downlinked TLM
    // packet. Each tick: signal_gen.sched (drain + sample) then
    // tlm_chan.run then comQueue.run on the rate-group-1 thread.
    let signal_value = SIGNAL_GEN_BASE_ID + signal_gen::CHANID_SIGNAL_VALUE;
    let signal_type = SIGNAL_GEN_BASE_ID + signal_gen::CHANID_SIGNAL_TYPE;
    let seen = wait_until(DEADLINE, || {
        harness.tick();
        harness
            .tlm_values()
            .iter()
            .any(|(id, _)| *id == signal_value)
    });
    assert!(
        seen,
        "expected SignalValue telemetry; got {:?}",
        harness.tlm_values()
    );
    // The type channel rides along; the default settings make every sample
    // amplitude * sin(...) with amplitude 0 => value bytes are 0.0f32.
    let values = harness.tlm_values();
    assert!(values.iter().any(|(id, _)| *id == signal_type));
    let (_, value_bytes) = values.iter().find(|(id, _)| *id == signal_value).unwrap();
    assert_eq!(
        f32::from_be_bytes([
            value_bytes[0],
            value_bytes[1],
            value_bytes[2],
            value_bytes[3]
        ]),
        0.0
    );
    // And the TOGGLE command completed on the downlink.
    let completed = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_COMPLETED;
    assert!(
        harness
            .log_events()
            .iter()
            .any(|(id, args)| *id == completed && args[..] == toggle.to_be_bytes())
    );
    harness.topology.teardown();
}

/// An unregistered opcode yields the InvalidCommand event and no dispatch.
#[test]
fn unknown_opcode_yields_invalid_command_and_no_dispatch() {
    let harness = Harness::up();
    let bogus: u32 = 0x0BAD_0000;
    harness.inject(&build_command_frame(bogus, &[]));

    let invalid = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_INVALID_COMMAND;
    assert!(
        wait_until(DEADLINE, || {
            harness
                .log_events()
                .iter()
                .any(|(id, args)| *id == invalid && args[..] == bogus.to_be_bytes())
        }),
        "expected InvalidCommand event; got {:?}",
        harness.log_events()
    );
    // No dispatch happened: no OpCodeDispatched (or Completed) packet ever
    // downlinked in this topology instance.
    let dispatched = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_DISPATCHED;
    let completed = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_COMPLETED;
    assert!(
        harness
            .log_events()
            .iter()
            .all(|(id, _)| *id != dispatched && *id != completed)
    );
    harness.topology.teardown();
}
