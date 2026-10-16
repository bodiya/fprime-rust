//! `serial_send` / `serial_recv` and the `serialIn` queues.

use super::*;

/// A RUN of `name` (blocking), drained to completion.
fn run(t: &Tester, cmd_seq: u32, name: &str) {
    let file = t.stage(name);
    t.send_run(cmd_seq, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
}

fn invalid_pointer(kind: HostFunction) -> Bytes {
    Bytes::new().ser(&kind).ser(&Status::ErrMemOutOfBounds)
}

// -- SERIAL_OUT -------------------------------------------------------------

/// WASM-SEQ-016: the payload goes out verbatim on port 1 and the guest
/// resumes at once.
#[test]
fn serial_out_byte_fidelity_and_resume() {
    let t = Tester::new();
    let file = t.stage("serial_out.wasm");
    t.send_run(200, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(
        t.serial_out(),
        [(1, vec![0xde, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03, 0x04])]
    );
}

/// WASM-SEQ-019: an out-of-range port traps.
#[test]
fn serial_out_invalid_port_traps() {
    let t = Tester::new();
    run(&t, 201, "serial_out_badport.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_PORT, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_PORT,
        0,
        Bytes::new().ser(&HostFunction::SerialOut).i32(5).u32(5),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.serial_out().is_empty());
}

/// WASM-SEQ-019: a payload over the configured buffer traps.
#[test]
fn serial_out_payload_too_large_traps() {
    let t = Tester::new();
    run(&t, 202, "serial_out_toobig.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_LARGE, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_LARGE,
        0,
        Bytes::new()
            .ser(&HostFunction::SerialOut)
            .u32(300)
            .u32(SERIAL_OUT_MAX_SIZE as u32),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.serial_out().is_empty());
}

/// WASM-SEQ-019.
#[test]
fn serial_out_bad_pointer_fails() {
    let t = Tester::new();
    run(&t, 206, "serial_out_badptr.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        invalid_pointer(HostFunction::SerialOut),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.serial_out().is_empty());
}

/// WASM-SEQ-019: an unconnected (in-range) port traps.
#[test]
fn serial_out_disconnected_port_traps() {
    let mut options = Options::default();
    options.serial_out[3] = SerialOut::Unconnected;
    let t = Tester::with(options);
    run(&t, 210, "serial_out_port3.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_PORT, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_PORT,
        0,
        Bytes::new().ser(&HostFunction::SerialOut).i32(3).u32(5),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.serial_out().is_empty());
}

/// WASM-SEQ-016: a port that fails to take the payload fails the sequence.
#[test]
fn serial_out_send_failure_fails_sequence() {
    let mut options = Options::default();
    options.serial_out[3] = SerialOut::Failing;
    let t = Tester::with(options);
    run(&t, 211, "serial_out_port3.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_SERIAL_PORT_SEND_FAILED, 1);
    t.assert_event(
        W::EVENTID_SERIAL_PORT_SEND_FAILED,
        0,
        Bytes::new()
            .ser(&HostFunction::SerialOut)
            .i32(SerializeStatus::NoRoomLeft as i32),
    );
    t.assert_sequence_failure_count(1);
}

// -- SERIAL_RECV ------------------------------------------------------------

/// WASM-SEQ-017: a queued message lands exactly at the data pointer.
#[test]
fn serial_recv_blocking_delivers_message_at_data_ptr() {
    let t = Tester::new();
    t.enqueue_serial_in(2, &[0xAA, 0xBB, 0xCC, 0xDD]);
    let file = t.stage("serial_recv.wasm");
    t.send_run(300, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

/// WASM-SEQ-017: a blocked `serial_recv` wakes on the message.
#[test]
fn serial_recv_blocks_then_wakes_on_message() {
    let t = Tester::new();
    let file = t.stage("serial_recv.wasm");
    t.send_run(301, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    assert_eq!(t.pending_kind(), HostFunction::SerialRecv);
    t.serial_in(2, &[0xAA, 0xBB, 0xCC, 0xDD]);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

/// WASM-SEQ-017: 40 bytes (one full 32-byte chunk and a partial).
#[test]
fn serial_recv_multi_chunk_message() {
    let t = Tester::new();
    let msg: Vec<u8> = (1..=40).collect();
    t.enqueue_serial_in(2, &msg);
    let file = t.stage("serial_recv_big.wasm");
    t.send_run(302, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

/// WASM-SEQ-017: exactly one chunk.
#[test]
fn serial_recv_exact_chunk_multiple_message() {
    let t = Tester::new();
    let msg: Vec<u8> = (1..=32).collect();
    t.enqueue_serial_in(2, &msg);
    let file = t.stage("serial_recv_chunk32.wasm");
    t.send_run(303, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

/// WASM-SEQ-017: non-blocking on an empty queue returns EMPTY (1).
#[test]
fn serial_recv_non_blocking_empty_returns_empty_status() {
    let t = Tester::new();
    let file = t.stage("serial_recv_nonblock_empty.wasm");
    t.send_run(304, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

/// WASM-SEQ-019.
#[test]
fn serial_recv_invalid_port_traps() {
    let t = Tester::new();
    run(&t, 305, "serial_recv_badport.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_PORT, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_PORT,
        0,
        Bytes::new().ser(&HostFunction::SerialRecv).i32(5).u32(5),
    );
    t.assert_sequence_failure_count(1);
}

/// WASM-SEQ-019: 256 is not a blocking type (its low byte would be).
#[test]
fn serial_recv_invalid_blocking_type_traps() {
    let t = Tester::new();
    run(&t, 306, "serial_recv_badblock.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_INVALID_BLOCKING_TYPE_VALUE, 1);
    t.assert_event(
        W::EVENTID_INVALID_BLOCKING_TYPE_VALUE,
        0,
        Bytes::new().i32(256),
    );
    t.assert_sequence_failure_count(1);
}

/// WASM-SEQ-019: a message larger than the guest buffer fails the call.
#[test]
fn serial_recv_message_larger_than_guest_buffer_fails() {
    let t = Tester::new();
    t.enqueue_serial_in(2, &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
    run(&t, 307, "serial_recv_toobig.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_SMALL, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_SMALL,
        0,
        Bytes::new().ser(&HostFunction::SerialRecv).u32(4).u32(8),
    );
    t.assert_sequence_failure_count(1);
}

/// WASM-SEQ-019: the size pointer is out of bounds.
#[test]
fn serial_recv_bad_pointer_fails() {
    let t = Tester::new();
    t.enqueue_serial_in(2, &[0x11, 0x22, 0x33, 0x44]);
    run(&t, 308, "serial_recv_badptr.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        invalid_pointer(HostFunction::SerialRecv),
    );
    t.assert_sequence_failure_count(1);
}

/// WASM-SEQ-019: the data pointer is out of bounds (partial chunk).
#[test]
fn serial_recv_bad_data_pointer_partial_chunk_fails() {
    let t = Tester::new();
    t.enqueue_serial_in(2, &[0x11, 0x22, 0x33, 0x44]);
    run(&t, 309, "serial_recv_baddata.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        invalid_pointer(HostFunction::SerialRecv),
    );
    t.assert_sequence_failure_count(1);
}

/// WASM-SEQ-019: the data pointer is out of bounds (full chunk).
#[test]
fn serial_recv_bad_data_pointer_full_chunk_fails() {
    let t = Tester::new();
    let msg: Vec<u8> = (1..=40).collect();
    t.enqueue_serial_in(2, &msg);
    run(&t, 310, "serial_recv_baddata_big.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        invalid_pointer(HostFunction::SerialRecv),
    );
    t.assert_sequence_failure_count(1);
}

// -- serialIn queues --------------------------------------------------------

fn front(t: &Tester, port: usize) -> (u32, u8) {
    let q = t.comp.serial_in_queues();
    let queue = q.queues[port].as_ref().unwrap();
    let mut size = 0u32;
    assert_eq!(queue.peek_u32_be(&mut size, 0), SerializeStatus::Ok);
    let mut byte = 0u8;
    assert_eq!(queue.peek_u8(&mut byte, 4), SerializeStatus::Ok);
    (size, byte)
}

fn allocated(t: &Tester, port: usize) -> usize {
    t.comp.serial_in_queues().queues[port]
        .as_ref()
        .map_or(0, |q| q.get_allocated_size())
}

/// WASM-SEQ-017: frames are `[U32 size BE][payload]`.
#[test]
fn serial_in_enqueue_frames_raw_size_prefixed_messages() {
    let t = Tester::new();
    t.serial_in(0, &[0x01, 0x02, 0x03, 0x04, 0x05]);
    t.serial_in(0, &[0xAA, 0xBB, 0xCC]);
    t.dispatch_all();
    assert_eq!(allocated(&t, 0), (4 + 5) + (4 + 3));
    assert_eq!(front(&t, 0), (5, 0x01));
}

/// Three ~1/3-capacity frames (bytes 0, 1, 2) then a fourth (byte 3) on
/// `port`.
fn fill_four(t: &Tester, port: FwIndexType) -> usize {
    let capacity = SERIAL_IN_QUEUE_SIZE as usize;
    let payload = capacity / 3 - 4;
    for n in 0..4u8 {
        t.serial_in(port, &vec![n; payload]);
        t.dispatch_all();
    }
    payload
}

/// WASM-SEQ-017: DROP_OLDEST evicts whole frames until the new one fits.
#[test]
fn serial_in_queue_full_drops_oldest() {
    let t = Tester::new();
    let capacity = SERIAL_IN_QUEUE_SIZE as usize;
    let payload = capacity / 3 - 4;
    for n in 0..3u8 {
        t.serial_in(0, &vec![n; payload]);
        t.dispatch_all();
    }
    let before_free = capacity - allocated(&t, 0);
    t.serial_in(0, &vec![3; payload]);
    t.dispatch_all();
    assert!(before_free < 4 + payload);
    assert!(allocated(&t, 0) <= capacity);
    let (size, byte) = front(&t, 0);
    assert_eq!(size as usize, payload);
    assert!(byte > 0, "the oldest frame should have been dropped");
}

/// WASM-SEQ-017: a frame larger than the queue can ever hold is reported
/// and dropped, not asserted.
#[test]
fn serial_in_oversized_frame_dropped_not_asserted() {
    let t = Tester::new();
    let max_payload = SERIAL_IN_QUEUE_SIZE as usize - 4;
    let payload: Vec<u8> = (0..=max_payload).map(|i| i as u8).collect();
    t.enqueue_serial_in(0, &payload);
    assert_eq!(allocated(&t, 0), 0);
    t.assert_events(W::EVENTID_SERIAL_IN_FRAME_TOO_LARGE, 1);
    t.assert_event(
        W::EVENTID_SERIAL_IN_FRAME_TOO_LARGE,
        0,
        Bytes::new()
            .u32(0)
            .u32(payload.len() as u32)
            .u32(max_payload as u32),
    );
    t.enqueue_serial_in(0, &[1, 2, 3, 4]);
    assert_eq!(allocated(&t, 0), 8);
    t.assert_events(W::EVENTID_SERIAL_IN_FRAME_TOO_LARGE, 1);
}

/// WASM-SEQ-017: the largest frame fills the queue exactly.
#[test]
fn serial_in_max_size_frame_enqueued() {
    let t = Tester::new();
    let max_payload = SERIAL_IN_QUEUE_SIZE as usize - 4;
    let payload: Vec<u8> = (0..max_payload).map(|i| i as u8).collect();
    t.enqueue_serial_in(0, &payload);
    t.assert_events(W::EVENTID_SERIAL_IN_FRAME_TOO_LARGE, 0);
    assert_eq!(allocated(&t, 0), SERIAL_IN_QUEUE_SIZE as usize);
    assert_eq!(front(&t, 0).0 as usize, max_payload);
}

/// WASM-SEQ-017: DROP_NEWEST discards the new frame.
#[test]
fn serial_in_queue_full_drops_newest() {
    let t = Tester::new();
    t.comp.serial_in_queues().behaviors[1] = SerialInQueueFullBehavior::DropNewest;
    let capacity = SERIAL_IN_QUEUE_SIZE as usize;
    let payload = capacity / 3 - 4;
    for n in 0..3u8 {
        t.serial_in(1, &vec![n; payload]);
        t.dispatch_all();
    }
    let before = allocated(&t, 1);
    t.serial_in(1, &vec![3; payload]);
    t.dispatch_all();
    assert_eq!(allocated(&t, 1), before);
    assert_eq!(front(&t, 1), (payload as u32, 0));
    t.assert_events(W::EVENTID_SERIAL_IN_FRAME_TOO_LARGE, 0);
}

/// WASM-SEQ-017: a port without a queue drops every frame with a warning
/// (the reported maximum is 0, not an underflow).
#[test]
fn serial_in_zero_size_queue_drops_frames_without_crashing() {
    let t = Tester::new();
    t.comp.serial_in_queues().queues[3] = None;
    t.enqueue_serial_in(3, &[1, 2, 3, 4]);
    assert_eq!(allocated(&t, 3), 0);
    t.assert_events(W::EVENTID_SERIAL_IN_FRAME_TOO_LARGE, 1);
    t.assert_event(
        W::EVENTID_SERIAL_IN_FRAME_TOO_LARGE,
        0,
        Bytes::new().u32(3).u32(4).u32(0),
    );
}

/// WASM-SEQ-017: `configure` wires each port's size and behavior.
#[test]
fn serial_in_per_port_behaviors_wired_by_configure() {
    let t = Tester::unconfigured();
    let mut cfg = Tester::standard_config();
    cfg.serial_in[1].full_behavior = SerialInQueueFullBehavior::DropNewest;
    cfg.serial_in[2].full_behavior = SerialInQueueFullBehavior::DropOldest;
    cfg.serial_in[3].size = 0;
    t.configure_with(&cfg);
    t.clear_history();
    fill_four(&t, 1);
    fill_four(&t, 2);
    assert_eq!(
        front(&t, 1).1,
        0,
        "port 1 (DROP_NEWEST) keeps its oldest frame"
    );
    assert!(
        front(&t, 2).1 > 0,
        "port 2 (DROP_OLDEST) evicts its oldest frame"
    );
    t.enqueue_serial_in(3, &[0xAA, 0xBB, 0xCC, 0xDD]);
    assert_eq!(allocated(&t, 3), 0);
    t.assert_events(W::EVENTID_SERIAL_IN_FRAME_TOO_LARGE, 1);
}

/// WASM-SEQ-019: without a `serial_send` buffer every send traps.
#[test]
fn serial_out_unconfigured_traps_send() {
    let t = Tester::unconfigured();
    let mut cfg = Tester::standard_config();
    cfg.serial_out_max = 0;
    t.configure_with(&cfg);
    t.clear_history();
    run(&t, 500, "serial_out.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_LARGE, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_LARGE,
        0,
        Bytes::new().ser(&HostFunction::SerialOut).u32(8).u32(0),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.serial_out().is_empty());
}

/// WASM-SEQ-019: the `serial_send` buffer size comes from the config.
#[test]
fn serial_out_buffer_size_from_config() {
    let t = Tester::unconfigured();
    let mut cfg = Tester::standard_config();
    cfg.serial_out_max = 4;
    t.configure_with(&cfg);
    t.clear_history();
    run(&t, 501, "serial_out.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_LARGE, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_LARGE,
        0,
        Bytes::new().ser(&HostFunction::SerialOut).u32(8).u32(4),
    );
    assert!(t.serial_out().is_empty());
}

/// WASM-SEQ-017: the ASSERT behavior asserts when a frame does not fit.
#[test]
#[should_panic(expected = "Assert")]
fn serial_in_queue_full_behavior_assert_aborts() {
    let t = Tester::unconfigured();
    let mut cfg = Tester::standard_config();
    cfg.serial_in[2].size = 32;
    cfg.serial_in[2].full_behavior = SerialInQueueFullBehavior::Assert;
    t.configure_with(&cfg);
    // A 24-byte frame fits the 32-byte queue; a second does not.
    t.enqueue_serial_in(2, &[0; 20]);
    t.enqueue_serial_in(2, &[0; 20]);
}
