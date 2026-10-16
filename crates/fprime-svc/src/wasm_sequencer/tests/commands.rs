//! The `cmd` host function and the command-response matching, and the
//! invalid-pointer paths of the host functions.

use super::*;

/// WASM-SEQ-013: the ComBuffer is the packet descriptor plus the guest's 8
/// bytes, and the response resumes the guest.
#[test]
fn command_byte_fidelity_and_resume() {
    let t = Tester::new();
    let file = t.stage("cmd.wasm");
    t.send_run(90, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    assert_eq!(t.pending_kind(), HostFunction::Command);
    let cmds = t.cmd_out();
    assert_eq!(cmds.len(), 1);
    // FW_PACKET_COMMAND (0) big-endian, then the payload verbatim.
    assert_eq!(
        cmds[0].0,
        [0, 0, 0xde, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03, 0x04]
    );
    t.cmd_response_in(0, t.last_cmd_context(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 1);
}

/// WASM-SEQ-019: 600 bytes do not fit a ComBuffer with its descriptor.
#[test]
fn command_too_big_traps() {
    let t = Tester::new();
    let file = t.stage("cmd_toobig.wasm");
    t.send_run(91, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_LARGE, 1);
    let max = (fprime_config::FW_COM_BUFFER_MAX_SIZE
        - size_of::<fprime_config::FwPacketDescriptorType>()) as u32;
    t.assert_event(
        W::EVENTID_BUFFER_TOO_LARGE,
        0,
        Bytes::new().ser(&HostFunction::Command).u32(600).u32(max),
    );
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_TRAPPED,
        0,
        Bytes::new()
            .u8(0)
            .ser(&SequencePhase::Main)
            .ser(&TrapReason::Host),
    );
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-013: a response while idle is harmless.
#[test]
fn unexpected_cmd_response_from_idle() {
    let t = Tester::new();
    t.cmd_response_in(0, 0, CmdResponse::Ok);
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(t.port_history_size(), 0);
}

fn invalid_pointer(kind: HostFunction) -> Bytes {
    Bytes::new().ser(&kind).ser(&Status::ErrMemOutOfBounds)
}

/// WASM-SEQ-019: an out-of-bounds command buffer is a host failure.
#[test]
fn command_bad_pointer_fails() {
    let t = Tester::new();
    let file = t.stage("cmd_badptr.wasm");
    t.send_run(92, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        invalid_pointer(HostFunction::Command),
    );
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_HOST_FAILURE,
        0,
        Bytes::new()
            .u8(0)
            .ser(&SequencePhase::Main)
            .ser(&ExitReason::HostFailure)
            .ser(&HostFunction::Command),
    );
    t.assert_response(0, W::OPCODE_RUN, 92, CmdResponse::ExecutionError);
    assert!(t.cmd_out().is_empty());
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-019.
#[test]
fn event_bad_pointer_fails() {
    let t = Tester::new();
    let file = t.stage("event_badptr.wasm");
    t.send_run(93, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        invalid_pointer(HostFunction::Event),
    );
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-019: the parameter was read, the guest write failed.
#[test]
fn parameter_bad_pointer_fails() {
    let t = Tester::new();
    t.set_next_prm(7, &[0xAA, 0xBB, 0xCC, 0xDD]);
    let file = t.stage("prm_badptr.wasm");
    t.send_run(94, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(*t.g.prm_gets.lock().unwrap(), [7]);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        invalid_pointer(HostFunction::Parameter),
    );
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 1);
}

/// WASM-SEQ-019: the time write fails.
#[test]
fn telemetry_bad_time_pointer_fails() {
    let t = Tester::new();
    t.set_next_tlm(42, time(1000, 2000), &[0x11, 0x22, 0x33, 0x44]);
    let file = t.stage("tlm_badtimeptr.wasm");
    t.send_run(95, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(*t.g.tlm_gets.lock().unwrap(), [42]);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        invalid_pointer(HostFunction::Telemetry),
    );
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 1);
}

/// WASM-SEQ-012, WASM-SEQ-019: the time write succeeds, the value write
/// fails.
#[test]
fn telemetry_bad_value_pointer_fails() {
    let t = Tester::new();
    t.set_next_tlm(42, time(1000, 2000), &[0x11, 0x22, 0x33, 0x44]);
    let file = t.stage("tlm_badvalptr.wasm");
    t.send_run(96, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(*t.g.tlm_gets.lock().unwrap(), [42]);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        invalid_pointer(HostFunction::Telemetry),
    );
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 1);
}
