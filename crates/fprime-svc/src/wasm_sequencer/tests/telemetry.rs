//! The telemetry channels (written by `writeTelemetry`, all update on
//! change) and the per-sequence reset of the exit status.

use super::*;

/// WASM-SEQ-020, WASM-SEQ-021: a fresh component reports zeroed counters,
/// IDLE, no trap and an empty name, and nothing again without a change.
#[test]
fn telemetry_initial_defaults() {
    let t = Tester::new();
    t.flush_telemetry();
    t.assert_tlm(
        W::CHANID_CONTROLLER_STATE,
        0,
        Bytes::new().ser(&ControllerState::IDLE),
    );
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(0));
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(0));
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(0));
    t.assert_tlm(W::CHANID_COMMANDS_DISPATCHED, 0, Bytes::new().u64(0));
    t.assert_tlm(W::CHANID_COMMANDS_FAILED, 0, Bytes::new().u64(0));
    t.assert_tlm(
        W::CHANID_LAST_TRAP_REASON,
        0,
        Bytes::new().ser(&TrapReason::None),
    );
    t.assert_tlm(W::CHANID_SEQ_NAME, 0, Bytes::new().str(""));
    // Update on change: nothing the second time.
    t.clear_history();
    t.flush_telemetry();
    for id in [
        W::CHANID_CONTROLLER_STATE,
        W::CHANID_INTERPRETER_STATE,
        W::CHANID_SEQUENCES_SUCCEEDED,
        W::CHANID_COMMANDS_DISPATCHED,
        W::CHANID_LAST_TRAP_REASON,
        W::CHANID_SEQ_NAME,
    ] {
        t.assert_tlm_size(id, 0);
    }
}

/// WASM-SEQ-020: InterpreterState reports the interpreter machine (mid-run
/// the two machines differ).
#[test]
fn telemetry_interpreter_state_reflects_engine() {
    let t = Tester::new();
    t.param_set_fuel(10);
    let file = t.stage("loop.wasm");
    t.send_run(310, &file, NO_BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
    t.flush_telemetry();
    t.assert_tlm(
        W::CHANID_CONTROLLER_STATE,
        0,
        Bytes::new().ser(&ControllerState::RUNNING_MAIN),
    );
    t.assert_tlm(
        W::CHANID_INTERPRETER_STATE,
        0,
        Bytes::new().ser(&InterpreterState::RUNNING_SPINNING),
    );
    t.send_cancel(311);
    t.dispatch_until_controller(ControllerState::IDLE);
}

/// WASM-SEQ-020, WASM-SEQ-021: a successful RUN counts, and the name is
/// the file stem.
#[test]
fn telemetry_success_count_and_name() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_run(300, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.flush_telemetry();
    t.assert_tlm(
        W::CHANID_CONTROLLER_STATE,
        0,
        Bytes::new().ser(&ControllerState::READY),
    );
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(0));
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(0));
    t.assert_tlm(
        W::CHANID_LAST_TRAP_REASON,
        0,
        Bytes::new().ser(&TrapReason::None),
    );
    t.assert_tlm(W::CHANID_SEQ_NAME, 0, Bytes::new().str("empty"));
    t.assert_events(W::EVENTID_SEQUENCE_STARTING, 1);
    t.assert_event(W::EVENTID_SEQUENCE_STARTING, 0, Bytes::new().u8(0));
}

/// WASM-SEQ-021: the counters accumulate.
#[test]
fn telemetry_success_count_accumulates() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_run(301, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.send_run(302, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 2);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(2));
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(0));
}

/// WASM-SEQ-020: LOAD records the module name verbatim.
#[test]
fn telemetry_load_name_records_module_name() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(303, &file, "mod");
    t.dispatch_until_controller(ControllerState::READY);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQ_NAME, 0, Bytes::new().str("mod"));
}

/// WASM-SEQ-021: a module without main counts as failed.
#[test]
fn telemetry_failed_count() {
    let t = Tester::new();
    let file = t.stage("no_main.wasm");
    t.send_run(304, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(0));
    t.assert_tlm(
        W::CHANID_LAST_TRAP_REASON,
        0,
        Bytes::new().ser(&TrapReason::None),
    );
}

/// WASM-SEQ-020, WASM-SEQ-021: a trap records its reason.
#[test]
fn telemetry_trap_records_reason_and_fails() {
    let t = Tester::new();
    let file = t.stage("unreachable.wasm");
    t.send_run(305, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    t.flush_telemetry();
    t.assert_tlm(
        W::CHANID_CONTROLLER_STATE,
        0,
        Bytes::new().ser(&ControllerState::IDLE),
    );
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(0));
    t.assert_tlm(
        W::CHANID_LAST_TRAP_REASON,
        0,
        Bytes::new().ser(&TrapReason::Unreachable),
    );
}

/// WASM-SEQ-020: a later clean run resets LastTrapReason.
#[test]
fn last_trap_reason_cleared_across_sequences() {
    let t = Tester::new();
    let trap = t.stage("unreachable.wasm");
    t.send_run(600, &trap, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    t.flush_telemetry();
    t.assert_tlm(
        W::CHANID_LAST_TRAP_REASON,
        0,
        Bytes::new().ser(&TrapReason::Unreachable),
    );
    let ok = t.stage("empty.wasm");
    t.send_run(601, &ok, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.flush_telemetry();
    t.assert_tlm(
        W::CHANID_LAST_TRAP_REASON,
        0,
        Bytes::new().ser(&TrapReason::None),
    );
}

/// WASM-SEQ-018: the host function in SequenceHostFailure is the current
/// sequence's (reset at RUNNING entry).
#[test]
fn last_host_function_cleared_across_sequences() {
    let t = Tester::new();
    let bad = t.stage("cmd_badptr.wasm");
    t.send_run(610, &bad, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    t.assert_event(
        W::EVENTID_SEQUENCE_HOST_FAILURE,
        0,
        Bytes::new()
            .u8(0)
            .ser(&SequencePhase::Main)
            .ser(&ExitReason::HostFailure)
            .ser(&HostFunction::Command),
    );
    t.clear_history();
    t.param_set_fuel(10);
    let looping = t.stage("loop.wasm");
    t.send_run(611, &looping, NO_BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
    t.cmd_response_in(0, t.current_cmd_uid(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_event(
        W::EVENTID_SEQUENCE_HOST_FAILURE,
        0,
        Bytes::new()
            .u8(0)
            .ser(&SequencePhase::Main)
            .ser(&ExitReason::UnexpectedReply)
            .ser(&HostFunction::None),
    );
}

/// WASM-SEQ-010, WASM-SEQ-021.
#[test]
fn telemetry_cancelled_count() {
    let t = Tester::new();
    t.param_set_fuel(10);
    let file = t.stage("loop.wasm");
    t.send_run(306, &file, NO_BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
    t.send_cancel(307);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(0));
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(0));
}

/// WASM-SEQ-013, WASM-SEQ-021: a failed command response counts.
#[test]
fn telemetry_commands_dispatched_and_failed() {
    let t = Tester::new();
    let file = t.stage("cmd.wasm");
    t.send_run(308, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    t.cmd_response_in(0, t.last_cmd_context(), CmdResponse::ExecutionError);
    t.dispatch_until_controller(ControllerState::READY);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_COMMANDS_DISPATCHED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_COMMANDS_FAILED, 0, Bytes::new().u64(1));
    // The guest still ran to completion.
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(1));
}

/// WASM-SEQ-013, WASM-SEQ-021: an OK response does not count as failed.
#[test]
fn telemetry_command_ok_does_not_count_failed() {
    let t = Tester::new();
    let file = t.stage("cmd.wasm");
    t.send_run(309, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    t.cmd_response_in(0, t.last_cmd_context(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::READY);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_COMMANDS_DISPATCHED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_COMMANDS_FAILED, 0, Bytes::new().u64(0));
}

/// SeqName goes out truncated to `FW_TLM_STRING_MAX_SIZE` (40), as the
/// generated `tlmWrite_SeqName` serializes it (not upstream: no golden
/// module has a name that long).
#[test]
fn seq_name_truncated_to_tlm_string_size() {
    let t = Tester::new();
    let long = "m".repeat(45);
    t.comp
        .set_sequence_name(format!("{long}.wasm").as_bytes(), b"");
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQ_NAME, 0, Bytes::new().str(&long[..40]));
}
