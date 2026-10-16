//! `PAUSE`, `CONTINUE` and `CANCEL` across the state machines, including
//! the deferred cancel during the load / start window.

use super::*;

/// RUN `loop.wasm` (no block) up to the first spin.
fn run_loop(t: &Tester, cmd_seq: u32) {
    t.param_set_fuel(10);
    let file = t.stage("loop.wasm");
    t.send_run(cmd_seq, &file, NO_BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
}

/// WASM-SEQ-009.
#[test]
fn pause_then_continue_completes() {
    let t = Tester::new();
    run_loop(&t, 110);
    t.send_pause(111);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_PAUSED);
    t.assert_events(W::EVENTID_SEQUENCE_PAUSED, 1);
    t.send_continue(112);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_response(2, W::OPCODE_CONTINUE, 112, CmdResponse::Ok);
    t.assert_event(W::EVENTID_SEQUENCE_SUCCEEDED, 0, Bytes::new().u8(0));
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-008, WASM-SEQ-010.
#[test]
fn cancel_while_spinning() {
    let t = Tester::new();
    run_loop(&t, 120);
    t.send_cancel(121);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_event(
        W::EVENTID_SEQUENCE_CANCELLED,
        0,
        Bytes::new().u8(0).ser(&SequencePhase::Main),
    );
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-010.
#[test]
fn cancel_while_paused() {
    let t = Tester::new();
    run_loop(&t, 122);
    t.send_pause(123);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_PAUSED);
    t.send_cancel(124);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_CANCELLED, 1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-010: a cancel while awaiting a command response clears the
/// pending host function.
#[test]
fn cancel_while_awaiting_response() {
    let t = Tester::new();
    let file = t.stage("cmd.wasm");
    t.send_run(125, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    t.send_cancel(126);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_CANCELLED, 1);
    assert_eq!(t.pending_kind(), HostFunction::None);
    assert_eq!(t.port_history_size(), 1);
}

/// WASM-SEQ-010.
#[test]
fn cancel_from_ready_returns_idle() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(130, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_cancel(131);
    t.dispatch_until_controller(ControllerState::IDLE);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-003, WASM-SEQ-010: CANCEL from IDLE re-enters IDLE, which
/// reallocates the store.
#[test]
fn cancel_from_idle_stays_idle() {
    let t = Tester::new();
    t.send_cancel(132);
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_CANCELLED, 0);
    t.assert_events(W::EVENTID_STORE_ALLOCATION_SUCCEEDED, 1);
    t.assert_event(
        W::EVENTID_STORE_ALLOCATION_SUCCEEDED,
        0,
        Bytes::new().u16(u16::from(Tester::standard_config().max_guest_modules)),
    );
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-010: the interpreter answers CANCEL, also from IDLE.
#[test]
fn cancel_command_from_idle_responds_ok() {
    let t = Tester::new();
    t.send_cancel(400);
    t.dispatch_all();
    assert_eq!(t.interpreter(), InterpreterState::IDLE);
    t.assert_responses(1);
    t.assert_response(0, W::OPCODE_CANCEL, 400, CmdResponse::Ok);
}

/// WASM-SEQ-010.
#[test]
fn cancel_command_while_running_responds_ok() {
    let t = Tester::new();
    run_loop(&t, 401);
    t.send_cancel(402);
    t.dispatch_until_controller(ControllerState::IDLE);
    assert_eq!(t.interpreter(), InterpreterState::IDLE);
    t.assert_responses(2);
    t.assert_response(0, W::OPCODE_RUN, 401, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_CANCEL, 402, CmdResponse::Ok);
    t.assert_events(W::EVENTID_SEQUENCE_CANCELLED, 1);
}

/// WASM-SEQ-010, WASM-SEQ-021: a cancel latched while loading diverts the
/// RUN before it runs.
#[test]
fn run_cancelled_during_load_diverts() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_run(140, &file, BLOCK, &SeqArgs::default());
    t.dispatch_one();
    t.send_cancel(141);
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_response(0, W::OPCODE_CANCEL, 141, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_RUN, 140, CmdResponse::ExecutionError);
    t.assert_events(W::EVENTID_SEQUENCE_STARTING, 0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 0);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(0));
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(0));
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-010, WASM-SEQ-021.
#[test]
fn load_cancelled_during_load_diverts() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(142, &file, "");
    t.dispatch_one();
    t.send_cancel(143);
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_response(0, W::OPCODE_CANCEL, 143, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_LOAD, 142, CmdResponse::ExecutionError);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(0));
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-010, WASM-SEQ-021: a cancel latched in INVOKING diverts the
/// invoke before main runs.
#[test]
fn invoke_cancelled_during_invoking_diverts() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(144, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_invoke(145, "", BLOCK, &SeqArgs::default());
    t.dispatch_one();
    t.send_cancel(146);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_response(0, W::OPCODE_LOAD, 144, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_CANCEL, 146, CmdResponse::Ok);
    t.assert_response(2, W::OPCODE_INVOKE, 145, CmdResponse::ExecutionError);
    t.assert_events(W::EVENTID_SEQUENCE_STARTING, 0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 0);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(1));
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-010, WASM-SEQ-021: a cancel in the start→main gap keeps main
/// from running.
#[test]
fn run_with_start_cancelled_in_start_main_gap_diverts() {
    let t = Tester::new();
    let file = t.stage("start.wasm");
    t.send_run(147, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::RUNNING_START_PENDING_MAIN);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
    t.dispatch_one(); // the start spin: interpreterFinished queued
    t.send_cancel(148);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_response(0, W::OPCODE_CANCEL, 148, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_RUN, 147, CmdResponse::ExecutionError);
    t.assert_events(W::EVENTID_SEQUENCE_STARTING, 0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 0);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(0));
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(0));
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-010, WASM-SEQ-021: a cancel of a running engine counts once.
#[test]
fn cancel_after_engine_running_counts_once() {
    let t = Tester::new();
    run_loop(&t, 149);
    t.send_cancel(150);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_CANCELLED, 1);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(1));
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-010, WASM-SEQ-021: the latch is cleared for the next attempt.
#[test]
fn cancel_latch_does_not_leak() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_run(151, &file, BLOCK, &SeqArgs::default());
    t.dispatch_one();
    t.send_cancel(152);
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_response(0, W::OPCODE_CANCEL, 152, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_RUN, 151, CmdResponse::ExecutionError);
    t.send_run(153, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_response(2, W::OPCODE_RUN, 153, CmdResponse::Ok);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(1));
}

/// WASM-SEQ-010, WASM-SEQ-021: `seqCancelIn` latches like CANCEL; a
/// port-sourced RUN cancelled before it runs reports neither start nor
/// done.
#[test]
fn seq_cancel_in_during_load_diverts() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.seq_run_in(&file, &SeqArgs::default());
    t.dispatch_one();
    t.seq_cancel_in();
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_STARTING, 0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 0);
    assert!(t.seq_start().is_empty());
    assert!(t.seq_done().is_empty());
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(1));
}

/// WASM-SEQ-010, WASM-SEQ-021: READY's entry clears a latch left by a
/// failed invoke.
#[test]
fn cancel_during_failed_invoke_does_not_leak_to_next_invoke() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(154, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_invoke(155, "nope", BLOCK, &SeqArgs::default());
    t.dispatch_one();
    t.send_cancel(156);
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_MODULE_NOT_FOUND, 1);
    t.send_invoke(157, "", BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_SUCCEEDED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(0));
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-013: a response while paused is unexpected and fails.
#[test]
fn unexpected_cmd_response_while_paused_fails() {
    let t = Tester::new();
    run_loop(&t, 133);
    t.send_pause(134);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_PAUSED);
    t.cmd_response_in(0, t.current_cmd_uid(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-009: a pause landing on a host function call keeps the call
/// pending; CONTINUE dispatches it.
#[test]
fn pause_at_host_function_then_continue_resumes() {
    let t = Tester::new();
    let file = t.stage("cmd.wasm");
    t.send_run(135, &file, NO_BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
    t.send_pause(136);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_PAUSED);
    t.assert_events(W::EVENTID_SEQUENCE_PAUSED, 1);
    assert!(t.cmd_out().is_empty());
    assert_eq!(t.pending_kind(), HostFunction::Command);
    t.send_continue(137);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    assert_eq!(t.cmd_out().len(), 1);
    t.cmd_response_in(0, t.last_cmd_context(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.cmd_out().len(), 1);
}

/// WASM-SEQ-009: a pause while awaiting a response takes effect when the
/// response arrives.
#[test]
fn pause_while_awaiting_response_is_pending() {
    let t = Tester::new();
    let file = t.stage("cmd.wasm");
    t.send_run(175, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    t.send_pause(176);
    t.dispatch_all();
    assert_eq!(
        t.interpreter(),
        InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING
    );
    t.cmd_response_in(0, t.last_cmd_context(), CmdResponse::Ok);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_PAUSED);
    t.assert_events(W::EVENTID_SEQUENCE_PAUSED, 1);
    t.send_continue(177);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

/// WASM-SEQ-009: a second PAUSE while paused changes nothing.
#[test]
fn pause_while_paused_is_idempotent() {
    let t = Tester::new();
    run_loop(&t, 178);
    t.send_pause(179);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_PAUSED);
    t.send_pause(180);
    t.dispatch_all();
    assert_eq!(t.interpreter(), InterpreterState::RUNNING_PAUSED);
    t.assert_events(W::EVENTID_SEQUENCE_PAUSED, 1);
    t.send_cancel(181);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_CANCELLED, 1);
}

/// WASM-SEQ-009: CONTINUE while running answers OK.
#[test]
fn continue_while_spinning_is_ok() {
    let t = Tester::new();
    run_loop(&t, 140);
    t.send_continue(141);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_response(1, W::OPCODE_CONTINUE, 141, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}
