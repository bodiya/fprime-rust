//! `RUN`, `INVOKE`, `WAIT`, the invalid-state rejections and the trap
//! mapping.

use super::*;
use crate::wasm_sequencer::status::{Trap, map_trap_reason};
use fprime_spacewasm::spacewasm::TrapReason as Sw;

/// A RUN of `name` (blocking) drained to completion.
fn run_to_end(t: &Tester, cmd_seq: u32, name: &str) {
    let file = t.stage(name);
    t.send_run(cmd_seq, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
}

/// The `SequenceTrapped` arguments for module 0 in `phase`.
fn trapped(phase: SequencePhase, reason: TrapReason) -> Bytes {
    Bytes::new().u8(0).ser(&phase).ser(&reason)
}

/// WASM-SEQ-007: NO_BLOCK answers once the module loads; the program then
/// runs to completion.
#[test]
fn run_empty_no_block() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_run(20, &file, NO_BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_response(0, W::OPCODE_RUN, 20, CmdResponse::Ok);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-007: BLOCK answers when the interpreter finishes.
#[test]
fn run_empty_block() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_run(21, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.assert_response(0, W::OPCODE_RUN, 21, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018: a module with a start function.
#[test]
fn run_start_module() {
    let t = Tester::new();
    let file = t.stage("start.wasm");
    t.send_run(22, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.assert_response(0, W::OPCODE_RUN, 22, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-001: no `main` export.
#[test]
fn run_no_main_fails_invoke() {
    let t = Tester::new();
    let file = t.stage("no_main.wasm");
    t.send_run(23, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_INVALID_MODULE_ENTRYPOINT, 1);
    t.assert_event(
        W::EVENTID_INVALID_MODULE_ENTRYPOINT,
        0,
        Bytes::new().u8(0).ser(&Status::ErrNotFound),
    );
    t.assert_response(0, W::OPCODE_RUN, 23, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018: main's locals do not fit the stack, so the invoke itself
/// fails.
#[test]
fn run_main_invoke_fails() {
    let t = Tester::new();
    run_to_end(&t, 40, "main_overflow.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_MODULE_MAIN_INVOKE_FAILED, 1);
    t.assert_event(
        W::EVENTID_MODULE_MAIN_INVOKE_FAILED,
        0,
        Bytes::new().ser(&Status::ErrStackOverflow),
    );
    t.assert_response(0, W::OPCODE_RUN, 40, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018: an i32 main returning non-zero is a failure.
#[test]
fn run_main_returning_non_zero_fails() {
    let t = Tester::new();
    run_to_end(&t, 42, "main_i32.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 0);
    t.assert_sequence_failure_count(1);
    t.assert_events(W::EVENTID_SEQUENCE_EXITED, 1);
    t.assert_event(
        W::EVENTID_SEQUENCE_EXITED,
        0,
        Bytes::new().u8(0).ser(&SequencePhase::Main).i32(42),
    );
    t.assert_response(0, W::OPCODE_RUN, 42, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018.
#[test]
fn run_unreachable_traps() {
    let t = Tester::new();
    run_to_end(&t, 24, "unreachable.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_TRAPPED,
        0,
        trapped(SequencePhase::Main, TrapReason::Unreachable),
    );
    t.assert_response(0, W::OPCODE_RUN, 24, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018.
#[test]
fn run_div_zero_traps() {
    let t = Tester::new();
    run_to_end(&t, 25, "divzero.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_TRAPPED,
        0,
        trapped(SequencePhase::Main, TrapReason::DivideByZero),
    );
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018, WASM-SEQ-021: `exit(1)` is a failure, not a trap.
#[test]
fn run_exit_non_zero_fails() {
    let t = Tester::new();
    run_to_end(&t, 26, "exit.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_EXITED,
        0,
        Bytes::new().u8(0).ser(&SequencePhase::Main).i32(1),
    );
    t.assert_response(0, W::OPCODE_RUN, 26, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(0));
}

/// WASM-SEQ-018: `panic(7)`.
#[test]
fn run_panic_fails() {
    let t = Tester::new();
    run_to_end(&t, 27, "panic.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_PANIC,
        0,
        Bytes::new().u8(0).ser(&SequencePhase::Main).i32(7),
    );
    t.assert_response(0, W::OPCODE_RUN, 27, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018: `exit(0)` succeeds.
#[test]
fn run_exit_zero_succeeds() {
    let t = Tester::new();
    run_to_end(&t, 29, "exit0.wasm");
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.assert_response(0, W::OPCODE_RUN, 29, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018, WASM-SEQ-021: a start function that traps.
#[test]
fn run_start_traps_to_idle() {
    let t = Tester::new();
    run_to_end(&t, 28, "start_trap.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_TRAPPED,
        0,
        trapped(SequencePhase::Start, TrapReason::Unreachable),
    );
    t.assert_response(0, W::OPCODE_RUN, 28, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(0));
}

/// WASM-SEQ-018, WASM-SEQ-021: a start function whose invoke fails at
/// call setup (stack overflow).
#[test]
fn run_start_overflow_traps_to_idle() {
    let t = Tester::new();
    run_to_end(&t, 29, "start_overflow.wasm");
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_MODULE_START_INVOKE_FAILED, 1);
    t.assert_response(0, W::OPCODE_RUN, 29, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(0));
}

/// WASM-SEQ-006: LOAD then INVOKE the unnamed module.
#[test]
fn invoke_after_load() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(30, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_invoke(31, "", BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.assert_response(1, W::OPCODE_INVOKE, 31, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-006, WASM-SEQ-018: the exit status is reset per invoke, so a
/// trap after an `exit(0)` is reported as a trap.
#[test]
fn invoke_trap_after_exit_zero_is_not_misreported() {
    let t = Tester::new();
    let file = t.stage("exit0_then_trap.wasm");
    t.send_load(37, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_invoke(38, "", BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.assert_response(1, W::OPCODE_INVOKE, 38, CmdResponse::Ok);
    t.send_invoke(39, "", BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_TRAPPED,
        0,
        trapped(SequencePhase::Main, TrapReason::Unreachable),
    );
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.assert_response(2, W::OPCODE_INVOKE, 39, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-006, WASM-SEQ-007: a NO_BLOCK INVOKE answers once the module
/// resolves.
#[test]
fn invoke_no_block_responds_immediately() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(35, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_invoke(36, "", NO_BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_response(1, W::OPCODE_INVOKE, 36, CmdResponse::Ok);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-006: an unknown module name.
#[test]
fn invoke_unknown_module() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(32, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_invoke(33, "nope", NO_BLOCK, &SeqArgs::default());
    t.dispatch_all();
    t.assert_events(W::EVENTID_MODULE_NOT_FOUND, 1);
    t.assert_event(W::EVENTID_MODULE_NOT_FOUND, 0, Bytes::new().str("nope"));
    t.assert_response(1, W::OPCODE_INVOKE, 33, CmdResponse::ExecutionError);
    assert_eq!(t.controller(), ControllerState::READY);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-006: INVOKE is only valid from READY.
#[test]
fn invoke_from_idle_invalid() {
    let t = Tester::new();
    t.send_invoke(34, "", NO_BLOCK, &SeqArgs::default());
    t.dispatch_all();
    t.assert_events(W::EVENTID_CONTROLLER_CANNOT_INVOKE, 1);
    t.assert_response(0, W::OPCODE_INVOKE, 34, CmdResponse::ExecutionError);
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-007.
#[test]
fn wait_from_idle_responds_immediately() {
    let t = Tester::new();
    t.send_wait(40);
    t.dispatch_all();
    t.assert_response(0, W::OPCODE_WAIT, 40, CmdResponse::Ok);
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-007.
#[test]
fn wait_from_ready_responds_immediately() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(41, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_wait(42);
    t.dispatch_all();
    t.assert_response(1, W::OPCODE_WAIT, 42, CmdResponse::Ok);
    assert_eq!(t.controller(), ControllerState::READY);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-007: a WAIT queued during a LOAD-with-start is answered when
/// the load settles.
#[test]
fn wait_during_load_responds_on_load_complete() {
    let t = Tester::new();
    let file = t.stage("start.wasm");
    t.send_load(60, &file, "");
    t.dispatch_until_controller(ControllerState::LOAD_RUNNING_START);
    t.send_wait(61);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_response(0, W::OPCODE_LOAD, 60, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_WAIT, 61, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-009.
#[test]
fn continue_from_idle_invalid() {
    let t = Tester::new();
    t.send_continue(50);
    t.dispatch_all();
    t.assert_events(W::EVENTID_SEQUENCE_NOT_RUNNING, 1);
    t.assert_response(0, W::OPCODE_CONTINUE, 50, CmdResponse::ExecutionError);
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-009.
#[test]
fn pause_from_idle_invalid() {
    let t = Tester::new();
    t.send_pause(51);
    t.dispatch_all();
    t.assert_events(W::EVENTID_SEQUENCE_NOT_RUNNING, 1);
    t.assert_response(0, W::OPCODE_PAUSE, 51, CmdResponse::ExecutionError);
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-020.
#[test]
fn map_trap_reason_all_cases() {
    use TrapReason as TR;
    let cases = [
        (Sw::Unreachable, TR::Unreachable),
        (Sw::Host, TR::Host),
        (Sw::DivideByZero, TR::DivideByZero),
        (Sw::InvalidTableIndex, TR::InvalidTableIndex),
        (Sw::InvalidTableFunctionType, TR::InvalidTableFunctionType),
        (Sw::UninitializedTableElement, TR::UninitializedTableElement),
        (Sw::GlobalGetFailed, TR::GlobalGetFailed),
        (Sw::GlobalSetFailed, TR::GlobalSetFailed),
        (Sw::OutOfMemory, TR::OutOfMemory),
        (Sw::MemoryRefNotUnique, TR::MemoryRefNotUnique),
        (Sw::MemoryOutOfBounds, TR::MemoryOutOfBounds),
        (Sw::StackOverflow, TR::StackOverflow),
        (Sw::UnrepresentableResult, TR::UnrepresentableResult),
        (Sw::IntegerOverflow, TR::IntegerOverflow),
        (Sw::BadConversionToInteger, TR::BadConversionToInteger),
    ];
    for (trap, reason) in cases {
        assert_eq!(map_trap_reason(Trap::Reason(trap)), reason);
    }
    // Anything unmapped (TRAP_NONE, and MALFORMED_IR, which the C++ switch
    // omits) falls through to HOST.
    assert_eq!(map_trap_reason(Trap::None), TR::Host);
    assert_eq!(map_trap_reason(Trap::Reason(Sw::MalformedIr)), TR::Host);
}
