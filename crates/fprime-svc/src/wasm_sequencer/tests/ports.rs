//! `seqRunIn` / `seqCancelIn`, `seqStartOut` / `seqDoneOut`, the
//! host-function timeout, the command UID checks, the busy rejections and
//! the `WAIT` queue.

use super::*;

/// RUN `loop.wasm` (no block) up to the first spin.
fn run_loop(t: &Tester, cmd_seq: u32) -> String {
    t.param_set_fuel(10);
    let file = t.stage("loop.wasm");
    t.send_run(cmd_seq, &file, NO_BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
    file
}

/// RUN `cmd.wasm` (blocking) up to its command dispatch.
fn run_cmd(t: &Tester, cmd_seq: u32) {
    let file = t.stage("cmd.wasm");
    t.send_run(cmd_seq, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
}

fn host_failure(reason: ExitReason, kind: HostFunction) -> Bytes {
    Bytes::new()
        .u8(0)
        .ser(&SequencePhase::Main)
        .ser(&reason)
        .ser(&kind)
}

/// `HOST_FUNCTION_TIMEOUT_SECS` set and loaded.
fn set_timeout(t: &Tester, secs: f32) {
    t.param_set(W::PARAMID_HOST_FUNCTION_TIMEOUT_SECS, &secs.to_be_bytes());
    t.comp.load_parameters();
}

/// WASM-SEQ-005, WASM-SEQ-018: a RUN command reports start (file + args)
/// and done.
#[test]
fn seq_start_done_emitted_on_run_command_success() {
    let t = Tester::new();
    let args = make_seq_args(&[0xDE, 0xAD, 0xBE, 0xEF]);
    let file = t.stage("empty.wasm");
    t.send_run(200, &file, NO_BLOCK, &args);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_response(0, W::OPCODE_RUN, 200, CmdResponse::Ok);
    let starts = t.seq_start();
    assert_eq!(starts.len(), 1);
    assert_eq!(starts[0].0, file.as_bytes());
    assert_eq!(starts[0].1, args);
    assert_eq!(t.seq_done(), [CmdResponse::Ok]);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018.
#[test]
fn seq_done_emitted_with_error_on_run_failure() {
    let t = Tester::new();
    let file = t.stage("unreachable.wasm");
    t.send_run(201, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_response(0, W::OPCODE_RUN, 201, CmdResponse::ExecutionError);
    assert_eq!(t.seq_start().len(), 1);
    assert_eq!(t.seq_done(), [CmdResponse::ExecutionError]);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-010.
#[test]
fn seq_done_emitted_with_error_on_run_cancel() {
    let t = Tester::new();
    run_loop(&t, 202);
    assert_eq!(t.seq_start().len(), 1);
    assert!(t.seq_done().is_empty());
    t.send_cancel(203);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_CANCELLED, 1);
    assert_eq!(t.seq_done(), [CmdResponse::ExecutionError]);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-006: INVOKE reports neither start nor done.
#[test]
fn invoke_does_not_emit_seq_start_or_done() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(204, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_invoke(205, "", BLOCK, &SeqArgs::default());
    t.dispatch_all();
    t.assert_response(1, W::OPCODE_INVOKE, 205, CmdResponse::Ok);
    assert!(t.seq_start().is_empty());
    assert!(t.seq_done().is_empty());
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-005, WASM-SEQ-018: `seqRunIn` runs without a command response.
#[test]
fn seq_run_in_port_runs_sequence() {
    let t = Tester::new();
    let args = make_seq_args(&[0x01, 0x02, 0x03]);
    let file = t.stage("empty.wasm");
    t.seq_run_in(&file, &args);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    let starts = t.seq_start();
    assert_eq!(starts.len(), 1);
    assert_eq!(starts[0].0, file.as_bytes());
    assert_eq!(starts[0].1, args);
    assert_eq!(t.seq_done(), [CmdResponse::Ok]);
    t.assert_responses(0);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018: `seqRunIn` from READY resets the store and runs.
#[test]
fn seq_run_in_from_ready_runs() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(206, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.seq_run_in(&file, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    assert_eq!(t.seq_start().len(), 1);
    assert_eq!(t.seq_done(), [CmdResponse::Ok]);
}

/// WASM-SEQ-024: `seqRunIn` while running is BUSY.
#[test]
fn seq_run_in_while_running_rejected() {
    let t = Tester::new();
    let file = run_loop(&t, 207);
    let starts_before = t.seq_start().len();
    t.seq_run_in(&file, &SeqArgs::default());
    t.dispatch_all();
    t.assert_events(W::EVENTID_CONTROLLER_BUSY, 1);
    t.assert_event(
        W::EVENTID_CONTROLLER_BUSY,
        0,
        Bytes::new()
            .ser(&SignalSource::PortRun)
            .ser(&ControllerState::RUNNING_MAIN),
    );
    assert_eq!(t.seq_start().len(), starts_before);
    t.send_cancel(208);
    t.dispatch_until_controller(ControllerState::IDLE);
}

/// WASM-SEQ-010.
#[test]
fn seq_cancel_in_cancels_running_sequence() {
    let t = Tester::new();
    t.param_set_fuel(10);
    let file = t.stage("loop.wasm");
    t.seq_run_in(&file, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
    t.seq_cancel_in();
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_CANCELLED, 1);
    assert_eq!(t.seq_done(), [CmdResponse::ExecutionError]);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-013: `checkTimers` while awaiting a command (no sleep timer).
#[test]
fn check_timers_while_awaiting_without_timer() {
    let t = Tester::new();
    run_cmd(&t, 138);
    assert!(!t.has_pending_timer());
    t.check_timers();
    t.dispatch_all();
    assert_eq!(
        t.interpreter(),
        InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING
    );
    t.cmd_response_in(0, t.last_cmd_context(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

/// WASM-SEQ-011, WASM-SEQ-021: a command without a response times out.
#[test]
fn host_function_timeout_fails_awaiting_command() {
    let t = Tester::new();
    set_timeout(&t, 5.0);
    t.set_time(time(0, 0));
    let file = t.stage("cmd.wasm");
    t.send_run(400, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    t.set_time(time(3, 0));
    t.check_timers();
    t.dispatch_all();
    assert_eq!(
        t.interpreter(),
        InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING
    );
    t.set_time(time(10, 0));
    t.check_timers();
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_HOST_FAILURE,
        0,
        host_failure(ExitReason::ReplyTimeout, HostFunction::Command),
    );
    t.assert_response(0, W::OPCODE_RUN, 400, CmdResponse::ExecutionError);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(1));
}

/// WASM-SEQ-011, WASM-SEQ-021: a time base change makes the deadline
/// incomparable.
#[test]
fn host_function_timeout_time_incomparable_fails() {
    let t = Tester::new();
    set_timeout(&t, 5.0);
    t.set_time(time_in(TimeBase::TbWorkstationTime, 0, 0));
    let file = t.stage("cmd.wasm");
    t.send_run(401, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    t.set_time(time_in(TimeBase::TbScTime, 10, 0));
    t.check_timers();
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_HOST_FAILURE,
        0,
        host_failure(ExitReason::TimerIncomparable, HostFunction::Command),
    );
    t.assert_response(0, W::OPCODE_RUN, 401, CmdResponse::ExecutionError);
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(1));
}

/// WASM-SEQ-011: a timeout of 0 disables the check.
#[test]
fn host_function_timeout_disabled_by_zero() {
    let t = Tester::new();
    set_timeout(&t, 0.0);
    t.set_time(time(0, 0));
    let file = t.stage("cmd.wasm");
    t.send_run(401, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    t.set_time(time(100_000, 0));
    t.check_timers();
    t.dispatch_all();
    assert_eq!(
        t.interpreter(),
        InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING
    );
    t.cmd_response_in(0, t.last_cmd_context(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

/// WASM-SEQ-011, WASM-SEQ-017: a blocking `serial_recv` times out too.
#[test]
fn host_function_timeout_fails_blocking_serial_recv() {
    let t = Tester::new();
    set_timeout(&t, 2.0);
    t.set_time(time(0, 0));
    let file = t.stage("serial_recv.wasm");
    t.send_run(402, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    assert_eq!(t.pending_kind(), HostFunction::SerialRecv);
    t.set_time(time(30, 0));
    t.check_timers();
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_response(0, W::OPCODE_RUN, 402, CmdResponse::ExecutionError);
}

/// The fractional-second timeout rounds its microseconds up and carries
/// into the seconds (not upstream: its tests use whole seconds).
#[test]
fn host_function_timeout_fractional_deadline() {
    let t = Tester::new();
    set_timeout(&t, 1.5);
    t.set_time(time(10, 0));
    run_cmd(&t, 403);
    t.set_time(time(11, 499_999));
    t.check_timers();
    t.dispatch_all();
    assert_eq!(
        t.interpreter(),
        InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING
    );
    t.set_time(time(11, 500_000));
    t.check_timers();
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_event(
        W::EVENTID_SEQUENCE_HOST_FAILURE,
        0,
        host_failure(ExitReason::ReplyTimeout, HostFunction::Command),
    );
}

/// WASM-SEQ-013: a response from an older sequence is reported and
/// ignored.
#[test]
fn late_cmd_response_from_old_sequence_ignored() {
    let t = Tester::new();
    run_cmd(&t, 410);
    let current = t.current_cmd_uid();
    let old_seq = (((current >> 16) & 0xFFFF) as u16).wrapping_sub(1);
    let stale = (u32::from(old_seq) << 16) | (current & 0xFFFF);
    t.cmd_response_in(0, stale, CmdResponse::Ok);
    t.dispatch_all();
    assert_eq!(
        t.interpreter(),
        InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING
    );
    t.assert_events(W::EVENTID_CMD_RESPONSE_FROM_OLD_SEQUENCE, 1);
    t.assert_event(
        W::EVENTID_CMD_RESPONSE_FROM_OLD_SEQUENCE,
        0,
        Bytes::new()
            .u32(0)
            .ser(&CmdResponse::Ok)
            .u16(old_seq)
            .u16(((current >> 16) & 0xFFFF) as u16),
    );
    t.cmd_response_in(0, t.last_cmd_context(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

/// WASM-SEQ-013: a response for another instance fails the sequence.
#[test]
fn wrong_cmd_response_index_fails() {
    let t = Tester::new();
    run_cmd(&t, 411);
    let current = t.current_cmd_uid();
    let wrong = (current & 0xFFFF_0000) | (current.wrapping_add(1) & 0xFFFF);
    t.cmd_response_in(0, wrong, CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_events(W::EVENTID_WRONG_CMD_RESPONSE_INDEX, 1);
    t.assert_event(
        W::EVENTID_WRONG_CMD_RESPONSE_INDEX,
        0,
        Bytes::new()
            .u32(0)
            .ser(&CmdResponse::Ok)
            .u16((wrong & 0xFFFF) as u16)
            .u16((current & 0xFFFF) as u16),
    );
    t.assert_sequence_failure_count(1);
    t.assert_response(0, W::OPCODE_RUN, 411, CmdResponse::ExecutionError);
}

/// WASM-SEQ-013: each run stamps the next sequence index.
#[test]
fn sequence_index_increments_across_runs() {
    let t = Tester::new();
    run_cmd(&t, 420);
    let first = (t.last_cmd_context() >> 16) & 0xFFFF;
    t.send_cancel(421);
    t.dispatch_until_controller(ControllerState::IDLE);
    let file = t.stage("cmd.wasm");
    t.send_run(422, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    let second = (t.last_cmd_context() >> 16) & 0xFFFF;
    assert_ne!(first, second);
    assert_eq!((first + 1) & 0xFFFF, second);
    t.cmd_response_in(0, t.last_cmd_context(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

/// WASM-SEQ-013: the real command UID of a cancelled run is recognized as
/// old by the next run.
#[test]
fn late_cmd_response_from_previous_run_ignored() {
    let t = Tester::new();
    run_cmd(&t, 423);
    let first = t.last_cmd_context();
    t.send_cancel(424);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_CANCELLED, 1);
    let file = t.stage("cmd.wasm");
    t.send_run(425, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING);
    let second = t.last_cmd_context();
    assert_ne!(first, second);
    t.cmd_response_in(0, first, CmdResponse::Ok);
    t.dispatch_all();
    assert_eq!(
        t.interpreter(),
        InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING
    );
    t.assert_events(W::EVENTID_CMD_RESPONSE_FROM_OLD_SEQUENCE, 1);
    t.assert_event(
        W::EVENTID_CMD_RESPONSE_FROM_OLD_SEQUENCE,
        0,
        Bytes::new()
            .u32(0)
            .ser(&CmdResponse::Ok)
            .u16(((first >> 16) & 0xFFFF) as u16)
            .u16(((second >> 16) & 0xFFFF) as u16),
    );
    t.cmd_response_in(0, second, CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
}

fn busy(source: SignalSource) -> Bytes {
    Bytes::new()
        .ser(&source)
        .ser(&ControllerState::RUNNING_MAIN)
}

/// WASM-SEQ-024.
#[test]
fn run_while_running_rejected() {
    let t = Tester::new();
    let file = run_loop(&t, 150);
    t.send_run(151, &file, NO_BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_CONTROLLER_BUSY, 1);
    t.assert_event(
        W::EVENTID_CONTROLLER_BUSY,
        0,
        busy(SignalSource::CommandRun),
    );
    t.assert_response(0, W::OPCODE_RUN, 150, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_RUN, 151, CmdResponse::Busy);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-024.
#[test]
fn load_while_running_rejected() {
    let t = Tester::new();
    let file = run_loop(&t, 152);
    t.send_load(153, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_CONTROLLER_BUSY, 1);
    t.assert_event(
        W::EVENTID_CONTROLLER_BUSY,
        0,
        busy(SignalSource::CommandLoad),
    );
    t.assert_response(0, W::OPCODE_RUN, 152, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_LOAD, 153, CmdResponse::Busy);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-024.
#[test]
fn invoke_while_running_rejected() {
    let t = Tester::new();
    run_loop(&t, 154);
    t.send_invoke(155, "", NO_BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_CONTROLLER_BUSY, 1);
    t.assert_event(
        W::EVENTID_CONTROLLER_BUSY,
        0,
        busy(SignalSource::CommandInvoke),
    );
    t.assert_response(0, W::OPCODE_RUN, 154, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_INVOKE, 155, CmdResponse::Busy);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-007: WAIT while running answers on finish.
#[test]
fn wait_while_running_queues_until_finish() {
    let t = Tester::new();
    run_loop(&t, 160);
    t.send_wait(161);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_response(0, W::OPCODE_RUN, 160, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_WAIT, 161, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-013: a response while spinning is unexpected.
#[test]
fn unexpected_cmd_response_while_spinning_fails() {
    let t = Tester::new();
    run_loop(&t, 170);
    t.cmd_response_in(0, t.current_cmd_uid(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_HOST_FAILURE,
        0,
        host_failure(ExitReason::UnexpectedReply, HostFunction::None),
    );
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-013: a response while sleeping is unexpected.
#[test]
fn unexpected_cmd_response_while_awaiting_sleep_fails() {
    let t = Tester::new();
    let file = t.stage("rsleep.wasm");
    t.set_time(time(0, 0));
    t.send_run(172, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_SLEEPING);
    assert!(t.has_pending_timer());
    t.cmd_response_in(0, t.current_cmd_uid(), CmdResponse::Ok);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-001.
#[test]
fn load_while_ready_reloads() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(173, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_load(174, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_response(0, W::OPCODE_LOAD, 173, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_LOAD, 174, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-001.
#[test]
fn load_memoryless_module_succeeds() {
    let t = Tester::new();
    let file = t.stage("nomem.wasm");
    t.send_load(180, &file, "lib");
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_response(0, W::OPCODE_LOAD, 180, CmdResponse::Ok);
    t.assert_events(W::EVENTID_MODULE_LOAD_FAILED, 0);
}

/// WASM-SEQ-001.
#[test]
fn load_after_memoryless_module_succeeds() {
    let t = Tester::new();
    let nomem = t.stage("nomem.wasm");
    let empty = t.stage("empty.wasm");
    t.send_load(181, &nomem, "lib");
    t.dispatch_until_controller(ControllerState::READY);
    t.send_load(182, &empty, "app");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_response(0, W::OPCODE_LOAD, 181, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_LOAD, 182, CmdResponse::Ok);
    t.assert_events(W::EVENTID_MODULE_LOAD_FAILED, 0);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-007: the 9th WAIT overflows the 8-deep queue.
#[test]
fn wait_finish_queue_overflow() {
    let t = Tester::new();
    run_loop(&t, 180);
    for i in 0..9 {
        t.send_wait(181 + i);
    }
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_TOO_MANY_BLOCKING_COMMANDS, 1);
    t.assert_response(1, W::OPCODE_WAIT, 189, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-007, WASM-SEQ-010.
#[test]
fn wait_drained_with_error_when_running_sequence_cancelled() {
    let t = Tester::new();
    run_loop(&t, 500);
    t.send_wait(501);
    t.send_cancel(502);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_response(0, W::OPCODE_RUN, 500, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_CANCEL, 502, CmdResponse::Ok);
    t.assert_response(2, W::OPCODE_WAIT, 501, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-007, WASM-SEQ-010.
#[test]
fn wait_drained_with_error_when_run_with_start_cancelled() {
    let t = Tester::new();
    let file = t.stage("start.wasm");
    t.send_run(510, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::RUNNING_START_PENDING_MAIN);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
    t.dispatch_one();
    t.send_wait(511);
    t.send_cancel(512);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_response(0, W::OPCODE_CANCEL, 512, CmdResponse::Ok);
    t.assert_response(1, W::OPCODE_RUN, 510, CmdResponse::ExecutionError);
    t.assert_response(2, W::OPCODE_WAIT, 511, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-007, WASM-SEQ-018.
#[test]
fn wait_drained_with_error_when_start_fails() {
    let t = Tester::new();
    let file = t.stage("start_trap.wasm");
    t.send_load(520, &file, "");
    t.dispatch_until_controller(ControllerState::LOAD_RUNNING_START);
    t.send_wait(521);
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_response(0, W::OPCODE_LOAD, 520, CmdResponse::ExecutionError);
    t.assert_response(1, W::OPCODE_WAIT, 521, CmdResponse::ExecutionError);
}

/// The sequencer interface (`seqRunIn`, `seqCancelIn`, `seqStartOut`,
/// `seqDoneOut`) wires to `Svc::SeqDispatcher` as upstream's does: a run
/// dispatched to the sequencer takes it out of the pool until its done
/// returns it (not upstream: its WasmSequencer tests have no dispatcher).
#[test]
fn seq_dispatcher_drives_the_sequencer() {
    use crate::seq_dispatcher::SeqDispatcher;
    let t = Tester::with(Options {
        seq_out: false,
        ..Options::default()
    });
    let disp = SeqDispatcher::new("seqDisp");
    disp.seq_run_out[0].connect_to(t.comp.seq_run_in(0));
    disp.seq_cancel_out[0].connect_to(t.comp.seq_cancel_in(0));
    t.comp.seq_start_out.connect_to(disp.seq_start_in(0));
    t.comp.seq_done_out.connect_to(disp.seq_done_in(0));
    disp.init(10);
    let dispatch_disp = || {
        while disp.active.queued.queue().get_messages_available() > 0 {
            let status = disp
                .active
                .queued
                .do_dispatch(&*disp, BlockingType::NonBlocking);
            assert_eq!(status, MsgDispatchStatus::Ok);
        }
    };
    let file = t.stage("empty.wasm");
    let run = || {
        let p = disp.seq_run_in(0);
        p.target.invoke(
            p.port_num,
            &FileNameString::from(file.as_str()),
            &SeqArgs::default(),
        );
        dispatch_disp();
    };
    // The first run goes to the sequencer; a second one finds it busy.
    run();
    run();
    assert_eq!(t.messages_available(), 1);
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    // seqStartOut / seqDoneOut reached the dispatcher, which frees the
    // sequencer: the next run is dispatched again.
    dispatch_disp();
    run();
    assert_eq!(t.messages_available(), 1);
    t.dispatch_all();
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 2);
}
