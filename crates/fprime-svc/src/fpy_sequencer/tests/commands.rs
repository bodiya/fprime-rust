//! Commands, ports, parameters and telemetry through the queue (upstream
//! `cmd_*`, `seqBaseDir_*`, `validate_emptySequenceFilePath`,
//! `prmSeqBaseDirTlm`, `isRunningState`, `checkTimers`, `ping`,
//! `cmdResponse`, `tlmWrite`, `seqRunIn`, `seqCancelIn`, `seqRunInArgs`),
//! plus the generated-code contract (registration, argument decoding,
//! priorities, the signal message).

use super::*;

const OK: CmdResponse = CmdResponse::Ok;
const ERR: CmdResponse = CmdResponse::ExecutionError;

#[test]
fn cmd_run() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    t.send_run(&path, BlockState::Block);
    t.dispatch_until_state(State::VALIDATING);
    assert_eq!(t.st().sequences_started, 0);
    assert_eq!(t.st().statements_dispatched, 0);
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    assert_eq!(
        t.seq_start(),
        vec![(path.as_bytes().to_vec(), SeqArgs::default())]
    );
    assert_eq!(t.st().sequences_started, 1);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.st().statements_dispatched, 1);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN, OK);
    assert_eq!(t.seq_done(), vec![(0, 0, OK)]);
    t.clear_history();

    // non-blocking: answered at once
    t.send_run(&path, BlockState::NoBlock);
    t.dispatch();
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN, OK);
    assert!(t.seq_done().is_empty());
    t.dispatch_until_state(State::VALIDATING);
    assert_eq!(t.seq_start().len(), 1);
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.responses().len(), 1);
    assert_eq!(t.seq_done(), vec![(0, 0, OK)]);
    t.clear_history();

    // a file that fails validation
    t.send_run("invalid seq", BlockState::Block);
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_RUN, ERR);
    assert_eq!(t.seq_done(), vec![(0, 0, ERR)]);
    t.clear_history();

    // already running
    t.set_state(State::RUNNING_DISPATCH_STATEMENT);
    t.send_run("invalid seq", BlockState::Block);
    t.dispatch();
    t.assert_response(0, FpySequencer::OPCODE_RUN, ERR);
    // InvalidCommand carries the state (RUNNING_DISPATCH_STATEMENT = 4)
    assert_eq!(
        t.events(FpySequencer::EVENTID_INVALID_COMMAND),
        vec![vec![0, 0, 0, 4]]
    );
}

#[test]
fn cmd_run_args() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_argument_spec("arg1", "U32", 4);
    t.add_argument_spec("arg2", "U32", 4);
    t.add_load_rel(0, 4);
    t.add_load_rel(4, 4);
    t.add_discard(16);
    let path = t.write_to_file("test.bin");
    let args = seq_args_u32(&[10, 20]);
    t.send_run_args(&path, BlockState::Block, &args);
    t.dispatch_until_state(State::VALIDATING);
    assert_eq!(t.st().sequences_started, 0);
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    assert_eq!(t.seq_start(), vec![(path.as_bytes().to_vec(), args)]);
    assert_eq!(t.st().sequences_started, 1);
    // both arguments are on the stack
    assert_eq!(t.stack_size(), 8);
    assert_eq!(
        t.st().runtime.stack.bytes[..8].to_vec(),
        vec![0, 0, 0, 10, 0, 0, 0, 20]
    );
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.st().statements_dispatched, 3);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN_ARGS, OK);
    assert_eq!(t.seq_done(), vec![(0, 0, OK)]);
    assert_eq!(t.stack_size(), 0);
}

#[test]
fn cmd_run_args_oversized() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    let args = SeqArgs {
        size: MAX_STACK_SIZE as FwSizeType + 1,
        ..SeqArgs::default()
    };
    t.send_run_args(&path, BlockState::Block, &args);
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN_ARGS, ERR);
    assert_eq!(t.seq_done(), vec![(0, 0, ERR)]);
}

use fprime_config::fpy_sequencer::MAX_STACK_SIZE;

#[test]
fn cmd_run_args_residual_stack() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_argument_spec("arg1", "U32", 4);
    t.add_discard(4);
    let path = t.write_to_file("test.bin");
    // a full stack left over from a previous run: validation checks the
    // arguments against the stack limit, not against the leftovers
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType);
    t.send_run_args(&path, BlockState::Block, &seq_args_u32(&[10]));
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    // the runtime was reset on entering RUNNING, then the args pushed
    assert_eq!(t.stack_size(), 4);
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_RUN_ARGS, OK);
    assert_eq!(t.seq_done(), vec![(0, 0, OK)]);
    assert_eq!(
        t.event_count(FpySequencer::EVENTID_ARG_TOTAL_SIZE_EXCEEDS_STACK_LIMIT),
        0
    );
}

#[test]
fn is_running_state() {
    let t = Tester::new();
    // the argument is evaluated, not the current state
    t.set_state(State::IDLE);
    assert!(FpySequencer::is_running_state(
        State::RUNNING_AWAITING_STATEMENT_RESPONSE
    ));
    assert!(FpySequencer::is_running_state(
        State::RUNNING_DISPATCH_STATEMENT
    ));
    assert!(FpySequencer::is_running_state(State::RUNNING_PAUSED));
    assert!(FpySequencer::is_running_state(State::RUNNING_SLEEPING));
    assert!(!FpySequencer::is_running_state(State::IDLE));
    assert!(!FpySequencer::is_running_state(State::VALIDATING));
    assert!(!FpySequencer::is_running_state(
        State::AWAITING_CMD_RUN_VALIDATED
    ));
}

#[test]
fn cmd_validate() {
    let mut t = Tester::new();
    t.send_validate("invalid seq");
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_VALIDATE, ERR);
    t.clear_history();

    t.alloc_mem();
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    t.send_validate(&path);
    t.dispatch_until_state(State::VALIDATING);
    assert_eq!(t.st().sequences_started, 0);
    t.dispatch_until_state(State::AWAITING_CMD_RUN_VALIDATED);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_VALIDATE, OK);
    t.clear_history();

    // only from IDLE
    t.set_state(State::VALIDATING);
    t.send_validate(&path);
    t.dispatch();
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_VALIDATE, ERR);
}

#[test]
fn cmd_run_validated() {
    let mut t = Tester::new();
    // not validated
    t.set_state(State::IDLE);
    t.send_run_validated(BlockState::NoBlock);
    t.dispatch_current();
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN_VALIDATED, ERR);
    t.clear_history();

    t.alloc_mem();
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    t.send_validate(&path);
    t.dispatch_until_state(State::AWAITING_CMD_RUN_VALIDATED);
    t.clear_history();
    t.send_run_validated(BlockState::NoBlock);
    t.dispatch();
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN_VALIDATED, OK);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.responses().len(), 1);
    t.clear_history();

    // blocking: answered when the sequence ends
    t.send_validate(&path);
    t.dispatch_until_state(State::AWAITING_CMD_RUN_VALIDATED);
    t.clear_history();
    t.send_run_validated(BlockState::Block);
    t.dispatch();
    assert!(t.responses().is_empty());
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN_VALIDATED, OK);
}

#[test]
fn cmd_validate_args() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_argument_spec("arg1", "U32", 4);
    t.add_argument_spec("arg2", "U32", 4);
    t.add_load_rel(0, 4);
    t.add_load_rel(4, 4);
    t.add_discard(16);
    let path = t.write_to_file("test.bin");
    let args = seq_args_u32(&[10, 20]);
    t.send_validate_args(&path, &args);
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::AWAITING_CMD_RUN_VALIDATED);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_VALIDATE_ARGS, OK);
    t.clear_history();

    t.send_run_validated(BlockState::NoBlock);
    t.dispatch();
    t.assert_response(0, FpySequencer::OPCODE_RUN_VALIDATED, OK);
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    assert_eq!(t.st().sequences_started, 1);
    assert_eq!(t.stack_size(), 8);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.st().statements_dispatched, 3);
    assert_eq!(t.responses().len(), 1);
    t.clear_history();

    t.send_validate_args(&path, &args);
    t.dispatch_until_state(State::AWAITING_CMD_RUN_VALIDATED);
    t.clear_history();
    t.send_run_validated(BlockState::Block);
    t.dispatch();
    assert!(t.responses().is_empty());
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    assert_eq!(t.st().sequences_started, 2);
    assert_eq!(t.stack_size(), 8);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN_VALIDATED, OK);
    assert_eq!(t.stack_size(), 0);
}

#[test]
fn cmd_validate_args_oversized() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    let args = SeqArgs {
        size: MAX_STACK_SIZE as FwSizeType + 1,
        ..SeqArgs::default()
    };
    t.send_validate_args(&path, &args);
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_VALIDATE_ARGS, ERR);
    assert_eq!(t.seq_done(), vec![(0, 0, ERR)]);
}

#[test]
fn cmd_validate_args_spec_exceeds_stack_limit() {
    let mut t = Tester::new();
    t.alloc_mem();
    let half = MAX_STACK_SIZE as StackSizeType / 2 + 1;
    t.add_argument_spec("arg1", "bytes", half);
    t.add_argument_spec("arg2", "bytes", half);
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    t.send_validate_args(&path, &SeqArgs::default());
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_VALIDATE_ARGS, ERR);
    assert_eq!(
        t.events(FpySequencer::EVENTID_ARG_TOTAL_SIZE_EXCEEDS_STACK_LIMIT),
        vec![(half * 2).to_be_bytes().to_vec()]
    );
}

fn validate_args_with_spec(arg_name: &str, type_name: &str) {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_argument_spec(arg_name, type_name, 4);
    t.add_load_rel(0, 4);
    t.add_discard(8);
    let path = t.write_to_file("test.bin");
    t.send_validate_args(&path, &seq_args_u32(&[42]));
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::AWAITING_CMD_RUN_VALIDATED);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_VALIDATE_ARGS, OK);
}

#[test]
fn cmd_validate_args_zero_length_arg_name() {
    validate_args_with_spec("", "U32");
}

#[test]
fn cmd_validate_args_zero_length_type_name() {
    validate_args_with_spec("arg1", "");
}

#[test]
fn cmd_validate_args_max_length_strings() {
    validate_args_with_spec(&"A".repeat(255), &"B".repeat(255));
}

#[test]
fn cmd_validate_args_size_mismatch() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_argument_spec("arg", "U64", 8);
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    t.send_validate_args(&path, &seq_args_u32(&[99]));
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_VALIDATE_ARGS, ERR);
    assert_eq!(t.seq_done(), vec![(0, 0, ERR)]);
    assert_eq!(t.all_events(), 1);
    assert_eq!(t.event_count(FpySequencer::EVENTID_ARG_SIZE_MISMATCH), 1);
}

#[test]
fn cmd_cancel() {
    let t = Tester::new();
    t.set_state(State::IDLE);
    t.send_cmd(FpySequencer::OPCODE_CANCEL, &[]);
    t.dispatch();
    t.assert_response(0, FpySequencer::OPCODE_CANCEL, ERR);
    t.dispatch_current();
    assert_eq!(t.state(), State::IDLE);
    t.clear_history();

    t.set_state(State::RUNNING_SLEEPING);
    t.send_cmd(FpySequencer::OPCODE_CANCEL, &[]);
    t.dispatch();
    // answered at once
    t.assert_response(0, FpySequencer::OPCODE_CANCEL, OK);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.event_count(FpySequencer::EVENTID_SEQUENCE_CANCELLED), 1);
    assert_eq!(t.st().tlm.sequences_cancelled, 1);
}

#[test]
fn cmd_clear_breakpoint() {
    let t = Tester::new();
    t.st().breakpoint.breakpoint_in_use = true;
    t.send_cmd(FpySequencer::OPCODE_CLEAR_BREAKPOINT, &[]);
    t.dispatch();
    t.assert_response(0, FpySequencer::OPCODE_CLEAR_BREAKPOINT, OK);
    // the event comes with the command, before the signal is handled
    assert_eq!(t.event_count(FpySequencer::EVENTID_BREAKPOINT_CLEARED), 1);
    assert!(t.st().breakpoint.breakpoint_in_use);
    t.dispatch();
    assert!(!t.st().breakpoint.breakpoint_in_use);
}

#[test]
fn cmd_set_breakpoint() {
    let t = Tester::new();
    t.send_set_breakpoint(123, true);
    t.dispatch();
    t.assert_response(0, FpySequencer::OPCODE_SET_BREAKPOINT, OK);
    t.dispatch();
    let breakpoint = t.st().breakpoint;
    assert!(breakpoint.breakpoint_in_use);
    assert!(breakpoint.break_only_once_on_breakpoint);
    assert_eq!(breakpoint.breakpoint_index, 123);
    assert_eq!(
        t.events(FpySequencer::EVENTID_BREAKPOINT_SET),
        vec![vec![0, 0, 0, 123, 0xFF]]
    );
}

#[test]
fn cmd_break() {
    let t = Tester::new();
    t.set_state(State::IDLE);
    t.send_cmd(FpySequencer::OPCODE_BREAK, &[]);
    t.dispatch_current();
    t.assert_response(0, FpySequencer::OPCODE_BREAK, ERR);
    assert!(!t.st().breakpoint.break_before_next_line);

    t.clear_history();
    t.set_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    t.send_cmd(FpySequencer::OPCODE_BREAK, &[]);
    t.dispatch_current();
    t.assert_response(0, FpySequencer::OPCODE_BREAK, OK);
    t.dispatch_current();
    assert!(t.st().breakpoint.break_before_next_line);

    t.st().breakpoint.break_before_next_line = false;
    t.clear_history();
    t.set_state(State::RUNNING_PAUSED);
    t.send_cmd(FpySequencer::OPCODE_BREAK, &[]);
    t.dispatch_current();
    t.assert_response(0, FpySequencer::OPCODE_BREAK, ERR);
    assert!(!t.st().breakpoint.break_before_next_line);
}

#[test]
fn cmd_continue() {
    let t = Tester::new();
    t.set_state(State::IDLE);
    t.send_cmd(FpySequencer::OPCODE_CONTINUE, &[]);
    t.dispatch();
    t.assert_response(0, FpySequencer::OPCODE_CONTINUE, ERR);
    t.clear_history();

    t.set_state(State::RUNNING_PAUSED);
    t.send_cmd(FpySequencer::OPCODE_CONTINUE, &[]);
    t.dispatch();
    t.assert_response(0, FpySequencer::OPCODE_CONTINUE, OK);
    t.dispatch();
    assert_eq!(t.state(), State::RUNNING_DISPATCH_STATEMENT);
}

#[test]
fn cmd_step() {
    let mut t = Tester::new();
    t.set_state(State::IDLE);
    t.send_cmd(FpySequencer::OPCODE_STEP, &[]);
    t.dispatch_current();
    t.assert_response(0, FpySequencer::OPCODE_STEP, ERR);

    t.clear_history();
    t.set_state(State::RUNNING_DISPATCH_STATEMENT);
    t.send_cmd(FpySequencer::OPCODE_STEP, &[]);
    t.dispatch_current();
    t.assert_response(0, FpySequencer::OPCODE_STEP, ERR);
    t.set_state(State::IDLE);

    t.clear_history();
    t.alloc_mem();
    t.add_no_op();
    t.add_no_op();
    t.add_no_op();
    t.write_and_run();
    // break before statement 0
    t.st().breakpoint.breakpoint_in_use = true;
    t.st().breakpoint.breakpoint_index = 0;
    t.dispatch_until_state(State::RUNNING_PAUSED);
    assert_eq!(
        t.events(FpySequencer::EVENTID_SEQUENCE_PAUSED),
        vec![vec![0, 0, 0, 0]]
    );

    t.send_cmd(FpySequencer::OPCODE_STEP, &[]);
    t.dispatch_until_state(State::RUNNING_DISPATCH_STATEMENT);
    t.assert_response(0, FpySequencer::OPCODE_STEP, OK);
    // and paused again before statement 1
    t.dispatch_until_state(State::RUNNING_PAUSED);
    assert_eq!(t.st().runtime.next_statement_index, 1);
    assert!(!t.st().breakpoint.break_before_next_line);

    t.clear_history();
    // step past the end
    t.st().runtime.next_statement_index = 3;
    t.send_cmd(FpySequencer::OPCODE_STEP, &[]);
    t.dispatch_until_state(State::RUNNING_DISPATCH_STATEMENT);
    t.assert_response(0, FpySequencer::OPCODE_STEP, OK);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.event_count(FpySequencer::EVENTID_SEQUENCE_DONE), 1);
}

#[test]
fn breakpoint_with_break_once_clears_itself() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_no_op();
    t.add_goto(0);
    t.write_and_run();
    {
        let mut st = t.st();
        st.breakpoint.breakpoint_in_use = true;
        st.breakpoint.breakpoint_index = 1;
        st.breakpoint.break_only_once_on_breakpoint = true;
    }
    t.dispatch_until_state(State::RUNNING_PAUSED);
    // `entered` with the breakOnce guard cleared it
    t.dispatch_current();
    assert!(!t.st().breakpoint.breakpoint_in_use);
    t.send_cmd(FpySequencer::OPCODE_CANCEL, &[]);
    t.dispatch_until_state(State::IDLE);
}

#[test]
fn cmd_dump_stack_to_file() {
    let mut t = Tester::new();
    t.set_state(State::IDLE);
    let out = t.path("output.bin");
    t.send_dump_stack_to_file(&out);
    t.dispatch_current();
    t.assert_response(0, FpySequencer::OPCODE_DUMP_STACK_TO_FILE, ERR);

    t.clear_history();
    t.alloc_mem();
    t.add_push_val(0x00u8);
    t.add_push_val(0x11u8);
    t.add_push_val(0x22u8);
    t.write_and_run();
    t.st().breakpoint.breakpoint_in_use = true;
    t.st().breakpoint.breakpoint_index = 3;
    t.dispatch_until_state(State::RUNNING_PAUSED);
    assert_eq!(t.stack_size(), 3);

    t.send_dump_stack_to_file(&out);
    t.dispatch_current();
    t.assert_response(0, FpySequencer::OPCODE_DUMP_STACK_TO_FILE, OK);
    assert_eq!(std::fs::read(&out).unwrap(), vec![0x00, 0x11, 0x22]);
}

#[test]
fn cmd_dump_stack_to_file_open_error_logs_dump_file_name() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(0x00u8);
    t.add_push_val(0x11u8);
    t.add_push_val(0x22u8);
    t.write_and_run();
    t.st().breakpoint.breakpoint_in_use = true;
    t.st().breakpoint.breakpoint_index = 3;
    t.dispatch_until_state(State::RUNNING_PAUSED);
    t.st().sequence_file_path.set("loaded_sequence.bin");
    t.clear_history();

    let bad = t.path("none/dump.bin");
    t.send_dump_stack_to_file(&bad);
    t.dispatch_current();
    t.assert_response(0, FpySequencer::OPCODE_DUMP_STACK_TO_FILE, ERR);
    let events = t.events(FpySequencer::EVENTID_FILE_OPEN_ERROR);
    assert_eq!(events.len(), 1);
    assert_eq!(first_string(&events[0]), bad);
}

#[test]
fn seq_base_dir_resolves_path() {
    // the base directory is prepended; telemetry, events and file I/O see
    // the resolved path
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    let dir = path.strip_suffix("/test.bin").unwrap().to_string();
    t.set_base_dir_param(&dir);
    t.clear_history();
    t.send_validate("test.bin");
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::AWAITING_CMD_RUN_VALIDATED);
    assert_eq!(t.st().sequence_file_path, path.as_str());
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_VALIDATE, OK);
}

#[test]
fn seq_base_dir_empty_keeps_raw_path() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    t.set_base_dir_param("");
    t.clear_history();
    t.send_validate(&path);
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::AWAITING_CMD_RUN_VALIDATED);
    assert_eq!(t.st().sequence_file_path, path.as_str());
}

#[test]
fn validate_empty_sequence_file_path() {
    let t = Tester::new();
    t.alloc_mem();
    t.set_base_dir_param("");
    t.clear_history();
    t.send_validate("");
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::IDLE);
    let events = t.events(FpySequencer::EVENTID_FILE_OPEN_ERROR);
    assert_eq!(events, vec![vec![0, 0, 0, 0, 0, 9]]);
    t.assert_response(0, FpySequencer::OPCODE_VALIDATE, ERR);
}

#[test]
fn seq_base_dir_file_open_logs_resolved_path() {
    let t = Tester::new();
    t.alloc_mem();
    t.set_base_dir_param("nonexistent_dir");
    t.clear_history();
    t.send_validate("test.bin");
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::IDLE);
    let events = t.events(FpySequencer::EVENTID_FILE_OPEN_ERROR);
    assert_eq!(events.len(), 1);
    assert_eq!(first_string(&events[0]), "nonexistent_dir/test.bin");
}

#[test]
fn seq_base_dir_path_too_long_truncates() {
    // "<base>/<file>" one character longer than FileNameStringSize; the
    // action is called directly since a command would truncate the name
    let t = Tester::new();
    let base_dir = "a".repeat(8);
    t.set_base_dir_param(&base_dir);
    t.clear_history();
    let file_name = "b".repeat(FILE_NAME_STRING_SIZE - 8);
    let args = SequenceExecutionArgs::new(
        FileNameString::from(file_name.as_str()),
        BlockState::Block,
        SeqArgs::default(),
    );
    Handlers::action_setSequenceFilePath(&*t.comp, 0, Signal::cmd_VALIDATE, &args);
    let events = t.events(FpySequencer::EVENTID_SEQUENCE_FILE_PATH_TOO_LONG);
    assert_eq!(events.len(), 1);
    assert_eq!(first_string(&events[0]), base_dir);
    assert_eq!(t.st().sequence_file_path.len(), FILE_NAME_STRING_SIZE);

    // one character shorter fits exactly and logs nothing
    t.clear_history();
    let file_name = "b".repeat(FILE_NAME_STRING_SIZE - 9);
    let args = SequenceExecutionArgs::new(
        FileNameString::from(file_name.as_str()),
        BlockState::Block,
        SeqArgs::default(),
    );
    Handlers::action_setSequenceFilePath(&*t.comp, 0, Signal::cmd_VALIDATE, &args);
    assert_eq!(t.all_events(), 0);
    assert_eq!(t.st().sequence_file_path.len(), FILE_NAME_STRING_SIZE);
}

use fprime_config::FILE_NAME_STRING_SIZE;

#[test]
fn prm_seq_base_dir_tlm() {
    // parameterUpdated echoes the value on its channel
    let t = Tester::new();
    t.set_base_dir_param("/seq");
    t.assert_response(0, FpySequencer::OPCODE_SEQ_BASE_DIR_PRM_SET, OK);
    assert_eq!(
        t.tlm(),
        vec![(
            FpySequencer::CHANID_PRM_SEQ_BASE_DIR,
            vec![0, 4, b'/', b's', b'e', b'q']
        )]
    );
    assert_eq!(
        t.comp.param_get_seq_base_dir(),
        (ParamString::from("/seq"), ParamValid::Valid)
    );
}

#[test]
fn prm_statement_timeout_secs_set_and_save() {
    let t = Tester::new();
    t.set_timeout_param(2.5);
    t.assert_response(0, FpySequencer::OPCODE_STATEMENT_TIMEOUT_SECS_PRM_SET, OK);
    assert_eq!(
        t.tlm(),
        vec![(
            FpySequencer::CHANID_PRM_STATEMENT_TIMEOUT_SECS,
            2.5f32.to_be_bytes().to_vec()
        )]
    );
    // the same value again: telemetry is "update on change"
    t.set_timeout_param(2.5);
    assert_eq!(t.tlm().len(), 1);
    // parameter commands are synchronous: nothing was queued
    assert_eq!(t.queued(), 0);

    t.send_cmd(FpySequencer::OPCODE_STATEMENT_TIMEOUT_SECS_PRM_SAVE, &[]);
    t.assert_response(2, FpySequencer::OPCODE_STATEMENT_TIMEOUT_SECS_PRM_SAVE, OK);
    t.send_cmd(FpySequencer::OPCODE_SEQ_BASE_DIR_PRM_SAVE, &[]);
    assert_eq!(
        *t.ground.prm_sets.lock().unwrap(),
        vec![
            (
                FpySequencer::PARAMID_STATEMENT_TIMEOUT_SECS,
                2.5f32.to_be_bytes().to_vec()
            ),
            (FpySequencer::PARAMID_SEQ_BASE_DIR, vec![0, 0]),
        ]
    );

    // a malformed value
    t.clear_history();
    t.send_cmd(FpySequencer::OPCODE_STATEMENT_TIMEOUT_SECS_PRM_SET, &[1, 2]);
    t.assert_response(
        0,
        FpySequencer::OPCODE_STATEMENT_TIMEOUT_SECS_PRM_SET,
        CmdResponse::FormatError,
    );
    // a base directory longer than a ParamString
    let mut long = vec![0, 41];
    long.extend([b'x'; 41]);
    t.send_cmd(FpySequencer::OPCODE_SEQ_BASE_DIR_PRM_SET, &long);
    t.assert_response(
        1,
        FpySequencer::OPCODE_SEQ_BASE_DIR_PRM_SET,
        CmdResponse::FormatError,
    );
}

#[test]
fn load_parameters_installs_defaults_and_echoes_them() {
    let t = Tester::new();
    assert_eq!(
        t.comp.param_get_statement_timeout_secs(),
        (0.0, ParamValid::Uninit)
    );
    // the tester's parameter database knows nothing
    t.comp.load_parameters();
    assert_eq!(
        t.comp.param_get_statement_timeout_secs(),
        (0.0, ParamValid::Default)
    );
    assert_eq!(t.comp.param_get_seq_base_dir().1, ParamValid::Default);
    assert_eq!(
        t.tlm(),
        vec![
            (FpySequencer::CHANID_PRM_STATEMENT_TIMEOUT_SECS, vec![0; 4]),
            (FpySequencer::CHANID_PRM_SEQ_BASE_DIR, vec![0, 0]),
        ]
    );
}

#[test]
fn check_timers() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(10u32);
    t.add_push_val(0u32);
    t.add_wait_rel();
    let path = t.write_to_file("test.bin");
    t.send_run(&path, BlockState::Block);
    t.set_time(Time::from_seconds_useconds(0, 0));
    t.dispatch_until_state(State::RUNNING_SLEEPING);

    t.set_time(Time::from_seconds_useconds(5, 0));
    t.check_timers();
    t.dispatch_current();
    assert_eq!(t.state(), State::RUNNING_SLEEPING);

    t.set_time(Time::from_seconds_useconds(10, 0));
    t.check_timers();
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_RUN, OK);
    t.clear_history();

    // now with a statement timeout
    t.set_timeout_param(5.0);
    t.clear_history();
    t.send_run(&path, BlockState::Block);
    t.dispatch_until_state(State::RUNNING_SLEEPING);
    t.set_time(Time::from_seconds_useconds(12, 0));
    t.check_timers();
    t.dispatch_current();
    assert_eq!(t.state(), State::RUNNING_SLEEPING);
    t.set_time(Time::from_seconds_useconds(15, 0));
    t.check_timers();
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN, ERR);
    // WAIT_REL (1) at index 2
    assert_eq!(
        t.events(FpySequencer::EVENTID_DIRECTIVE_TIMED_OUT)[0][..5],
        [1, 0, 0, 0, 2]
    );
}

#[test]
fn ping() {
    let t = Tester::new();
    let p = t.comp.ping_in(0);
    p.target.invoke(p.port_num, 0x1234);
    t.dispatch();
    assert_eq!(*t.ground.pings.lock().unwrap(), vec![0x1234]);
}

#[test]
fn cmd_response() {
    let mut t = Tester::new();
    t.cmd_response_in(0, 0, OK);
    t.dispatch();
    assert_eq!(
        t.event_count(FpySequencer::EVENTID_CMD_RESPONSE_WHILE_NOT_RUNNING_SEQUENCE),
        1
    );
    t.clear_history();

    t.alloc_mem();
    t.add_const_cmd(123);
    let path = t.write_to_file("test.bin");
    let start = |t: &Tester| {
        t.st().sequences_started = 255;
        t.st().statements_dispatched = 255;
        t.send_run(&path, BlockState::Block);
        t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    };
    start(&t);
    // the CONST_CMD directive, which sends the command
    t.dispatch();
    assert_eq!(t.cmd_out()[0].1, 0x0100_0100);
    // sequence 256, statement 256
    t.cmd_response_in(123, 0x0100_0100, OK);
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_RUN, OK);
    // the response code was pushed for the sequence to branch on
    assert_eq!(t.stack_size(), 1);
    assert_eq!(t.byte(0), OK.as_repr());
    t.clear_history();

    // the wrong statement index fails the sequence
    start(&t);
    t.cmd_response_in(123, 0x0100_00FF, ERR);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN, ERR);
    t.clear_history();

    // an old sequence's response is only reported
    start(&t);
    t.cmd_response_in(123, 0x00FF_0100, OK);
    t.dispatch_current();
    assert_eq!(t.state(), State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    // right sequence, wrong statement
    t.cmd_response_in(123, 0x0100_0101, OK);
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_RUN, ERR);
    assert_eq!(
        t.events(FpySequencer::EVENTID_CMD_RESPONSE_FROM_OLD_SEQUENCE),
        vec![vec![0, 0, 0, 123, 0, 0, 0xFF, 0x01, 0x00]]
    );
    assert_eq!(
        t.events(FpySequencer::EVENTID_WRONG_CMD_RESPONSE_INDEX),
        vec![vec![0, 0, 0, 123, 0, 0x01, 0x01, 0x01, 0x00]]
    );
    t.clear_history();

    // a response from this sequence while sleeping
    t.clear_seq();
    t.add_push_val(10u32);
    t.add_push_val(0u32);
    t.add_wait_rel();
    t.add_const_cmd(123);
    let path = t.write_to_file("test.bin");
    t.st().sequences_started = 255;
    t.st().statements_dispatched = 255;
    t.send_run(&path, BlockState::Block);
    t.dispatch_until_state(State::RUNNING_SLEEPING);
    t.cmd_response_in(123, 0x0100_0100, OK);
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_RUN, ERR);
    assert_eq!(
        t.event_count(FpySequencer::EVENTID_CMD_RESPONSE_WHILE_NOT_AWAITING),
        1
    );
    t.clear_history();

    // the wrong opcode
    t.clear_seq();
    t.add_const_cmd(123);
    let path = t.write_to_file("test.bin");
    t.st().sequences_started = 255;
    t.st().statements_dispatched = 255;
    t.send_run(&path, BlockState::Block);
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    t.cmd_response_in(456, 0x0100_0100, OK);
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_RUN, ERR);
    assert_eq!(
        t.events(FpySequencer::EVENTID_WRONG_CMD_RESPONSE_OPCODE),
        vec![vec![0, 0, 1, 0xC8, 0, 0, 0, 0, 123]]
    );
}

#[test]
fn cmd_response_while_awaiting_a_directive_fails_the_sequence() {
    // a NO_OP's response arrives before any command response could, so
    // set the awaiting state up by hand
    let t = Tester::new();
    {
        let mut st = t.st();
        st.sequences_started = 1;
        st.statements_dispatched = 1;
        st.runtime.current_statement_opcode = DirectiveId::NoOp.as_repr();
    }
    t.set_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    t.cmd_response_in(5, 0x0001_0001, OK);
    t.dispatch();
    assert_eq!(
        t.events(FpySequencer::EVENTID_CMD_RESPONSE_WHILE_AWAITING_DIRECTIVE),
        vec![vec![0, 0, 0, 5, 0, DirectiveId::NoOp.as_repr()]]
    );
    // stmtResponse_unexpected fails the sequence
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.seq_done(), vec![(0, 0, ERR)]);
}

#[test]
fn tlm_write() {
    let t = Tester::new();
    let p = t.comp.tlm_write_in(0);
    p.target.invoke(p.port_num, 0);
    t.dispatch();
    // every channel but the two parameter echoes
    let tlm = t.tlm();
    assert_eq!(tlm.len(), 20);
    let channels: Vec<_> = tlm.iter().map(|(id, _)| *id).collect();
    for id in 0..20 {
        assert!(channels.contains(&id), "channel {id}");
    }
    // State is IDLE (2)
    assert!(tlm.contains(&(FpySequencer::CHANID_STATE, vec![0, 0, 0, 2])));
    // update on change: nothing changed, nothing written
    t.clear_history();
    p.target.invoke(p.port_num, 0);
    t.dispatch();
    assert!(t.tlm().is_empty());
}

#[test]
fn tlm_write_describes_the_next_statement_while_paused() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(7u8);
    t.add_const_cmd(0x42);
    t.write_and_run();
    t.st().breakpoint.breakpoint_in_use = true;
    t.st().breakpoint.breakpoint_index = 1;
    t.dispatch_until_state(State::RUNNING_PAUSED);
    // the PAUSED entry's `entered` signal
    t.dispatch_current();
    t.clear_history();
    let p = t.comp.tlm_write_in(0);
    p.target.invoke(p.port_num, 0);
    t.dispatch();
    let tlm = t.tlm();
    let value = |id| tlm.iter().find(|(c, _)| *c == id).unwrap().1.clone();
    assert_eq!(value(FpySequencer::CHANID_STATE), vec![0, 0, 0, 5]);
    assert_eq!(
        value(FpySequencer::CHANID_DEBUG_NEXT_STATEMENT_OPCODE),
        vec![DirectiveId::ConstCmd.as_repr()]
    );
    assert_eq!(
        value(FpySequencer::CHANID_DEBUG_NEXT_CMD_OPCODE),
        vec![0, 0, 0, 0x42]
    );
    assert_eq!(
        value(FpySequencer::CHANID_DEBUG_NEXT_STATEMENT_INDEX),
        vec![0, 0, 0, 1]
    );
    assert_eq!(
        value(FpySequencer::CHANID_DEBUG_STACK_SIZE),
        vec![0, 0, 0, 1]
    );
    assert_eq!(
        value(FpySequencer::CHANID_DEBUG_NEXT_STATEMENT_READ_SUCCESS),
        vec![0xFF]
    );
    assert_eq!(
        value(FpySequencer::CHANID_BREAKPOINT_INDEX),
        vec![0, 0, 0, 1]
    );
    let mut seq_path = vec![0, t.st().sequence_file_path.len() as u8];
    seq_path.extend_from_slice(t.st().sequence_file_path.as_bytes());
    assert_eq!(value(FpySequencer::CHANID_SEQ_PATH), seq_path);

    // at the end of the file
    t.st().runtime.next_statement_index = 2;
    t.clear_history();
    p.target.invoke(p.port_num, 0);
    t.dispatch();
    let tlm = t.tlm();
    assert!(tlm.contains(&(FpySequencer::CHANID_DEBUG_REACHED_END_OF_FILE, vec![0xFF])));
    assert!(tlm.contains(&(
        FpySequencer::CHANID_DEBUG_NEXT_STATEMENT_READ_SUCCESS,
        vec![0x00]
    )));
}

#[test]
fn seq_run_in() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_no_op();
    let path = t.write_to_file("test.bin");
    t.seq_run_in(&path, &SeqArgs::default());
    t.dispatch();
    t.dispatch_until_state(State::VALIDATING);
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(
        t.seq_start(),
        vec![(path.as_bytes().to_vec(), SeqArgs::default())]
    );
    assert_eq!(t.seq_done(), vec![(0, 0, OK)]);
    // never blocking: no command response
    assert!(t.responses().is_empty());
    t.clear_history();

    t.set_state(State::RUNNING_DISPATCH_STATEMENT);
    t.seq_run_in(&path, &SeqArgs::default());
    t.dispatch();
    assert_eq!(t.event_count(FpySequencer::EVENTID_INVALID_SEQ_RUN_CALL), 1);
}

#[test]
fn seq_cancel_in() {
    let t = Tester::new();
    t.set_state(State::IDLE);
    t.seq_cancel_in();
    t.dispatch();
    assert_eq!(
        t.event_count(FpySequencer::EVENTID_INVALID_SEQ_CANCEL_CALL),
        1
    );
    t.dispatch_current();
    assert_eq!(t.state(), State::IDLE);
    t.clear_history();

    t.set_state(State::RUNNING_SLEEPING);
    t.seq_cancel_in();
    t.dispatch();
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.event_count(FpySequencer::EVENTID_SEQUENCE_CANCELLED), 1);
    assert_eq!(t.seq_done(), vec![(0, 0, ERR)]);
}

#[test]
fn seq_run_in_args() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_argument_spec("arg1", "U32", 4);
    t.add_argument_spec("arg2", "U32", 4);
    t.add_load_rel(0, 4);
    t.add_load_rel(4, 4);
    t.add_discard(16);
    let path = t.write_to_file("test.bin");
    let args = seq_args_u32(&[10, 20]);
    t.seq_run_in(&path, &args);
    t.dispatch_until_state(State::VALIDATING);
    assert_eq!(t.st().sequences_started, 0);
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    assert_eq!(t.st().sequences_started, 1);
    assert_eq!(t.stack_size(), 8);
    t.dispatch_until_state(State::IDLE);
    assert_eq!(t.st().statements_dispatched, 3);
    assert_eq!(t.seq_start(), vec![(path.as_bytes().to_vec(), args)]);
    assert_eq!(t.seq_done(), vec![(0, 0, OK)]);
    assert_eq!(t.stack_size(), 0);
}

// -- the generated-code contract ---------------------------------------------

#[test]
fn reg_commands_registers_every_opcode() {
    let t = Tester::new();
    t.comp.reg_commands();
    assert_eq!(*t.ground.regs.lock().unwrap(), (0..16).collect::<Vec<_>>());
}

#[test]
fn ids_match_the_fpp_model() {
    // From `fpp-to-rust --dict` on Svc/FpySequencer at the upstream pin:
    // commands 0..=11 then the implicit parameter commands, 45 events and
    // 22 channels in declaration order, two parameters.
    assert_eq!(FpySequencer::OPCODE_DUMP_STACK_TO_FILE, 11);
    assert_eq!(FpySequencer::OPCODE_SEQ_BASE_DIR_PRM_SAVE, 15);
    assert_eq!(FpySequencer::EVENTID_SEQUENCE_DONE, 0x0F);
    assert_eq!(FpySequencer::EVENTID_DIRECTIVE_DESERIALIZE_ERROR, 0x19);
    assert_eq!(
        FpySequencer::EVENTID_ARG_TOTAL_SIZE_EXCEEDS_STACK_LIMIT,
        0x22
    );
    assert_eq!(FpySequencer::EVENTID_LOG_DIAGNOSTIC, 0x2C);
    assert_eq!(FpySequencer::CHANID_SEQ_PATH, 9);
    assert_eq!(FpySequencer::CHANID_BREAK_BEFORE_NEXT_LINE, 0x13);
    assert_eq!(FpySequencer::CHANID_PRM_SEQ_BASE_DIR, 0x15);
    assert_eq!(FpySequencer::PARAMID_SEQ_BASE_DIR, 1);
    // the enums' values
    assert_eq!(DirectiveId::Fabs.as_repr(), 81);
    assert_eq!(DirectiveId::PopSerializable.as_repr(), 78);
    assert_eq!(DirectiveErrorCode::SerialPortInvalidIndex.as_repr(), 19);
    assert_eq!(GoalState::Idle.as_repr(), 2);
    assert_eq!(FileReadStage::Footer.as_repr(), 2);
    assert_eq!(SCHEMA_VERSION, 7);
}

#[test]
fn command_arguments_are_checked() {
    let t = Tester::new();
    // an undeclared BlockState
    let mut args = vec![0, 1, b'x'];
    args.push(7);
    t.send_cmd(FpySequencer::OPCODE_RUN, &args);
    t.dispatch();
    t.assert_response(0, FpySequencer::OPCODE_RUN, CmdResponse::ValidationError);
    // a leftover byte
    t.send_cmd(FpySequencer::OPCODE_CANCEL, &[0]);
    t.dispatch();
    t.assert_response(1, FpySequencer::OPCODE_CANCEL, CmdResponse::FormatError);
    // a missing argument
    t.send_cmd(FpySequencer::OPCODE_SET_BREAKPOINT, &[0, 0, 0, 1]);
    t.dispatch();
    t.assert_response(
        2,
        FpySequencer::OPCODE_SET_BREAKPOINT,
        CmdResponse::FormatError,
    );
    // a file name longer than a command string
    let mut long = vec![0, 41];
    long.extend([b'x'; 41]);
    t.send_cmd(FpySequencer::OPCODE_VALIDATE, &long);
    t.dispatch();
    t.assert_response(3, FpySequencer::OPCODE_VALIDATE, CmdResponse::FormatError);
    // an unknown opcode, answered at once
    t.send_cmd(99, &[]);
    t.assert_response(4, 99, CmdResponse::InvalidOpcode);
    assert_eq!(t.state(), State::IDLE);
}

#[test]
fn queue_priorities_follow_the_fpp_model() {
    // CANCEL (8) overtakes an earlier BREAK (7); a state machine signal (9)
    // overtakes both; checkTimers (4) and tlmWrite (1) come last
    let t = Tester::new();
    let p = t.comp.tlm_write_in(0);
    p.target.invoke(p.port_num, 0);
    t.check_timers();
    t.send_cmd(FpySequencer::OPCODE_BREAK, &[]);
    t.send_cmd(FpySequencer::OPCODE_CANCEL, &[]);
    t.send_cmd(FpySequencer::OPCODE_CLEAR_BREAKPOINT, &[]);
    t.dispatch(); // CANCEL
    t.assert_response(0, FpySequencer::OPCODE_CANCEL, ERR);
    t.dispatch(); // BREAK
    t.assert_response(1, FpySequencer::OPCODE_BREAK, ERR);
    t.dispatch(); // CLEAR_BREAKPOINT, which sends a signal
    t.assert_response(2, FpySequencer::OPCODE_CLEAR_BREAKPOINT, OK);
    let mut bytes = vec![0u8; QUEUE_MSG_SIZE];
    let mut actual: FwSizeType = 0;
    let mut priority: FwQueuePriorityType = 0;
    let status = t.comp.active.queued.queue().receive(
        &mut bytes,
        BlockingType::NonBlocking,
        &mut actual,
        &mut priority,
    );
    assert_eq!(status, fprime_os::queue::Status::OpOk);
    // [INTERNAL_STATE_MACHINE_SIGNAL][port 0][SmId 0][cmd_CLEAR_BREAKPOINT 4]
    assert_eq!(priority, 9);
    assert_eq!(
        &bytes[..actual as usize],
        &[
            0,
            0,
            0,
            FpySequencer::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL as u8,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            4
        ]
    );
    let mut bytes = vec![0u8; QUEUE_MSG_SIZE];
    let _ = t.comp.active.queued.queue().receive(
        &mut bytes,
        BlockingType::NonBlocking,
        &mut actual,
        &mut priority,
    );
    assert_eq!(priority, 4);
    let _ = t.comp.active.queued.queue().receive(
        &mut bytes,
        BlockingType::NonBlocking,
        &mut actual,
        &mut priority,
    );
    assert_eq!(priority, 1);
}

#[test]
fn run_signal_carries_the_execution_args() {
    // cmd_RUN: [type][port][SmId][6][path: u16 len + bytes][block u8]
    // [SeqArgs: u64 size + 12 bytes]
    let t = Tester::new();
    t.send_run("/a", BlockState::NoBlock);
    t.dispatch();
    let mut bytes = vec![0u8; QUEUE_MSG_SIZE];
    let mut actual: FwSizeType = 0;
    let mut priority: FwQueuePriorityType = 0;
    let _ = t.comp.active.queued.queue().receive(
        &mut bytes,
        BlockingType::NonBlocking,
        &mut actual,
        &mut priority,
    );
    let mut expected = vec![
        0,
        0,
        0,
        FpySequencer::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL as u8,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        6,
        0,
        2,
        b'/',
        b'a',
        1,
    ];
    expected.extend([0u8; 8 + 12]);
    assert_eq!(&bytes[..actual as usize], &expected[..]);
}
