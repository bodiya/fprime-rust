//! Whole sequences through the state machine and the internal ports
//! (upstream `FpySequencerTestSequences.cpp` and the `Integration*`
//! tests), and the component on its own thread.

use super::*;

fn run_to_idle(t: &mut Tester) {
    t.write_and_run();
    t.dispatch_until_state(State::IDLE);
}

fn assert_run_ok(t: &Tester) {
    assert_eq!(t.responses().len(), 1);
    t.assert_response(0, FpySequencer::OPCODE_RUN, CmdResponse::Ok);
}

fn set_next_tlm(t: &Tester, id: FwChanIdType, value: &[u8]) {
    *t.data.next_tlm_id.lock().unwrap() = id;
    let mut buf = TlmBuffer::new();
    assert!(buf.set_buff(value).is_ok());
    *t.data.next_tlm_value.lock().unwrap() = buf;
}

#[test]
fn empty() {
    let mut t = Tester::new();
    t.alloc_mem();
    run_to_idle(&mut t);
    assert_run_ok(&t);
    assert_eq!(t.st().tlm.sequences_succeeded, 1);
}

#[test]
fn full() {
    let mut t = Tester::new();
    t.alloc_mem();
    for _ in 0..fprime_config::fpy_sequencer::MAX_SEQUENCE_STATEMENT_COUNT {
        t.add_no_op();
    }
    t.write_and_run();
    t.dispatch_until_state_bounded(
        State::IDLE,
        fprime_config::fpy_sequencer::MAX_SEQUENCE_STATEMENT_COUNT as u32 * 4,
    );
    assert_run_ok(&t);
    assert_eq!(t.st().statements_dispatched, 2048);
}

#[test]
fn complex_control_flow() {
    let mut t = Tester::new();
    t.alloc_mem();
    set_next_tlm(&t, 123, &[0xFF]);
    t.add_push_tlm_val(123);
    t.add_if(4);
    // true: exit(0)
    t.add_push_val(0i32);
    t.add_exit();
    // false: exit(1)
    t.add_push_val(1i32);
    t.add_exit();
    run_to_idle(&mut t);
    assert_eq!(t.st().tlm.last_directive_error, DirectiveError::NoError);
    assert_eq!(t.st().statements_dispatched, 4);

    set_next_tlm(&t, 123, &[0x00]);
    t.st().statements_dispatched = 0;
    t.clear_history();
    run_to_idle(&mut t);
    assert_eq!(
        t.st().tlm.last_directive_error,
        DirectiveError::ExitWithError
    );
    assert_eq!(t.st().statements_dispatched, 4);
    t.assert_response(0, FpySequencer::OPCODE_RUN, CmdResponse::ExecutionError);
    // where it failed: EXIT at index 5
    assert_eq!(t.st().tlm.directive_error_index, 5);
    assert_eq!(t.st().tlm.directive_error_id, DirectiveId::Exit);
}

#[test]
fn or_of_tlm_and_reg() {
    let mut t = Tester::new();
    t.alloc_mem();
    set_next_tlm(&t, 123, &[0xFF]);
    t.add_push_tlm_val(123);
    t.add_push_val(0u8);
    t.add_stack_op(DirectiveId::Or);
    t.add_if(6);
    t.add_push_val(0i32);
    t.add_exit();
    t.add_push_val(1i32);
    t.add_exit();
    run_to_idle(&mut t);
    assert_eq!(t.st().tlm.last_directive_error, DirectiveError::NoError);
    assert_eq!(t.st().statements_dispatched, 6);

    set_next_tlm(&t, 123, &[0x00]);
    t.st().statements_dispatched = 0;
    run_to_idle(&mut t);
    assert_eq!(
        t.st().tlm.last_directive_error,
        DirectiveError::ExitWithError
    );
    assert_eq!(t.st().statements_dispatched, 6);
}

#[test]
fn cmp_int_tlm() {
    let mut t = Tester::new();
    t.alloc_mem();
    set_next_tlm(&t, 123, &999u64.to_be_bytes());
    t.add_push_tlm_val(123);
    t.add_push_val(999u64);
    t.add_stack_op(DirectiveId::Uge);
    t.add_if(6);
    t.add_push_val(0i32);
    t.add_exit();
    t.add_push_val(1i32);
    t.add_exit();
    run_to_idle(&mut t);
    assert_eq!(t.st().tlm.last_directive_error, DirectiveError::NoError);
    assert_eq!(t.st().statements_dispatched, 6);

    set_next_tlm(&t, 123, &998u64.to_be_bytes());
    t.st().statements_dispatched = 0;
    run_to_idle(&mut t);
    assert_eq!(
        t.st().tlm.last_directive_error,
        DirectiveError::ExitWithError
    );
    assert_eq!(t.st().statements_dispatched, 6);
}

#[test]
fn not_true_seq() {
    // NOT of 255 is false
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(255u8);
    t.add_stack_op(DirectiveId::Not);
    t.add_if(5);
    t.add_push_val(1i32);
    t.add_exit();
    t.add_push_val(0i32);
    t.add_exit();
    run_to_idle(&mut t);
    assert_eq!(t.st().tlm.last_directive_error, DirectiveError::NoError);
    assert_run_ok(&t);
}

#[test]
fn integration_goto() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_goto(1);
    t.add_no_op();
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_wait_abs() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(TimeBase::TbWorkstationTime.as_repr());
    t.add_push_val(0u8);
    t.add_push_val(0u32);
    t.add_push_val(0u32);
    t.add_wait_abs();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 1, 0));
    t.write_and_run();
    t.dispatch_until_state(State::RUNNING_SLEEPING);
    t.check_timers();
    t.dispatch_until_state(State::IDLE);
    assert_run_ok(&t);
}

#[test]
fn integration_wait_abs_on_another_time_base_fails() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(TimeBase::TbScTime.as_repr());
    t.add_push_val(0u8);
    t.add_push_val(0u32);
    t.add_push_val(0u32);
    t.add_wait_abs();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 1, 0));
    t.write_and_run();
    t.dispatch_until_state(State::RUNNING_SLEEPING);
    t.check_timers();
    t.dispatch_until_state(State::IDLE);
    t.assert_response(0, FpySequencer::OPCODE_RUN, CmdResponse::ExecutionError);
    assert_eq!(t.event_count(FpySequencer::EVENTID_MISMATCHED_TIME_BASE), 1);
}

#[test]
fn integration_allocate_discard() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_allocate(8);
    t.add_discard(8);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_store_rel_const_offset_load_rel() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_allocate(4);
    t.add_push_val(0x42u8);
    t.add_store_rel_const_offset(0, 1);
    t.add_load_rel(0, 1);
    t.add_discard(5);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_store_rel() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_allocate(4);
    t.add_push_val(0x42u8);
    t.add_push_val(0i32);
    t.add_store_rel(1);
    t.add_discard(4);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_mem_cmp() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(0x42u8);
    t.add_push_val(0x42u8);
    t.add_memcmp(1);
    t.add_discard(1);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_push_time() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_time();
    t.add_discard(Time::SERIALIZED_SIZE as StackSizeType);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_push_rand() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_rand();
    t.add_discard(4);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_set_seed_push_rand() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(123_456_789u32);
    t.add_set_seed();
    t.add_push_rand();
    run_to_idle(&mut t);
    assert_run_ok(&t);
    // the value std::mt19937 seeded with 123456789 produces first
    assert_eq!(t.stack_size(), 4);
    assert_eq!(t.pop::<u32>(), 2_288_500_408);
}

#[test]
fn integration_get_field() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(0xAAu8);
    t.add_push_val(0xBBu8);
    t.add_push_val(0u32);
    t.add_get_field(2, 1);
    t.add_discard(1);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_peek() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(0xAAu8);
    t.add_push_val(1u32);
    t.add_push_val(0u32);
    t.add_peek();
    t.add_discard(2);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_call_return() {
    // 0 -> 1 (CALL) -> 4 (RETURN) -> 2 (GOTO 5) -> end
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(4u32);
    t.add_call();
    t.add_goto(5);
    t.add_no_op();
    t.add_return(0, 0);
    run_to_idle(&mut t);
    assert_run_ok(&t);
    assert_eq!(t.st().statements_dispatched, 4);
}

#[test]
fn integration_call_with_an_argument_and_a_return_value() {
    // f(x) = x * 3, called with 14: the result replaces the argument
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(14i64); // 0: argument
    t.add_push_val(4u32); // 1: target
    t.add_call(); // 2
    t.add_goto(9); // 3: after the return
    t.add_load_rel(-16, 8); // 4: the argument, below the saved frame
    t.add_push_val(3i64); // 5
    t.add_stack_op(DirectiveId::Mul); // 6
    t.add_return(8, 8); // 7: 8-byte result, 8 bytes of arguments
    t.add_no_op(); // 8: never reached
    run_to_idle(&mut t);
    assert_run_ok(&t);
    assert_eq!(t.stack_size(), 8);
    assert_eq!(t.pop::<i64>(), 42);
    assert_eq!(t.st().runtime.stack.current_frame_start, 0);
}

#[test]
fn integration_store_abs_const_offset_load_abs() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_allocate(4);
    t.add_push_val(0x42u8);
    t.add_store_abs_const_offset(0, 1);
    t.add_load_abs(0, 1);
    t.add_discard(5);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_store_abs() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_allocate(4);
    t.add_push_val(0x42u8);
    t.add_push_val(0u32);
    t.add_store_abs(1);
    t.add_discard(4);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_push_prm() {
    let mut t = Tester::new();
    t.alloc_mem();
    *t.data.next_prm_id.lock().unwrap() = 456;
    assert!(
        t.data
            .next_prm_value
            .lock()
            .unwrap()
            .set_buff(&[0xAA, 0xBB])
            .is_ok()
    );
    t.add_push_prm(456);
    t.add_discard(2);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_push_tlm_val_and_time() {
    let mut t = Tester::new();
    t.alloc_mem();
    set_next_tlm(&t, 456, &[0xCC]);
    *t.data.next_tlm_time.lock().unwrap() = Time::from_seconds_useconds(888, 777);
    t.add_push_tlm_val_and_time(456);
    t.add_discard(1 + Time::SERIALIZED_SIZE as StackSizeType);
    run_to_idle(&mut t);
    assert_run_ok(&t);
}

#[test]
fn integration_stack_cmd() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(123u32);
    t.add_stack_cmd(0);
    t.write_and_run();
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    // the PUSH_VAL directive; the STACK_CMD statement follows (upstream
    // sends the response now and relies on the queue priorities: the
    // command goes out before the response, priority 5, is handled)
    t.dispatch();
    assert_eq!(t.state(), State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    // sequence 1, statement 2
    t.cmd_response_in(123, (1 << 16) | 2, CmdResponse::Ok);
    t.dispatch_until_state(State::IDLE);
    assert_run_ok(&t);
    assert_eq!(t.cmd_out(), vec![(vec![0, 0, 0, 0, 0, 123], (1 << 16) | 2)]);
}

#[test]
fn integration_pop_event_and_pop_serializable() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(LogSeverity::WarningLo.as_repr());
    for b in b"hi" {
        t.add_push_val(*b);
    }
    t.add_push_val(2u32);
    t.add_pop_event();
    t.add_push_val(0xBEEFu16);
    t.add_pop_serializable(3, 2);
    run_to_idle(&mut t);
    assert_run_ok(&t);
    let events = t.events(FpySequencer::EVENTID_LOG_WARNING_LO);
    assert_eq!(events.len(), 1);
    let path = t.path("test.bin");
    assert_eq!(first_string(&events[0]), path);
    assert_eq!(&events[0][2 + path.len()..], &[0, 2, b'h', b'i']);
    assert_eq!(t.serial_calls(), vec![(3, vec![0xBE, 0xEF])]);
}

#[test]
fn a_failing_directive_fails_the_sequence() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_push_val(1i64);
    t.add_push_val(0i64);
    t.add_stack_op(DirectiveId::Sdiv);
    t.add_no_op();
    run_to_idle(&mut t);
    t.assert_response(0, FpySequencer::OPCODE_RUN, CmdResponse::ExecutionError);
    assert_eq!(t.seq_done(), vec![(0, 0, CmdResponse::ExecutionError)]);
    let st = t.st();
    assert_eq!(st.tlm.last_directive_error, DirectiveError::DomainError);
    assert_eq!(st.tlm.directive_error_id, DirectiveId::Sdiv);
    assert_eq!(st.tlm.directive_error_index, 2);
    // the NO_OP was never dispatched
    assert_eq!(st.statements_dispatched, 3);
}

#[test]
fn an_undecodable_statement_fails_the_sequence() {
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_no_op();
    t.add_directive(DirectiveId::Goto, &[0, 1]);
    run_to_idle(&mut t);
    t.assert_response(0, FpySequencer::OPCODE_RUN, CmdResponse::ExecutionError);
    assert_eq!(
        t.event_count(FpySequencer::EVENTID_DIRECTIVE_DESERIALIZE_ERROR),
        1
    );
    assert_eq!(t.st().statements_dispatched, 1);
}

#[test]
fn cancel_while_waiting_for_a_command() {
    // the command's late response is then only reported
    let mut t = Tester::new();
    t.alloc_mem();
    t.add_const_cmd(77);
    t.write_and_run();
    t.dispatch_until_state(State::RUNNING_AWAITING_STATEMENT_RESPONSE);
    t.dispatch();
    let uid = t.cmd_out()[0].1;
    t.send_cmd(FpySequencer::OPCODE_CANCEL, &[]);
    t.dispatch_until_state(State::IDLE);
    // CANCEL is answered; the blocked RUN fails
    assert_eq!(
        t.responses(),
        vec![
            (FpySequencer::OPCODE_CANCEL, 0, CmdResponse::Ok),
            (FpySequencer::OPCODE_RUN, 0, CmdResponse::ExecutionError),
        ]
    );
    t.cmd_response_in(77, uid, CmdResponse::Ok);
    t.dispatch();
    assert_eq!(
        t.event_count(FpySequencer::EVENTID_CMD_RESPONSE_WHILE_NOT_RUNNING_SEQUENCE),
        1
    );
    assert_eq!(t.state(), State::IDLE);
}

// -- on the component thread ---------------------------------------------------

/// `cmdOut` wired back to `cmdResponseIn`, like a command dispatcher that
/// executes every command successfully.
struct Loopback {
    comp: std::sync::OnceLock<Arc<FpySequencer>>,
    sent: Mutex<Vec<FwOpcodeType>>,
}

impl ComPort for Loopback {
    fn invoke(&self, _port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
        let bytes = data.as_slice();
        let opcode = u32::from_be_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
        self.sent.lock().unwrap().push(opcode);
        let comp = self.comp.get().unwrap();
        let p = comp.cmd_response_in(0);
        p.target
            .invoke(p.port_num, opcode, context, CmdResponse::Ok);
    }
}

#[test]
fn a_sequence_runs_on_the_component_thread() {
    let mut builder = Tester::new();
    builder.add_const_cmd(0x10); // 0: pushes the response code (U8)
    builder.add_stack_op(DirectiveId::Ziext8To64); // 1
    builder.add_push_val(0u64); // 2: OK
    builder.add_stack_op(DirectiveId::Ieq); // 3
    builder.add_if(6); // 4
    builder.add_goto(8); // 5: OK, skip the exit
    builder.add_push_val(1i32); // 6
    builder.add_exit(); // 7
    builder.add_push_val(0x20u32); // 8
    builder.add_stack_cmd(0); // 9
    let path = builder.write_to_file("threaded.bin");

    let ground = Arc::new(Ground::default());
    let loopback = Arc::new(Loopback {
        comp: std::sync::OnceLock::new(),
        sent: Mutex::new(Vec::new()),
    });
    let comp = FpySequencer::new("fpyThread");
    comp.active.queued.base.set_id_base(ID_BASE);
    comp.cmd.cmd_response_out.connect(ground.clone(), 0);
    comp.evt.log_out.connect(ground.clone(), 0);
    comp.evt.time_out.connect(ground.clone(), 0);
    comp.cmd_out.connect(loopback.clone(), 0);
    assert!(loopback.comp.set(comp.clone()).is_ok());
    comp.init(10);
    comp.allocate_buffer(0, 4096);
    comp.active.start(
        &comp,
        100,
        fprime_os::task::TASK_DEFAULT,
        fprime_os::task::TASK_DEFAULT,
    );

    let mut args = vec![0, path.len() as u8];
    args.extend_from_slice(path.as_bytes());
    args.push(BlockState::Block.as_repr());
    let mut buf = CmdArgBuffer::new();
    assert!(buf.set_buff(&args).is_ok());
    let p = comp.cmd_in(0);
    p.target
        .invoke(p.port_num, ID_BASE + FpySequencer::OPCODE_RUN, 7, &mut buf);

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    while ground.responses.lock().unwrap().is_empty() {
        assert!(std::time::Instant::now() < deadline, "no response");
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    comp.active.exit();
    assert_eq!(comp.active.join(), fprime_os::task::Status::OpOk);
    assert_eq!(
        *ground.responses.lock().unwrap(),
        vec![(FpySequencer::OPCODE_RUN, 7, CmdResponse::Ok)]
    );
    assert_eq!(*loopback.sent.lock().unwrap(), vec![0x10, 0x20]);
    assert_eq!(comp.sequencer_get_state(), State::IDLE);
}
