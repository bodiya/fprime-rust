//! The directive handlers, called directly (upstream `waitRel` …
//! `storeAbsConstOffset`, `allocate`, `loadRel`, `storeRelConstOffset`,
//! `pushVal`, `popEvent`, `popSerializable_*`), and the timer checks.

use super::*;

type Handler<D> = fn(&FpySequencer, &mut FpyState, &D, &mut DirectiveError) -> Signal;

/// Call a directive handler; returns its signal and error code.
fn call<D>(t: &Tester, handler: Handler<D>, directive: &D) -> (Signal, DirectiveError) {
    let mut error = DirectiveError::NoError;
    let signal = t.with(|c, st| handler(c, st, directive, &mut error));
    (signal, error)
}

const OK: (Signal, DirectiveError) = (Signal::stmtResponse_success, DirectiveError::NoError);

fn fail(error: DirectiveError) -> (Signal, DirectiveError) {
    (Signal::stmtResponse_failure, error)
}

fn set_statement_count(t: &Tester, count: u16) {
    t.st().sequence_obj.header.statement_count = count;
}

fn next_index(t: &Tester) -> u32 {
    t.st().runtime.next_statement_index
}

fn set_next_index(t: &Tester, index: u32) {
    t.st().runtime.next_statement_index = index;
}

fn wakeup(t: &Tester) -> WakeupTime {
    t.st().runtime.wakeup_time
}

#[test]
fn wait_rel() {
    let t = Tester::new();
    t.set_time(Time::from_seconds_useconds(100, 100));
    t.push(5u32);
    t.push(123u32);
    let result = call(
        &t,
        FpySequencer::wait_rel_directive_handler,
        &WaitRelDirective::default(),
    );
    assert_eq!(
        result,
        (Signal::stmtResponse_beginSleep, DirectiveError::NoError)
    );
    assert_eq!(
        wakeup(&t),
        WakeupTime::from(Time::from_seconds_useconds(105, 223))
    );
    // carry into the seconds
    t.set_time(Time::from_seconds_useconds(1, 999_999));
    t.push(0u32);
    t.push(2u32);
    call(
        &t,
        FpySequencer::wait_rel_directive_handler,
        &WaitRelDirective::default(),
    );
    assert_eq!(
        wakeup(&t),
        WakeupTime::from(Time::from_seconds_useconds(2, 1))
    );
    // underflow
    t.push(0u32);
    assert_eq!(
        call(
            &t,
            FpySequencer::wait_rel_directive_handler,
            &WaitRelDirective::default()
        ),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
#[should_panic(expected = "Assert")]
fn wait_rel_with_two_seconds_of_microseconds_asserts_like_cpp() {
    let t = Tester::new();
    t.push(0u32);
    t.push(2_000_000u32);
    call(
        &t,
        FpySequencer::wait_rel_directive_handler,
        &WaitRelDirective::default(),
    );
}

#[test]
fn wait_abs() {
    let t = Tester::new();
    t.push(0u16);
    t.push(0u8);
    t.push(5u32);
    t.push(123u32);
    let result = call(
        &t,
        FpySequencer::wait_abs_directive_handler,
        &WaitAbsDirective::default(),
    );
    assert_eq!(
        result,
        (Signal::stmtResponse_beginSleep, DirectiveError::NoError)
    );
    assert_eq!(
        wakeup(&t),
        WakeupTime::from(Time::from_seconds_useconds(5, 123))
    );
    // the time base is kept as pushed, even an undeclared one
    t.push(7u16);
    t.push(9u8);
    t.push(1u32);
    t.push(2u32);
    call(
        &t,
        FpySequencer::wait_abs_directive_handler,
        &WaitAbsDirective::default(),
    );
    assert_eq!(
        wakeup(&t),
        WakeupTime {
            time_base: 7,
            time_context: 9,
            seconds: 1,
            useconds: 2
        }
    );
    // underflow: 10 of the 11 bytes
    t.set_stack_size(10);
    assert_eq!(
        call(
            &t,
            FpySequencer::wait_abs_directive_handler,
            &WaitAbsDirective::default()
        ),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
fn goto() {
    let t = Tester::new();
    set_statement_count(&t, 456);
    let handler = FpySequencer::goto_directive_handler;
    assert_eq!(call(&t, handler, &GotoDirective::new(123)), OK);
    assert_eq!(next_index(&t), 123);

    set_next_index(&t, 0);
    assert_eq!(
        call(&t, handler, &GotoDirective::new(111_111)),
        fail(DirectiveError::StmtOutOfBounds)
    );
    assert_eq!(next_index(&t), 0);

    // just inside bounds: the end of the sequence
    assert_eq!(call(&t, handler, &GotoDirective::new(456)), OK);
    assert_eq!(next_index(&t), 456);
}

#[test]
fn if_directive() {
    let t = Tester::new();
    set_next_index(&t, 100);
    set_statement_count(&t, 123);
    let handler = FpySequencer::if_directive_handler;
    let mut directive = IfDirective::new(111);
    t.push(1u8);
    assert_eq!(call(&t, handler, &directive), OK);
    // true: no jump
    assert_eq!(next_index(&t), 100);

    t.push(0u8);
    assert_eq!(call(&t, handler, &directive), OK);
    assert_eq!(next_index(&t), 111);

    set_next_index(&t, 100);
    directive.false_goto_stmt_index = 123;
    t.push(0u8);
    assert_eq!(call(&t, handler, &directive), OK);
    assert_eq!(next_index(&t), 123);

    set_next_index(&t, 100);
    assert_eq!(
        call(&t, handler, &directive),
        fail(DirectiveError::StackUnderflow)
    );
    assert_eq!(next_index(&t), 100);

    t.push(1u8);
    directive.false_goto_stmt_index = 124;
    assert_eq!(
        call(&t, handler, &directive),
        fail(DirectiveError::StmtOutOfBounds)
    );
    assert_eq!(next_index(&t), 100);
    // the bounds are checked before the condition is popped
    assert_eq!(t.stack_size(), 1);
}

#[test]
fn no_op() {
    let t = Tester::new();
    assert_eq!(
        call(
            &t,
            FpySequencer::no_op_directive_handler,
            &NoOpDirective::default()
        ),
        OK
    );
}

fn set_next_tlm(t: &Tester, id: FwChanIdType, value: &[u8], time: Time) {
    *t.data.next_tlm_id.lock().unwrap() = id;
    let mut buf = TlmBuffer::new();
    assert!(buf.set_buff(value).is_ok());
    *t.data.next_tlm_value.lock().unwrap() = buf;
    *t.data.next_tlm_time.lock().unwrap() = time;
}

#[test]
fn push_tlm_val() {
    let t = Tester::new();
    set_next_tlm(&t, 456, &[200, 100], Time::from_seconds_useconds(888, 777));
    let handler = FpySequencer::push_tlm_val_directive_handler;
    t.set_stack_size(1);
    assert_eq!(call(&t, handler, &PushTlmValDirective::new(456)), OK);
    assert_eq!(*t.data.tlm_gets.lock().unwrap(), vec![456]);
    assert_eq!((t.byte(1), t.byte(2)), (200, 100));
    assert_eq!(t.stack_size(), 3);

    assert_eq!(
        call(&t, handler, &PushTlmValDirective::new(111)),
        fail(DirectiveError::TlmChanNotFound)
    );

    // one byte short
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 1);
    assert_eq!(
        call(&t, handler, &PushTlmValDirective::new(456)),
        fail(DirectiveError::StackOverflow)
    );
    // exact fit
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 2);
    assert_eq!(call(&t, handler, &PushTlmValDirective::new(456)), OK);
    assert_eq!(t.stack_size(), MAX_STACK_SIZE as StackSizeType);
}

use fprime_config::fpy_sequencer::MAX_STACK_SIZE;

#[test]
fn push_tlm_val_and_time() {
    let t = Tester::new();
    let tlm_time = Time::from_seconds_useconds(888, 777);
    set_next_tlm(&t, 456, &[200], tlm_time);
    let handler = FpySequencer::push_tlm_val_and_time_directive_handler;
    assert_eq!(call(&t, handler, &PushTlmValAndTimeDirective::new(456)), OK);
    assert_eq!(*t.data.tlm_gets.lock().unwrap(), vec![456]);
    assert_eq!(t.byte(0), 200);
    assert_eq!(t.stack_size() as usize, 1 + Time::SERIALIZED_SIZE);
    let bytes = t.st().runtime.stack.bytes[1..1 + Time::SERIALIZED_SIZE].to_vec();
    assert_eq!(time_from(&bytes), tlm_time);

    assert_eq!(
        call(&t, handler, &PushTlmValAndTimeDirective::new(111)),
        fail(DirectiveError::TlmChanNotFound)
    );

    // one byte over: room for the time only
    t.set_stack_size((MAX_STACK_SIZE - Time::SERIALIZED_SIZE) as StackSizeType);
    assert_eq!(
        call(&t, handler, &PushTlmValAndTimeDirective::new(456)),
        fail(DirectiveError::StackOverflow)
    );
    // exact fit
    t.set_stack_size((MAX_STACK_SIZE - Time::SERIALIZED_SIZE - 1) as StackSizeType);
    assert_eq!(call(&t, handler, &PushTlmValAndTimeDirective::new(456)), OK);
    assert_eq!(t.stack_size(), MAX_STACK_SIZE as StackSizeType);
}

#[test]
fn push_tlm_val_and_time_full_buffer() {
    let t = Tester::new();
    let full = [0u8; fprime_config::FW_TLM_BUFFER_MAX_SIZE];
    set_next_tlm(&t, 456, &full, Time::from_seconds_useconds(888, 777));
    let needed = fprime_config::FW_TLM_BUFFER_MAX_SIZE + Time::SERIALIZED_SIZE;
    let result = call(
        &t,
        FpySequencer::push_tlm_val_and_time_directive_handler,
        &PushTlmValAndTimeDirective::new(456),
    );
    // the default configuration has room for it
    assert!(needed <= MAX_STACK_SIZE);
    assert_eq!(result, OK);
    assert_eq!(t.stack_size() as usize, needed);
}

#[test]
fn push_tlm_val_without_get_tlm_chan() {
    // C++ checks `isConnected_getTlmChan_OutputPort`; the Rust tester has
    // the port connected, so check the error path with a bare component.
    let comp = FpySequencer::new("bare");
    let mut st = comp.state.lock().unwrap();
    let mut error = DirectiveError::NoError;
    let signal =
        comp.push_tlm_val_directive_handler(&mut st, &PushTlmValDirective::new(1), &mut error);
    assert_eq!((signal, error), fail(DirectiveError::TlmGetNotConnected));
    let signal = comp.push_tlm_val_and_time_directive_handler(
        &mut st,
        &PushTlmValAndTimeDirective::new(1),
        &mut error,
    );
    assert_eq!((signal, error), fail(DirectiveError::TlmGetNotConnected));
}

#[test]
fn push_prm() {
    let t = Tester::new();
    *t.data.next_prm_id.lock().unwrap() = 456;
    assert!(
        t.data
            .next_prm_value
            .lock()
            .unwrap()
            .set_buff(&[200, 100])
            .is_ok()
    );
    let handler = FpySequencer::push_prm_directive_handler;
    t.set_stack_size(1);
    assert_eq!(call(&t, handler, &PushPrmDirective::new(456)), OK);
    assert_eq!(*t.data.prm_gets.lock().unwrap(), vec![456]);
    assert_eq!((t.byte(1), t.byte(2)), (200, 100));
    assert_eq!(t.stack_size(), 3);

    assert_eq!(
        call(&t, handler, &PushPrmDirective::new(111)),
        fail(DirectiveError::PrmNotFound)
    );
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 1);
    assert_eq!(
        call(&t, handler, &PushPrmDirective::new(456)),
        fail(DirectiveError::StackOverflow)
    );
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 2);
    assert_eq!(call(&t, handler, &PushPrmDirective::new(456)), OK);
    assert_eq!(t.stack_size(), MAX_STACK_SIZE as StackSizeType);

    let t = Tester::without_get_param();
    t.set_stack_size(1);
    assert_eq!(
        call(&t, handler, &PushPrmDirective::new(456)),
        fail(DirectiveError::PrmGetNotConnected)
    );
}

/// The command packet `[FW_PACKET_COMMAND u16][opcode u32][args]`.
fn command_packet(opcode: FwOpcodeType, args: &[u8]) -> Vec<u8> {
    let mut out = vec![0, 0];
    out.extend_from_slice(&opcode.to_be_bytes());
    out.extend_from_slice(args);
    out
}

#[test]
fn cmd() {
    let t = Tester::new();
    let data = [0x12, 0x23, 0x34, 0x45];
    let mut directive = ConstCmdDirective::new(123, DirectiveBytes::default(), 4);
    directive.arg_buf.0[..4].copy_from_slice(&data);
    let handler = FpySequencer::const_cmd_directive_handler;
    assert_eq!(
        call(&t, handler, &directive),
        (Signal::stmtResponse_keepWaiting, DirectiveError::NoError)
    );
    let expected = command_packet(123, &data);
    assert_eq!(t.cmd_out(), vec![(expected.clone(), 0)]);
    t.clear_history();

    // the command UID holds the statement counter...
    t.st().statements_dispatched = 123;
    call(&t, handler, &directive);
    assert_eq!(t.cmd_out(), vec![(expected.clone(), 123)]);
    t.clear_history();

    // ...and the sequence counter
    t.st().sequences_started = 456;
    call(&t, handler, &directive);
    assert_eq!(t.cmd_out(), vec![(expected, (456 << 16) | 123)]);
}

#[test]
fn cmd_stack_overflow() {
    let t = Tester::new();
    let directive = ConstCmdDirective::new(123, DirectiveBytes::default(), 0);
    let handler = FpySequencer::const_cmd_directive_handler;
    // no room for the response code
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType);
    assert_eq!(
        call(&t, handler, &directive),
        fail(DirectiveError::StackOverflow)
    );
    assert!(t.cmd_out().is_empty());
    // exactly enough room
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 1);
    assert_eq!(
        call(&t, handler, &directive),
        (Signal::stmtResponse_keepWaiting, DirectiveError::NoError)
    );
    assert_eq!(t.cmd_out().len(), 1);
}

#[test]
fn stack_op() {
    let t = Tester::new();
    let handler = FpySequencer::stack_op_directive_handler;
    let run = |op: DirectiveId| call(&t, handler, &StackOpDirective::new(op));

    t.push(10i64);
    t.push(10i64);
    assert_eq!(run(DirectiveId::Ieq), OK);
    assert_eq!((t.byte(0), t.stack_size()), (0xFF, 1));
    t.set_stack_size(0);

    t.push(10i64);
    t.push(20i64);
    assert_eq!(run(DirectiveId::Ine), OK);
    assert_eq!((t.byte(0), t.stack_size()), (0xFF, 1));
    t.set_stack_size(0);

    t.push(5u8);
    t.push(10u8);
    assert_eq!(run(DirectiveId::Or), OK);
    assert_eq!((t.byte(0), t.stack_size()), (15, 1));
    t.set_stack_size(0);

    t.push(12u8);
    t.push(10u8);
    assert_eq!(run(DirectiveId::And), OK);
    assert_eq!((t.byte(0), t.stack_size()), (8, 1));
    t.set_stack_size(0);

    t.push(-5i64);
    t.push(10i64);
    assert_eq!(run(DirectiveId::Slt), OK);
    assert_eq!(t.byte(0), 0xFF);
    t.set_stack_size(0);

    t.push(10i64);
    t.push(5i64);
    assert_eq!(run(DirectiveId::Ugt), OK);
    assert_eq!(t.byte(0), 0xFF);
    t.set_stack_size(0);

    t.push(5.5f64);
    t.push(10.1f64);
    assert_eq!(run(DirectiveId::Flt), OK);
    assert_eq!(t.byte(0), 0xFF);
    t.set_stack_size(0);

    // an op's failure is a statement failure
    assert_eq!(run(DirectiveId::Ieq), fail(DirectiveError::StackUnderflow));
    // the extended ops at the end of the id range are stack ops too
    t.push(-2.5f64);
    assert_eq!(run(DirectiveId::Fabs), OK);
    assert_eq!(t.pop::<f64>(), 2.5);
}

#[test]
#[should_panic(expected = "Assert")]
fn stack_op_with_a_non_stack_op_asserts() {
    let t = Tester::new();
    call(
        &t,
        FpySequencer::stack_op_directive_handler,
        &StackOpDirective::new(DirectiveId::NoOp),
    );
}

#[test]
fn exit() {
    let t = Tester::new();
    let handler = FpySequencer::exit_directive_handler;
    set_statement_count(&t, 123);
    t.push(0i32);
    assert_eq!(call(&t, handler, &ExitDirective::default()), OK);
    // exit(0) goes to the end of the sequence
    assert_eq!(next_index(&t), 123);

    t.push(123i32);
    assert_eq!(
        call(&t, handler, &ExitDirective::default()),
        fail(DirectiveError::ExitWithError)
    );
    let events = t.events(FpySequencer::EVENTID_SEQUENCE_EXITED_WITH_ERROR);
    assert_eq!(events.len(), 1);
    // the path (empty in IDLE) and the code
    assert_eq!(events[0], [0, 0, 0, 0, 0, 123]);

    // the exit code is signed
    t.push(-5i32);
    assert_eq!(
        call(&t, handler, &ExitDirective::default()),
        fail(DirectiveError::ExitWithError)
    );
    let events = t.events(FpySequencer::EVENTID_SEQUENCE_EXITED_WITH_ERROR);
    assert_eq!(events[1], [0, 0, 0xFF, 0xFF, 0xFF, 0xFB]);

    assert_eq!(
        call(&t, handler, &ExitDirective::default()),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
fn discard() {
    let t = Tester::new();
    let handler = FpySequencer::discard_directive_handler;
    t.push(0x1234_5678u32);
    assert_eq!(call(&t, handler, &DiscardDirective::new(4)), OK);
    assert_eq!(t.stack_size(), 0);
    t.push(0x12u8);
    assert_eq!(
        call(&t, handler, &DiscardDirective::new(2)),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
fn stack_cmd() {
    let t = Tester::new();
    let handler = FpySequencer::stack_cmd_directive_handler;
    t.push(0x1234_5678u32); // args
    t.push(123u32); // opcode
    assert_eq!(
        call(&t, handler, &StackCmdDirective::new(4)),
        (Signal::stmtResponse_keepWaiting, DirectiveError::NoError)
    );
    assert_eq!(
        t.cmd_out(),
        vec![(command_packet(123, &[0x12, 0x34, 0x56, 0x78]), 0)]
    );
    assert_eq!(t.stack_size(), 0);
    assert_eq!(t.st().runtime.current_cmd_opcode, 123);

    t.push(0x12u8);
    assert_eq!(
        call(&t, handler, &StackCmdDirective::new(4)),
        fail(DirectiveError::StackUnderflow)
    );

    // args size near U32 max must not wrap the check
    t.set_stack_size(0);
    assert_eq!(
        call(&t, handler, &StackCmdDirective::new(0xFFFF_FFFC)),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
fn stack_cmd_push_overflow() {
    // a full stack before the pop: popping the opcode frees more room than
    // the response code needs, so the command goes out
    let t = Tester::new();
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 4);
    t.push(42u32);
    assert_eq!(t.stack_size(), MAX_STACK_SIZE as StackSizeType);
    assert_eq!(
        call(
            &t,
            FpySequencer::stack_cmd_directive_handler,
            &StackCmdDirective::new(0)
        ),
        (Signal::stmtResponse_keepWaiting, DirectiveError::NoError)
    );
    assert_eq!(t.cmd_out().len(), 1);
    assert_eq!(t.stack_size(), MAX_STACK_SIZE as StackSizeType - 4);
}

#[test]
fn mem_cmp() {
    let t = Tester::new();
    let handler = FpySequencer::mem_cmp_directive_handler;
    t.push(0x1234_5678u32);
    t.push(0x1234_5678u32);
    assert_eq!(call(&t, handler, &MemCmpDirective::new(4)), OK);
    assert_eq!(t.pop::<u8>(), 0xFF);

    t.push(0x1234_5678u32);
    t.push(0x8765_4321u32);
    assert_eq!(call(&t, handler, &MemCmpDirective::new(4)), OK);
    assert_eq!(t.pop::<u8>(), 0x00);

    t.push(0x1234_5678u32);
    t.push(0x11u8);
    assert_eq!(
        call(&t, handler, &MemCmpDirective::new(3)),
        fail(DirectiveError::StackUnderflow)
    );

    // size * 2 would wrap a U32
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType);
    assert_eq!(
        call(&t, handler, &MemCmpDirective::new(0x8000_0001)),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
fn push_time() {
    let t = Tester::new();
    let now = time(TimeBase::TbWorkstationTime, 0, 100, 100);
    t.set_time(now);
    let handler = FpySequencer::push_time_directive_handler;
    assert_eq!(call(&t, handler, &PushTimeDirective::default()), OK);
    assert_eq!(t.stack_size() as usize, Time::SERIALIZED_SIZE);
    let bytes = t.st().runtime.stack.bytes[..Time::SERIALIZED_SIZE].to_vec();
    assert_eq!(time_from(&bytes), now);
    // the wire format of Fw::Time
    assert_eq!(bytes, [0, 2, 0, 0, 0, 0, 100, 0, 0, 0, 100]);

    // just fits
    t.set_stack_size((MAX_STACK_SIZE - Time::SERIALIZED_SIZE) as StackSizeType);
    assert_eq!(call(&t, handler, &PushTimeDirective::default()), OK);
    assert_eq!(t.stack_size(), MAX_STACK_SIZE as StackSizeType);

    t.set_stack_size((MAX_STACK_SIZE - Time::SERIALIZED_SIZE + 1) as StackSizeType);
    assert_eq!(
        call(&t, handler, &PushTimeDirective::default()),
        fail(DirectiveError::StackOverflow)
    );
}

#[test]
fn push_rand() {
    let t = Tester::new();
    let now = time(TimeBase::TbWorkstationTime, 7, 123, 456);
    t.set_time(now);
    // C++ seeds with std::seed_seq{base, context, seconds, useconds}
    let mut expected = Mt19937::new();
    expected.seed_seq(&[2, 7, 123, 456]);
    let handler = FpySequencer::push_rand_directive_handler;
    assert_eq!(call(&t, handler, &PushRandDirective::default()), OK);
    assert_eq!(t.stack_size(), 4);
    assert_eq!(t.pop::<u32>(), expected.next_u32());
    // libstdc++'s first output for that seed
    let mut check = Mt19937::new();
    check.seed_seq(&[2, 7, 123, 456]);
    assert_eq!(check.next_u32(), 469_824_542);

    // later calls advance the generator instead of reseeding from the time
    t.set_time(time(TimeBase::TbWorkstationTime, 99, 999, 999));
    assert_eq!(call(&t, handler, &PushRandDirective::default()), OK);
    assert_eq!(t.pop::<u32>(), expected.next_u32());

    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 4);
    assert_eq!(call(&t, handler, &PushRandDirective::default()), OK);
    assert_eq!(t.stack_size(), MAX_STACK_SIZE as StackSizeType);
    assert_eq!(
        call(&t, handler, &PushRandDirective::default()),
        fail(DirectiveError::StackOverflow)
    );
}

#[test]
fn set_seed() {
    let t = Tester::new();
    let mut expected = Mt19937::new();
    expected.seed(123_456_789);
    t.push(123_456_789u32);
    assert_eq!(
        call(
            &t,
            FpySequencer::set_seed_directive_handler,
            &SetSeedDirective::default()
        ),
        OK
    );
    assert_eq!(t.stack_size(), 0);
    // the explicit seed overrides the time-based one
    t.set_time(time(TimeBase::TbWorkstationTime, 77, 888, 999));
    assert_eq!(
        call(
            &t,
            FpySequencer::push_rand_directive_handler,
            &PushRandDirective::default()
        ),
        OK
    );
    assert_eq!(t.pop::<u32>(), expected.next_u32());
    assert_eq!(
        call(
            &t,
            FpySequencer::set_seed_directive_handler,
            &SetSeedDirective::default()
        ),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
fn get_field() {
    let t = Tester::new();
    let handler = FpySequencer::get_field_directive_handler;
    for b in [123u8, 45, 67, 99] {
        t.push(b);
    }
    t.push(1u32);
    assert_eq!(call(&t, handler, &GetFieldDirective::new(4, 2)), OK);
    assert_eq!(t.stack_size(), 2);
    assert_eq!((t.byte(0), t.byte(1)), (45, 67));

    // parent bigger than the stack
    t.set_stack_size(0);
    t.push(0u64);
    t.push(0u32);
    assert_eq!(
        call(&t, handler, &GetFieldDirective::new(20, 1)),
        fail(DirectiveError::StackUnderflow)
    );

    // member past the parent
    t.set_stack_size(0);
    t.push(0u64);
    t.push(0u32);
    assert_eq!(
        call(&t, handler, &GetFieldDirective::new(1, 6)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );

    // no offset on the stack
    t.set_stack_size(0);
    assert_eq!(
        call(&t, handler, &GetFieldDirective::new(0, 0)),
        fail(DirectiveError::StackUnderflow)
    );

    // offset + member past the parent: 60 + 50 > 100
    t.set_stack_size(0);
    for i in 0..100u8 {
        t.push(i);
    }
    t.push(60u32);
    assert_eq!(
        call(&t, handler, &GetFieldDirective::new(100, 50)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );

    // an offset that would wrap a U32 when the member size is added
    t.set_stack_size(0);
    for i in 0..100u8 {
        t.push(i);
    }
    t.push(0xFFFF_FFFFu32);
    assert_eq!(
        call(&t, handler, &GetFieldDirective::new(100, 10)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );

    // only the offset, no parent
    t.set_stack_size(0);
    t.push(0u32);
    assert_eq!(
        call(&t, handler, &GetFieldDirective::new(100, 10)),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
fn store_rel() {
    let t = Tester::new();
    let handler = FpySequencer::store_rel_directive_handler;
    t.set_stack_size(10);
    t.st().runtime.stack.current_frame_start = 0;
    t.push(123u8);
    t.push(100u8);
    t.push(4i32);
    assert_eq!(call(&t, handler, &StoreRelDirective::new(2)), OK);
    assert_eq!(t.stack_size(), 10);
    assert_eq!((t.byte(4), t.byte(5)), (123, 100));

    // a negative offset reaches the call arguments
    t.set_stack_size(10);
    t.st().runtime.stack.current_frame_start = 4;
    t.push(55u8);
    t.push(66u8);
    t.push(-4i32);
    assert_eq!(call(&t, handler, &StoreRelDirective::new(2)), OK);
    assert_eq!((t.byte(0), t.byte(1)), (55, 66));

    // too high
    t.set_stack_size(0);
    t.st().runtime.stack.current_frame_start = 0;
    t.push(123u8);
    t.push(100u8);
    t.push(100i32);
    assert_eq!(
        call(&t, handler, &StoreRelDirective::new(2)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );

    // below zero
    t.set_stack_size(0);
    t.st().runtime.stack.current_frame_start = 2;
    t.push(123u8);
    t.push(100u8);
    t.push(-10i32);
    assert_eq!(
        call(&t, handler, &StoreRelDirective::new(2)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );

    t.set_stack_size(0);
    t.st().runtime.stack.current_frame_start = 0;
    assert_eq!(
        call(&t, handler, &StoreRelDirective::new(100)),
        fail(DirectiveError::StackUnderflow)
    );
    t.set_stack_size(10);
    assert_eq!(
        call(
            &t,
            handler,
            &StoreRelDirective::new(MAX_STACK_SIZE as StackSizeType)
        ),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
fn peek() {
    let t = Tester::new();
    let handler = FpySequencer::peek_directive_handler;
    let peek = || call(&t, handler, &PeekDirective::default());
    assert_eq!(peek(), fail(DirectiveError::StackUnderflow));

    // duplicate the top two bytes
    t.push(12u8);
    t.push(34u8);
    t.push(2u32); // size
    t.push(0u32); // offset
    assert_eq!(peek(), OK);
    assert_eq!(t.stack_size(), 4);
    assert_eq!(
        t.st().runtime.stack.bytes[..4].to_vec(),
        vec![12, 34, 12, 34]
    );

    // two bytes below the top
    t.set_stack_size(0);
    for b in [12u8, 34, 0, 0] {
        t.push(b);
    }
    t.push(2u32);
    t.push(2u32);
    assert_eq!(peek(), OK);
    assert_eq!(
        t.st().runtime.stack.bytes[..6].to_vec(),
        vec![12, 34, 0, 0, 12, 34]
    );

    // offset past the bottom
    t.set_stack_size(10);
    t.push(0u32);
    t.push(11u32);
    assert_eq!(peek(), fail(DirectiveError::StackAccessOutOfBounds));

    // offset at the bottom with a zero count is fine
    t.set_stack_size(10);
    t.push(0u32);
    t.push(10u32);
    assert_eq!(peek(), OK);
    assert_eq!(t.stack_size(), 10);

    // overflow
    t.set_stack_size((MAX_STACK_SIZE - 1 - 8) as StackSizeType);
    t.push(2u32 + 8);
    t.push(0u32);
    assert_eq!(peek(), fail(DirectiveError::StackOverflow));

    // underflow
    t.set_stack_size(0);
    assert_eq!(peek(), fail(DirectiveError::StackUnderflow));

    // a byte count beyond the free space
    t.set_stack_size(100);
    t.push(MAX_STACK_SIZE as u32);
    t.push(100u32);
    assert_eq!(peek(), fail(DirectiveError::StackOverflow));
}

#[test]
fn call_directive() {
    let t = Tester::new();
    let handler = FpySequencer::call_directive_handler;
    t.st().runtime.stack.current_frame_start = 4;
    set_statement_count(&t, 20);
    set_next_index(&t, 5);
    t.push(10u32);
    assert_eq!(call(&t, handler, &CallDirective::default()), OK);
    // return address (5) and old frame pointer (4)
    assert_eq!(t.stack_size(), 8);
    assert_eq!(
        t.st().runtime.stack.bytes[..8].to_vec(),
        vec![0, 0, 0, 5, 0, 0, 0, 4]
    );
    assert_eq!(next_index(&t), 10);
    assert_eq!(t.st().runtime.stack.current_frame_start, 8);

    t.set_stack_size(0);
    assert_eq!(
        call(&t, handler, &CallDirective::default()),
        fail(DirectiveError::StackUnderflow)
    );

    t.set_stack_size(0);
    set_statement_count(&t, 5);
    t.push(10u32);
    assert_eq!(
        call(&t, handler, &CallDirective::default()),
        fail(DirectiveError::StmtOutOfBounds)
    );

    // no room for the frame
    set_statement_count(&t, 100);
    t.set_stack_size(0);
    t.push(10u32);
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType);
    t.st().runtime.stack.current_frame_start = 0;
    assert_eq!(
        call(&t, handler, &CallDirective::default()),
        fail(DirectiveError::StackOverflow)
    );
}

#[test]
fn return_directive() {
    let t = Tester::new();
    let handler = FpySequencer::return_directive_handler;
    set_statement_count(&t, 20);
    // [args 8][return addr][old frame ptr][locals][return value]
    t.push(0x1122_3344_5566_7788u64);
    t.push(15u32);
    t.push(2u32);
    t.st().runtime.stack.current_frame_start = 16;
    t.push(0xDEAD_BEEFu32);
    t.push(0x1234_5678u32);
    assert_eq!(call(&t, handler, &ReturnDirective::new(4, 8)), OK);
    assert_eq!(t.stack_size(), 4);
    assert_eq!(
        t.st().runtime.stack.bytes[..4].to_vec(),
        vec![0x12, 0x34, 0x56, 0x78]
    );
    assert_eq!(t.st().runtime.stack.current_frame_start, 2);
    assert_eq!(next_index(&t), 15);

    // return address past the end
    set_statement_count(&t, 10);
    t.set_stack_size(0);
    t.push(100u32);
    t.push(0u32);
    t.st().runtime.stack.current_frame_start = 8;
    assert_eq!(
        call(&t, handler, &ReturnDirective::new(0, 0)),
        fail(DirectiveError::StmtOutOfBounds)
    );

    // saved frame pointer past the popped stack
    set_statement_count(&t, 100);
    t.set_stack_size(0);
    t.push(5u32);
    t.push(100u32);
    t.st().runtime.stack.current_frame_start = 8;
    assert_eq!(
        call(&t, handler, &ReturnDirective::new(0, 0)),
        fail(DirectiveError::FrameStartOutOfBounds)
    );

    // frame start past the stack
    t.set_stack_size(8);
    t.push(5u32);
    t.push(0u32);
    t.st().runtime.stack.current_frame_start = 100;
    assert_eq!(
        call(&t, handler, &ReturnDirective::new(0, 0)),
        fail(DirectiveError::FrameStartOutOfBounds)
    );

    // a return value bigger than the stack
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType);
    t.st().runtime.stack.current_frame_start = 0;
    assert_eq!(
        call(
            &t,
            handler,
            &ReturnDirective::new(MAX_STACK_SIZE as StackSizeType + 1, 0)
        ),
        fail(DirectiveError::StackUnderflow)
    );

    // more arguments than there are bytes
    set_statement_count(&t, 100);
    t.set_stack_size(0);
    for i in 0..20u8 {
        t.push(i);
    }
    t.push(5u32);
    t.push(0u32);
    let frame = t.stack_size();
    t.st().runtime.stack.current_frame_start = frame;
    for i in 0..10u8 {
        t.push(i);
    }
    assert_eq!(
        call(&t, handler, &ReturnDirective::new(10, 50)),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
fn load_abs() {
    let t = Tester::new();
    let handler = FpySequencer::load_abs_directive_handler;
    t.set_stack_size(10);
    t.st().runtime.stack.bytes[4] = 0xAB;
    t.st().runtime.stack.bytes[5] = 0xCD;
    assert_eq!(call(&t, handler, &LoadAbsDirective::new(4, 2)), OK);
    assert_eq!(t.stack_size(), 12);
    assert_eq!((t.byte(10), t.byte(11)), (0xAB, 0xCD));

    t.set_stack_size(3);
    assert_eq!(
        call(&t, handler, &LoadAbsDirective::new(4, 2)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 1);
    assert_eq!(
        call(&t, handler, &LoadAbsDirective::new(0, 2)),
        fail(DirectiveError::StackOverflow)
    );
    t.set_stack_size(1000);
    assert_eq!(
        call(
            &t,
            handler,
            &LoadAbsDirective::new(MAX_STACK_SIZE as StackSizeType, 100)
        ),
        fail(DirectiveError::StackAccessOutOfBounds)
    );
}

#[test]
fn store_abs() {
    let t = Tester::new();
    let handler = FpySequencer::store_abs_directive_handler;
    t.set_stack_size(10);
    t.push(0xEEu8);
    t.push(0xFFu8);
    t.push(4u32);
    assert_eq!(call(&t, handler, &StoreAbsDirective::new(2)), OK);
    assert_eq!((t.byte(4), t.byte(5)), (0xEE, 0xFF));
    assert_eq!(t.stack_size(), 10);

    t.set_stack_size(0);
    t.push(0x11u8);
    t.push(0x22u8);
    t.push(100u32);
    assert_eq!(
        call(&t, handler, &StoreAbsDirective::new(2)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );
    t.set_stack_size(0);
    assert_eq!(
        call(&t, handler, &StoreAbsDirective::new(10)),
        fail(DirectiveError::StackUnderflow)
    );
    t.set_stack_size(10);
    assert_eq!(
        call(
            &t,
            handler,
            &StoreAbsDirective::new(MAX_STACK_SIZE as StackSizeType)
        ),
        fail(DirectiveError::StackUnderflow)
    );
}

#[test]
fn store_abs_const_offset() {
    let t = Tester::new();
    let handler = FpySequencer::store_abs_const_offset_directive_handler;
    t.set_stack_size(10);
    t.push(0xAAu8);
    t.push(0xBBu8);
    assert_eq!(
        call(&t, handler, &StoreAbsConstOffsetDirective::new(4, 2)),
        OK
    );
    assert_eq!((t.byte(4), t.byte(5)), (0xAA, 0xBB));
    assert_eq!(t.stack_size(), 10);

    t.set_stack_size(0);
    t.push(0x11u8);
    t.push(0x22u8);
    assert_eq!(
        call(&t, handler, &StoreAbsConstOffsetDirective::new(100, 2)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );
    t.set_stack_size(0);
    assert_eq!(
        call(&t, handler, &StoreAbsConstOffsetDirective::new(0, 10)),
        fail(DirectiveError::StackUnderflow)
    );
    t.set_stack_size(100);
    assert_eq!(
        call(
            &t,
            handler,
            &StoreAbsConstOffsetDirective::new(MAX_STACK_SIZE as StackSizeType, 100)
        ),
        fail(DirectiveError::StackAccessOutOfBounds)
    );
}

#[test]
fn allocate() {
    let t = Tester::new();
    let handler = FpySequencer::allocate_directive_handler;
    t.st().runtime.stack.bytes[..20].fill(0xFF);
    assert_eq!(call(&t, handler, &AllocateDirective::new(8)), OK);
    assert_eq!(t.stack_size(), 8);
    assert!(t.st().runtime.stack.bytes[..8].iter().all(|&b| b == 0));
    assert_eq!(call(&t, handler, &AllocateDirective::new(4)), OK);
    assert_eq!(t.stack_size(), 12);
    assert!(t.st().runtime.stack.bytes[8..12].iter().all(|&b| b == 0));
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 50);
    assert_eq!(
        call(&t, handler, &AllocateDirective::new(100)),
        fail(DirectiveError::StackOverflow)
    );
}

#[test]
fn load_rel() {
    let t = Tester::new();
    let handler = FpySequencer::load_rel_directive_handler;
    for b in [0xAAu8, 0xBB, 0xCC, 0xDD] {
        t.push(b);
    }
    t.st().runtime.stack.current_frame_start = 0;
    assert_eq!(call(&t, handler, &LoadRelDirective::new(1, 2)), OK);
    assert_eq!(t.stack_size(), 6);
    assert_eq!((t.byte(4), t.byte(5)), (0xBB, 0xCC));

    // relative to the frame
    t.set_stack_size(10);
    t.st().runtime.stack.bytes[4] = 0x11;
    t.st().runtime.stack.bytes[5] = 0x22;
    t.st().runtime.stack.current_frame_start = 4;
    assert_eq!(call(&t, handler, &LoadRelDirective::new(0, 2)), OK);
    assert_eq!((t.byte(10), t.byte(11)), (0x11, 0x22));

    // a negative offset
    t.set_stack_size(10);
    t.st().runtime.stack.bytes[2] = 0x33;
    t.st().runtime.stack.bytes[3] = 0x44;
    assert_eq!(call(&t, handler, &LoadRelDirective::new(-2, 2)), OK);
    assert_eq!((t.byte(10), t.byte(11)), (0x33, 0x44));

    t.set_stack_size(10);
    t.st().runtime.stack.current_frame_start = 2;
    assert_eq!(
        call(&t, handler, &LoadRelDirective::new(-10, 2)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );
    t.set_stack_size(50);
    t.st().runtime.stack.current_frame_start = 0;
    assert_eq!(
        call(&t, handler, &LoadRelDirective::new(0, 100)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );
    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 50);
    assert_eq!(
        call(&t, handler, &LoadRelDirective::new(0, 100)),
        fail(DirectiveError::StackOverflow)
    );
    t.set_stack_size(100);
    t.st().runtime.stack.current_frame_start = MAX_STACK_SIZE as StackSizeType;
    assert_eq!(
        call(&t, handler, &LoadRelDirective::new(1, 1)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );
}

#[test]
fn store_rel_const_offset() {
    let t = Tester::new();
    let handler = FpySequencer::store_rel_const_offset_directive_handler;
    t.set_stack_size(10);
    t.st().runtime.stack.current_frame_start = 0;
    t.push(0xEEu8);
    t.push(0xFFu8);
    assert_eq!(
        call(&t, handler, &StoreRelConstOffsetDirective::new(4, 2)),
        OK
    );
    assert_eq!(t.stack_size(), 10);
    assert_eq!((t.byte(4), t.byte(5)), (0xEE, 0xFF));

    t.set_stack_size(10);
    t.st().runtime.stack.current_frame_start = 4;
    t.push(0x11u8);
    t.push(0x22u8);
    assert_eq!(
        call(&t, handler, &StoreRelConstOffsetDirective::new(2, 2)),
        OK
    );
    assert_eq!((t.byte(6), t.byte(7)), (0x11, 0x22));

    t.set_stack_size(10);
    t.st().runtime.stack.current_frame_start = 4;
    t.push(0x33u8);
    t.push(0x44u8);
    assert_eq!(
        call(&t, handler, &StoreRelConstOffsetDirective::new(-2, 2)),
        OK
    );
    assert_eq!((t.byte(2), t.byte(3)), (0x33, 0x44));

    t.set_stack_size(10);
    t.st().runtime.stack.current_frame_start = 2;
    t.push(0x55u8);
    t.push(0x66u8);
    assert_eq!(
        call(&t, handler, &StoreRelConstOffsetDirective::new(-10, 2)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );
    t.set_stack_size(50);
    t.st().runtime.stack.current_frame_start = 0;
    assert_eq!(
        call(&t, handler, &StoreRelConstOffsetDirective::new(0, 100)),
        fail(DirectiveError::StackUnderflow)
    );
    t.set_stack_size(100);
    t.st().runtime.stack.current_frame_start = MAX_STACK_SIZE as StackSizeType;
    t.push(0x77u8);
    assert_eq!(
        call(&t, handler, &StoreRelConstOffsetDirective::new(1, 1)),
        fail(DirectiveError::StackAccessOutOfBounds)
    );
}

#[test]
fn push_val() {
    let t = Tester::new();
    let handler = FpySequencer::push_val_directive_handler;
    let data = [0x11u8, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA];
    let mut directive = PushValDirective::new(DirectiveBytes::default(), 10);
    directive.val.0[..10].copy_from_slice(&data);
    assert_eq!(call(&t, handler, &directive), OK);
    assert_eq!(t.stack_size(), 10);
    assert_eq!(t.st().runtime.stack.bytes[..10].to_vec(), data.to_vec());

    let mut more = PushValDirective::new(DirectiveBytes::default(), 4);
    more.val.0[..4].copy_from_slice(&[0xBB, 0xCC, 0xDD, 0xEE]);
    assert_eq!(call(&t, handler, &more), OK);
    assert_eq!(t.stack_size(), 14);
    assert_eq!(
        t.st().runtime.stack.bytes[10..14].to_vec(),
        vec![0xBB, 0xCC, 0xDD, 0xEE]
    );

    t.set_stack_size(MAX_STACK_SIZE as StackSizeType - 5);
    assert_eq!(
        call(&t, handler, &directive),
        fail(DirectiveError::StackOverflow)
    );
}

/// Push a `POP_EVENT` frame: severity, message bytes, message size.
fn push_event(t: &Tester, severity: u8, message: &[u8]) {
    t.push(severity);
    for b in message {
        t.push(*b);
    }
    t.push(message.len() as u32);
}

#[test]
fn pop_event() {
    let t = Tester::new();
    let handler = FpySequencer::pop_event_directive_handler;
    let message = b"hello world";
    for (severity, id) in [
        (LogSeverity::Fatal, FpySequencer::EVENTID_LOG_FATAL),
        (LogSeverity::WarningHi, FpySequencer::EVENTID_LOG_WARNING_HI),
        (LogSeverity::WarningLo, FpySequencer::EVENTID_LOG_WARNING_LO),
        (LogSeverity::Command, FpySequencer::EVENTID_LOG_COMMAND),
        (
            LogSeverity::ActivityHi,
            FpySequencer::EVENTID_LOG_ACTIVITY_HI,
        ),
        (
            LogSeverity::ActivityLo,
            FpySequencer::EVENTID_LOG_ACTIVITY_LO,
        ),
        (
            LogSeverity::Diagnostic,
            FpySequencer::EVENTID_LOG_DIAGNOSTIC,
        ),
    ] {
        push_event(&t, severity.as_repr(), message);
        assert_eq!(call(&t, handler, &PopEventDirective::default()), OK);
        let events = t.events(id);
        assert_eq!(events.len(), 1, "{severity:?}");
        // the (empty) sequence path, then the message
        let mut expected = vec![0, 0, 0, 11];
        expected.extend_from_slice(message);
        assert_eq!(events[0], expected);
        assert_eq!(t.ground.events.lock().unwrap()[0].1, severity);
        assert_eq!(t.stack_size(), 0);
        t.clear_history();
    }

    // an unknown severity
    push_event(&t, 0, message);
    assert_eq!(
        call(&t, handler, &PopEventDirective::default()),
        fail(DirectiveError::InvalidArg)
    );
    t.set_stack_size(0);

    // 10 message bytes claimed, none there
    t.push(LogSeverity::ActivityHi.as_repr());
    t.push(10u32);
    assert_eq!(
        call(&t, handler, &PopEventDirective::default()),
        fail(DirectiveError::StackUnderflow)
    );
    t.set_stack_size(0);

    // a size that would wrap (nasa/fprime#5307)
    t.push(LogSeverity::ActivityHi.as_repr());
    t.push(0xFFFF_FFFFu32);
    assert_eq!(
        call(&t, handler, &PopEventDirective::default()),
        fail(DirectiveError::StackUnderflow)
    );
    t.set_stack_size(0);

    // nothing at all
    assert_eq!(
        call(&t, handler, &PopEventDirective::default()),
        fail(DirectiveError::StackUnderflow)
    );

    // an empty message
    push_event(&t, LogSeverity::ActivityHi.as_repr(), b"");
    assert_eq!(call(&t, handler, &PopEventDirective::default()), OK);
    assert_eq!(
        t.events(FpySequencer::EVENTID_LOG_ACTIVITY_HI),
        vec![vec![0, 0, 0, 0]]
    );
}

#[test]
fn pop_event_keeps_the_start_of_a_long_message() {
    // at most FW_LOG_STRING_MAX_SIZE - 1 bytes are kept; the excess (the
    // end of the message, on top) is dropped
    let t = Tester::new();
    let message: Vec<u8> = (0..300u32).map(|i| b'a' + (i % 26) as u8).collect();
    push_event(&t, LogSeverity::ActivityLo.as_repr(), &message);
    assert_eq!(
        call(
            &t,
            FpySequencer::pop_event_directive_handler,
            &PopEventDirective::default()
        ),
        OK
    );
    assert_eq!(t.stack_size(), 0);
    let events = t.events(FpySequencer::EVENTID_LOG_ACTIVITY_LO);
    let kept = fprime_config::FW_LOG_STRING_MAX_SIZE - 1;
    assert_eq!(&events[0][2..4], &(kept as u16).to_be_bytes());
    assert_eq!(&events[0][4..], &message[..kept]);
}

#[test]
fn pop_serializable_success() {
    let t = Tester::new();
    t.push(0x1234_5678u32);
    assert_eq!(
        call(
            &t,
            FpySequencer::pop_serializable_directive_handler,
            &PopSerializableDirective::new(0, 4)
        ),
        OK
    );
    assert_eq!(t.stack_size(), 0);
    assert_eq!(t.serial_calls(), vec![(0, vec![0x12, 0x34, 0x56, 0x78])]);
}

#[test]
fn pop_serializable_port_index_out_of_bounds() {
    let t = Tester::new();
    let handler = FpySequencer::pop_serializable_directive_handler;
    t.push(0xDEAD_BEEFu32);
    assert_eq!(
        call(
            &t,
            handler,
            &PopSerializableDirective::new(SerialPortIndex::MaxSerialPorts.as_repr(), 4)
        ),
        fail(DirectiveError::SerialPortInvalidIndex)
    );
    assert_eq!(
        call(&t, handler, &PopSerializableDirective::new(-1, 4)),
        fail(DirectiveError::SerialPortInvalidIndex)
    );
    assert_eq!(t.stack_size(), 4);
}

#[test]
fn pop_serializable_port_not_connected() {
    // the tester leaves serialOut[1] unconnected
    let t = Tester::new();
    t.push(0xABCD_1234u32);
    assert_eq!(
        call(
            &t,
            FpySequencer::pop_serializable_directive_handler,
            &PopSerializableDirective::new(1, 4)
        ),
        fail(DirectiveError::SerialPortNotConnected)
    );
    assert_eq!(t.stack_size(), 4);
}

#[test]
fn pop_serializable_stack_underflow() {
    let t = Tester::new();
    t.push(0x1234_5678u32);
    assert_eq!(
        call(
            &t,
            FpySequencer::pop_serializable_directive_handler,
            &PopSerializableDirective::new(0, 10)
        ),
        fail(DirectiveError::StackUnderflow)
    );
    assert_eq!(t.stack_size(), 4);
}

#[test]
fn pop_serializable_multiple_types() {
    let t = Tester::new();
    let handler = FpySequencer::pop_serializable_directive_handler;
    t.push(0xABu8);
    let sent_time = time(TimeBase::TbWorkstationTime, 7, 123, 456);
    {
        let mut st = t.st();
        let top = st.runtime.stack.top();
        let mut buf =
            fprime_fw::ExtBuf::new(&mut st.runtime.stack.bytes[top..top + Time::SERIALIZED_SIZE]);
        assert!(sent_time.serialize_to(&mut buf, Endianness::Big).is_ok());
        st.runtime.stack.size += Time::SERIALIZED_SIZE as StackSizeType;
    }
    t.push(DirectiveError::StackOverflow.as_repr());
    assert_eq!(t.stack_size(), 13);

    assert_eq!(call(&t, handler, &PopSerializableDirective::new(0, 1)), OK);
    assert_eq!(t.stack_size(), 12);
    assert_eq!(
        t.serial_calls(),
        vec![(0, vec![DirectiveError::StackOverflow.as_repr()])]
    );
    t.clear_history();

    assert_eq!(
        call(
            &t,
            handler,
            &PopSerializableDirective::new(0, Time::SERIALIZED_SIZE as StackSizeType)
        ),
        OK
    );
    assert_eq!(t.stack_size(), 1);
    assert_eq!(time_from(&t.serial_calls()[0].1), sent_time);
    t.clear_history();

    assert_eq!(call(&t, handler, &PopSerializableDirective::new(0, 1)), OK);
    assert_eq!(t.stack_size(), 0);
    assert_eq!(t.serial_calls(), vec![(0, vec![0xAB])]);
}

#[test]
fn pop_serializable_different_ports() {
    let t = Tester::new();
    let handler = FpySequencer::pop_serializable_directive_handler;
    t.push(0x1234u16);
    assert_eq!(call(&t, handler, &PopSerializableDirective::new(0, 2)), OK);
    assert_eq!(t.serial_calls()[0].0, 0);
    t.clear_history();
    t.push(0x5678u16);
    let last = SerialPortIndex::MaxSerialPorts.as_repr() - 1;
    assert_eq!(
        call(&t, handler, &PopSerializableDirective::new(last, 2)),
        OK
    );
    assert_eq!(t.serial_calls(), vec![(4, vec![0x56, 0x78])]);
}

// -- the timer checks ----------------------------------------------------------

fn should_wake(t: &Tester) -> Signal {
    t.with(|c, st| c.check_should_wake(st))
}

fn statement_timeout(t: &Tester) -> Signal {
    t.with(|c, st| c.check_statement_timeout(st))
}

#[test]
fn check_should_wake_mismatch_base() {
    let t = Tester::new();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 100, 100));
    t.st().runtime.wakeup_time = time(TimeBase::TbDontCare, 0, 100, 100).into();
    assert_eq!(should_wake(&t), Signal::result_timeOpFailed);
    assert_eq!(
        t.events(FpySequencer::EVENTID_MISMATCHED_TIME_BASE),
        vec![vec![0, 0, 0, 2, 0, 0, 0xFF, 0xFF]]
    );
}

#[test]
fn check_should_wake_mismatch_context() {
    // the context is not compared
    let t = Tester::new();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 100, 100));
    t.st().runtime.wakeup_time = time(TimeBase::TbWorkstationTime, 10, 100, 100).into();
    assert_eq!(should_wake(&t), Signal::result_checkShouldWake_wakeup);
}

#[test]
fn check_should_wake() {
    let t = Tester::new();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 100, 100));
    t.st().runtime.wakeup_time = time(TimeBase::TbWorkstationTime, 0, 200, 100).into();
    assert_eq!(should_wake(&t), Signal::result_checkShouldWake_keepSleeping);
    t.st().runtime.wakeup_time = time(TimeBase::TbWorkstationTime, 0, 50, 100).into();
    assert_eq!(should_wake(&t), Signal::result_checkShouldWake_wakeup);
    // the wakeup time itself wakes
    t.st().runtime.wakeup_time = time(TimeBase::TbWorkstationTime, 0, 100, 100).into();
    assert_eq!(should_wake(&t), Signal::result_checkShouldWake_wakeup);
}

fn set_dispatch_time(t: &Tester, dispatch: Time) {
    t.st().runtime.current_statement_dispatch_time = dispatch;
}

#[test]
fn check_statement_timeout() {
    let t = Tester::new();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 300, 100));
    t.set_timeout_param(0.0);
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_noTimeout
    );
    t.set_timeout_param(10.0);
    set_dispatch_time(&t, time(TimeBase::TbWorkstationTime, 0, 200, 100));
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_statementTimeout
    );
    set_dispatch_time(&t, time(TimeBase::TbWorkstationTime, 0, 295, 100));
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_noTimeout
    );
    set_dispatch_time(&t, time(TimeBase::TbWorkstationTime, 0, 290, 100));
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_statementTimeout
    );
    // a directive timed out (opcode 0 here)
    assert_eq!(t.event_count(FpySequencer::EVENTID_DIRECTIVE_TIMED_OUT), 2);
    // time running backwards is ignored
    set_dispatch_time(&t, time(TimeBase::TbWorkstationTime, 0, 300, 200));
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_noTimeout
    );
    set_dispatch_time(&t, time(TimeBase::TbWorkstationTime, 0, 301, 0));
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_noTimeout
    );
    // a timeout above U32 max seconds is no timeout
    t.set_timeout_param(1e10);
    set_dispatch_time(&t, time(TimeBase::TbWorkstationTime, 0, 0, 0));
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_noTimeout
    );
}

#[test]
fn check_statement_timeout_of_a_command_names_its_opcode() {
    let t = Tester::new();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 300, 0));
    t.set_timeout_param(1.0);
    set_dispatch_time(&t, time(TimeBase::TbWorkstationTime, 0, 200, 0));
    {
        let mut st = t.st();
        st.runtime.current_statement_opcode = DirectiveId::StackCmd.as_repr();
        st.runtime.current_cmd_opcode = 0x55;
        st.runtime.next_statement_index = 3;
    }
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_statementTimeout
    );
    // opcode, statement index 2, empty path
    assert_eq!(
        t.events(FpySequencer::EVENTID_COMMAND_TIMED_OUT),
        vec![vec![0, 0, 0, 0x55, 0, 0, 0, 2, 0, 0]]
    );
}

#[test]
fn check_statement_timeout_u32_overflow_missed() {
    // seconds are widened before * 1e6 (nasa/fprime#5424)
    let t = Tester::new();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 4300, 0));
    t.set_timeout_param(10.0);
    set_dispatch_time(&t, time(TimeBase::TbWorkstationTime, 0, 0, 0));
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_statementTimeout
    );
}

#[test]
fn check_statement_timeout_u32_overflow_spurious() {
    let t = Tester::new();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 4295, 0));
    t.set_timeout_param(10.0);
    set_dispatch_time(&t, time(TimeBase::TbWorkstationTime, 0, 4294, 0));
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_noTimeout
    );
}

#[test]
fn check_statement_timeout_mismatch_base() {
    let t = Tester::new();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 300, 100));
    t.set_timeout_param(10.0);
    set_dispatch_time(&t, time(TimeBase::TbDontCare, 0, 200, 100));
    assert_eq!(statement_timeout(&t), Signal::result_timeOpFailed);
}

#[test]
fn check_statement_timeout_mismatch_context() {
    let t = Tester::new();
    t.set_time(time(TimeBase::TbWorkstationTime, 0, 300, 100));
    t.set_timeout_param(10.0);
    set_dispatch_time(&t, time(TimeBase::TbWorkstationTime, 10, 200, 100));
    assert_eq!(
        statement_timeout(&t),
        Signal::result_checkStatementTimeout_statementTimeout
    );
}

#[test]
fn handle_directive_error_code_records_where_it_failed() {
    let t = Tester::new();
    let mut st = t.st();
    st.runtime.next_statement_index = 8;
    FpySequencer::handle_directive_error_code(
        &mut st,
        DirectiveId::Sdiv,
        DirectiveError::DomainError,
    );
    assert_eq!(st.tlm.last_directive_error, DirectiveError::DomainError);
    assert_eq!(st.tlm.directive_error_index, 7);
    assert_eq!(st.tlm.directive_error_id, DirectiveId::Sdiv);
    // success clears the last error but keeps where the last one was
    FpySequencer::handle_directive_error_code(&mut st, DirectiveId::NoOp, DirectiveError::NoError);
    assert_eq!(st.tlm.last_directive_error, DirectiveError::NoError);
    assert_eq!(st.tlm.directive_error_index, 7);
    assert_eq!(st.tlm.directive_error_id, DirectiveId::Sdiv);
}

#[test]
#[should_panic(expected = "Assert")]
fn wait_abs_with_a_second_of_microseconds_asserts_like_cpp() {
    // C++ parity: `Fw::Time(base, ctx, s, us)` asserts us < 1,000,000
    let t = Tester::new();
    t.push(2u16);
    t.push(0u8);
    t.push(1u32);
    t.push(1_000_000u32);
    call(
        &t,
        FpySequencer::wait_abs_directive_handler,
        &WaitAbsDirective::default(),
    );
}
