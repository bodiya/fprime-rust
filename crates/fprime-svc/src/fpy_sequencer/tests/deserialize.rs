//! Statement decoding (upstream `deserialize_*`, `dispatchStatement`).

use super::*;

fn deserialize(t: &Tester, stmt: &Statement) -> Option<DirectiveUnion> {
    t.with(|c, st| c.deserialize_directive(st, stmt))
}

/// Upstream's pattern for every directive: the statement decodes to the
/// directive; a trailing `I32` makes it fail with
/// `DirectiveDeserializeError`; with no argument bytes at all it decodes
/// again only if the directive takes none.
fn check(build: impl FnOnce(&mut Tester), expected: DirectiveUnion, takes_no_args: bool) {
    let mut t = Tester::new();
    build(&mut t);
    let mut stmt = t.seq.statements[0].clone();
    assert_eq!(deserialize(&t, &stmt), Some(expected.clone()));

    assert!(stmt.arg_buf.serialize_i32_be(123).is_ok());
    assert_eq!(deserialize(&t, &stmt), None);
    assert_eq!(
        t.event_count(FpySequencer::EVENTID_DIRECTIVE_DESERIALIZE_ERROR),
        1
    );
    t.clear_history();

    stmt.arg_buf.reset_ser();
    let result = deserialize(&t, &stmt);
    if takes_no_args {
        assert_eq!(result, Some(expected));
        assert_eq!(
            t.event_count(FpySequencer::EVENTID_DIRECTIVE_DESERIALIZE_ERROR),
            0
        );
    } else {
        assert_eq!(result, None);
        assert_eq!(
            t.event_count(FpySequencer::EVENTID_DIRECTIVE_DESERIALIZE_ERROR),
            1
        );
    }
}

#[test]
fn deserialize_wait_rel() {
    check(
        |t| t.add_wait_rel(),
        DirectiveUnion::WaitRel(WaitRelDirective::default()),
        true,
    );
}

#[test]
fn deserialize_wait_abs() {
    check(
        |t| t.add_wait_abs(),
        DirectiveUnion::WaitAbs(WaitAbsDirective::default()),
        true,
    );
}

#[test]
fn deserialize_goto() {
    check(
        |t| t.add_goto(123),
        DirectiveUnion::Goto(GotoDirective::new(123)),
        false,
    );
}

#[test]
fn deserialize_if() {
    check(
        |t| t.add_if(123),
        DirectiveUnion::If(IfDirective::new(123)),
        false,
    );
}

#[test]
fn deserialize_no_op() {
    check(
        |t| t.add_no_op(),
        DirectiveUnion::NoOp(NoOpDirective::default()),
        true,
    );
}

#[test]
fn deserialize_push_tlm_val() {
    check(
        |t| t.add_push_tlm_val(123),
        DirectiveUnion::PushTlmVal(PushTlmValDirective::new(123)),
        false,
    );
}

#[test]
fn deserialize_push_tlm_val_and_time() {
    check(
        |t| t.add_push_tlm_val_and_time(123),
        DirectiveUnion::PushTlmValAndTime(PushTlmValAndTimeDirective::new(123)),
        false,
    );
}

#[test]
fn deserialize_push_prm() {
    check(
        |t| t.add_push_prm(123),
        DirectiveUnion::PushPrm(PushPrmDirective::new(123)),
        false,
    );
}

#[test]
fn deserialize_stack_op() {
    check(
        |t| t.add_stack_op(DirectiveId::And),
        DirectiveUnion::StackOp(StackOpDirective::new(DirectiveId::And)),
        true,
    );
}

#[test]
fn deserialize_exit() {
    check(
        |t| t.add_exit(),
        DirectiveUnion::Exit(ExitDirective::default()),
        true,
    );
}

#[test]
fn deserialize_discard() {
    check(
        |t| t.add_discard(123),
        DirectiveUnion::Discard(DiscardDirective::new(123)),
        false,
    );
}

#[test]
fn deserialize_stack_cmd() {
    check(
        |t| t.add_stack_cmd(123),
        DirectiveUnion::StackCmd(StackCmdDirective::new(123)),
        false,
    );
}

#[test]
fn deserialize_set_seed() {
    check(
        |t| t.add_set_seed(),
        DirectiveUnion::SetSeed(SetSeedDirective::default()),
        true,
    );
}

#[test]
fn deserialize_push_rand() {
    check(
        |t| t.add_push_rand(),
        DirectiveUnion::PushRand(PushRandDirective::default()),
        true,
    );
}

#[test]
fn deserialize_mem_cmp() {
    check(
        |t| t.add_memcmp(123),
        DirectiveUnion::MemCmp(MemCmpDirective::new(123)),
        false,
    );
}

#[test]
fn deserialize_get_field() {
    check(
        |t| t.add_get_field(123, 123),
        DirectiveUnion::GetField(GetFieldDirective::new(123, 123)),
        false,
    );
}

#[test]
fn deserialize_store_rel() {
    check(
        |t| t.add_store_rel(123),
        DirectiveUnion::StoreRel(StoreRelDirective::new(123)),
        false,
    );
}

#[test]
fn deserialize_peek() {
    check(
        |t| t.add_peek(),
        DirectiveUnion::Peek(PeekDirective::default()),
        true,
    );
}

#[test]
fn deserialize_pop_event() {
    check(
        |t| t.add_pop_event(),
        DirectiveUnion::PopEvent(PopEventDirective::default()),
        true,
    );
}

#[test]
fn deserialize_pop_serializable() {
    check(
        |t| t.add_pop_serializable(0, 4),
        DirectiveUnion::PopSerializable(PopSerializableDirective::new(0, 4)),
        false,
    );
}

#[test]
fn deserialize_the_remaining_directives() {
    // not covered upstream: the other argument-carrying directives
    check(
        |t| t.add_allocate(9),
        DirectiveUnion::Allocate(AllocateDirective::new(9)),
        false,
    );
    check(
        |t| t.add_store_rel_const_offset(-3, 2),
        DirectiveUnion::StoreRelConstOffset(StoreRelConstOffsetDirective::new(-3, 2)),
        false,
    );
    check(
        |t| t.add_load_rel(-3, 2),
        DirectiveUnion::LoadRel(LoadRelDirective::new(-3, 2)),
        false,
    );
    check(
        |t| t.add_return(4, 8),
        DirectiveUnion::Return(ReturnDirective::new(4, 8)),
        false,
    );
    check(
        |t| t.add_load_abs(1, 2),
        DirectiveUnion::LoadAbs(LoadAbsDirective::new(1, 2)),
        false,
    );
    check(
        |t| t.add_store_abs(2),
        DirectiveUnion::StoreAbs(StoreAbsDirective::new(2)),
        false,
    );
    check(
        |t| t.add_store_abs_const_offset(1, 2),
        DirectiveUnion::StoreAbsConstOffset(StoreAbsConstOffsetDirective::new(1, 2)),
        false,
    );
    check(
        |t| t.add_call(),
        DirectiveUnion::Call(CallDirective::default()),
        true,
    );
    check(
        |t| t.add_push_time(),
        DirectiveUnion::PushTime(PushTimeDirective::default()),
        true,
    );
}

#[test]
fn deserialize_const_cmd_and_push_val_take_every_remaining_byte() {
    let mut t = Tester::new();
    t.add_directive(DirectiveId::ConstCmd, &[0, 0, 0, 7, 1, 2, 3]);
    t.add_directive(DirectiveId::PushVal, &[9, 8]);
    t.add_directive(DirectiveId::PushVal, &[]);
    t.add_directive(DirectiveId::ConstCmd, &[0, 0, 7]);
    let Some(DirectiveUnion::ConstCmd(cmd)) = deserialize(&t, &t.seq.statements[0].clone()) else {
        panic!("not a CONST_CMD");
    };
    assert_eq!((cmd.op_code, cmd.arg_buf_size), (7, 3));
    assert_eq!(&cmd.arg_buf.0[..3], &[1, 2, 3]);
    let Some(DirectiveUnion::PushVal(val)) = deserialize(&t, &t.seq.statements[1].clone()) else {
        panic!("not a PUSH_VAL");
    };
    assert_eq!(val.val_size, 2);
    assert_eq!(&val.val.0[..2], &[9, 8]);
    // an empty PUSH_VAL pushes nothing
    let Some(DirectiveUnion::PushVal(val)) = deserialize(&t, &t.seq.statements[2].clone()) else {
        panic!("not a PUSH_VAL");
    };
    assert_eq!(val.val_size, 0);
    // a CONST_CMD needs its whole opcode
    assert_eq!(deserialize(&t, &t.seq.statements[3].clone()), None);
    let events = t.events(FpySequencer::EVENTID_DIRECTIVE_DESERIALIZE_ERROR);
    // opcode 8, index 0, DeserSizeMismatch (5), 3 bytes left of 3
    assert_eq!(
        events,
        vec![vec![
            8, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 3
        ]]
    );
}

#[test]
fn deserialize_invalid_is_an_unknown_directive() {
    let mut t = Tester::new();
    t.add_directive(DirectiveId::Invalid, &[]);
    assert_eq!(deserialize(&t, &t.seq.statements[0].clone()), None);
    // opcode 0, index 0, the (empty) path
    assert_eq!(
        t.events(FpySequencer::EVENTID_UNKNOWN_SEQUENCER_DIRECTIVE),
        vec![vec![0, 0, 0, 0, 0, 0, 0]]
    );
}

#[test]
fn dispatch_statement() {
    let mut t = Tester::new();
    let now = Time::from_seconds_useconds(123, 123);
    t.set_time(now);
    t.add_no_op();
    let seq = t.seq.clone();
    t.st().sequence_obj.assign(&seq);
    assert_eq!(
        t.with(|c, st| c.dispatch_statement(st)),
        Signal::result_dispatchStatement_success
    );
    {
        let st = t.st();
        assert_eq!(
            st.runtime.current_statement_opcode,
            DirectiveId::NoOp.as_repr()
        );
        assert_eq!(st.runtime.current_statement_dispatch_time, now);
        assert_eq!(st.statements_dispatched, 1);
    }
    // the directive went to its internal port
    assert_eq!(t.queued(), 1);
    assert_eq!(
        t.with(|c, st| c.dispatch_statement(st)),
        Signal::result_dispatchStatement_noMoreStatements
    );
    assert_eq!(t.st().statements_dispatched, 1);

    // a command records the opcode it waits for
    t.clear_seq();
    let now = Time::from_seconds_useconds(456, 123);
    t.set_time(now);
    t.add_const_cmd(123);
    let seq = t.seq.clone();
    t.st().sequence_obj.assign(&seq);
    t.st().runtime.next_statement_index = 0;
    assert_eq!(
        t.with(|c, st| c.dispatch_statement(st)),
        Signal::result_dispatchStatement_success
    );
    {
        let st = t.st();
        assert_eq!(
            st.runtime.current_statement_opcode,
            DirectiveId::ConstCmd.as_repr()
        );
        assert_eq!(st.runtime.current_cmd_opcode, 123);
        assert_eq!(st.runtime.current_statement_dispatch_time, now);
    }

    // a statement that does not decode fails the dispatch
    t.clear_seq();
    t.add_directive(DirectiveId::Goto, &[1]);
    let seq = t.seq.clone();
    t.st().sequence_obj.assign(&seq);
    t.st().runtime.next_statement_index = 0;
    assert_eq!(
        t.with(|c, st| c.dispatch_statement(st)),
        Signal::result_dispatchStatement_failure
    );
}

#[test]
#[should_panic(expected = "Assert")]
fn dispatch_statement_past_the_end_asserts() {
    let mut t = Tester::new();
    t.add_no_op();
    let seq = t.seq.clone();
    t.st().sequence_obj.assign(&seq);
    t.st().runtime.next_statement_index = 2;
    t.with(|c, st| c.dispatch_statement(st));
}

#[test]
fn directive_messages_are_the_serialized_directive() {
    // `directive_constCmd` carries the whole FPP struct: opcode, 2048
    // argument bytes, the argument size
    let t = Tester::new();
    let mut directive = ConstCmdDirective::new(0x0102_0304, DirectiveBytes::default(), 2);
    directive.arg_buf.0[..2].copy_from_slice(&[0xAA, 0xBB]);
    t.comp
        .dispatch_directive(&DirectiveUnion::ConstCmd(directive), DirectiveId::ConstCmd);
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
    assert_eq!(priority, 6);
    assert_eq!(actual as usize, QUEUE_MSG_SIZE);
    assert_eq!(
        &bytes[..6],
        &[
            0,
            0,
            0,
            FpySequencer::MSG_TYPE_DIRECTIVE_CONST_CMD as u8,
            0,
            0
        ]
    );
    assert_eq!(&bytes[6..12], &[1, 2, 3, 4, 0xAA, 0xBB]);
    assert_eq!(&bytes[QUEUE_MSG_SIZE - 8..], &[0, 0, 0, 0, 0, 0, 0, 2]);
}
