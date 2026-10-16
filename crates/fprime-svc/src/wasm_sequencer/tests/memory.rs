//! `memory.grow`: the guest bump allocator, end to end and white box
//! (offsets stand in for the C++ pointers), and the lifecycle test.

use super::*;

/// WASM-SEQ-022: the last allocation grows in place.
#[test]
fn memory_grow_succeeds_and_region_usable() {
    let t = Tester::new();
    let file = t.stage("mem_grow.wasm");
    t.send_run(600, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.assert_response(0, W::OPCODE_RUN, 600, CmdResponse::Ok);
    t.assert_sequence_failure_count(0);
}

/// WASM-SEQ-022: a grow past the pool returns -1 to the guest.
#[test]
fn memory_grow_beyond_pool_returns_minus_one_gracefully() {
    let t = Tester::new();
    let file = t.stage("mem_grow_toobig.wasm");
    t.send_run(601, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.assert_response(0, W::OPCODE_RUN, 601, CmdResponse::Ok);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_MEMORY_GROW_REJECTED, 1);
    t.assert_event(
        W::EVENTID_MEMORY_GROW_REJECTED,
        0,
        Bytes::new()
            .ser(&MemoryGrowFailReason::InsufficientPoolSpace)
            .u64(4097),
    );
}

fn pool(t: &Tester) -> Arc<GuestPool> {
    t.comp.guest_pool().unwrap()
}

/// WASM-SEQ-022.
#[test]
fn guest_realloc_grows_last_allocation_in_place() {
    let t = Tester::new();
    let p = pool(&t);
    let a = p.alloc_offset(100, 1).unwrap();
    assert_eq!(t.guest_offset(), 100);
    assert_eq!(p.realloc_offset(a, 100, 200, 1), Some(a));
    assert_eq!(t.guest_offset(), 200);
    t.assert_events(W::EVENTID_MEMORY_GROW_REJECTED, 0);
}

/// WASM-SEQ-022: only the last allocation may grow.
#[test]
fn guest_realloc_rejects_non_last_allocation() {
    let t = Tester::new();
    let p = pool(&t);
    let a = p.alloc_offset(100, 1).unwrap();
    p.alloc_offset(50, 1).unwrap();
    let before = t.guest_offset();
    assert_eq!(p.realloc_offset(a, 100, 120, 1), None);
    assert_eq!(t.guest_offset(), before);
    t.assert_events(W::EVENTID_MEMORY_GROW_REJECTED, 1);
    t.assert_event(
        W::EVENTID_MEMORY_GROW_REJECTED,
        0,
        Bytes::new()
            .ser(&MemoryGrowFailReason::NotLastAllocation)
            .u64(120),
    );
}

/// WASM-SEQ-022.
#[test]
fn guest_realloc_rejects_grow_beyond_pool() {
    let t = Tester::new();
    let p = pool(&t);
    let a = p.alloc_offset(100, 1).unwrap();
    let before = t.guest_offset();
    let too_big = GUEST_POOL_SIZE as usize + 1;
    assert_eq!(p.realloc_offset(a, 100, too_big, 1), None);
    assert_eq!(t.guest_offset(), before);
    t.assert_events(W::EVENTID_MEMORY_GROW_REJECTED, 1);
    t.assert_event(
        W::EVENTID_MEMORY_GROW_REJECTED,
        0,
        Bytes::new()
            .ser(&MemoryGrowFailReason::InsufficientPoolSpace)
            .u64(too_big as u64),
    );
}

/// WASM-SEQ-022: growing to exactly the end of the pool is allowed.
#[test]
fn guest_realloc_exact_fit_to_pool_end_succeeds() {
    let t = Tester::new();
    let p = pool(&t);
    let a = p.alloc_offset(100, 1).unwrap();
    let exact = GUEST_POOL_SIZE as usize;
    assert_eq!(p.realloc_offset(a, 100, exact, 1), Some(a));
    assert_eq!(t.guest_offset(), exact);
    t.assert_events(W::EVENTID_MEMORY_GROW_REJECTED, 0);
}

/// WASM-SEQ-022: a 4 GiB grow is not truncated into a small one.
#[test]
fn guest_realloc_rejects_four_gib_grow_without_truncation() {
    let t = Tester::new();
    let p = pool(&t);
    let a = p.alloc_offset(100, 1).unwrap();
    let before = t.guest_offset();
    if size_of::<usize>() >= 8 {
        let four_gib = 1usize << 32;
        assert_eq!(p.realloc_offset(a, 100, four_gib, 1), None);
        assert_eq!(t.guest_offset(), before);
        t.assert_events(W::EVENTID_MEMORY_GROW_REJECTED, 1);
        t.assert_event(
            W::EVENTID_MEMORY_GROW_REJECTED,
            0,
            Bytes::new()
                .ser(&MemoryGrowFailReason::InsufficientPoolSpace)
                .u64(four_gib as u64),
        );
    }
}

/// WASM-SEQ-022: a module whose memory is no longer last cannot grow.
#[test]
fn memory_grow_fails_when_not_last_allocation() {
    let t = Tester::new();
    let a = t.stage("mem_grow_notlast.wasm");
    let b = t.stage("mem_holder.wasm");
    t.send_load(700, &a, "a");
    t.dispatch_all();
    t.send_load(701, &b, "b");
    t.dispatch_all();
    t.send_invoke(702, "a", BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_MEMORY_GROW_REJECTED, 1);
    t.assert_event(
        W::EVENTID_MEMORY_GROW_REJECTED,
        0,
        Bytes::new()
            .ser(&MemoryGrowFailReason::NotLastAllocation)
            .u64(64),
    );
}

/// WASM-SEQ-003, WASM-SEQ-021: many loads and runs with failures between;
/// the store recovers every time and the counters add up.
#[test]
fn lifecycle_multiple_loads_with_failures() {
    let t = Tester::new();
    t.param_set_fuel(10);
    let empty = t.stage("empty.wasm");
    let unreachable = t.stage("unreachable.wasm");
    let malformed = t.stage("malformed.wasm");
    let looping = t.stage("loop.wasm");
    let exit0 = t.stage("exit0.wasm");
    let panic = t.stage("panic.wasm");
    let none = SeqArgs::default();
    let (mut succeeded, mut cancelled, mut seq_failed_events, mut failed) = (0, 0, 0, 0u64);

    t.send_run(1, &empty, BLOCK, &none);
    t.dispatch_until_controller(ControllerState::READY);
    succeeded += 1;
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, succeeded);

    t.send_run(2, &unreachable, BLOCK, &none);
    t.dispatch_until_controller(ControllerState::IDLE);
    seq_failed_events += 1;
    failed += 1;
    t.assert_sequence_failure_count(seq_failed_events);

    t.send_load(3, &empty, "");
    t.dispatch_until_controller(ControllerState::READY);

    t.send_run(4, &malformed, BLOCK, &none);
    t.dispatch_until_controller(ControllerState::IDLE);
    failed += 1;
    t.assert_events(W::EVENTID_MODULE_LOAD_FAILED, 1);
    t.assert_sequence_failure_count(seq_failed_events);

    t.send_run(5, &looping, NO_BLOCK, &none);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
    t.send_cancel(6);
    t.dispatch_until_controller(ControllerState::IDLE);
    cancelled += 1;
    t.assert_events(W::EVENTID_SEQUENCE_CANCELLED, cancelled);

    t.send_run(7, &exit0, BLOCK, &none);
    t.dispatch_until_controller(ControllerState::READY);
    succeeded += 1;
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, succeeded);

    t.send_run(8, &panic, BLOCK, &none);
    t.dispatch_until_controller(ControllerState::IDLE);
    seq_failed_events += 1;
    failed += 1;
    t.assert_sequence_failure_count(seq_failed_events);

    t.flush_telemetry();
    t.assert_tlm(
        W::CHANID_SEQUENCES_SUCCEEDED,
        0,
        Bytes::new().u64(succeeded as u64),
    );
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(failed));
    t.assert_tlm(
        W::CHANID_SEQUENCES_CANCELLED,
        0,
        Bytes::new().u64(cancelled as u64),
    );
    let expected = [
        (W::OPCODE_RUN, 1, CmdResponse::Ok),
        (W::OPCODE_RUN, 2, CmdResponse::ExecutionError),
        (W::OPCODE_LOAD, 3, CmdResponse::Ok),
        (W::OPCODE_RUN, 4, CmdResponse::ExecutionError),
        (W::OPCODE_RUN, 5, CmdResponse::Ok),
        (W::OPCODE_CANCEL, 6, CmdResponse::Ok),
        (W::OPCODE_RUN, 7, CmdResponse::Ok),
        (W::OPCODE_RUN, 8, CmdResponse::ExecutionError),
    ];
    for (i, (opcode, seq, response)) in expected.into_iter().enumerate() {
        t.assert_response(i, opcode, seq, response);
    }
}
