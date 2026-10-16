//! The sleep host functions and the `checkTimers` wake path.

use super::*;

/// WASM-SEQ-015: a 1 s relative sleep wakes on the first tick past it.
#[test]
fn relative_sleep_wakes() {
    let t = Tester::new();
    let file = t.stage("rsleep.wasm");
    t.set_time(time(0, 0));
    t.send_run(100, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_SLEEPING);
    assert!(t.has_pending_timer());
    // Before the deadline: still asleep.
    t.set_time(time(0, 500_000));
    t.check_timers();
    t.dispatch_all();
    assert_eq!(
        t.interpreter(),
        InterpreterState::RUNNING_AWAITING_RESPONSE_SLEEPING
    );
    // Past it: wakes and finishes.
    t.set_time(time(2, 0));
    t.check_timers();
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-015: a deadline past `U32` seconds saturates instead of
/// wrapping into the past.
#[test]
fn relative_sleep_deadline_overflow_does_not_wake_early() {
    let t = Tester::new();
    let file = t.stage("rsleep_near_max.wasm");
    t.set_time(time(10, 0));
    t.send_run(104, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_SLEEPING);
    assert!(t.has_pending_timer());
    t.assert_events(W::EVENTID_SLEEP_DURATION_TOO_LARGE, 0);
    t.set_time(time(1_000_000, 0));
    t.check_timers();
    t.dispatch_all();
    assert_eq!(
        t.interpreter(),
        InterpreterState::RUNNING_AWAITING_RESPONSE_SLEEPING
    );
    assert!(t.has_pending_timer());
}

/// WASM-SEQ-015: an absolute sleep (to 10 s).
#[test]
fn absolute_sleep_wakes() {
    let t = Tester::new();
    let file = t.stage("asleep.wasm");
    t.set_time(time(0, 0));
    t.send_run(101, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_SLEEPING);
    assert!(t.has_pending_timer());
    t.set_time(time(20, 0));
    t.check_timers();
    t.dispatch_until_controller(ControllerState::READY);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-015: a time base change makes the timer incomparable.
#[test]
fn sleep_time_base_mismatch_fails() {
    let t = Tester::new();
    let file = t.stage("rsleep.wasm");
    t.set_time(time_in(TimeBase::TbWorkstationTime, 0, 0));
    t.send_run(102, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_AWAITING_RESPONSE_SLEEPING);
    assert!(t.has_pending_timer());
    t.set_time(time_in(TimeBase::TbScTime, 5, 0));
    t.check_timers();
    t.dispatch_until_controller(ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 0);
}

fn sleep_too_large(kind: HostFunction) -> Bytes {
    Bytes::new().ser(&kind).u64(0x7FFF_FFFF_FFFF_FFFF)
}

/// WASM-SEQ-019: an `rsleep` whose seconds overflow `U32` traps.
#[test]
fn sleep_duration_overflow_fails() {
    let t = Tester::new();
    let file = t.stage("sleep_overflow.wasm");
    t.send_run(103, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_SLEEP_DURATION_TOO_LARGE, 1);
    t.assert_event(
        W::EVENTID_SLEEP_DURATION_TOO_LARGE,
        0,
        sleep_too_large(HostFunction::Rsleep),
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

/// WASM-SEQ-019: the `asleep` analogue.
#[test]
fn absolute_sleep_duration_overflow_fails() {
    let t = Tester::new();
    let file = t.stage("asleep_overflow.wasm");
    t.send_run(104, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_SLEEP_DURATION_TOO_LARGE, 1);
    t.assert_event(
        W::EVENTID_SLEEP_DURATION_TOO_LARGE,
        0,
        sleep_too_large(HostFunction::Asleep),
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
