//! The host functions: `event`, `tlm`, `prm`, `args`, `time`, and the
//! port-not-connected guards of `tlm`, `prm` and `cmd`.

use super::*;

/// A RUN of `name` with `args` (blocking), drained to completion.
fn run(t: &Tester, cmd_seq: u32, name: &str, args: &SeqArgs) {
    let file = t.stage(name);
    t.send_run(cmd_seq, &file, BLOCK, args);
    t.dispatch_all();
}

fn trapped(reason: TrapReason) -> Bytes {
    Bytes::new().u8(0).ser(&SequencePhase::Main).ser(&reason)
}

fn host_fn(kind: HostFunction) -> Bytes {
    Bytes::new().ser(&kind)
}

const TIME_SIZE: u32 = Time::SERIALIZED_SIZE as u32;

// -- EVENT ------------------------------------------------------------------

/// WASM-SEQ-014.
#[test]
fn event_activity_hi() {
    let t = Tester::new();
    run(&t, 60, "event.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_LOG_ACTIVITY_HI, 1);
    t.assert_event(
        W::EVENTID_LOG_ACTIVITY_HI,
        0,
        Bytes::new().str("hello wasm"),
    );
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 0);
    // The severity of the guest event is the requested one.
    let event = t
        .all_events()
        .into_iter()
        .find(|e| e.0 == W::EVENTID_LOG_ACTIVITY_HI)
        .unwrap();
    assert_eq!(event.1, LogSeverity::ActivityHi);
}

/// WASM-SEQ-014, WASM-SEQ-019: severities 1..7; FATAL and COMMAND are
/// rejected with the message preserved.
#[test]
fn event_all_severities() {
    let t = Tester::new();
    run(&t, 61, "event_all_sev.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::READY);
    for id in [
        W::EVENTID_LOG_WARNING_HI,
        W::EVENTID_LOG_WARNING_LO,
        W::EVENTID_LOG_ACTIVITY_HI,
        W::EVENTID_LOG_ACTIVITY_LO,
        W::EVENTID_LOG_DIAGNOSTIC,
    ] {
        t.assert_events(id, 1);
        t.assert_event(id, 0, Bytes::new().str("sev"));
    }
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_SEVERITY, 2);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_SEVERITY,
        0,
        Bytes::new()
            .i32(LogSeverity::Fatal.as_repr().into())
            .str("sev"),
    );
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_SEVERITY,
        1,
        Bytes::new()
            .i32(LogSeverity::Command.as_repr().into())
            .str("sev"),
    );
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-014, WASM-SEQ-019: a 250-byte length is clamped to
/// FW_LOG_STRING_MAX_SIZE when read, and the event argument to its
/// declared 128.
#[test]
fn event_message_truncated_to_max() {
    let t = Tester::new();
    run(&t, 63, "event_toobig.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_LOG_ACTIVITY_HI, 1);
    t.assert_event(
        W::EVENTID_LOG_ACTIVITY_HI,
        0,
        Bytes::new().str(&"A".repeat(128)),
    );
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-019: severity 99 is reported and the guest continues.
#[test]
fn event_bad_severity_reported() {
    let t = Tester::new();
    run(&t, 62, "event_bad_sev.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_SEVERITY, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_SEVERITY,
        0,
        Bytes::new().i32(99).str("x"),
    );
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 0);
}

// -- TELEMETRY ----------------------------------------------------------------

fn tlm_value(t: &Tester, value: [u8; 4]) {
    t.set_next_tlm(42, Time::from_seconds_useconds(1000, 2000), &value);
}

/// WASM-SEQ-012.
#[test]
fn telemetry_read() {
    let t = Tester::new();
    tlm_value(&t, [0x11, 0x22, 0x33, 0x44]);
    run(&t, 70, "tlm.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::READY);
    assert_eq!(*t.g.tlm_gets.lock().unwrap(), [42]);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 1);
}

/// WASM-SEQ-019: an unconnected `getTlmChan` traps instead of asserting.
#[test]
fn telemetry_read_port_not_connected_traps() {
    let t = Tester::with(Options {
        get_tlm_chan: false,
        ..Options::default()
    });
    run(&t, 74, "tlm.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_PORT, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_PORT,
        0,
        host_fn(HostFunction::Telemetry).i32(0).u32(0),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.g.tlm_gets.lock().unwrap().is_empty());
}

/// WASM-SEQ-019: an unconnected `getParam` traps.
#[test]
fn parameter_read_port_not_connected_traps() {
    let t = Tester::with(Options {
        get_param: false,
        ..Options::default()
    });
    run(&t, 75, "prm.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_PORT, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_PORT,
        0,
        host_fn(HostFunction::Parameter).i32(0).u32(0),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.g.prm_gets.lock().unwrap().is_empty());
}

/// WASM-SEQ-019: a negative channel id is rejected, not aliased.
#[test]
fn telemetry_read_id_negative_traps() {
    let t = Tester::new();
    run(&t, 77, "tlm_badid_negative.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_ID, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_ID,
        0,
        host_fn(HostFunction::Telemetry).i64(-1),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.g.tlm_gets.lock().unwrap().is_empty());
}

/// WASM-SEQ-019: a channel id past `U32` is rejected.
#[test]
fn telemetry_read_id_too_large_traps() {
    let t = Tester::new();
    run(&t, 78, "tlm_badid_toolarge.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_ID, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_ID,
        0,
        host_fn(HostFunction::Telemetry).i64(0x1_0000_0000),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.g.tlm_gets.lock().unwrap().is_empty());
}

/// WASM-SEQ-019.
#[test]
fn parameter_read_id_negative_traps() {
    let t = Tester::new();
    run(&t, 79, "prm_badid_negative.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_ID, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_ID,
        0,
        host_fn(HostFunction::Parameter).i64(-1),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.g.prm_gets.lock().unwrap().is_empty());
}

/// WASM-SEQ-019.
#[test]
fn parameter_read_id_too_large_traps() {
    let t = Tester::new();
    run(&t, 80, "prm_badid_toolarge.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_ID, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_ID,
        0,
        host_fn(HostFunction::Parameter).i64(0x1_0000_0000),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.g.prm_gets.lock().unwrap().is_empty());
}

/// WASM-SEQ-019: an unconnected `cmdOut` traps.
#[test]
fn command_port_not_connected_traps() {
    let t = Tester::with(Options {
        cmd_out: false,
        ..Options::default()
    });
    run(&t, 76, "cmd.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_PORT, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_PORT,
        0,
        host_fn(HostFunction::Command).i32(0).u32(0),
    );
    t.assert_sequence_failure_count(1);
    assert!(t.cmd_out().is_empty());
}

/// WASM-SEQ-012: the guest checks the value it read and traps on a
/// mismatch.
#[test]
fn telemetry_read_value_mismatch_traps() {
    let t = Tester::new();
    tlm_value(&t, [0xDE, 0xAD, 0xBE, 0xEF]);
    run(&t, 73, "tlm.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(*t.g.tlm_gets.lock().unwrap(), [42]);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_TRAPPED,
        0,
        trapped(TrapReason::Unreachable),
    );
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 0);
    assert_eq!(t.port_history_size(), 1);
}

/// WASM-SEQ-019: a time buffer smaller than a serialized time.
#[test]
fn telemetry_bad_time_size_traps() {
    let t = Tester::new();
    run(&t, 71, "tlm_badtime.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_SMALL, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_SMALL,
        0,
        host_fn(HostFunction::Telemetry).u32(8).u32(TIME_SIZE),
    );
    t.assert_sequence_failure_count(1);
    t.assert_event(W::EVENTID_SEQUENCE_TRAPPED, 0, trapped(TrapReason::Host));
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-019: a time buffer larger than a serialized time.
#[test]
fn telemetry_bad_time_size_too_large_traps() {
    let t = Tester::new();
    run(&t, 74, "tlm_bigtime.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_LARGE, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_LARGE,
        0,
        host_fn(HostFunction::Telemetry).u32(16).u32(TIME_SIZE),
    );
    t.assert_sequence_failure_count(1);
    t.assert_event(W::EVENTID_SEQUENCE_TRAPPED, 0, trapped(TrapReason::Host));
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-012, WASM-SEQ-019: an oversized value buffer gets only the
/// value bytes (the guest checks the byte after them).
#[test]
fn telemetry_oversized_request_writes_only_value_bytes() {
    let t = Tester::new();
    tlm_value(&t, [0x11, 0x22, 0x33, 0x44]);
    run(&t, 72, "tlm_largebuf.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::READY);
    assert_eq!(*t.g.tlm_gets.lock().unwrap(), [42]);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_BUFFER_TOO_LARGE, 0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 1);
}

/// WASM-SEQ-019: a value buffer smaller than the value.
#[test]
fn telemetry_undersized_request_fails() {
    let t = Tester::new();
    tlm_value(&t, [0x11, 0x22, 0x33, 0x44]);
    run(&t, 74, "tlm_toosmall.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(*t.g.tlm_gets.lock().unwrap(), [42]);
    t.assert_events(W::EVENTID_BUFFER_TOO_SMALL, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_SMALL,
        0,
        host_fn(HostFunction::Telemetry).u32(2).u32(4),
    );
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 1);
}

// -- PARAMETER ----------------------------------------------------------------

/// WASM-SEQ-012.
#[test]
fn parameter_read() {
    let t = Tester::new();
    t.set_next_prm(7, &[0xAA, 0xBB, 0xCC, 0xDD]);
    run(&t, 80, "prm.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::READY);
    assert_eq!(*t.g.prm_gets.lock().unwrap(), [7]);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 1);
}

/// WASM-SEQ-012.
#[test]
fn parameter_read_value_mismatch_traps() {
    let t = Tester::new();
    t.set_next_prm(7, &[0x00, 0x11, 0x22, 0x33]);
    run(&t, 82, "prm.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(*t.g.prm_gets.lock().unwrap(), [7]);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_TRAPPED,
        0,
        trapped(TrapReason::Unreachable),
    );
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 0);
    assert_eq!(t.port_history_size(), 1);
}

/// WASM-SEQ-012, WASM-SEQ-019.
#[test]
fn parameter_oversized_request_writes_only_value_bytes() {
    let t = Tester::new();
    t.set_next_prm(7, &[0xAA, 0xBB, 0xCC, 0xDD]);
    run(&t, 81, "prm_largebuf.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::READY);
    assert_eq!(*t.g.prm_gets.lock().unwrap(), [7]);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_BUFFER_TOO_LARGE, 0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 1);
}

/// WASM-SEQ-019.
#[test]
fn parameter_undersized_request_fails() {
    let t = Tester::new();
    t.set_next_prm(7, &[0xAA, 0xBB, 0xCC, 0xDD]);
    run(&t, 83, "prm_toosmall.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(*t.g.prm_gets.lock().unwrap(), [7]);
    t.assert_events(W::EVENTID_BUFFER_TOO_SMALL, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_SMALL,
        0,
        host_fn(HostFunction::Parameter).u32(2).u32(4),
    );
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 1);
}

// -- ARGS -------------------------------------------------------------------

/// WASM-SEQ-005: the RUN arguments round-trip into guest memory.
#[test]
fn args_round_trip() {
    let t = Tester::new();
    run(
        &t,
        200,
        "args.wasm",
        &make_seq_args(&[0xCA, 0xFE, 0xBA, 0xBE]),
    );
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-005: negative control (the guest checks the pattern).
#[test]
fn args_round_trip_mismatch_traps() {
    let t = Tester::new();
    run(
        &t,
        201,
        "args.wasm",
        &make_seq_args(&[0xDE, 0xAD, 0xBE, 0xEF]),
    );
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_TRAPPED,
        0,
        trapped(TrapReason::Unreachable),
    );
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 0);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-005.
#[test]
fn args_empty() {
    let t = Tester::new();
    run(&t, 202, "args_empty.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-019: a guest buffer smaller than the arguments.
#[test]
fn args_undersized_buffer_fails() {
    let t = Tester::new();
    run(
        &t,
        203,
        "args_toosmall.wasm",
        &make_seq_args(&[0xCA, 0xFE, 0xBA, 0xBE]),
    );
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_SMALL, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_SMALL,
        0,
        host_fn(HostFunction::Args).u32(2).u32(4),
    );
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-019: an out-of-bounds guest pointer.
#[test]
fn args_bad_pointer_fails() {
    let t = Tester::new();
    run(
        &t,
        204,
        "args_badptr.wasm",
        &make_seq_args(&[0xCA, 0xFE, 0xBA, 0xBE]),
    );
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        host_fn(HostFunction::Args).ser(&Status::ErrMemOutOfBounds),
    );
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-005: a mixed little-endian struct filling the whole argument
/// buffer.
#[test]
fn args_mixed_struct_round_trip() {
    let t = Tester::new();
    let bytes = [
        0x44, 0x33, 0x22, 0x11, // U32 0x11223344 (LE)
        0x66, 0x55, // U16 0x5566 (LE)
        0x77, // U8
        0x88, // U8
        0xCC, 0xBB, 0xAA, 0x99, // U32 0x99AABBCC (LE)
    ];
    assert_eq!(bytes.len(), crate::seq::SEQUENCE_ARGUMENTS_MAX_SIZE);
    run(&t, 205, "args_mixed.wasm", &make_seq_args(&bytes));
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-019: a claimed size above the 12-byte buffer is rejected
/// before any copy (no host memory leaks into the guest).
#[test]
fn args_oversized_size_rejected() {
    let t = Tester::new();
    let bytes: Vec<u8> = (0..12).map(|i| 0xA0 + i).collect();
    let mut args = make_seq_args(&bytes);
    args.size = 32;
    run(&t, 206, "args.wasm", &args);
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_LARGE, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_LARGE,
        0,
        host_fn(HostFunction::Args).u32(32).u32(12),
    );
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 0);
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 0);
}

// -- TIME -------------------------------------------------------------------

/// WASM-SEQ-012.
#[test]
fn time_read() {
    let t = Tester::new();
    t.set_time(Time::from_seconds_useconds(1000, 2000));
    run(&t, 210, "time.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_sequence_failure_count(0);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-019.
#[test]
fn time_bad_pointer_fails() {
    let t = Tester::new();
    run(&t, 211, "time_badptr.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_HOST_FUNCTION_INVALID_POINTER, 1);
    t.assert_event(
        W::EVENTID_HOST_FUNCTION_INVALID_POINTER,
        0,
        host_fn(HostFunction::Time).ser(&Status::ErrMemOutOfBounds),
    );
    t.assert_sequence_failure_count(1);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-019.
#[test]
fn time_bad_size_traps() {
    let t = Tester::new();
    run(&t, 212, "time_toosmall.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_SMALL, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_SMALL,
        0,
        host_fn(HostFunction::Time).u32(8).u32(TIME_SIZE),
    );
    t.assert_sequence_failure_count(1);
    t.assert_event(W::EVENTID_SEQUENCE_TRAPPED, 0, trapped(TrapReason::Host));
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-019.
#[test]
fn time_bad_size_too_large_traps() {
    let t = Tester::new();
    run(&t, 213, "time_toobig.wasm", &SeqArgs::default());
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_BUFFER_TOO_LARGE, 1);
    t.assert_event(
        W::EVENTID_BUFFER_TOO_LARGE,
        0,
        host_fn(HostFunction::Time).u32(16).u32(TIME_SIZE),
    );
    t.assert_sequence_failure_count(1);
    t.assert_event(W::EVENTID_SEQUENCE_TRAPPED, 0, trapped(TrapReason::Host));
    assert_eq!(t.port_history_size(), 0);
}
