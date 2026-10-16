//! `GLOBAL_SET_*` / `GLOBAL_GET`: direct access to the loaded store
//! (`globals.wasm` exports g_i32=100, g_i64=1000, g_f32=1.5, g_f64=2.5 and
//! the const c_i32=7).

use super::*;

/// LOAD `name` as `module` and settle in READY.
fn load(t: &Tester, cmd_seq: u32, name: &str, module: &str) {
    let file = t.stage(name);
    t.send_load(cmd_seq, &file, module);
    t.dispatch_until_controller(ControllerState::READY);
}

fn value(module: &str, global: &str) -> Bytes {
    Bytes::new().str(module).str(global)
}

fn failed(module: &str, global: &str, status: Status) -> Bytes {
    value(module, global).ser(&status)
}

/// WASM-SEQ-023.
#[test]
fn global_get_reads_initial_i32() {
    let t = Tester::new();
    load(&t, 60, "globals.wasm", "");
    t.send_global_get(61, "", "g_i32");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I32, 1);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I32, 0, value("", "g_i32").i32(100));
    t.assert_events(W::EVENTID_GLOBAL_GET_FAILED, 0);
    t.assert_response(1, W::OPCODE_GLOBAL_GET, 61, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-023.
#[test]
fn global_get_reads_initial_i64() {
    let t = Tester::new();
    load(&t, 62, "globals.wasm", "");
    t.send_global_get(63, "", "g_i64");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I64, 1);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I64, 0, value("", "g_i64").i64(1000));
    t.assert_response(1, W::OPCODE_GLOBAL_GET, 63, CmdResponse::Ok);
}

/// WASM-SEQ-023.
#[test]
fn global_get_reads_initial_f32() {
    let t = Tester::new();
    load(&t, 64, "globals.wasm", "");
    t.send_global_get(65, "", "g_f32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_F32, 1);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_F32, 0, value("", "g_f32").f32(1.5));
    t.assert_response(1, W::OPCODE_GLOBAL_GET, 65, CmdResponse::Ok);
}

/// WASM-SEQ-023.
#[test]
fn global_get_reads_initial_f64() {
    let t = Tester::new();
    load(&t, 66, "globals.wasm", "");
    t.send_global_get(67, "", "g_f64");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_F64, 1);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_F64, 0, value("", "g_f64").f64(2.5));
    t.assert_response(1, W::OPCODE_GLOBAL_GET, 67, CmdResponse::Ok);
}

/// WASM-SEQ-023: a const global is readable.
#[test]
fn global_get_reads_const_global() {
    let t = Tester::new();
    load(&t, 68, "globals.wasm", "");
    t.send_global_get(69, "", "c_i32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I32, 1);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I32, 0, value("", "c_i32").i32(7));
    t.assert_response(1, W::OPCODE_GLOBAL_GET, 69, CmdResponse::Ok);
}

/// WASM-SEQ-023.
#[test]
fn global_set_get_round_trip_i32() {
    let t = Tester::new();
    load(&t, 70, "globals.wasm", "");
    t.send_global_set_i32(71, "", "g_i32", -12345);
    t.dispatch_all();
    t.assert_response(1, W::OPCODE_GLOBAL_SET_I32, 71, CmdResponse::Ok);
    t.assert_events(W::EVENTID_GLOBAL_SET_FAILED, 0);
    t.send_global_get(72, "", "g_i32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I32, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_VALUE_I32,
        0,
        value("", "g_i32").i32(-12345),
    );
    t.assert_response(2, W::OPCODE_GLOBAL_GET, 72, CmdResponse::Ok);
    assert_eq!(t.controller(), ControllerState::READY);
}

/// WASM-SEQ-023: a value that needs all 64 bits.
#[test]
fn global_set_get_round_trip_i64() {
    let t = Tester::new();
    load(&t, 73, "globals.wasm", "");
    let big = 0x0123_4567_89AB_CDEF_i64;
    t.send_global_set_i64(74, "", "g_i64", big);
    t.dispatch_all();
    t.assert_response(1, W::OPCODE_GLOBAL_SET_I64, 74, CmdResponse::Ok);
    t.send_global_get(75, "", "g_i64");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I64, 1);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I64, 0, value("", "g_i64").i64(big));
    t.assert_response(2, W::OPCODE_GLOBAL_GET, 75, CmdResponse::Ok);
}

/// WASM-SEQ-023.
#[test]
fn global_set_get_round_trip_f32() {
    let t = Tester::new();
    load(&t, 76, "globals.wasm", "");
    t.send_global_set_f32(77, "", "g_f32", 3.25);
    t.dispatch_all();
    t.assert_response(1, W::OPCODE_GLOBAL_SET_F32, 77, CmdResponse::Ok);
    t.send_global_get(78, "", "g_f32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_F32, 1);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_F32, 0, value("", "g_f32").f32(3.25));
    t.assert_response(2, W::OPCODE_GLOBAL_GET, 78, CmdResponse::Ok);
}

/// WASM-SEQ-023.
#[test]
fn global_set_get_round_trip_f64() {
    let t = Tester::new();
    load(&t, 79, "globals.wasm", "");
    t.send_global_set_f64(80, "", "g_f64", 6.5);
    t.dispatch_all();
    t.assert_response(1, W::OPCODE_GLOBAL_SET_F64, 80, CmdResponse::Ok);
    t.send_global_get(81, "", "g_f64");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_F64, 1);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_F64, 0, value("", "g_f64").f64(6.5));
    t.assert_response(2, W::OPCODE_GLOBAL_GET, 81, CmdResponse::Ok);
}

/// WASM-SEQ-023: a named module is addressed by its name only.
#[test]
fn global_set_get_named_module() {
    let t = Tester::new();
    load(&t, 82, "globals.wasm", "mod");
    t.send_global_set_i32(83, "mod", "g_i32", 555);
    t.dispatch_all();
    t.assert_response(1, W::OPCODE_GLOBAL_SET_I32, 83, CmdResponse::Ok);
    t.send_global_get(84, "mod", "g_i32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I32, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_VALUE_I32,
        0,
        value("mod", "g_i32").i32(555),
    );
    t.assert_response(2, W::OPCODE_GLOBAL_GET, 84, CmdResponse::Ok);
    t.send_global_get(85, "", "g_i32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_GET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_GET_FAILED,
        0,
        failed("", "g_i32", Status::ErrNotFound),
    );
    t.assert_response(3, W::OPCODE_GLOBAL_GET, 85, CmdResponse::ExecutionError);
}

/// WASM-SEQ-023: the type is checked before mutability.
#[test]
fn global_set_type_mismatch_i64_on_i32() {
    let t = Tester::new();
    load(&t, 86, "globals.wasm", "");
    t.send_global_set_i64(87, "", "g_i32", 1);
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_SET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_SET_FAILED,
        0,
        failed("", "g_i32", Status::ErrGlobalTypeMismatch),
    );
    t.assert_response(1, W::OPCODE_GLOBAL_SET_I64, 87, CmdResponse::ExecutionError);
    t.send_global_get(88, "", "g_i32");
    t.dispatch_all();
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I32, 0, value("", "g_i32").i32(100));
}

/// WASM-SEQ-023.
#[test]
fn global_set_type_mismatch_f32_on_i32() {
    let t = Tester::new();
    load(&t, 89, "globals.wasm", "");
    t.send_global_set_f32(90, "", "g_i32", 1.0);
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_SET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_SET_FAILED,
        0,
        failed("", "g_i32", Status::ErrGlobalTypeMismatch),
    );
    t.assert_response(1, W::OPCODE_GLOBAL_SET_F32, 90, CmdResponse::ExecutionError);
}

/// WASM-SEQ-023.
#[test]
fn global_set_const_is_not_mutable() {
    let t = Tester::new();
    load(&t, 91, "globals.wasm", "");
    t.send_global_set_i32(92, "", "c_i32", 999);
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_SET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_SET_FAILED,
        0,
        failed("", "c_i32", Status::ErrGlobalNotMutable),
    );
    t.assert_response(1, W::OPCODE_GLOBAL_SET_I32, 92, CmdResponse::ExecutionError);
    t.send_global_get(93, "", "c_i32");
    t.dispatch_all();
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I32, 0, value("", "c_i32").i32(7));
}

/// WASM-SEQ-023.
#[test]
fn global_set_unknown_global_not_found() {
    let t = Tester::new();
    load(&t, 94, "globals.wasm", "");
    t.send_global_set_i32(95, "", "nope", 1);
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_SET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_SET_FAILED,
        0,
        failed("", "nope", Status::ErrNotFound),
    );
    t.assert_response(1, W::OPCODE_GLOBAL_SET_I32, 95, CmdResponse::ExecutionError);
    assert_eq!(t.controller(), ControllerState::READY);
}

/// WASM-SEQ-023.
#[test]
fn global_get_unknown_global_not_found() {
    let t = Tester::new();
    load(&t, 96, "globals.wasm", "");
    t.send_global_get(97, "", "nope");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_GET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_GET_FAILED,
        0,
        failed("", "nope", Status::ErrNotFound),
    );
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I32, 0);
    t.assert_response(1, W::OPCODE_GLOBAL_GET, 97, CmdResponse::ExecutionError);
}

/// WASM-SEQ-023.
#[test]
fn global_set_unknown_module_not_found() {
    let t = Tester::new();
    load(&t, 98, "globals.wasm", "");
    t.send_global_set_i32(99, "other", "g_i32", 1);
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_SET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_SET_FAILED,
        0,
        failed("other", "g_i32", Status::ErrNotFound),
    );
    t.assert_response(1, W::OPCODE_GLOBAL_SET_I32, 99, CmdResponse::ExecutionError);
}

/// WASM-SEQ-023.
#[test]
fn global_get_unknown_module_not_found() {
    let t = Tester::new();
    load(&t, 100, "globals.wasm", "");
    t.send_global_get(101, "other", "g_i32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_GET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_GET_FAILED,
        0,
        failed("other", "g_i32", Status::ErrNotFound),
    );
    t.assert_response(1, W::OPCODE_GLOBAL_GET, 101, CmdResponse::ExecutionError);
}

/// WASM-SEQ-023: from IDLE the (empty) store has no module.
#[test]
fn global_set_from_idle_module_not_found() {
    let t = Tester::new();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.send_global_set_i32(102, "", "g_i32", 1);
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_SET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_SET_FAILED,
        0,
        failed("", "g_i32", Status::ErrNotFound),
    );
    t.assert_response(
        0,
        W::OPCODE_GLOBAL_SET_I32,
        102,
        CmdResponse::ExecutionError,
    );
    assert_eq!(t.controller(), ControllerState::IDLE);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-023.
#[test]
fn global_get_from_idle_module_not_found() {
    let t = Tester::new();
    t.send_global_get(103, "", "g_i32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_GET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_GET_FAILED,
        0,
        failed("", "g_i32", Status::ErrNotFound),
    );
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I32, 0);
    t.assert_response(0, W::OPCODE_GLOBAL_GET, 103, CmdResponse::ExecutionError);
    assert_eq!(t.controller(), ControllerState::IDLE);
}

/// WASM-SEQ-023: a SET before INVOKE is seen by main, and main's write by
/// a later GET.
#[test]
fn global_set_persists_into_invoke() {
    let t = Tester::new();
    load(&t, 104, "global_incr.wasm", "");
    t.send_global_set_i32(105, "", "g_i32", 100);
    t.dispatch_all();
    t.assert_response(1, W::OPCODE_GLOBAL_SET_I32, 105, CmdResponse::Ok);
    t.send_invoke(106, "", BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.assert_response(2, W::OPCODE_INVOKE, 106, CmdResponse::Ok);
    t.send_global_get(107, "", "g_i32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I32, 1);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I32, 0, value("", "g_i32").i32(110));
    t.assert_response(3, W::OPCODE_GLOBAL_GET, 107, CmdResponse::Ok);
}

/// WASM-SEQ-023: RUN resets the store, so the global starts over.
#[test]
fn global_set_overwritten_after_run_resets_store() {
    let t = Tester::new();
    let file = t.stage("global_incr.wasm");
    t.send_run(108, &file, BLOCK, &SeqArgs::default());
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 1);
    t.send_global_get(109, "", "g_i32");
    t.dispatch_all();
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I32, 0, value("", "g_i32").i32(10));
    t.send_run(110, &file, BLOCK, &SeqArgs::default());
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_events(W::EVENTID_SEQUENCE_SUCCEEDED, 2);
    t.send_global_get(111, "", "g_i32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I32, 2);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I32, 1, value("", "g_i32").i32(10));
}

/// WASM-SEQ-023: the four typed globals are independent.
#[test]
fn global_set_all_types_independent() {
    let t = Tester::new();
    load(&t, 112, "globals.wasm", "");
    t.send_global_set_i32(113, "", "g_i32", 11);
    t.send_global_set_i64(114, "", "g_i64", 22);
    t.send_global_set_f32(115, "", "g_f32", 0.5);
    t.send_global_set_f64(116, "", "g_f64", 0.25);
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_SET_FAILED, 0);
    t.send_global_get(117, "", "g_i32");
    t.send_global_get(118, "", "g_i64");
    t.send_global_get(119, "", "g_f32");
    t.send_global_get(120, "", "g_f64");
    t.dispatch_all();
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I32, 0, value("", "g_i32").i32(11));
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I64, 0, value("", "g_i64").i64(22));
    t.assert_event(W::EVENTID_GLOBAL_VALUE_F32, 0, value("", "g_f32").f32(0.5));
    t.assert_event(W::EVENTID_GLOBAL_VALUE_F64, 0, value("", "g_f64").f64(0.25));
    for id in [
        W::EVENTID_GLOBAL_VALUE_I32,
        W::EVENTID_GLOBAL_VALUE_I64,
        W::EVENTID_GLOBAL_VALUE_F32,
        W::EVENTID_GLOBAL_VALUE_F64,
    ] {
        t.assert_events(id, 1);
    }
}

/// WASM-SEQ-023.
#[test]
fn global_set_f64_type_mismatch_emits_failure() {
    let t = Tester::new();
    load(&t, 121, "globals.wasm", "");
    t.send_global_set_f64(122, "", "g_i32", 1.0);
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_SET_FAILED, 1);
    t.assert_event(
        W::EVENTID_GLOBAL_SET_FAILED,
        0,
        failed("", "g_i32", Status::ErrGlobalTypeMismatch),
    );
    t.assert_response(
        1,
        W::OPCODE_GLOBAL_SET_F64,
        122,
        CmdResponse::ExecutionError,
    );
}

/// WASM-SEQ-009, WASM-SEQ-023: the global commands work mid-sequence.
#[test]
fn global_get_set_while_sequence_paused() {
    let t = Tester::new();
    t.param_set_fuel(10);
    let file = t.stage("global_loop.wasm");
    t.send_run(123, &file, NO_BLOCK, &SeqArgs::default());
    t.dispatch_until_interpreter(InterpreterState::RUNNING_SPINNING);
    t.send_pause(124);
    t.dispatch_until_interpreter(InterpreterState::RUNNING_PAUSED);
    assert_eq!(t.controller(), ControllerState::RUNNING_MAIN);
    t.send_global_get(125, "", "g_i32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I32, 1);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I32, 0, value("", "g_i32").i32(42));
    t.assert_response(2, W::OPCODE_GLOBAL_GET, 125, CmdResponse::Ok);
    assert_eq!(t.interpreter(), InterpreterState::RUNNING_PAUSED);
    assert_eq!(t.controller(), ControllerState::RUNNING_MAIN);
    t.send_global_set_i32(126, "", "g_i32", 7);
    t.dispatch_all();
    t.assert_response(3, W::OPCODE_GLOBAL_SET_I32, 126, CmdResponse::Ok);
    t.assert_events(W::EVENTID_GLOBAL_SET_FAILED, 0);
    assert_eq!(t.interpreter(), InterpreterState::RUNNING_PAUSED);
    t.send_global_get(127, "", "g_i32");
    t.dispatch_all();
    t.assert_events(W::EVENTID_GLOBAL_VALUE_I32, 2);
    t.assert_event(W::EVENTID_GLOBAL_VALUE_I32, 1, value("", "g_i32").i32(7));
    t.send_cancel(128);
    t.dispatch_until_controller(ControllerState::IDLE);
}
