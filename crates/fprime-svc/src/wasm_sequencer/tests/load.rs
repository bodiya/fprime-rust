//! Construction, `LOAD` and module validation.

use super::*;

/// Requirement: none (harness check).
#[test]
fn initial_state_is_idle() {
    let t = Tester::new();
    assert_eq!(t.controller(), ControllerState::IDLE);
    // No pending work on a fresh component.
    assert!(!t.has_pending_timer());
    assert_eq!(t.pending_kind(), HostFunction::None);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-001.
#[test]
fn load_empty_module_ready() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(10, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_responses(1);
    t.assert_response(0, W::OPCODE_LOAD, 10, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-004: with `SEQ_BASE_DIR` set, the base dir is prepended to the
/// requested (bare) name; a base dir ending in `/` gets no extra separator.
/// (Upstream uses `./` and stages in the working directory; the staging
/// directory with its trailing `/` plays that role here.)
#[test]
fn load_resolves_against_seq_base_dir() {
    let t = Tester::new();
    t.set_base_dir(&t.base_dir());
    let file = t.stage("empty.wasm");
    t.send_load(11, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_response(0, W::OPCODE_LOAD, 11, CmdResponse::Ok);
    t.assert_events(W::EVENTID_FILE_OPEN_ERROR, 0);
}

/// WASM-SEQ-004: a base dir that does not hold the file fails to open.
#[test]
fn load_against_missing_base_dir_fails_to_open() {
    let t = Tester::new();
    t.set_base_dir("no_such_dir/");
    let file = t.stage("empty.wasm");
    t.send_load(12, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_FILE_OPEN_ERROR, 1);
    // The path that failed to open is the base dir joined with the file.
    assert_eq!(
        t.events(W::EVENTID_FILE_OPEN_ERROR)[0][..2 + 22],
        Bytes::new().str("no_such_dir/empty.wasm").0[..]
    );
    t.assert_response(0, W::OPCODE_LOAD, 12, CmdResponse::ExecutionError);
}

/// WASM-SEQ-004: a base dir without a trailing `/` gets exactly one.
#[test]
fn load_inserts_separator_for_base_dir_without_trailing_slash() {
    let t = Tester::new();
    t.set_base_dir("no_such_dir");
    let file = t.stage("empty.wasm");
    t.send_load(13, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_FILE_OPEN_ERROR, 1);
    assert_eq!(
        t.events(W::EVENTID_FILE_OPEN_ERROR)[0][..2 + 22],
        Bytes::new().str("no_such_dir/empty.wasm").0[..]
    );
    t.assert_response(0, W::OPCODE_LOAD, 13, CmdResponse::ExecutionError);
}

/// WASM-SEQ-004: with a base dir set, a `..` component is rejected before
/// any open.
#[test]
fn load_rejects_path_traversal_outside_base_dir() {
    let t = Tester::new();
    t.set_base_dir("seqs");
    t.send_load(20, "../../etc/evil.wasm", "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_SEQUENCE_FILE_PATH_NOT_CONTAINED, 1);
    t.assert_event(
        W::EVENTID_SEQUENCE_FILE_PATH_NOT_CONTAINED,
        0,
        Bytes::new().str("seqs").str("../../etc/evil.wasm"),
    );
    t.assert_events(W::EVENTID_FILE_OPEN_ERROR, 0);
    t.assert_response(0, W::OPCODE_LOAD, 20, CmdResponse::ExecutionError);
}

/// WASM-SEQ-004: joining the base dir and a long name overflows the
/// 256-byte path; the run arrives over `seqRunIn`, whose file name is long
/// enough.
#[test]
fn seq_run_path_too_long_emits_event() {
    let t = Tester::new();
    let base = "b".repeat(35);
    t.set_base_dir(&base);
    let name = "a".repeat(230);
    t.seq_run_in(&name, &SeqArgs::default());
    t.dispatch_all();
    t.assert_events(W::EVENTID_SEQUENCE_FILE_PATH_TOO_LONG, 1);
    let ev = &t.events(W::EVENTID_SEQUENCE_FILE_PATH_TOO_LONG)[0];
    let base_arg = Bytes::new().str(&base).0;
    assert_eq!(ev[..base_arg.len()], base_arg[..]);
    // A non-empty run of the requested characters.
    let name_len = u16::from_be_bytes([ev[base_arg.len()], ev[base_arg.len() + 1]]) as usize;
    assert!(name_len > 0);
    assert!(ev[base_arg.len() + 2..].iter().all(|&c| c == b'a'));
    assert_eq!(ev.len(), base_arg.len() + 2 + name_len);
    assert_eq!(t.controller(), ControllerState::IDLE);
}

/// WASM-SEQ-002.
#[test]
fn load_named_module_ready() {
    let t = Tester::new();
    let file = t.stage("empty.wasm");
    t.send_load(11, &file, "mod");
    t.dispatch_until_controller(ControllerState::READY);
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_response(0, W::OPCODE_LOAD, 11, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-001: a LOAD with a start function answers when the start
/// finishes, in READY.
#[test]
fn load_start_module_responds_ok() {
    let t = Tester::new();
    let file = t.stage("start.wasm");
    t.send_load(40, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_responses(1);
    t.assert_response(0, W::OPCODE_LOAD, 40, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-018, WASM-SEQ-021: a start function that traps answers
/// EXECUTION_ERROR and counts one failure.
#[test]
fn load_start_module_trap_responds_error() {
    let t = Tester::new();
    let file = t.stage("start_trap.wasm");
    t.send_load(41, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_sequence_failure_count(1);
    t.assert_event(
        W::EVENTID_SEQUENCE_TRAPPED,
        0,
        Bytes::new()
            .u8(0)
            .ser(&SequencePhase::Start)
            .ser(&TrapReason::Unreachable),
    );
    t.assert_responses(1);
    t.assert_response(0, W::OPCODE_LOAD, 41, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
    // Counted exactly once, never as a cancel.
    t.flush_telemetry();
    t.assert_tlm(W::CHANID_SEQUENCES_FAILED, 0, Bytes::new().u64(1));
    t.assert_tlm(W::CHANID_SEQUENCES_CANCELLED, 0, Bytes::new().u64(0));
}

/// WASM-SEQ-001: a LOAD-with-start settles in READY, so a second one is
/// accepted.
#[test]
fn load_start_module_twice_does_not_wedge() {
    let t = Tester::new();
    let file = t.stage("start.wasm");
    t.send_load(42, &file, "");
    t.dispatch_until_controller(ControllerState::READY);
    t.assert_response(0, W::OPCODE_LOAD, 42, CmdResponse::Ok);
    t.send_load(43, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::READY);
    t.assert_responses(2);
    t.assert_response(1, W::OPCODE_LOAD, 43, CmdResponse::Ok);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-001: a missing file is a FileOpenError (naming the resolved
/// path, here inside the staging directory).
#[test]
fn load_file_not_found() {
    let t = Tester::new();
    t.send_load(12, "does_not_exist.wasm", "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_FILE_OPEN_ERROR, 1);
    let path = format!("{}does_not_exist.wasm", t.base_dir());
    let ev = &t.events(W::EVENTID_FILE_OPEN_ERROR)[0];
    let expected = Bytes::new().str(&path).0;
    assert_eq!(ev[..expected.len()], expected[..]);
    t.assert_events(W::EVENTID_MODULE_LOAD_FAILED, 0);
    t.assert_response(0, W::OPCODE_LOAD, 12, CmdResponse::ExecutionError);
}

/// WASM-SEQ-001.
#[test]
fn load_malformed_magic() {
    let t = Tester::new();
    let file = t.stage("malformed.wasm");
    t.send_load(13, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_MODULE_LOAD_FAILED, 1);
    t.assert_event(
        W::EVENTID_MODULE_LOAD_FAILED,
        0,
        Bytes::new().ser(&Status::ErrMalformedMagic),
    );
    t.assert_response(0, W::OPCODE_LOAD, 13, CmdResponse::ExecutionError);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-001.
#[test]
fn load_truncated() {
    let t = Tester::new();
    let file = t.stage("truncated.wasm");
    t.send_load(14, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_MODULE_LOAD_FAILED, 1);
    t.assert_event(
        W::EVENTID_MODULE_LOAD_FAILED,
        0,
        Bytes::new().ser(&Status::ErrEof),
    );
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-003: more guest memory than the 2048-byte pool.
#[test]
fn load_big_mem_fails_allocator() {
    let t = Tester::new();
    let file = t.stage("bigmem.wasm");
    t.send_load(15, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_MODULE_LOAD_FAILED, 1);
    t.assert_event(
        W::EVENTID_MODULE_LOAD_FAILED,
        0,
        Bytes::new().ser(&Status::ErrGuestMemoryAllocFailed),
    );
    // A failed load leaves the guest allocator empty.
    assert_eq!(t.guest_offset(), 0);
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-001.
#[test]
fn load_bad_import_name_fails() {
    let t = Tester::new();
    let file = t.stage("bad_import_name.wasm");
    t.send_load(16, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_MODULE_LOAD_FAILED, 1);
    t.assert_event(
        W::EVENTID_MODULE_LOAD_FAILED,
        0,
        Bytes::new().ser(&Status::ErrFunctionImportNotFound),
    );
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-001.
#[test]
fn load_bad_import_module_fails() {
    let t = Tester::new();
    let file = t.stage("bad_import_module.wasm");
    t.send_load(17, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_MODULE_LOAD_FAILED, 1);
    t.assert_event(
        W::EVENTID_MODULE_LOAD_FAILED,
        0,
        Bytes::new().ser(&Status::ErrFunctionImportNotFound),
    );
    assert_eq!(t.port_history_size(), 0);
}

/// WASM-SEQ-001.
#[test]
fn load_bad_import_sig_fails() {
    let t = Tester::new();
    let file = t.stage("bad_import_sig.wasm");
    t.send_load(18, &file, "");
    t.dispatch_all();
    assert_eq!(t.controller(), ControllerState::IDLE);
    t.assert_events(W::EVENTID_MODULE_LOAD_FAILED, 1);
    t.assert_event(
        W::EVENTID_MODULE_LOAD_FAILED,
        0,
        Bytes::new().ser(&Status::ErrFunctionImportTypeMismatch),
    );
    assert_eq!(t.port_history_size(), 0);
}
