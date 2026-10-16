//! In-process test of the `wasm`-feature `wasmSeq` instance (upstream
//! Ref's `Svc.WasmSequencer`, see the `topology` module docs): a framed
//! blocking `RUN` of a module whose `main` dispatches `CMD_NO_OP_STRING`
//! through `cmdDisp.seqCmdBuff[2]`, waits for its response on
//! `cmdResponseIn`, and finishes.
#![cfg(feature = "wasm")]

mod common;

use common::{DEADLINE, Harness, build_command_frame, cmd_string_arg, wait_until};

use fprime_ref::topology::{CMD_DISPATCHER_BASE_ID, WASM_SEQUENCER_BASE_ID};
use fprime_svc::cmd_dispatcher::CmdDispatcher;
use fprime_svc::wasm_sequencer::WasmSequencer;

/// The WasmSequencer unit tests' `cmd.wasm` (`main` calls
/// `fprime_v1.cmd(0, 8)`) with its 8-byte data segment replaced by a
/// `CMD_NO_OP_STRING("hi")` command: opcode `0x0100_0001`, then the
/// string argument `[u16 2]"hi"`.
fn no_op_string_module() -> Vec<u8> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../fprime-svc/src/wasm_sequencer/tests/wasm/cmd.wasm"
    );
    let mut module = std::fs::read(path).expect("cmd.wasm");
    let pattern = [0xde, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03, 0x04];
    let at = module
        .windows(pattern.len())
        .position(|w| w == pattern)
        .expect("cmd.wasm data segment");
    let opcode = CMD_DISPATCHER_BASE_ID + CmdDispatcher::OPCODE_CMD_NO_OP_STRING;
    let mut command = opcode.to_be_bytes().to_vec();
    command.extend_from_slice(&cmd_string_arg("hi"));
    module[at..at + pattern.len()].copy_from_slice(&command);
    module
}

#[test]
fn wasm_seq_runs_a_module_that_dispatches_a_command() {
    let harness = Harness::up();
    let file = format!("{}/no_op_string.wasm", harness.data_dir());
    std::fs::write(&file, no_op_string_module()).expect("write module");

    // RUN(fileName, BLOCK, SeqArgs { size 0, 12 zero bytes }).
    let run = WASM_SEQUENCER_BASE_ID + WasmSequencer::OPCODE_RUN;
    let mut args = cmd_string_arg(&file);
    args.push(0); // Svc.BlockState.BLOCK
    args.extend_from_slice(&[0; 8 + 12]);
    harness.inject(&build_command_frame(run, &args));

    let no_op_received = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_NO_OP_STRING_RECEIVED;
    let succeeded = WASM_SEQUENCER_BASE_ID + WasmSequencer::EVENTID_SEQUENCE_SUCCEEDED;
    let completed = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_COMPLETED;
    assert!(
        wait_until(DEADLINE, || {
            harness.saw_event(no_op_received)
                && harness.saw_event(succeeded)
                && harness
                    .log_events()
                    .iter()
                    .any(|(id, a)| *id == completed && a[..] == run.to_be_bytes())
        }),
        "expected NO_OP_STRING, SequenceSucceeded and the RUN completion; got {:?}",
        harness.log_events()
    );
    // The guest's command reached cmdDisp verbatim.
    let no_op_args = harness.event_args(no_op_received).unwrap();
    assert_eq!(no_op_args, cmd_string_arg("hi"));
    harness.topology.teardown();
}
