//! Component-level ports of the upstream AMPCS unit tests
//! (`Svc/CmdSequencer/test/ut/CmdSequencerMain.cpp`: `AMPCS.*` and every
//! `*AMPCS` variant of the `Immediate`, `InvalidFiles`, `Mixed`, `NoFiles`
//! and `Relative` suites). The tester setup matches
//! `CmdSequencerTester`: `setSequenceFormat(ampcsSequence)` BEFORE
//! `allocateBuffer(100, 1024)`, then `setTimeout(100)`.
//!
//! Not ported: `Immediate.FileErrorsAMPCS`, which needs the `Os::File`
//! open/read interceptors (OS fault injection). Its reachable branches —
//! a missing/unreadable sidecar and short files — are covered here and in
//! `ampcs_sequence::tests`.

use super::*;
use crate::cmd_sequencer::ampcs_sequence::TimeFlag;
use crate::cmd_sequencer::ampcs_sequence::test_files::{
    bad_descriptor_file, command_buffer, command_record, header, immediate_file, mixed_file,
    relative_file, temp_dir, too_large_file, write_bad_crc_file, write_sequence, write_with_crc,
};

/// `CmdSequencerTester.hpp` `ALLOCATOR_ID`.
const ALLOCATOR_ID: FwEnumStoreType = 100;
/// `CmdSequencerTester.hpp` `BUFFER_SIZE`.
const TEST_BUFFER_SIZE: usize = 1024;
/// `CmdSequencerTester.hpp` `TIMEOUT` (seconds).
const TIMEOUT: u32 = 100;

/// The upstream tester with `Format::AMPCS`.
fn build_ampcs() -> Harness {
    let h = build();
    // `formats/README.md`: set the format, THEN allocate (the earlier
    // allocation went to the default FPrimeSequence and is dropped).
    h.comp.set_sequence_format(Box::new(AMPCSSequence::new()));
    h.comp.allocate_buffer(ALLOCATOR_ID, TEST_BUFFER_SIZE);
    h.comp.set_timeout(TIMEOUT);
    clear(&h);
    h
}

/// `clearHistory()`.
fn clear(h: &Harness) {
    h.ground.clear();
    h.seq_done.calls.lock().unwrap().clear();
}

/// `setTestTime(Fw::Time(TB_WORKSTATION_TIME, seconds, useconds))`.
fn set_time(h: &Harness, seconds: u32, useconds: u32) {
    h.clock
        .set_full(TimeBase::TbWorkstationTime, 0, seconds, useconds);
}

fn split_name(args: &[u8]) -> (&[u8], &[u8]) {
    let len = u16::from_be_bytes([args[0], args[1]]) as usize;
    (&args[2..2 + len], &args[2 + len..])
}

fn u32_at(tail: &[u8], index: usize) -> u32 {
    let at = 4 * index;
    u32::from_be_bytes([tail[at], tail[at + 1], tail[at + 2], tail[at + 3]])
}

/// `ASSERT_EVENTS_SIZE(ids.len())` plus the id of each, in order.
fn assert_events(h: &Harness, ids: &[FwEventIdType]) {
    assert_eq!(h.ground.event_ids(), ids);
}

/// The file-name argument and the trailing u32 arguments of event `id`.
fn event_args(h: &Harness, id: FwEventIdType) -> (Vec<u8>, Vec<u32>) {
    let event = h.ground.find_event(id).expect("event emitted");
    let (name, tail) = split_name(&event.2);
    let words = (0..tail.len() / 4).map(|i| u32_at(tail, i)).collect();
    (name.to_vec(), words)
}

fn tlm_count(h: &Harness) -> usize {
    h.ground.tlm.lock().unwrap().len()
}

fn current_sequence(h: &Harness) -> Vec<u8> {
    let bytes = h
        .ground
        .last_tlm(CmdSequencer::CHANID_CS_CURRENT_SEQUENCE)
        .expect("CS_CurrentSequence");
    split_name(&bytes).0.to_vec()
}

fn seq_done(h: &Harness) -> Vec<(FwOpcodeType, u32, CmdResponse)> {
    h.seq_done.calls.lock().unwrap().clone()
}

fn timeout_timer_armed(h: &Harness) -> bool {
    h.comp.state.lock().unwrap().cmd_timeout_timer.is_armed()
}

fn cmd_timer_armed(h: &Harness) -> bool {
    h.comp.state.lock().unwrap().cmd_timer.is_armed()
}

fn response(
    opcode: FwOpcodeType,
    cmd_seq: u32,
    status: CmdResponse,
) -> (FwOpcodeType, u32, CmdResponse) {
    (ID_BASE + opcode, cmd_seq, status)
}

/// `validateFile(cmdSeq, fileName)`.
fn validate_file(h: &Harness, cmd_seq: u32, path: &str) {
    clear(h);
    h.command(
        CmdSequencer::OPCODE_CS_VALIDATE,
        cmd_seq,
        &validate_args(path),
    );
    assert_eq!(
        h.responses(),
        vec![response(
            CmdSequencer::OPCODE_CS_VALIDATE,
            cmd_seq,
            CmdResponse::Ok
        )]
    );
    assert_events(
        h,
        &[
            CmdSequencer::EVENTID_CS_SEQUENCE_LOADED,
            CmdSequencer::EVENTID_CS_SEQUENCE_VALID,
        ],
    );
    assert_eq!(
        event_args(h, CmdSequencer::EVENTID_CS_SEQUENCE_VALID).0,
        path.as_bytes()
    );
}

/// `runSequence(cmdSeq, fileName)`.
fn run_sequence(h: &Harness, cmd_seq: u32, path: &str) {
    clear(h);
    h.command(
        CmdSequencer::OPCODE_CS_RUN,
        cmd_seq,
        &run_args(path, BlockState::NoBlock),
    );
    assert_eq!(
        h.responses(),
        vec![response(
            CmdSequencer::OPCODE_CS_RUN,
            cmd_seq,
            CmdResponse::Ok
        )]
    );
    assert_events(h, &[CmdSequencer::EVENTID_CS_SEQUENCE_LOADED]);
    assert_eq!(
        event_args(h, CmdSequencer::EVENTID_CS_SEQUENCE_LOADED).0,
        path.as_bytes()
    );
}

/// `runSequenceByPortCall(fileName)`.
fn run_sequence_by_port_call(h: &Harness, path: &str) {
    clear(h);
    h.seq_run_in(path);
    assert!(h.responses().is_empty());
    assert_events(
        h,
        &[
            CmdSequencer::EVENTID_CS_SEQUENCE_LOADED,
            CmdSequencer::EVENTID_CS_PORT_SEQUENCE_STARTED,
        ],
    );
}

/// `ASSERT_from_comCmdOut_SIZE(1); ASSERT_from_comCmdOut(0, comBuff, 0)`.
fn assert_one_command(h: &Harness, opcode: FwOpcodeType, argument: u32) {
    assert_eq!(
        *h.ground.com.lock().unwrap(),
        vec![(command_buffer(opcode, argument), 0)]
    );
}

/// The last-command branch: `CS_CommandComplete` + `CS_SequenceComplete`,
/// three channels, `CS_CurrentSequence` back to `<no seq>`.
fn assert_sequence_complete(h: &Harness, path: &str, record: u32, opcode: u32, executed: u32) {
    assert_events(
        h,
        &[
            CmdSequencer::EVENTID_CS_COMMAND_COMPLETE,
            CmdSequencer::EVENTID_CS_SEQUENCE_COMPLETE,
        ],
    );
    assert_eq!(
        event_args(h, CmdSequencer::EVENTID_CS_COMMAND_COMPLETE),
        (path.as_bytes().to_vec(), vec![record, opcode])
    );
    assert_eq!(tlm_count(h), 3);
    assert_eq!(
        h.ground.tlm_u32(CmdSequencer::CHANID_CS_COMMANDS_EXECUTED),
        Some(executed)
    );
    assert_eq!(
        h.ground
            .tlm_u32(CmdSequencer::CHANID_CS_SEQUENCES_COMPLETED),
        Some(1)
    );
    assert_eq!(current_sequence(h), NO_SEQ.as_bytes());
}

/// The intermediate branch: one `CS_CommandComplete`, one channel.
fn assert_command_complete(h: &Harness, path: &str, record: u32, opcode: u32, executed: u32) {
    assert_events(h, &[CmdSequencer::EVENTID_CS_COMMAND_COMPLETE]);
    assert_eq!(
        event_args(h, CmdSequencer::EVENTID_CS_COMMAND_COMPLETE),
        (path.as_bytes().to_vec(), vec![record, opcode])
    );
    assert_eq!(tlm_count(h), 1);
    assert_eq!(
        h.ground.tlm_u32(CmdSequencer::CHANID_CS_COMMANDS_EXECUTED),
        Some(executed)
    );
}

/// `ImmediateBase::executeCommandsAuto` with `NO_NEW_SEQUENCE`.
fn execute_commands_auto(h: &Harness, path: &str, num_commands: u32, bound: u32) {
    for i in 0..bound {
        assert_one_command(h, i, i + 1);
        assert!(timeout_timer_armed(h), "record {i}");
        clear(h);
        h.cmd_response_in(i, 0, CmdResponse::Ok);
        if i < num_commands - 1 {
            assert_command_complete(h, path, i, i, i + 1);
        } else {
            assert_sequence_complete(h, path, i, i, i + 1);
        }
    }
}

/// `ImmediateBase::parameterizedAutoByCommand`.
fn auto_by_command(h: &Harness, path: &str, num_commands: u32) {
    set_time(h, 1, 1);
    validate_file(h, 0, path);
    assert!(!timeout_timer_armed(h));
    run_sequence(h, 0, path);
    execute_commands_auto(h, path, num_commands, num_commands);
    assert!(!timeout_timer_armed(h));
    assert_eq!(seq_done(h), vec![(0, 0, CmdResponse::Ok)]);
}

// -- AMPCS.* -----------------------------------------------------------------

#[test]
fn ampcs_missing_crc() {
    let h = build_ampcs();
    let dir = temp_dir();
    // `MissingCRCFile::serializeAMPCS`: header + one byte, no sidecar.
    let mut bytes = header();
    bytes.push(1);
    let path = write_sequence(&dir, "m.bin", &bytes);
    h.command(
        CmdSequencer::OPCODE_CS_RUN,
        0,
        &run_args(&path, BlockState::NoBlock),
    );
    assert!(seq_done(&h).is_empty());
    assert_eq!(
        h.responses(),
        vec![response(
            CmdSequencer::OPCODE_CS_RUN,
            0,
            CmdResponse::ExecutionError
        )]
    );
    assert_events(&h, &[CmdSequencer::EVENTID_CS_FILE_NOT_FOUND]);
    assert_eq!(
        event_args(&h, CmdSequencer::EVENTID_CS_FILE_NOT_FOUND).0,
        format!("{path}.CRC32").as_bytes()
    );
    assert_eq!(tlm_count(&h), 1);
    assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
}

#[test]
fn ampcs_missing_file() {
    let h = build_ampcs();
    let dir = temp_dir();
    // `MissingFile::serializeAMPCS` writes only the sidecar (of nothing);
    // the test then removes the sequence file.
    let path = write_with_crc(&dir, "m.bin", &[]);
    std::fs::remove_file(&path).unwrap();
    h.command(
        CmdSequencer::OPCODE_CS_RUN,
        0,
        &run_args(&path, BlockState::NoBlock),
    );
    assert_eq!(
        h.responses(),
        vec![response(
            CmdSequencer::OPCODE_CS_RUN,
            0,
            CmdResponse::ExecutionError
        )]
    );
    assert_events(&h, &[CmdSequencer::EVENTID_CS_FILE_INVALID]);
    let event = h
        .ground
        .find_event(CmdSequencer::EVENTID_CS_FILE_INVALID)
        .unwrap();
    let (name, tail) = split_name(&event.2);
    assert_eq!(name, path.as_bytes());
    // [stage u8 READ_HEADER_SIZE][error i32 = Os::FileSystem::DOESNT_EXIST]
    assert_eq!(tail[0], FileReadStage::ReadHeaderSize.as_repr());
    assert_eq!(
        i32::from_be_bytes([tail[1], tail[2], tail[3], tail[4]]),
        fprime_os::filesystem::Status::DoesntExist as i32
    );
    assert_eq!(tlm_count(&h), 1);
    assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
}

// -- Immediate.*AMPCS --------------------------------------------------------

#[test]
fn immediate_auto_by_command_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    auto_by_command(&h, &path, 5);
}

#[test]
fn immediate_auto_by_port_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    set_time(&h, 1, 1);
    validate_file(&h, 0, &path);
    run_sequence_by_port_call(&h, &path);
    execute_commands_auto(&h, &path, 5, 5);
    assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::Ok)]);
}

#[test]
fn immediate_auto_by_file_dispatch_port_ampcs() {
    // No upstream AMPCS variant; the seqDispatchIn path is format-agnostic.
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    set_time(&h, 1, 1);
    validate_file(&h, 0, &path);
    clear(&h);
    h.seq_dispatch_in(&path);
    assert_events(
        &h,
        &[
            CmdSequencer::EVENTID_CS_SEQUENCE_LOADED,
            CmdSequencer::EVENTID_CS_PORT_SEQUENCE_STARTED,
        ],
    );
    execute_commands_auto(&h, &path, 5, 5);
    assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::Ok)]);
}

#[test]
fn immediate_cancel_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    set_time(&h, 1, 1);
    validate_file(&h, 0, &path);
    run_sequence(&h, 0, &path);
    execute_commands_auto(&h, &path, 5, 4);
    // `cancelSequence(100, fileName)`.
    clear(&h);
    h.command(CmdSequencer::OPCODE_CS_CANCEL, 100, &[]);
    assert_events(&h, &[CmdSequencer::EVENTID_CS_SEQUENCE_CANCELED]);
    assert_eq!(
        event_args(&h, CmdSequencer::EVENTID_CS_SEQUENCE_CANCELED).0,
        path.as_bytes()
    );
    assert_eq!(h.comp.run_mode(), RunMode::Stopped);
    assert!(!timeout_timer_armed(&h));
}

#[test]
fn immediate_failed_commands_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(3));
    set_time(&h, 1, 1);
    validate_file(&h, 0, &path);
    run_sequence(&h, 0, &path);
    // `executeCommandsError(fileName, 3)`.
    assert_one_command(&h, 0, 1);
    clear(&h);
    h.cmd_response_in(0, 0, CmdResponse::Ok);
    assert_command_complete(&h, &path, 0, 0, 1);
    assert_one_command(&h, 1, 2);
    clear(&h);
    h.cmd_response_in(1, 0, CmdResponse::ExecutionError);
    assert_events(&h, &[CmdSequencer::EVENTID_CS_COMMAND_ERROR]);
    assert_eq!(
        event_args(&h, CmdSequencer::EVENTID_CS_COMMAND_ERROR),
        (
            path.as_bytes().to_vec(),
            vec![1, 1, CmdResponse::ExecutionError as u32]
        )
    );
    assert_eq!(tlm_count(&h), 1);
    assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
    assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::ExecutionError)]);
    assert_eq!(h.comp.run_mode(), RunMode::Stopped);
    assert!(!timeout_timer_armed(&h));
}

#[test]
fn immediate_invalid_manual_commands_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    set_time(&h, 1, 1);
    // CS_START without a sequence.
    h.command(CmdSequencer::OPCODE_CS_START, 14, &[]);
    assert_eq!(
        h.responses(),
        vec![response(
            CmdSequencer::OPCODE_CS_START,
            14,
            CmdResponse::ExecutionError
        )]
    );
    assert_events(&h, &[CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE]);
    validate_file(&h, 0, &path);
    assert!(!timeout_timer_armed(&h));
    run_sequence(&h, 0, &path);
    assert_one_command(&h, 0, 1);
    assert!(timeout_timer_armed(&h));
    for opcode in [
        CmdSequencer::OPCODE_CS_START,
        CmdSequencer::OPCODE_CS_AUTO,
        CmdSequencer::OPCODE_CS_MANUAL,
    ] {
        clear(&h);
        h.command(opcode, 14, &[]);
        assert_eq!(
            h.responses(),
            vec![response(opcode, 14, CmdResponse::ExecutionError)]
        );
        assert_events(&h, &[CmdSequencer::EVENTID_CS_INVALID_MODE]);
    }
}

#[test]
fn immediate_load_on_init_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    set_time(&h, 1, 1);
    // `loadSequence(fileName)`.
    h.comp.load_sequence(&cmd_string(&path));
    assert_events(&h, &[CmdSequencer::EVENTID_CS_SEQUENCE_LOADED]);
    // `runLoadedSequence()`: seqRunIn with an empty name.
    clear(&h);
    h.seq_run_in("");
    assert!(h.responses().is_empty());
    assert_events(&h, &[CmdSequencer::EVENTID_CS_PORT_SEQUENCE_STARTED]);
    assert_eq!(
        event_args(&h, CmdSequencer::EVENTID_CS_PORT_SEQUENCE_STARTED).0,
        path.as_bytes()
    );
    execute_commands_auto(&h, &path, 5, 5);
    assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::Ok)]);
}

#[test]
fn immediate_load_run_run_ampcs() {
    // No upstream AMPCS variant; exercises clear() after completion.
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    set_time(&h, 1, 1);
    h.comp.load_sequence(&cmd_string(&path));
    validate_file(&h, 0, &path);
    run_sequence_by_port_call(&h, &path);
    execute_commands_auto(&h, &path, 5, 5);
    clear(&h);
    h.seq_run_in("");
    assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::ExecutionError)]);
    assert_events(&h, &[CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE]);
    assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
}

#[test]
fn immediate_manual_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    set_time(&h, 1, 1);
    validate_file(&h, 0, &path);
    assert!(!timeout_timer_armed(&h));
    // `goToManualMode(10)`.
    clear(&h);
    h.command(CmdSequencer::OPCODE_CS_MANUAL, 10, &[]);
    assert_eq!(
        h.responses(),
        vec![response(
            CmdSequencer::OPCODE_CS_MANUAL,
            10,
            CmdResponse::Ok
        )]
    );
    assert_events(&h, &[CmdSequencer::EVENTID_CS_MODE_SWITCHED]);
    run_sequence(&h, 0, &path);
    assert!(h.ground.com.lock().unwrap().is_empty());
    // `startSequence(14, fileName)`.
    clear(&h);
    h.command(CmdSequencer::OPCODE_CS_START, 14, &[]);
    assert_eq!(
        h.responses(),
        vec![response(CmdSequencer::OPCODE_CS_START, 14, CmdResponse::Ok)]
    );
    assert_events(&h, &[CmdSequencer::EVENTID_CS_CMD_STARTED]);
    // `Immediate::executeCommandsManual(fileName, 5)`.
    for i in 0..5u32 {
        assert_one_command(&h, i, i + 1);
        assert!(
            !timeout_timer_armed(&h),
            "manual mode never arms the watchdog"
        );
        clear(&h);
        h.cmd_response_in(i, 0, CmdResponse::Ok);
        if i < 4 {
            assert_command_complete(&h, &path, i, i, i + 1);
            clear(&h);
            h.command(CmdSequencer::OPCODE_CS_STEP, 12, &[]);
            assert_eq!(
                h.responses(),
                vec![response(CmdSequencer::OPCODE_CS_STEP, 12, CmdResponse::Ok)]
            );
            assert_events(&h, &[CmdSequencer::EVENTID_CS_CMD_STEPPED]);
            assert_eq!(
                event_args(&h, CmdSequencer::EVENTID_CS_CMD_STEPPED),
                (path.as_bytes().to_vec(), vec![i + 1])
            );
        } else {
            assert_sequence_complete(&h, &path, i, i, i + 1);
        }
    }
    assert!(!timeout_timer_armed(&h));
    assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::Ok)]);
    clear(&h);
    h.command(CmdSequencer::OPCODE_CS_STEP, 12, &[]);
    assert_eq!(
        h.responses(),
        vec![response(
            CmdSequencer::OPCODE_CS_STEP,
            12,
            CmdResponse::ExecutionError
        )]
    );
    assert_events(&h, &[CmdSequencer::EVENTID_CS_INVALID_MODE]);
    clear(&h);
    h.command(CmdSequencer::OPCODE_CS_AUTO, 12, &[]);
    assert_eq!(
        h.responses(),
        vec![response(CmdSequencer::OPCODE_CS_AUTO, 12, CmdResponse::Ok)]
    );
}

#[test]
fn immediate_new_sequence_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    set_time(&h, 1, 1);
    validate_file(&h, 0, &path);
    run_sequence(&h, 0, &path);
    for i in 0..5u32 {
        assert_one_command(&h, i, i + 1);
        assert!(timeout_timer_armed(&h));
        if i == 0 {
            // `startNewSequence(fileName)`.
            clear(&h);
            h.command(
                CmdSequencer::OPCODE_CS_RUN,
                0,
                &run_args(&path, BlockState::NoBlock),
            );
            assert_eq!(
                h.responses(),
                vec![response(
                    CmdSequencer::OPCODE_CS_RUN,
                    0,
                    CmdResponse::ExecutionError
                )]
            );
            assert_events(&h, &[CmdSequencer::EVENTID_CS_INVALID_MODE]);
            clear(&h);
            h.command(CmdSequencer::OPCODE_CS_VALIDATE, 0, &validate_args(&path));
            assert_eq!(
                h.responses(),
                vec![response(
                    CmdSequencer::OPCODE_CS_VALIDATE,
                    0,
                    CmdResponse::ExecutionError
                )]
            );
            assert_events(&h, &[CmdSequencer::EVENTID_CS_INVALID_MODE]);
            clear(&h);
            h.seq_run_in(&path);
            assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::ExecutionError)]);
            assert_events(&h, &[CmdSequencer::EVENTID_CS_INVALID_MODE]);
        }
        clear(&h);
        h.cmd_response_in(i, 0, CmdResponse::Ok);
        if i < 4 {
            assert_command_complete(&h, &path, i, i, i + 1);
        } else {
            assert_sequence_complete(&h, &path, i, i, i + 1);
        }
    }
    assert!(!timeout_timer_armed(&h));
    assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::Ok)]);
}

#[test]
fn immediate_sequence_timeout_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    set_time(&h, 1, 1);
    validate_file(&h, 0, &path);
    assert!(!timeout_timer_armed(&h));
    run_sequence(&h, 0, &path);
    assert_one_command(&h, 0, 1);
    assert!(timeout_timer_armed(&h));
    set_time(&h, 2 * TIMEOUT, 1);
    clear(&h);
    h.sched_in(0);
    assert_events(&h, &[CmdSequencer::EVENTID_CS_SEQUENCE_TIMEOUT]);
    assert_eq!(
        event_args(&h, CmdSequencer::EVENTID_CS_SEQUENCE_TIMEOUT),
        (path.as_bytes().to_vec(), vec![0])
    );
    assert_eq!(h.comp.run_mode(), RunMode::Stopped);
    assert!(!timeout_timer_armed(&h));
    assert!(!cmd_timer_armed(&h));
    assert_eq!(h.comp.state.lock().unwrap().executed_count, 0);
    assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::ExecutionError)]);
}

#[test]
fn immediate_unexpected_command_response_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    auto_by_command(&h, &path, 5);
    clear(&h);
    h.cmd_response_in(0x10, 0, CmdResponse::Ok);
    assert_events(&h, &[CmdSequencer::EVENTID_CS_UNEXPECTED_COMPLETION]);
    let event = h
        .ground
        .find_event(CmdSequencer::EVENTID_CS_UNEXPECTED_COMPLETION)
        .unwrap();
    assert_eq!(event.2, 0x10u32.to_be_bytes());
}

#[test]
fn immediate_validate_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "i.bin", &immediate_file(5));
    set_time(&h, 1, 1);
    validate_file(&h, 0, &path);
}

#[test]
fn immediate_never_loaded_ampcs() {
    // No upstream AMPCS variant; `parameterizedNeverLoaded`.
    let h = build_ampcs();
    h.seq_run_in("");
    assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::ExecutionError)]);
    assert_events(&h, &[CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE]);
    assert_eq!(tlm_count(&h), 1);
    assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
}

// -- InvalidFiles.*AMPCS -----------------------------------------------------

#[test]
fn invalid_files_bad_crc_ampcs() {
    let h = build_ampcs();
    set_time(&h, 0, 0);
    let (path, stored, computed) = write_bad_crc_file(&temp_dir(), "k.bin");
    h.command(
        CmdSequencer::OPCODE_CS_RUN,
        0,
        &run_args(&path, BlockState::NoBlock),
    );
    assert_eq!(
        h.responses(),
        vec![response(
            CmdSequencer::OPCODE_CS_RUN,
            0,
            CmdResponse::ExecutionError
        )]
    );
    assert_events(&h, &[CmdSequencer::EVENTID_CS_FILE_CRC_FAILURE]);
    assert_eq!(
        event_args(&h, CmdSequencer::EVENTID_CS_FILE_CRC_FAILURE),
        (path.as_bytes().to_vec(), vec![stored, computed])
    );
    assert_eq!(tlm_count(&h), 1);
    assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
}

#[test]
fn invalid_files_bad_record_descriptor_ampcs() {
    let h = build_ampcs();
    set_time(&h, 1, 1);
    let path = write_with_crc(&temp_dir(), "b.bin", &bad_descriptor_file(1));
    for opcode in [
        CmdSequencer::OPCODE_CS_VALIDATE,
        CmdSequencer::OPCODE_CS_RUN,
    ] {
        clear(&h);
        let args = if opcode == CmdSequencer::OPCODE_CS_RUN {
            run_args(&path, BlockState::NoBlock)
        } else {
            validate_args(&path)
        };
        h.command(opcode, 0, &args);
        assert_eq!(
            h.responses(),
            vec![response(opcode, 0, CmdResponse::ExecutionError)]
        );
        assert_events(&h, &[CmdSequencer::EVENTID_CS_RECORD_INVALID]);
        assert_eq!(
            event_args(&h, CmdSequencer::EVENTID_CS_RECORD_INVALID),
            (
                path.as_bytes().to_vec(),
                vec![0, SerializeStatus::DeserFormatError as u32]
            )
        );
    }
}

#[test]
fn invalid_files_empty_file_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "e.bin", &[]);
    h.command(
        CmdSequencer::OPCODE_CS_RUN,
        0,
        &run_args(&path, BlockState::NoBlock),
    );
    assert_eq!(
        h.responses(),
        vec![response(
            CmdSequencer::OPCODE_CS_RUN,
            0,
            CmdResponse::ExecutionError
        )]
    );
    assert_events(&h, &[CmdSequencer::EVENTID_CS_FILE_INVALID]);
    let event = h
        .ground
        .find_event(CmdSequencer::EVENTID_CS_FILE_INVALID)
        .unwrap();
    let (name, tail) = split_name(&event.2);
    assert_eq!(name, path.as_bytes());
    assert_eq!(tail, [FileReadStage::ReadHeaderSize.as_repr(), 0, 0, 0, 0]);
    assert_eq!(tlm_count(&h), 1);
    assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
}

#[test]
fn invalid_files_file_too_large_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "x.bin", &too_large_file(TEST_BUFFER_SIZE));
    set_time(&h, 0, 0);
    h.command(
        CmdSequencer::OPCODE_CS_RUN,
        0,
        &run_args(&path, BlockState::NoBlock),
    );
    assert_eq!(
        h.responses(),
        vec![response(
            CmdSequencer::OPCODE_CS_RUN,
            0,
            CmdResponse::ExecutionError
        )]
    );
    assert_events(&h, &[CmdSequencer::EVENTID_CS_FILE_SIZE_ERROR]);
    assert_eq!(
        event_args(&h, CmdSequencer::EVENTID_CS_FILE_SIZE_ERROR),
        (path.as_bytes().to_vec(), vec![2 * TEST_BUFFER_SIZE as u32])
    );
    assert_eq!(tlm_count(&h), 1);
    assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
}

#[test]
fn invalid_files_no_records_ampcs_loads_but_running_asserts() {
    // No upstream AMPCS variant: `NoRecordsFile::serializeAMPCS` exists but
    // the NoRecords suite runs F Prime only. The AMPCS format has no
    // CS_NoRecords check, so validation SUCCEEDS...
    let h = build_ampcs();
    set_time(&h, 1, 1);
    let path = write_with_crc(&temp_dir(), "n.bin", &header());
    validate_file(&h, 0, &path);
    // ...and CS_RUN would step into `nextRecord`'s FW_ASSERT (verified at
    // the format level in `next_record_on_a_zero_record_sequence_asserts`).
}

// -- Mixed.*AMPCS / Relative.*AMPCS (MixedRelativeBase) ----------------------

/// `MixedRelativeBase::parameterizedAutoByCommand` up to
/// `executeCommandsAuto`.
fn mixed_relative_start(h: &Harness, path: &str) {
    set_time(h, 0, 0);
    validate_file(h, 0, path);
    run_sequence(h, 0, path);
    assert!(cmd_timer_armed(h));
    clear(h);
    h.sched_in(0);
    assert!(h.ground.com.lock().unwrap().is_empty());
    assert_events(h, &[]);
    assert!(cmd_timer_armed(h));
}

#[test]
fn mixed_auto_by_command_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "x.bin", &mixed_file());
    mixed_relative_start(&h, &path);
    // Command 1: absolute t=2, due at t=3.
    set_time(&h, 3, 0);
    clear(&h);
    h.sched_in(0);
    assert!(!cmd_timer_armed(&h));
    assert_one_command(&h, 0, 1);
    clear(&h);
    h.cmd_response_in(0, 0, CmdResponse::Ok);
    assert_command_complete(&h, &path, 0, 0, 1);
    assert!(!cmd_timer_armed(&h));
    // Command 2: immediate, dispatched by the response above.
    assert_one_command(&h, 2, 3);
    clear(&h);
    h.cmd_response_in(2, 0, CmdResponse::Ok);
    assert_command_complete(&h, &path, 1, 2, 2);
    assert!(cmd_timer_armed(&h));
    // Command 3: relative t+1 from t=3, due at t=5.
    set_time(&h, 5, 0);
    clear(&h);
    h.sched_in(0);
    assert!(!cmd_timer_armed(&h));
    assert_one_command(&h, 4, 5);
    clear(&h);
    h.cmd_response_in(4, 0, CmdResponse::Ok);
    assert_command_complete(&h, &path, 2, 4, 3);
    assert!(!cmd_timer_armed(&h));
    // Command 4: immediate.
    assert_one_command(&h, 6, 7);
    clear(&h);
    h.cmd_response_in(6, 0, CmdResponse::Ok);
    assert!(!cmd_timer_armed(&h));
    assert_sequence_complete(&h, &path, 3, 6, 4);
    assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::Ok)]);
    clear(&h);
    h.sched_in(0);
    assert!(h.ground.com.lock().unwrap().is_empty());
    assert_events(&h, &[]);
    assert_eq!(tlm_count(&h), 0);
}

#[test]
fn mixed_validate_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "x.bin", &mixed_file());
    set_time(&h, 1, 1);
    validate_file(&h, 0, &path);
}

#[test]
fn relative_auto_by_command_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "r.bin", &relative_file(3));
    mixed_relative_start(&h, &path);
    // `Relative::executeCommandsAuto(fileName, 3, 3)`.
    for i in 0..3u32 {
        set_time(&h, 2 * i + 3, 0);
        clear(&h);
        h.sched_in(0);
        assert!(!cmd_timer_armed(&h));
        assert_one_command(&h, i, i + 1);
        clear(&h);
        h.cmd_response_in(i, 0, CmdResponse::Ok);
        if i < 2 {
            assert_command_complete(&h, &path, i, i, i + 1);
            assert!(cmd_timer_armed(&h));
        } else {
            assert_sequence_complete(&h, &path, i, i, i + 1);
            assert!(!cmd_timer_armed(&h));
            assert_eq!(seq_done(&h), vec![(0, 0, CmdResponse::Ok)]);
        }
        assert!(h.ground.com.lock().unwrap().is_empty());
        clear(&h);
        h.sched_in(0);
        assert!(h.ground.com.lock().unwrap().is_empty());
        assert_events(&h, &[]);
        assert_eq!(tlm_count(&h), 0);
    }
}

#[test]
fn relative_validate_ampcs() {
    let h = build_ampcs();
    let path = write_with_crc(&temp_dir(), "r.bin", &relative_file(5));
    set_time(&h, 1, 1);
    validate_file(&h, 0, &path);
}

#[test]
fn relative_time_tag_is_whole_seconds_added_to_now() {
    // AMPCS times carry no useconds: a RELATIVE t+2 at now = 10.5 s is
    // due at 12.5 s, not before.
    let h = build_ampcs();
    let mut bytes = header();
    bytes.extend(command_record(TimeFlag::Relative as u8, 2, 9, 9));
    let path = write_with_crc(&temp_dir(), "w.bin", &bytes);
    h.clock
        .set_full(TimeBase::TbWorkstationTime, 0, 10, 500_000);
    run_sequence(&h, 0, &path);
    assert!(h.ground.com.lock().unwrap().is_empty());
    assert_eq!(
        h.comp.state.lock().unwrap().cmd_timer.expiration_time(),
        Time::new(TimeBase::TbWorkstationTime, 0, 12, 500_000)
    );
    h.clock
        .set_full(TimeBase::TbWorkstationTime, 0, 12, 499_999);
    h.sched_in(0);
    assert!(h.ground.com.lock().unwrap().is_empty());
    h.clock
        .set_full(TimeBase::TbWorkstationTime, 0, 12, 500_000);
    h.sched_in(0);
    assert_one_command(&h, 9, 9);
}

// -- NoFiles.*AMPCS ----------------------------------------------------------

#[test]
fn no_files_init_ampcs() {
    let h = build_ampcs();
    assert_eq!(h.comp.run_mode(), RunMode::Stopped);
    assert_eq!(
        h.comp.state.lock().unwrap().sequence.capacity(),
        TEST_BUFFER_SIZE
    );
}

#[test]
fn no_files_no_sequence_active_ampcs() {
    let h = build_ampcs();
    h.command(CmdSequencer::OPCODE_CS_CANCEL, 0, &[]);
    assert_events(&h, &[CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE]);
}

// -- Format selection --------------------------------------------------------

#[test]
fn an_fprime_file_is_not_accepted_by_the_ampcs_format() {
    // The format is a deployment-wide choice: with AMPCS selected, a
    // seqgen-style file without a sidecar is CS_FileNotFound(<name>.CRC32).
    let h = build_ampcs();
    let path = write_file(&temp_dir(), "f.bin", &simple_sequence(0x42));
    h.command(CmdSequencer::OPCODE_CS_VALIDATE, 1, &validate_args(&path));
    assert_events(&h, &[CmdSequencer::EVENTID_CS_FILE_NOT_FOUND]);
    assert_eq!(
        event_args(&h, CmdSequencer::EVENTID_CS_FILE_NOT_FOUND).0,
        format!("{path}.CRC32").as_bytes()
    );
}
