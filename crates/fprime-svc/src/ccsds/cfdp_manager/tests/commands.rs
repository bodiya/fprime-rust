//! Command tests — ports of `CfdpManagerCommandTests.cpp` (plus its
//! coverage tests): every command's nominal path and validation errors,
//! the transaction commands on a live transfer, playback and polling,
//! and the argument decoding of the generated dispatch.

use super::*;

const OP: fn(FwOpcodeType) -> FwOpcodeType = |o| o;

fn assert_one_response(h: &Harness, opcode: FwOpcodeType, rsp: CmdResponse) {
    assert_eq!(h.cmd_responses(), [(OP(opcode), 0, rsp)]);
}

fn assert_invalid_channel(h: &Harness, channel: u8, max: u8) {
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_INVALID_CHANNEL]);
    assert_eq!(
        h.events(CfdpManager::EVENTID_INVALID_CHANNEL)[0],
        [channel, max]
    );
}

fn write_string(a: &mut CmdArgBuffer, s: &str) {
    assert!(
        CmdStringArg::from(s)
            .serialize_to(a, Endianness::Big)
            .is_ok()
    );
}

#[allow(clippy::too_many_arguments)]
fn playback_cmd(
    h: &Harness,
    channel: u8,
    dest: EntityId,
    class: Class,
    keep: Keep,
    src: &str,
    dst: &str,
) {
    h.send_cmd(CfdpManager::OPCODE_PLAYBACK_DIRECTORY, 0, |a| {
        assert!(a.serialize_u8_be(channel).is_ok());
        assert!(a.serialize_u32_be(dest).is_ok());
        assert!(a.serialize_u8_be(class.as_repr()).is_ok());
        assert!(a.serialize_u8_be(keep.as_repr()).is_ok());
        assert!(a.serialize_u8_be(0).is_ok());
        write_string(a, src);
        write_string(a, dst);
    });
}

#[allow(clippy::too_many_arguments)]
fn poll_cmd(
    h: &Harness,
    channel: u8,
    poll: u8,
    dest: EntityId,
    class: Class,
    interval: u32,
    src: &str,
    dst: &str,
) {
    h.send_cmd(CfdpManager::OPCODE_POLL_DIRECTORY, 0, |a| {
        assert!(a.serialize_u8_be(channel).is_ok());
        assert!(a.serialize_u8_be(poll).is_ok());
        assert!(a.serialize_u32_be(dest).is_ok());
        assert!(a.serialize_u8_be(class.as_repr()).is_ok());
        assert!(a.serialize_u8_be(0).is_ok());
        assert!(a.serialize_u32_be(interval).is_ok());
        write_string(a, src);
        write_string(a, dst);
    });
}

fn stop_poll_cmd(h: &Harness, channel: u8, poll: u8) {
    h.send_cmd(CfdpManager::OPCODE_STOP_POLL_DIRECTORY, 0, |a| {
        assert!(a.serialize_u8_be(channel).is_ok());
        assert!(a.serialize_u8_be(poll).is_ok());
    });
}

fn txn_cmd(
    h: &Harness,
    opcode: FwOpcodeType,
    channel: u8,
    seq: u32,
    eid: u32,
    action: Option<SuspendResume>,
) {
    h.send_cmd(opcode, 0, |a| {
        assert!(a.serialize_u8_be(channel).is_ok());
        assert!(a.serialize_u32_be(seq).is_ok());
        assert!(a.serialize_u32_be(eid).is_ok());
        if let Some(action) = action {
            assert!(a.serialize_u8_be(action.as_repr()).is_ok());
        }
    });
}

/// A class 2 transfer of a 100-byte file, three cycles in (upstream's
/// setup for the suspend/cancel/abandon tests).
fn live_class2_transfer(h: &Harness) -> TransactionSeq {
    let dir = temp_dir();
    let src = path_in(&dir, "live.bin");
    make_file(&src, 100);
    h.send_file_cmd(
        CH0,
        TEST_GROUND_EID,
        Class::Class2,
        Keep::Delete,
        0,
        &src,
        "/dest/live.bin",
    );
    h.run_cycles(3);
    h.ground.clear();
    1
}

#[test]
fn send_file_zero_length_fails_gracefully() {
    let h = Harness::new();
    let dir = temp_dir();
    let src = path_in(&dir, "zero.bin");
    std::fs::write(&src, b"").unwrap();
    h.send_file_cmd(CH0, 2, Class::Class1, Keep::Keep, 0, &src, "dst.bin");
    h.run_cycles(10);
    assert_one_response(&h, CfdpManager::OPCODE_SEND_FILE, CmdResponse::Ok);
    let ev = h.events(CfdpManager::EVENTID_TX_ZERO_LENGTH_FILE);
    assert_eq!(ev.len(), 1);
    assert_eq!(args::str_at(&ev[0], 9).0, src);
    assert_eq!(h.tlm()[0].fault_file_size_mismatch, 1);
}

#[test]
fn send_file_nonexistent_reports_open_failure() {
    let h = Harness::new();
    let dir = temp_dir();
    let src = path_in(&dir, "missing.bin");
    h.send_file_cmd(CH0, 2, Class::Class1, Keep::Keep, 0, &src, "dst.bin");
    assert_one_response(&h, CfdpManager::OPCODE_SEND_FILE, CmdResponse::Ok);
    h.run_cycles(10);
    let ev = h.events(CfdpManager::EVENTID_TX_FILE_OPEN_FAILED);
    assert_eq!(ev.len(), 1);
    let b = &ev[0];
    assert_eq!(b[0], Class::Class1.as_repr());
    assert_eq!((args::u32_at(b, 1), args::u32_at(b, 5)), (42, 1));
    let (name, at) = args::str_at(b, 9);
    assert_eq!(name, src);
    assert_eq!(
        args::u32_at(b, at) as i32,
        fprime_os::file::Status::DoesntExist as i32
    );
    // Failed command-initiated transfer.
    assert_eq!(
        h.events(CfdpManager::EVENTID_TX_FILE_TRANSFER_FAILED).len(),
        1
    );
}

#[test]
fn send_file_invalid_channel() {
    let h = Harness::new();
    h.send_file_cmd(
        NUM_CHANNELS as u8,
        2,
        Class::Class1,
        Keep::Keep,
        0,
        "a",
        "b",
    );
    assert_one_response(
        &h,
        CfdpManager::OPCODE_SEND_FILE,
        CmdResponse::ValidationError,
    );
    assert_invalid_channel(&h, NUM_CHANNELS as u8, NUM_CHANNELS as u8);
}

#[test]
fn send_file_rejects_more_than_the_commanded_limit() {
    let h = Harness::new();
    for _ in 0..MAX_COMMANDED_PLAYBACK_FILES_PER_CHAN {
        h.send_file_cmd(CH0, 2, Class::Class1, Keep::Keep, 0, "/nonexistent", "b");
    }
    h.ground.clear();
    h.send_file_cmd(CH0, 2, Class::Class1, Keep::Keep, 0, "/nonexistent", "b");
    assert_one_response(
        &h,
        CfdpManager::OPCODE_SEND_FILE,
        CmdResponse::ExecutionError,
    );
    assert_eq!(
        h.event_ids(),
        [CfdpManager::EVENTID_MAX_TX_TRANSACTIONS_REACHED]
    );
}

#[test]
fn send_file_queued_event_and_priority_ordering() {
    let h = Harness::new();
    h.send_file_cmd(CH0, 2, Class::Class1, Keep::Keep, 5, "/x/low", "b");
    h.send_file_cmd(CH0, 2, Class::Class1, Keep::Keep, 1, "/x/high", "b");
    h.send_file_cmd(CH0, 2, Class::Class1, Keep::Keep, 5, "/x/low2", "b");
    let queued = h.events(CfdpManager::EVENTID_TX_FILE_QUEUED);
    assert_eq!(queued.len(), 3);
    let (name, at) = args::str_at(&queued[0], 0);
    assert_eq!((name.as_str(), args::u32_at(&queued[0], at)), ("/x/low", 1));
    // C++ parity: `insertSortPrio` inserts after the last transaction (from
    // the tail) of equal or higher priority, and at the BACK when there is
    // none (CF 3.0.0 inserts at the front there), so a more urgent
    // transaction queued behind less urgent ones stays behind them.
    let order = h.chan(CH0, |c| {
        let mut seqs = Vec::new();
        c.traverse_all_transactions(|c, t| seqs.push(c.hist(t).seq_num));
        seqs
    });
    assert_eq!(order, [1, 2, 3]);
}

#[test]
fn stop_poll_dir_not_active() {
    let h = Harness::new();
    stop_poll_cmd(&h, CH0, 0);
    assert_one_response(&h, CfdpManager::OPCODE_STOP_POLL_DIRECTORY, CmdResponse::Ok);
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_POLL_DIR_NOT_ACTIVE]);
    assert_eq!(
        h.events(CfdpManager::EVENTID_POLL_DIR_NOT_ACTIVE)[0],
        [0, 0]
    );
}

#[test]
fn poll_directory_start_busy_stop() {
    let h = Harness::new();
    let dir = temp_dir();
    let src = dir.to_str().unwrap().to_string();
    poll_cmd(&h, CH0, 0, 2, Class::Class1, 60, &src, "/poll_dst");
    assert_one_response(&h, CfdpManager::OPCODE_POLL_DIRECTORY, CmdResponse::Ok);
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_POLL_DIR_INITIATED]);
    let ev = &h.events(CfdpManager::EVENTID_POLL_DIR_INITIATED)[0];
    let (name, at) = args::str_at(ev, 0);
    assert_eq!((name.as_str(), ev[at]), (src.as_str(), 0));
    h.ground.clear();
    poll_cmd(&h, CH0, 0, 2, Class::Class1, 60, &src, "/poll_dst");
    assert_one_response(
        &h,
        CfdpManager::OPCODE_POLL_DIRECTORY,
        CmdResponse::ExecutionError,
    );
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_POLL_DIR_BUSY]);
    h.ground.clear();
    stop_poll_cmd(&h, CH0, 0);
    assert_one_response(&h, CfdpManager::OPCODE_STOP_POLL_DIRECTORY, CmdResponse::Ok);
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_POLL_DIR_STOPPED]);
    assert_eq!(h.events(CfdpManager::EVENTID_POLL_DIR_STOPPED)[0], [0, 0]);
}

#[test]
fn poll_directory_validation_errors() {
    let h = Harness::new();
    poll_cmd(&h, NUM_CHANNELS as u8, 0, 2, Class::Class1, 60, "/a", "/b");
    assert_one_response(
        &h,
        CfdpManager::OPCODE_POLL_DIRECTORY,
        CmdResponse::ValidationError,
    );
    assert_invalid_channel(&h, NUM_CHANNELS as u8, NUM_CHANNELS as u8);
    h.ground.clear();
    poll_cmd(
        &h,
        CH0,
        MAX_POLLING_DIR_PER_CHAN as u8,
        2,
        Class::Class1,
        60,
        "/a",
        "/b",
    );
    assert_one_response(
        &h,
        CfdpManager::OPCODE_POLL_DIRECTORY,
        CmdResponse::ValidationError,
    );
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_INVALID_CHANNEL_POLL]);
    assert_eq!(
        h.events(CfdpManager::EVENTID_INVALID_CHANNEL_POLL)[0],
        [
            MAX_POLLING_DIR_PER_CHAN as u8,
            MAX_POLLING_DIR_PER_CHAN as u8
        ]
    );
    h.ground.clear();
    poll_cmd(&h, CH0, 0, 2, Class::Class1, 0, "/a", "/b");
    assert_one_response(
        &h,
        CfdpManager::OPCODE_POLL_DIRECTORY,
        CmdResponse::ValidationError,
    );
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_INVALID_POLL_INTERVAL]);
    assert_eq!(
        h.events(CfdpManager::EVENTID_INVALID_POLL_INTERVAL)[0],
        [0, 0, 0, 0]
    );
}

#[test]
fn poll_directory_timer_expiry_starts_a_playback_and_sends_the_file() {
    let h = Harness::new();
    let dir = temp_dir();
    let src = dir.to_str().unwrap().to_string();
    let file = format!("{src}/poll_file.bin");
    std::fs::write(&file, [0u8; 64]).unwrap();
    let interval = 3;
    poll_cmd(&h, CH0, 0, 2, Class::Class1, interval, &src, "/poll_dst");
    assert_one_response(&h, CfdpManager::OPCODE_POLL_DIRECTORY, CmdResponse::Ok);
    assert!(!h.chan(CH0, |c| c.polldir[0].pb.busy));
    h.run_cycles(interval);
    assert!(
        !h.chan(CH0, |c| c.polldir[0].pb.busy),
        "not before the interval"
    );
    h.run_cycle();
    assert!(h.chan(CH0, |c| c.polldir[0].pb.busy), "playback started");
    assert_eq!(h.tlm()[0].poll_counter, 1);
    // The playback queues the file, which is sent and (keep = DELETE)
    // removed when done.
    h.run_cycles(3);
    let sent = h.sent();
    let md = sent
        .iter()
        .find(|p| peek_pdu_type(p) == PduTypeEnum::Metadata)
        .map(|p| decode_md(p))
        .expect("metadata sent");
    assert_eq!(md.get_source_filename().as_bytes(), file.as_bytes());
    assert_eq!(
        md.get_dest_filename().as_bytes(),
        b"/poll_dst/poll_file.bin"
    );
    assert!(
        !std::path::Path::new(&file).exists(),
        "sent poll file deleted"
    );
    stop_poll_cmd(&h, CH0, 0);
}

#[test]
fn playback_directory_nominal_sends_every_file() {
    let h = Harness::new();
    let dir = temp_dir();
    let src = dir.to_str().unwrap().to_string();
    for name in ["a.bin", "b.bin"] {
        std::fs::write(format!("{src}/{name}"), [1u8; 100]).unwrap();
    }
    playback_cmd(&h, CH0, 2, Class::Class1, Keep::Keep, &src, "/pb_dst");
    assert_one_response(&h, CfdpManager::OPCODE_PLAYBACK_DIRECTORY, CmdResponse::Ok);
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_PLAYBACK_INITIATED]);
    assert_eq!(
        args::str_at(&h.events(CfdpManager::EVENTID_PLAYBACK_INITIATED)[0], 0).0,
        src
    );
    h.run_cycles(6);
    let mut names: Vec<String> = h
        .sent()
        .iter()
        .filter(|p| peek_pdu_type(p) == PduTypeEnum::Metadata)
        .map(|p| String::from_utf8(decode_md(p).get_dest_filename().as_bytes().to_vec()).unwrap())
        .collect();
    names.sort();
    assert_eq!(names, ["/pb_dst/a.bin", "/pb_dst/b.bin"]);
    assert_eq!(
        h.events(CfdpManager::EVENTID_TX_FILE_TRANSFER_COMPLETED)
            .len(),
        2
    );
    // Both transactions finished and the directory is exhausted.
    h.run_cycle();
    assert!(!h.chan(CH0, |c| c.playback[0].busy));
    assert_eq!(h.tlm()[0].playback_counter, 0);
}

#[test]
fn playback_directory_errors() {
    let h = Harness::new();
    playback_cmd(
        &h,
        NUM_CHANNELS as u8,
        2,
        Class::Class1,
        Keep::Keep,
        "/a",
        "/b",
    );
    assert_one_response(
        &h,
        CfdpManager::OPCODE_PLAYBACK_DIRECTORY,
        CmdResponse::ValidationError,
    );
    assert_invalid_channel(&h, NUM_CHANNELS as u8, NUM_CHANNELS as u8);
    h.ground.clear();
    let dir = temp_dir();
    let missing = path_in(&dir, "nope");
    playback_cmd(&h, CH0, 2, Class::Class1, Keep::Keep, &missing, "/b");
    assert_one_response(
        &h,
        CfdpManager::OPCODE_PLAYBACK_DIRECTORY,
        CmdResponse::ExecutionError,
    );
    assert_eq!(
        h.event_ids(),
        [CfdpManager::EVENTID_PLAYBACK_DIR_OPEN_FAILED]
    );
    let (name, at) = args::str_at(
        &h.events(CfdpManager::EVENTID_PLAYBACK_DIR_OPEN_FAILED)[0],
        0,
    );
    assert_eq!(name, missing);
    assert_eq!(
        args::u32_at(
            &h.events(CfdpManager::EVENTID_PLAYBACK_DIR_OPEN_FAILED)[0],
            at
        ) as i32,
        fprime_os::directory::Status::DoesntExist as i32
    );
    h.run_cycle();
    assert_eq!(h.tlm()[0].fault_directory_read, 1);
    // Both slots busy.
    h.ground.clear();
    let d = dir.to_str().unwrap().to_string();
    std::fs::write(format!("{d}/f"), [0u8; 4]).unwrap();
    playback_cmd(&h, CH0, 2, Class::Class1, Keep::Keep, &d, "/b");
    playback_cmd(&h, CH0, 2, Class::Class1, Keep::Keep, &d, "/b");
    h.ground.clear();
    playback_cmd(&h, CH0, 2, Class::Class1, Keep::Keep, &d, "/b");
    assert_one_response(
        &h,
        CfdpManager::OPCODE_PLAYBACK_DIRECTORY,
        CmdResponse::ExecutionError,
    );
    assert_eq!(
        h.event_ids(),
        [CfdpManager::EVENTID_PLAYBACK_DIR_SLOT_UNAVAILABLE]
    );
}

#[test]
fn set_channel_flow_freezes_the_channel() {
    let h = Harness::new();
    h.send_cmd(CfdpManager::OPCODE_SET_CHANNEL_FLOW, 0, |a| {
        assert!(a.serialize_u8_be(CH0).is_ok());
        assert!(a.serialize_i32_be(Flow::Frozen.as_repr()).is_ok());
    });
    assert_one_response(&h, CfdpManager::OPCODE_SET_CHANNEL_FLOW, CmdResponse::Ok);
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_SET_FLOW_STATE]);
    assert_eq!(
        h.events(CfdpManager::EVENTID_SET_FLOW_STATE)[0],
        [0, 0, 0, 0, 1]
    );
    assert_eq!(h.chan(CH0, |c| c.get_flow_state()), Flow::Frozen);
    // A frozen channel does not run its transactions.
    let dir = temp_dir();
    let src = path_in(&dir, "frozen.bin");
    make_file(&src, 10);
    h.send_file_cmd(CH0, 2, Class::Class1, Keep::Keep, 0, &src, "d");
    h.run_cycles(3);
    assert!(h.sent().is_empty());
    h.ground.clear();
    h.send_cmd(CfdpManager::OPCODE_SET_CHANNEL_FLOW, 0, |a| {
        assert!(a.serialize_u8_be(NUM_CHANNELS as u8).is_ok());
        assert!(a.serialize_i32_be(Flow::Frozen.as_repr()).is_ok());
    });
    assert_one_response(
        &h,
        CfdpManager::OPCODE_SET_CHANNEL_FLOW,
        CmdResponse::ValidationError,
    );
    assert_invalid_channel(&h, NUM_CHANNELS as u8, NUM_CHANNELS as u8);
}

#[test]
fn suspend_resume_transaction() {
    let h = Harness::new();
    let seq = live_class2_transfer(&h);
    txn_cmd(
        &h,
        CfdpManager::OPCODE_SUSPEND_RESUME_TRANSACTION,
        CH0,
        seq,
        42,
        Some(SuspendResume::Suspend),
    );
    assert_one_response(
        &h,
        CfdpManager::OPCODE_SUSPEND_RESUME_TRANSACTION,
        CmdResponse::Ok,
    );
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_TRANSACTION_SUSPENDED]);
    let t = h.find_txn(CH0, seq).unwrap();
    assert!(h.txn(CH0, t, |c, t| c.txn(t).flags.com.suspended));
    h.ground.clear();
    txn_cmd(
        &h,
        CfdpManager::OPCODE_SUSPEND_RESUME_TRANSACTION,
        CH0,
        seq,
        42,
        Some(SuspendResume::Resume),
    );
    assert_one_response(
        &h,
        CfdpManager::OPCODE_SUSPEND_RESUME_TRANSACTION,
        CmdResponse::Ok,
    );
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_TRANSACTION_RESUMED]);
    assert!(!h.txn(CH0, t, |c, t| c.txn(t).flags.com.suspended));
}

#[test]
fn transaction_commands_not_found_and_invalid_channel() {
    let h = Harness::new();
    for (opcode, action) in [
        (
            CfdpManager::OPCODE_SUSPEND_RESUME_TRANSACTION,
            Some(SuspendResume::Suspend),
        ),
        (CfdpManager::OPCODE_CANCEL_TRANSACTION, None),
        (CfdpManager::OPCODE_ABANDON_TRANSACTION, None),
    ] {
        h.ground.clear();
        txn_cmd(&h, opcode, CH0, 999, 100, action);
        assert_one_response(&h, opcode, CmdResponse::ExecutionError);
        assert_eq!(h.event_ids(), [CfdpManager::EVENTID_TRANSACTION_NOT_FOUND]);
        let b = &h.events(CfdpManager::EVENTID_TRANSACTION_NOT_FOUND)[0];
        assert_eq!((args::u32_at(b, 0), args::u32_at(b, 4)), (999, 100));
        h.ground.clear();
        txn_cmd(&h, opcode, NUM_CHANNELS as u8, 1, 100, action);
        assert_one_response(&h, opcode, CmdResponse::ValidationError);
        assert_invalid_channel(&h, NUM_CHANNELS as u8, NUM_CHANNELS as u8);
    }
}

#[test]
fn cancel_transaction_closes_out_with_a_cancel_eof() {
    let h = Harness::new();
    let seq = live_class2_transfer(&h);
    txn_cmd(
        &h,
        CfdpManager::OPCODE_CANCEL_TRANSACTION,
        CH0,
        seq,
        42,
        None,
    );
    assert_one_response(&h, CfdpManager::OPCODE_CANCEL_TRANSACTION, CmdResponse::Ok);
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_TRANSACTION_CANCELED]);
    let t = h.find_txn(CH0, seq).unwrap();
    h.txn(CH0, t, |c, t| {
        assert!(c.txn(t).flags.com.canceled);
        assert_eq!(c.hist(t).txn_stat, TxnStatus::CANCEL_REQUEST_RECEIVED);
    });
    // The ACK timer re-sends the EOF with the cancel condition code.
    h.run_cycles(4);
    let eof = h
        .sent()
        .iter()
        .find(|p| peek_pdu_type(p) == PduTypeEnum::EndOfFile)
        .map(|p| decode_eof(p))
        .expect("cancel EOF");
    assert_eq!(
        eof.get_condition_code(),
        ConditionCode::CANCEL_REQUEST_RECEIVED
    );
    assert_eq!(eof.get_tlv_list().get_tlv(0).get_data().get_entity_id(), 42);
    assert!(h.tlm()[0].sent_eof_canceled >= 1);
    // The EOF-ACK finishes a canceled transaction (no FIN comes).
    h.send_pdu(
        CH0,
        &ack_bytes(
            42,
            TEST_GROUND_EID,
            seq,
            FileDirective::END_OF_FILE,
            0,
            ConditionCode::CANCEL_REQUEST_RECEIVED,
            AckTxnStatus::ACTIVE,
        ),
    );
    assert_eq!(h.txn(CH0, t, |c, t| c.txn(t).state), TxnState::Hold);
    assert_eq!(
        h.events(CfdpManager::EVENTID_TX_FILE_TRANSFER_FAILED).len(),
        1
    );
}

#[test]
fn abandon_transaction_finishes_at_once() {
    let h = Harness::new();
    let seq = live_class2_transfer(&h);
    txn_cmd(
        &h,
        CfdpManager::OPCODE_ABANDON_TRANSACTION,
        CH0,
        seq,
        42,
        None,
    );
    assert_one_response(&h, CfdpManager::OPCODE_ABANDON_TRANSACTION, CmdResponse::Ok);
    assert_eq!(
        h.events(CfdpManager::EVENTID_TRANSACTION_ABANDONED).len(),
        1
    );
    let t = h.find_txn(CH0, seq).unwrap();
    h.txn(CH0, t, |c, t| {
        assert_eq!(c.txn(t).state, TxnState::Hold);
        assert!(!c.txn(t).flags.com.keep_history);
    });
}

#[test]
fn reset_counters_single_all_and_invalid() {
    let h = Harness::new();
    h.chan(CH0, |c| c.tlm.recv_pdu = 5);
    h.chan(CH1, |c| c.tlm.recv_pdu = 6);
    h.send_cmd(CfdpManager::OPCODE_RESET_COUNTERS, 0, |a| {
        assert!(a.serialize_u8_be(CH0).is_ok());
    });
    assert_one_response(&h, CfdpManager::OPCODE_RESET_COUNTERS, CmdResponse::Ok);
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_RESET_COUNTERS]);
    assert_eq!(h.events(CfdpManager::EVENTID_RESET_COUNTERS)[0], [0]);
    let tlm = h.tlm();
    assert_eq!((tlm[0].recv_pdu, tlm[1].recv_pdu), (0, 6));
    h.ground.clear();
    h.send_cmd(CfdpManager::OPCODE_RESET_COUNTERS, 0, |a| {
        assert!(a.serialize_u8_be(0xFF).is_ok());
    });
    assert_one_response(&h, CfdpManager::OPCODE_RESET_COUNTERS, CmdResponse::Ok);
    assert_eq!(h.events(CfdpManager::EVENTID_RESET_COUNTERS)[0], [0xFF]);
    assert_eq!(h.tlm()[1].recv_pdu, 0);
    h.ground.clear();
    h.send_cmd(CfdpManager::OPCODE_RESET_COUNTERS, 0, |a| {
        assert!(a.serialize_u8_be(NUM_CHANNELS as u8).is_ok());
    });
    assert_one_response(
        &h,
        CfdpManager::OPCODE_RESET_COUNTERS,
        CmdResponse::ValidationError,
    );
    // C++ parity: this one reports NumChannels - 1 as the maximum.
    assert_invalid_channel(&h, NUM_CHANNELS as u8, NUM_CHANNELS as u8 - 1);
}

#[test]
fn command_argument_errors() {
    let h = Harness::new();
    // Short arguments.
    h.send_cmd(CfdpManager::OPCODE_RESET_COUNTERS, 3, |_| {});
    // Leftover byte.
    h.send_cmd(CfdpManager::OPCODE_RESET_COUNTERS, 4, |a| {
        assert!(a.serialize_u8_be(0).is_ok());
        assert!(a.serialize_u8_be(0).is_ok());
    });
    // Undeclared enum value (class 7).
    h.send_cmd(CfdpManager::OPCODE_SEND_FILE, 5, |a| {
        assert!(a.serialize_u8_be(0).is_ok());
        assert!(a.serialize_u32_be(2).is_ok());
        assert!(a.serialize_u8_be(7).is_ok());
        assert!(a.serialize_u8_be(0).is_ok());
        assert!(a.serialize_u8_be(0).is_ok());
        write_string(a, "a");
        write_string(a, "b");
    });
    // Unknown opcode.
    h.send_cmd(0x99, 6, |_| {});
    assert_eq!(
        h.cmd_responses(),
        [
            (
                CfdpManager::OPCODE_RESET_COUNTERS,
                3,
                CmdResponse::FormatError
            ),
            (
                CfdpManager::OPCODE_RESET_COUNTERS,
                4,
                CmdResponse::FormatError
            ),
            (
                CfdpManager::OPCODE_SEND_FILE,
                5,
                CmdResponse::ValidationError
            ),
            (0x99, 6, CmdResponse::InvalidOpcode),
        ]
    );
    assert!(h.event_ids().is_empty());
}

#[test]
fn data_in_ignores_short_buffers_and_other_packet_types() {
    let h = Harness::new();
    h.send_raw(CH0, &[0x00]);
    h.send_raw(CH0, &[0x00, 0x00, 0, 0, 0, 0, 0, 0]);
    assert_eq!(*h.ground.data_in_returned.lock(), [(0, 1), (0, 8)]);
    assert!(h.event_ids().is_empty());
    h.run_cycle();
    assert_eq!(h.tlm()[0].recv_pdu, 0);
}

#[test]
fn get_pdu_buffer_respects_the_per_cycle_limit() {
    let h = Harness::new();
    let max = h.comp.get_max_outgoing_pdus_per_cycle_param(0);
    let mut counter = max;
    let (status, buffer) = h.comp.get_pdu_buffer(0, &mut counter, 64);
    assert_eq!(status, Status::SendPduNoBufAvailError);
    assert_eq!(buffer.size(), 0);
    assert_eq!(h.ground.allocations.load(Ordering::SeqCst), 0);
    let mut counter = 0;
    let (status, buffer) = h.comp.get_pdu_buffer(0, &mut counter, 64);
    assert_eq!((status, buffer.size(), counter), (Status::Success, 64, 1));
}

#[test]
fn txn_status_and_search_helpers() {
    let h = Harness::new();
    let t = h.chan(CH0, |c| {
        let t = c.find_unused_transaction(Direction::Tx).unwrap();
        c.txn_mut(t).state = TxnState::S2;
        c.txn_mut(t).priority = 3;
        let hist = c.hist_mut(t);
        hist.src_eid = 5;
        hist.seq_num = 42;
        c.insert_sort_prio(t, QueueId::Pend);
        t
    });
    h.chan(CH0, |c| {
        assert_eq!(c.find_transaction_by_sequence_number(42, 5), Some(t));
        assert_eq!(c.find_transaction_by_sequence_number(7, 5), None);
        for s in [TxnState::S1, TxnState::R1, TxnState::S2, TxnState::R2] {
            c.txn_mut(t).state = s;
            assert_eq!(
                utils::get_txn_status(Some(c.txn(t).state)),
                AckTxnStatus::ACTIVE
            );
        }
    });
    // A lower-priority (higher number) transaction goes after it; a higher
    // one finds no equal-or-higher entry and goes to the back (C++ parity).
    h.chan(CH0, |c| {
        let low = c.find_unused_transaction(Direction::Tx).unwrap();
        c.txn_mut(low).priority = 5;
        c.insert_sort_prio(low, QueueId::Pend);
        let high = c.find_unused_transaction(Direction::Tx).unwrap();
        c.txn_mut(high).priority = 1;
        c.insert_sort_prio(high, QueueId::Pend);
        let mut order = Vec::new();
        c.traverse_all_transactions(|c, n| order.push(c.txn(n).priority));
        assert_eq!(order, [3, 5, 1]);
    });
}

#[test]
fn file_in_port_validation_and_busy_queue() {
    let h = Harness::new();
    let p = h.comp.file_in(0);
    let resp = p.target.invoke(p.port_num, &"a".into(), &"b".into(), 1, 0);
    assert_eq!(resp.status, SendFileStatus::StatusInvalid);
    let ev = h.events(CfdpManager::EVENTID_UNSUPPORTED_SEND_FILE_ARGUMENTS);
    assert_eq!(ev, [vec![0, 0, 0, 1, 0, 0, 0, 0]]);
    // The handoff queue holds 10 requests.
    for _ in 0..10 {
        let resp = p
            .target
            .invoke(p.port_num, &"/nonexistent".into(), &"b".into(), 0, 0);
        assert_eq!(resp.status, SendFileStatus::StatusOk);
    }
    h.ground.clear();
    let resp = p
        .target
        .invoke(p.port_num, &"/nonexistent".into(), &"b".into(), 0, 0);
    assert_eq!(resp.status, SendFileStatus::StatusBusy);
    assert_eq!(
        h.event_ids(),
        [CfdpManager::EVENTID_SEND_FILE_INITIATE_FAIL]
    );
    // The cycles drain them; one transaction runs per cycle, and each
    // failed open is reported on fileDoneOut.
    h.ground.clear();
    h.run_cycles(12);
    let done = h.file_done();
    assert_eq!(done.len(), 10);
    assert!(done.iter().all(|r| r.status == SendFileStatus::StatusError));
}

#[test]
fn file_in_with_invalid_default_channel_reports_invalid() {
    let h = Harness::with_params(|g| {
        g.prm_db
            .lock()
            .insert(CfdpManager::PARAMID_FILE_IN_DEFAULT_CHANNEL, vec![9]);
    });
    let p = h.comp.file_in(0);
    let resp = p.target.invoke(p.port_num, &"a".into(), &"b".into(), 0, 0);
    assert_eq!(resp.status, SendFileStatus::StatusOk);
    h.run_cycle();
    assert_eq!(h.events(CfdpManager::EVENTID_INVALID_CHANNEL), [vec![9, 2]]);
    assert_eq!(h.file_done()[0].status, SendFileStatus::StatusInvalid);
}
