//! Event tests — ports of `CfdpManagerEventTests.cpp`, driving each event
//! through the protocol where upstream's tester pokes internal state.
//! Events whose trigger needs OS fault injection (write, seek and read
//! failures on an open file, PDU encode failures) are not reproduced.

use super::*;

const LOCAL: EntityId = 42;
const GROUND: EntityId = TEST_GROUND_EID;

/// `[class u8][src eid u32][seq u32]` — the common prefix of the
/// transaction events.
fn txn_prefix(b: &[u8]) -> (u8, u32, u32) {
    (b[0], args::u32_at(b, 1), args::u32_at(b, 5))
}

/// Start an R2 receive of `size` bytes into `dst` (metadata only).
fn start_r2(h: &Harness, seq: TransactionSeq, size: FileSize, dst: &str) -> TxnIdx {
    h.send_pdu(
        CH0,
        &md_bytes(Class::Class2, GROUND, LOCAL, seq, size, "/ground/src", dst),
    );
    h.find_txn(CH0, seq).expect("R2 transaction")
}

/// Queue a class 2 send of a `size`-byte file and run it to close-out.
fn s2_in_closeout(h: &Harness, size: usize) -> (TransactionSeq, TxnIdx) {
    let dir = temp_dir();
    let src = path_in(&dir, "s2.bin");
    make_file(&src, size);
    h.send_file_cmd(
        CH0,
        GROUND,
        Class::Class2,
        Keep::Keep,
        0,
        &src,
        "/dst/s2.bin",
    );
    h.run_cycles(2);
    let seq = h.seq_num();
    let t = h.find_txn(CH0, seq).unwrap();
    assert_eq!(
        h.txn(CH0, t, |c, t| c.txn(t).state_data.send.sub_state),
        TxSubState::CloseoutSync
    );
    h.ground.clear();
    (seq, t)
}

#[test]
fn tx_file_transfer_started_and_metadata_received() {
    let h = Harness::new();
    let (seq, _) = {
        let dir = temp_dir();
        let src = path_in(&dir, "start.bin");
        make_file(&src, 300);
        h.send_file_cmd(
            CH0,
            GROUND,
            Class::Class2,
            Keep::Keep,
            0,
            &src,
            "/dst/start.bin",
        );
        h.run_cycle();
        let ev = h.events(CfdpManager::EVENTID_TX_FILE_TRANSFER_STARTED);
        assert_eq!(ev.len(), 1);
        let b = &ev[0];
        assert_eq!(
            (b[0], args::u32_at(b, 1), args::u32_at(b, 5)),
            (0, 1, LOCAL)
        );
        let (name, at) = args::str_at(b, 9);
        assert_eq!(name, src);
        assert_eq!(args::u32_at(b, at), GROUND);
        let (dst, at) = args::str_at(b, at + 4);
        assert_eq!(dst, "/dst/start.bin");
        assert_eq!(args::u32_at(b, at), 300);
        (1, ())
    };
    let _ = seq;
    let dir = temp_dir();
    let dst = path_in(&dir, "md.bin");
    start_r2(&h, 77, 10, &dst);
    let ev = h.events(CfdpManager::EVENTID_METADATA_RECEIVED);
    let (src, at) = args::str_at(&ev[0], 0);
    let (d, at) = args::str_at(&ev[0], at);
    assert_eq!(
        (src.as_str(), d.as_str(), args::u32_at(&ev[0], at)),
        ("/ground/src", dst.as_str(), 77)
    );
}

#[test]
fn buffers_exhausted_when_allocation_fails() {
    let h = Harness::new();
    h.ground.fail_alloc.store(true, Ordering::SeqCst);
    let mut counter = 0;
    let (status, _) = h.comp.get_pdu_buffer(CH0, &mut counter, 100);
    assert_eq!(status, Status::SendPduNoBufAvailError);
    assert_eq!(counter, 0);
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_BUFFERS_EXHAUSTED]);
}

#[test]
fn class2_send_waits_out_buffer_exhaustion() {
    let h = Harness::new();
    let dir = temp_dir();
    let src = path_in(&dir, "exh.bin");
    make_file(&src, 3 * h.chunk());
    h.send_file_cmd(CH0, GROUND, Class::Class2, Keep::Keep, 0, &src, "/d");
    h.ground.fail_alloc.store(true, Ordering::SeqCst);
    h.run_cycles(2);
    assert!(h.sent().is_empty());
    assert!(!h.events(CfdpManager::EVENTID_BUFFERS_EXHAUSTED).is_empty());
    h.ground.fail_alloc.store(false, Ordering::SeqCst);
    h.run_cycle();
    // Metadata and the whole file go out once buffers are back.
    assert_eq!(h.sent().len(), 4);
}

#[test]
fn per_cycle_pdu_throttle_spreads_file_data_over_cycles() {
    let h = Harness::with_params(|g| {
        let mut cfg = ChannelArrayParams::fpp_default();
        cfg[0].max_outgoing_pdus_per_cycle = 2;
        let mut buf = ParamBuffer::new();
        assert!(cfg.serialize_to(&mut buf, Endianness::Big).is_ok());
        g.prm_db
            .lock()
            .unwrap()
            .insert(CfdpManager::PARAMID_CHANNEL_CONFIG, buf.as_slice().to_vec());
    });
    let dir = temp_dir();
    let src = path_in(&dir, "thr.bin");
    make_file(&src, 5 * h.chunk());
    h.send_file_cmd(CH0, GROUND, Class::Class1, Keep::Keep, 0, &src, "/d");
    let mut per_cycle = Vec::new();
    for _ in 0..4 {
        let before = h.sent().len();
        h.run_cycle();
        per_cycle.push(h.sent().len() - before);
    }
    // md+fd, fd+fd, fd+fd, then the EOF.
    assert_eq!(per_cycle, [2, 2, 2, 1]);
    assert!(h.events(CfdpManager::EVENTID_BUFFERS_EXHAUSTED).is_empty());
}

#[test]
fn fail_pdu_header_deserialization() {
    let h = Harness::new();
    h.send_pdu(CH0, &[0x20, 0x00]);
    assert_eq!(
        h.event_ids(),
        [CfdpManager::EVENTID_FAIL_PDU_HEADER_DESERIALIZATION]
    );
    let b = &h.events(CfdpManager::EVENTID_FAIL_PDU_HEADER_DESERIALIZATION)[0];
    assert_eq!(b[0], 0);
    assert_eq!(
        args::u32_at(b, 1) as i32,
        SerializeStatus::DeserSizeMismatch as i32
    );
    h.run_cycle();
    assert_eq!(h.tlm()[0].recv_pdu, 0);
}

#[test]
fn fail_metadata_pdu_deserialization_frees_the_transaction() {
    let h = Harness::new();
    let mut md = md_bytes(Class::Class2, GROUND, LOCAL, 5, 10, "s", "d");
    // Zero-length source name.
    md[12] = 0;
    md[2] -= 1;
    md.remove(13);
    h.send_pdu(CH0, &md);
    let ev = h.events(CfdpManager::EVENTID_FAIL_METADATA_PDU_DESERIALIZATION);
    assert_eq!(ev.len(), 1);
    assert_eq!(
        args::u32_at(&ev[0], 1) as i32,
        SerializeStatus::DeserSizeMismatch as i32
    );
    // The idle transaction is finished (never reaching R1/R2).
    let t = h.find_txn(CH0, 5).unwrap();
    assert_eq!(h.txn(CH0, t, |c, t| c.txn(t).state), TxnState::Hold);
}

#[test]
fn fail_file_data_and_eof_deserialization_end_an_r1() {
    let h = Harness::new();
    let dir = temp_dir();
    let dst = path_in(&dir, "r1.bin");
    h.send_pdu(
        CH0,
        &md_bytes(Class::Class1, GROUND, LOCAL, 6, 10, "s", &dst),
    );
    let mut fd = fd_bytes(Class::Class1, GROUND, LOCAL, 6, 0, &[1, 2, 3]);
    fd.pop();
    h.send_pdu(CH0, &fd);
    let ev = h.events(CfdpManager::EVENTID_FAIL_FILE_DATA_PDU_DESERIALIZATION);
    assert_eq!(ev.len(), 1);
    assert_eq!(
        args::u32_at(&ev[0], 1) as i32,
        SerializeStatus::DeserSizeMismatch as i32
    );
    let t = h.find_txn(CH0, 6).unwrap();
    assert_eq!(h.txn(CH0, t, |c, t| c.txn(t).state), TxnState::Hold);
    // A broken EOF on another R1.
    h.send_pdu(
        CH0,
        &md_bytes(Class::Class1, GROUND, LOCAL, 8, 10, "s", &dst),
    );
    let eof = eof_bytes(
        Class::Class1,
        GROUND,
        LOCAL,
        8,
        ConditionCode::NO_ERROR,
        0,
        10,
    );
    h.send_pdu(CH0, &eof[..eof.len() - 2]);
    assert_eq!(
        h.events(CfdpManager::EVENTID_FAIL_EOF_PDU_DESERIALIZATION)
            .len(),
        1
    );
    let t = h.find_txn(CH0, 8).unwrap();
    assert_eq!(h.txn(CH0, t, |c, t| c.txn(t).state), TxnState::Hold);
}

#[test]
fn fail_ack_fin_nak_deserialization_on_a_sender() {
    let h = Harness::new();
    let (seq, t) = s2_in_closeout(&h, 100);
    let ack = ack_bytes(
        LOCAL,
        GROUND,
        seq,
        FileDirective::END_OF_FILE,
        0,
        ConditionCode::NO_ERROR,
        AckTxnStatus::ACTIVE,
    );
    h.send_pdu(CH0, &ack[..ack.len() - 1]);
    assert_eq!(
        h.events(CfdpManager::EVENTID_FAIL_ACK_PDU_DESERIALIZATION)
            .len(),
        1
    );
    let fin = fin_bytes(
        LOCAL,
        GROUND,
        seq,
        ConditionCode::NO_ERROR,
        FinDeliveryCode::Complete,
        FinFileStatus::Retained,
    );
    h.send_pdu(CH0, &fin[..fin.len() - 1]);
    assert_eq!(
        h.events(CfdpManager::EVENTID_FAIL_FIN_PDU_DESERIALIZATION)
            .len(),
        1
    );
    let nak = nak_bytes(LOCAL, GROUND, seq, (0, 100), &[]);
    h.send_pdu(CH0, &nak[..nak.len() - 2]);
    assert_eq!(
        h.events(CfdpManager::EVENTID_FAIL_NAK_PDU_DESERIALIZATION)
            .len(),
        1
    );
    // None of them changed the transaction.
    h.txn(CH0, t, |c, t| {
        assert_eq!(c.txn(t).state, TxnState::S2);
        assert!(!c.txn(t).flags.tx.eof_ack_recv);
        assert!(!c.txn(t).flags.tx.fin_recv);
    });
    h.run_cycle();
    assert_eq!(h.tlm()[0].recv_errors, 1, "the bad NAK counts");
}

#[test]
fn rx_file_create_failed() {
    let h = Harness::new();
    let dir = temp_dir();
    let dst = format!("{}/nodir/x.bin", dir.to_str().unwrap());
    h.send_pdu(
        CH0,
        &md_bytes(Class::Class1, GROUND, LOCAL, 9, 10, "s", &dst),
    );
    let ev = h.events(CfdpManager::EVENTID_RX_FILE_CREATE_FAILED);
    assert_eq!(ev.len(), 1);
    assert_eq!(txn_prefix(&ev[0]), (Class::Class1.as_repr(), GROUND, 9));
    let (name, at) = args::str_at(&ev[0], 9);
    assert_eq!(name, dst);
    assert_eq!(
        args::u32_at(&ev[0], at) as i32,
        fprime_os::file::Status::DoesntExist as i32
    );
    // R1 ends; the failure is reported.
    assert_eq!(
        h.events(CfdpManager::EVENTID_RX_FILE_TRANSFER_COMPLETED)
            .len(),
        1
    );
    h.run_cycle();
    assert_eq!(h.tlm()[0].fault_file_open, 1);
}

#[test]
fn rx_crc_mismatch_on_class1() {
    let h = Harness::new();
    let dir = temp_dir();
    let dst = path_in(&dir, "crc.bin");
    let data = [1u8, 2, 3, 4, 5];
    h.send_pdu(
        CH0,
        &md_bytes(Class::Class1, GROUND, LOCAL, 10, 5, "s", &dst),
    );
    h.send_pdu(CH0, &fd_bytes(Class::Class1, GROUND, LOCAL, 10, 0, &data));
    h.send_pdu(
        CH0,
        &eof_bytes(
            Class::Class1,
            GROUND,
            LOCAL,
            10,
            ConditionCode::NO_ERROR,
            0xBAD,
            5,
        ),
    );
    let ev = h.events(CfdpManager::EVENTID_RX_CRC_MISMATCH);
    assert_eq!(ev.len(), 1);
    assert_eq!(txn_prefix(&ev[0]), (Class::Class1.as_repr(), GROUND, 10));
    assert_eq!(
        (args::u32_at(&ev[0], 9), args::u32_at(&ev[0], 13)),
        (0xBAD, checksum(&data))
    );
    // C++ parity: R1 marks no error status for a CRC mismatch; the file is
    // not kept (keep stays KEEP from reset, so it survives).
    h.run_cycle();
    assert_eq!(h.tlm()[0].fault_crc_mismatch, 1);
}

#[test]
fn rx_file_size_mismatch_sends_a_fin_with_file_size_error() {
    let h = Harness::new();
    let dir = temp_dir();
    let dst = path_in(&dir, "size.bin");
    let t = start_r2(&h, 11, 10, &dst);
    h.send_pdu(
        CH0,
        &eof_bytes(
            Class::Class2,
            GROUND,
            LOCAL,
            11,
            ConditionCode::NO_ERROR,
            0,
            20,
        ),
    );
    let ev = h.events(CfdpManager::EVENTID_RX_FILE_SIZE_MISMATCH);
    assert_eq!(ev.len(), 1);
    assert_eq!(
        (args::u32_at(&ev[0], 9), args::u32_at(&ev[0], 13)),
        (10, 20)
    );
    h.txn(CH0, t, |c, t| {
        assert_eq!(c.hist(t).txn_stat, TxnStatus::FILE_SIZE_ERROR);
        assert!(c.txn(t).flags.rx.send_fin);
    });
    h.run_cycle();
    let fin = decode_fin(h.sent().last().unwrap());
    assert_eq!(fin.get_condition_code(), ConditionCode::FILE_SIZE_ERROR);
    assert_eq!(
        fin.get_tlv_list().get_tlv(0).get_data().get_entity_id(),
        LOCAL
    );
}

#[test]
fn rx_file_data_out_of_bounds_with_metadata() {
    let h = Harness::new();
    let dir = temp_dir();
    let dst = path_in(&dir, "oob.bin");
    let t = start_r2(&h, 12, 10, &dst);
    h.send_pdu(CH0, &fd_bytes(Class::Class2, GROUND, LOCAL, 12, 8, &[0; 4]));
    let ev = h.events(CfdpManager::EVENTID_RX_FILE_DATA_OUT_OF_BOUNDS);
    assert_eq!(ev.len(), 1);
    assert_eq!(
        (
            args::u32_at(&ev[0], 9),
            args::u32_at(&ev[0], 13),
            args::u32_at(&ev[0], 17)
        ),
        (8, 4, 10)
    );
    // r2Reset: an error status closes it out directly.
    assert_eq!(h.txn(CH0, t, |c, t| c.txn(t).state), TxnState::Hold);
}

#[test]
fn rx_eof_cancel_and_error_condition_codes() {
    let h = Harness::new();
    let dir = temp_dir();
    let t = start_r2(&h, 13, 10, &path_in(&dir, "c.bin"));
    h.send_pdu(
        CH0,
        &eof_bytes(
            Class::Class2,
            GROUND,
            LOCAL,
            13,
            ConditionCode::CANCEL_REQUEST_RECEIVED,
            0,
            0,
        ),
    );
    let ev = h.events(CfdpManager::EVENTID_RX_EOF_CANCEL_RECEIVED);
    assert_eq!(ev.len(), 1);
    assert_eq!(txn_prefix(&ev[0]), (Class::Class2.as_repr(), GROUND, 13));
    // EOF-ACK pending, transaction closed out with the cancel status.
    h.txn(CH0, t, |c, t| {
        assert_eq!(c.hist(t).txn_stat, TxnStatus::CANCEL_REQUEST_RECEIVED);
        assert!(c.txn(t).flags.rx.send_eof_ack);
        assert_eq!(c.txn(t).state, TxnState::Hold);
    });
    let t2 = start_r2(&h, 14, 10, &path_in(&dir, "e.bin"));
    h.send_pdu(
        CH0,
        &eof_bytes(
            Class::Class2,
            GROUND,
            LOCAL,
            14,
            ConditionCode::FILE_CHECKSUM_FAILURE,
            0,
            0,
        ),
    );
    let ev = h.events(CfdpManager::EVENTID_RX_EOF_WITH_ERROR);
    assert_eq!(ev.len(), 1);
    assert_eq!(ev[0][9], ConditionCode::FILE_CHECKSUM_FAILURE.0);
    assert_eq!(
        h.txn(CH0, t2, |c, t| c.hist(t).txn_stat),
        TxnStatus::FILE_CHECKSUM_FAILURE
    );
    h.run_cycle();
    let tlm = h.tlm()[0];
    assert_eq!((tlm.recv_eof_canceled, tlm.fault_rx_eof_error), (1, 1));
}

#[test]
fn rx_eof_md_size_mismatch_and_rename_failure() {
    let h = Harness::new();
    let dir = temp_dir();
    let tmp = dir.to_str().unwrap().to_string();
    let h = {
        drop(h);
        Harness::with_params(|g| {
            let mut cfg = ChannelArrayParams::fpp_default();
            cfg[0].tmp_dir = PathString::from(tmp.as_str());
            let mut buf = ParamBuffer::new();
            assert!(cfg.serialize_to(&mut buf, Endianness::Big).is_ok());
            g.prm_db
                .lock()
                .unwrap()
                .insert(CfdpManager::PARAMID_CHANNEL_CONFIG, buf.as_slice().to_vec());
        })
    };
    // R2 started by file data; EOF says 4 bytes, metadata says 10.
    h.send_pdu(
        CH0,
        &fd_bytes(Class::Class2, GROUND, LOCAL, 15, 0, &[1, 2, 3, 4]),
    );
    h.send_pdu(
        CH0,
        &eof_bytes(
            Class::Class2,
            GROUND,
            LOCAL,
            15,
            ConditionCode::NO_ERROR,
            checksum(&[1, 2, 3, 4]),
            4,
        ),
    );
    h.send_pdu(
        CH0,
        &md_bytes(Class::Class2, GROUND, LOCAL, 15, 10, "s", "/x"),
    );
    let ev = h.events(CfdpManager::EVENTID_RX_EOF_MD_SIZE_MISMATCH);
    assert_eq!(ev.len(), 1);
    assert_eq!((args::u32_at(&ev[0], 9), args::u32_at(&ev[0], 13)), (10, 4));
    // Rename into a missing directory.
    h.send_pdu(CH0, &fd_bytes(Class::Class2, GROUND, LOCAL, 16, 0, &[1, 2]));
    let bad = format!("{tmp}/missing/f.bin");
    h.send_pdu(
        CH0,
        &md_bytes(Class::Class2, GROUND, LOCAL, 16, 2, "s", &bad),
    );
    let ev = h.events(CfdpManager::EVENTID_RX_FILE_RENAME_FAILED);
    assert_eq!(ev.len(), 1);
    let (temp, at) = args::str_at(&ev[0], 9);
    let (fin, _) = args::str_at(&ev[0], at);
    assert_eq!((temp, fin), (format!("{tmp}/{GROUND}:16.tmp"), bad));
    let t = h.find_txn(CH0, 16).unwrap();
    assert_eq!(
        h.txn(CH0, t, |c, t| c.hist(t).txn_stat),
        TxnStatus::FILESTORE_REJECTION
    );
}

#[test]
fn rx_transaction_limit_reached() {
    let h = Harness::new();
    let dir = temp_dir();
    for seq in 0..NUM_TRANSACTIONS_PER_CHANNEL as u32 {
        h.send_pdu(
            CH0,
            &md_bytes(
                Class::Class1,
                GROUND,
                LOCAL,
                1000 + seq,
                10,
                "s",
                &path_in(&dir, &format!("l{seq}")),
            ),
        );
    }
    h.ground.clear();
    h.send_pdu(
        CH0,
        &md_bytes(Class::Class1, GROUND, LOCAL, 2000, 10, "s", "/x"),
    );
    let ev = h.events(CfdpManager::EVENTID_RX_TRANSACTION_LIMIT_REACHED);
    assert_eq!(ev.len(), 1);
    assert_eq!(
        (args::u32_at(&ev[0], 0), args::u32_at(&ev[0], 4)),
        (GROUND, 2000)
    );
}

#[test]
fn rx_and_tx_invalid_directive_codes() {
    let h = Harness::new();
    let dir = temp_dir();
    let t = start_r2(&h, 17, 10, &path_in(&dir, "d.bin"));
    let mut pdu = eof_bytes(
        Class::Class2,
        GROUND,
        LOCAL,
        17,
        ConditionCode::NO_ERROR,
        0,
        0,
    );
    pdu[7] = 0x0D;
    h.ground.clear();
    h.send_pdu(CH0, &pdu);
    let ev = h.events(CfdpManager::EVENTID_RX_INVALID_DIRECTIVE_CODE);
    assert_eq!(ev.len(), 1);
    assert_eq!(txn_prefix(&ev[0]), (Class::Class2.as_repr(), GROUND, 17));
    assert_eq!((ev[0][9], ev[0][10]), (0x0D, RxSubState::FileData as u8));
    let _ = t;
    h.run_cycle();
    assert_eq!(h.tlm()[0].recv_spurious, 1);

    let (seq, _) = s2_in_closeout(&h, 50);
    let mut pdu = fin_bytes(
        LOCAL,
        GROUND,
        seq,
        ConditionCode::NO_ERROR,
        FinDeliveryCode::Complete,
        FinFileStatus::Retained,
    );
    pdu[7] = 0x0E;
    h.send_pdu(CH0, &pdu);
    let ev = h.events(CfdpManager::EVENTID_TX_INVALID_DIRECTIVE_CODE);
    assert_eq!(ev.len(), 1);
    assert_eq!(
        (ev[0][9], ev[0][10]),
        (0x0E, TxSubState::CloseoutSync as u8)
    );
}

#[test]
fn rx_inactivity_timeout_sends_an_early_fin() {
    let h = Harness::new();
    let dir = temp_dir();
    let t = start_r2(&h, 18, 100, &path_in(&dir, "ina.bin"));
    // Keep the ACK/NAK limits out of the way: the NAKs alone (every ACK
    // timer) would hit the NAK limit first, so stop them by suspending
    // nothing but checking the event once the timer fires.
    h.chan(CH0, |c| c.txn_mut(t).inactivity_timer.set_timer(2));
    h.run_cycles(2);
    let ev = h.events(CfdpManager::EVENTID_RX_INACTIVITY_TIMEOUT);
    assert_eq!(ev.len(), 1);
    assert_eq!(txn_prefix(&ev[0]), (Class::Class2.as_repr(), GROUND, 18));
    h.txn(CH0, t, |c, t| {
        assert!(c.txn(t).flags.com.inactivity_fired);
        assert_eq!(c.hist(t).txn_stat, TxnStatus::INACTIVITY_DETECTED);
    });
    assert_eq!(h.tlm()[0].fault_inactivity_timer, 1);
}

#[test]
fn rx_nak_limit_reached() {
    let h = Harness::new();
    let dir = temp_dir();
    let t = start_r2(&h, 19, 1000, &path_in(&dir, "nak.bin"));
    h.send_pdu(
        CH0,
        &fd_bytes(Class::Class2, GROUND, LOCAL, 19, 0, &[0xAB; 100]),
    );
    h.send_pdu(
        CH0,
        &eof_bytes(
            Class::Class2,
            GROUND,
            LOCAL,
            19,
            ConditionCode::NO_ERROR,
            0,
            1000,
        ),
    );
    // A NAK every ACK-timer period until the NAK limit (10) gives up.
    let mut cycles = 0;
    while h
        .events(CfdpManager::EVENTID_RX_NAK_LIMIT_REACHED)
        .is_empty()
    {
        h.run_cycle();
        cycles += 1;
        assert!(cycles < 100, "NAK limit never reached");
    }
    let naks = h
        .sent()
        .iter()
        .filter(|p| peek_pdu_type(p) == PduTypeEnum::NegativeAck)
        .count();
    assert_eq!(naks, 9, "the tenth would-be NAK hits the limit");
    h.txn(CH0, t, |c, t| {
        assert_eq!(c.hist(t).txn_stat, TxnStatus::NAK_LIMIT_REACHED);
        assert!(c.txn(t).flags.rx.send_fin);
    });
    h.run_cycle();
    let fin = decode_fin(h.sent().last().unwrap());
    assert_eq!(fin.get_condition_code(), ConditionCode::NAK_LIMIT_REACHED);
    assert_eq!(fin.get_delivery_code(), FinDeliveryCode::Incomplete);
    assert_eq!(h.tlm()[0].fault_nak_limit, 1);
}

#[test]
fn rx_ack_limit_reached_without_fin_ack() {
    let h = Harness::new();
    let dir = temp_dir();
    let data = [5u8; 10];
    let t = start_r2(&h, 20, 10, &path_in(&dir, "ack.bin"));
    h.send_pdu(CH0, &fd_bytes(Class::Class2, GROUND, LOCAL, 20, 0, &data));
    h.send_pdu(
        CH0,
        &eof_bytes(
            Class::Class2,
            GROUND,
            LOCAL,
            20,
            ConditionCode::NO_ERROR,
            checksum(&data),
            10,
        ),
    );
    let mut cycles = 0;
    while h
        .events(CfdpManager::EVENTID_RX_ACK_LIMIT_REACHED)
        .is_empty()
    {
        h.run_cycle();
        cycles += 1;
        assert!(cycles < 100, "ACK limit never reached");
    }
    let fins = h
        .sent()
        .iter()
        .filter(|p| peek_pdu_type(p) == PduTypeEnum::Finished)
        .count();
    assert_eq!(fins, 10, "the FIN and its retransmissions");
    assert_eq!(
        h.events(CfdpManager::EVENTID_RX_FILE_TRANSFER_FAILED).len(),
        1
    );
    h.txn(CH0, t, |c, t| {
        assert_eq!(c.hist(t).txn_stat, TxnStatus::ACK_LIMIT_NO_FIN);
        assert_eq!(c.txn(t).state, TxnState::Hold);
    });
}

#[test]
fn unhandled_pdu_in_idle_state_and_invalid_destination() {
    let h = Harness::new();
    h.send_pdu(
        CH0,
        &eof_bytes(
            Class::Class2,
            GROUND,
            LOCAL,
            21,
            ConditionCode::NO_ERROR,
            0,
            0,
        ),
    );
    // C++ parity: the freed idle transaction has no error status, so it is
    // reported as a completed receive.
    assert_eq!(
        h.event_ids(),
        [
            CfdpManager::EVENTID_UNHANDLED_PDU_IN_IDLE_STATE,
            CfdpManager::EVENTID_RX_FILE_TRANSFER_COMPLETED
        ]
    );
    let t = h.find_txn(CH0, 21).unwrap();
    assert_eq!(h.txn(CH0, t, |c, t| c.txn(t).state), TxnState::Hold);
    h.ground.clear();
    h.send_pdu(
        CH0,
        &eof_bytes(Class::Class2, GROUND, 99, 22, ConditionCode::NO_ERROR, 0, 0),
    );
    assert_eq!(
        h.events(CfdpManager::EVENTID_INVALID_DESTINATION_EID),
        [vec![0, 0, 0, 99]]
    );
    h.run_cycle();
    assert_eq!(h.tlm()[0].recv_errors, 1);
    assert_eq!(h.tlm()[0].recv_pdu, 2);
}

#[test]
fn class1_file_data_without_metadata_drops() {
    let h = Harness::new();
    h.send_pdu(CH0, &fd_bytes(Class::Class1, GROUND, LOCAL, 23, 0, &[1]));
    let t = h.find_txn(CH0, 23).unwrap();
    assert_eq!(h.txn(CH0, t, |c, t| c.txn(t).state), TxnState::Drop);
    h.send_pdu(CH0, &fd_bytes(Class::Class1, GROUND, LOCAL, 23, 1, &[2]));
    h.run_cycle();
    assert_eq!(h.tlm()[0].recv_dropped, 1);
}

#[test]
fn tx_early_fin_cancels_the_send() {
    let h = Harness::new();
    let dir = temp_dir();
    let src = path_in(&dir, "early.bin");
    make_file(&src, 100);
    h.send_file_cmd(CH0, GROUND, Class::Class2, Keep::Keep, 0, &src, "/d");
    h.ground.clear();
    // Still pending (METADATA) when the FIN arrives.
    h.send_pdu(
        CH0,
        &fin_bytes(
            LOCAL,
            GROUND,
            1,
            ConditionCode::NO_ERROR,
            FinDeliveryCode::Complete,
            FinFileStatus::Retained,
        ),
    );
    let ev = h.events(CfdpManager::EVENTID_TX_EARLY_FIN_RECEIVED);
    assert_eq!(ev.len(), 1);
    assert_eq!(txn_prefix(&ev[0]), (Class::Class2.as_repr(), LOCAL, 1));
    let t = h.find_txn(CH0, 1).unwrap();
    h.txn(CH0, t, |c, t| {
        assert_eq!(c.hist(t).txn_stat, TxnStatus::EARLY_FIN);
        assert!(c.txn(t).flags.tx.fin_recv);
        assert_eq!(c.txn(t).state, TxnState::Hold);
    });
    assert_eq!(
        h.events(CfdpManager::EVENTID_TX_FILE_TRANSFER_FAILED).len(),
        1
    );
}

#[test]
fn tx_non_file_directive_invalid_nak_and_bad_segments() {
    let h = Harness::new();
    let (seq, t) = s2_in_closeout(&h, 100);
    h.send_pdu(CH0, &fd_bytes(Class::Class2, LOCAL, GROUND, seq, 0, &[1]));
    assert_eq!(
        h.event_ids(),
        [CfdpManager::EVENTID_TX_NON_FILE_DIRECTIVE_PDU_RECEIVED]
    );
    h.ground.clear();
    h.send_pdu(CH0, &nak_bytes(LOCAL, GROUND, seq, (0, 100), &[]));
    assert_eq!(h.event_ids(), [CfdpManager::EVENTID_TX_INVALID_NAK_PDU]);
    h.ground.clear();
    let segs = [
        SegmentRequest {
            offset_start: 50,
            offset_end: 10,
        },
        SegmentRequest {
            offset_start: 10,
            offset_end: 200,
        },
        SegmentRequest {
            offset_start: 10,
            offset_end: 20,
        },
        SegmentRequest {
            offset_start: 0,
            offset_end: 0,
        },
    ];
    h.send_pdu(CH0, &nak_bytes(LOCAL, GROUND, seq, (0, 100), &segs));
    let ev = h.events(CfdpManager::EVENTID_TX_INVALID_SEGMENT_REQUESTS);
    assert_eq!(ev.len(), 1);
    assert_eq!(args::u32_at(&ev[0], 9), 2);
    h.txn(CH0, t, |c, t| {
        assert!(c.txn(t).flags.tx.md_need_send);
        assert_eq!(
            c.chunk_list(t).chunks(),
            &[chunk::Chunk {
                offset: 10,
                size: 10
            }]
        );
    });
    // The NAK tick re-sends the metadata, then the range.
    h.ground.clear();
    h.run_cycle();
    let types: Vec<_> = h.sent().iter().map(|p| peek_pdu_type(p)).collect();
    assert_eq!(types, [PduTypeEnum::Metadata, PduTypeEnum::FileData]);
    let sent = h.sent();
    let fd = decode_fd(&sent[1]);
    assert_eq!((fd.get_offset(), fd.get_data_size()), (10, 10));
    assert_eq!(h.tlm()[0].recv_nak_segment_requests, 4);
}

#[test]
fn tx_ack_limit_reached_without_eof_ack() {
    let h = Harness::new();
    let (seq, t) = s2_in_closeout(&h, 100);
    let mut cycles = 0;
    while h
        .events(CfdpManager::EVENTID_TX_ACK_LIMIT_REACHED)
        .is_empty()
    {
        h.run_cycle();
        cycles += 1;
        assert!(cycles < 100, "ACK limit never reached");
    }
    let eofs = h
        .sent()
        .iter()
        .filter(|p| peek_pdu_type(p) == PduTypeEnum::EndOfFile)
        .count();
    assert_eq!(eofs, 10, "the EOF and its retransmissions");
    let ev = h.events(CfdpManager::EVENTID_TX_ACK_LIMIT_REACHED);
    assert_eq!(txn_prefix(&ev[0]), (Class::Class2.as_repr(), LOCAL, seq));
    h.txn(CH0, t, |c, t| {
        assert_eq!(c.hist(t).txn_stat, TxnStatus::ACK_LIMIT_NO_EOF);
        assert_eq!(c.txn(t).state, TxnState::Hold);
    });
    assert_eq!(h.tlm()[0].fault_ack_limit, 1);
    let failed = h.events(CfdpManager::EVENTID_TX_FILE_TRANSFER_FAILED);
    assert_eq!(failed.len(), 1);
    assert_eq!(
        *failed[0].last().unwrap(),
        TxnStatus::ACK_LIMIT_NO_EOF.0 as u8
    );
}

#[test]
fn tx_inactivity_timeout_with_a_long_ack_timer() {
    let h = Harness::with_params(|g| {
        let mut cfg = ChannelArrayParams::fpp_default();
        cfg[0].ack_timer = 1000;
        cfg[0].inactivity_timer = 5;
        let mut buf = ParamBuffer::new();
        assert!(cfg.serialize_to(&mut buf, Endianness::Big).is_ok());
        g.prm_db
            .lock()
            .unwrap()
            .insert(CfdpManager::PARAMID_CHANNEL_CONFIG, buf.as_slice().to_vec());
    });
    let (seq, _) = s2_in_closeout(&h, 100);
    h.run_cycles(6);
    let ev = h.events(CfdpManager::EVENTID_TX_INACTIVITY_TIMEOUT);
    assert_eq!(ev.len(), 1);
    assert_eq!(txn_prefix(&ev[0]), (Class::Class2.as_repr(), LOCAL, seq));
    // Nothing pending: recycled at once (no completion event: the
    // transaction never finished).
    assert_eq!(h.find_txn(CH0, seq), None);
    assert_eq!(h.tlm()[0].fault_inactivity_timer, 1);
}

#[test]
fn file_remove_failed_and_fail_keep_file_move() {
    let dir = temp_dir();
    // C++ parity: the file is moved to the move-directory path itself (a
    // rename), so only a missing parent makes the move fail.
    let missing_dir = format!("{}/no_such_dir/sub", dir.to_str().unwrap());
    let h = Harness::with_params(|g| {
        let mut cfg = ChannelArrayParams::fpp_default();
        cfg[0].move_dir = PathString::from(missing_dir.as_str());
        let mut buf = ParamBuffer::new();
        assert!(cfg.serialize_to(&mut buf, Endianness::Big).is_ok());
        g.prm_db
            .lock()
            .unwrap()
            .insert(CfdpManager::PARAMID_CHANNEL_CONFIG, buf.as_slice().to_vec());
    });
    let src = path_in(&dir, "del.bin");
    make_file(&src, 10);
    h.send_file_cmd(CH0, GROUND, Class::Class1, Keep::Delete, 0, &src, "/d");
    h.run_cycle();
    // Moving into a missing directory fails, then the file is deleted.
    let ev = h.events(CfdpManager::EVENTID_FAIL_KEEP_FILE_MOVE);
    assert_eq!(ev.len(), 1);
    let (name, at) = args::str_at(&ev[0], 0);
    let (to, _) = args::str_at(&ev[0], at);
    assert_eq!((name, to), (src.clone(), missing_dir.clone()));
    assert!(!std::path::Path::new(&src).exists());
    assert!(h.events(CfdpManager::EVENTID_FILE_REMOVE_FAILED).is_empty());

    // The file vanishing mid-transfer: the delete fails.
    let src2 = path_in(&dir, "gone.bin");
    make_file(&src2, 3 * h.chunk());
    let h2 = Harness::with_params(|g| {
        g.prm_db
            .lock()
            .unwrap()
            .insert(CfdpManager::PARAMID_CHANNEL_CONFIG, {
                let mut cfg = ChannelArrayParams::fpp_default();
                cfg[0].max_outgoing_pdus_per_cycle = 2;
                let mut buf = ParamBuffer::new();
                assert!(cfg.serialize_to(&mut buf, Endianness::Big).is_ok());
                buf.as_slice().to_vec()
            });
    });
    h2.send_file_cmd(CH0, GROUND, Class::Class1, Keep::Delete, 0, &src2, "/d");
    h2.run_cycle();
    std::fs::remove_file(&src2).unwrap();
    h2.run_cycles(2);
    let ev = h2.events(CfdpManager::EVENTID_FILE_REMOVE_FAILED);
    assert_eq!(ev.len(), 1);
    let (name, at) = args::str_at(&ev[0], 0);
    assert_eq!(name, src2);
    assert_eq!(
        args::u32_at(&ev[0], at) as i32,
        fprime_os::filesystem::Status::DoesntExist as i32
    );
}

#[test]
fn fail_poll_file_move_for_a_failed_poll_transfer() {
    let h = Harness::new();
    let dir = temp_dir();
    let src_dir = dir.to_str().unwrap().to_string();
    // A zero-length file fails at metadata; it sits in a polling directory,
    // so it goes to the fail directory ("/fail", absent here) or is deleted.
    let file = format!("{src_dir}/empty.bin");
    std::fs::write(&file, b"").unwrap();
    h.send_cmd(CfdpManager::OPCODE_POLL_DIRECTORY, 0, |a| {
        assert!(a.serialize_u8_be(CH0).is_ok());
        assert!(a.serialize_u8_be(0).is_ok());
        assert!(a.serialize_u32_be(GROUND).is_ok());
        assert!(a.serialize_u8_be(Class::Class1.as_repr()).is_ok());
        assert!(a.serialize_u8_be(0).is_ok());
        assert!(a.serialize_u32_be(1).is_ok());
        assert!(
            CmdStringArg::from(src_dir.as_str())
                .serialize_to(a, Endianness::Big)
                .is_ok()
        );
        assert!(
            CmdStringArg::from("/pd")
                .serialize_to(a, Endianness::Big)
                .is_ok()
        );
    });
    h.run_cycles(4);
    assert_eq!(h.events(CfdpManager::EVENTID_TX_ZERO_LENGTH_FILE).len(), 1);
    let ev = h.events(CfdpManager::EVENTID_FAIL_POLL_FILE_MOVE);
    assert_eq!(ev.len(), 1);
    let (name, at) = args::str_at(&ev[0], 0);
    let (to, _) = args::str_at(&ev[0], at);
    assert_eq!((name, to.as_str()), (file.clone(), "/fail"));
    assert!(!std::path::Path::new(&file).exists());
}

#[test]
fn reset_freed_transaction_and_dangling_file_handle() {
    let h = Harness::new();
    let mgr = h.comp.clone();
    h.chan(CH0, |c| {
        let t = c.qs[QueueId::Free as usize].unwrap();
        c.finish_transaction(&mgr, t, true);
    });
    assert_eq!(
        h.event_ids(),
        [CfdpManager::EVENTID_RESET_FREED_TRANSACTION]
    );
    h.ground.clear();
    let dir = temp_dir();
    let path = path_in(&dir, "dangle.bin");
    make_file(&path, 4);
    h.chan(CH0, |c| {
        let t = c.start_rx_transaction().unwrap();
        c.hist_mut(t).seq_num = 31;
        assert_eq!(
            c.txn_mut(t).fd.open(&path, fprime_os::file::Mode::OpenRead),
            fprime_os::file::Status::OpOk
        );
        c.recycle_transaction(&mgr, t);
        assert!(!c.txn(t).fd.is_open());
        assert_eq!(c.txn(t).flags.com.q_index, QueueId::Free);
    });
    assert_eq!(
        h.events(CfdpManager::EVENTID_DANGLING_FILE_HANDLE_CLOSED),
        [vec![0, 0, 0, 0, 31]]
    );
}

#[test]
fn file_data_segment_metadata_is_a_protocol_error() {
    let h = Harness::new();
    let dir = temp_dir();
    let t = start_r2(&h, 32, 10, &path_in(&dir, "seg.bin"));
    let mut pdu = fd_bytes(Class::Class2, GROUND, LOCAL, 32, 0, &[1, 2]);
    pdu[3] |= 0x08;
    h.send_pdu(CH0, &pdu);
    assert_eq!(
        h.events(CfdpManager::EVENTID_FILE_DATA_SEGMENT_METADATA)
            .len(),
        1
    );
    assert_eq!(
        h.txn(CH0, t, |c, t| c.hist(t).txn_stat),
        TxnStatus::PROTOCOL_ERROR
    );
    h.run_cycle();
    assert_eq!(h.tlm()[0].recv_errors, 1);
}

#[test]
fn chunklist_unavailable_frees_the_new_transaction() {
    let h = Harness::new();
    h.chan(CH0, |c| c.cs[Direction::Rx as usize] = None);
    h.send_pdu(
        CH0,
        &md_bytes(Class::Class2, GROUND, LOCAL, 33, 10, "s", "/d"),
    );
    assert_eq!(
        h.events(CfdpManager::EVENTID_CHUNKLIST_UNAVAILABLE),
        [vec![0, 0, 0, 33]]
    );
    let t = h.find_txn(CH0, 33).unwrap();
    assert_eq!(h.txn(CH0, t, |c, t| c.txn(t).state), TxnState::Hold);
}

#[test]
fn recv_hold_re_acks_a_repeated_fin() {
    let h = Harness::new();
    let (seq, t) = s2_in_closeout(&h, 100);
    let fin = fin_bytes(
        LOCAL,
        GROUND,
        seq,
        ConditionCode::NO_ERROR,
        FinDeliveryCode::Complete,
        FinFileStatus::Retained,
    );
    h.send_pdu(CH0, &fin);
    assert_eq!(h.txn(CH0, t, |c, t| c.txn(t).state), TxnState::Hold);
    h.ground.clear();
    // In HOLD the repeated FIN is spurious but re-acknowledged at once.
    h.send_pdu(CH0, &fin);
    let sent = h.sent();
    assert_eq!(sent.len(), 1);
    let ack = decode_ack(&sent[0]);
    assert_eq!(ack.get_directive_code(), FileDirective::FIN);
    assert_eq!(ack.get_transaction_status(), AckTxnStatus::TERMINATED);
    h.run_cycle();
    assert_eq!(h.tlm()[0].recv_spurious, 1);
}

#[test]
fn event_text_matches_the_fpp_format() {
    // The text path (logTextOut) formats the FPP format strings.
    struct Text(Mutex<Vec<String>>);
    impl fprime_comp::LogTextPort for Text {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            _id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            text: &mut fprime_fw::TextLogString,
        ) {
            self.0
                .lock()
                .unwrap()
                .push(text.as_str().unwrap_or("").to_string());
        }
    }
    let h = Harness::new();
    let text = Arc::new(Text(Mutex::new(Vec::new())));
    h.comp.evt.text_log_out.connect(text.clone(), 0);
    h.comp
        .log_warning_lo_rx_crc_mismatch(Class::Class2, 100, 7, 0xAB, 0xCD);
    h.comp.log_activity_lo_set_flow_state(1, Flow::Frozen);
    assert_eq!(
        *text.0.lock().unwrap(),
        [
            "RX class CLASS_2 transaction 100:7 CRC mismatch: expected 0xab actual 0xcd",
            "Set channel 1 to FROZEN"
        ]
    );
}
