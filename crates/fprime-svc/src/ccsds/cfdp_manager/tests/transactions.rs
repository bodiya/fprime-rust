//! Transaction tests — ports of `CfdpManagerTester.cpp` (the class 1/2
//! send and receive state machines, NAK handling, port-initiated sends,
//! the late-FIN path, transactions in series), `CfdpManagerMultiChannelTests.cpp`
//! and the ping test.

use super::*;

struct TxSetup {
    seq: TransactionSeq,
    t: TxnIdx,
}

/// Upstream `setupTxTransaction`.
#[allow(clippy::too_many_arguments)]
fn setup_tx(
    h: &Harness,
    src: &str,
    dst: &str,
    ch: u8,
    dest: EntityId,
    class: Class,
    priority: u8,
    expected_state: TxnState,
) -> TxSetup {
    let initial = h.seq_num();
    h.send_file_cmd(ch, dest, class, Keep::Keep, priority, src, dst);
    assert_eq!(
        h.cmd_responses(),
        [(CfdpManager::OPCODE_SEND_FILE, 0, CmdResponse::Ok)]
    );
    let seq = initial + 1;
    assert_eq!(h.seq_num(), seq);
    let t = h.find_txn(ch, seq).expect("transaction exists");
    h.txn(ch, t, |c, t| {
        let txn = c.txn(t);
        assert_eq!(txn.state, expected_state);
        assert_eq!(txn.foffs, 0);
        assert_eq!(txn.state_data.send.sub_state, TxSubState::Metadata);
        assert_eq!(txn.chan_num, ch);
        assert_eq!(txn.priority, priority);
        let hist = c.hist(t);
        assert_eq!(hist.seq_num, seq);
        assert_eq!(hist.src_eid, 42);
        assert_eq!(hist.peer_eid, dest);
        assert_eq!(hist.fnames.src_filename.as_bytes(), src.as_bytes());
        assert_eq!(hist.fnames.dst_filename.as_bytes(), dst.as_bytes());
    });
    TxSetup { seq, t }
}

/// Upstream `setupTxPortTransaction`: the guarded port only queues; the
/// transaction appears when the queue is drained.
fn setup_tx_port(h: &Harness, src: &str, dst: &str, ch: u8) -> TxSetup {
    let initial = h.seq_num();
    let p = h.comp.file_in(0);
    let resp = p.target.invoke(
        p.port_num,
        &SendFileNameArg::from(src),
        &SendFileNameArg::from(dst),
        0,
        0,
    );
    assert_eq!(resp.status, SendFileStatus::StatusOk);
    h.comp
        .with_engine(|engine| h.comp.drain_file_in_queue(engine));
    let seq = initial + 1;
    assert_eq!(h.seq_num(), seq);
    let t = h.find_txn(ch, seq).expect("transaction exists");
    h.txn(ch, t, |c, t| {
        assert_eq!(c.txn(t).state, TxnState::S2);
        assert_eq!(c.txn(t).init_type, TransactionInitType::InitByPort);
        assert_eq!(c.txn(t).state_data.send.sub_state, TxSubState::Metadata);
        assert_eq!(c.hist(t).fnames.src_filename.as_bytes(), src.as_bytes());
    });
    TxSetup { seq, t }
}

/// Upstream `setupRxTransaction`: a metadata PDU starts it.
#[allow(clippy::too_many_arguments)]
fn setup_rx(
    h: &Harness,
    ground_src: &str,
    dst: &str,
    ch: u8,
    class: Class,
    size: FileSize,
    seq: TransactionSeq,
    expected_state: TxnState,
) -> TxnIdx {
    h.send_pdu(
        ch,
        &md_bytes(
            class,
            TEST_GROUND_EID,
            h.local_eid(),
            seq,
            size,
            ground_src,
            dst,
        ),
    );
    let t = h.find_txn(ch, seq).expect("RX transaction created");
    h.txn(ch, t, |c, t| {
        assert_eq!(c.txn(t).state, expected_state);
        assert_eq!(c.txn(t).state_data.receive.sub_state, RxSubState::FileData);
        assert!(c.txn(t).flags.rx.md_recv);
        let hist = c.hist(t);
        assert_eq!(hist.seq_num, seq);
        assert_eq!(hist.src_eid, TEST_GROUND_EID);
        assert_eq!(hist.peer_eid, TEST_GROUND_EID);
        assert_eq!(hist.fnames.src_filename.as_bytes(), ground_src.as_bytes());
        assert_eq!(hist.fnames.dst_filename.as_bytes(), dst.as_bytes());
    });
    t
}

fn verify_fd(
    bytes: &[u8],
    local: EntityId,
    seq: TransactionSeq,
    offset: usize,
    data: &[u8],
    class: Class,
) {
    let fd = decode_fd(bytes);
    assert_eq!(fd.as_header().get_source_eid(), local);
    assert_eq!(fd.as_header().get_dest_eid(), TEST_GROUND_EID);
    assert_eq!(fd.as_header().get_transaction_seq(), seq);
    assert_eq!(fd.as_header().get_txm_mode(), class);
    assert_eq!(fd.as_header().get_direction(), PduDirection::TowardReceiver);
    assert_eq!(fd.get_offset() as usize, offset);
    assert_eq!(fd.get_data(), data);
}

fn verify_md(
    bytes: &[u8],
    local: EntityId,
    seq: TransactionSeq,
    size: usize,
    src: &str,
    dst: &str,
    class: Class,
) {
    let md = decode_md(bytes);
    assert_eq!(md.as_header().get_source_eid(), local);
    assert_eq!(md.as_header().get_dest_eid(), TEST_GROUND_EID);
    assert_eq!(md.as_header().get_transaction_seq(), seq);
    assert_eq!(md.as_header().get_txm_mode(), class);
    assert_eq!(md.get_file_size() as usize, size);
    assert_eq!(md.get_source_filename().as_bytes(), src.as_bytes());
    assert_eq!(md.get_dest_filename().as_bytes(), dst.as_bytes());
    assert_eq!(md.get_closure_requested(), u8::from(class == Class::Class2));
    assert_eq!(md.get_checksum_type(), ChecksumType::MODULAR);
}

/// Upstream `completeClass2Handshake`: EOF-ACK, FIN, then the FIN-ACK
/// cycle.
fn complete_class2_handshake(h: &Harness, ch: u8, seq: TransactionSeq, t: TxnIdx) {
    h.send_pdu(
        ch,
        &ack_bytes(
            h.local_eid(),
            TEST_GROUND_EID,
            seq,
            FileDirective::END_OF_FILE,
            0,
            ConditionCode::NO_ERROR,
            AckTxnStatus::ACTIVE,
        ),
    );
    h.txn(ch, t, |c, t| {
        let txn = c.txn(t);
        assert!(txn.flags.tx.eof_ack_recv);
        assert!(!txn.flags.com.ack_timer_armed);
        assert_eq!(txn.state, TxnState::S2);
        assert_eq!(txn.state_data.send.sub_state, TxSubState::CloseoutSync);
    });
    h.send_pdu(
        ch,
        &fin_bytes(
            h.local_eid(),
            TEST_GROUND_EID,
            seq,
            ConditionCode::NO_ERROR,
            FinDeliveryCode::Complete,
            FinFileStatus::Retained,
        ),
    );
    h.txn(ch, t, |c, t| {
        let txn = c.txn(t);
        assert!(txn.flags.tx.fin_recv);
        assert_eq!(txn.state, TxnState::Hold);
        assert!(txn.flags.tx.send_fin_ack);
    });
    h.run_cycle();
}

fn verify_fin_ack(
    bytes: &[u8],
    src: EntityId,
    dst: EntityId,
    seq: TransactionSeq,
    status: AckTxnStatus,
) {
    let ack = decode_ack(bytes);
    assert_eq!(ack.as_header().get_source_eid(), src);
    assert_eq!(ack.as_header().get_dest_eid(), dst);
    assert_eq!(ack.as_header().get_transaction_seq(), seq);
    assert_eq!(ack.get_directive_code(), FileDirective::FIN);
    assert_eq!(ack.get_directive_subtype_code(), 1);
    assert_eq!(ack.get_condition_code(), ConditionCode::NO_ERROR);
    assert_eq!(ack.get_transaction_status(), status);
}

/// `TxFileTransferCompleted` / `RxFileTransferCompleted` arguments:
/// class, seq, src eid, src file, dest eid, dest file, size.
fn completed_args(b: &[u8]) -> (u8, u32, u32, String, u32, String, u32) {
    let (src, at) = args::str_at(b, 9);
    let dest_eid = args::u32_at(b, at);
    let (dst, at) = args::str_at(b, at + 4);
    (
        b[0],
        args::u32_at(b, 1),
        args::u32_at(b, 5),
        src,
        dest_eid,
        dst,
        args::u32_at(b, at),
    )
}

/// Upstream `sendAndVerifyClass1Tx`.
fn send_and_verify_class1_tx(h: &Harness, src: &str, dst: &str, size: usize) {
    let data = make_file(src, size);
    let s = setup_tx(
        h,
        src,
        dst,
        CH0,
        TEST_GROUND_EID,
        Class::Class1,
        0,
        TxnState::S1,
    );
    let local = h.local_eid();
    let sent_before = h.sent().len();
    h.run_cycle();
    let sent = h.sent();
    assert_eq!(sent.len(), sent_before + 2, "metadata + file data");
    verify_md(
        &sent[sent_before],
        local,
        s.seq,
        size,
        src,
        dst,
        Class::Class1,
    );
    verify_fd(
        &sent[sent_before + 1],
        local,
        s.seq,
        0,
        &data,
        Class::Class1,
    );
    h.txn(CH0, s.t, |c, t| {
        assert_eq!(c.txn(t).fsize as usize, size);
        assert_eq!(c.txn(t).foffs as usize, size);
        // Class 1 finishes as soon as the last data goes out; the EOF is
        // sent by the next tick.
        assert_eq!(c.txn(t).state, TxnState::Hold);
        assert!(c.txn(t).flags.tx.send_eof);
    });
    h.run_cycle();
    let sent = h.sent();
    assert_eq!(sent.len(), sent_before + 3);
    let eof = decode_eof(&sent[sent_before + 2]);
    assert_eq!(eof.as_header().get_source_eid(), local);
    assert_eq!(eof.as_header().get_dest_eid(), TEST_GROUND_EID);
    assert_eq!(eof.get_condition_code(), ConditionCode::NO_ERROR);
    assert_eq!(eof.get_file_size() as usize, size);
    assert_eq!(eof.get_checksum(), checksum(&data));
    let tlm = h.tlm()[usize::from(CH0)];
    assert_eq!(tlm.sent_file_data_bytes as usize, size);
    assert_eq!(tlm.recv_errors, 0);
    assert_eq!(tlm.fault_ack_limit, 0);
    assert_eq!(tlm.fault_nak_limit, 0);
    let completed = h.events(CfdpManager::EVENTID_TX_FILE_TRANSFER_COMPLETED);
    assert_eq!(completed.len(), 1);
    assert_eq!(
        completed_args(&completed[0]),
        (
            Class::Class1.as_repr(),
            s.seq,
            local,
            src.to_string(),
            TEST_GROUND_EID,
            dst.to_string(),
            size as u32
        )
    );
    h.wait_for_recycle(CH0, s.seq);
}

/// Upstream `sendAndVerifyClass2Tx`: command- or port-initiated, with an
/// optional NAK for the second and fifth PDUs.
fn send_and_verify_class2_tx(
    h: &Harness,
    by_port: bool,
    src: &str,
    dst: &str,
    size: usize,
    simulate_nak: bool,
    exact: bool,
) {
    let chunk = h.chunk();
    let ch = if by_port { CH0 } else { CH1 };
    let data = make_file(src, size);
    let s = if by_port {
        setup_tx_port(h, src, dst, ch)
    } else {
        setup_tx(
            h,
            src,
            dst,
            ch,
            TEST_GROUND_EID,
            Class::Class2,
            0,
            TxnState::S2,
        )
    };
    let local = h.local_eid();
    let num_fd = size.div_ceil(chunk);
    h.ground.clear();
    h.run_cycle();
    let sent = h.sent();
    assert_eq!(sent.len(), 1 + num_fd, "metadata + file data in one cycle");
    verify_md(&sent[0], local, s.seq, size, src, dst, Class::Class2);
    for i in 0..num_fd {
        let end = ((i + 1) * chunk).min(size);
        verify_fd(
            &sent[1 + i],
            local,
            s.seq,
            i * chunk,
            &data[i * chunk..end],
            Class::Class2,
        );
    }
    h.txn(ch, s.t, |c, t| {
        assert_eq!(c.txn(t).foffs as usize, size);
        assert_eq!(c.txn(t).state_data.send.sub_state, TxSubState::CloseoutSync);
        assert!(c.txn(t).flags.tx.send_eof);
        assert_eq!(c.txn(t).flags.com.q_index, QueueId::Txw);
    });
    h.run_cycle();
    let sent = h.sent();
    assert_eq!(sent.len(), 2 + num_fd);
    let eof = decode_eof(&sent[1 + num_fd]);
    assert_eq!(eof.get_condition_code(), ConditionCode::NO_ERROR);
    assert_eq!(eof.get_file_size() as usize, size);
    assert_eq!(eof.get_checksum(), checksum(&data));
    h.txn(ch, s.t, |c, t| assert!(!c.txn(t).flags.tx.send_eof));

    if simulate_nak {
        h.ground.clear();
        let segs = [
            SegmentRequest {
                offset_start: chunk as u32,
                offset_end: 2 * chunk as u32,
            },
            SegmentRequest {
                offset_start: 4 * chunk as u32,
                offset_end: 5 * chunk as u32,
            },
        ];
        h.send_pdu(
            ch,
            &nak_bytes(local, TEST_GROUND_EID, s.seq, (0, size as u32), &segs),
        );
        // The NAK re-arms the ACK timer; the NAKed ranges go out on the
        // next tick, and the ACK timer re-sends the EOF.
        let mut found_eof = false;
        for _ in 0..20 {
            h.run_cycle();
            if h.sent()
                .last()
                .is_some_and(|p| peek_pdu_type(p) == PduTypeEnum::EndOfFile)
            {
                found_eof = true;
                break;
            }
        }
        assert!(found_eof, "second EOF after the retransmission");
        let sent = h.sent();
        let retransmitted: Vec<_> = sent
            .iter()
            .filter(|p| peek_pdu_type(p) == PduTypeEnum::FileData)
            .map(|p| decode_fd(p).get_offset())
            .collect();
        assert_eq!(retransmitted, [chunk as u32, 4 * chunk as u32]);
    }

    complete_class2_handshake(h, ch, s.seq, s.t);
    let fin_ack = h.sent().last().unwrap().clone();
    verify_fin_ack(
        &fin_ack,
        local,
        TEST_GROUND_EID,
        s.seq,
        AckTxnStatus::TERMINATED,
    );
    if by_port {
        let done = h.file_done();
        assert_eq!(done.len(), 1);
        assert_eq!(done[0].status, SendFileStatus::StatusOk);
    }
    let completed = h.events(CfdpManager::EVENTID_TX_FILE_TRANSFER_COMPLETED);
    assert_eq!(completed.len(), 1);
    assert_eq!(
        completed_args(&completed[0]),
        (
            Class::Class2.as_repr(),
            s.seq,
            local,
            src.to_string(),
            TEST_GROUND_EID,
            dst.to_string(),
            size as u32
        )
    );
    h.wait_for_recycle(ch, s.seq);
    let tlm = h.tlm()[usize::from(ch)];
    let expected_sent = (1 + num_fd + 1 + 1) as u32;
    if exact && !simulate_nak {
        assert_eq!(tlm.sent_pdu, expected_sent);
        assert_eq!(tlm.sent_file_data_bytes as usize, size);
        assert_eq!(tlm.recv_pdu, 2);
    } else {
        assert!(tlm.sent_pdu >= expected_sent - 1);
        assert!(tlm.sent_file_data_bytes as usize >= size);
        assert!(tlm.recv_pdu >= 2);
    }
    if simulate_nak {
        assert!(tlm.recv_nak_segment_requests > 0);
    }
    assert_eq!(tlm.recv_errors, 0);
    let _ = std::fs::remove_file(src);
}

/// Upstream `sendAndVerifyClass1Rx`.
fn send_and_verify_class1_rx(
    h: &Harness,
    dir: &std::path::Path,
    dst: &str,
    ground_src: &str,
    size: usize,
    ch: u8,
) {
    let seq = 100;
    let data: Vec<u8> = (0..size).map(|i| i as u8).collect();
    let t = setup_rx(
        h,
        ground_src,
        dst,
        ch,
        Class::Class1,
        size as u32,
        seq,
        TxnState::R1,
    );
    h.send_pdu(
        ch,
        &fd_bytes(Class::Class1, TEST_GROUND_EID, h.local_eid(), seq, 0, &data),
    );
    h.txn(ch, t, |c, t| {
        assert_eq!(c.txn(t).state, TxnState::R1);
        assert_eq!(c.txn(t).state_data.receive.sub_state, RxSubState::FileData);
    });
    h.send_pdu(
        ch,
        &eof_bytes(
            Class::Class1,
            TEST_GROUND_EID,
            h.local_eid(),
            seq,
            ConditionCode::NO_ERROR,
            checksum(&data),
            size as u32,
        ),
    );
    h.txn(ch, t, |c, t| assert_eq!(c.txn(t).state, TxnState::Hold));
    assert_eq!(std::fs::read(dst).unwrap(), data);
    h.run_cycle();
    let tlm = h.tlm()[usize::from(ch)];
    assert_eq!(tlm.recv_pdu, 3);
    assert_eq!(tlm.recv_file_data_bytes as usize, size);
    assert_eq!(tlm.recv_errors, 0);
    let completed = h.events(CfdpManager::EVENTID_RX_FILE_TRANSFER_COMPLETED);
    assert_eq!(completed.len(), 1);
    assert_eq!(
        completed_args(&completed[0]),
        (
            Class::Class1.as_repr(),
            seq,
            TEST_GROUND_EID,
            ground_src.to_string(),
            h.local_eid(),
            dst.to_string(),
            size as u32
        )
    );
    h.wait_for_recycle(ch, seq);
    let _ = std::fs::remove_file(dst);
    let _ = dir;
}

/// Upstream `sendAndVerifyClass2Rx`: optionally drop PDUs 1, 2 and 4 so the
/// receiver NAKs them.
fn send_and_verify_class2_rx(
    h: &Harness,
    dst: &str,
    ground_src: &str,
    size: usize,
    simulate_nak: bool,
    exact: bool,
    ch: u8,
) {
    let chunk = h.chunk();
    let seq = if simulate_nak { 300 } else { 200 };
    let local = h.local_eid();
    let data: Vec<u8> = (0..size).map(|i| i as u8).collect();
    let t = setup_rx(
        h,
        ground_src,
        dst,
        ch,
        Class::Class2,
        size as u32,
        seq,
        TxnState::R2,
    );
    let send_fd = |i: usize| {
        h.send_pdu(
            ch,
            &fd_bytes(
                Class::Class2,
                TEST_GROUND_EID,
                local,
                seq,
                (i * chunk) as u32,
                &data[i * chunk..(i + 1) * chunk],
            ),
        );
    };
    let indices: Vec<usize> = if simulate_nak {
        vec![0, 3]
    } else {
        (0..size / chunk).collect()
    };
    for i in indices {
        send_fd(i);
    }
    h.txn(ch, t, |c, t| {
        assert_eq!(c.txn(t).state, TxnState::R2);
        assert_eq!(c.txn(t).state_data.receive.sub_state, RxSubState::FileData);
    });
    let before_eof = h.sent().len();
    h.send_pdu(
        ch,
        &eof_bytes(
            Class::Class2,
            TEST_GROUND_EID,
            local,
            seq,
            ConditionCode::NO_ERROR,
            checksum(&data),
            size as u32,
        ),
    );
    h.txn(ch, t, |c, t| {
        let f = c.txn(t).flags.rx;
        assert!(f.eof_recv);
        assert!(f.send_eof_ack);
        if simulate_nak {
            assert!(!f.send_fin);
            assert!(f.send_nak);
        } else {
            assert!(f.send_fin);
        }
    });
    h.run_cycle();
    let after_tick = h.sent().len();
    assert!(after_tick > before_eof);
    let ack = decode_ack(&h.sent()[before_eof]);
    assert_eq!(ack.as_header().get_source_eid(), TEST_GROUND_EID);
    assert_eq!(ack.as_header().get_dest_eid(), local);
    assert_eq!(ack.get_directive_code(), FileDirective::END_OF_FILE);
    assert_eq!(ack.get_transaction_status(), AckTxnStatus::ACTIVE);

    if simulate_nak {
        let mut nak = None;
        for _ in 0..20 {
            if let Some(p) = h.sent()[before_eof..]
                .iter()
                .find(|p| peek_pdu_type(p) == PduTypeEnum::NegativeAck)
            {
                nak = Some(decode_nak(p));
                break;
            }
            h.run_cycle();
        }
        let nak = nak.expect("NAK sent");
        let segs: Vec<(u32, u32)> = (0..nak.get_num_segments())
            .map(|i| {
                (
                    nak.get_segment(i).offset_start,
                    nak.get_segment(i).offset_end,
                )
            })
            .collect();
        let c = chunk as u32;
        assert_eq!(segs, [(c, 3 * c), (4 * c, 5 * c)]);
        for i in [1, 2, 4] {
            send_fd(i);
        }
        h.txn(ch, t, |c, t| assert!(c.txn(t).flags.rx.complete));
    }
    let before_fin = h.sent().len();
    let mut found_fin = false;
    for _ in 0..20 {
        h.run_cycle();
        if h.sent()[before_fin..]
            .iter()
            .any(|p| peek_pdu_type(p) == PduTypeEnum::Finished)
        {
            found_fin = true;
            break;
        }
    }
    assert!(found_fin, "FIN sent");
    let fin = decode_fin(h.sent().last().unwrap());
    assert_eq!(fin.get_condition_code(), ConditionCode::NO_ERROR);
    assert_eq!(fin.get_delivery_code(), FinDeliveryCode::Complete);
    assert_eq!(fin.get_file_status(), FinFileStatus::Retained);
    h.txn(ch, t, |c, t| {
        assert_eq!(c.txn(t).state, TxnState::R2);
        assert_eq!(
            c.txn(t).state_data.receive.sub_state,
            RxSubState::CloseoutSync
        );
    });
    h.send_pdu(
        ch,
        &ack_bytes(
            TEST_GROUND_EID,
            local,
            seq,
            FileDirective::FIN,
            1,
            ConditionCode::NO_ERROR,
            AckTxnStatus::TERMINATED,
        ),
    );
    h.txn(ch, t, |c, t| assert_eq!(c.txn(t).state, TxnState::Hold));
    let completed = h.events(CfdpManager::EVENTID_RX_FILE_TRANSFER_COMPLETED);
    assert_eq!(completed.len(), 1);
    assert_eq!(
        completed_args(&completed[0]),
        (
            Class::Class2.as_repr(),
            seq,
            TEST_GROUND_EID,
            ground_src.to_string(),
            local,
            dst.to_string(),
            size as u32
        )
    );
    h.wait_for_recycle(ch, seq);
    assert_eq!(std::fs::read(dst).unwrap(), data);
    let tlm = h.tlm()[usize::from(ch)];
    let num_fd = (size / chunk) as u32;
    if exact && !simulate_nak {
        assert_eq!(tlm.recv_pdu, 1 + num_fd + 1 + 1);
        assert_eq!(tlm.recv_file_data_bytes as usize, size);
        assert_eq!(tlm.sent_pdu, 2);
    } else {
        assert!(tlm.recv_pdu > num_fd);
        assert!(tlm.recv_file_data_bytes as usize >= size);
        assert!(tlm.sent_pdu >= 2);
    }
    if simulate_nak {
        assert!(tlm.sent_nak_segment_requests > 0);
    }
    assert_eq!(tlm.recv_errors, 0);
    let _ = std::fs::remove_file(dst);
}

#[test]
fn class1_tx_nominal() {
    let h = Harness::new();
    let dir = temp_dir();
    send_and_verify_class1_tx(
        &h,
        &path_in(&dir, "c1tx.bin"),
        &path_in(&dir, "c1tx_dst.dat"),
        h.chunk(),
    );
}

#[test]
fn class2_tx_nominal() {
    let h = Harness::new();
    let dir = temp_dir();
    let size = 5 * h.chunk();
    send_and_verify_class2_tx(
        &h,
        false,
        &path_in(&dir, "c2tx.bin"),
        &path_in(&dir, "c2tx_dst"),
        size,
        false,
        true,
    );
}

#[test]
fn class2_tx_nak() {
    let h = Harness::new();
    let dir = temp_dir();
    let size = 5 * h.chunk();
    send_and_verify_class2_tx(
        &h,
        false,
        &path_in(&dir, "c2nak.bin"),
        &path_in(&dir, "c2nak_dst"),
        size,
        true,
        false,
    );
}

#[test]
fn class2_tx_port_based() {
    let h = Harness::new();
    let dir = temp_dir();
    let size = h.chunk();
    send_and_verify_class2_tx(
        &h,
        true,
        &path_in(&dir, "port.bin"),
        &path_in(&dir, "port_dst"),
        size,
        false,
        true,
    );
}

#[test]
fn class2_tx_port_based_nak() {
    let h = Harness::new();
    let dir = temp_dir();
    let size = 5 * h.chunk();
    send_and_verify_class2_tx(
        &h,
        true,
        &path_in(&dir, "pnak.bin"),
        &path_in(&dir, "pnak_dst"),
        size,
        true,
        false,
    );
}

#[test]
fn class2_tx_late_fin_is_acked_statelessly() {
    let h = Harness::new();
    let dir = temp_dir();
    let src = path_in(&dir, "late.bin");
    let dst = path_in(&dir, "late_dst");
    make_file(&src, 5 * h.chunk());
    let s = setup_tx(
        &h,
        &src,
        &dst,
        CH1,
        TEST_GROUND_EID,
        Class::Class2,
        0,
        TxnState::S2,
    );
    h.run_cycle();
    h.run_cycle();
    complete_class2_handshake(&h, CH1, s.seq, s.t);
    assert_eq!(
        h.events(CfdpManager::EVENTID_TX_FILE_TRANSFER_COMPLETED)
            .len(),
        1
    );
    h.wait_for_recycle(CH1, s.seq);
    h.ground.clear();
    // A retransmitted FIN for the recycled transaction: source = us.
    h.send_pdu(
        CH1,
        &fin_bytes(
            h.local_eid(),
            TEST_GROUND_EID,
            s.seq,
            ConditionCode::NO_ERROR,
            FinDeliveryCode::Complete,
            FinFileStatus::Retained,
        ),
    );
    h.run_cycle();
    let sent = h.sent();
    assert_eq!(sent.len(), 1, "exactly the stateless FIN-ACK");
    verify_fin_ack(
        &sent[0],
        h.local_eid(),
        TEST_GROUND_EID,
        s.seq,
        AckTxnStatus::UNRECOGNIZED,
    );
    let late = h.events(CfdpManager::EVENTID_TX_LATE_FIN_ACKED);
    assert_eq!(late.len(), 1);
    assert_eq!(
        (args::u32_at(&late[0], 0), args::u32_at(&late[0], 4)),
        (42, s.seq)
    );
    assert_eq!(h.find_txn(CH1, s.seq), None, "no stray transaction");
}

#[test]
fn class1_rx_nominal() {
    let h = Harness::new();
    let dir = temp_dir();
    send_and_verify_class1_rx(
        &h,
        &dir,
        &path_in(&dir, "rx1.bin"),
        "/ground/test_rx_source.bin",
        h.chunk(),
        CH0,
    );
}

#[test]
fn class2_rx_nominal() {
    let h = Harness::new();
    let dir = temp_dir();
    let size = 5 * h.chunk();
    send_and_verify_class2_rx(
        &h,
        &path_in(&dir, "rx2.bin"),
        "/ground/c2rx.bin",
        size,
        false,
        true,
        CH0,
    );
}

#[test]
fn class2_rx_nak() {
    let h = Harness::new();
    let dir = temp_dir();
    let size = 5 * h.chunk();
    send_and_verify_class2_rx(
        &h,
        &path_in(&dir, "rx2nak.bin"),
        "/ground/c2nak.bin",
        size,
        true,
        false,
        CH0,
    );
}

#[test]
fn multi_channel_class1_rx_on_channel_1() {
    let h = Harness::new();
    let dir = temp_dir();
    send_and_verify_class1_rx(
        &h,
        &dir,
        &path_in(&dir, "mc1.bin"),
        "/ground/mc1.bin",
        h.chunk(),
        CH1,
    );
}

#[test]
fn multi_channel_class2_rx_on_channel_1() {
    let h = Harness::new();
    let dir = temp_dir();
    let size = 5 * h.chunk();
    send_and_verify_class2_rx(
        &h,
        &path_in(&dir, "mc2.bin"),
        "/ground/mc2.bin",
        size,
        false,
        true,
        CH1,
    );
}

#[test]
fn class2_rx_zero_length_file() {
    let h = Harness::new();
    let dir = temp_dir();
    let dst = path_in(&dir, "zero.bin");
    let seq = 500;
    let t = setup_rx(
        &h,
        "/ground/zero.bin",
        &dst,
        CH0,
        Class::Class2,
        0,
        seq,
        TxnState::R2,
    );
    h.send_pdu(
        CH0,
        &eof_bytes(
            Class::Class2,
            TEST_GROUND_EID,
            h.local_eid(),
            seq,
            ConditionCode::NO_ERROR,
            0,
            0,
        ),
    );
    h.txn(CH0, t, |c, t| {
        assert!(c.txn(t).flags.rx.eof_recv);
        assert!(c.txn(t).flags.rx.send_fin, "no gaps: FIN requested");
    });
    let mut found = false;
    for _ in 0..20 {
        h.run_cycle();
        if h.sent()
            .iter()
            .any(|p| peek_pdu_type(p) == PduTypeEnum::Finished)
        {
            found = true;
            break;
        }
    }
    assert!(found);
    h.txn(CH0, t, |c, t| {
        assert_eq!(
            c.txn(t).state_data.receive.sub_state,
            RxSubState::CloseoutSync
        );
        assert!(!utils::txn_status_is_error(c.hist(t).txn_stat));
    });
}

#[test]
fn class2_rx_file_data_offset_overflow_is_rejected() {
    let h = Harness::new();
    let seq = 501;
    let offset = FileSize::MAX - 16;
    h.send_pdu(
        CH0,
        &fd_bytes(
            Class::Class2,
            TEST_GROUND_EID,
            h.local_eid(),
            seq,
            offset,
            &[0xA5; 32],
        ),
    );
    let t = h.find_txn(CH0, seq).expect("R2 started without metadata");
    let dst = h.txn(CH0, t, |c, t| {
        assert!(!c.txn(t).flags.rx.md_recv);
        c.hist(t).fnames.dst_filename.clone()
    });
    let oob = h.events(CfdpManager::EVENTID_RX_FILE_DATA_OUT_OF_BOUNDS);
    assert_eq!(oob.len(), 1);
    let b = &oob[0];
    assert_eq!(b[0], Class::Class2.as_repr());
    assert_eq!(
        (
            args::u32_at(b, 1),
            args::u32_at(b, 5),
            args::u32_at(b, 9),
            args::u32_at(b, 13),
            args::u32_at(b, 17)
        ),
        (TEST_GROUND_EID, seq, offset, 32, FileSize::MAX)
    );
    h.run_cycle();
    assert_eq!(h.tlm()[0].recv_file_data_bytes, 0);
    let _ = std::fs::remove_file(dst.to_string());
}

#[test]
fn class2_rx_zero_length_file_data_is_a_no_op() {
    // GHSA-mh5x-2m6h-8267: an empty segment must not reach the chunk list.
    let h = Harness::new();
    let seq = 502;
    h.send_pdu(
        CH0,
        &fd_bytes(Class::Class2, TEST_GROUND_EID, h.local_eid(), seq, 0, &[]),
    );
    let t = h.find_txn(CH0, seq).expect("R2 started");
    let dst = h.txn(CH0, t, |c, t| {
        assert!(!c.txn(t).flags.rx.md_recv);
        assert!(!utils::txn_status_is_error(c.hist(t).txn_stat));
        assert_eq!(c.chunk_list(t).get_count(), 0);
        c.hist(t).fnames.dst_filename.clone()
    });
    assert!(
        h.events(CfdpManager::EVENTID_RX_FILE_DATA_OUT_OF_BOUNDS)
            .is_empty()
    );
    h.run_cycle();
    assert_eq!(h.tlm()[0].recv_file_data_bytes, 0);
    assert_eq!(h.tlm()[0].recv_errors, 0);
    let _ = std::fs::remove_file(dst.to_string());
}

#[test]
fn multiple_transactions_in_series() {
    let h = Harness::new();
    let dir = temp_dir();
    let chunk = h.chunk();
    send_and_verify_class1_tx(
        &h,
        &path_in(&dir, "s1tx.bin"),
        &path_in(&dir, "s1tx_dst"),
        chunk,
    );
    send_and_verify_class1_rx(
        &h,
        &dir,
        &path_in(&dir, "s1rx.bin"),
        "/ground/s1rx.bin",
        chunk,
        CH0,
    );
    send_and_verify_class2_tx(
        &h,
        true,
        &path_in(&dir, "s2tx.bin"),
        &path_in(&dir, "s2tx_dst"),
        5 * chunk,
        false,
        false,
    );
    send_and_verify_class2_rx(
        &h,
        &path_in(&dir, "s2rx.bin"),
        "/ground/s2rx.bin",
        5 * chunk,
        false,
        false,
        CH0,
    );
}

#[test]
fn ping_is_echoed() {
    let h = Harness::new();
    let p = h.comp.ping_in(0);
    p.target.invoke(p.port_num, 1234);
    h.dispatch();
    assert_eq!(*h.ground.pings.lock(), [1234]);
}

#[test]
fn data_return_in_deallocates_on_the_same_port() {
    let h = Harness::new();
    for ch in [CH0, CH1] {
        let p = h.comp.data_return_in(FwIndexType::from(ch));
        p.target.invoke(p.port_num, Buffer::allocate(16));
        h.dispatch();
    }
    assert_eq!(*h.ground.deallocated.lock(), [(0, 16), (1, 16)]);
}

#[test]
fn class2_rx_without_metadata_uses_a_temp_file_and_naks_for_metadata() {
    let dir = temp_dir();
    let tmp = dir.to_str().unwrap().to_string();
    // ChannelConfig with tmp_dir pointed at the test directory.
    let h = Harness::with_params(|g| {
        let mut cfg = ChannelArrayParams::fpp_default();
        cfg[0].tmp_dir = PathString::from(tmp.as_str());
        let mut buf = ParamBuffer::new();
        assert!(cfg.serialize_to(&mut buf, Endianness::Big).is_ok());
        g.prm_db
            .lock()
            .insert(CfdpManager::PARAMID_CHANNEL_CONFIG, buf.as_slice().to_vec());
    });
    let local = h.local_eid();
    let seq = 77;
    let chunk = h.chunk();
    let data: Vec<u8> = (0..2 * chunk).map(|i| (i * 7) as u8).collect();
    // File data before the metadata.
    h.send_pdu(
        CH0,
        &fd_bytes(
            Class::Class2,
            TEST_GROUND_EID,
            local,
            seq,
            0,
            &data[..chunk],
        ),
    );
    let t = h.find_txn(CH0, seq).unwrap();
    let temp = format!("{tmp}/{TEST_GROUND_EID}:{seq}.tmp");
    h.txn(CH0, t, |c, t| {
        assert_eq!(c.txn(t).state, TxnState::R2);
        assert_eq!(c.hist(t).fnames.dst_filename.as_bytes(), temp.as_bytes());
    });
    assert_eq!(h.events(CfdpManager::EVENTID_RX_TEMP_FILE_CREATED).len(), 1);
    // The ACK timer (2 s) expires: a NAK [0, 0] asks for the metadata.
    h.run_cycles(4);
    let nak = h
        .sent()
        .iter()
        .find(|p| peek_pdu_type(p) == PduTypeEnum::NegativeAck)
        .map(|p| decode_nak(p))
        .expect("metadata NAK");
    assert_eq!(nak.get_num_segments(), 1);
    assert_eq!(*nak.get_segment(0), SegmentRequest::default());
    // The metadata renames the temporary file to the real destination.
    let dst = path_in(&dir, "late_md.bin");
    h.send_pdu(
        CH0,
        &md_bytes(
            Class::Class2,
            TEST_GROUND_EID,
            local,
            seq,
            data.len() as u32,
            "/g/src",
            &dst,
        ),
    );
    assert!(!std::path::Path::new(&temp).exists());
    h.send_pdu(
        CH0,
        &fd_bytes(
            Class::Class2,
            TEST_GROUND_EID,
            local,
            seq,
            chunk as u32,
            &data[chunk..],
        ),
    );
    h.send_pdu(
        CH0,
        &eof_bytes(
            Class::Class2,
            TEST_GROUND_EID,
            local,
            seq,
            ConditionCode::NO_ERROR,
            checksum(&data),
            data.len() as u32,
        ),
    );
    h.run_cycles(3);
    let fin = h
        .sent()
        .iter()
        .rev()
        .find(|p| peek_pdu_type(p) == PduTypeEnum::Finished)
        .map(|p| decode_fin(p))
        .expect("FIN");
    assert_eq!(fin.get_condition_code(), ConditionCode::NO_ERROR);
    assert_eq!(fin.get_file_status(), FinFileStatus::Retained);
    assert_eq!(std::fs::read(&dst).unwrap(), data);
}
