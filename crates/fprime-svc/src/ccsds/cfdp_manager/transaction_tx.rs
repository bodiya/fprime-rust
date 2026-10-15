//! The send (S1/S2) transaction state machines — port of
//! `Svc/Ccsds/CfdpManager/TransactionTx.cpp`, derived from NASA CF 3.0.0
//! `cf_cfdp_s.c` and `cf_cfdp_dispatch.c`.
//!
//! Copyright (c) 2019 United States Government as represented by the
//! Administrator of the National Aeronautics and Space Administration.
//! All Rights Reserved. NASA Docket No. GSC-18,447-1. Licensed under the
//! Apache License, Version 2.0.
//!
//! ```text
//! METADATA --md sent--> FILEDATA --last byte--> EOF
//!   S1: EOF --> finished (EOF sent by the tick)
//!   S2: EOF --> CLOSEOUT_SYNC on TXW: EOF (re)sent on the ACK timer until
//!       EOF-ACK; FIN --> finished (FIN-ACK sent by the tick); NAKs queue
//!       retransmissions, answered first in FILEDATA and by the NAK tick
//! ```
//!
//! The C++ dispatch tables (`SSubstateRecvDispatchTable`, ...) are `match`
//! statements on the sub-state and directive code with the same entries.

use super::CfdpManager;
use super::channel::Channel;
use super::engine::path_str;
use super::pdu::{
    AckPdu, FileDataPdu, FinPdu, NakPdu, PduDirection, PduHeader, PduReader, peek_pdu_type,
};
use super::timer::TimerStatus;
use super::transaction::TxnIdx;
use super::types::{
    ConditionCode, FileDirective, FileSize, PduTypeEnum, QueueId, Status, TxSubState, TxnState,
    TxnStatus,
};
use super::utils::get_txn_status;
use fprime_config::FwSizeType;
use fprime_config::cfdp::MAX_PDU_SIZE;
use fprime_fw::{SerializeStatus, fw_assert};
use fprime_os::file::{Mode as FileMode, Status as FileStatus, WaitType};

/// The receive handlers a send transaction can select (C++
/// `StateRecvFunc` entries of the S2 tables).
#[derive(Clone, Copy)]
enum SRecv {
    EarlyFin,
    Fin,
    Nak,
    NakArm,
    EofAck,
}

impl Channel {
    /// C++ `s1Recv`: class 1 senders take no directives.
    pub fn s1_recv(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        self.s_dispatch_recv(mgr, t, data, |_, _| None);
    }

    /// C++ `s2Recv`.
    pub fn s2_recv(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        self.s_dispatch_recv(mgr, t, data, |sub_state, directive| match sub_state {
            TxSubState::Metadata => match directive {
                FileDirective::FIN => Some(SRecv::EarlyFin),
                _ => None,
            },
            TxSubState::FileData | TxSubState::Eof => match directive {
                FileDirective::FIN => Some(SRecv::EarlyFin),
                FileDirective::NAK => Some(SRecv::Nak),
                _ => None,
            },
            TxSubState::CloseoutSync => match directive {
                FileDirective::FIN => Some(SRecv::Fin),
                FileDirective::ACK => Some(SRecv::EofAck),
                FileDirective::NAK => Some(SRecv::NakArm),
                _ => None,
            },
            TxSubState::NumStates => None,
        });
    }

    /// C++ `s1Tx`.
    pub fn s1_tx(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        match self.txn(t).state_data.send.sub_state {
            TxSubState::Metadata => self.s_substate_send_metadata(mgr, t),
            TxSubState::FileData => self.s_substate_send_file_data(mgr, t),
            TxSubState::Eof => self.s1_substate_send_eof(mgr, t),
            _ => {}
        }
    }

    /// C++ `s2Tx`.
    pub fn s2_tx(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        match self.txn(t).state_data.send.sub_state {
            TxSubState::Metadata => self.s_substate_send_metadata(mgr, t),
            TxSubState::FileData => self.s2_substate_send_file_data(mgr, t),
            TxSubState::Eof => self.s2_substate_send_eof(mgr, t),
            _ => {}
        }
    }

    /// C++ `sAckTimerTick` (class 2): while waiting in CLOSEOUT_SYNC, each
    /// ACK-timer expiry re-sends the EOF until the ACK limit gives up.
    pub fn s_ack_timer_tick(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        if self.txn(t).state != TxnState::S2 || !self.txn(t).flags.com.ack_timer_armed {
            return;
        }
        if self.txn(t).ack_timer.get_status() == TimerStatus::Running {
            self.txn_mut(t).ack_timer.run();
        } else if self.txn(t).state_data.send.sub_state == TxSubState::CloseoutSync {
            let ack_limit = mgr.get_ack_limit_param(self.txn(t).chan_num);
            if self.txn(t).state_data.send.s2.acknak_count >= ack_limit {
                let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
                mgr.log_warning_lo_tx_ack_limit_reached(self.txn(t).txn_class, src, seq);
                self.set_txn_status(t, TxnStatus::ACK_LIMIT_NO_EOF);
                self.tlm.fault_ack_limit = self.tlm.fault_ack_limit.wrapping_add(1);
                self.finish_transaction(mgr, t, true);
                self.txn_mut(t).flags.com.ack_timer_armed = false;
            } else {
                let txn = self.txn_mut(t);
                txn.state_data.send.s2.acknak_count += 1;
                // A FIN is an implicit EOF-ACK; nothing is re-sent after it.
                if !txn.flags.tx.eof_ack_recv && !txn.flags.tx.fin_recv {
                    txn.flags.tx.send_eof = true;
                } else {
                    txn.flags.com.ack_timer_armed = false;
                }
            }
            if self.txn(t).flags.com.ack_timer_armed {
                self.arm_ack_timer(mgr, t);
            }
        } else {
            // Not waiting for anything: disarm.
            self.txn_mut(t).flags.com.ack_timer_armed = false;
        }
    }

    /// C++ `sTick`: run the inactivity timer, send a pending EOF or FIN-ACK,
    /// and recycle the transaction once inactive with nothing left to send
    /// (or after `PostInactivitySendRetries` failed sends); otherwise tick
    /// the ACK timer.
    pub fn s_tick(&mut self, mgr: &CfdpManager, t: TxnIdx, _cont: &mut bool) {
        if !self.txn(t).flags.com.inactivity_fired
            && self.txn(t).inactivity_timer.get_status() == TimerStatus::Running
        {
            self.txn_mut(t).inactivity_timer.run();
            if self.txn(t).inactivity_timer.get_status() == TimerStatus::Expired {
                self.txn_mut(t).flags.com.inactivity_fired = true;
                // HOLD (the normal recycle path) and canceled transactions
                // waiting for an EOF-ACK are not faults.
                if self.txn(t).state != TxnState::Hold
                    && self.txn(t).state == TxnState::S2
                    && !self.txn(t).flags.com.canceled
                {
                    let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
                    mgr.log_warning_lo_tx_inactivity_timeout(self.txn(t).txn_class, src, seq);
                    self.set_txn_status(t, TxnStatus::INACTIVITY_DETECTED);
                    self.tlm.fault_inactivity_timer =
                        self.tlm.fault_inactivity_timer.wrapping_add(1);
                }
            }
        }
        // On ERROR stop retrying; on no buffer, retry next tick.
        let mut pending_send = true;
        if self.txn(t).flags.tx.send_eof {
            match self.s_send_eof(mgr, t) {
                Status::Success => self.txn_mut(t).flags.tx.send_eof = false,
                Status::Error => {
                    self.txn_mut(t).flags.tx.send_eof = false;
                    pending_send = false;
                }
                _ => {}
            }
        } else if self.txn(t).flags.tx.send_fin_ack {
            match self.s_send_fin_ack(mgr, t) {
                Status::Success => self.txn_mut(t).flags.tx.send_fin_ack = false,
                Status::Error => {
                    self.txn_mut(t).flags.tx.send_fin_ack = false;
                    pending_send = false;
                }
                _ => {}
            }
        } else {
            pending_send = false;
        }
        let mut retries_exhausted = false;
        if self.txn(t).flags.com.inactivity_fired && pending_send {
            if self.txn(t).flags.com.post_inactivity_send_retries
                >= mgr.get_post_inactivity_send_retries_param()
            {
                retries_exhausted = true;
            } else {
                self.txn_mut(t).flags.com.post_inactivity_send_retries += 1;
            }
        }
        if self.txn(t).flags.com.inactivity_fired && (!pending_send || retries_exhausted) {
            // Recyclable: later PDUs for this sequence number are spurious.
            self.recycle_transaction(mgr, t);
        } else {
            self.s_ack_timer_tick(mgr, t);
        }
    }

    /// C++ `sTickNak`: answer one NAKed range per call (class 2); `cont`
    /// asks the channel to tick again this cycle.
    pub fn s_tick_nak(&mut self, mgr: &CfdpManager, t: TxnIdx, cont: &mut bool) {
        if self.txn(t).txn_class == super::types::Class::Class2 {
            let mut nak_processed = false;
            let status = self.s_check_and_respond_nak(mgr, t, &mut nak_processed);
            if status == Status::Success && nak_processed {
                *cont = true;
            }
        }
    }

    /// C++ `sCancel`: skip to EOF if not there yet.
    pub fn s_cancel(&mut self, t: TxnIdx) {
        let send = &mut self.txn_mut(t).state_data.send;
        if send.sub_state < TxSubState::Eof {
            send.sub_state = TxSubState::Eof;
        }
    }

    /// C++ `sSendEof`: the checksum is final whatever happened.
    fn s_send_eof(&mut self, mgr: &CfdpManager, t: TxnIdx) -> Status {
        if !self.txn(t).flags.com.crc_calc {
            // The F´ checksum needs no finalize step.
            self.txn_mut(t).flags.com.crc_calc = true;
        }
        self.send_eof(mgr, t)
    }

    /// C++ `s1SubstateSendEof`: the tick sends the EOF; class 1 is done.
    fn s1_substate_send_eof(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        self.txn_mut(t).flags.tx.send_eof = true;
        self.finish_transaction(mgr, t, true);
    }

    /// C++ `s2SubstateSendEof`: the tick sends the EOF; wait on TXW for the
    /// close-out with the ACK timer armed.
    fn s2_substate_send_eof(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let txn = self.txn_mut(t);
        txn.flags.tx.send_eof = true;
        txn.state_data.send.sub_state = TxSubState::CloseoutSync;
        self.dequeue_transaction(t);
        self.insert_sort_prio(t, QueueId::Txw);
        self.arm_ack_timer(mgr, t);
    }

    /// C++ `sSendFileData`: read up to `bytes_to_read` (bounded by the
    /// `OutgoingFileChunkSize` parameter and the PDU capacity) at `foffs`
    /// and send it as one file data PDU, optionally folding it into the
    /// checksum.
    fn s_send_file_data(
        &mut self,
        mgr: &CfdpManager,
        t: TxnIdx,
        foffs: FileSize,
        bytes_to_read: FileSize,
        calc_crc: bool,
        bytes_processed: &mut FileSize,
    ) -> Status {
        *bytes_processed = 0;
        let mut file_data = [0u8; MAX_PDU_SIZE];
        let mut fd_pdu = FileDataPdu::new();
        let max_data_capacity = fd_pdu.get_max_file_data_size();
        let max_data_bytes = bytes_to_read
            .min(mgr.get_outgoing_file_chunk_size_param())
            .min(max_data_capacity);
        let mut actual_bytes = FwSizeType::from(max_data_bytes);
        if self.txn(t).state_data.send.cached_pos != foffs
            && self.txn_mut(t).fd.seek_absolute(FwSizeType::from(foffs)) != FileStatus::OpOk
        {
            return Status::Error;
        }
        let read_status = self.txn_mut(t).fd.read(
            &mut file_data[..max_data_bytes as usize],
            &mut actual_bytes,
            WaitType::Wait,
        );
        if read_status != FileStatus::OpOk {
            return Status::Error;
        }
        // Advance before sending so a throttled send does not re-read.
        let send = &mut self.txn_mut(t).state_data.send;
        send.cached_pos = send.cached_pos.wrapping_add(actual_bytes as FileSize);
        let local_eid = mgr.get_local_eid_param();
        let (seq, peer) = (self.hist(t).seq_num, self.hist(t).peer_eid);
        fd_pdu.initialize(
            PduDirection::TowardReceiver,
            self.txn(t).txn_class,
            local_eid,
            seq,
            peer,
            foffs,
            actual_bytes as u16,
            &file_data,
        );
        let status = self.send_fd(mgr, &fd_pdu);
        if status == Status::Success {
            let fsize = self.txn(t).fsize;
            fw_assert!(
                u64::from(foffs) + actual_bytes <= u64::from(fsize),
                foffs as i32,
                actual_bytes as i32,
                fsize as i32
            );
            if calc_crc {
                self.txn_mut(t)
                    .crc
                    .update(&file_data[..actual_bytes as usize], foffs);
            }
            *bytes_processed = actual_bytes as FileSize;
        }
        status
    }

    /// C++ `sSubstateSendFileData`: send the next piece of the file; on no
    /// buffer stop for this cycle, on an I/O error fail to EOF, at the end
    /// of the file go to EOF.
    fn s_substate_send_file_data(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let mut bytes_processed: FileSize = 0;
        let (foffs, fsize) = (self.txn(t).foffs, self.txn(t).fsize);
        let status = self.s_send_file_data(
            mgr,
            t,
            foffs,
            fsize.wrapping_sub(foffs),
            true,
            &mut bytes_processed,
        );
        if status == Status::SendPduNoBufAvailError {
            // Throttled or out of buffers: retry next cycle.
            self.set_current_txn(t);
        } else if status != Status::Success {
            self.set_txn_status(t, TxnStatus::FILESTORE_REJECTION);
            self.txn_mut(t).state_data.send.sub_state = TxSubState::Eof;
        } else if bytes_processed > 0 {
            let txn = self.txn_mut(t);
            txn.foffs += bytes_processed;
            if txn.foffs == txn.fsize {
                txn.state_data.send.sub_state = TxSubState::Eof;
            }
        }
    }

    /// C++ `sCheckAndRespondNak`: re-send the metadata if NAKed, else
    /// retransmit from the first NAKed range.
    fn s_check_and_respond_nak(
        &mut self,
        mgr: &CfdpManager,
        t: TxnIdx,
        nak_processed: &mut bool,
    ) -> Status {
        *nak_processed = false;
        fw_assert!(self.txn(t).chunks.is_some());
        if self.txn(t).flags.tx.md_need_send {
            let sret = self.send_md(mgr, t);
            if sret == Status::Error {
                return Status::Error;
            }
            if sret == Status::Success {
                self.txn_mut(t).flags.tx.md_need_send = false;
            } else {
                // Deviation (upstream livelock): without a buffer nothing
                // breaks `cycleTxFirstActive`'s loop, so mark where this
                // cycle stopped, as CF's `CF_CFDP_MsgOutGet` does.
                self.set_current_txn(t);
            }
            // Sent or throttled: no file data this cycle either way.
            *nak_processed = true;
            return Status::Success;
        }
        let mut ret = Status::Success;
        if let Some(chunk) = self.chunk_list(t).get_first_chunk() {
            let mut bytes_processed: FileSize = 0;
            ret = self.s_send_file_data(
                mgr,
                t,
                chunk.offset,
                chunk.size,
                false,
                &mut bytes_processed,
            );
            if ret != Status::Success {
                ret = Status::Error;
            } else if bytes_processed > 0 {
                self.chunk_list_mut(t).remove_from_first(bytes_processed);
                *nak_processed = true;
            }
        }
        ret
    }

    /// C++ `s2SubstateSendFileData`: NAK responses first, then new data; a
    /// failed NAK response ends the transaction (still sending the EOF).
    fn s2_substate_send_file_data(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let mut nak_processed = false;
        let status = self.s_check_and_respond_nak(mgr, t, &mut nak_processed);
        if status != Status::Success {
            self.set_txn_status(t, TxnStatus::NAK_RESPONSE_ERROR);
            self.txn_mut(t).flags.tx.send_eof = true;
            self.finish_transaction(mgr, t, true);
            return;
        }
        if !nak_processed {
            self.s_substate_send_file_data(mgr, t);
        }
    }

    /// C++ `sSubstateSendMetadata`: open the file (a zero-length file is
    /// refused) and send the metadata; failures end the transaction.
    fn s_substate_send_metadata(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let mut success = true;
        let class = self.txn(t).txn_class;
        if !self.txn(t).fd.is_open() {
            let path = path_str(self.hist(t).fnames.src_filename.as_bytes()).into_owned();
            let status = self.txn_mut(t).fd.open(&path, FileMode::OpenRead);
            if status != FileStatus::OpOk {
                let hist = self.hist(t);
                mgr.log_warning_lo_tx_file_open_failed(
                    class,
                    hist.src_eid,
                    hist.seq_num,
                    hist.fnames.src_filename.as_bytes(),
                    status as i32,
                );
                self.tlm.fault_file_open = self.tlm.fault_file_open.wrapping_add(1);
                success = false;
            }
            if success {
                let mut file_size: FwSizeType = 0;
                let status = self.txn_mut(t).fd.size(&mut file_size);
                self.txn_mut(t).fsize = file_size as FileSize;
                let hist = self.hist(t);
                if status != FileStatus::OpOk {
                    mgr.log_warning_lo_tx_file_seek_failed(
                        class,
                        hist.src_eid,
                        hist.seq_num,
                        status as i32,
                    );
                    self.tlm.fault_file_seek = self.tlm.fault_file_seek.wrapping_add(1);
                    success = false;
                } else if self.txn(t).fsize == 0 {
                    mgr.log_warning_lo_tx_zero_length_file(
                        class,
                        hist.src_eid,
                        hist.seq_num,
                        hist.fnames.src_filename.as_bytes(),
                    );
                    self.tlm.fault_file_size_mismatch =
                        self.tlm.fault_file_size_mismatch.wrapping_add(1);
                    success = false;
                }
            }
        }
        if success {
            let status = self.send_md(mgr, t);
            if status == Status::Error {
                let hist = self.hist(t);
                mgr.log_warning_lo_tx_send_metadata_failed(class, hist.src_eid, hist.seq_num);
                success = false;
            } else if status == Status::Success {
                self.txn_mut(t).state_data.send.sub_state = TxSubState::FileData;
                let hist = self.hist(t);
                mgr.log_activity_hi_tx_file_transfer_started(
                    class,
                    hist.seq_num,
                    hist.src_eid,
                    hist.fnames.src_filename.as_bytes(),
                    hist.peer_eid,
                    hist.fnames.dst_filename.as_bytes(),
                    self.txn(t).fsize,
                );
            }
            if status == Status::SendPduNoBufAvailError {
                // Deviation (upstream livelock): retry the metadata next
                // cycle, stopping this cycle's transmit loop as CF's
                // `CF_CFDP_MsgOutGet` does; upstream leaves
                // `cycleTxFirstActive` spinning on it.
                self.set_current_txn(t);
            }
        }
        if !success {
            self.set_txn_status(t, TxnStatus::FILESTORE_REJECTION);
            self.finish_transaction(mgr, t, true);
        }
    }

    /// C++ `sSendFinAck`.
    fn s_send_fin_ack(&mut self, mgr: &CfdpManager, t: TxnIdx) -> Status {
        let ts = get_txn_status(Some(self.txn(t).state));
        let cc = ConditionCode(self.txn(t).state_data.send.s2.fin_cc);
        let (peer, seq) = (self.hist(t).peer_eid, self.hist(t).seq_num);
        self.send_ack(mgr, t, ts, FileDirective::FIN, cc, peer, seq)
    }

    /// C++ `s2EarlyFin`: a FIN before the EOF cancels the transfer.
    fn s2_early_fin(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
        mgr.log_warning_lo_tx_early_fin_received(self.txn(t).txn_class, src, seq);
        self.set_txn_status(t, TxnStatus::EARLY_FIN);
        self.txn_mut(t).state_data.send.sub_state = TxSubState::CloseoutSync;
        self.s2_fin(mgr, t, data);
    }

    /// C++ `s2Fin`: always (re-)acknowledge; the first FIN records its
    /// condition code and finishes the transaction.
    fn s2_fin(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        let mut fin = FinPdu::new();
        let status = fin.deserialize_from(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_fin_pdu_deserialization(self.txn(t).chan_num, status as i32);
            return;
        }
        if self.recv_fin(t, &fin) == Status::Success {
            self.txn_mut(t).flags.tx.send_fin_ack = true;
            if !self.txn(t).flags.tx.fin_recv {
                let txn = self.txn_mut(t);
                txn.flags.tx.fin_recv = true;
                txn.state_data.send.s2.fin_cc = fin.get_condition_code().0;
                txn.state_data.send.s2.acknak_count = 0;
                let fin_cc = txn.state_data.send.s2.fin_cc;
                // A no-op unless no status was set yet.
                self.set_txn_status(t, TxnStatus(i32::from(fin_cc)));
                // FIN ends the exchange: stop sending to the peer.
                self.finish_transaction(mgr, t, true);
            }
        }
    }

    /// C++ `s2Nak`: queue the requested ranges for retransmission (`[0,0]`
    /// re-requests the metadata); reversed or past-the-end ranges are
    /// counted as invalid.
    fn s2_nak(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        let mut nak = NakPdu::new();
        let status = nak.deserialize_from(&mut PduReader::new(data));
        let chan_num = self.txn(t).chan_num;
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_nak_pdu_deserialization(chan_num, status as i32);
            self.tlm.recv_errors = self.tlm.recv_errors.wrapping_add(1);
            return;
        }
        let (src, seq, class) = (
            self.hist(t).src_eid,
            self.hist(t).seq_num,
            self.txn(t).txn_class,
        );
        if self.recv_nak(t, &nak) == Status::Success && nak.get_num_segments() > 0 {
            let mut bad_sr: u8 = 0;
            for counter in 0..nak.get_num_segments() {
                let sr = *nak.get_segment(counter);
                if sr.offset_start == 0 && sr.offset_end == 0 {
                    self.txn_mut(t).flags.tx.md_need_send = true;
                } else if sr.offset_end < sr.offset_start || sr.offset_end > self.txn(t).fsize {
                    bad_sr = bad_sr.wrapping_add(1);
                } else {
                    self.chunk_list_mut(t)
                        .add(sr.offset_start, sr.offset_end - sr.offset_start);
                }
            }
            self.tlm.recv_nak_segment_requests = self
                .tlm
                .recv_nak_segment_requests
                .wrapping_add(u32::from(nak.get_num_segments()));
            if bad_sr > 0 {
                mgr.log_warning_lo_tx_invalid_segment_requests(class, src, seq, u32::from(bad_sr));
            }
        } else {
            mgr.log_warning_lo_tx_invalid_nak_pdu(class, src, seq);
            self.tlm.recv_errors = self.tlm.recv_errors.wrapping_add(1);
        }
    }

    /// C++ `s2NakArm`: a NAK in close-out also re-arms the ACK timer.
    fn s2_nak_arm(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        self.arm_ack_timer(mgr, t);
        self.s2_nak(mgr, t, data);
    }

    /// C++ `s2EofAck`: an EOF-ACK stops the EOF retransmissions; a canceled
    /// transaction (no FIN coming) or one already FINed is finished.
    fn s2_eof_ack(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        let mut ack = AckPdu::new();
        let status = ack.deserialize_from(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_ack_pdu_deserialization(self.txn(t).chan_num, status as i32);
            return;
        }
        if ack.get_directive_code() == FileDirective::END_OF_FILE {
            let txn = self.txn_mut(t);
            txn.flags.tx.eof_ack_recv = true;
            txn.flags.com.ack_timer_armed = false;
            txn.state_data.send.s2.acknak_count = 0;
            if txn.flags.com.canceled || txn.flags.tx.fin_recv {
                self.finish_transaction(mgr, t, true);
            }
        }
    }

    /// C++ `sDispatchRecv`: file data is rejected with an event; a directive
    /// selects a handler from the sub-state's table (silently ignored when
    /// there is none); an out-of-range directive code is reported.
    fn s_dispatch_recv(
        &mut self,
        mgr: &CfdpManager,
        t: TxnIdx,
        data: &[u8],
        table: impl Fn(TxSubState, FileDirective) -> Option<SRecv>,
    ) {
        let sub_state = self.txn(t).state_data.send.sub_state;
        fw_assert!(
            sub_state < TxSubState::NumStates,
            sub_state as i32,
            TxSubState::NumStates as i32
        );
        let mut selected = None;
        let (src, seq, class) = (
            self.hist(t).src_eid,
            self.hist(t).seq_num,
            self.txn(t).txn_class,
        );
        if peek_pdu_type(data) == PduTypeEnum::FileData {
            mgr.log_warning_lo_tx_non_file_directive_pdu_received(class, src, seq);
        } else {
            let mut reader = PduReader::new(data);
            let mut header = PduHeader::default();
            let mut directive_code = 0u8;
            if header.from_serial_buffer(&mut reader) == SerializeStatus::Ok
                && reader.read_u8(&mut directive_code) == SerializeStatus::Ok
            {
                let directive = FileDirective(directive_code);
                if directive < FileDirective::INVALID_MAX {
                    selected = table(sub_state, directive);
                } else {
                    mgr.log_warning_lo_tx_invalid_directive_code(
                        class,
                        src,
                        seq,
                        directive_code,
                        sub_state as u8,
                    );
                }
            }
        }
        match selected {
            Some(SRecv::EarlyFin) => self.s2_early_fin(mgr, t, data),
            Some(SRecv::Fin) => self.s2_fin(mgr, t, data),
            Some(SRecv::Nak) => self.s2_nak(mgr, t, data),
            Some(SRecv::NakArm) => self.s2_nak_arm(mgr, t, data),
            Some(SRecv::EofAck) => self.s2_eof_ack(mgr, t, data),
            None => {}
        }
    }
}
