//! The receive (R1/R2) transaction state machines — port of
//! `Svc/Ccsds/CfdpManager/TransactionRx.cpp`, derived from NASA CF 3.0.0
//! `cf_cfdp_r.c` and `cf_cfdp_dispatch.c`.
//!
//! Copyright (c) 2019 United States Government as represented by the
//! Administrator of the National Aeronautics and Space Administration.
//! All Rights Reserved. NASA Docket No. GSC-18,447-1. Licensed under the
//! Apache License, Version 2.0.
//!
//! ```text
//! R1: FILEDATA (write, checksum as it comes) --EOF--> checksum check --> finished
//! R2: FILEDATA (write, track chunks) --EOF--> EOF-ACK; gaps --> NAK on the ACK
//!     timer (NAK limit); complete --> checksum the file in slices per tick
//!     --> FIN --> CLOSEOUT_SYNC --FIN-ACK--> finished (ACK limit on FIN)
//! ```
//!
//! Without metadata an R2 transaction writes to `<tmp_dir>/<eid>:<seq>.tmp`
//! and NAKs for the metadata, renaming the file when it arrives.

use super::CfdpManager;
use super::channel::Channel;
use super::chunk::Chunk;
use super::engine::path_str;
use super::pdu::{
    AckPdu, EofPdu, FileDataPdu, MetadataPdu, NakPdu, PduDirection, PduHeader, PduReader,
    peek_pdu_type,
};
use super::timer::TimerStatus;
use super::transaction::TxnIdx;
use super::types::{
    AckTxnStatus, ConditionCode, FileDirective, FileSize, FinDeliveryCode, FinFileStatus, Keep,
    PduTypeEnum, RxSubState, Status, TxnState, TxnStatus,
};
use super::utils::{txn_status_is_error, txn_status_to_condition_code};
use fprime_config::FwSizeType;
use fprime_config::cfdp::R2_CRC_CHUNK_SIZE;
use fprime_fw::{SerializeStatus, fw_assert};
use fprime_os::file::{Mode as FileMode, OverwriteType, Status as FileStatus, WaitType};
use fprime_os::filesystem::{self, Status as FsStatus};
use fprime_utils::cfdp::Checksum;
use std::fmt::Write as _;

/// The receive handlers a receive transaction can select (C++
/// `StateRecvFunc` entries of the R tables).
#[derive(Clone, Copy)]
enum RRecv {
    R1Eof,
    R2Eof,
    R2Md,
    R2FinAck,
}

impl Channel {
    /// C++ `r1Recv`: file data, and EOF in every sub-state.
    pub fn r1_recv(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        self.r_dispatch_recv(mgr, t, data, true, |_, directive| {
            (directive == FileDirective::END_OF_FILE).then_some(RRecv::R1Eof)
        });
    }

    /// C++ `r2Recv`: file data; EOF always; metadata before close-out,
    /// FIN-ACK in close-out.
    pub fn r2_recv(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        self.r_dispatch_recv(mgr, t, data, false, |sub_state, directive| {
            match (sub_state, directive) {
                (_, FileDirective::END_OF_FILE) => Some(RRecv::R2Eof),
                (RxSubState::FileData | RxSubState::Eof, FileDirective::METADATA) => {
                    Some(RRecv::R2Md)
                }
                (RxSubState::CloseoutSync, FileDirective::ACK) => Some(RRecv::R2FinAck),
                _ => None,
            }
        });
    }

    /// C++ `rAckTimerTick` (class 2): on expiry, check for completion
    /// (NAKing gaps) or, in close-out, re-send the FIN until the ACK limit.
    pub fn r_ack_timer_tick(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        if self.txn(t).state != TxnState::R2 || !self.txn(t).flags.com.ack_timer_armed {
            return;
        }
        if self.txn(t).ack_timer.get_status() == TimerStatus::Running {
            self.txn_mut(t).ack_timer.run();
            return;
        }
        if !self.txn(t).flags.rx.complete {
            self.r2_complete(mgr, t, true);
        } else if self.txn(t).state_data.receive.sub_state == RxSubState::CloseoutSync {
            let txn = self.txn_mut(t);
            txn.state_data.receive.r2.acknak_count =
                txn.state_data.receive.r2.acknak_count.wrapping_add(1);
            let ack_limit = mgr.get_ack_limit_param(self.txn(t).chan_num);
            if self.txn(t).state_data.receive.r2.acknak_count >= ack_limit {
                let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
                mgr.log_warning_lo_rx_ack_limit_reached(self.txn(t).txn_class, src, seq);
                self.set_txn_status(t, TxnStatus::ACK_LIMIT_NO_FIN);
                self.tlm.fault_ack_limit = self.tlm.fault_ack_limit.wrapping_add(1);
                self.finish_transaction(mgr, t, true);
                self.txn_mut(t).flags.com.ack_timer_armed = false;
            } else {
                self.txn_mut(t).flags.rx.send_fin = true;
            }
        }
        if self.txn(t).flags.com.ack_timer_armed {
            self.arm_ack_timer(mgr, t);
        }
    }

    /// C++ `rTick`: run the inactivity timer (an R2 timeout sends an early
    /// FIN), send a pending EOF-ACK, NAK or FIN, and recycle once inactive
    /// with nothing left to send (or after `PostInactivitySendRetries`);
    /// otherwise tick the ACK timer.
    pub fn r_tick(&mut self, mgr: &CfdpManager, t: TxnIdx, _cont: &mut bool) {
        if !self.txn(t).flags.com.inactivity_fired
            && self.txn(t).inactivity_timer.get_status() == TimerStatus::Running
        {
            self.txn_mut(t).inactivity_timer.run();
            if self.txn(t).inactivity_timer.get_status() == TimerStatus::Expired {
                self.txn_mut(t).flags.com.inactivity_fired = true;
                if self.txn(t).state != TxnState::Hold && !self.txn(t).flags.com.canceled {
                    self.r_send_inactivity_event(mgr, t);
                    if self.txn(t).state == TxnState::R2 {
                        self.r2_set_fin_txn_status(t, TxnStatus::INACTIVITY_DETECTED);
                    }
                }
            }
        }
        let mut pending_send = true;
        if self.txn(t).flags.rx.send_eof_ack {
            let cc = ConditionCode(self.txn(t).state_data.receive.r2.eof_cc);
            let (peer, seq) = (self.hist(t).peer_eid, self.hist(t).seq_num);
            let sret = self.send_ack(
                mgr,
                t,
                AckTxnStatus::ACTIVE,
                FileDirective::END_OF_FILE,
                cc,
                peer,
                seq,
            );
            match sret {
                Status::Success => self.txn_mut(t).flags.rx.send_eof_ack = false,
                Status::Error => {
                    self.txn_mut(t).flags.rx.send_eof_ack = false;
                    pending_send = false;
                }
                _ => {}
            }
        } else if self.txn(t).flags.rx.send_nak {
            // C++ parity: only SUCCESS clears the flag; it re-enters on error.
            if self.r_substate_send_nak(mgr, t) == Status::Success {
                self.txn_mut(t).flags.rx.send_nak = false;
            }
        } else if self.txn(t).flags.rx.send_fin {
            if self.r2_substate_send_fin(mgr, t) == Status::Success {
                self.txn_mut(t).flags.rx.send_fin = false;
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
            self.recycle_transaction(mgr, t);
        } else {
            self.r_ack_timer_tick(mgr, t);
        }
    }

    /// C++ `rCancel`: an R2 not yet in close-out sends a FIN; otherwise done.
    pub fn r_cancel(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        if self.txn(t).state == TxnState::R2
            && self.txn(t).state_data.receive.sub_state < RxSubState::CloseoutSync
        {
            self.txn_mut(t).flags.rx.send_fin = true;
        } else {
            self.r1_reset(mgr, t);
        }
    }

    /// C++ `rInit`: create the destination file (a temporary one for R2
    /// without metadata, arming the ACK timer for R2). Failure ends R1 and
    /// sends a FIN for R2.
    pub fn r_init(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let chan_num = self.txn(t).chan_num;
        let class = self.txn(t).txn_class;
        if self.txn(t).state == TxnState::R2 {
            if !self.txn(t).flags.rx.md_recv {
                let tmp_dir = mgr.get_tmp_dir_param(chan_num);
                let hist = self.hist_mut(t);
                let (eid, seq) = (hist.src_eid, hist.seq_num);
                // <tmpDir>/<src_eid>:<seq_num>.tmp
                hist.fnames.dst_filename.clear();
                let _ = write!(
                    hist.fnames.dst_filename,
                    "{}/{}:{}.tmp",
                    path_str(tmp_dir.as_bytes()),
                    eid,
                    seq
                );
                let hist = self.hist(t);
                mgr.log_activity_lo_rx_temp_file_created(
                    class,
                    hist.src_eid,
                    hist.seq_num,
                    hist.fnames.dst_filename.as_bytes(),
                );
            }
            self.arm_ack_timer(mgr, t);
        }
        let path = path_str(self.hist(t).fnames.dst_filename.as_bytes()).into_owned();
        let status = self.txn_mut(t).fd.open_with_overwrite(
            &path,
            FileMode::OpenCreate,
            OverwriteType::Overwrite,
        );
        if status != FileStatus::OpOk {
            let hist = self.hist(t);
            mgr.log_warning_lo_rx_file_create_failed(
                class,
                hist.src_eid,
                hist.seq_num,
                hist.fnames.dst_filename.as_bytes(),
                status as i32,
            );
            self.tlm.fault_file_open = self.tlm.fault_file_open.wrapping_add(1);
            if self.txn(t).state == TxnState::R2 {
                self.r2_set_fin_txn_status(t, TxnStatus::FILESTORE_REJECTION);
            } else {
                self.r1_reset(mgr, t);
            }
        } else {
            self.txn_mut(t).state_data.receive.sub_state = RxSubState::FileData;
        }
    }

    /// C++ `r2SetFinTxnStatus`: record the status and send a FIN.
    pub fn r2_set_fin_txn_status(&mut self, t: TxnIdx, txn_stat: TxnStatus) {
        self.set_txn_status(t, txn_stat);
        self.txn_mut(t).flags.rx.send_fin = true;
    }

    /// C++ `r1Reset`: finish, keeping the history.
    pub fn r1_reset(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        self.finish_transaction(mgr, t, true);
    }

    /// C++ `r2Reset`: done when in close-out, after an EOF error, on any
    /// error or when canceled; otherwise send the FIN first.
    pub fn r2_reset(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let txn = self.txn(t);
        if txn.state_data.receive.sub_state == RxSubState::CloseoutSync
            || txn.state_data.receive.r2.eof_cc != ConditionCode::NO_ERROR.0
            || txn_status_is_error(self.hist(t).txn_stat)
            || txn.flags.com.canceled
        {
            self.r1_reset(mgr, t);
        } else {
            self.txn_mut(t).flags.rx.send_fin = true;
        }
    }

    /// C++ `rCheckCrc`: compare the running checksum with the expected one.
    pub fn r_check_crc(&mut self, mgr: &CfdpManager, t: TxnIdx, expected_crc: u32) -> Status {
        let crc_result = self.txn(t).crc.get_value();
        if crc_result != expected_crc {
            let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
            mgr.log_warning_lo_rx_crc_mismatch(
                self.txn(t).txn_class,
                src,
                seq,
                expected_crc,
                crc_result,
            );
            self.tlm.fault_crc_mismatch = self.tlm.fault_crc_mismatch.wrapping_add(1);
            return Status::Error;
        }
        Status::Success
    }

    /// C++ `r2Complete`: without metadata, or with a gap, NAK (when
    /// allowed; the NAK limit ends it with a FIN); with everything and the
    /// EOF, complete and send the FIN.
    pub fn r2_complete(&mut self, mgr: &CfdpManager, t: TxnIdx, ok_to_send_nak: bool) {
        if txn_status_is_error(self.hist(t).txn_stat) {
            return;
        }
        let mut send_nak = false;
        let mut send_fin = false;
        if !self.txn(t).flags.rx.md_recv {
            send_nak = true;
        } else {
            // One gap is enough to know there are gaps.
            let fsize = self.txn(t).fsize;
            if self.chunk_list(t).compute_gaps(1, fsize, 0, None) > 0 {
                send_nak = true;
            } else if self.txn(t).flags.rx.eof_recv {
                send_fin = true;
            }
        }
        if send_nak && ok_to_send_nak {
            let txn = self.txn_mut(t);
            txn.state_data.receive.r2.acknak_count =
                txn.state_data.receive.r2.acknak_count.wrapping_add(1);
            let nack_limit = mgr.get_nack_limit_param(self.txn(t).chan_num);
            if self.txn(t).state_data.receive.r2.acknak_count >= nack_limit {
                let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
                mgr.log_warning_lo_rx_nak_limit_reached(self.txn(t).txn_class, src, seq);
                send_fin = true;
                self.tlm.fault_nak_limit = self.tlm.fault_nak_limit.wrapping_add(1);
                self.set_txn_status(t, TxnStatus::NAK_LIMIT_REACHED);
                self.txn_mut(t).state_data.receive.r2.acknak_count = 0;
            } else {
                self.txn_mut(t).flags.rx.send_nak = true;
            }
        }
        if send_fin {
            // Latched: send_fin is cleared once the FIN goes out.
            self.txn_mut(t).flags.rx.complete = true;
            self.r2_set_fin_txn_status(t, TxnStatus::NO_ERROR);
        }
        self.txn_mut(t).state_data.receive.sub_state = RxSubState::FileData;
    }

    /// C++ `rProcessFd`: bounds-check and write one file data PDU (a
    /// zero-length segment is a successful no-op).
    fn r_process_fd(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) -> Status {
        let mut ret = Status::Success;
        let mut fd = FileDataPdu::new();
        let status = fd.deserialize_from(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_file_data_pdu_deserialization(
                self.txn(t).chan_num,
                status as i32,
            );
            ret = Status::Error;
        }
        let offset = fd.get_offset();
        let data_size = fd.get_data_size();
        let class = self.txn(t).txn_class;
        if ret == Status::Success {
            let bound = if self.txn(t).flags.rx.md_recv {
                self.txn(t).fsize
            } else {
                FileSize::MAX
            };
            if offset > bound || bound - offset < FileSize::from(data_size) {
                let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
                mgr.log_warning_lo_rx_file_data_out_of_bounds(
                    class,
                    src,
                    seq,
                    offset,
                    u32::from(data_size),
                    bound,
                );
                self.set_txn_status(t, TxnStatus::FILE_SIZE_ERROR);
                self.tlm.fault_file_size_mismatch =
                    self.tlm.fault_file_size_mismatch.wrapping_add(1);
                ret = Status::Error;
            }
        }
        if ret == Status::Success && data_size == 0 {
            return ret;
        }
        if ret == Status::Success && self.txn(t).state_data.receive.cached_pos != offset {
            let status = self.txn_mut(t).fd.seek_absolute(FwSizeType::from(offset));
            if status != FileStatus::OpOk {
                let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
                mgr.log_warning_lo_rx_seek_failed(class, src, seq, offset, status as i32);
                self.set_txn_status(t, TxnStatus::FILE_SIZE_ERROR);
                self.tlm.fault_file_seek = self.tlm.fault_file_seek.wrapping_add(1);
                ret = Status::Error;
            }
        }
        if ret == Status::Success {
            let mut write_size = FwSizeType::from(data_size);
            let status = self
                .txn_mut(t)
                .fd
                .write(fd.get_data(), &mut write_size, WaitType::Wait);
            if status != FileStatus::OpOk {
                let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
                mgr.log_warning_lo_rx_write_failed(
                    class,
                    src,
                    seq,
                    u32::from(data_size),
                    write_size as i32,
                );
                self.set_txn_status(t, TxnStatus::FILESTORE_REJECTION);
                self.tlm.fault_file_write = self.tlm.fault_file_write.wrapping_add(1);
                ret = Status::Error;
            } else {
                self.txn_mut(t).state_data.receive.cached_pos =
                    FileSize::from(data_size).wrapping_add(offset);
                self.tlm.recv_file_data_bytes = self
                    .tlm
                    .recv_file_data_bytes
                    .wrapping_add(u64::from(data_size));
            }
        }
        ret
    }

    /// C++ `rSubstateRecvEof`: decode the EOF; with metadata and no error
    /// the size must match; a non-zero condition code becomes the status
    /// (cancel is counted as normal, anything else as a fault).
    fn r_substate_recv_eof(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) -> Status {
        let mut eof = EofPdu::new();
        let status = eof.deserialize_from(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_eof_pdu_deserialization(self.txn(t).chan_num, status as i32);
            return Status::RecPduBadEofError;
        }
        let mut ret = Status::Success;
        if self.recv_eof(t, &eof) == Status::Success {
            let cc = eof.get_condition_code();
            let (src, seq, class) = (
                self.hist(t).src_eid,
                self.hist(t).seq_num,
                self.txn(t).txn_class,
            );
            if self.txn(t).flags.rx.md_recv
                && cc == ConditionCode::NO_ERROR
                && eof.get_file_size() != self.txn(t).fsize
            {
                mgr.log_warning_lo_rx_file_size_mismatch(
                    class,
                    src,
                    seq,
                    self.txn(t).fsize,
                    eof.get_file_size(),
                );
                self.tlm.fault_file_size_mismatch =
                    self.tlm.fault_file_size_mismatch.wrapping_add(1);
                ret = Status::RecPduFsizeMismatchError;
            }
            if cc != ConditionCode::NO_ERROR {
                self.set_txn_status(t, TxnStatus::from_condition_code(cc));
                if cc == ConditionCode::CANCEL_REQUEST_RECEIVED {
                    self.tlm.recv_eof_canceled = self.tlm.recv_eof_canceled.wrapping_add(1);
                    mgr.log_activity_hi_rx_eof_cancel_received(class, src, seq);
                } else {
                    self.tlm.fault_rx_eof_error = self.tlm.fault_rx_eof_error.wrapping_add(1);
                    mgr.log_warning_lo_rx_eof_with_error(class, src, seq, cc.0);
                }
            }
        }
        ret
    }

    /// C++ `r1SubstateRecvEof`: check the checksum (keep the file when it
    /// matches) and finish, whatever happened.
    fn r1_substate_recv_eof(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        let mut eof = EofPdu::new();
        let status = eof.deserialize_from(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_eof_pdu_deserialization(self.txn(t).chan_num, status as i32);
            self.r1_reset(mgr, t);
            return;
        }
        let ret = self.r_substate_recv_eof(mgr, t, data);
        if ret == Status::Success
            && eof.get_condition_code() == ConditionCode::NO_ERROR
            && self.r_check_crc(mgr, t, eof.get_checksum()) == Status::Success
        {
            self.txn_mut(t).keep = Keep::Keep;
        }
        // Class 1: the EOF will not come again, so always finish.
        self.r1_reset(mgr, t);
    }

    /// C++ `r2SubstateRecvEof`: on the first good EOF remember its checksum
    /// and size, ACK it, and either check for completion or (EOF with an
    /// error) close out.
    fn r2_substate_recv_eof(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        if self.txn(t).flags.rx.eof_recv {
            return;
        }
        let mut eof = EofPdu::new();
        let status = eof.deserialize_from(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_eof_pdu_deserialization(self.txn(t).chan_num, status as i32);
            self.txn_mut(t).state_data.receive.sub_state = RxSubState::FileData;
            return;
        }
        let ret = self.r_substate_recv_eof(mgr, t, data);
        if ret == Status::Success {
            let txn = self.txn_mut(t);
            txn.flags.rx.eof_recv = true;
            txn.state_data.receive.r2.eof_crc = eof.get_checksum();
            txn.state_data.receive.r2.eof_size = eof.get_file_size();
            // Always ACK the EOF (from the tick), even if not done.
            txn.state_data.receive.r2.eof_cc = eof.get_condition_code().0;
            txn.flags.rx.send_eof_ack = true;
            let eof_cc = txn.state_data.receive.r2.eof_cc;
            if eof_cc == ConditionCode::NO_ERROR.0 {
                self.r2_complete(mgr, t, true);
            } else {
                self.set_txn_status(t, TxnStatus(i32::from(eof_cc)));
                self.r2_reset(mgr, t);
            }
        } else if ret == Status::RecPduFsizeMismatchError {
            self.r2_set_fin_txn_status(t, TxnStatus::FILE_SIZE_ERROR);
        } else {
            self.txn_mut(t).state_data.receive.sub_state = RxSubState::FileData;
        }
    }

    /// C++ `r1SubstateRecvFileData`: write and checksum; any failure ends
    /// the transaction.
    fn r1_substate_recv_file_data(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        let mut fd = FileDataPdu::new();
        let status = fd.deserialize_from(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_file_data_pdu_deserialization(
                self.txn(t).chan_num,
                status as i32,
            );
            self.r1_reset(mgr, t);
            return;
        }
        let mut ret = self.recv_fd(mgr, t, &fd);
        if ret == Status::Success {
            ret = self.r_process_fd(mgr, t, data);
        }
        if ret == Status::Success {
            self.txn_mut(t).crc.update(fd.get_data(), fd.get_offset());
        } else {
            self.r1_reset(mgr, t);
        }
    }

    /// C++ `r2SubstateRecvFileData`: write and record the chunk (ignored
    /// once the checksum pass has started); after a NAK, every segment
    /// re-checks completion.
    fn r2_substate_recv_file_data(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        if self.txn(t).state_data.receive.r2.rx_crc_calc_bytes > 0 {
            // Late retransmissions after the file was reopened for the
            // checksum are expected in class 2: ignore them.
            return;
        }
        let mut fd = FileDataPdu::new();
        let status = fd.deserialize_from(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_file_data_pdu_deserialization(
                self.txn(t).chan_num,
                status as i32,
            );
            self.r2_reset(mgr, t);
            return;
        }
        let mut ret = self.recv_fd(mgr, t, &fd);
        if ret == Status::Success {
            ret = self.r_process_fd(mgr, t, data);
        }
        if ret == Status::Success {
            self.chunk_list_mut(t)
                .add(fd.get_offset(), FileSize::from(fd.get_data_size()));
            if self.txn(t).flags.rx.fd_nak_sent {
                self.r2_complete(mgr, t, false);
            }
            if !self.txn(t).flags.rx.complete {
                self.arm_ack_timer(mgr, t);
            }
            self.txn_mut(t).state_data.receive.r2.acknak_count = 0;
        } else {
            self.r2_reset(mgr, t);
        }
    }

    /// C++ `rSubstateSendNak`: with metadata, NAK the gaps (or mark
    /// complete when there are none); without, NAK `[0, 0]` for the
    /// metadata.
    fn r_substate_send_nak(&mut self, mgr: &CfdpManager, t: TxnIdx) -> Status {
        let mut nak = NakPdu::new();
        let local_eid = mgr.get_local_eid_param();
        let (peer, seq, class) = (
            self.hist(t).peer_eid,
            self.hist(t).seq_num,
            self.txn(t).txn_class,
        );
        nak.initialize(
            PduDirection::TowardSender,
            class,
            peer,
            seq,
            local_eid,
            0,
            0,
        );
        if self.txn(t).flags.rx.md_recv {
            let list = self.chunk_list(t);
            let chunk_count = u32::from(list.get_count());
            let max_chunks = u32::from(list.get_max_chunks());
            let gap_limit = if chunk_count < max_chunks {
                max_chunks
            } else {
                max_chunks - 1
            };
            let fsize = self.txn(t).fsize;
            let scope_start = nak.get_scope_start();
            let mut add_gap = |chunk: &Chunk| {
                // C++ `r2GapCompute`: segments relative to the scope start.
                fw_assert!(chunk.size > 0, chunk.size as i32);
                let offset_start = chunk.offset.wrapping_sub(scope_start);
                let offset_end = offset_start.wrapping_add(chunk.size);
                nak.add_segment(offset_start, offset_end);
            };
            let gap_count = list.compute_gaps(gap_limit as u16, fsize, 0, Some(&mut add_gap));
            if gap_count == 0 {
                self.txn_mut(t).flags.rx.complete = true;
                Status::Success
            } else {
                let status = self.send_nak(mgr, t, &nak);
                if status == Status::Success {
                    self.txn_mut(t).flags.rx.fd_nak_sent = true;
                    self.tlm.sent_nak_segment_requests =
                        self.tlm.sent_nak_segment_requests.wrapping_add(gap_count);
                }
                status
            }
        } else {
            nak.add_segment(0, 0);
            self.send_nak(mgr, t, &nak)
        }
    }

    /// C++ `r2CalcCrcChunk`: checksum up to `RxCrcCalcBytesPerCycle` bytes
    /// of the received file (reopened for reading on the first call) and,
    /// at the end, compare with the EOF checksum. `Error` means "not done
    /// yet" as well as a real failure (C++ parity).
    fn r2_calc_crc_chunk(&mut self, mgr: &CfdpManager, t: TxnIdx) -> Status {
        let mut buf = [0u8; R2_CRC_CHUNK_SIZE];
        let mut count_bytes: FileSize = 0;
        let mut ret = Status::Success;
        let class = self.txn(t).txn_class;
        if self.txn(t).state_data.receive.r2.rx_crc_calc_bytes == 0 {
            self.txn_mut(t).crc = Checksum::from_value(0);
            // The file was opened for writing; reopen it for reading.
            if self.txn(t).fd.is_open() {
                self.txn_mut(t).fd.close();
            }
            let path = path_str(self.hist(t).fnames.dst_filename.as_bytes()).into_owned();
            if self.txn_mut(t).fd.open(&path, FileMode::OpenRead) != FileStatus::OpOk {
                self.set_txn_status(t, TxnStatus::FILE_SIZE_ERROR);
                ret = Status::Error;
            } else {
                self.txn_mut(t).state_data.receive.cached_pos = 0;
            }
        }
        if ret == Status::Success {
            let per_cycle = mgr.get_rx_crc_calc_bytes_per_cycle_param();
            while ret == Status::Success
                && count_bytes < per_cycle
                && self.txn(t).state_data.receive.r2.rx_crc_calc_bytes < self.txn(t).fsize
            {
                let calc = self.txn(t).state_data.receive.r2.rx_crc_calc_bytes;
                let fsize = self.txn(t).fsize;
                let want_offs_size = calc.wrapping_add(buf.len() as FileSize);
                let mut read_size: FwSizeType = if want_offs_size > fsize {
                    FwSizeType::from(fsize - calc)
                } else {
                    buf.len() as FwSizeType
                };
                if self.txn(t).state_data.receive.cached_pos != calc {
                    let status = self.txn_mut(t).fd.seek_absolute(FwSizeType::from(calc));
                    if status != FileStatus::OpOk {
                        let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
                        mgr.log_warning_lo_rx_seek_crc_failed(class, src, seq, calc, status as i32);
                        self.tlm.fault_file_seek = self.tlm.fault_file_seek.wrapping_add(1);
                        ret = Status::Error;
                    }
                }
                if ret == Status::Success {
                    let expected_read_size = read_size;
                    let status = self.txn_mut(t).fd.read(
                        &mut buf[..read_size as usize],
                        &mut read_size,
                        WaitType::Wait,
                    );
                    if status != FileStatus::OpOk {
                        let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
                        mgr.log_warning_lo_rx_read_crc_failed(
                            class,
                            src,
                            seq,
                            expected_read_size as u32,
                            read_size as i32,
                        );
                        self.set_txn_status(t, TxnStatus::FILE_SIZE_ERROR);
                        self.tlm.fault_file_read = self.tlm.fault_file_read.wrapping_add(1);
                        ret = Status::Error;
                    } else {
                        let txn = self.txn_mut(t);
                        txn.crc.update(&buf[..read_size as usize], calc);
                        txn.state_data.receive.r2.rx_crc_calc_bytes =
                            calc.wrapping_add(read_size as FileSize);
                        txn.state_data.receive.cached_pos =
                            txn.state_data.receive.r2.rx_crc_calc_bytes;
                        count_bytes = count_bytes.wrapping_add(read_size as FileSize);
                        // Actively processing: keep the transaction alive.
                        self.arm_inact_timer(mgr, t);
                    }
                }
            }
        }
        if ret == Status::Success {
            if self.txn(t).state_data.receive.r2.rx_crc_calc_bytes == self.txn(t).fsize {
                let eof_crc = self.txn(t).state_data.receive.r2.eof_crc;
                if self.r_check_crc(mgr, t, eof_crc) == Status::Success {
                    let txn = self.txn_mut(t);
                    txn.keep = Keep::Keep;
                    txn.state_data.receive.r2.dc = FinDeliveryCode::Complete;
                    txn.state_data.receive.r2.fs = FinFileStatus::Retained;
                } else {
                    self.r2_set_fin_txn_status(t, TxnStatus::FILE_CHECKSUM_FAILURE);
                }
                self.txn_mut(t).flags.com.crc_calc = true;
            } else {
                // More to do next tick.
                ret = Status::Error;
            }
        }
        ret
    }

    /// C++ `r2SubstateSendFin`: finish the checksum first (when no error),
    /// then send the FIN and go to close-out (even if the send failed).
    fn r2_substate_send_fin(&mut self, mgr: &CfdpManager, t: TxnIdx) -> Status {
        let mut ret = Status::Success;
        if !txn_status_is_error(self.hist(t).txn_stat)
            && !self.txn(t).flags.com.crc_calc
            && self.r2_calc_crc_chunk(mgr, t) != Status::Success
        {
            // Re-enter next tick.
            ret = Status::Error;
        }
        if ret != Status::Error {
            let r2 = self.txn(t).state_data.receive.r2;
            let cc = txn_status_to_condition_code(self.hist(t).txn_stat);
            let sret = self.send_fin(mgr, t, r2.dc, r2.fs, cc);
            self.txn_mut(t).state_data.receive.sub_state = RxSubState::CloseoutSync;
            if sret != Status::Success {
                ret = Status::Error;
            }
        }
        ret
    }

    /// C++ `r2RecvFinAck`: the FIN-ACK closes the transaction.
    fn r2_recv_fin_ack(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        let mut ack = AckPdu::new();
        let status = ack.deserialize_from(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_ack_pdu_deserialization(self.txn(t).chan_num, status as i32);
            self.tlm.recv_errors = self.tlm.recv_errors.wrapping_add(1);
            return;
        }
        self.r2_reset(mgr, t);
    }

    /// C++ `r2RecvMd`: metadata after file data (R2 without metadata):
    /// check it against an EOF already received, rename the temporary file
    /// to the real name, reopen it and check for completion.
    fn r2_recv_md(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        if self.txn(t).flags.rx.md_recv {
            return;
        }
        let fname = self.hist(t).fnames.dst_filename.clone();
        let mut md = MetadataPdu::new();
        let status = md.deserialize_from(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_metadata_pdu_deserialization(
                self.txn(t).chan_num,
                status as i32,
            );
            return;
        }
        self.recv_md(mgr, t, &md);
        let class = self.txn(t).txn_class;
        let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
        let mut success = true;
        if self.txn(t).flags.rx.eof_recv
            && self.txn(t).state_data.receive.r2.eof_size != self.txn(t).fsize
        {
            mgr.log_warning_lo_rx_eof_md_size_mismatch(
                class,
                src,
                seq,
                self.txn(t).fsize,
                self.txn(t).state_data.receive.r2.eof_size,
            );
            self.tlm.fault_file_size_mismatch = self.tlm.fault_file_size_mismatch.wrapping_add(1);
            self.r2_set_fin_txn_status(t, TxnStatus::FILE_SIZE_ERROR);
            success = false;
        }
        if success {
            self.txn_mut(t).fd.close();
            let dst = self.hist(t).fnames.dst_filename.clone();
            let fs_status =
                filesystem::move_file(&path_str(fname.as_bytes()), &path_str(dst.as_bytes()));
            if fs_status != FsStatus::OpOk {
                mgr.log_warning_lo_rx_file_rename_failed(
                    class,
                    src,
                    seq,
                    fname.as_bytes(),
                    dst.as_bytes(),
                    fs_status as i32,
                );
                self.r2_set_fin_txn_status(t, TxnStatus::FILESTORE_REJECTION);
                self.tlm.fault_file_rename = self.tlm.fault_file_rename.wrapping_add(1);
                success = false;
            } else {
                let status = self
                    .txn_mut(t)
                    .fd
                    .open(&path_str(dst.as_bytes()), FileMode::OpenWrite);
                if status != FileStatus::OpOk {
                    mgr.log_warning_lo_rx_file_reopen_failed(
                        class,
                        src,
                        seq,
                        dst.as_bytes(),
                        status as i32,
                    );
                    self.r2_set_fin_txn_status(t, TxnStatus::FILESTORE_REJECTION);
                    self.tlm.fault_file_open = self.tlm.fault_file_open.wrapping_add(1);
                    success = false;
                }
            }
            if success {
                let txn = self.txn_mut(t);
                txn.state_data.receive.cached_pos = 0;
                txn.flags.rx.md_recv = true;
                txn.state_data.receive.r2.acknak_count = 0;
                self.r2_complete(mgr, t, true);
            }
        }
    }

    /// C++ `rSendInactivityEvent`.
    fn r_send_inactivity_event(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
        mgr.log_warning_lo_rx_inactivity_timeout(self.txn(t).txn_class, src, seq);
        self.tlm.fault_inactivity_timer = self.tlm.fault_inactivity_timer.wrapping_add(1);
    }

    /// C++ `rDispatchRecv`: file data goes to the class's handler unless the
    /// transaction has failed; a directive selects from the sub-state's
    /// table; an out-of-range directive code is spurious; anything without
    /// a handler is dropped.
    fn r_dispatch_recv(
        &mut self,
        mgr: &CfdpManager,
        t: TxnIdx,
        data: &[u8],
        class1: bool,
        table: impl Fn(RxSubState, FileDirective) -> Option<RRecv>,
    ) {
        let sub_state = self.txn(t).state_data.receive.sub_state;
        fw_assert!(
            sub_state < RxSubState::NumStates,
            sub_state as i32,
            RxSubState::NumStates as i32
        );
        enum Selected {
            FileData,
            Directive(RRecv),
        }
        let mut selected = None;
        if peek_pdu_type(data) == PduTypeEnum::FileData {
            if !txn_status_is_error(self.hist(t).txn_stat) {
                selected = Some(Selected::FileData);
            }
        } else {
            let mut reader = PduReader::new(data);
            let mut header = PduHeader::default();
            let mut directive_code = 0u8;
            if header.from_serial_buffer(&mut reader) == SerializeStatus::Ok
                && reader.read_u8(&mut directive_code) == SerializeStatus::Ok
            {
                let directive = FileDirective(directive_code);
                if directive < FileDirective::INVALID_MAX {
                    selected = table(sub_state, directive).map(Selected::Directive);
                } else {
                    self.tlm.recv_spurious = self.tlm.recv_spurious.wrapping_add(1);
                    let (src, seq) = (self.hist(t).src_eid, self.hist(t).seq_num);
                    mgr.log_warning_lo_rx_invalid_directive_code(
                        self.txn(t).txn_class,
                        src,
                        seq,
                        directive_code,
                        sub_state as u8,
                    );
                }
            }
        }
        match selected {
            Some(Selected::FileData) if class1 => self.r1_substate_recv_file_data(mgr, t, data),
            Some(Selected::FileData) => self.r2_substate_recv_file_data(mgr, t, data),
            Some(Selected::Directive(RRecv::R1Eof)) => self.r1_substate_recv_eof(mgr, t, data),
            Some(Selected::Directive(RRecv::R2Eof)) => self.r2_substate_recv_eof(mgr, t, data),
            Some(Selected::Directive(RRecv::R2Md)) => self.r2_recv_md(mgr, t, data),
            Some(Selected::Directive(RRecv::R2FinAck)) => self.r2_recv_fin_ack(mgr, t, data),
            None => self.tlm.recv_dropped = self.tlm.recv_dropped.wrapping_add(1),
        }
    }
}
