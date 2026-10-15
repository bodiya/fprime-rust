//! The CFDP engine — port of `Svc/Ccsds/CfdpManager/Engine.{hpp,cpp}`,
//! derived from NASA CF 3.0.0 `cf_cfdp.{h,c}`.
//!
//! Copyright (c) 2019 United States Government as represented by the
//! Administrator of the National Aeronautics and Space Administration.
//! All Rights Reserved. NASA Docket No. GSC-18,447-1. Licensed under the
//! Apache License, Version 2.0.
//!
//! [`Engine`] owns the channels and the transaction sequence counter and
//! carries the engine-wide operations (cycle, PDU receive routing, file and
//! directory requests, the transaction commands). The C++ `Engine` methods
//! that act on one transaction (`finishTransaction`, the PDU send and
//! receive helpers, the timers, ...) are `Channel` methods here, in the
//! second `impl` block of this file: a transaction is an index into its
//! channel's pool, so they operate on the channel that owns it. The
//! manager (events, parameters, ports) is passed in as `mgr`.

use super::CfdpManager;
use super::PACKET_DESCRIPTOR_SIZE;
use super::channel::Channel;
use super::pdu::{
    AckPdu, EofPdu, FileDataPdu, FinPdu, MetadataPdu, NakPdu, Pdu, PduDirection, PduHeader,
    PduReader, Tlv, TlvType, peek_pdu_type,
};
use super::transaction::{PbRef, TxnIdx};
use super::types::{
    AckTxnStatus, ChecksumType, Class, ConditionCode, Direction, EntityId, FileDirective,
    FinDeliveryCode, FinFileStatus, Flow, Keep, PduTypeEnum, QueueId, Status, SuspendResume,
    TransactionInitType, TransactionSeq, TxnState, TxnStatus,
};
use super::utils::{get_txn_status, txn_status_is_error, txn_status_to_condition_code};
use crate::file_downlink::SendFileStatus;
use fprime_config::cfdp::{
    MAX_COMMANDED_PLAYBACK_DIRECTORIES_PER_CHAN, MAX_COMMANDED_PLAYBACK_FILES_PER_CHAN,
    MAX_POLLING_DIR_PER_CHAN, NUM_CHANNELS,
};
use fprime_fw::{Enabled, ExtBuf, SerBufAny, SerializeStatus, fw_assert};
use fprime_os::directory::{OpenMode as DirOpenMode, Status as DirStatus};
use fprime_os::filesystem::{self, Status as FsStatus};
use std::borrow::Cow;

/// A byte path as `&str` for the OS layer (lossless for UTF-8 paths).
pub(crate) fn path_str(bytes: &[u8]) -> Cow<'_, str> {
    String::from_utf8_lossy(bytes)
}

/// C++ `Engine`.
pub struct Engine {
    /// The channels (C++ `m_channels`).
    pub(crate) channels: Box<[Channel]>,
    /// The last transaction sequence number assigned (C++ `m_seqNum`).
    pub(crate) seq_num: TransactionSeq,
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

impl Engine {
    /// C++ constructor + `init`: allocate every channel's pools.
    #[must_use]
    pub fn new() -> Self {
        Self {
            channels: (0..NUM_CHANNELS).map(|i| Channel::new(i as u8)).collect(),
            seq_num: 0,
        }
    }

    fn chan(&mut self, chan_id: u8) -> &mut Channel {
        fw_assert!(
            usize::from(chan_id) < NUM_CHANNELS,
            i32::from(chan_id),
            NUM_CHANNELS as i32
        );
        &mut self.channels[usize::from(chan_id)]
    }

    /// C++ `cycle`: per channel, reset the PDU budget and, unless frozen,
    /// tick the transactions, run transmit, and service playbacks and polls.
    pub fn cycle(&mut self, mgr: &CfdpManager) {
        let seq_num = &mut self.seq_num;
        for chan in self.channels.iter_mut() {
            chan.outgoing_counter = 0;
            if chan.flow_state == Flow::NotFrozen {
                // Ticks first: ACK/NAK responses outrank new file data for
                // the cycle's limited PDUs.
                chan.tick_transactions(mgr);
                chan.cycle_tx(mgr);
                chan.process_playback_directories(mgr, seq_num);
                chan.process_polling_directories(mgr, seq_num);
            }
        }
    }

    /// C++ `receivePdu`: route a received PDU to its transaction (matched by
    /// sequence number and source entity), acknowledge a FIN for a recycled
    /// transaction this entity sourced, or start a receive transaction when
    /// this entity is the destination.
    pub fn receive_pdu(&mut self, mgr: &CfdpManager, chan_id: u8, data: &[u8]) {
        let local_eid = mgr.get_local_eid_param();
        let chan = self.chan(chan_id);
        let mut reader = PduReader::new(data);
        let mut header = PduHeader::default();
        let status = header.from_serial_buffer(&mut reader);
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_pdu_header_deserialization(chan_id, status as i32);
            return;
        }
        chan.tlm.recv_pdu = chan.tlm.recv_pdu.wrapping_add(1);
        let transaction_seq = header.get_transaction_seq();
        let source_eid = header.get_source_eid();
        let dest_eid = header.get_dest_eid();
        let mut txn = chan.find_transaction_by_sequence_number(transaction_seq, source_eid);
        if txn.is_none() {
            if peek_pdu_type(data) == PduTypeEnum::Finished && source_eid == local_eid {
                // A retransmitted FIN for a transaction this entity sourced
                // that has been recycled: acknowledge it statelessly.
                let mut fin = FinPdu::new();
                if fin.deserialize_from(&mut PduReader::new(data)) == SerializeStatus::Ok {
                    let _ = chan.send_fin_ack_stateless(
                        mgr,
                        transaction_seq,
                        source_eid,
                        dest_eid,
                        fin.get_condition_code(),
                    );
                    mgr.log_diagnostic_tx_late_fin_acked(source_eid, transaction_seq);
                } else {
                    // C++ parity: the event reports FORMAT_ERROR whatever
                    // the decode status was.
                    mgr.log_warning_lo_fail_fin_pdu_deserialization(
                        chan_id,
                        SerializeStatus::DeserFormatError as i32,
                    );
                }
            } else if dest_eid == local_eid {
                // Transmit transactions are only commanded, so this starts
                // a receive transaction.
                txn = chan.start_rx_transaction();
                if txn.is_none() {
                    mgr.log_warning_lo_rx_transaction_limit_reached(source_eid, transaction_seq);
                }
            } else {
                mgr.log_warning_lo_invalid_destination_eid(dest_eid);
            }
        }
        if let Some(t) = txn {
            chan.dispatch_recv(mgr, t, data);
        }
    }

    /// C++ `txFile`: queue a transmit transaction on `chan_num` (commanded
    /// transmits are limited to `MaxCommandedPlaybackFilesPerChan`).
    #[allow(clippy::too_many_arguments)]
    pub fn tx_file(
        &mut self,
        mgr: &CfdpManager,
        src_filename: &[u8],
        dst_filename: &[u8],
        cfdp_class: Class,
        keep: Keep,
        chan_num: u8,
        priority: u8,
        dest_id: EntityId,
        init_type: TransactionInitType,
    ) -> Status {
        let seq_num = &mut self.seq_num;
        fw_assert!(
            usize::from(chan_num) < NUM_CHANNELS,
            i32::from(chan_num),
            NUM_CHANNELS as i32
        );
        let chan = &mut self.channels[usize::from(chan_num)];
        let txn = if (chan.num_cmd_tx as usize) < MAX_COMMANDED_PLAYBACK_FILES_PER_CHAN {
            chan.find_unused_transaction(Direction::Tx)
        } else {
            None
        };
        let Some(t) = txn else {
            mgr.log_warning_lo_max_tx_transactions_reached();
            return Status::Error;
        };
        let hist = chan.hist_mut(t);
        hist.fnames.src_filename.set_bytes(src_filename);
        hist.fnames.dst_filename.set_bytes(dst_filename);
        chan.tx_file_initiate(
            mgr, t, cfdp_class, keep, chan_num, priority, dest_id, seq_num,
        );
        chan.num_cmd_tx += 1;
        chan.txn_mut(t).flags.tx.cmd_tx = true;
        chan.txn_mut(t).init_type = init_type;
        let hist = chan.hist(t);
        mgr.log_activity_lo_tx_file_queued(hist.fnames.src_filename.as_bytes(), hist.seq_num);
        Status::Success
    }

    /// C++ `playbackDir`: start a playback of a directory in the first free
    /// commanded-playback slot of `chan`.
    #[allow(clippy::too_many_arguments)]
    pub fn playback_dir(
        &mut self,
        mgr: &CfdpManager,
        src: &[u8],
        dst: &[u8],
        cfdp_class: Class,
        keep: Keep,
        chan: u8,
        priority: u8,
        dest_id: EntityId,
    ) -> Status {
        let channel = self.chan(chan);
        let slot = (0..MAX_COMMANDED_PLAYBACK_DIRECTORIES_PER_CHAN)
            .find(|&i| !channel.get_playback(i).busy);
        match slot {
            None => {
                mgr.log_warning_lo_playback_dir_slot_unavailable();
                Status::Error
            }
            Some(i) => channel.playback_dir_initiate(
                mgr,
                PbRef::Playback(i as u8),
                src,
                dst,
                cfdp_class,
                keep,
                priority,
                dest_id,
            ),
        }
    }

    /// C++ `startPollDir`: arm polling slot `poll_id` of `chan_id` (busy if
    /// already enabled).
    #[allow(clippy::too_many_arguments)]
    pub fn start_poll_dir(
        &mut self,
        mgr: &CfdpManager,
        chan_id: u8,
        poll_id: u8,
        src_dir: &[u8],
        dst_dir: &[u8],
        cfdp_class: Class,
        priority: u8,
        dest_eid: EntityId,
        interval_sec: u32,
    ) -> Status {
        fw_assert!(
            usize::from(poll_id) < MAX_POLLING_DIR_PER_CHAN,
            i32::from(poll_id),
            MAX_POLLING_DIR_PER_CHAN as i32
        );
        let pd = self.chan(chan_id).get_poll_dir(usize::from(poll_id));
        if pd.enabled == Enabled::Disabled {
            pd.interval_sec = interval_sec;
            pd.priority = priority;
            pd.cfdp_class = cfdp_class;
            pd.dest_eid = dest_eid;
            pd.src_dir.set_bytes(src_dir);
            pd.dst_dir.set_bytes(dst_dir);
            pd.interval_timer.set_timer(pd.interval_sec);
            pd.enabled = Enabled::Enabled;
            Status::Success
        } else {
            mgr.log_warning_lo_poll_dir_busy(chan_id, poll_id);
            Status::Error
        }
    }

    /// C++ `stopPollDir`: clear and disable polling slot `poll_id`.
    pub fn stop_poll_dir(&mut self, mgr: &CfdpManager, chan_id: u8, poll_id: u8) -> Status {
        fw_assert!(
            usize::from(poll_id) < MAX_POLLING_DIR_PER_CHAN,
            i32::from(poll_id),
            MAX_POLLING_DIR_PER_CHAN as i32
        );
        let pd = self.chan(chan_id).get_poll_dir(usize::from(poll_id));
        if pd.enabled == Enabled::Enabled {
            pd.interval_sec = 0;
            pd.priority = 0;
            pd.cfdp_class = Class::Class2;
            pd.dest_eid = 0;
            pd.src_dir.clear();
            pd.dst_dir.clear();
            pd.interval_timer.disable_timer();
            pd.enabled = Enabled::Disabled;
            Status::Success
        } else {
            mgr.log_warning_lo_poll_dir_not_active(chan_id, poll_id);
            Status::Error
        }
    }

    /// C++ `setChannelFlowState`.
    pub fn set_channel_flow_state(&mut self, channel_id: u8, flow_state: Flow) {
        self.chan(channel_id).set_flow_state(flow_state);
    }

    /// C++ `setSuspendResumeTransaction`.
    pub fn set_suspend_resume_transaction(
        &mut self,
        channel_id: u8,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
        action: SuspendResume,
    ) -> Status {
        let chan = self.chan(channel_id);
        match chan.find_transaction_by_sequence_number(transaction_seq, entity_id) {
            Some(t) => {
                chan.txn_mut(t).flags.com.suspended = action == SuspendResume::Suspend;
                Status::Success
            }
            None => Status::Error,
        }
    }

    /// C++ `cancelTransactionBySeq`.
    pub fn cancel_transaction_by_seq(
        &mut self,
        mgr: &CfdpManager,
        channel_id: u8,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
    ) -> Status {
        let chan = self.chan(channel_id);
        match chan.find_transaction_by_sequence_number(transaction_seq, entity_id) {
            Some(t) => {
                chan.cancel_transaction(mgr, t);
                Status::Success
            }
            None => Status::Error,
        }
    }

    /// C++ `abandonTransaction`: finish without close-out.
    pub fn abandon_transaction(
        &mut self,
        mgr: &CfdpManager,
        channel_id: u8,
        transaction_seq: TransactionSeq,
        entity_id: EntityId,
    ) -> Status {
        let chan = self.chan(channel_id);
        match chan.find_transaction_by_sequence_number(transaction_seq, entity_id) {
            Some(t) => {
                chan.finish_transaction(mgr, t, false);
                Status::Success
            }
            None => Status::Error,
        }
    }
}

// ---------------------------------------------------------------------------
// The C++ `Engine` members that act on one transaction.
// ---------------------------------------------------------------------------

impl Channel {
    /// C++ `Engine::armAckTimer`.
    pub fn arm_ack_timer(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let duration = mgr.get_ack_timer_param(self.txn(t).chan_num);
        let txn = self.txn_mut(t);
        txn.ack_timer.set_timer(duration);
        txn.flags.com.ack_timer_armed = true;
    }

    /// C++ `Engine::armInactTimer`: the inactivity timer while active, else
    /// twice the ACK timer (long enough to catch late retransmissions).
    pub fn arm_inact_timer(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let chan_num = self.txn(t).chan_num;
        let duration = if get_txn_status(Some(self.txn(t).state)) == AckTxnStatus::ACTIVE {
            mgr.get_inactivity_timer_param(chan_num)
        } else {
            mgr.get_ack_timer_param(chan_num).wrapping_mul(2)
        };
        self.txn_mut(t).inactivity_timer.set_timer(duration);
    }

    /// C++ `Engine::dispatchRecv`: hand a received PDU to the transaction's
    /// state handler (re-dispatching once when an idle transaction becomes
    /// R2), then re-arm its inactivity timer.
    pub fn dispatch_recv(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        let mut needs_dispatch = true;
        while needs_dispatch {
            needs_dispatch = false;
            match self.txn(t).state {
                TxnState::Init => needs_dispatch = self.recv_init(mgr, t, data),
                TxnState::R1 => self.r1_recv(mgr, t, data),
                TxnState::S1 => self.s1_recv(mgr, t, data),
                TxnState::R2 => self.r2_recv(mgr, t, data),
                TxnState::S2 => self.s2_recv(mgr, t, data),
                TxnState::Drop => self.recv_drop(t),
                TxnState::Hold => self.recv_hold(mgr, t, data),
                _ => {}
            }
        }
        self.arm_inact_timer(mgr, t);
    }

    /// C++ `Engine::dispatchTx` / `Transaction::txStateDispatch`.
    pub fn dispatch_tx(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let state = self.txn(t).state;
        fw_assert!(state < TxnState::Invalid, state as i32);
        match state {
            TxnState::S1 => self.s1_tx(mgr, t),
            TxnState::S2 => self.s2_tx(mgr, t),
            _ => {}
        }
    }

    /// C++ `Engine::serializeAndSendPdu(OnChannel)`: allocate a buffer with
    /// room for the packet descriptor, encode the PDU after it and send it.
    pub fn serialize_and_send_pdu(&mut self, mgr: &CfdpManager, pdu: &dyn Pdu) -> Status {
        let chan_id = self.channel_id;
        let buffer_size = pdu.get_buffer_size() as usize + PACKET_DESCRIPTOR_SIZE;
        let (status, mut buffer) =
            mgr.get_pdu_buffer(chan_id, &mut self.outgoing_counter, buffer_size);
        if status != Status::Success {
            return status;
        }
        let (ser_status, len) = {
            let data = buffer.data_mut();
            let mut sb = ExtBuf::new(&mut data[PACKET_DESCRIPTOR_SIZE..]);
            let st = pdu.serialize_to(&mut sb);
            (st, sb.ser_loc())
        };
        if ser_status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_pdu_serialization(chan_id, pdu.get_type(), ser_status as i32);
            mgr.return_pdu_buffer(chan_id, buffer);
            return Status::Error;
        }
        buffer.set_size(len + PACKET_DESCRIPTOR_SIZE);
        mgr.send_pdu_buffer(chan_id, buffer);
        self.tlm.sent_pdu = self.tlm.sent_pdu.wrapping_add(1);
        Status::Success
    }

    /// C++ `Engine::sendMd`.
    pub fn send_md(&mut self, mgr: &CfdpManager, t: TxnIdx) -> Status {
        let state = self.txn(t).state;
        fw_assert!(state == TxnState::S1 || state == TxnState::S2, state as i32);
        let closure_requested = u8::from(state == TxnState::S2);
        let mut md = MetadataPdu::new();
        let hist = self.hist(t);
        md.initialize(
            PduDirection::TowardReceiver,
            self.txn(t).txn_class,
            mgr.get_local_eid_param(),
            hist.seq_num,
            hist.peer_eid,
            self.txn(t).fsize,
            hist.fnames.src_filename.as_bytes(),
            hist.fnames.dst_filename.as_bytes(),
            ChecksumType::MODULAR,
            closure_requested,
        );
        self.serialize_and_send_pdu(mgr, &md)
    }

    /// C++ `Engine::sendFd`.
    pub fn send_fd(&mut self, mgr: &CfdpManager, fd_pdu: &FileDataPdu<'_>) -> Status {
        let status = self.serialize_and_send_pdu(mgr, fd_pdu);
        if status == Status::Success {
            self.tlm.sent_file_data_bytes = self
                .tlm
                .sent_file_data_bytes
                .wrapping_add(u64::from(fd_pdu.get_data_size()));
        }
        status
    }

    /// C++ `Engine::sendEof`: the condition code from the transaction
    /// status, the running checksum and the file size; an entity-ID TLV on
    /// error.
    pub fn send_eof(&mut self, mgr: &CfdpManager, t: TxnIdx) -> Status {
        let condition_code = txn_status_to_condition_code(self.hist(t).txn_stat);
        if condition_code == ConditionCode::CANCEL_REQUEST_RECEIVED {
            self.tlm.sent_eof_canceled = self.tlm.sent_eof_canceled.wrapping_add(1);
        } else if condition_code != ConditionCode::NO_ERROR {
            self.tlm.fault_tx_eof_error = self.tlm.fault_tx_eof_error.wrapping_add(1);
        }
        let local_eid = mgr.get_local_eid_param();
        let mut eof = EofPdu::new();
        let hist = self.hist(t);
        eof.initialize(
            PduDirection::TowardReceiver,
            self.txn(t).txn_class,
            local_eid,
            hist.seq_num,
            hist.peer_eid,
            condition_code,
            self.txn(t).crc.get_value(),
            self.txn(t).fsize,
        );
        if condition_code != ConditionCode::NO_ERROR {
            let mut tlv = Tlv::new();
            tlv.initialize_entity_id(local_eid);
            eof.append_tlv(&tlv);
        }
        self.serialize_and_send_pdu(mgr, &eof)
    }

    /// C++ `Engine::sendAck`: acknowledge an EOF (toward the sender) or a
    /// FIN (toward the receiver). The local entity is the source for a
    /// transmit transaction, the destination for a receive one.
    #[allow(clippy::too_many_arguments)]
    pub fn send_ack(
        &mut self,
        mgr: &CfdpManager,
        t: TxnIdx,
        ts: AckTxnStatus,
        dir_code: FileDirective,
        cc: ConditionCode,
        peer_eid: EntityId,
        tsn: TransactionSeq,
    ) -> Status {
        fw_assert!(
            dir_code == FileDirective::END_OF_FILE || dir_code == FileDirective::FIN,
            i32::from(dir_code.0)
        );
        let local_eid = mgr.get_local_eid_param();
        let (src_eid, dst_eid) = if self.hist(t).dir == Direction::Tx {
            (local_eid, peer_eid)
        } else {
            (peer_eid, local_eid)
        };
        let direction = if dir_code == FileDirective::END_OF_FILE {
            PduDirection::TowardSender
        } else {
            PduDirection::TowardReceiver
        };
        let mut ack = AckPdu::new();
        ack.initialize(
            direction,
            self.txn(t).txn_class,
            src_eid,
            tsn,
            dst_eid,
            dir_code,
            1,
            cc,
            ts,
        );
        self.serialize_and_send_pdu(mgr, &ack)
    }

    /// C++ `Engine::sendFin`: an entity-ID TLV on error.
    pub fn send_fin(
        &mut self,
        mgr: &CfdpManager,
        t: TxnIdx,
        dc: FinDeliveryCode,
        fs: FinFileStatus,
        cc: ConditionCode,
    ) -> Status {
        let local_eid = mgr.get_local_eid_param();
        let mut fin = FinPdu::new();
        let hist = self.hist(t);
        fin.initialize(
            PduDirection::TowardSender,
            self.txn(t).txn_class,
            hist.peer_eid,
            hist.seq_num,
            local_eid,
            cc,
            dc,
            fs,
        );
        if cc != ConditionCode::NO_ERROR {
            let mut tlv = Tlv::new();
            tlv.initialize_entity_id(local_eid);
            fin.append_tlv(&tlv);
        }
        self.serialize_and_send_pdu(mgr, &fin)
    }

    /// C++ `Engine::sendNak` (class 2 only, asserted).
    pub fn send_nak(&mut self, mgr: &CfdpManager, t: TxnIdx, nak: &NakPdu) -> Status {
        let class = self.txn(t).txn_class;
        fw_assert!(class == Class::Class2, i32::from(class.as_repr()));
        self.serialize_and_send_pdu(mgr, nak)
    }

    /// C++ `Engine::sendFinAckStateless`: an ACK(FIN) for a transaction no
    /// longer known, from the FIN's header (`UNRECOGNIZED` status).
    pub fn send_fin_ack_stateless(
        &mut self,
        mgr: &CfdpManager,
        tsn: TransactionSeq,
        fin_src_eid: EntityId,
        fin_dst_eid: EntityId,
        cc: ConditionCode,
    ) -> Status {
        let mut ack = AckPdu::new();
        ack.initialize(
            PduDirection::TowardReceiver,
            Class::Class2,
            fin_src_eid,
            tsn,
            fin_dst_eid,
            FileDirective::FIN,
            1,
            cc,
            AckTxnStatus::UNRECOGNIZED,
        );
        self.serialize_and_send_pdu(mgr, &ack)
    }

    /// C++ `Engine::recvMd`: record the file size and names.
    pub fn recv_md(&mut self, mgr: &CfdpManager, t: TxnIdx, md: &MetadataPdu) {
        self.txn_mut(t).fsize = md.get_file_size();
        let hist = self.hist_mut(t);
        hist.fnames
            .src_filename
            .set_bytes(md.get_source_filename().as_bytes());
        hist.fnames
            .dst_filename
            .set_bytes(md.get_dest_filename().as_bytes());
        let hist = self.hist(t);
        mgr.log_activity_lo_metadata_received(
            hist.fnames.src_filename.as_bytes(),
            hist.fnames.dst_filename.as_bytes(),
            hist.seq_num,
        );
    }

    /// C++ `Engine::recvFd`: segment metadata is not supported.
    pub fn recv_fd(&mut self, mgr: &CfdpManager, t: TxnIdx, fd: &FileDataPdu<'_>) -> Status {
        if fd.as_header().has_segment_metadata() {
            mgr.log_warning_lo_file_data_segment_metadata();
            self.set_txn_status(t, TxnStatus::PROTOCOL_ERROR);
            self.tlm.recv_errors = self.tlm.recv_errors.wrapping_add(1);
            return Status::Error;
        }
        Status::Success
    }

    /// C++ `Engine::recvEof`: TLVs are accepted without validation.
    pub fn recv_eof(&mut self, _t: TxnIdx, eof: &EofPdu) -> Status {
        let tlvs = eof.get_tlv_list();
        for i in 0..tlvs.get_num_tlv() {
            // Entity-ID TLVs are not validated (upstream future work).
            let _ = tlvs.get_tlv(i).get_type() == TlvType::ENTITY_ID;
        }
        Status::Success
    }

    /// C++ `Engine::recvFin`: TLVs are accepted without validation.
    pub fn recv_fin(&mut self, _t: TxnIdx, fin: &FinPdu) -> Status {
        let tlvs = fin.get_tlv_list();
        for i in 0..tlvs.get_num_tlv() {
            let _ = tlvs.get_tlv(i).get_type() == TlvType::ENTITY_ID;
        }
        Status::Success
    }

    /// C++ `Engine::recvNak`.
    pub fn recv_nak(&mut self, _t: TxnIdx, _nak: &NakPdu) -> Status {
        Status::Success
    }

    /// C++ `Engine::recvDrop`.
    fn recv_drop(&mut self, _t: TxnIdx) {
        self.tlm.recv_dropped = self.tlm.recv_dropped.wrapping_add(1);
    }

    /// C++ `Engine::recvHold`: everything is spurious, but a class 2 FIN is
    /// re-acknowledged (the peer may have missed the FIN-ACK).
    fn recv_hold(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) {
        self.tlm.recv_spurious = self.tlm.recv_spurious.wrapping_add(1);
        if peek_pdu_type(data) == PduTypeEnum::Finished && self.txn(t).txn_class == Class::Class2 {
            let mut fin = FinPdu::new();
            if fin.deserialize_from(&mut PduReader::new(data)) == SerializeStatus::Ok {
                let (peer, seq) = (self.hist(t).peer_eid, self.hist(t).seq_num);
                let _ = self.send_ack(
                    mgr,
                    t,
                    AckTxnStatus::TERMINATED,
                    FileDirective::FIN,
                    fin.get_condition_code(),
                    peer,
                    seq,
                );
            }
        }
    }

    /// C++ `Engine::recvInit`: the first PDU of a new receive transaction.
    /// Metadata starts R1/R2; class 2 file data starts R2 without metadata
    /// (returns true to re-dispatch it); class 1 file data drops; anything
    /// else frees the transaction.
    fn recv_init(&mut self, mgr: &CfdpManager, t: TxnIdx, data: &[u8]) -> bool {
        let pdu_type = peek_pdu_type(data);
        let mut header = PduHeader::default();
        let status = header.from_serial_buffer(&mut PduReader::new(data));
        if status != SerializeStatus::Ok {
            mgr.log_warning_lo_fail_pdu_header_deserialization(self.channel_id, status as i32);
            return false;
        }
        let transaction_seq = header.get_transaction_seq();
        let source_eid = header.get_source_eid();
        let txm_mode = header.get_txm_mode();
        let hist = self.hist_mut(t);
        hist.seq_num = transaction_seq;
        hist.peer_eid = source_eid;
        hist.src_eid = source_eid;
        if self.txn(t).chunks.is_none() {
            let chunks = self.find_unused_chunks(Direction::Rx);
            self.txn_mut(t).chunks = chunks;
        }
        if self.txn(t).chunks.is_none() {
            mgr.log_warning_lo_chunklist_unavailable(transaction_seq);
        } else if pdu_type == PduTypeEnum::FileData {
            // File data with no transaction: PDUs were probably missed.
            if txm_mode == Class::Class1 {
                // R1 cannot work without metadata; the inactivity timer
                // recycles the dropping transaction.
                self.txn_mut(t).state = TxnState::Drop;
            } else {
                let txn = self.txn_mut(t);
                txn.state = TxnState::R2;
                txn.txn_class = Class::Class2;
                self.r_init(mgr, t);
                return true;
            }
        } else if pdu_type == PduTypeEnum::Metadata {
            let mut md = MetadataPdu::new();
            let status = md.deserialize_from(&mut PduReader::new(data));
            if status == SerializeStatus::Ok {
                self.recv_md(mgr, t, &md);
                let txn = self.txn_mut(t);
                txn.state = if txm_mode == Class::Class1 {
                    TxnState::R1
                } else {
                    TxnState::R2
                };
                txn.txn_class = txm_mode;
                txn.flags.rx.md_recv = true;
                self.r_init(mgr, t);
            } else {
                mgr.log_warning_lo_fail_metadata_pdu_deserialization(
                    self.channel_id,
                    status as i32,
                );
            }
        } else {
            mgr.log_warning_lo_unhandled_pdu_in_idle_state();
            self.tlm.recv_errors = self.tlm.recv_errors.wrapping_add(1);
        }
        if self.txn(t).state == TxnState::Init {
            self.finish_transaction(mgr, t, false);
        }
        false
    }

    /// C++ `Engine::startRxTransaction`: a receive transaction on RX with
    /// the default FIN status (incomplete, discarded).
    pub fn start_rx_transaction(&mut self) -> Option<TxnIdx> {
        let t = self.find_unused_transaction(Direction::Rx)?;
        let txn = self.txn_mut(t);
        txn.state_data.receive.r2.dc = FinDeliveryCode::Incomplete;
        txn.state_data.receive.r2.fs = FinFileStatus::Discarded;
        txn.flags.com.q_index = QueueId::Rx;
        self.insert_back_txn(QueueId::Rx, t);
        Some(t)
    }

    /// C++ `Engine::txFileInitiate`: set the transaction up for sending,
    /// assign the next sequence number and queue it on PEND by priority.
    #[allow(clippy::too_many_arguments)]
    pub fn tx_file_initiate(
        &mut self,
        mgr: &CfdpManager,
        t: TxnIdx,
        cfdp_class: Class,
        keep: Keep,
        chan: u8,
        priority: u8,
        dest_id: EntityId,
        seq_num: &mut TransactionSeq,
    ) {
        self.txn_mut(t)
            .init_tx_file(cfdp_class, keep, chan, priority);
        *seq_num = seq_num.wrapping_add(1);
        let local_eid = mgr.get_local_eid_param();
        let hist = self.hist_mut(t);
        hist.seq_num = *seq_num;
        hist.src_eid = local_eid;
        hist.peer_eid = dest_id;
        self.insert_sort_prio(t, QueueId::Pend);
    }

    /// C++ `Engine::playbackDirInitiate`: open the directory and record the
    /// playback (the channel starts the transfers on its next cycle).
    #[allow(clippy::too_many_arguments)]
    pub fn playback_dir_initiate(
        &mut self,
        mgr: &CfdpManager,
        pb: PbRef,
        src: &[u8],
        dst: &[u8],
        cfdp_class: Class,
        keep: Keep,
        priority: u8,
        dest_id: EntityId,
    ) -> Status {
        let playback = self.playback_mut(pb);
        let status = playback.dir.open(&path_str(src), DirOpenMode::Read);
        if status != DirStatus::OpOk {
            mgr.log_warning_lo_playback_dir_open_failed(src, status as i32);
            self.tlm.fault_directory_read = self.tlm.fault_directory_read.wrapping_add(1);
            return Status::Error;
        }
        playback.diropen = true;
        playback.busy = true;
        playback.keep = keep;
        playback.priority = priority;
        playback.dest_id = dest_id;
        playback.cfdp_class = cfdp_class;
        playback.fnames.src_filename.set_bytes(src);
        playback.fnames.dst_filename.set_bytes(dst);
        Status::Success
    }

    /// C++ `Engine::finishTransaction`: close out a transaction — move a
    /// transmit off TXA, close the file (deleting/moving it unless kept),
    /// report completion or failure, notify a port-initiated sender, release
    /// a playback slot, and put it in HOLD for the inactivity timer to
    /// recycle. A freed transaction is ignored (`ResetFreedTransaction`).
    pub fn finish_transaction(&mut self, mgr: &CfdpManager, t: TxnIdx, keep_history: bool) {
        if self.txn(t).flags.com.q_index == QueueId::Free {
            mgr.log_diagnostic_reset_freed_transaction();
            return;
        }
        if self.txn(t).flags.com.q_index == QueueId::Txa {
            self.dequeue_transaction(t);
            self.insert_sort_prio(t, QueueId::Txw);
        }
        if self.txn(t).fd.is_open() {
            self.txn_mut(t).fd.close();
            if self.txn(t).keep == Keep::Delete {
                self.handle_not_keep_file(mgr, t);
            }
        }
        if self.txn(t).history.is_some() {
            let class = self.txn(t).txn_class;
            let fsize = self.txn(t).fsize;
            let hist = self.hist(t);
            let (src, dst) = (
                hist.fnames.src_filename.as_bytes(),
                hist.fnames.dst_filename.as_bytes(),
            );
            if !txn_status_is_error(hist.txn_stat) {
                if hist.dir == Direction::Tx {
                    mgr.log_activity_hi_tx_file_transfer_completed(
                        class,
                        hist.seq_num,
                        hist.src_eid,
                        src,
                        hist.peer_eid,
                        dst,
                        fsize,
                    );
                } else {
                    mgr.log_activity_hi_rx_file_transfer_completed(
                        class,
                        hist.seq_num,
                        hist.src_eid,
                        src,
                        mgr.get_local_eid_param(),
                        dst,
                        fsize,
                    );
                }
            } else if hist.dir == Direction::Tx {
                mgr.log_warning_lo_tx_file_transfer_failed(
                    class,
                    hist.seq_num,
                    hist.src_eid,
                    src,
                    hist.peer_eid,
                    dst,
                    hist.txn_stat.0 as u8,
                );
            } else {
                mgr.log_warning_lo_rx_file_transfer_failed(
                    class,
                    hist.seq_num,
                    hist.src_eid,
                    src,
                    hist.peer_eid,
                    dst,
                    hist.txn_stat.0 as u8,
                );
            }
            let (dir, failed) = (hist.dir, txn_status_is_error(hist.txn_stat));
            if dir == Direction::Tx && self.txn(t).flags.tx.cmd_tx {
                self.decrement_cmd_tx_counter();
            }
            if self.txn(t).init_type == TransactionInitType::InitByPort {
                mgr.send_file_complete(if failed {
                    SendFileStatus::StatusError
                } else {
                    SendFileStatus::StatusOk
                });
            }
            self.txn_mut(t).flags.com.keep_history = keep_history;
        }
        if let Some(pb) = self.txn(t).pb {
            let playback = self.playback_mut(pb);
            fw_assert!(playback.num_ts > 0);
            playback.num_ts -= 1;
        }
        self.clear_current_if_match(t);
        self.txn_mut(t).state = TxnState::Hold;
        self.arm_inact_timer(mgr, t);
    }

    /// C++ `Engine::setTxnStatus`: set the status unless an error is
    /// already recorded.
    pub fn set_txn_status(&mut self, t: TxnIdx, txn_stat: TxnStatus) {
        let hist = self.hist_mut(t);
        if !txn_status_is_error(hist.txn_stat) {
            hist.txn_stat = txn_stat;
        }
    }

    /// C++ `Engine::cancelTransaction`: mark canceled (once) and let the
    /// direction's cancel close it out.
    pub fn cancel_transaction(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        if self.txn(t).flags.com.canceled {
            return;
        }
        self.txn_mut(t).flags.com.canceled = true;
        self.set_txn_status(t, TxnStatus::CANCEL_REQUEST_RECEIVED);
        match self.hist(t).dir {
            Direction::Rx => self.r_cancel(mgr, t),
            Direction::Tx => self.s_cancel(t),
        }
    }

    /// C++ `Engine::isPollingDir`: whether the directory part of `src_file`
    /// (everything before the last `/`, empty without one) is the source
    /// directory of one of the channel's polling slots — an unused slot's
    /// empty source directory matches a bare file name (C++ parity).
    #[must_use]
    pub fn is_polling_dir(&self, src_file: &[u8]) -> bool {
        let src_dir = match src_file.iter().rposition(|&b| b == b'/') {
            Some(pos) => &src_file[..pos],
            None => &[][..],
        };
        self.polldir
            .iter()
            .any(|pd| pd.src_dir.as_bytes() == src_dir)
    }

    /// C++ `Engine::handleNotKeepFile`: a sent file is moved to the move
    /// directory (or deleted) on success; a failed file from a polling
    /// directory is moved to the fail directory (or deleted); a received
    /// file is deleted.
    pub fn handle_not_keep_file(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        let chan = self.channel_id;
        let hist = self.hist(t);
        let src = hist.fnames.src_filename.clone();
        let src_path = path_str(src.as_bytes()).into_owned();
        if hist.dir == Direction::Tx {
            let (dir_param, polling) = if !txn_status_is_error(hist.txn_stat) {
                (Some(mgr.get_move_dir_param(chan)), true)
            } else if self.is_polling_dir(src.as_bytes()) {
                (Some(mgr.get_fail_dir_param(chan)), true)
            } else {
                (None, false)
            };
            if !polling {
                return;
            }
            let failed = txn_status_is_error(hist.txn_stat);
            let mut file_status = FsStatus::OtherError;
            if let Some(dir) = dir_param.filter(|d| !d.is_empty()) {
                // C++ parity: moved to the directory path itself.
                file_status = filesystem::move_file(&src_path, &path_str(dir.as_bytes()));
                if file_status != FsStatus::OpOk {
                    if failed {
                        mgr.log_warning_lo_fail_poll_file_move(
                            src.as_bytes(),
                            dir.as_bytes(),
                            file_status as i32,
                        );
                    } else {
                        mgr.log_warning_lo_fail_keep_file_move(
                            src.as_bytes(),
                            dir.as_bytes(),
                            file_status as i32,
                        );
                    }
                }
            }
            if file_status != FsStatus::OpOk {
                let status = filesystem::remove_file(&src_path);
                if status != FsStatus::OpOk {
                    mgr.log_warning_lo_file_remove_failed(src.as_bytes(), status as i32);
                }
            }
        } else {
            let dst = hist.fnames.dst_filename.clone();
            let status = filesystem::remove_file(&path_str(dst.as_bytes()));
            if status != FsStatus::OpOk {
                mgr.log_warning_lo_file_remove_failed(dst.as_bytes(), status as i32);
            }
        }
    }
}
