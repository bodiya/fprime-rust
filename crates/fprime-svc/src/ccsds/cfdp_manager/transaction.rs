//! CFDP transactions — port of `Svc/Ccsds/CfdpManager/Transaction.hpp`
//! (derived from NASA CF 3.0.0 `cf_cfdp_r.h`, `cf_cfdp_s.h`,
//! `cf_cfdp_dispatch.h`) and the construction/reset half of
//! `TransactionRx.cpp`.
//!
//! Copyright (c) 2019 United States Government as represented by the
//! Administrator of the National Aeronautics and Space Administration.
//! All Rights Reserved. NASA Docket No. GSC-18,447-1. Licensed under the
//! Apache License, Version 2.0.
//!
//! A transaction is one element of its channel's preallocated pool. Upstream
//! each transaction holds pointers to its channel, engine and manager, and
//! its state-machine methods reach through them; here the transaction is
//! plain data and its state-machine methods are `Channel` methods taking
//! the transaction's pool index ([`TxnIdx`]) — see `transaction_tx.rs` and
//! `transaction_rx.rs`. The history entry, chunk list and playback a
//! transaction refers to are likewise indices into the channel's pools.

use super::clist::NodeIdx;
use super::timer::Timer;
use super::types::{
    Class, FileSize, Keep, QueueId, StateData, StateFlags, TransactionInitType, TxnState,
};
use fprime_os::file::File;
use fprime_utils::cfdp::Checksum;

/// Index of a transaction in its channel's pool (C++ `Transaction*`).
pub type TxnIdx = NodeIdx;

/// Which playback a transaction belongs to (C++ `Playback* m_pb`): one of
/// the channel's commanded playbacks, or the playback of one of its
/// polling directories.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PbRef {
    /// `m_playback[i]`.
    Playback(u8),
    /// `m_polldir[i].pb`.
    PollDir(u8),
}

/// C++ `Transaction` (the data members).
pub struct Transaction {
    /// High-level state.
    pub(crate) state: TxnState,
    /// Class 1 or 2.
    pub(crate) txn_class: Class,
    /// History entry (C++ `m_history`), when active.
    pub(crate) history: Option<NodeIdx>,
    /// Chunk list (C++ `m_chunks`), when assigned.
    pub(crate) chunks: Option<NodeIdx>,
    /// Inactivity timer.
    pub(crate) inactivity_timer: Timer,
    /// ACK timer.
    pub(crate) ack_timer: Timer,
    /// File size.
    pub(crate) fsize: FileSize,
    /// File offset (send).
    pub(crate) foffs: FileSize,
    /// The file.
    pub(crate) fd: File,
    /// Running file checksum.
    pub(crate) crc: Checksum,
    /// Keep or delete the file at the end.
    pub(crate) keep: Keep,
    /// Channel number.
    pub(crate) chan_num: u8,
    /// Priority (0 is the highest).
    pub(crate) priority: u8,
    /// Who started the transaction.
    pub(crate) init_type: TransactionInitType,
    /// The owning playback, if any.
    pub(crate) pb: Option<PbRef>,
    /// Send/receive state data.
    pub(crate) state_data: StateData,
    /// Flags.
    pub(crate) flags: StateFlags,
}

impl Transaction {
    /// C++ constructor.
    #[must_use]
    pub fn new(channel_id: u8) -> Self {
        Self {
            state: TxnState::Undef,
            txn_class: Class::Class1,
            history: None,
            chunks: None,
            inactivity_timer: Timer::new(),
            ack_timer: Timer::new(),
            fsize: 0,
            foffs: 0,
            fd: File::new(),
            crc: Checksum::new(),
            keep: Keep::Keep,
            chan_num: channel_id,
            priority: 0,
            init_type: TransactionInitType::InitByCommand,
            pb: None,
            state_data: StateData::default(),
            flags: StateFlags::default(),
        }
    }

    /// C++ `reset`: back to the unused state. Keeps the channel binding,
    /// the history and chunk assignments (managed by the channel) and the
    /// list links; closes the file and disables both timers.
    pub fn reset(&mut self) {
        self.state = TxnState::Undef;
        self.txn_class = Class::Class1;
        self.fsize = 0;
        self.foffs = 0;
        self.keep = Keep::Keep;
        self.priority = 0;
        self.init_type = TransactionInitType::InitByCommand;
        self.crc = Checksum::from_value(0);
        self.pb = None;
        self.state_data = StateData::default();
        self.flags = StateFlags::default();
        if self.fd.is_open() {
            self.fd.close();
        }
        self.inactivity_timer.disable_timer();
        self.ack_timer.disable_timer();
    }

    /// C++ `initTxFile`.
    pub fn init_tx_file(&mut self, cfdp_class: Class, keep: Keep, chan: u8, priority: u8) {
        self.chan_num = chan;
        self.priority = priority;
        self.keep = keep;
        self.txn_class = cfdp_class;
        self.state = if cfdp_class == Class::Class2 {
            TxnState::S2
        } else {
            TxnState::S1
        };
        self.state_data.send.sub_state = super::types::TxSubState::Metadata;
    }

    /// The priority.
    #[must_use]
    pub fn get_priority(&self) -> u8 {
        self.priority
    }

    /// The channel number.
    #[must_use]
    pub fn get_channel_id(&self) -> u8 {
        self.chan_num
    }

    /// The class.
    #[must_use]
    pub fn get_class(&self) -> Class {
        self.txn_class
    }

    /// The state.
    #[must_use]
    pub fn get_state(&self) -> TxnState {
        self.state
    }

    /// The queue the transaction is on.
    #[must_use]
    pub fn q_index(&self) -> QueueId {
        self.flags.com.q_index
    }
}
