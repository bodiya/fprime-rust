//! CFDP channels — port of `Svc/Ccsds/CfdpManager/Channel.{hpp,cpp}`,
//! derived from the channel functions of NASA CF 3.0.0 `cf_cfdp.c` and
//! `cf_utils.c`.
//!
//! Copyright (c) 2019 United States Government as represented by the
//! Administrator of the National Aeronautics and Space Administration.
//! All Rights Reserved. NASA Docket No. GSC-18,447-1. Licensed under the
//! Apache License, Version 2.0.
//!
//! A channel owns its transaction pool, history pool and chunk-list pool
//! (allocated once, at configure time) and the queues threading them:
//!
//! ```text
//! PEND -> TXA -> TXW            transmit transactions (priority sorted)
//! RX                            receive transactions
//! FREE                          unused transactions
//! HIST / HIST_FREE              history entries
//! cs[RX] / cs[TX]               unused chunk lists
//! ```
//!
//! Every engine cycle the channel ticks its receive and waiting-transmit
//! transactions (timers, ACK/NAK/FIN/EOF sends, NAK responses), then runs
//! the first active transmit transaction until it finishes its file data
//! or the per-cycle PDU budget runs out, then services directory playbacks
//! and polls. Its per-channel telemetry counters live here too (upstream
//! they are the manager's `m_channelTelemetry[channel]`).

use super::CfdpManager;
use super::chunk::{ChunkIdx, ChunkList};
use super::clist::{CListLinks, CListTraverseStatus, ListHead, NodeIdx, traverse, traverse_r};
use super::timer::TimerStatus;
use super::transaction::{PbRef, Transaction, TxnIdx};
use super::types::{
    ChannelTelemetry, ChunkWrapper, Class, DIRECTION_NUM, Direction, EntityId, Flow, History, Keep,
    NUM_TRANSACTIONS_PER_CHANNEL, Playback, PollDir, QueueId, TICK_TYPE_NUM_TYPES, TickType,
    TransactionSeq, TxnState, TxnStatus,
};
use fprime_config::cfdp::{
    CHANNEL_NUM_RX_CHUNKS_PER_TRANSACTION, CHANNEL_NUM_TX_CHUNKS_PER_TRANSACTION,
    MAX_COMMANDED_PLAYBACK_DIRECTORIES_PER_CHAN, MAX_POLLING_DIR_PER_CHAN,
    NUM_HISTORIES_PER_CHANNEL, NUM_TRANSACTIONS_PER_PLAYBACK,
};
use fprime_fw::{Enabled, fw_assert};
use fprime_os::directory::Status as DirStatus;

/// The traversal and tick-retry bound (upstream's `maxIterations` /
/// `maxRetries`): the largest possible transaction queue.
pub(crate) const MAX_ITERATIONS: u32 = NUM_TRANSACTIONS_PER_CHANNEL as u32;

/// C++ `TickArgs` (the function is chosen by the tick type).
struct TickArgs {
    tick_type: TickType,
    early_exit: bool,
    cont: bool,
}

/// C++ `Channel`.
pub struct Channel {
    /// Channel number (index into the engine).
    pub(crate) channel_id: u8,
    /// Queue heads, indexed by `QueueId` (HIST/HIST_FREE over histories,
    /// the rest over transactions).
    pub(crate) qs: [ListHead; QueueId::Num as usize],
    /// Free chunk lists, by direction.
    pub(crate) cs: [ListHead; DIRECTION_NUM],
    /// Commanded transmit transactions in flight.
    pub(crate) num_cmd_tx: u32,
    /// Commanded directory playbacks.
    pub(crate) playback: [Playback; MAX_COMMANDED_PLAYBACK_DIRECTORIES_PER_CHAN],
    /// Polling directories.
    pub(crate) polldir: [PollDir; MAX_POLLING_DIR_PER_CHAN],
    /// Where tick/transmit processing left off when the PDU budget ran out.
    pub(crate) current_txn: Option<TxnIdx>,
    /// The tick type to resume with.
    pub(crate) tick_type: u8,
    /// Frozen or not.
    pub(crate) flow_state: Flow,
    /// PDUs sent this cycle (throttle).
    pub(crate) outgoing_counter: u32,
    /// The transaction pool.
    pub(crate) txns: Box<[Transaction]>,
    /// Transaction list links.
    pub(crate) txn_links: CListLinks,
    /// The history pool.
    pub(crate) histories: Box<[History]>,
    /// History list links.
    pub(crate) hist_links: CListLinks,
    /// The chunk-list pool: `2 * NUM_TRANSACTIONS_PER_CHANNEL`, alternating
    /// receive and transmit lists.
    pub(crate) chunks: Box<[ChunkWrapper]>,
    /// Chunk-list pool links.
    pub(crate) chunk_links: CListLinks,
    /// Chunks per list, by direction.
    pub(crate) dir_max_chunks: [u32; DIRECTION_NUM],
    /// The channel's telemetry counters.
    pub(crate) tlm: ChannelTelemetry,
}

impl Channel {
    /// C++ constructor: allocate and link the pools. Every transaction goes
    /// on FREE, every history entry on HIST_FREE, and two chunk lists per
    /// transaction (receive, transmit) on the per-direction free lists.
    #[must_use]
    pub fn new(channel_id: u8) -> Self {
        let rx_max = CHANNEL_NUM_RX_CHUNKS_PER_TRANSACTION[usize::from(channel_id)];
        let tx_max = CHANNEL_NUM_TX_CHUNKS_PER_TRANSACTION[usize::from(channel_id)];
        let dir_max_chunks = [rx_max, tx_max];
        let mut chunks = Vec::with_capacity(NUM_TRANSACTIONS_PER_CHANNEL * DIRECTION_NUM);
        for _ in 0..NUM_TRANSACTIONS_PER_CHANNEL {
            for max in dir_max_chunks {
                chunks.push(ChunkWrapper {
                    chunks: ChunkList::new(max as ChunkIdx),
                });
            }
        }
        let mut chan = Self {
            channel_id,
            qs: [None; QueueId::Num as usize],
            cs: [None; DIRECTION_NUM],
            num_cmd_tx: 0,
            playback: core::array::from_fn(|_| Playback::default()),
            polldir: core::array::from_fn(|_| PollDir::default()),
            current_txn: None,
            tick_type: 0,
            flow_state: Flow::NotFrozen,
            outgoing_counter: 0,
            txns: (0..NUM_TRANSACTIONS_PER_CHANNEL)
                .map(|_| Transaction::new(channel_id))
                .collect(),
            txn_links: CListLinks::new(NUM_TRANSACTIONS_PER_CHANNEL),
            histories: (0..NUM_HISTORIES_PER_CHANNEL)
                .map(|_| History::default())
                .collect(),
            hist_links: CListLinks::new(NUM_HISTORIES_PER_CHANNEL),
            chunks: chunks.into_boxed_slice(),
            chunk_links: CListLinks::new(NUM_TRANSACTIONS_PER_CHANNEL * DIRECTION_NUM),
            dir_max_chunks,
            tlm: ChannelTelemetry::default(),
        };
        for j in 0..NUM_TRANSACTIONS_PER_CHANNEL {
            chan.free_transaction(j as TxnIdx);
            for k in 0..DIRECTION_NUM {
                let cw = (j * DIRECTION_NUM + k) as NodeIdx;
                chan.chunk_links.init_node(cw);
                chan.chunk_links.insert_back(&mut chan.cs[k], cw);
            }
        }
        for j in 0..NUM_HISTORIES_PER_CHANNEL {
            let h = j as NodeIdx;
            chan.hist_links.init_node(h);
            chan.insert_back_in_queue(QueueId::HistFree, h);
        }
        chan
    }

    // -- accessors -----------------------------------------------------------

    /// The transaction at `t`.
    #[must_use]
    pub fn txn(&self, t: TxnIdx) -> &Transaction {
        &self.txns[usize::from(t)]
    }

    /// The transaction at `t`, mutably.
    pub fn txn_mut(&mut self, t: TxnIdx) -> &mut Transaction {
        &mut self.txns[usize::from(t)]
    }

    /// The history entry of transaction `t` (asserts it has one: upstream
    /// dereferences `m_history`).
    #[must_use]
    pub fn hist(&self, t: TxnIdx) -> &History {
        let h = self.txn(t).history;
        fw_assert!(h.is_some(), i32::from(t));
        &self.histories[usize::from(h.unwrap_or_default())]
    }

    /// The history entry of transaction `t`, mutably.
    pub fn hist_mut(&mut self, t: TxnIdx) -> &mut History {
        let h = self.txn(t).history;
        fw_assert!(h.is_some(), i32::from(t));
        &mut self.histories[usize::from(h.unwrap_or_default())]
    }

    /// The chunk list of transaction `t` (asserts it has one).
    pub fn chunk_list_mut(&mut self, t: TxnIdx) -> &mut ChunkList {
        let c = self.txn(t).chunks;
        fw_assert!(c.is_some(), i32::from(t));
        &mut self.chunks[usize::from(c.unwrap_or_default())].chunks
    }

    /// The chunk list of transaction `t` (asserts it has one).
    #[must_use]
    pub fn chunk_list(&self, t: TxnIdx) -> &ChunkList {
        let c = self.txn(t).chunks;
        fw_assert!(c.is_some(), i32::from(t));
        &self.chunks[usize::from(c.unwrap_or_default())].chunks
    }

    /// The playback `pb` refers to.
    pub fn playback_mut(&mut self, pb: PbRef) -> &mut Playback {
        match pb {
            PbRef::Playback(i) => &mut self.playback[usize::from(i)],
            PbRef::PollDir(i) => &mut self.polldir[usize::from(i)].pb,
        }
    }

    /// The playback `pb` refers to.
    #[must_use]
    pub fn playback_ref(&self, pb: PbRef) -> &Playback {
        match pb {
            PbRef::Playback(i) => &self.playback[usize::from(i)],
            PbRef::PollDir(i) => &self.polldir[usize::from(i)].pb,
        }
    }

    /// Chunks per list for `dir` (C++ `m_dirMaxChunks`).
    #[must_use]
    pub fn get_dir_max_chunks(&self, dir: Direction) -> u32 {
        self.dir_max_chunks[dir as usize]
    }

    /// C++ `getChannelId`.
    #[must_use]
    pub fn get_channel_id(&self) -> u8 {
        self.channel_id
    }

    /// C++ `getOutgoingCounter`.
    #[must_use]
    pub fn get_outgoing_counter(&self) -> u32 {
        self.outgoing_counter
    }

    /// C++ `getNumCmdTx`.
    #[must_use]
    pub fn get_num_cmd_tx(&self) -> u32 {
        self.num_cmd_tx
    }

    /// C++ `getFlowState`.
    #[must_use]
    pub fn get_flow_state(&self) -> Flow {
        self.flow_state
    }

    /// C++ `setFlowState`.
    pub fn set_flow_state(&mut self, flow_state: Flow) {
        self.flow_state = flow_state;
    }

    /// C++ `getPlayback` (asserts the index).
    pub fn get_playback(&mut self, index: usize) -> &mut Playback {
        fw_assert!(
            index < MAX_COMMANDED_PLAYBACK_DIRECTORIES_PER_CHAN,
            index as i32
        );
        &mut self.playback[index]
    }

    /// C++ `getPollDir` (asserts the index).
    pub fn get_poll_dir(&mut self, index: usize) -> &mut PollDir {
        fw_assert!(index < MAX_POLLING_DIR_PER_CHAN, index as i32);
        &mut self.polldir[index]
    }

    /// C++ `decrementCmdTxCounter` (asserts it is non-zero).
    pub fn decrement_cmd_tx_counter(&mut self) {
        fw_assert!(self.num_cmd_tx > 0);
        self.num_cmd_tx -= 1;
    }

    /// C++ `clearCurrentIfMatch`.
    pub fn clear_current_if_match(&mut self, t: TxnIdx) {
        if self.current_txn == Some(t) {
            self.current_txn = None;
        }
    }

    /// C++ `setCurrentTxn`.
    pub fn set_current_txn(&mut self, t: TxnIdx) {
        self.current_txn = Some(t);
    }

    // -- queues --------------------------------------------------------------

    fn txn_links(c: &Self) -> &CListLinks {
        &c.txn_links
    }

    /// C++ `removeFromQueue` (transaction queues).
    pub(crate) fn remove_from_queue(&mut self, q: QueueId, t: TxnIdx) {
        self.txn_links
            .remove(&mut self.qs[usize::from(q.as_repr())], t);
    }

    /// C++ `insertBackInQueue` on a transaction queue.
    pub(crate) fn insert_back_txn(&mut self, q: QueueId, t: TxnIdx) {
        self.txn_links
            .insert_back(&mut self.qs[usize::from(q.as_repr())], t);
    }

    /// C++ `insertBackInQueue` on a history queue.
    fn insert_back_in_queue(&mut self, q: QueueId, h: NodeIdx) {
        self.hist_links
            .insert_back(&mut self.qs[usize::from(q.as_repr())], h);
    }

    /// Apply a +1/-1 to the queue-depth counter of `q` (PEND and HIST_FREE
    /// have none; anything else asserts, C++ parity). The counters wrap like
    /// the C++ `U16` arithmetic (upstream inserts into FREE and RX without
    /// counting, so they can underflow).
    fn adjust_queue_tlm(&mut self, q: QueueId, up: bool) {
        let adjust = |v: u16| {
            if up {
                v.wrapping_add(1)
            } else {
                v.wrapping_sub(1)
            }
        };
        match q {
            QueueId::Free => self.tlm.queue_free = adjust(self.tlm.queue_free),
            QueueId::Txa => self.tlm.queue_tx_active = adjust(self.tlm.queue_tx_active),
            QueueId::Txw => self.tlm.queue_tx_waiting = adjust(self.tlm.queue_tx_waiting),
            QueueId::Rx => self.tlm.queue_rx = adjust(self.tlm.queue_rx),
            QueueId::Hist => self.tlm.queue_history = adjust(self.tlm.queue_history),
            QueueId::Pend | QueueId::HistFree => {}
            QueueId::Num => fw_assert!(false, i32::from(q.as_repr())),
        }
    }

    /// C++ `dequeueTransaction`: unlink `t` from its queue.
    pub fn dequeue_transaction(&mut self, t: TxnIdx) {
        let q = self.txn(t).flags.com.q_index;
        self.remove_from_queue(q, t);
        self.adjust_queue_tlm(q, false);
    }

    /// C++ `moveTransaction`: from its queue to the back of `queue`.
    pub fn move_transaction(&mut self, t: TxnIdx, queue: QueueId) {
        let q = self.txn(t).flags.com.q_index;
        self.remove_from_queue(q, t);
        self.adjust_queue_tlm(q, false);
        self.insert_back_txn(queue, t);
        self.txn_mut(t).flags.com.q_index = queue;
        self.adjust_queue_tlm(queue, true);
    }

    /// C++ `freeTransaction`: reset `t` and put it on FREE.
    pub fn free_transaction(&mut self, t: TxnIdx) {
        self.txn_mut(t).reset();
        self.txn_links.init_node(t);
        self.insert_back_txn(QueueId::Free, t);
        self.txn_mut(t).flags.com.q_index = QueueId::Free;
    }

    /// C++ `recycleTransaction`: close a dangling file, return the chunk
    /// list and the history entry (to HIST when kept, else HIST_FREE) and
    /// free the transaction.
    pub fn recycle_transaction(&mut self, mgr: &CfdpManager, t: TxnIdx) {
        if self.txn(t).fd.is_open() {
            let seq = self.hist(t).seq_num;
            mgr.log_warning_lo_dangling_file_handle_closed(self.txn(t).chan_num, seq);
            self.txn_mut(t).fd.close();
        }
        self.dequeue_transaction(t);
        if let Some(h) = self.txn(t).history {
            if let Some(c) = self.txn(t).chunks {
                let dir = self.histories[usize::from(h)].dir as usize;
                self.chunks[usize::from(c)].chunks.reset();
                self.chunk_links.insert_back(&mut self.cs[dir], c);
                self.txn_mut(t).chunks = None;
            }
            let dest = if self.txn(t).flags.com.keep_history {
                QueueId::Hist
            } else {
                QueueId::HistFree
            };
            self.insert_back_in_queue(dest, h);
            self.txn_mut(t).history = None;
        }
        self.free_transaction(t);
    }

    /// C++ `insertSortPrio`: insert after the last transaction (searching
    /// from the tail) whose priority is not lower than `t`'s, else at the
    /// back, and record the queue.
    pub fn insert_sort_prio(&mut self, t: TxnIdx, queue: QueueId) {
        let qi = usize::from(queue.as_repr());
        let mut insert_back = false;
        if self.qs[qi].is_none() {
            insert_back = true;
        } else {
            let priority = self.txn(t).priority;
            let mut found: Option<TxnIdx> = None;
            traverse_r(
                self,
                Self::txn_links,
                self.qs[qi],
                MAX_ITERATIONS,
                |c, n| {
                    if c.txn(n).priority <= priority {
                        found = Some(n);
                        CListTraverseStatus::Exit
                    } else {
                        CListTraverseStatus::Continue
                    }
                },
            );
            match found {
                Some(after) => {
                    let head = self.qs[qi];
                    self.txn_links.insert_after(&head, after, t);
                }
                None => insert_back = true,
            }
        }
        if insert_back {
            self.insert_back_txn(queue, t);
        }
        self.txn_mut(t).flags.com.q_index = queue;
    }

    /// C++ `resetHistory`: move a history entry from HIST to HIST_FREE.
    pub fn reset_history(&mut self, h: NodeIdx) {
        self.hist_links
            .remove(&mut self.qs[QueueId::Hist as usize], h);
        self.insert_back_in_queue(QueueId::HistFree, h);
    }

    /// C++ `findUnusedChunks`: pop a free chunk list for `dir`.
    pub fn find_unused_chunks(&mut self, dir: Direction) -> Option<NodeIdx> {
        let head = &mut self.cs[dir as usize];
        if head.is_some() {
            self.chunk_links.pop(head)
        } else {
            None
        }
    }

    /// C++ `findUnusedTransaction`: take a transaction off FREE with a
    /// history entry (a free one, else the oldest kept one), reset the
    /// history, and mark it `INIT` on no queue (`q_index` PEND).
    pub fn find_unused_transaction(&mut self, direction: Direction) -> Option<TxnIdx> {
        let t = self.qs[QueueId::Free as usize]?;
        self.remove_from_queue(QueueId::Free, t);
        let q = if self.qs[QueueId::HistFree as usize].is_some() {
            QueueId::HistFree
        } else {
            fw_assert!(self.qs[QueueId::Hist as usize].is_some());
            QueueId::Hist
        };
        let h = self.qs[q as usize].unwrap_or_default();
        self.hist_links.remove(&mut self.qs[q as usize], h);
        let history = &mut self.histories[usize::from(h)];
        history.txn_stat = TxnStatus::UNDEFINED;
        history.src_eid = 0;
        history.peer_eid = 0;
        history.seq_num = 0;
        history.fnames.src_filename.clear();
        history.fnames.dst_filename.clear();
        history.dir = direction;
        let txn = self.txn_mut(t);
        txn.history = Some(h);
        txn.state = TxnState::Init;
        txn.flags.com.q_index = QueueId::Pend;
        self.txn_links.init_node(t);
        Some(t)
    }

    /// C++ `findTransactionBySequenceNumber`: search RX, PEND, TXA and TXW
    /// (in that order) for the transaction `(seq, src_eid)`.
    pub fn find_transaction_by_sequence_number(
        &mut self,
        seq: TransactionSeq,
        src_eid: EntityId,
    ) -> Option<TxnIdx> {
        for q in [QueueId::Rx, QueueId::Pend, QueueId::Txa, QueueId::Txw] {
            let mut found = None;
            let head = self.qs[q as usize];
            traverse(self, Self::txn_links, head, MAX_ITERATIONS, |c, n| {
                if let Some(h) = c.txn(n).history {
                    let hist = &c.histories[usize::from(h)];
                    if hist.src_eid == src_eid && hist.seq_num == seq {
                        found = Some(n);
                        return CListTraverseStatus::Exit;
                    }
                }
                CListTraverseStatus::Continue
            });
            if found.is_some() {
                return found;
            }
        }
        None
    }

    /// C++ `traverseAllTransactions`: call `f` for every transaction on
    /// PEND, TXA, TXW and RX; returns the count.
    pub fn traverse_all_transactions(&mut self, mut f: impl FnMut(&mut Self, TxnIdx)) -> i32 {
        let mut counter = 0;
        for q in [QueueId::Pend, QueueId::Txa, QueueId::Txw, QueueId::Rx] {
            let head = self.qs[q as usize];
            traverse(self, Self::txn_links, head, MAX_ITERATIONS, |c, n| {
                f(c, n);
                counter += 1;
                CListTraverseStatus::Continue
            });
        }
        counter
    }

    // -- per-cycle processing ------------------------------------------------

    /// C++ `cycleTx`: if dequeue is enabled and no transaction is mid-send,
    /// run the first active transmit transaction; when none ran, promote the
    /// head of PEND to TXA (with a transmit chunk list for class 2) and try
    /// again.
    pub fn cycle_tx(&mut self, mgr: &CfdpManager) {
        if mgr.get_dequeue_enabled_param(self.channel_id) != Enabled::Enabled {
            return;
        }
        let mut ran_one = false;
        if self.current_txn.is_none() {
            loop {
                let head = self.qs[QueueId::Txa as usize];
                traverse(self, Self::txn_links, head, MAX_ITERATIONS, |c, n| {
                    c.cycle_tx_first_active(mgr, n, &mut ran_one)
                });
                let Some(t) = self.qs[QueueId::Pend as usize] else {
                    break;
                };
                if ran_one {
                    break;
                }
                if self.txn(t).txn_class == Class::Class2 {
                    if self.txn(t).chunks.is_none() {
                        let chunks = self.find_unused_chunks(Direction::Tx);
                        self.txn_mut(t).chunks = chunks;
                    }
                    if self.txn(t).chunks.is_none() {
                        // Leave it pending until a chunk list is free.
                        break;
                    }
                }
                self.arm_inact_timer(mgr, t);
                self.move_transaction(t, QueueId::Txa);
            }
        }
        self.current_txn = None;
    }

    /// C++ `cycleTxFirstActive`: run the first non-suspended TXA transaction
    /// until it leaves TXA or the PDU budget runs out.
    fn cycle_tx_first_active(
        &mut self,
        mgr: &CfdpManager,
        t: TxnIdx,
        ran_one: &mut bool,
    ) -> CListTraverseStatus {
        if self.txn(t).flags.com.suspended {
            return CListTraverseStatus::Continue;
        }
        fw_assert!(self.txn(t).flags.com.q_index == QueueId::Txa);
        while self.current_txn.is_none() && self.txn(t).flags.com.q_index == QueueId::Txa {
            self.dispatch_tx(mgr, t);
        }
        *ran_one = true;
        CListTraverseStatus::Exit
    }

    /// C++ `tickTransactions`: tick RX, then TXW (timers), then TXW (NAK
    /// responses), repeating a pass while a tick asks to continue. Running
    /// out of PDUs leaves the tick type where it stopped (except during NAK
    /// responses), so the next cycle resumes there.
    pub fn tick_transactions(&mut self, mgr: &CfdpManager) {
        let mut reset = true;
        fw_assert!(
            self.tick_type < TICK_TYPE_NUM_TYPES,
            i32::from(self.tick_type)
        );
        while self.tick_type < TICK_TYPE_NUM_TYPES {
            let (tick_type, q) = match self.tick_type {
                0 => (TickType::Rx, QueueId::Rx),
                1 => (TickType::TxwNorm, QueueId::Txw),
                _ => (TickType::TxwNak, QueueId::Txw),
            };
            let mut args = TickArgs {
                tick_type,
                early_exit: false,
                cont: false,
            };
            for _retry in 0..MAX_ITERATIONS {
                args.cont = false;
                let head = self.qs[q as usize];
                traverse(self, Self::txn_links, head, MAX_ITERATIONS, |c, n| {
                    c.do_tick(mgr, n, &mut args)
                });
                if args.early_exit {
                    // Out of PDUs. NAK responses must not starve the RX and
                    // TXW ticks next cycle, so only they restart from RX.
                    if self.tick_type != TickType::TxwNak as u8 {
                        reset = false;
                    }
                    break;
                }
                if !args.cont {
                    break;
                }
            }
            if !reset {
                break;
            }
            self.tick_type += 1;
        }
        if reset {
            self.tick_type = TickType::Rx as u8;
        }
    }

    /// C++ `doTick`: tick `t` unless an earlier pass stopped at another
    /// transaction; exit when this tick ran out of PDUs.
    fn do_tick(
        &mut self,
        mgr: &CfdpManager,
        t: TxnIdx,
        args: &mut TickArgs,
    ) -> CListTraverseStatus {
        let mut ret = CListTraverseStatus::Continue;
        if self.current_txn.is_none() || self.current_txn == Some(t) {
            self.current_txn = None;
            if !self.txn(t).flags.com.suspended {
                match args.tick_type {
                    TickType::Rx => self.r_tick(mgr, t, &mut args.cont),
                    TickType::TxwNorm => self.s_tick(mgr, t, &mut args.cont),
                    TickType::TxwNak => self.s_tick_nak(mgr, t, &mut args.cont),
                }
            }
            if self.current_txn.is_some() {
                ret = CListTraverseStatus::Exit;
                args.early_exit = true;
            }
        }
        ret
    }

    /// C++ `processPlaybackDirectories`: step each commanded playback and
    /// report how many are busy.
    pub fn process_playback_directories(
        &mut self,
        mgr: &CfdpManager,
        seq_num: &mut TransactionSeq,
    ) {
        let mut playback_count: u8 = 0;
        for i in 0..MAX_COMMANDED_PLAYBACK_DIRECTORIES_PER_CHAN {
            self.process_playback_directory(mgr, PbRef::Playback(i as u8), seq_num);
            if self.playback[i].busy {
                playback_count += 1;
            }
        }
        self.tlm.playback_counter = playback_count;
    }

    /// C++ `processPollingDirectories`: for each enabled poll, count down
    /// its interval while idle and start a playback of the directory when
    /// it expires (re-arming the interval), or step the running playback.
    pub fn process_polling_directories(&mut self, mgr: &CfdpManager, seq_num: &mut TransactionSeq) {
        let mut poll_count: u8 = 0;
        for i in 0..MAX_POLLING_DIR_PER_CHAN {
            if self.polldir[i].enabled != Enabled::Enabled {
                continue;
            }
            poll_count += 1;
            let pd = &self.polldir[i];
            if !pd.pb.busy && pd.pb.num_ts == 0 {
                if pd.interval_timer.get_status() == TimerStatus::Expired {
                    // The status is ignored: the initiate emits its own
                    // event and the interval is re-armed regardless.
                    let src = pd.src_dir.clone();
                    let dst = pd.dst_dir.clone();
                    let (class, priority, dest) = (pd.cfdp_class, pd.priority, pd.dest_eid);
                    let _ = self.playback_dir_initiate(
                        mgr,
                        PbRef::PollDir(i as u8),
                        src.as_bytes(),
                        dst.as_bytes(),
                        class,
                        Keep::Delete,
                        priority,
                        dest,
                    );
                    let pd = &mut self.polldir[i];
                    if pd.interval_sec > 0 {
                        pd.interval_timer.set_timer(pd.interval_sec);
                    }
                } else {
                    self.polldir[i].interval_timer.run();
                }
            } else {
                self.process_playback_directory(mgr, PbRef::PollDir(i as u8), seq_num);
            }
        }
        self.tlm.poll_counter = poll_count;
    }

    /// C++ `processPlaybackDirectory`: read directory entries and start a
    /// transmit transaction per file, up to `NumTransactionsPerPlayback` at
    /// a time; the playback stops being busy once the directory is
    /// exhausted and its transactions are done.
    pub fn process_playback_directory(
        &mut self,
        mgr: &CfdpManager,
        pb: PbRef,
        seq_num: &mut TransactionSeq,
    ) {
        while self.playback_ref(pb).diropen
            && usize::from(self.playback_ref(pb).num_ts) < NUM_TRANSACTIONS_PER_PLAYBACK
        {
            if self.playback_ref(pb).pending_file.is_empty() {
                let mut path = fprime_fw::FileNameString::new();
                let playback = self.playback_mut(pb);
                let status = playback.dir.read(&mut path);
                if status == DirStatus::NoMoreFiles {
                    playback.dir.close();
                    playback.diropen = false;
                    break;
                }
                if status != DirStatus::OpOk {
                    let src = playback.fnames.src_filename.clone();
                    mgr.log_warning_lo_playback_dir_read_failed(src.as_bytes(), status as i32);
                    let playback = self.playback_mut(pb);
                    playback.dir.close();
                    playback.diropen = false;
                    break;
                }
                // C++ parity: assigned into a MaxFilePathSize string.
                playback.pending_file.set_bytes(path.as_bytes());
            } else {
                let Some(t) = self.find_unused_transaction(Direction::Tx) else {
                    // Receive transactions use the pool too; retry next tick.
                    break;
                };
                let playback = self.playback_ref(pb);
                let (src_dir, dst_dir, pending) = (
                    playback.fnames.src_filename.clone(),
                    playback.fnames.dst_filename.clone(),
                    playback.pending_file.clone(),
                );
                let (class, keep, priority, dest) = (
                    playback.cfdp_class,
                    playback.keep,
                    playback.priority,
                    playback.dest_id,
                );
                let hist = self.hist_mut(t);
                hist.fnames.src_filename = src_dir;
                hist.fnames.src_filename.append("/");
                hist.fnames.src_filename.append_bytes(pending.as_bytes());
                hist.fnames.dst_filename = dst_dir;
                hist.fnames.dst_filename.append("/");
                hist.fnames.dst_filename.append_bytes(pending.as_bytes());
                let chan = self.channel_id;
                self.tx_file_initiate(mgr, t, class, keep, chan, priority, dest, seq_num);
                self.txn_mut(t).pb = Some(pb);
                let playback = self.playback_mut(pb);
                playback.num_ts += 1;
                playback.pending_file.clear();
            }
        }
        let playback = self.playback_mut(pb);
        if !playback.diropen && playback.num_ts == 0 {
            playback.busy = false;
        }
    }
}
