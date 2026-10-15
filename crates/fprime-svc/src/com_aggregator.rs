//! # ComAggregator — port of `Svc::ComAggregator` (active)
//!
//! C++ sources: `Svc/ComAggregator/ComAggregator.{cpp,hpp,fpp}` and its
//! `docs/sdd.md`. Spanning depends on the CCSDS idle packet
//! ([`crate::ccsds::idle_packet`]) and the TM First Header Pointer
//! constants, as upstream does.
//!
//! Aggregates downlink buffers (Space Packets) into one fixed-capacity
//! aggregate for a fixed-size framer such as the
//! [`TmFramer`](crate::ccsds::tm_framer::TmFramer). It implements the
//! `Svc.Framer` interface plus a `Svc.Sched` `timeout` port that flushes a
//! partial aggregate.
//!
//! ## The aggregation state machine
//!
//! Upstream drives everything through the FPP state machine
//! `AggregationMachine`; every input port is `sync` and only *sends a
//! signal* onto the component queue, so all actions and guards run serially
//! on the component's own thread. FPP state-machine autocoding is not
//! ported (ROADMAP item 11), so the machine is hand-written here as
//! [`AggregationState`] plus a private `dispatch_signal`, keeping the
//! FPP transition table exactly:
//!
//! ```text
//! WAIT_STATUS: fill    -> doHold
//!              status  -> IS_GOOD_STATUS
//!              timeout -> (ignored)
//! FILL:        fill    -> IS_FULL_THEN_SEND
//!              timeout -> if isNotEmpty { doSend } -> WAIT_STATUS
//!              status  -> assertNoStatus
//! IS_FULL_THEN_SEND:   isFull   ? { doSplitHold, doSend } -> WAIT_STATUS : WILL_FILL_THEN_SEND
//! WILL_FILL_THEN_SEND: willFill ? { doFill, doSend } -> WAIT_STATUS : { doFill } -> FILL
//! IS_GOOD_STATUS:      isGood   ? { doClear } -> SPAN_CHECK : { doNoteFailure } -> WAIT_STATUS
//! SPAN_CHECK:          isSpanFull ? { doSend } -> WAIT_STATUS : FILL
//! ```
//!
//! The signal message on the queue mirrors the C++ generated
//! `sendSignalStart`: `[INTERNAL_STATE_MACHINE_SIGNAL i32][port 0 i16]
//! [smId i32][signal i32][signal data]`, sent at priority 0 with the
//! default `assert` overflow policy. The `Svc.ComDataContextPair` data of a
//! `fill` signal carries the buffer as a [`BufferEscrow`] token where C++
//! serializes the `Fw::Buffer` pointer.
//!
//! ## Spanning (`configure(true)`)
//!
//! The capacity becomes the full [`AGGREGATION_SIZE`], packets that do not
//! fit are split across aggregates, residual space at send time is filled
//! with an SPP idle packet (itself spanning into the next aggregate when
//! fewer than 7 bytes remain), and the TM First Header Pointer is reported
//! in the context's `firstHeaderPointer`. Without spanning the capacity is
//! [`NON_SPANNING_CAPACITY`] so the framer can add a minimum idle packet.
//!
//! ## Deviations (documented)
//!
//! - The aggregation store is owned storage moved out on `dataOut` and back
//!   on `dataReturnIn` (the C++ atomic `m_bufferState` + member array). C++
//!   writes into the member array regardless of ownership; here a write
//!   while the store is still downstream is an `fw_assert` (it can only
//!   happen if a downstream component reports status before returning the
//!   buffer, which the communication adapter protocol forbids).
//! - The state the C++ touches only from the component thread lives behind
//!   a `Mutex` that the dispatching thread holds for a whole transition
//!   (including its port calls); it is never contended in a deployment.

use crate::ccsds::idle_packet;
use crate::ccsds::types::{AGGREGATION_SIZE, SpacePacketHeader, tm_subfields};
use fprime_comp::{
    ActiveBase, ActiveComponent, BufferEscrow, ComDataWithContextPort, ComponentDispatch,
    MsgDispatchStatus, OutputPort, PortRef, QueueFullPolicy, SchedPort, SuccessConditionPort,
    input_port_adapter, msg,
};
use fprime_config::{
    FW_COM_BUFFER_MAX_SIZE, FW_FILE_BUFFER_MAX_SIZE, FwEnumStoreType, FwIndexType,
    FwQueuePriorityType, FwSizeType,
};
use fprime_fw::buffer::BufferStorage;
use fprime_fw::{
    Buffer, Deserialize, Endianness, ExtBuf, FrameContext, LinearBuffer, SerBuf, SerBufAny,
    Serialize, Success, fw_assert,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// Aggregation capacity without spanning
/// (`ComAggregator::NON_SPANNING_CAPACITY`): room is left for the TM
/// framer's minimum idle packet. Equals 1009.
pub const NON_SPANNING_CAPACITY: usize = AGGREGATION_SIZE - idle_packet::MIN_SIZE;

// C++ static_asserts: a full com buffer and a full file buffer Space Packet
// must fit without spanning, and the aggregate must exceed an idle packet.
const _: () =
    assert!(NON_SPANNING_CAPACITY >= FW_COM_BUFFER_MAX_SIZE + SpacePacketHeader::SERIALIZED_SIZE);
const _: () =
    assert!(NON_SPANNING_CAPACITY >= FW_FILE_BUFFER_MAX_SIZE + SpacePacketHeader::SERIALIZED_SIZE);
const _: () = assert!(AGGREGATION_SIZE > idle_packet::MIN_SIZE);

/// Sentinel: no packet header recorded in the current aggregate
/// (`ComAggregator::FHP_UNSET`).
const FHP_UNSET: u16 = 0xFFFF;

/// The generated message type for state-machine signals (the only message
/// type: every port is sync).
pub const MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL: FwEnumStoreType = 1;

/// `SmId::aggregationMachine` — the single state-machine instance.
pub const SM_ID_AGGREGATION_MACHINE: FwEnumStoreType = 0;

/// `Svc_AggregationMachine::Signal` values (the generated enum reserves 0
/// for the initial transition; signals follow in declaration order).
pub const SIGNAL_TIMEOUT: FwEnumStoreType = 1;
/// `fill: Svc.ComDataContextPair`.
pub const SIGNAL_FILL: FwEnumStoreType = 2;
/// `status: Fw.Success`.
pub const SIGNAL_STATUS: FwEnumStoreType = 3;

/// Signal priority (an instance without a `priority` clause sends at 0).
const SIGNAL_PRIORITY: FwQueuePriorityType = 0;

/// Queue message size: envelope + smId + signal + the largest signal data
/// (the `fill` pair: an 8-byte escrow token + a `FrameContext`).
pub const MSG_SIZE: FwSizeType = (msg::ENVELOPE_HEADER_SIZE
    + 2 * size_of::<FwEnumStoreType>()
    + size_of::<u64>()
    + FrameContext::SERIALIZED_SIZE) as FwSizeType;

type MsgBuffer = LinearBuffer<{ MSG_SIZE as usize }>;

/// States of `Svc.AggregationMachine` (choices are transient and never
/// observable).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AggregationState {
    /// Waiting for a com status from downstream (initial state).
    WaitStatus,
    /// Aggregation in progress.
    Fill,
}

/// A buffer with its frame context (`Svc.ComDataContextPair`).
type Pair = (Buffer, FrameContext);

/// State touched only by the component thread (the C++ plain members).
struct AggState {
    sm_state: AggregationState,
    /// `m_frameSerializer.getSize()` — bytes aggregated so far.
    fill: usize,
    /// Context for the current aggregate.
    last_context: FrameContext,
    /// Held data while waiting for a send.
    held: Option<Pair>,
    /// Bytes of the held buffer already consumed into previous aggregates.
    held_offset: usize,
    /// First Header Pointer for the current aggregate (`FHP_UNSET` if none).
    fhp: u16,
    /// Idle packet bytes spanning into the next aggregate.
    pending_idle: [u8; idle_packet::MIN_SIZE],
    /// Number of valid bytes in `pending_idle`.
    pending_idle_count: usize,
    /// Number of carried idle bytes at the start of the current aggregate.
    leading_idle_count: usize,
    /// Whether the last frame was not acknowledged.
    last_frame_lost: bool,
    /// Whether packet spanning is enabled.
    spanning: bool,
    /// Active aggregation capacity in bytes.
    capacity: usize,
}

/// `Svc::ComAggregator` — active aggregator for fixed-size downlink frames.
pub struct ComAggregator {
    /// Active core (PassiveBase + queue + task).
    pub active: ActiveBase,
    /// `dataOut` — the aggregate to the framer.
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — consumed buffers back to their sender.
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `comStatusOut` — ready-for-more status upstream.
    pub com_status_out: OutputPort<dyn SuccessConditionPort>,
    /// Escrow for buffers riding `fill` signals.
    escrow: BufferEscrow,
    /// The aggregation store: `Some` is the C++ `OWNED` state. Separate
    /// from [`AggState`] because `dataReturnIn` runs on the returning
    /// caller's thread, inside the component thread's `dataOut` call.
    store: Mutex<Option<BufferStorage>>,
    /// C++ `std::atomic<bool> m_allow_timeout`: timeouts are only queued
    /// while they would be processed (the FILL state).
    allow_timeout: AtomicBool,
    /// Configuration mirrors read by the caller-thread `dataIn` assert.
    spanning: AtomicBool,
    /// Component-thread state.
    state: Mutex<AggState>,
}

impl ComAggregator {
    /// Construct the component (spanning disabled). Create its queue with
    /// [`ComAggregator::create_queue`] before starting it.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            com_status_out: OutputPort::new(),
            escrow: BufferEscrow::new(),
            store: Mutex::new(Some(vec![0u8; AGGREGATION_SIZE].into_boxed_slice())),
            allow_timeout: AtomicBool::new(false),
            spanning: AtomicBool::new(false),
            state: Mutex::new(AggState {
                sm_state: AggregationState::WaitStatus,
                fill: 0,
                last_context: FrameContext::default(),
                held: None,
                held_offset: 0,
                fhp: FHP_UNSET,
                pending_idle: [0; idle_packet::MIN_SIZE],
                pending_idle_count: 0,
                leading_idle_count: 0,
                last_frame_lost: false,
                spanning: false,
                capacity: NON_SPANNING_CAPACITY,
            }),
        })
    }

    /// Create the component queue with `depth` messages of [`MSG_SIZE`].
    pub fn create_queue(&self, depth: FwSizeType) {
        self.active.queued.create_queue(depth, MSG_SIZE);
    }

    /// C++ `configure(spanningEnabled)`: enable CCSDS TM packet spanning.
    /// Must be called before any data is aggregated (asserted). With
    /// spanning, every header offset must be representable as an 11-bit
    /// First Header Pointer distinct from the reserved values.
    pub fn configure(&self, spanning_enabled: bool) {
        let mut st = self.state.lock().unwrap();
        fw_assert!(st.fill == 0, st.fill as i32);
        if spanning_enabled {
            fw_assert!(
                AGGREGATION_SIZE <= usize::from(tm_subfields::FHP_IDLE_DATA_ONLY),
                AGGREGATION_SIZE as i32
            );
        }
        st.spanning = spanning_enabled;
        st.capacity = if spanning_enabled {
            AGGREGATION_SIZE
        } else {
            NON_SPANNING_CAPACITY
        };
        self.spanning.store(spanning_enabled, Ordering::Relaxed);
    }

    /// The current state-machine state (the generated
    /// `aggregationMachine_getState`).
    pub fn aggregation_state(&self) -> AggregationState {
        self.state.lock().unwrap().sm_state
    }

    /// Bytes aggregated so far (`m_frameSerializer.getSize()`).
    pub fn aggregate_size(&self) -> usize {
        self.state.lock().unwrap().fill
    }

    /// The context of the current aggregate (`m_lastContext`).
    pub fn last_context(&self) -> FrameContext {
        self.state.lock().unwrap().last_context
    }

    /// True while the aggregation store is held by this component (C++
    /// `m_bufferState == OWNED`).
    pub fn owns_aggregate(&self) -> bool {
        self.store.lock().unwrap().is_some()
    }

    /// Bytes of the current aggregate (introspection; the store must be
    /// owned).
    pub fn aggregate_bytes(&self) -> Vec<u8> {
        let fill = self.aggregate_size();
        let store = self.store.lock().unwrap();
        store
            .as_ref()
            .map(|s| s[..fill].to_vec())
            .unwrap_or_default()
    }

    // -- Input ports ----------------------------------------------------------

    /// `comStatusIn` — SYNC: sends the `status` signal.
    pub fn com_status_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn SuccessConditionPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `timeout` — SYNC `Svc.Sched`: sends the `timeout` signal, but only
    /// while timeouts would be processed (FILL state).
    pub fn timeout(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(self.clone(), port_num)
    }

    fn data_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        let spanning = self.spanning.load(Ordering::Relaxed);
        fw_assert!(
            spanning || data.size() <= NON_SPANNING_CAPACITY,
            data.size() as i32
        );
        let token = self.escrow.deposit(data);
        let mut buf = Self::signal_start(SIGNAL_FILL);
        let status = buf.serialize_u64_be(token);
        fw_assert!(status.is_ok(), status as i32);
        let status = context.serialize_to(&mut buf, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        self.signal_finish(&buf);
    }

    fn data_return_in_handler(
        &self,
        _port_num: FwIndexType,
        data: Buffer,
        _context: &FrameContext,
    ) {
        // Runs on the returning caller's thread: take ownership back.
        let mut store = self.store.lock().unwrap();
        fw_assert!(store.is_none());
        fw_assert!(data.capacity() == AGGREGATION_SIZE, data.capacity() as i32);
        *store = Some(data.into_storage());
    }

    fn com_status_in_handler(&self, _port_num: FwIndexType, condition: Success) {
        let mut buf = Self::signal_start(SIGNAL_STATUS);
        let status = condition.serialize_to(&mut buf, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        self.signal_finish(&buf);
    }

    fn timeout_handler(&self, _port_num: FwIndexType, _context: u32) {
        // C++ parity: timeouts are only queued while they would be
        // processed (FILL); in WAIT_STATUS they would be discarded anyway
        // and could overflow a finite queue behind a slow downstream.
        if self.allow_timeout.load(Ordering::SeqCst) {
            let buf = Self::signal_start(SIGNAL_TIMEOUT);
            self.signal_finish(&buf);
        }
    }

    /// Generated `sendSignalStart`: the envelope, the SM id and the signal.
    fn signal_start(signal: FwEnumStoreType) -> MsgBuffer {
        let mut buf = MsgBuffer::new();
        let status =
            msg::write_envelope_header(&mut buf, MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL, 0);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_i32_be(SM_ID_AGGREGATION_MACHINE);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_i32_be(signal);
        fw_assert!(status.is_ok(), status as i32);
        buf
    }

    /// Generated `aggregationMachine_sendSignalFinish`: priority 0,
    /// non-blocking, assert on overflow.
    fn signal_finish(&self, buf: &MsgBuffer) {
        let _ = self
            .active
            .queued
            .send_message(buf, SIGNAL_PRIORITY, QueueFullPolicy::Assert);
    }

    // -- State machine --------------------------------------------------------

    /// Run one signal through the state machine (component thread).
    fn dispatch_signal(&self, signal: Signal) {
        let mut st = self.state.lock().unwrap();
        match (st.sm_state, signal) {
            (AggregationState::WaitStatus, Signal::Fill(pair)) => self.do_hold(&mut st, pair),
            (AggregationState::WaitStatus, Signal::Status(status)) => {
                // IS_GOOD_STATUS
                if status == Success::Success {
                    self.do_clear(&mut st);
                    // SPAN_CHECK
                    if st.spanning && st.fill == st.capacity {
                        self.do_send(&mut st);
                        st.sm_state = AggregationState::WaitStatus;
                    } else {
                        st.sm_state = AggregationState::Fill;
                    }
                } else {
                    st.last_frame_lost = true; // doNoteFailure
                    st.sm_state = AggregationState::WaitStatus;
                }
            }
            (AggregationState::WaitStatus, Signal::Timeout) => {}
            (AggregationState::Fill, Signal::Fill(pair)) => {
                let size = pair.0.size();
                let remaining = Self::remaining_capacity(&st);
                if remaining < size {
                    // IS_FULL_THEN_SEND: isFull
                    self.do_split_hold(&mut st, pair);
                    self.do_send(&mut st);
                    st.sm_state = AggregationState::WaitStatus;
                } else if remaining == size {
                    // WILL_FILL_THEN_SEND: willFill
                    self.do_fill(&mut st, pair);
                    self.do_send(&mut st);
                    st.sm_state = AggregationState::WaitStatus;
                } else {
                    self.do_fill(&mut st, pair);
                    st.sm_state = AggregationState::Fill;
                }
            }
            (AggregationState::Fill, Signal::Timeout) => {
                // Carried-over idle bytes are not payload: an aggregate
                // holding only those is empty for timeout purposes.
                if st.fill > st.leading_idle_count {
                    self.do_send(&mut st);
                    st.sm_state = AggregationState::WaitStatus;
                }
            }
            (AggregationState::Fill, Signal::Status(_)) => {
                // assertNoStatus: status is not possible while filling.
                fw_assert!(false);
            }
        }
    }

    /// `doClear`: clear the fill state, carry spanned idle bytes and refill
    /// from the held buffer.
    fn do_clear(&self, st: &mut AggState) {
        self.allow_timeout.store(true, Ordering::SeqCst);
        st.fill = 0;
        st.last_context = FrameContext::default();
        st.fhp = FHP_UNSET;
        self.drop_lost_frame_state(st);
        st.leading_idle_count = st.pending_idle_count;
        if st.pending_idle_count > 0 {
            let pending = st.pending_idle;
            let count = st.pending_idle_count;
            self.write(st, &pending[..count]);
            st.pending_idle_count = 0;
        }
        self.fill_from_held(st);
    }

    /// `doFill`: append a whole buffer and return it.
    fn do_fill(&self, st: &mut AggState, pair: Pair) {
        Self::mark_first_header_if_unset(st);
        self.write(st, pair.0.data());
        st.last_context = pair.1;
        self.return_and_signal_ready(pair);
    }

    /// `doSend`: emit the aggregate (only when it is non-empty).
    fn do_send(&self, st: &mut AggState) {
        if st.fill == 0 {
            return;
        }
        fw_assert!(st.fill <= st.capacity, st.fill as i32);
        if st.spanning {
            self.fill_residual_with_idle(st);
            st.last_context.first_header_pointer = if st.fhp == FHP_UNSET {
                tm_subfields::FHP_NO_PACKET_START
            } else {
                st.fhp
            };
        }
        let storage = self.store.lock().unwrap().take();
        fw_assert!(storage.is_some());
        let Some(storage) = storage else { return };
        let mut frame = Buffer::from_storage(storage, Buffer::NO_CONTEXT);
        frame.set_size(st.fill);
        // Timeouts are discarded in WAIT_STATUS.
        self.allow_timeout.store(false, Ordering::SeqCst);
        let context = st.last_context;
        let p = self.data_out.get();
        p.target.invoke(p.port_num, frame, &context);
    }

    /// `doHold`: hold a buffer while waiting for status.
    fn do_hold(&self, st: &mut AggState, pair: Pair) {
        fw_assert!(st.held.is_none());
        st.held = Some(pair);
        st.held_offset = 0;
    }

    /// `doSplitHold`: hold a buffer, first splitting its leading bytes into
    /// the remaining space when spanning.
    fn do_split_hold(&self, st: &mut AggState, pair: Pair) {
        self.do_hold(st, pair);
        if st.spanning {
            let remaining = Self::remaining_capacity(st);
            if remaining > 0 {
                // The held packet's header starts at the current fill offset.
                Self::mark_first_header_if_unset(st);
                self.copy_from_held(st, 0, remaining);
                st.held_offset = remaining;
                if let Some((_, context)) = st.held.as_ref() {
                    st.last_context = *context;
                }
            }
        }
    }

    fn remaining_capacity(st: &AggState) -> usize {
        fw_assert!(st.fill <= st.capacity, st.fill as i32);
        st.capacity - st.fill
    }

    fn mark_first_header_if_unset(st: &mut AggState) {
        if st.spanning && st.fhp == FHP_UNSET {
            st.fhp = st.fill as u16;
        }
    }

    /// Fill from the held (possibly partially consumed) buffer, returning
    /// it when fully consumed.
    fn fill_from_held(&self, st: &mut AggState) {
        let (held_size, context) = match st.held.as_ref() {
            Some((held, context)) => (held.size(), *context),
            None => return,
        };
        let held_remaining = held_size - st.held_offset;
        let fill_size = Self::remaining_capacity(st).min(held_remaining);
        let start = st.held_offset;
        if start == 0 {
            // The held packet's header starts at the current fill offset.
            Self::mark_first_header_if_unset(st);
        }
        self.copy_from_held(st, start, fill_size);
        st.last_context = context;
        st.held_offset += fill_size;
        if st.held_offset == held_size {
            if let Some(pair) = st.held.take() {
                self.return_and_signal_ready(pair);
            }
            st.held_offset = 0;
        }
    }

    /// Fill the residual space with an SPP idle packet, spanning a minimum
    /// idle packet into the next aggregate when the residual is too small
    /// (CCSDS 132.0-B-3 4.1.4).
    fn fill_residual_with_idle(&self, st: &mut AggState) {
        let residual = Self::remaining_capacity(st);
        if residual == 0 {
            return;
        }
        // The idle packet's header starts at the current fill offset.
        Self::mark_first_header_if_unset(st);
        if residual >= idle_packet::MIN_SIZE {
            let mut store = self.store.lock().unwrap();
            fw_assert!(store.is_some());
            if let Some(storage) = store.as_mut() {
                let mut ser = ExtBuf::new(&mut storage[st.fill..]);
                let status = idle_packet::serialize(&mut ser, residual);
                fw_assert!(status.is_ok(), status as i32);
            }
            drop(store);
            st.fill += residual;
        } else {
            // Stage a minimum-size idle packet, emit the leading bytes now
            // and span the rest.
            let mut staging = [0u8; idle_packet::MIN_SIZE];
            {
                let mut stager = ExtBuf::new(&mut staging);
                let status = idle_packet::serialize(&mut stager, idle_packet::MIN_SIZE);
                fw_assert!(status.is_ok(), status as i32);
            }
            self.write(st, &staging[..residual]);
            st.pending_idle_count = idle_packet::MIN_SIZE - residual;
            st.pending_idle[..st.pending_idle_count].copy_from_slice(&staging[residual..]);
        }
    }

    /// Recover from FAILURE on the next SUCCESS: drop carried idle bytes and
    /// a held remainder whose head was in the lost frame.
    fn drop_lost_frame_state(&self, st: &mut AggState) {
        if st.last_frame_lost {
            st.pending_idle_count = 0;
            if st.held.is_some() && st.held_offset > 0 {
                if let Some(pair) = st.held.take() {
                    self.return_and_signal_ready(pair);
                }
                st.held_offset = 0;
            }
            st.last_frame_lost = false;
        }
    }

    /// Return a consumed buffer and signal readiness for another.
    fn return_and_signal_ready(&self, pair: Pair) {
        let (buffer, context) = pair;
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, buffer, &context);
        let mut good = Success::Success;
        let p = self.com_status_out.get();
        p.target.invoke(p.port_num, &mut good);
    }

    /// Append `held[start..start + len]` to the aggregate without a
    /// temporary copy (the held buffer and the store are distinct).
    fn copy_from_held(&self, st: &mut AggState, start: usize, len: usize) {
        let mut store = self.store.lock().unwrap();
        fw_assert!(store.is_some());
        if let (Some(storage), Some((held, _))) = (store.as_mut(), st.held.as_ref()) {
            fw_assert!(st.fill + len <= storage.len(), st.fill as i32);
            storage[st.fill..st.fill + len].copy_from_slice(&held.data()[start..start + len]);
        }
        drop(store);
        st.fill += len;
    }

    /// Append raw bytes to the aggregate (the C++ `serializeFrom(...,
    /// OMIT_LENGTH)` + `FW_ASSERT(status == OK)`).
    fn write(&self, st: &mut AggState, bytes: &[u8]) {
        let mut store = self.store.lock().unwrap();
        // See the module docs: the store must be home before it is written.
        fw_assert!(store.is_some());
        if let Some(storage) = store.as_mut() {
            fw_assert!(st.fill + bytes.len() <= storage.len(), st.fill as i32);
            storage[st.fill..st.fill + bytes.len()].copy_from_slice(bytes);
            st.fill += bytes.len();
        }
    }
}

/// A decoded state-machine signal.
enum Signal {
    Timeout,
    Fill(Pair),
    Status(Success),
}

impl SuccessConditionPort for ComAggregator {
    fn invoke(&self, port_num: FwIndexType, condition: &mut Success) {
        self.com_status_in_handler(port_num, *condition);
    }
}

impl SchedPort for ComAggregator {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        self.timeout_handler(port_num, context);
    }
}

input_port_adapter! {
    /// `dataIn` — SYNC `Svc.ComDataWithContext`: sends the `fill` signal.
    component: ComAggregator;
    adapter: DataInAdapter;
    port: ComDataWithContextPort;
    input: pub data_in;
    handler: data_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

input_port_adapter! {
    /// `dataReturnIn` — SYNC: the aggregate coming back from the framer.
    component: ComAggregator;
    adapter: DataReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub data_return_in;
    handler: data_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

impl ComponentDispatch for ComAggregator {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        if msg_type != MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL {
            return MsgDispatchStatus::Error;
        }
        let mut sm_id: FwEnumStoreType = 0;
        let mut signal: FwEnumStoreType = 0;
        if !buf.deserialize_i32_be(&mut sm_id).is_ok()
            || !buf.deserialize_i32_be(&mut signal).is_ok()
        {
            return MsgDispatchStatus::Error;
        }
        // C++ parity: the generated dispatch asserts a known SM id.
        fw_assert!(sm_id == SM_ID_AGGREGATION_MACHINE, sm_id);
        let signal = match signal {
            SIGNAL_TIMEOUT => Signal::Timeout,
            SIGNAL_FILL => {
                let mut token = 0u64;
                let mut context = FrameContext::default();
                if !buf.deserialize_u64_be(&mut token).is_ok()
                    || !context.deserialize_from(buf, Endianness::Big).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                Signal::Fill((self.escrow.claim(token), context))
            }
            SIGNAL_STATUS => {
                let mut condition = Success::Failure;
                if !condition.deserialize_from(buf, Endianness::Big).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                Signal::Status(condition)
            }
            other => {
                fw_assert!(false, other);
                return MsgDispatchStatus::Error;
            }
        };
        self.dispatch_signal(signal);
        MsgDispatchStatus::Ok
    }

    /// C++ `preamble()`: announce readiness upstream.
    fn preamble(&self) {
        let mut good = Success::Success;
        let p = self.com_status_out.get();
        p.target.invoke(p.port_num, &mut good);
    }
}

impl ActiveComponent for ComAggregator {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

// ---------------------------------------------------------------------------
// Tests — port of ComAggregatorTester (+ Rules/Spanning.cpp)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_os::queue::BlockingType;
    use std::collections::VecDeque;
    use std::sync::Mutex as StdMutex;

    /// Queue depth supplied to the instance under test.
    const TEST_INSTANCE_QUEUE_DEPTH: usize = 20;

    /// Deterministic stand-in for `STest::Pick` (xorshift64*).
    struct Pick(u64);
    impl Pick {
        fn lower_upper(&mut self, lower: u64, upper: u64) -> u64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            let r = self.0.wrapping_mul(0x2545_F491_4F6C_DD1D);
            lower + r % (upper - lower + 1)
        }
    }

    /// One recorded output: (bytes, context, buffer identity).
    type Rec = (Vec<u8>, FrameContext, u32);

    #[derive(Default)]
    struct Harness {
        data_out: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        /// Emitted aggregates still held by the harness (for return).
        frames: StdMutex<Vec<Buffer>>,
        data_return: StdMutex<Vec<Rec>>,
        status: StdMutex<Vec<Success>>,
    }

    impl ComDataWithContextPort for Harness {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            if port_num == 0 {
                self.data_out
                    .lock()
                    .unwrap()
                    .push((data.data().to_vec(), *context));
                self.frames.lock().unwrap().push(data);
            } else {
                self.data_return.lock().unwrap().push((
                    data.data().to_vec(),
                    *context,
                    data.context(),
                ));
            }
        }
    }

    impl SuccessConditionPort for Harness {
        fn invoke(&self, _port_num: FwIndexType, condition: &mut Success) {
            self.status.lock().unwrap().push(*condition);
        }
    }

    impl Harness {
        fn clear(&self) {
            self.data_out.lock().unwrap().clear();
            self.data_return.lock().unwrap().clear();
            self.status.lock().unwrap().clear();
        }
        fn n_out(&self) -> usize {
            self.data_out.lock().unwrap().len()
        }
        fn n_ret(&self) -> usize {
            self.data_return.lock().unwrap().len()
        }
        fn n_status(&self) -> usize {
            self.status.lock().unwrap().len()
        }
        fn out(&self, i: usize) -> (Vec<u8>, FrameContext) {
            self.data_out.lock().unwrap()[i].clone()
        }
    }

    struct Tester {
        comp: Arc<ComAggregator>,
        h: Arc<Harness>,
        pick: Pick,
        /// Shadow aggregation for validation.
        aggregation: Vec<u8>,
        /// Buffer identity counter (carried in the buffer context).
        next_id: u32,
        // Spanning shadow model (Rules/Spanning.cpp).
        stream: Vec<u8>,
        headers: Vec<usize>,
        unreturned: VecDeque<u32>,
        outstanding: bool,
        failure_pending: bool,
        held_pending: bool,
        held_consumed: bool,
        leading_idle_count: usize,
    }

    impl Tester {
        fn new(seed: u64) -> Self {
            let comp = ComAggregator::new("ComAggregator");
            comp.create_queue(TEST_INSTANCE_QUEUE_DEPTH as FwSizeType);
            let h = Arc::new(Harness::default());
            comp.data_out.connect(h.clone(), 0);
            comp.data_return_out.connect(h.clone(), 1);
            comp.com_status_out.connect(h.clone(), 0);
            Self {
                comp,
                h,
                pick: Pick(seed | 1),
                aggregation: Vec::new(),
                next_id: 0,
                stream: Vec::new(),
                headers: Vec::new(),
                unreturned: VecDeque::new(),
                outstanding: false,
                failure_pending: false,
                held_pending: false,
                held_consumed: false,
                leading_idle_count: 0,
            }
        }

        fn dispatch_one(&self) -> MsgDispatchStatus {
            self.comp
                .active
                .queued
                .do_dispatch(&*self.comp, BlockingType::NonBlocking)
        }

        fn messages_available(&self) -> usize {
            self.comp.active.queued.queue().get_messages_available() as usize
        }

        fn fill_buffer(&mut self, size: usize) -> Buffer {
            assert!(size > 0);
            let mut b = Buffer::allocate(size);
            for byte in b.data_mut() {
                *byte = self.pick.lower_upper(0, 255) as u8;
            }
            self.next_id += 1;
            b.set_context(self.next_id);
            b
        }

        fn invoke_data_in(&self, buffer: Buffer, context: &FrameContext) {
            let p = self.comp.data_in(0);
            p.target.invoke(p.port_num, buffer, context);
        }

        fn invoke_status(&self, condition: Success) {
            let p = self.comp.com_status_in(0);
            let mut c = condition;
            p.target.invoke(p.port_num, &mut c);
        }

        fn invoke_timeout(&self) {
            let p = self.comp.timeout(0);
            p.target.invoke(p.port_num, 0);
        }

        /// Return the oldest emitted aggregate (C++ `invoke_to_dataReturnIn`
        /// with the dataOut history entry).
        fn return_frame(&self) {
            let frame = self.h.frames.lock().unwrap().remove(0);
            let p = self.comp.data_return_in(0);
            p.target.invoke(p.port_num, frame, &FrameContext::default());
        }

        fn validate_aggregation(&self, bytes: &[u8]) {
            assert_eq!(bytes, &self.aggregation[..]);
        }

        fn validate_buffer_aggregated(&mut self, data: &[u8], context: &FrameContext) {
            let agg = self.comp.aggregate_bytes();
            let start = agg.len() - data.len();
            assert_eq!(&agg[start..], data);
            assert_eq!(*context, self.comp.last_context());
            self.aggregation.extend_from_slice(data);
        }

        // -- Nominal tests ------------------------------------------------------

        fn test_initial(&mut self) {
            assert_eq!(self.comp.aggregate_size(), 0);
            assert!(self.comp.owns_aggregate());
            self.comp.preamble();
            assert_eq!(*self.h.status.lock().unwrap(), vec![Success::Success]);
            self.invoke_status(Success::Success);
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            assert_eq!(self.messages_available(), 0);
        }

        fn test_fill(&mut self, expect_hold: bool) -> Option<(Vec<u8>, FrameContext)> {
            let original = self.comp.aggregate_size();
            let max_fill =
                NON_SPANNING_CAPACITY - original - usize::from(original != NON_SPANNING_CAPACITY);
            if max_fill == 0 {
                return None;
            }
            let len = self.pick.lower_upper(1, max_fill as u64) as usize;
            let buffer = self.fill_buffer(len);
            let bytes = buffer.data().to_vec();
            let context = FrameContext::default();
            assert_eq!(self.messages_available(), 0);
            self.invoke_data_in(buffer, &context);
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            assert_eq!(self.messages_available(), 0);
            if expect_hold {
                assert_eq!(self.comp.aggregate_size(), original);
            } else {
                assert_eq!(self.comp.aggregate_size(), original + len);
                self.validate_buffer_aggregated(&bytes, &context);
            }
            self.h.clear();
            Some((bytes, context))
        }

        fn test_fill_multi(&mut self) {
            let count = self.pick.lower_upper(1, 5);
            for _ in 0..count {
                let _ = self.test_fill(false);
            }
        }

        /// Several FAILURE statuses change nothing.
        fn some_failures(&mut self) {
            let n = self.pick.lower_upper(1, 5);
            for _ in 0..n {
                self.invoke_status(Success::Failure);
                assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
                assert_eq!(self.h.n_out(), 1);
            }
        }

        fn return_and_good(&mut self) {
            self.return_frame();
            self.invoke_status(Success::Success);
            self.aggregation.clear();
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
        }

        fn test_full(&mut self) {
            let original = self.comp.aggregate_size();
            let len = self.pick.lower_upper(
                (NON_SPANNING_CAPACITY - original + 1) as u64,
                NON_SPANNING_CAPACITY as u64,
            ) as usize;
            let buffer = self.fill_buffer(len);
            let bytes = buffer.data().to_vec();
            let context = FrameContext::default();
            self.invoke_data_in(buffer, &context);
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            assert_eq!(self.h.n_out(), 1);
            self.validate_aggregation(&self.h.out(0).0);
            self.some_failures();
            self.return_and_good();
            self.validate_buffer_aggregated(&bytes, &context);
            self.h.clear();
        }

        fn test_exactly_full(&mut self) {
            let original = self.comp.aggregate_size();
            let buffer = self.fill_buffer(NON_SPANNING_CAPACITY - original);
            let bytes = buffer.data().to_vec();
            let context = FrameContext::default();
            self.invoke_data_in(buffer, &context);
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            assert_eq!(self.h.n_out(), 1);
            // The sent aggregate ends with the buffer and matches the shadow.
            self.aggregation.extend_from_slice(&bytes);
            self.validate_aggregation(&self.h.out(0).0);
            self.some_failures();
            self.return_and_good();
            assert_eq!(self.comp.aggregate_size(), 0);
            self.h.clear();
        }

        fn test_timeout(&mut self) {
            self.invoke_timeout();
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            assert_eq!(self.h.n_out(), 1);
            self.validate_aggregation(&self.h.out(0).0);
            self.some_failures();
            self.return_and_good();
            self.h.clear();
        }

        fn test_timeout_overflow_prevention(&mut self) {
            assert_eq!(self.messages_available(), 0);
            self.invoke_timeout();
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            assert_eq!(self.h.n_out(), 1);
            self.validate_aggregation(&self.h.out(0).0);
            self.some_failures();
            // Enough extra timeouts to overflow the queue if they were queued.
            let n = self.pick.lower_upper(1, TEST_INSTANCE_QUEUE_DEPTH as u64)
                + TEST_INSTANCE_QUEUE_DEPTH as u64;
            for _ in 0..n {
                self.invoke_timeout();
                assert_eq!(self.h.n_out(), 1);
            }
            assert_eq!(self.messages_available(), 0);
            self.return_and_good();
            self.h.clear();
        }

        fn test_timeout_zero(&mut self) {
            self.invoke_timeout();
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            assert_eq!(self.h.n_out(), 0);
            self.h.clear();
        }

        fn test_hold_while_waiting(&mut self) {
            self.invoke_timeout();
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            assert_eq!(self.h.n_out(), 1);
            self.validate_aggregation(&self.h.out(0).0);
            self.some_failures();
            // Force a hold.
            let (minor, context) = self.test_fill(true).unwrap();
            self.return_and_good();
            self.validate_buffer_aggregated(&minor, &context);
            self.h.clear();
        }

        // -- Spanning helpers ---------------------------------------------------

        fn append_idle_packet(expected: &mut Vec<u8>, idle_size: usize) {
            assert!(idle_size >= idle_packet::MIN_SIZE);
            expected.extend_from_slice(&[0x07, 0xFF, 0xC0, 0x00]);
            let token = (idle_size - SpacePacketHeader::SERIALIZED_SIZE - 1) as u16;
            expected.extend_from_slice(&token.to_be_bytes());
            expected.extend(std::iter::repeat_n(
                idle_packet::DATA_PATTERN,
                idle_size - SpacePacketHeader::SERIALIZED_SIZE,
            ));
        }

        fn spanning_send(&mut self, buffer: Buffer) {
            self.invoke_data_in(buffer, &FrameContext::default());
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
        }

        fn expect_frame(&self, index: usize, expected: &[u8], expected_fhp: u16) {
            let (frame, context) = self.h.out(index);
            assert_eq!(frame.len(), AGGREGATION_SIZE);
            assert_eq!(frame, expected);
            assert_eq!(context.first_header_pointer, expected_fhp);
        }

        fn return_and_status(&self) {
            self.return_frame();
            self.invoke_status(Success::Success);
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
        }

        // -- Spanning rule-based testing (Rules/Spanning.cpp) -------------------

        fn shadow_send_packet(&mut self) {
            let size = self.pick.lower_upper(1, (2 * AGGREGATION_SIZE + 64) as u64) as usize;
            let packet = self.fill_buffer(size);
            self.unreturned.push_back(packet.context());
            self.headers.push(self.stream.len());
            self.stream.extend_from_slice(packet.data());
            self.spanning_send(packet);
        }

        fn shadow_emit(&mut self) {
            assert_eq!(self.h.n_out(), 1);
            let (frame, context) = self.h.out(0);
            assert!(self.stream.len() >= AGGREGATION_SIZE);
            assert_eq!(frame.len(), AGGREGATION_SIZE);
            assert_eq!(&frame[..], &self.stream[..AGGREGATION_SIZE]);
            let has_header = self.headers.first().is_some_and(|&h| h < AGGREGATION_SIZE);
            let expected_fhp = if has_header {
                self.headers[0] as u16
            } else {
                tm_subfields::FHP_NO_PACKET_START
            };
            assert_eq!(context.first_header_pointer, expected_fhp);
            self.stream.drain(..AGGREGATION_SIZE);
            self.leading_idle_count = self.leading_idle_count.saturating_sub(AGGREGATION_SIZE);
            self.headers = self
                .headers
                .iter()
                .filter(|&&o| o >= AGGREGATION_SIZE)
                .map(|&o| o - AGGREGATION_SIZE)
                .collect();
            self.outstanding = true;
        }

        fn shadow_expect_return(&mut self, index: usize) {
            let returned = self.h.data_return.lock().unwrap()[index].2;
            let expected = self.unreturned.pop_front().unwrap();
            assert_eq!(returned, expected);
        }

        fn rule_send_packet(&mut self) {
            self.h.clear();
            self.shadow_send_packet();
            if self.stream.len() < AGGREGATION_SIZE {
                assert_eq!(self.h.n_out(), 0);
                assert_eq!(self.h.n_ret(), 1);
                assert_eq!(self.h.n_status(), 1);
                self.shadow_expect_return(0);
                assert_eq!(self.comp.aggregate_size(), self.stream.len());
            } else if self.stream.len() == AGGREGATION_SIZE {
                assert_eq!(self.h.n_ret(), 1);
                assert_eq!(self.h.n_status(), 1);
                self.shadow_expect_return(0);
                self.shadow_emit();
            } else {
                assert_eq!(self.h.n_ret(), 0);
                assert_eq!(self.h.n_status(), 0);
                self.held_pending = true;
                self.held_consumed = true;
                self.shadow_emit();
            }
        }

        fn rule_send_packet_while_waiting(&mut self) {
            self.h.clear();
            self.shadow_send_packet();
            assert_eq!(self.h.n_out(), 0);
            assert_eq!(self.h.n_ret(), 0);
            assert_eq!(self.h.n_status(), 0);
            self.held_pending = true;
            self.held_consumed = false;
        }

        fn rule_timeout(&mut self) {
            self.h.clear();
            self.invoke_timeout();
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            if self.stream.is_empty()
                || (!self.held_pending && self.stream.len() == self.leading_idle_count)
            {
                assert_eq!(self.h.n_out(), 0);
                assert_eq!(self.h.n_ret(), 0);
                assert_eq!(self.h.n_status(), 0);
                return;
            }
            let residual = AGGREGATION_SIZE - self.stream.len();
            let idle_size = residual.max(idle_packet::MIN_SIZE);
            self.headers.push(self.stream.len());
            Self::append_idle_packet(&mut self.stream, idle_size);
            let idle_spans = idle_size > residual;
            assert_eq!(self.h.n_ret(), 0);
            assert_eq!(self.h.n_status(), 0);
            self.shadow_emit();
            self.leading_idle_count = if idle_spans { self.stream.len() } else { 0 };
        }

        fn rule_status_failure(&mut self) {
            self.h.clear();
            self.invoke_status(Success::Failure);
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            assert_eq!(self.h.n_out(), 0);
            assert_eq!(self.h.n_ret(), 0);
            assert_eq!(self.h.n_status(), 0);
            self.failure_pending = true;
        }

        fn rule_return_and_status(&mut self) {
            self.h.clear();
            self.return_frame();
            self.invoke_status(Success::Success);
            assert_eq!(self.dispatch_one(), MsgDispatchStatus::Ok);
            self.outstanding = false;

            if self.failure_pending {
                if self.held_pending && self.held_consumed {
                    assert_eq!(self.h.n_ret(), 1);
                    assert_eq!(self.h.n_status(), 1);
                    self.shadow_expect_return(0);
                    self.held_pending = false;
                    self.held_consumed = false;
                    assert_eq!(self.h.n_out(), 0);
                    self.stream.clear();
                    self.headers.clear();
                    self.leading_idle_count = 0;
                } else if self.held_pending {
                    if self.leading_idle_count > 0 {
                        self.stream.drain(..self.leading_idle_count);
                        for offset in &mut self.headers {
                            *offset -= self.leading_idle_count;
                        }
                        self.leading_idle_count = 0;
                    }
                    if self.stream.len() <= AGGREGATION_SIZE {
                        assert_eq!(self.h.n_ret(), 1);
                        assert_eq!(self.h.n_status(), 1);
                        self.shadow_expect_return(0);
                        self.held_pending = false;
                        self.held_consumed = false;
                        if self.stream.len() == AGGREGATION_SIZE {
                            self.shadow_emit();
                        } else {
                            assert_eq!(self.h.n_out(), 0);
                        }
                    } else {
                        assert_eq!(self.h.n_ret(), 0);
                        assert_eq!(self.h.n_status(), 0);
                        self.held_consumed = true;
                        self.shadow_emit();
                        self.outstanding = true;
                    }
                } else {
                    assert_eq!(self.h.n_ret(), 0);
                    assert_eq!(self.h.n_status(), 0);
                    assert_eq!(self.h.n_out(), 0);
                    self.stream.clear();
                    self.headers.clear();
                    self.leading_idle_count = 0;
                }
                self.failure_pending = false;
                return;
            }

            if self.held_pending && self.stream.len() <= AGGREGATION_SIZE {
                assert_eq!(self.h.n_ret(), 1);
                assert_eq!(self.h.n_status(), 1);
                self.shadow_expect_return(0);
                self.held_pending = false;
                self.held_consumed = false;
            } else {
                assert_eq!(self.h.n_ret(), 0);
                assert_eq!(self.h.n_status(), 0);
            }
            if self.stream.len() >= AGGREGATION_SIZE {
                self.held_consumed = true;
                self.shadow_emit();
            } else {
                assert_eq!(self.h.n_out(), 0);
                assert_eq!(self.comp.aggregate_size(), self.stream.len());
            }
        }
    }

    // -- Nominal ------------------------------------------------------------------

    #[test]
    fn nominal_initial() {
        let mut t = Tester::new(1);
        t.test_initial();
        assert_eq!(t.comp.aggregation_state(), AggregationState::Fill);
    }

    #[test]
    fn nominal_fill() {
        let mut t = Tester::new(2);
        t.test_initial();
        let _ = t.test_fill(false);
    }

    #[test]
    fn nominal_multi_fill() {
        let mut t = Tester::new(3);
        t.test_initial();
        t.test_fill_multi();
    }

    #[test]
    fn nominal_full() {
        for seed in 0..20 {
            let mut t = Tester::new(seed);
            t.test_initial();
            t.test_fill_multi();
            t.test_full();
        }
    }

    #[test]
    fn nominal_exactly_full() {
        for seed in 0..20 {
            let mut t = Tester::new(seed);
            t.test_initial();
            t.test_fill_multi();
            t.test_exactly_full();
        }
    }

    #[test]
    fn nominal_timeout() {
        let mut t = Tester::new(4);
        t.test_initial();
        t.test_fill_multi();
        t.test_timeout();
    }

    #[test]
    fn off_nominal_timeout_empty() {
        let mut t = Tester::new(5);
        t.test_initial();
        t.test_timeout_zero();
        t.test_fill_multi();
        t.test_full();
    }

    #[test]
    fn off_nominal_timeout_overflow_prevention() {
        let mut t = Tester::new(6);
        t.test_initial();
        t.test_fill_multi();
        t.test_timeout_overflow_prevention();
        // Normal operation resumes.
        t.test_fill_multi();
        t.test_timeout();
    }

    #[test]
    fn nominal_hold_while_waiting() {
        let mut t = Tester::new(7);
        t.test_initial();
        t.test_fill_multi();
        t.test_hold_while_waiting();
    }

    #[test]
    fn nominal_clear() {
        let mut t = Tester::new(8);
        t.test_initial();
        t.test_fill_multi();
        t.test_full();
        t.test_fill_multi();
        t.test_timeout();
        t.test_fill_multi();
        t.test_full();
    }

    /// Without spanning the context is the last buffer's, unmodified (no
    /// First Header Pointer is reported).
    #[test]
    fn non_spanning_context_is_the_last_buffers() {
        let mut t = Tester::new(9);
        t.test_initial();
        let ctx = FrameContext {
            apid: fprime_fw::Apid::FwPacketLog,
            vc_id: 3,
            ..FrameContext::default()
        };
        let b = t.fill_buffer(10);
        t.invoke_data_in(b, &ctx);
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        t.invoke_timeout();
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        assert_eq!(t.h.out(0).1, ctx);
        assert_eq!(t.h.out(0).0.len(), 10);
    }

    // -- Spanning -------------------------------------------------------------------

    #[test]
    fn spanning_split_across_two_frames() {
        const CAP: usize = AGGREGATION_SIZE;
        const FIRST_SIZE: usize = 100;
        const SPAN_SIZE: usize = CAP + 184;
        const FRAME1_PORTION: usize = CAP - FIRST_SIZE;
        const REMAINDER: usize = SPAN_SIZE - FRAME1_PORTION;
        let mut t = Tester::new(10);
        t.comp.configure(true);
        t.test_initial();
        t.h.clear();

        let first = t.fill_buffer(FIRST_SIZE);
        let first_bytes = first.data().to_vec();
        t.spanning_send(first);
        assert_eq!(t.h.n_out(), 0);
        assert_eq!(t.h.n_ret(), 1); // whole packet returned immediately
        assert_eq!(t.h.n_status(), 1);
        let span = t.fill_buffer(SPAN_SIZE);
        let span_bytes = span.data().to_vec();
        t.spanning_send(span);

        assert_eq!(t.h.n_out(), 1);
        assert_eq!(t.h.n_ret(), 1); // spanning packet retained
        assert_eq!(t.h.n_status(), 1);
        let mut expected1 = first_bytes.clone();
        expected1.extend_from_slice(&span_bytes[..FRAME1_PORTION]);
        t.expect_frame(0, &expected1, 0);
        t.return_and_status();
        assert_eq!(t.h.n_ret(), 2); // remainder consumed: packet returned
        assert_eq!(t.h.n_status(), 2);

        t.invoke_timeout();
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        assert_eq!(t.h.n_out(), 2);
        let mut expected2 = span_bytes[FRAME1_PORTION..].to_vec();
        Tester::append_idle_packet(&mut expected2, CAP - REMAINDER);
        t.expect_frame(1, &expected2, REMAINDER as u16);
        t.return_and_status();
        assert_eq!(t.h.n_status(), 2);
    }

    #[test]
    fn spanning_complete_middle_frame() {
        const CAP: usize = AGGREGATION_SIZE;
        const FIRST_SIZE: usize = 200;
        const TAIL: usize = 300;
        const SPAN_SIZE: usize = (CAP - FIRST_SIZE) + CAP + TAIL;
        let mut t = Tester::new(11);
        t.comp.configure(true);
        t.test_initial();
        t.h.clear();

        let first = t.fill_buffer(FIRST_SIZE);
        let first_bytes = first.data().to_vec();
        t.spanning_send(first);
        assert_eq!(t.h.n_status(), 1);
        let span = t.fill_buffer(SPAN_SIZE);
        let span_bytes = span.data().to_vec();
        t.spanning_send(span);

        assert_eq!(t.h.n_out(), 1);
        let mut expected1 = first_bytes.clone();
        expected1.extend_from_slice(&span_bytes[..CAP - FIRST_SIZE]);
        t.expect_frame(0, &expected1, 0);
        t.return_and_status();
        assert_eq!(t.h.n_status(), 1);

        // Frame 2: sent immediately on good status, continuation only.
        assert_eq!(t.h.n_out(), 2);
        assert_eq!(t.h.n_ret(), 1);
        let expected2 = span_bytes[CAP - FIRST_SIZE..2 * CAP - FIRST_SIZE].to_vec();
        t.expect_frame(1, &expected2, tm_subfields::FHP_NO_PACKET_START);
        t.return_and_status();
        assert_eq!(t.h.n_ret(), 2);
        assert_eq!(t.h.n_status(), 2);

        t.invoke_timeout();
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        assert_eq!(t.h.n_out(), 3);
        let mut expected3 = span_bytes[2 * CAP - FIRST_SIZE..].to_vec();
        Tester::append_idle_packet(&mut expected3, CAP - TAIL);
        t.expect_frame(2, &expected3, TAIL as u16);
        t.return_and_status();
        assert_eq!(t.h.n_status(), 2);
    }

    #[test]
    fn spanning_idle_packet_spans_frames() {
        const CAP: usize = AGGREGATION_SIZE;
        const RESIDUAL: usize = 3;
        const PACKET_SIZE: usize = CAP - RESIDUAL;
        let mut t = Tester::new(12);
        t.comp.configure(true);
        t.test_initial();
        t.h.clear();

        let packet = t.fill_buffer(PACKET_SIZE);
        let packet_bytes = packet.data().to_vec();
        t.spanning_send(packet);
        assert_eq!(t.h.n_out(), 0);
        assert_eq!(t.h.n_status(), 1);

        t.invoke_timeout();
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        assert_eq!(t.h.n_out(), 1);
        let mut idle = Vec::new();
        Tester::append_idle_packet(&mut idle, idle_packet::MIN_SIZE);
        let mut expected1 = packet_bytes.clone();
        expected1.extend_from_slice(&idle[..RESIDUAL]);
        t.expect_frame(0, &expected1, 0);
        t.return_and_status();
        assert_eq!(t.h.n_status(), 1);

        // Carried idle bytes alone do not trigger a timeout send.
        t.invoke_timeout();
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        assert_eq!(t.h.n_out(), 1);
        assert_eq!(t.h.n_status(), 1);

        const SMALL: usize = 10;
        let small = t.fill_buffer(SMALL);
        let small_bytes = small.data().to_vec();
        t.spanning_send(small);
        assert_eq!(t.h.n_out(), 1);
        assert_eq!(t.h.n_ret(), 2);
        assert_eq!(t.h.n_status(), 2);
        t.invoke_timeout();
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        assert_eq!(t.h.n_out(), 2);
        let continuation = idle_packet::MIN_SIZE - RESIDUAL;
        let mut expected2 = idle[RESIDUAL..].to_vec();
        expected2.extend_from_slice(&small_bytes);
        Tester::append_idle_packet(&mut expected2, CAP - continuation - SMALL);
        t.expect_frame(1, &expected2, continuation as u16);
        t.return_and_status();
        assert_eq!(t.h.n_status(), 2);
    }

    #[test]
    fn spanning_failure_drops_split_remainder() {
        const CAP: usize = AGGREGATION_SIZE;
        let mut t = Tester::new(13);
        t.comp.configure(true);
        t.test_initial();
        t.h.clear();

        let first = t.fill_buffer(100);
        t.spanning_send(first);
        assert_eq!(t.h.n_ret(), 1);
        assert_eq!(t.h.n_status(), 1);
        t.h.clear();

        let span = t.fill_buffer(CAP + 184);
        let span_id = span.context();
        t.spanning_send(span);
        assert_eq!(t.h.n_out(), 1);
        assert_eq!(t.h.n_ret(), 0);
        assert_eq!(t.h.n_status(), 0);
        t.h.clear();

        t.return_frame();
        t.invoke_status(Success::Failure);
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        assert_eq!(t.h.n_out(), 0);
        assert_eq!(t.h.n_ret(), 0);
        assert_eq!(t.h.n_status(), 0);

        t.invoke_status(Success::Success);
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        assert_eq!(t.h.n_out(), 0);
        assert_eq!(t.h.n_ret(), 1);
        assert_eq!(t.h.n_status(), 1);
        assert_eq!(t.h.data_return.lock().unwrap()[0].2, span_id);
        t.h.clear();

        let packet = t.fill_buffer(50);
        let bytes = packet.data().to_vec();
        t.spanning_send(packet);
        assert_eq!(t.h.n_out(), 0);
        assert_eq!(t.h.n_ret(), 1);
        assert_eq!(t.h.n_status(), 1);
        t.invoke_timeout();
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        assert_eq!(t.h.n_out(), 1);
        let mut expected = bytes;
        Tester::append_idle_packet(&mut expected, CAP - 50);
        t.expect_frame(0, &expected, 0);
        t.return_and_status();
    }

    /// Randomized spanning: the rules in a random sequence against the
    /// shadow byte-stream model (upstream `Spanning.RandomizedTesting`).
    #[test]
    fn spanning_randomized_testing() {
        for seed in [0x5EED_u64, 0xC0FFEE, 42] {
            let mut t = Tester::new(seed);
            t.comp.configure(true);
            t.test_initial();
            let mut steps = 0;
            for _ in 0..10_000 {
                let rule = t.pick.lower_upper(0, 4);
                match rule {
                    0 if !t.outstanding => t.rule_send_packet(),
                    1 if t.outstanding && !t.held_pending => t.rule_send_packet_while_waiting(),
                    2 if !t.outstanding => t.rule_timeout(),
                    3 if t.outstanding => t.rule_status_failure(),
                    4 if t.outstanding => t.rule_return_and_status(),
                    _ => continue,
                }
                steps += 1;
            }
            assert!(steps > 1000);
        }
    }

    // -- Assertions ----------------------------------------------------------------

    #[test]
    #[should_panic(expected = "Assert:")]
    fn configure_after_fill_asserts() {
        let mut t = Tester::new(14);
        t.test_initial();
        let _ = t.test_fill(false);
        t.comp.configure(true);
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn oversize_hold_without_spanning_asserts() {
        let mut t = Tester::new(15);
        t.test_initial();
        t.test_fill_multi();
        t.invoke_timeout();
        assert_eq!(t.dispatch_one(), MsgDispatchStatus::Ok);
        assert_eq!(t.h.n_out(), 1);
        let oversize = t.fill_buffer(NON_SPANNING_CAPACITY + 1);
        t.invoke_data_in(oversize, &FrameContext::default());
        let _ = t.dispatch_one();
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn oversize_fill_without_spanning_asserts() {
        let mut t = Tester::new(16);
        t.test_initial();
        let oversize = t.fill_buffer(NON_SPANNING_CAPACITY + 1);
        t.invoke_data_in(oversize, &FrameContext::default());
        let _ = t.dispatch_one();
    }

    /// Status while filling is impossible by protocol and asserts
    /// (`assertNoStatus`).
    #[test]
    #[should_panic(expected = "Assert:")]
    fn status_while_filling_asserts() {
        let mut t = Tester::new(17);
        t.test_initial();
        t.invoke_status(Success::Success);
        let _ = t.dispatch_one();
    }

    /// Constants match upstream.
    #[test]
    fn capacities_match_cpp() {
        assert_eq!(AGGREGATION_SIZE, 1016);
        assert_eq!(NON_SPANNING_CAPACITY, 1009);
        assert_eq!(MSG_SIZE, 37);
    }

    /// The signal message envelope, byte for byte.
    #[test]
    fn signal_envelope_bytes() {
        let buf = ComAggregator::signal_start(SIGNAL_STATUS);
        assert_eq!(buf.as_slice(), &[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3]);
    }

    /// The active task runs the preamble and dispatches signals.
    #[test]
    fn active_task_lifecycle() {
        let comp = ComAggregator::new("agg");
        comp.create_queue(10);
        let h = Arc::new(Harness::default());
        comp.data_out.connect(h.clone(), 0);
        comp.data_return_out.connect(h.clone(), 1);
        comp.com_status_out.connect(h.clone(), 0);
        comp.active
            .start(&comp, 0, 0, fprime_os::task::TASK_DEFAULT);
        let p = comp.com_status_in(0);
        p.target.invoke(p.port_num, &mut Success::Success);
        let p = comp.data_in(0);
        p.target
            .invoke(p.port_num, Buffer::allocate(5), &FrameContext::default());
        comp.active.exit();
        let _ = comp.active.join();
        // Preamble SUCCESS + one SUCCESS for the consumed buffer.
        assert_eq!(h.n_status(), 2);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(comp.aggregate_size(), 5);
    }
}
