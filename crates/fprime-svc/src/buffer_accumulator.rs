//! `Svc::BufferAccumulator` — fill/drain buffer FIFO between a producer and a
//! consumer (ACTIVE).
//!
//! Port of `Svc/BufferAccumulator/BufferAccumulator.{fpp,hpp,cpp}`,
//! `ArrayFIFOBuffer.cpp`, `Commands.fppi`, `Events.fppi` and
//! `Telemetry.fppi`; the reference Ref deployment interposes it between
//! `dpMgr.productSendOut` and `dpWriter.bufferSendIn` exactly as
//! `Svc/Subtopologies/DataProducts/DataProducts.fpp` does (analysis of the
//! surrounding chain: `docs/cpp-analysis/data-products.md`).
//!
//! Buffers arriving on `bufferSendInFill` are enqueued in a fixed-capacity
//! FIFO allocated once by [`BufferAccumulator::allocate_queue`]. In `DRAIN`
//! mode the component forwards ONE buffer at a time on `bufferSendOutDrain`
//! and waits for it to come back on `bufferSendInReturn` (which it passes
//! upstream on `bufferSendOutReturn`) before sending the next. In
//! `ACCUMULATE` mode buffers only pile up, until `BA_SetMode DRAIN` or a
//! partial `BA_DrainBuffers N` releases them. A partial drain answers its
//! command once the N-th buffer has been SENT (not returned).
//!
//! # C++ quirks reproduced
//!
//! * `m_numDrained` is incremented on every send, including full-`DRAIN`
//!   sends that are not part of a partial drain; it is only reset by a
//!   partial drain (start / completion / cancellation by `BA_SetMode`).
//! * `BA_DrainBuffers` in `DRAIN` mode is a `VALIDATION_ERROR`
//!   (`BA_AlreadyDraining`); a second one while a partial drain is in
//!   progress is `BUSY` (`BA_StillDraining`), checked first.
//! * A `BLOCK` partial drain with too few buffers stalls
//!   (`BA_DrainStalled`) and keeps the command open; the next buffer to
//!   arrive on `bufferSendInFill` resumes it.
//! * `BA_SetMode` cancels an in-progress partial drain and answers the
//!   ORIGINAL `BA_DrainBuffers` command `OK` before answering itself.
//! * `BA_QueueFull` / `BA_BufferAccepted` are edge-triggered on the
//!   `m_numWarnings` counter (no FPP throttle); the dropped buffer is
//!   returned upstream on `bufferSendOutReturn`.
//! * `BA_NumQueuedBuffers` has no `update on change`, so it is written on
//!   every fill and every send attempt.
//! * Before `allocate_queue` the FIFO has capacity 0: every fill is a
//!   `BA_QueueFull` drop and every drain finds nothing (C++: null element
//!   array).
//!
//! # Deviations from C++
//!
//! * `allocateQueue(identifier, allocator, maxNumBuffers, initialMode)`
//!   takes no `Fw::MemAllocator`: the FIFO storage is a
//!   `Box<[Option<Buffer>]>` from the global heap, allocated once here and
//!   never grown (the same choice `CmdSequencer::allocate_buffer` makes).
//!   The identifier is recorded for parity.
//! * `deallocateQueue` frees the storage AND resets the FIFO to its
//!   unallocated (capacity 0) state; C++ leaves `m_bufferQueue` pointing at
//!   freed memory. Buffers still queued are dropped with the storage.
//! * Calling `allocate_queue` again replaces the FIFO with a fresh, empty
//!   one; C++ re-runs `ArrayFIFOBuffer::init` without resetting its
//!   indices or size (and leaks the old block).
//! * Async commands share ONE queue message type, `cmdIn` (the workspace
//!   convention), where C++ generates one per command; the envelope layout
//!   is otherwise identical. An undeclared `OpState`/`BlockMode` value is
//!   answered `VALIDATION_ERROR` (workspace command-glue discipline; stock
//!   generated code fails the deserialization with `FORMAT_ERROR`).
//! * Async buffer ports carry the owned `Buffer` through a
//!   [`BufferEscrow`] token where C++ serializes the raw `Fw::Buffer`.
//!
//! # Threading
//!
//! Every input (`bufferSendInFill`, `bufferSendInReturn`, `pingIn`, both
//! commands) is async, so all state is touched only on the component
//! thread, as in C++ (which has no mutex at all). The `Mutex` exists for
//! Rust's `Sync` requirement and the introspection accessors; it is never
//! held across an output-port call.

use fprime_comp::escrow::BufferEscrow;
use fprime_comp::{
    ActiveBase, ActiveComponent, BufferSendPort, CmdGlue, CmdPort, ComponentDispatch, EventGlue,
    MsgDispatchStatus, OutputPort, PingPort, PortRef, QueueFullPolicy, TlmGlue,
    async_input_port_adapter, component_msg_types, msg,
};
use fprime_config::{
    FW_CMD_ARG_BUFFER_MAX_SIZE, FwChanIdType, FwEnumStoreType, FwEventIdType, FwIdType,
    FwIndexType, FwOpcodeType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, LinearBuffer, LogSeverity, SerBuf, SerBufAny,
    SerializeStatus, fpp_enum, fw_assert, fw_try,
};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// FPP types (Commands.fppi)
// ---------------------------------------------------------------------------

fpp_enum! {
    /// `Svc.BufferAccumulator.OpState` (C++ `BufferAccumulator_OpState`) —
    /// the operating mode, the `BA_SetMode` argument.
    pub enum OpState : u8 {
        /// Queue incoming buffers; send nothing unless a partial drain asks.
        Accumulate = 0,
        /// Forward queued buffers one at a time as each one is returned.
        Drain = 1,
    }
    default Accumulate
}

fpp_enum! {
    /// `Svc.BufferAccumulator.BlockMode` (C++ `BufferAccumulator_BlockMode`)
    /// — the `BA_DrainBuffers` behavior when fewer buffers are queued than
    /// requested.
    pub enum BlockMode : u8 {
        /// Drain only what is queued now (`BA_NonBlockDrain`).
        NoBlock = 0,
        /// Wait for further buffers until the requested count is sent.
        Block = 1,
    }
    default NoBlock
}

// ---------------------------------------------------------------------------
// Queue message layout
// ---------------------------------------------------------------------------

/// Queue message size = the largest async invocation, the command envelope:
/// 6 + 4 + 4 + 2 + 506 = 522 (the buffer ports need 6 + 8, `pingIn` 6 + 4).
pub const QUEUE_MSG_SIZE: usize =
    msg::ENVELOPE_HEADER_SIZE + 4 + 4 + 2 + FW_CMD_ARG_BUFFER_MAX_SIZE;

/// FPP declares no `priority` qualifier on any async input.
const PORT_PRIORITY: FwQueuePriorityType = 1;

type MsgBuffer = LinearBuffer<QUEUE_MSG_SIZE>;

// ---------------------------------------------------------------------------
// ArrayFIFOBuffer
// ---------------------------------------------------------------------------

/// Port of the private `BufferAccumulator::ArrayFIFOBuffer`: a ring of
/// `capacity` buffer slots allocated once, never grown.
///
/// The C++ queue stores `Fw::Buffer` copies and leaves a stale copy in the
/// slot after a dequeue; here the slot is emptied (`Option::take`) because
/// the buffer is owned.
#[derive(Debug, Default)]
struct ArrayFifoBuffer {
    /// The element storage (`m_elements`); empty = unallocated.
    elements: Box<[Option<Buffer>]>,
    /// `m_enqueueIndex`.
    enqueue_index: usize,
    /// `m_dequeueIndex`.
    dequeue_index: usize,
    /// `m_size`.
    size: usize,
}

impl ArrayFifoBuffer {
    /// C++ `init(elements, capacity)` over freshly allocated storage.
    fn with_capacity(capacity: usize) -> Self {
        let elements: Box<[Option<Buffer>]> = (0..capacity).map(|_| None).collect();
        Self {
            elements,
            enqueue_index: 0,
            dequeue_index: 0,
            size: 0,
        }
    }

    /// C++ `getCapacity()`.
    fn capacity(&self) -> usize {
        self.elements.len()
    }

    /// C++ `getSize()`.
    fn size(&self) -> usize {
        self.size
    }

    /// C++ `enqueue`: fails when full (or unallocated — C++ checks the null
    /// element pointer, here capacity 0), handing the buffer back.
    fn enqueue(&mut self, buffer: Buffer) -> Result<(), Buffer> {
        let capacity = self.capacity();
        if self.size >= capacity {
            return Err(buffer);
        }
        fw_assert!(
            self.enqueue_index < capacity,
            self.enqueue_index as fprime_config::FwAssertArgType
        );
        self.elements[self.enqueue_index] = Some(buffer);
        self.enqueue_index = (self.enqueue_index + 1) % capacity;
        self.size += 1;
        Ok(())
    }

    /// C++ `dequeue`: `None` when empty (or unallocated).
    fn dequeue(&mut self) -> Option<Buffer> {
        if self.size == 0 {
            return None;
        }
        let capacity = self.capacity();
        fw_assert!(
            self.dequeue_index < capacity,
            self.dequeue_index as fprime_config::FwAssertArgType
        );
        let buffer = self.elements[self.dequeue_index].take();
        // Corrupted-ring invariant: every counted slot holds a buffer.
        fw_assert!(
            buffer.is_some(),
            self.dequeue_index as fprime_config::FwAssertArgType
        );
        self.dequeue_index = (self.dequeue_index + 1) % capacity;
        self.size -= 1;
        buffer
    }
}

// ---------------------------------------------------------------------------
// Component state
// ---------------------------------------------------------------------------

/// The C++ member variables (all component-thread state).
#[derive(Debug, Default)]
struct BufferAccumulatorState {
    /// `m_mode`.
    mode: OpState,
    /// `m_bufferQueue` (+ `m_bufferMemory`).
    buffer_queue: ArrayFifoBuffer,
    /// `m_send` — whether to send a buffer to the downstream client.
    send: bool,
    /// `m_waitForBuffer` — a drained buffer has not come back yet.
    wait_for_buffer: bool,
    /// `m_numWarnings` — QueueFull drops since the last accepted buffer.
    num_warnings: u32,
    /// `m_numDrained` — buffers sent in the current partial drain (also
    /// counts full-DRAIN sends; see the module header).
    num_drained: FwSizeType,
    /// `m_numToDrain` — the partial-drain target; 0 = no partial drain.
    num_to_drain: FwSizeType,
    /// `m_opCode` of the `BA_DrainBuffers` to answer.
    op_code: FwOpcodeType,
    /// `m_cmdSeq` of the `BA_DrainBuffers` to answer.
    cmd_seq: u32,
    /// `m_allocatorId`.
    allocator_id: FwEnumStoreType,
}

/// What `sendStoredBuffer` decided under the lock, performed after it.
struct SendOutcome {
    drained: Option<Buffer>,
    stalled: Option<(u32, u32)>,
    /// `(numDrained, opCode, cmdSeq)` of a partial drain that just finished.
    done: Option<(u32, FwOpcodeType, u32)>,
    queued: u32,
}

/// `Svc::BufferAccumulator` — active fill/drain buffer FIFO.
pub struct BufferAccumulator {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Command registration/response glue (`cmdRegOut`, `cmdResponseOut`).
    pub cmd: CmdGlue,
    /// Event ports (`eventOut`, `eventOutText`) and `timeCaller`.
    pub evt: EventGlue,
    /// Telemetry port (`tlmOut`).
    pub tlm: TlmGlue,
    /// `bufferSendOutDrain`: pass a buffer on to the downstream component.
    pub buffer_send_out_drain: OutputPort<dyn BufferSendPort>,
    /// `bufferSendOutReturn`: return a buffer to the original upstream
    /// component (the allocator).
    pub buffer_send_out_return: OutputPort<dyn BufferSendPort>,
    /// `pingOut`: health ping reply.
    pub ping_out: OutputPort<dyn PingPort>,
    /// Escrow for the two async `Fw.BufferSend` inputs.
    escrow: BufferEscrow,
    state: Mutex<BufferAccumulatorState>,
}

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel), FPP port order.
    impl BufferAccumulator {
        /// `bufferSendInFill` async input port.
        MSG_TYPE_BUFFER_SEND_IN_FILL,
        /// `bufferSendInReturn` async input port.
        MSG_TYPE_BUFFER_SEND_IN_RETURN,
        /// `pingIn` async input port (`drop`).
        MSG_TYPE_PING_IN,
        /// `cmdIn` — both async commands.
        MSG_TYPE_CMD_IN,
    }
}

impl BufferAccumulator {
    // -- Commands (FPP-relative opcodes) -----------------------------------

    /// `BA_SetMode(mode: OpState)` — opcode 0x00.
    pub const OPCODE_BA_SETMODE: FwOpcodeType = 0x00;
    /// `BA_DrainBuffers(numToDrain: U32, blockMode: BlockMode)` — opcode
    /// 0x01.
    pub const OPCODE_BA_DRAINBUFFERS: FwOpcodeType = 0x01;

    // -- Events (FPP-relative ids) -----------------------------------------

    /// `BA_BufferAccepted` — ACTIVITY_HI, id 0x00.
    pub const EVENTID_BA_BUFFERACCEPTED: FwEventIdType = 0x00;
    /// `BA_QueueFull` — WARNING_HI, id 0x01.
    pub const EVENTID_BA_QUEUEFULL: FwEventIdType = 0x01;
    /// `BA_StillDraining(numDrained: U32, numToDrain: U32)` — WARNING_HI,
    /// id 0x02.
    pub const EVENTID_BA_STILLDRAINING: FwEventIdType = 0x02;
    /// `BA_AlreadyDraining` — WARNING_HI, id 0x03.
    pub const EVENTID_BA_ALREADYDRAINING: FwEventIdType = 0x03;
    /// `BA_DrainStalled(numDrained: U32, numToDrain: U32)` — WARNING_HI,
    /// id 0x04.
    pub const EVENTID_BA_DRAINSTALLED: FwEventIdType = 0x04;
    /// `BA_PartialDrainDone(numDrained: U32)` — ACTIVITY_HI, id 0x05.
    pub const EVENTID_BA_PARTIALDRAINDONE: FwEventIdType = 0x05;
    /// `BA_NonBlockDrain(numWillDrain: U32, numReqDrain: U32)` —
    /// WARNING_LO, id 0x06.
    pub const EVENTID_BA_NONBLOCKDRAIN: FwEventIdType = 0x06;

    // -- Telemetry (FPP-relative ids) --------------------------------------

    /// `BA_NumQueuedBuffers: U32` — id 0 (no `update on change`).
    pub const CHANID_BA_NUMQUEUEDBUFFERS: FwChanIdType = 0;

    /// Construct (topology phase 1). The FIFO is unallocated (capacity 0)
    /// and the mode is `ACCUMULATE` until [`Self::allocate_queue`].
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            buffer_send_out_drain: OutputPort::new(),
            buffer_send_out_return: OutputPort::new(),
            ping_out: OutputPort::new(),
            escrow: BufferEscrow::new(),
            state: Mutex::new(BufferAccumulatorState::default()),
        })
    }

    /// Create the message queue (C++ `init(queueDepth, instance)`).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    /// C++ `allocateQueue(identifier, allocator, maxNumBuffers,
    /// initialMode)`: allocate the FIFO storage for `max_num_buffers`
    /// buffers (once, at configuration time) and set the initial mode;
    /// `DRAIN` arms sending immediately. Asserts `max_num_buffers > 0` and
    /// that the storage size does not overflow, as C++ does.
    pub fn allocate_queue(
        &self,
        identifier: FwEnumStoreType,
        max_num_buffers: FwSizeType,
        initial_mode: OpState,
    ) {
        fw_assert!(max_num_buffers > 0);
        // C++ parity: FW_ASSERT((max / maxNumBuffers) >= sizeof(Fw::Buffer)).
        fw_assert!(
            (FwSizeType::MAX / max_num_buffers) >= size_of::<Option<Buffer>>() as FwSizeType
        );
        let capacity = usize::try_from(max_num_buffers).unwrap_or(usize::MAX);
        let fifo = ArrayFifoBuffer::with_capacity(capacity);
        let mut st = self.state.lock().unwrap();
        st.allocator_id = identifier;
        st.buffer_queue = fifo;
        st.mode = initial_mode;
        st.send = initial_mode == OpState::Drain;
    }

    /// C++ `deallocateQueue(allocator)`: release the FIFO storage (during
    /// teardown, after the task is joined). The FIFO returns to the
    /// unallocated state; still-queued buffers are dropped.
    pub fn deallocate_queue(&self) {
        let mut st = self.state.lock().unwrap();
        st.buffer_queue = ArrayFifoBuffer::default();
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[Self::OPCODE_BA_SETMODE, Self::OPCODE_BA_DRAINBUFFERS],
        );
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    // -- Introspection -----------------------------------------------------

    /// The current operating mode.
    #[must_use]
    pub fn mode(&self) -> OpState {
        self.state.lock().unwrap().mode
    }

    /// Number of buffers currently queued (`BA_NumQueuedBuffers`).
    #[must_use]
    pub fn num_queued_buffers(&self) -> FwSizeType {
        self.state.lock().unwrap().buffer_queue.size() as FwSizeType
    }

    /// FIFO capacity (0 before [`Self::allocate_queue`]).
    #[must_use]
    pub fn queue_capacity(&self) -> FwSizeType {
        self.state.lock().unwrap().buffer_queue.capacity() as FwSizeType
    }

    /// The allocator identifier recorded by [`Self::allocate_queue`].
    #[must_use]
    pub fn allocator_id(&self) -> FwEnumStoreType {
        self.state.lock().unwrap().allocator_id
    }

    // -- Input-port factories ----------------------------------------------

    /// `bufferSendInFill` — ASYNC `Fw.BufferSend` input (\[1\]).
    pub fn buffer_send_in_fill(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn BufferSendPort> {
        fw_assert!(port_num == 0, port_num as fprime_config::FwAssertArgType);
        PortRef::new(
            Arc::new(BufferSendInAdapter {
                comp: self.clone(),
                msg_type: Self::MSG_TYPE_BUFFER_SEND_IN_FILL,
            }),
            port_num,
        )
    }

    /// `bufferSendInReturn` — ASYNC `Fw.BufferSend` input (\[1\]).
    pub fn buffer_send_in_return(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn BufferSendPort> {
        fw_assert!(port_num == 0, port_num as fprime_config::FwAssertArgType);
        PortRef::new(
            Arc::new(BufferSendInAdapter {
                comp: self.clone(),
                msg_type: Self::MSG_TYPE_BUFFER_SEND_IN_RETURN,
            }),
            port_num,
        )
    }

    // -- Port handlers (component thread) ----------------------------------

    /// `bufferSendInFill_handler`: enqueue; on a full queue emit the
    /// edge-triggered `BA_QueueFull` and return the buffer upstream; then
    /// send if armed and report the queue depth.
    fn buffer_send_in_fill_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        let (accepted, full, rejected, send) = {
            let st = &mut *self.state.lock().unwrap();
            match st.buffer_queue.enqueue(buffer) {
                Ok(()) => {
                    let accepted = st.num_warnings > 0;
                    st.num_warnings = 0;
                    (accepted, false, None, st.send)
                }
                Err(buffer) => {
                    let full = st.num_warnings == 0;
                    // C++ parity: U32 `m_numWarnings++` (wraps).
                    st.num_warnings = st.num_warnings.wrapping_add(1);
                    (false, full, Some(buffer), st.send)
                }
            }
        };
        if accepted {
            self.log_buffer_accepted();
        }
        if full {
            self.log_queue_full();
        }
        if let Some(buffer) = rejected {
            // The buffer is dropped; ownership must go back to its sender
            // or the pool is depleted (C++ comment, reproduced).
            let port = self.buffer_send_out_return.get();
            port.target.invoke(port.port_num, buffer);
        }
        if send {
            self.send_stored_buffer();
        }
        let queued = self.state.lock().unwrap().buffer_queue.size();
        self.tlm_write_num_queued_buffers(queued as u32);
    }

    /// `bufferSendInReturn_handler`: pass the buffer upstream, then send
    /// the next one when draining everything or mid partial drain.
    fn buffer_send_in_return_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        let port = self.buffer_send_out_return.get();
        port.target.invoke(port.port_num, buffer);
        let send = {
            let st = &mut *self.state.lock().unwrap();
            st.wait_for_buffer = false;
            if st.mode == OpState::Drain || st.num_drained < st.num_to_drain {
                st.send = true;
                true
            } else {
                false
            }
        };
        if send {
            self.send_stored_buffer();
        }
    }

    /// `pingIn_handler`: echo the key.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        let port = self.ping_out.get();
        port.target.invoke(port.port_num, key);
    }

    /// C++ `sendStoredBuffer()`: send one queued buffer if draining
    /// everything or a partial drain is short; finish a partial drain once
    /// its last buffer is SENT; always report the queue depth.
    fn send_stored_buffer(&self) {
        let outcome = {
            let st = &mut *self.state.lock().unwrap();
            fw_assert!(st.send);
            let mut drained = None;
            let mut stalled = None;
            if st.num_to_drain == 0 || st.num_drained < st.num_to_drain {
                match st.buffer_queue.dequeue() {
                    Some(buffer) => {
                        st.num_drained += 1;
                        st.wait_for_buffer = true;
                        st.send = false;
                        drained = Some(buffer);
                    }
                    None => {
                        if st.num_to_drain > 0 {
                            stalled = Some((st.num_drained as u32, st.num_to_drain as u32));
                        }
                    }
                }
            }
            // C++ comment: this used to be "else if", but then a partial
            // drain would wait for its buffers to be RETURNED before
            // answering; it answers once they are SENT.
            let mut done = None;
            if st.num_to_drain > 0 && st.num_drained == st.num_to_drain {
                done = Some((st.num_drained as u32, st.op_code, st.cmd_seq));
                st.num_to_drain = 0;
                st.num_drained = 0;
                st.send = false;
            }
            SendOutcome {
                drained,
                stalled,
                done,
                queued: st.buffer_queue.size() as u32,
            }
        };
        if let Some(buffer) = outcome.drained {
            let port = self.buffer_send_out_drain.get();
            port.target.invoke(port.port_num, buffer);
        }
        if let Some((num_drained, num_to_drain)) = outcome.stalled {
            self.log_drain_stalled(num_drained, num_to_drain);
        }
        if let Some((num_drained, op_code, cmd_seq)) = outcome.done {
            self.log_partial_drain_done(num_drained);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
        }
        self.tlm_write_num_queued_buffers(outcome.queued);
    }

    // -- Command handlers ----------------------------------------------------

    /// `BA_SetMode_cmdHandler`: cancel any partial drain (answering it
    /// `OK`), switch mode, kick a send when entering `DRAIN` unless a
    /// drained buffer is still out, answer `OK`.
    fn ba_set_mode_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32, mode: OpState) {
        let cancelled = {
            let st = &mut *self.state.lock().unwrap();
            if st.num_to_drain > 0 {
                st.num_to_drain = 0;
                st.num_drained = 0;
                Some((st.op_code, st.cmd_seq))
            } else {
                None
            }
        };
        if let Some((drain_op_code, drain_cmd_seq)) = cancelled {
            self.cmd
                .cmd_response(drain_op_code, drain_cmd_seq, CmdResponse::Ok);
        }
        let send = {
            let st = &mut *self.state.lock().unwrap();
            st.mode = mode;
            if mode == OpState::Drain {
                if st.wait_for_buffer {
                    false
                } else {
                    st.send = true;
                    true
                }
            } else {
                st.send = false;
                false
            }
        };
        if send {
            self.send_stored_buffer();
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `BA_DrainBuffers_cmdHandler`: start a partial drain of
    /// `num_to_drain` buffers (only in `ACCUMULATE` mode, one at a time).
    /// The command is answered when the drain completes, unless it is
    /// rejected or trivially done here.
    fn ba_drain_buffers_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        num_to_drain: u32,
        block_mode: BlockMode,
    ) {
        enum Next {
            StillDraining(u32, u32),
            AlreadyDraining,
            DoneNow { non_block: Option<u32> },
            Start { non_block: Option<u32>, send: bool },
        }
        let next = {
            let st = &mut *self.state.lock().unwrap();
            if st.num_drained < st.num_to_drain {
                Next::StillDraining(st.num_drained as u32, st.num_to_drain as u32)
            } else if st.mode == OpState::Drain {
                Next::AlreadyDraining
            } else if num_to_drain == 0 {
                Next::DoneNow { non_block: None }
            } else {
                st.op_code = op_code;
                st.cmd_seq = cmd_seq;
                st.num_drained = 0;
                st.num_to_drain = FwSizeType::from(num_to_drain);
                let mut non_block = None;
                let mut zero = false;
                if block_mode == BlockMode::NoBlock {
                    let num_buffers = st.buffer_queue.size() as FwSizeType;
                    if num_buffers < FwSizeType::from(num_to_drain) {
                        st.num_to_drain = num_buffers;
                        non_block = Some(st.num_to_drain as u32);
                    }
                    // C++: OK if there were 0 buffers queued and we end up
                    // setting numToDrain to 0.
                    zero = st.num_to_drain == 0;
                }
                if zero {
                    Next::DoneNow { non_block }
                } else if st.wait_for_buffer {
                    // Still waiting for a buffer from last time; its return
                    // resumes the drain.
                    Next::Start {
                        non_block,
                        send: false,
                    }
                } else {
                    st.send = true;
                    Next::Start {
                        non_block,
                        send: true,
                    }
                }
            }
        };
        match next {
            Next::StillDraining(num_drained, to_drain) => {
                self.log_still_draining(num_drained, to_drain);
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Busy);
            }
            Next::AlreadyDraining => {
                self.log_already_draining();
                self.cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            }
            Next::DoneNow { non_block } => {
                if let Some(will_drain) = non_block {
                    self.log_non_block_drain(will_drain, num_to_drain);
                }
                self.log_partial_drain_done(0);
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            }
            Next::Start { non_block, send } => {
                if let Some(will_drain) = non_block {
                    self.log_non_block_drain(will_drain, num_to_drain);
                }
                if send {
                    self.send_stored_buffer(); // kick off the draining
                }
            }
        }
    }

    /// `cmdIn` dispatch on the local opcode: `FORMAT_ERROR` on a short or
    /// over-long argument buffer, `VALIDATION_ERROR` on an undeclared enum
    /// value, exactly one response per command.
    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_BA_SETMODE => {
                let mut raw = 0u8;
                if !args.deserialize_u8_be(&mut raw).is_ok() || args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                let Ok(mode) = OpState::try_from(raw) else {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
                    return;
                };
                self.ba_set_mode_cmd_handler(op_code, cmd_seq, mode);
            }
            Self::OPCODE_BA_DRAINBUFFERS => {
                let mut num_to_drain = 0u32;
                let mut raw = 0u8;
                if !args.deserialize_u32_be(&mut num_to_drain).is_ok()
                    || !args.deserialize_u8_be(&mut raw).is_ok()
                    || args.deserialize_size_left() != 0
                {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                let Ok(block_mode) = BlockMode::try_from(raw) else {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
                    return;
                };
                self.ba_drain_buffers_cmd_handler(op_code, cmd_seq, num_to_drain, block_mode);
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }

    // -- Telemetry -----------------------------------------------------------

    fn tlm_write_num_queued_buffers(&self, value: u32) {
        self.tlm.tlm_write(
            self.id_base(),
            Self::CHANID_BA_NUMQUEUEDBUFFERS,
            &value,
            self.evt.time_get(),
        );
    }

    // -- Events (none throttled) ---------------------------------------------

    fn log_buffer_accepted(&self) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BA_BUFFERACCEPTED,
            LogSeverity::ActivityHi,
            "Buffer accepted",
            |_buf| SerializeStatus::Ok,
        );
    }

    fn log_queue_full(&self) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BA_QUEUEFULL,
            LogSeverity::WarningHi,
            "Queue full",
            |_buf| SerializeStatus::Ok,
        );
    }

    fn log_still_draining(&self, num_drained: u32, num_to_drain: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BA_STILLDRAINING,
            LogSeverity::WarningHi,
            &format!("Still draining {num_drained} of {num_to_drain}"),
            |buf| {
                fw_try!(buf.serialize_u32_be(num_drained));
                buf.serialize_u32_be(num_to_drain)
            },
        );
    }

    fn log_already_draining(&self) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BA_ALREADYDRAINING,
            LogSeverity::WarningHi,
            "Already in DRAIN mode",
            |_buf| SerializeStatus::Ok,
        );
    }

    fn log_drain_stalled(&self, num_drained: u32, num_to_drain: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BA_DRAINSTALLED,
            LogSeverity::WarningHi,
            &format!("Drain stalling - only drained {num_drained} of {num_to_drain}"),
            |buf| {
                fw_try!(buf.serialize_u32_be(num_drained));
                buf.serialize_u32_be(num_to_drain)
            },
        );
    }

    fn log_partial_drain_done(&self, num_drained: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BA_PARTIALDRAINDONE,
            LogSeverity::ActivityHi,
            &format!("Partial drain of {num_drained} finished"),
            |buf| buf.serialize_u32_be(num_drained),
        );
    }

    fn log_non_block_drain(&self, num_will_drain: u32, num_req_drain: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BA_NONBLOCKDRAIN,
            LogSeverity::WarningLo,
            &format!("Only have {num_will_drain}; requested drain of {num_req_drain}"),
            |buf| {
                fw_try!(buf.serialize_u32_be(num_will_drain));
                buf.serialize_u32_be(num_req_drain)
            },
        );
    }
}

// -- Async input adapters ----------------------------------------------------

/// `bufferSendInFill` / `bufferSendInReturn` adapter:
/// `[msg_type][port_num][escrow token u64]` (the token sits where C++
/// serializes the `Fw::Buffer`).
struct BufferSendInAdapter {
    comp: Arc<BufferAccumulator>,
    msg_type: FwEnumStoreType,
}

impl BufferSendPort for BufferSendInAdapter {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        let token = self.comp.escrow.deposit(buffer);
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, self.msg_type, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u64_be(token);
        fw_assert!(status.is_ok(), status as i32);
        // FPP default `assert` queue-full policy.
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

async_input_port_adapter! {
    /// `pingIn` — ASYNC `Svc.Ping` input with the FPP `drop` policy.
    component: BufferAccumulator;
    adapter: PingInAdapter;
    port: PingPort;
    input: pub ping_in;
    deserialize: ping_in_deserialize;
    handler: ping_in_handler;
    base: active.queued;
    msg_type: BufferAccumulator::MSG_TYPE_PING_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: PORT_PRIORITY;
    queue_full: QueueFullPolicy::Drop;
    args { val key: u32 }
}

async_input_port_adapter! {
    /// `cmdIn` — ASYNC `Fw.Cmd` input for `BA_SetMode` / `BA_DrainBuffers`.
    component: BufferAccumulator;
    adapter: CmdInAdapter;
    port: CmdPort;
    input: pub cmd_in;
    deserialize: cmd_in_deserialize;
    handler: cmd_in_handler;
    base: active.queued;
    msg_type: BufferAccumulator::MSG_TYPE_CMD_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: PORT_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, buf args: CmdArgBuffer }
}

// -- Dispatch (the generated `doDispatch` switch) -----------------------------

impl ComponentDispatch for BufferAccumulator {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            Self::MSG_TYPE_BUFFER_SEND_IN_FILL | Self::MSG_TYPE_BUFFER_SEND_IN_RETURN => {
                let mut token = 0u64;
                if !buf.deserialize_u64_be(&mut token).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                let buffer = self.escrow.claim(token);
                if msg_type == Self::MSG_TYPE_BUFFER_SEND_IN_FILL {
                    self.buffer_send_in_fill_handler(port_num, buffer);
                } else {
                    self.buffer_send_in_return_handler(port_num, buffer);
                }
                MsgDispatchStatus::Ok
            }
            Self::MSG_TYPE_PING_IN => match Self::ping_in_deserialize(buf) {
                Some((key,)) => {
                    self.ping_in_handler(port_num, key);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_CMD_IN => match Self::cmd_in_deserialize(buf) {
                Some((op_code, cmd_seq, mut args)) => {
                    self.cmd_in_handler(port_num, op_code, cmd_seq, &mut args);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for BufferAccumulator {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort, TimePort, TlmPort};
    use fprime_fw::{LogBuffer, TextLogString, Time, TimeBase, TlmBuffer};
    use fprime_os::task::TASK_DEFAULT;
    use std::collections::VecDeque;

    const ID_BASE: FwIdType = 0x4000;
    /// `MAX_NUM_BUFFERS` of the C++ tester.
    const MAX_NUM_BUFFERS: u32 = 10;
    /// `QUEUE_DEPTH` of the C++ tester.
    const QUEUE_DEPTH: FwSizeType = 30;

    const SET_MODE: FwOpcodeType = ID_BASE + BufferAccumulator::OPCODE_BA_SETMODE;
    const DRAIN_BUFFERS: FwOpcodeType = ID_BASE + BufferAccumulator::OPCODE_BA_DRAINBUFFERS;

    /// One typed from-port invocation (C++ `ASSERT_FROM_PORT_HISTORY`).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum FromPort {
        /// `bufferSendOutDrain` with the buffer context.
        Drain(u32),
        /// `bufferSendOutReturn` with the buffer context.
        Return(u32),
        /// `pingOut` with the key.
        Ping(u32),
    }

    type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

    #[derive(Default)]
    struct Ground {
        from_ports: Mutex<Vec<FromPort>>,
        /// Drained buffers the test has not returned yet (oldest first).
        outstanding: Mutex<VecDeque<Buffer>>,
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: Mutex<Vec<EventRecord>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
    }

    struct DrainSink(Arc<Ground>);

    impl BufferSendPort for DrainSink {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.0
                .from_ports
                .lock()
                .unwrap()
                .push(FromPort::Drain(buffer.context()));
            self.0.outstanding.lock().unwrap().push_back(buffer);
        }
    }

    struct ReturnSink(Arc<Ground>);

    impl BufferSendPort for ReturnSink {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.0
                .from_ports
                .lock()
                .unwrap()
                .push(FromPort::Return(buffer.context()));
        }
    }

    impl PingPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.from_ports.lock().unwrap().push(FromPort::Ping(key));
        }
    }

    impl CmdRegPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }

    impl CmdResponsePort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            op_code: FwOpcodeType,
            cmd_seq: u32,
            response: CmdResponse,
        ) {
            self.responses
                .lock()
                .unwrap()
                .push((op_code, cmd_seq, response));
        }
    }

    impl LogPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.events
                .lock()
                .unwrap()
                .push((id, severity, args.as_slice().to_vec()));
        }
    }

    impl LogTextPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            _id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            _text: &mut TextLogString,
        ) {
        }
    }

    impl TlmPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            _time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            self.tlm.lock().unwrap().push((id, val.as_slice().to_vec()));
        }
    }

    struct TimeStub;

    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 100, 42);
        }
    }

    /// The C++ `BufferAccumulatorTester`: component + ground, queue depth
    /// 30, and (by default) `allocateQueue(0, .., MAX_NUM_BUFFERS, DRAIN)`.
    struct Tester {
        comp: Arc<BufferAccumulator>,
        ground: Arc<Ground>,
    }

    impl Tester {
        fn new() -> Self {
            Self::with_allocation(true)
        }

        fn with_allocation(do_allocate_queue: bool) -> Self {
            let comp = BufferAccumulator::new("BufferAccumulator");
            let ground = Arc::new(Ground::default());
            comp.active.queued.base.set_id_base(ID_BASE);
            comp.buffer_send_out_drain
                .connect(Arc::new(DrainSink(ground.clone())), 0);
            comp.buffer_send_out_return
                .connect(Arc::new(ReturnSink(ground.clone())), 0);
            comp.ping_out.connect(ground.clone(), 0);
            comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
            comp.cmd.cmd_response_out.connect(ground.clone(), 0);
            comp.evt.log_out.connect(ground.clone(), 0);
            comp.evt.text_log_out.connect(ground.clone(), 0);
            comp.evt.time_out.connect(Arc::new(TimeStub), 0);
            comp.tlm.tlm_out.connect(ground.clone(), 0);
            comp.init(QUEUE_DEPTH);
            if do_allocate_queue {
                comp.allocate_queue(0, FwSizeType::from(MAX_NUM_BUFFERS), OpState::Drain);
            }
            Self { comp, ground }
        }

        fn do_dispatch(&self) {
            let _ = self
                .comp
                .active
                .queued
                .dispatch_available_messages(self.comp.as_ref());
        }

        /// `invoke_to_bufferSendInFill` + `doDispatch` with a 10-byte
        /// buffer whose context is `id` (the C++ "bufferID").
        fn fill(&self, id: u32) {
            let mut buffer = Buffer::allocate(10);
            buffer.set_context(id);
            let port = self.comp.buffer_send_in_fill(0);
            port.target.invoke(port.port_num, buffer);
            self.do_dispatch();
        }

        /// `invoke_to_bufferSendInReturn` + `doDispatch` with the oldest
        /// drained buffer still out; returns its context.
        fn return_oldest(&self) -> u32 {
            let buffer = self
                .ground
                .outstanding
                .lock()
                .unwrap()
                .pop_front()
                .expect("a drained buffer to return");
            let context = buffer.context();
            let port = self.comp.buffer_send_in_return(0);
            port.target.invoke(port.port_num, buffer);
            self.do_dispatch();
            context
        }

        fn send_cmd(&self, op_code: FwOpcodeType, cmd_seq: u32, arg_bytes: &[u8]) {
            let mut args = CmdArgBuffer::new();
            assert!(args.set_buff(arg_bytes).is_ok());
            let port = self.comp.cmd_in(0);
            port.target
                .invoke(port.port_num, op_code, cmd_seq, &mut args);
            self.do_dispatch();
        }

        fn set_mode(&self, mode: OpState) {
            self.send_cmd(SET_MODE, 0, &[mode.as_repr()]);
        }

        fn drain_buffers(&self, num_to_drain: u32, block_mode: BlockMode) {
            let mut args = num_to_drain.to_be_bytes().to_vec();
            args.push(block_mode.as_repr());
            self.send_cmd(DRAIN_BUFFERS, 0, &args);
        }

        fn port_history(&self) -> Vec<FromPort> {
            self.ground.from_ports.lock().unwrap().clone()
        }

        fn drains(&self) -> Vec<u32> {
            self.port_history()
                .into_iter()
                .filter_map(|p| match p {
                    FromPort::Drain(c) => Some(c),
                    _ => None,
                })
                .collect()
        }

        fn returns(&self) -> Vec<u32> {
            self.port_history()
                .into_iter()
                .filter_map(|p| match p {
                    FromPort::Return(c) => Some(c),
                    _ => None,
                })
                .collect()
        }

        fn responses(&self) -> Vec<(FwOpcodeType, u32, CmdResponse)> {
            self.ground.responses.lock().unwrap().clone()
        }

        fn events(&self) -> Vec<EventRecord> {
            self.ground.events.lock().unwrap().clone()
        }

        /// Argument bytes of every event with local id `id`.
        fn events_with(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
            self.events()
                .into_iter()
                .filter(|(seen, _, _)| *seen == ID_BASE + id)
                .map(|(_, _, args)| args)
                .collect()
        }

        /// C++ `clearHistory()` (outstanding buffers stay with the test).
        fn clear_history(&self) {
            self.ground.from_ports.lock().unwrap().clear();
            self.ground.responses.lock().unwrap().clear();
            self.ground.events.lock().unwrap().clear();
            self.ground.tlm.lock().unwrap().clear();
        }

        fn state<R>(&self, f: impl FnOnce(&BufferAccumulatorState) -> R) -> R {
            f(&self.comp.state.lock().unwrap())
        }
    }

    fn u32_args(values: &[u32]) -> Vec<u8> {
        values.iter().flat_map(|v| v.to_be_bytes()).collect()
    }

    // -- Model / dictionary --------------------------------------------------

    #[test]
    fn dictionary_ids_match_the_fpp_model() {
        assert_eq!(BufferAccumulator::OPCODE_BA_SETMODE, 0x00);
        assert_eq!(BufferAccumulator::OPCODE_BA_DRAINBUFFERS, 0x01);
        assert_eq!(BufferAccumulator::EVENTID_BA_BUFFERACCEPTED, 0x00);
        assert_eq!(BufferAccumulator::EVENTID_BA_QUEUEFULL, 0x01);
        assert_eq!(BufferAccumulator::EVENTID_BA_STILLDRAINING, 0x02);
        assert_eq!(BufferAccumulator::EVENTID_BA_ALREADYDRAINING, 0x03);
        assert_eq!(BufferAccumulator::EVENTID_BA_DRAINSTALLED, 0x04);
        assert_eq!(BufferAccumulator::EVENTID_BA_PARTIALDRAINDONE, 0x05);
        assert_eq!(BufferAccumulator::EVENTID_BA_NONBLOCKDRAIN, 0x06);
        assert_eq!(BufferAccumulator::CHANID_BA_NUMQUEUEDBUFFERS, 0);
        assert_eq!(QUEUE_MSG_SIZE, 522);
        assert_eq!(BufferAccumulator::MSG_TYPE_BUFFER_SEND_IN_FILL, 1);
        assert_eq!(BufferAccumulator::MSG_TYPE_CMD_IN, 4);
    }

    #[test]
    fn enums_keep_their_fpp_discriminants_and_u8_width() {
        assert_eq!(OpState::Accumulate.as_repr(), 0);
        assert_eq!(OpState::Drain.as_repr(), 1);
        assert_eq!(OpState::default(), OpState::Accumulate);
        assert_eq!(BlockMode::NoBlock.as_repr(), 0);
        assert_eq!(BlockMode::Block.as_repr(), 1);
        assert_eq!(BlockMode::default(), BlockMode::NoBlock);
        assert_eq!(OpState::SERIALIZED_SIZE, 1);
        assert!(OpState::try_from(2u8).is_err());
    }

    #[test]
    fn reg_commands_registers_both_absolute_opcodes() {
        let t = Tester::new();
        t.comp.reg_commands();
        assert_eq!(
            *t.ground.regs.lock().unwrap(),
            vec![SET_MODE, DRAIN_BUFFERS]
        );
    }

    // -- ArrayFIFOBuffer -----------------------------------------------------

    fn ctx_buffer(context: u32) -> Buffer {
        let mut buffer = Buffer::allocate(1);
        buffer.set_context(context);
        buffer
    }

    #[test]
    fn fifo_is_first_in_first_out_across_the_wrap() {
        let mut fifo = ArrayFifoBuffer::with_capacity(3);
        assert_eq!(fifo.capacity(), 3);
        for round in 0..4u32 {
            assert!(fifo.enqueue(ctx_buffer(round * 10)).is_ok());
            assert!(fifo.enqueue(ctx_buffer(round * 10 + 1)).is_ok());
            assert_eq!(fifo.size(), 2);
            assert_eq!(fifo.dequeue().unwrap().context(), round * 10);
            assert_eq!(fifo.dequeue().unwrap().context(), round * 10 + 1);
        }
        assert!(fifo.dequeue().is_none());
    }

    #[test]
    fn fifo_full_hands_the_buffer_back() {
        let mut fifo = ArrayFifoBuffer::with_capacity(2);
        assert!(fifo.enqueue(ctx_buffer(1)).is_ok());
        assert!(fifo.enqueue(ctx_buffer(2)).is_ok());
        let rejected = fifo.enqueue(ctx_buffer(3)).unwrap_err();
        assert_eq!(rejected.context(), 3);
        assert_eq!(fifo.size(), 2);
    }

    #[test]
    fn unallocated_fifo_rejects_everything() {
        let mut fifo = ArrayFifoBuffer::default();
        assert_eq!(fifo.capacity(), 0);
        assert!(fifo.enqueue(ctx_buffer(1)).is_err());
        assert!(fifo.dequeue().is_none());
    }

    // -- Upstream unit tests (test/ut) ---------------------------------------

    /// C++ `TestErrors.QueueFull`.
    #[test]
    fn errors_queue_full() {
        let t = Tester::new();
        // Go to Accumulate mode.
        assert_eq!(t.comp.mode(), OpState::Drain);
        t.set_mode(OpState::Accumulate);
        assert_eq!(t.comp.mode(), OpState::Accumulate);
        assert!(t.port_history().is_empty());

        // Fill up the buffer queue.
        for i in 0..MAX_NUM_BUFFERS {
            t.fill(i);
            assert!(t.port_history().is_empty());
        }

        // Send another buffer and expect an event; the dropped buffer is
        // returned to its sender.
        t.fill(100);
        assert_eq!(t.events().len(), 1);
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_QUEUEFULL).len(),
            1
        );
        assert_eq!(t.returns(), vec![100]);

        // Send another buffer and expect no new event, but the buffer is
        // still returned.
        t.fill(101);
        assert_eq!(t.events().len(), 1);
        assert_eq!(t.returns(), vec![100, 101]);

        // Drain one buffer.
        t.set_mode(OpState::Drain);
        assert_eq!(t.port_history().len(), 3);
        assert_eq!(t.drains(), vec![0]);

        // Send another buffer and expect an event.
        t.fill(102);
        assert_eq!(t.events().len(), 2);
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_BUFFERACCEPTED)
                .len(),
            1
        );

        // Return the original buffer in order to drain one buffer.
        assert_eq!(t.return_oldest(), 0);
        assert_eq!(t.port_history().len(), 5);
        assert_eq!(t.drains(), vec![0, 1]);

        // Send another buffer and expect no new event.
        t.fill(103);
        assert_eq!(t.events().len(), 2);

        // Send another buffer and expect an event.
        t.fill(104);
        assert_eq!(t.events().len(), 3);
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_QUEUEFULL).len(),
            2
        );
        assert_eq!(t.returns(), vec![100, 101, 0, 104]);
        // Severities per Events.fppi.
        let severities: Vec<LogSeverity> = t.events().iter().map(|e| e.1).collect();
        assert_eq!(
            severities,
            vec![
                LogSeverity::WarningHi,
                LogSeverity::ActivityHi,
                LogSeverity::WarningHi
            ]
        );
    }

    /// C++ `TestErrors.PartialDrain`.
    #[test]
    fn errors_partial_drain() {
        let t = Tester::new();
        assert_eq!(t.comp.mode(), OpState::Drain);
        // Fails — we are still in DRAIN mode.
        t.drain_buffers(1, BlockMode::Block);
        assert_eq!(t.comp.mode(), OpState::Drain);
        assert!(t.port_history().is_empty());
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_ALREADYDRAINING)
                .len(),
            1
        );
        assert_eq!(t.state(|s| (s.num_drained, s.num_to_drain)), (0, 0));
        assert_eq!(
            t.responses(),
            vec![(DRAIN_BUFFERS, 0, CmdResponse::ValidationError)]
        );

        t.set_mode(OpState::Accumulate);
        assert_eq!(t.comp.mode(), OpState::Accumulate);
        assert!(t.port_history().is_empty());

        // Succeeds — now we are in ACCUMULATE — but the queue is empty.
        t.drain_buffers(10, BlockMode::Block);
        assert_eq!(t.comp.mode(), OpState::Accumulate);
        assert!(t.port_history().is_empty());
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_DRAINSTALLED),
            vec![u32_args(&[0, 10])]
        );
        assert!(
            t.events_with(BufferAccumulator::EVENTID_BA_PARTIALDRAINDONE)
                .is_empty()
        );
        assert!(t.state(|s| s.send));
        assert_eq!(t.state(|s| (s.num_drained, s.num_to_drain)), (0, 10));

        // Fails — a partial drain is still in progress.
        t.drain_buffers(1, BlockMode::Block);
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_STILLDRAINING),
            vec![u32_args(&[0, 10])]
        );
        assert!(
            t.events_with(BufferAccumulator::EVENTID_BA_PARTIALDRAINDONE)
                .is_empty()
        );
        assert_eq!(t.comp.mode(), OpState::Accumulate);
        assert!(t.port_history().is_empty());
        assert!(t.state(|s| s.send));
        assert_eq!(t.state(|s| (s.num_drained, s.num_to_drain)), (0, 10));
        // The stalled drain is still open; the rejected one answered BUSY.
        assert_eq!(
            t.responses(),
            vec![
                (DRAIN_BUFFERS, 0, CmdResponse::ValidationError),
                (SET_MODE, 0, CmdResponse::Ok),
                (DRAIN_BUFFERS, 0, CmdResponse::Busy),
            ]
        );
    }

    /// C++ `TestAccumulate.OK`.
    #[test]
    fn accumulate_ok() {
        let t = Tester::new();
        assert_eq!(t.comp.mode(), OpState::Drain);
        t.set_mode(OpState::Accumulate);
        assert_eq!(t.comp.mode(), OpState::Accumulate);
        assert!(t.port_history().is_empty());

        for i in 0..MAX_NUM_BUFFERS {
            t.fill(i);
            assert!(t.port_history().is_empty());
        }

        t.set_mode(OpState::Drain);
        assert_eq!(t.comp.mode(), OpState::Drain);
        assert_eq!(t.port_history().len(), 1);
        assert_eq!(t.drains(), vec![0]);

        let mut expected_num_buffers = 1usize;
        for i in 0..MAX_NUM_BUFFERS {
            assert_eq!(t.return_oldest(), i);
            expected_num_buffers += 1;
            if i + 1 < MAX_NUM_BUFFERS {
                expected_num_buffers += 1;
                assert_eq!(t.drains().len(), i as usize + 2);
                assert_eq!(t.drains()[i as usize + 1], i + 1);
            }
            assert_eq!(t.port_history().len(), expected_num_buffers);
            assert_eq!(t.returns().len(), i as usize + 1);
            assert_eq!(t.returns()[i as usize], i);
        }

        assert_eq!(2 * MAX_NUM_BUFFERS as usize, expected_num_buffers);
        let all: Vec<u32> = (0..MAX_NUM_BUFFERS).collect();
        assert_eq!(t.drains(), all);
        assert_eq!(t.returns(), all);
    }

    /// C++ `TestDrain.OK`.
    #[test]
    fn drain_ok() {
        let t = Tester::new();
        assert_eq!(t.comp.mode(), OpState::Drain);
        for i in 0..MAX_NUM_BUFFERS {
            assert_eq!(t.drains().len(), i as usize);
            t.fill(i);
            assert_eq!(t.drains().len(), i as usize + 1);
            assert_eq!(t.drains()[i as usize], i);
            assert_eq!(t.return_oldest(), i);
            assert_eq!(t.returns()[i as usize], i);
        }
    }

    /// C++ `TestDrain.InitialMode`.
    #[test]
    fn drain_initial_mode() {
        let t = Tester::with_allocation(false);
        assert_eq!(t.comp.mode(), OpState::Accumulate);
        assert!(!t.state(|s| s.send));

        t.comp
            .allocate_queue(0, FwSizeType::from(MAX_NUM_BUFFERS), OpState::Drain);
        t.fill(7);

        assert_eq!(t.comp.mode(), OpState::Drain);
        assert_eq!(t.drains(), vec![7]);

        t.comp.deallocate_queue();
        assert_eq!(t.comp.queue_capacity(), 0);
    }

    /// C++ `TestPartialDrain.OK`.
    #[test]
    fn partial_drain_ok() {
        let t = Tester::new();
        t.set_mode(OpState::Accumulate);
        assert_eq!(t.responses(), vec![(SET_MODE, 0, CmdResponse::Ok)]);
        assert_eq!(t.comp.mode(), OpState::Accumulate);
        assert!(t.port_history().is_empty());

        let done = BufferAccumulator::EVENTID_BA_PARTIALDRAINDONE;
        for i in 0..MAX_NUM_BUFFERS {
            let i_us = i as usize;
            t.fill(i);
            t.drain_buffers(1, BlockMode::Block);
            assert_eq!(t.events_with(done).len(), i_us + 1);
            assert_eq!(t.events_with(done)[i_us], u32_args(&[1]));
            // + 1 for the first ACCUMULATE command; + 1 for the buffer
            // drained immediately.
            assert_eq!(t.responses().len(), i_us + 2);
            assert_eq!(t.responses()[i_us + 1], (DRAIN_BUFFERS, 0, CmdResponse::Ok));
            // One buffer drained.
            assert_eq!(t.drains().len(), i_us + 1);
            assert_eq!(t.drains()[i_us], i);

            assert_eq!(t.return_oldest(), i);
            assert_eq!(t.returns()[i_us], i);

            assert_eq!(t.events_with(done).len(), i_us + 1);
            assert_eq!(t.responses().len(), i_us + 2);
            // ONLY one buffer drained.
            assert_eq!(t.drains().len(), i_us + 1);
        }

        t.clear_history();
        // Refill buffers.
        for i in 0..MAX_NUM_BUFFERS {
            t.fill(i);
            assert!(t.port_history().is_empty());
        }

        assert!(t.responses().is_empty());
        for i in 0..MAX_NUM_BUFFERS {
            let i_us = i as usize;
            assert_eq!(t.drains().len(), i_us);
            t.drain_buffers(1, BlockMode::Block);
            assert_eq!(t.responses().len(), i_us + 1);
            assert_eq!(t.responses()[i_us], (DRAIN_BUFFERS, 0, CmdResponse::Ok));

            // One buffer drained.
            assert_eq!(t.drains().len(), i_us + 1);
            assert_eq!(t.drains()[i_us], i);
            assert_eq!(t.return_oldest(), i);
            assert_eq!(t.returns()[i_us], i);

            assert_eq!(t.events_with(done).len(), i_us + 1);
            assert_eq!(t.events_with(done)[i_us], u32_args(&[1]));

            // ONLY one buffer drained.
            assert_eq!(t.drains().len(), i_us + 1);
            assert_eq!(t.responses().len(), i_us + 1);
        }

        t.clear_history();
        // Refill buffers.
        for i in 0..MAX_NUM_BUFFERS {
            t.fill(i);
            assert!(t.port_history().is_empty());
        }

        assert_eq!(t.comp.mode(), OpState::Accumulate);
        assert!(t.port_history().is_empty());
        assert_eq!(t.state(|s| (s.num_drained, s.num_to_drain)), (0, 0));

        assert!(t.events_with(done).is_empty());
        t.drain_buffers(MAX_NUM_BUFFERS, BlockMode::Block);
        assert!(t.responses().is_empty());

        for i in 0..MAX_NUM_BUFFERS {
            let i_us = i as usize;
            if i + 1 < MAX_NUM_BUFFERS {
                assert_eq!(t.state(|s| s.num_drained), FwSizeType::from(i + 1));
                assert_eq!(
                    t.state(|s| s.num_to_drain),
                    FwSizeType::from(MAX_NUM_BUFFERS)
                );
                assert!(t.events_with(done).is_empty());
            }
            // One buffer drained.
            assert_eq!(t.drains().len(), i_us + 1);
            assert_eq!(t.drains()[i_us], i);
            assert_eq!(t.return_oldest(), i);
            assert_eq!(t.returns()[i_us], i);
        }

        assert_eq!(t.responses(), vec![(DRAIN_BUFFERS, 0, CmdResponse::Ok)]);
        assert_eq!(t.events_with(done), vec![u32_args(&[MAX_NUM_BUFFERS])]);
    }

    /// C++ `TestHealth.Ping`.
    #[test]
    fn health_ping() {
        let t = Tester::new();
        let port = t.comp.ping_in(0);
        port.target.invoke(port.port_num, 42);
        t.do_dispatch();
        assert!(t.events().is_empty());
        assert_eq!(t.port_history(), vec![FromPort::Ping(42)]);
    }

    // -- Additional coverage ---------------------------------------------------

    /// NOBLOCK with fewer buffers than requested drains only what is
    /// queued: `BA_NonBlockDrain(have, requested)`, then the partial drain
    /// completes as soon as the last of those is SENT.
    #[test]
    fn non_block_drain_drains_only_what_is_queued() {
        let t = Tester::new();
        t.set_mode(OpState::Accumulate);
        t.fill(0);
        t.fill(1);
        t.drain_buffers(5, BlockMode::NoBlock);
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_NONBLOCKDRAIN),
            vec![u32_args(&[2, 5])]
        );
        assert_eq!(t.drains(), vec![0]);
        assert_eq!(t.responses().len(), 1); // only SetMode so far
        assert_eq!(t.return_oldest(), 0);
        assert_eq!(t.drains(), vec![0, 1]);
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_PARTIALDRAINDONE),
            vec![u32_args(&[2])]
        );
        assert_eq!(
            t.responses(),
            vec![
                (SET_MODE, 0, CmdResponse::Ok),
                (DRAIN_BUFFERS, 0, CmdResponse::Ok)
            ]
        );
        // The second buffer coming back does not drain anything further.
        t.fill(2);
        assert_eq!(t.return_oldest(), 1);
        assert_eq!(t.drains(), vec![0, 1]);
        assert_eq!(t.comp.num_queued_buffers(), 1);
    }

    /// NOBLOCK on an empty queue and a zero-count drain both finish
    /// immediately with `BA_PartialDrainDone(0)` and `OK`.
    #[test]
    fn empty_and_zero_drains_finish_immediately() {
        let t = Tester::new();
        t.set_mode(OpState::Accumulate);
        t.drain_buffers(3, BlockMode::NoBlock);
        t.drain_buffers(0, BlockMode::Block);
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_NONBLOCKDRAIN),
            vec![u32_args(&[0, 3])]
        );
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_PARTIALDRAINDONE),
            vec![u32_args(&[0]), u32_args(&[0])]
        );
        assert_eq!(
            t.responses(),
            vec![
                (SET_MODE, 0, CmdResponse::Ok),
                (DRAIN_BUFFERS, 0, CmdResponse::Ok),
                (DRAIN_BUFFERS, 0, CmdResponse::Ok),
            ]
        );
        assert!(t.port_history().is_empty());
    }

    /// `BA_SetMode` cancels a stalled partial drain and answers the
    /// ORIGINAL `BA_DrainBuffers` (its own opcode and sequence) first.
    #[test]
    fn set_mode_cancels_a_partial_drain_and_answers_it() {
        let t = Tester::new();
        t.set_mode(OpState::Accumulate);
        let mut args = 4u32.to_be_bytes().to_vec();
        args.push(BlockMode::Block.as_repr());
        t.send_cmd(DRAIN_BUFFERS, 77, &args);
        assert_eq!(t.state(|s| s.num_to_drain), 4);
        t.send_cmd(SET_MODE, 78, &[OpState::Accumulate.as_repr()]);
        assert_eq!(
            t.responses(),
            vec![
                (SET_MODE, 0, CmdResponse::Ok),
                (DRAIN_BUFFERS, 77, CmdResponse::Ok),
                (SET_MODE, 78, CmdResponse::Ok),
            ]
        );
        assert_eq!(t.state(|s| (s.num_drained, s.num_to_drain)), (0, 0));
        assert!(!t.state(|s| s.send));
        // The cancelled drain no longer wakes on a new buffer.
        t.fill(1);
        assert!(t.drains().is_empty());
    }

    /// A stalled BLOCK drain resumes when the next buffer is filled.
    #[test]
    fn stalled_block_drain_resumes_on_fill() {
        let t = Tester::new();
        t.set_mode(OpState::Accumulate);
        t.drain_buffers(1, BlockMode::Block);
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_DRAINSTALLED)
                .len(),
            1
        );
        t.fill(9);
        assert_eq!(t.drains(), vec![9]);
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_PARTIALDRAINDONE),
            vec![u32_args(&[1])]
        );
        assert_eq!(
            t.responses().last(),
            Some(&(DRAIN_BUFFERS, 0, CmdResponse::Ok))
        );
    }

    /// In DRAIN mode only one buffer is ever out: further fills queue
    /// until the outstanding one is returned. Entering DRAIN while a
    /// buffer is out does not send another.
    #[test]
    fn drain_mode_keeps_one_buffer_outstanding() {
        let t = Tester::new();
        t.fill(0);
        t.fill(1);
        t.fill(2);
        assert_eq!(t.drains(), vec![0]);
        assert_eq!(t.comp.num_queued_buffers(), 2);
        t.set_mode(OpState::Accumulate);
        t.set_mode(OpState::Drain);
        assert_eq!(t.drains(), vec![0], "still waiting for buffer 0");
        assert_eq!(t.return_oldest(), 0);
        assert_eq!(t.drains(), vec![0, 1]);
    }

    /// `numDrained` counts full-DRAIN sends too (C++ quirk) — harmless,
    /// because only a partial drain compares it.
    #[test]
    fn full_drain_sends_count_in_num_drained() {
        let t = Tester::new();
        t.fill(0);
        assert_eq!(t.return_oldest(), 0);
        t.fill(1);
        assert_eq!(t.state(|s| (s.num_drained, s.num_to_drain)), (2, 0));
    }

    /// Before `allocate_queue` every fill is a QueueFull drop and the buffer
    /// goes straight back upstream.
    #[test]
    fn unallocated_queue_returns_every_buffer() {
        let t = Tester::with_allocation(false);
        assert_eq!(t.comp.queue_capacity(), 0);
        t.fill(3);
        t.fill(4);
        assert_eq!(t.returns(), vec![3, 4]);
        assert_eq!(
            t.events_with(BufferAccumulator::EVENTID_BA_QUEUEFULL).len(),
            1
        );
    }

    /// `allocate_queue` records the identifier and the initial mode.
    #[test]
    fn allocate_queue_sets_capacity_mode_and_identifier() {
        let t = Tester::with_allocation(false);
        t.comp.allocate_queue(301, 4, OpState::Accumulate);
        assert_eq!(t.comp.queue_capacity(), 4);
        assert_eq!(t.comp.allocator_id(), 301);
        assert_eq!(t.comp.mode(), OpState::Accumulate);
        assert!(!t.state(|s| s.send));
    }

    #[test]
    #[should_panic]
    fn allocate_queue_asserts_on_zero_buffers() {
        let t = Tester::with_allocation(false);
        t.comp.allocate_queue(0, 0, OpState::Drain);
    }

    /// `BA_NumQueuedBuffers` (U32 BE) is written on every fill and every
    /// send attempt — no `update on change`.
    #[test]
    fn num_queued_buffers_is_written_every_time() {
        let t = Tester::new();
        t.set_mode(OpState::Accumulate); // no send: no write
        t.fill(0);
        t.fill(1);
        t.set_mode(OpState::Drain); // send attempt: write 1
        let tlm = t.ground.tlm.lock().unwrap().clone();
        let id = ID_BASE + BufferAccumulator::CHANID_BA_NUMQUEUEDBUFFERS;
        assert_eq!(
            tlm,
            vec![
                (id, 1u32.to_be_bytes().to_vec()),
                (id, 2u32.to_be_bytes().to_vec()),
                (id, 1u32.to_be_bytes().to_vec()),
            ]
        );
    }

    /// Exactly one response per malformed command: short/long args are
    /// FORMAT_ERROR, undeclared enum values VALIDATION_ERROR, unknown
    /// opcodes INVALID_OPCODE.
    #[test]
    fn malformed_commands_are_rejected() {
        let t = Tester::new();
        t.send_cmd(SET_MODE, 1, &[]);
        t.send_cmd(SET_MODE, 2, &[1, 0]);
        t.send_cmd(SET_MODE, 3, &[2]);
        t.send_cmd(DRAIN_BUFFERS, 4, &[0, 0, 0, 1]);
        t.send_cmd(DRAIN_BUFFERS, 5, &[0, 0, 0, 1, 1, 0]);
        t.send_cmd(DRAIN_BUFFERS, 6, &[0, 0, 0, 1, 7]);
        t.send_cmd(ID_BASE + 9, 7, &[]);
        assert_eq!(
            t.responses(),
            vec![
                (SET_MODE, 1, CmdResponse::FormatError),
                (SET_MODE, 2, CmdResponse::FormatError),
                (SET_MODE, 3, CmdResponse::ValidationError),
                (DRAIN_BUFFERS, 4, CmdResponse::FormatError),
                (DRAIN_BUFFERS, 5, CmdResponse::FormatError),
                (DRAIN_BUFFERS, 6, CmdResponse::ValidationError),
                (ID_BASE + 9, 7, CmdResponse::InvalidOpcode),
            ]
        );
        assert_eq!(t.comp.mode(), OpState::Drain);
    }

    /// `pingIn` is `drop`: a ping that finds the queue full is counted and
    /// discarded instead of asserting.
    #[test]
    fn ping_on_a_full_queue_is_dropped() {
        let comp = BufferAccumulator::new("ba");
        comp.init(1);
        let port = comp.ping_in(0);
        port.target.invoke(port.port_num, 1);
        port.target.invoke(port.port_num, 2);
        assert_eq!(comp.active.queued.get_num_msgs_dropped(), 1);
    }

    /// The fill message is `[msg_type i32][port i16][escrow token u64]`.
    #[test]
    fn fill_message_envelope_layout() {
        let t = Tester::new();
        let port = t.comp.buffer_send_in_fill(0);
        port.target.invoke(port.port_num, ctx_buffer(5));
        let mut raw = MsgBuffer::new();
        let mut priority: FwQueuePriorityType = 0;
        let status = t.comp.active.queued.queue().receive_serial(
            &mut raw,
            fprime_os::queue::BlockingType::NonBlocking,
            &mut priority,
        );
        assert_eq!(status, fprime_os::queue::Status::OpOk);
        assert_eq!(raw.get_size(), 4 + 2 + 8);
        assert_eq!(&raw.as_slice()[..6], &[0, 0, 0, 1, 0, 0]);
        let token = u64::from_be_bytes(raw.as_slice()[6..14].try_into().unwrap());
        assert_eq!(t.comp.escrow.claim(token).context(), 5);
    }

    /// Runs on its own thread like any active component.
    #[test]
    fn active_task_round_trip() {
        let t = Tester::new();
        t.comp.active.start(&t.comp, 21, TASK_DEFAULT, TASK_DEFAULT);
        let port = t.comp.buffer_send_in_fill(0);
        port.target.invoke(port.port_num, ctx_buffer(11));
        t.comp.active.exit();
        assert_eq!(t.comp.active.join(), fprime_os::task::Status::OpOk);
        assert_eq!(t.drains(), vec![11]);
    }
}
