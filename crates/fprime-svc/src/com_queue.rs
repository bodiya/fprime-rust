//! # ComQueue — port of `Svc::ComQueue` (active)
//!
//! C++ sources: `Svc/ComQueue/ComQueue.{cpp,hpp,fpp}` + the `.fppi` command/
//! event/telemetry includes.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (ComQueue section + gotchas).
//!
//! Prioritized downlink queueing with one-outstanding-send flow control:
//! the component starts in `WAITING` and transmits nothing until the first
//! `comStatusIn SUCCESS` (emitted by ComStub on driver connect). Each
//! `SUCCESS` releases exactly one buffer, chosen from the highest-priority
//! non-empty queue with round-robin balancing inside equal-priority runs.
//!
//! ## Storage divergences (documented)
//!
//! - C++ persists serialized `Fw::ComBuffer`/`Fw::Buffer` objects in
//!   `Types::Queue` slots. Here com queues store fixed 514-byte records
//!   (`[len u16 BE][bytes][zero pad]`) and buffer queues store 8-byte
//!   [`BufferEscrow`] tokens (the C++ slot serializes the raw data pointer
//!   at the same 8-byte width). The pre-emptive `DROP_OLDEST` pop still
//!   returns ownership via `bufferReturnOut` BEFORE the overwrite.
//! - C++ `sendComBuffer` aliases internal storage `m_dequeued_com_buffer`;
//!   here an owned `Buffer` is built from a recycled `BufferStorage` (the
//!   storage returns through `dataReturnIn`, keeping steady state
//!   allocation-free after the first send).
//! - C++ casts the leading packet descriptor to `Apid` unchecked; a Rust
//!   enum cannot hold arbitrary values, so unknown descriptors map to
//!   `Apid::InvalidUninitialized`.

use fprime_comp::{
    ActiveBase, ActiveComponent, BufferEscrow, BufferSendPort, CmdGlue, CmdPort,
    ComDataWithContextPort, ComPort, ComponentDispatch, EventGlue, MsgDispatchStatus, OutputPort,
    PortRef, QueueFullPolicy, SchedPort, SuccessConditionPort, TlmGlue, msg,
};
use fprime_config::{
    FW_COM_BUFFER_MAX_SIZE, FwChanIdType, FwEnumStoreType, FwEventIdType, FwIndexType,
    FwOpcodeType, FwPacketDescriptorType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::buffer::BufferStorage;
use fprime_fw::{
    Apid, Buffer, CmdArgBuffer, CmdResponse, ComBuffer, Endianness, FrameContext, LinearBuffer,
    LogSeverity, SerBufAny, Serialize, SerializeStatus, Success, fw_assert,
};
use fprime_fw::{SerBuf, fw_try};
use fprime_utils::{Queue as TypesQueue, QueueMode, QueueOverflowMode};
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Dictionary constants
// ---------------------------------------------------------------------------

/// Count of `Fw.Com` input ports / com queues (`ComQueueComPorts`).
pub const COM_PORT_COUNT: usize = fprime_config::com_queue::COM_PORTS;
/// Count of `Fw.Buffer` input ports / buffer queues (`ComQueueBufferPorts`).
pub const BUFFER_PORT_COUNT: usize = fprime_config::com_queue::BUFFER_PORTS;
/// Total queue count.
pub const TOTAL_PORT_COUNT: usize = COM_PORT_COUNT + BUFFER_PORT_COUNT;

/// Queue message types (0 = EXIT sentinel).
const MSG_TYPE_COM_STATUS_IN: FwEnumStoreType = 1;
const MSG_TYPE_COM_PACKET_QUEUE_IN: FwEnumStoreType = 2;
const MSG_TYPE_BUFFER_QUEUE_IN: FwEnumStoreType = 3;
const MSG_TYPE_RUN: FwEnumStoreType = 4;
const MSG_TYPE_CMD_IN: FwEnumStoreType = 5;

/// Relative command opcodes (FPP declaration order).
pub const OPCODE_FLUSH_QUEUE: FwOpcodeType = 0;
pub const OPCODE_FLUSH_ALL_QUEUES: FwOpcodeType = 1;
pub const OPCODE_SET_QUEUE_PRIORITY: FwOpcodeType = 2;

/// Relative event ids.
pub const EVENTID_QUEUE_OVERFLOW: FwEventIdType = 0;
pub const EVENTID_QUEUE_PRIORITY_CHANGED: FwEventIdType = 1;

/// Telemetry channel ids (explicit in the FPP).
pub const CHANID_COM_QUEUE_DEPTH: FwChanIdType = 0;
pub const CHANID_BUFF_QUEUE_DEPTH: FwChanIdType = 1;

/// Queue message size: max over async invocations (the com-packet message:
/// 6-byte envelope + [u16 len + 512] nested ComBuffer + u32 context).
pub const MSG_SIZE: FwSizeType =
    (msg::ENVELOPE_HEADER_SIZE + 2 + FW_COM_BUFFER_MAX_SIZE + 4) as FwSizeType;

/// All async inputs share one priority (FIFO dispatch; EXIT is priority 0).
const PORT_PRIORITY: FwQueuePriorityType = 1;

/// Internal record sizes (see the module-header divergence note).
const COM_RECORD_SIZE: usize = 2 + FW_COM_BUFFER_MAX_SIZE;
const BUFFER_RECORD_SIZE: usize = 8;

/// `Svc::QueueType` (FPP enum, repr U8).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum QueueType {
    /// An `Fw::ComBuffer` queue.
    #[default]
    ComQueue = 0,
    /// An `Fw::Buffer` queue.
    BufferQueue = 1,
}

impl TryFrom<u8> for QueueType {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, ()> {
        match value {
            0 => Ok(Self::ComQueue),
            1 => Ok(Self::BufferQueue),
            _ => Err(()),
        }
    }
}

/// `Fw::Buffer` ownership handshake states (C++ `BufferState`).
const BUFFER_STATE_OWNED: u8 = 0;
const BUFFER_STATE_UNOWNED: u8 = 1;

/// Flow-control state (C++ `SendState`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SendState {
    /// Ready to send the next priority message.
    Ready,
    /// Waiting for the status of the last sent message.
    Waiting,
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Per-queue configuration (C++ `QueueConfigurationEntry`).
#[derive(Debug, Clone, Copy)]
pub struct QueueConfigurationEntry {
    /// Queue depth in messages (must be > 0 at configure).
    pub depth: FwSizeType,
    /// Priority in `[0, TOTAL_PORT_COUNT)`; LOWER values are serviced first.
    pub priority: FwIndexType,
    /// FIFO or LIFO dequeue order.
    pub mode: QueueMode,
    /// Overflow handling (drop newest or oldest).
    pub overflow_mode: QueueOverflowMode,
}

impl Default for QueueConfigurationEntry {
    /// C++ default-constructed table entry (depth 0 — must be overridden).
    fn default() -> Self {
        Self {
            depth: 0,
            priority: 0,
            mode: QueueMode::Fifo,
            overflow_mode: QueueOverflowMode::DropNewest,
        }
    }
}

/// The per-port configuration table (C++ `QueueConfigurationTable`):
/// entries address the com ports first, then the buffer ports.
#[derive(Debug, Clone, Copy, Default)]
pub struct QueueConfigurationTable {
    /// One entry per queue, com ports first.
    pub entries: [QueueConfigurationEntry; TOTAL_PORT_COUNT],
}

/// Priority-sorted queue metadata (C++ `QueueMetadata`).
#[derive(Debug, Clone, Copy)]
struct QueueMetadata {
    depth: FwSizeType,
    priority: FwIndexType,
    overflow_mode: QueueOverflowMode,
    /// Index of this queue in `queues` (== the configuration entry index).
    index: usize,
}

/// Guarded component state (all touched only from the component thread in a
/// real topology; the mutex provides safe interior mutability).
struct ComQueueState {
    /// Un-prioritized backing queues, indexed by entry index.
    queues: Vec<TypesQueue>,
    /// Priority-sorted metadata referencing `queues` by index.
    prioritized: Vec<QueueMetadata>,
    /// Per-queue overflow-event throttles (cleared by a successful send).
    throttle: [bool; TOTAL_PORT_COUNT],
    send_state: SendState,
}

// ---------------------------------------------------------------------------
// Telemetry array types (FPP `array ComQueueDepth = [N] U32`: raw elements,
// no length prefix)
// ---------------------------------------------------------------------------

/// `Svc.ComQueueDepth` telemetry array.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ComQueueDepth(pub [u32; COM_PORT_COUNT]);

/// `Svc.BuffQueueDepth` telemetry array.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BuffQueueDepth(pub [u32; BUFFER_PORT_COUNT]);

impl Serialize for ComQueueDepth {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        for v in self.0 {
            fw_try!(buf.serialize_u32(v, e));
        }
        SerializeStatus::Ok
    }
    fn serialized_size(&self) -> usize {
        4 * COM_PORT_COUNT
    }
}

impl Serialize for BuffQueueDepth {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        for v in self.0 {
            fw_try!(buf.serialize_u32(v, e));
        }
        SerializeStatus::Ok
    }
    fn serialized_size(&self) -> usize {
        4 * BUFFER_PORT_COUNT
    }
}

// ---------------------------------------------------------------------------
// The component
// ---------------------------------------------------------------------------

/// `Svc::ComQueue` — active prioritized downlink queue.
pub struct ComQueue {
    /// Active core (PassiveBase + queue + task).
    pub active: ActiveBase,
    /// Command registration + response ports.
    pub cmd: CmdGlue,
    /// Event ports and the time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// `dataOut` — data ready to be sent (to the framer).
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `bufferReturnOut` — `Fw::Buffer` ownership back to original senders.
    pub buffer_return_out: [OutputPort<dyn BufferSendPort>; BUFFER_PORT_COUNT],
    /// Escrow for `Fw::Buffer`s riding the message queue / buffer queues.
    escrow: BufferEscrow,
    /// C++ `std::atomic<BufferState>`: exactly one outstanding dataOut
    /// buffer; `dataReturnIn` runs on the returning caller's thread.
    buffer_state: AtomicU8,
    /// Recycled storage for outgoing com buffers (see divergence note).
    recycle: Mutex<Option<BufferStorage>>,
    /// Guarded state.
    state: Mutex<ComQueueState>,
}

impl ComQueue {
    /// Construct the component ([`configure`](Self::configure) before use).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            data_out: OutputPort::new(),
            buffer_return_out: core::array::from_fn(|_| OutputPort::new()),
            escrow: BufferEscrow::new(),
            buffer_state: AtomicU8::new(BUFFER_STATE_OWNED),
            recycle: Mutex::new(None),
            state: Mutex::new(ComQueueState {
                queues: Vec::new(),
                prioritized: Vec::new(),
                throttle: [false; TOTAL_PORT_COUNT],
                send_state: SendState::Waiting,
            }),
        })
    }

    fn id_base(&self) -> u32 {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                OPCODE_FLUSH_QUEUE,
                OPCODE_FLUSH_ALL_QUEUES,
                OPCODE_SET_QUEUE_PRIORITY,
            ],
        );
    }

    /// Configure queue depths, priorities, and storage (C++ `configure`;
    /// the Rust `Types::Queue` owns its ring, so the allocator parameters
    /// are dropped). Asserts each priority is in range and each depth > 0.
    pub fn configure(&self, queue_config: &QueueConfigurationTable) {
        let mut state = self.state.lock().unwrap();
        // Validate every entry up front (C++ asserts inside the loop).
        for (entry_index, entry) in queue_config.entries.iter().enumerate() {
            fw_assert!(
                entry.priority >= 0 && (entry.priority as usize) < TOTAL_PORT_COUNT,
                entry.priority as i32,
                TOTAL_PORT_COUNT as i32,
                entry_index as i32
            );
            fw_assert!(entry.depth > 0, entry_index as i32);
        }
        // Build the priority-sorted metadata list: walk priorities 0..TOTAL,
        // then entries in order — a stable sort by construction.
        state.prioritized.clear();
        for current_priority in 0..TOTAL_PORT_COUNT as FwIndexType {
            for (entry_index, entry) in queue_config.entries.iter().enumerate() {
                if entry.priority == current_priority {
                    state.prioritized.push(QueueMetadata {
                        depth: entry.depth,
                        priority: entry.priority,
                        overflow_mode: entry.overflow_mode,
                        index: entry_index,
                    });
                }
            }
        }
        fw_assert!(state.prioritized.len() == TOTAL_PORT_COUNT);
        // Create the backing queues in entry-index order. Message size is
        // determined by the entry index: com record vs escrow token.
        state.queues = queue_config
            .entries
            .iter()
            .enumerate()
            .map(|(i, entry)| {
                let msg_size = if i < COM_PORT_COUNT {
                    COM_RECORD_SIZE
                } else {
                    BUFFER_RECORD_SIZE
                };
                TypesQueue::new(
                    entry.depth as usize,
                    msg_size,
                    entry.mode,
                    entry.overflow_mode,
                )
            })
            .collect();
    }

    // -- Input-port factories -----------------------------------------------

    /// `comStatusIn` — ASYNC `Fw.SuccessCondition` input (assert policy).
    pub fn com_status_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn SuccessConditionPort> {
        PortRef::new(
            Arc::new(ComStatusInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `comPacketQueueIn[n]` — ASYNC `Fw.Com` input, `drop` policy.
    pub fn com_packet_queue_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComPort> {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < COM_PORT_COUNT,
            port_num as i32
        );
        PortRef::new(
            Arc::new(ComPacketQueueInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `bufferQueueIn[n]` — ASYNC `Fw.BufferSend` input, `hook` policy
    /// (overflow returns the buffer via `bufferReturnOut`).
    pub fn buffer_queue_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn BufferSendPort> {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < BUFFER_PORT_COUNT,
            port_num as i32
        );
        PortRef::new(
            Arc::new(BufferQueueInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `run` — ASYNC `Svc.Sched` input, `drop` policy (telemetry).
    pub fn run_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(Arc::new(RunInAdapter { comp: self.clone() }), port_num)
    }

    /// `dataReturnIn` — SYNC input: the sent buffer coming back. Runs on the
    /// returning caller's thread; touches only the atomic handshake, the
    /// escrowed/recycled storage, and `bufferReturnOut`.
    pub fn data_return_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(
            Arc::new(DataReturnInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `CmdDisp` — ASYNC command input.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }

    // -- Handlers (component thread) ----------------------------------------

    /// `comStatusIn` handler: the flow-control state machine.
    fn com_status_in_handler(&self, _port_num: FwIndexType, condition: Success) {
        let mut state = self.state.lock().unwrap();
        match state.send_state {
            SendState::Waiting => {
                if condition == Success::Success {
                    state.send_state = SendState::Ready;
                    self.process_queue(&mut state);
                    // A message may or may not have been sent; both final
                    // states are acceptable (C++ parity assert shape).
                    fw_assert!(
                        state.send_state == SendState::Waiting
                            || state.send_state == SendState::Ready
                    );
                } else {
                    state.send_state = SendState::Waiting;
                }
            }
            // Receiving a status while READY is a protocol violation.
            _ => {
                fw_assert!(false, state.send_state as i32);
            }
        }
    }

    /// `comPacketQueueIn` handler.
    fn com_packet_queue_in_handler(&self, port_num: FwIndexType, data: &ComBuffer, _context: u32) {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < COM_PORT_COUNT,
            port_num as i32
        );
        let mut state = self.state.lock().unwrap();
        let _ = self.enqueue_com(&mut state, port_num as usize, data);
    }

    /// `bufferQueueIn` handler.
    fn buffer_queue_in_handler(&self, port_num: FwIndexType, buffer: Buffer) {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < BUFFER_PORT_COUNT,
            port_num as i32
        );
        let queue_num = port_num as usize + COM_PORT_COUNT;
        let mut state = self.state.lock().unwrap();
        fw_assert!(queue_num < state.queues.len(), queue_num as i32);

        // Pre-emptive DROP_OLDEST: when the queue is full, pop the oldest
        // entry FIRST and return its ownership before the overwrite —
        // omitting this leaks pool buffers (gotcha).
        let mut pre_emptive_overflow = false;
        let meta = state
            .prioritized
            .iter()
            .find(|m| m.index == queue_num)
            .copied();
        if let Some(meta) = meta {
            if meta.overflow_mode == QueueOverflowMode::DropOldest
                && state.queues[queue_num].get_queue_size() >= meta.depth as usize
            {
                let mut record = [0u8; BUFFER_RECORD_SIZE];
                // popFront always removes the oldest, matching the
                // rotate-based removal Queue::enqueue uses for DROP_OLDEST.
                let dequeue_status = state.queues[queue_num].pop_front(&mut record);
                fw_assert!(dequeue_status.is_ok(), dequeue_status as i32);
                let dropped = self.escrow.claim(u64::from_be_bytes(record));
                let p = self.buffer_return_out[port_num as usize].get();
                p.target.invoke(p.port_num, dropped);
                pre_emptive_overflow = true;
            }
        }

        let token = self.escrow.deposit(buffer);
        let status = state.queues[queue_num].enqueue(&token.to_be_bytes());
        let accepted = self.handle_enqueue_status(
            &mut state,
            queue_num,
            QueueType::BufferQueue,
            port_num,
            pre_emptive_overflow,
            status,
        );
        if !accepted {
            // Rejected (DROP_NEWEST full): return the buffer to its sender.
            let buffer = self.escrow.claim(token);
            let p = self.buffer_return_out[port_num as usize].get();
            p.target.invoke(p.port_num, buffer);
        }
    }

    /// `run` handler: downlink queue-depth high-water marks.
    fn run_handler(&self, _port_num: FwIndexType, _context: u32) {
        let mut state = self.state.lock().unwrap();
        fw_assert!(state.queues.len() == TOTAL_PORT_COUNT);
        let mut com_depth = ComQueueDepth::default();
        for (i, out) in com_depth.0.iter_mut().enumerate() {
            *out = state.queues[i].get_high_water_mark() as u32;
            state.queues[i].clear_high_water_mark();
        }
        let mut buff_depth = BuffQueueDepth::default();
        for (i, out) in buff_depth.0.iter_mut().enumerate() {
            *out = state.queues[i + COM_PORT_COUNT].get_high_water_mark() as u32;
            state.queues[i + COM_PORT_COUNT].clear_high_water_mark();
        }
        drop(state);
        let now = self.evt.time_get();
        self.tlm
            .tlm_write(self.id_base(), CHANID_COM_QUEUE_DEPTH, &com_depth, now);
        self.tlm
            .tlm_write(self.id_base(), CHANID_BUFF_QUEUE_DEPTH, &buff_depth, now);
    }

    /// `dataReturnIn` handler — SYNC, returning caller's thread.
    fn data_return_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        // Take ownership back atomically (exactly-one-outstanding protocol).
        let previous = self.buffer_state.swap(BUFFER_STATE_OWNED, Ordering::AcqRel);
        fw_assert!(previous == BUFFER_STATE_UNOWNED, previous as i32);
        // Buffer-queue sends carry comQueueIndex >= COM_PORT_COUNT; a
        // modified apid/index here would be a coding error (C++ parity).
        let buffer_return_port_num = context.com_queue_index - COM_PORT_COUNT as FwIndexType;
        fw_assert!(
            buffer_return_port_num < BUFFER_PORT_COUNT as FwIndexType,
            buffer_return_port_num as i32
        );
        if buffer_return_port_num >= 0 {
            // It is a coding error not to connect the paired return port.
            let p = self.buffer_return_out[buffer_return_port_num as usize].get();
            p.target.invoke(p.port_num, data);
        } else {
            // Com-buffer return: the data was copied at enqueue time, so
            // C++ drops it here; we recycle the storage for the next send.
            *self.recycle.lock().unwrap() = Some(data.into_storage());
        }
    }

    // -- Command handlers ----------------------------------------------------

    /// C++ `getQueueNum`.
    fn get_queue_num(queue_type: QueueType, port_num: FwIndexType) -> FwIndexType {
        port_num
            + match queue_type {
                QueueType::ComQueue => 0,
                QueueType::BufferQueue => COM_PORT_COUNT as FwIndexType,
            }
    }

    /// Drain one queue, returning buffer-queue entries via `bufferReturnOut`.
    fn drain_queue(&self, state: &mut ComQueueState, index: usize) {
        fw_assert!(index < TOTAL_PORT_COUNT, index as i32);
        let available = state.queues[index].get_queue_size();
        for _ in 0..available {
            if index < COM_PORT_COUNT {
                let mut record = [0u8; COM_RECORD_SIZE];
                let status = state.queues[index].dequeue(&mut record);
                fw_assert!(status.is_ok(), status as i32);
            } else {
                let mut record = [0u8; BUFFER_RECORD_SIZE];
                let status = state.queues[index].dequeue(&mut record);
                fw_assert!(status.is_ok(), status as i32);
                let buffer = self.escrow.claim(u64::from_be_bytes(record));
                let p = self.buffer_return_out[index - COM_PORT_COUNT].get();
                p.target.invoke(p.port_num, buffer);
            }
        }
    }

    /// `FLUSH_QUEUE(queueType, indexType)`.
    fn flush_queue_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        queue_type: QueueType,
        index: FwIndexType,
    ) {
        let queue_index = Self::get_queue_num(queue_type, index);
        if queue_index < 0 || queue_index as usize >= TOTAL_PORT_COUNT {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        let mut state = self.state.lock().unwrap();
        self.drain_queue(&mut state, queue_index as usize);
        drop(state);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `FLUSH_ALL_QUEUES()`.
    fn flush_all_queues_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        let mut state = self.state.lock().unwrap();
        for i in 0..TOTAL_PORT_COUNT {
            self.drain_queue(&mut state, i);
        }
        drop(state);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `SET_QUEUE_PRIORITY(queueType, indexType, newPriority)`.
    fn set_queue_priority_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        queue_type: QueueType,
        index: FwIndexType,
        new_priority: FwIndexType,
    ) {
        let queue_index = Self::get_queue_num(queue_type, index);
        if queue_index < 0 || queue_index as usize >= TOTAL_PORT_COUNT {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        if new_priority < 0 || new_priority as usize >= TOTAL_PORT_COUNT {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
            return;
        }
        {
            let mut state = self.state.lock().unwrap();
            // Update the matching entry (at most one per queue index).
            for meta in state.prioritized.iter_mut() {
                if meta.index == queue_index as usize {
                    meta.priority = new_priority;
                    break;
                }
            }
            // Re-sort by priority: the exact C++ bubble sort (stable — only
            // strictly-greater neighbors swap).
            let n = state.prioritized.len();
            for i in 0..n.saturating_sub(1) {
                for j in 0..(n - i - 1) {
                    if state.prioritized[j].priority > state.prioritized[j + 1].priority {
                        state.prioritized.swap(j, j + 1);
                    }
                }
            }
        }
        // Emit event for the successful priority change (ACTIVITY_HI).
        self.evt.log_event(
            self.id_base(),
            EVENTID_QUEUE_PRIORITY_CHANGED,
            LogSeverity::ActivityHi,
            &format!("{queue_type:?} {index} priority changed to {new_priority}"),
            |buf| {
                fw_try!(buf.serialize_u8_be(queue_type as u8));
                fw_try!(buf.serialize_i16_be(index));
                buf.serialize_i16_be(new_priority)
            },
        );
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    // -- Helpers -------------------------------------------------------------

    /// Enqueue a com record; emits the overflow event when rejected.
    fn enqueue_com(&self, state: &mut ComQueueState, queue_num: usize, data: &ComBuffer) -> bool {
        fw_assert!(queue_num < COM_PORT_COUNT, queue_num as i32);
        fw_assert!(queue_num < state.queues.len(), queue_num as i32);
        let bytes = data.as_slice();
        fw_assert!(bytes.len() <= FW_COM_BUFFER_MAX_SIZE, bytes.len() as i32);
        let mut record = [0u8; COM_RECORD_SIZE];
        record[0..2].copy_from_slice(&(bytes.len() as u16).to_be_bytes());
        record[2..2 + bytes.len()].copy_from_slice(bytes);
        let status = state.queues[queue_num].enqueue(&record);
        self.handle_enqueue_status(
            state,
            queue_num,
            QueueType::ComQueue,
            queue_num as FwIndexType,
            false,
            status,
        )
    }

    /// C++ `handleEnqueueStatus`: overflow event + throttle, then immediate
    /// processing when READY. Returns whether the message was accepted.
    fn handle_enqueue_status(
        &self,
        state: &mut ComQueueState,
        queue_num: usize,
        queue_type: QueueType,
        port_num: FwIndexType,
        pre_emptive_overflow: bool,
        status: SerializeStatus,
    ) -> bool {
        let overflowed = pre_emptive_overflow
            || status == SerializeStatus::NoRoomLeft
            || status == SerializeStatus::DiscardedExisting;
        if overflowed && !state.throttle[queue_num] {
            let type_name = match queue_type {
                QueueType::ComQueue => "COM_QUEUE",
                QueueType::BufferQueue => "BUFFER_QUEUE",
            };
            self.evt.log_event(
                self.id_base(),
                EVENTID_QUEUE_OVERFLOW,
                LogSeverity::WarningHi,
                &format!("The {type_name} queue at index {port_num} overflowed"),
                |buf| {
                    fw_try!(buf.serialize_u8_be(queue_type as u8));
                    buf.serialize_i16_be(port_num)
                },
            );
            state.throttle[queue_num] = true;
        }
        // Already READY: send the next available message immediately.
        if state.send_state == SendState::Ready {
            self.process_queue(state);
        }
        status != SerializeStatus::NoRoomLeft
    }

    /// C++ `processQueue`: send the first message available in priority
    /// order, then round-robin the equal-priority run.
    fn process_queue(&self, state: &mut ComQueueState) {
        fw_assert!(state.send_state == SendState::Ready);
        let total = state.prioritized.len();
        let mut send_priority: FwIndexType = 0;
        let mut priority_index = 0usize;
        while priority_index < total {
            let entry = state.prioritized[priority_index];
            if state.queues[entry.index].get_queue_size() == 0 {
                priority_index += 1;
                continue;
            }
            if entry.index < COM_PORT_COUNT {
                fw_assert!(self.buffer_state.load(Ordering::Acquire) == BUFFER_STATE_OWNED);
                let mut record = [0u8; COM_RECORD_SIZE];
                let dequeue_status = state.queues[entry.index].dequeue(&mut record);
                fw_assert!(dequeue_status.is_ok(), dequeue_status as i32);
                self.send_com_record(state, &record, entry.index);
            } else {
                let mut record = [0u8; BUFFER_RECORD_SIZE];
                let dequeue_status = state.queues[entry.index].dequeue(&mut record);
                fw_assert!(dequeue_status.is_ok(), dequeue_status as i32);
                let buffer = self.escrow.claim(u64::from_be_bytes(record));
                self.send_buffer(state, buffer, entry.index);
            }
            // Successful send clears this queue's overflow throttle.
            state.throttle[entry.index] = false;
            send_priority = entry.priority;
            break;
        }
        // Rotate the dispatched entry to the end of its equal-priority run
        // (round-robin — exact C++ swap loop).
        priority_index += 1;
        while priority_index < total && state.prioritized[priority_index].priority == send_priority
        {
            state.prioritized.swap(priority_index, priority_index - 1);
            priority_index += 1;
        }
    }

    /// C++ `sendComBuffer` (see the storage divergence note).
    fn send_com_record(
        &self,
        state: &mut ComQueueState,
        record: &[u8; COM_RECORD_SIZE],
        queue_index: usize,
    ) {
        fw_assert!(state.send_state == SendState::Ready);
        let len = u16::from_be_bytes([record[0], record[1]]) as usize;
        // Recycled storage (allocated once; returns via dataReturnIn).
        let storage = {
            self.recycle
                .lock()
                .unwrap()
                .take()
                .unwrap_or_else(|| vec![0u8; FW_COM_BUFFER_MAX_SIZE].into_boxed_slice())
        };
        let mut buffer = Buffer::from_storage(storage, Buffer::NO_CONTEXT);
        buffer.data_mut()[..len].copy_from_slice(&record[2..2 + len]);
        buffer.set_size(len);
        self.send_prepared(state, buffer, queue_index);
    }

    /// C++ `sendBuffer`.
    fn send_buffer(&self, state: &mut ComQueueState, buffer: Buffer, queue_index: usize) {
        fw_assert!(state.send_state == SendState::Ready);
        self.send_prepared(state, buffer, queue_index);
    }

    /// Shared tail of sendComBuffer/sendBuffer: read the leading packet
    /// descriptor into the context, flip the ownership atomic, send, WAIT.
    fn send_prepared(&self, state: &mut ComQueueState, mut buffer: Buffer, queue_index: usize) {
        let mut descriptor: FwPacketDescriptorType = 0;
        {
            let mut deser = buffer.get_deserializer();
            let status = deser.deserialize_u16_be(&mut descriptor);
            // Queued data always carries a descriptor (C++ asserts).
            fw_assert!(status.is_ok(), status as i32);
        }
        // C++ casts the descriptor to Apid unchecked; unknown values map to
        // INVALID_UNINITIALIZED here (divergence note in module header).
        let context = FrameContext {
            apid: Apid::try_from(descriptor).unwrap_or(Apid::InvalidUninitialized),
            com_queue_index: queue_index as FwIndexType,
            ..FrameContext::default()
        };
        let previous = self
            .buffer_state
            .swap(BUFFER_STATE_UNOWNED, Ordering::AcqRel);
        fw_assert!(previous == BUFFER_STATE_OWNED, previous as i32);
        let p = self.data_out.get();
        p.target.invoke(p.port_num, buffer, &context);
        // Wait for the status to come back (one outstanding send).
        state.send_state = SendState::Waiting;
    }
}

// ---------------------------------------------------------------------------
// Async input adapters (queue envelopes)
// ---------------------------------------------------------------------------

type MsgBuffer = LinearBuffer<{ MSG_SIZE as usize }>;

/// `comStatusIn` — async, default (assert) policy.
struct ComStatusInAdapter {
    comp: Arc<ComQueue>,
}

impl SuccessConditionPort for ComStatusInAdapter {
    fn invoke(&self, port_num: FwIndexType, condition: &mut Success) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_COM_STATUS_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = condition.serialize_to(&mut buf, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// `comPacketQueueIn[n]` — async, `drop` policy.
struct ComPacketQueueInAdapter {
    comp: Arc<ComQueue>,
}

impl ComPort for ComPacketQueueInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_COM_PACKET_QUEUE_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(data, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(context);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Drop);
    }
}

/// `bufferQueueIn[n]` — async, `hook` policy: on a full message queue the
/// overflow hook returns the buffer via `bufferReturnOut` (C++
/// `bufferQueueIn_overflowHook`).
struct BufferQueueInAdapter {
    comp: Arc<ComQueue>,
}

impl BufferSendPort for BufferQueueInAdapter {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        let token = self.comp.escrow.deposit(buffer);
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_BUFFER_QUEUE_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u64_be(token);
        fw_assert!(status.is_ok(), status as i32);
        let send_status =
            self.comp
                .active
                .queued
                .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Hook);
        if send_status != fprime_os::queue::Status::OpOk {
            // Overflow hook: reclaim and return ownership.
            fw_assert!(
                port_num >= 0 && (port_num as usize) < BUFFER_PORT_COUNT,
                port_num as i32
            );
            let buffer = self.comp.escrow.claim(token);
            let p = self.comp.buffer_return_out[port_num as usize].get();
            p.target.invoke(p.port_num, buffer);
        }
    }
}

/// `run` — async, `drop` policy.
struct RunInAdapter {
    comp: Arc<ComQueue>,
}

impl SchedPort for RunInAdapter {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_RUN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(context);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Drop);
    }
}

/// `CmdDisp` — async command input, default (assert) policy.
struct CmdInAdapter {
    comp: Arc<ComQueue>,
}

impl CmdPort for CmdInAdapter {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_CMD_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(op_code);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(cmd_seq);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(args, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// `dataReturnIn` — sync adapter.
struct DataReturnInAdapter {
    comp: Arc<ComQueue>,
}

impl ComDataWithContextPort for DataReturnInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.comp.data_return_in_handler(port_num, data, context);
    }
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

impl ComponentDispatch for ComQueue {
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
            MSG_TYPE_COM_STATUS_IN => {
                let mut condition = Success::Failure;
                if !buf.deserialize(&mut condition, Endianness::Big).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.com_status_in_handler(port_num, condition);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_COM_PACKET_QUEUE_IN => {
                let mut data = ComBuffer::new();
                let mut context = 0u32;
                if !buf.deserialize_buffer(&mut data, Endianness::Big).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                if !buf.deserialize_u32_be(&mut context).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.com_packet_queue_in_handler(port_num, &data, context);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_BUFFER_QUEUE_IN => {
                let mut token = 0u64;
                if !buf.deserialize_u64_be(&mut token).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                let buffer = self.escrow.claim(token);
                self.buffer_queue_in_handler(port_num, buffer);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_RUN => {
                let mut context = 0u32;
                if !buf.deserialize_u32_be(&mut context).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.run_handler(port_num, context);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_CMD_IN => {
                let mut op_code: FwOpcodeType = 0;
                let mut cmd_seq = 0u32;
                let mut args = CmdArgBuffer::new();
                if !buf.deserialize_u32_be(&mut op_code).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                if !buf.deserialize_u32_be(&mut cmd_seq).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                if !buf.deserialize_buffer(&mut args, Endianness::Big).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.handle_command(op_code, cmd_seq, &mut args);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ComQueue {
    /// Command dispatch on the local opcode (exactly-once response
    /// discipline: FormatError on deserialization failure or residual
    /// bytes, ValidationError on invalid enum args).
    fn handle_command(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        match op_code.wrapping_sub(self.id_base()) {
            OPCODE_FLUSH_QUEUE => {
                let mut queue_type_raw = 0u8;
                let mut index: FwIndexType = 0;
                if !args.deserialize_u8_be(&mut queue_type_raw).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                let queue_type = match QueueType::try_from(queue_type_raw) {
                    Ok(t) => t,
                    Err(()) => {
                        self.cmd
                            .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
                        return;
                    }
                };
                if !args.deserialize_i16_be(&mut index).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.flush_queue_cmd_handler(op_code, cmd_seq, queue_type, index);
            }
            OPCODE_FLUSH_ALL_QUEUES => {
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.flush_all_queues_cmd_handler(op_code, cmd_seq);
            }
            OPCODE_SET_QUEUE_PRIORITY => {
                let mut queue_type_raw = 0u8;
                let mut index: FwIndexType = 0;
                let mut new_priority: FwIndexType = 0;
                if !args.deserialize_u8_be(&mut queue_type_raw).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                let queue_type = match QueueType::try_from(queue_type_raw) {
                    Ok(t) => t,
                    Err(()) => {
                        self.cmd
                            .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
                        return;
                    }
                };
                if !args.deserialize_i16_be(&mut index).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                if !args.deserialize_i16_be(&mut new_priority).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.set_queue_priority_cmd_handler(
                    op_code,
                    cmd_seq,
                    queue_type,
                    index,
                    new_priority,
                );
            }
            _ => {
                self.cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode);
            }
        }
    }
}

impl ActiveComponent for ComQueue {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, TlmPort};
    use fprime_fw::{LogBuffer, Time, TlmBuffer};
    use fprime_os::queue::{BlockingType, Status as QueueStatus};
    use fprime_os::task::{Status as TaskStatus, TASK_DEFAULT};
    use std::sync::Mutex as StdMutex;

    #[derive(Default)]
    struct Recorder {
        /// dataOut records: (bytes, context).
        data_out: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        /// bufferReturnOut records.
        buffer_returns: StdMutex<Vec<Vec<u8>>>,
        /// events: (id, arg bytes).
        events: StdMutex<Vec<(FwEventIdType, Vec<u8>)>>,
        /// telemetry: (id, bytes).
        tlm: StdMutex<Vec<(FwChanIdType, Vec<u8>)>>,
        responses: StdMutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        regs: StdMutex<Vec<FwOpcodeType>>,
    }

    impl ComDataWithContextPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            self.data_out
                .lock()
                .unwrap()
                .push((data.data().to_vec(), *context));
        }
    }

    impl BufferSendPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.buffer_returns
                .lock()
                .unwrap()
                .push(buffer.data().to_vec());
        }
    }

    impl LogPort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.events
                .lock()
                .unwrap()
                .push((id, args.as_slice().to_vec()));
        }
    }

    impl TlmPort for Recorder {
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

    impl CmdResponsePort for Recorder {
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

    impl CmdRegPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }

    /// Default config: all depth 2, priority 0, FIFO, DropNewest.
    fn default_table() -> QueueConfigurationTable {
        let mut table = QueueConfigurationTable::default();
        for entry in table.entries.iter_mut() {
            entry.depth = 2;
        }
        table
    }

    fn build(table: &QueueConfigurationTable) -> (Arc<ComQueue>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let comp = ComQueue::new("comQueue");
        comp.data_out.connect(rec.clone(), 0);
        for port in comp.buffer_return_out.iter() {
            port.connect(rec.clone(), 0);
        }
        comp.evt.log_out.connect(rec.clone(), 0);
        comp.tlm.tlm_out.connect(rec.clone(), 0);
        comp.cmd.cmd_response_out.connect(rec.clone(), 0);
        comp.cmd.cmd_reg_out.connect(rec.clone(), 0);
        comp.configure(table);
        (comp, rec)
    }

    /// A com packet whose first two bytes are the descriptor.
    fn com_packet(descriptor: u16, extra: &[u8]) -> ComBuffer {
        let mut c = ComBuffer::new();
        let mut bytes = descriptor.to_be_bytes().to_vec();
        bytes.extend_from_slice(extra);
        assert!(c.set_buff(&bytes).is_ok());
        c
    }

    fn file_buffer(tag: u8) -> Buffer {
        let mut b = Buffer::allocate(4);
        let descriptor = (Apid::FwPacketFile as u16).to_be_bytes();
        b.data_mut()[0] = descriptor[0];
        b.data_mut()[1] = descriptor[1];
        b.data_mut()[2] = tag;
        b
    }

    fn send_com(comp: &Arc<ComQueue>, port: FwIndexType, descriptor: u16, extra: &[u8]) {
        comp.com_packet_queue_in_handler(port, &com_packet(descriptor, extra), 0);
    }

    fn send_status(comp: &Arc<ComQueue>, condition: Success) {
        comp.com_status_in_handler(0, condition);
    }

    /// Nothing transmits before the first SUCCESS (initial WAITING state —
    /// gotcha: without the ComStub handshake nothing is ever sent).
    #[test]
    fn waiting_until_first_success() {
        let (comp, rec) = build(&default_table());
        send_com(&comp, 0, Apid::FwPacketTelem as u16, b"tlm");
        assert!(rec.data_out.lock().unwrap().is_empty());
        send_status(&comp, Success::Success);
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(
            out[0].0,
            com_packet(Apid::FwPacketTelem as u16, b"tlm").as_slice()
        );
        assert_eq!(out[0].1.apid, Apid::FwPacketTelem);
        assert_eq!(out[0].1.com_queue_index, 0);
    }

    /// One outstanding send: a queued second message waits for the next
    /// SUCCESS.
    #[test]
    fn one_outstanding_send() {
        let (comp, rec) = build(&default_table());
        send_status(&comp, Success::Success); // READY, nothing to send
        send_com(&comp, 0, 1, b"a"); // READY -> sends immediately
        assert_eq!(rec.data_out.lock().unwrap().len(), 1);
        send_com(&comp, 0, 1, b"b"); // WAITING -> queued
        assert_eq!(rec.data_out.lock().unwrap().len(), 1);
        // Return the outstanding buffer + status.
        return_last(&comp, &rec);
        send_status(&comp, Success::Success);
        assert_eq!(rec.data_out.lock().unwrap().len(), 2);
    }

    /// Return the last sent buffer through dataReturnIn.
    fn return_last(comp: &Arc<ComQueue>, rec: &Arc<Recorder>) {
        let (bytes, context) = rec.data_out.lock().unwrap().last().unwrap().clone();
        let mut b = Buffer::allocate(bytes.len().max(1));
        b.data_mut()[..bytes.len()].copy_from_slice(&bytes);
        b.set_size(bytes.len());
        let drin = comp.data_return_in(0);
        drin.target.invoke(drin.port_num, b, &context);
    }

    /// FAILURE keeps WAITING; a later SUCCESS releases the send.
    #[test]
    fn failure_keeps_waiting() {
        let (comp, rec) = build(&default_table());
        send_com(&comp, 0, 1, b"x");
        send_status(&comp, Success::Failure);
        assert!(rec.data_out.lock().unwrap().is_empty());
        send_status(&comp, Success::Success);
        assert_eq!(rec.data_out.lock().unwrap().len(), 1);
    }

    /// A status while READY is a protocol violation and asserts.
    #[test]
    #[should_panic]
    fn status_while_ready_asserts() {
        let (comp, _rec) = build(&default_table());
        send_status(&comp, Success::Success); // WAITING -> READY (queues empty)
        send_status(&comp, Success::Success); // READY -> assert
    }

    /// Priority order: lower priority values are serviced first.
    #[test]
    fn priority_order() {
        let mut table = default_table();
        table.entries[0].priority = 2; // com 0 last
        table.entries[1].priority = 0; // com 1 first
        table.entries[2].priority = 1; // buffer in between
        let (comp, rec) = build(&table);
        send_com(&comp, 0, 1, b"low");
        send_com(&comp, 1, 2, b"high");
        comp.buffer_queue_in_handler(0, file_buffer(9));
        // Drive three sends.
        for _ in 0..3 {
            send_status(&comp, Success::Success);
            return_last(&comp, &rec);
        }
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 3);
        assert_eq!(out[0].1.com_queue_index, 1); // priority 0
        assert_eq!(out[1].1.com_queue_index, 2); // priority 1 (buffer queue)
        assert_eq!(out[2].1.com_queue_index, 0); // priority 2
    }

    /// Round-robin within an equal-priority run: com0 and com1 alternate.
    #[test]
    fn round_robin_within_equal_priority() {
        let mut table = default_table();
        table.entries[0].depth = 4;
        table.entries[1].depth = 4;
        table.entries[2].priority = 2; // keep the buffer queue out of the way
        let (comp, rec) = build(&table);
        for _ in 0..2 {
            send_com(&comp, 0, 1, b"q0");
            send_com(&comp, 1, 1, b"q1");
        }
        for _ in 0..4 {
            send_status(&comp, Success::Success);
            return_last(&comp, &rec);
        }
        let indices: Vec<_> = rec
            .data_out
            .lock()
            .unwrap()
            .iter()
            .map(|(_, c)| c.com_queue_index)
            .collect();
        assert_eq!(indices, vec![0, 1, 0, 1]); // alternating
    }

    /// QueueOverflow fires once per queue until a successful send clears the
    /// throttle; the event args are [queueType u8][index i16 BE].
    #[test]
    fn overflow_event_throttled_until_send() {
        let mut table = default_table();
        table.entries[0].depth = 1;
        let (comp, rec) = build(&table);
        send_com(&comp, 0, 1, b"1"); // fills the queue
        send_com(&comp, 0, 1, b"2"); // overflow -> event
        send_com(&comp, 0, 1, b"3"); // overflow -> throttled
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(EVENTID_QUEUE_OVERFLOW, vec![0x00, 0x00, 0x00])]
        );
        // A successful send clears the throttle.
        send_status(&comp, Success::Success);
        return_last(&comp, &rec);
        send_com(&comp, 0, 1, b"4"); // queued (WAITING)
        send_com(&comp, 0, 1, b"5"); // overflow -> new event
        assert_eq!(rec.events.lock().unwrap().len(), 2);
    }

    /// Buffer queue DROP_NEWEST full: the NEW buffer is returned.
    #[test]
    fn buffer_queue_drop_newest_returns_new_buffer() {
        let mut table = default_table();
        table.entries[2].depth = 1;
        let (comp, rec) = build(&table);
        comp.buffer_queue_in_handler(0, file_buffer(1));
        comp.buffer_queue_in_handler(0, file_buffer(2)); // rejected
        let returns = rec.buffer_returns.lock().unwrap();
        assert_eq!(returns.len(), 1);
        assert_eq!(returns[0][2], 2); // the new one came back
        drop(returns);
        assert_eq!(rec.events.lock().unwrap()[0].0, EVENTID_QUEUE_OVERFLOW);
        assert_eq!(rec.events.lock().unwrap()[0].1, vec![0x01, 0x00, 0x00]);
    }

    /// Buffer queue DROP_OLDEST full: the OLDEST buffer is returned BEFORE
    /// the overwrite (pre-emptive pop — gotcha).
    #[test]
    fn buffer_queue_drop_oldest_preemptively_returns_oldest() {
        let mut table = default_table();
        table.entries[2].depth = 1;
        table.entries[2].overflow_mode = QueueOverflowMode::DropOldest;
        let (comp, rec) = build(&table);
        comp.buffer_queue_in_handler(0, file_buffer(1));
        comp.buffer_queue_in_handler(0, file_buffer(2));
        let returns = rec.buffer_returns.lock().unwrap();
        assert_eq!(returns.len(), 1);
        assert_eq!(returns[0][2], 1); // oldest returned
        drop(returns);
        // The new buffer is the one that transmits.
        send_status(&comp, Success::Success);
        assert_eq!(rec.data_out.lock().unwrap()[0].0[2], 2);
        // Overflow event emitted for the pre-emptive drop.
        assert_eq!(rec.events.lock().unwrap().len(), 1);
    }

    /// dataReturnIn routes buffer-queue returns to bufferReturnOut and drops
    /// com returns (storage recycled).
    #[test]
    fn data_return_routing() {
        let (comp, rec) = build(&default_table());
        // Buffer-queue send/return.
        comp.buffer_queue_in_handler(0, file_buffer(7));
        send_status(&comp, Success::Success);
        assert_eq!(rec.data_out.lock().unwrap().len(), 1);
        return_last(&comp, &rec);
        assert_eq!(rec.buffer_returns.lock().unwrap().len(), 1);
        assert_eq!(rec.buffer_returns.lock().unwrap()[0][2], 7);
        // Com send/return: no bufferReturnOut, storage recycled.
        send_com(&comp, 0, 1, b"c");
        send_status(&comp, Success::Success);
        assert_eq!(rec.data_out.lock().unwrap().len(), 2);
        return_last(&comp, &rec);
        assert_eq!(rec.buffer_returns.lock().unwrap().len(), 1); // unchanged
        assert!(comp.recycle.lock().unwrap().is_some());
    }

    /// The descriptor of the queued data becomes the context APID.
    #[test]
    fn descriptor_sets_context_apid() {
        let (comp, rec) = build(&default_table());
        send_com(&comp, 0, Apid::FwPacketLog as u16, b"evt");
        send_status(&comp, Success::Success);
        assert_eq!(rec.data_out.lock().unwrap()[0].1.apid, Apid::FwPacketLog);
    }

    /// run emits comQueueDepth/buffQueueDepth high-water arrays (raw u32 BE
    /// elements, no length prefix) and clears the marks.
    #[test]
    fn run_emits_high_water_telemetry() {
        let (comp, rec) = build(&default_table());
        send_com(&comp, 0, 1, b"a");
        send_com(&comp, 0, 1, b"b");
        send_com(&comp, 1, 1, b"c");
        comp.buffer_queue_in_handler(0, file_buffer(1));
        comp.run_handler(0, 0);
        {
            let tlm = rec.tlm.lock().unwrap();
            assert_eq!(
                *tlm,
                vec![
                    (
                        CHANID_COM_QUEUE_DEPTH,
                        vec![0, 0, 0, 2, 0, 0, 0, 1] // [2, 1] u32 BE
                    ),
                    (CHANID_BUFF_QUEUE_DEPTH, vec![0, 0, 0, 1]),
                ]
            );
        }
        rec.tlm.lock().unwrap().clear();
        // Marks were cleared.
        comp.run_handler(0, 0);
        let tlm = rec.tlm.lock().unwrap();
        assert_eq!(tlm[0].1, vec![0, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(tlm[1].1, vec![0, 0, 0, 0]);
    }

    // -- Commands ------------------------------------------------------------

    const ID_BASE: u32 = 0x400;

    fn built_with_id_base(table: &QueueConfigurationTable) -> (Arc<ComQueue>, Arc<Recorder>) {
        let (comp, rec) = build(table);
        comp.active.queued.base.set_id_base(ID_BASE);
        (comp, rec)
    }

    fn run_cmd(comp: &Arc<ComQueue>, opcode: FwOpcodeType, seq: u32, arg_bytes: &[u8]) {
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(arg_bytes).is_ok());
        comp.handle_command(ID_BASE + opcode, seq, &mut args);
    }

    /// FLUSH_QUEUE drains a buffer queue, returning ownership.
    #[test]
    fn flush_queue_returns_buffers() {
        let (comp, rec) = built_with_id_base(&default_table());
        comp.buffer_queue_in_handler(0, file_buffer(1));
        comp.buffer_queue_in_handler(0, file_buffer(2));
        // FLUSH_QUEUE(BUFFER_QUEUE, 0): [u8 1][i16 0]
        run_cmd(&comp, OPCODE_FLUSH_QUEUE, 1, &[1, 0, 0]);
        assert_eq!(rec.buffer_returns.lock().unwrap().len(), 2);
        assert_eq!(
            rec.responses.lock().unwrap().last().unwrap().2,
            CmdResponse::Ok
        );
        // Nothing left to send.
        send_status(&comp, Success::Success);
        assert!(rec.data_out.lock().unwrap().is_empty());
    }

    /// FLUSH_QUEUE with an out-of-range index: ValidationError.
    #[test]
    fn flush_queue_validation_error() {
        let (comp, rec) = built_with_id_base(&default_table());
        run_cmd(&comp, OPCODE_FLUSH_QUEUE, 2, &[0, 0, 9]); // com index 9
        assert_eq!(
            rec.responses.lock().unwrap()[0].2,
            CmdResponse::ValidationError
        );
    }

    /// Invalid QueueType enum value: ValidationError; short args:
    /// FormatError; residual bytes: FormatError.
    #[test]
    fn flush_queue_arg_errors() {
        let (comp, rec) = built_with_id_base(&default_table());
        run_cmd(&comp, OPCODE_FLUSH_QUEUE, 1, &[7, 0, 0]); // bad enum
        run_cmd(&comp, OPCODE_FLUSH_QUEUE, 2, &[0]); // short
        run_cmd(&comp, OPCODE_FLUSH_QUEUE, 3, &[0, 0, 0, 0xEE]); // residual
        let responses = rec.responses.lock().unwrap();
        assert_eq!(responses[0].2, CmdResponse::ValidationError);
        assert_eq!(responses[1].2, CmdResponse::FormatError);
        assert_eq!(responses[2].2, CmdResponse::FormatError);
    }

    /// FLUSH_ALL_QUEUES drains everything.
    #[test]
    fn flush_all_queues() {
        let (comp, rec) = built_with_id_base(&default_table());
        send_com(&comp, 0, 1, b"a");
        send_com(&comp, 1, 1, b"b");
        comp.buffer_queue_in_handler(0, file_buffer(3));
        run_cmd(&comp, OPCODE_FLUSH_ALL_QUEUES, 1, &[]);
        assert_eq!(rec.buffer_returns.lock().unwrap().len(), 1);
        assert_eq!(rec.responses.lock().unwrap()[0].2, CmdResponse::Ok);
        send_status(&comp, Success::Success);
        assert!(rec.data_out.lock().unwrap().is_empty());
    }

    /// SET_QUEUE_PRIORITY re-orders servicing and emits the event.
    #[test]
    fn set_queue_priority_reorders() {
        let (comp, rec) = built_with_id_base(&default_table());
        // Raise com0's priority value so com1 goes first... initially all 0
        // (FIFO order com0 first). Set com0 to priority 2:
        // SET_QUEUE_PRIORITY(COM_QUEUE, 0, 2) = [u8 0][i16 0][i16 2]
        run_cmd(&comp, OPCODE_SET_QUEUE_PRIORITY, 1, &[0, 0, 0, 0, 2]);
        assert_eq!(rec.responses.lock().unwrap()[0].2, CmdResponse::Ok);
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(
                ID_BASE + EVENTID_QUEUE_PRIORITY_CHANGED,
                vec![0x00, 0x00, 0x00, 0x00, 0x02] // [type u8][idx i16][prio i16]
            )]
        );
        send_com(&comp, 0, 1, b"q0");
        send_com(&comp, 1, 1, b"q1");
        send_status(&comp, Success::Success);
        // com1 (still priority 0) is serviced before com0 (now priority 2).
        assert_eq!(rec.data_out.lock().unwrap()[0].1.com_queue_index, 1);
    }

    /// SET_QUEUE_PRIORITY validation errors.
    #[test]
    fn set_queue_priority_validation_errors() {
        let (comp, rec) = built_with_id_base(&default_table());
        run_cmd(&comp, OPCODE_SET_QUEUE_PRIORITY, 1, &[0, 0, 9, 0, 0]); // bad index
        run_cmd(&comp, OPCODE_SET_QUEUE_PRIORITY, 2, &[0, 0, 0, 0, 9]); // bad priority
        let responses = rec.responses.lock().unwrap();
        assert_eq!(responses[0].2, CmdResponse::ValidationError);
        assert_eq!(responses[1].2, CmdResponse::ValidationError);
    }

    /// Unknown opcode: InvalidOpcode.
    #[test]
    fn unknown_opcode() {
        let (comp, rec) = built_with_id_base(&default_table());
        run_cmd(&comp, 0xEE, 1, &[]);
        assert_eq!(
            rec.responses.lock().unwrap()[0].2,
            CmdResponse::InvalidOpcode
        );
    }

    /// reg_commands registers all three opcodes offset by id_base.
    #[test]
    fn reg_commands_registers_opcodes() {
        let (comp, rec) = built_with_id_base(&default_table());
        comp.reg_commands();
        assert_eq!(
            *rec.regs.lock().unwrap(),
            vec![ID_BASE, ID_BASE + 1, ID_BASE + 2]
        );
    }

    // -- Envelope / async-machinery tests ------------------------------------

    /// The comPacketQueueIn adapter serializes the byte-exact envelope:
    /// [msg_type i32][port i16][u16 len][com bytes][context u32].
    #[test]
    fn com_packet_envelope_bytes_are_byte_exact() {
        let (comp, _rec) = build(&default_table());
        comp.active.queued.create_queue(4, MSG_SIZE);
        let port = comp.com_packet_queue_in(1);
        let mut data = com_packet(0x0001, &[0xAB]);
        port.target.invoke(port.port_num, &mut data, 0x01020304);
        let mut dest = vec![0u8; MSG_SIZE as usize];
        let mut size: FwSizeType = 0;
        let mut priority: FwQueuePriorityType = 0;
        let status = comp.active.queued.queue().receive(
            &mut dest,
            BlockingType::NonBlocking,
            &mut size,
            &mut priority,
        );
        assert_eq!(status, QueueStatus::OpOk);
        assert_eq!(
            &dest[..size as usize],
            &[
                0x00, 0x00, 0x00, 0x02, // MSG_TYPE_COM_PACKET_QUEUE_IN
                0x00, 0x01, // port 1
                0x00, 0x03, // com buffer length
                0x00, 0x01, 0xAB, // descriptor + byte
                0x01, 0x02, 0x03, 0x04, // context
            ]
        );
        assert_eq!(priority, PORT_PRIORITY);
    }

    /// bufferQueueIn hook policy: a full message queue returns the buffer
    /// via bufferReturnOut (C++ overflowHook).
    #[test]
    fn buffer_queue_in_hook_returns_on_full_message_queue() {
        let (comp, rec) = build(&default_table());
        comp.active.queued.create_queue(1, MSG_SIZE);
        let port = comp.buffer_queue_in(0);
        port.target.invoke(port.port_num, file_buffer(1)); // fills msg queue
        port.target.invoke(port.port_num, file_buffer(2)); // hook fires
        let returns = rec.buffer_returns.lock().unwrap();
        assert_eq!(returns.len(), 1);
        assert_eq!(returns[0][2], 2);
    }

    /// Full lifecycle: start the task, drive the flow control through the
    /// message queue, exit, join.
    #[test]
    fn active_lifecycle_end_to_end() {
        let (comp, rec) = build(&default_table());
        comp.active.queued.create_queue(16, MSG_SIZE);
        comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);

        let com_in = comp.com_packet_queue_in(0);
        let mut pkt = com_packet(Apid::FwPacketTelem as u16, b"live");
        com_in.target.invoke(com_in.port_num, &mut pkt, 0);
        let status_in = comp.com_status_in(0);
        let mut cond = Success::Success;
        status_in.target.invoke(status_in.port_num, &mut cond);

        comp.active.exit();
        assert_eq!(comp.active.join(), TaskStatus::OpOk);

        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].1.apid, Apid::FwPacketTelem);
        assert_eq!(out[0].1.com_queue_index, 0);
    }

    /// configure rejects zero depths (C++ assert).
    #[test]
    #[should_panic]
    fn configure_zero_depth_asserts() {
        let comp = ComQueue::new("bad");
        comp.configure(&QueueConfigurationTable::default()); // depths 0
    }

    /// configure rejects out-of-range priorities (C++ assert).
    #[test]
    #[should_panic]
    fn configure_bad_priority_asserts() {
        let comp = ComQueue::new("bad");
        let mut table = default_table();
        table.entries[0].priority = TOTAL_PORT_COUNT as FwIndexType;
        comp.configure(&table);
    }
}
