//! `Svc::DpManager` — the data-product buffer manager (ACTIVE).
//!
//! Port of `Svc/DpManager/DpManager.{fpp,hpp,cpp}`; analysis:
//! `docs/cpp-analysis/data-products.md` (section "Svc::DpManager
//! component").
//!
//! DpManager hands data-product buffers to client components and forwards
//! the filled containers to `Svc::DpWriter`:
//!
//! * `productGetIn` (**sync**, 5 ports) allocates on the CALLER's thread and
//!   returns the buffer inline;
//! * `productRequestIn` (async, 5 ports) allocates on the component thread
//!   and ALWAYS answers on `productResponseOut` at the same port index —
//!   even on failure, with an invalid buffer and `Fw::Success::FAILURE`;
//! * `productSendIn` (async, 5 ports) is a pure pass-through to
//!   `productSendOut` at the same index; the header is never inspected.
//!
//! Because the sync path runs on foreign threads, the two allocation
//! counters are atomics (C++ `std::atomic<U32>`) while `NumDataProducts` /
//! `NumBytes` are plain fields behind the component-thread state mutex,
//! exactly as in C++.

use fprime_comp::escrow::BufferEscrow;
use fprime_comp::{
    ActiveBase, ActiveComponent, BufferGetPort, BufferSendPort, CmdGlue, CmdPort,
    ComponentDispatch, EventGlue, EventThrottle, MsgDispatchStatus, OutputPort, PortRef,
    QueueFullPolicy, SchedPort, TlmGlue, msg,
};
use fprime_config::{
    FW_CMD_ARG_BUFFER_MAX_SIZE, FwChanIdType, FwDpIdType, FwEnumStoreType, FwEventIdType, FwIdType,
    FwIndexType, FwOpcodeType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::dp::{DpGetPort, DpRequestPort, DpResponsePort, DpSendPort};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, Endianness, LinearBuffer, LogSeverity, SerBuf, SerBufAny,
    Success, fw_assert,
};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Dictionary constants
// ---------------------------------------------------------------------------

/// `DpManagerNumPorts` (`default/config/AcConstants.fpp`): the array size of
/// `productGetIn`, `productRequestIn`, `productResponseOut`, `bufferGetOut`,
/// `productSendIn` and `productSendOut`.
pub const DP_MANAGER_NUM_PORTS: usize = 5;

/// Queue message types, numbered from 1 in FPP declaration order
/// (`schedIn`, `productRequestIn`, `productSendIn`, `cmdIn`).
const MSG_TYPE_SCHED_IN: FwEnumStoreType = 1;
const MSG_TYPE_PRODUCT_REQUEST_IN: FwEnumStoreType = 2;
const MSG_TYPE_PRODUCT_SEND_IN: FwEnumStoreType = 3;
const MSG_TYPE_CMD_IN: FwEnumStoreType = 4;

/// Queue message size = the largest async invocation, the command envelope:
/// 6 (envelope) + 4 (opcode) + 4 (cmdSeq) + 2 + 506 (nested `CmdArgBuffer`)
/// = 522. `productRequestIn` needs 18 and `productSendIn` 18.
pub const QUEUE_MSG_SIZE: FwSizeType =
    (msg::ENVELOPE_HEADER_SIZE + 4 + 4 + 2 + FW_CMD_ARG_BUFFER_MAX_SIZE) as FwSizeType;

/// FPP declares no `priority` qualifier on any async input.
const PORT_PRIORITY: FwQueuePriorityType = 1;

type MsgBuffer = LinearBuffer<{ QUEUE_MSG_SIZE as usize }>;

// ---------------------------------------------------------------------------
// Component state
// ---------------------------------------------------------------------------

/// State touched only by the component thread (C++ plain members).
#[derive(Default)]
struct DpManagerState {
    /// `numDataProducts`.
    num_data_products: u32,
    /// `numBytes`.
    num_bytes: u64,
    // `update on change` caches for the four telemetry channels.
    last_successful_allocations: Option<u32>,
    last_failed_allocations: Option<u32>,
    last_data_products: Option<u32>,
    last_bytes: Option<u64>,
}

/// `Svc::DpManager` — active data-product manager.
pub struct DpManager {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Command registration/response glue.
    pub cmd: CmdGlue,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// `productResponseOut`: \[5\] `Fw.DpResponse` out.
    pub product_response_out: [OutputPort<dyn DpResponsePort>; DP_MANAGER_NUM_PORTS],
    /// `bufferGetOut`: \[5\] `Fw.BufferGet` out (a `Svc::BufferManager`).
    pub buffer_get_out: [OutputPort<dyn BufferGetPort>; DP_MANAGER_NUM_PORTS],
    /// `productSendOut`: \[5\] `Fw.BufferSend` out (a `Svc::DpWriter`).
    pub product_send_out: [OutputPort<dyn BufferSendPort>; DP_MANAGER_NUM_PORTS],
    /// `numSuccessfulAllocations` — touched by the sync path on foreign
    /// threads, hence atomic (C++ `std::atomic<U32>`).
    num_successful_allocations: AtomicU32,
    /// `numFailedAllocations` — likewise atomic.
    num_failed_allocations: AtomicU32,
    /// `throttle 10` on `BufferAllocationFailed`.
    buffer_allocation_failed_throttle: EventThrottle,
    /// Escrow for the async `productSendIn` buffer.
    escrow: BufferEscrow,
    state: Mutex<DpManagerState>,
}

impl DpManager {
    // -- Commands (FPP-relative opcodes) -----------------------------------

    /// `CLEAR_EVENT_THROTTLE` — opcode 0x00.
    pub const OPCODE_CLEAR_EVENT_THROTTLE: FwOpcodeType = 0x00;

    // -- Events (FPP-relative ids) -----------------------------------------

    /// `BufferAllocationFailed(id: FwDpIdType)` — WARNING_HI, id 0,
    /// `throttle 10`.
    pub const EVENTID_BUFFER_ALLOCATION_FAILED: FwEventIdType = 0;
    /// The `throttle 10` limit of `BufferAllocationFailed`.
    const THROTTLE_10: u32 = 10;

    // -- Telemetry (FPP-relative ids) --------------------------------------

    /// `NumSuccessfulAllocations: U32 update on change` — id 0.
    pub const CHANID_NUM_SUCCESSFUL_ALLOCATIONS: FwChanIdType = 0;
    /// `NumFailedAllocations: U32 update on change` — id 1.
    pub const CHANID_NUM_FAILED_ALLOCATIONS: FwChanIdType = 1;
    /// `NumDataProducts: U32 update on change` — id 2.
    pub const CHANID_NUM_DATA_PRODUCTS: FwChanIdType = 2;
    /// `NumBytes: U64 update on change` — id 3.
    pub const CHANID_NUM_BYTES: FwChanIdType = 3;

    /// Construct (topology phase 1). Follow with `set_id_base`, wiring,
    /// [`Self::init`], [`Self::reg_commands`] and `active.start`.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            product_response_out: std::array::from_fn(|_| OutputPort::new()),
            buffer_get_out: std::array::from_fn(|_| OutputPort::new()),
            product_send_out: std::array::from_fn(|_| OutputPort::new()),
            num_successful_allocations: AtomicU32::new(0),
            num_failed_allocations: AtomicU32::new(0),
            buffer_allocation_failed_throttle: EventThrottle::new(Self::THROTTLE_10),
            escrow: BufferEscrow::new(),
            state: Mutex::new(DpManagerState::default()),
        })
    }

    /// Create the message queue (topology "configure" step).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active.queued.create_queue(queue_depth, QUEUE_MSG_SIZE);
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd
            .reg_commands(self.id_base(), &[Self::OPCODE_CLEAR_EVENT_THROTTLE]);
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    // -- Input-port factories ---------------------------------------------

    /// `productGetIn` — **SYNC** `Fw.DpGet` input \[5\]: the handler runs on
    /// the caller's thread and allocates inline.
    pub fn product_get_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn DpGetPort> {
        check_port_num(port_num);
        PortRef::new(self.clone(), port_num)
    }

    /// `productRequestIn` — ASYNC `Fw.DpRequest` input \[5\].
    pub fn product_request_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn DpRequestPort> {
        check_port_num(port_num);
        PortRef::new(
            Arc::new(ProductRequestInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `productSendIn` — ASYNC `Fw.DpSend` input \[5\]; the owned buffer
    /// rides through the escrow in the 8 message bytes C++ uses for the
    /// `Fw::Buffer` pointer.
    pub fn product_send_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn DpSendPort> {
        check_port_num(port_num);
        PortRef::new(
            Arc::new(ProductSendInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `schedIn` — ASYNC `Svc.Sched` input.
    pub fn sched_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(Arc::new(SchedInAdapter { comp: self.clone() }), port_num)
    }

    /// `cmdIn` — ASYNC `Fw.Cmd` input.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(Arc::new(CmdInAdapter { comp: self.clone() }), port_num)
    }

    // -- Handlers ----------------------------------------------------------

    /// C++ `getBuffer`: allocate on `bufferGetOut[portNum]`, count the
    /// outcome and log on failure. Runs on the caller's thread for the sync
    /// path and on the component thread for the async one.
    fn get_buffer(
        &self,
        port_num: FwIndexType,
        id: FwDpIdType,
        size: FwSizeType,
    ) -> (Success, Buffer) {
        check_port_num(port_num);
        let port = self.buffer_get_out[port_num as usize].get();
        let buffer = port.target.invoke(port.port_num, size);
        if buffer.is_valid() {
            self.num_successful_allocations
                .fetch_add(1, Ordering::Relaxed);
            (Success::Success, buffer)
        } else {
            self.num_failed_allocations.fetch_add(1, Ordering::Relaxed);
            self.log_buffer_allocation_failed(id);
            (Success::Failure, buffer)
        }
    }

    /// `productGetIn_handler` — SYNC: returns the allocation result to the
    /// caller with no queueing at all.
    fn product_get_handler(
        &self,
        port_num: FwIndexType,
        id: FwDpIdType,
        data_size: FwSizeType,
        buffer: &mut Buffer,
    ) -> Success {
        let (status, allocated) = self.get_buffer(port_num, id, data_size);
        *buffer = allocated;
        status
    }

    /// `productRequestIn_handler` — component thread. The response goes out
    /// on the SAME port index unconditionally, even when allocation failed
    /// (invalid buffer + FAILURE).
    fn product_request_handler(
        &self,
        port_num: FwIndexType,
        id: FwDpIdType,
        data_size: FwSizeType,
    ) {
        let (status, buffer) = self.get_buffer(port_num, id, data_size);
        let port = self.product_response_out[port_num as usize].get();
        port.target.invoke(port.port_num, id, buffer, status);
    }

    /// `productSendIn_handler` — component thread. Pure pass-through; the
    /// container id is ignored and `NumBytes` counts the WHOLE `Fw::Buffer`
    /// size (the BufferManager bin), not the packet size (C++ parity).
    fn product_send_handler(&self, port_num: FwIndexType, _id: FwDpIdType, buffer: Buffer) {
        check_port_num(port_num);
        {
            let mut state = self.state.lock().unwrap();
            state.num_data_products = state.num_data_products.wrapping_add(1);
            state.num_bytes = state.num_bytes.wrapping_add(buffer.size() as u64);
        }
        let port = self.product_send_out[port_num as usize].get();
        port.target.invoke(port.port_num, buffer);
    }

    /// `schedIn_handler` — writes the four `update on change` channels in
    /// the C++ order.
    fn sched_handler(&self, _port_num: FwIndexType, _context: u32) {
        let successful = self.num_successful_allocations.load(Ordering::Relaxed);
        let failed = self.num_failed_allocations.load(Ordering::Relaxed);
        let (successful, failed, products, bytes) = {
            let state = &mut *self.state.lock().unwrap();
            (
                changed_u32(&mut state.last_successful_allocations, successful),
                changed_u32(&mut state.last_failed_allocations, failed),
                changed_u32(&mut state.last_data_products, state.num_data_products),
                changed_u64(&mut state.last_bytes, state.num_bytes),
            )
        };
        let id_base = self.id_base();
        if let Some(v) = successful {
            self.tlm.tlm_write(
                id_base,
                Self::CHANID_NUM_SUCCESSFUL_ALLOCATIONS,
                &v,
                self.evt.time_get(),
            );
        }
        if let Some(v) = failed {
            self.tlm.tlm_write(
                id_base,
                Self::CHANID_NUM_FAILED_ALLOCATIONS,
                &v,
                self.evt.time_get(),
            );
        }
        if let Some(v) = products {
            self.tlm.tlm_write(
                id_base,
                Self::CHANID_NUM_DATA_PRODUCTS,
                &v,
                self.evt.time_get(),
            );
        }
        if let Some(v) = bytes {
            self.tlm
                .tlm_write(id_base, Self::CHANID_NUM_BYTES, &v, self.evt.time_get());
        }
    }

    /// `CLEAR_EVENT_THROTTLE` handler.
    fn clear_event_throttle_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        self.buffer_allocation_failed_throttle.clear();
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// Command dispatch on the local opcode (exactly-once response
    /// discipline: FormatError on residual bytes, InvalidOpcode otherwise).
    fn handle_command(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_CLEAR_EVENT_THROTTLE => {
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.clear_event_throttle_cmd_handler(op_code, cmd_seq);
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }

    // -- Events ------------------------------------------------------------

    fn log_buffer_allocation_failed(&self, id: FwDpIdType) {
        if !self.buffer_allocation_failed_throttle.ok_to_emit() {
            return;
        }
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_BUFFER_ALLOCATION_FAILED,
            LogSeverity::WarningHi,
            &format!("Buffer allocation failed for container id {id}"),
            |buf| buf.serialize_u32_be(id),
        );
    }
}

/// C++ generated port-array bounds check.
fn check_port_num(port_num: FwIndexType) {
    fw_assert!(
        port_num >= 0 && (port_num as usize) < DP_MANAGER_NUM_PORTS,
        port_num
    );
}

/// `update on change` helper for the `u32` channels.
fn changed_u32(last: &mut Option<u32>, value: u32) -> Option<u32> {
    if *last == Some(value) {
        None
    } else {
        *last = Some(value);
        Some(value)
    }
}

/// `update on change` helper for `NumBytes` (U64).
fn changed_u64(last: &mut Option<u64>, value: u64) -> Option<u64> {
    if *last == Some(value) {
        None
    } else {
        *last = Some(value);
        Some(value)
    }
}

// -- Sync input port, implemented directly on the component -----------------

impl DpGetPort for DpManager {
    fn invoke(
        &self,
        port_num: FwIndexType,
        id: FwDpIdType,
        data_size: FwSizeType,
        buffer: &mut Buffer,
    ) -> Success {
        self.product_get_handler(port_num, id, data_size, buffer)
    }
}

// -- Async input adapters ---------------------------------------------------

/// `productRequestIn` adapter: `[msg_type][port_num][id u32][dataSize u64]`.
struct ProductRequestInAdapter {
    comp: Arc<DpManager>,
}

impl DpRequestPort for ProductRequestInAdapter {
    fn invoke(&self, port_num: FwIndexType, id: FwDpIdType, data_size: FwSizeType) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_PRODUCT_REQUEST_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(id);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u64_be(data_size);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// `productSendIn` adapter: `[msg_type][port_num][id u32][escrow token u64]`.
struct ProductSendInAdapter {
    comp: Arc<DpManager>,
}

impl DpSendPort for ProductSendInAdapter {
    fn invoke(&self, port_num: FwIndexType, id: FwDpIdType, buffer: Buffer) {
        let token = self.comp.escrow.deposit(buffer);
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_PRODUCT_SEND_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(id);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u64_be(token);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// `schedIn` adapter.
struct SchedInAdapter {
    comp: Arc<DpManager>,
}

impl SchedPort for SchedInAdapter {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_SCHED_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(context);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, PORT_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// `cmdIn` adapter.
struct CmdInAdapter {
    comp: Arc<DpManager>,
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

// -- Dispatch ---------------------------------------------------------------

impl ComponentDispatch for DpManager {
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
            MSG_TYPE_SCHED_IN => {
                let mut context = 0u32;
                if !buf.deserialize_u32_be(&mut context).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.sched_handler(port_num, context);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_PRODUCT_REQUEST_IN => {
                let mut id: FwDpIdType = 0;
                let mut data_size: FwSizeType = 0;
                if !buf.deserialize_u32_be(&mut id).is_ok()
                    || !buf.deserialize_u64_be(&mut data_size).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                self.product_request_handler(port_num, id, data_size);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_PRODUCT_SEND_IN => {
                let mut id: FwDpIdType = 0;
                let mut token = 0u64;
                if !buf.deserialize_u32_be(&mut id).is_ok()
                    || !buf.deserialize_u64_be(&mut token).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                let buffer = self.escrow.claim(token);
                self.product_send_handler(port_num, id, buffer);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_CMD_IN => {
                let mut op_code: FwOpcodeType = 0;
                let mut cmd_seq = 0u32;
                let mut args = CmdArgBuffer::new();
                if !buf.deserialize_u32_be(&mut op_code).is_ok()
                    || !buf.deserialize_u32_be(&mut cmd_seq).is_ok()
                    || !buf.deserialize_buffer(&mut args, Endianness::Big).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                self.handle_command(op_code, cmd_seq, &mut args);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for DpManager {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort, TimePort, TlmPort};
    use fprime_fw::{LogBuffer, TextLogString, Time, TimeBase, TlmBuffer};
    use fprime_os::queue::{BlockingType, Status as QueueStatus};

    const ID_BASE: FwIdType = 0x2000;

    /// Ground stub: records commands, events and telemetry.
    #[derive(Default)]
    struct GroundStub {
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
    }

    impl CmdRegPort for GroundStub {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }

    impl CmdResponsePort for GroundStub {
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

    impl LogPort for GroundStub {
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

    impl LogTextPort for GroundStub {
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

    impl TlmPort for GroundStub {
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

    /// `Svc::BufferManager` stub: hands out `size` bytes, or an invalid
    /// buffer once `fail` is set. Records every requested size.
    #[derive(Default)]
    struct BufferMgrStub {
        requests: Mutex<Vec<(FwIndexType, FwSizeType)>>,
        fail: Mutex<bool>,
    }

    impl BufferGetPort for BufferMgrStub {
        fn invoke(&self, port_num: FwIndexType, size: FwSizeType) -> Buffer {
            self.requests.lock().unwrap().push((port_num, size));
            if *self.fail.lock().unwrap() {
                Buffer::empty()
            } else {
                Buffer::allocate(size as usize)
            }
        }
    }

    /// Records `productResponseOut` invocations.
    #[derive(Default)]
    struct ResponseStub {
        responses: Mutex<Vec<(FwIndexType, FwDpIdType, usize, Success)>>,
    }

    impl DpResponsePort for ResponseStub {
        fn invoke(&self, port_num: FwIndexType, id: FwDpIdType, buffer: Buffer, status: Success) {
            self.responses
                .lock()
                .unwrap()
                .push((port_num, id, buffer.size(), status));
        }
    }

    /// Records `productSendOut` invocations.
    #[derive(Default)]
    struct SendStub {
        sent: Mutex<Vec<(FwIndexType, usize)>>,
    }

    impl BufferSendPort for SendStub {
        fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
            self.sent.lock().unwrap().push((port_num, buffer.size()));
        }
    }

    struct Harness {
        comp: Arc<DpManager>,
        ground: Arc<GroundStub>,
        mgr: Arc<BufferMgrStub>,
        responses: Arc<ResponseStub>,
        sends: Arc<SendStub>,
    }

    impl Harness {
        fn new() -> Self {
            let comp = DpManager::new("dpManager");
            let ground = Arc::new(GroundStub::default());
            let mgr = Arc::new(BufferMgrStub::default());
            let responses = Arc::new(ResponseStub::default());
            let sends = Arc::new(SendStub::default());
            comp.active.queued.base.set_id_base(ID_BASE);
            comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
            comp.cmd.cmd_response_out.connect(ground.clone(), 0);
            comp.evt.log_out.connect(ground.clone(), 0);
            comp.evt.text_log_out.connect(ground.clone(), 0);
            comp.evt.time_out.connect(Arc::new(TimeStub), 0);
            comp.tlm.tlm_out.connect(ground.clone(), 0);
            for i in 0..DP_MANAGER_NUM_PORTS {
                comp.buffer_get_out[i].connect(mgr.clone(), i as FwIndexType);
                comp.product_response_out[i].connect(responses.clone(), i as FwIndexType);
                comp.product_send_out[i].connect(sends.clone(), i as FwIndexType);
            }
            comp.init(16);
            Self {
                comp,
                ground,
                mgr,
                responses,
                sends,
            }
        }

        fn drain(&self) {
            let _ = self
                .comp
                .active
                .queued
                .dispatch_available_messages(self.comp.as_ref());
        }

        fn send_cmd(&self, op_code: FwOpcodeType, cmd_seq: u32, arg_bytes: &[u8]) {
            let mut args = CmdArgBuffer::new();
            assert!(args.set_buff(arg_bytes).is_ok());
            let port = self.comp.cmd_in(0);
            port.target
                .invoke(port.port_num, op_code, cmd_seq, &mut args);
        }

        fn tap_queue(&self) -> Vec<u8> {
            let mut dest = [0u8; QUEUE_MSG_SIZE as usize];
            let mut size: FwSizeType = 0;
            let mut priority: FwQueuePriorityType = 0;
            let status = self.comp.active.queued.queue().receive(
                &mut dest,
                BlockingType::NonBlocking,
                &mut size,
                &mut priority,
            );
            assert_eq!(status, QueueStatus::OpOk);
            dest[..size as usize].to_vec()
        }
    }

    #[test]
    fn queue_message_size_fits_a_full_command_argument_buffer() {
        assert_eq!(QUEUE_MSG_SIZE, 522);
    }

    #[test]
    fn dictionary_ids_match_the_fpp_model() {
        assert_eq!(DpManager::OPCODE_CLEAR_EVENT_THROTTLE, 0);
        assert_eq!(DpManager::EVENTID_BUFFER_ALLOCATION_FAILED, 0);
        assert_eq!(DpManager::CHANID_NUM_SUCCESSFUL_ALLOCATIONS, 0);
        assert_eq!(DpManager::CHANID_NUM_FAILED_ALLOCATIONS, 1);
        assert_eq!(DpManager::CHANID_NUM_DATA_PRODUCTS, 2);
        assert_eq!(DpManager::CHANID_NUM_BYTES, 3);
        let h = Harness::new();
        h.comp.reg_commands();
        assert_eq!(*h.ground.regs.lock().unwrap(), vec![ID_BASE]);
    }

    #[test]
    fn product_get_allocates_on_the_callers_thread() {
        let h = Harness::new();
        let port = h.comp.product_get_in(2);
        let mut buffer = Buffer::empty();
        let status = port.target.invoke(port.port_num, 0xAB, 100, &mut buffer);
        assert_eq!(status, Success::Success);
        assert_eq!(buffer.size(), 100);
        // No message was queued: the sync path never touches the queue.
        assert_eq!(h.comp.active.queued.queue().get_messages_available(), 0);
        assert_eq!(*h.mgr.requests.lock().unwrap(), vec![(2, 100)]);
        assert_eq!(h.comp.num_successful_allocations.load(Ordering::Relaxed), 1);
        assert!(h.ground.events.lock().unwrap().is_empty());
    }

    #[test]
    fn product_get_failure_counts_and_logs_the_event() {
        let h = Harness::new();
        *h.mgr.fail.lock().unwrap() = true;
        let port = h.comp.product_get_in(0);
        let mut buffer = Buffer::allocate(4);
        let status = port
            .target
            .invoke(port.port_num, 0x0102_0304, 64, &mut buffer);
        assert_eq!(status, Success::Failure);
        assert!(!buffer.is_valid());
        assert_eq!(h.comp.num_failed_allocations.load(Ordering::Relaxed), 1);
        let events = h.ground.events.lock().unwrap();
        assert_eq!(
            *events,
            vec![(
                ID_BASE + DpManager::EVENTID_BUFFER_ALLOCATION_FAILED,
                LogSeverity::WarningHi,
                vec![0x01, 0x02, 0x03, 0x04]
            )]
        );
    }

    #[test]
    fn buffer_allocation_failed_is_throttled_at_ten_and_cleared_by_command() {
        let h = Harness::new();
        *h.mgr.fail.lock().unwrap() = true;
        let port = h.comp.product_get_in(0);
        for _ in 0..12 {
            let mut buffer = Buffer::empty();
            let _ = port.target.invoke(port.port_num, 1, 64, &mut buffer);
        }
        assert_eq!(h.ground.events.lock().unwrap().len(), 10);
        assert_eq!(h.comp.num_failed_allocations.load(Ordering::Relaxed), 12);

        h.send_cmd(ID_BASE + DpManager::OPCODE_CLEAR_EVENT_THROTTLE, 7, &[]);
        h.drain();
        assert_eq!(
            *h.ground.responses.lock().unwrap(),
            vec![(
                ID_BASE + DpManager::OPCODE_CLEAR_EVENT_THROTTLE,
                7,
                CmdResponse::Ok
            )]
        );
        let mut buffer = Buffer::empty();
        let _ = port.target.invoke(port.port_num, 1, 64, &mut buffer);
        assert_eq!(h.ground.events.lock().unwrap().len(), 11);
    }

    #[test]
    fn product_request_envelope_is_byte_exact() {
        let h = Harness::new();
        let port = h.comp.product_request_in(3);
        port.target.invoke(port.port_num, 0x0102_0304, 0x0A);
        assert_eq!(
            h.tap_queue(),
            vec![
                0x00, 0x00, 0x00, 0x02, // msg_type = PRODUCT_REQUEST_IN
                0x00, 0x03, // port_num = 3
                0x01, 0x02, 0x03, 0x04, // id
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, // dataSize (u64)
            ]
        );
    }

    #[test]
    fn product_request_answers_on_the_same_port_index() {
        let h = Harness::new();
        let port = h.comp.product_request_in(4);
        port.target.invoke(port.port_num, 42, 128);
        h.drain();
        assert_eq!(
            *h.responses.responses.lock().unwrap(),
            vec![(4, 42, 128, Success::Success)]
        );
    }

    #[test]
    fn product_request_answers_even_when_allocation_fails() {
        let h = Harness::new();
        *h.mgr.fail.lock().unwrap() = true;
        let port = h.comp.product_request_in(1);
        port.target.invoke(port.port_num, 9, 32);
        h.drain();
        // An invalid (zero-size) buffer with FAILURE still goes out.
        assert_eq!(
            *h.responses.responses.lock().unwrap(),
            vec![(1, 9, 0, Success::Failure)]
        );
        assert_eq!(h.ground.events.lock().unwrap().len(), 1);
    }

    #[test]
    fn product_send_forwards_the_buffer_on_the_same_index() {
        let h = Harness::new();
        let port = h.comp.product_send_in(2);
        port.target.invoke(port.port_num, 77, Buffer::allocate(200));
        h.drain();
        assert_eq!(*h.sends.sent.lock().unwrap(), vec![(2, 200)]);
        let state = h.comp.state.lock().unwrap();
        assert_eq!(state.num_data_products, 1);
        // NumBytes counts the WHOLE buffer, not the packet size (C++ parity).
        assert_eq!(state.num_bytes, 200);
    }

    #[test]
    fn sched_writes_the_four_channels_only_when_they_change() {
        let h = Harness::new();
        let sched = h.comp.sched_in(0);
        sched.target.invoke(sched.port_num, 0);
        h.drain();
        // First run: all four channels are written with their zero values.
        assert_eq!(
            *h.ground.tlm.lock().unwrap(),
            vec![
                (ID_BASE, vec![0, 0, 0, 0]),
                (ID_BASE + 1, vec![0, 0, 0, 0]),
                (ID_BASE + 2, vec![0, 0, 0, 0]),
                (ID_BASE + 3, vec![0, 0, 0, 0, 0, 0, 0, 0]),
            ]
        );
        h.ground.tlm.lock().unwrap().clear();

        // Nothing changed: nothing is written.
        sched.target.invoke(sched.port_num, 0);
        h.drain();
        assert!(h.ground.tlm.lock().unwrap().is_empty());

        // One successful allocation and one product moves two channels.
        let get = h.comp.product_get_in(0);
        let mut buffer = Buffer::empty();
        let _ = get.target.invoke(get.port_num, 1, 8, &mut buffer);
        let send = h.comp.product_send_in(0);
        send.target.invoke(send.port_num, 1, Buffer::allocate(70));
        sched.target.invoke(sched.port_num, 0);
        h.drain();
        assert_eq!(
            *h.ground.tlm.lock().unwrap(),
            vec![
                (ID_BASE, vec![0, 0, 0, 1]),
                (ID_BASE + 2, vec![0, 0, 0, 1]),
                (ID_BASE + 3, vec![0, 0, 0, 0, 0, 0, 0, 70]),
            ]
        );
    }

    #[test]
    fn command_envelope_is_byte_exact_and_unknown_opcodes_are_rejected() {
        let h = Harness::new();
        h.send_cmd(ID_BASE + 0x55, 3, &[0xAA]);
        assert_eq!(
            h.tap_queue(),
            vec![
                0x00, 0x00, 0x00, 0x04, // msg_type = CMD_IN
                0x00, 0x00, // port_num
                0x00, 0x00, 0x20, 0x55, // opcode
                0x00, 0x00, 0x00, 0x03, // cmdSeq
                0x00, 0x01, // arg buffer length
                0xAA,
            ]
        );
        h.send_cmd(ID_BASE + 0x55, 3, &[]);
        h.drain();
        assert_eq!(
            *h.ground.responses.lock().unwrap(),
            vec![(ID_BASE + 0x55, 3, CmdResponse::InvalidOpcode)]
        );
    }

    #[test]
    fn residual_command_bytes_are_a_format_error() {
        let h = Harness::new();
        h.send_cmd(ID_BASE + DpManager::OPCODE_CLEAR_EVENT_THROTTLE, 1, &[0x00]);
        h.drain();
        assert_eq!(
            *h.ground.responses.lock().unwrap(),
            vec![(
                ID_BASE + DpManager::OPCODE_CLEAR_EVENT_THROTTLE,
                1,
                CmdResponse::FormatError
            )]
        );
    }

    #[test]
    fn product_send_envelope_carries_the_escrow_token() {
        let h = Harness::new();
        let port = h.comp.product_send_in(1);
        port.target
            .invoke(port.port_num, 0x11, Buffer::allocate(65));
        let bytes = h.tap_queue();
        assert_eq!(bytes.len(), 6 + 4 + 8);
        assert_eq!(&bytes[..6], &[0x00, 0x00, 0x00, 0x03, 0x00, 0x01]);
        assert_eq!(&bytes[6..10], &[0x00, 0x00, 0x00, 0x11]);
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn a_port_number_beyond_the_array_asserts() {
        let comp = DpManager::new("dpManager");
        let _ = comp.product_get_in(DP_MANAGER_NUM_PORTS as FwIndexType);
    }
}
