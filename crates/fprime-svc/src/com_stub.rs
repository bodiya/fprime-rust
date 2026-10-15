//! # ComStub — port of `Svc::ComStub` (passive com adapter)
//!
//! C++ sources: `Svc/ComStub/ComStub.{cpp,hpp,fpp}` and the
//! `Drv/ByteStreamDriverModel` / `Drv/Interfaces` port definitions.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (ComStub + ByteStreamDriver).
//!
//! Implements the `Svc.Com` adapter interface over either a SYNCHRONOUS
//! byte-stream driver (`Drv.ByteStreamDriverClient`: `drvSendOut` returns
//! the status, the caller keeps the buffer) or an ASYNCHRONOUS one
//! (`Drv.AsyncByteStreamDriver`: `drvAsyncSendOut` hands the buffer to the
//! driver, which gives it back with the status on `drvAsyncSendReturnIn`).
//! The path is chosen per send exactly as C++ does: `drvSendOut` connected
//! → synchronous; else `drvAsyncSendOut` connected → asynchronous; neither →
//! `fw_assert!`.
//!
//! Shared behavior: bounded send retries ([`RETRY_LIMIT`] = 10 — inline for
//! the sync path, counted across callbacks for the async path), the
//! `m_reinitialize` handshake (the system bootstrap — the first
//! `comStatus SUCCESS` is emitted only on driver `ready`, priming ComQueue),
//! and the receive path forwarding driver buffers upstream with a DEFAULT
//! [`FrameContext`].
//!
//! ## Async-path ownership
//!
//! C++ passes `Fw::Buffer&` everywhere; here the buffer MOVES: `dataIn` →
//! `drvAsyncSendOut` (driver owns it while the send is in flight) →
//! `drvAsyncSendReturnIn` (back to ComStub with the status) → either
//! `drvAsyncSendOut` again (SEND_RETRY, same buffer) or `dataReturnOut`
//! (ownership back upstream to the framer, paired with the context stored at
//! `dataIn`). Exactly one buffer is in flight — the ComQueue one-outstanding
//! protocol guarantees it, and C++ relies on the same thing for its single
//! `m_storedContext`.
//!
//! ## Byte-stream port seam (documented)
//!
//! `fprime-svc` may not depend on `fprime-drv` (dependency DAG), so the
//! byte-stream port traits and [`ByteStreamStatus`] are defined PUBLICLY
//! here. `fprime-drv` defines its own structurally-identical traits; the
//! reference deployment glues the two with tiny adapter shims. The async
//! send direction needs no seam: `drvAsyncSendOut` is the framework
//! `Fw.BufferSend` port ([`BufferSendPort`]) on both sides.

use fprime_comp::{
    BufferSendPort, ComDataWithContextPort, OutputPort, PassiveBase, PortRef, SuccessConditionPort,
};
use fprime_config::FwIndexType;
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::sync::{AtomicBool, Ordering};
use fprime_fw::{Buffer, FrameContext, Success, fw_assert, fw_log};

/// `Drv::ByteStreamStatus` (repr U8) — duplicated here per the module-header
/// seam note; `fprime-drv` owns the canonical copy for driver code.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum ByteStreamStatus {
    /// Operation succeeded.
    #[default]
    OpOk = 0,
    /// Send should be retried (e.g. would block).
    SendRetry = 1,
    /// No data available on a receive.
    RecvNoData = 2,
    /// Any other error.
    OtherError = 3,
}

/// `Drv.ByteStreamSend` — synchronous send; the caller retains buffer
/// ownership (hence `&mut Buffer`), the driver returns a status.
pub trait ByteStreamSendPort: Send + Sync {
    /// Send the buffer's window over the stream.
    fn invoke(&self, port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus;
}

/// `Drv.ByteStreamReady` — driver-ready signal (no arguments).
pub trait ByteStreamReadyPort: Send + Sync {
    /// The driver (re)connected.
    fn invoke(&self, port_num: FwIndexType);
}

/// `Drv.ByteStreamData` — receive delivery (and async send callback):
/// buffer ownership moves to the receiver along with the status.
pub trait ByteStreamDataPort: Send + Sync {
    /// Deliver a buffer with its transfer status.
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer, status: ByteStreamStatus);
}

/// Bounded send retry limit (C++ `ComStub::RETRY_LIMIT`, a class member
/// constant upstream — not a project config value). Applies to both the
/// inline synchronous retry loop and the asynchronous retry counter.
pub const RETRY_LIMIT: FwIndexType = 10;

/// Asynchronous-send bookkeeping (C++ `m_storedContext` / `m_retry_count`).
///
/// C++ keeps these as plain members with no lock (every handler is `sync`).
/// They are written from `dataIn` (the sender's thread) and from
/// `drvAsyncSendReturnIn` (the driver's callback thread), so they live under
/// a mutex here. The lock is held only around the field accesses, never
/// across an output-port call: an async driver may call
/// `drvAsyncSendReturnIn` re-entrantly from inside `drvAsyncSendOut`.
#[derive(Debug, Default)]
struct AsyncSendState {
    /// Context of the (single) in-flight async send, returned with the
    /// buffer on `dataReturnOut`.
    stored_context: FrameContext,
    /// Async SEND_RETRY count for the in-flight buffer.
    retry_count: FwIndexType,
}

/// `Svc::ComStub` — passive com adapter over a synchronous or asynchronous
/// byte-stream driver. No events, telemetry, or commands.
pub struct ComStub {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `dataOut` — received data upstream (to the FrameAccumulator).
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — send-buffer ownership back upstream (to the framer).
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `comStatusOut` — com status upstream (ultimately to ComQueue).
    pub com_status_out: OutputPort<dyn SuccessConditionPort>,
    /// `drvSendOut` — synchronous driver send.
    pub drv_send_out: OutputPort<dyn ByteStreamSendPort>,
    /// `drvReceiveReturnOut` — received-buffer ownership back to the driver.
    pub drv_receive_return_out: OutputPort<dyn BufferSendPort>,
    /// `drvAsyncSendOut` — asynchronous driver send (`Fw.BufferSend`): the
    /// buffer moves to the driver, which returns it with the send status on
    /// [`ComStub::drv_async_send_return_in`].
    pub drv_async_send_out: OutputPort<dyn BufferSendPort>,
    /// C++ `m_reinitialize`: starts true; a driver `ready` emits the
    /// (re-)initialization SUCCESS and clears it. Relaxed atomic — the C++
    /// member is a plain bool touched from sync handlers.
    reinitialize: AtomicBool,
    /// Async-send context + retry counter (see [`AsyncSendState`]).
    async_state: Mutex<AsyncSendState>,
}

impl ComStub {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            com_status_out: OutputPort::new(),
            drv_send_out: OutputPort::new(),
            drv_receive_return_out: OutputPort::new(),
            drv_async_send_out: OutputPort::new(),
            reinitialize: AtomicBool::new(true),
            async_state: Mutex::new(AsyncSendState::default()),
        })
    }

    // -- Input-port factories -----------------------------------------------

    /// `dataIn` — SYNC `Svc.ComDataWithContext` input: data to send on the
    /// wire.
    pub fn data_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(Arc::new(DataInAdapter { comp: self.clone() }), port_num)
    }

    /// `dataReturnIn` — SYNC input: received-data buffer coming back from
    /// upstream; returned to the driver.
    pub fn data_return_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(
            Arc::new(DataReturnInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `drvConnected` — SYNC `Drv.ByteStreamReady` input.
    pub fn drv_connected(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn ByteStreamReadyPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `drvReceiveIn` — SYNC `Drv.ByteStreamData` input: data read from the
    /// wire.
    pub fn drv_receive_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn ByteStreamDataPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `drvAsyncSendReturnIn` — SYNC `Drv.ByteStreamData` input: status and
    /// buffer ownership coming back from an asynchronous driver send
    /// (connect the driver's `sendReturnOut` here).
    pub fn drv_async_send_return_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn ByteStreamDataPort> {
        PortRef::new(
            Arc::new(DrvAsyncSendReturnInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    // -- Handlers ------------------------------------------------------------

    /// `dataIn` handler: dispatch to the synchronous or asynchronous send
    /// path depending on which driver send port is connected (sync wins
    /// when both are, C++ parity).
    fn data_in_handler(&self, _port_num: FwIndexType, send_buffer: Buffer, context: &FrameContext) {
        // A message should never get here while we need reinitialization
        // (C++ parity assert).
        fw_assert!(
            !self.reinitialize.load(Ordering::Relaxed) || !self.com_status_out.is_connected()
        );
        if self.drv_send_out.is_connected() {
            self.handle_synchronous_send(send_buffer, context);
        } else if self.drv_async_send_out.is_connected() {
            self.handle_asynchronous_send(send_buffer, context);
        } else {
            // Neither send port is connected; this should never happen.
            fw_assert!(false);
        }
    }

    /// C++ `handleSynchronousSend`: send with bounded inline retry, then
    /// always return the buffer and emit the com status.
    fn handle_synchronous_send(&self, mut send_buffer: Buffer, context: &FrameContext) {
        // Send to the driver, retrying up to RETRY_LIMIT on SEND_RETRY.
        let mut send_status = ByteStreamStatus::SendRetry;
        let p = self.drv_send_out.get();
        let mut i = 0;
        while send_status == ByteStreamStatus::SendRetry && i < RETRY_LIMIT {
            send_status = p.target.invoke(p.port_num, &mut send_buffer);
            i += 1;
        }
        let com_success = match send_status {
            ByteStreamStatus::OpOk => Success::Success,
            ByteStreamStatus::SendRetry => {
                // Retry exhaustion requires a driver reconnect to emit the
                // recovery SUCCESS.
                self.reinitialize.store(true, Ordering::Relaxed);
                fw_log!("ComStub RETRY_LIMIT exceeded, skipped sending data");
                Success::Failure
            }
            _ => {
                // Other error — need to reinitialize.
                self.reinitialize.store(true, Ordering::Relaxed);
                Success::Failure
            }
        };
        // Always return the buffer, then emit the com status.
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, send_buffer, context);
        let p = self.com_status_out.get();
        let mut condition = com_success;
        p.target.invoke(p.port_num, &mut condition);
    }

    /// C++ `handleAsynchronousSend`: remember the context for the callback
    /// and hand the buffer to the driver. Status and ownership come back on
    /// `drvAsyncSendReturnIn`.
    fn handle_asynchronous_send(&self, send_buffer: Buffer, context: &FrameContext) {
        self.async_state.lock().stored_context = *context;
        let p = self.drv_async_send_out.get();
        p.target.invoke(p.port_num, send_buffer);
    }

    /// `drvAsyncSendReturnIn` handler: SEND_RETRY resends (bounded); any
    /// other status ends the send — buffer back upstream with the stored
    /// context, reinitialize iff not OP_OK, retry count reset, then
    /// SUCCESS iff OP_OK on `comStatusOut`.
    fn drv_async_send_return_in_handler(
        &self,
        _port_num: FwIndexType,
        fw_buffer: Buffer,
        send_status: ByteStreamStatus,
    ) {
        // This should never be called if drvAsyncSendOut is not connected.
        fw_assert!(self.drv_async_send_out.is_connected());
        if send_status == ByteStreamStatus::SendRetry {
            self.handle_async_retry(fw_buffer);
        } else {
            // Return buffer ownership and send status.
            let stored_context = self.async_state.lock().stored_context;
            let p = self.data_return_out.get();
            p.target.invoke(p.port_num, fw_buffer, &stored_context);
            self.reinitialize
                .store(send_status != ByteStreamStatus::OpOk, Ordering::Relaxed);
            self.async_state.lock().retry_count = 0;
            let mut com_success = if send_status == ByteStreamStatus::OpOk {
                Success::Success
            } else {
                Success::Failure
            };
            let p = self.com_status_out.get();
            p.target.invoke(p.port_num, &mut com_success);
        }
    }

    /// C++ `handleAsyncRetry`: resend the same buffer while under
    /// [`RETRY_LIMIT`]; on exhaustion return it upstream with FAILURE and
    /// re-arm the reinitialize handshake.
    fn handle_async_retry(&self, fw_buffer: Buffer) {
        let mut state = self.async_state.lock();
        fw_assert!(state.retry_count <= RETRY_LIMIT, state.retry_count);
        if state.retry_count < RETRY_LIMIT {
            // Attempt retry if under the limit.
            state.retry_count += 1;
            drop(state);
            let p = self.drv_async_send_out.get();
            p.target.invoke(p.port_num, fw_buffer);
        } else {
            let stored_context = state.stored_context;
            drop(state);
            // Exceeded retry limit — return buffer and notify failure. Retry
            // exhaustion requires a driver reconnect to emit the recovery
            // SUCCESS.
            self.reinitialize.store(true, Ordering::Relaxed);
            let p = self.data_return_out.get();
            p.target.invoke(p.port_num, fw_buffer, &stored_context);
            let mut com_status = Success::Failure;
            let p = self.com_status_out.get();
            p.target.invoke(p.port_num, &mut com_status);
            fw_log!("ComStub RETRY_LIMIT exceeded, skipped sending data");
            // C++ parity: the count is reset only after the status/log calls.
            self.async_state.lock().retry_count = 0;
        }
    }

    fn data_return_in_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        // Received-data buffer coming back from upstream: give it back to
        // the driver.
        let p = self.drv_receive_return_out.get();
        p.target.invoke(p.port_num, buffer);
    }
}

/// `drvConnected` handler: emit the (re-)initialization SUCCESS exactly once
/// per reconnect — this is what primes ComQueue out of its initial WAITING
/// state (gotcha 6 in the analysis).
impl ByteStreamReadyPort for ComStub {
    fn invoke(&self, _port_num: FwIndexType) {
        if self.com_status_out.is_connected() && self.reinitialize.load(Ordering::Relaxed) {
            self.reinitialize.store(false, Ordering::Relaxed);
            let p = self.com_status_out.get();
            let mut condition = Success::Success;
            p.target.invoke(p.port_num, &mut condition);
        }
    }
}

/// `drvReceiveIn` handler: forward good reads upstream with a DEFAULT
/// context (ComStub knows nothing about the bytes); return failed reads to
/// the driver immediately.
impl ByteStreamDataPort for ComStub {
    fn invoke(&self, _port_num: FwIndexType, recv_buffer: Buffer, recv_status: ByteStreamStatus) {
        if recv_status != ByteStreamStatus::OpOk {
            let p = self.drv_receive_return_out.get();
            p.target.invoke(p.port_num, recv_buffer);
        } else {
            let empty_context = FrameContext::default();
            let p = self.data_out.get();
            p.target.invoke(p.port_num, recv_buffer, &empty_context);
        }
    }
}

/// Adapter for the sync `dataIn` port.
struct DataInAdapter {
    comp: Arc<ComStub>,
}

impl ComDataWithContextPort for DataInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.comp.data_in_handler(port_num, data, context);
    }
}

/// Adapter for the sync `dataReturnIn` port.
struct DataReturnInAdapter {
    comp: Arc<ComStub>,
}

impl ComDataWithContextPort for DataReturnInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, _context: &FrameContext) {
        self.comp.data_return_in_handler(port_num, data);
    }
}

/// Adapter for the sync `drvAsyncSendReturnIn` port (ComStub itself
/// implements [`ByteStreamDataPort`] as `drvReceiveIn`).
struct DrvAsyncSendReturnInAdapter {
    comp: Arc<ComStub>,
}

impl ByteStreamDataPort for DrvAsyncSendReturnInAdapter {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer, status: ByteStreamStatus) {
        self.comp
            .drv_async_send_return_in_handler(port_num, buffer, status);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    /// Scripted byte-stream driver: pops one status per send call (last
    /// entry repeats when exhausted).
    struct DriverStub {
        script: Mutex<Vec<ByteStreamStatus>>,
        sends: Mutex<Vec<Vec<u8>>>,
    }

    impl DriverStub {
        fn scripted(script: Vec<ByteStreamStatus>) -> Arc<Self> {
            Arc::new(Self {
                script: Mutex::new(script),
                sends: Mutex::new(vec![]),
            })
        }
    }

    impl ByteStreamSendPort for DriverStub {
        fn invoke(&self, _port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus {
            self.sends.lock().unwrap().push(buffer.data().to_vec());
            let mut script = self.script.lock().unwrap();
            if script.len() > 1 {
                script.remove(0)
            } else {
                *script.first().unwrap_or(&ByteStreamStatus::OpOk)
            }
        }
    }

    #[derive(Default)]
    struct Recorder {
        data_out: Mutex<Vec<(Vec<u8>, FrameContext)>>,
        data_return: Mutex<Vec<(Vec<u8>, FrameContext)>>,
        statuses: Mutex<Vec<Success>>,
        receive_returns: Mutex<Vec<Vec<u8>>>,
    }

    impl ComDataWithContextPort for Recorder {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            let rec = (data.data().to_vec(), *context);
            if port_num == 0 {
                self.data_out.lock().unwrap().push(rec);
            } else {
                self.data_return.lock().unwrap().push(rec);
            }
        }
    }

    impl SuccessConditionPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, condition: &mut Success) {
            self.statuses.lock().unwrap().push(*condition);
        }
    }

    impl BufferSendPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.receive_returns
                .lock()
                .unwrap()
                .push(buffer.data().to_vec());
        }
    }

    fn build(driver: &Arc<DriverStub>) -> (Arc<ComStub>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let stub = ComStub::new("comStub");
        stub.data_out.connect(rec.clone(), 0);
        stub.data_return_out.connect(rec.clone(), 1);
        stub.com_status_out.connect(rec.clone(), 0);
        stub.drv_send_out.connect(driver.clone(), 0);
        stub.drv_receive_return_out.connect(rec.clone(), 0);
        (stub, rec)
    }

    fn buffer_of(bytes: &[u8]) -> Buffer {
        let mut b = Buffer::allocate(bytes.len().max(1));
        b.data_mut()[..bytes.len()].copy_from_slice(bytes);
        b.set_size(bytes.len());
        b
    }

    fn connect_driver(stub: &Arc<ComStub>) {
        let ready = stub.drv_connected(0);
        ready.target.invoke(ready.port_num);
    }

    /// drvConnected emits the bootstrap SUCCESS exactly once.
    #[test]
    fn driver_ready_emits_success_once() {
        let driver = DriverStub::scripted(vec![ByteStreamStatus::OpOk]);
        let (stub, rec) = build(&driver);
        connect_driver(&stub);
        assert_eq!(*rec.statuses.lock().unwrap(), vec![Success::Success]);
        // A second ready without a failure in between emits nothing.
        connect_driver(&stub);
        assert_eq!(rec.statuses.lock().unwrap().len(), 1);
    }

    /// drvConnected with comStatusOut unconnected does nothing (and keeps
    /// the reinitialize flag set).
    #[test]
    fn driver_ready_without_status_port_is_noop() {
        let driver = DriverStub::scripted(vec![ByteStreamStatus::OpOk]);
        let stub = ComStub::new("stub");
        stub.drv_send_out.connect(driver.clone(), 0);
        connect_driver(&stub);
        assert!(stub.reinitialize.load(Ordering::Relaxed));
    }

    /// Happy send: one driver call, buffer returned, SUCCESS status.
    #[test]
    fn send_ok_returns_buffer_and_success() {
        let driver = DriverStub::scripted(vec![ByteStreamStatus::OpOk]);
        let (stub, rec) = build(&driver);
        connect_driver(&stub);
        let ctx = FrameContext::default();
        let din = stub.data_in(0);
        din.target.invoke(din.port_num, buffer_of(b"frame"), &ctx);

        assert_eq!(*driver.sends.lock().unwrap(), vec![b"frame".to_vec()]);
        assert_eq!(rec.data_return.lock().unwrap().len(), 1);
        assert_eq!(rec.data_return.lock().unwrap()[0].0, b"frame");
        // Bootstrap SUCCESS + send SUCCESS.
        assert_eq!(
            *rec.statuses.lock().unwrap(),
            vec![Success::Success, Success::Success]
        );
    }

    /// SEND_RETRY a few times then OP_OK: retried inline, still SUCCESS.
    #[test]
    fn send_retry_then_ok() {
        let driver = DriverStub::scripted(vec![
            ByteStreamStatus::SendRetry,
            ByteStreamStatus::SendRetry,
            ByteStreamStatus::OpOk,
        ]);
        let (stub, rec) = build(&driver);
        connect_driver(&stub);
        let din = stub.data_in(0);
        din.target
            .invoke(din.port_num, buffer_of(b"r"), &FrameContext::default());
        assert_eq!(driver.sends.lock().unwrap().len(), 3);
        assert_eq!(
            *rec.statuses.lock().unwrap(),
            vec![Success::Success, Success::Success]
        );
        assert!(!stub.reinitialize.load(Ordering::Relaxed));
    }

    /// Retry exhaustion: exactly RETRY_LIMIT driver calls, FAILURE, and the
    /// reinitialize handshake re-arms (recovery SUCCESS on next ready).
    #[test]
    fn retry_exhaustion_fails_and_rearms_handshake() {
        let driver = DriverStub::scripted(vec![ByteStreamStatus::SendRetry]);
        let (stub, rec) = build(&driver);
        connect_driver(&stub);
        let din = stub.data_in(0);
        din.target
            .invoke(din.port_num, buffer_of(b"x"), &FrameContext::default());
        assert_eq!(driver.sends.lock().unwrap().len(), RETRY_LIMIT as usize);
        assert_eq!(
            *rec.statuses.lock().unwrap(),
            vec![Success::Success, Success::Failure]
        );
        // Buffer still returned.
        assert_eq!(rec.data_return.lock().unwrap().len(), 1);
        assert!(stub.reinitialize.load(Ordering::Relaxed));
        // Driver reconnect emits the recovery SUCCESS.
        connect_driver(&stub);
        assert_eq!(
            *rec.statuses.lock().unwrap(),
            vec![Success::Success, Success::Failure, Success::Success]
        );
    }

    /// OTHER_ERROR: single attempt, FAILURE, reinitialize set.
    #[test]
    fn other_error_fails_immediately() {
        let driver = DriverStub::scripted(vec![ByteStreamStatus::OtherError]);
        let (stub, rec) = build(&driver);
        connect_driver(&stub);
        let din = stub.data_in(0);
        din.target
            .invoke(din.port_num, buffer_of(b"e"), &FrameContext::default());
        assert_eq!(driver.sends.lock().unwrap().len(), 1);
        assert_eq!(
            *rec.statuses.lock().unwrap(),
            vec![Success::Success, Success::Failure]
        );
        assert!(stub.reinitialize.load(Ordering::Relaxed));
    }

    /// Sending while reinitialization is pending (and comStatusOut is
    /// connected) is a protocol violation and asserts (C++ parity).
    #[test]
    #[should_panic]
    fn send_while_reinitialize_pending_asserts() {
        let driver = DriverStub::scripted(vec![ByteStreamStatus::OpOk]);
        let (stub, _rec) = build(&driver);
        // No drvConnected yet -> reinitialize still true.
        let din = stub.data_in(0);
        din.target
            .invoke(din.port_num, buffer_of(b"early"), &FrameContext::default());
    }

    /// Receive OP_OK: forwarded upstream with a DEFAULT FrameContext
    /// (gotcha: not the driver's context).
    #[test]
    fn receive_ok_forwards_with_default_context() {
        let driver = DriverStub::scripted(vec![ByteStreamStatus::OpOk]);
        let (stub, rec) = build(&driver);
        let rin = stub.drv_receive_in(0);
        rin.target
            .invoke(rin.port_num, buffer_of(b"rx"), ByteStreamStatus::OpOk);
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].0, b"rx");
        assert_eq!(out[0].1, FrameContext::default());
        assert!(rec.receive_returns.lock().unwrap().is_empty());
    }

    /// Receive failure: buffer returned to the driver, nothing forwarded.
    #[test]
    fn receive_error_returns_buffer_to_driver() {
        let driver = DriverStub::scripted(vec![ByteStreamStatus::OpOk]);
        let (stub, rec) = build(&driver);
        let rin = stub.drv_receive_in(0);
        rin.target.invoke(
            rin.port_num,
            buffer_of(b"bad"),
            ByteStreamStatus::RecvNoData,
        );
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert_eq!(*rec.receive_returns.lock().unwrap(), vec![b"bad".to_vec()]);
    }

    /// dataReturnIn hands the received buffer back to the driver.
    #[test]
    fn data_return_in_goes_to_driver() {
        let driver = DriverStub::scripted(vec![ByteStreamStatus::OpOk]);
        let (stub, rec) = build(&driver);
        let drin = stub.data_return_in(0);
        drin.target
            .invoke(drin.port_num, buffer_of(b"back"), &FrameContext::default());
        assert_eq!(*rec.receive_returns.lock().unwrap(), vec![b"back".to_vec()]);
    }

    // -----------------------------------------------------------------------
    // Port of the upstream `Svc/ComStub/test/ut` suite (ComStubTester +
    // ComStubTestMain), including its SYNC / ASYNC / UNSPECIFIED test modes,
    // plus the async-path cases the Rust ownership model adds.
    // -----------------------------------------------------------------------

    /// Async byte-stream driver double: takes ownership of every buffer sent
    /// on `drvAsyncSendOut` and keeps it (C++ `from_drvAsyncSendOut` history).
    #[derive(Default)]
    struct AsyncDriverStub {
        sends: Mutex<Vec<Buffer>>,
    }

    impl AsyncDriverStub {
        fn sent(&self) -> Vec<(Vec<u8>, u32)> {
            self.sends
                .lock()
                .unwrap()
                .iter()
                .map(|b| (b.data().to_vec(), b.context()))
                .collect()
        }
    }

    impl BufferSendPort for AsyncDriverStub {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.sends.lock().unwrap().push(buffer);
        }
    }

    /// C++ `ComStubTester::TestMode`.
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum TestMode {
        /// Connect every port (both driver send paths).
        Unspecified,
        /// Only the synchronous `drvSendOut`.
        Sync,
        /// Only the asynchronous `drvAsyncSendOut` (+ its return input).
        Async,
    }

    struct Tester {
        stub: Arc<ComStub>,
        rec: Arc<Recorder>,
        sync_driver: Arc<DriverStub>,
        async_driver: Arc<AsyncDriverStub>,
    }

    impl Tester {
        /// C++ `connectPorts` / `connectPortsWithTestMode`. The sync driver
        /// is scripted per test (last status repeats).
        fn new(mode: TestMode, sync_script: Vec<ByteStreamStatus>) -> Self {
            let sync_driver = DriverStub::scripted(sync_script);
            let async_driver = Arc::new(AsyncDriverStub::default());
            let rec = Arc::new(Recorder::default());
            let stub = ComStub::new("ComStub");
            stub.data_out.connect(rec.clone(), 0);
            stub.data_return_out.connect(rec.clone(), 1);
            stub.com_status_out.connect(rec.clone(), 0);
            stub.drv_receive_return_out.connect(rec.clone(), 0);
            if mode != TestMode::Async {
                stub.drv_send_out.connect(sync_driver.clone(), 0);
            }
            if mode != TestMode::Sync {
                stub.drv_async_send_out.connect(async_driver.clone(), 0);
            }
            Self {
                stub,
                rec,
                sync_driver,
                async_driver,
            }
        }

        fn statuses(&self) -> Vec<Success> {
            self.rec.statuses.lock().unwrap().clone()
        }

        fn returned(&self) -> Vec<(Vec<u8>, FrameContext)> {
            self.rec.data_return.lock().unwrap().clone()
        }

        fn retry_count(&self) -> FwIndexType {
            self.stub.async_state.lock().retry_count
        }

        fn data_in(&self, buffer: Buffer, context: &FrameContext) {
            let din = self.stub.data_in(0);
            din.target.invoke(din.port_num, buffer, context);
        }

        fn async_return(&self, buffer: Buffer, status: ByteStreamStatus) {
            let ret = self.stub.drv_async_send_return_in(0);
            ret.target.invoke(ret.port_num, buffer, status);
        }

        fn receive(&self, buffer: Buffer, status: ByteStreamStatus) {
            let rin = self.stub.drv_receive_in(0);
            rin.target.invoke(rin.port_num, buffer, status);
        }

        /// C++ `test_initial`: drvConnected emits exactly one SUCCESS; the
        /// status history is then cleared.
        fn test_initial(&self) {
            connect_driver(&self.stub);
            assert_eq!(self.statuses(), vec![Success::Success]);
            self.rec.statuses.lock().unwrap().clear();
        }
    }

    /// An 8-byte buffer with a distinguishing context word (C++
    /// `buffers[i].set(storage[i], 8, i)` + `fill`).
    fn tagged_buffer(tag: u32) -> Buffer {
        let mut b = buffer_of(&[tag as u8, 0xA5, 0x5A, (tag >> 8) as u8]);
        b.set_context(tag);
        b
    }

    /// C++ `TEST(Common, Initial)`.
    #[test]
    fn common_initial_emits_single_success() {
        let t = Tester::new(TestMode::Unspecified, vec![ByteStreamStatus::OpOk]);
        t.test_initial();
        assert!(!t.stub.reinitialize.load(Ordering::Relaxed));
    }

    /// C++ `TEST(Sync, BasicIo)`.
    #[test]
    fn sync_basic_io() {
        let t = Tester::new(TestMode::Sync, vec![ByteStreamStatus::OpOk]);
        t.test_initial();
        t.data_in(buffer_of(b"downlink"), &FrameContext::default());
        assert_eq!(
            *t.sync_driver.sends.lock().unwrap(),
            vec![b"downlink".to_vec()]
        );
        assert!(t.async_driver.sent().is_empty());
        // Uplink: forwarded with a default (empty) context.
        t.receive(buffer_of(b"uplink"), ByteStreamStatus::OpOk);
        assert_eq!(
            *t.rec.data_out.lock().unwrap(),
            vec![(b"uplink".to_vec(), FrameContext::default())]
        );
    }

    /// C++ `TEST(Async, BasicIo)`: dataIn goes out drvAsyncSendOut; nothing
    /// is returned or reported until the driver calls back.
    #[test]
    fn async_basic_io() {
        let t = Tester::new(TestMode::Async, vec![ByteStreamStatus::OpOk]);
        t.test_initial();
        t.data_in(tagged_buffer(7), &FrameContext::default());
        assert_eq!(
            t.async_driver.sent(),
            vec![(tagged_buffer(7).data().to_vec(), 7)]
        );
        assert!(t.returned().is_empty());
        assert!(t.statuses().is_empty());
        // Uplink is identical to the sync mode.
        t.receive(buffer_of(b"uplink"), ByteStreamStatus::OpOk);
        assert_eq!(
            *t.rec.data_out.lock().unwrap(),
            vec![(b"uplink".to_vec(), FrameContext::default())]
        );
    }

    /// C++ `TEST(Sync, Fail)`: no retry on OTHER_ERROR, buffer returned;
    /// failed receive returns the buffer to the driver.
    #[test]
    fn sync_fail() {
        let t = Tester::new(TestMode::Sync, vec![ByteStreamStatus::OtherError]);
        t.test_initial();
        t.data_in(buffer_of(b"x"), &FrameContext::default());
        assert_eq!(t.sync_driver.sends.lock().unwrap().len(), 1);
        assert_eq!(t.returned().len(), 1);
        t.receive(buffer_of(b"bad"), ByteStreamStatus::OtherError);
        assert!(t.rec.data_out.lock().unwrap().is_empty());
        assert_eq!(t.rec.receive_returns.lock().unwrap().len(), 1);
    }

    /// C++ `TEST(Async, Fail)`.
    #[test]
    fn async_fail() {
        let t = Tester::new(TestMode::Async, vec![ByteStreamStatus::OpOk]);
        t.test_initial();
        t.data_in(tagged_buffer(1), &FrameContext::default());
        assert_eq!(t.async_driver.sent().len(), 1);
        t.receive(buffer_of(b"bad"), ByteStreamStatus::OtherError);
        assert!(t.rec.data_out.lock().unwrap().is_empty());
        assert_eq!(t.rec.receive_returns.lock().unwrap().len(), 1);
    }

    /// C++ `TEST(Nominal, BufferReturn)`: the very same storage goes back
    /// out drvReceiveReturnOut.
    #[test]
    fn nominal_buffer_return_moves_same_storage() {
        struct Sink(Mutex<Vec<Buffer>>);
        impl BufferSendPort for Sink {
            fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
                self.0.lock().unwrap().push(buffer);
            }
        }
        let sink = Arc::new(Sink(Mutex::new(vec![])));
        let stub = ComStub::new("ComStub");
        stub.drv_receive_return_out.connect(sink.clone(), 0);
        let buffer = buffer_of(&[0x42]);
        let data_ptr = buffer.data().as_ptr();
        let drin = stub.data_return_in(0);
        drin.target
            .invoke(drin.port_num, buffer, &FrameContext::default());
        let returned = sink.0.lock().unwrap();
        assert_eq!(returned.len(), 1);
        assert_eq!(returned[0].data().as_ptr(), data_ptr);
        assert_eq!(returned[0].size(), 1);
    }

    /// C++ `TEST(Sync, Retry)`: SEND_RETRY forever -> exactly RETRY_LIMIT
    /// driver calls, buffer + context returned, one FAILURE.
    #[test]
    fn sync_retry() {
        let t = Tester::new(TestMode::Sync, vec![ByteStreamStatus::SendRetry]);
        t.test_initial();
        let mut ctx = FrameContext::default();
        ctx.set_com_queue_index(2);
        t.data_in(buffer_of(b"retry"), &ctx);
        assert_eq!(
            t.sync_driver.sends.lock().unwrap().len(),
            RETRY_LIMIT as usize
        );
        assert_eq!(t.returned(), vec![(b"retry".to_vec(), ctx)]);
        assert_eq!(t.statuses(), vec![Success::Failure]);
    }

    /// C++ `TEST(Sync, RetryReset)`: the RETRY_LIMIT-th attempt succeeds,
    /// then a fresh send succeeds first time.
    #[test]
    fn sync_retry_reset() {
        let mut script = vec![ByteStreamStatus::SendRetry; RETRY_LIMIT as usize - 1];
        script.push(ByteStreamStatus::OpOk);
        let t = Tester::new(TestMode::Sync, script);
        t.test_initial();
        let ctx = FrameContext::default();
        t.data_in(buffer_of(b"r"), &ctx);
        assert_eq!(
            t.sync_driver.sends.lock().unwrap().len(),
            RETRY_LIMIT as usize
        );
        assert_eq!(t.returned(), vec![(b"r".to_vec(), ctx)]);
        assert_eq!(t.statuses(), vec![Success::Success]);
        t.data_in(buffer_of(b"s"), &ctx);
        assert_eq!(
            t.sync_driver.sends.lock().unwrap().len(),
            RETRY_LIMIT as usize + 1
        );
        assert_eq!(t.returned().len(), 2);
        assert_eq!(t.returned()[1], (b"s".to_vec(), ctx));
        assert_eq!(t.statuses(), vec![Success::Success, Success::Success]);
    }

    /// C++ `TEST(Async, Retry)`: RETRY_LIMIT SEND_RETRY callbacks are each
    /// resent with the same buffer; the next one exhausts the limit — no
    /// resend, buffer back upstream with the stored context, FAILURE, count
    /// reset, reinitialize re-armed.
    #[test]
    fn async_retry() {
        let t = Tester::new(TestMode::Async, vec![ByteStreamStatus::OpOk]);
        t.test_initial();
        for i in 0..RETRY_LIMIT {
            t.async_return(tagged_buffer(i as u32), ByteStreamStatus::SendRetry);
            let sent = t.async_driver.sent();
            assert_eq!(sent.len(), i as usize + 1);
            assert_eq!(sent[i as usize].1, i as u32);
        }
        assert_eq!(t.retry_count(), RETRY_LIMIT);
        assert!(t.returned().is_empty());
        t.async_return(
            tagged_buffer(RETRY_LIMIT as u32),
            ByteStreamStatus::SendRetry,
        );
        assert_eq!(t.async_driver.sent().len(), RETRY_LIMIT as usize);
        assert_eq!(
            t.returned(),
            vec![(
                tagged_buffer(RETRY_LIMIT as u32).data().to_vec(),
                FrameContext::default()
            )]
        );
        assert_eq!(t.retry_count(), 0);
        assert_eq!(t.statuses(), vec![Success::Failure]);
        assert!(t.stub.reinitialize.load(Ordering::Relaxed));
        // Recovery SUCCESS on the next driver ready.
        connect_driver(&t.stub);
        assert_eq!(t.statuses(), vec![Success::Failure, Success::Success]);
    }

    /// C++ `TEST(Async, RetryReset)`: an OP_OK callback after RETRY_LIMIT
    /// retries does not resend and resets the count, so a full new run of
    /// RETRY_LIMIT retries is allowed again.
    #[test]
    fn async_retry_reset() {
        let t = Tester::new(TestMode::Async, vec![ByteStreamStatus::OpOk]);
        t.test_initial();
        let mut expected = 0usize;
        for i in 0..RETRY_LIMIT {
            t.async_return(tagged_buffer(i as u32), ByteStreamStatus::SendRetry);
            assert_eq!(t.async_driver.sent()[expected].1, i as u32);
            expected += 1;
            assert_eq!(t.async_driver.sent().len(), expected);
        }
        t.async_return(tagged_buffer(0), ByteStreamStatus::OpOk);
        assert_eq!(t.async_driver.sent().len(), expected);
        assert_eq!(t.retry_count(), 0);
        assert_eq!(t.statuses(), vec![Success::Success]);
        for i in 0..RETRY_LIMIT {
            t.async_return(tagged_buffer(i as u32), ByteStreamStatus::SendRetry);
            assert_eq!(t.async_driver.sent()[expected].1, i as u32);
            expected += 1;
            assert_eq!(t.async_driver.sent().len(), expected);
        }
        assert_eq!(t.async_driver.sent().len(), 2 * RETRY_LIMIT as usize);
    }

    /// Async OP_OK: the returned buffer goes upstream paired with the
    /// context stored at dataIn (not a default one), then SUCCESS.
    #[test]
    fn async_ok_returns_buffer_with_stored_context() {
        let t = Tester::new(TestMode::Async, vec![ByteStreamStatus::OpOk]);
        t.test_initial();
        let mut ctx = FrameContext::default();
        ctx.set_com_queue_index(2);
        ctx.set_sequence_count(0x1234);
        t.data_in(tagged_buffer(9), &ctx);
        // Hand the very buffer the driver received back, as a driver would.
        let in_flight = t.async_driver.sends.lock().unwrap().pop().unwrap();
        t.async_return(in_flight, ByteStreamStatus::OpOk);
        assert_eq!(t.returned(), vec![(tagged_buffer(9).data().to_vec(), ctx)]);
        assert_eq!(t.statuses(), vec![Success::Success]);
        assert!(!t.stub.reinitialize.load(Ordering::Relaxed));
    }

    /// Any non-OP_OK, non-SEND_RETRY callback status (OTHER_ERROR, and even
    /// RECV_NO_DATA) ends the send with FAILURE and re-arms the handshake.
    #[test]
    fn async_error_statuses_fail_and_rearm_handshake() {
        for status in [ByteStreamStatus::OtherError, ByteStreamStatus::RecvNoData] {
            let t = Tester::new(TestMode::Async, vec![ByteStreamStatus::OpOk]);
            t.test_initial();
            let ctx = FrameContext::default();
            t.data_in(tagged_buffer(3), &ctx);
            let in_flight = t.async_driver.sends.lock().unwrap().pop().unwrap();
            t.async_return(in_flight, status);
            assert_eq!(t.returned(), vec![(tagged_buffer(3).data().to_vec(), ctx)]);
            assert_eq!(t.statuses(), vec![Success::Failure]);
            assert!(t.stub.reinitialize.load(Ordering::Relaxed));
            assert_eq!(t.retry_count(), 0);
            connect_driver(&t.stub);
            assert_eq!(t.statuses(), vec![Success::Failure, Success::Success]);
        }
    }

    /// With both send ports connected (UNSPECIFIED mode) the synchronous
    /// path wins (C++ checks drvSendOut first).
    #[test]
    fn sync_path_preferred_when_both_connected() {
        let t = Tester::new(TestMode::Unspecified, vec![ByteStreamStatus::OpOk]);
        t.test_initial();
        t.data_in(buffer_of(b"both"), &FrameContext::default());
        assert_eq!(t.sync_driver.sends.lock().unwrap().len(), 1);
        assert!(t.async_driver.sent().is_empty());
        assert_eq!(t.statuses(), vec![Success::Success]);
    }

    /// dataIn with neither driver send port connected is FW_ASSERT.
    #[test]
    #[should_panic(expected = "com_stub.rs")]
    fn send_without_any_driver_port_asserts() {
        let rec = Arc::new(Recorder::default());
        let stub = ComStub::new("ComStub");
        stub.data_return_out.connect(rec.clone(), 1);
        stub.data_in(0)
            .target
            .invoke(0, buffer_of(b"x"), &FrameContext::default());
    }

    /// drvAsyncSendReturnIn while drvAsyncSendOut is unconnected is
    /// FW_ASSERT.
    #[test]
    #[should_panic(expected = "com_stub.rs")]
    fn async_return_without_async_port_asserts() {
        let t = Tester::new(TestMode::Sync, vec![ByteStreamStatus::OpOk]);
        t.async_return(buffer_of(b"x"), ByteStreamStatus::OpOk);
    }

    /// An async driver that calls `drvAsyncSendReturnIn` re-entrantly from
    /// inside `drvAsyncSendOut` (completing on the caller's thread) must
    /// not deadlock: ComStub holds no lock across port calls.
    #[test]
    fn async_reentrant_driver_callback_completes() {
        struct ReentrantDriver {
            ret: std::sync::OnceLock<PortRef<dyn ByteStreamDataPort>>,
            script: Mutex<Vec<ByteStreamStatus>>,
            calls: Mutex<usize>,
        }
        impl BufferSendPort for ReentrantDriver {
            fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
                *self.calls.lock().unwrap() += 1;
                let status = self.script.lock().unwrap().remove(0);
                let ret = self.ret.get().unwrap();
                ret.target.invoke(ret.port_num, buffer, status);
            }
        }
        let driver = Arc::new(ReentrantDriver {
            ret: std::sync::OnceLock::new(),
            script: Mutex::new(vec![
                ByteStreamStatus::SendRetry,
                ByteStreamStatus::SendRetry,
                ByteStreamStatus::OpOk,
            ]),
            calls: Mutex::new(0),
        });
        let rec = Arc::new(Recorder::default());
        let stub = ComStub::new("ComStub");
        stub.data_return_out.connect(rec.clone(), 1);
        stub.com_status_out.connect(rec.clone(), 0);
        stub.drv_async_send_out.connect(driver.clone(), 0);
        assert!(driver.ret.set(stub.drv_async_send_return_in(0)).is_ok());
        connect_driver(&stub);
        let mut ctx = FrameContext::default();
        ctx.set_com_queue_index(1);
        stub.data_in(0)
            .target
            .invoke(0, buffer_of(b"reentrant"), &ctx);
        assert_eq!(*driver.calls.lock().unwrap(), 3);
        assert_eq!(
            *rec.data_return.lock().unwrap(),
            vec![(b"reentrant".to_vec(), ctx)]
        );
        assert_eq!(
            *rec.statuses.lock().unwrap(),
            vec![Success::Success, Success::Success]
        );
        assert_eq!(stub.async_state.lock().retry_count, 0);
    }

    /// An async driver completing on its own thread: ownership round-trips
    /// and the callback runs on the driver thread.
    #[test]
    fn async_driver_completing_on_another_thread() {
        struct ThreadedDriver {
            ret: std::sync::OnceLock<PortRef<dyn ByteStreamDataPort>>,
            workers: Mutex<Vec<std::thread::JoinHandle<()>>>,
        }
        impl BufferSendPort for ThreadedDriver {
            fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
                let ret = self.ret.get().unwrap().clone();
                self.workers
                    .lock()
                    .unwrap()
                    .push(std::thread::spawn(move || {
                        ret.target
                            .invoke(ret.port_num, buffer, ByteStreamStatus::OpOk);
                    }));
            }
        }
        let driver = Arc::new(ThreadedDriver {
            ret: std::sync::OnceLock::new(),
            workers: Mutex::new(vec![]),
        });
        let rec = Arc::new(Recorder::default());
        let stub = ComStub::new("ComStub");
        stub.data_return_out.connect(rec.clone(), 1);
        stub.com_status_out.connect(rec.clone(), 0);
        stub.drv_async_send_out.connect(driver.clone(), 0);
        assert!(driver.ret.set(stub.drv_async_send_return_in(0)).is_ok());
        connect_driver(&stub);
        for n in 0..3u8 {
            stub.data_in(0)
                .target
                .invoke(0, buffer_of(&[n]), &FrameContext::default());
            let worker = driver.workers.lock().unwrap().pop().unwrap();
            worker.join().unwrap();
        }
        let returned: Vec<Vec<u8>> = rec
            .data_return
            .lock()
            .unwrap()
            .iter()
            .map(|(d, _)| d.clone())
            .collect();
        assert_eq!(returned, vec![vec![0], vec![1], vec![2]]);
        assert_eq!(*rec.statuses.lock().unwrap(), vec![Success::Success; 4]);
    }
}
