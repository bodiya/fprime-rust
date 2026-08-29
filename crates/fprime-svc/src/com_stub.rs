//! # ComStub — port of `Svc::ComStub` (passive com adapter)
//!
//! C++ sources: `Svc/ComStub/ComStub.{cpp,hpp,fpp}` and the
//! `Drv/ByteStreamDriverModel` / `Drv/Interfaces` port definitions.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (ComStub + ByteStreamDriver).
//!
//! Implements the `Svc.Com` adapter interface over a SYNCHRONOUS byte-stream
//! driver (`Drv.ByteStreamDriverClient`): bounded send retries
//! ([`RETRY_LIMIT`] = 10), the `m_reinitialize` handshake (the system
//! bootstrap — the first `comStatus SUCCESS` is emitted only on driver
//! `ready`, priming ComQueue), and the receive path forwarding driver
//! buffers upstream with a DEFAULT [`FrameContext`].
//!
//! ## Byte-stream port seam (documented)
//!
//! `fprime-svc` may not depend on `fprime-drv` (dependency DAG), so the
//! byte-stream port traits and [`ByteStreamStatus`] are defined PUBLICLY
//! here. `fprime-drv` defines its own structurally-identical traits; the
//! reference deployment glues the two with tiny adapter shims. The async
//! driver ports (`drvAsyncSendOut`/`drvAsyncSendReturnIn`) are not ported
//! in phase 1 (no async byte-stream driver exists yet).

use fprime_comp::{
    BufferSendPort, ComDataWithContextPort, OutputPort, PassiveBase, PortRef, SuccessConditionPort,
};
use fprime_config::FwIndexType;
use fprime_fw::{Buffer, FrameContext, Success, fw_assert, fw_log};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

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

/// Bounded synchronous-send retry limit (C++ `ComStub::RETRY_LIMIT`).
pub const RETRY_LIMIT: FwIndexType = 10;

/// `Svc::ComStub` — passive com adapter over a synchronous byte-stream
/// driver. No events, telemetry, or commands.
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
    /// C++ `m_reinitialize`: starts true; a driver `ready` emits the
    /// (re-)initialization SUCCESS and clears it. Relaxed atomic — the C++
    /// member is a plain bool touched from sync handlers.
    reinitialize: AtomicBool,
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
            reinitialize: AtomicBool::new(true),
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

    // -- Handlers ------------------------------------------------------------

    /// `dataIn` handler: synchronous send with bounded retry.
    fn data_in_handler(
        &self,
        _port_num: FwIndexType,
        mut send_buffer: Buffer,
        context: &FrameContext,
    ) {
        // A message should never get here while we need reinitialization
        // (C++ parity assert).
        fw_assert!(
            !self.reinitialize.load(Ordering::Relaxed) || !self.com_status_out.is_connected()
        );
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
}
