//! Port of the F Prime port model (`Fw/Port/*`, FPP-generated typed ports;
//! analysis: `docs/cpp-analysis/fw-comp.md`).
//!
//! Each F Prime port *type* is a Rust trait with one `invoke` method
//! matching the FPP signature (`ref` args become `&mut`). An *input port* of
//! a component is a small adapter struct holding `Arc<TheComponent>` and
//! implementing the port trait (the hand-written equivalent of the static
//! thunks the C++ autocoder generates); the component exposes it via a
//! factory method returning a [`PortRef`]. An *output port* is an
//! [`OutputPort`] field, connected exactly once during topology wiring.
//!
//! Buffer-carrying comms ports: C++ passes `ref Fw::Buffer` (a view) and
//! pairs every `dataOut` with a `dataReturnOut` return path. Rust `Buffer`
//! is owned, so these traits take the `Buffer` **by move**, and the return
//! path travels via the paired return ports, keeping the C++ call-graph
//! shape (see the note in ARCHITECTURE.md).

use fprime_config::{
    FwChanIdType, FwEventIdType, FwIndexType, FwOpcodeType, FwPrmIdType, FwSizeType,
};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, ComBuffer, FrameContext, LogBuffer, LogSeverity,
    ParamBuffer, ParamValid, Success, TextLogString, Time, TlmBuffer, fw_assert,
};
use fprime_os::RawTime;
use std::sync::{Arc, OnceLock};

/// A connection endpoint: the target port object plus the port number the
/// target knows itself by (C++ `InputPortBase::m_portNum`).
pub struct PortRef<P: ?Sized> {
    /// The input-port adapter (or component) implementing the port trait.
    pub target: Arc<P>,
    /// Port number passed as the first `invoke` argument.
    pub port_num: FwIndexType,
}

impl<P: ?Sized> PortRef<P> {
    /// Convenience constructor.
    pub fn new(target: Arc<P>, port_num: FwIndexType) -> Self {
        Self { target, port_num }
    }
}

impl<P: ?Sized> Clone for PortRef<P> {
    fn clone(&self) -> Self {
        Self {
            target: Arc::clone(&self.target),
            port_num: self.port_num,
        }
    }
}

impl<P: ?Sized> std::fmt::Debug for PortRef<P> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PortRef")
            .field("port_num", &self.port_num)
            .finish_non_exhaustive()
    }
}

/// A typed output port (C++ generated `OutputXPort` + `m_port` pointer).
///
/// Connected exactly once, before tasks start; the `OnceLock` makes
/// post-wiring invocation lock-free. A second `connect` is `fw_assert`
/// (the C++ topology would silently overwrite — the stricter behavior only
/// catches wiring bugs, it cannot change a valid topology).
pub struct OutputPort<P: ?Sized> {
    conn: OnceLock<PortRef<P>>,
}

impl<P: ?Sized> std::fmt::Debug for OutputPort<P> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OutputPort")
            .field("connected", &self.is_connected())
            .field("port_num", &self.try_get().map(|p| p.port_num))
            .finish()
    }
}

impl<P: ?Sized> OutputPort<P> {
    /// New, unconnected output port.
    pub const fn new() -> Self {
        Self {
            conn: OnceLock::new(),
        }
    }

    /// Connect to a target input port. Asserts on a negative port number and
    /// on a second connection.
    pub fn connect(&self, target: Arc<P>, port_num: FwIndexType) {
        self.connect_to(PortRef::new(target, port_num));
    }

    /// Connect using a [`PortRef`] (what input-port factory methods return).
    pub fn connect_to(&self, port_ref: PortRef<P>) {
        // C++ parity: InputPortBase::setPortNum asserts portNum >= 0.
        fw_assert!(port_ref.port_num >= 0, port_ref.port_num);
        let ok = self.conn.set(port_ref).is_ok();
        fw_assert!(ok);
    }

    /// C++ `isConnected_<port>_OutputPort`.
    pub fn is_connected(&self) -> bool {
        self.conn.get().is_some()
    }

    /// The connection; `fw_assert` if unconnected (C++ parity: invoking an
    /// unconnected output port is FW_ASSERT).
    pub fn get(&self) -> &PortRef<P> {
        match self.conn.get() {
            Some(port_ref) => port_ref,
            None => {
                fw_assert!(false);
                // C++ parity: FW_ASSERT aborts here; a hook whose do_assert
                // returns cannot conjure a connection, so this stays fatal.
                unreachable!("output port invoked while unconnected")
            }
        }
    }

    /// The connection, or `None` if unconnected (for optional ports guarded
    /// by `isConnected` checks in the C++ code).
    pub fn try_get(&self) -> Option<&PortRef<P>> {
        self.conn.get()
    }
}

impl<P: ?Sized> Default for OutputPort<P> {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Standard framework port traits (Fw/*.fpp, Svc/Sched, Svc/Cycle, ...).
// All object-safe, Send + Sync so Arc<dyn P> can cross threads.
// ---------------------------------------------------------------------------

/// `Svc.Sched`: `port Sched(context: U32)`.
pub trait SchedPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, context: u32);
}

/// `Svc.Cycle`: `port Cycle(ref cycleStart: Os.RawTime)`.
pub trait CyclePort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, cycle_start: &RawTime);
}

/// `Svc.Ping`: `port Ping(key: U32)`.
pub trait PingPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, key: u32);
}

/// `Svc.WatchDog`: `port WatchDog(code: U32)`.
pub trait WatchDogPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, code: u32);
}

/// `Fw.Cmd`: `port Cmd(opCode: FwOpcodeType, cmdSeq: U32, ref args: CmdArgBuffer)`.
pub trait CmdPort: Send + Sync {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    );
}

/// `Fw.CmdReg`: `port CmdReg(opCode: FwOpcodeType)`.
pub trait CmdRegPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, op_code: FwOpcodeType);
}

/// `Fw.CmdResponse`: `port CmdResponse(opCode, cmdSeq, response)`.
pub trait CmdResponsePort: Send + Sync {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    );
}

/// `Fw.Log`: `port Log(id, ref timeTag, severity, ref args: LogBuffer)`.
pub trait LogPort: Send + Sync {
    fn invoke(
        &self,
        port_num: FwIndexType,
        id: FwEventIdType,
        time_tag: &mut Time,
        severity: LogSeverity,
        args: &mut LogBuffer,
    );
}

/// `Fw.LogText`: `port LogText(id, ref timeTag, severity, ref text)`.
pub trait LogTextPort: Send + Sync {
    fn invoke(
        &self,
        port_num: FwIndexType,
        id: FwEventIdType,
        time_tag: &mut Time,
        severity: LogSeverity,
        text: &mut TextLogString,
    );
}

/// `Fw.Tlm`: `port Tlm(id, ref timeTag, ref val: TlmBuffer)`.
pub trait TlmPort: Send + Sync {
    fn invoke(
        &self,
        port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    );
}

/// `Fw.Time`: `port Time(ref time: Fw.Time)`.
pub trait TimePort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, time: &mut Time);
}

/// `Fw.Com`: `port Com(ref data: ComBuffer, context: U32)`.
pub trait ComPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, data: &mut ComBuffer, context: u32);
}

/// `Fw.BufferSend`: `port BufferSend(ref fwBuffer: Fw.Buffer)` — the buffer
/// moves (ownership transfer replaces the C++ shared view).
pub trait BufferSendPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer);
}

/// `Fw.BufferGet`: `port BufferGet(size) -> Fw.Buffer` (sync-only; ports
/// with return values cannot be async).
pub trait BufferGetPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, size: FwSizeType) -> Buffer;
}

/// `Svc.ComDataWithContext`: `port ComDataWithContext(ref data: Fw.Buffer,
/// ref context: ComCfg.FrameContext)`. The buffer moves in; the return
/// travels via the paired `dataReturn` ports (see ARCHITECTURE.md note).
pub trait ComDataWithContextPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext);
}

/// `Fw.SuccessCondition`: `port SuccessCondition(ref condition: Fw.Success)`.
pub trait SuccessConditionPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, condition: &mut Success);
}

/// `Fw.PrmGet`: `port PrmGet(id, ref val) -> Fw.ParamValid` (sync-only).
pub trait PrmGetPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) -> ParamValid;
}

/// `Fw.PrmSet`: `port PrmSet(id, ref val)`.
pub trait PrmSetPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer);
}

/// `Svc.FatalEvent`: `port FatalEvent(id: FwEventIdType)`.
pub trait FatalEventPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType, id: FwEventIdType);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::panic::{AssertUnwindSafe, catch_unwind};
    use std::sync::Mutex;

    struct Recorder {
        calls: Mutex<Vec<(FwIndexType, u32)>>,
    }

    impl SchedPort for Recorder {
        fn invoke(&self, port_num: FwIndexType, context: u32) {
            self.calls.lock().unwrap().push((port_num, context));
        }
    }

    fn recorder() -> Arc<Recorder> {
        Arc::new(Recorder {
            calls: Mutex::new(Vec::new()),
        })
    }

    #[test]
    fn connect_then_invoke_passes_target_port_num() {
        let rec = recorder();
        let out: OutputPort<dyn SchedPort> = OutputPort::new();
        assert!(!out.is_connected());
        out.connect(rec.clone(), 3);
        assert!(out.is_connected());
        let p = out.get();
        p.target.invoke(p.port_num, 42);
        assert_eq!(*rec.calls.lock().unwrap(), vec![(3, 42)]);
    }

    #[test]
    fn second_connect_asserts() {
        let rec = recorder();
        let out: OutputPort<dyn SchedPort> = OutputPort::new();
        out.connect(rec.clone(), 0);
        let result = catch_unwind(AssertUnwindSafe(|| out.connect(rec.clone(), 1)));
        assert!(result.is_err());
        // The original connection survives.
        assert_eq!(out.get().port_num, 0);
    }

    #[test]
    fn negative_port_num_asserts() {
        let rec = recorder();
        let out: OutputPort<dyn SchedPort> = OutputPort::new();
        let result = catch_unwind(AssertUnwindSafe(|| out.connect(rec, -1)));
        assert!(result.is_err());
        assert!(!out.is_connected());
    }

    #[test]
    fn get_unconnected_asserts() {
        let out: OutputPort<dyn SchedPort> = OutputPort::new();
        let result = catch_unwind(AssertUnwindSafe(|| {
            let _ = out.get();
        }));
        assert!(result.is_err());
    }

    #[test]
    fn try_get_reports_connection_state() {
        let out: OutputPort<dyn SchedPort> = OutputPort::new();
        assert!(out.try_get().is_none());
        out.connect(recorder(), 7);
        assert_eq!(out.try_get().map(|p| p.port_num), Some(7));
    }

    #[test]
    fn connect_to_takes_a_port_ref() {
        let rec = recorder();
        let out: OutputPort<dyn SchedPort> = OutputPort::new();
        out.connect_to(PortRef::new(rec.clone(), 5));
        let p = out.get();
        p.target.invoke(p.port_num, 1);
        assert_eq!(*rec.calls.lock().unwrap(), vec![(5, 1)]);
    }
}
