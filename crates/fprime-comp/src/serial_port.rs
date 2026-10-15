//! Serialize ports — port of `Fw::InputSerializePort` /
//! `Fw::OutputSerializePort` and the `FW_PORT_SERIALIZATION` half of the
//! FPP-generated typed ports (`Fw/Port/*SerializePort.{hpp,cpp}`,
//! `Fw/Port/{Input,Output}PortBase`; analysis: `docs/cpp-analysis/fw-comp.md`
//! "Fw::InputSerializePort / OutputSerializePort").
//!
//! A *serial* port (FPP `serial`) carries a port call as bytes: the
//! arguments serialized in declaration order, big-endian, exactly as the
//! generated `<Port>PortSerializer::serializePortArgs` writes them. That is
//! what lets `Svc::GenericHub` tunnel any typed call to another deployment
//! and replay it there.
//!
//! The C++ model has three connection shapes, all reproduced here:
//!
//! | C++ | Rust |
//! |-----|------|
//! | serial output → `InputSerializePort` (`set_<p>_OutputPort(n, InputSerializePort*)` on an `OutputSerializePort`) | `OutputPort<dyn SerializePort>::connect_to` (plain connection) |
//! | typed output → `InputSerializePort` (`Output<P>Port::registerSerialPort`; `invoke` serializes the args and calls `invokeSerial`) | [`OutputPort::connect_serial`] — wraps the serial target in a [`SerialCallAdapter`] that implements the typed port trait |
//! | serial output → typed input (`OutputSerializePort::invokeSerial` → `Input<P>Port::invokeSerial`, which deserializes and calls the handler) | [`OutputPort::connect_typed`] — wraps the typed target in a [`TypedCallAdapter`] that implements [`SerializePort`] |
//!
//! A port trait opts in with [`serializable_port!`](crate::serializable_port),
//! which generates both directions from one argument list (so they cannot
//! drift); the standard framework ports that C++ can serialize are declared
//! at the bottom of this module. Like the C++ `ref` parameters, `&mut`
//! arguments are copy-in only across a serialized call.
//!
//! Deliberate deviations from C++:
//!
//! - The pass-through case returns the status of the serial handler, which
//!   for a component's `serial` input is always `Ok` — C++
//!   `InputSerializePort::invokeSerial` hard-codes `FW_SERIALIZE_OK`; a
//!   Rust serial handler returns `SerializeStatus::Ok` to match.
//! - Ports with a return value and ports carrying an owned `Fw::Buffer`
//!   simply do not implement [`SerializablePort`], so connecting them to a
//!   serial port is a compile error. C++ compiles both and then asserts
//!   (`invokeSerial` is `FW_ASSERT(false)` for return-value ports) or
//!   serializes a raw pointer (`Fw::Buffer`), which the GenericHub docs
//!   forbid.
//! - `Fw::ObjRegistry` is not needed: C++ serial ports never consult it.

use crate::port::{
    CmdPort, CmdRegPort, CmdResponsePort, ComPort, CyclePort, FatalEventPort, LogPort, LogTextPort,
    OutputPort, PingPort, PortRef, PrmSetPort, SchedPort, SuccessConditionPort, TimePort, TlmPort,
    WatchDogPort,
};
use fprime_config::{FwChanIdType, FwEventIdType, FwIndexType, FwOpcodeType, FwPrmIdType};
use fprime_fw::sync::Arc;
use fprime_fw::{
    CmdArgBuffer, CmdResponse, ComBuffer, LogBuffer, LogSeverity, ParamBuffer, SerBufAny,
    SerializeStatus, Success, TextLogString, Time, TlmBuffer,
};
use fprime_os::RawTime;

/// The FPP `serial` port type: a port call carried as bytes (C++
/// `Fw::InputPortBase::invokeSerial(LinearBufferBase&)`).
///
/// `buffer` holds the serialized arguments with its read cursor at the
/// start. A component's `serial` input implements this directly and returns
/// [`SerializeStatus::Ok`] (C++ `InputSerializePort::invokeSerial` always
/// does); a typed input reached through a [`TypedCallAdapter`] returns its
/// argument-deserialization status.
pub trait SerializePort: Send + Sync {
    /// Invoke the port with a serialized call.
    fn invoke_serial(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) -> SerializeStatus;
}

/// A port trait whose calls can be serialized — implemented for the trait
/// object `dyn XPort` by [`serializable_port!`](crate::serializable_port).
///
/// This is the per-port code the FPP autocoder generates under
/// `FW_PORT_SERIALIZATION`: `<Port>PortBuffer::CAPACITY`,
/// `Input<Port>Port::invokeSerial`, and the serializing branch of
/// `Output<Port>Port::invoke`.
pub trait SerializablePort: Send + Sync {
    /// Worst-case serialized argument size (C++ `<Port>PortBuffer::CAPACITY`).
    const CAPACITY: usize;

    /// Deserialize the arguments from `buffer` and invoke `target` (C++
    /// `Input<Port>Port::invokeSerial`). Returns the first failing
    /// deserialization status without invoking.
    fn invoke_serial_on(
        target: &Self,
        port_num: FwIndexType,
        buffer: &mut dyn SerBufAny,
    ) -> SerializeStatus;

    /// Wrap a serial target as this typed port (C++
    /// `Output<Port>Port::registerSerialPort`).
    fn serial_call_adapter(target: Arc<dyn SerializePort>) -> Arc<Self>;
}

/// Typed → serial adapter: implements every serializable port trait by
/// serializing the arguments and calling the wrapped [`SerializePort`].
/// Built by [`OutputPort::connect_serial`].
pub struct SerialCallAdapter {
    target: Arc<dyn SerializePort>,
}

impl SerialCallAdapter {
    /// Wrap a serial target.
    pub fn new(target: Arc<dyn SerializePort>) -> Self {
        Self { target }
    }

    /// The wrapped serial target.
    pub fn target(&self) -> &dyn SerializePort {
        &*self.target
    }
}

/// Serial → typed adapter: implements [`SerializePort`] by deserializing the
/// arguments and invoking the wrapped typed port (C++
/// `Input<Port>Port::invokeSerial`). Built by [`OutputPort::connect_typed`].
pub struct TypedCallAdapter<P: ?Sized> {
    target: Arc<P>,
}

impl<P: ?Sized + SerializablePort> TypedCallAdapter<P> {
    /// Wrap a typed target.
    pub fn new(target: Arc<P>) -> Self {
        Self { target }
    }
}

impl<P: ?Sized + SerializablePort> SerializePort for TypedCallAdapter<P> {
    fn invoke_serial(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) -> SerializeStatus {
        P::invoke_serial_on(&self.target, port_num, buffer)
    }
}

impl<P: ?Sized + SerializablePort> OutputPort<P> {
    /// Connect this typed output port to a serial input port (C++
    /// `set_<port>_OutputPort(portNum, Fw::InputSerializePort*)` →
    /// `registerSerialPort`): each call is serialized and delivered as bytes.
    pub fn connect_serial(&self, serial: PortRef<dyn SerializePort>) {
        self.connect(P::serial_call_adapter(serial.target), serial.port_num);
    }
}

impl OutputPort<dyn SerializePort> {
    /// Connect this serial output port to a typed input port (C++
    /// `set_<port>_OutputPort(portNum, Fw::InputPortBase*)` on an
    /// `OutputSerializePort`): each serialized call is deserialized and
    /// replayed on the typed port.
    pub fn connect_typed<P: ?Sized + SerializablePort + 'static>(&self, typed: PortRef<P>) {
        self.connect(
            Arc::new(TypedCallAdapter::new(typed.target)),
            typed.port_num,
        );
    }
}

// ---------------------------------------------------------------------------
// The standard framework ports C++ can serialize. Not here: `Fw.BufferSend`
// and `Svc.ComDataWithContext` (carry an `Fw::Buffer`), `Fw.BufferGet` and
// `Fw.PrmGet` (return values).
// ---------------------------------------------------------------------------

crate::serializable_port! {
    port: SchedPort;
    args { val context: u32 }
}

crate::serializable_port! {
    port: CyclePort;
    args { ref cycle_start: RawTime }
}

crate::serializable_port! {
    port: PingPort;
    args { val key: u32 }
}

crate::serializable_port! {
    port: WatchDogPort;
    args { val code: u32 }
}

crate::serializable_port! {
    port: CmdPort;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, buf args: CmdArgBuffer }
}

crate::serializable_port! {
    port: CmdRegPort;
    args { val op_code: FwOpcodeType }
}

crate::serializable_port! {
    port: CmdResponsePort;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, val response: CmdResponse }
}

crate::serializable_port! {
    port: LogPort;
    args {
        val id: FwEventIdType,
        mut time_tag: Time,
        val severity: LogSeverity,
        buf args: LogBuffer,
    }
}

crate::serializable_port! {
    port: LogTextPort;
    args {
        val id: FwEventIdType,
        mut time_tag: Time,
        val severity: LogSeverity,
        mut text: TextLogString,
    }
}

crate::serializable_port! {
    port: TlmPort;
    args { val id: FwChanIdType, mut time_tag: Time, buf val: TlmBuffer }
}

crate::serializable_port! {
    port: TimePort;
    args { mut time: Time }
}

crate::serializable_port! {
    port: ComPort;
    args { buf data: ComBuffer, val context: u32 }
}

crate::serializable_port! {
    port: SuccessConditionPort;
    args { mut condition: Success }
}

crate::serializable_port! {
    port: PrmSetPort;
    args { val id: FwPrmIdType, buf val: ParamBuffer }
}

crate::serializable_port! {
    port: FatalEventPort;
    args { val id: FwEventIdType }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::{LinearBuffer, SerBuf, TimeBase};
    use std::sync::Mutex;
    use std::vec;
    use std::vec::Vec;

    /// A component `serial` input: records each call's bytes.
    #[derive(Default)]
    struct SerialSink {
        calls: Mutex<Vec<(FwIndexType, Vec<u8>)>>,
    }

    impl SerializePort for SerialSink {
        fn invoke_serial(
            &self,
            port_num: FwIndexType,
            buffer: &mut dyn SerBufAny,
        ) -> SerializeStatus {
            let bytes = buffer.remaining_slice().to_vec();
            self.calls.lock().unwrap().push((port_num, bytes));
            SerializeStatus::Ok
        }
    }

    type LogCall = (FwIndexType, FwEventIdType, Time, LogSeverity, Vec<u8>);

    #[derive(Default)]
    struct LogSink {
        calls: Mutex<Vec<LogCall>>,
    }

    impl LogPort for LogSink {
        fn invoke(
            &self,
            port_num: FwIndexType,
            id: FwEventIdType,
            time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.calls.lock().unwrap().push((
                port_num,
                id,
                *time_tag,
                severity,
                args.as_slice().to_vec(),
            ));
            // Copy-in only: this write must not reach the caller.
            time_tag.set(9, 9);
            args.reset_ser();
        }
    }

    #[derive(Default)]
    struct PingSink {
        calls: Mutex<Vec<(FwIndexType, u32)>>,
    }

    impl PingPort for PingSink {
        fn invoke(&self, port_num: FwIndexType, key: u32) {
            self.calls.lock().unwrap().push((port_num, key));
        }
    }

    #[test]
    fn capacities_match_generated_port_buffers() {
        // C++ <Port>PortBuffer::CAPACITY = sum of the argument SERIALIZED_SIZEs.
        assert_eq!(<dyn SchedPort as SerializablePort>::CAPACITY, 4);
        assert_eq!(<dyn CyclePort as SerializablePort>::CAPACITY, 8);
        // FwEventIdType + Time + LogSeverity + LogBuffer(506 + 2).
        assert_eq!(
            <dyn LogPort as SerializablePort>::CAPACITY,
            4 + 11 + 1 + 508
        );
        assert_eq!(<dyn TlmPort as SerializablePort>::CAPACITY, 4 + 11 + 508);
        assert_eq!(<dyn ComPort as SerializablePort>::CAPACITY, 514 + 4);
        assert_eq!(<dyn CmdResponsePort as SerializablePort>::CAPACITY, 9);
        assert_eq!(
            <dyn LogTextPort as SerializablePort>::CAPACITY,
            4 + 11 + 1 + 258
        );
    }

    #[test]
    fn typed_call_serializes_arguments_in_declaration_order() {
        let sink = Arc::new(SerialSink::default());
        let out: OutputPort<dyn LogPort> = OutputPort::new();
        out.connect_serial(PortRef::new(sink.clone() as Arc<dyn SerializePort>, 3));

        let mut time = Time::new(TimeBase::TbWorkstationTime, 1, 0x0102_0304, 0x0005_0607);
        let mut args = LogBuffer::new();
        assert!(args.serialize_u8_be(0xAB).is_ok());
        let p = out.get();
        p.target.invoke(
            p.port_num,
            0x1122_3344,
            &mut time,
            LogSeverity::WarningHi,
            &mut args,
        );

        let calls = sink.calls.lock().unwrap();
        assert_eq!(calls.len(), 1);
        assert_eq!(calls[0].0, 3);
        assert_eq!(
            calls[0].1,
            vec![
                0x11, 0x22, 0x33, 0x44, // id
                0x00, 0x02, 0x01, 0x01, 0x02, 0x03, 0x04, 0x00, 0x05, 0x06, 0x07, // time
                0x02, // severity WARNING_HI
                0x00, 0x01, 0xAB, // LogBuffer: u16 size + bytes
            ]
        );
    }

    #[test]
    fn serial_call_replays_on_typed_port() {
        let sink = Arc::new(LogSink::default());
        let out: OutputPort<dyn SerializePort> = OutputPort::new();
        out.connect_typed(PortRef::new(sink.clone() as Arc<dyn LogPort>, 2));

        let mut bytes = LinearBuffer::<64>::new();
        let wire = [
            0x00, 0x00, 0x00, 0x07, // id
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x06, // time
            0x05, // ACTIVITY_HI
            0x00, 0x02, 0xDE, 0xAD, // args
        ];
        assert!(bytes.set_buff(&wire).is_ok());
        let p = out.get();
        assert_eq!(
            p.target.invoke_serial(p.port_num, &mut bytes),
            SerializeStatus::Ok
        );
        let calls = sink.calls.lock().unwrap();
        assert_eq!(calls.len(), 1);
        let (port, id, time, severity, args) = &calls[0];
        assert_eq!(*port, 2);
        assert_eq!(*id, 7);
        assert_eq!(time.get_seconds(), 5);
        assert_eq!(time.get_useconds(), 6);
        assert_eq!(*severity, LogSeverity::ActivityHi);
        assert_eq!(args, &vec![0xDE, 0xAD]);
    }

    #[test]
    fn malformed_serial_call_returns_status_without_invoking() {
        let sink = Arc::new(PingSink::default());
        let adapter = TypedCallAdapter::new(sink.clone() as Arc<dyn PingPort>);
        // Two bytes cannot hold a u32 key.
        let mut bytes = LinearBuffer::<8>::new();
        assert!(bytes.set_buff(&[0x00, 0x01]).is_ok());
        assert_eq!(
            adapter.invoke_serial(0, &mut bytes),
            SerializeStatus::DeserSizeMismatch
        );
        // An empty buffer is DeserBufferEmpty (C++ LinearBufferBase parity).
        let mut empty = LinearBuffer::<8>::new();
        assert_eq!(
            adapter.invoke_serial(0, &mut empty),
            SerializeStatus::DeserBufferEmpty
        );
        // An invalid enum value is a format error.
        let resp = Arc::new(RespSink::default());
        let adapter = TypedCallAdapter::new(resp.clone() as Arc<dyn CmdResponsePort>);
        let mut bytes = LinearBuffer::<16>::new();
        assert!(bytes.set_buff(&[0, 0, 0, 1, 0, 0, 0, 2, 0x09]).is_ok());
        assert_eq!(
            adapter.invoke_serial(0, &mut bytes),
            SerializeStatus::DeserFormatError
        );
        assert!(sink.calls.lock().unwrap().is_empty());
        assert!(resp.calls.lock().unwrap().is_empty());
    }

    #[derive(Default)]
    struct RespSink {
        calls: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
    }

    impl CmdResponsePort for RespSink {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            op_code: FwOpcodeType,
            cmd_seq: u32,
            response: CmdResponse,
        ) {
            self.calls
                .lock()
                .unwrap()
                .push((op_code, cmd_seq, response));
        }
    }

    /// A serial "wire" that replays every call on a serial output port — the
    /// shape of a hub pair, minus the transport.
    struct Loopback {
        out: OutputPort<dyn SerializePort>,
    }

    impl SerializePort for Loopback {
        fn invoke_serial(
            &self,
            port_num: FwIndexType,
            buffer: &mut dyn SerBufAny,
        ) -> SerializeStatus {
            let _ = port_num;
            let p = self.out.get();
            p.target.invoke_serial(p.port_num, buffer)
        }
    }

    #[test]
    fn typed_to_serial_to_typed_round_trip() {
        let sink = Arc::new(PingSink::default());
        let wire = Arc::new(Loopback {
            out: OutputPort::new(),
        });
        wire.out
            .connect_typed(PortRef::new(sink.clone() as Arc<dyn PingPort>, 4));
        let out: OutputPort<dyn PingPort> = OutputPort::new();
        out.connect_serial(PortRef::new(wire as Arc<dyn SerializePort>, 0));
        let p = out.get();
        p.target.invoke(p.port_num, 0xCAFE_F00D);
        assert_eq!(*sink.calls.lock().unwrap(), vec![(4, 0xCAFE_F00D)]);

        // `ref` arguments are copy-in only: the callee's writes to its
        // time tag and argument buffer never reach the caller.
        let log_sink = Arc::new(LogSink::default());
        let wire = Arc::new(Loopback {
            out: OutputPort::new(),
        });
        wire.out
            .connect_typed(PortRef::new(log_sink.clone() as Arc<dyn LogPort>, 0));
        let log_out: OutputPort<dyn LogPort> = OutputPort::new();
        log_out.connect_serial(PortRef::new(wire as Arc<dyn SerializePort>, 0));
        let mut time = Time::new(TimeBase::TbProcTime, 0, 1, 2);
        let mut args = LogBuffer::new();
        assert!(args.serialize_u8_be(0x42).is_ok());
        let p = log_out.get();
        p.target
            .invoke(p.port_num, 5, &mut time, LogSeverity::Diagnostic, &mut args);
        assert_eq!(log_sink.calls.lock().unwrap().len(), 1);
        assert_eq!((time.get_seconds(), time.get_useconds()), (1, 2));
        assert_eq!(args.as_slice(), &[0x42]);
    }

    #[test]
    fn serial_to_serial_is_a_plain_connection() {
        let sink = Arc::new(SerialSink::default());
        let out: OutputPort<dyn SerializePort> = OutputPort::new();
        out.connect(sink.clone(), 1);
        let mut bytes = LinearBuffer::<4>::new();
        assert!(bytes.set_buff(&[1, 2, 3]).is_ok());
        let p = out.get();
        assert!(p.target.invoke_serial(p.port_num, &mut bytes).is_ok());
        assert_eq!(*sink.calls.lock().unwrap(), vec![(1, vec![1, 2, 3])]);
    }
}
