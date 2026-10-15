//! # Serialize ports for a project-defined port type
//!
//! What a project's FPP `port` gets from the autocoder under
//! `FW_PORT_SERIALIZATION`, declared from OUTSIDE `fprime-comp` with
//! [`serializable_port!`] (so the orphan rules are exercised the way a
//! deployment crate meets them). The serialized form is checked against
//! literal bytes of the C++ `<Port>PortSerializer::serializePortArgs`
//! layout: arguments in declaration order, big-endian, strings and buffers
//! with a `u16` length, structs member by member, enums at their
//! representation width, `bool` as `0xFF`/`0x00`.

use fprime_comp::{
    OutputPort, PortRef, SerializablePort, SerializePort, TypedCallAdapter, serializable_port,
};
use fprime_config::FwIndexType;
use fprime_fw::{
    ExtBuf, FwString, LinearBuffer, SerBuf, SerBufAny, SerializeStatus, fpp_enum, fpp_struct,
};
use std::sync::{Arc, Mutex};

fpp_enum! {
    /// A project enum (`enum Mode : U8`).
    pub enum Mode : u8 {
        /// Idle.
        Idle = 0,
        /// Running.
        Running = 3,
    }
    default Idle
}

fpp_struct! {
    /// A project struct.
    #[derive(Clone, Copy, Eq)]
    pub struct Point {
        /// X.
        x: u32,
        /// Y.
        y: i16,
    }
}

type Label = FwString<16>;

/// `port Value(value: U32, ref point: Point, ref label: string size 16,
/// mode: Mode, ok: bool)` — `ref` non-buffer arguments are `&mut`.
pub trait ValuePort: Send + Sync {
    fn invoke(
        &self,
        port_num: FwIndexType,
        value: u32,
        point: &Point,
        label: &mut Label,
        mode: Mode,
        ok: bool,
    );
}

serializable_port! {
    port: ValuePort;
    args {
        val value: u32,
        ref point: Point,
        mut label: Label,
        val mode: Mode,
        val ok: bool,
    }
}

/// A port with no arguments (FPP `port Signal`).
pub trait SignalPort: Send + Sync {
    fn invoke(&self, port_num: FwIndexType);
}

serializable_port! {
    port: SignalPort;
    args {}
}

type Call = (FwIndexType, u32, Point, String, Mode, bool);

#[derive(Default)]
struct Consumer {
    calls: Mutex<Vec<Call>>,
}

impl ValuePort for Consumer {
    fn invoke(
        &self,
        port_num: FwIndexType,
        value: u32,
        point: &Point,
        label: &mut Label,
        mode: Mode,
        ok: bool,
    ) {
        self.calls.lock().unwrap().push((
            port_num,
            value,
            *point,
            label.as_str().unwrap().to_string(),
            mode,
            ok,
        ));
        label.set("callee wrote this");
    }
}

impl SignalPort for Consumer {
    fn invoke(&self, port_num: FwIndexType) {
        self.calls.lock().unwrap().push((
            port_num,
            0,
            Point::default(),
            String::new(),
            Mode::Idle,
            false,
        ));
    }
}

/// A serial input that records the bytes of each call.
#[derive(Default)]
struct Wire {
    calls: Mutex<Vec<(FwIndexType, Vec<u8>)>>,
}

impl SerializePort for Wire {
    fn invoke_serial(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) -> SerializeStatus {
        self.calls
            .lock()
            .unwrap()
            .push((port_num, buffer.as_slice().to_vec()));
        SerializeStatus::Ok
    }
}

const VALUE_BYTES: [u8; 20] = [
    0xDE, 0xAD, 0xBE, 0xEF, // value: U32
    0x00, 0x00, 0x00, 0x05, 0xFF, 0xFE, // point: {x: U32 = 5, y: I16 = -2}
    0x00, 0x05, b'h', b'e', b'l', b'l', b'o', // label: U16 length + bytes
    0x03, // mode: U8 = Running
    0xFF, // ok: true
    0x00, // (not part of the call — trimmed below)
];

#[test]
fn capacity_is_the_sum_of_argument_sizes() {
    // U32 + Point(4 + 2) + string(2 + 16) + U8 + bool.
    assert_eq!(
        <dyn ValuePort as SerializablePort>::CAPACITY,
        4 + 6 + 18 + 1 + 1
    );
    assert_eq!(<dyn SignalPort as SerializablePort>::CAPACITY, 0);
}

#[test]
fn typed_call_serializes_like_the_generated_port() {
    let wire = Arc::new(Wire::default());
    let out: OutputPort<dyn ValuePort> = OutputPort::new();
    out.connect_serial(PortRef::new(wire.clone() as Arc<dyn SerializePort>, 6));
    let mut label = Label::from("hello");
    let p = out.get();
    p.target.invoke(
        p.port_num,
        0xDEAD_BEEF,
        &Point::new(5, -2),
        &mut label,
        Mode::Running,
        true,
    );
    let calls = wire.calls.lock().unwrap();
    assert_eq!(calls.len(), 1);
    assert_eq!(calls[0].0, 6);
    assert_eq!(calls[0].1, &VALUE_BYTES[..19]);
}

#[test]
fn serialized_call_replays_on_the_typed_port() {
    let consumer = Arc::new(Consumer::default());
    let out: OutputPort<dyn SerializePort> = OutputPort::new();
    out.connect_typed(PortRef::new(consumer.clone() as Arc<dyn ValuePort>, 1));
    let mut storage = VALUE_BYTES;
    let mut bytes = ExtBuf::with_len(&mut storage, 19);
    let p = out.get();
    assert_eq!(
        p.target.invoke_serial(p.port_num, &mut bytes),
        SerializeStatus::Ok
    );
    assert_eq!(
        *consumer.calls.lock().unwrap(),
        vec![(
            1,
            0xDEAD_BEEF,
            Point::new(5, -2),
            "hello".to_string(),
            Mode::Running,
            true
        )]
    );
}

#[test]
fn round_trip_is_copy_in_only() {
    // typed -> serial -> typed, as through a hub pair.
    let consumer = Arc::new(Consumer::default());
    let replay = Arc::new(TypedCallAdapter::new(consumer.clone() as Arc<dyn ValuePort>));
    let out: OutputPort<dyn ValuePort> = OutputPort::new();
    out.connect_serial(PortRef::new(replay as Arc<dyn SerializePort>, 2));
    let mut label = Label::from("caller");
    let p = out.get();
    p.target.invoke(
        p.port_num,
        1,
        &Point::new(1, 1),
        &mut label,
        Mode::Idle,
        false,
    );
    // The callee's write to its `ref` argument does not come back.
    assert_eq!(label, "caller");
    assert_eq!(consumer.calls.lock().unwrap()[0].3, "caller");
}

#[test]
fn invalid_enum_or_bool_is_rejected_before_invoking() {
    let consumer = Arc::new(Consumer::default());
    let adapter = TypedCallAdapter::new(consumer.clone() as Arc<dyn ValuePort>);
    let mut bad_mode = VALUE_BYTES;
    bad_mode[17] = 0x01; // Mode has no 1
    let mut bytes = ExtBuf::with_len(&mut bad_mode, 19);
    assert_eq!(
        adapter.invoke_serial(0, &mut bytes),
        SerializeStatus::DeserFormatError
    );
    let mut bad_bool = VALUE_BYTES;
    bad_bool[18] = 0x01; // strict bool decode
    let mut bytes = ExtBuf::with_len(&mut bad_bool, 19);
    assert_eq!(
        adapter.invoke_serial(0, &mut bytes),
        SerializeStatus::DeserFormatError
    );
    assert!(consumer.calls.lock().unwrap().is_empty());
}

#[test]
fn argumentless_port_sends_an_empty_call() {
    let wire = Arc::new(Wire::default());
    let out: OutputPort<dyn SignalPort> = OutputPort::new();
    out.connect_serial(PortRef::new(wire.clone() as Arc<dyn SerializePort>, 0));
    let p = out.get();
    p.target.invoke(p.port_num);
    assert_eq!(*wire.calls.lock().unwrap(), vec![(0, Vec::new())]);

    let consumer = Arc::new(Consumer::default());
    let adapter = TypedCallAdapter::new(consumer.clone() as Arc<dyn SignalPort>);
    let mut empty = LinearBuffer::<1>::new();
    assert_eq!(adapter.invoke_serial(3, &mut empty), SerializeStatus::Ok);
    assert_eq!(consumer.calls.lock().unwrap().len(), 1);
}
