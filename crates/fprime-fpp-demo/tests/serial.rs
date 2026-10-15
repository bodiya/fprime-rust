//! Serial ports, end to end: the generated `Relay` (sync, guarded and
//! queued `serial` inputs, a `serial` output array) wired by the generated
//! topology between typed ports of the ground stub. Covers typed -> serial
//! -> typed through a generated port type (`Demo.Report`) and a framework
//! one (`Fw.Com`), serial -> serial into a queued serial port, the queue
//! message layout, the overflow hook, and the deserialization status a
//! typed input answers when invoked serially.

use fprime_comp::SerializablePort;
use fprime_config::{FwIndexType, FwQueuePriorityType, FwSizeType};
use fprime_fpp_demo::generated::Demo::{self, DemoTopology, Reading, RelayBase, RelayComponent};
use fprime_fpp_demo::ground::Ground;
use fprime_fpp_demo::relay::Relay;
use fprime_fpp_demo::sensor::Sensor;
use fprime_fw::{
    ComBuffer, Endianness, FwString, LinearBuffer, SerBuf, SerBufAny, Serialize, SerializeStatus,
};
use fprime_os::queue::{BlockingType, Status};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::Arc;

fn build() -> (DemoTopology, Arc<Ground>, Arc<Relay>) {
    let ground = Ground::new("ground");
    let relay = Relay::new("relay");
    let top = DemoTopology {
        sensor: Sensor::new("sensor"),
        ground: Arc::clone(&ground),
        relay: Arc::clone(&relay),
    };
    top.set_id_bases();
    top.init();
    top.connect();
    (top, ground, relay)
}

fn reading() -> Reading {
    Reading {
        label: FwString::from("relay"),
        ..Default::default()
    }
}

/// A `Demo.Report` call serialized as the C++ `ReportPortBuffer` holds it.
fn report_bytes(seq: u32, reading: &Reading) -> Vec<u8> {
    let mut b = LinearBuffer::<64>::new();
    assert!(b.serialize_u32_be(seq).is_ok());
    assert!(reading.serialize_to(&mut b, Endianness::Big).is_ok());
    b.as_slice().to_vec()
}

#[test]
fn generated_ports_are_serializable_like_the_cpp_port_buffers() {
    // `serializable_port!` for every generated port without a return
    // value or an Fw.Buffer: seq (4) + Reading (24).
    assert_eq!(<dyn Demo::ReportPort as SerializablePort>::CAPACITY, 28);
    assert_eq!(<dyn Demo::TickPort as SerializablePort>::CAPACITY, 0);
    // (`Measure`, which returns a value, and `Deliver`, which carries an
    // Fw.Buffer, are not serializable.) The message size of a component
    // whose only async port is serial is the bare envelope (C++
    // `SERIALIZATION_SIZE` with an empty argument union).
    assert_eq!(RelayBase::MSG_TYPE_SERIAL_ASYNC, 1);
    assert_eq!(RelayBase::MSG_SIZE, 6);
}

#[test]
fn typed_to_serial_to_typed_through_a_sync_serial_port() {
    let (_top, ground, relay) = build();
    let r = reading();
    ground.base().relay_out_out(0, 7, &r);

    // The relay saw exactly the serialized call ...
    let log = relay.log.lock().unwrap();
    assert_eq!(log.forwarded.len(), 1);
    let (input, port, bytes, status) = &log.forwarded[0];
    assert_eq!((*input, *port), ("serialIn", 0));
    assert_eq!(bytes, &report_bytes(7, &r));
    assert_eq!(*status, SerializeStatus::Ok);
    // ... and the ground got the typed call back on relayIn[0].
    assert_eq!(ground.log.lock().unwrap().relayed, vec![(0, 7, r)]);
}

#[test]
fn serial_to_serial_into_a_queued_serial_port() {
    let (_top, ground, relay) = build();
    let r = reading();
    let mut call = LinearBuffer::<64>::new();
    assert!(call.set_buff(&report_bytes(9, &r)).is_ok());
    // ground.serialOut -> relay.serialIn[1] -> relay.serialOut[1]
    //   -> relay.serialAsync (queued)
    assert_eq!(
        ground.base().serial_out_out(0, &mut call),
        SerializeStatus::Ok
    );
    assert!(ground.log.lock().unwrap().relayed.is_empty());
    assert_eq!(relay.base().queued.queue().get_messages_available(), 1);

    // Dispatching runs the async handler: relay.serialOut[3] -> relayIn.
    relay.drain();
    assert_eq!(ground.log.lock().unwrap().relayed, vec![(0, 9, r.clone())]);
    let log = relay.log.lock().unwrap();
    let seen: Vec<(&str, FwIndexType, SerializeStatus)> = log
        .forwarded
        .iter()
        .map(|(i, p, _, s)| (*i, *p, *s))
        .collect();
    assert_eq!(
        seen,
        vec![
            ("serialIn", 1, SerializeStatus::Ok),
            ("serialAsync", 0, SerializeStatus::Ok)
        ]
    );
    assert_eq!(log.forwarded[1].2, report_bytes(9, &r));
}

#[test]
fn queued_serial_call_layout_and_message_size() {
    let (_top, ground, relay) = build();
    // The topology sizes serial calls for a com buffer: msg_size + port
    // number + message type.
    assert_eq!(
        relay.base().serial_msg.lock().unwrap().len(),
        fprime_config::FW_COM_BUFFER_MAX_SIZE + 2 + 4
    );
    let mut call = LinearBuffer::<8>::new();
    assert!(call.set_buff(&[0xA1, 0xB2, 0xC3]).is_ok());
    let _ = ground.base().serial_out_out(0, &mut call);

    // [msg type i32 = 1][port num i16 = 0][u16 size = 3][bytes], priority 4.
    let mut raw = vec![0u8; relay.base().serial_msg.lock().unwrap().len()];
    let mut size: FwSizeType = 0;
    let mut priority: FwQueuePriorityType = 0;
    let status = relay.base().queued.queue().receive(
        &mut raw,
        BlockingType::NonBlocking,
        &mut size,
        &mut priority,
    );
    assert_eq!(status, Status::OpOk);
    assert_eq!(priority, 4);
    assert_eq!(
        &raw[..size as usize],
        &[0, 0, 0, 1, 0, 0, 0, 3, 0xA1, 0xB2, 0xC3]
    );
}

#[test]
fn a_full_queue_calls_the_serial_overflow_hook() {
    let (_top, ground, relay) = build();
    for b in 1..=3u8 {
        let mut call = LinearBuffer::<4>::new();
        assert!(call.set_buff(&[b]).is_ok());
        // `invokeSerial` of a serial input answers OK even on overflow.
        assert_eq!(
            ground.base().serial_out_out(0, &mut call),
            SerializeStatus::Ok
        );
    }
    // Queue depth 2: the third call overflowed into the hook.
    assert_eq!(relay.log.lock().unwrap().overflowed, vec![(0, vec![3])]);
    relay.drain();
    let log = relay.log.lock().unwrap();
    let delivered: Vec<&Vec<u8>> = log
        .forwarded
        .iter()
        .filter(|f| f.0 == "serialAsync")
        .map(|f| &f.2)
        .collect();
    assert_eq!(delivered, vec![&vec![1u8], &vec![2u8]]);
}

#[test]
fn a_framework_port_type_crosses_a_guarded_serial_port() {
    let (_top, ground, relay) = build();
    let mut data = ComBuffer::new();
    assert!(data.set_buff(&[1, 2, 3, 4, 5]).is_ok());
    ground.base().com_relay_out_out(0, &mut data, 0xCAFE_F00D);

    // Fw.Com serializes as [u16 size][data][u32 context].
    let log = relay.log.lock().unwrap();
    assert_eq!(log.forwarded.len(), 1);
    assert_eq!(log.forwarded[0].0, "serialGuarded");
    assert_eq!(
        log.forwarded[0].2,
        vec![0, 5, 1, 2, 3, 4, 5, 0xCA, 0xFE, 0xF0, 0x0D]
    );
    assert_eq!(
        ground.log.lock().unwrap().relayed_com,
        vec![(0, vec![1, 2, 3, 4, 5], 0xCAFE_F00D)]
    );
}

#[test]
fn a_typed_input_invoked_serially_answers_the_deserialization_status() {
    let (_top, ground, relay) = build();
    let r = reading();
    let full = report_bytes(3, &r);

    // relay.serialOut[0] is wired to ground.relayIn through the Report
    // deserializer: short calls are refused with C++'s statuses and the
    // handler is not called.
    let mut empty = LinearBuffer::<64>::new();
    assert_eq!(
        relay.base().serial_out_out(0, &mut empty),
        SerializeStatus::DeserBufferEmpty
    );
    let mut short = LinearBuffer::<64>::new();
    assert!(short.set_buff(&full[..2]).is_ok());
    assert_eq!(
        relay.base().serial_out_out(0, &mut short),
        SerializeStatus::DeserSizeMismatch
    );
    let mut truncated = LinearBuffer::<64>::new();
    assert!(truncated.set_buff(&full[..10]).is_ok());
    assert_ne!(
        relay.base().serial_out_out(0, &mut truncated),
        SerializeStatus::Ok
    );
    assert!(ground.log.lock().unwrap().relayed.is_empty());

    let mut ok = LinearBuffer::<64>::new();
    assert!(ok.set_buff(&full).is_ok());
    assert_eq!(relay.base().serial_out_out(0, &mut ok), SerializeStatus::Ok);
    assert_eq!(ground.log.lock().unwrap().relayed, vec![(0, 3, r)]);
}

#[test]
fn serial_input_factories_and_output_helpers() {
    let (_top, _ground, relay) = build();
    for n in 0..4 {
        assert!(relay.base().is_connected_serial_out_out(n));
    }
    assert!(!relay.base().is_connected_serial_out_out(4));
    assert!(!relay.base().is_connected_serial_out_out(-1));
    let mut b = LinearBuffer::<1>::new();
    let result = catch_unwind(AssertUnwindSafe(|| relay.base().serial_out_out(4, &mut b)));
    assert!(result.is_err(), "out-of-range port number asserts");

    // A serial input port reference built by the generated factory: the
    // sync handler forwards, and the port answers OK.
    let port = relay.serial_in(0);
    let mut call = LinearBuffer::<64>::new();
    assert!(call.set_buff(&report_bytes(1, &reading())).is_ok());
    let dyn_call: &mut dyn SerBufAny = &mut call;
    assert_eq!(
        port.target.invoke_serial(port.port_num, dyn_call),
        SerializeStatus::Ok
    );
}

#[test]
fn an_unconnected_serial_output_asserts() {
    let relay = Relay::new("lonely");
    let mut b = LinearBuffer::<1>::new();
    let result = catch_unwind(AssertUnwindSafe(|| relay.base().serial_out_out(0, &mut b)));
    assert!(result.is_err());
}
