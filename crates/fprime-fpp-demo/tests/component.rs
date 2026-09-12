//! The generated component and topology, driven end to end: wiring by the
//! generated topology, command dispatch (async/sync/guarded, format and
//! opcode errors), events (args, text, throttle), telemetry (always and on
//! change), parameters (load, set, save), async ports with escrowed
//! buffers, guarded ports with return values, internal ports, port-array
//! numbering, and data products with byte-exact records.

use fprime_fpp_demo::generated::Demo::{
    self, DemoTopology, SensorBase, SensorComponent, SensorHandlers,
};
use fprime_fpp_demo::ground::Ground;
use fprime_fpp_demo::sensor::Sensor;
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, Endianness, FwString, LogSeverity, ParamValid, SerBuf,
    Serialize, Time, TimeBase,
};
use std::sync::Arc;

const SENSOR: u32 = 0x1000;

fn build() -> (DemoTopology, Arc<Sensor>, Arc<Ground>) {
    let sensor = Sensor::new("sensor");
    let ground = Ground::new("ground");
    let top = DemoTopology {
        sensor: Arc::clone(&sensor),
        ground: Arc::clone(&ground),
    };
    top.set_id_bases();
    top.init();
    top.connect();
    top.reg_commands();
    (top, sensor, ground)
}

fn tick(ground: &Ground, context: u32) {
    ground.base().tick_out_out(0, context);
}

fn args(build: impl FnOnce(&mut CmdArgBuffer)) -> CmdArgBuffer {
    let mut a = CmdArgBuffer::new();
    build(&mut a);
    a
}

fn send(ground: &Ground, opcode: u32, seq: u32, mut a: CmdArgBuffer) {
    // The dispatcher's compCmdSend[0] is the one wired to the sensor.
    ground.base().cmd_out_out(0, SENSOR + opcode, seq, &mut a);
}

#[test]
fn dictionary_constants_and_registration() {
    assert_eq!(SensorBase::OPCODE_CONFIGURE, 0x10);
    assert_eq!(SensorBase::OPCODE_PING, 0x11);
    assert_eq!(SensorBase::OPCODE_RESET, 0x12);
    assert_eq!(SensorBase::OPCODE_GAIN_PARAM_SET, 0x13);
    assert_eq!(SensorBase::OPCODE_GAIN_PARAM_SAVE, 0x14);
    assert_eq!(SensorBase::OPCODE_THRESHOLD_PARAM_SET, 0x15);
    assert_eq!(SensorBase::OPCODE_THRESHOLD_PARAM_SAVE, 0x16);
    assert_eq!(SensorBase::EVENTID_CONFIGURED, 0);
    assert_eq!(SensorBase::EVENTID_OVERRUN, 1);
    assert_eq!(SensorBase::OVERRUN_THROTTLE, 2);
    assert_eq!(SensorBase::CHANID_VALUE, 0);
    assert_eq!(SensorBase::CHANID_LABEL, 2);
    assert_eq!(SensorBase::PARAMID_GAIN, 0);
    assert_eq!(SensorBase::PARAMID_THRESHOLD, 1);
    assert_eq!(SensorBase::CONTAINER_ID_SAMPLES, 0);
    assert_eq!(SensorBase::CONTAINER_PRIORITY_SAMPLES, 9);
    assert_eq!(SensorBase::RECORD_ID_SAMPLE, 0);
    assert_eq!(SensorBase::RECORD_ID_RAW, 1);
    // Message types number from 1 in the reference order: async ports,
    // commands, internal ports.
    assert_eq!(SensorBase::MSG_TYPE_DELIVER, 1);
    assert_eq!(SensorBase::MSG_TYPE_CMD_IN, 2);
    assert_eq!(SensorBase::MSG_TYPE_RECOMPUTE_INTERNAL, 3);
    // The command message is the largest: 6 + 4 + 4 + 2 + 506.
    assert_eq!(SensorBase::MSG_SIZE, 522);
    // id (4) + Reading (24)
    assert_eq!(SensorBase::SIZE_OF_SAMPLE_RECORD, 28);
    assert_eq!(SensorBase::size_of_raw_record(10), 16);

    let (_top, _sensor, ground) = build();
    let regs = ground.log.lock().unwrap().regs.clone();
    assert_eq!(
        regs,
        vec![
            SENSOR + 0x10,
            SENSOR + 0x11,
            SENSOR + 0x12,
            SENSOR + 0x13,
            SENSOR + 0x14,
            SENSOR + 0x15,
            SENSOR + 0x16
        ]
    );
}

#[test]
fn async_command_is_queued_until_the_tick_and_emits_events_and_telemetry() {
    let (_top, sensor, ground) = build();
    ground.log.lock().unwrap().time = Time::new(TimeBase::TbWorkstationTime, 0, 12, 34);
    let a = args(|a| {
        assert!(Demo::Mode::FAULTED.serialize_to(a, Endianness::Big).is_ok());
        assert!(a.serialize_f32_be(1.5).is_ok());
        assert!(
            FwString::<8>::from("abc")
                .serialize_to(a, Endianness::Big)
                .is_ok()
        );
    });
    send(&ground, 0x10, 7, a);
    // Nothing happened yet: the command sits in the queue.
    assert!(ground.log.lock().unwrap().responses.is_empty());
    assert_eq!(sensor.state().mode, Demo::Mode::default());

    tick(&ground, 3);
    let st = sensor.state();
    assert_eq!(st.mode, Demo::Mode::FAULTED);
    assert_eq!(st.gain, 1.5);
    assert_eq!(st.label.as_str(), Some("abc"));
    let log = ground.log.lock().unwrap();
    assert_eq!(log.responses, vec![(SENSOR + 0x10, 7, CmdResponse::Ok)]);
    // Configured(mode, gain): [07][1.5 f32].
    let mut expected = vec![7u8];
    expected.extend_from_slice(&1.5f32.to_be_bytes());
    assert_eq!(log.events[0], (SENSOR, LogSeverity::ActivityHi, expected));
    assert_eq!(
        log.texts[0],
        (SENSOR, "Configured FAULTED with gain 1.50".into())
    );
    // Labelled(label, reading): [u16 len][abc][Reading default].
    let (id, sev, bytes) = &log.events[1];
    assert_eq!((*id, *sev), (SENSOR + 2, LogSeverity::Diagnostic));
    assert_eq!(&bytes[..5], &[0, 3, b'a', b'b', b'c']);
    // Reading default: mode 1 + counts 8 + samples 4 + label (2 + 3) + valid 1.
    assert_eq!(bytes.len(), 5 + 19);
    assert!(log.texts[1].1.starts_with("abc -> Reading"));
    // Telemetry on the tick: Value (always), Mode and Label (on change).
    let ids: Vec<u32> = log.tlm.iter().map(|(id, _)| *id).collect();
    assert_eq!(ids, vec![SENSOR, SENSOR + 1, SENSOR + 2]);
    assert_eq!(log.tlm[0].1, (1.5f32 * 3.0).to_be_bytes());
    assert_eq!(log.tlm[1].1, vec![7]);
    assert_eq!(log.tlm[2].1, vec![0, 3, b'a', b'b', b'c']);
    // The report went out with the sequence number and the fan-out ports
    // were numbered 0 and 1 by the topology.
    assert_eq!(log.reports.len(), 1);
    assert_eq!(log.reports[0].0, 1);
    assert_eq!(log.reports[0].1.mode, Demo::Mode::FAULTED);
    assert_eq!(log.ticks, vec![0, 1]);
    drop(log);

    // A second tick with the same mode and label: only Value is written.
    tick(&ground, 4);
    let log = ground.log.lock().unwrap();
    let ids: Vec<u32> = log.tlm.iter().skip(3).map(|(id, _)| *id).collect();
    assert_eq!(ids, vec![SENSOR]);
}

#[test]
fn sync_and_guarded_commands_run_immediately_with_throttled_events() {
    let (_top, sensor, ground) = build();
    send(&ground, 0x11, 1, args(|_| {}));
    assert_eq!(sensor.state().pings, 1);
    for (i, count) in [5u32, 6, 7, 1].iter().enumerate() {
        send(
            &ground,
            0x12,
            10 + i as u32,
            args(|a| {
                assert!(a.serialize_u32_be(*count).is_ok());
            }),
        );
    }
    let st = sensor.state();
    assert_eq!(st.resets, vec![5, 6, 7, 1]);
    let log = ground.log.lock().unwrap();
    assert_eq!(log.responses.len(), 5);
    assert!(log.responses.iter().all(|r| r.2 == CmdResponse::Ok));
    // Overrun is throttled at 2: the third overrun is dropped.
    let overruns: Vec<&Vec<u8>> = log
        .events
        .iter()
        .filter(|(id, _, _)| *id == SENSOR + 1)
        .map(|(_, _, b)| b)
        .collect();
    assert_eq!(
        overruns,
        vec![&5u32.to_be_bytes().to_vec(), &6u32.to_be_bytes().to_vec()]
    );
    drop(log);
    sensor.base().log_warning_lo_overrun_throttle_clear();
    send(
        &ground,
        0x12,
        20,
        args(|a| {
            assert!(a.serialize_u32_be(9).is_ok());
        }),
    );
    let log = ground.log.lock().unwrap();
    assert_eq!(
        log.events
            .iter()
            .filter(|(id, _, _)| *id == SENSOR + 1)
            .count(),
        3
    );
}

#[test]
fn format_and_opcode_errors_are_answered_by_the_generated_dispatch() {
    let (_top, _sensor, ground) = build();
    // RESET with a missing argument.
    send(&ground, 0x12, 1, args(|_| {}));
    // RESET with trailing bytes.
    send(
        &ground,
        0x12,
        2,
        args(|a| {
            assert!(a.serialize_u32_be(1).is_ok());
            assert!(a.serialize_u8(0, Endianness::Big).is_ok());
        }),
    );
    // Unknown opcode.
    send(&ground, 0x77, 3, args(|_| {}));
    // Async command with a bad enum value is a FORMAT_ERROR at dispatch.
    send(
        &ground,
        0x10,
        4,
        args(|a| {
            assert!(a.serialize_u8(200, Endianness::Big).is_ok());
            assert!(a.serialize_f32_be(1.0).is_ok());
            assert!(
                FwString::<8>::from("x")
                    .serialize_to(a, Endianness::Big)
                    .is_ok()
            );
        }),
    );
    tick(&ground, 0);
    let log = ground.log.lock().unwrap();
    assert_eq!(
        log.responses,
        vec![
            (SENSOR + 0x12, 1, CmdResponse::FormatError),
            (SENSOR + 0x12, 2, CmdResponse::FormatError),
            (SENSOR + 0x77, 3, CmdResponse::InvalidOpcode),
            (SENSOR + 0x10, 4, CmdResponse::FormatError),
        ]
    );
}

#[test]
fn parameters_load_set_and_save() {
    let (top, sensor, ground) = build();
    ground
        .log
        .lock()
        .unwrap()
        .prm_values
        .insert(SENSOR, 3.5f32.to_be_bytes().to_vec());
    top.load_parameters();
    assert_eq!(sensor.base().param_get_gain(), (3.5, ParamValid::Valid));
    // Threshold has no stored value and no FPP default: invalid, zero.
    assert_eq!(
        sensor.base().param_get_threshold(),
        (0, ParamValid::Invalid)
    );
    assert!(sensor.state().loaded);

    // GAIN_PARAM_SET stages a new value (queued: takes effect on the tick).
    send(
        &ground,
        0x13,
        1,
        args(|a| {
            assert!(a.serialize_f32_be(9.25).is_ok());
        }),
    );
    assert_eq!(sensor.base().param_get_gain().0, 3.5);
    tick(&ground, 0);
    assert_eq!(sensor.base().param_get_gain(), (9.25, ParamValid::Valid));
    assert_eq!(sensor.state().param_updates, vec![0]);
    // GAIN_PARAM_SAVE pushes it to the database.
    send(&ground, 0x14, 2, args(|_| {}));
    tick(&ground, 0);
    let log = ground.log.lock().unwrap();
    assert_eq!(log.prm_sets, vec![(SENSOR, 9.25f32.to_be_bytes().to_vec())]);
    assert_eq!(
        log.responses,
        vec![
            (SENSOR + 0x13, 1, CmdResponse::Ok),
            (SENSOR + 0x14, 2, CmdResponse::Ok)
        ]
    );
}

#[test]
fn async_port_with_buffer_guarded_port_and_internal_port() {
    let (_top, sensor, ground) = build();
    let mut t = Time::new(TimeBase::TbWorkstationTime, 0, 5, 6);
    ground
        .base()
        .deliver_out_out(0, Buffer::allocate(11), &mut t);
    assert!(sensor.state().delivered.is_empty());
    SensorComponent::recompute_internal_interface_invoke(&*sensor, 2.5);
    tick(&ground, 0);
    let st = sensor.state();
    assert_eq!(st.delivered, vec![(11, t)]);
    assert_eq!(st.recomputed, vec![2.5]);

    let mut frame = Demo::Frame::default();
    let level = ground.base().measure_out_out(
        0,
        42,
        Demo::Mode::RECOVERING,
        &Demo::Reading::default(),
        &FwString::from("lbl"),
        &mut frame,
    );
    assert_eq!(level, Demo::Level::HIGH);
    assert_eq!(frame.seq, 42);
    assert_eq!(frame.reading.mode, Demo::Mode::RECOVERING);
    assert_eq!(sensor.state().measured, 1);
    assert!(sensor.base().is_connected_report_out(0));
    assert!(!sensor.base().is_connected_report_out(1));
}

#[test]
fn data_products_are_requested_filled_and_sent_with_exact_records() {
    let (_top, sensor, ground) = build();
    assert!(sensor.produce(3));
    let log = ground.log.lock().unwrap();
    assert_eq!(log.dp_sent.len(), 1);
    let (id, packet) = &log.dp_sent[0];
    assert_eq!(*id, SENSOR);
    // Header: descriptor, id, priority 9.
    assert_eq!(&packet[0..2], &0x0005u16.to_be_bytes());
    assert_eq!(&packet[2..6], &SENSOR.to_be_bytes());
    assert_eq!(&packet[6..10], &9u32.to_be_bytes());
    // Data region: [id base+0][Reading default (19 B)][id base+1][u16 3][0 1 2].
    let data_size = u16::from_be_bytes([packet[55], packet[56]]) as usize;
    assert_eq!(data_size, 4 + 19 + 4 + 2 + 3);
    let data = &packet[61..61 + data_size];
    assert_eq!(&data[0..4], &SENSOR.to_be_bytes());
    assert_eq!(&data[23..27], &(SENSOR + 1).to_be_bytes());
    assert_eq!(&data[27..32], &[0, 3, 0, 1, 2]);
    drop(log);
    ground.log.lock().unwrap().starve = true;
    assert!(!sensor.produce(1));
}
