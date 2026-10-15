//! Telemetry packet sets: the packet tables `fpp-to-rust` generates from
//! the `Demo` topology's `telemetry packets` sets (C++
//! `<Topology>_<Set>TlmPacketsAc`), checked entry by entry, against the
//! ground dictionary, and end to end: a real `Svc::TlmPacketizer`
//! configured with them packetizes the generated sensor's telemetry
//! byte for byte.

use fprime_config::{FwChanIdType, FwEventIdType, FwIndexType};
use fprime_fpp::json::Json;
use fprime_fpp_demo::generated::Demo::{
    Demo_DemoPacketsTlmPackets as DemoPackets, Demo_MinimalTlmPackets as Minimal, Mode, SensorBase,
    SensorHandlers,
};
use fprime_fpp_demo::sensor::Sensor;
use fprime_fw::{ComBuffer, LogBuffer, LogSeverity, SerBuf, TextLogString, Time, TimeBase};
use fprime_os::queue::BlockingType;
use fprime_svc::tlm_packetizer::{
    IGNORE_OMIT_LIST, TlmPacketizer, TlmPacketizerChannelEntry as Entry, TlmPacketizerPacket,
};
use std::sync::{Arc, Mutex};

const SENSOR: FwChanIdType = 0x1000;
const VALUE: FwChanIdType = SENSOR + SensorBase::CHANID_VALUE;
const MODE: FwChanIdType = SENSOR + SensorBase::CHANID_MODE;
const LABEL: FwChanIdType = SENSOR + SensorBase::CHANID_LABEL;

fn summary(list: &[TlmPacketizerPacket<'_>]) -> Vec<(u16, FwChanIdType, Vec<Entry>)> {
    list.iter()
        .map(|p| (p.id, p.level, p.channels.to_vec()))
        .collect()
}

#[test]
fn packet_tables_follow_the_reference_writer() {
    // Packets are listed by name (`Name` before `Status`), channels in
    // declaration order, with global ids and maximum serialized sizes.
    assert_eq!(
        summary(DemoPackets::PACKET_LIST),
        vec![
            (4, 2, vec![Entry::new(LABEL, 10)]),
            (0, 1, vec![Entry::new(MODE, 1), Entry::new(VALUE, 4)]),
        ]
    );
    assert!(DemoPackets::OMITTED_CHANNELS.is_empty());
    // Omitted channels are listed by qualified name.
    assert_eq!(
        summary(Minimal::PACKET_LIST),
        vec![(0, 0, vec![Entry::new(VALUE, 4)])]
    );
    assert_eq!(
        Minimal::OMITTED_CHANNELS,
        &[Entry::new(LABEL, 10), Entry::new(MODE, 1)]
    );
    // The C++ file-local tables.
    assert_eq!(DemoPackets::ChannelIds::Demo_sensor_Label, LABEL);
    assert_eq!(DemoPackets::ChannelSizes::Demo_sensor_Label, 10);
    assert_eq!(DemoPackets::PacketIds::Name, 4);
    assert_eq!(DemoPackets::PacketGroups::Status, 1);
    assert_eq!(DemoPackets::PacketDataSizes::Status, 5);
    assert_eq!(DemoPackets::Packets::NUM_PACKETS, 2);
    // Header: time (11) + packet id (2) + descriptor (2).
    assert_eq!(DemoPackets::SizeBounds::PACKET_HEADER_SIZE, 15);
    assert_eq!(
        DemoPackets::SizeBounds::PACKET_MAX_DATA_SIZE,
        fprime_config::FW_COM_BUFFER_MAX_SIZE as u64 - 15
    );
}

#[test]
fn packet_tables_agree_with_the_dictionary() {
    let text = std::fs::read_to_string(concat!(env!("OUT_DIR"), "/DemoTopologyDictionary.json"))
        .expect("build.rs writes the dictionary");
    let d = Json::parse(&text).expect("valid JSON");
    let channel_id = |name: &str| -> FwChanIdType {
        let e = d
            .get("telemetryChannels")
            .and_then(Json::as_arr)
            .unwrap()
            .iter()
            .find(|c| c.get("name").and_then(Json::as_str) == Some(name))
            .unwrap_or_else(|| panic!("no channel {name}"));
        match e.get("id") {
            Some(Json::Int(i)) => *i as FwChanIdType,
            other => panic!("{other:?}"),
        }
    };
    let sets = d.get("telemetryPacketSets").and_then(Json::as_arr).unwrap();
    for (name, list, omitted) in [
        (
            "DemoPackets",
            DemoPackets::PACKET_LIST,
            DemoPackets::OMITTED_CHANNELS,
        ),
        ("Minimal", Minimal::PACKET_LIST, Minimal::OMITTED_CHANNELS),
    ] {
        let set = sets
            .iter()
            .find(|s| s.get("name").and_then(Json::as_str) == Some(name))
            .unwrap_or_else(|| panic!("no set {name}"));
        // Same packets (the dictionary orders them by id).
        let mut from_dict: Vec<(u16, FwChanIdType, Vec<FwChanIdType>)> = set
            .get("members")
            .and_then(Json::as_arr)
            .unwrap()
            .iter()
            .map(|p| {
                let int = |k: &str| match p.get(k) {
                    Some(Json::Int(i)) => *i,
                    other => panic!("{other:?}"),
                };
                let members = p
                    .get("members")
                    .and_then(Json::as_arr)
                    .unwrap()
                    .iter()
                    .map(|m| channel_id(m.as_str().unwrap()))
                    .collect();
                (int("id") as u16, int("group") as FwChanIdType, members)
            })
            .collect();
        from_dict.sort();
        let mut generated: Vec<(u16, FwChanIdType, Vec<FwChanIdType>)> = list
            .iter()
            .map(|p| (p.id, p.level, p.channels.iter().map(|c| c.id).collect()))
            .collect();
        generated.sort();
        assert_eq!(generated, from_dict, "{name}");
        let dict_omitted: Vec<FwChanIdType> = set
            .get("omitted")
            .and_then(Json::as_arr)
            .unwrap()
            .iter()
            .map(|m| channel_id(m.as_str().unwrap()))
            .collect();
        let gen_omitted: Vec<FwChanIdType> = omitted.iter().map(|c| c.id).collect();
        assert_eq!(gen_omitted, dict_omitted, "{name}");
    }
}

// -- End to end with the real packetizer ------------------------------------------

/// Collects the packetizer's packets and events.
#[derive(Default)]
struct Sink {
    packets: Mutex<Vec<(FwIndexType, Vec<u8>)>>,
    events: Mutex<Vec<FwEventIdType>>,
}

impl fprime_comp::ComPort for Sink {
    fn invoke(&self, port_num: FwIndexType, data: &mut ComBuffer, _context: u32) {
        self.packets
            .lock()
            .unwrap()
            .push((port_num, data.as_slice().to_vec()));
    }
}

impl fprime_comp::LogPort for Sink {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        _time_tag: &mut Time,
        _severity: LogSeverity,
        _args: &mut LogBuffer,
    ) {
        self.events.lock().unwrap().push(id);
    }
}

impl fprime_comp::LogTextPort for Sink {
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

/// A fixed clock for both the sensor and the packetizer.
struct Clock;
impl fprime_comp::TimePort for Clock {
    fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
        *time = Time::new(TimeBase::TbWorkstationTime, 0, 0x0102_0304, 5);
    }
}
const TIME_BYTES: [u8; 11] = [0, 2, 0, 1, 2, 3, 4, 0, 0, 0, 5];

const PACKETIZER_BASE: u32 = 0x5000;

/// The generated sensor wired to a packetizer configured with `list`,
/// `ignore` (what a deployment's `configComponents` phase does).
fn packetizer(
    list: &[TlmPacketizerPacket<'_>],
    ignore: &[Entry],
) -> (Arc<Sensor>, Arc<TlmPacketizer>, Arc<Sink>) {
    let sink = Arc::new(Sink::default());
    let pk = TlmPacketizer::new("tlmSend");
    pk.active.queued.base.set_id_base(PACKETIZER_BASE);
    pk.pkt_send[0].connect(sink.clone(), 0);
    pk.evt.log_out.connect(sink.clone(), 0);
    pk.evt.text_log_out.connect(sink.clone(), 0);
    pk.evt.time_out.connect(Arc::new(Clock), 0);
    pk.init(8);
    pk.set_packet_list(list, ignore, 1);

    let sensor = Sensor::new("sensor");
    sensor.base().set_id_base(SENSOR);
    sensor.base().tlm.tlm_out.connect_to(pk.tlm_recv_in(0));
    sensor.base().evt.time_out.connect(Arc::new(Clock), 0);
    (sensor, pk, sink)
}

fn run(pk: &Arc<TlmPacketizer>) {
    let port = pk.run_in(0);
    port.target.invoke(port.port_num, 0);
    let _ = pk
        .active
        .queued
        .do_dispatch(&**pk, BlockingType::NonBlocking);
}

#[test]
fn the_packetizer_packetizes_the_generated_tables_byte_for_byte() {
    let (sensor, pk, sink) = packetizer(DemoPackets::PACKET_LIST, DemoPackets::OMITTED_CHANNELS);
    sensor.base().tlm_write_value(1.5);
    sensor.base().tlm_write_mode(Mode::FAULTED);
    sensor.base().tlm_write_label("hi");
    run(&pk);

    let packets = sink.packets.lock().unwrap();
    // Both groups are enabled by default; section 0 goes to port 0. The
    // packets go out in table (name) order: `Name`, then `Status`.
    let mut name = vec![0x00, 0x04, 0x00, 0x04];
    name.extend_from_slice(&TIME_BYTES);
    name.extend_from_slice(&[0x00, 0x02, b'h', b'i', 0, 0, 0, 0, 0, 0]);
    let mut status = vec![0x00, 0x04, 0x00, 0x00];
    status.extend_from_slice(&TIME_BYTES);
    status.push(7); // Mode.FAULTED
    status.extend_from_slice(&1.5f32.to_be_bytes());
    assert_eq!(*packets, vec![(0, name), (0, status)]);
    assert!(sink.events.lock().unwrap().is_empty());
}

#[test]
fn omitted_channels_are_ignored_by_the_packetizer() {
    // With the generated omitted list, writing a channel that is in no
    // packet is silent ...
    let (sensor, pk, sink) = packetizer(Minimal::PACKET_LIST, Minimal::OMITTED_CHANNELS);
    sensor.base().tlm_write_value(2.0);
    sensor.base().tlm_write_mode(Mode::RUNNING);
    run(&pk);
    assert!(sink.events.lock().unwrap().is_empty());
    let packets = sink.packets.lock().unwrap();
    let mut values = vec![0x00, 0x04, 0x00, 0x00];
    values.extend_from_slice(&TIME_BYTES);
    values.extend_from_slice(&2.0f32.to_be_bytes());
    assert_eq!(*packets, vec![(0, values)]);
    drop(packets);

    // ... without it the packetizer reports the unknown channel.
    let (sensor, _pk, sink) = packetizer(Minimal::PACKET_LIST, IGNORE_OMIT_LIST);
    sensor.base().tlm_write_mode(Mode::RUNNING);
    assert_eq!(
        *sink.events.lock().unwrap(),
        vec![PACKETIZER_BASE + TlmPacketizer::EVENTID_NO_CHAN]
    );
}
