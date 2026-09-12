//! The generated data types: constants, enum values and defaults, array
//! and struct defaults, serialized sizes and byte-exact serialization.

use fprime_fpp_demo::Demo;
use fprime_fw::{Endianness, FwString, LinearBuffer, SerBuf, Serialize};

fn bytes<T: Serialize>(v: &T) -> Vec<u8> {
    let mut buf = LinearBuffer::<256>::new();
    assert_eq!(
        v.serialize_to(&mut buf, Endianness::Big),
        fprime_fw::SerializeStatus::Ok
    );
    buf.as_slice().to_vec()
}

#[test]
fn constants_have_the_evaluated_values_and_types() {
    assert_eq!(Demo::COUNT, 4i64);
    assert_eq!(Demo::DOUBLE_COUNT, 8i64);
    assert_eq!(Demo::MASK, 0xFF00i64);
    assert_eq!(Demo::SCALE, 1.5f64);
    assert_eq!(Demo::NAME, "demo");
    assert!(Demo::ENABLED);
    assert_eq!(Demo::START_MODE, Demo::Mode::FAULTED);
}

#[test]
fn enums_follow_the_reference_numbering() {
    assert_eq!(Demo::Mode::IDLE.as_repr(), 0u8);
    assert_eq!(Demo::Mode::RUNNING.as_repr(), 1);
    assert_eq!(Demo::Mode::FAULTED.as_repr(), 7);
    assert_eq!(Demo::Mode::RECOVERING.as_repr(), 8);
    assert_eq!(Demo::Mode::default(), Demo::Mode::RUNNING);
    assert_eq!(Demo::Mode::SERIALIZED_SIZE, 1);
    assert_eq!(Demo::Level::LOW.as_repr(), -1i32);
    assert_eq!(Demo::Level::default(), Demo::Level::LOW);
    assert_eq!(Demo::Level::SERIALIZED_SIZE, 4);
    assert_eq!(bytes(&Demo::Level::HIGH), vec![0, 0, 0, 1]);
    assert_eq!(Demo::Mode::try_from(3u8), Err(3));
}

#[test]
fn arrays_have_their_defaults_and_wire_format() {
    let c = Demo::Counts::default();
    assert_eq!(c.as_slice(), &[0xABCD; 4]);
    assert_eq!(Demo::Counts::SERIALIZED_SIZE, 8);
    assert_eq!(
        bytes(&c),
        vec![0xAB, 0xCD, 0xAB, 0xCD, 0xAB, 0xCD, 0xAB, 0xCD]
    );
    let g = Demo::Gains::default();
    assert_eq!(g.as_slice(), &[1.0f32, 2.5, -0.5]);
    assert_eq!(bytes(&g)[..4], 1.0f32.to_be_bytes());
}

#[test]
fn structs_have_their_defaults_and_wire_format() {
    let r = Demo::Reading::default();
    assert_eq!(r.mode, Demo::Mode::IDLE);
    assert_eq!(r.counts.as_slice(), &[0xABCD; 4]);
    assert_eq!(r.samples.as_slice(), &[3i16, -4]);
    assert_eq!(r.label.as_str(), Some("abc"));
    assert!(r.valid);
    // mode 1 + counts 8 + samples 4 + label (2 + 8) + valid 1
    assert_eq!(Demo::Reading::SERIALIZED_SIZE, 1 + 8 + 4 + 10 + 1);
    let b = bytes(&r);
    let mut expected = vec![0u8];
    expected.extend_from_slice(&[0xAB, 0xCD, 0xAB, 0xCD, 0xAB, 0xCD, 0xAB, 0xCD]);
    expected.extend_from_slice(&[0, 3, 0xFF, 0xFC]);
    expected.extend_from_slice(&[0, 3, b'a', b'b', b'c']);
    expected.push(0xFF);
    assert_eq!(b, expected);

    let f = Demo::Frame::default();
    assert_eq!(f.seq, 0);
    assert_eq!(f.reading, r);
    assert_eq!(f.level, Demo::Level::LOW);
    let built = Demo::Frame::new(
        7,
        Demo::Reading::new(
            Demo::Mode::RECOVERING,
            Demo::Counts::fill(1),
            Demo::Reading_samples_Array::new([1, 2]),
            FwString::from("hi"),
            false,
        ),
        Demo::Level::HIGH,
    );
    assert_eq!(*built.get_seq(), 7);
    assert_eq!(bytes(&built)[..4], [0, 0, 0, 7]);
}

#[test]
fn aliases_and_ports_are_usable() {
    let s: Demo::Seq = 5;
    let i: Demo::Ident = 6;
    assert_eq!(u32::from(s) + i, 11);

    struct Impl;
    impl Demo::MeasurePort for Impl {
        fn invoke(
            &self,
            _port_num: fprime_config::FwIndexType,
            seq: Demo::Seq,
            mode: Demo::Mode,
            reading: &Demo::Reading,
            label: &FwString<8>,
            result: &mut Demo::Frame,
        ) -> Demo::Level {
            result.seq = seq;
            result.reading = reading.clone();
            result.reading.mode = mode;
            result.reading.label = label.clone();
            Demo::Level::MID
        }
    }
    impl Demo::TickPort for Impl {
        fn invoke(&self, _port_num: fprime_config::FwIndexType) {}
    }
    impl Demo::DeliverPort for Impl {
        fn invoke(
            &self,
            _port_num: fprime_config::FwIndexType,
            buffer: fprime_fw::Buffer,
            time_tag: &mut fprime_fw::Time,
        ) {
            let _ = (buffer, time_tag);
        }
    }
    let p = Impl;
    let mut out = Demo::Frame::default();
    let lvl = Demo::MeasurePort::invoke(
        &p,
        0,
        9,
        Demo::Mode::FAULTED,
        &Demo::Reading::default(),
        &FwString::from("x"),
        &mut out,
    );
    assert_eq!(lvl, Demo::Level::MID);
    assert_eq!(out.seq, 9);
    assert_eq!(out.reading.mode, Demo::Mode::FAULTED);
    Demo::TickPort::invoke(&p, 0);
}
