//! # `Svc::AssertFatalAdapter` tests
//!
//! Port of `Svc/AssertFatalAdapter/test/ut` (`AssertFatalAdapterTester`:
//! `testAsserts`, `testTruncation`) plus the fallback paths. The adapter
//! installs the process-wide assert hook, under which a failed
//! `fw_assert!` no longer panics, so these tests live in their own test
//! binary (no other crate test can be disturbed by the hook) and run one
//! at a time, each removing the hook when done.

use fprime_comp::{LogPort, LogTextPort};
use fprime_config::{FwEventIdType, FwIndexType};
use fprime_fw::assert::deregister_assert_hook;
use fprime_fw::{LogBuffer, LogSeverity, SerBuf, TextLogString, Time, fw_assert};
use fprime_svc::assert_fatal_adapter::{AssertFatalAdapter, EVENT_FILE_ARG_SIZE};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

/// Serializes the tests: the hook is global.
static HOOK: Mutex<()> = Mutex::new(());

/// Holds the test lock and removes the hook on drop (also on a failed
/// assertion, so one failure does not cascade into the other tests).
struct HookGuard(#[allow(dead_code)] MutexGuard<'static, ()>);

impl HookGuard {
    fn take() -> Self {
        Self(HOOK.lock().unwrap_or_else(PoisonError::into_inner))
    }
}

impl Drop for HookGuard {
    fn drop(&mut self) {
        let _ = deregister_assert_hook();
    }
}

/// One decoded `AF_ASSERT_*` / `AF_UNEXPECTED_ASSERT` event.
#[derive(Debug, PartialEq)]
struct Event {
    id: FwEventIdType,
    severity: LogSeverity,
    file: String,
    line: u32,
    args: Vec<u32>,
}

#[derive(Default)]
struct Sink {
    events: Mutex<Vec<Event>>,
    text: Mutex<Vec<String>>,
}

impl LogPort for Sink {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        _time_tag: &mut Time,
        severity: LogSeverity,
        args: &mut LogBuffer,
    ) {
        let mut len = 0u16;
        assert!(args.deserialize_u16_be(&mut len).is_ok());
        let mut file = vec![0u8; usize::from(len)];
        for byte in &mut file {
            assert!(args.deserialize_u8_be(byte).is_ok());
        }
        let mut line = 0u32;
        assert!(args.deserialize_u32_be(&mut line).is_ok());
        let mut values = Vec::new();
        while args.deserialize_size_left() > 0 {
            let mut value = 0u32;
            assert!(args.deserialize_u32_be(&mut value).is_ok());
            values.push(value);
        }
        self.events.lock().unwrap().push(Event {
            id,
            severity,
            file: String::from_utf8(file).unwrap(),
            line,
            args: values,
        });
    }
}

impl LogTextPort for Sink {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        _id: FwEventIdType,
        _time_tag: &mut Time,
        _severity: LogSeverity,
        text: &mut TextLogString,
    ) {
        self.text.lock().unwrap().push(text.to_string());
    }
}

impl Sink {
    fn take_events(&self) -> Vec<Event> {
        std::mem::take(&mut *self.events.lock().unwrap())
    }
    fn last_text(&self) -> String {
        self.text
            .lock()
            .unwrap()
            .last()
            .cloned()
            .expect("a text log")
    }
}

fn adapter() -> (Arc<AssertFatalAdapter>, Arc<Sink>) {
    let comp = AssertFatalAdapter::new("AssertFatalAdapter");
    let sink = Arc::new(Sink::default());
    comp.evt.log_out.connect(sink.clone(), 0);
    comp.evt.text_log_out.connect(sink.clone(), 0);
    (comp, sink)
}

fn event(id: FwEventIdType, file: &str, line: u32, args: &[u32]) -> Event {
    Event {
        id,
        severity: LogSeverity::Fatal,
        file: file.to_string(),
        line,
        args: args.to_vec(),
    }
}

/// `doLeadingTruncation`: keep the last `n` bytes.
fn leading_truncation(s: &str, n: usize) -> &str {
    &s[s.len().saturating_sub(n)..]
}

/// `fillWithBytePattern`: "0011223344..." (each position's (i / 2) % 100).
fn byte_pattern(len: usize) -> String {
    (0..len)
        .map(|i| {
            let digit = (i / 2) % 100;
            char::from(b'0' + if i % 2 == 0 { digit / 10 } else { digit % 10 } as u8)
        })
        .collect()
}

#[test]
fn asserts_become_fatal_events() {
    // TEST(Nominal, NominalInit): testAsserts
    let _hook = HookGuard::take();
    let (comp, sink) = adapter();
    let file = leading_truncation(file!(), EVENT_FILE_ARG_SIZE);

    fw_assert!(false);
    let line = line!() - 1;
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_0,
            file,
            line,
            &[]
        )]
    );

    fw_assert!(false, 1);
    let line = line!() - 1;
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_1,
            file,
            line,
            &[1]
        )]
    );

    fw_assert!(false, 1, 2);
    let line = line!() - 1;
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_2,
            file,
            line,
            &[1, 2]
        )]
    );

    fw_assert!(false, 1, 2, 3);
    let line = line!() - 1;
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_3,
            file,
            line,
            &[1, 2, 3]
        )]
    );

    fw_assert!(false, 1, 2, 3, 4);
    let line = line!() - 1;
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_4,
            file,
            line,
            &[1, 2, 3, 4]
        )]
    );

    fw_assert!(false, 1, 2, 3, 4, 5);
    let line = line!() - 1;
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_5,
            file,
            line,
            &[1, 2, 3, 4, 5]
        )]
    );

    fw_assert!(false, 1, 2, 3, 4, 5, 6);
    let line = line!() - 1;
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_6,
            file,
            line,
            &[1, 2, 3, 4, 5, 6]
        )]
    );
    assert_eq!(
        sink.last_text(),
        format!("Assert in file {file}, line {line}: 1 2 3 4 5 6")
    );

    // Test unexpected assert
    comp.report_assert("foo", 1000, 10, [1, 2, 3, 4, 5, 6]);
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_UNEXPECTED_ASSERT,
            "foo",
            1000,
            &[10]
        )]
    );
    assert_eq!(
        sink.last_text(),
        "Unexpected assert in file foo, line 1000, args 10"
    );
}

#[test]
fn file_path_truncation() {
    // TEST(Nominal, FilePathTruncation): testTruncation
    let _hook = HookGuard::take();
    let (comp, sink) = adapter();
    let max_size = EVENT_FILE_ARG_SIZE;
    assert_eq!(max_size, 200);

    // Test 1: a short path is unchanged.
    let short_path = "test/short.cpp";
    comp.report_assert(short_path, 100, 0, [0; 6]);
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_0,
            short_path,
            100,
            &[]
        )]
    );
    assert!(sink.last_text().contains(short_path));

    // Test 2: a path exactly at the limit is unchanged.
    let exact_path = byte_pattern(max_size);
    comp.report_assert(&exact_path, 200, 0, [0; 6]);
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_0,
            &exact_path,
            200,
            &[]
        )]
    );
    assert!(sink.last_text().contains(&exact_path));

    // Test 3: a long path keeps its trailing portion.
    let long_size = max_size + 2;
    let suffix = "/suffix/truncation_test.cpp";
    let prefix_len = long_size - max_size;
    let mut long_path: Vec<u8> = byte_pattern(long_size).into_bytes();
    for (i, byte) in long_path.iter_mut().take(prefix_len).enumerate() {
        *byte = b'a' + (i % 10) as u8;
    }
    let at = prefix_len + (max_size - suffix.len());
    long_path[at..].copy_from_slice(suffix.as_bytes());
    let long_path = String::from_utf8(long_path).unwrap();
    let expected = leading_truncation(&long_path, max_size);
    assert_eq!(expected.len(), max_size);
    comp.report_assert(&long_path, 300, 1, [42, 0, 0, 0, 0, 0]);
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_1,
            expected,
            300,
            &[42]
        )]
    );
    let text = sink.last_text();
    assert!(text.contains(expected));
    assert!(!text.contains(&long_path[..10]));

    // Test 4: the same truncation for another variant (AF_ASSERT_3).
    let very_long_path = byte_pattern(long_size);
    let expected = leading_truncation(&very_long_path, max_size);
    comp.report_assert(&very_long_path, 400, 3, [1, 2, 3, 0, 0, 0]);
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_3,
            expected,
            400,
            &[1, 2, 3]
        )]
    );
    let text = sink.last_text();
    assert!(text.contains(expected));
    assert!(!text.contains(&very_long_path[..10]));
}

#[test]
fn arguments_and_lines_are_reported_as_u32() {
    let _hook = HookGuard::take();
    let (comp, sink) = adapter();
    comp.report_assert("f.rs", 0x1_0000_0007, 2, [-1, i32::MIN, 0, 0, 0, 0]);
    assert_eq!(
        sink.take_events(),
        vec![event(
            AssertFatalAdapter::EVENTID_AF_ASSERT_2,
            "f.rs",
            7,
            &[u32::MAX, 0x8000_0000]
        )]
    );
    assert_eq!(
        sink.last_text(),
        "Assert in file f.rs, line 7: 4294967295 2147483648"
    );
}

#[test]
fn the_id_base_offsets_the_event_ids() {
    let _hook = HookGuard::take();
    let (comp, sink) = adapter();
    comp.base.set_id_base(0x0100_6000);
    fw_assert!(1 + 1 == 3, 5);
    let events = sink.take_events();
    assert_eq!(events.len(), 1);
    assert_eq!(
        events[0].id,
        0x0100_6000 + AssertFatalAdapter::EVENTID_AF_ASSERT_1
    );
}

#[test]
fn the_newest_adapter_receives_the_asserts() {
    let _hook = HookGuard::take();
    let (_first, first_sink) = adapter();
    let (_second, second_sink) = adapter();
    fw_assert!(false);
    assert!(first_sink.take_events().is_empty());
    assert_eq!(second_sink.take_events().len(), 1);
}

#[test]
fn an_unconnected_log_port_falls_back_to_assert() {
    let _hook = HookGuard::take();
    let comp = AssertFatalAdapter::new("AssertFatalAdapter");
    let result = std::panic::catch_unwind(|| {
        fw_assert!(false, 1);
    });
    assert!(result.is_err(), "C assert(false) fallback must panic");
    // The counter was released: once connected, asserts are reported.
    let sink = Arc::new(Sink::default());
    comp.evt.log_out.connect(sink.clone(), 0);
    for _ in 0..=fprime_svc::assert_fatal_adapter::FW_ASSERT_COUNT_MAX {
        fw_assert!(false);
    }
    assert_eq!(
        sink.take_events().len(),
        fprime_svc::assert_fatal_adapter::FW_ASSERT_COUNT_MAX + 1
    );
}

/// A `Log` sink whose handling asserts again (an assert in the event path).
struct Cascade {
    depth: Mutex<usize>,
}

impl LogPort for Cascade {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        _id: FwEventIdType,
        _time_tag: &mut Time,
        _severity: LogSeverity,
        _args: &mut LogBuffer,
    ) {
        *self.depth.lock().unwrap() += 1;
        fw_assert!(false);
    }
}

#[test]
fn cascading_asserts_fall_back_to_assert_past_the_limit() {
    let _hook = HookGuard::take();
    let comp = AssertFatalAdapter::new("AssertFatalAdapter");
    let cascade = Arc::new(Cascade {
        depth: Mutex::new(0),
    });
    comp.evt.log_out.connect(cascade.clone(), 0);
    let result = std::panic::catch_unwind(|| {
        fw_assert!(false);
    });
    assert!(result.is_err());
    // FW_ASSERT_COUNT_MAX nested reports emitted an event; the next one
    // fell back.
    assert_eq!(
        *cascade.depth.lock().unwrap(),
        fprime_svc::assert_fatal_adapter::FW_ASSERT_COUNT_MAX
    );
}

#[test]
fn a_dropped_adapter_panics_as_unregistered() {
    let _hook = HookGuard::take();
    drop(AssertFatalAdapter::new("AssertFatalAdapter"));
    let result = std::panic::catch_unwind(|| {
        fw_assert!(false);
    });
    let err = result.expect_err("the unregistered hook must panic");
    let text = err
        .downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| err.downcast_ref::<String>().cloned())
        .unwrap();
    assert!(text.contains("not registered"), "got: {text}");
}

#[test]
fn event_ids_match_upstream() {
    assert_eq!(AssertFatalAdapter::EVENTID_AF_ASSERT_0, 0);
    assert_eq!(AssertFatalAdapter::EVENTID_AF_ASSERT_1, 1);
    assert_eq!(AssertFatalAdapter::EVENTID_AF_ASSERT_2, 2);
    assert_eq!(AssertFatalAdapter::EVENTID_AF_ASSERT_3, 3);
    assert_eq!(AssertFatalAdapter::EVENTID_AF_ASSERT_4, 4);
    assert_eq!(AssertFatalAdapter::EVENTID_AF_ASSERT_5, 5);
    assert_eq!(AssertFatalAdapter::EVENTID_AF_ASSERT_6, 6);
    assert_eq!(AssertFatalAdapter::EVENTID_AF_UNEXPECTED_ASSERT, 7);
}
