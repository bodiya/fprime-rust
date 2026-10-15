//! `CdhCore`'s `fatalAdapter` in the Rust Ref: a failed `fw_assert!` in the
//! deployment becomes an `AF_ASSERT_*` FATAL event, which `events`
//! downlinks and announces to `fatalHandler`.
//!
//! Alone in its test binary: constructing the topology installs the
//! process-wide assert hook (the newest topology's adapter receives every
//! assert), so no other test may share the process.

mod common;

use std::sync::{Arc, Mutex};

use common::Harness;
use fprime_fw::fw_assert;
use fprime_ref::topology::FATAL_ADAPTER_BASE_ID;
use fprime_svc::assert_fatal_adapter::AssertFatalAdapter;

#[test]
fn a_failed_assert_is_a_downlinked_fatal_that_reaches_the_fatal_handler() {
    let harness = Harness::up();
    let fatals = Arc::new(Mutex::new(Vec::new()));
    let seen = fatals.clone();
    harness
        .topology
        .fatal_handler
        .set_exit_action(Box::new(move |id| seen.lock().unwrap().push(id)));

    fw_assert!(false, 7);
    let line = line!() - 1;

    // The FATAL path runs on the asserting thread: events -> FatalAnnounce
    // -> fatalHandler (its one-second flush delay, then the exit action).
    let id = FATAL_ADAPTER_BASE_ID + AssertFatalAdapter::EVENTID_AF_ASSERT_1;
    assert_eq!(*fatals.lock().unwrap(), vec![id]);

    // ... and the event goes down to the ground with its arguments:
    // [file: u16 length + bytes][line U32][arg1 U32].
    assert!(harness.tick_until(|| harness.saw_event(id)));
    let args = harness.event_args(id).unwrap();
    let file = file!().as_bytes();
    let mut expected = (file.len() as u16).to_be_bytes().to_vec();
    expected.extend_from_slice(file);
    expected.extend_from_slice(&line.to_be_bytes());
    expected.extend_from_slice(&7u32.to_be_bytes());
    assert_eq!(args, expected);
}
