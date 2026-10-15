//! Runs the `#![no_std]` example component (`examples/no_std_component.rs`)
//! on the host, so the superloop it documents is checked, not just
//! compiled. Pulled in as a module, the example's crate-level `#![no_std]`
//! is inert here, and the parts it only declares are unused (hence the
//! allows).

#![allow(unused_attributes, dead_code)]

#[path = "../examples/no_std_component.rs"]
mod example;

#[test]
fn superloop_drains_async_messages_and_drives_the_state_machine() {
    // Two `runIn` invocations, each handled once and each toggling the
    // blinker state machine.
    assert_eq!(example::run_once(), (vec![7, 8], 2));
}
