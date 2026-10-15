//! Runs the `#![no_std]` example deployment
//! (`examples/no_std_deployment.rs`) on the host, so the superloop it
//! documents is checked, not just compiled. Pulled in as a module, the
//! example's crate-level `#![no_std]` is inert here (hence the allow).
//! Runs in both configurations (`std`, and `--no-default-features
//! --features spin`), with no platform registered.

#![allow(unused_attributes)]

#[path = "../examples/no_std_deployment.rs"]
mod example;

use example::{CMD_DISP_BASE_ID, Deployment, EVENTS_BASE_ID, command_frame, packet_ids};
use fprime_svc::cmd_dispatcher::CmdDispatcher;

const FW_PACKET_TELEM: u16 = 1;
const FW_PACKET_LOG: u16 = 2;

#[test]
fn superloop_dispatches_an_uplinked_command_and_downlinks_events_and_telemetry() {
    let d = Deployment::new();

    // Registration is quiet: `OpCodeRegistered` is a DIAGNOSTIC event,
    // which the event manager filters by default.
    d.drain();
    assert_eq!(packet_ids(&d.take_downlink()), []);

    // A NO_OP over the wire: comStub -> frameAccumulator -> deframer ->
    // router -> cmdDisp -> cmdDisp (its own command) -> status back.
    d.uplink(&command_frame(
        CMD_DISP_BASE_ID + CmdDispatcher::OPCODE_CMD_NO_OP,
    ));
    d.drain();
    let down = packet_ids(&d.take_downlink());
    for event in [
        CmdDispatcher::EVENTID_OP_CODE_DISPATCHED,
        CmdDispatcher::EVENTID_NO_OP_RECEIVED,
        CmdDispatcher::EVENTID_OP_CODE_COMPLETED,
    ] {
        assert!(
            down.contains(&(FW_PACKET_LOG, CMD_DISP_BASE_ID + event)),
            "event {event} missing from {down:x?}"
        );
    }

    // The rate group runs tlmSend, which downlinks the dispatcher's
    // CommandsDispatched channel (= 1) inside a telemetry packet:
    // `[id u32][time 11 B][value u32]` records after the packet type.
    d.step();
    let frames = d.take_downlink();
    let mut record = (CMD_DISP_BASE_ID + CmdDispatcher::CHANID_COMMANDS_DISPATCHED)
        .to_be_bytes()
        .to_vec();
    let tlm_frame = frames
        .iter()
        .find(|f| f[8..10] == FW_PACKET_TELEM.to_be_bytes() && contains(&f[10..], &record))
        .expect("CommandsDispatched downlinked");
    let at = 10 + find(&tlm_frame[10..], &record).unwrap();
    record.clear();
    record.extend_from_slice(&tlm_frame[at + 4 + 11..at + 4 + 11 + 4]);
    assert_eq!(record, 1u32.to_be_bytes());
    // Only the dispatcher spoke: no warning from any other component (an
    // overflowing queue, a bad frame, an exhausted buffer pool).
    assert!(down.iter().all(
        |&(kind, id)| kind == FW_PACKET_LOG && (CMD_DISP_BASE_ID..EVENTS_BASE_ID).contains(&id)
    ));
}

#[test]
fn run_scenario_reports_the_no_op() {
    let ids = example::run_scenario();
    assert!(ids.contains(&(
        FW_PACKET_LOG,
        CMD_DISP_BASE_ID + CmdDispatcher::EVENTID_NO_OP_RECEIVED
    )));
}

fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}

fn contains(haystack: &[u8], needle: &[u8]) -> bool {
    find(haystack, needle).is_some()
}
