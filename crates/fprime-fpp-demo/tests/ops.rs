//! The `Ops` topology end to end: a generated topology holding a generated
//! component with an internal state machine instance (`lifecycle`) and an
//! external one (`watchdog`), a generated console, and the hand-written
//! `Svc::TlmPacketizer` (an instance of a bound component), set up by the
//! generated phase functions — including the instance init specifiers:
//! the console is built by its `instances` code, and the packetizer's
//! `configComponents` code calls `set_packet_list` with the topology's
//! generated packet table and a `configConstants` constant.

use fprime_comp::MsgDispatchStatus;
use fprime_comp::os::queue::BlockingType;
use fprime_fpp::json::Json;
use fprime_fpp_demo::controller::{Controller, WATCHDOG_LIMIT, WatchdogSignal};
use fprime_fpp_demo::generated::Demo::{
    ControllerBase, ControllerComponent, ControllerHandlers, Lifecycle, Ops_ConfigConstants,
    Ops_OpsPacketsTlmPackets as OpsPackets, OpsTopology,
};
use fprime_fw::{CmdResponse, SmSignalBuffer};
use fprime_svc::tlm_packetizer::TlmPacketizer;
use std::sync::Arc;
use std::time::{Duration, Instant};

const CONTROLLER: u32 = 0x4000;
const PACKETIZER: u32 = 0x5000;

/// Every phase up to (not including) starting the tasks, in the C++
/// `setup` order.
fn build() -> OpsTopology {
    let top = OpsTopology::new();
    top.init();
    top.config_components();
    top.set_id_bases();
    top.connect();
    top.reg_commands();
    top.read_parameters();
    top.load_parameters();
    top
}

/// Dispatch the controller's queue until it is empty.
fn drain_controller(c: &Arc<Controller>) {
    while c.base().active.queued.queue().get_messages_available() > 0 {
        assert_eq!(
            c.base()
                .active
                .queued
                .do_dispatch(&**c, BlockingType::NonBlocking),
            MsgDispatchStatus::Ok
        );
    }
}

fn drain_packetizer(p: &Arc<TlmPacketizer>) {
    while p.active.queued.queue().get_messages_available() > 0 {
        assert_eq!(
            p.active.queued.do_dispatch(&**p, BlockingType::NonBlocking),
            MsgDispatchStatus::Ok
        );
    }
}

fn events(top: &OpsTopology) -> Vec<u32> {
    top.console
        .log
        .lock()
        .unwrap()
        .events
        .iter()
        .map(|(id, _)| *id)
        .collect()
}

#[test]
fn instances_phase_and_command_registration() {
    let top = build();
    // `instances` init specifier: the console's own constructor call.
    assert_eq!(top.console.name, "ops console");
    // Without one, `Impl::new("<qualified instance name>")`.
    assert_eq!(Ops_ConfigConstants::tlmSend::START_LEVEL, 1);
    let regs = top.console.log.lock().unwrap().regs.clone();
    let mut expected: Vec<u32> = [
        ControllerBase::OPCODE_START,
        ControllerBase::OPCODE_STOP,
        ControllerBase::OPCODE_KICK,
    ]
    .iter()
    .map(|o| CONTROLLER + o)
    .collect();
    expected.extend((0..6).map(|o| PACKETIZER + o));
    assert_eq!(regs, expected);
    // `init` ran the lifecycle's initial transition before creating the
    // queue; the external machine was initialized too.
    assert_eq!(
        top.controller.base().lifecycle_get_state(),
        Lifecycle::State::IDLE
    );
    assert_eq!(top.controller.base().watchdog.count(), 0);
}

#[test]
fn commands_drive_the_internal_state_machine_through_the_queue() {
    let top = build();
    let c = &top.controller;
    top.console
        .send_command(0, CONTROLLER + ControllerBase::OPCODE_START, 7);
    // The command is queued, then its handler queues the `start` signal.
    drain_controller(c);
    assert_eq!(
        c.base().lifecycle_get_state(),
        Lifecycle::State::RUNNING_WARMUP
    );
    assert_eq!(
        top.console.log.lock().unwrap().responses,
        [(
            CONTROLLER + ControllerBase::OPCODE_START,
            7,
            CmdResponse::Ok
        )]
    );
    assert_eq!(
        events(&top),
        [CONTROLLER + ControllerBase::EVENTID_RUNNING_ENTERED]
    );

    // Ticks: WARMUP -> CHECK (typed U32 choice) until warmed up, then
    // ACTIVE, where ticks are counted by an internal transition.
    for n in 1..=4 {
        top.console.tick(0, n);
        drain_controller(c);
    }
    assert_eq!(c.log.lock().unwrap().counted, [1, 2, 3, 4]);
    assert_eq!(
        c.base().lifecycle_get_state(),
        Lifecycle::State::RUNNING_ACTIVE
    );

    top.console
        .send_command(0, CONTROLLER + ControllerBase::OPCODE_STOP, 8);
    drain_controller(c);
    assert_eq!(c.base().lifecycle_get_state(), Lifecycle::State::IDLE);
    assert_eq!(
        events(&top),
        [
            CONTROLLER + ControllerBase::EVENTID_RUNNING_ENTERED,
            CONTROLLER + ControllerBase::EVENTID_RUNNING_EXITED
        ]
    );
}

#[test]
fn the_external_watchdog_stops_the_lifecycle_unless_kicked() {
    let top = build();
    let c = &top.controller;
    c.lifecycle_send_signal_start();
    drain_controller(c);
    for n in 1..WATCHDOG_LIMIT {
        top.console.tick(0, n);
        drain_controller(c);
    }
    assert_eq!(c.base().watchdog.count(), WATCHDOG_LIMIT - 1);
    // A kick resets the count.
    top.console
        .send_command(0, CONTROLLER + ControllerBase::OPCODE_KICK, 1);
    drain_controller(c);
    assert_eq!(c.base().watchdog.count(), 0);
    for n in 0..WATCHDOG_LIMIT {
        top.console.tick(0, n);
        drain_controller(c);
    }
    // The watchdog expired on the controller's thread and stopped the
    // lifecycle.
    assert_eq!(c.base().lifecycle_get_state(), Lifecycle::State::IDLE);
    assert_eq!(
        events(&top),
        [
            CONTROLLER + ControllerBase::EVENTID_RUNNING_ENTERED,
            CONTROLLER + ControllerBase::EVENTID_WATCHDOG_EXPIRED,
            CONTROLLER + ControllerBase::EVENTID_RUNNING_EXITED
        ]
    );
}

#[test]
fn state_machine_queue_policies() {
    let top = build();
    let c = &top.controller;
    // Fill the queue (depth 10) with watchdog signals: the eleventh goes to
    // the `hook` instance's overflow hook; the `drop` lifecycle loses its.
    for _ in 0..10 {
        c.watchdog_state_machine_invoke(WatchdogSignal::Tick, &SmSignalBuffer::new());
    }
    c.watchdog_state_machine_invoke(WatchdogSignal::Kick, &SmSignalBuffer::new());
    assert_eq!(c.log.lock().unwrap().watchdog_overflows, [1]);
    c.lifecycle_send_signal_start();
    assert_eq!(c.base().active.queued.get_num_msgs_dropped(), 1);
    drain_controller(c);
    assert_eq!(c.base().watchdog.count(), 10);
    assert_eq!(c.base().lifecycle_get_state(), Lifecycle::State::IDLE);
}

#[test]
fn the_packetizer_is_configured_by_the_init_specifier() {
    let top = build();
    let c = &top.controller;
    c.lifecycle_send_signal_start();
    drain_controller(c);
    for n in 1..=4 {
        top.console.tick(0, n);
        drain_controller(c);
    }
    // Run the packetizer: the generated packet table (set by the
    // configComponents code) packetizes the controller's channels.
    top.console.tick(1, 0);
    drain_packetizer(&top.tlm_send);
    let packets = top.console.log.lock().unwrap().packets.clone();
    let mut status = vec![0x00, 0x04, 0x00, 0x00]; // packetized telemetry, packet id 0
    status.extend_from_slice(&[0, 2, 0, 0, 0, 0, 100, 0, 0, 0, 0]); // the console's time
    status.push(Lifecycle::State::RUNNING_ACTIVE.as_repr()); // State
    status.extend_from_slice(&4u32.to_be_bytes()); // Ticks
    assert_eq!(packets, vec![(0, status)]);
    // Neither the table nor the omit list is missing a channel: no
    // `NoChan` event.
    assert!(
        !events(&top).contains(&(PACKETIZER + TlmPacketizer::EVENTID_NO_CHAN)),
        "{:?}",
        events(&top)
    );
    assert_eq!(OpsPackets::PACKET_LIST.len(), 1);
    assert_eq!(OpsPackets::OMITTED_CHANNELS.len(), 2);
}

#[test]
fn setup_and_teardown_run_the_deployment_on_its_threads() {
    let top = OpsTopology::new();
    top.setup();
    top.console
        .send_command(0, CONTROLLER + ControllerBase::OPCODE_START, 1);
    let deadline = Instant::now() + Duration::from_secs(5);
    while top.controller.base().lifecycle_get_state() != Lifecycle::State::RUNNING_WARMUP {
        assert!(
            Instant::now() < deadline,
            "the controller thread did not run"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
    top.teardown();
}

#[test]
fn the_ops_dictionary_has_the_state_enum_and_the_packetizer() {
    let text = std::fs::read_to_string(concat!(env!("OUT_DIR"), "/OpsTopologyDictionary.json"))
        .expect("build.rs writes the dictionary");
    let d = Json::parse(&text).expect("valid JSON");
    let types = d.get("typeDefinitions").and_then(Json::as_arr).unwrap();
    assert!(
        types
            .iter()
            .any(|t| t.get("qualifiedName").and_then(Json::as_str) == Some("Demo.Lifecycle.State"))
    );
    let channels = d.get("telemetryChannels").and_then(Json::as_arr).unwrap();
    let names: Vec<&str> = channels
        .iter()
        .filter_map(|c| c.get("name").and_then(Json::as_str))
        .collect();
    assert!(names.contains(&"Demo.controller.State"));
    assert!(names.contains(&"Demo.tlmSend.SectionEnabled"));
}
