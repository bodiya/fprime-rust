//! In-process integration test of the full reference topology WITHOUT TCP
//! (shared harness in `tests/common/mod.rs`): a loopback driver captures
//! downlink frames and the test injects uplink bytes; rate groups are
//! ticked manually.
//!
//! This file covers the command/event/telemetry core: a framed NO_OP round
//! trip, SignalGen telemetry reaching the downlink after rate-group ticks,
//! and the invalid-opcode path. The newer subsystems (file services,
//! sequencer, parameters, data products) are covered by
//! `subsystems_test.rs`.

mod common;

use common::{DEADLINE, Harness, build_command_frame, deframe, wait_until};

use fprime_ref::signal_gen;
use fprime_ref::topology::{
    BUFFER_MANAGER_BASE_ID, CMD_DISPATCHER_BASE_ID, CMD_SEQUENCER_BASE_ID, COM_QUEUE_BASE_ID,
    DP_BUFFER_MANAGER_BASE_ID, DP_CATALOG_BASE_ID, DP_MANAGER_BASE_ID, DP_WRITER_BASE_ID,
    EVENT_MANAGER_BASE_ID, FILE_DOWNLINK_BASE_ID, FILE_MANAGER_BASE_ID, FILE_UPLINK_BASE_ID,
    HEALTH_BASE_ID, RATE_GROUP_1_BASE_ID, RATE_GROUP_2_BASE_ID, RATE_GROUP_3_BASE_ID,
    SIGNAL_GEN_BASE_ID, SYSTEM_RESOURCES_BASE_ID,
};
use fprime_svc::cmd_dispatcher::CmdDispatcher;

/// Snapshot of every telemetry (channel id, value-bytes) pair downlinked
/// so far.
fn tlm_values(harness: &Harness) -> Vec<(u32, Vec<u8>)> {
    harness
        .frames()
        .iter()
        .flat_map(|frame| parse_tlm_packet(&deframe(frame)))
        .collect()
}

// ---------------------------------------------------------------------------
// Telemetry dictionary (the packets carry no per-entry length)
// ---------------------------------------------------------------------------

/// Value size (bytes) for a telemetry channel — the test's mini dictionary
/// (TLM packet entries carry no per-entry length, C++ parity). `value` is
/// the remaining payload, needed only by the one variable-length channel.
fn tlm_value_size(id: u32, value: &[u8]) -> usize {
    // Fixed-width helpers keyed on the instance base id.
    const U32: usize = 4;
    const U64: usize = 8;
    const F32: usize = 4;
    match id {
        _ if id == SIGNAL_GEN_BASE_ID + signal_gen::CHANID_SIGNAL_VALUE => F32,
        _ if id == SIGNAL_GEN_BASE_ID + signal_gen::CHANID_SIGNAL_TYPE => 1, // U8
        _ if id == CMD_DISPATCHER_BASE_ID + CmdDispatcher::CHANID_COMMANDS_DISPATCHED => U32,
        _ if id == CMD_DISPATCHER_BASE_ID + CmdDispatcher::CHANID_COMMAND_ERRORS => U32,
        _ if id == CMD_DISPATCHER_BASE_ID + CmdDispatcher::CHANID_COMMANDS_DROPPED => U32,
        _ if id == EVENT_MANAGER_BASE_ID => U64, // EventsDropped: FwSizeType
        _ if id == COM_QUEUE_BASE_ID => U64,     // ComQueueDepth [2] U32
        _ if id == COM_QUEUE_BASE_ID + 1 => U32, // BuffQueueDepth [1] U32
        _ if id == HEALTH_BASE_ID => U32,        // PingLateWarnings U32
        _ if (BUFFER_MANAGER_BASE_ID..BUFFER_MANAGER_BASE_ID + 5).contains(&id) => U32,
        _ if (DP_BUFFER_MANAGER_BASE_ID..DP_BUFFER_MANAGER_BASE_ID + 5).contains(&id) => U32,
        _ if id == RATE_GROUP_1_BASE_ID || id == RATE_GROUP_1_BASE_ID + 1 => U32,
        _ if id == RATE_GROUP_2_BASE_ID || id == RATE_GROUP_2_BASE_ID + 1 => U32,
        _ if id == RATE_GROUP_3_BASE_ID || id == RATE_GROUP_3_BASE_ID + 1 => U32,
        // fileUplink FilesReceived/PacketsReceived/Warnings/FilesReceivedFailed
        _ if (FILE_UPLINK_BASE_ID..FILE_UPLINK_BASE_ID + 4).contains(&id) => U32,
        // fileDownlink FilesSent/PacketsSent/Warnings
        _ if (FILE_DOWNLINK_BASE_ID..FILE_DOWNLINK_BASE_ID + 3).contains(&id) => U32,
        // fileManager CommandsExecuted/Errors
        _ if (FILE_MANAGER_BASE_ID..FILE_MANAGER_BASE_ID + 2).contains(&id) => U32,
        // dpMgr NumSuccessful/NumFailed/NumDataProducts (U32) + NumBytes (U64)
        _ if (DP_MANAGER_BASE_ID..DP_MANAGER_BASE_ID + 3).contains(&id) => U32,
        _ if id == DP_MANAGER_BASE_ID + 3 => U64,
        // dpWriter NumBuffersReceived (U32), NumBytesWritten (U64), then U32s
        _ if id == DP_WRITER_BASE_ID => U32,
        _ if id == DP_WRITER_BASE_ID + 1 => U64,
        _ if (DP_WRITER_BASE_ID + 2..DP_WRITER_BASE_ID + 5).contains(&id) => U32,
        // dpCat CatalogDps/DpsSent (declared, never written)
        _ if (DP_CATALOG_BASE_ID..DP_CATALOG_BASE_ID + 2).contains(&id) => U32,
        // cmdSeq counters (U32) + CS_CurrentSequence (string size 240)
        _ if (CMD_SEQUENCER_BASE_ID..CMD_SEQUENCER_BASE_ID + 5).contains(&id) => U32,
        _ if id == CMD_SEQUENCER_BASE_ID + 5 => {
            assert!(value.len() >= 2, "truncated CS_CurrentSequence");
            2 + usize::from(u16::from_be_bytes([value[0], value[1]]))
        }
        // systemResources memory/non-volatile (U64) then CPU + CPU_nn (F32)
        _ if (SYSTEM_RESOURCES_BASE_ID..SYSTEM_RESOURCES_BASE_ID + 4).contains(&id) => U64,
        _ if (SYSTEM_RESOURCES_BASE_ID + 4..SYSTEM_RESOURCES_BASE_ID + 26).contains(&id) => F32,
        _ => panic!("telemetry channel 0x{id:x} missing from the test dictionary"),
    }
}

/// TLM packet: `[0x0001 u16]` + N x `[id u32][time 11B][value]` ->
/// [(id, value bytes)]. Entries have no length prefix; sizes come from the
/// dictionary above.
fn parse_tlm_packet(payload: &[u8]) -> Vec<(u32, Vec<u8>)> {
    if payload.len() < 2 || payload[0..2] != [0x00, 0x01] {
        return Vec::new();
    }
    let mut entries = Vec::new();
    let mut at = 2;
    while at < payload.len() {
        assert!(payload.len() >= at + 15, "truncated tlm entry header");
        let id = u32::from_be_bytes([
            payload[at],
            payload[at + 1],
            payload[at + 2],
            payload[at + 3],
        ]);
        let start = at + 4 + 11; // id + time tag
        let size = tlm_value_size(id, &payload[start..]);
        assert!(payload.len() >= start + size, "truncated tlm value");
        entries.push((id, payload[start..start + size].to_vec()));
        at = start + size;
    }
    entries
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// NO_OP command round trip: framed uplink in, LOG-packet downlink frames
/// out (OpCodeDispatched, NoOpReceived, OpCodeCompleted — the completed
/// event IS the observable command response flow).
#[test]
fn no_op_command_round_trip() {
    let harness = Harness::up();
    let no_op = CMD_DISPATCHER_BASE_ID + CmdDispatcher::OPCODE_CMD_NO_OP;
    harness.inject(&build_command_frame(no_op, &[]));

    let dispatched = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_DISPATCHED;
    let no_op_received = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_NO_OP_RECEIVED;
    let completed = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_COMPLETED;
    assert!(
        wait_until(DEADLINE, || {
            let ids: Vec<u32> = harness.log_events().iter().map(|(id, _)| *id).collect();
            [dispatched, no_op_received, completed]
                .iter()
                .all(|want| ids.contains(want))
        }),
        "expected NO_OP event packets; got {:?}",
        harness.log_events()
    );

    // Event argument bytes: OpCodeDispatched(opcode u32, port i32) and
    // OpCodeCompleted(opcode u32) carry the absolute opcode.
    let events = harness.log_events();
    let (_, dispatched_args) = events.iter().find(|(id, _)| *id == dispatched).unwrap();
    assert_eq!(dispatched_args[0..4], no_op.to_be_bytes());
    let (_, completed_args) = events.iter().find(|(id, _)| *id == completed).unwrap();
    assert_eq!(completed_args[..], no_op.to_be_bytes());

    // The uplink ownership chain returned the injected buffer to the
    // driver.
    assert_eq!(harness.loopback.returned_buffers.lock().unwrap().len(), 1);
    harness.topology.teardown();
}

/// TOGGLE + manual rate-group ticks: SignalValue telemetry frames appear
/// on the downlink (queued-component drain + TlmChan run + comms chain).
#[test]
fn telemetry_downlinks_after_ticks() {
    let harness = Harness::up();
    let toggle = SIGNAL_GEN_BASE_ID + signal_gen::OPCODE_TOGGLE;
    harness.inject(&build_command_frame(toggle, &[]));

    // Tick until the SignalValue channel shows up in a downlinked TLM
    // packet. Each tick: signal_gen.sched (drain + sample) then
    // tlm_chan.run then comQueue.run on the rate-group-1 thread.
    let signal_value = SIGNAL_GEN_BASE_ID + signal_gen::CHANID_SIGNAL_VALUE;
    let signal_type = SIGNAL_GEN_BASE_ID + signal_gen::CHANID_SIGNAL_TYPE;
    let seen = harness.tick_until(|| {
        tlm_values(&harness)
            .iter()
            .any(|(id, _)| *id == signal_value)
    });
    assert!(
        seen,
        "expected SignalValue telemetry; got {:?}",
        tlm_values(&harness)
    );
    // The type channel rides along; the default settings make every sample
    // amplitude * sin(...) with amplitude 0 => value bytes are 0.0f32.
    let values = tlm_values(&harness);
    assert!(values.iter().any(|(id, _)| *id == signal_type));
    let (_, value_bytes) = values.iter().find(|(id, _)| *id == signal_value).unwrap();
    assert_eq!(
        f32::from_be_bytes([
            value_bytes[0],
            value_bytes[1],
            value_bytes[2],
            value_bytes[3]
        ]),
        0.0
    );
    // And the TOGGLE command completed on the downlink.
    let completed = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_COMPLETED;
    assert!(
        harness
            .log_events()
            .iter()
            .any(|(id, args)| *id == completed && args[..] == toggle.to_be_bytes())
    );
    harness.topology.teardown();
}

/// An unregistered opcode yields the InvalidCommand event and no dispatch.
#[test]
fn unknown_opcode_yields_invalid_command_and_no_dispatch() {
    let harness = Harness::up();
    let bogus: u32 = 0x0BAD_0000;
    harness.inject(&build_command_frame(bogus, &[]));

    let invalid = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_INVALID_COMMAND;
    assert!(
        wait_until(DEADLINE, || {
            harness
                .log_events()
                .iter()
                .any(|(id, args)| *id == invalid && args[..] == bogus.to_be_bytes())
        }),
        "expected InvalidCommand event; got {:?}",
        harness.log_events()
    );
    // No dispatch happened: no OpCodeDispatched (or Completed) packet ever
    // downlinked in this topology instance.
    let dispatched = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_DISPATCHED;
    let completed = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_COMPLETED;
    assert!(
        harness
            .log_events()
            .iter()
            .all(|(id, _)| *id != dispatched && *id != completed)
    );
    harness.topology.teardown();
}
