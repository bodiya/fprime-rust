//! The Ref wired with `ComCcsds` framing (`--framing ccsds`), in process:
//! a TC transfer frame carrying a space packet with a NO_OP command goes up
//! through `frameAccumulator` (CCSDS TC frame detector) -> `tcDeframer` ->
//! `spacePacketDeframer` -> `fprimeRouter`, and the command's events come
//! back down as space packets in fixed-size TM transfer frames
//! (`comQueue` -> `spacePacketFramer` -> `aggregator` -> `tmFramer`; the
//! aggregator packs several packets per frame and rate group 1 flushes it).
//!
//! Wire formats (CCSDS 232.0 TC, 132.0 TM, 133.0 Space Packet), big endian:
//! TC frame = `[0x2000 | scid u16][vcid << 10 | (len - 1) u16][seq u8]
//! [space packet][CRC-16/CCITT-FALSE u16]`; TM frame = 1024 bytes,
//! `[scid << 4 | vcid << 1 u16][mc u8][vc u8][0x1800 | fhp u16][data field]
//! [CRC u16]`; space packet = `[type << 12 | apid u16][0xC000 | count u16]
//! [len - 1 u16][F Prime packet]`.

mod common;

use common::{DEADLINE, Harness, parse_log_packet, scratch_data_dir};

use fprime_ref::topology::{
    APID_MANAGER_BASE_ID, CMD_DISPATCHER_BASE_ID, Framing, TC_DEFRAMER_BASE_ID, TopologyConfig,
};
use fprime_svc::ccsds::crc16::Crc16;
use fprime_svc::ccsds::types::{SPACECRAFT_ID, TM_FRAME_FIXED_SIZE};
use fprime_svc::cmd_dispatcher::CmdDispatcher;

/// The GDS's default virtual channel (`--vcid 1`) and the `FrameContext`
/// default the Ref frames with.
const VCID: u16 = 1;

fn ccsds_harness() -> Harness {
    Harness::up_with(TopologyConfig {
        data_dir: Some(scratch_data_dir()),
        framing: Framing::Ccsds,
        ..TopologyConfig::default()
    })
}

/// A TC frame (bypass, data) around a telecommand space packet carrying
/// `fprime_packet`, exactly as fprime-gds's `space-packet-space-data-link`
/// framer builds it.
fn tc_frame(fprime_packet: &[u8], apid: u16, count: u16) -> Vec<u8> {
    let mut packet = Vec::new();
    packet.extend_from_slice(&((1u16 << 12) | apid).to_be_bytes());
    packet.extend_from_slice(&(0xC000 | count).to_be_bytes());
    packet.extend_from_slice(&(fprime_packet.len() as u16 - 1).to_be_bytes());
    packet.extend_from_slice(fprime_packet);
    let length = 5 + packet.len() + 2;
    let mut frame = Vec::new();
    frame.extend_from_slice(&((1u16 << 13) | SPACECRAFT_ID).to_be_bytes());
    frame.extend_from_slice(&((VCID << 10) | (length as u16 - 1)).to_be_bytes());
    frame.push(0);
    frame.extend_from_slice(&packet);
    let crc = Crc16::compute(&frame);
    frame.extend_from_slice(&crc.to_be_bytes());
    frame
}

/// The F Prime packets in one TM frame's data field (idle packets
/// skipped), after checking the frame's header and CRC.
fn tm_packets(frame: &[u8]) -> Vec<Vec<u8>> {
    assert_eq!(frame.len(), TM_FRAME_FIXED_SIZE, "TM frames are fixed size");
    let gvcid = u16::from_be_bytes([frame[0], frame[1]]);
    assert_eq!((gvcid >> 4) & 0x3FF, SPACECRAFT_ID);
    assert_eq!((gvcid >> 1) & 0x7, VCID);
    let status = u16::from_be_bytes([frame[4], frame[5]]);
    assert_eq!(
        status & 0x07FF,
        0,
        "no spanning: the first packet starts the data field"
    );
    let crc_at = TM_FRAME_FIXED_SIZE - 2;
    let crc = u16::from_be_bytes([frame[crc_at], frame[crc_at + 1]]);
    assert_eq!(crc, Crc16::compute(&frame[..crc_at]));
    let mut packets = Vec::new();
    let mut at = 6;
    while at + 6 <= crc_at {
        let id = u16::from_be_bytes([frame[at], frame[at + 1]]);
        let len = usize::from(u16::from_be_bytes([frame[at + 4], frame[at + 5]])) + 1;
        assert_eq!(id >> 12, 0, "telemetry space packets, version 0");
        if id & 0x7FF != 0x7FF {
            packets.push(frame[at + 6..at + 6 + len].to_vec());
        }
        at += 6 + len;
    }
    assert_eq!(at, crc_at, "the idle packet fills the data field");
    packets
}

fn downlinked_events(h: &Harness) -> Vec<(u32, Vec<u8>)> {
    h.frames()
        .iter()
        .flat_map(|frame| tm_packets(frame))
        .filter_map(|packet| parse_log_packet(&packet))
        .collect()
}

fn command_packet(opcode: u32) -> Vec<u8> {
    let mut packet = vec![0x00, 0x00]; // FW_PACKET_COMMAND
    packet.extend_from_slice(&opcode.to_be_bytes());
    packet
}

#[test]
fn ccsds_no_op_round_trip() {
    let h = ccsds_harness();
    let opcode = CMD_DISPATCHER_BASE_ID + CmdDispatcher::OPCODE_CMD_NO_OP;
    // Two commands with consecutive sequence counts, the second split
    // across two driver reads so the frame accumulator has to join them.
    h.inject(&tc_frame(&command_packet(opcode), 0, 0));
    let second = tc_frame(&command_packet(opcode), 0, 1);
    h.inject(&second[..7]);
    h.inject(&second[7..]);
    let no_op = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_NO_OP_RECEIVED;
    let completed = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_OP_CODE_COMPLETED;
    assert!(
        h.tick_until(|| {
            let events = downlinked_events(&h);
            events.iter().filter(|(id, _)| *id == no_op).count() == 2
                && events.iter().filter(|(id, _)| *id == completed).count() == 2
        }),
        "two NO_OPs through TC frames did not come back down in TM frames within {DEADLINE:?}"
    );
    // The aggregator packed several packets into at least one frame.
    assert!(
        h.frames().iter().any(|frame| tm_packets(frame).len() > 1),
        "no TM frame carried more than one packet"
    );
    // Neither the deframers nor the sequence-count check complained.
    let events = downlinked_events(&h);
    for (id, _) in &events {
        assert!(
            !(TC_DEFRAMER_BASE_ID..TC_DEFRAMER_BASE_ID + 0x1000).contains(id)
                && !(APID_MANAGER_BASE_ID..APID_MANAGER_BASE_ID + 0x1000).contains(id),
            "unexpected CCSDS event {id:#x}"
        );
    }
    h.topology.request_stop();
    h.topology.teardown();
}

#[test]
fn ccsds_corrupt_frame_is_skipped_and_the_next_one_accepted() {
    let h = ccsds_harness();
    let opcode = CMD_DISPATCHER_BASE_ID + CmdDispatcher::OPCODE_CMD_NO_OP;
    // The CCSDS TC frame detector checks the CRC itself (upstream
    // `CcsdsTcFrameDetector`), so a corrupt frame never reaches the
    // deframer: the accumulator discards it byte by byte and resynchronizes
    // on the next good frame.
    let mut bad = tc_frame(&command_packet(opcode), 0, 0);
    let last = bad.len() - 1;
    bad[last] ^= 0xFF;
    h.inject(&bad);
    h.inject(&tc_frame(&command_packet(opcode), 0, 0));
    let no_op = CMD_DISPATCHER_BASE_ID + CmdDispatcher::EVENTID_NO_OP_RECEIVED;
    assert!(
        h.tick_until(|| downlinked_events(&h).iter().any(|(id, _)| *id == no_op)),
        "the good frame after a corrupt one was not accepted"
    );
    // Give a (wrongly) accepted corrupt frame time to show up too.
    for _ in 0..10 {
        h.tick();
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    let n = downlinked_events(&h)
        .iter()
        .filter(|(id, _)| *id == no_op)
        .count();
    assert_eq!(n, 1, "exactly the good frame's NO_OP executes");
    h.topology.request_stop();
    h.topology.teardown();
}
