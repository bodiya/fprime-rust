//! End-to-end `ComCcsdsSdls`-style chains (ROADMAP item 12), uplink and
//! downlink, wired exactly as upstream's `Svc/Subtopologies/ComCcsdsSdls`
//! (`ComCcsds.SpacePacketFraming` + `ComCcsds.TmTcFraming` +
//! `SdlsEncryption` + `SdlsDecryption` + `ComCcsds.comStub`):
//!
//! ```text
//! down: comQueue -> spacePacketFramer -> aggregator -> sdlsFramer -> tmFramer -> comStub -> driver
//!                                                      |  encryptionSaRouter -> encryptor
//! up:   driver -> comStub -> frameAccumulator(CcsdsTcFrameDetector) -> tcDeframer
//!         -> sdlsDeframer -> spacePacketDeframer -> fprimeRouter -> commandOut
//!            |  decryptionSaRouter -> decryptor
//! ```
//!
//! The plain `ComCcsds` chain (no SDLS layer) runs in the Rust Ref under
//! `--framing ccsds` (`crates/fprime-ref/tests/ccsds_framing_test.rs`) and
//! is exercised here too. The crypto is the default `ClearTextEncryptor` /
//! `ClearTextDecryptor` pair, and — with the `aes-gcm` feature — the
//! `AesGcmEncryptor` / `AesGcmDecryptor` pair keyed by an
//! `SdlsFileKeyManager`; the ground side of the AES chain is built and
//! checked with the `aes-gcm` crate directly, independent of the
//! components. Active components (`comQueue`, `aggregator`) are driven by
//! dispatching their queues on the test thread.

use fprime_comp::{
    BufferSendPort, ComDataWithContextPort, ComPort, ComponentDispatch, PortRef, PrmGetPort,
};
use fprime_config::sdls::SA_ROUTER_PORT_PLAINTEXT;
use fprime_config::{FwIndexType, FwPrmIdType, FwSizeType};
use fprime_fw::{Buffer, ComBuffer, FrameContext, ParamBuffer, ParamValid, SerBuf};
use fprime_svc::buffer_manager::{BufferBin, BufferManager};
use fprime_svc::ccsds::apid_manager::ApidManager;
use fprime_svc::ccsds::ccsds_sdls_deframer::CcsdsSdlsDeframer;
use fprime_svc::ccsds::ccsds_sdls_framer::CcsdsSdlsFramer;
use fprime_svc::ccsds::clear_text_decryptor::ClearTextDecryptor;
use fprime_svc::ccsds::clear_text_encryptor::ClearTextEncryptor;
use fprime_svc::ccsds::crc16::Crc16;
use fprime_svc::ccsds::sdls_sa_router::SdlsSaRouter;
use fprime_svc::ccsds::space_packet_deframer::SpacePacketDeframer;
use fprime_svc::ccsds::space_packet_framer::SpacePacketFramer;
use fprime_svc::ccsds::tc_deframer::TcDeframer;
use fprime_svc::ccsds::tm_framer::TmFramer;
use fprime_svc::ccsds::types::{CcsdsSdlsEncryptionPort, SPACECRAFT_ID, TM_FRAME_FIXED_SIZE};
use fprime_svc::com_aggregator::ComAggregator;
use fprime_svc::com_queue::{ComQueue, QueueConfigurationTable};
use fprime_svc::com_stub::{ByteStreamSendPort, ByteStreamStatus, ComStub};
use fprime_svc::fprime_router::FprimeRouter;
use fprime_svc::frame_accumulator::{CcsdsTcFrameDetector, FrameAccumulator};
use std::sync::{Arc, Mutex};

/// The security association both directions use (`SdlsCfg.SaMap` routes
/// SA 1 to the plain-text port; `CcsdsSdlsFramer.SA_INDEX` defaults to 1).
const SA: u16 = 1;
/// The `FrameContext` default virtual channel.
const VCID: u8 = 1;

// ---------------------------------------------------------------------------
// The ground/driver side
// ---------------------------------------------------------------------------

#[derive(Default)]
struct Ground {
    /// Bytes the comStub sent to the driver (TM frames).
    downlink: Mutex<Vec<Vec<u8>>>,
    /// Driver receive buffers returned by the comStub.
    receive_returned: Mutex<usize>,
    /// Command packets the router sent to the command dispatcher.
    commands: Mutex<Vec<Vec<u8>>>,
}

impl ByteStreamSendPort for Ground {
    fn invoke(&self, _p: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus {
        self.downlink.lock().unwrap().push(buffer.data().to_vec());
        ByteStreamStatus::OpOk
    }
}

impl BufferSendPort for Ground {
    fn invoke(&self, _p: FwIndexType, _buffer: Buffer) {
        *self.receive_returned.lock().unwrap() += 1;
    }
}

impl ComPort for Ground {
    fn invoke(&self, _p: FwIndexType, data: &mut ComBuffer, _context: u32) {
        self.commands.lock().unwrap().push(data.as_slice().to_vec());
    }
}

/// A parameter database with no stored values: `SA_INDEX` takes its FPP
/// default.
struct NoParams;

impl PrmGetPort for NoParams {
    fn invoke(&self, _p: FwIndexType, _id: FwPrmIdType, _val: &mut ParamBuffer) -> ParamValid {
        ParamValid::Invalid
    }
}

// ---------------------------------------------------------------------------
// The flight side
// ---------------------------------------------------------------------------

/// The crypto pair connected to the routers' `PLAINTEXT` port.
struct Crypto {
    encrypt_in: PortRef<dyn CcsdsSdlsEncryptionPort>,
    encrypt_return_in: PortRef<dyn ComDataWithContextPort>,
    decrypt_in: PortRef<dyn CcsdsSdlsEncryptionPort>,
    decrypt_return_in: PortRef<dyn ComDataWithContextPort>,
}

struct Chain {
    com_queue: Arc<ComQueue>,
    aggregator: Arc<ComAggregator>,
    com_stub: Arc<ComStub>,
    encryption_router: Arc<SdlsSaRouter>,
    decryption_router: Arc<SdlsSaRouter>,
    ground: Arc<Ground>,
}

/// Wire the chain. `connect_crypto` connects the crypto pair's own output
/// ports to the two routers and returns its input ports.
fn build(connect_crypto: impl FnOnce(&Arc<SdlsSaRouter>, &Arc<SdlsSaRouter>) -> Crypto) -> Chain {
    let ground = Arc::new(Ground::default());

    let com_queue = ComQueue::new("comQueue");
    let buffer_manager = BufferManager::new("commsBufferManager");
    let space_packet_framer = SpacePacketFramer::new("spacePacketFramer");
    let apid_manager = ApidManager::new("apidManager");
    let aggregator = ComAggregator::new("aggregator");
    let sdls_framer = CcsdsSdlsFramer::new("sdlsFramer");
    let encryption_router = SdlsSaRouter::new("encryptionSaRouter");
    let framer = TmFramer::new("framer");
    let com_stub = ComStub::new("comStub");
    let frame_accumulator = FrameAccumulator::new("frameAccumulator");
    let tc_deframer = TcDeframer::new("tcDeframer");
    let sdls_deframer = CcsdsSdlsDeframer::new("sdlsDeframer");
    let decryption_router = SdlsSaRouter::new("decryptionSaRouter");
    let space_packet_deframer = SpacePacketDeframer::new("spacePacketDeframer");
    let router = FprimeRouter::new("fprimeRouter");

    // ComCcsds.SpacePacketFraming.Downlink
    com_queue
        .data_out
        .connect_to(space_packet_framer.data_in(0));
    space_packet_framer
        .data_return_out
        .connect_to(com_queue.data_return_in(0));
    space_packet_framer
        .buffer_allocate
        .connect_to(buffer_manager.buffer_get_callee_in(0));
    space_packet_framer
        .buffer_deallocate
        .connect_to(buffer_manager.buffer_send_in(0));
    space_packet_framer
        .get_apid_seq_count
        .connect_to(apid_manager.get_apid_seq_count_in(0));
    space_packet_framer
        .data_out
        .connect_to(aggregator.data_in(0));
    aggregator
        .data_return_out
        .connect_to(space_packet_framer.data_return_in(0));
    aggregator
        .com_status_out
        .connect_to(space_packet_framer.com_status_in(0));
    space_packet_framer
        .com_status_out
        .connect_to(com_queue.com_status_in(0));
    // ComCcsdsSdls.FramingSubtopology.Downlink
    aggregator.data_out.connect_to(sdls_framer.data_in(0));
    sdls_framer
        .data_return_out
        .connect_to(aggregator.data_return_in(0));
    sdls_framer.data_out.connect_to(framer.data_in(0));
    framer
        .data_return_out
        .connect_to(sdls_framer.data_return_in(0));
    sdls_framer
        .buffer_allocate
        .connect_to(buffer_manager.buffer_get_callee_in(0));
    sdls_framer
        .buffer_deallocate
        .connect_to(buffer_manager.buffer_send_in(0));
    framer
        .com_status_out
        .connect_to(sdls_framer.com_status_in(0));
    sdls_framer
        .com_status_out
        .connect_to(aggregator.com_status_in(0));
    sdls_framer.prm.prm_get_out.connect(Arc::new(NoParams), 0);
    // ComCcsdsSdls.SdlsEncryption.Encryption (router side)
    sdls_framer
        .encrypt_out
        .connect_to(encryption_router.data_in(0));
    encryption_router
        .data_out
        .connect_to(sdls_framer.encrypt_in(0));
    sdls_framer
        .encrypt_return_out
        .connect_to(encryption_router.data_return_in(0));
    encryption_router
        .buffer_return_out
        .connect_to(sdls_framer.buffer_return_in(0));
    // ComCcsdsSdls.SdlsDecryption.Decryption (router side)
    sdls_deframer
        .decrypt_out
        .connect_to(decryption_router.data_in(0));
    decryption_router
        .data_out
        .connect_to(sdls_deframer.decrypt_in(0));
    sdls_deframer
        .decrypt_return_out
        .connect_to(decryption_router.data_return_in(0));
    decryption_router
        .buffer_return_out
        .connect_to(sdls_deframer.buffer_return_in(0));
    // The crypto pair on the routers' PLAINTEXT ports.
    let crypto = connect_crypto(&encryption_router, &decryption_router);
    let pt = SA_ROUTER_PORT_PLAINTEXT as usize;
    encryption_router.sa_data_out[pt].connect_to(crypto.encrypt_in);
    encryption_router.sa_data_return_out[pt].connect_to(crypto.encrypt_return_in);
    decryption_router.sa_data_out[pt].connect_to(crypto.decrypt_in);
    decryption_router.sa_data_return_out[pt].connect_to(crypto.decrypt_return_in);
    // ComCcsdsSdls.Subtopology.ComStub
    framer.data_out.connect_to(com_stub.data_in(0));
    com_stub
        .data_return_out
        .connect_to(framer.data_return_in(0));
    com_stub.com_status_out.connect_to(framer.com_status_in(0));
    com_stub.data_out.connect_to(frame_accumulator.data_in(0));
    frame_accumulator
        .data_return_out
        .connect_to(com_stub.data_return_in(0));
    com_stub.drv_send_out.connect(ground.clone(), 0);
    com_stub.drv_receive_return_out.connect(ground.clone(), 0);
    // ComCcsdsSdls.FramingSubtopology.Uplink + TmTcFraming.Uplink
    frame_accumulator
        .buffer_allocate
        .connect_to(buffer_manager.buffer_get_callee_in(0));
    frame_accumulator
        .buffer_deallocate
        .connect_to(buffer_manager.buffer_send_in(0));
    frame_accumulator
        .data_out
        .connect_to(tc_deframer.data_in(0));
    tc_deframer
        .data_return_out
        .connect_to(frame_accumulator.data_return_in(0));
    tc_deframer.data_out.connect_to(sdls_deframer.data_in(0));
    sdls_deframer
        .data_return_out
        .connect_to(tc_deframer.data_return_in(0));
    sdls_deframer
        .data_out
        .connect_to(space_packet_deframer.data_in(0));
    space_packet_deframer
        .data_return_out
        .connect_to(sdls_deframer.data_return_in(0));
    // ComCcsds.SpacePacketFraming.Uplink
    space_packet_deframer
        .validate_apid_seq_count
        .connect_to(apid_manager.validate_apid_seq_count_in(0));
    space_packet_deframer.data_out.connect_to(router.data_in(0));
    router
        .data_return_out
        .connect_to(space_packet_deframer.data_return_in(0));
    router.command_out.connect(ground.clone(), 0);

    // configComponents (ComCcsdsConfig values).
    buffer_manager.setup(
        200,
        &[BufferBin {
            buffer_size: 2048,
            num_buffers: 20,
        }],
    );
    frame_accumulator.configure(Box::new(CcsdsTcFrameDetector::new()), 2048);
    let mut table = QueueConfigurationTable::default();
    table.entries[0].depth = 200;
    table.entries[0].priority = 0;
    table.entries[1].depth = 500;
    table.entries[1].priority = 2;
    table.entries[2].depth = 100;
    table.entries[2].priority = 1;
    com_queue.configure(&table);
    com_queue
        .active
        .queued
        .create_queue(50, fprime_svc::com_queue::MSG_SIZE);
    aggregator.configure(false);
    aggregator.create_queue(10);
    sdls_framer.load_parameters();

    Chain {
        com_queue,
        aggregator,
        com_stub,
        encryption_router,
        decryption_router,
        ground,
    }
}

impl Chain {
    /// Dispatch the active components' queues until both are empty.
    fn pump(&self) {
        loop {
            let queued = self
                .com_queue
                .active
                .queued
                .queue()
                .get_messages_available()
                + self
                    .aggregator
                    .active
                    .queued
                    .queue()
                    .get_messages_available();
            if queued == 0 {
                return;
            }
            let _ = self
                .com_queue
                .active
                .queued
                .dispatch_available_messages(&*self.com_queue);
            let _ = self
                .aggregator
                .active
                .queued
                .dispatch_available_messages(&*self.aggregator);
        }
    }

    /// Task start-up (the aggregator preamble) and the driver connecting.
    fn start(&self) {
        self.aggregator.preamble();
        let p = self.com_stub.drv_connected(0);
        p.target.invoke(p.port_num);
        self.pump();
    }

    /// Queue an event packet (`[FW_PACKET_LOG u16][payload]`).
    fn send_event(&self, payload: &[u8]) {
        let mut com = ComBuffer::new();
        assert!(com.serialize_u16_be(0x0002).is_ok());
        for &b in payload {
            assert!(com.serialize_u8_be(b).is_ok());
        }
        let p = self.com_queue.com_packet_queue_in(0);
        p.target.invoke(p.port_num, &mut com, 0);
        self.pump();
    }

    /// The rate-group flush of the aggregate.
    fn timeout(&self) {
        let p = self.aggregator.timeout(0);
        p.target.invoke(p.port_num, 0);
        self.pump();
    }

    /// Bytes arriving from the driver.
    fn uplink(&self, bytes: &[u8]) {
        let mut b = Buffer::allocate(bytes.len());
        b.data_mut().copy_from_slice(bytes);
        let p = self.com_stub.drv_receive_in(0);
        p.target.invoke(p.port_num, b, ByteStreamStatus::OpOk);
        self.pump();
    }
}

// ---------------------------------------------------------------------------
// Ground-side encoding/decoding (independent of the components)
// ---------------------------------------------------------------------------

/// Check a TM frame (header, CRC) and return its data field.
fn tm_data_field(frame: &[u8]) -> &[u8] {
    assert_eq!(frame.len(), TM_FRAME_FIXED_SIZE);
    let gvcid = u16::from_be_bytes([frame[0], frame[1]]);
    assert_eq!((gvcid >> 4) & 0x3FF, SPACECRAFT_ID);
    assert_eq!((gvcid >> 1) & 0x7, u16::from(VCID));
    assert_eq!(u16::from_be_bytes([frame[4], frame[5]]), 0x1800);
    assert_eq!(Crc16::compute(frame), 0, "FECF");
    &frame[6..TM_FRAME_FIXED_SIZE - 2]
}

/// Split an SDLS frame into its SA index and payload.
fn sdls_split(field: &[u8]) -> (u16, &[u8]) {
    (u16::from_be_bytes([field[0], field[1]]), &field[2..])
}

/// The non-idle space packets at the front of `bytes`: (APID, sequence
/// count, data field). Stops at the idle packet or the end.
fn space_packets(bytes: &[u8]) -> Vec<(u16, u16, Vec<u8>)> {
    let mut out = Vec::new();
    let mut at = 0;
    while at + 6 <= bytes.len() {
        let id = u16::from_be_bytes([bytes[at], bytes[at + 1]]);
        let seq = u16::from_be_bytes([bytes[at + 2], bytes[at + 3]]);
        let len = usize::from(u16::from_be_bytes([bytes[at + 4], bytes[at + 5]])) + 1;
        if id & 0x7FF == 0x7FF {
            break;
        }
        out.push((
            id & 0x7FF,
            seq & 0x3FFF,
            bytes[at + 6..at + 6 + len].to_vec(),
        ));
        at += 6 + len;
    }
    out
}

/// A TC frame (bypass, data) around `data_field`.
fn tc_frame(data_field: &[u8]) -> Vec<u8> {
    let length = 5 + data_field.len() + 2;
    let mut frame = Vec::new();
    frame.extend_from_slice(&((1u16 << 13) | SPACECRAFT_ID).to_be_bytes());
    frame.extend_from_slice(&((u16::from(VCID) << 10) | (length as u16 - 1)).to_be_bytes());
    frame.push(0);
    frame.extend_from_slice(data_field);
    let crc = Crc16::compute(&frame);
    frame.extend_from_slice(&crc.to_be_bytes());
    frame
}

/// A telecommand space packet (APID 0, FW_PACKET_COMMAND) carrying a
/// `CMD_NO_OP`-shaped F Prime command packet.
fn command_space_packet(opcode: u32, count: u16) -> (Vec<u8>, Vec<u8>) {
    let mut fprime = vec![0x00, 0x00];
    fprime.extend_from_slice(&opcode.to_be_bytes());
    let mut packet = Vec::new();
    packet.extend_from_slice(&(1u16 << 12).to_be_bytes());
    packet.extend_from_slice(&(0xC000 | count).to_be_bytes());
    packet.extend_from_slice(&(fprime.len() as u16 - 1).to_be_bytes());
    packet.extend_from_slice(&fprime);
    (packet, fprime)
}

/// The events this test sends, by index.
fn event_payload(i: u8) -> Vec<u8> {
    (0..(20 + i)).map(|b| b ^ i).collect()
}

/// Check the downlinked aggregate: every event packet, in order, as
/// FW_PACKET_LOG space packets with consecutive sequence counts, followed
/// by the idle fill.
fn check_aggregate(aggregate: &[u8], first: u8, count: u8) {
    let packets = space_packets(aggregate);
    assert_eq!(
        packets.len(),
        usize::from(count),
        "packets in one aggregate"
    );
    for (k, (apid, seq, data)) in packets.iter().enumerate() {
        let i = first + k as u8;
        assert_eq!(*apid, 0x0002);
        assert_eq!(*seq, u16::from(i));
        let mut expected = vec![0x00, 0x02];
        expected.extend_from_slice(&event_payload(i));
        assert_eq!(*data, expected);
    }
}

/// The shared scenario: three events aggregated into one SDLS-protected
/// TM frame, flushed by the timeout; then two more in a second frame; and
/// two TC commands up. `open_down` turns the SDLS payload (followed by the
/// TM framer's idle fill) into the aggregate of the given length — the
/// ground knows the protected length out of band, as the TM data field
/// carries no SDLS length; `seal_up` builds the SDLS payload of a TC frame.
fn run_scenario(
    chain: &Chain,
    open_down: impl Fn(&[u8], usize) -> Vec<u8>,
    seal_up: impl Fn(&[u8]) -> Vec<u8>,
) {
    chain.start();
    assert!(chain.ground.downlink.lock().unwrap().is_empty());

    // Downlink: three packets share one frame, flushed on timeout.
    for i in 0..3 {
        chain.send_event(&event_payload(i));
    }
    assert!(
        chain.ground.downlink.lock().unwrap().is_empty(),
        "held until the flush"
    );
    chain.timeout();
    // And two more after the first frame came back.
    for i in 3..5 {
        chain.send_event(&event_payload(i));
    }
    chain.timeout();
    let frames = chain.ground.downlink.lock().unwrap().clone();
    assert_eq!(frames.len(), 2);
    for (frame, (first, count)) in frames.iter().zip([(0u8, 3u8), (3, 2)]) {
        let (sa, payload) = sdls_split(tm_data_field(frame));
        assert_eq!(sa, SA);
        let aggregate_len: usize = (first..first + count)
            .map(|i| 6 + 2 + event_payload(i).len())
            .sum();
        check_aggregate(&open_down(payload, aggregate_len), first, count);
    }

    // Uplink: two commands, the second split across two driver reads.
    let opcode = 0x0100_0000;
    for count in 0..2u16 {
        let (packet, _) = command_space_packet(opcode, count);
        let mut field = SA.to_be_bytes().to_vec();
        field.extend_from_slice(&seal_up(&packet));
        let frame = tc_frame(&field);
        if count == 0 {
            chain.uplink(&frame);
        } else {
            chain.uplink(&frame[..9]);
            chain.uplink(&frame[9..]);
        }
    }
    let (_, fprime) = command_space_packet(opcode, 0);
    assert_eq!(
        *chain.ground.commands.lock().unwrap(),
        vec![fprime.clone(), fprime]
    );
    assert_eq!(*chain.ground.receive_returned.lock().unwrap(), 3);

    // Every buffer found its way home: nothing is still tracked by the
    // routers, and the downlink is idle and ready again.
    assert_eq!(chain.encryption_router.outstanding_count(), 0);
    assert_eq!(chain.decryption_router.outstanding_count(), 0);
    assert!(chain.aggregator.owns_aggregate());
    assert_eq!(chain.aggregator.aggregate_size(), 0);
}

fn clear_text(enc: &Arc<SdlsSaRouter>, dec: &Arc<SdlsSaRouter>) -> Crypto {
    let encryptor = ClearTextEncryptor::new("encryptor");
    let decryptor = ClearTextDecryptor::new("decryptor");
    let pt = SA_ROUTER_PORT_PLAINTEXT;
    encryptor.encrypt_out.connect_to(enc.sa_data_in(pt));
    encryptor
        .buffer_return_out
        .connect_to(enc.sa_buffer_return_in(pt));
    decryptor.decrypt_out.connect_to(dec.sa_data_in(pt));
    decryptor
        .buffer_return_out
        .connect_to(dec.sa_buffer_return_in(pt));
    Crypto {
        encrypt_in: encryptor.encrypt_in(0),
        encrypt_return_in: encryptor.encrypt_return_in(0),
        decrypt_in: decryptor.decrypt_in(0),
        decrypt_return_in: decryptor.decrypt_return_in(0),
    }
}

/// The default `ComCcsdsSdls` chain with the clear-text null ciphers.
#[test]
fn com_ccsds_sdls_clear_text_round_trip() {
    let chain = build(clear_text);
    run_scenario(
        &chain,
        |payload, len| payload[..len].to_vec(),
        |packet| packet.to_vec(),
    );
}

/// A frame for an SA the decryption router does not map is dropped with
/// `UNKNOWN_SA`, and its buffer still comes home.
#[test]
fn unknown_sa_uplink_is_dropped() {
    let chain = build(clear_text);
    chain.start();
    let (packet, _) = command_space_packet(0x0100_0000, 0);
    let mut field = 7u16.to_be_bytes().to_vec();
    field.extend_from_slice(&packet);
    chain.uplink(&tc_frame(&field));
    assert!(chain.ground.commands.lock().unwrap().is_empty());
    assert_eq!(chain.decryption_router.outstanding_count(), 0);
    assert_eq!(*chain.ground.receive_returned.lock().unwrap(), 1);
}

/// `ComCcsdsSdls` with the AES-256-GCM pair, keyed from a file (so
/// `std`-only: `SdlsFileKeyManager` reads the key file).
#[cfg(all(feature = "aes-gcm", feature = "std"))]
mod aes {
    use super::*;
    use aes_gcm::aead::AeadInPlace;
    use aes_gcm::{Aes256Gcm, KeyInit, Nonce, Tag};
    use fprime_svc::ccsds::aes_gcm_decryptor::AesGcmDecryptor;
    use fprime_svc::ccsds::aes_gcm_encryptor::AesGcmEncryptor;
    use fprime_svc::ccsds::sdls_file_key_manager::SdlsFileKeyManager;

    const KEY: [u8; 32] = [
        0x3C, 0x91, 0x05, 0x7E, 0xA2, 0x44, 0x18, 0xD0, 0x6B, 0x2F, 0xE9, 0x73, 0x0A, 0xC5, 0x5D,
        0x81, 0x96, 0x12, 0xFB, 0x48, 0x27, 0xB3, 0x6E, 0x09, 0xD4, 0x7A, 0x31, 0xE0, 0x5F, 0x8C,
        0xA9, 0x16,
    ];

    /// The SDLS AAD: masked primary header + SPI + zeroed IV field.
    fn aad(tm: bool) -> Vec<u8> {
        if tm {
            let mut a = vec![0u8; 20];
            a[1] = (VCID << 1) & 0x0E;
            a[6..8].copy_from_slice(&SA.to_be_bytes());
            a
        } else {
            let mut a = vec![0u8; 19];
            a[2] = (VCID << 2) & 0xFC;
            a[5..7].copy_from_slice(&SA.to_be_bytes());
            a
        }
    }

    fn open_down(payload: &[u8], len: usize) -> Vec<u8> {
        let (iv, rest) = payload.split_at(12);
        let (ct, rest) = rest.split_at(len);
        let tag = &rest[..16];
        let mut buf = ct.to_vec();
        Aes256Gcm::new_from_slice(&KEY)
            .unwrap()
            .decrypt_in_place_detached(
                Nonce::from_slice(iv),
                &aad(true),
                &mut buf,
                Tag::from_slice(tag),
            )
            .expect("the downlink frame authenticates");
        buf
    }

    fn seal_up(packet: &[u8]) -> Vec<u8> {
        let iv = [0x5Au8; 12];
        let mut buf = packet.to_vec();
        let tag = Aes256Gcm::new_from_slice(&KEY)
            .unwrap()
            .encrypt_in_place_detached(Nonce::from_slice(&iv), &aad(false), &mut buf)
            .unwrap();
        let mut out = iv.to_vec();
        out.extend_from_slice(&buf);
        out.extend_from_slice(&tag);
        out
    }

    fn key_file() -> String {
        let path = std::env::temp_dir().join(format!(
            "fprime_ccsds_sdls_chain_{}.key",
            std::process::id()
        ));
        std::fs::write(&path, KEY).unwrap();
        path.to_str().unwrap().to_string()
    }

    fn aes_gcm(enc: &Arc<SdlsSaRouter>, dec: &Arc<SdlsSaRouter>) -> Crypto {
        let keys = SdlsFileKeyManager::new("keyManager");
        keys.configure(&key_file(), 32 as FwSizeType);
        let encryptor = AesGcmEncryptor::new("encryptor");
        let decryptor = AesGcmDecryptor::new("decryptor");
        encryptor.key_get.connect_to(keys.key_get(0));
        decryptor.key_get.connect_to(keys.key_get(0));
        let pt = SA_ROUTER_PORT_PLAINTEXT;
        encryptor.encrypt_out.connect_to(enc.sa_data_in(pt));
        encryptor
            .buffer_return_out
            .connect_to(enc.sa_buffer_return_in(pt));
        decryptor.decrypt_out.connect_to(dec.sa_data_in(pt));
        decryptor
            .buffer_return_out
            .connect_to(dec.sa_buffer_return_in(pt));
        Crypto {
            encrypt_in: encryptor.encrypt_in(0),
            encrypt_return_in: encryptor.encrypt_return_in(0),
            decrypt_in: decryptor.decrypt_in(0),
            decrypt_return_in: decryptor.decrypt_return_in(0),
        }
    }

    #[test]
    fn com_ccsds_sdls_aes_gcm_round_trip() {
        let chain = build(aes_gcm);
        run_scenario(&chain, open_down, seal_up);
    }

    /// A tampered uplink frame fails its MAC check and is dropped; the
    /// buffer still comes home and the next good frame decrypts.
    #[test]
    fn tampered_uplink_is_rejected() {
        let chain = build(aes_gcm);
        chain.start();
        let (packet, fprime) = command_space_packet(0x0100_0000, 0);
        let mut field = SA.to_be_bytes().to_vec();
        field.extend_from_slice(&seal_up(&packet));
        let mut bad = field.clone();
        bad[2 + 12] ^= 0x01;
        chain.uplink(&tc_frame(&bad));
        assert!(chain.ground.commands.lock().unwrap().is_empty());
        chain.uplink(&tc_frame(&field));
        assert_eq!(*chain.ground.commands.lock().unwrap(), vec![fprime]);
        assert_eq!(chain.decryption_router.outstanding_count(), 0);
    }
}

/// The AOS alternative to the TM layer: an `AosFramer` frame stream decoded
/// by an `AosDeframer` recovers every packet, including one striped across
/// frames.
#[test]
fn aos_framer_to_deframer_round_trip() {
    use fprime_comp::BufferGetPort;
    use fprime_svc::ccsds::aos_deframer::AosDeframer;
    use fprime_svc::ccsds::aos_framer::AosFramer;
    use fprime_svc::ccsds::types::pvn_bitfield;

    struct Loop {
        deframer: Mutex<Option<Arc<AosDeframer>>>,
        framer: Mutex<Option<Arc<AosFramer>>>,
        packets: Mutex<Vec<Vec<u8>>>,
    }
    impl ComDataWithContextPort for Loop {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            match port_num {
                // A frame: copy it to the deframer, return it to the framer.
                0 => {
                    let mut copy = Buffer::allocate(data.size());
                    copy.data_mut().copy_from_slice(data.data());
                    let deframer = self.deframer.lock().unwrap().clone().unwrap();
                    let p = deframer.data_in(0);
                    p.target.invoke(p.port_num, copy, context);
                    let framer = self.framer.lock().unwrap().clone().unwrap();
                    let p = framer.data_return_in(0);
                    p.target.invoke(p.port_num, data, context);
                }
                // An extracted packet.
                1 => self.packets.lock().unwrap().push(data.data().to_vec()),
                _ => {}
            }
        }
    }
    impl BufferGetPort for Loop {
        fn invoke(&self, _p: FwIndexType, size: FwSizeType) -> Buffer {
            Buffer::allocate(size as usize)
        }
    }
    impl BufferSendPort for Loop {
        fn invoke(&self, _p: FwIndexType, _b: Buffer) {}
    }

    let l = Arc::new(Loop {
        deframer: Mutex::new(None),
        framer: Mutex::new(None),
        packets: Mutex::new(Vec::new()),
    });
    let framer = AosFramer::new("aosFramer");
    let deframer = AosDeframer::new("aosDeframer");
    framer.configure(256, true, SPACECRAFT_ID, VCID, pvn_bitfield::SPP_MASK);
    framer.data_out.connect(l.clone(), 0);
    framer.data_return_out.connect(l.clone(), 2);
    deframer.data_out.connect(l.clone(), 1);
    deframer.data_return_out.connect(l.clone(), 2);
    deframer.allocate.connect(l.clone(), 0);
    deframer.deallocate.connect(l.clone(), 0);
    deframer.configure(256, true, SPACECRAFT_ID, VCID, pvn_bitfield::VALID_MASK);
    *l.deframer.lock().unwrap() = Some(deframer);
    *l.framer.lock().unwrap() = Some(framer.clone());

    let spp = |apid: u16, len: usize, fill: u8| {
        let mut p = apid.to_be_bytes().to_vec();
        p.extend_from_slice(&0xC000u16.to_be_bytes());
        p.extend_from_slice(&(len as u16 - 1).to_be_bytes());
        p.extend(std::iter::repeat_n(fill, len));
        p
    };
    let packets = [spp(1, 30, 0xA1), spp(2, 500, 0xB2), spp(3, 12, 0xC3)];
    let context = FrameContext {
        vc_id: VCID,
        ..FrameContext::default()
    };
    for (i, packet) in packets.iter().enumerate() {
        let mut b = Buffer::allocate(packet.len());
        b.data_mut().copy_from_slice(packet);
        let ctx = FrameContext {
            send_now: i == packets.len() - 1,
            ..context
        };
        let p = framer.data_in(0);
        p.target.invoke(p.port_num, b, &ctx);
    }
    assert_eq!(*l.packets.lock().unwrap(), packets.to_vec());
}
