//! CfdpManager unit tests — ports of upstream's
//! `Svc/Ccsds/CfdpManager/test/ut` (`CfdpManagerTester`, `PduTester`, the
//! command, event, parameter, multi-channel and helper tests) and
//! `Types/test/ut/PduTests.cpp`.
//!
//! [`Harness`] plays the role of the generated `CfdpManagerGTestBase`: it
//! wires every output port of a component to a recording [`Ground`], and
//! gives the tests the friend access upstream's tester has (the engine
//! behind the component's lock).

mod commands;
mod events;
mod params;
mod pdus;
mod transactions;

use super::*;
use crate::ccsds::cfdp_manager::channel::Channel;
use crate::ccsds::cfdp_manager::pdu::peek_pdu_type;
use crate::ccsds::cfdp_manager::pdu::{
    AckPdu, EofPdu, FileDataPdu, FinPdu, MetadataPdu, NakPdu, Pdu, PduDirection, PduReader,
    SegmentRequest,
};
use crate::ccsds::cfdp_manager::transaction::TxnIdx;
use fprime_comp::queued::MsgDispatchStatus as DispatchStatus;
use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, PrmGetPort, PrmSetPort, TlmPort};
use fprime_config::cfdp::{MAX_COMMANDED_PLAYBACK_FILES_PER_CHAN, MAX_PDU_SIZE};
use fprime_fw::{ExtBuf, ParamBuffer, Time, TlmBuffer};
use fprime_utils::cfdp::Checksum;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

pub(super) const ID_BASE: FwIdType = 0x4000;
pub(super) const TEST_GROUND_EID: EntityId = 100;
pub(super) const CH0: u8 = 0;
pub(super) const CH1: u8 = 1;

static COUNTER: AtomicU32 = AtomicU32::new(0);

/// A fresh directory with a short absolute path (command file names are
/// `CmdStringArg`s, at most 40 characters).
pub(super) fn temp_dir() -> PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "cfdp{}_{}",
        std::process::id() % 100_000,
        COUNTER.fetch_add(1, Ordering::SeqCst)
    ));
    std::fs::create_dir_all(&path).unwrap();
    path
}

/// `dir/name` as a string, asserted to fit a command string argument.
pub(super) fn path_in(dir: &std::path::Path, name: &str) -> String {
    let p = dir.join(name).to_str().unwrap().to_string();
    assert!(p.len() <= 40, "test path too long for a CmdStringArg: {p}");
    p
}

/// A file of `size` bytes of the repeating 0..=255 pattern (upstream's
/// `createAndVerifyTestFile`); returns the contents.
pub(super) fn make_file(path: &str, size: usize) -> Vec<u8> {
    let data: Vec<u8> = (0..size).map(|i| i as u8).collect();
    std::fs::write(path, &data).unwrap();
    data
}

/// Everything the component sends out.
#[derive(Default)]
pub(super) struct Ground {
    pub events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
    pub tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
    pub cmd_responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
    pub data_out: Mutex<Vec<(FwIndexType, Vec<u8>)>>,
    pub deallocated: Mutex<Vec<(FwIndexType, usize)>>,
    pub data_in_returned: Mutex<Vec<(FwIndexType, usize)>>,
    pub file_done: Mutex<Vec<SendFileResponse>>,
    pub pings: Mutex<Vec<u32>>,
    pub prm_db: Mutex<HashMap<FwPrmIdType, Vec<u8>>>,
    pub prm_sets: Mutex<Vec<(FwPrmIdType, Vec<u8>)>>,
    pub fail_alloc: AtomicBool,
    pub allocations: AtomicU32,
}

impl Ground {
    pub fn clear(&self) {
        self.events.lock().unwrap().clear();
        self.tlm.lock().unwrap().clear();
        self.cmd_responses.lock().unwrap().clear();
        self.data_out.lock().unwrap().clear();
        self.deallocated.lock().unwrap().clear();
        self.data_in_returned.lock().unwrap().clear();
        self.file_done.lock().unwrap().clear();
        self.pings.lock().unwrap().clear();
    }
}

impl LogPort for Ground {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        _time_tag: &mut Time,
        severity: LogSeverity,
        args: &mut LogBuffer,
    ) {
        self.events
            .lock()
            .unwrap()
            .push((id - ID_BASE, severity, args.as_slice().to_vec()));
    }
}

impl TlmPort for Ground {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwChanIdType,
        _time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) {
        self.tlm
            .lock()
            .unwrap()
            .push((id - ID_BASE, val.as_slice().to_vec()));
    }
}

impl CmdResponsePort for Ground {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        self.cmd_responses
            .lock()
            .unwrap()
            .push((op_code - ID_BASE, cmd_seq, response));
    }
}

impl CmdRegPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, _op_code: FwOpcodeType) {}
}

impl PingPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, key: u32) {
        self.pings.lock().unwrap().push(key);
    }
}

impl BufferGetPort for Ground {
    /// Upstream's `from_bufferAllocate_handler`: an empty buffer when told
    /// to fail or for a request of `MaxPduSize` or more.
    fn invoke(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
        if self.fail_alloc.load(Ordering::SeqCst) || size as usize >= MAX_PDU_SIZE {
            return Buffer::empty();
        }
        self.allocations.fetch_add(1, Ordering::SeqCst);
        Buffer::allocate(size as usize)
    }
}

impl SendFileCompletePort for Ground {
    fn invoke(&self, _port_num: FwIndexType, resp: SendFileResponse) {
        self.file_done.lock().unwrap().push(resp);
    }
}

impl PrmGetPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) -> ParamValid {
        match self.prm_db.lock().unwrap().get(&(id - ID_BASE)) {
            Some(bytes) => {
                val.reset_ser();
                assert!(
                    val.serialize_bytes(bytes, LengthMode::OmitLength, Endianness::Big)
                        .is_ok()
                );
                ParamValid::Valid
            }
            None => ParamValid::Invalid,
        }
    }
}

impl PrmSetPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) {
        self.prm_sets
            .lock()
            .unwrap()
            .push((id - ID_BASE, val.as_slice().to_vec()));
    }
}

/// Which `Fw.BufferSend` output a sink records.
#[derive(Clone, Copy)]
enum SinkKind {
    DataOut,
    Deallocate,
    DataInReturn,
}

struct Sink {
    ground: Arc<Ground>,
    kind: SinkKind,
}

impl BufferSendPort for Sink {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        match self.kind {
            SinkKind::DataOut => self
                .ground
                .data_out
                .lock()
                .unwrap()
                .push((port_num, buffer.data().to_vec())),
            SinkKind::Deallocate => self
                .ground
                .deallocated
                .lock()
                .unwrap()
                .push((port_num, buffer.size())),
            SinkKind::DataInReturn => self
                .ground
                .data_in_returned
                .lock()
                .unwrap()
                .push((port_num, buffer.size())),
        }
    }
}

/// The component under test and its recording ground (upstream's
/// `CfdpManagerTester`).
pub(super) struct Harness {
    pub comp: Arc<CfdpManager>,
    pub ground: Arc<Ground>,
}

impl Harness {
    /// Construct, wire, init (queue depth 64), load parameters (defaults)
    /// and configure (`fileIn` queue depth 10), as upstream's tester.
    pub fn new() -> Self {
        Self::with_params(|_| {})
    }

    /// As [`Harness::new`], with parameter-database contents set first.
    pub fn with_params(setup: impl FnOnce(&Ground)) -> Self {
        let comp = CfdpManager::new("cfdp");
        let ground = Arc::new(Ground::default());
        setup(&ground);
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.prm.prm_get_out.connect(ground.clone(), 0);
        comp.prm.prm_set_out.connect(ground.clone(), 0);
        comp.ping_out.connect(ground.clone(), 0);
        comp.file_done_out.connect(ground.clone(), 0);
        for i in 0..NUM_CHANNELS {
            let port = i as FwIndexType;
            comp.buffer_allocate[i].connect(ground.clone(), port);
            for (out, kind) in [
                (&comp.data_out[i], SinkKind::DataOut),
                (&comp.buffer_deallocate[i], SinkKind::Deallocate),
                (&comp.data_in_return[i], SinkKind::DataInReturn),
            ] {
                out.connect(
                    Arc::new(Sink {
                        ground: ground.clone(),
                        kind,
                    }),
                    port,
                );
            }
        }
        comp.init(256);
        comp.load_parameters();
        comp.configure(10);
        comp.reg_commands();
        Self { comp, ground }
    }

    /// Dispatch every queued message (upstream `doDispatch`, repeated).
    pub fn dispatch(&self) {
        let status = self
            .comp
            .active
            .queued
            .dispatch_available_messages(self.comp.as_ref());
        assert!(status == DispatchStatus::Ok || status == DispatchStatus::Empty);
    }

    /// `invoke_to_run1Hz(0, 0)` + dispatch.
    pub fn run_cycle(&self) {
        let p = self.comp.run_1hz(0);
        p.target.invoke(p.port_num, 0);
        self.dispatch();
    }

    /// Run `n` cycles.
    pub fn run_cycles(&self, n: u32) {
        for _ in 0..n {
            self.run_cycle();
        }
    }

    /// Send a command (`sendCmd_*`) and dispatch it.
    pub fn send_cmd(
        &self,
        opcode: FwOpcodeType,
        cmd_seq: u32,
        write: impl FnOnce(&mut CmdArgBuffer),
    ) {
        let mut args = CmdArgBuffer::new();
        write(&mut args);
        CmdPort::invoke(self.comp.as_ref(), 0, ID_BASE + opcode, cmd_seq, &mut args);
        self.dispatch();
    }

    /// The `SendFile` command.
    #[allow(clippy::too_many_arguments)]
    pub fn send_file_cmd(
        &self,
        channel: u8,
        dest: EntityId,
        class: Class,
        keep: Keep,
        priority: u8,
        src: &str,
        dst: &str,
    ) {
        self.send_cmd(CfdpManager::OPCODE_SEND_FILE, 0, |a| {
            assert!(a.serialize_u8_be(channel).is_ok());
            assert!(a.serialize_u32_be(dest).is_ok());
            assert!(a.serialize_u8_be(class.as_repr()).is_ok());
            assert!(a.serialize_u8_be(keep.as_repr()).is_ok());
            assert!(a.serialize_u8_be(priority).is_ok());
            assert!(
                CmdStringArg::from(src)
                    .serialize_to(a, Endianness::Big)
                    .is_ok()
            );
            assert!(
                CmdStringArg::from(dst)
                    .serialize_to(a, Endianness::Big)
                    .is_ok()
            );
        });
    }

    /// Hand raw PDU bytes to `dataIn[channel]` with the `FW_PACKET_FILE`
    /// descriptor (upstream's `send*Pdu`), then dispatch.
    pub fn send_pdu(&self, channel: u8, pdu: &[u8]) {
        let mut bytes = vec![0x00, 0x03];
        bytes.extend_from_slice(pdu);
        self.send_raw(channel, &bytes);
    }

    /// Hand raw bytes to `dataIn[channel]` and dispatch.
    pub fn send_raw(&self, channel: u8, bytes: &[u8]) {
        let p = self.comp.data_in(FwIndexType::from(channel));
        let mut buffer = Buffer::allocate(bytes.len().max(1));
        buffer.set_size(bytes.len());
        buffer.data_mut().copy_from_slice(bytes);
        p.target.invoke(p.port_num, buffer);
        self.dispatch();
    }

    /// The local entity id.
    pub fn local_eid(&self) -> EntityId {
        self.comp.get_local_eid_param()
    }

    /// The outgoing file chunk size parameter.
    pub fn chunk(&self) -> usize {
        self.comp.get_outgoing_file_chunk_size_param() as usize
    }

    /// Run `f` on channel `ch` of the engine.
    pub fn chan<R>(&self, ch: u8, f: impl FnOnce(&mut Channel) -> R) -> R {
        self.comp
            .with_engine(|engine| f(&mut engine.channels[usize::from(ch)]))
    }

    /// The engine's sequence counter.
    pub fn seq_num(&self) -> TransactionSeq {
        self.comp.with_engine(|engine| engine.seq_num)
    }

    /// Upstream's `findTransaction`: the transaction with history sequence
    /// number `seq` on any transaction queue of channel `ch`.
    pub fn find_txn(&self, ch: u8, seq: TransactionSeq) -> Option<TxnIdx> {
        self.chan(ch, |c| {
            for q in [
                QueueId::Pend,
                QueueId::Txa,
                QueueId::Txw,
                QueueId::Rx,
                QueueId::Free,
            ] {
                let Some(head) = c.qs[q as usize] else {
                    continue;
                };
                let mut node = head;
                loop {
                    if let Some(h) = c.txn(node).history {
                        if c.histories[usize::from(h)].seq_num == seq {
                            return Some(node);
                        }
                    }
                    node = c.txn_links.next(node);
                    if node == head {
                        break;
                    }
                }
            }
            None
        })
    }

    /// Run `f` on transaction `t` of channel `ch`.
    pub fn txn<R>(&self, ch: u8, t: TxnIdx, f: impl FnOnce(&mut Channel, TxnIdx) -> R) -> R {
        self.chan(ch, |c| f(c, t))
    }

    /// Ids of the events emitted, in order.
    pub fn event_ids(&self) -> Vec<FwEventIdType> {
        self.ground
            .events
            .lock()
            .unwrap()
            .iter()
            .map(|e| e.0)
            .collect()
    }

    /// Argument bytes of every emitted event `id`.
    pub fn events(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
        self.ground
            .events
            .lock()
            .unwrap()
            .iter()
            .filter(|e| e.0 == id)
            .map(|e| e.2.clone())
            .collect()
    }

    /// The command responses.
    pub fn cmd_responses(&self) -> Vec<(FwOpcodeType, u32, CmdResponse)> {
        self.ground.cmd_responses.lock().unwrap().clone()
    }

    /// The PDUs sent, without their descriptors.
    pub fn sent(&self) -> Vec<Vec<u8>> {
        self.ground
            .data_out
            .lock()
            .unwrap()
            .iter()
            .map(|(_, b)| {
                assert_eq!(&b[..2], &[0x00, 0x03], "FW_PACKET_FILE descriptor");
                b[2..].to_vec()
            })
            .collect()
    }

    /// The latest `ChannelTelemetry` value.
    pub fn tlm(&self) -> ChannelTelemetryArray {
        let tlm = self.ground.tlm.lock().unwrap();
        let (id, bytes) = tlm.last().expect("telemetry written");
        assert_eq!(*id, CfdpManager::CHANID_CHANNEL_TELEMETRY);
        let mut buf = TlmBuffer::new();
        assert!(
            buf.serialize_bytes(bytes, LengthMode::OmitLength, Endianness::Big)
                .is_ok()
        );
        let mut value = ChannelTelemetryArray::default();
        assert!(buf.deserialize(&mut value, Endianness::Big).is_ok());
        value
    }

    /// The `fileDoneOut` responses.
    pub fn file_done(&self) -> Vec<SendFileResponse> {
        self.ground.file_done.lock().unwrap().clone()
    }

    /// Upstream's `waitForTransactionRecycle`: clear the history and run
    /// `inactivity_timer + 1` cycles; the transaction must be gone.
    pub fn wait_for_recycle(&self, ch: u8, seq: TransactionSeq) {
        self.ground.clear();
        let cycles = self.comp.get_inactivity_timer_param(ch) + 1;
        self.run_cycles(cycles);
        assert_eq!(self.find_txn(ch, seq), None, "recycled after inactivity");
    }
}

// ---------------------------------------------------------------------------
// PDU builders (upstream's `send*Pdu` encode step) and decoders.
// ---------------------------------------------------------------------------

/// Encode a PDU to bytes.
pub(super) fn encode(pdu: &dyn Pdu) -> Vec<u8> {
    let mut out = vec![0u8; pdu.get_buffer_size() as usize];
    let mut sb = ExtBuf::new(&mut out);
    assert_eq!(pdu.serialize_to(&mut sb), SerializeStatus::Ok);
    let len = sb.ser_loc();
    out.truncate(len);
    out
}

#[allow(clippy::too_many_arguments)]
pub(super) fn md_bytes(
    class: Class,
    src_eid: EntityId,
    dst_eid: EntityId,
    seq: TransactionSeq,
    size: FileSize,
    src: &str,
    dst: &str,
) -> Vec<u8> {
    let mut md = MetadataPdu::new();
    md.initialize(
        PduDirection::TowardReceiver,
        class,
        src_eid,
        seq,
        dst_eid,
        size,
        src.as_bytes(),
        dst.as_bytes(),
        ChecksumType::MODULAR,
        u8::from(class == Class::Class2),
    );
    encode(&md)
}

pub(super) fn fd_bytes(
    class: Class,
    src_eid: EntityId,
    dst_eid: EntityId,
    seq: TransactionSeq,
    offset: FileSize,
    data: &[u8],
) -> Vec<u8> {
    let mut fd = FileDataPdu::new();
    fd.initialize(
        PduDirection::TowardReceiver,
        class,
        src_eid,
        seq,
        dst_eid,
        offset,
        data.len() as u16,
        data,
    );
    encode(&fd)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn eof_bytes(
    class: Class,
    src_eid: EntityId,
    dst_eid: EntityId,
    seq: TransactionSeq,
    cc: ConditionCode,
    checksum: u32,
    size: FileSize,
) -> Vec<u8> {
    let mut eof = EofPdu::new();
    eof.initialize(
        PduDirection::TowardReceiver,
        class,
        src_eid,
        seq,
        dst_eid,
        cc,
        checksum,
        size,
    );
    encode(&eof)
}

pub(super) fn fin_bytes(
    src_eid: EntityId,
    dst_eid: EntityId,
    seq: TransactionSeq,
    cc: ConditionCode,
    dc: FinDeliveryCode,
    fs: FinFileStatus,
) -> Vec<u8> {
    let mut fin = FinPdu::new();
    fin.initialize(
        PduDirection::TowardSender,
        Class::Class2,
        src_eid,
        seq,
        dst_eid,
        cc,
        dc,
        fs,
    );
    encode(&fin)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn ack_bytes(
    src_eid: EntityId,
    dst_eid: EntityId,
    seq: TransactionSeq,
    directive: FileDirective,
    subtype: u8,
    cc: ConditionCode,
    status: AckTxnStatus,
) -> Vec<u8> {
    let mut ack = AckPdu::new();
    let direction = if directive == FileDirective::END_OF_FILE {
        PduDirection::TowardSender
    } else {
        PduDirection::TowardReceiver
    };
    ack.initialize(
        direction,
        Class::Class2,
        src_eid,
        seq,
        dst_eid,
        directive,
        subtype,
        cc,
        status,
    );
    encode(&ack)
}

pub(super) fn nak_bytes(
    src_eid: EntityId,
    dst_eid: EntityId,
    seq: TransactionSeq,
    scope: (FileSize, FileSize),
    segments: &[SegmentRequest],
) -> Vec<u8> {
    let mut nak = NakPdu::new();
    nak.initialize(
        PduDirection::TowardSender,
        Class::Class2,
        src_eid,
        seq,
        dst_eid,
        scope.0,
        scope.1,
    );
    for s in segments {
        assert!(nak.add_segment(s.offset_start, s.offset_end));
    }
    encode(&nak)
}

pub(super) fn decode_md(bytes: &[u8]) -> MetadataPdu {
    let mut md = MetadataPdu::new();
    assert_eq!(
        md.deserialize_from(&mut PduReader::new(bytes)),
        SerializeStatus::Ok
    );
    md
}

pub(super) fn decode_fd(bytes: &[u8]) -> FileDataPdu<'_> {
    let mut fd = FileDataPdu::new();
    assert_eq!(fd.from_buffer(bytes), SerializeStatus::Ok);
    fd
}

pub(super) fn decode_eof(bytes: &[u8]) -> EofPdu {
    let mut eof = EofPdu::new();
    assert_eq!(
        eof.deserialize_from(&mut PduReader::new(bytes)),
        SerializeStatus::Ok
    );
    eof
}

pub(super) fn decode_fin(bytes: &[u8]) -> FinPdu {
    let mut fin = FinPdu::new();
    assert_eq!(
        fin.deserialize_from(&mut PduReader::new(bytes)),
        SerializeStatus::Ok
    );
    fin
}

pub(super) fn decode_ack(bytes: &[u8]) -> AckPdu {
    let mut ack = AckPdu::new();
    assert_eq!(
        ack.deserialize_from(&mut PduReader::new(bytes)),
        SerializeStatus::Ok
    );
    ack
}

pub(super) fn decode_nak(bytes: &[u8]) -> NakPdu {
    let mut nak = NakPdu::new();
    assert_eq!(
        nak.deserialize_from(&mut PduReader::new(bytes)),
        SerializeStatus::Ok
    );
    nak
}

/// The CFDP checksum of `data` at offset 0.
pub(super) fn checksum(data: &[u8]) -> u32 {
    let mut crc = Checksum::new();
    crc.update(data, 0);
    crc.get_value()
}

/// Event argument decoding helpers.
pub(super) mod args {
    /// A big-endian `U32` at `at`.
    pub fn u32_at(b: &[u8], at: usize) -> u32 {
        u32::from_be_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
    }

    /// A length-prefixed string at `at`; returns it and the next offset.
    pub fn str_at(b: &[u8], at: usize) -> (String, usize) {
        let len = usize::from(u16::from_be_bytes([b[at], b[at + 1]]));
        (
            String::from_utf8(b[at + 2..at + 2 + len].to_vec()).unwrap(),
            at + 2 + len,
        )
    }
}
