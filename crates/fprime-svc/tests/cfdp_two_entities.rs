//! Two `CfdpManager` instances transferring files to each other over a
//! lossy in-process link: entity 42 (`a`) and entity 100 (`b`), each
//! one's `dataOut[ch]` wired to the other's `dataIn[ch]` through a [`Link`]
//! that can drop chosen PDUs. Upstream's Ref does not instantiate
//! `FileHandlingCfdp`, so the Rust Ref has no CFDP instance; this test is
//! the end-to-end check instead.
//!
//! Covered: class 1 (unacknowledged) and class 2 (acknowledged) transfers
//! in both directions, a class 2 transfer that loses file data PDUs (the
//! receiver NAKs the gaps and the sender retransmits them), one that loses
//! the metadata PDU (the receiver writes a temporary file and NAKs `[0, 0]`
//! for the metadata, then renames the file when it arrives), and a
//! `fileIn` port transfer reported on `fileDoneOut`. Active components are
//! driven by dispatching their queues on the test thread.

#![cfg(feature = "std")]

use fprime_comp::{
    BufferGetPort, BufferSendPort, CmdPort, CmdResponsePort, ComponentDispatch, LogPort, PrmGetPort,
};
use fprime_config::{FwEventIdType, FwIndexType, FwOpcodeType, FwPrmIdType, FwSizeType};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, CmdStringArg, Endianness, LengthMode, LogBuffer,
    LogSeverity, ParamBuffer, ParamValid, SerBuf, Serialize, Time,
};
use fprime_svc::ccsds::cfdp_manager::pdu::peek_pdu_type;
use fprime_svc::ccsds::cfdp_manager::{
    CfdpManager, ChannelArrayParams, Class, Keep, PathString, PduTypeEnum,
};
use fprime_svc::file_downlink::{
    SendFileCompletePort, SendFileNameArg, SendFileResponse, SendFileStatus,
};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

const ID_BASE_A: u32 = 0x1000;
const ID_BASE_B: u32 = 0x2000;
const EID_A: u32 = 42;
const EID_B: u32 = 100;

static COUNTER: AtomicU32 = AtomicU32::new(0);

fn temp_dir() -> PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!(
        "cfdp2e{}_{}",
        std::process::id() % 100_000,
        COUNTER.fetch_add(1, Ordering::SeqCst)
    ));
    std::fs::create_dir_all(&path).unwrap();
    path
}

/// Everything one entity reports, plus its parameter database.
#[derive(Default)]
struct Ground {
    id_base: u32,
    events: Mutex<Vec<FwEventIdType>>,
    responses: Mutex<Vec<CmdResponse>>,
    done: Mutex<Vec<SendFileResponse>>,
    prm_db: Mutex<HashMap<FwPrmIdType, Vec<u8>>>,
}

impl Ground {
    fn count(&self, id: FwEventIdType) -> usize {
        self.events
            .lock()
            .unwrap()
            .iter()
            .filter(|e| **e == id)
            .count()
    }
}

impl LogPort for Ground {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        _time_tag: &mut Time,
        _severity: LogSeverity,
        _args: &mut LogBuffer,
    ) {
        self.events.lock().unwrap().push(id - self.id_base);
    }
}

impl CmdResponsePort for Ground {
    fn invoke(&self, _port_num: FwIndexType, _op: FwOpcodeType, _seq: u32, response: CmdResponse) {
        self.responses.lock().unwrap().push(response);
    }
}

impl PrmGetPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) -> ParamValid {
        match self.prm_db.lock().unwrap().get(&(id - self.id_base)) {
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

impl SendFileCompletePort for Ground {
    fn invoke(&self, _port_num: FwIndexType, resp: SendFileResponse) {
        self.done.lock().unwrap().push(resp);
    }
}

/// The buffer pool of `bufferAllocate` and the sink of `bufferDeallocate`
/// and `dataInReturn`.
struct Pool;

impl BufferGetPort for Pool {
    fn invoke(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
        Buffer::allocate(size as usize)
    }
}

impl BufferSendPort for Pool {
    fn invoke(&self, _port_num: FwIndexType, _buffer: Buffer) {}
}

/// One direction of the space link: forwards every PDU buffer to the peer's
/// `dataIn` (same channel) unless the drop rule takes it, and logs the PDU
/// types that went by.
struct Link {
    to: Mutex<Option<Arc<CfdpManager>>>,
    /// Drop the n-th PDU of a type (0-based), per (type, n).
    drop: Mutex<Vec<(PduTypeEnum, usize)>>,
    seen: Mutex<Vec<PduTypeEnum>>,
    dropped: Mutex<Vec<PduTypeEnum>>,
}

impl Link {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            to: Mutex::new(None),
            drop: Mutex::new(Vec::new()),
            seen: Mutex::new(Vec::new()),
            dropped: Mutex::new(Vec::new()),
        })
    }

    fn seen_of(&self, t: PduTypeEnum) -> usize {
        self.seen
            .lock()
            .unwrap()
            .iter()
            .filter(|x| **x == t)
            .count()
    }
}

impl BufferSendPort for Link {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        let data = buffer.data().to_vec();
        assert_eq!(&data[..2], &[0x00, 0x03], "FW_PACKET_FILE descriptor");
        let pdu_type = peek_pdu_type(&data[2..]);
        let nth = {
            let mut seen = self.seen.lock().unwrap();
            let nth = seen.iter().filter(|t| **t == pdu_type).count();
            seen.push(pdu_type);
            nth
        };
        if self.drop.lock().unwrap().contains(&(pdu_type, nth)) {
            self.dropped.lock().unwrap().push(pdu_type);
            return;
        }
        let peer = self.to.lock().unwrap().clone().expect("link connected");
        let p = peer.data_in(port_num);
        let mut copy = Buffer::allocate(data.len());
        copy.data_mut().copy_from_slice(&data);
        p.target.invoke(p.port_num, copy);
    }
}

struct Entity {
    comp: Arc<CfdpManager>,
    ground: Arc<Ground>,
    /// The link carrying this entity's PDUs to the other one.
    out: Arc<Link>,
}

fn entity(name: &str, id_base: u32, eid: u32, tmp_dir: &str) -> Entity {
    let comp = CfdpManager::new(name);
    let ground = Arc::new(Ground {
        id_base,
        ..Ground::default()
    });
    {
        let mut db = ground.prm_db.lock().unwrap();
        db.insert(CfdpManager::PARAMID_LOCAL_EID, eid.to_be_bytes().to_vec());
        let peer = if eid == EID_A { EID_B } else { EID_A };
        db.insert(
            CfdpManager::PARAMID_FILE_IN_DEFAULT_DEST_ENTITY_ID,
            peer.to_be_bytes().to_vec(),
        );
        let mut cfg = ChannelArrayParams::fpp_default();
        for ch in 0..2 {
            cfg[ch].tmp_dir = PathString::from(tmp_dir);
        }
        let mut buf = ParamBuffer::new();
        assert!(cfg.serialize_to(&mut buf, Endianness::Big).is_ok());
        db.insert(CfdpManager::PARAMID_CHANNEL_CONFIG, buf.as_slice().to_vec());
    }
    let out = Link::new();
    let pool = Arc::new(Pool);
    comp.active.queued.base.set_id_base(id_base);
    comp.evt.log_out.connect(ground.clone(), 0);
    comp.cmd.cmd_response_out.connect(ground.clone(), 0);
    comp.prm.prm_get_out.connect(ground.clone(), 0);
    comp.file_done_out.connect(ground.clone(), 0);
    for ch in 0..2 {
        let port = ch as FwIndexType;
        comp.data_out[ch].connect(out.clone(), port);
        comp.buffer_allocate[ch].connect(pool.clone(), port);
        comp.buffer_deallocate[ch].connect(pool.clone(), port);
        comp.data_in_return[ch].connect(pool.clone(), port);
    }
    comp.init(1024);
    comp.load_parameters();
    comp.configure(10);
    Entity { comp, ground, out }
}

struct Pair {
    a: Entity,
    b: Entity,
    dir: PathBuf,
}

fn pair() -> Pair {
    let dir = temp_dir();
    let tmp = dir.to_str().unwrap().to_string();
    let a = entity("cfdpA", ID_BASE_A, EID_A, &tmp);
    let b = entity("cfdpB", ID_BASE_B, EID_B, &tmp);
    *a.out.to.lock().unwrap() = Some(b.comp.clone());
    *b.out.to.lock().unwrap() = Some(a.comp.clone());
    Pair { a, b, dir }
}

impl Pair {
    /// Dispatch both queues until neither has a message.
    fn settle(&self) {
        for _ in 0..10_000 {
            let mut busy = false;
            for e in [&self.a, &self.b] {
                if e.comp.active.queued.queue().get_messages_available() > 0 {
                    busy = true;
                    let _ = e
                        .comp
                        .active
                        .queued
                        .dispatch_available_messages(e.comp.as_ref() as &dyn ComponentDispatch);
                }
            }
            if !busy {
                return;
            }
        }
        panic!("queues never settled");
    }

    /// One second: `run1Hz` on both entities, then let the PDUs flow.
    fn tick(&self) {
        for e in [&self.a, &self.b] {
            let p = e.comp.run_1hz(0);
            p.target.invoke(p.port_num, 0);
        }
        self.settle();
    }

    fn ticks_until(&self, max: u32, mut done: impl FnMut() -> bool) -> u32 {
        for n in 0..max {
            if done() {
                return n;
            }
            self.tick();
        }
        assert!(done(), "condition not reached in {max} cycles");
        max
    }

    fn path(&self, name: &str) -> String {
        let p = self.dir.join(name).to_str().unwrap().to_string();
        assert!(p.len() <= 40, "{p} too long for a command string");
        p
    }
}

fn send_file(e: &Entity, ch: u8, dest: u32, class: Class, src: &str, dst: &str) {
    let mut args = CmdArgBuffer::new();
    assert!(args.serialize_u8_be(ch).is_ok());
    assert!(args.serialize_u32_be(dest).is_ok());
    assert!(args.serialize_u8_be(class.as_repr()).is_ok());
    assert!(args.serialize_u8_be(Keep::Keep.as_repr()).is_ok());
    assert!(args.serialize_u8_be(0).is_ok());
    assert!(
        CmdStringArg::from(src)
            .serialize_to(&mut args, Endianness::Big)
            .is_ok()
    );
    assert!(
        CmdStringArg::from(dst)
            .serialize_to(&mut args, Endianness::Big)
            .is_ok()
    );
    let base = e.comp.active.queued.base.get_id_base();
    CmdPort::invoke(
        e.comp.as_ref(),
        0,
        base + CfdpManager::OPCODE_SEND_FILE,
        1,
        &mut args,
    );
}

fn contents(size: usize, seed: u8) -> Vec<u8> {
    (0..size)
        .map(|i| (i as u8).wrapping_mul(31).wrapping_add(seed))
        .collect()
}

#[test]
fn class1_transfer_a_to_b() {
    let p = pair();
    let src = p.path("c1.src");
    let dst = p.path("c1.dst");
    let data = contents(3000, 1);
    std::fs::write(&src, &data).unwrap();
    send_file(&p.a, 0, EID_B, Class::Class1, &src, &dst);
    p.ticks_until(10, || {
        p.b.ground
            .count(CfdpManager::EVENTID_RX_FILE_TRANSFER_COMPLETED)
            == 1
    });
    assert_eq!(std::fs::read(&dst).unwrap(), data);
    assert_eq!(
        p.a.ground
            .count(CfdpManager::EVENTID_TX_FILE_TRANSFER_COMPLETED),
        1
    );
    assert_eq!(*p.a.ground.responses.lock().unwrap(), [CmdResponse::Ok]);
    // Unacknowledged: nothing ever comes back.
    assert!(p.b.out.seen.lock().unwrap().is_empty());
    assert_eq!(p.a.out.seen_of(PduTypeEnum::FileData), 4);
}

#[test]
fn class2_transfer_b_to_a_on_channel_1() {
    let p = pair();
    let src = p.path("c2.src");
    let dst = p.path("c2.dst");
    let data = contents(5000, 2);
    std::fs::write(&src, &data).unwrap();
    send_file(&p.b, 1, EID_A, Class::Class2, &src, &dst);
    p.ticks_until(20, || {
        p.b.ground
            .count(CfdpManager::EVENTID_TX_FILE_TRANSFER_COMPLETED)
            == 1
    });
    // The sender finishes on the FIN; the receiver on the FIN-ACK, which
    // goes out the next cycle.
    p.ticks_until(5, || {
        p.a.ground
            .count(CfdpManager::EVENTID_RX_FILE_TRANSFER_COMPLETED)
            == 1
    });
    assert_eq!(std::fs::read(&dst).unwrap(), data);
    // The full class 2 exchange: EOF-ACK and FIN back, FIN-ACK out. (The
    // receiver's ACK timer, armed by the last file data, may expire before
    // the FIN-ACK lands and repeat the FIN, which the sender re-ACKs from
    // HOLD — upstream timing.)
    assert_eq!(p.a.out.seen_of(PduTypeEnum::Acknowledgment), 1);
    assert!(p.a.out.seen_of(PduTypeEnum::Finished) >= 1);
    assert!(p.b.out.seen_of(PduTypeEnum::Acknowledgment) >= 1);
    assert_eq!(p.a.out.seen_of(PduTypeEnum::NegativeAck), 0);
}

#[test]
fn class2_transfer_recovers_dropped_file_data_with_naks() {
    let p = pair();
    let src = p.path("nak.src");
    let dst = p.path("nak.dst");
    let data = contents(992 * 6 + 100, 3);
    std::fs::write(&src, &data).unwrap();
    p.a.out
        .drop
        .lock()
        .unwrap()
        .extend([(PduTypeEnum::FileData, 1), (PduTypeEnum::FileData, 4)]);
    send_file(&p.a, 0, EID_B, Class::Class2, &src, &dst);
    let cycles = p.ticks_until(40, || {
        p.a.ground
            .count(CfdpManager::EVENTID_TX_FILE_TRANSFER_COMPLETED)
            == 1
            && p.b
                .ground
                .count(CfdpManager::EVENTID_RX_FILE_TRANSFER_COMPLETED)
                == 1
    });
    assert_eq!(
        *p.a.out.dropped.lock().unwrap(),
        [PduTypeEnum::FileData, PduTypeEnum::FileData]
    );
    assert!(
        p.b.out.seen_of(PduTypeEnum::NegativeAck) >= 1,
        "receiver NAKed"
    );
    // 7 PDUs of new data + the 2 retransmitted (twice when the receiver's
    // ACK timer re-NAKs before the first retransmission lands).
    let fd = p.a.out.seen_of(PduTypeEnum::FileData);
    assert!(fd == 9 || fd == 11, "{fd} file data PDUs");
    assert_eq!(std::fs::read(&dst).unwrap(), data);
    assert_eq!(
        p.b.ground
            .count(CfdpManager::EVENTID_RX_FILE_TRANSFER_COMPLETED),
        1
    );
    assert!(p.b.ground.count(CfdpManager::EVENTID_RX_CRC_MISMATCH) == 0);
    assert!(cycles < 40);
}

#[test]
fn class2_transfer_recovers_a_dropped_metadata_pdu() {
    let p = pair();
    let src = p.path("md.src");
    let dst = p.path("md.dst");
    let data = contents(2500, 4);
    std::fs::write(&src, &data).unwrap();
    p.a.out
        .drop
        .lock()
        .unwrap()
        .push((PduTypeEnum::Metadata, 0));
    send_file(&p.a, 0, EID_B, Class::Class2, &src, &dst);
    p.ticks_until(40, || {
        p.a.ground
            .count(CfdpManager::EVENTID_TX_FILE_TRANSFER_COMPLETED)
            == 1
            && p.b
                .ground
                .count(CfdpManager::EVENTID_RX_FILE_TRANSFER_COMPLETED)
                == 1
    });
    // The receiver started from file data into a temporary file, NAKed for
    // the metadata, and renamed the file once it arrived.
    assert_eq!(
        p.b.ground.count(CfdpManager::EVENTID_RX_TEMP_FILE_CREATED),
        1
    );
    assert!(p.a.out.seen_of(PduTypeEnum::Metadata) >= 2);
    assert_eq!(
        p.a.ground
            .count(CfdpManager::EVENTID_TX_FILE_TRANSFER_STARTED),
        1
    );
    assert_eq!(std::fs::read(&dst).unwrap(), data);
    let temp = p.dir.join(format!("{EID_A}:1.tmp"));
    assert!(!temp.exists());
}

#[test]
fn class2_transfer_survives_a_dropped_eof_and_fin() {
    let p = pair();
    let src = p.path("eof.src");
    let dst = p.path("eof.dst");
    let data = contents(1500, 5);
    std::fs::write(&src, &data).unwrap();
    p.a.out
        .drop
        .lock()
        .unwrap()
        .push((PduTypeEnum::EndOfFile, 0));
    p.b.out
        .drop
        .lock()
        .unwrap()
        .push((PduTypeEnum::Finished, 0));
    send_file(&p.a, 0, EID_B, Class::Class2, &src, &dst);
    p.ticks_until(60, || {
        p.a.ground
            .count(CfdpManager::EVENTID_TX_FILE_TRANSFER_COMPLETED)
            == 1
            && p.b
                .ground
                .count(CfdpManager::EVENTID_RX_FILE_TRANSFER_COMPLETED)
                == 1
    });
    // The ACK timers re-sent both.
    assert!(p.a.out.seen_of(PduTypeEnum::EndOfFile) >= 2);
    assert!(p.b.out.seen_of(PduTypeEnum::Finished) >= 2);
    assert_eq!(std::fs::read(&dst).unwrap(), data);
}

#[test]
fn file_in_port_transfer_reports_on_file_done_out() {
    let p = pair();
    let src = p.dir.join("port.src").to_str().unwrap().to_string();
    let dst = p.dir.join("port.dst").to_str().unwrap().to_string();
    let data = contents(800, 6);
    std::fs::write(&src, &data).unwrap();
    // FileInDefault*: channel 0, entity 100 (b), class 2, DELETE.
    let port = p.a.comp.file_in(0);
    let resp = port.target.invoke(
        port.port_num,
        &SendFileNameArg::from(src.as_str()),
        &SendFileNameArg::from(dst.as_str()),
        0,
        0,
    );
    assert_eq!(resp.status, SendFileStatus::StatusOk);
    p.ticks_until(20, || !p.a.ground.done.lock().unwrap().is_empty());
    assert_eq!(
        p.a.ground.done.lock().unwrap()[0].status,
        SendFileStatus::StatusOk
    );
    assert_eq!(std::fs::read(&dst).unwrap(), data);
    assert!(
        !std::path::Path::new(&src).exists(),
        "Keep::Delete removed the source"
    );
}
