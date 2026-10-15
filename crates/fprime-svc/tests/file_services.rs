//! The file services together, on their own threads: `FileUplink`'s
//! `fileAnnounce` feeds `FileDispatcher`, whose `fileDispatch` ports reach a
//! sequence consumer (`CmdSequencer::seqDispatchIn`) and a user client that
//! hands its files to a `FileWorker`; the worker writes a file with its
//! `.CRC32` sidecar, verifies the announced file against the CRC and reads
//! it back, returning the client's buffers on its done ports. A
//! `BufferLogger` logs the same bytes, and its file validates through
//! `Os::ValidatedFile`.
#![cfg(feature = "std")]

use fprime_comp::{BufferSendPort, CmdResponsePort, ComPort};
use fprime_config::{FwIndexType, FwOpcodeType, FwSizeType};
use fprime_fw::{Buffer, CmdArgBuffer, CmdResponse, ComBuffer, FileNameString, SerBuf};
use fprime_os::task::TASK_DEFAULT;
use fprime_svc::buffer_logger::BufferLogger;
use fprime_svc::cmd_sequencer::{CmdSequencer, FileDispatchPort};
use fprime_svc::file_dispatcher::file_dispatcher_cfg::FileDispatchPort as DispatchType;
use fprime_svc::file_dispatcher::{FileDispatcher, FileDispatcherTable};
use fprime_svc::file_uplink::FileUplink;
use fprime_svc::file_worker::{FileWorker, FileWorkerStatus, SignalDonePort};
use fprime_utils::hash::Hash;
use fprime_utils::validate_file::{self, ValidatedFile};
use std::path::PathBuf;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

fn temp_dir() -> PathBuf {
    let path =
        std::env::temp_dir().join(format!("fprime_rust_file_services_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&path);
    std::fs::create_dir_all(&path).unwrap();
    path
}

/// Which done port a record came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Read,
    Write,
    Verify,
}

/// (kind, status, size, returned buffer bytes)
type Record = (Kind, u32, FwSizeType, Vec<u8>);

/// A client of the worker: it owns the buffers it lends, verifies every
/// file the dispatcher hands it, and records the done signals.
struct Client {
    worker: Arc<FileWorker>,
    expected_crc: u32,
    records: Mutex<Vec<Record>>,
    changed: Condvar,
}

impl Client {
    fn wait_for(&self, count: usize) -> Vec<Record> {
        let guard = self.records.lock().unwrap();
        let (guard, timeout) = self
            .changed
            .wait_timeout_while(guard, Duration::from_secs(10), |r| r.len() < count)
            .unwrap();
        assert!(!timeout.timed_out(), "only {} done signals", guard.len());
        guard.clone()
    }
}

impl FileDispatchPort for Client {
    fn invoke(&self, _port_num: FwIndexType, file_name: &mut FileNameString) {
        let p = self.worker.verify_in(0);
        p.target.invoke(p.port_num, file_name, self.expected_crc);
    }
}

/// One done port of the worker, tagged with its kind.
struct DoneTap {
    client: Arc<Client>,
    kind: Kind,
}

impl SignalDonePort for DoneTap {
    fn invoke(&self, _port_num: FwIndexType, status: u32, size_bytes: FwSizeType, buffer: Buffer) {
        self.client.records.lock().unwrap().push((
            self.kind,
            status,
            size_bytes,
            buffer.data().to_vec(),
        ));
        self.client.changed.notify_all();
    }
}

/// Where `BufferLogger` forwards its buffers.
#[derive(Default)]
struct Owner {
    returned: Mutex<Vec<Vec<u8>>>,
}

impl BufferSendPort for Owner {
    fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
        self.returned.lock().unwrap().push(buffer.data().to_vec());
    }
}

impl CmdResponsePort for Owner {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        _op_code: FwOpcodeType,
        _cmd_seq: u32,
        response: CmdResponse,
    ) {
        assert_eq!(response, CmdResponse::Ok);
    }
}

#[test]
fn uplinked_files_are_dispatched_to_a_worker_client_and_verified() {
    let dir = temp_dir();
    let payload: Vec<u8> = (0..1000u32).map(|i| (i * 7 % 251) as u8).collect();
    let crc = Hash::hash_u32(&payload);

    let worker = FileWorker::new("fileWorker");
    worker.init(10);
    worker.configure(256);
    let client = Arc::new(Client {
        worker: worker.clone(),
        expected_crc: crc,
        records: Mutex::new(Vec::new()),
        changed: Condvar::new(),
    });
    for (port, kind) in [
        (&worker.read_done_out, Kind::Read),
        (&worker.write_done_out, Kind::Write),
        (&worker.verify_done_out, Kind::Verify),
    ] {
        port.connect(
            Arc::new(DoneTap {
                client: client.clone(),
                kind,
            }),
            0,
        );
    }

    let dispatcher = FileDispatcher::new("fileDispatcher");
    dispatcher.init(10);
    let mut table = FileDispatcherTable::new();
    table.num_entries = 2;
    table.entries[0].file_ext.set(".seq");
    table.entries[0].port = DispatchType::SequenceFilePort;
    table.entries[0].enabled = true;
    table.entries[1].file_ext.set(".dat");
    table.entries[1].port = DispatchType::UserFilePort;
    table.entries[1].enabled = true;
    dispatcher.configure(&table);
    // The sequence port reaches a CmdSequencer exactly as in a topology
    // (only a .dat file is announced below).
    let sequencer = CmdSequencer::new("cmdSeq");
    dispatcher.file_dispatch[DispatchType::SequenceFilePort as usize]
        .connect_to(sequencer.seq_dispatch_in(0));
    dispatcher.file_dispatch[DispatchType::UserFilePort as usize].connect(client.clone(), 0);
    // FileUplink announces into the dispatcher.
    let uplink = FileUplink::new("fileUplink");
    uplink
        .file_announce
        .connect_to(dispatcher.file_announce_recv(0));

    worker
        .active
        .start(&worker, 100, TASK_DEFAULT, TASK_DEFAULT);
    dispatcher
        .active
        .start(&dispatcher, 100, TASK_DEFAULT, TASK_DEFAULT);

    // 1. The client writes the file through the worker.
    let path = dir.join("science.dat").to_str().unwrap().to_string();
    let p = worker.write_in(0);
    p.target.invoke(
        p.port_num,
        &FileNameString::from(path.as_str()),
        Buffer::from_storage(payload.clone().into_boxed_slice(), 0),
        0,
        false,
    );
    let records = client.wait_for(1);
    assert_eq!(
        records[0],
        (
            Kind::Write,
            FileWorkerStatus::DoneWrite as u32,
            payload.len() as FwSizeType,
            payload.clone()
        )
    );

    // 2. The file is announced (as FileUplink does when a file completes);
    //    the dispatcher routes it to the client, which asks the worker to
    //    verify it.
    let mut announced = FileNameString::from(path.as_str());
    let p = dispatcher.file_announce_recv(0);
    p.target.invoke(p.port_num, &mut announced);
    let records = client.wait_for(2);
    assert_eq!(
        records[1],
        (
            Kind::Verify,
            FileWorkerStatus::Done as u32,
            payload.len() as FwSizeType,
            Vec::new()
        )
    );

    // 3. The client reads it back into a buffer it lends the worker.
    let p = worker.read_in(0);
    p.target.invoke(
        p.port_num,
        &FileNameString::from(path.as_str()),
        Buffer::allocate(payload.len()),
    );
    let records = client.wait_for(3);
    assert_eq!(
        records[2],
        (
            Kind::Read,
            FileWorkerStatus::DoneRead as u32,
            payload.len() as FwSizeType,
            payload.clone()
        )
    );

    worker.active.exit();
    dispatcher.active.exit();
    assert_eq!(worker.active.join(), fprime_os::task::Status::OpOk);
    assert_eq!(dispatcher.active.join(), fprime_os::task::Status::OpOk);

    // 4. A BufferLogger log of the same bytes validates like the worker's
    //    file (both write the big-endian CRC-32 sidecar).
    let logger = BufferLogger::new("bufferLogger");
    logger.init(10);
    let owner = Arc::new(Owner::default());
    logger.buffer_send_out.connect(owner.clone(), 0);
    logger.cmd.cmd_response_out.connect(owner.clone(), 0);
    let prefix = dir.join("log_").to_str().unwrap().to_string();
    logger.init_log(&prefix, ".bin", 1 << 20, 2);
    let mut open_args = CmdArgBuffer::new();
    assert!(open_args.serialize_u16_be(3).is_ok());
    for b in b"run" {
        assert!(open_args.serialize_u8_be(*b).is_ok());
    }
    let p = logger.cmd_in(0);
    p.target.invoke(
        p.port_num,
        BufferLogger::OPCODE_BL_OPENFILE,
        1,
        &mut open_args,
    );
    let p = logger.buffer_send_in(0);
    p.target.invoke(
        p.port_num,
        Buffer::from_storage(payload.clone().into_boxed_slice(), 7),
    );
    let mut packet = ComBuffer::new();
    assert!(packet.set_buff(&payload[..100]).is_ok());
    let p = logger.com_in(0);
    ComPort::invoke(p.target.as_ref(), p.port_num, &mut packet, 0);
    let p = logger.cmd_in(0);
    p.target.invoke(
        p.port_num,
        BufferLogger::OPCODE_BL_CLOSEFILE,
        2,
        &mut CmdArgBuffer::new(),
    );
    logger
        .active
        .start(&logger, 100, TASK_DEFAULT, TASK_DEFAULT);
    logger.active.exit();
    assert_eq!(logger.active.join(), fprime_os::task::Status::OpOk);

    let log_name = format!("{prefix}run.bin");
    let mut expected = (payload.len() as u16).to_be_bytes().to_vec();
    expected.extend_from_slice(&payload);
    expected.extend_from_slice(&100u16.to_be_bytes());
    expected.extend_from_slice(&payload[..100]);
    assert_eq!(std::fs::read(&log_name).unwrap(), expected);
    let mut validated = ValidatedFile::new(&log_name);
    assert_eq!(validated.validate(), validate_file::Status::ValidationOk);
    assert_eq!(*owner.returned.lock().unwrap(), vec![payload]);

    let _ = std::fs::remove_dir_all(&dir);
}
