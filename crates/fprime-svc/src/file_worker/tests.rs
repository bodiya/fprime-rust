//! Unit tests for `Svc::FileWorker`: ports of upstream
//! `Svc/FileWorker/test/ut/FileWorkerTester.cpp` (nominal) and
//! `FileWorkerErrTester.cpp` (errors), each in its own temporary directory.
//!
//! Upstream's error suite swaps in a mock `Os::File` delegate
//! (`test/FileTester`) that returns chosen statuses; the Rust OSAL has no
//! delegate seam, so the same branches are reached with real faults: a
//! missing directory (open fails), a file opened in the wrong mode (read or
//! write fails with `INVALID_MODE`), a directory read as a file (`EISDIR`),
//! a path under a regular file (`ENOTDIR`). The upstream fixture
//! `nominalread.bin` (8247 bytes, the byte ramp 0x61, 0x62, ...) and its
//! sidecar are generated here; its CRC is upstream's 0xBC0905F4.

use super::*;
use fprime_comp::{LogPort, LogTextPort, TimePort};
use fprime_fw::TimeBase;
use fprime_os::queue::BlockingType;
use fprime_utils::validate_file;
use std::path::PathBuf;
use std::sync::Mutex as StdMutex;
use std::sync::atomic::AtomicU32 as StdAtomicU32;

const ID_BASE: u32 = 0x6600;

/// Size of upstream's `nominalread.bin`.
const NOMINAL_SIZE: usize = 8247;
/// Upstream's `nominalread.bin.CRC32`.
const NOMINAL_CRC: u32 = 0xBC09_05F4;

/// A unique scratch directory, removed on drop.
struct TempDir {
    path: PathBuf,
}

impl TempDir {
    fn new(tag: &str) -> Self {
        static COUNTER: StdAtomicU32 = StdAtomicU32::new(0);
        let path = std::env::temp_dir().join(format!(
            "fprime_rust_fileworker_{tag}_{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::Relaxed)
        ));
        let _ = std::fs::remove_dir_all(&path);
        std::fs::create_dir_all(&path).unwrap();
        Self { path }
    }

    fn file(&self, name: &str) -> String {
        self.path.join(name).to_str().unwrap().to_string()
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.path);
    }
}

/// Upstream's `nominalread.bin` content.
fn nominal_data() -> Vec<u8> {
    (0..NOMINAL_SIZE).map(|i| (0x61 + i) as u8).collect()
}

/// Write `nominalread.bin` and its sidecar into `dir`; return its path.
fn write_nominal(dir: &TempDir) -> String {
    let name = dir.file("nominalread.bin");
    std::fs::write(&name, nominal_data()).unwrap();
    std::fs::write(format!("{name}.CRC32"), NOMINAL_CRC.to_be_bytes()).unwrap();
    name
}

/// The upstream tests' 1024-byte ramp `i % 256` (repeated to `len`).
fn ramp(len: usize) -> Vec<u8> {
    (0..len).map(|i| (i % 256) as u8).collect()
}

fn buffer_of(data: &[u8]) -> Buffer {
    Buffer::from_storage(data.to_vec().into_boxed_slice(), 0)
}

/// (id, severity, raw arg bytes)
type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);
/// (status, size, returned buffer contents)
type DoneRecord = (u32, FwSizeType, Vec<u8>);

#[derive(Default)]
struct Ground {
    events: StdMutex<Vec<EventRecord>>,
}

impl Ground {
    fn count(&self, id: FwEventIdType) -> usize {
        self.with(id).len()
    }

    fn with(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
        self.events
            .lock()
            .unwrap()
            .iter()
            .filter(|e| e.0 == ID_BASE + id)
            .map(|e| e.2.clone())
            .collect()
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
            .push((id, severity, args.as_slice().to_vec()));
    }
}

impl LogTextPort for Ground {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        _id: FwEventIdType,
        _time_tag: &mut Time,
        _severity: LogSeverity,
        _text: &mut TextLogString,
    ) {
    }
}

/// One done port's history.
#[derive(Default)]
struct Done {
    calls: StdMutex<Vec<DoneRecord>>,
}

impl Done {
    fn take(&self) -> Vec<DoneRecord> {
        std::mem::take(&mut *self.calls.lock().unwrap())
    }

    /// (status, size) pairs, buffers ignored.
    fn statuses(&self) -> Vec<(u32, FwSizeType)> {
        self.calls
            .lock()
            .unwrap()
            .iter()
            .map(|c| (c.0, c.1))
            .collect()
    }
}

impl SignalDonePort for Done {
    fn invoke(&self, _port_num: FwIndexType, status: u32, size_bytes: FwSizeType, buffer: Buffer) {
        self.calls
            .lock()
            .unwrap()
            .push((status, size_bytes, buffer.data().to_vec()));
    }
}

/// The test time source (upstream `setTestTime`): a fixed time that can
/// advance by `step_us` on every read.
struct TestTime {
    now: StdMutex<Time>,
    step_us: StdMutex<u32>,
}

impl TestTime {
    fn new() -> Self {
        Self {
            now: StdMutex::new(Time::new(TimeBase::TbWorkstationTime, 0, 1000, 0)),
            step_us: StdMutex::new(0),
        }
    }

    fn advance(&self, seconds: u32) {
        self.now.lock().unwrap().add_duration(seconds, 0);
    }
}

impl TimePort for TestTime {
    fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
        let mut now = self.now.lock().unwrap();
        *time = *now;
        let step = *self.step_us.lock().unwrap();
        now.add_duration(step / 1_000_000, step % 1_000_000);
    }
}

struct Tester {
    comp: Arc<FileWorker>,
    ground: Arc<Ground>,
    read_done: Arc<Done>,
    write_done: Arc<Done>,
    verify_done: Arc<Done>,
    time: Arc<TestTime>,
    dir: TempDir,
}

impl Tester {
    /// Upstream constructor: connect everything, `configure(100)`.
    fn new(tag: &str) -> Self {
        let comp = FileWorker::new("FileWorker");
        let ground = Arc::new(Ground::default());
        let read_done = Arc::new(Done::default());
        let write_done = Arc::new(Done::default());
        let verify_done = Arc::new(Done::default());
        let time = Arc::new(TestTime::new());
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.evt.time_out.connect(time.clone(), 0);
        comp.read_done_out.connect(read_done.clone(), 0);
        comp.write_done_out.connect(write_done.clone(), 0);
        comp.verify_done_out.connect(verify_done.clone(), 0);
        comp.init(20);
        comp.configure(100);
        Self {
            comp,
            ground,
            read_done,
            write_done,
            verify_done,
            time,
            dir: TempDir::new(tag),
        }
    }

    fn clear_history(&self) {
        self.ground.events.lock().unwrap().clear();
        self.read_done.take();
        self.write_done.take();
        self.verify_done.take();
    }

    fn do_dispatch(&self) {
        assert_eq!(
            self.comp
                .active
                .queued
                .do_dispatch(self.comp.as_ref(), BlockingType::NonBlocking),
            MsgDispatchStatus::Ok
        );
    }

    fn read_in(&self, path: &str, buffer: Buffer) {
        let port = self.comp.read_in(0);
        port.target
            .invoke(port.port_num, &FileNameString::from(path), buffer);
        self.do_dispatch();
    }

    fn write_in(&self, path: &str, buffer: Buffer, offset: FwSizeType, append: bool) {
        let port = self.comp.write_in(0);
        port.target.invoke(
            port.port_num,
            &FileNameString::from(path),
            buffer,
            offset,
            append,
        );
        self.do_dispatch();
    }

    fn verify_in(&self, path: &str, crc: u32) {
        let port = self.comp.verify_in(0);
        port.target
            .invoke(port.port_num, &FileNameString::from(path), crc);
        self.do_dispatch();
    }

    fn cancel_in(&self) {
        let port = self.comp.cancel_in(0);
        port.target.invoke(port.port_num);
    }

    fn set_state(&self, state: FileWorkerState) {
        self.comp.set_state(state);
    }

    fn abort(&self) -> bool {
        self.comp.abort.load(Ordering::Relaxed)
    }

    fn set_abort(&self, value: bool) {
        self.comp.abort.store(value, Ordering::Relaxed);
    }

    fn assert_no_write_errors(&self) {
        for id in [
            FileWorker::EVENTID_WRITEABORTED,
            FileWorker::EVENTID_OPENFILEERROR,
            FileWorker::EVENTID_WRITEFILEERROR,
            FileWorker::EVENTID_WRITEVALIDATIONERROR,
            FileWorker::EVENTID_WRITETIMEOUT,
        ] {
            assert_eq!(self.ground.count(id), 0, "event {id}");
        }
    }
}

fn validate(file_name: &str) -> validate_file::Status {
    validate_file::validate(file_name, &format!("{file_name}.CRC32"))
}

fn status(s: FileWorkerStatus) -> u32 {
    s as u32
}

/// The `InvalidInput(handler, issue)` argument bytes.
fn invalid_input_args(handler: &str, issue: &str) -> Vec<u8> {
    let mut bytes = (handler.len() as u16).to_be_bytes().to_vec();
    bytes.extend_from_slice(handler.as_bytes());
    bytes.extend_from_slice(&(issue.len() as u16).to_be_bytes());
    bytes.extend_from_slice(issue.as_bytes());
    bytes
}

// ----------------------------------------------------------------------
// FileWorkerTester.cpp (nominal)
// ----------------------------------------------------------------------

#[test]
fn test_read_file() {
    let t = Tester::new("readfile");
    let name = write_nominal(&t.dir);

    // Nominal read.
    assert_eq!(t.comp.state(), FileWorkerState::Idle);
    t.read_in(&name, buffer_of(&[0u8; 1024]));
    assert_eq!(t.ground.count(FileWorker::EVENTID_NOTINIDLE), 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_READFAILEDFILESIZE), 0);
    let done = t.read_done.take();
    assert_eq!(done.len(), 1);
    assert_eq!(done[0].0, status(FileWorkerStatus::DoneRead));
    assert_eq!(done[0].1, 1024);
    // The buffer comes back holding the start of the file.
    assert_eq!(done[0].2, nominal_data()[..1024]);
    assert_eq!(t.comp.state(), FileWorkerState::Idle);

    // Not in IDLE.
    t.clear_history();
    t.set_state(FileWorkerState::Writing);
    t.read_in(&name, buffer_of(&[0u8; 1024]));
    assert_eq!(t.comp.state(), FileWorkerState::Writing);
    assert_eq!(
        t.read_done.statuses(),
        vec![(status(FileWorkerStatus::NotIdle), 0)]
    );
    assert_eq!(
        t.ground.with(FileWorker::EVENTID_NOTINIDLE),
        vec![(FileWorkerState::Writing as u32).to_be_bytes().to_vec()]
    );

    // CRC failed.
    t.clear_history();
    t.set_state(FileWorkerState::Idle);
    t.read_in(&t.dir.file("nominaldne.bin"), buffer_of(&[0u8; 1024]));
    assert_eq!(t.comp.state(), FileWorkerState::Idle);
    assert_eq!(
        t.read_done.statuses(),
        vec![(status(FileWorkerStatus::FailedCrc), 0)]
    );
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 1);
}

#[test]
fn test_cancel() {
    let t = Tester::new("cancel");
    let name = t.dir.file("testfile.txt");
    let size = 1024 * 100;
    std::fs::write(&name, vec![0u8; size]).unwrap();

    // Read: the abort is seen after the first chunk.
    let mut f = File::new();
    assert_eq!(f.open(&name, Mode::OpenRead), file::Status::OpOk);
    assert!(!t.abort());
    t.cancel_in();
    assert!(t.abort());
    let mut buffer = buffer_of(&vec![0u8; size]);
    let mut read_bytes: FwSizeType = 0;
    assert_eq!(
        t.comp
            .read_file_bytes(&mut buffer, size as FwSizeType, &mut f, &mut read_bytes),
        FileWorkerReadStatus::Abort
    );
    assert_eq!(read_bytes, 0);
    t.set_abort(false);
    f.close();

    // Write: the first chunk is written but not counted.
    assert_eq!(f.open(&name, Mode::OpenWrite), file::Status::OpOk);
    assert!(!t.abort());
    t.cancel_in();
    assert!(t.abort());
    assert_eq!(t.comp.write_to_file(&vec![0u8; size], &mut f, &name), 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITEABORTED), 1);
    t.set_abort(false);
    f.close();
}

#[test]
fn test_file_error() {
    let t = Tester::new("fileerror");
    let name = t.dir.file("testfile.txt");
    let mut f = File::new();
    let mut buffer = buffer_of(&vec![0u8; 1024 * 100]);
    // An unopened file is a read error without an event.
    assert_eq!(
        t.comp.read_file(&mut buffer, 1024 * 100, &mut f, &name),
        FileWorkerReadStatus::Error
    );
    assert!(t.ground.events.lock().unwrap().is_empty());
}

#[test]
fn test_verify() {
    let t = Tester::new("verify");
    let name = write_nominal(&t.dir);

    t.verify_in(&name, 0x0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 0);
    assert_eq!(
        t.ground.with(FileWorker::EVENTID_CRCVERIFICATIONERROR),
        vec![[0u32.to_be_bytes(), NOMINAL_CRC.to_be_bytes()].concat()]
    );
    assert_eq!(
        t.verify_done.statuses(),
        vec![(
            status(FileWorkerStatus::FailedCrc),
            NOMINAL_SIZE as FwSizeType
        )]
    );

    t.clear_history();
    t.verify_in(&name, NOMINAL_CRC);
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCVERIFICATIONERROR), 0);
    assert_eq!(
        t.verify_done.statuses(),
        vec![(status(FileWorkerStatus::Done), NOMINAL_SIZE as FwSizeType)]
    );

    t.clear_history();
    t.verify_in(&t.dir.file("nominaldne.bin"), NOMINAL_CRC);
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 1);
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCVERIFICATIONERROR), 1);
    let done = t.verify_done.take();
    assert_eq!(done.len(), 1);
    assert_eq!(
        (done[0].0, done[0].1),
        (status(FileWorkerStatus::FailedFileSize), 0)
    );
    // verifyDoneOut carries no buffer.
    assert!(done[0].2.is_empty());
}

#[test]
fn test_transfer() {
    let t = Tester::new("transfer");
    t.set_state(FileWorkerState::Writing);
    t.write_in(
        &t.dir.file("nominalwrite.bin"),
        buffer_of(&[0u8; 1024]),
        0,
        true,
    );
    assert_eq!(t.comp.state(), FileWorkerState::Writing);
    // Upstream asserts FW_STATE_IDLE here: the same 0 as FW_STATUS_NOT_IDLE.
    assert_eq!(
        t.write_done.statuses(),
        vec![(FileWorkerState::Idle as u32, 0)]
    );
    assert_eq!(
        t.ground.with(FileWorker::EVENTID_NOTINIDLE),
        vec![(FileWorkerState::Writing as u32).to_be_bytes().to_vec()]
    );
    assert!(!std::path::Path::new(&t.dir.file("nominalwrite.bin")).exists());
}

#[test]
fn test_writing() {
    let t = Tester::new("writing");
    let data = ramp(1024);
    let name = t.dir.file("testwrite.txt");
    t.write_in(&name, buffer_of(&data), 0, true);
    assert_eq!(std::fs::read(&name).unwrap(), data);
    assert_eq!(validate(&name), validate_file::Status::ValidationOk);
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITEBEGIN), 1);
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITECOMPLETED), 1);
    t.assert_no_write_errors();
    let done = t.write_done.take();
    assert_eq!(
        (done[0].0, done[0].1),
        (status(FileWorkerStatus::DoneWrite), 1024)
    );
    // The buffer comes back to the client.
    assert_eq!(done[0].2, data);
}

#[test]
fn test_writing_offset() {
    let t = Tester::new("offset");
    let data = ramp(1024);
    let name = t.dir.file("testwriteoffset.txt");
    t.write_in(&name, buffer_of(&data), 12, true);
    assert_eq!(std::fs::read(&name).unwrap(), data[12..]);
    assert_eq!(validate(&name), validate_file::Status::ValidationOk);
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITEBEGIN), 1);
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITECOMPLETED), 1);
    t.assert_no_write_errors();
    assert_eq!(
        t.write_done.statuses(),
        vec![(status(FileWorkerStatus::DoneWrite), 1024 - 12)]
    );
}

#[test]
fn test_write_zero_length() {
    let t = Tester::new("zero");
    let name = t.dir.file("testwritezero.txt");
    let preserved = b"PRESERVE-ME\0";
    std::fs::write(&name, preserved).unwrap();

    let data = ramp(1024);
    t.write_in(&name, buffer_of(&data), data.len() as FwSizeType, false);
    assert_eq!(t.ground.count(FileWorker::EVENTID_INVALIDINPUT), 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITEFILEERROR), 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_OPENFILEERROR), 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITEVALIDATIONERROR), 0);
    assert_eq!(
        t.write_done.statuses(),
        vec![(status(FileWorkerStatus::DoneWrite), 0)]
    );
    assert_eq!(t.comp.state(), FileWorkerState::Idle);
    assert_eq!(std::fs::read(&name).unwrap(), preserved);
    // No sidecar was written either.
    assert!(!std::path::Path::new(&format!("{name}.CRC32")).exists());

    // Nor is a missing file created.
    let missing = t.dir.file("never.txt");
    t.write_in(&missing, buffer_of(&data), data.len() as FwSizeType, true);
    assert!(!std::path::Path::new(&missing).exists());
}

#[test]
fn test_appending() {
    let t = Tester::new("appending");
    let n = 3;
    let data = ramp(1024);
    let name = t.dir.file("testwrite.txt");
    for _ in 0..n {
        t.write_in(&name, buffer_of(&data), 0, true);
    }
    assert_eq!(std::fs::read(&name).unwrap(), data.repeat(n));
    // The sidecar was extended, not recomputed from the last buffer.
    assert_eq!(validate(&name), validate_file::Status::ValidationOk);
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITEBEGIN), n);
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITECOMPLETED), n);
    t.assert_no_write_errors();
}

#[test]
fn test_write_read_round_trip() {
    let t = Tester::new("roundtrip");
    let name = t.dir.file("testroundtrip.txt");
    let data = ramp(1024);

    t.write_in(&name, buffer_of(&data), 0, false);
    assert_eq!(
        t.write_done.statuses(),
        vec![(status(FileWorkerStatus::DoneWrite), 1024)]
    );
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITEFILEERROR), 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_WRITEVALIDATIONERROR), 0);

    t.clear_history();
    t.read_in(&name, buffer_of(&[0u8; 1024]));
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 0);
    let done = t.read_done.take();
    assert_eq!(
        (done[0].0, done[0].1),
        (status(FileWorkerStatus::DoneRead), 1024)
    );
    assert_eq!(done[0].2, data);

    // CRC-32 of the byte ramp.
    t.clear_history();
    t.verify_in(&name, 0xB70B_4C26);
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCVERIFICATIONERROR), 0);
    assert_eq!(
        t.verify_done.statuses(),
        vec![(status(FileWorkerStatus::Done), 1024)]
    );
}

#[test]
fn test_timeout() {
    // With a stationary clock no timeout fires.
    let t = Tester::new("timeout");
    let name = write_nominal(&t.dir);
    let mut f = File::new();
    assert_eq!(f.open(&name, Mode::OpenRead), file::Status::OpOk);
    let mut buffer = buffer_of(&[0u8; 32]);
    let mut read_bytes: FwSizeType = 0;
    assert_eq!(
        t.comp
            .read_file_bytes(&mut buffer, 32, &mut f, &mut read_bytes),
        FileWorkerReadStatus::Done
    );
    assert_eq!(read_bytes, 32);
    assert_eq!(buffer.data(), &nominal_data()[..32]);
}

#[test]
fn test_path_too_long() {
    let t = Tester::new("toolong");
    let ext = Hash::get_file_extension_length() as usize;
    let longest_ok = FileNameString::CAPACITY - ext;
    assert_eq!(longest_ok, 234);
    let data: Vec<u8> = (0..16u8).collect();

    for len in [longest_ok + 1, FileNameString::CAPACITY] {
        let long_name = "p".repeat(len);
        assert_eq!(FileNameString::from(long_name.as_str()).len(), len);

        t.clear_history();
        t.read_in(&long_name, buffer_of(&data));
        assert_eq!(
            t.ground.with(FileWorker::EVENTID_INVALIDINPUT),
            vec![invalid_input_args("readIn", "path too long")]
        );
        assert_eq!(
            t.read_done.statuses(),
            vec![(status(FileWorkerStatus::InvalidInput), 0)]
        );
        assert_eq!(t.comp.state(), FileWorkerState::Idle);

        t.clear_history();
        t.verify_in(&long_name, 0);
        assert_eq!(
            t.ground.with(FileWorker::EVENTID_INVALIDINPUT),
            vec![invalid_input_args("verifyIn", "path too long")]
        );
        assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 0);
        assert_eq!(
            t.verify_done.statuses(),
            vec![(status(FileWorkerStatus::InvalidInput), 0)]
        );

        t.clear_history();
        t.write_in(&long_name, buffer_of(&data), 0, false);
        assert_eq!(
            t.ground.with(FileWorker::EVENTID_INVALIDINPUT),
            vec![invalid_input_args("writeIn", "path too long")]
        );
        assert_eq!(
            t.write_done.statuses(),
            vec![(status(FileWorkerStatus::InvalidInput), 0)]
        );
        assert_eq!(t.comp.state(), FileWorkerState::Idle);
        assert!(!filesystem::exists(&long_name));
    }

    // The longest path that still fits reaches the file layer: no sidecar,
    // so verify reports a CRC failure (open of the sidecar).
    let dir_len = t.dir.path.to_str().unwrap().len() + 1;
    let ok_name = t.dir.file(&"q".repeat(longest_ok - dir_len));
    assert_eq!(ok_name.len(), longest_ok);
    std::fs::write(&ok_name, &data).unwrap();
    t.clear_history();
    t.verify_in(&ok_name, 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_INVALIDINPUT), 0);
    assert_eq!(
        t.ground.with(FileWorker::EVENTID_CRCFAILED),
        vec![(CrcStat::FailedFileCrcOpen as u32).to_be_bytes().to_vec()]
    );
    assert_eq!(
        t.verify_done.statuses(),
        vec![(
            status(FileWorkerStatus::FailedCrc),
            data.len() as FwSizeType
        )]
    );
}

// ----------------------------------------------------------------------
// FileWorkerErrTester.cpp (errors, with real faults)
// ----------------------------------------------------------------------

#[test]
fn test_read_err() {
    let t = Tester::new("readerr");
    let name = write_nominal(&t.dir);
    let size = 1024 * 100;
    let mut buffer = buffer_of(&vec![0u8; size]);
    let mut read_bytes: FwSizeType = 0;

    // Read from an unopened file.
    let mut f = File::new();
    assert_eq!(
        t.comp
            .read_file_bytes(&mut buffer, size as FwSizeType, &mut f, &mut read_bytes),
        FileWorkerReadStatus::Error
    );

    // Open fails.
    t.clear_history();
    assert_eq!(
        t.comp
            .read_buffer_from_file(&mut buffer, &t.dir.file("missing/nominalread.bin")),
        FileWorkerStatus::FailedToOpen
    );
    assert_eq!(t.ground.count(FileWorker::EVENTID_OPENFILEERROR), 1);

    // Read fails (INVALID_MODE: the file is open for writing).
    let mut w = File::new();
    assert_eq!(w.open(&name, Mode::OpenWrite), file::Status::OpOk);
    t.clear_history();
    let _ = t
        .comp
        .read_file(&mut buffer, size as FwSizeType, &mut w, &name);
    assert_eq!(t.ground.count(FileWorker::EVENTID_READERROR), 1);
    w.close();

    // Abort.
    assert_eq!(f.open(&name, Mode::OpenRead), file::Status::OpOk);
    t.clear_history();
    t.set_abort(true);
    let _ = t
        .comp
        .read_file(&mut buffer, size as FwSizeType, &mut f, &name);
    assert_eq!(t.ground.count(FileWorker::EVENTID_READABORTED), 1);
    t.set_abort(false);

    // A readIn whose CRC check cannot open the file.
    t.clear_history();
    t.read_in(
        &t.dir.file("missing/nominalread.bin"),
        buffer_of(&vec![0u8; size]),
    );
    assert_eq!(
        t.read_done.statuses(),
        vec![(status(FileWorkerStatus::FailedCrc), 0)]
    );
}

#[test]
fn test_read_err_status_reported() {
    let t = Tester::new("readerrstatus");
    let name = write_nominal(&t.dir);
    let mut buffer = buffer_of(&vec![0u8; 1024]);

    // Positive control.
    t.clear_history();
    t.set_abort(false);
    assert_eq!(
        t.comp.read_buffer_from_file(&mut buffer, &name),
        FileWorkerStatus::DoneRead
    );

    // A read error (a directory opens, but reading it fails).
    t.clear_history();
    let dir = t.dir.path.to_str().unwrap().to_string();
    assert_eq!(
        t.comp.read_buffer_from_file(&mut buffer, &dir),
        FileWorkerStatus::FailedToRead
    );
    assert_eq!(t.ground.count(FileWorker::EVENTID_READERROR), 1);
    // ReadCompleted is emitted whatever the outcome.
    assert_eq!(t.ground.count(FileWorker::EVENTID_READCOMPLETED), 1);

    // A cancelled read.
    t.clear_history();
    t.set_abort(true);
    assert_eq!(
        t.comp.read_buffer_from_file(&mut buffer, &name),
        FileWorkerStatus::FailedToRead
    );
    assert_eq!(t.ground.count(FileWorker::EVENTID_READABORTED), 1);
    t.set_abort(false);
}

#[test]
fn test_verify_err() {
    let t = Tester::new("verifyerr");
    // No such file: the CRC check fails before computing a CRC, so the
    // computed CRC is 0 and an expected 0 raises no verification error.
    t.verify_in(&t.dir.file("testfile.txt"), 0);
    assert_eq!(
        t.verify_done.statuses(),
        vec![(status(FileWorkerStatus::FailedFileSize), 0)]
    );
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 1);
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCVERIFICATIONERROR), 0);
    assert_eq!(t.ground.count(FileWorker::EVENTID_READFAILEDFILESIZE), 1);
}

#[test]
fn test_write_err() {
    let t = Tester::new("writeerr");
    let data = ramp(1024);
    let name = t.dir.file("testwrite.txt");
    std::fs::write(&name, b"x").unwrap();

    // A write to a file open for reading fails (INVALID_MODE).
    let mut f = File::new();
    assert_eq!(f.open(&name, Mode::OpenRead), file::Status::OpOk);
    t.clear_history();
    t.set_abort(true);
    assert_eq!(t.comp.write_to_file(&data, &mut f, &name), 0);
    let errors = t.ground.with(FileWorker::EVENTID_WRITEFILEERROR);
    assert_eq!(errors.len(), 1);
    assert_eq!(
        errors[0][errors[0].len() - 4..],
        (file::Status::InvalidMode as i32).to_be_bytes()
    );
    t.set_abort(false);

    // Open fails.
    t.clear_history();
    assert!(!t.comp.write_buffer_to_file(
        &buffer_of(&data),
        &t.dir.file("missing/testwrite.txt"),
        0,
        false
    ));
    assert_eq!(t.ground.count(FileWorker::EVENTID_OPENFILEERROR), 1);
}

#[test]
fn test_write_err_status_reported() {
    let t = Tester::new("writeerrstatus");
    let data = ramp(1024);
    t.write_in(
        &t.dir.file("missing/testwrite.txt"),
        buffer_of(&data),
        0,
        false,
    );
    let done = t.write_done.take();
    assert_eq!(
        (done[0].0, done[0].1),
        (status(FileWorkerStatus::FailedToWrite), 0)
    );
    // The buffer still comes back.
    assert_eq!(done[0].2, data);
    assert_eq!(t.comp.state(), FileWorkerState::Idle);
}

#[test]
fn test_write_hash_err() {
    let t = Tester::new("hasherr");
    let data = vec![0u8; 1024 * 100];
    let name = t.dir.file("testfile.txt");
    std::fs::write(&name, b"x").unwrap();

    // Nominal.
    t.clear_history();
    t.comp
        .write_buffer_hash_to_file(&buffer_of(&data), &name, 0, false);
    assert_eq!(
        std::fs::read(format!("{name}.CRC32")).unwrap(),
        Hash::hash(&data).as_slice()
    );
    assert!(t.ground.events.lock().unwrap().is_empty());

    // The sidecar cannot be opened.
    t.clear_history();
    t.comp.write_buffer_hash_to_file(
        &buffer_of(&data),
        &t.dir.file("missing/testfile.txt"),
        0,
        false,
    );
    assert_eq!(t.ground.count(FileWorker::EVENTID_OPENFILEERROR), 1);

    // getHash: an open error other than DOESNT_EXIST (ENOTDIR: a path under
    // a regular file).
    let under_file = format!("{name}/x.CRC32");
    let mut hash_buffer = HashBuffer::new();
    t.clear_history();
    assert!(!t.comp.get_hash(&under_file, &mut hash_buffer, &data));
    assert_eq!(
        t.ground
            .with(FileWorker::EVENTID_WRITEVALIDATIONOPENERROR)
            .len(),
        1
    );

    // getHash: no sidecar yet hashes the data alone.
    t.clear_history();
    assert!(
        t.comp
            .get_hash(&t.dir.file("none.CRC32"), &mut hash_buffer, &data)
    );
    assert_eq!(hash_buffer, Hash::hash(&data));
    assert!(t.ground.events.lock().unwrap().is_empty());

    // getHash: a read error (the sidecar is a directory).
    let dir_sidecar = t.dir.file("dir.CRC32");
    std::fs::create_dir(&dir_sidecar).unwrap();
    t.clear_history();
    assert!(!t.comp.get_hash(&dir_sidecar, &mut hash_buffer, &data));
    assert_eq!(
        t.ground
            .with(FileWorker::EVENTID_WRITEVALIDATIONREADERROR)
            .len(),
        1
    );
}

#[test]
fn test_read_invalid_input() {
    let t = Tester::new("readinvalid");
    t.read_in("", buffer_of(&[0u8; 1024]));
    assert_eq!(
        t.ground.with(FileWorker::EVENTID_INVALIDINPUT),
        vec![invalid_input_args("readIn", "empty path")]
    );
    assert_eq!(
        t.read_done.statuses(),
        vec![(status(FileWorkerStatus::InvalidInput), 0)]
    );
    assert_eq!(t.comp.state(), FileWorkerState::Idle);

    t.clear_history();
    t.read_in(&t.dir.file("nominalread.bin"), Buffer::empty());
    assert_eq!(
        t.ground.with(FileWorker::EVENTID_INVALIDINPUT),
        vec![invalid_input_args("readIn", "invalid buffer")]
    );
    assert_eq!(
        t.read_done.statuses(),
        vec![(status(FileWorkerStatus::InvalidInput), 0)]
    );
    assert_eq!(t.comp.state(), FileWorkerState::Idle);
}

#[test]
fn test_verify_invalid_input() {
    let t = Tester::new("verifyinvalid");
    t.verify_in("", 0);
    assert_eq!(
        t.ground.with(FileWorker::EVENTID_INVALIDINPUT),
        vec![invalid_input_args("verifyIn", "empty path")]
    );
    assert_eq!(
        t.verify_done.statuses(),
        vec![(status(FileWorkerStatus::InvalidInput), 0)]
    );
    assert_eq!(t.comp.state(), FileWorkerState::Idle);
}

#[test]
fn test_write_invalid_input() {
    let t = Tester::new("writeinvalid");
    let valid_path = t.dir.file("testwrite.txt");
    let data = vec![0u8; 1024];

    t.write_in("", buffer_of(&data), 0, false);
    assert_eq!(
        t.ground.with(FileWorker::EVENTID_INVALIDINPUT),
        vec![invalid_input_args("writeIn", "empty path")]
    );
    assert_eq!(
        t.write_done.statuses(),
        vec![(status(FileWorkerStatus::InvalidInput), 0)]
    );

    t.clear_history();
    t.write_in(&valid_path, Buffer::empty(), 0, false);
    assert_eq!(
        t.ground.with(FileWorker::EVENTID_INVALIDINPUT),
        vec![invalid_input_args("writeIn", "invalid buffer")]
    );
    assert_eq!(
        t.write_done.statuses(),
        vec![(status(FileWorkerStatus::InvalidInput), 0)]
    );

    t.clear_history();
    t.write_in(&valid_path, buffer_of(&data), 1024 + 1024, false);
    assert_eq!(
        t.ground.with(FileWorker::EVENTID_INVALIDINPUT),
        vec![invalid_input_args("writeIn", "invalid offset")]
    );
    let done = t.write_done.take();
    assert_eq!(
        (done[0].0, done[0].1),
        (status(FileWorkerStatus::InvalidInput), 0)
    );
    assert_eq!(done[0].2, data);
    assert_eq!(t.comp.state(), FileWorkerState::Idle);
    assert!(!std::path::Path::new(&valid_path).exists());
}

// ----------------------------------------------------------------------
// Gotchas beyond the upstream suites
// ----------------------------------------------------------------------

#[test]
fn a_read_that_runs_past_its_timeout_reports_read_timeout() {
    let t = Tester::new("readtimeout");
    let name = write_nominal(&t.dir);
    // Every time read advances 2 s; with 100-byte chunks a 250-byte read is
    // 3 chunks, i.e. a 3 s timeout, exceeded after the second chunk.
    *t.time.step_us.lock().unwrap() = 2_000_000;
    t.read_in(&name, buffer_of(&[0u8; 250]));
    let timeouts = t.ground.with(FileWorker::EVENTID_READTIMEOUT);
    assert_eq!(timeouts.len(), 1);
    let args = &timeouts[0];
    // bytesRead (the timed-out chunk is not counted), readSize, ..., timeout.
    assert_eq!(args[..8], 100u64.to_be_bytes());
    assert_eq!(args[8..16], 250u64.to_be_bytes());
    assert_eq!(args[args.len() - 8..], 3_000_000u64.to_be_bytes());
    assert_eq!(
        t.read_done.statuses(),
        vec![(status(FileWorkerStatus::FailedToRead), 0)]
    );
}

#[test]
fn a_write_that_runs_past_its_timeout_reports_write_timeout() {
    let t = Tester::new("writetimeout");
    let name = t.dir.file("slow.bin");
    *t.time.step_us.lock().unwrap() = 1_500_000;
    // 150 bytes = 2 chunks = 2 s; elapsed 1.5 s after chunk 1, 3 s after 2.
    t.write_in(&name, buffer_of(&ramp(150)), 0, false);
    let timeouts = t.ground.with(FileWorker::EVENTID_WRITETIMEOUT);
    assert_eq!(timeouts.len(), 1);
    assert_eq!(timeouts[0][..8], 100u64.to_be_bytes());
    assert_eq!(
        timeouts[0][timeouts[0].len() - 8..],
        2_000_000u64.to_be_bytes()
    );
    assert_eq!(
        t.write_done.statuses(),
        vec![(status(FileWorkerStatus::FailedToWrite), 0)]
    );
    // Both chunks reached the disk; no sidecar was written.
    assert_eq!(std::fs::read(&name).unwrap(), ramp(150));
    assert!(!std::path::Path::new(&format!("{name}.CRC32")).exists());
}

#[test]
fn throttled_events_reopen_ten_seconds_after_the_first() {
    let t = Tester::new("throttle");
    let missing = t.dir.file("missing.bin");
    for _ in 0..4 {
        t.read_in(&missing, buffer_of(&[0u8; 8]));
    }
    // throttle 2.
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 2);
    t.time.advance(9);
    t.read_in(&missing, buffer_of(&[0u8; 8]));
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 2);
    // every {seconds=10}, measured from the first emission.
    t.time.advance(1);
    for _ in 0..3 {
        t.read_in(&missing, buffer_of(&[0u8; 8]));
    }
    assert_eq!(t.ground.count(FileWorker::EVENTID_CRCFAILED), 4);
    // Each throttle is independent; every request was still answered.
    assert_eq!(t.read_done.statuses().len(), 8);
    t.set_state(FileWorkerState::Reading);
    t.read_in(&missing, buffer_of(&[0u8; 8]));
    assert_eq!(t.ground.count(FileWorker::EVENTID_NOTINIDLE), 1);
}

#[test]
fn a_non_append_write_over_a_longer_file_keeps_its_tail() {
    // C++ parity: OPEN_WRITE does not truncate.
    let t = Tester::new("notrunc");
    let name = t.dir.file("over.bin");
    std::fs::write(&name, [0xEEu8; 300]).unwrap();
    t.write_in(&name, buffer_of(&ramp(200)), 0, false);
    let mut expected = ramp(200);
    expected.extend_from_slice(&[0xEE; 100]);
    assert_eq!(std::fs::read(&name).unwrap(), expected);
    // The sidecar holds the CRC of the written bytes only.
    assert_eq!(
        std::fs::read(format!("{name}.CRC32")).unwrap(),
        Hash::hash(&ramp(200)).as_slice()
    );
    assert_eq!(validate(&name), validate_file::Status::ValidationFail);
}

#[test]
fn append_with_an_offset_extends_the_stored_crc() {
    let t = Tester::new("appendoffset");
    let name = t.dir.file("log.bin");
    let first = ramp(300);
    let second: Vec<u8> = (0..64u8).rev().collect();
    t.write_in(&name, buffer_of(&first), 0, true);
    t.write_in(&name, buffer_of(&second), 10, true);
    let mut expected = first.clone();
    expected.extend_from_slice(&second[10..]);
    assert_eq!(std::fs::read(&name).unwrap(), expected);
    assert_eq!(validate(&name), validate_file::Status::ValidationOk);
    assert_eq!(
        t.write_done.statuses(),
        vec![
            (status(FileWorkerStatus::DoneWrite), 300),
            (status(FileWorkerStatus::DoneWrite), 54)
        ]
    );
}

#[test]
#[should_panic]
fn appending_to_a_short_sidecar_asserts() {
    // C++ parity: Utils::Hash::setHashValue FW_ASSERTs on a sidecar shorter
    // than 4 bytes.
    let t = Tester::new("shortsidecar");
    let name = t.dir.file("short.bin");
    std::fs::write(format!("{name}.CRC32"), [1u8, 2]).unwrap();
    t.write_in(&name, buffer_of(&ramp(10)), 0, true);
}

#[test]
fn a_read_of_a_file_shorter_than_the_buffer_is_an_error() {
    let t = Tester::new("shortfile");
    let name = t.dir.file("short.bin");
    std::fs::write(&name, ramp(150)).unwrap();
    assert_eq!(
        crc_checker::create_checksum_file(&name),
        CrcStat::PassedFileCrcWrite
    );
    t.read_in(&name, buffer_of(&[0u8; 400]));
    let errors = t.ground.with(FileWorker::EVENTID_READERROR);
    assert_eq!(errors.len(), 1);
    // bytesRead counts the short chunk.
    assert_eq!(errors[0][..8], 150u64.to_be_bytes());
    assert_eq!(
        t.read_done.statuses(),
        vec![(status(FileWorkerStatus::FailedToRead), 0)]
    );
}

#[test]
fn cmd_in_is_ignored_and_has_no_registrations() {
    let t = Tester::new("cmdin");
    t.comp.reg_commands();
    let mut args = CmdArgBuffer::new();
    let port = t.comp.cmd_in(0);
    port.target.invoke(port.port_num, ID_BASE + 3, 1, &mut args);
    assert!(t.ground.events.lock().unwrap().is_empty());
}

#[test]
#[should_panic]
fn cmd_in_asserts_on_an_opcode_below_the_id_base() {
    let t = Tester::new("cmdbelow");
    let mut args = CmdArgBuffer::new();
    let port = t.comp.cmd_in(0);
    port.target.invoke(port.port_num, ID_BASE - 1, 1, &mut args);
}

#[test]
fn configure_sets_the_chunk_size() {
    let t = Tester::new("chunks");
    let name = t.dir.file("chunks.bin");
    t.comp.configure(7);
    *t.time.step_us.lock().unwrap() = 600_000;
    // 20 bytes in 7-byte chunks = 3 chunks = 3 s; never exceeded at
    // 0.6 s per chunk.
    t.write_in(&name, buffer_of(&ramp(20)), 0, false);
    assert_eq!(
        t.write_done.statuses(),
        vec![(status(FileWorkerStatus::DoneWrite), 20)]
    );
    assert_eq!(FileWorker::BLOCK_SIZE_BYTES, 4096);
}

#[test]
#[should_panic]
fn configure_asserts_on_a_zero_chunk_size() {
    let t = Tester::new("zerochunk");
    t.comp.configure(0);
}

#[test]
fn write_in_envelope_bytes() {
    let t = Tester::new("envelope");
    let port = t.comp.write_in(0);
    port.target.invoke(
        port.port_num,
        &FileNameString::from("ab"),
        buffer_of(&[9]),
        1,
        true,
    );
    let mut dest = [0u8; QUEUE_MSG_SIZE];
    let mut size: FwSizeType = 0;
    let mut priority: FwQueuePriorityType = 0;
    let status = t.comp.active.queued.queue().receive(
        &mut dest,
        BlockingType::NonBlocking,
        &mut size,
        &mut priority,
    );
    assert_eq!(status, fprime_os::queue::Status::OpOk);
    let bytes = &dest[..size as usize];
    // [msg_type][port_num][path][token][offsetBytes][append]
    assert_eq!(bytes[..10], [0, 0, 0, 3, 0, 0, 0, 2, b'a', b'b']);
    assert_eq!(bytes[18..], [0, 0, 0, 0, 0, 0, 0, 1, 0xFF]);
    let token = u64::from_be_bytes(bytes[10..18].try_into().unwrap());
    assert_eq!(t.comp.escrow.claim(token).data(), &[9]);
}

#[test]
fn enum_values_and_dictionary_ids_match_upstream() {
    assert_eq!(FileWorkerState::Idle as u32, 0);
    assert_eq!(FileWorkerState::Writing as u32, 1);
    assert_eq!(FileWorkerState::Reading as u32, 2);
    assert_eq!(FileWorkerStatus::NotIdle as u32, 0);
    assert_eq!(FileWorkerStatus::FailedToOpen as u32, 2);
    assert_eq!(FileWorkerStatus::FailedToWrite as u32, 3);
    assert_eq!(FileWorkerStatus::FailedToRead as u32, 4);
    assert_eq!(FileWorkerStatus::FailedCrc as u32, 7);
    assert_eq!(FileWorkerStatus::FailedFileSize as u32, 8);
    assert_eq!(FileWorkerStatus::DoneWrite as u32, 12);
    assert_eq!(FileWorkerStatus::DoneRead as u32, 14);
    assert_eq!(FileWorkerStatus::Done as u32, 15);
    assert_eq!(FileWorkerStatus::InvalidInput as u32, 16);
    assert_eq!(FileWorkerReadStatus::Done as u32, 0);
    assert_eq!(FileWorkerReadStatus::Unknown as u32, 4);
    let ids = [
        FileWorker::EVENTID_NOTINIDLE,
        FileWorker::EVENTID_CRCFAILED,
        FileWorker::EVENTID_CRCVERIFICATIONERROR,
        FileWorker::EVENTID_READFAILEDFILESIZE,
        FileWorker::EVENTID_OPENFILEERROR,
        FileWorker::EVENTID_READBEGIN,
        FileWorker::EVENTID_READCOMPLETED,
        FileWorker::EVENTID_READERROR,
        FileWorker::EVENTID_READABORTED,
        FileWorker::EVENTID_READTIMEOUT,
        FileWorker::EVENTID_WRITEBEGIN,
        FileWorker::EVENTID_WRITECOMPLETED,
        FileWorker::EVENTID_WRITEFILEERROR,
        FileWorker::EVENTID_WRITEVALIDATIONOPENERROR,
        FileWorker::EVENTID_WRITEVALIDATIONREADERROR,
        FileWorker::EVENTID_WRITEVALIDATIONERROR,
        FileWorker::EVENTID_WRITETIMEOUT,
        FileWorker::EVENTID_WRITEABORTED,
        FileWorker::EVENTID_INVALIDINPUT,
    ];
    for (expected, id) in ids.iter().enumerate() {
        assert_eq!(*id, expected as FwEventIdType);
    }
    assert_eq!(FileWorker::EVENTID_THROTTLE, 2);
    assert_eq!(FileWorker::MSG_TYPE_READ_IN, 1);
    assert_eq!(FileWorker::MSG_TYPE_VERIFY_IN, 2);
    assert_eq!(FileWorker::MSG_TYPE_WRITE_IN, 3);
}
