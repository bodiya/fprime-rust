//! Unit tests for `Svc::BufferLogger`: ports of upstream
//! `Svc/BufferLogger/test/ut/{BufferLoggerMain,BufferLoggerTester,Logging,
//! Errors,Health}.cpp`, each in its own temporary directory (upstream works
//! in a shared `buf/` directory), plus the gotchas from the module docs.

use super::*;
use fprime_comp::{CmdRegPort, CmdResponsePort, LogPort, LogTextPort};
use fprime_fw::{TextLogString, Time};
use fprime_os::queue::BlockingType;
use std::path::PathBuf;
use std::sync::Mutex as StdMutex;
use std::sync::atomic::{AtomicU32, Ordering};

const ID_BASE: u32 = 0x5500;

/// Upstream `COM_BUFFER_LENGTH`.
const COM_BUFFER_LENGTH: usize = 4;
/// Upstream `MAX_ENTRIES_PER_FILE`.
const MAX_ENTRIES_PER_FILE: u32 = 5;
/// Upstream `SIZE_TYPE` (`U32`).
const SIZE_OF_SIZE: u8 = 4;
/// Upstream `MAX_BYTES_PER_FILE`.
const MAX_BYTES_PER_FILE: FwSizeType = (MAX_ENTRIES_PER_FILE as FwSizeType)
    * (COM_BUFFER_LENGTH as FwSizeType + SIZE_OF_SIZE as FwSizeType);
/// Upstream `BufferLoggerTester::data`.
const DATA: [u8; COM_BUFFER_LENGTH] = [0xDE, 0xAD, 0xBE, 0xEF];

/// A unique scratch directory, removed on drop.
struct TempDir {
    path: PathBuf,
}

impl TempDir {
    fn new(tag: &str) -> Self {
        static COUNTER: AtomicU32 = AtomicU32::new(0);
        let path = std::env::temp_dir().join(format!(
            "fprime_rust_bufferlogger_{tag}_{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::Relaxed)
        ));
        let _ = std::fs::remove_dir_all(&path);
        std::fs::create_dir_all(&path).unwrap();
        Self { path }
    }

    /// The tests' `buf` directory.
    fn buf(&self) -> PathBuf {
        self.path.join("buf")
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let buf = self.buf();
        if buf.exists() {
            // Undo a chmod -w from the validation test.
            let mut perms = std::fs::metadata(&buf).unwrap().permissions();
            #[allow(clippy::permissions_set_readonly_false)]
            perms.set_readonly(false);
            let _ = std::fs::set_permissions(&buf, perms);
        }
        let _ = std::fs::remove_dir_all(&self.path);
    }
}

/// (id, severity, raw arg bytes)
type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

#[derive(Default)]
struct Ground {
    events: StdMutex<Vec<EventRecord>>,
    regs: StdMutex<Vec<FwOpcodeType>>,
    responses: StdMutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
    pings: StdMutex<Vec<u32>>,
    forwarded: StdMutex<Vec<Vec<u8>>>,
}

impl Ground {
    fn clear(&self) {
        self.events.lock().unwrap().clear();
        self.responses.lock().unwrap().clear();
        self.pings.lock().unwrap().clear();
        self.forwarded.lock().unwrap().clear();
    }

    fn events_size(&self) -> usize {
        self.events.lock().unwrap().len()
    }

    fn events_with(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
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

impl CmdRegPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
        self.regs.lock().unwrap().push(op_code);
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
        self.responses
            .lock()
            .unwrap()
            .push((op_code, cmd_seq, response));
    }
}

impl PingPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, key: u32) {
        self.pings.lock().unwrap().push(key);
    }
}

impl BufferSendPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
        self.forwarded.lock().unwrap().push(buffer.data().to_vec());
    }
}

/// Upstream `BufferLoggerTester`.
struct Tester {
    comp: Arc<BufferLogger>,
    ground: Arc<Ground>,
    dir: TempDir,
}

impl Tester {
    /// Upstream constructor: a fresh `buf` directory; `initLog("buf/log",
    /// ".buf", MAX_BYTES_PER_FILE, sizeof(SIZE_TYPE))` when `do_init_log`.
    fn new(tag: &str, do_init_log: bool) -> Self {
        let dir = TempDir::new(tag);
        std::fs::create_dir_all(dir.buf()).unwrap();
        let comp = BufferLogger::new("BufferLogger");
        let ground = Arc::new(Ground::default());
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.buffer_send_out.connect(ground.clone(), 0);
        comp.ping_out.connect(ground.clone(), 0);
        comp.init(10);
        if do_init_log {
            comp.init_log(
                &format!("{}/log", dir.buf().to_str().unwrap()),
                ".buf",
                MAX_BYTES_PER_FILE,
                SIZE_OF_SIZE,
            );
        }
        Self { comp, ground, dir }
    }

    fn dispatch_one(&self) {
        assert_eq!(
            self.comp
                .active
                .queued
                .do_dispatch(self.comp.as_ref(), BlockingType::NonBlocking),
            MsgDispatchStatus::Ok
        );
    }

    fn set_base_name(&self, name: &str) {
        self.comp.state.lock().file.base_name.set(name);
    }

    fn prefix(&self) -> String {
        self.comp.state.lock().file.prefix.to_string()
    }

    fn suffix(&self) -> String {
        self.comp.state.lock().file.suffix.to_string()
    }

    fn mode(&self) -> FileMode {
        self.comp.state.lock().file.mode
    }

    fn name(&self) -> String {
        self.comp.file_name().to_string()
    }

    fn send_com_buffers(&self, n: u32) {
        for _ in 0..n {
            let mut buffer = ComBuffer::new();
            assert!(buffer.set_buff(&DATA).is_ok());
            let port = self.comp.com_in(0);
            port.target.invoke(port.port_num, &mut buffer, 0);
            self.dispatch_one();
        }
    }

    fn send_managed_buffers(&self, n: u32) {
        for _ in 0..n {
            let buffer = Buffer::from_storage(DATA.to_vec().into_boxed_slice(), 0);
            let port = self.comp.buffer_send_in(0);
            port.target.invoke(port.port_num, buffer);
            self.dispatch_one();
        }
    }

    fn send_cmd(&self, opcode: FwOpcodeType, cmd_seq: u32, arg_bytes: &[u8]) {
        let mut args = CmdArgBuffer::new();
        assert!(args.set_buff(arg_bytes).is_ok());
        let port = self.comp.cmd_in(0);
        port.target
            .invoke(port.port_num, ID_BASE + opcode, cmd_seq, &mut args);
        self.dispatch_one();
    }

    fn send_open_file(&self, cmd_seq: u32, base_name: &str) {
        let mut arg = vec![0u8, base_name.len() as u8];
        arg.extend_from_slice(base_name.as_bytes());
        self.send_cmd(BufferLogger::OPCODE_BL_OPENFILE, cmd_seq, &arg);
    }

    fn send_close_file(&self, cmd_seq: u32) {
        self.send_cmd(BufferLogger::OPCODE_BL_CLOSEFILE, cmd_seq, &[]);
    }

    fn set_state(&self, state: LogState) {
        self.ground.clear();
        self.send_cmd(BufferLogger::OPCODE_BL_SETLOGGING, 0, &[state as u8]);
        assert_eq!(
            *self.ground.responses.lock().unwrap(),
            vec![(
                ID_BASE + BufferLogger::OPCODE_BL_SETLOGGING,
                0,
                CmdResponse::Ok
            )]
        );
    }

    /// Upstream `checkLogFileIntegrity`: `expected_num_buffers` records of
    /// a 4-byte big-endian size and `DATA`, then end of file.
    fn check_log_file_integrity(
        file_name: &str,
        expected_size: FwSizeType,
        expected_num_buffers: u32,
    ) {
        let bytes = std::fs::read(file_name).unwrap();
        assert!(expected_size <= bytes.len() as FwSizeType);
        let mut record = (COM_BUFFER_LENGTH as u32).to_be_bytes().to_vec();
        record.extend_from_slice(&DATA);
        assert_eq!(bytes, record.repeat(expected_num_buffers as usize));
    }

    /// Upstream `checkFileValidation`.
    fn check_file_validation(file_name: &str) {
        let mut validated_file = ValidatedFile::new(file_name);
        assert_eq!(
            validated_file.validate(),
            validate_file::Status::ValidationOk
        );
    }

    /// Upstream `checkFileExists` + `checkHashFileExists`.
    fn check_files_exist(file_name: &str) {
        assert!(std::path::Path::new(file_name).is_file(), "{file_name}");
        let validated_file = ValidatedFile::new(file_name);
        let hash_name = validated_file.get_hash_file_name().to_string();
        assert!(std::path::Path::new(&hash_name).is_file(), "{hash_name}");
    }
}

/// The `BL_LogFileClosed` argument bytes for a name.
fn string_arg(name: &str) -> Vec<u8> {
    let mut bytes = (name.len() as u16).to_be_bytes().to_vec();
    bytes.extend_from_slice(name.as_bytes());
    bytes
}

/// The `BL_LogFileWriteError` argument bytes.
fn write_error_args(errornum: file::Status, written: u32, to_write: u32, name: &str) -> Vec<u8> {
    let mut bytes = (errornum as u32).to_be_bytes().to_vec();
    bytes.extend_from_slice(&written.to_be_bytes());
    bytes.extend_from_slice(&to_write.to_be_bytes());
    bytes.extend_from_slice(&string_arg(name));
    bytes
}

// ----------------------------------------------------------------------
// BufferLoggerMain.cpp: LogNoInit
// ----------------------------------------------------------------------

#[test]
fn log_no_init() {
    let t = Tester::new("noinit", false);
    t.set_base_name("LogNoInit");
    t.send_com_buffers(3);
    assert_eq!(t.ground.events_size(), 3);
    assert_eq!(
        t.ground
            .events_with(BufferLogger::EVENTID_BL_NOLOGFILEOPENINITERROR)
            .len(),
        3
    );
}

// ----------------------------------------------------------------------
// Errors.cpp
// ----------------------------------------------------------------------

#[test]
fn errors_log_file_open() {
    let t = Tester::new("openerr", true);
    std::fs::remove_dir_all(t.dir.buf()).unwrap();
    t.set_base_name("LogFileOpen");
    assert_eq!(t.mode(), FileMode::Closed);
    assert_eq!(t.ground.events_size(), 0);

    t.send_com_buffers(3);
    // Not throttled.
    let errors = t
        .ground
        .events_with(BufferLogger::EVENTID_BL_LOGFILEOPENERROR);
    assert_eq!(t.ground.events_size(), 3);
    assert_eq!(errors.len(), 3);
    for args in &errors {
        let mut expected = (file::Status::DoesntExist as u32).to_be_bytes().to_vec();
        expected.extend_from_slice(&string_arg(&t.name()));
        assert_eq!(*args, expected);
    }

    // Create the directory and try again: no new events.
    std::fs::create_dir_all(t.dir.buf()).unwrap();
    assert_eq!(t.mode(), FileMode::Closed);
    t.send_com_buffers(3);
    assert_eq!(t.ground.events_size(), 3);
    {
        let mut state = t.comp.state.lock();
        t.comp.close(&mut state.file);
    }

    // Remove the directory and try again: three more.
    std::fs::remove_dir_all(t.dir.buf()).unwrap();
    assert_eq!(t.mode(), FileMode::Closed);
    t.send_com_buffers(3);
    assert_eq!(t.ground.events_size(), 6);
    assert_eq!(
        t.ground
            .events_with(BufferLogger::EVENTID_BL_LOGFILEOPENERROR)
            .len(),
        6
    );
}

#[test]
fn errors_log_file_write() {
    let t = Tester::new("writeerr", true);
    assert_eq!(t.mode(), FileMode::Closed);
    assert_eq!(t.ground.events_size(), 0);
    t.set_base_name("LogFileWrite");

    t.send_com_buffers(1);
    // Force close the file underneath the component.
    t.comp.state.lock().file.os_file.close();
    t.send_com_buffers(1);

    let file_name = format!("{}LogFileWrite{}", t.prefix(), t.suffix());
    // Not throttled; the size field is the write that fails.
    assert_eq!(t.ground.events_size(), 1);
    assert_eq!(
        t.ground
            .events_with(BufferLogger::EVENTID_BL_LOGFILEWRITEERROR),
        vec![write_error_args(
            file::Status::NotOpened,
            0,
            u32::from(SIZE_OF_SIZE),
            &file_name
        )]
    );

    // Make the logger open a new file: the counter has incremented.
    {
        let mut state = t.comp.state.lock();
        state.file.mode = FileMode::Closed;
        t.comp.open(&mut state.file);
    }
    let file_name = format!("{}LogFileWrite1{}", t.prefix(), t.suffix());
    t.send_com_buffers(3);
    assert_eq!(t.ground.events_size(), 1);

    // Force close again: three more, not throttled.
    t.comp.state.lock().file.os_file.close();
    t.send_com_buffers(3);
    let errors = t
        .ground
        .events_with(BufferLogger::EVENTID_BL_LOGFILEWRITEERROR);
    assert_eq!(t.ground.events_size(), 4);
    for args in &errors[1..] {
        assert_eq!(
            *args,
            write_error_args(
                file::Status::NotOpened,
                0,
                u32::from(SIZE_OF_SIZE),
                &file_name
            )
        );
    }
}

/// True when the process can still create files in a directory without
/// write permission (root), where upstream skips the test.
fn permission_checks_bypassed(dir: &std::path::Path) -> bool {
    let probe = dir.join("probe");
    let bypassed = std::fs::write(&probe, b"x").is_ok();
    let _ = std::fs::remove_file(probe);
    bypassed
}

#[test]
fn errors_log_file_validation() {
    let t = Tester::new("valerr", true);
    t.set_base_name("LogFileValidation");
    t.send_com_buffers(1);

    // Remove write permission on the buf directory.
    let buf = t.dir.buf();
    let mut perms = std::fs::metadata(&buf).unwrap().permissions();
    perms.set_readonly(true);
    std::fs::set_permissions(&buf, perms).unwrap();
    if permission_checks_bypassed(&buf) {
        eprintln!("skipped: running with permission checks bypassed (root)");
        return;
    }

    t.send_close_file(0);
    assert_eq!(t.ground.events_size(), 2);
    let file_name = format!("{}LogFileValidation{}", t.prefix(), t.suffix());
    assert_eq!(
        t.ground.events_with(BufferLogger::EVENTID_BL_LOGFILECLOSED),
        vec![string_arg(&file_name)]
    );
    let validated_file = ValidatedFile::new(&file_name);
    let mut expected = string_arg(&validated_file.get_hash_file_name().to_string());
    expected.extend_from_slice(
        &(validate_file::Status::ValidationFileNoPermission as u32).to_be_bytes(),
    );
    assert_eq!(
        t.ground
            .events_with(BufferLogger::EVENTID_BL_LOGFILEVALIDATIONERROR),
        vec![expected]
    );
    // C++ order: the hash file is written (and fails) before the close
    // event.
    let events = t.ground.events.lock().unwrap();
    assert_eq!(
        events[0].0,
        ID_BASE + BufferLogger::EVENTID_BL_LOGFILEVALIDATIONERROR
    );
    assert_eq!(events[0].1, LogSeverity::WarningHi);
}

// ----------------------------------------------------------------------
// Logging.cpp
// ----------------------------------------------------------------------

/// Upstream `CloseFileTester::test`.
#[test]
fn logging_close_file() {
    let t = Tester::new("close", true);
    t.set_base_name("CloseFileTester");
    assert_eq!(t.ground.events_size(), 0);

    let send_close_file_commands = |n: u32| {
        t.ground.clear();
        for i in 0..n {
            t.send_close_file(i);
        }
        let responses = t.ground.responses.lock().unwrap();
        assert_eq!(responses.len(), n as usize);
        for (i, response) in responses.iter().enumerate() {
            assert_eq!(
                *response,
                (
                    ID_BASE + BufferLogger::OPCODE_BL_CLOSEFILE,
                    i as u32,
                    CmdResponse::Ok
                )
            );
        }
    };

    send_close_file_commands(3);
    t.send_com_buffers(3);
    send_close_file_commands(3);
    assert_eq!(t.ground.events_size(), 1);
    assert_eq!(
        t.ground.events_with(BufferLogger::EVENTID_BL_LOGFILECLOSED),
        vec![string_arg(&t.name())]
    );
    Tester::check_files_exist(&t.name());
}

/// Upstream `SendBuffersTester::test`.
fn send_buffers_test(t: &Tester, num_files: u32, base_name: &str, send: &dyn Fn(&Tester, u32)) {
    t.send_open_file(0, base_name);
    let mut current = format!("{}{base_name}{}", t.prefix(), t.suffix());
    send(t, 1);
    // The 0th event (file open) has already happened.
    for i in 1..=num_files {
        assert_eq!(current, t.name());
        send(t, MAX_ENTRIES_PER_FILE - 1);
        assert_eq!(current, t.name());
        // This opens a new file with the updated counter.
        send(t, 1);
        current = format!("{}{base_name}{i}{}", t.prefix(), t.suffix());
        assert_eq!(t.mode(), FileMode::Open);
        assert_eq!(current, t.name());
        assert_eq!(t.ground.events_size(), i as usize);
        assert_eq!(
            t.ground
                .events_with(BufferLogger::EVENTID_BL_LOGFILECLOSED)
                .len(),
            i as usize
        );
    }

    t.send_close_file(0);

    let closed = t.ground.events_with(BufferLogger::EVENTID_BL_LOGFILECLOSED);
    for i in 0..num_files {
        let file_name = if i == 0 {
            format!("{}{base_name}{}", t.prefix(), t.suffix())
        } else {
            format!("{}{base_name}{i}{}", t.prefix(), t.suffix())
        };
        assert_eq!(closed[i as usize], string_arg(&file_name));
        Tester::check_log_file_integrity(&file_name, MAX_BYTES_PER_FILE, MAX_ENTRIES_PER_FILE);
        Tester::check_file_validation(&file_name);
    }
}

#[test]
fn logging_com_in() {
    let t = Tester::new("comin", true);
    send_buffers_test(&t, 3, "ComIn", &|t, n| t.send_com_buffers(n));
    // Com buffers are not forwarded.
    assert!(t.ground.forwarded.lock().unwrap().is_empty());
}

#[test]
fn logging_buffer_send_in() {
    let t = Tester::new("bufsendin", true);
    send_buffers_test(&t, 3, "BufferSendIn", &|t, n| t.send_managed_buffers(n));
    // Every buffer is forwarded on bufferSendOut.
    let forwarded = t.ground.forwarded.lock().unwrap();
    assert_eq!(forwarded.len(), 1 + 3 * MAX_ENTRIES_PER_FILE as usize);
    assert!(forwarded.iter().all(|b| b == &DATA));
}

/// Upstream `OnOffTester`.
#[test]
fn logging_on_off() {
    {
        // testLoggingOn
        let t = Tester::new("on", true);
        t.set_base_name("OnOffTester");
        t.send_com_buffers(MAX_ENTRIES_PER_FILE);
        t.set_state(LogState::LoggingOff);
        Tester::check_log_file_integrity(&t.name(), MAX_BYTES_PER_FILE, MAX_ENTRIES_PER_FILE);
        assert!(!t.comp.is_file_open());
        assert_eq!(t.comp.log_state(), LogState::LoggingOff);
    }
    {
        // testLoggingOff
        let t = Tester::new("off", true);
        t.set_state(LogState::LoggingOff);
        t.send_com_buffers(MAX_ENTRIES_PER_FILE);
        assert_eq!(t.ground.events_size(), 0);
        t.set_state(LogState::LoggingOn);
        // No BL_Activated / BL_Deactivated (C++ parity).
        assert_eq!(t.ground.events_size(), 0);
    }
}

// ----------------------------------------------------------------------
// Health.cpp
// ----------------------------------------------------------------------

#[test]
fn health_ping() {
    let t = Tester::new("ping", true);
    let port = t.comp.ping_in(0);
    port.target.invoke(port.port_num, 42);
    t.dispatch_one();
    assert_eq!(t.ground.events_size(), 0);
    assert_eq!(*t.ground.pings.lock().unwrap(), vec![42]);
}

// ----------------------------------------------------------------------
// Gotchas beyond the upstream suite
// ----------------------------------------------------------------------

#[test]
fn size_field_width_is_configurable_and_big_endian() {
    for size_of_size in [0u8, 1, 2, 8] {
        let t = Tester::new("width", false);
        let path = format!("{}/w", t.dir.buf().to_str().unwrap());
        t.comp.init_log(&path, ".dat", 1024, size_of_size);
        t.send_open_file(0, "x");
        // A size of zero fits any width; the 4-byte record needs >= 1.
        let mut empty = ComBuffer::new();
        let port = t.comp.com_in(0);
        port.target.invoke(port.port_num, &mut empty, 0);
        t.dispatch_one();
        t.send_com_buffers(1);
        t.send_close_file(1);
        let bytes = std::fs::read(format!("{path}x.dat")).unwrap();
        let field = |size: u64| size.to_be_bytes()[8 - usize::from(size_of_size)..].to_vec();
        let mut expected = field(0);
        if size_of_size > 0 {
            expected.extend(field(COM_BUFFER_LENGTH as u64));
            expected.extend_from_slice(&DATA);
        }
        assert_eq!(bytes, expected, "size_of_size {size_of_size}");
        if size_of_size == 0 {
            // The 4-byte record does not fit a 0-byte size field.
            assert_eq!(
                t.ground
                    .events_with(BufferLogger::EVENTID_BL_LOGFILEWRITEERROR),
                vec![write_error_args(
                    file::Status::InvalidArgument,
                    0,
                    COM_BUFFER_LENGTH as u32,
                    &format!("{path}x.dat")
                )]
            );
        }
    }
}

#[test]
fn a_size_that_does_not_fit_the_field_is_rejected_and_the_file_stays_open() {
    let t = Tester::new("toobig", false);
    let path = format!("{}/big", t.dir.buf().to_str().unwrap());
    t.comp.init_log(&path, "", 4096, 1);
    t.send_open_file(0, "f");
    let payload = vec![0x5Au8; 300];
    let port = t.comp.buffer_send_in(0);
    port.target.invoke(
        port.port_num,
        Buffer::from_storage(payload.clone().into_boxed_slice(), 0),
    );
    t.dispatch_one();
    assert_eq!(
        t.ground
            .events_with(BufferLogger::EVENTID_BL_LOGFILEWRITEERROR),
        vec![write_error_args(
            file::Status::InvalidArgument,
            0,
            300,
            &format!("{path}f")
        )]
    );
    assert!(t.comp.is_file_open());
    // Still forwarded.
    assert_eq!(*t.ground.forwarded.lock().unwrap(), vec![payload]);
    t.send_com_buffers(1);
    t.send_close_file(1);
    let mut expected = vec![COM_BUFFER_LENGTH as u8];
    expected.extend_from_slice(&DATA);
    assert_eq!(std::fs::read(format!("{path}f")).unwrap(), expected);
    Tester::check_file_validation(&format!("{path}f"));
}

#[test]
fn open_file_closes_the_current_file_and_resets_the_counter() {
    let t = Tester::new("reopen", true);
    t.send_open_file(0, "A");
    // Roll over to A1.
    t.send_com_buffers(MAX_ENTRIES_PER_FILE + 1);
    assert_eq!(t.name(), format!("{}A1{}", t.prefix(), t.suffix()));
    t.ground.clear();
    t.send_open_file(1, "B");
    assert_eq!(
        t.ground.events_with(BufferLogger::EVENTID_BL_LOGFILECLOSED),
        vec![string_arg(&format!("{}A1{}", t.prefix(), t.suffix()))]
    );
    // B is open at once, with no counter.
    assert!(t.comp.is_file_open());
    assert_eq!(t.name(), format!("{}B{}", t.prefix(), t.suffix()));
    assert_eq!(
        *t.ground.responses.lock().unwrap(),
        vec![(
            ID_BASE + BufferLogger::OPCODE_BL_OPENFILE,
            1,
            CmdResponse::Ok
        )]
    );
}

#[test]
fn reopening_an_existing_file_overwrites_without_truncating() {
    // C++ parity: Os::File OPEN_WRITE is O_CREAT without O_TRUNC.
    let t = Tester::new("notrunc", true);
    let name = format!("{}/logOld.buf", t.dir.buf().to_str().unwrap());
    std::fs::write(&name, [0xAAu8; 12]).unwrap();
    t.send_open_file(0, "Old");
    t.send_com_buffers(1);
    t.send_close_file(1);
    let mut expected = (COM_BUFFER_LENGTH as u32).to_be_bytes().to_vec();
    expected.extend_from_slice(&DATA);
    expected.extend_from_slice(&[0xAA; 4]);
    assert_eq!(std::fs::read(&name).unwrap(), expected);
    // The hash covers only what was written, so the file no longer
    // validates.
    let mut validated_file = ValidatedFile::new(&name);
    assert_eq!(
        validated_file.validate(),
        validate_file::Status::ValidationFail
    );
}

#[test]
fn an_overlong_file_name_is_a_name_error() {
    let t = Tester::new("longname", false);
    let prefix = "p".repeat(FwDefaultString::max_length() - 3);
    t.comp.init_log(&prefix, ".suffix", 100, 2);
    t.send_open_file(0, "base");
    assert_eq!(
        t.ground
            .events_with(BufferLogger::EVENTID_BL_LOGFILENAMEERROR),
        vec![vec![StringFormatStatus::Overflowed as u8]]
    );
    assert!(!t.comp.is_file_open());
    // The command itself still answers OK.
    assert_eq!(t.ground.responses.lock().unwrap()[0].2, CmdResponse::Ok);
}

#[test]
fn flush_always_succeeds_and_commands_check_their_arguments() {
    let t = Tester::new("cmds", true);
    t.comp.reg_commands();
    assert_eq!(
        *t.ground.regs.lock().unwrap(),
        vec![ID_BASE, ID_BASE + 1, ID_BASE + 2, ID_BASE + 3]
    );
    t.send_cmd(BufferLogger::OPCODE_BL_FLUSHFILE, 1, &[]);
    t.send_cmd(BufferLogger::OPCODE_BL_FLUSHFILE, 2, &[0]);
    t.send_cmd(BufferLogger::OPCODE_BL_CLOSEFILE, 3, &[0]);
    t.send_cmd(BufferLogger::OPCODE_BL_SETLOGGING, 4, &[2]);
    t.send_cmd(BufferLogger::OPCODE_BL_SETLOGGING, 5, &[]);
    t.send_cmd(BufferLogger::OPCODE_BL_OPENFILE, 6, &[0, 5, b'a']);
    t.send_cmd(0x20, 7, &[]);
    let r = |op: FwOpcodeType, seq, resp| (ID_BASE + op, seq, resp);
    assert_eq!(
        *t.ground.responses.lock().unwrap(),
        vec![
            r(BufferLogger::OPCODE_BL_FLUSHFILE, 1, CmdResponse::Ok),
            r(
                BufferLogger::OPCODE_BL_FLUSHFILE,
                2,
                CmdResponse::FormatError
            ),
            r(
                BufferLogger::OPCODE_BL_CLOSEFILE,
                3,
                CmdResponse::FormatError
            ),
            r(
                BufferLogger::OPCODE_BL_SETLOGGING,
                4,
                CmdResponse::ValidationError
            ),
            r(
                BufferLogger::OPCODE_BL_SETLOGGING,
                5,
                CmdResponse::FormatError
            ),
            r(
                BufferLogger::OPCODE_BL_OPENFILE,
                6,
                CmdResponse::FormatError
            ),
            r(0x20, 7, CmdResponse::InvalidOpcode),
        ]
    );
    assert_eq!(t.comp.log_state(), LogState::LoggingOn);
}

#[test]
fn logging_off_still_forwards_buffers() {
    let t = Tester::new("offfwd", true);
    t.set_base_name("Off");
    t.set_state(LogState::LoggingOff);
    t.send_managed_buffers(2);
    assert_eq!(t.ground.forwarded.lock().unwrap().len(), 2);
    assert!(!t.comp.is_file_open());
    assert_eq!(t.ground.events_size(), 0);
}

#[test]
fn sched_in_does_nothing() {
    let t = Tester::new("sched", true);
    let port = t.comp.sched_in(0);
    port.target.invoke(port.port_num, 7);
    t.dispatch_one();
    assert_eq!(t.ground.events_size(), 0);
    assert!(!t.comp.is_file_open());
}

#[test]
fn dropping_the_component_writes_the_hash_file_without_an_event() {
    let t = Tester::new("drop", true);
    t.send_open_file(0, "Drop");
    t.send_com_buffers(2);
    let name = t.name();
    t.ground.clear();
    let Tester { comp, ground, dir } = t;
    drop(comp);
    assert!(ground.events.lock().unwrap().is_empty());
    Tester::check_log_file_integrity(&name, 16, 2);
    Tester::check_file_validation(&name);
    drop(dir);
}

#[test]
#[should_panic]
fn a_storage_less_buffer_asserts_while_logging() {
    // C++ parity: File::logBuffer FW_ASSERTs data != nullptr.
    let t = Tester::new("nullbuf", true);
    let port = t.comp.buffer_send_in(0);
    port.target.invoke(port.port_num, Buffer::empty());
    t.dispatch_one();
}

#[test]
#[should_panic]
fn init_log_asserts_when_the_size_field_is_too_wide() {
    let t = Tester::new("wide", false);
    t.comp.init_log("p", "s", 100, MAX_SIZE_OF_SIZE + 1);
}

#[test]
#[should_panic]
fn init_log_asserts_when_the_max_size_does_not_exceed_the_size_field() {
    let t = Tester::new("small", false);
    t.comp.init_log("p", "s", 4, 4);
}

#[test]
fn buffer_send_in_envelope_carries_the_escrow_token() {
    let t = Tester::new("envelope", true);
    let port = t.comp.buffer_send_in(0);
    port.target.invoke(
        port.port_num,
        Buffer::from_storage(vec![1, 2].into_boxed_slice(), 0),
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
    // [msg_type][port_num][u64 token]
    assert_eq!(size, 14);
    assert_eq!(dest[..6], [0, 0, 0, 1, 0, 0]);
    assert_eq!(priority, QUEUE_PRIORITY);
    // Claim the buffer back so the escrow is empty.
    let token = u64::from_be_bytes(dest[6..14].try_into().unwrap());
    assert_eq!(t.comp.escrow.claim(token).data(), &[1, 2]);
}

#[test]
fn dictionary_ids_match_the_fpp_model() {
    assert_eq!(BufferLogger::OPCODE_BL_OPENFILE, 0x00);
    assert_eq!(BufferLogger::OPCODE_BL_CLOSEFILE, 0x01);
    assert_eq!(BufferLogger::OPCODE_BL_SETLOGGING, 0x02);
    assert_eq!(BufferLogger::OPCODE_BL_FLUSHFILE, 0x03);
    assert_eq!(BufferLogger::EVENTID_BL_LOGFILECLOSED, 0x00);
    assert_eq!(BufferLogger::EVENTID_BL_LOGFILEOPENERROR, 0x01);
    assert_eq!(BufferLogger::EVENTID_BL_LOGFILEVALIDATIONERROR, 0x02);
    assert_eq!(BufferLogger::EVENTID_BL_LOGFILEWRITEERROR, 0x03);
    assert_eq!(BufferLogger::EVENTID_BL_ACTIVATED, 0x04);
    assert_eq!(BufferLogger::EVENTID_BL_DEACTIVATED, 0x05);
    assert_eq!(BufferLogger::EVENTID_BL_NOLOGFILEOPENINITERROR, 0x06);
    assert_eq!(BufferLogger::EVENTID_BL_LOGFILENAMEERROR, 0x07);
    assert_eq!(BufferLogger::CHANID_BUFFERLOGGER_NUMLOGGEDBUFFERS, 0x00);
    assert_eq!(LogState::LoggingOn as u8, 0);
    assert_eq!(LogState::LoggingOff as u8, 1);
    assert_eq!(BufferLogger::MSG_TYPE_BUFFER_SEND_IN, 1);
    assert_eq!(BufferLogger::MSG_TYPE_COM_IN, 2);
    assert_eq!(BufferLogger::MSG_TYPE_PING_IN, 3);
    assert_eq!(BufferLogger::MSG_TYPE_SCHED_IN, 4);
    assert_eq!(BufferLogger::MSG_TYPE_CMD_IN, 5);
}
