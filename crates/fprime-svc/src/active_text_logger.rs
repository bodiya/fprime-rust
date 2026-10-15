//! # ActiveTextLogger — port of `Svc::ActiveTextLogger` (active)
//!
//! C++ sources: `Svc/ActiveTextLogger/ActiveTextLogger.{cpp,hpp,fpp}`,
//! `Svc/ActiveTextLogger/LogFile.{cpp,hpp}` ([`LogFile`]),
//! `default/config/ActiveTextLoggerCfg.hpp`
//! ([`fprime_config::active_text_logger`]).
//! Analysis: `docs/cpp-analysis/svc-misc.md` (ActiveTextLogger + LogFile).
//!
//! Like [`PassiveTextLogger`](crate::passive_text_logger::PassiveTextLogger)
//! it prints text events to the console, but from its own thread so lines
//! keep a consistent order, and it can also append them to a size-capped
//! file ([`ActiveTextLogger::set_log_file`]).
//!
//! - The sync `TextLogger` input runs on the CALLER's thread: the severity
//!   filter (shared `Svc::EventSeverityFilter`, DIAGNOSTIC off by default,
//!   FATAL never filtered), the configure-time event-ID filter, and the
//!   formatting into a 256-byte `Fw::InternalInterfaceString`:
//!
//!   ```text
//!   EVENT: (<id>) (<timeBase>:<seconds>,<useconds>) <SEVERITY>: <text>\n
//!   ```
//!
//!   Overlong lines are truncated at 256 bytes (losing the newline), as
//!   the C++ `format` does.
//! - The formatted line goes through the internal `TextQueue` port, whose
//!   queue-full policy is `drop`: under pressure lines are silently lost and
//!   only counted in the base `msgs_dropped` counter.
//! - On the component thread each line is written to the console
//!   (`Fw::Logger`) and then to the log file, if one is open; the file
//!   status is ignored (C++ parity).
//!
//! The component has no commands, events or telemetry.

mod log_file;

pub use log_file::LogFile;

pub use fprime_config::active_text_logger::{
    FILTER_ACTIVITY_HI_DEFAULT, FILTER_ACTIVITY_LO_DEFAULT, FILTER_COMMAND_DEFAULT,
    FILTER_DIAGNOSTIC_DEFAULT, FILTER_WARNING_HI_DEFAULT, FILTER_WARNING_LO_DEFAULT,
    ID_FILTER_SIZE,
};

use crate::event_manager::EventSeverityFilter;
use fprime_comp::msg;
use fprime_comp::{
    ActiveBase, ActiveComponent, ComponentDispatch, LogTextPort, MsgDispatchStatus, PortRef,
    QueueFullPolicy,
};
use fprime_config::{
    FW_INTERNAL_INTERFACE_STRING_MAX_SIZE, FwEnumStoreType, FwEventIdType, FwIndexType,
    FwQueuePriorityType, FwSizeType, FwTimeBaseStoreType,
};
use fprime_fw::{
    Endianness, InternalInterfaceString, LinearBuffer, LogSeverity, SerBuf, SerBufAny,
    TextLogString, Time, fw_assert, logger,
};
use std::sync::{Arc, Mutex};

/// Queue message type of the internal `TextQueue` port (0 is EXIT).
pub const MSG_TYPE_TEXT_QUEUE: FwEnumStoreType = 1;

/// Queue message size: the internal `TextQueue` message, 6 (envelope) +
/// 2 + 256 (length-prefixed `string size 256`) = 264.
///
/// The envelope carries a `port_num` of 0 after the message type because
/// the `fprime-comp` dispatch contract reads one for every non-EXIT message
/// (the same choice as `EventManager`'s internal `loqQueue`). Internal
/// only; nothing on the ground sees it.
pub const QUEUE_MSG_SIZE: usize =
    msg::ENVELOPE_HEADER_SIZE + InternalInterfaceString::SERIALIZED_SIZE;

/// Queue priority of the internal port (no FPP `priority` qualifier).
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

type MsgBuffer = LinearBuffer<QUEUE_MSG_SIZE>;

/// Configure-time event-ID filter (plain array + count, C++ parity).
struct IdFilter {
    ids: [FwEventIdType; ID_FILTER_SIZE],
    count: usize,
}

/// `Svc::ActiveTextLogger` — ordered console (and optional file) text
/// event printer.
pub struct ActiveTextLogger {
    /// Active core: PassiveBase + queue + task.
    pub active: ActiveBase,
    /// Severity filter (shared `Svc::EventSeverityFilter`; FATAL never
    /// filterable). Lock-free like the C++ member.
    severity_filter: EventSeverityFilter,
    /// C++ `m_filteredIDs` / `m_numFilteredIDs`, read on every caller's
    /// thread (C++ leaves it unguarded: it is set at configure time).
    id_filter: Mutex<IdFilter>,
    /// C++ `m_log_file`. DEVIATION: guarded by a mutex — C++ lets
    /// `set_log_file` (any thread) race the component thread's writes.
    log_file: Mutex<LogFile>,
}

impl ActiveTextLogger {
    /// The C++ default for `set_log_file`'s `maxBackups` argument.
    pub const DEFAULT_MAX_BACKUPS: u32 = 10;

    /// C++ `ActiveTextLogger(compName)`: severity defaults from
    /// `ActiveTextLoggerCfg.hpp` (DIAGNOSTIC filtered), no ID filter, no
    /// log file.
    pub fn new(name: &str) -> Arc<Self> {
        let severity_filter = EventSeverityFilter::new();
        severity_filter.set_filter(LogSeverity::WarningHi, FILTER_WARNING_HI_DEFAULT);
        severity_filter.set_filter(LogSeverity::WarningLo, FILTER_WARNING_LO_DEFAULT);
        severity_filter.set_filter(LogSeverity::Command, FILTER_COMMAND_DEFAULT);
        severity_filter.set_filter(LogSeverity::ActivityHi, FILTER_ACTIVITY_HI_DEFAULT);
        severity_filter.set_filter(LogSeverity::ActivityLo, FILTER_ACTIVITY_LO_DEFAULT);
        severity_filter.set_filter(LogSeverity::Diagnostic, FILTER_DIAGNOSTIC_DEFAULT);
        Arc::new(Self {
            active: ActiveBase::new(name),
            severity_filter,
            id_filter: Mutex::new(IdFilter {
                ids: [0; ID_FILTER_SIZE],
                count: 0,
            }),
            log_file: Mutex::new(LogFile::new()),
        })
    }

    /// Create the component queue (C++ `init(queueDepth, instance)`).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    /// C++ `configure(filteredIds, count)`: install the event-ID filter
    /// (replaces any previous set; asserts on more than
    /// [`ID_FILTER_SIZE`] ids).
    pub fn configure(&self, filtered_ids: &[FwEventIdType]) {
        fw_assert!(
            filtered_ids.len() <= ID_FILTER_SIZE,
            filtered_ids.len() as i32,
            ID_FILTER_SIZE as i32
        );
        let mut filter = self.id_filter.lock().unwrap();
        filter.count = filtered_ids.len();
        filter.ids[..filtered_ids.len()].copy_from_slice(filtered_ids);
    }

    /// C++ `setSeverityFilter(severity, enabled)`: `true` = events pass,
    /// `false` = dropped. FATAL is ignored (never filterable).
    pub fn set_severity_filter(&self, severity: LogSeverity, enabled: bool) {
        self.severity_filter.set_filter(severity, enabled);
    }

    /// C++ `set_log_file(fileName, maxSize, maxBackups)`: also write every
    /// line to `file_name`, until it would grow past `max_size` bytes.
    ///
    /// If `file_name` exists, `file_name0` … `file_name<max_backups - 1>`
    /// are tried in turn; if all exist, `file_name` itself is truncated and
    /// reused. Any open log file is closed first. Returns `true` when a file
    /// was opened; `false` when a name does not fit a `FileNameString` or
    /// the open fails, leaving no file open. See [`LogFile::set_log_file`].
    ///
    /// The C++ `maxBackups` defaults to 10 ([`Self::DEFAULT_MAX_BACKUPS`]);
    /// Rust has no default arguments, so it is passed explicitly.
    pub fn set_log_file(&self, file_name: &str, max_size: u32, max_backups: u32) -> bool {
        self.log_file.lock().unwrap().set_log_file(
            file_name,
            FwSizeType::from(max_size),
            FwSizeType::from(max_backups),
        )
    }

    /// `TextLogger` — SYNC `Fw.LogText` input.
    pub fn text_logger_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn LogTextPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `TextLogger_handler` — caller thread: filter, format, enqueue.
    fn text_logger_handler(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        time_tag: &Time,
        severity: LogSeverity,
        text: &TextLogString,
    ) {
        if self.severity_filter.is_filtered(severity) {
            return;
        }
        {
            let filter = self.id_filter.lock().unwrap();
            if filter.ids[..filter.count].contains(&id) {
                return;
            }
        }

        // C++ prints "SEVERITY ERROR" for out-of-range values; the Rust
        // enum cannot hold one, so that branch is unrepresentable.
        let severity_string = match severity {
            LogSeverity::Fatal => "FATAL",
            LogSeverity::WarningHi => "WARNING_HI",
            LogSeverity::WarningLo => "WARNING_LO",
            LogSeverity::Command => "COMMAND",
            LogSeverity::ActivityHi => "ACTIVITY_HI",
            LogSeverity::ActivityLo => "ACTIVITY_LO",
            LogSeverity::Diagnostic => "DIAGNOSTIC",
        };
        // Truncation is accepted (C++ ignores the format status). The text
        // is appended as bytes, so non-UTF-8 content reaches the file
        // unchanged; sequential truncating appends equal one truncating
        // format.
        let mut int_text = InternalInterfaceString::new();
        int_text.format(format_args!(
            "EVENT: ({}) ({}:{},{}) {}: ",
            id,
            time_tag.get_time_base() as FwTimeBaseStoreType,
            time_tag.get_seconds(),
            time_tag.get_useconds(),
            severity_string,
        ));
        int_text.append_bytes(text.as_bytes());
        int_text.append("\n");

        // Everything else happens on the component thread, which keeps the
        // printed lines in a consistent order.
        self.text_queue_internal_interface_invoke(&int_text);
    }

    /// `TextQueue_internalInterfaceInvoke` — `drop` policy: a full queue
    /// counts the line in `msgs_dropped` and discards it.
    fn text_queue_internal_interface_invoke(&self, text: &InternalInterfaceString) {
        let mut buf = MsgBuffer::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_TEXT_QUEUE, 0);
        fw_assert!(status.is_ok(), status as i32);
        let status = text.serialize_to_truncated(
            &mut buf,
            FW_INTERNAL_INTERFACE_STRING_MAX_SIZE,
            Endianness::Big,
        );
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .active
            .queued
            .send_message(&buf, QUEUE_PRIORITY, QueueFullPolicy::Drop);
    }

    /// `TextQueue_internalInterfaceHandler` — component thread: console,
    /// then file.
    fn text_queue_internal_interface_handler(&self, text: &InternalInterfaceString) {
        // Print to console. Lossy rendering only for non-UTF-8 bytes (the
        // console is a `&str` sink); the file below gets the raw bytes.
        match text.as_str() {
            Some(line) => logger::log_message(line),
            None => logger::log_message(&String::from_utf8_lossy(text.as_bytes())),
        }
        // Print to file if there is one; the status is ignored (C++ parity).
        let _ = self.log_file.lock().unwrap().write_to_log(text.as_bytes());
    }
}

impl LogTextPort for ActiveTextLogger {
    fn invoke(
        &self,
        port_num: FwIndexType,
        id: FwEventIdType,
        time_tag: &mut Time,
        severity: LogSeverity,
        text: &mut TextLogString,
    ) {
        self.text_logger_handler(port_num, id, time_tag, severity, text);
    }
}

impl ComponentDispatch for ActiveTextLogger {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            MSG_TYPE_TEXT_QUEUE => {
                let mut text = InternalInterfaceString::new();
                if !buf.deserialize(&mut text, Endianness::Big).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.text_queue_internal_interface_handler(&text);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for ActiveTextLogger {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

#[cfg(test)]
mod tests {
    //! Ports of `Svc/ActiveTextLogger/test/ut` (Nominal.Logging,
    //! Nominal.WorkstationTime, OffNominal.FileHandling,
    //! Filtering.SeverityFilter, Filtering.DiagnosticDisabled), using a
    //! per-test directory instead of the working directory, plus the
    //! branches upstream does not reach. The console path goes through the
    //! process-global `Fw::Logger`, which other tests in this binary
    //! capture, so — as upstream — these tests check the file.

    use super::*;
    use fprime_config::FILE_NAME_STRING_SIZE;
    use fprime_fw::TimeBase;
    use fprime_os::queue::BlockingType;
    use fprime_os::task::TASK_DEFAULT;
    use std::path::{Path, PathBuf};
    use std::sync::atomic::{AtomicU32, Ordering};

    const QUEUE_DEPTH: FwSizeType = 10;

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    /// A per-test scratch directory, removed on drop.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(tag: &str) -> Self {
            let mut path = std::env::temp_dir();
            path.push(format!(
                "fprime_rust_atl_{tag}_{}_{}",
                std::process::id(),
                COUNTER.fetch_add(1, Ordering::SeqCst)
            ));
            let _ = std::fs::remove_dir_all(&path);
            std::fs::create_dir_all(&path).unwrap();
            Self(path)
        }

        fn file(&self, name: &str) -> String {
            self.0.join(name).to_str().unwrap().to_string()
        }

        fn dir(&self) -> String {
            self.0.to_str().unwrap().to_string()
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn build(name: &str) -> Arc<ActiveTextLogger> {
        let comp = ActiveTextLogger::new(name);
        comp.init(QUEUE_DEPTH);
        comp
    }

    fn invoke(
        comp: &Arc<ActiveTextLogger>,
        id: FwEventIdType,
        time: Time,
        severity: LogSeverity,
        text: &str,
    ) {
        let mut time = time;
        let mut text = TextLogString::from(text);
        let port = comp.text_logger_in(0);
        port.target
            .invoke(port.port_num, id, &mut time, severity, &mut text);
    }

    /// C++ `component.doDispatch()`.
    fn do_dispatch(comp: &Arc<ActiveTextLogger>) -> MsgDispatchStatus {
        comp.active
            .queued
            .do_dispatch(comp.as_ref(), BlockingType::NonBlocking)
    }

    fn lines(path: &str) -> Vec<String> {
        std::fs::read_to_string(path)
            .unwrap()
            .lines()
            .map(str::to_string)
            .collect()
    }

    fn line(id: FwEventIdType, time: Time, severity: &str, text: &str) -> String {
        format!(
            "EVENT: ({id}) ({}:{},{}) {severity}: {text}",
            time.get_time_base() as FwTimeBaseStoreType,
            time.get_seconds(),
            time.get_useconds()
        )
    }

    fn log_file(comp: &ActiveTextLogger) -> std::sync::MutexGuard<'_, LogFile> {
        comp.log_file.lock().unwrap()
    }

    #[test]
    fn nominal_logging_writes_formatted_lines_to_the_file() {
        let dir = TempDir::new("nominal");
        let comp = build("atlNominal");
        assert!(!log_file(&comp).is_open());

        // Console only.
        invoke(
            &comp,
            1,
            Time::new(TimeBase::TbNone, 0, 3, 6),
            LogSeverity::ActivityHi,
            "This component is the greatest!",
        );
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        invoke(
            &comp,
            2,
            Time::new(TimeBase::TbProcTime, 0, 4, 7),
            LogSeverity::ActivityLo,
            "This component is the probably the greatest!",
        );
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        invoke(
            &comp,
            3,
            Time::new(TimeBase::TbWorkstationTime, 0, 5, 876),
            LogSeverity::WarningLo,
            "This component is maybe the greatest!",
        );
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);

        // Console and file.
        let name = dir.file("test_file");
        assert!(comp.set_log_file(&name, 512, ActiveTextLogger::DEFAULT_MAX_BACKUPS));
        {
            let log = log_file(&comp);
            assert!(log.is_open());
            assert_eq!(log.file_name().as_str(), Some(name.as_str()));
            assert_eq!(log.current_file_size(), 0);
            assert_eq!(log.max_file_size(), 512);
        }

        let time4 = Time::new(TimeBase::TbNone, 0, 5, 8);
        let text4 = "This component may be the greatest!";
        invoke(&comp, 4, time4, LogSeverity::WarningLo, text4);
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        let past_size = text4.len() as FwSizeType + 32;
        {
            let log = log_file(&comp);
            assert!(log.is_open());
            assert_eq!(log.current_file_size(), past_size);
            assert_eq!(log.max_file_size(), 512);
        }
        let first = line(4, time4, "WARNING_LO", text4);
        assert_eq!(lines(&name), vec![first.clone()]);

        let time5 = Time::new(TimeBase::TbProcTime, 0, 6, 9);
        let text5 = "This component is probably not the greatest!";
        invoke(&comp, 5, time5, LogSeverity::WarningHi, text5);
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        {
            let log = log_file(&comp);
            assert!(log.is_open());
            assert_eq!(
                log.current_file_size(),
                past_size + text5.len() as FwSizeType + 32
            );
            // The predicted size matches the file.
            let mut file_size = 0;
            assert_eq!(
                fprime_os::filesystem::get_file_size(&name, &mut file_size),
                fprime_os::filesystem::Status::OpOk
            );
            assert_eq!(file_size, log.current_file_size());
        }
        assert_eq!(
            lines(&name),
            vec![first, line(5, time5, "WARNING_HI", text5)]
        );
    }

    #[test]
    fn workstation_timestamp_is_printed_as_base_seconds_and_useconds() {
        let dir = TempDir::new("workstation");
        let comp = build("atlWorkstation");
        let name = dir.file("test_file");
        assert!(comp.set_log_file(&name, 512, ActiveTextLogger::DEFAULT_MAX_BACKUPS));
        assert!(log_file(&comp).is_open());

        let time = Time::new(TimeBase::TbWorkstationTime, 0, 3, 6);
        let text = "This line has a valid timestamp.";
        invoke(&comp, 1, time, LogSeverity::ActivityHi, text);
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        assert_eq!(lines(&name), vec![line(1, time, "ACTIVITY_HI", text)]);
        assert_eq!(
            std::fs::read_to_string(&name).unwrap(),
            "EVENT: (1) (2:3,6) ACTIVITY_HI: This line has a valid timestamp.\n"
        );
    }

    #[test]
    fn off_nominal_file_handling() {
        let dir = TempDir::new("offnominal");
        let comp = build("atlOffNominal");
        let mut tmp: FwSizeType = 0;
        let fs_ok = fprime_os::filesystem::Status::OpOk;

        // Writing more than the max file size.
        let max_name = dir.file("test_file_max");
        assert!(comp.set_log_file(&max_name, 45, ActiveTextLogger::DEFAULT_MAX_BACKUPS));
        {
            let log = log_file(&comp);
            assert!(log.is_open());
            assert_eq!(log.file_name().as_str(), Some(max_name.as_str()));
            assert_eq!(log.current_file_size(), 0);
            assert_eq!(log.max_file_size(), 45);
        }
        assert_eq!(
            fprime_os::filesystem::get_file_size(&max_name, &mut tmp),
            fs_ok
        );

        let time = Time::new(TimeBase::TbNone, 0, 3, 6);
        invoke(&comp, 1, time, LogSeverity::ActivityHi, "abcd");
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        let past_size = 4 + 33;
        {
            let log = log_file(&comp);
            assert!(log.is_open());
            assert_eq!(log.current_file_size(), past_size);
            assert_eq!(log.max_file_size(), 45);
        }
        let old_line = line(1, time, "ACTIVITY_HI", "abcd");
        assert_eq!(lines(&max_name), vec![old_line.clone()]);

        // Going over the max closes the file; the size and the contents do
        // not change.
        invoke(
            &comp,
            1,
            time,
            LogSeverity::ActivityHi,
            "This will go over max size of the file!",
        );
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        {
            let log = log_file(&comp);
            assert!(!log.is_open());
            assert_eq!(log.current_file_size(), past_size);
            assert_eq!(log.max_file_size(), 45);
        }
        assert_eq!(lines(&max_name), vec![old_line]);

        // A file name that already exists gets the 0 suffix.
        assert!(comp.set_log_file(&max_name, 50, ActiveTextLogger::DEFAULT_MAX_BACKUPS));
        let max_name0 = dir.file("test_file_max0");
        {
            let log = log_file(&comp);
            assert!(log.is_open());
            assert_eq!(log.file_name().as_str(), Some(max_name0.as_str()));
            assert_eq!(log.current_file_size(), 0);
            assert_eq!(log.max_file_size(), 50);
        }
        assert_eq!(
            fprime_os::filesystem::get_file_size(&max_name0, &mut tmp),
            fs_ok
        );

        // A file name one byte longer than FileNameString fails.
        let prefix = format!("{}/", dir.dir());
        let pad = FILE_NAME_STRING_SIZE - prefix.len();
        let long_name = format!("{prefix}{}", "a".repeat(pad + 1));
        assert!(!comp.set_log_file(&long_name, 50, ActiveTextLogger::DEFAULT_MAX_BACKUPS));
        assert!(!log_file(&comp).is_open());
        assert!(!fprime_os::filesystem::exists(&long_name));

        // A file name of exactly the maximum size works once; the second
        // time its suffixed name overflows and the call fails.
        let long_name_dup = format!("{prefix}{}", "a".repeat(pad));
        assert_eq!(long_name_dup.len(), FILE_NAME_STRING_SIZE);
        assert!(comp.set_log_file(&long_name_dup, 50, ActiveTextLogger::DEFAULT_MAX_BACKUPS));
        assert!(log_file(&comp).is_open());
        assert_eq!(
            fprime_os::filesystem::get_file_size(&long_name_dup, &mut tmp),
            fs_ok
        );
        assert!(!comp.set_log_file(&long_name_dup, 50, ActiveTextLogger::DEFAULT_MAX_BACKUPS));
        assert!(!log_file(&comp).is_open());
        assert!(!Path::new(&format!("{long_name_dup}0")).exists());

        // The original name and the next 10 suffixes: the 11th reuses (and
        // truncates) the original.
        let base_name = dir.file("test_mult_dup");
        assert!(comp.set_log_file(&base_name, 50, ActiveTextLogger::DEFAULT_MAX_BACKUPS));
        {
            let log = log_file(&comp);
            assert!(log.is_open());
            assert_eq!(log.file_name().as_str(), Some(base_name.as_str()));
        }
        assert_eq!(
            fprime_os::filesystem::get_file_size(&base_name, &mut tmp),
            fs_ok
        );
        for i in 0..10 {
            assert!(comp.set_log_file(&base_name, 50, ActiveTextLogger::DEFAULT_MAX_BACKUPS));
            let with_suffix = format!("{base_name}{i}");
            let log = log_file(&comp);
            assert!(log.is_open());
            assert_eq!(log.file_name().as_str(), Some(with_suffix.as_str()));
            assert_eq!(
                fprime_os::filesystem::get_file_size(&with_suffix, &mut tmp),
                fs_ok
            );
        }
        std::fs::write(&base_name, b"stale").unwrap();
        assert!(comp.set_log_file(&base_name, 50, ActiveTextLogger::DEFAULT_MAX_BACKUPS));
        {
            let log = log_file(&comp);
            assert!(log.is_open());
            assert_eq!(log.file_name().as_str(), Some(base_name.as_str()));
        }
        assert_eq!(
            fprime_os::filesystem::get_file_size(&base_name, &mut tmp),
            fs_ok
        );
        assert_eq!(tmp, 0, "the reused original is truncated");
        assert!(!Path::new(&format!("{base_name}10")).exists());
    }

    #[test]
    fn severity_filter_drops_disabled_severities_before_the_queue() {
        let dir = TempDir::new("severity");
        let comp = build("atlSeverity");
        let name = dir.file("test_severity_filter");
        assert!(comp.set_log_file(&name, 4096, ActiveTextLogger::DEFAULT_MAX_BACKUPS));

        comp.set_severity_filter(LogSeverity::WarningLo, false);
        let time = Time::new(TimeBase::TbNone, 0, 1, 0);
        // Filtered events never enqueue: one dispatch drains the queue.
        invoke(
            &comp,
            1,
            time,
            LogSeverity::WarningLo,
            "This should be filtered",
        );
        invoke(
            &comp,
            2,
            time,
            LogSeverity::WarningHi,
            "This should pass through",
        );
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Empty);
        assert_eq!(
            lines(&name),
            vec![line(2, time, "WARNING_HI", "This should pass through")]
        );

        // Re-enabled, it passes again.
        comp.set_severity_filter(LogSeverity::WarningLo, true);
        invoke(&comp, 3, time, LogSeverity::WarningLo, "now visible");
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        assert_eq!(lines(&name).len(), 2);
    }

    #[test]
    fn diagnostic_is_filtered_by_default_and_fatal_never_is() {
        let dir = TempDir::new("diag");
        let comp = build("atlDiag");
        let name = dir.file("test_diag_filter");
        assert!(comp.set_log_file(&name, 4096, ActiveTextLogger::DEFAULT_MAX_BACKUPS));

        // The config default already drops DIAGNOSTIC; upstream's test
        // disables it explicitly, which changes nothing.
        comp.set_severity_filter(LogSeverity::Diagnostic, false);
        // FATAL cannot be filtered.
        comp.set_severity_filter(LogSeverity::Fatal, false);
        let time = Time::new(TimeBase::TbNone, 0, 1, 0);
        invoke(&comp, 1, time, LogSeverity::Diagnostic, "Diagnostic event");
        invoke(&comp, 2, time, LogSeverity::Fatal, "Fatal event");
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Empty);
        assert_eq!(lines(&name), vec![line(2, time, "FATAL", "Fatal event")]);

        let fresh = build("atlDiagDefault");
        let fresh_name = dir.file("default");
        assert!(fresh.set_log_file(&fresh_name, 4096, 0));
        invoke(&fresh, 3, time, LogSeverity::Diagnostic, "hidden");
        assert_eq!(do_dispatch(&fresh), MsgDispatchStatus::Empty);
        assert_eq!(lines(&fresh_name), Vec::<String>::new());
    }

    #[test]
    fn severity_strings_cover_all_levels() {
        let dir = TempDir::new("strings");
        let comp = build("atlStrings");
        comp.set_severity_filter(LogSeverity::Diagnostic, true);
        let name = dir.file("log");
        assert!(comp.set_log_file(&name, 4096, 0));
        let time = Time::new(TimeBase::TbScTime, 0, 7, 8);
        let cases = [
            (LogSeverity::Fatal, "FATAL"),
            (LogSeverity::WarningHi, "WARNING_HI"),
            (LogSeverity::WarningLo, "WARNING_LO"),
            (LogSeverity::Command, "COMMAND"),
            (LogSeverity::ActivityHi, "ACTIVITY_HI"),
            (LogSeverity::ActivityLo, "ACTIVITY_LO"),
            (LogSeverity::Diagnostic, "DIAGNOSTIC"),
        ];
        for (i, (severity, _)) in cases.iter().enumerate() {
            invoke(&comp, i as FwEventIdType, time, *severity, "x");
            assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        }
        let expected: Vec<String> = cases
            .iter()
            .enumerate()
            .map(|(i, (_, s))| line(i as FwEventIdType, time, s, "x"))
            .collect();
        assert_eq!(lines(&name), expected);
    }

    #[test]
    fn configured_id_filter_drops_matching_ids() {
        let dir = TempDir::new("ids");
        let comp = build("atlIds");
        comp.configure(&[10, 12]);
        let name = dir.file("log");
        assert!(comp.set_log_file(&name, 4096, 0));
        let time = Time::new(TimeBase::TbNone, 0, 0, 0);
        for id in 10..13 {
            invoke(&comp, id, time, LogSeverity::WarningHi, "x");
        }
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Empty);
        assert_eq!(lines(&name), vec![line(11, time, "WARNING_HI", "x")]);

        // configure replaces the set.
        comp.configure(&[]);
        invoke(&comp, 10, time, LogSeverity::WarningHi, "x");
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
    }

    #[test]
    #[should_panic]
    fn configure_with_too_many_ids_asserts() {
        let comp = ActiveTextLogger::new("atlTooMany");
        comp.configure(&[0; ID_FILTER_SIZE + 1]);
    }

    #[test]
    fn overlong_lines_are_truncated_to_the_internal_string_size() {
        let dir = TempDir::new("truncate");
        let comp = build("atlTruncate");
        let name = dir.file("log");
        assert!(comp.set_log_file(&name, 4096, 0));
        let text = "z".repeat(TextLogString::max_length());
        let time = Time::new(TimeBase::TbNone, 0, 0, 0);
        invoke(&comp, 1, time, LogSeverity::WarningHi, &text);
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        let written = std::fs::read(&name).unwrap();
        assert_eq!(written.len(), FW_INTERNAL_INTERFACE_STRING_MAX_SIZE);
        assert!(written.starts_with(b"EVENT: (1) (0:0,0) WARNING_HI: zzz"));
        // The newline is lost to the truncation.
        assert_eq!(written.last(), Some(&b'z'));
        assert_eq!(
            log_file(&comp).current_file_size(),
            FW_INTERNAL_INTERFACE_STRING_MAX_SIZE as FwSizeType
        );
    }

    #[test]
    fn a_full_queue_drops_lines_and_counts_them() {
        let dir = TempDir::new("drop");
        let comp = ActiveTextLogger::new("atlDrop");
        comp.init(1);
        let name = dir.file("log");
        assert!(comp.set_log_file(&name, 4096, 0));
        let time = Time::new(TimeBase::TbNone, 0, 0, 0);
        invoke(&comp, 1, time, LogSeverity::WarningHi, "kept");
        invoke(&comp, 2, time, LogSeverity::WarningHi, "dropped");
        assert_eq!(comp.active.queued.get_num_msgs_dropped(), 1);
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Empty);
        assert_eq!(lines(&name), vec![line(1, time, "WARNING_HI", "kept")]);
    }

    #[test]
    fn set_log_file_moves_logging_to_the_new_file() {
        let dir = TempDir::new("switch");
        let comp = build("atlSwitch");
        let first = dir.file("first");
        let second = dir.file("second");
        let time = Time::new(TimeBase::TbNone, 0, 0, 0);
        assert!(comp.set_log_file(&first, 4096, 0));
        invoke(&comp, 1, time, LogSeverity::WarningHi, "one");
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        assert!(comp.set_log_file(&second, 4096, 0));
        invoke(&comp, 2, time, LogSeverity::WarningHi, "two");
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        assert_eq!(lines(&first), vec![line(1, time, "WARNING_HI", "one")]);
        assert_eq!(lines(&second), vec![line(2, time, "WARNING_HI", "two")]);

        // A failed set_log_file leaves no file open: nothing more is
        // written anywhere.
        assert!(!comp.set_log_file(&dir.file("missing/log"), 4096, 0));
        invoke(&comp, 3, time, LogSeverity::WarningHi, "three");
        assert_eq!(do_dispatch(&comp), MsgDispatchStatus::Ok);
        assert_eq!(lines(&second).len(), 1);
    }

    #[test]
    fn the_component_thread_writes_lines_in_order() {
        let dir = TempDir::new("thread");
        let comp = build("atlThread");
        let name = dir.file("log");
        assert!(comp.set_log_file(&name, 4096, 0));
        let time = Time::new(TimeBase::TbNone, 0, 0, 0);
        for id in 0..5 {
            invoke(&comp, id, time, LogSeverity::ActivityHi, "ordered");
        }
        comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);
        comp.active.exit();
        assert_eq!(comp.active.join(), fprime_os::task::Status::OpOk);
        let expected: Vec<String> = (0..5)
            .map(|id| line(id, time, "ACTIVITY_HI", "ordered"))
            .collect();
        assert_eq!(lines(&name), expected);
    }

    #[test]
    fn queue_message_size_is_envelope_plus_internal_string() {
        assert_eq!(QUEUE_MSG_SIZE, 6 + 2 + 256);
    }
}
