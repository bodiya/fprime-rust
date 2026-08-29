//! # PassiveTextLogger — port of `Svc::PassiveTextLogger`
//! (`ConsoleTextLoggerImpl`, passive)
//!
//! C++ sources: `Svc/PassiveConsoleTextLogger/ConsoleTextLoggerImpl.{cpp,hpp}`,
//! `Svc/PassiveConsoleTextLogger/PassiveTextLogger.fpp`,
//! `default/config/PassiveTextLoggerCfg.hpp`.
//! Analysis: `docs/cpp-analysis/svc-core.md` (component roster) — the exact
//! print format is ported from the C++ handler.
//!
//! Single sync `TextLogger` input (`Fw.LogText`) that prints each event via
//! the global `Fw::Logger` in the C++ format:
//!
//! ```text
//! EVENT: (<id>) (<timeBase>:<seconds>,<useconds>) <SEVERITY>: <text>\n
//! ```
//!
//! This fork's ConsoleTextLoggerImpl also carries a severity filter
//! (shared `Svc::EventSeverityFilter`, DIAGNOSTIC off by default) and a
//! fixed configure-time event-ID filter — both ported.
//!
//! The `PassiveTextLoggerCfg.hpp` constants live here because
//! `fprime-config` (not writable by this module's wave) has no
//! `passive_text_logger` submodule yet; move them there when the config
//! crate is next revised.

use crate::event_manager::EventSeverityFilter;
use fprime_comp::{LogTextPort, PassiveBase, PortRef};
use fprime_config::{FwEventIdType, FwIndexType, FwTimeBaseStoreType};
use fprime_fw::{LogSeverity, TextLogString, Time, fw_assert, fw_log};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Config constants (default/config/PassiveTextLoggerCfg.hpp).
// ---------------------------------------------------------------------------

/// Size of the configure-time event-ID filter.
pub const PASSIVE_TEXT_LOGGER_ID_FILTER_SIZE: usize = 25;
/// WARNING_HI events pass by default.
pub const PASSIVE_TEXT_LOGGER_FILTER_WARNING_HI_DEFAULT: bool = true;
/// WARNING_LO events pass by default.
pub const PASSIVE_TEXT_LOGGER_FILTER_WARNING_LO_DEFAULT: bool = true;
/// COMMAND events pass by default.
pub const PASSIVE_TEXT_LOGGER_FILTER_COMMAND_DEFAULT: bool = true;
/// ACTIVITY_HI events pass by default.
pub const PASSIVE_TEXT_LOGGER_FILTER_ACTIVITY_HI_DEFAULT: bool = true;
/// ACTIVITY_LO events pass by default.
pub const PASSIVE_TEXT_LOGGER_FILTER_ACTIVITY_LO_DEFAULT: bool = true;
/// DIAGNOSTIC events are dropped by default.
pub const PASSIVE_TEXT_LOGGER_FILTER_DIAGNOSTIC_DEFAULT: bool = false;

/// Configure-time event-ID filter (plain array + count, C++ parity).
struct IdFilter {
    ids: [FwEventIdType; PASSIVE_TEXT_LOGGER_ID_FILTER_SIZE],
    count: usize,
}

/// `Svc::PassiveTextLogger` (`ConsoleTextLoggerImpl`) — console text-event
/// printer.
pub struct PassiveTextLogger {
    /// Passive core (name / id_base / instance).
    pub base: PassiveBase,
    /// Severity filter (shared `Svc::EventSeverityFilter`; FATAL never
    /// filterable). Lock-free like the C++ member.
    severity_filter: EventSeverityFilter,
    id_filter: Mutex<IdFilter>,
}

impl PassiveTextLogger {
    /// Construct with the `PassiveTextLoggerCfg.hpp` severity defaults
    /// (DIAGNOSTIC filtered, everything else passes).
    pub fn new(name: &str) -> Arc<Self> {
        let severity_filter = EventSeverityFilter::new();
        severity_filter.set_filter(
            LogSeverity::WarningHi,
            PASSIVE_TEXT_LOGGER_FILTER_WARNING_HI_DEFAULT,
        );
        severity_filter.set_filter(
            LogSeverity::WarningLo,
            PASSIVE_TEXT_LOGGER_FILTER_WARNING_LO_DEFAULT,
        );
        severity_filter.set_filter(
            LogSeverity::Command,
            PASSIVE_TEXT_LOGGER_FILTER_COMMAND_DEFAULT,
        );
        severity_filter.set_filter(
            LogSeverity::ActivityHi,
            PASSIVE_TEXT_LOGGER_FILTER_ACTIVITY_HI_DEFAULT,
        );
        severity_filter.set_filter(
            LogSeverity::ActivityLo,
            PASSIVE_TEXT_LOGGER_FILTER_ACTIVITY_LO_DEFAULT,
        );
        severity_filter.set_filter(
            LogSeverity::Diagnostic,
            PASSIVE_TEXT_LOGGER_FILTER_DIAGNOSTIC_DEFAULT,
        );
        Arc::new(Self {
            base: PassiveBase::new(name),
            severity_filter,
            id_filter: Mutex::new(IdFilter {
                ids: [0; PASSIVE_TEXT_LOGGER_ID_FILTER_SIZE],
                count: 0,
            }),
        })
    }

    /// C++ `configure(filteredIds, count)`: install the event-ID filter
    /// (replaces any previous set; asserts on more than
    /// [`PASSIVE_TEXT_LOGGER_ID_FILTER_SIZE`] ids).
    pub fn configure(&self, filtered_ids: &[FwEventIdType]) {
        fw_assert!(
            filtered_ids.len() <= PASSIVE_TEXT_LOGGER_ID_FILTER_SIZE,
            filtered_ids.len() as i32,
            PASSIVE_TEXT_LOGGER_ID_FILTER_SIZE as i32
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

    /// `TextLogger` — SYNC `Fw.LogText` input.
    pub fn text_logger_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn LogTextPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `TextLogger_handler`: filter, format, print via the global logger.
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
        let text = String::from_utf8_lossy(text.as_bytes());
        fw_log!(
            "EVENT: ({}) ({}:{},{}) {}: {}\n",
            id,
            time_tag.get_time_base() as FwTimeBaseStoreType,
            time_tag.get_seconds(),
            time_tag.get_useconds(),
            severity_string,
            text
        );
    }
}

impl LogTextPort for PassiveTextLogger {
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

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::TimeBase;

    /// Recording global logger. The `fw::logger` registry is process-wide,
    /// so these tests serialize on [`TEST_LOCK`] and assert on
    /// per-test-unique event ids to stay robust if other tests in the
    /// binary also log.
    struct RecLogger {
        lines: Mutex<Vec<String>>,
    }
    impl fprime_fw::FwLogger for RecLogger {
        fn write_message(&self, message: &str) {
            self.lines.lock().unwrap().push(message.to_string());
        }
    }
    static REC: RecLogger = RecLogger {
        lines: Mutex::new(Vec::new()),
    };
    static TEST_LOCK: Mutex<()> = Mutex::new(());

    fn with_recorder<F: FnOnce()>(f: F) -> Vec<String> {
        let _guard = TEST_LOCK.lock().unwrap();
        fprime_fw::logger::register_logger(&REC);
        REC.lines.lock().unwrap().clear();
        f();
        let lines = REC.lines.lock().unwrap().clone();
        fprime_fw::logger::deregister_logger();
        lines
    }

    fn emit(
        comp: &Arc<PassiveTextLogger>,
        id: FwEventIdType,
        severity: LogSeverity,
        time: Time,
        text: &str,
    ) {
        let mut time = time;
        let mut text_arg = TextLogString::from(text);
        let p = comp.text_logger_in(0);
        p.target
            .invoke(p.port_num, id, &mut time, severity, &mut text_arg);
    }

    #[test]
    fn prints_exact_console_format() {
        let comp = PassiveTextLogger::new("textLogger");
        let time = Time::new(TimeBase::TbWorkstationTime, 0, 123, 456);
        let lines = with_recorder(|| {
            emit(&comp, 9001, LogSeverity::ActivityHi, time, "Hello events");
        });
        assert!(
            lines.contains(&"EVENT: (9001) (2:123,456) ACTIVITY_HI: Hello events\n".to_string()),
            "lines: {lines:?}"
        );
    }

    #[test]
    fn severity_strings_cover_all_levels() {
        let comp = PassiveTextLogger::new("textLogger2");
        // Enable everything so all severities print.
        comp.set_severity_filter(LogSeverity::Diagnostic, true);
        let time = Time::new(TimeBase::TbNone, 0, 0, 0);
        let cases = [
            (LogSeverity::Fatal, "FATAL"),
            (LogSeverity::WarningHi, "WARNING_HI"),
            (LogSeverity::WarningLo, "WARNING_LO"),
            (LogSeverity::Command, "COMMAND"),
            (LogSeverity::ActivityHi, "ACTIVITY_HI"),
            (LogSeverity::ActivityLo, "ACTIVITY_LO"),
            (LogSeverity::Diagnostic, "DIAGNOSTIC"),
        ];
        let lines = with_recorder(|| {
            for (i, (severity, _)) in cases.iter().enumerate() {
                emit(&comp, 9100 + i as u32, *severity, time, "x");
            }
        });
        for (i, (_, name)) in cases.iter().enumerate() {
            let expected = format!("EVENT: ({}) (0:0,0) {}: x\n", 9100 + i as u32, name);
            assert!(
                lines.contains(&expected),
                "missing {expected:?} in {lines:?}"
            );
        }
    }

    #[test]
    fn diagnostic_is_filtered_by_default() {
        let comp = PassiveTextLogger::new("textLogger3");
        let time = Time::new(TimeBase::TbNone, 0, 0, 0);
        let lines = with_recorder(|| {
            emit(&comp, 9200, LogSeverity::Diagnostic, time, "hidden");
            emit(&comp, 9201, LogSeverity::WarningHi, time, "visible");
        });
        assert!(
            !lines.iter().any(|l| l.contains("(9200)")),
            "lines: {lines:?}"
        );
        assert!(
            lines.iter().any(|l| l.contains("(9201)")),
            "lines: {lines:?}"
        );
    }

    #[test]
    fn set_severity_filter_toggles_and_fatal_is_never_filterable() {
        let comp = PassiveTextLogger::new("textLogger4");
        comp.set_severity_filter(LogSeverity::WarningHi, false);
        // setSeverityFilter(FATAL, false) is ignored (C++ parity).
        comp.set_severity_filter(LogSeverity::Fatal, false);
        let time = Time::new(TimeBase::TbNone, 0, 0, 0);
        let lines = with_recorder(|| {
            emit(&comp, 9300, LogSeverity::WarningHi, time, "hidden");
            emit(&comp, 9301, LogSeverity::Fatal, time, "always");
        });
        assert!(!lines.iter().any(|l| l.contains("(9300)")));
        assert!(lines.iter().any(|l| l.contains("(9301)")));
    }

    #[test]
    fn configured_id_filter_drops_matching_ids() {
        let comp = PassiveTextLogger::new("textLogger5");
        comp.configure(&[9400, 9402]);
        let time = Time::new(TimeBase::TbNone, 0, 0, 0);
        let lines = with_recorder(|| {
            emit(&comp, 9400, LogSeverity::WarningHi, time, "hidden");
            emit(&comp, 9401, LogSeverity::WarningHi, time, "visible");
            emit(&comp, 9402, LogSeverity::WarningHi, time, "hidden");
        });
        assert!(!lines.iter().any(|l| l.contains("(9400)")));
        assert!(lines.iter().any(|l| l.contains("(9401)")));
        assert!(!lines.iter().any(|l| l.contains("(9402)")));
    }

    #[test]
    #[should_panic]
    fn configure_with_too_many_ids_asserts() {
        let comp = PassiveTextLogger::new("textLogger6");
        let ids = [0u32; PASSIVE_TEXT_LOGGER_ID_FILTER_SIZE + 1];
        comp.configure(&ids);
    }
}
