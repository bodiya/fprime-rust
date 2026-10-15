//! # AssertFatalAdapter — port of `Svc::AssertFatalAdapter` (passive)
//!
//! C++ sources: `Svc/AssertFatalAdapter/AssertFatalAdapter.fpp`,
//! `AssertFatalEvents.fppi`, `AssertFatalAdapterComponentImpl.{cpp,hpp}`,
//! `docs/sdd.md`.
//!
//! Turns failed `fw_assert!`s into FATAL events. Constructing the component
//! registers an `Fw::AssertHook` ([`fprime_fw::assert::register_assert_hook`],
//! the same global hook in `std` and `no_std` builds) that, for every
//! failed assert:
//!
//! 1. logs the standard assert message (`Assert: "file:line" a1 ... aN`)
//!    through `Fw::Logger` ([`fprime_fw::fw_log!`]);
//! 2. emits `AF_ASSERT_<N>` (FATAL) for an assert with `N` arguments
//!    (`AF_UNEXPECTED_ASSERT` beyond six) — the file name leading-truncated
//!    to its last `min(AssertFatalAdapterEventFileSize,
//!    FW_LOG_STRING_MAX_SIZE)` bytes, the line and arguments as `U32`;
//! 3. returns without the default assert action: whatever handles FATAL
//!    events (`EventManager` → `FatalAnnounce` → `FatalHandler`) takes the
//!    deployment down. Execution continues after the failed `fw_assert!`
//!    until it does, exactly as with upstream's hook.
//!
//! Before the `Log` port is connected, and while more than
//! [`FW_ASSERT_COUNT_MAX`] asserts are being reported at once (an assert
//! raised while reporting an assert, e.g. in the event path), the adapter
//! falls back to the C `assert(false)` — here a `panic!`, the fail-stop of
//! the default assert path (release builds abort on panic). A hook whose
//! component has been dropped logs "Svc::AssertFatalAdapter not
//! registered!" and panics the same way (upstream's null component
//! pointer).
//!
//! Deviations from C++:
//! - The hook holds a `Weak` reference to the component (upstream's raw
//!   back-pointer); the hook it replaced is kept by the component, the
//!   `previousHook` link (upstream never deregisters either).
//! - Upstream's `assert(false)` is a no-op in `NDEBUG` builds (the report
//!   is then dropped); the Rust fallback always panics.
//! - Only the `FW_FILENAME_ASSERT` level exists in Rust (`fw_assert!`
//!   always carries `file!()`), so the `FW_FILEID_ASSERT` hex file id and
//!   the `FW_NO_ASSERT` paths are not ported.
//! - The cascading-assert counter (`m_assertCount`, a plain member) is a
//!   relaxed atomic.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use fprime_comp::{EventGlue, PassiveBase};
use fprime_config::FW_LOG_STRING_MAX_SIZE;
pub use fprime_config::{ASSERT_FATAL_ADAPTER_EVENT_FILE_SIZE, FW_ASSERT_COUNT_MAX};
use fprime_config::{FwAssertArgType, FwEventIdType, FwSizeType};
use fprime_fw::assert::{AssertHook, format_assert_msg, register_assert_hook};
use fprime_fw::sync::{Arc, AtomicUsize, Mutex, Ordering, Weak};
use fprime_fw::{Endianness, LogSeverity, LogStringArg, SerBuf, SerializeStatus, fw_log, fw_try};

/// The file-name bytes an event carries: `min(AssertFatalAdapterEventFileSize,
/// FW_LOG_STRING_MAX_SIZE)` (C++ `outputSize`).
pub const EVENT_FILE_ARG_SIZE: usize =
    if ASSERT_FATAL_ADAPTER_EVENT_FILE_SIZE < FW_LOG_STRING_MAX_SIZE {
        ASSERT_FATAL_ADAPTER_EVENT_FILE_SIZE
    } else {
        FW_LOG_STRING_MAX_SIZE
    };

/// `Svc::AssertFatalAdapter` (`AssertFatalAdapterComponentImpl`).
pub struct AssertFatalAdapter {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `Log` / `LogText` / `Time` ports.
    pub evt: EventGlue,
    /// `m_assertCount`: asserts currently being reported.
    assert_count: AtomicUsize,
    /// The hook this component's adapter replaced (`previousHook`).
    previous_hook: Mutex<Option<Box<dyn AssertHook>>>,
}

impl AssertFatalAdapter {
    /// `AF_ASSERT_0` event id.
    pub const EVENTID_AF_ASSERT_0: FwEventIdType = 0;
    /// `AF_ASSERT_1` event id.
    pub const EVENTID_AF_ASSERT_1: FwEventIdType = 1;
    /// `AF_ASSERT_2` event id.
    pub const EVENTID_AF_ASSERT_2: FwEventIdType = 2;
    /// `AF_ASSERT_3` event id.
    pub const EVENTID_AF_ASSERT_3: FwEventIdType = 3;
    /// `AF_ASSERT_4` event id.
    pub const EVENTID_AF_ASSERT_4: FwEventIdType = 4;
    /// `AF_ASSERT_5` event id.
    pub const EVENTID_AF_ASSERT_5: FwEventIdType = 5;
    /// `AF_ASSERT_6` event id.
    pub const EVENTID_AF_ASSERT_6: FwEventIdType = 6;
    /// `AF_UNEXPECTED_ASSERT` event id.
    pub const EVENTID_AF_UNEXPECTED_ASSERT: FwEventIdType = 7;

    /// Construct and register the assert hook (C++ constructor:
    /// `m_adapter.regAssertReporter(this); m_adapter.registerHook()`).
    /// The most recently constructed adapter receives the asserts.
    pub fn new(name: &str) -> Arc<Self> {
        let comp = Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            assert_count: AtomicUsize::new(0),
            previous_hook: Mutex::new(None),
        });
        let previous = register_assert_hook(Box::new(Adapter {
            comp: Arc::downgrade(&comp),
        }));
        *comp.previous_hook.lock() = previous;
        comp
    }

    /// C++ `reportAssert`: report one assert as a FATAL event. `num_args`
    /// selects the event (`args[..num_args]` are its arguments); above six
    /// it is `AF_UNEXPECTED_ASSERT`. Called by the registered hook; public
    /// as upstream's is.
    pub fn report_assert(
        &self,
        file: &str,
        line_no: FwSizeType,
        num_args: FwSizeType,
        args: [FwAssertArgType; 6],
    ) {
        // Leading truncation: keep the last EVENT_FILE_ARG_SIZE bytes
        // (Fw::StringUtils::string_last_n).
        let bytes = file.as_bytes();
        let start = bytes.len().saturating_sub(EVENT_FILE_ARG_SIZE);
        let mut file_arg = LogStringArg::new();
        file_arg.set_bytes(&bytes[start..]);

        // Fw::defaultReportAssert writes nothing for more than six
        // arguments ("in an assert already, what can we do?").
        let line = line_no as u32;
        let msg = if num_args <= 6 {
            format_assert_msg(file, line, &args[..num_args as usize])
        } else {
            String::new()
        };
        fw_log!("{}\n", msg);

        // Increment the active assert counter.
        let count = self.assert_count.fetch_add(1, Ordering::Relaxed) + 1;

        // Ports not connected yet, or too many cascading asserts: fall back
        // to the C assert (released first, as upstream does for NDEBUG).
        if !self.evt.log_out.is_connected() || count > FW_ASSERT_COUNT_MAX {
            self.assert_count.fetch_sub(1, Ordering::Relaxed);
            panic!("{msg}");
        }

        let a = args.map(|arg| arg as u32);
        let id_base = self.base.get_id_base();
        let emit = |id: FwEventIdType, text: String, values: &[u32]| {
            self.evt.log_event(
                id_base,
                id,
                LogSeverity::Fatal,
                &text,
                |buf| -> SerializeStatus {
                    fw_try!(file_arg.serialize_to_truncated(
                        buf,
                        EVENT_FILE_ARG_SIZE,
                        Endianness::Big
                    ));
                    fw_try!(buf.serialize_u32_be(line));
                    for &value in values {
                        fw_try!(buf.serialize_u32_be(value));
                    }
                    SerializeStatus::Ok
                },
            );
        };
        let head = format!("Assert in file {file_arg}, line {line}");
        match num_args {
            0 => emit(Self::EVENTID_AF_ASSERT_0, head, &[]),
            1 => emit(
                Self::EVENTID_AF_ASSERT_1,
                format!("{head}: {}", a[0]),
                &a[..1],
            ),
            2 => emit(
                Self::EVENTID_AF_ASSERT_2,
                format!("{head}: {} {}", a[0], a[1]),
                &a[..2],
            ),
            3 => emit(
                Self::EVENTID_AF_ASSERT_3,
                format!("{head}: {} {} {}", a[0], a[1], a[2]),
                &a[..3],
            ),
            4 => emit(
                Self::EVENTID_AF_ASSERT_4,
                format!("{head}: {} {} {} {}", a[0], a[1], a[2], a[3]),
                &a[..4],
            ),
            5 => emit(
                Self::EVENTID_AF_ASSERT_5,
                format!("{head}: {} {} {} {} {}", a[0], a[1], a[2], a[3], a[4]),
                &a[..5],
            ),
            6 => emit(
                Self::EVENTID_AF_ASSERT_6,
                format!(
                    "{head}: {} {} {} {} {} {}",
                    a[0], a[1], a[2], a[3], a[4], a[5]
                ),
                &a[..6],
            ),
            _ => emit(
                Self::EVENTID_AF_UNEXPECTED_ASSERT,
                format!(
                    "Unexpected assert in file {file_arg}, line {line}, args {}",
                    num_args as u32
                ),
                &[num_args as u32],
            ),
        }
        // Assert processing complete.
        self.assert_count.fetch_sub(1, Ordering::Relaxed);
    }
}

/// The private `AssertFatalAdapter : Fw::AssertHook` of the C++ component.
struct Adapter {
    /// `m_compPtr`.
    comp: Weak<AssertFatalAdapter>,
}

impl AssertHook for Adapter {
    fn report_assert(&self, file: &str, line: u32, args: &[FwAssertArgType]) {
        match self.comp.upgrade() {
            Some(comp) => {
                let mut padded = [0; 6];
                let n = args.len().min(6);
                padded[..n].copy_from_slice(&args[..n]);
                comp.report_assert(
                    file,
                    FwSizeType::from(line),
                    args.len() as FwSizeType,
                    padded,
                );
            }
            None => {
                // Can't assert, what else can we do? Maybe somebody will
                // see it.
                fw_log!("Svc::AssertFatalAdapter not registered!\n");
                panic!("Svc::AssertFatalAdapter not registered");
            }
        }
    }

    /// Do nothing: the FATAL event's handler deals with it.
    fn do_assert(&self) {}
}
