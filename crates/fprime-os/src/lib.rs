//! fprime-os: the F Prime OS abstraction layer (OSAL), ported from
//! `/home/user/fprime/Os` per `docs/cpp-analysis/os.md`.
//!
//! The C++ delegate/placement-new machinery is collapsed: each facility is a
//! single concrete std-backed type (the "one implementation chosen at build
//! time, front type is concrete" property is kept — a different backend would
//! be selected with `cfg` type aliases). Every status enum mirrors the C++
//! enum verbatim with explicit discriminants.
//!
//! Modules:
//! - [`queue`] — `Os::Queue` over `Os::Generic::PriorityQueue`: the priority
//!   message queue with FIFO-within-priority ordering and blocking semantics.
//! - [`task`] — `Os::Task` state machine over `std::thread`.
//! - [`mutex`] / [`condition`] — `Os::Mutex` (+ `ScopeLock`) and
//!   `Os::ConditionVariable` wrappers.
//! - [`file`] — `Os::File` with open-mode gating and the historical
//!   un-complemented file CRC.
//! - [`filesystem`] / [`directory`] — `Os::FileSystem` composites and
//!   `Os::Directory`.
//! - [`console`] — `Os::Console`, an [`fprime_fw::FwLogger`] backend.
//! - [`rawtime`] / [`interval_timer`] — `Os::RawTime` (CLOCK_REALTIME
//!   semantics, 8-byte wire format) and `Os::IntervalTimer`.

pub mod condition;
pub mod console;
pub mod directory;
pub mod file;
pub mod filesystem;
pub mod interval_timer;
pub mod mutex;
pub mod queue;
pub mod rawtime;
pub mod task;

pub use condition::ConditionVariable;
pub use console::{CONSOLE, Console, ConsoleStream};
pub use directory::Directory;
pub use file::File;
pub use interval_timer::IntervalTimer;
pub use mutex::{OsMutex, ScopeLock};
pub use queue::{BlockingType, Queue};
pub use rawtime::RawTime;
pub use task::Task;

/// Port of `Os::init()` (Os/Os.cpp). The C++ version force-initializes the
/// Console, FileSystem, Cpu, Memory, and Task singletons; the load-bearing
/// side effect is the Console registering itself as the global `Fw::Logger`.
/// Call once at deployment startup, before any `fw_log!` output matters
/// (C++ parity: log output before this call is silently dropped).
pub fn init() {
    console::init();
}

#[cfg(test)]
pub(crate) mod test_util {
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    /// Create a unique, empty temporary directory for one test.
    pub fn temp_dir(tag: &str) -> PathBuf {
        let path = std::env::temp_dir().join(format!(
            "fprime_os_{}_{}_{}",
            tag,
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::Relaxed)
        ));
        let _ = std::fs::remove_dir_all(&path);
        std::fs::create_dir_all(&path).expect("test temp dir");
        path
    }
}
