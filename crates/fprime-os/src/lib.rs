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
//! - [`queue`] — `Os::Queue`: the priority message queue with
//!   FIFO-within-priority ordering and blocking semantics, over one of the
//!   [`generic`] delegates — `Os::Generic::PriorityQueue` (default) or the
//!   ISR-safe `Os::Generic::LocklessPriorityQueue` (`lockless-queue`
//!   feature).
//! - [`task`] — `Os::Task` state machine over `std::thread`.
//! - [`mutex`] / [`condition`] — `Os::Mutex` (+ `ScopeLock`) and
//!   `Os::ConditionVariable` wrappers.
//! - [`file`](mod@file) — `Os::File` with open-mode gating and the historical
//!   un-complemented file CRC.
//! - [`filesystem`] / [`directory`] — `Os::FileSystem` composites and
//!   `Os::Directory`.
//! - [`file_path_utils`] / [`sandboxed_file`] — `Os::FilePathUtils` lexical
//!   path resolution and `Os::SandboxedFile`, a file restricted to a
//!   directory subtree.
//! - [`console`] — `Os::Console`, an [`fprime_fw::FwLogger`] backend.
//! - [`rawtime`] / [`interval_timer`] — `Os::RawTime` (CLOCK_REALTIME
//!   semantics, 8-byte wire format) and `Os::IntervalTimer`.
//! - [`cpu`] / [`memory`] — `Os::Cpu` (per-CPU cumulative ticks) and
//!   `Os::Memory` (system RAM usage) singletons over the shared
//!   [`generic`] `Os::Generic::Status` / `UsedTotal` types. These are the
//!   facilities with more than one shipped backend: [`linux`] (procfs /
//!   sysfs, the default on Linux) and [`stub`] (the upstream stubs, the
//!   default elsewhere), selected by `cfg` type alias.
//!
//! `no_std` (ROADMAP item 13): with the default `std` feature off (and
//! `spin` on) the crate is the `no_std` + `alloc` subset [`queue`] (both
//! delegates), [`task`], [`mutex`], [`condition`], [`rawtime`],
//! [`interval_timer`], [`generic`] and the [`stub`] [`cpu`] / [`memory`]
//! backends; task start/join/delay, `RawTime::now` and the blocking
//! queue / mutex / condition-variable waits go through the `platform`
//! seam, which the deployment implements (`platform::Platform`, registered
//! with `register_platform`). Files, directories, the console and the
//! Linux cpu/memory backends (which read procfs/sysfs through `Os::File`;
//! a target with procfs is a hosted Linux target, which has `std`) are
//! `std`-only.
//!
//! Not here: `Os::ValidateFile` is `fprime_utils::validate_file`. It is
//! built on `Utils::Hash`, and the crate graph runs `fprime-utils ->
//! fprime-os`, not the reverse (rationale in that module's docs).

// Unlike fprime-fw / fprime-comp (always `#![no_std]`, std linked back in
// by the feature), most of this crate is std-only OS backend code, so the
// std build keeps the std prelude and only the `no_std` build drops it.
#![cfg_attr(not(feature = "std"), no_std)]
// The crate docs link every facility; in the `no_std` build the std-only
// modules are absent, so those links cannot resolve there.
#![cfg_attr(not(feature = "std"), allow(rustdoc::broken_intra_doc_links))]

extern crate alloc;
// Unit tests always run on a hosted target, so they get `std` even in a
// `no_std` build.
#[cfg(all(test, not(feature = "std")))]
extern crate std;

pub mod condition;
#[cfg(feature = "std")]
pub mod console;
pub mod cpu;
#[cfg(feature = "std")]
pub mod directory;
#[cfg(feature = "std")]
pub mod file;
#[cfg(feature = "std")]
pub mod file_path_utils;
#[cfg(feature = "std")]
pub mod filesystem;
pub mod generic;
pub mod interval_timer;
#[cfg(feature = "std")]
pub mod linux;
pub mod memory;
pub mod mutex;
#[cfg(not(feature = "std"))]
pub mod platform;
pub mod queue;
pub mod rawtime;
#[cfg(feature = "std")]
pub mod sandboxed_file;
pub mod stub;
pub mod task;

pub use condition::ConditionVariable;
#[cfg(feature = "std")]
pub use console::{CONSOLE, Console, ConsoleStream};
pub use cpu::{Cpu, CpuInterface};
#[cfg(feature = "std")]
pub use directory::Directory;
#[cfg(feature = "std")]
pub use file::File;
pub use interval_timer::IntervalTimer;
pub use memory::{Memory, MemoryInterface};
pub use mutex::{OsMutex, ScopeLock};
#[cfg(not(feature = "std"))]
pub use platform::{Platform, TaskHandle, register_platform};
pub use queue::{BlockingType, Queue};
pub use rawtime::RawTime;
#[cfg(feature = "std")]
pub use sandboxed_file::SandboxedFile;
pub use task::Task;

/// Port of `Os::init()` (Os/Os.cpp). The C++ version force-initializes the
/// Console, FileSystem, Cpu, Memory, and Task singletons; the load-bearing
/// side effect is the Console registering itself as the global `Fw::Logger`
/// (the Cpu and Memory singletons are constant-initialized statics, so
/// their `init` calls are kept only for parity).
/// Call once at deployment startup, before any `fw_log!` output matters
/// (C++ parity: log output before this call is silently dropped).
#[cfg(feature = "std")]
pub fn init() {
    console::init();
    cpu::init();
    memory::init();
}

/// `no_std` `Os::init()`: initializes the (stub) Cpu and Memory singletons.
/// There is no console backend without `std` — the deployment registers
/// its own `fprime_fw::FwLogger` — and the [`platform`] is registered
/// separately with [`register_platform`].
#[cfg(not(feature = "std"))]
pub fn init() {
    cpu::init();
    memory::init();
}

/// The unit tests' platform: in a `no_std` test build (which still runs on
/// a hosted target), [`install`](test_platform::install) registers a
/// `std::thread` / `SystemTime` [`platform::Platform`] so the task, raw-time
/// and interval-timer tests exercise the seam; with `std` it is a no-op.
#[cfg(test)]
pub(crate) mod test_platform {
    /// Register the test platform (idempotent).
    pub fn install() {
        #[cfg(not(feature = "std"))]
        let _ = crate::register_platform(&imp::StdThreads);
    }

    #[cfg(not(feature = "std"))]
    mod imp {
        use crate::platform::{Platform, TaskHandle};
        use crate::rawtime::{self, RawTime};
        use crate::task::{self, Arguments};
        use alloc::boxed::Box;
        use fprime_fw::TimeInterval;

        pub struct StdThreads;

        struct Handle(std::thread::JoinHandle<()>);

        impl TaskHandle for Handle {
            fn join(self: Box<Self>) -> task::Status {
                match self.0.join() {
                    Ok(()) => task::Status::OpOk,
                    Err(_) => task::Status::JoinError,
                }
            }
        }

        impl Platform for StdThreads {
            fn start_task(
                &self,
                arguments: Arguments,
            ) -> Result<Box<dyn TaskHandle>, task::Status> {
                std::thread::Builder::new()
                    .name(arguments.name)
                    .spawn(arguments.routine)
                    .map(|handle| Box::new(Handle(handle)) as Box<dyn TaskHandle>)
                    .map_err(|_| task::Status::ErrorResources)
            }

            fn delay(&self, interval: TimeInterval) -> task::Status {
                std::thread::sleep(std::time::Duration::new(
                    u64::from(interval.get_seconds()),
                    interval.get_useconds() * 1000,
                ));
                task::Status::OpOk
            }

            fn raw_time_now(&self) -> Result<RawTime, rawtime::Status> {
                let now = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_err(|_| rawtime::Status::OtherError)?;
                Ok(RawTime::from_parts(now.as_secs(), now.subsec_nanos()))
            }

            fn queue_wait(&self) {
                std::thread::yield_now();
            }
        }
    }
}

#[cfg(all(test, feature = "std"))]
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
