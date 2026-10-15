//! The `no_std` OSAL seam (ROADMAP item 13): what a bare-metal or RTOS
//! deployment supplies so the `no_std` subset of this crate can run.
//!
//! In the default `std` build this module does not exist: [`crate::task`]
//! runs on `std::thread`, [`crate::rawtime`] reads `std::time::SystemTime`
//! and [`crate::queue`] blocks on `std::sync::Condvar`. In a build without
//! `std` (`--no-default-features --features spin`), those three facilities
//! call the [`Platform`] registered here instead:
//!
//! | facility                        | `std` backend          | `no_std` seam                      |
//! |---------------------------------|------------------------|------------------------------------|
//! | `Os::Task::start` / `join`      | `std::thread`          | [`Platform::start_task`] + [`TaskHandle::join`] |
//! | `Os::Task::delay`               | `std::thread::sleep`   | [`Platform::delay`]                |
//! | `Os::RawTime::now`              | `SystemTime::now`      | [`Platform::raw_time_now`]         |
//! | blocking `Os::Queue` send/recv  | `Condvar` wait/notify  | poll + [`Platform::queue_wait`]    |
//!
//! Register the platform once, at startup, before any task starts
//! ([`register_platform`]). Until one is registered, `Task::start` and
//! `Task::delay` return [`task::Status::NotSupported`], `RawTime::now`
//! returns [`rawtime::Status::NotSupported`], and blocking queue waits spin
//! with `core::hint::spin_loop` — non-blocking queue operations, the
//! queued-component dispatch loop driven from a superloop or interrupt, and
//! everything in `fprime-fw` need no platform at all.
//!
//! Blocking queues poll instead of waiting on a condition variable: the
//! waiter releases the queue lock, calls [`Platform::queue_wait`], and
//! re-checks. This is the same shape as upstream's
//! `Os::Generic::LocklessPriorityQueue` (Os/Generic/LocklessPriorityQueue.cpp),
//! whose blocking paths back off with `Os::Task::delay` — an RTOS port
//! should yield or delay there so lower-priority senders can run. The queue
//! state itself is guarded by a `spin` lock (`fprime_fw::sync`); do not send
//! to a queue from an interrupt that can preempt a holder of that queue's
//! lock on the same core.

use crate::rawtime::{self, RawTime};
use crate::task::{self, Arguments};
use alloc::boxed::Box;
use fprime_fw::TimeInterval;
use fprime_fw::sync::OnceLock;

/// A started task, as returned by [`Platform::start_task`].
pub trait TaskHandle: Send {
    /// Wait for the task's routine to return (the `Os::Task::join` backend).
    /// Return [`task::Status::OpOk`] when it returned normally,
    /// [`task::Status::JoinError`] when it did not, or
    /// [`task::Status::NotSupported`] when the platform cannot join (common
    /// on RTOSes whose tasks never exit).
    fn join(self: Box<Self>) -> task::Status;
}

/// The services a `no_std` deployment provides to `fprime-os` (see the
/// module docs for which facility uses which method).
pub trait Platform: Sync {
    /// Start `arguments.routine` on a new thread of execution (the
    /// `Os::Task::start` backend). `arguments.routine` is already wrapped
    /// with the `Os::Task` state machine; the platform only has to run it
    /// once. Priority, stack size, CPU affinity and identifier are passed
    /// through untouched ([`task::TASK_DEFAULT`] /
    /// [`task::TASK_PRIORITY_DEFAULT`] mean "platform default"). On failure
    /// return the matching `Os::Task` status (e.g.
    /// [`task::Status::ErrorResources`]).
    fn start_task(&self, arguments: Arguments) -> Result<Box<dyn TaskHandle>, task::Status>;

    /// Suspend the calling task for at least `interval` (the
    /// `Os::Task::delay` backend).
    fn delay(&self, interval: TimeInterval) -> task::Status;

    /// Read the wall clock (the `Os::RawTime::now` backend): seconds and
    /// nanoseconds since an epoch that is consistent across the deployment.
    fn raw_time_now(&self) -> Result<RawTime, rawtime::Status>;

    /// Called each time a blocking `Os::Queue` send finds the queue full or
    /// a blocking receive finds it empty, with the queue lock released;
    /// the operation re-checks the queue when this returns. Default: one
    /// `core::hint::spin_loop` (busy-wait). An RTOS port should yield or
    /// delay briefly here.
    fn queue_wait(&self) {
        core::hint::spin_loop();
    }
}

static PLATFORM: OnceLock<&'static dyn Platform> = OnceLock::new();

/// Register the deployment's platform. Call once at startup, before any
/// task starts. Returns `false` (and keeps the first one) if a platform was
/// already registered — the OSAL backend is a build-time choice in C++, so
/// it is fixed once chosen here as well.
pub fn register_platform(platform: &'static dyn Platform) -> bool {
    PLATFORM.set(platform).is_ok()
}

/// The registered platform, if any.
pub fn get() -> Option<&'static dyn Platform> {
    PLATFORM.get().copied()
}

/// One blocking-queue backoff step: [`Platform::queue_wait`], or a spin
/// hint while no platform is registered.
pub(crate) fn queue_wait() {
    match get() {
        Some(platform) => platform.queue_wait(),
        None => core::hint::spin_loop(),
    }
}
