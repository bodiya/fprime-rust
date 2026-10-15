//! The F Prime mutex.
//!
//! Port of `Os::Mutex` (Os/Mutex.{hpp,cpp}) as a thin wrapper over
//! `std::sync::Mutex<()>` — see `docs/cpp-analysis/os.md`.
//!
//! Deviation from C++ (documented): the C++ `take()`/`release()` raw pair is
//! replaced by the RAII [`ScopeLock`] guard (`Os::ScopeLock` made
//! structural) — Rust cannot hold a std mutex across unpaired calls without
//! unsafe code. `lock()` is the C++ `Mutex::lock()` (assert-on-failure)
//! equivalent; [`OsMutex::try_lock`] maps contention to
//! [`Status::ErrorBusy`]. Poisoning (a panicked holder) is recovered rather
//! than surfaced as a new status: in C++ that thread would already have
//! crashed the process via FW_ASSERT.
//!
//! Priority inheritance and errorcheck semantics of the Posix backend are
//! not reproduced by std (noted divergence; recursive locking deadlocks
//! instead of asserting).
//!
//! `no_std`: the same API over the `fprime_fw::sync` spin mutex. A
//! contended [`OsMutex::lock`] polls `try_lock`, calling
//! `platform::Platform::sync_wait` between attempts (a spin hint with no
//! platform registered), so an RTOS port can yield to the holder instead
//! of spinning against it; [`OsMutex::try_lock`] never waits. Upstream's
//! `Os::Stub::Mutex` (for platforms that cannot block) fails a contended
//! `take()` with `ERROR_BUSY` and so trips the `lock()` assert; here
//! `lock()` waits instead, and `try_lock()` is the stub's `take()`.
//! Like any spin lock it must not be taken from an interrupt that can
//! preempt a holder on the same core.

#[cfg(feature = "std")]
use std::sync::{Condvar, Mutex, MutexGuard, PoisonError};

#[cfg(not(feature = "std"))]
use fprime_fw::sync::{Mutex, MutexGuard};

/// Port of `Os::MutexInterface::Status` (Os/Mutex.hpp) — exact C++
/// discriminants.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Operation was successful.
    OpOk = 0,
    /// Mutex is busy.
    ErrorBusy = 1,
    /// Deadlock condition detected.
    ErrorDeadlock = 2,
    /// Mutex does not support operation.
    NotSupported = 3,
    /// All other errors.
    ErrorOther = 4,
}

/// The F Prime raw mutex (see module docs). Created unlocked (C++ parity).
#[derive(Default)]
pub struct OsMutex {
    inner: Mutex<()>,
}

#[cfg(feature = "std")]
impl OsMutex {
    /// Construct an unlocked mutex.
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(()),
        }
    }

    /// Lock the mutex, blocking (C++ `Mutex::lock()`, which FW_ASSERTs
    /// success). The lock is released when the returned guard drops.
    pub fn lock(&self) -> ScopeLock<'_> {
        ScopeLock {
            mutex: self,
            guard: self.inner.lock().unwrap_or_else(PoisonError::into_inner),
        }
    }

    /// Try to lock the mutex without blocking; [`Status::ErrorBusy`] when
    /// held elsewhere (the C++ `take()` EBUSY mapping).
    pub fn try_lock(&self) -> Result<ScopeLock<'_>, Status> {
        match self.inner.try_lock() {
            Ok(guard) => Ok(ScopeLock { mutex: self, guard }),
            Err(std::sync::TryLockError::WouldBlock) => Err(Status::ErrorBusy),
            Err(std::sync::TryLockError::Poisoned(poisoned)) => Ok(ScopeLock {
                mutex: self,
                guard: poisoned.into_inner(),
            }),
        }
    }

    /// Stable identity of this mutex for the condition variable's sticky
    /// same-mutex check.
    pub(crate) fn id(&self) -> usize {
        std::ptr::from_ref(self) as usize
    }
}

#[cfg(not(feature = "std"))]
impl OsMutex {
    /// Construct an unlocked mutex.
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(()),
        }
    }

    /// Lock the mutex, waiting until available (C++ `Mutex::lock()`). A
    /// contended lock re-tries after each `Platform::sync_wait` backoff
    /// (see the module docs). The lock is released when the returned guard
    /// drops.
    pub fn lock(&self) -> ScopeLock<'_> {
        loop {
            if let Some(guard) = self.inner.try_lock() {
                return ScopeLock { mutex: self, guard };
            }
            crate::platform::sync_wait();
        }
    }

    /// Try to lock the mutex without waiting; [`Status::ErrorBusy`] when
    /// held elsewhere (the C++ `take()` busy mapping, and exactly upstream's
    /// `Os::Stub::Mutex::take`).
    pub fn try_lock(&self) -> Result<ScopeLock<'_>, Status> {
        match self.inner.try_lock() {
            Some(guard) => Ok(ScopeLock { mutex: self, guard }),
            None => Err(Status::ErrorBusy),
        }
    }

    /// Stable identity of this mutex for the condition variable's sticky
    /// same-mutex check.
    pub(crate) fn id(&self) -> usize {
        core::ptr::from_ref(self) as usize
    }
}

/// RAII lock guard (port of `Os::ScopeLock`): holds the mutex from
/// construction to drop.
pub struct ScopeLock<'a> {
    mutex: &'a OsMutex,
    // Held for its Drop (the unlock); read only by the std `wait_on`.
    #[cfg_attr(not(feature = "std"), allow(dead_code))]
    guard: MutexGuard<'a, ()>,
}

impl<'a> ScopeLock<'a> {
    /// The mutex this guard locks (used by the condition variable to verify
    /// and re-acquire).
    pub(crate) fn mutex(&self) -> &'a OsMutex {
        self.mutex
    }

    /// Atomically release the lock, wait on `condvar`, and re-acquire
    /// (internal plumbing for [`crate::ConditionVariable`]).
    #[cfg(feature = "std")]
    pub(crate) fn wait_on(self, condvar: &Condvar) -> ScopeLock<'a> {
        let mutex = self.mutex;
        let guard = condvar
            .wait(self.guard)
            .unwrap_or_else(PoisonError::into_inner);
        ScopeLock { mutex, guard }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn lock_and_release_via_guard() {
        let mutex = OsMutex::new();
        {
            let _lock = mutex.lock();
            // Held: try_lock from the same thread reports busy (std
            // non-recursive semantics).
            assert_eq!(mutex.try_lock().err(), Some(Status::ErrorBusy));
        }
        // Released on guard drop.
        assert!(mutex.try_lock().is_ok());
    }

    #[test]
    fn guard_excludes_other_threads() {
        let mutex = Arc::new(OsMutex::new());
        let contender_mutex = Arc::clone(&mutex);
        let _lock = mutex.lock();
        let contender = std::thread::spawn(move || contender_mutex.try_lock().is_err());
        assert!(contender.join().unwrap());
    }

    #[test]
    fn contended_lock_waits_for_release() {
        use std::sync::atomic::{AtomicU32, Ordering};
        struct Shared {
            mutex: OsMutex,
            // Read-yield-write under the lock: lost updates would show any
            // failure of mutual exclusion.
            value: AtomicU32,
        }
        let shared = Arc::new(Shared {
            mutex: OsMutex::new(),
            value: AtomicU32::new(0),
        });
        let workers: std::vec::Vec<_> = (0..4)
            .map(|_| {
                let shared = Arc::clone(&shared);
                std::thread::spawn(move || {
                    for _ in 0..500 {
                        let _lock = shared.mutex.lock();
                        let value = shared.value.load(Ordering::Relaxed);
                        std::thread::yield_now();
                        shared.value.store(value + 1, Ordering::Relaxed);
                    }
                })
            })
            .collect();
        for worker in workers {
            worker.join().unwrap();
        }
        assert_eq!(shared.value.load(Ordering::Relaxed), 2000);
    }
}
