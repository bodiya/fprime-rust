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

use std::sync::{Condvar, Mutex, MutexGuard, PoisonError};

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

/// RAII lock guard (port of `Os::ScopeLock`): holds the mutex from
/// construction to drop.
pub struct ScopeLock<'a> {
    mutex: &'a OsMutex,
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
}
