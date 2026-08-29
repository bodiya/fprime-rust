//! The F Prime condition variable.
//!
//! Port of `Os::ConditionVariable` (Os/Condition.{hpp,cpp}) over
//! `std::sync::Condvar` — see `docs/cpp-analysis/os.md`.
//!
//! C++ semantics kept:
//! - the wrapper permanently binds to the FIRST mutex used with
//!   [`ConditionVariable::pend`]; any later pend with a different mutex
//!   returns [`Status::ErrorDifferentMutex`] forever (the binding never
//!   resets, even after all waiters leave);
//! - [`ConditionVariable::wait`] is pend + FW_ASSERT success;
//! - callers must hold the mutex (structurally enforced here: pend consumes
//!   a [`ScopeLock`]) and MUST re-check their predicate in a loop;
//! - `notify`/`notify_all` are intended to be called WITHOUT holding the
//!   mutex (the priority queue notifies after unlock).
//!
//! There is no timed wait in the interface (C++ parity).

use crate::mutex::ScopeLock;
use fprime_fw::fw_assert;
use std::sync::Condvar;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Port of `Os::ConditionVariableInterface::Status` (Os/Condition.hpp) —
/// exact C++ discriminants.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Operation was successful.
    OpOk = 0,
    /// Trying to wait without holding the mutex.
    ErrorMutexNotHeld = 1,
    /// Trying to use a different mutex than the expected mutex.
    ErrorDifferentMutex = 2,
    /// Trying to use a feature that isn't implemented.
    ErrorNotImplemented = 3,
    /// ConditionVariable does not support operation.
    NotSupported = 4,
    /// All other errors.
    ErrorOther = 5,
}

/// The F Prime condition variable (see module docs).
#[derive(Default)]
pub struct ConditionVariable {
    condvar: Condvar,
    /// Identity of the first mutex passed to pend (0 = unbound). Sticky for
    /// the lifetime of the condition variable (C++ parity).
    bound_mutex: AtomicUsize,
}

impl ConditionVariable {
    /// Construct an unbound condition variable.
    pub const fn new() -> Self {
        Self {
            condvar: Condvar::new(),
            bound_mutex: AtomicUsize::new(0),
        }
    }

    /// Atomically release the lock, block until notified, and re-acquire
    /// (port of `pend`). Returns the status and the (re-acquired) lock;
    /// [`Status::ErrorDifferentMutex`] when `lock` guards a different mutex
    /// than the first one ever pended on — the lock is returned still held
    /// and no wait happens.
    pub fn pend<'a>(&self, lock: ScopeLock<'a>) -> (Status, ScopeLock<'a>) {
        let id = lock.mutex().id();
        // Bind to the first mutex; reject any other forever (C++ parity:
        // m_lock is set once and never cleared).
        if self
            .bound_mutex
            .compare_exchange(0, id, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
            && self.bound_mutex.load(Ordering::Acquire) != id
        {
            return (Status::ErrorDifferentMutex, lock);
        }
        (Status::OpOk, lock.wait_on(&self.condvar))
    }

    /// Pend and FW_ASSERT success (port of `wait(mutex)`).
    pub fn wait<'a>(&self, lock: ScopeLock<'a>) -> ScopeLock<'a> {
        let (status, lock) = self.pend(lock);
        fw_assert!(status == Status::OpOk, status as i32);
        lock
    }

    /// Wake one waiter (port of `notify`). Call without holding the mutex.
    pub fn notify(&self) {
        self.condvar.notify_one();
    }

    /// Wake all waiters (port of `notifyAll`). Call without holding the
    /// mutex.
    pub fn notify_all(&self) {
        self.condvar.notify_all();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mutex::OsMutex;
    use std::sync::Arc;
    use std::sync::atomic::AtomicBool;
    use std::time::Duration;

    #[test]
    fn wait_and_notify_round_trip() {
        struct Fixture {
            mutex: OsMutex,
            condition: ConditionVariable,
            flag: AtomicBool,
        }
        let fixture = Arc::new(Fixture {
            mutex: OsMutex::new(),
            condition: ConditionVariable::new(),
            flag: AtomicBool::new(false),
        });
        let waiter_fixture = Arc::clone(&fixture);
        let waiter = std::thread::spawn(move || {
            let mut lock = waiter_fixture.mutex.lock();
            // Predicate re-check loop, as the contract requires.
            while !waiter_fixture.flag.load(Ordering::Acquire) {
                lock = waiter_fixture.condition.wait(lock);
            }
            true
        });
        std::thread::sleep(Duration::from_millis(20));
        {
            let _lock = fixture.mutex.lock();
            fixture.flag.store(true, Ordering::Release);
        }
        // Notify after unlock, as the priority queue does.
        fixture.condition.notify();
        assert!(waiter.join().unwrap());
    }

    // Gotcha: the binding to the first mutex is sticky forever.
    #[test]
    fn pend_with_different_mutex_is_rejected_forever() {
        struct Fixture {
            condition: ConditionVariable,
            mutex_a: OsMutex,
            mutex_b: OsMutex,
        }
        let fixture = Arc::new(Fixture {
            condition: ConditionVariable::new(),
            mutex_a: OsMutex::new(),
            mutex_b: OsMutex::new(),
        });

        // A notifier that keeps signaling until told to stop, so a pend can
        // never miss its wakeup.
        let done = Arc::new(AtomicBool::new(false));
        let notifier_fixture = Arc::clone(&fixture);
        let notifier_done = Arc::clone(&done);
        let notifier = std::thread::spawn(move || {
            while !notifier_done.load(Ordering::Acquire) {
                notifier_fixture.condition.notify_all();
                std::thread::sleep(Duration::from_millis(5));
            }
        });

        // Bind to mutex A.
        let lock = fixture.mutex_a.lock();
        let (status, lock) = fixture.condition.pend(lock);
        assert_eq!(status, Status::OpOk);
        drop(lock);

        // A different mutex is now rejected without waiting.
        let other = fixture.mutex_b.lock();
        let (status, other) = fixture.condition.pend(other);
        assert_eq!(status, Status::ErrorDifferentMutex);
        drop(other);

        // The original mutex still works (rejection did not rebind).
        let lock = fixture.mutex_a.lock();
        let (status, lock) = fixture.condition.pend(lock);
        assert_eq!(status, Status::OpOk);
        drop(lock);

        done.store(true, Ordering::Release);
        notifier.join().unwrap();
    }
}
