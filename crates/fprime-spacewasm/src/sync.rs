//! A minimal spin lock: the registry and the guest pool are shared between
//! threads (and the registry is a `static`), and this crate is `no_std`
//! with no lock dependency. Every critical section is a handful of
//! bookkeeping operations; the blocking, OS-level serialization of whole
//! store operations is the component's (C++ `getGlobalAllocatorLock`).

use core::cell::UnsafeCell;
use core::hint::spin_loop;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, Ordering};

/// A value guarded by a test-and-set spin lock.
pub(crate) struct SpinLock<T> {
    locked: AtomicBool,
    value: UnsafeCell<T>,
}

// SAFETY: `value` is only reached through a `SpinGuard`, and at most one
// guard exists at a time (`lock` acquires `locked` with a successful
// compare-exchange, `SpinGuard::drop` releases it), so sharing the lock
// between threads only ever moves `T` between them: `T: Send` suffices.
#[allow(unsafe_code)]
unsafe impl<T: Send> Sync for SpinLock<T> {}

impl<T> SpinLock<T> {
    /// A new unlocked lock.
    pub(crate) const fn new(value: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            value: UnsafeCell::new(value),
        }
    }

    /// Spin until the lock is ours.
    pub(crate) fn lock(&self) -> SpinGuard<'_, T> {
        while self
            .locked
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            spin_loop();
        }
        SpinGuard { lock: self }
    }
}

/// Exclusive access to a [`SpinLock`]'s value.
pub(crate) struct SpinGuard<'a, T> {
    lock: &'a SpinLock<T>,
}

impl<T> Deref for SpinGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: the guard holds the lock, so no other reference to the
        // value exists (see the `Sync` impl).
        #[allow(unsafe_code)]
        unsafe {
            &*self.lock.value.get()
        }
    }
}

impl<T> DerefMut for SpinGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: as in `deref`; `&mut self` makes this the only borrow
        // through the guard.
        #[allow(unsafe_code)]
        unsafe {
            &mut *self.lock.value.get()
        }
    }
}

impl<T> Drop for SpinGuard<'_, T> {
    fn drop(&mut self) {
        self.lock.locked.store(false, Ordering::Release);
    }
}
