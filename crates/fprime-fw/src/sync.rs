//! The framework's lock shim (ROADMAP item 13, the `no_std` OSAL seam).
//!
//! `fprime-fw`, `fprime-os` and `fprime-comp` take every lock, once-cell,
//! reference count and atomic they use from here, so the same code builds
//! in both configurations:
//!
//! - `std` (the default): [`Mutex`] / [`RwLock`] / [`OnceLock`] wrap
//!   `std::sync`, and the guards ARE the `std::sync` guards (so an OS
//!   backend can still pair a [`MutexGuard`] with `std::sync::Condvar`).
//!   No third-party crate is involved.
//! - `spin` without `std` (`no_std` + `alloc`): the same API over the
//!   `spin` crate's spin locks and `spin::Once`.
//!
//! The API is deliberately the subset the framework uses, with infallible
//! locking. `std` locks poison when a holder panics; every framework call
//! site already recovered the guard with `PoisonError::into_inner` (a
//! panic is a failed `fw_assert!`, which fail-stops a release build — see
//! [`crate::assert`] — so recovery only matters to unwinding dev/test
//! builds). The std backend keeps exactly that recovery inside
//! [`Mutex::lock`] / [`RwLock::read`] / [`RwLock::write`]; spin locks never
//! poison, so both backends present the same `lock() -> guard` signature.
//!
//! Spin-lock caveat for `no_std` deployments: a spin lock busy-waits, so a
//! lock must never be taken from an interrupt handler that can preempt a
//! holder of the same lock on the same core (that deadlocks). The framework
//! holds these locks only for short, bounded critical sections.
//!
//! [`AtomicU64`] is `core`'s on targets with 64-bit atomics and otherwise a
//! [`Mutex`]-backed stand-in with the same method subset (thumbv7em, for
//! example, has 32-bit atomics only). [`Arc`] needs pointer-width atomic
//! compare-and-swap, which every supported target has.

pub use alloc::sync::{Arc, Weak};
pub use core::sync::atomic::{
    AtomicBool, AtomicI8, AtomicI16, AtomicI32, AtomicU8, AtomicU16, AtomicU32, AtomicUsize,
    Ordering,
};

#[cfg(target_has_atomic = "64")]
pub use core::sync::atomic::AtomicU64;

#[cfg(not(any(feature = "std", feature = "spin")))]
compile_error!(
    "fprime-fw needs a lock backend: enable the default `std` feature, or build \
     `no_std` with `--no-default-features --features spin`"
);

// ---------------------------------------------------------------------------
// std backend
// ---------------------------------------------------------------------------

#[cfg(feature = "std")]
mod backend {
    use std::sync::PoisonError;

    /// Mutual-exclusion lock (`std::sync::Mutex`, poison-recovering).
    #[derive(Debug, Default)]
    pub struct Mutex<T>(std::sync::Mutex<T>);

    /// Guard returned by [`Mutex::lock`] (the `std::sync` guard itself).
    pub type MutexGuard<'a, T> = std::sync::MutexGuard<'a, T>;

    impl<T> Mutex<T> {
        /// New unlocked mutex.
        pub const fn new(value: T) -> Self {
            Self(std::sync::Mutex::new(value))
        }

        /// Lock, blocking until available. A poisoned lock is recovered
        /// (`PoisonError::into_inner`) — see the module docs.
        pub fn lock(&self) -> MutexGuard<'_, T> {
            self.0.lock().unwrap_or_else(PoisonError::into_inner)
        }

        /// Lock without blocking; `None` when another holder has it.
        pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
            match self.0.try_lock() {
                Ok(guard) => Some(guard),
                Err(std::sync::TryLockError::Poisoned(poisoned)) => Some(poisoned.into_inner()),
                Err(std::sync::TryLockError::WouldBlock) => None,
            }
        }

        /// Mutable access without locking (exclusive borrow).
        pub fn get_mut(&mut self) -> &mut T {
            self.0.get_mut().unwrap_or_else(PoisonError::into_inner)
        }

        /// Consume the mutex, returning the value.
        pub fn into_inner(self) -> T {
            self.0.into_inner().unwrap_or_else(PoisonError::into_inner)
        }
    }

    /// Reader-writer lock (`std::sync::RwLock`, poison-recovering).
    #[derive(Debug, Default)]
    pub struct RwLock<T>(std::sync::RwLock<T>);

    /// Shared guard returned by [`RwLock::read`].
    pub type RwLockReadGuard<'a, T> = std::sync::RwLockReadGuard<'a, T>;
    /// Exclusive guard returned by [`RwLock::write`].
    pub type RwLockWriteGuard<'a, T> = std::sync::RwLockWriteGuard<'a, T>;

    impl<T> RwLock<T> {
        /// New unlocked reader-writer lock.
        pub const fn new(value: T) -> Self {
            Self(std::sync::RwLock::new(value))
        }

        /// Shared lock (poison-recovering).
        pub fn read(&self) -> RwLockReadGuard<'_, T> {
            self.0.read().unwrap_or_else(PoisonError::into_inner)
        }

        /// Exclusive lock (poison-recovering).
        pub fn write(&self) -> RwLockWriteGuard<'_, T> {
            self.0.write().unwrap_or_else(PoisonError::into_inner)
        }
    }

    /// Write-once cell (`std::sync::OnceLock`).
    #[derive(Debug, Default)]
    pub struct OnceLock<T>(std::sync::OnceLock<T>);

    impl<T> OnceLock<T> {
        /// New empty cell.
        pub const fn new() -> Self {
            Self(std::sync::OnceLock::new())
        }

        /// The value, if set.
        pub fn get(&self) -> Option<&T> {
            self.0.get()
        }

        /// Set the value; `Err(value)` if it was already set.
        pub fn set(&self, value: T) -> Result<(), T> {
            self.0.set(value)
        }

        /// The value, initializing it with `init` first if empty.
        pub fn get_or_init(&self, init: impl FnOnce() -> T) -> &T {
            self.0.get_or_init(init)
        }
    }
}

// ---------------------------------------------------------------------------
// spin backend (no_std)
// ---------------------------------------------------------------------------

#[cfg(all(not(feature = "std"), feature = "spin"))]
mod backend {
    /// Mutual-exclusion spin lock (`spin::Mutex`; never poisons).
    #[derive(Debug, Default)]
    pub struct Mutex<T>(spin::Mutex<T>);

    /// Guard returned by [`Mutex::lock`].
    pub type MutexGuard<'a, T> = spin::MutexGuard<'a, T>;

    impl<T> Mutex<T> {
        /// New unlocked mutex.
        pub const fn new(value: T) -> Self {
            Self(spin::Mutex::new(value))
        }

        /// Lock, spinning until available.
        pub fn lock(&self) -> MutexGuard<'_, T> {
            self.0.lock()
        }

        /// Lock without spinning; `None` when another holder has it.
        pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
            self.0.try_lock()
        }

        /// Mutable access without locking (exclusive borrow).
        pub fn get_mut(&mut self) -> &mut T {
            self.0.get_mut()
        }

        /// Consume the mutex, returning the value.
        pub fn into_inner(self) -> T {
            self.0.into_inner()
        }
    }

    /// Reader-writer spin lock (`spin::RwLock`; never poisons).
    #[derive(Debug, Default)]
    pub struct RwLock<T>(spin::RwLock<T>);

    /// Shared guard returned by [`RwLock::read`].
    pub type RwLockReadGuard<'a, T> = spin::RwLockReadGuard<'a, T>;
    /// Exclusive guard returned by [`RwLock::write`].
    pub type RwLockWriteGuard<'a, T> = spin::RwLockWriteGuard<'a, T>;

    impl<T> RwLock<T> {
        /// New unlocked reader-writer lock.
        pub const fn new(value: T) -> Self {
            Self(spin::RwLock::new(value))
        }

        /// Shared lock.
        pub fn read(&self) -> RwLockReadGuard<'_, T> {
            self.0.read()
        }

        /// Exclusive lock.
        pub fn write(&self) -> RwLockWriteGuard<'_, T> {
            self.0.write()
        }
    }

    /// Write-once cell (`spin::Once`).
    #[derive(Debug, Default)]
    pub struct OnceLock<T>(spin::Once<T>);

    impl<T> OnceLock<T> {
        /// New empty cell.
        pub const fn new() -> Self {
            Self(spin::Once::new())
        }

        /// The value, if set.
        pub fn get(&self) -> Option<&T> {
            self.0.get()
        }

        /// Set the value; `Err(value)` if it was already set.
        pub fn set(&self, value: T) -> Result<(), T> {
            let mut slot = Some(value);
            self.0.call_once(|| match slot.take() {
                Some(value) => value,
                // `call_once` runs the closure at most once, and only here,
                // where `slot` was just filled.
                None => unreachable!(),
            });
            match slot {
                None => Ok(()),
                Some(value) => Err(value),
            }
        }

        /// The value, initializing it with `init` first if empty.
        pub fn get_or_init(&self, init: impl FnOnce() -> T) -> &T {
            self.0.call_once(init)
        }
    }
}

#[cfg(any(feature = "std", feature = "spin"))]
pub use backend::{Mutex, MutexGuard, OnceLock, RwLock, RwLockReadGuard, RwLockWriteGuard};

/// `AtomicU64` stand-in for targets without 64-bit atomics: the
/// `core::sync::atomic::AtomicU64` method subset the framework uses, over a
/// [`Mutex`]. Orderings are accepted for signature parity; the lock makes
/// every operation sequentially consistent.
#[cfg(all(not(target_has_atomic = "64"), any(feature = "std", feature = "spin")))]
#[derive(Debug, Default)]
pub struct AtomicU64(Mutex<u64>);

#[cfg(all(not(target_has_atomic = "64"), any(feature = "std", feature = "spin")))]
impl AtomicU64 {
    /// New atomic with the given value.
    pub const fn new(value: u64) -> Self {
        Self(Mutex::new(value))
    }

    /// Load the value.
    pub fn load(&self, _order: Ordering) -> u64 {
        *self.0.lock()
    }

    /// Store a value.
    pub fn store(&self, value: u64, _order: Ordering) {
        *self.0.lock() = value;
    }

    /// Wrapping add, returning the previous value.
    pub fn fetch_add(&self, value: u64, _order: Ordering) -> u64 {
        let mut guard = self.0.lock();
        let previous = *guard;
        *guard = previous.wrapping_add(value);
        previous
    }

    /// Swap, returning the previous value.
    pub fn swap(&self, value: u64, _order: Ordering) -> u64 {
        core::mem::replace(&mut *self.0.lock(), value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mutex_lock_and_try_lock() {
        let m = Mutex::new(5u32);
        {
            let mut guard = m.lock();
            *guard += 1;
            assert!(m.try_lock().is_none(), "held lock must not be re-acquired");
        }
        assert_eq!(*m.try_lock().expect("free lock"), 6);
        let mut m = m;
        *m.get_mut() += 1;
        assert_eq!(m.into_inner(), 7);
    }

    #[test]
    fn rwlock_read_and_write() {
        static L: RwLock<i32> = RwLock::new(1);
        *L.write() = 2;
        let a = L.read();
        let b = L.read();
        assert_eq!(*a + *b, 4);
    }

    #[test]
    fn once_lock_sets_once() {
        let cell: OnceLock<u8> = OnceLock::new();
        assert_eq!(cell.get(), None);
        assert_eq!(cell.set(1), Ok(()));
        assert_eq!(cell.set(2), Err(2));
        assert_eq!(*cell.get_or_init(|| 3), 1);
        let other: OnceLock<u8> = OnceLock::new();
        assert_eq!(*other.get_or_init(|| 3), 3);
    }

    #[test]
    fn atomic_u64_subset() {
        let a = AtomicU64::new(u64::MAX);
        assert_eq!(a.fetch_add(2, Ordering::Relaxed), u64::MAX);
        assert_eq!(a.load(Ordering::Relaxed), 1);
        a.store(9, Ordering::Relaxed);
        assert_eq!(a.swap(4, Ordering::Relaxed), 9);
        assert_eq!(a.load(Ordering::Relaxed), 4);
    }

    #[cfg(feature = "std")]
    #[test]
    fn std_mutex_recovers_from_poison() {
        let m = Arc::new(Mutex::new(0u8));
        let m2 = Arc::clone(&m);
        let _ = std::thread::spawn(move || {
            let _guard = m2.lock();
            panic!("poison the lock");
        })
        .join();
        *m.lock() = 1;
        assert_eq!(*m.lock(), 1);
    }
}
