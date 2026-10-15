//! The F Prime memory-usage facility.
//!
//! Port of `Os::Memory` / `Os::MemoryInterface` (Os/Memory.{hpp,cpp}) — see
//! `docs/cpp-analysis/os.md` ("Cpu / Memory / Generic status").
//!
//! Same OSAL collapse as [`crate::cpu`]: [`MemoryInterface`] is the backend
//! trait (C++ `_getUsage` becomes `get_usage`), [`Memory`] is a
//! compile-time alias for the default backend
//! ([`crate::linux::memory::LinuxMemory`] on Linux, otherwise
//! [`crate::stub::memory::StubMemory`]), the singleton is a `static`
//! ([`get_singleton`]) and the C++ static wrapper `Os::Memory::getUsage` is
//! the free function [`get_usage`].
//!
//! Usage is reported in BYTES (`Svc::SystemResources` divides down to KiB).
//!
//! Not ported: `getHandle()` / `MemoryHandle` (C++ delegate-storage
//! artifact) and the Darwin backend (needs FFI; non-Linux hosts get the
//! stub).

use crate::generic;

/// C++ `Os::MemoryInterface::Status` (an alias of `Os::Generic::Status`).
pub use crate::generic::Status;

/// C++ `Os::MemoryInterface::Usage` (an alias of `Os::Generic::UsedTotal`):
/// used and total system RAM in bytes.
pub type Usage = generic::UsedTotal;

/// Port of `Os::MemoryInterface`: one memory-statistics backend.
///
/// Implementations must be cheap enough to call from a rate-group tick and
/// must not block.
pub trait MemoryInterface: Send + Sync {
    /// Get system memory usage in bytes (C++ `_getUsage`).
    fn get_usage(&self, memory_usage: &mut Usage) -> Status;
}

/// The front type: the build-time-selected default backend (C++
/// `Os/Linux/DefaultMemory.cpp`).
#[cfg(target_os = "linux")]
pub type Memory = crate::linux::memory::LinuxMemory;

/// The front type: the build-time-selected default backend (C++
/// `Os/Stub/DefaultMemory.cpp`).
#[cfg(not(target_os = "linux"))]
pub type Memory = crate::stub::memory::StubMemory;

/// The process-wide memory instance (C++ `Memory::getSingleton()`'s
/// function-local static).
static MEMORY: Memory = Memory::new();

/// Initialize the singleton (C++ `Memory::init`, called by [`crate::init`]).
/// The instance is a constant-initialized `static`, so this is a no-op kept
/// for API parity.
pub fn init() {
    let _ = get_singleton();
}

/// The process-wide memory instance (C++ `Memory::getSingleton`).
pub fn get_singleton() -> &'static Memory {
    &MEMORY
}

/// Get system memory usage through the singleton (C++ static
/// `Os::Memory::getUsage`).
pub fn get_usage(memory_usage: &mut Usage) -> Status {
    get_singleton().get_usage(memory_usage)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Port of Os/test/ut/memory/CommonMemoryTests.cpp (Basic.Usage) against
    // the default backend; the stub deliberately reports 0/0.
    #[cfg(target_os = "linux")]
    #[test]
    fn default_backend_reports_nonzero_usage() {
        let mut usage = Usage::default();
        assert_eq!(get_usage(&mut usage), Status::OpOk);
        assert!(usage.used > 0);
        assert!(usage.total > 0);
        assert!(usage.total >= usage.used);
    }

    #[test]
    fn singleton_and_init_are_stable() {
        init();
        assert!(std::ptr::eq(get_singleton(), get_singleton()));
    }
}
