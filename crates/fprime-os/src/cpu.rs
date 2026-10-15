//! The F Prime CPU facility.
//!
//! Port of `Os::Cpu` / `Os::CpuInterface` (Os/Cpu.{hpp,cpp}) — see
//! `docs/cpp-analysis/os.md` ("Cpu / Memory / Generic status").
//!
//! C++ shape: an abstract `CpuInterface` (`_getCount`, `_getTicks`), a
//! `final` front class `Os::Cpu` that placement-news the link-time-selected
//! delegate, and a function-local-static singleton behind the static
//! `Os::Cpu::getCount` / `Os::Cpu::getTicks` wrappers. Here, following the
//! crate's OSAL idiom:
//!
//! - [`CpuInterface`] is the trait every backend implements (the C++
//!   `_getCount`/`_getTicks` delegate methods become `get_count` /
//!   `get_ticks`);
//! - [`Cpu`] is a compile-time type alias for the default backend:
//!   [`crate::linux::cpu::LinuxCpu`] (`/proc/stat`) on Linux, otherwise
//!   [`crate::stub::cpu::StubCpu`] (C++ `Os/Stub/DefaultCpu.cpp`);
//! - the singleton is a `static` instance ([`get_singleton`]) and the C++
//!   static wrappers are the free functions [`get_count`] / [`get_ticks`]
//!   (the same collapse `crate::filesystem` uses).
//!
//! Ticks are CUMULATIVE since boot; the caller differences consecutive
//! samples to get a load figure (C++ contract, see `Svc::SystemResources`).
//!
//! Not ported: `getHandle()` / `CpuHandle` (an opaque C++ delegate-storage
//! artifact with no consumer) and the Darwin (mach) backend, which needs
//! FFI — non-Linux hosts get the stub, exactly like a C++ build linking
//! `Os_Cpu_Stub`.

use crate::generic;
use fprime_config::FwSizeType;

/// C++ `Os::CpuInterface::Status` (an alias of `Os::Generic::Status`).
pub use crate::generic::Status;

/// C++ `Os::CpuInterface::Ticks` (an alias of `Os::Generic::UsedTotal`):
/// cumulative used and total ticks of one CPU.
pub type Ticks = generic::UsedTotal;

/// Port of `Os::CpuInterface`: one CPU-statistics backend.
///
/// Implementations must be cheap enough to call from a rate-group tick and
/// must not block.
pub trait CpuInterface: Send + Sync {
    /// Request the count of the CPUs detected by the system (C++
    /// `_getCount`). On [`Status::Error`] the count is unspecified (the
    /// shipped backends write 0).
    fn get_count(&self, cpu_count: &mut FwSizeType) -> Status;

    /// Get the cumulative tick information for CPU `cpu_index` (C++
    /// `_getTicks`). The caller differences consecutive samples.
    fn get_ticks(&self, ticks: &mut Ticks, cpu_index: FwSizeType) -> Status;
}

/// The front type: the build-time-selected default backend (C++
/// `Os/Linux/DefaultCpu.cpp`).
#[cfg(all(feature = "std", target_os = "linux"))]
pub type Cpu = crate::linux::cpu::LinuxCpu;

/// The front type: the build-time-selected default backend (C++
/// `Os/Stub/DefaultCpu.cpp` — no zero-dependency, `unsafe`-free CPU tick
/// source exists off Linux).
#[cfg(not(all(feature = "std", target_os = "linux")))]
pub type Cpu = crate::stub::cpu::StubCpu;

/// The process-wide CPU instance (C++ `Cpu::getSingleton()`'s
/// function-local static).
static CPU: Cpu = Cpu::new();

/// Initialize the singleton (C++ `Cpu::init`, called by [`crate::init`]).
/// The instance is a constant-initialized `static`, so this is a no-op kept
/// for API parity.
pub fn init() {
    let _ = get_singleton();
}

/// The process-wide CPU instance (C++ `Cpu::getSingleton`).
pub fn get_singleton() -> &'static Cpu {
    &CPU
}

/// Request the count of the CPUs detected by the system through the
/// singleton (C++ static `Os::Cpu::getCount`).
pub fn get_count(cpu_count: &mut FwSizeType) -> Status {
    get_singleton().get_count(cpu_count)
}

/// Get the cumulative tick information for CPU `cpu_index` through the
/// singleton (C++ static `Os::Cpu::getTicks`).
pub fn get_ticks(ticks: &mut Ticks, cpu_index: FwSizeType) -> Status {
    get_singleton().get_ticks(ticks, cpu_index)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Port of Os/test/ut/cpu/CommonCpuTests.cpp (Basic.Count / Basic.Ticks)
    // against the default backend. The stub backend deliberately fails
    // these, so they only run where the real backend is selected.
    #[cfg(all(feature = "std", target_os = "linux"))]
    #[test]
    fn default_backend_reports_a_positive_count() {
        let mut count: FwSizeType = 0;
        assert_eq!(get_count(&mut count), Status::OpOk);
        assert!(count > 0);
    }

    #[cfg(all(feature = "std", target_os = "linux"))]
    #[test]
    fn default_backend_reports_ticks_for_every_cpu() {
        let mut count: FwSizeType = 0;
        assert_eq!(get_count(&mut count), Status::OpOk);
        assert!(count > 0);
        for i in 0..count {
            let mut ticks = Ticks::default();
            assert_eq!(get_ticks(&mut ticks, i), Status::OpOk, "CPU {i}");
            assert!(ticks.used > 0, "no used ticks on CPU {i}");
            assert!(ticks.total > 0, "no total on CPU {i}");
            assert!(ticks.total >= ticks.used, "total below used on CPU {i}");
        }
    }

    #[test]
    fn index_past_the_count_is_an_error() {
        let mut count: FwSizeType = 0;
        let _ = get_count(&mut count);
        let mut ticks = Ticks::default();
        assert_eq!(get_ticks(&mut ticks, count), Status::Error);
    }

    #[test]
    fn singleton_and_init_are_stable() {
        init();
        assert!(std::ptr::eq(get_singleton(), get_singleton()));
    }
}
