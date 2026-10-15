//! Port of `Os::Stub::Cpu::StubCpu` (Os/Stub/Cpu.{hpp,cpp}).

use crate::cpu::{CpuInterface, Status, Ticks};
use fprime_config::FwSizeType;

/// Stub CPU backend: no CPU information is available.
#[derive(Debug, Clone, Copy, Default)]
pub struct StubCpu;

impl StubCpu {
    /// Construct the stub (`const` so it can back the static singleton).
    pub const fn new() -> Self {
        Self
    }
}

impl CpuInterface for StubCpu {
    /// C++ parity: `cpu_count = 0`, [`Status::Error`].
    fn get_count(&self, cpu_count: &mut FwSizeType) -> Status {
        *cpu_count = 0;
        Status::Error
    }

    /// C++ parity: `ticks = {used: 1, total: 1}`, [`Status::Error`] for
    /// every index.
    fn get_ticks(&self, ticks: &mut Ticks, _cpu_index: FwSizeType) -> Status {
        ticks.total = 1;
        ticks.used = 1;
        Status::Error
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stub_count_is_zero_and_an_error() {
        let mut count: FwSizeType = 42;
        assert_eq!(StubCpu::new().get_count(&mut count), Status::Error);
        assert_eq!(count, 0);
    }

    #[test]
    fn stub_ticks_are_one_of_one_and_an_error() {
        for index in [0, 1, 10_000] {
            let mut ticks = Ticks { used: 7, total: 9 };
            assert_eq!(StubCpu::new().get_ticks(&mut ticks, index), Status::Error);
            assert_eq!(ticks, Ticks { used: 1, total: 1 });
        }
    }
}
