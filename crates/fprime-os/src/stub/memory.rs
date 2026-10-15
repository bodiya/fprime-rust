//! Port of `Os::Stub::Memory::StubMemory` (Os/Stub/Memory.{hpp,cpp}).

use crate::memory::{MemoryInterface, Status, Usage};

/// Stub memory backend: no memory information is available.
#[derive(Debug, Clone, Copy, Default)]
pub struct StubMemory;

impl StubMemory {
    /// Construct the stub (`const` so it can back the static singleton).
    pub const fn new() -> Self {
        Self
    }
}

impl MemoryInterface for StubMemory {
    /// C++ parity: `usage = {used: 0, total: 0}`, [`Status::Error`].
    fn get_usage(&self, memory_usage: &mut Usage) -> Status {
        memory_usage.used = 0;
        memory_usage.total = 0;
        Status::Error
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stub_usage_is_zero_and_an_error() {
        let mut usage = Usage { used: 5, total: 6 };
        assert_eq!(StubMemory::new().get_usage(&mut usage), Status::Error);
        assert_eq!(usage, Usage { used: 0, total: 0 });
    }
}
