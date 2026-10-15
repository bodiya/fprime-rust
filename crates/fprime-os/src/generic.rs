//! The `Os::Generic` namespace: shared OSAL result types and the
//! OS-agnostic queue delegates.
//!
//! - The plain OK/ERROR [`Status`] and the used/total pair [`UsedTotal`]
//!   (`Os/Os.hpp`) shared by [`crate::cpu`] (as `Os::Cpu::Ticks`) and
//!   [`crate::memory`] (as `Os::Memory::Usage`).
//! - The two `Os::Queue` delegates (`Os/Generic/`):
//!   [`PriorityQueue`], the lock-based
//!   default, and
//!   [`LocklessPriorityQueue`],
//!   the ISR-safe one selected by the `lockless-queue` feature (see
//!   [`crate::queue`]).

use fprime_config::FwSizeType;

pub mod lockless_priority_queue;
pub mod priority_queue;

pub use lockless_priority_queue::LocklessPriorityQueue;
pub use priority_queue::PriorityQueue;

/// Port of `Os::Generic::Status` — exact C++ discriminants.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Operation succeeded (`OP_OK`).
    OpOk = 0,
    /// Operation failed (`ERROR`).
    Error = 1,
}

/// Port of `Os::Generic::UsedTotal`: a used amount out of a total amount
/// (CPU ticks or memory bytes).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct UsedTotal {
    /// Used amount.
    pub used: FwSizeType,
    /// Total amount.
    pub total: FwSizeType,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_discriminants_match_cpp() {
        assert_eq!(Status::OpOk as i32, 0);
        assert_eq!(Status::Error as i32, 1);
    }

    #[test]
    fn used_total_defaults_to_zero() {
        assert_eq!(UsedTotal::default(), UsedTotal { used: 0, total: 0 });
    }
}
