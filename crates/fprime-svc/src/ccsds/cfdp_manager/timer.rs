//! CFDP protocol timers — port of `Svc/Ccsds/CfdpManager/Timer.{hpp,cpp}`
//! (an F´ implementation): a seconds countdown advanced by `run1Hz`.

use fprime_fw::fw_assert;

/// C++ `Timer::Status`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TimerStatus {
    /// Never set.
    #[default]
    Uninitialized = 0,
    /// Counting down.
    Running = 1,
    /// Expired (or disabled).
    Expired = 2,
}

/// C++ `Timer`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Timer {
    timer_status: TimerStatus,
    seconds_remaining: u32,
}

impl Timer {
    /// An uninitialized timer.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            timer_status: TimerStatus::Uninitialized,
            seconds_remaining: 0,
        }
    }

    /// Start the timer for `duration` seconds.
    pub fn set_timer(&mut self, duration: u32) {
        self.timer_status = TimerStatus::Running;
        self.seconds_remaining = duration;
    }

    /// Stop the timer (it reads as expired).
    pub fn disable_timer(&mut self) {
        self.timer_status = TimerStatus::Expired;
        self.seconds_remaining = 0;
    }

    /// The timer status.
    #[must_use]
    pub fn get_status(&self) -> TimerStatus {
        self.timer_status
    }

    /// Seconds left.
    #[must_use]
    pub fn seconds_remaining(&self) -> u32 {
        self.seconds_remaining
    }

    /// Advance one second. C++ parity: a running timer with no seconds left
    /// (set with a zero duration) asserts.
    pub fn run(&mut self) {
        if self.timer_status == TimerStatus::Running {
            fw_assert!(self.seconds_remaining > 0);
            self.seconds_remaining -= 1;
            if self.seconds_remaining == 0 {
                self.timer_status = TimerStatus::Expired;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timer_counts_down_to_expired() {
        let mut t = Timer::new();
        assert_eq!(t.get_status(), TimerStatus::Uninitialized);
        t.run();
        assert_eq!(t.get_status(), TimerStatus::Uninitialized);
        t.set_timer(2);
        t.run();
        assert_eq!(t.get_status(), TimerStatus::Running);
        t.run();
        assert_eq!(t.get_status(), TimerStatus::Expired);
        t.run();
        assert_eq!(t.get_status(), TimerStatus::Expired);
        t.set_timer(5);
        t.disable_timer();
        assert_eq!(
            (t.get_status(), t.seconds_remaining()),
            (TimerStatus::Expired, 0)
        );
    }

    #[test]
    #[should_panic]
    fn running_a_zero_duration_timer_asserts() {
        let mut t = Timer::new();
        t.set_timer(0);
        t.run();
    }
}
