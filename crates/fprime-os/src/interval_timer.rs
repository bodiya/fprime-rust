//! The F Prime interval timer.
//!
//! Port of `Os::IntervalTimer` (Os/IntervalTimer.{hpp,cpp}) — two
//! [`RawTime`] samples with start/stop bookends. See
//! `docs/cpp-analysis/os.md`.
//!
//! C++ parity gotcha: [`IntervalTimer::get_diff_usec`] silently returns
//! `u32::MAX` on overflow (the underlying status is swallowed);
//! [`IntervalTimer::get_time_interval`] returns the status.

use crate::rawtime::{RawTime, Status};
use fprime_fw::TimeInterval;

/// Elapsed-time measurement between [`IntervalTimer::start`] and
/// [`IntervalTimer::stop`].
#[derive(Debug, Clone, Copy, Default)]
pub struct IntervalTimer {
    start_time: RawTime,
    stop_time: RawTime,
}

impl IntervalTimer {
    /// Construct a timer with zero start/stop samples.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sample the clock as the interval start (errors ignored, C++ parity).
    pub fn start(&mut self) {
        let _ = self.start_time.now();
    }

    /// Sample the clock as the interval stop (errors ignored, C++ parity).
    pub fn stop(&mut self) {
        let _ = self.stop_time.now();
    }

    /// Microseconds between start and stop; `u32::MAX` on overflow (the
    /// [`Status::OpOverflow`] is swallowed, C++ parity).
    pub fn get_diff_usec(&self) -> u32 {
        let mut result: u32 = 0;
        let status = self.stop_time.get_diff_usec(&self.start_time, &mut result);
        if status == Status::OpOverflow {
            result = u32::MAX;
        }
        result
    }

    /// The interval between start and stop as a [`TimeInterval`].
    pub fn get_time_interval(&self, interval: &mut TimeInterval) -> Status {
        self.stop_time.get_time_interval(&self.start_time, interval)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn measures_a_sleep() {
        let mut timer = IntervalTimer::new();
        timer.start();
        std::thread::sleep(std::time::Duration::from_millis(10));
        timer.stop();
        let elapsed = timer.get_diff_usec();
        assert!(elapsed >= 10_000, "elapsed {elapsed} us");
        // Sanity upper bound: well below a minute even on a loaded machine.
        assert!(elapsed < 60_000_000);

        let mut interval = TimeInterval::new(0, 0);
        assert_eq!(timer.get_time_interval(&mut interval), Status::OpOk);
        assert!(interval.get_seconds() > 0 || interval.get_useconds() >= 10_000);
    }

    #[test]
    fn overflow_saturates_silently() {
        let mut timer = IntervalTimer::new();
        // Fabricate a > 71.6 minute interval via the raw parts: start at 0,
        // stop at 2 hours.
        timer.start_time = RawTime::from_parts(0, 0);
        timer.stop_time = RawTime::from_parts(7200, 0);
        assert_eq!(timer.get_diff_usec(), u32::MAX);
    }
}
