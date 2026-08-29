//! Port of `Utils::RateLimiter` — counter-cycle / time-cycle rate limiting.
//!
//! C++ sources: `Utils/RateLimiter.cpp`, `Utils/RateLimiter.hpp`.
//! Analysis: `docs/cpp-analysis/utils-misc.md` (Utils::RateLimiter section).
//!
//! Semantics (C++ parity): with both cycles 0, `trigger` always returns
//! true. Otherwise the result is the OR of the enabled criteria — the
//! counter criterion fires when the counter is 0 (so `counter_cycle == 1`
//! fires every call), the time criterion fires when
//! `now >= last_trigger + time_cycle` OR the stored time is still at
//! "negative infinity" (the initial state). Each enabled dimension then
//! updates: the counter is set to 1-then-wrapped on trigger, else
//! incremented-then-wrapped; the stored time is set to `now` on trigger, and
//! the negative-infinity flag is always cleared. Not thread safe.

use fprime_fw::fw_assert;
use fprime_fw::time::{Time, TimeBase};

/// Rate limiter over a call counter and/or a time cycle
/// (port of `Utils::RateLimiter`).
#[derive(Debug, Clone)]
pub struct RateLimiter {
    // parameters
    counter_cycle: u32,
    time_cycle: u32,
    // state
    counter: u32,
    time: Time,
    time_at_negative_infinity: bool,
}

impl RateLimiter {
    /// Construct with defined cycles (C++ `RateLimiter(U32, U32)`); a cycle
    /// of 0 disables that dimension.
    pub fn new(counter_cycle: u32, time_cycle: u32) -> Self {
        let mut limiter = Self {
            counter_cycle,
            time_cycle,
            counter: 0,
            time: Time::default(),
            time_at_negative_infinity: true,
        };
        limiter.reset();
        limiter
    }

    /// Adjust the counter cycle at run time (C++ `setCounterCycle`).
    pub fn set_counter_cycle(&mut self, counter_cycle: u32) {
        self.counter_cycle = counter_cycle;
    }

    /// Adjust the time cycle (seconds) at run time (C++ `setTimeCycle`).
    pub fn set_time_cycle(&mut self, time_cycle: u32) {
        self.time_cycle = time_cycle;
    }

    /// Reset both dimensions (C++ `reset`).
    pub fn reset(&mut self) {
        self.reset_counter();
        self.reset_time();
    }

    /// Zero the counter (C++ `resetCounter`).
    pub fn reset_counter(&mut self) {
        self.counter = 0;
    }

    /// Reset the stored time to "negative infinity" (C++ `resetTime`).
    pub fn reset_time(&mut self) {
        self.time = Time::default();
        self.time_at_negative_infinity = true;
    }

    /// Manually set the counter state (C++ `setCounter`).
    pub fn set_counter(&mut self, counter: u32) {
        self.counter = counter;
    }

    /// Manually set the time state, clearing the negative-infinity flag
    /// (C++ `setTime`).
    pub fn set_time(&mut self, time: Time) {
        self.time = time;
        self.time_at_negative_infinity = false;
    }

    /// Main entry point (C++ `trigger(Fw::Time)`).
    ///
    /// Factors in only the dimensions with a nonzero cycle; when both are
    /// defined, satisfying EITHER one triggers. With both cycles 0, always
    /// returns true.
    pub fn trigger(&mut self, time: Time) -> bool {
        if self.counter_cycle == 0 && self.time_cycle == 0 {
            return true;
        }

        // Evaluate trigger criteria.
        let mut should_trigger = false;
        if self.counter_cycle > 0 {
            should_trigger = should_trigger || self.should_counter_trigger();
        }
        if self.time_cycle > 0 {
            should_trigger = should_trigger || self.should_time_trigger(time);
        }

        // Update states.
        if self.counter_cycle > 0 {
            self.update_counter(should_trigger);
        }
        if self.time_cycle > 0 {
            self.update_time(should_trigger, time);
        }

        should_trigger
    }

    /// Shorthand for counter-only limiters (C++ argument-less `trigger()`).
    ///
    /// C++ parity: if a time cycle is defined, the caller presumably forgot
    /// to supply a time — fw_asserts (crashes).
    pub fn trigger_counter_only(&mut self) -> bool {
        fw_assert!(self.time_cycle == 0, self.time_cycle as i64);
        self.trigger(Time::zero(TimeBase::TbNone))
    }

    /// C++ `shouldCounterTrigger`: triggers at counter == 0.
    fn should_counter_trigger(&self) -> bool {
        fw_assert!(self.counter_cycle > 0);
        self.counter == 0
    }

    /// C++ `shouldTimeTrigger`: triggers at previous trigger time +
    /// `time_cycle` seconds, OR when the stored time is at negative
    /// infinity.
    ///
    /// C++ parity: the cycle Time carries `TbNone`; `Time::add` fw_asserts
    /// if the stored time has a different base, and the `>=` comparison is
    /// false across differing bases (Incomparable).
    fn should_time_trigger(&self, time: Time) -> bool {
        fw_assert!(self.time_cycle > 0);
        let time_cycle = Time::from_seconds_useconds(self.time_cycle, 0);
        let next_trigger = Time::add(&self.time, &time_cycle);
        time >= next_trigger || self.time_at_negative_infinity
    }

    /// C++ `updateCounter`: 1-then-wrap on trigger (handles
    /// `counter_cycle == 1`), else increment-then-wrap.
    fn update_counter(&mut self, triggered: bool) {
        fw_assert!(self.counter_cycle > 0);
        if triggered {
            self.counter = 1;
        } else {
            self.counter += 1;
        }
        if self.counter >= self.counter_cycle {
            self.counter = 0;
        }
    }

    /// C++ `updateTime`: mark the trigger time; always clear the
    /// negative-infinity flag.
    fn update_time(&mut self, triggered: bool, time: Time) {
        fw_assert!(self.time_cycle > 0);
        if triggered {
            self.time = time;
        }
        self.time_at_negative_infinity = false;
    }
}

impl Default for RateLimiter {
    /// Cycles set to 0 (C++ argument-less constructor).
    fn default() -> Self {
        Self::new(0, 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(seconds: u32) -> Time {
        Time::from_seconds_useconds(seconds, 0)
    }

    #[test]
    fn both_cycles_zero_always_triggers() {
        let mut limiter = RateLimiter::default();
        for s in 0..5 {
            assert!(limiter.trigger(t(s)));
        }
        assert!(limiter.trigger_counter_only());
    }

    #[test]
    fn counter_cycle_of_one_triggers_every_call() {
        // Gotcha: on trigger the counter is set to 1 then wrapped, so
        // counter_cycle == 1 triggers every call.
        let mut limiter = RateLimiter::new(1, 0);
        for _ in 0..5 {
            assert!(limiter.trigger_counter_only());
        }
    }

    #[test]
    fn counter_cycle_triggers_every_nth_call() {
        let mut limiter = RateLimiter::new(3, 0);
        let results: Vec<bool> = (0..9).map(|_| limiter.trigger_counter_only()).collect();
        assert_eq!(
            results,
            [true, false, false, true, false, false, true, false, false]
        );
    }

    #[test]
    fn time_cycle_triggers_first_call_at_negative_infinity() {
        let mut limiter = RateLimiter::new(0, 10);
        // Initial state: time at negative infinity -> immediate trigger.
        assert!(limiter.trigger(t(0)));
        // Within the cycle window: no trigger.
        assert!(!limiter.trigger(t(5)));
        assert!(!limiter.trigger(t(9)));
        // At exactly last-trigger + cycle: trigger.
        assert!(limiter.trigger(t(10)));
        assert!(!limiter.trigger(t(19)));
        assert!(limiter.trigger(t(20)));
    }

    #[test]
    fn or_of_counter_and_time_criteria() {
        // counter cycle 3, time cycle 100: either satisfies.
        let mut limiter = RateLimiter::new(3, 100);
        assert!(limiter.trigger(t(0))); // counter==0 AND neg-infinity
        assert!(!limiter.trigger(t(1)));
        assert!(!limiter.trigger(t(2)));
        assert!(limiter.trigger(t(3))); // counter wrapped to 0
        // Time criterion fires early even though the counter says no.
        assert!(limiter.trigger(t(103)));
        // Trigger reset BOTH dimensions: counter back to 1, time to 103.
        assert!(!limiter.trigger(t(104)));
        assert!(!limiter.trigger(t(105)));
        assert!(limiter.trigger(t(106))); // counter wrap again
    }

    #[test]
    fn reset_time_restores_negative_infinity() {
        let mut limiter = RateLimiter::new(0, 50);
        assert!(limiter.trigger(t(0)));
        assert!(!limiter.trigger(t(1)));
        limiter.reset_time();
        // Negative infinity again: immediate trigger.
        assert!(limiter.trigger(t(2)));
    }

    #[test]
    fn set_time_clears_negative_infinity() {
        let mut limiter = RateLimiter::new(0, 10);
        limiter.set_time(t(100));
        // No longer at negative infinity; window runs from 100.
        assert!(!limiter.trigger(t(105)));
        assert!(limiter.trigger(t(110)));
    }

    #[test]
    fn set_counter_adjusts_phase() {
        let mut limiter = RateLimiter::new(4, 0);
        limiter.set_counter(3);
        // counter=3 -> not 0 -> no trigger; increments to 4 -> wraps to 0.
        assert!(!limiter.trigger_counter_only());
        assert!(limiter.trigger_counter_only());
    }

    #[test]
    fn reset_counter_retriggers_immediately() {
        let mut limiter = RateLimiter::new(5, 0);
        assert!(limiter.trigger_counter_only());
        assert!(!limiter.trigger_counter_only());
        limiter.reset_counter();
        assert!(limiter.trigger_counter_only());
    }

    #[test]
    fn non_trigger_updates_time_flag_but_not_time() {
        // After the first (neg-infinity) trigger at t=7, a non-triggering
        // call must NOT move the window start.
        let mut limiter = RateLimiter::new(0, 10);
        assert!(limiter.trigger(t(7)));
        assert!(!limiter.trigger(t(8)));
        assert!(!limiter.trigger(t(16)));
        assert!(limiter.trigger(t(17))); // 7 + 10
    }

    #[test]
    #[should_panic]
    fn trigger_counter_only_asserts_with_time_cycle() {
        let mut limiter = RateLimiter::new(0, 5);
        let _ = limiter.trigger_counter_only();
    }
}
