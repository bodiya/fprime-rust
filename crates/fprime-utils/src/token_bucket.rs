//! Port of `Utils::TokenBucket` — token-bucket rate limiting over `Fw::Time`.
//!
//! C++ sources: `Utils/TokenBucket.cpp`, `Utils/TokenBucket.hpp`.
//! Analysis: `docs/cpp-analysis/utils-misc.md` (Utils::TokenBucket section).
//!
//! Semantics (C++ parity): `trigger(now)` first replenishes — while below
//! `max_tokens` and `stored_time + interval <= now`, it adds
//! `min(replenish_rate, max - tokens)` tokens and advances the stored time
//! by one whole interval (the stored time is the last replenish instant, not
//! `now`); once full and the stored time is behind `now`, the stored time
//! snaps to `now`. It then consumes one token if available (returns true),
//! else returns false. If time moves backwards no replenish occurs, but a
//! stored token can still be consumed. `replenish()` SETS tokens to the
//! maximum (it does not add). Not thread safe.

use fprime_fw::fw_assert;
use fprime_fw::time::Time;

/// Maximum token count accepted by [`TokenBucket::new`]
/// (C++ `MAX_TOKEN_BUCKET_TOKENS`; asserted only in the 2-arg constructor).
pub const MAX_TOKEN_BUCKET_TOKENS: u32 = 1000;

/// Token bucket rate limiter (port of `Utils::TokenBucket`).
#[derive(Debug, Clone)]
pub struct TokenBucket {
    // parameters
    /// Replenish interval in MICROSECONDS.
    replenish_interval: u32,
    max_tokens: u32,
    replenish_rate: u32,
    // state
    tokens: u32,
    time: Time,
}

impl TokenBucket {
    /// Standard constructor (C++ `TokenBucket(U32, U32)`):
    /// `replenish_rate = 1`, starts full, start time (0, 0).
    ///
    /// `replenish_interval` is in microseconds. C++ parity: fw_asserts
    /// (crashes) when `max_tokens > MAX_TOKEN_BUCKET_TOKENS`; the full
    /// constructor [`TokenBucket::with_state`] does NOT check.
    pub fn new(replenish_interval: u32, max_tokens: u32) -> Self {
        fw_assert!(max_tokens <= MAX_TOKEN_BUCKET_TOKENS, max_tokens);
        Self {
            replenish_interval,
            max_tokens,
            replenish_rate: 1,
            tokens: max_tokens,
            time: Time::from_seconds_useconds(0, 0),
        }
    }

    /// Full constructor (C++ 5-arg `TokenBucket`): every parameter and the
    /// initial state supplied by the caller. `replenish_interval` is in
    /// microseconds; `replenish_rate` is tokens added per interval.
    pub fn with_state(
        replenish_interval: u32,
        max_tokens: u32,
        replenish_rate: u32,
        start_tokens: u32,
        start_time: Time,
    ) -> Self {
        Self {
            replenish_interval,
            max_tokens,
            replenish_rate,
            tokens: start_tokens,
            time: start_time,
        }
    }

    /// Adjust the replenish interval (microseconds) at run time
    /// (C++ `setReplenishInterval`).
    pub fn set_replenish_interval(&mut self, replenish_interval: u32) {
        self.replenish_interval = replenish_interval;
    }

    /// Adjust the maximum token count at run time (C++ `setMaxTokens`).
    pub fn set_max_tokens(&mut self, max_tokens: u32) {
        self.max_tokens = max_tokens;
    }

    /// Adjust the tokens-per-interval replenish rate at run time
    /// (C++ `setReplenishRate`).
    pub fn set_replenish_rate(&mut self, replenish_rate: u32) {
        self.replenish_rate = replenish_rate;
    }

    /// The replenish interval in microseconds (C++ `getReplenishInterval`).
    pub fn get_replenish_interval(&self) -> u32 {
        self.replenish_interval
    }

    /// The maximum token count (C++ `getMaxTokens`).
    pub fn get_max_tokens(&self) -> u32 {
        self.max_tokens
    }

    /// The tokens-per-interval replenish rate (C++ `getReplenishRate`).
    pub fn get_replenish_rate(&self) -> u32 {
        self.replenish_rate
    }

    /// The current token count (C++ `getTokens`).
    pub fn get_tokens(&self) -> u32 {
        self.tokens
    }

    /// Manual replenish: SET tokens to the maximum if below it — this does
    /// not add (C++ `replenish`; documented gotcha).
    pub fn replenish(&mut self) {
        if self.tokens < self.max_tokens {
            self.tokens = self.max_tokens;
        }
    }

    /// Main entry point (C++ `trigger(Fw::Time)`): replenish based on the
    /// time elapsed since the last replenish instant, then consume one token
    /// if available.
    ///
    /// C++ parity: the interval Time carries `TbNone` (from the
    /// seconds/useconds constructor); `Time::add` fw_asserts if the stored
    /// time has a different base, and comparisons across differing bases are
    /// false (Incomparable).
    pub fn trigger(&mut self, time: Time) -> bool {
        // Attempt replenishing.
        if self.replenish_rate > 0 {
            let replenish_interval = Time::from_seconds_useconds(
                self.replenish_interval / 1_000_000,
                self.replenish_interval % 1_000_000,
            );
            let mut next_time = Time::add(&self.time, &replenish_interval);
            while self.tokens < self.max_tokens && next_time <= time {
                // Replenish by the replenish rate, or up to max_tokens.
                self.tokens += self.replenish_rate.min(self.max_tokens - self.tokens);
                self.time = next_time;
                next_time = Time::add(&self.time, &replenish_interval);
            }
            if self.tokens >= self.max_tokens && self.time < time {
                self.time = time;
            }
        }

        // Attempt consuming a token.
        if self.tokens > 0 {
            self.tokens -= 1;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(seconds: u32, useconds: u32) -> Time {
        Time::from_seconds_useconds(seconds, useconds)
    }

    #[test]
    fn starts_full_and_drains() {
        // 1-second interval, 3 tokens, rate 1, starts full at time (0,0).
        let mut bucket = TokenBucket::new(1_000_000, 3);
        assert_eq!(bucket.get_tokens(), 3);
        assert!(bucket.trigger(t(0, 0)));
        assert!(bucket.trigger(t(0, 0)));
        assert!(bucket.trigger(t(0, 0)));
        // Empty, no time elapsed: no replenish, no token.
        assert!(!bucket.trigger(t(0, 0)));
        assert_eq!(bucket.get_tokens(), 0);
    }

    #[test]
    fn replenishes_one_token_per_interval() {
        let mut bucket = TokenBucket::new(1_000_000, 2);
        assert!(bucket.trigger(t(0, 0)));
        assert!(bucket.trigger(t(0, 0)));
        assert!(!bucket.trigger(t(0, 500_000))); // half an interval: nothing
        // One interval elapsed since the (0,0) start: one token back.
        assert!(bucket.trigger(t(1, 0)));
        assert!(!bucket.trigger(t(1, 0)));
        // Three more intervals elapse; capped at max (2), both consumable.
        assert!(bucket.trigger(t(4, 0)));
        assert!(bucket.trigger(t(4, 0)));
        assert!(!bucket.trigger(t(4, 0)));
    }

    #[test]
    fn replenish_loop_advances_stored_time_in_whole_intervals() {
        // Gotcha: m_time is the last replenish instant, not `now`.
        // Empty bucket, 1s interval, rate 1, max 10, start time (0,0).
        let mut bucket = TokenBucket::with_state(1_000_000, 10, 1, 0, t(0, 0));
        // At t=3.5s: three whole intervals -> 3 tokens; one consumed.
        assert!(bucket.trigger(t(3, 500_000)));
        assert_eq!(bucket.get_tokens(), 2);
        // Stored time advanced to 3.0 (not 3.5): the next token arrives at
        // 4.0, so nothing replenishes at 3.9.
        assert!(bucket.trigger(t(3, 900_000)));
        assert!(bucket.trigger(t(3, 900_000)));
        assert!(!bucket.trigger(t(3, 900_000)));
        assert!(bucket.trigger(t(4, 0)));
    }

    #[test]
    fn stored_time_snaps_to_now_once_full() {
        // Gotcha: once tokens >= max and m_time < now, m_time = now.
        let mut bucket = TokenBucket::with_state(1_000_000, 2, 1, 2, t(0, 0));
        // Full bucket at t=100: stored time snaps to 100, then consume.
        assert!(bucket.trigger(t(100, 0)));
        assert_eq!(bucket.get_tokens(), 1);
        // If the time had stayed at 0, dozens of intervals would replenish
        // here; the snap means only 1 interval (101) has elapsed.
        assert!(bucket.trigger(t(101, 0)));
        assert!(bucket.trigger(t(101, 0)));
        assert!(!bucket.trigger(t(101, 0)));
    }

    #[test]
    fn replenish_rate_adds_multiple_tokens_per_interval() {
        let mut bucket = TokenBucket::with_state(1_000_000, 5, 3, 0, t(0, 0));
        assert!(!bucket.trigger(t(0, 500_000)));
        // One interval: +3 tokens.
        assert!(bucket.trigger(t(1, 0)));
        assert_eq!(bucket.get_tokens(), 2);
        // Two intervals from t=1: +3 then capped at 5 (+2).
        assert!(bucket.trigger(t(3, 0)));
        assert_eq!(bucket.get_tokens(), 4);
    }

    #[test]
    fn zero_replenish_rate_never_replenishes() {
        let mut bucket = TokenBucket::with_state(1_000_000, 5, 0, 2, t(0, 0));
        assert!(bucket.trigger(t(10, 0)));
        assert!(bucket.trigger(t(20, 0)));
        assert!(!bucket.trigger(t(1000, 0)));
        assert_eq!(bucket.get_tokens(), 0);
    }

    #[test]
    fn time_moving_backwards_consumes_stored_tokens_only() {
        // Gotcha: no replenish when time < stored time, but stored tokens
        // are still consumable.
        let mut bucket = TokenBucket::with_state(1_000_000, 5, 1, 2, t(100, 0));
        assert!(bucket.trigger(t(50, 0)));
        assert!(bucket.trigger(t(50, 0)));
        assert!(!bucket.trigger(t(50, 0)));
        // Still no replenish while behind the stored time.
        assert!(!bucket.trigger(t(99, 0)));
        // Time catches up: replenish resumes at 101 (100 + interval).
        assert!(bucket.trigger(t(101, 0)));
    }

    #[test]
    fn manual_replenish_sets_to_max_does_not_add() {
        let mut bucket = TokenBucket::with_state(1_000_000, 4, 1, 1, t(0, 0));
        bucket.replenish();
        assert_eq!(bucket.get_tokens(), 4);
        // Already at max: no change (and never above max).
        bucket.replenish();
        assert_eq!(bucket.get_tokens(), 4);
    }

    #[test]
    fn sub_second_interval_arithmetic() {
        // 250 ms interval: interval Time = (0, 250000).
        let mut bucket = TokenBucket::with_state(250_000, 4, 1, 0, t(0, 0));
        assert!(!bucket.trigger(t(0, 200_000)));
        // 1.05 s = 4 whole intervals -> 4 tokens (capped at max 4).
        assert!(bucket.trigger(t(1, 50_000)));
        assert_eq!(bucket.get_tokens(), 3);
    }

    #[test]
    fn getters_and_setters() {
        let mut bucket = TokenBucket::new(500, 10);
        assert_eq!(bucket.get_replenish_interval(), 500);
        assert_eq!(bucket.get_max_tokens(), 10);
        assert_eq!(bucket.get_replenish_rate(), 1);
        assert_eq!(bucket.get_tokens(), 10);
        bucket.set_replenish_interval(1_000);
        bucket.set_max_tokens(20);
        bucket.set_replenish_rate(5);
        assert_eq!(bucket.get_replenish_interval(), 1_000);
        assert_eq!(bucket.get_max_tokens(), 20);
        assert_eq!(bucket.get_replenish_rate(), 5);
    }

    #[test]
    #[should_panic]
    fn new_asserts_max_tokens_limit() {
        // Gotcha: MAX_TOKEN_BUCKET_TOKENS asserted only in the 2-arg ctor.
        let _ = TokenBucket::new(1_000_000, MAX_TOKEN_BUCKET_TOKENS + 1);
    }

    #[test]
    fn with_state_skips_max_tokens_assert() {
        // The full constructor performs no limit check (C++ parity).
        let bucket = TokenBucket::with_state(1, MAX_TOKEN_BUCKET_TOKENS + 1, 1, 0, t(0, 0));
        assert_eq!(bucket.get_max_tokens(), MAX_TOKEN_BUCKET_TOKENS + 1);
    }
}
