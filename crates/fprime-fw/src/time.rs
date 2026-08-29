//! Time types.
//!
//! Port of `Fw/Time/Time.{hpp,cpp}` and `Fw/Time/TimeInterval.{hpp,cpp}`;
//! see `docs/cpp-analysis/fw-services.md`. `Time` is the 11-byte
//! `[timeBase u16][context u8][seconds u32][useconds u32]` value embedded in
//! every event and telemetry sample; `TimeInterval` is the 8-byte
//! `[seconds u32][useconds u32]` pair with a commutative absolute `sub`.

use crate::enums::fpp_enum;
use crate::serial::{Deserialize, Endianness, SerBuf, SerBufAny, Serialize, SerializeStatus};
use crate::{fw_assert, fw_try};
use fprime_config::{FwTimeBaseStoreType, FwTimeContextStoreType};

fpp_enum! {
    /// Time base (`TimeBase`, repr `FwTimeBaseStoreType` = u16).
    pub enum TimeBase : u16 { serialize_u16, deserialize_u16 } {
        /// No time base has been established.
        TbNone = 0,
        /// Indicates time is processor cycle time (not related to an epoch).
        TbProcTime = 1,
        /// Time as reported on a workstation where the software is running.
        TbWorkstationTime = 2,
        /// Time on the spacecraft.
        TbScTime = 3,
        /// Don't care value for sequences.
        TbDontCare = 0xFFFF,
    }
    default TbNone
}

/// Result of comparing two times (`Fw::TimeComparison`).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeComparison {
    /// Less than.
    Lt = -1,
    /// Equal.
    Eq = 0,
    /// Greater than.
    Gt = 1,
    /// Incomparable (differing time bases).
    Incomparable = 2,
}

/// A time point (`Fw::Time`): time base + context + seconds + microseconds.
///
/// The microseconds field is an invariant `< 1_000_000` — constructors and
/// setters `fw_assert!` it, and deserialization rejects violations with
/// `DeserFormatError`, leaving `self` unmodified.
#[derive(Debug, Clone, Copy)]
pub struct Time {
    time_base: TimeBase,
    time_context: FwTimeContextStoreType,
    seconds: u32,
    useconds: u32,
}

impl Time {
    /// On-wire size: `[base u16][context u8][sec u32][usec u32]` = 11 bytes.
    pub const SERIALIZED_SIZE: usize =
        size_of::<FwTimeBaseStoreType>() + size_of::<FwTimeContextStoreType>() + 4 + 4;

    /// The zero time (`Fw::ZERO_TIME`): TB_NONE, context 0, 0.000000 s.
    pub const ZERO: Time = Time {
        time_base: TimeBase::TbNone,
        time_context: 0,
        seconds: 0,
        useconds: 0,
    };

    /// Full constructor; asserts `useconds < 1_000_000` (C++ `set`).
    pub fn new(
        time_base: TimeBase,
        time_context: FwTimeContextStoreType,
        seconds: u32,
        useconds: u32,
    ) -> Self {
        fw_assert!(useconds < 1_000_000, useconds);
        Self {
            time_base,
            time_context,
            seconds,
            useconds,
        }
    }

    /// The C++ `Time(seconds, useconds)` constructor: NOTE this forces
    /// TB_NONE / context 0 — unlike [`Time::set`], which preserves both.
    pub fn from_seconds_useconds(seconds: u32, useconds: u32) -> Self {
        Self::new(TimeBase::TbNone, 0, seconds, useconds)
    }

    /// Zero time on the given base (`Time::zero(timeBase)`).
    pub fn zero(time_base: TimeBase) -> Self {
        Self::new(time_base, 0, 0, 0)
    }

    /// Set seconds/useconds, preserving the base and context (C++ 2-arg
    /// `set` — the counterpart of the base-forcing 2-arg constructor).
    pub fn set(&mut self, seconds: u32, useconds: u32) {
        fw_assert!(useconds < 1_000_000, useconds);
        self.seconds = seconds;
        self.useconds = useconds;
    }

    /// Set the time base.
    pub fn set_time_base(&mut self, time_base: TimeBase) {
        self.time_base = time_base;
    }

    /// Set the time context.
    pub fn set_time_context(&mut self, context: FwTimeContextStoreType) {
        self.time_context = context;
    }

    /// Seconds portion.
    pub fn get_seconds(&self) -> u32 {
        self.seconds
    }

    /// Microseconds portion (always `< 1_000_000`).
    pub fn get_useconds(&self) -> u32 {
        self.useconds
    }

    /// Time base.
    pub fn get_time_base(&self) -> TimeBase {
        self.time_base
    }

    /// Time context.
    pub fn get_context(&self) -> FwTimeContextStoreType {
        self.time_context
    }

    /// Compare two times. C++ parity: differing time bases are
    /// [`TimeComparison::Incomparable`]; the context is IGNORED entirely.
    pub fn compare(time1: &Time, time2: &Time) -> TimeComparison {
        if time1.time_base != time2.time_base {
            return TimeComparison::Incomparable;
        }
        match (time1.seconds, time1.useconds).cmp(&(time2.seconds, time2.useconds)) {
            std::cmp::Ordering::Less => TimeComparison::Lt,
            std::cmp::Ordering::Greater => TimeComparison::Gt,
            std::cmp::Ordering::Equal => TimeComparison::Eq,
        }
    }

    /// Add two times. C++ parity: `fw_assert!` (crash, not error) on
    /// mismatched time bases; microsecond carry at 1e6; the result context
    /// is `a`'s, or 0 when the contexts differ.
    pub fn add(a: &Time, b: &Time) -> Time {
        fw_assert!(
            a.time_base == b.time_base,
            a.time_base as i32,
            b.time_base as i32
        );
        let mut seconds = a.seconds + b.seconds;
        let mut useconds = a.useconds + b.useconds;
        fw_assert!(useconds < 1_999_999);
        if useconds >= 1_000_000 {
            seconds += 1;
            useconds -= 1_000_000;
        }
        let context = if a.time_context == b.time_context {
            a.time_context
        } else {
            0
        };
        Time::new(a.time_base, context, seconds, useconds)
    }

    /// Subtract `subtrahend` from `minuend`. C++ parity: `fw_assert!` on
    /// mismatched time bases AND on `minuend < subtrahend` (it does not
    /// return errors); microsecond borrow at 1e6; result context is the
    /// minuend's, or 0 when the contexts differ.
    pub fn sub(minuend: &Time, subtrahend: &Time) -> Time {
        fw_assert!(
            minuend.time_base == subtrahend.time_base,
            minuend.time_base as i32,
            subtrahend.time_base as i32
        );
        fw_assert!(Time::compare(minuend, subtrahend) != TimeComparison::Lt);
        let mut seconds = minuend.seconds - subtrahend.seconds;
        let useconds = if subtrahend.useconds > minuend.useconds {
            seconds -= 1;
            minuend.useconds + 1_000_000 - subtrahend.useconds
        } else {
            minuend.useconds - subtrahend.useconds
        };
        let context = if minuend.time_context == subtrahend.time_context {
            minuend.time_context
        } else {
            0
        };
        Time::new(minuend.time_base, context, seconds, useconds)
    }

    /// Add seconds/useconds in place, with carry (C++ member `add(U32, U32)`).
    pub fn add_duration(&mut self, seconds: u32, useconds: u32) {
        let mut new_seconds = self.seconds + seconds;
        let mut new_useconds = self.useconds + useconds;
        fw_assert!(new_useconds < 1_999_999, new_useconds);
        if new_useconds >= 1_000_000 {
            new_seconds += 1;
            new_useconds -= 1_000_000;
        }
        self.set(new_seconds, new_useconds);
    }

    /// Add fractional seconds in place (C++ member `add(F64)`), using the
    /// round-to-nearest-microsecond parse.
    pub fn add_f64(&mut self, seconds: f64) {
        self.add_duration(Self::parse_seconds(seconds), Self::parse_useconds(seconds));
    }

    /// Build a time from fractional seconds (the C++ `Time(F64)` /
    /// `set(F64)` path on a default time: TB_NONE, context 0), rounding to
    /// the nearest microsecond with carry into seconds when the fraction
    /// rounds up to a whole second.
    pub fn from_f64(seconds: f64) -> Time {
        let mut parsed_seconds = Self::parse_seconds(seconds);
        let mut parsed_useconds = Self::parse_useconds(seconds);
        // parseUSeconds rounds up to a whole second at .9999995+; carry it
        if parsed_useconds >= 1_000_000 {
            parsed_seconds += 1;
            parsed_useconds -= 1_000_000;
        }
        Time::new(TimeBase::TbNone, 0, parsed_seconds, parsed_useconds)
    }

    /// Fractional-second view (C++ `operator F64`).
    pub fn to_f64(&self) -> f64 {
        f64::from(self.seconds) + f64::from(self.useconds) / 1_000_000.0
    }

    // C++ parseSeconds: truncation toward zero; asserts non-negative.
    fn parse_seconds(seconds: f64) -> u32 {
        fw_assert!(seconds >= 0.0);
        seconds as u32
    }

    // C++ parseUSeconds: fractional part * 1e6 + 0.5 (round to nearest;
    // 0.9999995+ parses as a full second — callers carry it).
    fn parse_useconds(seconds: f64) -> u32 {
        fw_assert!(seconds >= 0.0);
        let fractional = seconds - f64::from(seconds as u32);
        (fractional * 1_000_000.0 + 0.5) as u32
    }
}

impl Default for Time {
    /// TB_NONE, context 0, 0.000000 s.
    fn default() -> Self {
        Self::ZERO
    }
}

/// C++ parity: equality via [`Time::compare`] — the context is ignored, and
/// times on different bases are never equal.
impl PartialEq for Time {
    fn eq(&self, other: &Self) -> bool {
        Time::compare(self, other) == TimeComparison::Eq
    }
}

/// Ordering via [`Time::compare`]; `None` for incomparable (differing bases).
impl PartialOrd for Time {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        match Time::compare(self, other) {
            TimeComparison::Lt => Some(std::cmp::Ordering::Less),
            TimeComparison::Eq => Some(std::cmp::Ordering::Equal),
            TimeComparison::Gt => Some(std::cmp::Ordering::Greater),
            TimeComparison::Incomparable => None,
        }
    }
}

impl Serialize for Time {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        fw_try!(self.time_base.serialize_to(buf, e));
        fw_try!(buf.serialize_u8(self.time_context, e));
        fw_try!(buf.serialize_u32(self.seconds, e));
        buf.serialize_u32(self.useconds, e)
    }
    fn serialized_size(&self) -> usize {
        Self::SERIALIZED_SIZE
    }
}

impl Deserialize for Time {
    /// C++ parity: deserializes into a temporary and rejects
    /// `useconds >= 1_000_000` (and an undeclared time base) with
    /// `DeserFormatError`, leaving `self` unmodified. The cursor stays
    /// advanced past the consumed fields, as in C++.
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let mut base = TimeBase::TbNone;
        let mut context: FwTimeContextStoreType = 0;
        let mut seconds = 0u32;
        let mut useconds = 0u32;
        fw_try!(base.deserialize_from(buf, e));
        fw_try!(buf.deserialize_u8(&mut context, e));
        fw_try!(buf.deserialize_u32(&mut seconds, e));
        fw_try!(buf.deserialize_u32(&mut useconds, e));
        if useconds >= 1_000_000 {
            return SerializeStatus::DeserFormatError;
        }
        self.time_base = base;
        self.time_context = context;
        self.seconds = seconds;
        self.useconds = useconds;
        SerializeStatus::Ok
    }
}

/// A time span (`Fw::TimeInterval`): seconds + microseconds, no base or
/// context. `sub` is a COMMUTATIVE absolute difference — unlike
/// [`Time::sub`], which asserts ordering.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TimeInterval {
    seconds: u32,
    useconds: u32,
}

impl TimeInterval {
    /// On-wire size: `[seconds u32][useconds u32]` = 8 bytes.
    pub const SERIALIZED_SIZE: usize = 8;

    /// Constructor; asserts `useconds < 1_000_000` (C++ `set`).
    pub fn new(seconds: u32, useconds: u32) -> Self {
        fw_assert!(useconds < 1_000_000, useconds);
        Self { seconds, useconds }
    }

    /// The interval between two times as an absolute difference of their
    /// (seconds, useconds) pairs — the C++ `TimeInterval(start, end)` ctor;
    /// bases and contexts are ignored.
    pub fn between(start: &Time, end: &Time) -> Self {
        TimeInterval::sub(
            &TimeInterval::new(end.get_seconds(), end.get_useconds()),
            &TimeInterval::new(start.get_seconds(), start.get_useconds()),
        )
    }

    /// Set the values; asserts `useconds < 1_000_000`.
    pub fn set(&mut self, seconds: u32, useconds: u32) {
        fw_assert!(useconds < 1_000_000, useconds);
        self.seconds = seconds;
        self.useconds = useconds;
    }

    /// Seconds portion.
    pub fn get_seconds(&self) -> u32 {
        self.seconds
    }

    /// Microseconds portion.
    pub fn get_useconds(&self) -> u32 {
        self.useconds
    }

    /// Lexicographic comparison of (seconds, useconds);
    /// [`TimeComparison::Incomparable`] is never returned.
    pub fn compare(t1: &TimeInterval, t2: &TimeInterval) -> TimeComparison {
        match (t1.seconds, t1.useconds).cmp(&(t2.seconds, t2.useconds)) {
            std::cmp::Ordering::Less => TimeComparison::Lt,
            std::cmp::Ordering::Greater => TimeComparison::Gt,
            std::cmp::Ordering::Equal => TimeComparison::Eq,
        }
    }

    /// Add two intervals with microsecond carry (asserts like `Time::add`).
    pub fn add(a: &TimeInterval, b: &TimeInterval) -> TimeInterval {
        let mut seconds = a.seconds + b.seconds;
        let mut useconds = a.useconds + b.useconds;
        fw_assert!(useconds < 1_999_999);
        if useconds >= 1_000_000 {
            seconds += 1;
            useconds -= 1_000_000;
        }
        TimeInterval::new(seconds, useconds)
    }

    /// COMMUTATIVE absolute difference (C++ parity: the operands are ordered
    /// so the larger becomes the minuend).
    pub fn sub(t1: &TimeInterval, t2: &TimeInterval) -> TimeInterval {
        let (minuend, subtrahend) = if TimeInterval::compare(t1, t2) == TimeComparison::Lt {
            (t2, t1)
        } else {
            (t1, t2)
        };
        let mut seconds = minuend.seconds - subtrahend.seconds;
        let useconds = if subtrahend.useconds > minuend.useconds {
            seconds -= 1;
            minuend.useconds + 1_000_000 - subtrahend.useconds
        } else {
            minuend.useconds - subtrahend.useconds
        };
        TimeInterval::new(seconds, useconds)
    }
}

impl PartialOrd for TimeInterval {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for TimeInterval {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (self.seconds, self.useconds).cmp(&(other.seconds, other.useconds))
    }
}

impl Serialize for TimeInterval {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        fw_try!(buf.serialize_u32(self.seconds, e));
        buf.serialize_u32(self.useconds, e)
    }
    fn serialized_size(&self) -> usize {
        Self::SERIALIZED_SIZE
    }
}

impl Deserialize for TimeInterval {
    /// C++ parity: NO validation of the microseconds field on deserialize
    /// (delegates to the plain autocoded struct) — unlike [`Time`].
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let mut seconds = 0u32;
        let mut useconds = 0u32;
        fw_try!(buf.deserialize_u32(&mut seconds, e));
        fw_try!(buf.deserialize_u32(&mut useconds, e));
        self.seconds = seconds;
        self.useconds = useconds;
        SerializeStatus::Ok
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::LinearBuffer;

    #[test]
    fn time_wire_format_is_11_bytes() {
        let t = Time::new(TimeBase::TbWorkstationTime, 3, 0x0102_0304, 999_999);
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(
            t.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(
            buf.as_slice(),
            &[
                0x00, 0x02, // TB_WORKSTATION_TIME
                0x03, // context
                0x01, 0x02, 0x03, 0x04, // seconds
                0x00, 0x0F, 0x42, 0x3F, // useconds = 999999
            ]
        );
        assert_eq!(Time::SERIALIZED_SIZE, 11);
        assert_eq!(t.serialized_size(), 11);
    }

    #[test]
    fn time_dont_care_base_roundtrips() {
        let t = Time::new(TimeBase::TbDontCare, 0xFF, 1, 2);
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(
            t.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(&buf.as_slice()[..2], &[0xFF, 0xFF]);
        let mut out = Time::default();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(out.get_time_base(), TimeBase::TbDontCare);
        assert_eq!(out.get_context(), 0xFF);
    }

    #[test]
    fn time_deserialize_rejects_bad_useconds_leaving_self_unmodified() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(buf.serialize_u16_be(0), SerializeStatus::Ok); // TB_NONE
        assert_eq!(buf.serialize_u8_be(0), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u32_be(5), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u32_be(1_000_000), SerializeStatus::Ok); // invalid
        let mut t = Time::new(TimeBase::TbScTime, 7, 42, 43);
        assert_eq!(
            t.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserFormatError
        );
        assert_eq!(t.get_time_base(), TimeBase::TbScTime);
        assert_eq!(t.get_context(), 7);
        assert_eq!(t.get_seconds(), 42);
        assert_eq!(t.get_useconds(), 43);
        // cursor stays advanced past the consumed 11 bytes (C++ parity)
        assert_eq!(buf.deser_loc(), 11);
    }

    #[test]
    fn time_deserialize_rejects_undeclared_time_base() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(buf.serialize_u16_be(5), SerializeStatus::Ok); // not a TimeBase
        assert_eq!(buf.serialize_u8_be(0), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u32_be(0), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u32_be(0), SerializeStatus::Ok);
        let mut t = Time::default();
        assert_eq!(
            t.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserFormatError
        );
    }

    #[test]
    fn compare_ignores_context_and_flags_incomparable_bases() {
        let a = Time::new(TimeBase::TbProcTime, 1, 100, 5);
        let b = Time::new(TimeBase::TbProcTime, 9, 100, 5);
        assert_eq!(Time::compare(&a, &b), TimeComparison::Eq);
        assert!(a == b, "context must be ignored by equality");

        let c = Time::new(TimeBase::TbScTime, 1, 100, 5);
        assert_eq!(Time::compare(&a, &c), TimeComparison::Incomparable);
        assert!(a != c);
        assert_eq!(a.partial_cmp(&c), None);

        let lt = Time::new(TimeBase::TbProcTime, 0, 100, 4);
        assert_eq!(Time::compare(&lt, &a), TimeComparison::Lt);
        assert_eq!(Time::compare(&a, &lt), TimeComparison::Gt);
        let lt_sec = Time::new(TimeBase::TbProcTime, 0, 99, 999_999);
        assert_eq!(Time::compare(&lt_sec, &a), TimeComparison::Lt);
    }

    #[test]
    fn add_carries_useconds_and_zeroes_mismatched_context() {
        let a = Time::new(TimeBase::TbProcTime, 5, 1, 999_999);
        let b = Time::new(TimeBase::TbProcTime, 5, 2, 2);
        let sum = Time::add(&a, &b);
        assert_eq!(sum.get_seconds(), 4);
        assert_eq!(sum.get_useconds(), 1);
        assert_eq!(sum.get_context(), 5);

        let c = Time::new(TimeBase::TbProcTime, 6, 0, 0);
        assert_eq!(Time::add(&a, &c).get_context(), 0, "context mismatch -> 0");
    }

    #[test]
    fn sub_borrows_useconds() {
        let minuend = Time::new(TimeBase::TbProcTime, 1, 10, 1);
        let subtrahend = Time::new(TimeBase::TbProcTime, 1, 9, 2);
        let diff = Time::sub(&minuend, &subtrahend);
        assert_eq!(diff.get_seconds(), 0);
        assert_eq!(diff.get_useconds(), 999_999);
        assert_eq!(diff.get_context(), 1);
    }

    #[test]
    #[should_panic]
    fn add_asserts_on_base_mismatch() {
        let a = Time::new(TimeBase::TbProcTime, 0, 1, 0);
        let b = Time::new(TimeBase::TbScTime, 0, 1, 0);
        let _ = Time::add(&a, &b);
    }

    #[test]
    #[should_panic]
    fn sub_asserts_when_minuend_smaller() {
        let a = Time::new(TimeBase::TbProcTime, 0, 1, 0);
        let b = Time::new(TimeBase::TbProcTime, 0, 2, 0);
        let _ = Time::sub(&a, &b);
    }

    #[test]
    #[should_panic]
    fn new_asserts_on_useconds_range() {
        let _ = Time::new(TimeBase::TbNone, 0, 0, 1_000_000);
    }

    #[test]
    fn two_arg_constructor_forces_tb_none_but_set_preserves() {
        let t = Time::from_seconds_useconds(3, 4);
        assert_eq!(t.get_time_base(), TimeBase::TbNone);
        assert_eq!(t.get_context(), 0);

        let mut t2 = Time::new(TimeBase::TbScTime, 9, 1, 1);
        t2.set(5, 6);
        assert_eq!(t2.get_time_base(), TimeBase::TbScTime, "set preserves base");
        assert_eq!(t2.get_context(), 9, "set preserves context");
        assert_eq!(t2.get_seconds(), 5);
    }

    #[test]
    fn f64_round_to_nearest_usec_with_carry() {
        // 999999.6 usec rounds to 1000000 -> carries into seconds
        let t = Time::from_f64(1.999_999_6);
        assert_eq!(t.get_seconds(), 2);
        assert_eq!(t.get_useconds(), 0);

        let t = Time::from_f64(1.25);
        assert_eq!(t.get_seconds(), 1);
        assert_eq!(t.get_useconds(), 250_000);

        // round-half-up at the microsecond
        let t = Time::from_f64(0.000_000_5);
        assert_eq!(t.get_useconds(), 1);

        assert!((Time::new(TimeBase::TbNone, 0, 1, 250_000).to_f64() - 1.25).abs() < 1e-9);
    }

    #[test]
    fn add_duration_carries() {
        let mut t = Time::new(TimeBase::TbProcTime, 2, 1, 999_999);
        t.add_duration(0, 1);
        assert_eq!(t.get_seconds(), 2);
        assert_eq!(t.get_useconds(), 0);
        assert_eq!(t.get_time_base(), TimeBase::TbProcTime);
        assert_eq!(t.get_context(), 2);
    }

    #[test]
    fn time_interval_wire_format_is_8_bytes() {
        let ti = TimeInterval::new(0x0102_0304, 0x0005_0607);
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(
            ti.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(
            buf.as_slice(),
            &[0x01, 0x02, 0x03, 0x04, 0x00, 0x05, 0x06, 0x07]
        );
        assert_eq!(TimeInterval::SERIALIZED_SIZE, 8);
    }

    #[test]
    fn time_interval_sub_is_commutative_absolute() {
        let a = TimeInterval::new(10, 1);
        let b = TimeInterval::new(9, 2);
        let d1 = TimeInterval::sub(&a, &b);
        let d2 = TimeInterval::sub(&b, &a);
        assert_eq!(d1, d2);
        assert_eq!(d1.get_seconds(), 0);
        assert_eq!(d1.get_useconds(), 999_999);
    }

    #[test]
    fn time_interval_between_times() {
        let start = Time::new(TimeBase::TbProcTime, 0, 5, 500_000);
        let end = Time::new(TimeBase::TbProcTime, 0, 7, 250_000);
        let ti = TimeInterval::between(&start, &end);
        assert_eq!(ti.get_seconds(), 1);
        assert_eq!(ti.get_useconds(), 750_000);
        // commutative: reversed operands give the same interval
        assert_eq!(TimeInterval::between(&end, &start), ti);
    }

    #[test]
    fn time_interval_deserialize_skips_usec_validation() {
        // C++ parity: TimeInterval deserialization does NOT validate usec
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(buf.serialize_u32_be(1), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u32_be(2_000_000), SerializeStatus::Ok);
        let mut ti = TimeInterval::default();
        assert_eq!(
            ti.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(ti.get_useconds(), 2_000_000);
    }

    #[test]
    fn zero_time_constant() {
        assert_eq!(Time::ZERO.get_time_base(), TimeBase::TbNone);
        assert_eq!(Time::ZERO, Time::default());
        assert_eq!(Time::zero(TimeBase::TbScTime).get_seconds(), 0);
    }
}
