//! The F Prime raw time facility.
//!
//! Port of `Os::RawTime` (Os/RawTimeInterface.hpp, Os/Posix/RawTime.cpp)
//! over `std::time::SystemTime` (CLOCK_REALTIME semantics — comparable
//! across processes, unlike `Instant`) — see `docs/cpp-analysis/os.md`.
//!
//! Semantics kept:
//! - [`RawTime::get_time_interval`] is the ABSOLUTE difference
//!   (commutative, always non-negative);
//! - [`RawTime::get_diff_usec`] saturates: `u32::MAX` with
//!   [`Status::OpOverflow`] beyond ~71.6 minutes;
//! - equality means the interval computes to exactly 0 seconds 0
//!   microseconds (two samples within the same microsecond compare equal);
//! - the wire format is exactly 8 bytes: `[u32 seconds][u32 nanoseconds]`,
//!   big-endian by default (`FW_RAW_TIME_SERIALIZATION_MAX_SIZE = 8`), with
//!   the seconds truncated-cast from the full internal value (C++ parity:
//!   `static_cast<U32>(tv_sec)`).

use fprime_fw::{
    Deserialize, Endianness, SerBuf, SerBufAny, Serialize, SerializeStatus, TimeInterval,
};

/// C++ `FW_RAW_TIME_SERIALIZATION_MAX_SIZE` (PlatformCfg.fpp).
pub const FW_RAW_TIME_SERIALIZATION_MAX_SIZE: usize = 8;

/// Port of `Os::RawTimeInterface::Status` (Os/RawTimeInterface.hpp) —
/// exact C++ discriminants.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Operation was successful.
    OpOk = 0,
    /// Operation result caused an overflow.
    OpOverflow = 1,
    /// Parameters invalid for current platform.
    InvalidParams = 2,
    /// RawTime does not support operation.
    NotSupported = 3,
    /// All other errors.
    OtherError = 4,
}

/// The F Prime raw time sample (see module docs). Internally holds the full
/// seconds value (like the C++ `timespec`); serialization truncates to u32.
#[derive(Debug, Clone, Copy, Default)]
pub struct RawTime {
    seconds: u64,
    nanoseconds: u32,
}

impl RawTime {
    /// Serialized size in bytes.
    pub const SERIALIZED_SIZE: usize = FW_RAW_TIME_SERIALIZATION_MAX_SIZE;

    /// Construct a zero raw time.
    pub fn new() -> Self {
        Self::default()
    }

    /// Construct from explicit parts (useful for tests and replay).
    pub fn from_parts(seconds: u64, nanoseconds: u32) -> Self {
        Self {
            seconds,
            nanoseconds,
        }
    }

    /// Seconds since the epoch.
    pub fn get_seconds(&self) -> u64 {
        self.seconds
    }

    /// Nanoseconds within the second.
    pub fn get_nanoseconds(&self) -> u32 {
        self.nanoseconds
    }

    /// Sample the wall clock (port of `now`; `clock_gettime(CLOCK_REALTIME)`
    /// semantics via `SystemTime`).
    pub fn now(&mut self) -> Status {
        match std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
            Ok(duration) => {
                self.seconds = duration.as_secs();
                self.nanoseconds = duration.subsec_nanos();
                Status::OpOk
            }
            // A pre-epoch clock: nothing in the C++ table fits better.
            Err(_) => Status::OtherError,
        }
    }

    /// Compute the ABSOLUTE time difference to `other` (port of
    /// `getTimeInterval`, Os/Posix/RawTime.cpp): commutative, always
    /// non-negative; nanoseconds truncate to whole microseconds; the
    /// seconds difference is truncated-cast to u32 (C++ parity).
    pub fn get_time_interval(&self, other: &RawTime, interval: &mut TimeInterval) -> Status {
        // Guarantee t1 is the later time.
        let (t1, t2) = if (self.seconds, self.nanoseconds) < (other.seconds, other.nanoseconds) {
            (other, self)
        } else {
            (self, other)
        };
        // C++ parity: the seconds difference is cast to U32 first, then the
        // nanosecond borrow is subtracted in u32 arithmetic.
        let mut seconds = (t1.seconds - t2.seconds) as u32;
        let nanoseconds = if t1.nanoseconds < t2.nanoseconds {
            seconds = seconds.wrapping_sub(1);
            t1.nanoseconds + (1_000_000_000 - t2.nanoseconds)
        } else {
            t1.nanoseconds - t2.nanoseconds
        };
        interval.set(seconds, nanoseconds / 1000);
        Status::OpOk
    }

    /// Difference to `other` in microseconds (port of the default
    /// `getDiffUsec`, Os/DelegateRawTime.cpp): saturates at `u32::MAX` with
    /// [`Status::OpOverflow`] (~71.6 minutes measurable).
    pub fn get_diff_usec(&self, other: &RawTime, result: &mut u32) -> Status {
        let mut interval = TimeInterval::new(0, 0);
        let status = self.get_time_interval(other, &mut interval);
        if status != Status::OpOk {
            return status;
        }
        let seconds = interval.get_seconds();
        let useconds = interval.get_useconds();
        if seconds > u32::MAX / 1_000_000 {
            *result = u32::MAX;
            return Status::OpOverflow;
        }
        let sec_to_usec = seconds * 1_000_000;
        if sec_to_usec > u32::MAX - useconds {
            *result = u32::MAX;
            return Status::OpOverflow;
        }
        *result = sec_to_usec + useconds;
        Status::OpOk
    }
}

/// C++ `RawTimeInterface::operator==`: equal when the interval computes to
/// exactly 0 seconds 0 microseconds (i.e. the same microsecond).
impl PartialEq for RawTime {
    fn eq(&self, other: &Self) -> bool {
        let mut interval = TimeInterval::new(0, 0);
        let _ = self.get_time_interval(other, &mut interval);
        interval.get_seconds() == 0 && interval.get_useconds() == 0
    }
}

impl Serialize for RawTime {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        // C++ parity (PosixRawTime::serializeTo): truncated u32 casts.
        let status = buf.serialize_u32(self.seconds as u32, e);
        if !status.is_ok() {
            return status;
        }
        buf.serialize_u32(self.nanoseconds, e)
    }

    fn serialized_size(&self) -> usize {
        Self::SERIALIZED_SIZE
    }
}

impl Deserialize for RawTime {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let mut seconds: u32 = 0;
        let mut nanoseconds: u32 = 0;
        let status = buf.deserialize_u32(&mut seconds, e);
        if !status.is_ok() {
            return status;
        }
        let status = buf.deserialize_u32(&mut nanoseconds, e);
        if !status.is_ok() {
            return status;
        }
        self.seconds = u64::from(seconds);
        self.nanoseconds = nanoseconds;
        SerializeStatus::Ok
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::LinearBuffer;

    #[test]
    fn serialization_wire_format_is_8_bytes_big_endian() {
        let raw = RawTime::from_parts(0x0102_0304, 0x0A0B_0C0D);
        let mut buffer: LinearBuffer<16> = LinearBuffer::new();
        assert!(raw.serialize_to(&mut buffer, Endianness::Big).is_ok());
        assert_eq!(raw.serialized_size(), 8);
        assert_eq!(
            buffer.as_slice(),
            &[0x01, 0x02, 0x03, 0x04, 0x0A, 0x0B, 0x0C, 0x0D]
        );

        let mut decoded = RawTime::new();
        assert!(
            decoded
                .deserialize_from(&mut buffer, Endianness::Big)
                .is_ok()
        );
        assert_eq!(decoded.get_seconds(), 0x0102_0304);
        assert_eq!(decoded.get_nanoseconds(), 0x0A0B_0C0D);
    }

    // Gotcha: seconds are truncated-cast to u32 on the wire (C++ parity).
    #[test]
    fn serialization_truncates_seconds_to_u32() {
        let raw = RawTime::from_parts(0x1_0000_0001, 5);
        let mut buffer: LinearBuffer<16> = LinearBuffer::new();
        assert!(raw.serialize_to(&mut buffer, Endianness::Big).is_ok());
        assert_eq!(
            buffer.as_slice(),
            &[0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x05]
        );
    }

    #[test]
    fn get_time_interval_is_commutative_absolute() {
        let earlier = RawTime::from_parts(100, 900_000_000);
        let later = RawTime::from_parts(103, 100_000_000);
        let mut forward = TimeInterval::new(0, 0);
        let mut backward = TimeInterval::new(0, 0);
        assert_eq!(
            later.get_time_interval(&earlier, &mut forward),
            Status::OpOk
        );
        assert_eq!(
            earlier.get_time_interval(&later, &mut backward),
            Status::OpOk
        );
        // 103.1 - 100.9 = 2.2 s, both directions.
        assert_eq!(forward.get_seconds(), 2);
        assert_eq!(forward.get_useconds(), 200_000);
        assert_eq!(backward, forward);
    }

    #[test]
    fn get_diff_usec_exact_and_overflow() {
        let base = RawTime::from_parts(1000, 250_000);
        let close = RawTime::from_parts(1002, 500_750_000);
        let mut result: u32 = 0;
        assert_eq!(close.get_diff_usec(&base, &mut result), Status::OpOk);
        // 2.5005 s = 2_500_500 us.
        assert_eq!(result, 2_500_500);

        // Beyond ~71.6 minutes: saturate with OP_OVERFLOW.
        let far = RawTime::from_parts(1000 + 5000, 250_000);
        assert_eq!(far.get_diff_usec(&base, &mut result), Status::OpOverflow);
        assert_eq!(result, u32::MAX);
    }

    #[test]
    fn equality_is_same_microsecond() {
        let a = RawTime::from_parts(50, 1_000);
        let b = RawTime::from_parts(50, 1_999); // same microsecond
        let c = RawTime::from_parts(50, 2_000); // next microsecond
        assert_eq!(a, b);
        assert!(a != c);
    }

    #[test]
    fn now_advances() {
        let mut first = RawTime::new();
        let mut second = RawTime::new();
        assert_eq!(first.now(), Status::OpOk);
        std::thread::sleep(std::time::Duration::from_millis(5));
        assert_eq!(second.now(), Status::OpOk);
        let mut result: u32 = 0;
        assert_eq!(second.get_diff_usec(&first, &mut result), Status::OpOk);
        assert!(result >= 5_000);
    }
}
