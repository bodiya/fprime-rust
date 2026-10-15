//! # ChronoTime — port of `Svc::ChronoTime` (passive time source)
//!
//! C++ sources: `Svc/ChronoTime/ChronoTime.{cpp,hpp,fpp}`,
//! `Svc/ChronoTime/docs/sdd.md`.
//!
//! Implements `Svc.Time` (`timeGetPort`) from the language's wall clock:
//! upstream reads `std::chrono::system_clock` (the Unix epoch in practice)
//! and sets the time to `TB_WORKSTATION_TIME`, whole seconds since the
//! epoch truncated to `U32`, and the microseconds within the second. The
//! caller's time context is left as it was (`Fw::Time::set(timeBase,
//! seconds, useconds)` keeps the context; contrast `PosixTime`, which
//! stamps its own).
//!
//! The clock here is `Os::RawTime::now`: with `std` that is
//! `SystemTime`, the `system_clock` equivalent; without `std` it is the
//! registered `fprime_os::platform::Platform::raw_time_now`, so the
//! component is available on bare metal like upstream's (which needs only
//! C++11, no POSIX). Deviation: `std::chrono` has no failure path, but a
//! raw-time read can fail (a pre-epoch clock, or no platform registered);
//! then the seconds and microseconds are reported as zero (the time base
//! is still set), the `PosixTime` convention.

use fprime_comp::{PassiveBase, PortRef, TimePort};
use fprime_config::FwIndexType;
use fprime_fw::sync::Arc;
use fprime_fw::{Time, TimeBase};
use fprime_os::{RawTime, rawtime::Status as RawTimeStatus};

/// `Svc::ChronoTime`.
pub struct ChronoTime {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
}

impl ChronoTime {
    /// Construct (C++ constructor).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
        })
    }

    /// `timeGetPort` — SYNC `Fw.Time` input.
    pub fn time_get_port_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn TimePort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `timeGetPort_handler`.
    fn time_get_port_handler(&self, _port_num: FwIndexType, time: &mut Time) {
        let mut now = RawTime::new();
        let (seconds, useconds) = if now.now() == RawTimeStatus::OpOk {
            // C++ parity: duration_cast<seconds> cast to U32 (truncating),
            // microseconds modulo 1e6.
            (now.get_seconds() as u32, now.get_nanoseconds() / 1000)
        } else {
            (0, 0)
        };
        time.set_time_base(TimeBase::TbWorkstationTime);
        time.set(seconds, useconds);
    }
}

impl TimePort for ChronoTime {
    fn invoke(&self, port_num: FwIndexType, time: &mut Time) {
        self.time_get_port_handler(port_num, time);
    }
}

#[cfg(test)]
mod tests {
    //! Port of `Svc/ChronoTime/test/ut` (`ChronoTimeTester::test_basic_time`).
    use super::*;

    /// Seconds since the epoch on 2024-11-25, when upstream's test was
    /// written.
    const SECONDS_SINCE_EPOCH_2024_11_25: u32 = 1_732_492_800;

    #[test]
    fn basic_time() {
        // SVC_CHRONO_TIME_002, SVC_CHRONO_TIME_003
        crate::test_platform::install();
        let comp = ChronoTime::new("ChronoTime");
        let port = comp.time_get_port_in(0);
        let mut time = Time::default();
        port.target.invoke(port.port_num, &mut time);
        assert!(time.get_seconds() > SECONDS_SINCE_EPOCH_2024_11_25);
        assert!(time.get_useconds() <= 999_999);
        assert_eq!(time.get_time_base(), TimeBase::TbWorkstationTime);
    }

    #[test]
    fn the_callers_context_is_kept() {
        crate::test_platform::install();
        let comp = ChronoTime::new("ChronoTime");
        let port = comp.time_get_port_in(0);
        let mut time = Time::new(TimeBase::TbNone, 9, 1, 2);
        port.target.invoke(port.port_num, &mut time);
        assert_eq!(time.get_context(), 9);
        assert_eq!(time.get_time_base(), TimeBase::TbWorkstationTime);
        assert!(time.get_seconds() > SECONDS_SINCE_EPOCH_2024_11_25);
    }

    #[test]
    fn successive_reads_do_not_go_backwards() {
        crate::test_platform::install();
        let comp = ChronoTime::new("ChronoTime");
        let port = comp.time_get_port_in(0);
        let mut t1 = Time::default();
        let mut t2 = Time::default();
        port.target.invoke(port.port_num, &mut t1);
        port.target.invoke(port.port_num, &mut t2);
        assert!(t2 >= t1);
    }
}
