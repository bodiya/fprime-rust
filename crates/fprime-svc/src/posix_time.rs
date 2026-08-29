//! # PosixTime — port of `Svc::PosixTime` (passive time source)
//!
//! C++ sources: `Svc/PosixTime/PosixTime.{cpp,hpp,fpp}`.
//! Analysis: `docs/cpp-analysis/svc-core.md` (shared framework contracts) /
//! ARCHITECTURE.md ("PosixTime-equivalent time source").
//!
//! A passive component with a single SYNC `Fw.Time` input port that fills
//! the `Fw::Time` from the system realtime clock (`SystemTime`, the
//! CLOCK_REALTIME equivalent) as `TB_WORKSTATION_TIME` with a settable
//! context (default 0), seconds and microseconds. A clock read failure
//! (time before the epoch) reports zero time, matching the C++
//! `clock_gettime` failure path.

use fprime_comp::{PassiveBase, PortRef, TimePort};
use fprime_config::FwIndexType;
use fprime_fw::{Time, TimeBase};
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

/// `Svc::PosixTime`.
pub struct PosixTime {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `m_timeContext` — stamped into every returned `Time` (relaxed
    /// atomic; set at init, read from any caller thread).
    time_context: AtomicU8,
}

impl PosixTime {
    /// Construct with time context 0 (C++ constructor).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            time_context: AtomicU8::new(0),
        })
    }

    /// C++ `setTimeContext`.
    pub fn set_time_context(&self, time_context: u8) {
        self.time_context.store(time_context, Ordering::Relaxed);
    }

    /// `timeGetPort` — SYNC `Fw.Time` input (factory for topology wiring).
    pub fn time_get_port_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn TimePort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `timeGetPort_handler`: fill `time` from the realtime clock.
    fn time_get_port_handler(&self, _port_num: FwIndexType, time: &mut Time) {
        // C++ parity: report zero rather than garbage when the clock read
        // fails (SystemTime before the epoch).
        let (seconds, useconds) = match SystemTime::now().duration_since(UNIX_EPOCH) {
            Ok(duration) => (duration.as_secs() as u32, duration.subsec_micros()),
            Err(_) => (0, 0),
        };
        time.set_time_base(TimeBase::TbWorkstationTime);
        time.set_time_context(self.time_context.load(Ordering::Relaxed));
        time.set(seconds, useconds);
    }
}

impl TimePort for PosixTime {
    fn invoke(&self, port_num: FwIndexType, time: &mut Time) {
        self.time_get_port_handler(port_num, time);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fills_workstation_time_with_context_zero() {
        let comp = PosixTime::new("posixTime");
        let port = comp.time_get_port_in(0);
        let mut time = Time::default();
        port.target.invoke(port.port_num, &mut time);
        assert_eq!(time.get_time_base(), TimeBase::TbWorkstationTime);
        assert_eq!(time.get_context(), 0);
        // A real wall clock: seconds are nonzero and useconds in range.
        assert!(time.get_seconds() > 0);
        assert!(time.get_useconds() < 1_000_000);
    }

    #[test]
    fn time_context_is_settable() {
        let comp = PosixTime::new("posixTime");
        comp.set_time_context(42);
        let port = comp.time_get_port_in(0);
        let mut time = Time::default();
        port.target.invoke(port.port_num, &mut time);
        assert_eq!(time.get_context(), 42);
    }

    #[test]
    fn successive_reads_are_monotonic_non_decreasing() {
        let comp = PosixTime::new("posixTime");
        let port = comp.time_get_port_in(0);
        let mut t1 = Time::default();
        let mut t2 = Time::default();
        port.target.invoke(port.port_num, &mut t1);
        port.target.invoke(port.port_num, &mut t2);
        // Realtime clock could in principle step backwards, but not in a
        // test process's lifetime under normal conditions.
        assert!(t2 >= t1);
    }
}
