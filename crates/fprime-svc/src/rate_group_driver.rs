//! # RateGroupDriver — port of `Svc::RateGroupDriver` (passive, ISR-capable)
//!
//! C++ sources: `Svc/RateGroupDriver/RateGroupDriver.{cpp,hpp,fpp}`.
//! Analysis: `docs/cpp-analysis/svc-core.md` (RateGroupDriver section).
//!
//! Divides a primary tick (`Svc.Cycle` carrying `Os::RawTime`) across
//! [`RATE_GROUP_DRIVER_CYCLE_PORTS`] `CycleOut` ports via a divisor/offset
//! table. The handler may run in ISR context, so it takes **no locks**: the
//! divider table is immutable after [`RateGroupDriver::configure`]
//! (`OnceLock`) and the tick counter is a relaxed atomic (single-caller
//! assumption, C++ parity — the C++ member is a plain `FwSizeType`).
//!
//! No events, telemetry, or commands.

use fprime_comp::{CyclePort, OutputPort, PassiveBase, PortRef};
use fprime_config::{FwIndexType, FwSizeType, RATE_GROUP_DRIVER_CYCLE_PORTS};
use fprime_fw::fw_assert;
use fprime_os::RawTime;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, OnceLock};

/// One entry of the divider table (`RateGroupDriver::Divider`).
/// `divisor == 0` marks the entry unused (the default).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Divider {
    /// The output port fires when `ticks % divisor == offset`.
    pub divisor: FwSizeType,
    /// Phase offset within the divisor period.
    pub offset: FwSizeType,
}

impl Divider {
    /// C++ two-arg constructor.
    pub const fn new(divisor: FwSizeType, offset: FwSizeType) -> Self {
        Self { divisor, offset }
    }
}

/// Immutable post-configure state (set once, read lock-free afterwards).
struct DriverConfig {
    dividers: [Divider; RATE_GROUP_DRIVER_CYCLE_PORTS],
    /// Product of all nonzero divisors — the tick counter wraps here so
    /// integer rollover never jumps a cycle.
    rollover: FwSizeType,
}

/// `Svc::RateGroupDriver` — passive tick divider.
pub struct RateGroupDriver {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `CycleOut: [3] Svc.Cycle` output ports.
    pub cycle_out: [OutputPort<dyn CyclePort>; RATE_GROUP_DRIVER_CYCLE_PORTS],
    /// Divider table + rollover, written once by `configure`.
    config: OnceLock<DriverConfig>,
    /// `m_ticks` — starts at 0, so offset-0 groups fire on the very first
    /// tick. Relaxed atomic: the C++ field is a plain non-atomic member
    /// with a single-caller assumption.
    ticks: AtomicU64,
}

impl RateGroupDriver {
    /// Construct (C++ constructor: ticks 0, rollover 1, unconfigured).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            cycle_out: [const { OutputPort::new() }; RATE_GROUP_DRIVER_CYCLE_PORTS],
            config: OnceLock::new(),
            ticks: AtomicU64::new(0),
        })
    }

    /// C++ `configure(DividerSet)`: validates and stores the divider table.
    ///
    /// Asserts per entry that `offset == 0 || offset < divisor` (an
    /// offset at or above the divisor would never fire) and that the
    /// rollover product does not overflow `FwSizeType`.
    ///
    /// Deviation from C++: the table is a `OnceLock` so the handler stays
    /// lock-free — a second `configure` call fw_asserts instead of
    /// replacing the table (C++ allows reconfiguration; no framework
    /// consumer reconfigures).
    pub fn configure(&self, dividers: &[Divider; RATE_GROUP_DRIVER_CYCLE_PORTS]) {
        let mut rollover: FwSizeType = 1;
        for d in dividers {
            fw_assert!(d.offset == 0 || d.offset < d.divisor, d.offset, d.divisor);
            if d.divisor != 0 {
                // C++ parity: FW_ASSERT that rollover * divisor fits.
                let product = rollover.checked_mul(d.divisor);
                fw_assert!(product.is_some(), rollover, d.divisor);
                rollover = product.unwrap_or(1);
            }
        }
        let stored = self.config.set(DriverConfig {
            dividers: *dividers,
            rollover,
        });
        // Documented deviation: reconfigure is a wiring bug here.
        fw_assert!(stored.is_ok());
    }

    /// `CycleIn` — SYNC `Svc.Cycle` input (factory for topology wiring).
    pub fn cycle_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CyclePort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `CycleIn_handler`: fan the tick out per the divider table,
    /// forwarding the same `RawTime`, then advance the tick counter modulo
    /// the rollover.
    fn cycle_in_handler(&self, _port_num: FwIndexType, cycle_start: &RawTime) {
        // C++ parity: FW_ASSERT(m_configured) — add configure() to init.
        let config = self.config.get();
        fw_assert!(config.is_some());
        let Some(config) = config else { return };

        let ticks = self.ticks.load(Ordering::Relaxed);
        for (entry, divider) in config.dividers.iter().enumerate() {
            if divider.divisor == 0 || ticks % divider.divisor != divider.offset {
                continue;
            }
            if let Some(port) = self.cycle_out[entry].try_get() {
                port.target.invoke(port.port_num, cycle_start);
            }
        }

        fw_assert!(config.rollover > 0);
        self.ticks
            .store((ticks + 1) % config.rollover, Ordering::Relaxed);
    }
}

impl CyclePort for RateGroupDriver {
    fn invoke(&self, port_num: FwIndexType, cycle_start: &RawTime) {
        self.cycle_in_handler(port_num, cycle_start);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    /// Records (tick-index-as-called, RawTime) per invocation.
    #[derive(Default)]
    struct CycleRecorder {
        calls: Mutex<Vec<(FwIndexType, u64, u32)>>,
    }

    impl CyclePort for CycleRecorder {
        fn invoke(&self, port_num: FwIndexType, cycle_start: &RawTime) {
            self.calls.lock().unwrap().push((
                port_num,
                cycle_start.get_seconds(),
                cycle_start.get_nanoseconds(),
            ));
        }
    }

    fn tick(driver: &Arc<RateGroupDriver>, n: u64) {
        let port = driver.cycle_in(0);
        for i in 0..n {
            let raw = RawTime::from_parts(i, 0);
            port.target.invoke(port.port_num, &raw);
        }
    }

    fn call_seconds(rec: &CycleRecorder) -> Vec<u64> {
        rec.calls.lock().unwrap().iter().map(|c| c.1).collect()
    }

    #[test]
    fn offset_zero_groups_fire_on_the_very_first_tick() {
        let driver = RateGroupDriver::new("rgDriver");
        let recs: Vec<Arc<CycleRecorder>> =
            (0..3).map(|_| Arc::new(CycleRecorder::default())).collect();
        for (i, rec) in recs.iter().enumerate() {
            driver.cycle_out[i].connect(rec.clone(), i as FwIndexType);
        }
        driver.configure(&[Divider::new(1, 0), Divider::new(2, 0), Divider::new(4, 0)]);
        tick(&driver, 1);
        // ticks starts at 0 => 0 % anything == 0 => all offset-0 fire.
        assert_eq!(call_seconds(&recs[0]), vec![0]);
        assert_eq!(call_seconds(&recs[1]), vec![0]);
        assert_eq!(call_seconds(&recs[2]), vec![0]);
    }

    #[test]
    fn divisors_and_offsets_select_ticks() {
        let driver = RateGroupDriver::new("rgDriver");
        let recs: Vec<Arc<CycleRecorder>> =
            (0..3).map(|_| Arc::new(CycleRecorder::default())).collect();
        for (i, rec) in recs.iter().enumerate() {
            driver.cycle_out[i].connect(rec.clone(), i as FwIndexType);
        }
        driver.configure(&[Divider::new(1, 0), Divider::new(2, 1), Divider::new(4, 3)]);
        tick(&driver, 8);
        assert_eq!(call_seconds(&recs[0]), vec![0, 1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(call_seconds(&recs[1]), vec![1, 3, 5, 7]); // odd ticks
        assert_eq!(call_seconds(&recs[2]), vec![3, 7]); // 3 mod 4
    }

    #[test]
    fn forwards_the_same_raw_time_and_port_number() {
        let driver = RateGroupDriver::new("rgDriver");
        let rec = Arc::new(CycleRecorder::default());
        driver.cycle_out[1].connect(rec.clone(), 7);
        driver.configure(&[Divider::default(), Divider::new(1, 0), Divider::default()]);
        let raw = RawTime::from_parts(1234, 5678);
        let port = driver.cycle_in(0);
        port.target.invoke(port.port_num, &raw);
        assert_eq!(*rec.calls.lock().unwrap(), vec![(7, 1234, 5678)]);
    }

    #[test]
    fn zero_divisor_entries_and_unconnected_ports_are_skipped() {
        let driver = RateGroupDriver::new("rgDriver");
        let rec = Arc::new(CycleRecorder::default());
        // Port 0 divisor=0 (unused), port 1 unconnected, port 2 live.
        driver.cycle_out[0].connect(rec.clone(), 0);
        driver.cycle_out[2].connect(rec.clone(), 2);
        driver.configure(&[Divider::new(0, 0), Divider::new(1, 0), Divider::new(1, 0)]);
        tick(&driver, 2);
        let calls = rec.calls.lock().unwrap();
        assert_eq!(calls.len(), 2);
        assert!(calls.iter().all(|c| c.0 == 2));
    }

    #[test]
    fn rollover_is_the_product_of_nonzero_divisors_and_wraps() {
        let driver = RateGroupDriver::new("rgDriver");
        let rec = Arc::new(CycleRecorder::default());
        driver.cycle_out[0].connect(rec.clone(), 0);
        // rollover = 2 * 3 = 6; port 0 fires at ticks 1, 3, 5 mod 6.
        driver.configure(&[Divider::new(2, 1), Divider::new(3, 0), Divider::default()]);
        tick(&driver, 13);
        // ticks 1,3,5 then wrap: 7,9,11 (== 1,3,5 again), then 13th tick is
        // counter value 0 again -> no fire. Exactly 6 calls.
        assert_eq!(rec.calls.lock().unwrap().len(), 6);
        assert_eq!(driver.ticks.load(Ordering::Relaxed), 1);
    }

    #[test]
    #[should_panic]
    fn unconfigured_cycle_asserts() {
        let driver = RateGroupDriver::new("rgDriver");
        tick(&driver, 1);
    }

    #[test]
    #[should_panic]
    fn offset_not_less_than_divisor_asserts() {
        let driver = RateGroupDriver::new("rgDriver");
        driver.configure(&[Divider::new(2, 2), Divider::default(), Divider::default()]);
    }

    #[test]
    #[should_panic]
    fn rollover_overflow_asserts() {
        let driver = RateGroupDriver::new("rgDriver");
        driver.configure(&[
            Divider::new(u64::MAX, 0),
            Divider::new(2, 0),
            Divider::default(),
        ]);
    }

    #[test]
    #[should_panic]
    fn reconfigure_asserts_documented_deviation() {
        let driver = RateGroupDriver::new("rgDriver");
        let set = [Divider::new(1, 0), Divider::default(), Divider::default()];
        driver.configure(&set);
        driver.configure(&set);
    }
}
