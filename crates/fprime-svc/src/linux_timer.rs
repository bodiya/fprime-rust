//! # LinuxTimer — port of `Svc::LinuxTimer` (passive cycle driver)
//!
//! C++ sources: `Svc/LinuxTimer/LinuxTimer{.hpp,Common.cpp,TaskDelay.cpp}`,
//! `Svc/LinuxTimer/LinuxTimer.fpp` (the `Drv.Tick` interface: one
//! `CycleOut: Svc.Cycle` output port).
//! Analysis: ARCHITECTURE.md ("Timer cycle source: blocking loop ->
//! CycleOut").
//!
//! [`LinuxTimer::start_timer`] BLOCKS the calling thread (typically the
//! deployment's main thread), looping: read `RawTime::now`, invoke
//! `CycleOut`, then sleep the *remainder* of the interval (elapsed handler
//! time is subtracted so the tick rate does not drift — the port's
//! documented refinement over the C++ fixed `Os::Task::delay`).
//! [`LinuxTimer::quit`] (an atomic flag, the C++ mutex-guarded bool) makes
//! `start_timer` return. A persistent `RawTime` read failure is logged
//! once through `Fw::Logger` (latched), C++ parity.
//!
//! [`LinuxTimer::tick`] is the non-blocking helper: one immediate
//! timestamp + `CycleOut` invocation, for tests and manual driving.

use fprime_comp::{CyclePort, OutputPort, PassiveBase};
use fprime_fw::{TimeInterval, fw_log};
use fprime_os::RawTime;
use fprime_os::rawtime::Status as RawTimeStatus;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// `Svc::LinuxTimer`.
pub struct LinuxTimer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `CycleOut: Svc.Cycle` output port.
    pub cycle_out: OutputPort<dyn CyclePort>,
    /// `m_quit` — makes `start_timer` return (C++ mutex-guarded bool;
    /// here a relaxed-consistency atomic is sufficient and lock-free).
    quit: AtomicBool,
    /// Latch so a persistent raw-time failure is logged once.
    raw_time_error_logged: AtomicBool,
}

impl LinuxTimer {
    /// Construct.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            cycle_out: OutputPort::new(),
            quit: AtomicBool::new(false),
            raw_time_error_logged: AtomicBool::new(false),
        })
    }

    /// C++ `startTimer(interval)`: blocks the calling thread, emitting a
    /// `CycleOut` tick per interval until [`quit`](Self::quit) is called.
    pub fn start_timer(&self, interval: TimeInterval) {
        let period = Duration::new(
            u64::from(interval.get_seconds()),
            interval.get_useconds() * 1_000,
        );
        loop {
            if self.quit.load(Ordering::Relaxed) {
                return;
            }
            let iteration_start = Instant::now();
            self.tick();
            // Sleep only the remaining interval so handler time does not
            // accumulate into rate drift.
            let elapsed = iteration_start.elapsed();
            if let Some(remaining) = period.checked_sub(elapsed) {
                let interval =
                    TimeInterval::new(remaining.as_secs() as u32, remaining.subsec_micros());
                let _ = fprime_os::Task::delay(interval);
            }
        }
    }

    /// C++ `quit()`: makes a blocked [`start_timer`](Self::start_timer)
    /// return (at the end of the current interval).
    pub fn quit(&self) {
        self.quit.store(true, Ordering::Relaxed);
    }

    /// Non-blocking helper: one timestamp read + `CycleOut` invocation.
    /// Used by tests (tick N times deterministically) and manual drivers.
    pub fn tick(&self) {
        let mut raw_time = RawTime::new();
        let status = raw_time.now();
        if status != RawTimeStatus::OpOk
            && !self.raw_time_error_logged.swap(true, Ordering::Relaxed)
        {
            // C++ parity: latch the report so a persistent failure does
            // not flood the console at the timer rate.
            fw_log!(
                "[ERROR] LinuxTimer failed to read raw time: {}",
                status as i32
            );
        }
        let port = self.cycle_out.get();
        port.target.invoke(port.port_num, &raw_time);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_config::FwIndexType;
    use std::sync::Mutex;

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

    #[test]
    fn tick_invokes_cycle_out_with_a_fresh_timestamp() {
        let timer = LinuxTimer::new("linuxTimer");
        let rec = Arc::new(CycleRecorder::default());
        timer.cycle_out.connect(rec.clone(), 3);
        for _ in 0..3 {
            timer.tick();
        }
        let calls = rec.calls.lock().unwrap();
        assert_eq!(calls.len(), 3);
        assert!(calls.iter().all(|c| c.0 == 3));
        // Wall-clock timestamps: nonzero seconds, monotonic non-decreasing.
        assert!(calls.iter().all(|c| c.1 > 0));
        assert!(
            calls
                .windows(2)
                .all(|w| (w[0].1, w[0].2) <= (w[1].1, w[1].2))
        );
    }

    #[test]
    fn quit_makes_start_timer_return() {
        let timer = LinuxTimer::new("linuxTimer");
        let rec = Arc::new(CycleRecorder::default());
        timer.cycle_out.connect(rec.clone(), 0);
        let timer_thread = timer.clone();
        let handle = std::thread::spawn(move || {
            timer_thread.start_timer(TimeInterval::new(0, 2_000)); // 2 ms
        });
        // Let it tick at least once, then quit.
        while rec.calls.lock().unwrap().is_empty() {
            std::thread::yield_now();
        }
        timer.quit();
        handle.join().expect("timer thread returned");
        assert!(!rec.calls.lock().unwrap().is_empty());
    }

    #[test]
    fn quit_before_start_prevents_any_tick() {
        let timer = LinuxTimer::new("linuxTimer");
        let rec = Arc::new(CycleRecorder::default());
        timer.cycle_out.connect(rec.clone(), 0);
        timer.quit();
        timer.start_timer(TimeInterval::new(0, 1_000)); // returns at once
        assert!(rec.calls.lock().unwrap().is_empty());
    }
}
