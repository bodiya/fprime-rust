//! # FatalHandler — port of `Svc::FatalHandler` (passive)
//!
//! C++ sources: `Svc/FatalHandler/FatalHandler.fpp`,
//! `Svc/FatalHandler/FatalHandlerComponentLinuxImpl.cpp` (the unix impl is
//! the one ported; baremetal spins, VxWorks suspends).
//! Analysis: `docs/cpp-analysis/svc-core.md` (FatalHandler section).
//!
//! Single sync `FatalReceive` input (`Svc.FatalEvent`), normally wired to
//! `EventManager::fatal_announce`. The unix handler logs
//! `FATAL <id> handled.`, delays one second so the FATAL event can flush
//! downlink, then terminates the process.
//!
//! Deviations from C++ (documented):
//! - `raise(SIGABRT)` has no safe zero-dependency Rust equivalent; the
//!   default action logs and calls `std::process::exit(1)` (no core dump).
//! - The terminal action is injectable via [`FatalHandler::set_exit_action`]
//!   so tests (and projects) can observe the shutdown instead of dying —
//!   the C++ equivalent is choosing a different platform impl at build
//!   time.

use fprime_comp::{FatalEventPort, PassiveBase, PortRef};
use fprime_config::{FwEventIdType, FwIndexType};
use fprime_fw::{TimeInterval, fw_log};
use fprime_os::Task;
use std::sync::{Arc, Mutex};

/// The terminal action invoked after the FATAL is logged and flushed.
pub type FatalExitAction = Box<dyn Fn(FwEventIdType) + Send + Sync>;

/// `Svc::FatalHandler` — passive FATAL-event terminator.
pub struct FatalHandler {
    /// Passive core (name / id_base / instance).
    pub base: PassiveBase,
    /// Injectable terminal action (default: log + `exit(1)`).
    exit_action: Mutex<FatalExitAction>,
}

impl FatalHandler {
    /// Construct with the default (process-terminating) action.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            exit_action: Mutex::new(Box::new(|_id| {
                // C++ logs and raises SIGABRT before exit(1); the abort
                // signal / core dump is not reproducible without libc.
                fw_log!("Exiting with error code 1.\n");
                std::process::exit(1);
            })),
        })
    }

    /// Replace the terminal action (call before wiring/start; tests use
    /// this to observe the shutdown instead of dying).
    pub fn set_exit_action(&self, action: FatalExitAction) {
        *self.exit_action.lock().unwrap() = action;
    }

    /// `FatalReceive` — SYNC `Svc.FatalEvent` input.
    pub fn fatal_receive_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn FatalEventPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `FatalReceive_handler` (unix impl): log, delay 1s so the FATAL
    /// event flushes downlink, then run the terminal action.
    fn fatal_receive_handler(&self, _port_num: FwIndexType, id: FwEventIdType) {
        fw_log!("FATAL {} handled.\n", id);
        let _ = Task::delay(TimeInterval::new(1, 0));
        (self.exit_action.lock().unwrap())(id);
    }
}

impl FatalEventPort for FatalHandler {
    fn invoke(&self, port_num: FwIndexType, id: FwEventIdType) {
        self.fatal_receive_handler(port_num, id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn injected_exit_action_receives_id_after_one_second_delay() {
        let comp = FatalHandler::new("fatalHandler");
        let seen: Arc<Mutex<Vec<FwEventIdType>>> = Arc::new(Mutex::new(Vec::new()));
        let seen_clone = seen.clone();
        comp.set_exit_action(Box::new(move |id| {
            seen_clone.lock().unwrap().push(id);
        }));

        let port = comp.fatal_receive_in(0);
        let start = Instant::now();
        port.target.invoke(port.port_num, 0xDEAD);
        let elapsed = start.elapsed();

        // The handler delays 1s (event-flush grace) before terminating.
        assert!(elapsed >= Duration::from_millis(950), "elapsed {elapsed:?}");
        assert_eq!(*seen.lock().unwrap(), vec![0xDEAD]);
    }

    #[test]
    fn exit_action_can_be_replaced() {
        let comp = FatalHandler::new("fatalHandler2");
        let count = Arc::new(Mutex::new(0u32));
        let c1 = count.clone();
        comp.set_exit_action(Box::new(move |_| *c1.lock().unwrap() += 1));
        let c2 = count.clone();
        comp.set_exit_action(Box::new(move |_| *c2.lock().unwrap() += 10));
        let port = comp.fatal_receive_in(0);
        port.target.invoke(port.port_num, 1);
        // Only the latest action runs.
        assert_eq!(*count.lock().unwrap(), 10);
    }
}
