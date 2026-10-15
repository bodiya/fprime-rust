//! The `Demo.Controller` implementation: an active component with an
//! internal state machine instance (`lifecycle: Lifecycle`, generated from
//! FPP) and an external one (`watchdog: Watchdog`, the hand-written
//! [`Watchdog`] below, bound with `--bind-type Demo.Watchdog=...`).
//!
//! Commands `START`/`STOP` send the lifecycle its signals through the
//! queue; each scheduler tick sends `tick` to the lifecycle and `Tick` to
//! the watchdog, which, unless `KICK`ed, expires after
//! [`WATCHDOG_LIMIT`] ticks and makes the controller stop the lifecycle.
//! The lifecycle's actions emit events and telemetry.

use crate::generated::Demo::{
    self, ControllerBase, ControllerComponent, ControllerHandlers, Lifecycle,
};
use fprime_config::{FwEnumStoreType, FwIndexType, FwOpcodeType};
use fprime_fw::{CmdResponse, SmSignalBuffer};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

/// Ticks without a `KICK` before the watchdog expires.
pub const WATCHDOG_LIMIT: u32 = 5;

/// The watchdog's signals (C++ `Watchdog_Interface::Watchdog_Signals`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchdogSignal {
    /// One scheduler tick.
    Tick = 0,
    /// Reset the count.
    Kick = 1,
}

impl From<WatchdogSignal> for FwEnumStoreType {
    fn from(s: WatchdogSignal) -> Self {
        s as FwEnumStoreType
    }
}

impl TryFrom<FwEnumStoreType> for WatchdogSignal {
    type Error = FwEnumStoreType;
    fn try_from(v: FwEnumStoreType) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(Self::Tick),
            1 => Ok(Self::Kick),
            other => Err(other),
        }
    }
}

/// What the watchdog calls back into (C++ `Watchdog_Interface`).
pub trait WatchdogParent {
    /// The watchdog expired.
    fn watchdog_expired(&self, sm_id: FwEnumStoreType);
}

/// `Demo.Watchdog`: counts ticks, expires once at [`WATCHDOG_LIMIT`].
#[derive(Debug, Default)]
pub struct Watchdog {
    count: AtomicU32,
}

impl Watchdog {
    /// Ticks since the last kick.
    pub fn count(&self) -> u32 {
        self.count.load(Ordering::Relaxed)
    }
}

impl fprime_comp::sm::ExternalStateMachine for Watchdog {
    type Signal = WatchdogSignal;
    type Parent = dyn WatchdogParent;

    fn init(&self, _parent: &Self::Parent, _sm_id: FwEnumStoreType) {
        self.count.store(0, Ordering::Relaxed);
    }

    fn update(
        &self,
        parent: &Self::Parent,
        sm_id: FwEnumStoreType,
        signal: WatchdogSignal,
        _data: &SmSignalBuffer,
    ) {
        match signal {
            WatchdogSignal::Tick => {
                if self.count.fetch_add(1, Ordering::Relaxed) + 1 == WATCHDOG_LIMIT {
                    parent.watchdog_expired(sm_id);
                }
            }
            WatchdogSignal::Kick => self.count.store(0, Ordering::Relaxed),
        }
    }
}

/// What the controller recorded (for the tests).
#[derive(Debug, Default)]
pub struct ControllerLog {
    /// Ticks the lifecycle counted (`countTick` values).
    pub counted: Vec<u32>,
    /// Watchdog signals dropped by the overflow hook.
    pub watchdog_overflows: Vec<FwEnumStoreType>,
}

/// The controller.
pub struct Controller {
    base: ControllerBase,
    /// The log.
    pub log: Mutex<ControllerLog>,
}

impl Controller {
    /// Construct with every port unconnected.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: ControllerBase::new(name),
            log: Mutex::new(ControllerLog::default()),
        })
    }
}

impl ControllerHandlers for Controller {
    fn base(&self) -> &ControllerBase {
        &self.base
    }

    fn sched_in_handler(&self, _port_num: FwIndexType, context: u32) {
        self.lifecycle_send_signal_tick(context);
        self.watchdog_state_machine_invoke(WatchdogSignal::Tick, &SmSignalBuffer::new());
        self.base.tlm_write_state(self.base.lifecycle_get_state());
    }

    fn start_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        self.lifecycle_send_signal_start();
        self.base
            .cmd
            .cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn stop_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        self.lifecycle_send_signal_stop();
        self.base
            .cmd
            .cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn kick_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        self.watchdog_state_machine_invoke(WatchdogSignal::Kick, &SmSignalBuffer::new());
        self.base
            .cmd
            .cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    fn Demo_Lifecycle_action_enterRunning(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Lifecycle::Signal,
    ) {
        self.base.log_activity_hi_running_entered();
    }

    fn Demo_Lifecycle_action_exitRunning(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Lifecycle::Signal,
    ) {
        self.base.log_activity_hi_running_exited();
    }

    fn Demo_Lifecycle_action_countTick(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Lifecycle::Signal,
        value: u32,
    ) {
        self.log.lock().unwrap().counted.push(value);
        self.base.tlm_write_ticks(value);
    }

    fn Demo_Lifecycle_guard_warmedUp(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Lifecycle::Signal,
        value: u32,
    ) -> bool {
        value >= Demo::WARMUP_TICKS as u32
    }

    fn watchdog_state_machine_overflow_hook(
        &self,
        signal: FwEnumStoreType,
        _data: &SmSignalBuffer,
    ) {
        self.log.lock().unwrap().watchdog_overflows.push(signal);
    }
}

impl WatchdogParent for Controller {
    /// Runs on the controller's thread (the watchdog's signals are
    /// dispatched there): stop the lifecycle.
    fn watchdog_expired(&self, _sm_id: FwEnumStoreType) {
        self.base.log_warning_hi_watchdog_expired();
        self.lifecycle_send_signal_stop();
    }
}

crate::generated::Demo::impl_controller_component!(Controller);
