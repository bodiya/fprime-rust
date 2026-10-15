//! A `#![no_std]` + `alloc` component crate (ROADMAP item 13).
//!
//! This example is built as a library (`crate-type = ["lib"]` in
//! Cargo.toml), so it needs no `main`, allocator or panic handler of its
//! own. It exists to prove that the framework's EXPORTED MACROS expand in a
//! crate without the `std` prelude: every `cargo build --workspace` /
//! `cargo clippy --all-targets` / `cargo test` compiles it, and the
//! bare-metal check builds it for a target that has no `std` at all:
//!
//! ```text
//! cargo build -p fprime-comp --no-default-features --features spin \
//!     --target thumbv7em-none-eabihf --example no_std_component
//! ```
//!
//! It declares FPP data types, a queued component whose async and sync
//! input ports are macro-generated, an active-component impl, a
//! serializable port and an FPP state machine — and [`run_once`] drives
//! the queued component the way a bare-metal superloop would (no task, no
//! platform needed).

#![no_std]

extern crate alloc;

use alloc::vec::Vec;
use fprime_comp::config::{FwEnumStoreType, FwIndexType, FwQueuePriorityType, FwSizeType};
use fprime_comp::fw::sync::{Arc, Mutex};
use fprime_comp::fw::{
    LinearBuffer, SerBuf, SerBufAny, Success, fpp_array, fpp_enum, fpp_struct, fw_assert, fw_log,
};
use fprime_comp::os::queue::BlockingType;
use fprime_comp::{
    ActiveBase, ActiveComponent, ComponentDispatch, MsgDispatchStatus, QueueFullPolicy, QueuedBase,
    SchedPort, SuccessConditionPort, async_input_port_adapter, component_msg_types,
    input_port_adapter, msg, serializable_port,
};

// ---------------------------------------------------------------------------
// FPP data types.
// ---------------------------------------------------------------------------

fpp_enum! {
    /// A mode.
    pub enum Mode : u8 {
        /// Idle.
        Idle = 0,
        /// Running.
        Run = 1,
    }
    default Idle
}

fpp_struct! {
    /// A sample.
    #[derive(Clone, Copy, Eq)]
    pub struct Sample {
        /// Channel.
        channel: u16,
        /// Mode at sampling time.
        mode: Mode,
    }
    default {}
}

fpp_array! {
    /// Three counters.
    #[derive(Clone, Copy, Eq)]
    pub array Counters = [u32; 3]
}

// ---------------------------------------------------------------------------
// A serializable port.
// ---------------------------------------------------------------------------

/// `port SampleOut(s: Sample, n: U32)`.
pub trait SamplePort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType, sample: Sample, n: u32);
}

serializable_port! {
    port: SamplePort;
    args { val sample: Sample, val n: u32 }
}

// ---------------------------------------------------------------------------
// A state machine.
// ---------------------------------------------------------------------------

/// Generated state machine module.
#[allow(non_snake_case)]
pub mod Blinker {
    fprime_comp::fpp_state_machine! {
        /// On / off.
        state machine Blinker: u8 {
            actions { toggle => action_toggle }
            guards {}
            signals {
                tick = 1 => send_signal_tick {
                    Off => do { toggle } enter On;
                    On => do { toggle } enter Off;
                }
            }
            initial enter Off;
            states {
                state Off = 1 {}
                state On = 2 {}
            }
            choices {}
        }
    }
}

// ---------------------------------------------------------------------------
// A queued component.
// ---------------------------------------------------------------------------

const MSG_SIZE: usize = 32;
const QUEUE_DEPTH: FwSizeType = 4;
const PRIORITY: FwQueuePriorityType = 1;

/// A queued component: `runIn` is async, `statusIn` is sync.
pub struct Blinky {
    /// Queued core.
    pub queued: QueuedBase,
    contexts: Mutex<Vec<u32>>,
    blinker: Blinker::StateMachine,
    toggles: Mutex<u32>,
}

component_msg_types! {
    /// Queue message types.
    impl Blinky {
        /// `runIn`.
        MSG_TYPE_RUN_IN,
    }
}

impl Blinker::Handlers for Blinky {
    fn action_toggle(&self, _sm_id: FwEnumStoreType, _signal: Blinker::Signal) {
        *self.toggles.lock() += 1;
    }
}

impl Blinky {
    /// Construct and create the queue.
    pub fn new(name: &str) -> Arc<Self> {
        let comp = Arc::new(Self {
            queued: QueuedBase::new(name),
            contexts: Mutex::new(Vec::new()),
            blinker: Blinker::StateMachine::new(),
            toggles: Mutex::new(0),
        });
        comp.queued
            .create_queue(QUEUE_DEPTH, MSG_SIZE as FwSizeType);
        comp.blinker.init(0, &*comp);
        comp
    }

    fn run_handler(&self, _port_num: FwIndexType, context: u32) {
        self.contexts.lock().push(context);
        self.blinker.send_signal_tick(self);
        fw_log!("run {}", context);
    }

    fn status_handler(&self, _port_num: FwIndexType, condition: &mut Success) {
        fw_assert!(*condition == Success::Success || *condition == Success::Failure);
    }
}

async_input_port_adapter! {
    /// `runIn` — async `Svc.Sched`.
    component: Blinky;
    adapter: RunInAdapter;
    port: SchedPort;
    input: run_in;
    deserialize: run_in_deserialize;
    handler: run_handler;
    base: queued;
    msg_type: Blinky::MSG_TYPE_RUN_IN;
    msg_size: MSG_SIZE;
    priority: PRIORITY;
    queue_full: QueueFullPolicy::Drop;
    args { val context: u32 }
}

input_port_adapter! {
    /// `statusIn` — sync `Fw.SuccessCondition`.
    component: Blinky;
    adapter: StatusInAdapter;
    port: SuccessConditionPort;
    input: status_in;
    handler: status_handler;
    args { mut condition: Success }
}

impl ComponentDispatch for Blinky {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            Self::MSG_TYPE_RUN_IN => match Self::run_in_deserialize(buf) {
                Some((context,)) => {
                    self.run_handler(port_num, context);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            _ => MsgDispatchStatus::Error,
        }
    }
}

/// An active component compiles the same way; starting it needs a
/// registered `fprime_os::platform::Platform` (see `fprime_os::platform`).
pub struct Worker {
    active: ActiveBase,
}

impl ComponentDispatch for Worker {
    fn dispatch_message(&self, _: FwEnumStoreType, _: &mut dyn SerBufAny) -> MsgDispatchStatus {
        MsgDispatchStatus::Ok
    }
}

impl ActiveComponent for Worker {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

/// One superloop pass: invoke the async port, then drain the queue
/// without blocking. Returns the contexts handled and the toggle count.
pub fn run_once() -> (Vec<u32>, u32) {
    let comp = Blinky::new("blinky");
    let run = comp.run_in(0);
    run.target.invoke(run.port_num, 7);
    run.target.invoke(run.port_num, 8);
    let mut condition = Success::Success;
    let status = comp.status_in(0);
    status.target.invoke(status.port_num, &mut condition);
    while comp.queued.do_dispatch(&*comp, BlockingType::NonBlocking) == MsgDispatchStatus::Ok {}
    let mut scratch = LinearBuffer::<8>::new();
    let _ = scratch.serialize_u32_be(Counters::default()[0]);
    let contexts = comp.contexts.lock().clone();
    let toggles = *comp.toggles.lock();
    (contexts, toggles)
}
