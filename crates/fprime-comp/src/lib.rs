//! # fprime-comp — object / port / component model
//!
//! Port of `Fw/Obj`, `Fw/Port`, `Fw/Comp` plus the contract the FPP
//! autocoder generates around them (see `docs/cpp-analysis/fw-comp.md` and
//! `docs/cpp-analysis/fpp-autocoder.md`). This crate replaces the generated
//! `*ComponentBase` classes with hand-written building blocks:
//!
//! - [`obj`]: [`PassiveBase`] — object name, id base, instance number
//!   (`Fw::PassiveComponentBase` state).
//! - [`port`]: [`PortRef`] / [`OutputPort`] connection model plus the
//!   standard framework port traits (`Fw::Sched`, `Fw::Cmd`, `Fw::Log`, ...).
//! - [`msg`]: the byte-exact async queue-message envelope
//!   (`[msg_type i32 BE][port_num i16 BE][args]`) and the per-port
//!   queue-full policy ([`QueueFullPolicy`]).
//! - [`queued`]: [`QueuedBase`] — `Fw::QueuedComponentBase` (queue +
//!   dispatch loop driven through the [`ComponentDispatch`] trait).
//! - [`active`]: [`ActiveBase`] — `Fw::ActiveComponentBase` (task +
//!   lifecycle state machine, `exit()`/`join()`).
//! - [`glue`]: command / event / telemetry / parameter / time helper blocks
//!   equivalent to the autocoded base-class glue.
//! - [`escrow`]: [`BufferEscrow`] — safe replacement for the C++ practice of
//!   serializing raw `Fw::Buffer` pointers into async queue messages, and
//!   [`EscrowedBuffer`] for an `Fw.Buffer` inside a serialized value (a
//!   state machine signal value).
//! - [`serial_port`]: serialize ports (`Fw::InputSerializePort` /
//!   `Fw::OutputSerializePort` and the `FW_PORT_SERIALIZATION` half of the
//!   generated typed ports) — any serializable typed call can be carried as
//!   bytes and replayed on a typed port elsewhere (`Svc::GenericHub`).
//! - [`macros`]: the component-side codegen layer —
//!   [`component_msg_types!`], [`input_port_adapter!`],
//!   [`async_input_port_adapter!`] and [`serializable_port!`] generate the
//!   input-port thunks, envelope codec, message-type discriminants and port
//!   serializers that the C++ FPP autocoder emits.
//! - [`sm`] + [`fpp_state_machine!`]: FPP state machines (`Fw/Sm`) — the
//!   declarative macro for internal (FPP-defined) machines, the runtime it
//!   expands against, and the component side of state machine instances
//!   (the byte-exact signal message, priority and queue-full policy) for
//!   internal and external machines.
//!
//! The integration test `tests/example_component.rs` is the normative
//! exemplar of how a component is assembled from these pieces.
//!
//! # `no_std`
//!
//! Like `fprime-fw`, the crate is always `#![no_std]` + `alloc`; the default
//! `std` feature links `std` back in. With `--no-default-features
//! --features spin` every module above still builds — passive, queued and
//! active components, ports, envelopes, escrow, serialize ports, state
//! machines and the macros — over `spin` locks (`fprime_fw::sync`) and the
//! `no_std` subset of `fprime-os`. An active component's task, `RawTime`
//! sampling and blocking queue waits then come from the platform the
//! deployment registers with `fprime_os::register_platform` (see
//! `fprime_os::platform` and ARCHITECTURE.md "no_std", ROADMAP item 13).

#![no_std]

extern crate alloc;
// Unit tests always run on a hosted target, so they get `std` even in a
// `no_std` build.
#[cfg(any(feature = "std", test))]
extern crate std;

pub mod active;
pub mod escrow;
pub mod glue;
pub mod macros;
pub mod msg;
pub mod obj;
pub mod port;
pub mod queued;
pub mod serial_port;
pub mod sm;
mod sm_macro;

/// The project configuration crate, re-exported so macro expansions and
/// downstream code can reach the `Fw*` type aliases through this crate.
pub use fprime_config as config;
/// The `Fw` core layer, re-exported for macro expansions.
pub use fprime_fw as fw;
/// The `Os` layer, re-exported for macro expansions.
pub use fprime_os as os;

pub use active::{ActiveBase, ActiveComponent, Lifecycle};
pub use escrow::{BufferEscrow, EscrowedBuffer};
pub use glue::{CmdGlue, EventGlue, EventThrottle, PrmGlue, TlmGlue, time_get};
pub use msg::QueueFullPolicy;
pub use obj::PassiveBase;
pub use port::{
    BufferGetPort, BufferSendPort, CmdPort, CmdRegPort, CmdResponsePort, ComDataWithContextPort,
    ComPort, CyclePort, FatalEventPort, LogPort, LogTextPort, OutputPort, PingPort, PortRef,
    PrmGetPort, PrmSetPort, SchedPort, SuccessConditionPort, TimePort, TlmPort, WatchDogPort,
};
pub use queued::{ComponentDispatch, MsgDispatchStatus, QueuedBase};
pub use serial_port::{SerialCallAdapter, SerializablePort, SerializePort, TypedCallAdapter};
pub use sm::{
    ExternalSmOverflowHook, ExternalSmSender, ExternalStateMachine, SmOverflowHook, SmSender,
};

/// The tests' platform (shared with the integration tests): in a `no_std`
/// test build — still a hosted target — `install()` registers a
/// `std::thread` `fprime_os::platform::Platform` so active components run
/// their task through the seam; with `std` it is a no-op.
#[cfg(test)]
#[path = "../tests/no_std_platform/mod.rs"]
pub(crate) mod test_platform;
