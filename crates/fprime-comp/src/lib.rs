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
//!   serializing raw `Fw::Buffer` pointers into async queue messages.
//!
//! The integration test `tests/example_component.rs` is the normative
//! exemplar of how a component is assembled from these pieces.

pub mod active;
pub mod escrow;
pub mod glue;
pub mod msg;
pub mod obj;
pub mod port;
pub mod queued;

pub use active::{ActiveBase, ActiveComponent, Lifecycle};
pub use escrow::BufferEscrow;
pub use glue::{CmdGlue, EventGlue, EventThrottle, PrmGlue, TlmGlue, time_get};
pub use msg::QueueFullPolicy;
pub use obj::PassiveBase;
pub use port::{
    BufferGetPort, BufferSendPort, CmdPort, CmdRegPort, CmdResponsePort, ComDataWithContextPort,
    ComPort, CyclePort, FatalEventPort, LogPort, LogTextPort, OutputPort, PingPort, PortRef,
    PrmGetPort, PrmSetPort, SchedPort, SuccessConditionPort, TimePort, TlmPort, WatchDogPort,
};
pub use queued::{ComponentDispatch, MsgDispatchStatus, QueuedBase};
