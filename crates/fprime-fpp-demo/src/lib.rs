//! # fprime-fpp-demo — the FPP-driven path of the port, end to end
//!
//! `build.rs` runs the `fprime-fpp` generator over the model in `fpp/demo`
//! (resolving against the real framework definitions vendored in
//! `fpp/framework`) and the result is included below as [`generated`].
//! Everything in that module is produced by `fpp-to-rust`; this crate
//! adds the component implementations ([`sensor`], [`ground`], [`relay`],
//! and for the `Ops` topology [`controller`] — an internal and an
//! external state machine instance — and [`console`]) and tests. The
//! `Ops` topology also holds the hand-written `Svc::TlmPacketizer`,
//! configured by an instance init specifier.
//!
//! [`sm_models`] is generated from upstream state machine models (FppTest,
//! `FpySequencer`, `ComAggregator`); [`fpptest`] implements its components
//! on [`sm_support`].

#![forbid(unsafe_code)]

/// The generated code (see `build.rs`).
pub mod generated {
    include!(concat!(env!("OUT_DIR"), "/generated.rs"));
}

/// The generated state machine models (see `build.rs`): upstream's
/// FppTest `state_machine` models, the `Svc.FpySequencer` and
/// `Svc.ComAggregator` machines, and `Probe`.
pub mod sm_models {
    include!(concat!(env!("OUT_DIR"), "/sm_models.rs"));
}

pub mod console;
pub mod controller;
pub mod fpptest;
pub mod ground;
pub mod relay;
pub mod sensor;
pub mod sm_support;

pub use generated::Demo;
