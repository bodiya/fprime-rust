//! # fprime-fpp-demo — the FPP-driven path of the port, end to end
//!
//! `build.rs` runs the `fprime-fpp` generator over the model in `fpp/demo`
//! (resolving against the real framework definitions vendored in
//! `fpp/framework`) and the result is included below as [`generated`].
//! Everything in that module is produced by `fpp-to-rust`; this crate
//! only adds the component implementations and tests.

#![forbid(unsafe_code)]

/// The generated code (see `build.rs`).
pub mod generated {
    include!(concat!(env!("OUT_DIR"), "/generated.rs"));
}

pub use generated::Demo;
