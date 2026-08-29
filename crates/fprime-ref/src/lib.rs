//! # fprime-ref — reference deployment library
//!
//! Rust port of the C++ `TestDeploymentsProject/Ref` deployment (analysis:
//! `docs/cpp-analysis/ref-topology.md`), exposing the demo component, the
//! byte-stream trait shims, and the topology for both the `fprime-ref`
//! binary (`src/main.rs`) and the integration tests under `tests/`.

pub mod shims;
pub mod signal_gen;
pub mod topology;
