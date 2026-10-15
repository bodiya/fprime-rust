//! Stub OSAL backends (port of `Os/Stub/`).
//!
//! Each stub reproduces the upstream stub implementation exactly — every
//! call fails with the documented placeholder outputs — so a deployment on a
//! platform without a real backend behaves like a C++ build linking the
//! `Os_*_Stub` implementations. They are the default backend on non-Linux
//! hosts and are always compiled so tests cover them everywhere.

pub mod cpu;
pub mod memory;
