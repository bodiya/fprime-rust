//! fprime-svc: Rust ports of the standard F Prime `Svc` service components.
//!
//! Each module ports one C++ component; the module header names the C++
//! source and the analysis document that grounds it. Modules follow the
//! component pattern established by `fprime-comp`
//! (see `crates/fprime-comp/tests/example_component.rs`).

pub mod active_rate_group;
pub mod buffer_manager;
pub mod cmd_dispatcher;
pub mod com_queue;
pub mod com_stub;
pub mod event_manager;
pub mod fatal_handler;
pub mod fprime_deframer;
pub mod fprime_framer;
pub mod fprime_router;
pub mod frame_accumulator;
pub mod health;
pub mod linux_timer;
pub mod passive_rate_group;
pub mod passive_text_logger;
pub mod posix_time;
pub mod rate_group_driver;
pub mod tlm_chan;
