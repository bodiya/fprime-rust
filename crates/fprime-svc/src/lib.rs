//! fprime-svc: Rust ports of the standard F Prime `Svc` service components.
//!
//! Each module ports one C++ component; the module header names the C++
//! source and the analysis document that grounds it. Modules follow the
//! component pattern established by `fprime-comp`
//! (see `crates/fprime-comp/tests/example_component.rs`).

pub mod active_rate_group;
pub mod active_text_logger;
pub mod buffer_accumulator;
pub mod buffer_manager;
pub mod ccsds;
pub mod cmd_dispatcher;
pub mod cmd_sequencer;
pub mod com_aggregator;
pub mod com_logger;
pub mod com_queue;
pub mod com_stub;
pub mod dp_catalog;
pub mod dp_compress_proc;
pub mod dp_manager;
pub mod dp_writer;
#[cfg(feature = "zlib")]
pub mod dp_zlib_compressor;
pub mod event_manager;
pub mod fatal_handler;
pub mod file_downlink;
pub mod file_manager;
pub mod file_uplink;
pub mod fprime_deframer;
pub mod fprime_framer;
pub mod fprime_router;
pub mod frame_accumulator;
pub mod generic_hub;
pub mod health;
pub mod linux_timer;
pub mod passive_rate_group;
pub mod passive_text_logger;
pub mod posix_time;
pub mod prm_db;
pub mod rate_group_driver;
pub mod system_resources;
pub mod tlm_chan;
pub mod tlm_packetizer;
