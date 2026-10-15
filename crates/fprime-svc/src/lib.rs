//! fprime-svc: Rust ports of the standard F Prime `Svc` service components.
//!
//! Each module ports one C++ component; the module header names the C++
//! source and the analysis document that grounds it. Modules follow the
//! component pattern established by `fprime-comp`
//! (see `crates/fprime-comp/tests/example_component.rs`).
//!
//! # `no_std`
//!
//! With the default `std` feature off (`--no-default-features --features
//! spin`) the crate is `no_std` + `alloc` and keeps every component that
//! needs no file system, console or process: the rate groups
//! ([`rate_group_driver`], [`active_rate_group`], [`passive_rate_group`]),
//! [`cmd_dispatcher`], [`event_manager`], [`tlm_chan`], [`tlm_packetizer`],
//! [`health`], the F Prime comms stack ([`fprime_framer`],
//! [`fprime_deframer`], [`frame_accumulator`], [`fprime_router`],
//! [`com_queue`], [`com_stub`], [`buffer_manager`]), the CCSDS stack
//! ([`ccsds`] without `SdlsFileKeyManager`), [`com_aggregator`],
//! [`generic_hub`], [`buffer_accumulator`], [`dp_manager`],
//! [`dp_compress_proc`] (and `dp_zlib_compressor` / the AES-GCM SDLS
//! components with their features), [`passive_text_logger`] and
//! [`active_text_logger`] (console output through `Fw::Logger`; the log
//! file is `std`-only), [`fatal_handler`] (upstream's bare-metal
//! implementation), [`assert_fatal_adapter`], [`cmd_splitter`],
//! [`com_retry`], [`buffer_repeater`], and [`posix_time`] /
//! [`chrono_time`] / [`os_time`] / [`linux_timer`] over the
//! `fprime_os::platform::Platform` clock and delay.
//!
//! `std`-only (they need a file system or the host OS): `file_uplink`,
//! `file_downlink`, `file_manager`, `prm_db`, `dp_writer`, `dp_catalog`,
//! `com_logger`, `cmd_sequencer`, `fpy_sequencer` (sequence files),
//! `system_resources` (procfs and file-system usage),
//! `ccsds::sdls_file_key_manager` and `ccsds::cfdp_manager`. See ARCHITECTURE.md "no_std".

// The std build keeps the std prelude (the file-based components are
// hosted code); only the `no_std` build drops it. Unit tests always run on
// a hosted target, so they keep it too — the non-test `no_std` builds
// (host and bare-metal, `tools/check-no-std.sh`) are what prove the
// library code needs no `std`.
#![cfg_attr(not(any(feature = "std", test)), no_std)]
// The crate docs link the std-only modules; in the `no_std` build they are
// absent, so those links cannot resolve there.
#![cfg_attr(not(feature = "std"), allow(rustdoc::broken_intra_doc_links))]

extern crate alloc;

#[cfg(not(any(feature = "std", feature = "spin")))]
compile_error!(
    "fprime-svc needs a lock backend: enable the default `std` feature, or build \
     `no_std` with `--no-default-features --features spin`"
);

pub mod active_rate_group;
pub mod active_text_logger;
pub mod assert_fatal_adapter;
pub mod buffer_accumulator;
pub mod buffer_manager;
pub mod buffer_repeater;
pub mod ccsds;
pub mod chrono_time;
pub mod cmd_dispatcher;
#[cfg(feature = "std")]
pub mod cmd_sequencer;
pub mod cmd_splitter;
pub mod com_aggregator;
#[cfg(feature = "std")]
pub mod com_logger;
pub mod com_queue;
pub mod com_retry;
pub mod com_stub;
#[cfg(feature = "std")]
pub mod dp_catalog;
pub mod dp_compress_proc;
pub mod dp_manager;
#[cfg(feature = "std")]
pub mod dp_writer;
#[cfg(feature = "zlib")]
pub mod dp_zlib_compressor;
pub mod event_manager;
pub mod fatal_handler;
#[cfg(feature = "std")]
pub mod file_downlink;
#[cfg(feature = "std")]
pub mod file_manager;
#[cfg(feature = "std")]
pub mod file_uplink;
pub mod fprime_deframer;
pub mod fprime_framer;
pub mod fprime_router;
#[cfg(feature = "std")]
pub mod fpy_sequencer;
pub mod frame_accumulator;
pub mod generic_hub;
pub mod health;
pub mod linux_timer;
pub mod os_time;
pub mod passive_rate_group;
pub mod passive_text_logger;
pub mod posix_time;
#[cfg(feature = "std")]
pub mod prm_db;
pub mod rate_group_driver;
#[cfg(feature = "std")]
pub mod system_resources;
pub mod tlm_chan;
pub mod tlm_packetizer;

/// The unit tests' platform: in a `no_std` test build (still a hosted
/// target), [`install`](test_platform::install) registers a `std::thread`
/// `fprime_os::platform::Platform` so active components run their task
/// through the seam; with `std` it is a no-op.
#[cfg(test)]
#[path = "../tests/no_std_platform/mod.rs"]
pub(crate) mod test_platform;
