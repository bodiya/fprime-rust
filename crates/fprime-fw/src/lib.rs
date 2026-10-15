//! fprime-fw: the F Prime `Fw` core layer.
//!
//! Rust port of `Fw/Types`, `Fw/Time`, `Fw/Com`, `Fw/Cmd`, `Fw/Log`,
//! `Fw/Tlm`, `Fw/Prm`, `Fw/Buffer` and `Fw/Logger` — the byte-exact
//! serialization engine, value types, strings, time, buffers, GDS packets,
//! assert machinery, and the diagnostic logger. Ground truth is the C++
//! implementation as summarized in `docs/cpp-analysis/fw-types.md` and
//! `docs/cpp-analysis/fw-services.md`.
//!
//! # `no_std`
//!
//! The crate is always `#![no_std]` + `alloc`; the default `std` feature
//! links `std` back in for the hosted pieces (the `std::sync` lock backend,
//! the stderr default of the assert print). Building with
//! `--no-default-features --features spin` gives the bare `no_std` +
//! `alloc` crate with `spin` locks behind the [`sync`] shim. Nothing else
//! in the public API changes between the two configurations — see
//! ARCHITECTURE.md "no_std" (ROADMAP item 13).

#![no_std]

extern crate alloc;
// Unit tests always run on a hosted target, so they get `std` even in a
// `no_std` build.
#[cfg(any(feature = "std", test))]
extern crate std;

pub mod assert;
pub mod buffer;
pub mod com;
pub mod dp;
pub mod enums;
pub mod file_packet;
pub mod fpp;
pub mod logger;
pub mod packets;
pub mod poly_type;
pub mod serial;
pub mod string;
pub mod sync;
pub mod time;

/// The project configuration crate, re-exported so `fw_assert!` expansion and
/// downstream code can reach the `Fw*` type aliases through this crate.
pub use fprime_config as config;

/// Re-exports used by this workspace's exported macros so their expansions
/// work in `no_std` crates (which have no `std` prelude). Not public API.
#[doc(hidden)]
pub mod __private {
    pub use alloc::format;
}

pub use buffer::{Buffer, BufferStorage};
pub use com::{Apid, ComPacketType, FrameContext, Pvn, SA_INDEX_UNSET};
pub use enums::{
    CmdResponse, Completed, DeserialStatus, Enabled, Health, LogSeverity, ParamValid,
    StringFormatStatus, Success, TlmValid, Wait,
};
pub use fpp::FppSized;
pub use logger::FwLogger;
pub use packets::{CmdPacket, LogPacket, TlmPacket};
pub use poly_type::PolyType;
pub use serial::{
    CmdArgBuffer, ComBuffer, Deserialize, Endianness, ExtBuf, LengthMode, LinearBuffer, LogBuffer,
    ParamBuffer, SerBuf, SerBufAny, Serialize, SerializeStatus, SmSignalBuffer, TlmBuffer,
};
pub use string::{
    CmdStringArg, FileNameString, FwDefaultString, FwString, InternalInterfaceString, LogStringArg,
    ObjectName, ParamString, TextLogString, TlmString,
};
pub use time::{Time, TimeBase, TimeComparison, TimeInterval};

pub use crate::assert::AssertHook;
