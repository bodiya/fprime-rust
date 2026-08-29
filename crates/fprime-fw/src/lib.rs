//! fprime-fw: the F Prime `Fw` core layer.
//!
//! Rust port of `Fw/Types`, `Fw/Time`, `Fw/Com`, `Fw/Cmd`, `Fw/Log`,
//! `Fw/Tlm`, `Fw/Prm`, `Fw/Buffer` and `Fw/Logger` — the byte-exact
//! serialization engine, value types, strings, time, buffers, GDS packets,
//! assert machinery, and the diagnostic logger. Ground truth is the C++
//! implementation as summarized in `docs/cpp-analysis/fw-types.md` and
//! `docs/cpp-analysis/fw-services.md`.

pub mod assert;
pub mod buffer;
pub mod com;
pub mod enums;
pub mod fpp;
pub mod logger;
pub mod packets;
pub mod poly_type;
pub mod serial;
pub mod string;
pub mod time;

/// The project configuration crate, re-exported so `fw_assert!` expansion and
/// downstream code can reach the `Fw*` type aliases through this crate.
pub use fprime_config as config;

pub use buffer::{Buffer, BufferStorage};
pub use com::{Apid, ComPacketType, FrameContext, Pvn, SA_INDEX_UNSET};
pub use enums::{
    CmdResponse, Completed, DeserialStatus, Enabled, Health, LogSeverity, ParamValid, Success,
    TlmValid, Wait,
};
pub use fpp::FppSized;
pub use logger::FwLogger;
pub use packets::{CmdPacket, LogPacket, TlmPacket};
pub use poly_type::PolyType;
pub use serial::{
    CmdArgBuffer, ComBuffer, Deserialize, Endianness, ExtBuf, LengthMode, LinearBuffer, LogBuffer,
    ParamBuffer, SerBuf, SerBufAny, Serialize, SerializeStatus, TlmBuffer,
};
pub use string::{
    CmdStringArg, FileNameString, FwDefaultString, FwString, LogStringArg, ObjectName, ParamString,
    TextLogString, TlmString,
};
pub use time::{Time, TimeBase, TimeComparison, TimeInterval};

pub use crate::assert::AssertHook;
