//! # ByteStreamDriver model — port of `Drv/ByteStreamDriverModel` / `Drv/Interfaces`
//!
//! C++ sources: `Drv/ByteStreamDriverModel/ByteStreamDriverModel.fpp`,
//! `Drv/Interfaces/ByteStreamDriver.fpp`.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (ByteStreamDriver model).
//!
//! ## Canonical-trait seam (documented)
//!
//! `fprime-svc` and `fprime-drv` must not depend on each other (dependency
//! DAG in ARCHITECTURE.md), matching the C++ layering where the `Drv`
//! interfaces are FPP-imported by `Svc` components without a library
//! dependency. `fprime-svc`'s `com_stub` module therefore defines
//! structurally-identical local copies of these traits. **The traits in THIS
//! module are the canonical ones** (they port the defining FPP files under
//! `Drv/`); the reference deployment bridges the two families with tiny
//! (2-line) adapter shims per port.
//!
//! The `recvReturnIn` direction (`Fw.BufferSend`) reuses
//! [`fprime_comp::BufferSendPort`] directly — re-exported below — exactly as
//! the C++ interface reuses the framework `Fw.BufferSend` port type.

use fprime_config::FwIndexType;
use fprime_fw::serial::{Deserialize, Endianness, SerBufAny, Serialize, SerializeStatus};
use fprime_fw::{Buffer, SerBuf};

/// `recvReturnIn` port type: the framework `Fw.BufferSend` port, reused
/// verbatim (C++ parity — the interface declares `Fw.BufferSend`, not a
/// driver-specific port).
pub use fprime_comp::BufferSendPort;

/// `Drv::ByteStreamStatus` — FPP enum, `repr U8`
/// (`Drv/ByteStreamDriverModel/ByteStreamDriverModel.fpp`).
///
/// FPP enum semantics: serializes at its representation width (1 byte) and
/// decode validates the exact declared values (`DeserFormatError` otherwise,
/// target left unmodified).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ByteStreamStatus {
    /// Operation worked as expected.
    #[default]
    OpOk = 0,
    /// Data send should be retried.
    SendRetry = 1,
    /// Receive worked but there is no data.
    RecvNoData = 2,
    /// Error occurred; buffer contents undefined.
    OtherError = 3,
}

impl TryFrom<u8> for ByteStreamStatus {
    type Error = u8;
    /// Map a raw byte to the enum; `Err` carries the unmatched raw value.
    fn try_from(v: u8) -> Result<Self, u8> {
        match v {
            0 => Ok(Self::OpOk),
            1 => Ok(Self::SendRetry),
            2 => Ok(Self::RecvNoData),
            3 => Ok(Self::OtherError),
            other => Err(other),
        }
    }
}

impl Serialize for ByteStreamStatus {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        buf.serialize_u8(*self as u8, e)
    }
    fn serialized_size(&self) -> usize {
        size_of::<u8>()
    }
}

impl Deserialize for ByteStreamStatus {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let mut raw: u8 = 0;
        let status = buf.deserialize_u8(&mut raw, e);
        if status != SerializeStatus::Ok {
            return status;
        }
        // C++ parity: an undeclared value is DeserFormatError; the raw byte
        // is consumed but the target is left unmodified.
        match Self::try_from(raw) {
            Ok(v) => {
                *self = v;
                SerializeStatus::Ok
            }
            Err(_) => SerializeStatus::DeserFormatError,
        }
    }
}

/// `Drv.ByteStreamSend` — synchronous send. The caller retains ownership of
/// the buffer (C++ `ref sendBuffer`), hence `&mut Buffer`; the driver only
/// reads the data window.
pub trait ByteStreamSendPort: Send + Sync {
    /// Send the buffer's data window; returns the send status.
    fn invoke(&self, port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus;
}

/// `Drv.ByteStreamData` — receive delivery (and, for async drivers, the send
/// callback). The buffer moves to the receiver, which must eventually return
/// it via the driver's `recvReturnIn` (`Fw.BufferSend`) for deallocation.
pub trait ByteStreamDataPort: Send + Sync {
    /// Deliver a buffer with its receive status.
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer, status: ByteStreamStatus);
}

/// `Drv.ByteStreamReady` — driver-ready signal (no arguments). Fired on
/// every successful connection (drives `ComStub.drvConnected`).
pub trait ByteStreamReadyPort: Send + Sync {
    /// Signal that the driver is ready for traffic.
    fn invoke(&self, port_num: FwIndexType);
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::LinearBuffer;

    #[test]
    fn byte_stream_status_discriminants_match_fpp() {
        assert_eq!(ByteStreamStatus::OpOk as u8, 0);
        assert_eq!(ByteStreamStatus::SendRetry as u8, 1);
        assert_eq!(ByteStreamStatus::RecvNoData as u8, 2);
        assert_eq!(ByteStreamStatus::OtherError as u8, 3);
        assert_eq!(ByteStreamStatus::default(), ByteStreamStatus::OpOk);
    }

    #[test]
    fn byte_stream_status_serializes_at_u8_width() {
        let mut buf = LinearBuffer::<4>::new();
        let status = ByteStreamStatus::RecvNoData.serialize_to(&mut buf, Endianness::Big);
        assert!(status.is_ok());
        assert_eq!(buf.as_slice(), &[2]);
        assert_eq!(ByteStreamStatus::OtherError.serialized_size(), 1);
    }

    #[test]
    fn byte_stream_status_round_trips() {
        for v in [
            ByteStreamStatus::OpOk,
            ByteStreamStatus::SendRetry,
            ByteStreamStatus::RecvNoData,
            ByteStreamStatus::OtherError,
        ] {
            let mut buf = LinearBuffer::<4>::new();
            assert!(v.serialize_to(&mut buf, Endianness::Big).is_ok());
            let mut out = ByteStreamStatus::OpOk;
            assert!(out.deserialize_from(&mut buf, Endianness::Big).is_ok());
            assert_eq!(out, v);
        }
    }

    #[test]
    fn byte_stream_status_decode_rejects_undeclared_value() {
        let mut buf = LinearBuffer::<4>::new();
        assert!(buf.serialize_u8_be(4).is_ok());
        let mut out = ByteStreamStatus::SendRetry;
        let status = out.deserialize_from(&mut buf, Endianness::Big);
        assert_eq!(status, SerializeStatus::DeserFormatError);
        // Target left unmodified on invalid decode.
        assert_eq!(out, ByteStreamStatus::SendRetry);
    }
}
