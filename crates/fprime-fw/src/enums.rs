//! Small framework-wide FPP enums.
//!
//! Port of `Fw/Types/Types.fpp`, `Fw/Cmd/Cmd.fpp` (`CmdResponse`),
//! `Fw/Log/Log.fpp` (`LogSeverity`), `Fw/Prm` (`ParamValid`), `Fw/Tlm`
//! (`TlmValid`) and the `DeserialStatus` shadow enum. FPP enums serialize at
//! their declared representation width (u8 here) — NOT the 4-byte
//! `FwEnumStoreType` of plain C++ enums — and deserialization validates the
//! exact declared values, returning `DeserFormatError` otherwise.
//!
//! Every enum here is declared with the [`fpp_enum!`](crate::fpp_enum) macro
//! of the codegen layer ([`crate::fpp`]).

use crate::fpp_enum;

fpp_enum! {
    /// Generic pass/fail (`Fw::Success`).
    pub enum Success : u8 {
        /// Representing failure.
        Failure = 0,
        /// Representing success.
        Success = 1,
    }
    default Failure
}

fpp_enum! {
    /// Enabled/disabled state (`Fw::Enabled`).
    pub enum Enabled : u8 {
        /// Disabled state.
        Disabled = 0,
        /// Enabled state.
        Enabled = 1,
    }
    default Disabled
}

fpp_enum! {
    /// Wait or don't wait for an operation (`Fw::Wait`).
    pub enum Wait : u8 {
        /// Wait for the operation.
        Wait = 0,
        /// Don't wait for the operation.
        NoWait = 1,
    }
    default Wait
}

fpp_enum! {
    /// Completion status (`Fw::Completed`).
    pub enum Completed : u8 {
        /// The operation ran to completion.
        Completed = 0,
        /// The operation was canceled.
        Canceled = 1,
        /// The operation failed.
        Failed = 2,
    }
    default Completed
}

fpp_enum! {
    /// Health state (`Fw::Health`).
    pub enum Health : u8 {
        /// Healthy.
        Healthy = 0,
        /// Sick (missed pings, below fatal threshold).
        Sick = 1,
        /// Failed (fatal ping timeout).
        Failed = 2,
    }
    default Healthy
}

fpp_enum! {
    /// Command completion status (`Fw::CmdResponse`).
    pub enum CmdResponse : u8 {
        /// Command successfully executed.
        Ok = 0,
        /// Invalid opcode dispatched.
        InvalidOpcode = 1,
        /// Command failed validation.
        ValidationError = 2,
        /// Command failed to deserialize.
        FormatError = 3,
        /// Command had an execution error.
        ExecutionError = 4,
        /// Component busy.
        Busy = 5,
    }
    default Ok
}

fpp_enum! {
    /// Event severity (`Fw::LogSeverity`). NOTE: values start at 1; 0 is not
    /// a declared value and fails deserialization.
    pub enum LogSeverity : u8 {
        /// A fatal non-recoverable event.
        Fatal = 1,
        /// A serious but recoverable event.
        WarningHi = 2,
        /// A less serious but recoverable event.
        WarningLo = 3,
        /// An activity related to command execution.
        Command = 4,
        /// Important informational events.
        ActivityHi = 5,
        /// Less important informational events.
        ActivityLo = 6,
        /// Software diagnostic events.
        Diagnostic = 7,
    }
    default Fatal
}

fpp_enum! {
    /// Parameter validity (`Fw::ParamValid`).
    pub enum ParamValid : u8 {
        /// Parameter uninitialized.
        Uninit = 0,
        /// Parameter valid.
        Valid = 1,
        /// Parameter invalid.
        Invalid = 2,
        /// Parameter default value in use.
        Default = 3,
    }
    default Uninit
}

impl ParamValid {
    /// The C++ `FW_PARAM_OK` macro: usable parameter value.
    #[must_use = "the validity flag should be checked"]
    pub fn is_ok(self) -> bool {
        matches!(self, ParamValid::Valid | ParamValid::Default)
    }
}

fpp_enum! {
    /// Telemetry-get validity (`Fw::TlmValid`).
    pub enum TlmValid : u8 {
        /// Valid channel value returned.
        Valid = 0,
        /// Channel not found / never written.
        Invalid = 1,
    }
    default Valid
}

fpp_enum! {
    /// GDS-visible u8 shadow of the deserialize side of
    /// [`SerializeStatus`](crate::serial::SerializeStatus) (`Fw::DeserialStatus`).
    pub enum DeserialStatus : u8 {
        /// Operation succeeded.
        Ok = 0,
        /// Buffer was empty.
        BufferEmpty = 3,
        /// Data format error.
        FormatError = 4,
        /// Size mismatch.
        SizeMismatch = 5,
        /// Type mismatch.
        TypeMismatch = 6,
    }
    default Ok
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::{
        Deserialize, Endianness, LinearBuffer, SerBuf, Serialize, SerializeStatus,
    };

    #[test]
    fn discriminants_match_cpp() {
        assert_eq!(Success::Failure as u8, 0);
        assert_eq!(Success::Success as u8, 1);
        assert_eq!(Enabled::Disabled as u8, 0);
        assert_eq!(Enabled::Enabled as u8, 1);
        assert_eq!(Wait::Wait as u8, 0);
        assert_eq!(Wait::NoWait as u8, 1);
        assert_eq!(Completed::Completed as u8, 0);
        assert_eq!(Completed::Canceled as u8, 1);
        assert_eq!(Completed::Failed as u8, 2);
        assert_eq!(Health::Healthy as u8, 0);
        assert_eq!(Health::Sick as u8, 1);
        assert_eq!(Health::Failed as u8, 2);
        assert_eq!(CmdResponse::Ok as u8, 0);
        assert_eq!(CmdResponse::InvalidOpcode as u8, 1);
        assert_eq!(CmdResponse::ValidationError as u8, 2);
        assert_eq!(CmdResponse::FormatError as u8, 3);
        assert_eq!(CmdResponse::ExecutionError as u8, 4);
        assert_eq!(CmdResponse::Busy as u8, 5);
        assert_eq!(LogSeverity::Fatal as u8, 1);
        assert_eq!(LogSeverity::WarningHi as u8, 2);
        assert_eq!(LogSeverity::WarningLo as u8, 3);
        assert_eq!(LogSeverity::Command as u8, 4);
        assert_eq!(LogSeverity::ActivityHi as u8, 5);
        assert_eq!(LogSeverity::ActivityLo as u8, 6);
        assert_eq!(LogSeverity::Diagnostic as u8, 7);
        assert_eq!(ParamValid::Uninit as u8, 0);
        assert_eq!(ParamValid::Valid as u8, 1);
        assert_eq!(ParamValid::Invalid as u8, 2);
        assert_eq!(ParamValid::Default as u8, 3);
        assert_eq!(TlmValid::Valid as u8, 0);
        assert_eq!(TlmValid::Invalid as u8, 1);
        assert_eq!(DeserialStatus::Ok as u8, 0);
        assert_eq!(DeserialStatus::BufferEmpty as u8, 3);
        assert_eq!(DeserialStatus::FormatError as u8, 4);
        assert_eq!(DeserialStatus::SizeMismatch as u8, 5);
        assert_eq!(DeserialStatus::TypeMismatch as u8, 6);
    }

    #[test]
    fn fpp_enums_serialize_at_repr_width() {
        let mut buf = LinearBuffer::<8>::new();
        let sev = LogSeverity::WarningHi;
        assert_eq!(sev.serialized_size(), 1);
        assert_eq!(
            sev.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0x02]);
    }

    #[test]
    fn deserialize_validates_declared_values() {
        // LogSeverity 0 is not declared (severities start at 1)
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.serialize_u8_be(0), SerializeStatus::Ok);
        let mut sev = LogSeverity::Diagnostic;
        assert_eq!(
            sev.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserFormatError
        );
        // target unmodified on invalid value
        assert_eq!(sev, LogSeverity::Diagnostic);

        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.serialize_u8_be(6), SerializeStatus::Ok);
        let mut resp = CmdResponse::Ok;
        assert_eq!(
            resp.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserFormatError
        );
        assert_eq!(resp, CmdResponse::Ok);
    }

    #[test]
    fn roundtrip_all_cmd_responses() {
        for resp in [
            CmdResponse::Ok,
            CmdResponse::InvalidOpcode,
            CmdResponse::ValidationError,
            CmdResponse::FormatError,
            CmdResponse::ExecutionError,
            CmdResponse::Busy,
        ] {
            let mut buf = LinearBuffer::<4>::new();
            assert_eq!(
                resp.serialize_to(&mut buf, Endianness::Big),
                SerializeStatus::Ok
            );
            let mut out = CmdResponse::Ok;
            assert_eq!(
                out.deserialize_from(&mut buf, Endianness::Big),
                SerializeStatus::Ok
            );
            assert_eq!(out, resp);
        }
    }

    #[test]
    fn param_valid_ok_matches_cpp_macro() {
        assert!(ParamValid::Valid.is_ok());
        assert!(ParamValid::Default.is_ok());
        assert!(!ParamValid::Uninit.is_ok());
        assert!(!ParamValid::Invalid.is_ok());
    }
}
