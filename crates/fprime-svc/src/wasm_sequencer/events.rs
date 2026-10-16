//! The generated `log_*` functions of `WasmSequencerEvents.fppi`.
//!
//! Binary arguments are byte-exact: string arguments are truncated to
//! `min(FW_LOG_STRING_MAX_SIZE, declared size)`, as the generated C++
//! does. The text form prints enum arguments with their Rust names (the
//! C++ text uses the FPP names), like the other components of this port.

use super::{
    ControllerState, ExitReason, HostFunction, MemoryGrowFailReason, ModuleIdx, SequencePhase,
    SignalSource, Status, TrapReason, WasmSequencer,
};
use fprime_config::wasm_sequencer::{
    GLOBAL_NAME_STRING_SIZE, GUEST_EVENT_MESSAGE_SIZE, MODULE_NAME_STRING_SIZE,
};
use fprime_config::{
    FILE_NAME_STRING_SIZE, FW_LOG_STRING_MAX_SIZE, FW_LOG_TEXT_BUFFER_SIZE, FwEventIdType,
    FwOpcodeType,
};
use fprime_fw::{
    CmdResponse, Endianness, LengthMode, LogBuffer, LogSeverity, SerBuf, Serialize,
    SerializeStatus, fw_try,
};
use std::fmt::{self, Write as _};

/// A text-event line built without allocating: formatted text truncated to
/// `FW_LOG_TEXT_BUFFER_SIZE` bytes on a character boundary.
struct EventText {
    buf: [u8; FW_LOG_TEXT_BUFFER_SIZE],
    len: usize,
}

impl EventText {
    fn new(args: fmt::Arguments<'_>) -> Self {
        let mut text = Self {
            buf: [0; FW_LOG_TEXT_BUFFER_SIZE],
            len: 0,
        };
        let _ = text.write_fmt(args);
        text
    }

    fn as_str(&self) -> &str {
        // Only whole characters are ever copied in.
        std::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl fmt::Write for EventText {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            let n = c.len_utf8();
            if self.len + n > self.buf.len() {
                break;
            }
            c.encode_utf8(&mut self.buf[self.len..self.len + n]);
            self.len += n;
        }
        Ok(())
    }
}

/// Displays raw string bytes, replacing invalid UTF-8.
struct Lossy<'a>(&'a [u8]);

impl fmt::Display for Lossy<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for chunk in self.0.utf8_chunks() {
            f.write_str(chunk.valid())?;
            if !chunk.invalid().is_empty() {
                f.write_str("\u{FFFD}")?;
            }
        }
        Ok(())
    }
}

/// The bytes of a string argument as the generated code serializes them:
/// up to the first NUL (a C++ string ends there), truncated to
/// `min(FW_LOG_STRING_MAX_SIZE, size)`.
fn arg_str(bytes: &[u8], size: usize) -> &[u8] {
    let bytes = match bytes.iter().position(|&b| b == 0) {
        Some(nul) => &bytes[..nul],
        None => bytes,
    };
    &bytes[..bytes.len().min(FW_LOG_STRING_MAX_SIZE.min(size))]
}

/// Serialize a string event argument of declared size `size`.
fn ser_str(buf: &mut LogBuffer, bytes: &[u8], size: usize) -> SerializeStatus {
    buf.serialize_bytes(
        arg_str(bytes, size),
        LengthMode::IncludeLength,
        Endianness::Big,
    )
}

/// Serialize an FPP enum / serializable argument.
fn ser<T: Serialize>(buf: &mut LogBuffer, value: &T) -> SerializeStatus {
    value.serialize_to(buf, Endianness::Big)
}

impl WasmSequencer {
    fn log(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        text: fmt::Arguments<'_>,
        write: impl FnOnce(&mut LogBuffer) -> SerializeStatus,
    ) {
        let text = EventText::new(text);
        self.evt
            .log_event(self.id_base(), id, severity, text.as_str(), write);
    }

    pub(crate) fn log_diagnostic_store_allocation_succeeded(&self, module_count: u16) {
        self.log(
            Self::EVENTID_STORE_ALLOCATION_SUCCEEDED,
            LogSeverity::Diagnostic,
            format_args!("Successfully allocated store with {module_count} modules"),
            |b| b.serialize_u16_be(module_count),
        );
    }

    pub(crate) fn log_warning_lo_controller_cannot_invoke(&self) {
        self.log(
            Self::EVENTID_CONTROLLER_CANNOT_INVOKE,
            LogSeverity::WarningLo,
            format_args!("Cannot invoke a module when no module is loaded"),
            |_| SerializeStatus::Ok,
        );
    }

    pub(crate) fn log_warning_lo_controller_busy(
        &self,
        signal_source: SignalSource,
        state: ControllerState,
    ) {
        self.log(
            Self::EVENTID_CONTROLLER_BUSY,
            LogSeverity::WarningLo,
            format_args!("Controller cannot process signal {signal_source:?} in state {state:?}"),
            |b| {
                fw_try!(ser(b, &signal_source));
                ser(b, &state)
            },
        );
    }

    pub(crate) fn log_warning_lo_module_not_found(&self, module_name: &[u8]) {
        let m = arg_str(module_name, MODULE_NAME_STRING_SIZE);
        self.log(
            Self::EVENTID_MODULE_NOT_FOUND,
            LogSeverity::WarningLo,
            format_args!("Module with name {} is not loaded into the store", Lossy(m)),
            |b| ser_str(b, m, MODULE_NAME_STRING_SIZE),
        );
    }

    pub(crate) fn log_warning_lo_memory_grow_rejected(
        &self,
        reason: MemoryGrowFailReason,
        requested_size: u64,
    ) {
        self.log(
            Self::EVENTID_MEMORY_GROW_REJECTED,
            LogSeverity::WarningLo,
            format_args!(
                "Guest memory.grow rejected: {reason:?} (requested {requested_size} bytes)"
            ),
            |b| {
                fw_try!(ser(b, &reason));
                b.serialize_u64_be(requested_size)
            },
        );
    }

    fn log_global_failed(
        &self,
        id: FwEventIdType,
        verb: &str,
        module_name: &[u8],
        global_name: &[u8],
        reason: Status,
    ) {
        let m = arg_str(module_name, MODULE_NAME_STRING_SIZE);
        let g = arg_str(global_name, GLOBAL_NAME_STRING_SIZE);
        self.log(
            id,
            LogSeverity::WarningLo,
            format_args!(
                "Failed to {verb} global value for module {} global name {}: {reason:?}",
                Lossy(m),
                Lossy(g)
            ),
            |b| {
                fw_try!(ser_str(b, m, MODULE_NAME_STRING_SIZE));
                fw_try!(ser_str(b, g, GLOBAL_NAME_STRING_SIZE));
                ser(b, &reason)
            },
        );
    }

    pub(crate) fn log_warning_lo_global_set_failed(
        &self,
        module_name: &[u8],
        global_name: &[u8],
        reason: Status,
    ) {
        self.log_global_failed(
            Self::EVENTID_GLOBAL_SET_FAILED,
            "set",
            module_name,
            global_name,
            reason,
        );
    }

    pub(crate) fn log_warning_lo_global_get_failed(
        &self,
        module_name: &[u8],
        global_name: &[u8],
        reason: Status,
    ) {
        self.log_global_failed(
            Self::EVENTID_GLOBAL_GET_FAILED,
            "get",
            module_name,
            global_name,
            reason,
        );
    }

    fn log_global_value(
        &self,
        id: FwEventIdType,
        module_name: &[u8],
        global_name: &[u8],
        value: &dyn fmt::Display,
        write_value: impl FnOnce(&mut LogBuffer) -> SerializeStatus,
    ) {
        let m = arg_str(module_name, MODULE_NAME_STRING_SIZE);
        let g = arg_str(global_name, GLOBAL_NAME_STRING_SIZE);
        self.log(
            id,
            LogSeverity::ActivityLo,
            format_args!(
                "Global value module={} name={} value={value}",
                Lossy(m),
                Lossy(g)
            ),
            |b| {
                fw_try!(ser_str(b, m, MODULE_NAME_STRING_SIZE));
                fw_try!(ser_str(b, g, GLOBAL_NAME_STRING_SIZE));
                write_value(b)
            },
        );
    }

    pub(crate) fn log_activity_lo_global_value_i32(&self, m: &[u8], g: &[u8], value: i32) {
        self.log_global_value(Self::EVENTID_GLOBAL_VALUE_I32, m, g, &value, |b| {
            b.serialize_i32_be(value)
        });
    }

    pub(crate) fn log_activity_lo_global_value_i64(&self, m: &[u8], g: &[u8], value: i64) {
        self.log_global_value(Self::EVENTID_GLOBAL_VALUE_I64, m, g, &value, |b| {
            b.serialize_i64_be(value)
        });
    }

    pub(crate) fn log_activity_lo_global_value_f32(&self, m: &[u8], g: &[u8], value: f32) {
        self.log_global_value(Self::EVENTID_GLOBAL_VALUE_F32, m, g, &value, |b| {
            b.serialize_f32_be(value)
        });
    }

    pub(crate) fn log_activity_lo_global_value_f64(&self, m: &[u8], g: &[u8], value: f64) {
        self.log_global_value(Self::EVENTID_GLOBAL_VALUE_F64, m, g, &value, |b| {
            b.serialize_f64_be(value)
        });
    }

    pub(crate) fn log_warning_hi_file_open_error(&self, file_name: &[u8], status: i32) {
        let f = arg_str(file_name, FILE_NAME_STRING_SIZE);
        self.log(
            Self::EVENTID_FILE_OPEN_ERROR,
            LogSeverity::WarningHi,
            format_args!(
                "Failed to open Wasm module file {}: status {status}",
                Lossy(f)
            ),
            |b| {
                fw_try!(ser_str(b, f, FILE_NAME_STRING_SIZE));
                b.serialize_i32_be(status)
            },
        );
    }

    pub(crate) fn log_warning_hi_sequence_file_path_too_long(
        &self,
        base_dir: &[u8],
        file_name: &[u8],
    ) {
        let d = arg_str(base_dir, FILE_NAME_STRING_SIZE);
        let f = arg_str(file_name, FILE_NAME_STRING_SIZE);
        self.log(
            Self::EVENTID_SEQUENCE_FILE_PATH_TOO_LONG,
            LogSeverity::WarningHi,
            format_args!(
                "Sequence file path too long: base dir {} + file {}",
                Lossy(d),
                Lossy(f)
            ),
            |b| {
                fw_try!(ser_str(b, d, FILE_NAME_STRING_SIZE));
                ser_str(b, f, FILE_NAME_STRING_SIZE)
            },
        );
    }

    /// C++ parity: the FPP format names the file first but substitutes the
    /// arguments in declaration order (base dir, then file name).
    pub(crate) fn log_warning_hi_sequence_file_path_not_contained(
        &self,
        base_dir: &[u8],
        file_name: &[u8],
    ) {
        let d = arg_str(base_dir, FILE_NAME_STRING_SIZE);
        let f = arg_str(file_name, FILE_NAME_STRING_SIZE);
        self.log(
            Self::EVENTID_SEQUENCE_FILE_PATH_NOT_CONTAINED,
            LogSeverity::WarningHi,
            format_args!(
                "Sequence file {} is not contained within base dir {} (path traversal rejected)",
                Lossy(d),
                Lossy(f)
            ),
            |b| {
                fw_try!(ser_str(b, d, FILE_NAME_STRING_SIZE));
                ser_str(b, f, FILE_NAME_STRING_SIZE)
            },
        );
    }

    fn log_status(&self, id: FwEventIdType, what: &str, status: Status) {
        self.log(
            id,
            LogSeverity::WarningHi,
            format_args!("{what}: {status:?}"),
            |b| ser(b, &status),
        );
    }

    pub(crate) fn log_warning_hi_module_load_failed(&self, status: Status) {
        self.log_status(
            Self::EVENTID_MODULE_LOAD_FAILED,
            "Module load failed",
            status,
        );
    }

    pub(crate) fn log_warning_hi_module_main_invoke_failed(&self, status: Status) {
        self.log_status(
            Self::EVENTID_MODULE_MAIN_INVOKE_FAILED,
            "Module main invoke failed",
            status,
        );
    }

    pub(crate) fn log_warning_hi_module_start_invoke_failed(&self, status: Status) {
        self.log_status(
            Self::EVENTID_MODULE_START_INVOKE_FAILED,
            "Module start invoke failed",
            status,
        );
    }

    pub(crate) fn log_warning_hi_too_many_blocking_commands(&self) {
        self.log(
            Self::EVENTID_TOO_MANY_BLOCKING_COMMANDS,
            LogSeverity::WarningHi,
            format_args!("Too many commands blocking on sequencer finish"),
            |_| SerializeStatus::Ok,
        );
    }

    pub(crate) fn log_warning_hi_buffer_too_large(
        &self,
        host_function: HostFunction,
        size: u32,
        max_size: u32,
    ) {
        self.log(
            Self::EVENTID_BUFFER_TOO_LARGE,
            LogSeverity::WarningHi,
            format_args!(
                "Host function {host_function:?} buffer payload size {size} larger than host maximum size {max_size}"
            ),
            |b| {
                fw_try!(ser(b, &host_function));
                fw_try!(b.serialize_u32_be(size));
                b.serialize_u32_be(max_size)
            },
        );
    }

    pub(crate) fn log_warning_hi_buffer_too_small(
        &self,
        host_function: HostFunction,
        size: u32,
        value_size: u32,
    ) {
        self.log(
            Self::EVENTID_BUFFER_TOO_SMALL,
            LogSeverity::WarningHi,
            format_args!(
                "Host function {host_function:?} buffer payload size {size} smaller than host value size {value_size}"
            ),
            |b| {
                fw_try!(ser(b, &host_function));
                fw_try!(b.serialize_u32_be(size));
                b.serialize_u32_be(value_size)
            },
        );
    }

    pub(crate) fn log_warning_hi_host_function_invalid_pointer(
        &self,
        host_function: HostFunction,
        code: Status,
    ) {
        self.log(
            Self::EVENTID_HOST_FUNCTION_INVALID_POINTER,
            LogSeverity::WarningHi,
            format_args!(
                "Failed to read guest pointer in host function {host_function:?}: {code:?}"
            ),
            |b| {
                fw_try!(ser(b, &host_function));
                ser(b, &code)
            },
        );
    }

    pub(crate) fn log_warning_hi_invalid_blocking_type_value(&self, invalid_value: i32) {
        self.log(
            Self::EVENTID_INVALID_BLOCKING_TYPE_VALUE,
            LogSeverity::WarningHi,
            format_args!("Invalid blocking type value passed to serial_recv: {invalid_value}"),
            |b| b.serialize_i32_be(invalid_value),
        );
    }

    pub(crate) fn log_warning_hi_host_function_invalid_port(
        &self,
        host_function: HostFunction,
        index: i32,
        num_ports: u32,
    ) {
        self.log(
            Self::EVENTID_HOST_FUNCTION_INVALID_PORT,
            LogSeverity::WarningHi,
            format_args!(
                "Host function {host_function:?} requested invalid serial port index {index} (have {num_ports} ports)"
            ),
            |b| {
                fw_try!(ser(b, &host_function));
                fw_try!(b.serialize_i32_be(index));
                b.serialize_u32_be(num_ports)
            },
        );
    }

    pub(crate) fn log_warning_hi_serial_port_send_failed(
        &self,
        host_function: HostFunction,
        status: i32,
    ) {
        self.log(
            Self::EVENTID_SERIAL_PORT_SEND_FAILED,
            LogSeverity::WarningHi,
            format_args!(
                "Host function {host_function:?} failed to send on serial port: serialize status {status}"
            ),
            |b| {
                fw_try!(ser(b, &host_function));
                b.serialize_i32_be(status)
            },
        );
    }

    pub(crate) fn log_warning_hi_serial_in_frame_too_large(
        &self,
        port_index: u32,
        size: u32,
        max_size: u32,
    ) {
        self.log(
            Self::EVENTID_SERIAL_IN_FRAME_TOO_LARGE,
            LogSeverity::WarningHi,
            format_args!(
                "Serial input port {port_index} dropped oversized frame of {size} bytes (queue max {max_size})"
            ),
            |b| {
                fw_try!(b.serialize_u32_be(port_index));
                fw_try!(b.serialize_u32_be(size));
                b.serialize_u32_be(max_size)
            },
        );
    }

    pub(crate) fn log_warning_hi_host_function_invalid_severity(&self, raw: i32, msg: &[u8]) {
        let m = arg_str(msg, GUEST_EVENT_MESSAGE_SIZE);
        self.log(
            Self::EVENTID_HOST_FUNCTION_INVALID_SEVERITY,
            LogSeverity::WarningHi,
            format_args!(
                "Guest event at invalid/forbidden severity '{raw}': {}",
                Lossy(m)
            ),
            |b| {
                fw_try!(b.serialize_i32_be(raw));
                ser_str(b, m, GUEST_EVENT_MESSAGE_SIZE)
            },
        );
    }

    pub(crate) fn log_activity_hi_sequence_starting(&self, index: ModuleIdx) {
        self.log(
            Self::EVENTID_SEQUENCE_STARTING,
            LogSeverity::ActivityHi,
            format_args!("Sequence at index {index} finished starting"),
            |b| b.serialize_u8_be(index),
        );
    }

    pub(crate) fn log_activity_hi_sequence_succeeded(&self, index: ModuleIdx) {
        self.log(
            Self::EVENTID_SEQUENCE_SUCCEEDED,
            LogSeverity::ActivityHi,
            format_args!("Sequence at index {index} finished successfully"),
            |b| b.serialize_u8_be(index),
        );
    }

    pub(crate) fn log_activity_hi_sequence_cancelled(
        &self,
        index: ModuleIdx,
        phase: SequencePhase,
    ) {
        self.log(
            Self::EVENTID_SEQUENCE_CANCELLED,
            LogSeverity::ActivityHi,
            format_args!("Sequence at index {index} was cancelled during {phase:?}"),
            |b| {
                fw_try!(b.serialize_u8_be(index));
                ser(b, &phase)
            },
        );
    }

    pub(crate) fn log_warning_hi_sequence_exited(
        &self,
        index: ModuleIdx,
        phase: SequencePhase,
        code: i32,
    ) {
        self.log(
            Self::EVENTID_SEQUENCE_EXITED,
            LogSeverity::WarningHi,
            format_args!(
                "Sequence at index {index} exited during {phase:?} with non-zero code {code}"
            ),
            |b| {
                fw_try!(b.serialize_u8_be(index));
                fw_try!(ser(b, &phase));
                b.serialize_i32_be(code)
            },
        );
    }

    pub(crate) fn log_warning_hi_sequence_panic(
        &self,
        index: ModuleIdx,
        phase: SequencePhase,
        code: i32,
    ) {
        self.log(
            Self::EVENTID_SEQUENCE_PANIC,
            LogSeverity::WarningHi,
            format_args!("Sequence at index {index} panicked during {phase:?} with code {code}"),
            |b| {
                fw_try!(b.serialize_u8_be(index));
                fw_try!(ser(b, &phase));
                b.serialize_i32_be(code)
            },
        );
    }

    pub(crate) fn log_warning_hi_sequence_trapped(
        &self,
        index: ModuleIdx,
        phase: SequencePhase,
        reason: TrapReason,
    ) {
        self.log(
            Self::EVENTID_SEQUENCE_TRAPPED,
            LogSeverity::WarningHi,
            format_args!("Sequence at index {index} trapped during {phase:?}: {reason:?}"),
            |b| {
                fw_try!(b.serialize_u8_be(index));
                fw_try!(ser(b, &phase));
                ser(b, &reason)
            },
        );
    }

    /// C++ parity: the FPP format substitutes the arguments in declaration
    /// order, so the text names the reason where it says "host function".
    pub(crate) fn log_warning_hi_sequence_host_failure(
        &self,
        index: ModuleIdx,
        phase: SequencePhase,
        reason: ExitReason,
        host_function: HostFunction,
    ) {
        self.log(
            Self::EVENTID_SEQUENCE_HOST_FAILURE,
            LogSeverity::WarningHi,
            format_args!(
                "Sequence at index {index} failed during {phase:?} awaiting host function {reason:?} (reason {host_function:?})"
            ),
            |b| {
                fw_try!(b.serialize_u8_be(index));
                fw_try!(ser(b, &phase));
                fw_try!(ser(b, &reason));
                ser(b, &host_function)
            },
        );
    }

    pub(crate) fn log_activity_hi_sequence_paused(&self) {
        self.log(
            Self::EVENTID_SEQUENCE_PAUSED,
            LogSeverity::ActivityHi,
            format_args!("Sequence execution paused"),
            |_| SerializeStatus::Ok,
        );
    }

    pub(crate) fn log_warning_lo_sequence_not_running(&self) {
        self.log(
            Self::EVENTID_SEQUENCE_NOT_RUNNING,
            LogSeverity::WarningLo,
            format_args!("Sequence cannot continue when interpreter is not running"),
            |_| SerializeStatus::Ok,
        );
    }

    pub(crate) fn log_warning_hi_sleep_duration_too_large(
        &self,
        host_function: HostFunction,
        micros: u64,
    ) {
        self.log(
            Self::EVENTID_SLEEP_DURATION_TOO_LARGE,
            LogSeverity::WarningHi,
            format_args!(
                "Host function {host_function:?} requested a sleep of {micros} us that overflows the host timer"
            ),
            |b| {
                fw_try!(ser(b, &host_function));
                b.serialize_u64_be(micros)
            },
        );
    }

    pub(crate) fn log_warning_hi_host_function_invalid_id(
        &self,
        host_function: HostFunction,
        id: i64,
    ) {
        self.log(
            Self::EVENTID_HOST_FUNCTION_INVALID_ID,
            LogSeverity::WarningHi,
            format_args!(
                "Host function {host_function:?} requested id {id} outside representable id range"
            ),
            |b| {
                fw_try!(ser(b, &host_function));
                b.serialize_i64_be(id)
            },
        );
    }

    pub(crate) fn log_warning_lo_cmd_response_from_old_sequence(
        &self,
        opcode: FwOpcodeType,
        response: CmdResponse,
        old_sequence_idx: u16,
        current_sequence_idx: u16,
    ) {
        self.log(
            Self::EVENTID_CMD_RESPONSE_FROM_OLD_SEQUENCE,
            LogSeverity::WarningLo,
            format_args!(
                "Command response (opcode {opcode}, response {response:?}) was from a previous sequence (old idx {old_sequence_idx}, current idx {current_sequence_idx})"
            ),
            |b| {
                fw_try!(b.serialize_u32_be(opcode));
                fw_try!(ser(b, &response));
                fw_try!(b.serialize_u16_be(old_sequence_idx));
                b.serialize_u16_be(current_sequence_idx)
            },
        );
    }

    pub(crate) fn log_warning_hi_wrong_cmd_response_index(
        &self,
        opcode: FwOpcodeType,
        response: CmdResponse,
        actual_cmd_idx: u16,
        expected_cmd_idx: u16,
    ) {
        self.log(
            Self::EVENTID_WRONG_CMD_RESPONSE_INDEX,
            LogSeverity::WarningHi,
            format_args!(
                "Command response (opcode {opcode}, response {response:?}) was for a different command instance (actual idx {actual_cmd_idx}, expected {expected_cmd_idx})"
            ),
            |b| {
                fw_try!(b.serialize_u32_be(opcode));
                fw_try!(ser(b, &response));
                fw_try!(b.serialize_u16_be(actual_cmd_idx));
                b.serialize_u16_be(expected_cmd_idx)
            },
        );
    }

    pub(crate) fn log_warning_hi_invalid_module_entrypoint(
        &self,
        module_idx: ModuleIdx,
        status: Status,
    ) {
        self.log(
            Self::EVENTID_INVALID_MODULE_ENTRYPOINT,
            LogSeverity::WarningHi,
            format_args!(
                "Module at index {module_idx} does not have a valid 'main' entrypoint: {status:?}"
            ),
            |b| {
                fw_try!(b.serialize_u8_be(module_idx));
                ser(b, &status)
            },
        );
    }

    /// `LogWarningHi` … `LogDiagnostic`: the guest's events, `{}`.
    pub(crate) fn log_guest_message(&self, severity: LogSeverity, msg: &[u8]) {
        let id = match severity {
            LogSeverity::WarningHi => Self::EVENTID_LOG_WARNING_HI,
            LogSeverity::WarningLo => Self::EVENTID_LOG_WARNING_LO,
            LogSeverity::ActivityHi => Self::EVENTID_LOG_ACTIVITY_HI,
            LogSeverity::ActivityLo => Self::EVENTID_LOG_ACTIVITY_LO,
            LogSeverity::Diagnostic => Self::EVENTID_LOG_DIAGNOSTIC,
            // FATAL and COMMAND are not guest-emittable (the host function
            // reports them as HostFunctionInvalidSeverity).
            LogSeverity::Fatal | LogSeverity::Command => {
                fprime_fw::fw_assert!(false, severity.as_repr());
                return;
            }
        };
        let m = arg_str(msg, GUEST_EVENT_MESSAGE_SIZE);
        self.log(id, severity, format_args!("{}", Lossy(m)), |b| {
            ser_str(b, m, GUEST_EVENT_MESSAGE_SIZE)
        });
    }
}
