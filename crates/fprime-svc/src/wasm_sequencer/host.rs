//! The `fprime_v1` host module: the functions a sequence imports
//! (`WasmSequencerHost.cpp`).
//!
//! Each function runs inside the interpreter, on the component thread,
//! with the store locked. It validates its arguments (reporting and
//! trapping on a bad one), records the call as the pending host function
//! and pauses the engine; the interpreter machine then dispatches it
//! ([`super::interpreter`]). `exit` and `panic` record the exit status and
//! trap instead.
//!
//! | import | signature | returns |
//! |--------|-----------|---------|
//! | `exit(code)` | `(i32) -> ()` | traps; the sequence ends with `code` |
//! | `panic(code)` | `(i32) -> ()` | traps; `SequencePanic` |
//! | `args(ptr, len)` | `(i32, i32) -> i32` | the argument size |
//! | `time(ptr, len)` | `(i32, i32) -> ()` | writes the serialized `Fw::Time` (11 bytes) |
//! | `tlm(id, time_ptr, time_len, val_ptr, val_len)` | `(i64, i32, i32, i32, i32) -> i32` | `Fw::TlmValid` |
//! | `prm(id, ptr, len)` | `(i64, i32, i32) -> i32` | `Fw::ParamValid` |
//! | `cmd(ptr, len)` | `(i32, i32) -> i32` | the `Fw::CmdResponse` |
//! | `event(severity, ptr, len)` | `(i32, i32, i32) -> ()` | |
//! | `rsleep(us)` / `asleep(us)` | `(i64) -> ()` | after the sleep |
//! | `serial_send(port, ptr, len)` | `(i32, i32, i32) -> ()` | |
//! | `serial_recv(port, ptr, len, size_ptr, block)` | `(i32, i32, i32, i32, i32) -> i32` | 0 OK, 1 queue empty |

use super::{
    BlockState, HostFunction, MemoryGrowFailReason, NUM_SERIAL_IN_PORTS, NUM_SERIAL_OUT_PORTS,
    PendingArgs, PendingHostFunction, WasmSequencer, lock,
};
use fprime_config::{
    FW_COM_BUFFER_MAX_SIZE, FW_LOG_STRING_MAX_SIZE, FwChanIdType, FwPacketDescriptorType,
    FwPrmIdType,
};
use fprime_fw::{Time, fw_assert};
use fprime_spacewasm::spacewasm::Value;
use fprime_spacewasm::{GrowRejected, GuestObserver, HostCallResult, HostModules};
use std::sync::Weak;

/// Reports the guest allocator's refusals as `MemoryGrowRejected` (C++
/// `guestRealloc` logs it itself).
pub(crate) struct GuestMemoryObserver {
    pub(crate) component: Weak<WasmSequencer>,
}

impl GuestObserver for GuestMemoryObserver {
    fn memory_grow_rejected(&self, reason: GrowRejected, requested_size: u64) {
        if let Some(c) = self.component.upgrade() {
            let reason = match reason {
                GrowRejected::NotLastAllocation => MemoryGrowFailReason::NotLastAllocation,
                GrowRejected::InsufficientPoolSpace => MemoryGrowFailReason::InsufficientPoolSpace,
            };
            c.log_warning_lo_memory_grow_rejected(reason, requested_size);
        }
    }
}

/// The `i32` argument at `i` (spacewasm validated the signature, so the
/// type is a programming error otherwise — C++ asserts the tag).
fn arg_i32(args: &[Value], i: usize) -> i32 {
    match args.get(i) {
        Some(Value::I32(v)) => *v,
        _ => {
            fw_assert!(false, i as i64);
            0
        }
    }
}

/// The `i64` argument at `i`.
fn arg_i64(args: &[Value], i: usize) -> i64 {
    match args.get(i) {
        Some(Value::I64(v)) => *v,
        _ => {
            fw_assert!(false, i as i64);
            0
        }
    }
}

type HostFn = fn(&WasmSequencer, &[Value]) -> HostCallResult;

impl WasmSequencer {
    /// C++ `hostFprimeV1`: the `fprime_v1` host module (12 functions, no
    /// globals).
    pub(crate) fn host_fprime_v1(&self, host: &mut HostModules<'_>) {
        let module_idx = match host.add_module("fprime_v1", 12, 0) {
            Ok(idx) => idx,
            Err(e) => {
                fw_assert!(false, super::status::error_status(&e).as_repr());
                return;
            }
        };
        let functions: [(&str, &str, &str, HostFn); 12] = [
            ("exit", "i", "", Self::wasm_exit),
            ("panic", "i", "", Self::wasm_panic),
            ("args", "ii", "i", Self::wasm_args),
            ("time", "ii", "", Self::wasm_time),
            ("tlm", "Iiiii", "i", Self::wasm_read_telemetry),
            ("prm", "Iii", "i", Self::wasm_read_parameter),
            ("cmd", "ii", "i", Self::wasm_command),
            ("event", "iii", "", Self::wasm_event),
            ("rsleep", "I", "", Self::wasm_rsleep),
            ("asleep", "I", "", Self::wasm_asleep),
            ("serial_send", "iii", "", Self::wasm_serial_out),
            ("serial_recv", "iiiii", "i", Self::wasm_serial_recv),
        ];
        for (name, params, returns, f) in functions {
            // The trampoline (C++ `hostFunctionTrampoline`): the store holds
            // a weak reference, so it never keeps its component alive.
            let component = self.weak_self.clone();
            let status =
                host.add_function(
                    module_idx,
                    name,
                    params,
                    returns,
                    move |args| match component.upgrade() {
                        Some(c) => f(&c, args),
                        None => HostCallResult::Trap,
                    },
                );
            if let Err(e) = status {
                fw_assert!(false, super::status::error_status(&e).as_repr());
            }
        }
    }

    /// Record `pending` as the call to dispatch (C++ asserts nothing is
    /// pending on entry to every host function).
    fn pend(&self, kind: HostFunction, args: PendingArgs) -> HostCallResult {
        let mut st = lock(&self.state);
        fw_assert!(!st.pending.is_pending());
        st.pending = PendingHostFunction { kind, args };
        // Always pause so the state machine processes the request.
        HostCallResult::Pause
    }

    fn assert_not_pending(&self) {
        fw_assert!(!lock(&self.state).pending.is_pending());
    }

    /// `exit(code)`: end the sequence with `code` (0 succeeds).
    fn wasm_exit(&self, args: &[Value]) -> HostCallResult {
        fw_assert!(args.len() == 1, args.len() as i64);
        let code = arg_i32(args, 0);
        let mut st = lock(&self.state);
        fw_assert!(!st.pending.is_pending());
        st.exit.reason = super::ExitReason::HostExit;
        st.exit.code = code;
        HostCallResult::Trap
    }

    /// `panic(code)`: fail the sequence with `code`.
    fn wasm_panic(&self, args: &[Value]) -> HostCallResult {
        fw_assert!(args.len() == 1, args.len() as i64);
        let code = arg_i32(args, 0);
        let mut st = lock(&self.state);
        fw_assert!(!st.pending.is_pending());
        st.exit.reason = super::ExitReason::HostPanic;
        st.exit.code = code;
        HostCallResult::Trap
    }

    /// `args(ptr, len)`.
    fn wasm_args(&self, args: &[Value]) -> HostCallResult {
        fw_assert!(args.len() == 2, args.len() as i64);
        let ptr = arg_i32(args, 0) as u32;
        let len = arg_i32(args, 1) as u32;
        self.pend(HostFunction::Args, PendingArgs::Args { ptr, len })
    }

    /// `time(ptr, len)`: the buffer must be exactly a serialized time.
    fn wasm_time(&self, args: &[Value]) -> HostCallResult {
        self.assert_not_pending();
        fw_assert!(args.len() == 2, args.len() as i64);
        let ptr = arg_i32(args, 0) as u32;
        let len = arg_i32(args, 1) as u32;
        let time_size = Time::SERIALIZED_SIZE as u32;
        if len < time_size {
            self.log_warning_hi_buffer_too_small(HostFunction::Time, len, time_size);
            HostCallResult::Trap
        } else if len > time_size {
            self.log_warning_hi_buffer_too_large(HostFunction::Time, len, time_size);
            HostCallResult::Trap
        } else {
            self.pend(HostFunction::Time, PendingArgs::Time { ptr, len })
        }
    }

    /// `tlm(id, time_ptr, time_len, value_ptr, value_len)`.
    fn wasm_read_telemetry(&self, args: &[Value]) -> HostCallResult {
        self.assert_not_pending();
        fw_assert!(args.len() == 5, args.len() as i64);
        // FwChanIdType always travels as 64 bits
        let id = arg_i64(args, 0);
        let time_ptr = arg_i32(args, 1) as u32;
        let time_len = arg_i32(args, 2) as u32;
        let value_ptr = arg_i32(args, 3) as u32;
        let value_len = arg_i32(args, 4) as u32;
        // getTlmChan is not a required port: guard against an unconnected
        // deployment (the generated invoker would assert).
        if !self.get_tlm_chan.is_connected() {
            self.log_warning_hi_host_function_invalid_port(HostFunction::Telemetry, 0, 0);
            return HostCallResult::Trap;
        }
        let time_size = Time::SERIALIZED_SIZE as u32;
        if time_len < time_size {
            self.log_warning_hi_buffer_too_small(HostFunction::Telemetry, time_len, time_size);
            HostCallResult::Trap
        } else if time_len > time_size {
            self.log_warning_hi_buffer_too_large(HostFunction::Telemetry, time_len, time_size);
            HostCallResult::Trap
        } else if id < 0 || id as u64 > u64::from(FwChanIdType::MAX) {
            // A guest id that does not fit FwChanIdType would alias a
            // different channel if cast: reject it.
            self.log_warning_hi_host_function_invalid_id(HostFunction::Telemetry, id);
            HostCallResult::Trap
        } else {
            self.pend(
                HostFunction::Telemetry,
                PendingArgs::Telemetry {
                    chan_id: id as FwChanIdType,
                    time_ptr,
                    time_len,
                    value_ptr,
                    value_len,
                },
            )
        }
    }

    /// `prm(id, ptr, len)`.
    fn wasm_read_parameter(&self, args: &[Value]) -> HostCallResult {
        self.assert_not_pending();
        fw_assert!(args.len() == 3, args.len() as i64);
        let id = arg_i64(args, 0);
        let ptr = arg_i32(args, 1) as u32;
        let len = arg_i32(args, 2) as u32;
        if !self.get_param.is_connected() {
            self.log_warning_hi_host_function_invalid_port(HostFunction::Parameter, 0, 0);
            return HostCallResult::Trap;
        }
        if id < 0 || id as u64 > u64::from(FwPrmIdType::MAX) {
            self.log_warning_hi_host_function_invalid_id(HostFunction::Parameter, id);
            return HostCallResult::Trap;
        }
        self.pend(
            HostFunction::Parameter,
            PendingArgs::Parameter {
                prm_id: id as FwPrmIdType,
                ptr,
                len,
            },
        )
    }

    /// `cmd(ptr, len)`: the encoded command (opcode and arguments, without
    /// the packet descriptor).
    fn wasm_command(&self, args: &[Value]) -> HostCallResult {
        self.assert_not_pending();
        fw_assert!(args.len() == 2, args.len() as i64);
        let ptr = arg_i32(args, 0) as u32;
        let len = arg_i32(args, 1) as u32;
        if !self.cmd_out.is_connected() {
            self.log_warning_hi_host_function_invalid_port(HostFunction::Command, 0, 0);
            return HostCallResult::Trap;
        }
        let max_payload = (FW_COM_BUFFER_MAX_SIZE - size_of::<FwPacketDescriptorType>()) as u32;
        if len > max_payload {
            self.log_warning_hi_buffer_too_large(HostFunction::Command, len, max_payload);
            return HostCallResult::Trap;
        }
        self.pend(HostFunction::Command, PendingArgs::Command { ptr, len })
    }

    /// `event(severity, ptr, len)`: the severity is validated at dispatch,
    /// where a forbidden one is reported with the message.
    fn wasm_event(&self, args: &[Value]) -> HostCallResult {
        fw_assert!(args.len() == 3, args.len() as i64);
        let raw_severity = arg_i32(args, 0) as u32;
        let msg_ptr = arg_i32(args, 1) as u32;
        let msg_len = (arg_i32(args, 2) as u32).min(FW_LOG_STRING_MAX_SIZE as u32);
        self.pend(
            HostFunction::Event,
            PendingArgs::Event {
                raw_severity,
                msg_ptr,
                msg_len,
            },
        )
    }

    /// `rsleep(us)`: the whole seconds must fit `Fw::Time`.
    fn wasm_rsleep(&self, args: &[Value]) -> HostCallResult {
        self.assert_not_pending();
        fw_assert!(args.len() == 1, args.len() as i64);
        let us = arg_i64(args, 0) as u64;
        if us / 1_000_000 > u64::from(u32::MAX) {
            self.log_warning_hi_sleep_duration_too_large(HostFunction::Rsleep, us);
            return HostCallResult::Trap;
        }
        self.pend(HostFunction::Rsleep, PendingArgs::Rsleep { us })
    }

    /// `asleep(us)`: the whole seconds must fit `Fw::Time`.
    fn wasm_asleep(&self, args: &[Value]) -> HostCallResult {
        self.assert_not_pending();
        fw_assert!(args.len() == 1, args.len() as i64);
        let us = arg_i64(args, 0) as u64;
        if us / 1_000_000 > u64::from(u32::MAX) {
            self.log_warning_hi_sleep_duration_too_large(HostFunction::Asleep, us);
            return HostCallResult::Trap;
        }
        self.pend(HostFunction::Asleep, PendingArgs::Asleep { us })
    }

    /// `serial_send(port, ptr, len)`.
    fn wasm_serial_out(&self, args: &[Value]) -> HostCallResult {
        self.assert_not_pending();
        fw_assert!(args.len() == 3, args.len() as i64);
        let index = arg_i32(args, 0);
        let ptr = arg_i32(args, 1) as u32;
        let len = arg_i32(args, 2) as u32;
        // The payload must fit the copy-out buffer; an unconfigured buffer
        // (serial_out_max 0) traps even a zero-length send.
        let capacity = lock(&self.state).serial_out_buffer.len();
        if len as usize > capacity || capacity == 0 {
            self.log_warning_hi_buffer_too_large(HostFunction::SerialOut, len, capacity as u32);
            return HostCallResult::Trap;
        }
        if index < 0
            || index >= NUM_SERIAL_OUT_PORTS as i32
            || !self.serial_out[index as usize].is_connected()
        {
            self.log_warning_hi_host_function_invalid_port(
                HostFunction::SerialOut,
                index,
                NUM_SERIAL_OUT_PORTS as u32,
            );
            return HostCallResult::Trap;
        }
        self.pend(
            HostFunction::SerialOut,
            PendingArgs::SerialOut {
                index: index as u32,
                ptr,
                len,
            },
        )
    }

    /// `serial_recv(port, data_ptr, data_len, actual_size_ptr, block)`.
    fn wasm_serial_recv(&self, args: &[Value]) -> HostCallResult {
        self.assert_not_pending();
        fw_assert!(args.len() == 5, args.len() as i64);
        let index = arg_i32(args, 0);
        let data_ptr = arg_i32(args, 1) as u32;
        let data_size = arg_i32(args, 2) as u32;
        let actual_size_ptr = arg_i32(args, 3) as u32;
        let block_type = arg_i32(args, 4);
        if index < 0 || index >= NUM_SERIAL_IN_PORTS as i32 {
            self.log_warning_hi_host_function_invalid_port(
                HostFunction::SerialRecv,
                index,
                NUM_SERIAL_IN_PORTS as u32,
            );
            return HostCallResult::Trap;
        }
        let blocking_type = match u8::try_from(block_type)
            .ok()
            .and_then(|raw| BlockState::try_from(raw).ok())
        {
            Some(b) => b,
            None => {
                self.log_warning_hi_invalid_blocking_type_value(block_type);
                return HostCallResult::Trap;
            }
        };
        self.pend(
            HostFunction::SerialRecv,
            PendingArgs::SerialRecv {
                index: index as u32,
                data_ptr,
                data_size,
                actual_size_ptr,
                blocking_type,
            },
        )
    }
}
