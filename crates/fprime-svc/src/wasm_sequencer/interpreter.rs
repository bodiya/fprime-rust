//! The interpreter state machine's actions and guards
//! (`WasmSequencerInterpreter.cpp`): running the engine in fuel-bounded
//! slices, the exit status, and the dispatch of each pending host function
//! (the arms of C++ `dispatchPendingHostFunction`).

use super::interpreter_sm::{Handlers, Signal};
use super::status::{Trap, map_trap_reason, memory_status};
use super::{
    BlockState, CommandRequest, ExitReason, ExitStatus, HostFunction, NUM_SERIAL_IN_PORTS,
    PendingArgs, RequestContext, TrapReason, WasmSequencer, lock,
};
use fprime_config::{FW_COM_BUFFER_MAX_SIZE, FW_LOG_STRING_MAX_SIZE, FwEnumStoreType, FwIndexType};
use fprime_fw::{
    CmdResponse, ComBuffer, ComPacketType, Endianness, ExtBuf, LengthMode, LinearBuffer,
    LogSeverity, ParamBuffer, SerBuf, Serialize, SerializeStatus, Time, TimeComparison, TlmBuffer,
    fw_assert,
};
use fprime_spacewasm::Error;
use fprime_spacewasm::spacewasm::{InterpreterResult, ValType, Value};

/// `serial_recv` result: a message was delivered.
const FPRIME_SERIAL_RECV_QUEUE_STATUS_OK: i32 = 0;
/// `serial_recv` result: non-blocking and the queue is empty.
const FPRIME_SERIAL_RECV_QUEUE_STATUS_EMPTY: i32 = 1;
/// The chunk size `serial_recv` copies a message in.
const SERIAL_RECV_CHUNK_SIZE: usize = 32;

/// `Fw::Time::add(seconds, useconds)` with the C++ `U32` wrap-around (the
/// Rust `add_duration` asserts on overflow instead).
fn time_add_wrapping(time: &Time, seconds: u32, useconds: u32) -> Time {
    let mut out = *time;
    let total_useconds = time.get_useconds() + useconds;
    let seconds = time
        .get_seconds()
        .wrapping_add(seconds)
        .wrapping_add(total_useconds / 1_000_000);
    out.set(seconds, total_useconds % 1_000_000);
    out
}

impl WasmSequencer {
    /// C++ `readGuestMemory`: copy guest memory at `addr` into `dst`,
    /// reporting `HostFunctionInvalidPointer` on a bad pointer.
    fn read_guest_memory(&self, kind: HostFunction, addr: u32, dst: &mut [u8]) -> bool {
        match self.with_engine(|e| e.mem_read(addr, dst)) {
            Ok(()) => true,
            Err(e) => {
                self.report_bad_pointer(kind, &e);
                false
            }
        }
    }

    /// C++ `writeGuestMemory`.
    fn write_guest_memory(&self, kind: HostFunction, addr: u32, src: &[u8]) -> bool {
        match self.with_engine(|e| e.mem_write(addr, src)) {
            Ok(()) => true,
            Err(e) => {
                self.report_bad_pointer(kind, &e);
                false
            }
        }
    }

    fn report_bad_pointer(&self, kind: HostFunction, e: &Error) {
        let status = match e {
            Error::Memory(m) => memory_status(m),
            other => super::status::error_status(other),
        };
        self.log_warning_hi_host_function_invalid_pointer(kind, status);
    }

    fn pending_args(&self) -> PendingArgs {
        lock(&self.state).pending.args
    }

    /// COMMAND: forward the encoded command from guest memory to the
    /// command dispatcher, with the command UID as its context.
    fn dispatch_command(&self) {
        let PendingArgs::Command { ptr, len } = self.pending_args() else {
            fw_assert!(false);
            return;
        };
        let header = size_of::<fprime_config::FwPacketDescriptorType>();
        let mut bytes = [0u8; FW_COM_BUFFER_MAX_SIZE];
        // `len` fits (the host function checked it).
        let len = len as usize;
        fw_assert!(len <= FW_COM_BUFFER_MAX_SIZE - header, len as i64);
        if !self.read_guest_memory(HostFunction::Command, ptr, &mut bytes[..len]) {
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        let mut cmd = ComBuffer::new();
        let status = cmd.serialize_u16_be(ComPacketType::FwPacketCommand.as_repr());
        fw_assert!(status.is_ok(), status as i32);
        let status = cmd.serialize_bytes(&bytes[..len], LengthMode::OmitLength, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let now = self.evt.time_get();
        let cmd_uid = {
            let mut st = lock(&self.state);
            st.tlm.commands_dispatched += 1;
            // Start the host-function timeout clock: the interpreter waits
            // for the command response.
            st.host_function_start = Some(now);
            st.make_cmd_uid()
        };
        let p = self.cmd_out.get();
        p.target.invoke(p.port_num, &mut cmd, cmd_uid);
    }

    /// TELEMETRY: read the channel and write its time and value into guest
    /// memory; resume with the `TlmValid`.
    fn dispatch_telemetry(&self) {
        let PendingArgs::Telemetry {
            chan_id,
            time_ptr,
            time_len,
            value_ptr,
            value_len,
        } = self.pending_args()
        else {
            fw_assert!(false);
            return;
        };
        let mut time = Time::default();
        let mut value = TlmBuffer::new();
        let p = self.get_tlm_chan.get();
        let valid = p.target.invoke(p.port_num, chan_id, &mut time, &mut value);
        fw_assert!(time_len as usize == Time::SERIALIZED_SIZE, time_len as i64);
        let mut time_buf = LinearBuffer::<{ Time::SERIALIZED_SIZE }>::new();
        let status = time.serialize_to(&mut time_buf, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        if !self.write_guest_memory(HostFunction::Telemetry, time_ptr, time_buf.as_slice()) {
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        let size = value.as_slice().len();
        if size > value_len as usize {
            self.log_warning_hi_buffer_too_small(HostFunction::Telemetry, value_len, size as u32);
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        if !self.write_guest_memory(HostFunction::Telemetry, value_ptr, value.as_slice()) {
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        self.itp()
            .send_signal_hostResumeI32(i32::from(valid.as_repr()));
    }

    /// PARAMETER: read the parameter and write it into guest memory; resume
    /// with the `ParamValid`.
    fn dispatch_parameter(&self) {
        let PendingArgs::Parameter { prm_id, ptr, len } = self.pending_args() else {
            fw_assert!(false);
            return;
        };
        let mut value = ParamBuffer::new();
        let p = self.get_param.get();
        let prm_status = p.target.invoke(p.port_num, prm_id, &mut value);
        let size = value.as_slice().len();
        if size > len as usize {
            self.log_warning_hi_buffer_too_small(HostFunction::Parameter, len, size as u32);
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        if !self.write_guest_memory(HostFunction::Parameter, ptr, value.as_slice()) {
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        self.itp()
            .send_signal_hostResumeI32(i32::from(prm_status.as_repr()));
    }

    /// EVENT: emit the guest's message at the guest's severity. FATAL and
    /// COMMAND are forbidden (FATAL would let guest code trigger the
    /// FatalHandler; COMMAND belongs to the command dispatcher): those and
    /// unknown severities are reported as `HostFunctionInvalidSeverity`,
    /// and the guest continues either way.
    fn dispatch_event(&self) {
        let PendingArgs::Event {
            raw_severity,
            msg_ptr,
            msg_len,
        } = self.pending_args()
        else {
            fw_assert!(false);
            return;
        };
        fw_assert!(msg_len as usize <= FW_LOG_STRING_MAX_SIZE, msg_len as i64);
        let mut storage = [0u8; FW_LOG_STRING_MAX_SIZE];
        let msg = &mut storage[..msg_len as usize];
        if !self.read_guest_memory(HostFunction::Event, msg_ptr, msg) {
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        let msg = &storage[..msg_len as usize];
        let raw = raw_severity as i32;
        let severity = match u8::try_from(raw)
            .ok()
            .and_then(|r| LogSeverity::try_from(r).ok())
        {
            Some(
                s @ (LogSeverity::WarningHi
                | LogSeverity::WarningLo
                | LogSeverity::ActivityHi
                | LogSeverity::ActivityLo
                | LogSeverity::Diagnostic),
            ) => Some(s),
            _ => None,
        };
        match severity {
            Some(s) => self.log_guest_message(s, msg),
            None => self.log_warning_hi_host_function_invalid_severity(raw, msg),
        }
        self.itp().send_signal_hostResume();
    }

    /// RSLEEP: arm the wake timer `us` from now (saturating at the largest
    /// time when the deadline's seconds overflow `U32`).
    fn dispatch_relative_sleep(&self) {
        let PendingArgs::Rsleep { us } = self.pending_args() else {
            fw_assert!(false);
            return;
        };
        let seconds = (us / 1_000_000) as u32;
        let useconds = (us % 1_000_000) as u32;
        let now = self.evt.time_get();
        let deadline_seconds = u64::from(now.get_seconds())
            + u64::from(seconds)
            + (u64::from(now.get_useconds()) + u64::from(useconds)) / 1_000_000;
        let mut timer = now;
        if deadline_seconds > u64::from(u32::MAX) {
            timer.set(u32::MAX, 999_999);
        } else {
            timer.add_duration(seconds, useconds);
        }
        lock(&self.state).pending_timer = Some(timer);
    }

    /// ASLEEP: arm the wake timer at `us` since the epoch, in the current
    /// time base and context.
    fn dispatch_absolute_sleep(&self) {
        let PendingArgs::Asleep { us } = self.pending_args() else {
            fw_assert!(false);
            return;
        };
        let seconds = (us / 1_000_000) as u32;
        let useconds = (us % 1_000_000) as u32;
        let mut timer = self.evt.time_get();
        timer.set(seconds, useconds);
        lock(&self.state).pending_timer = Some(timer);
    }

    /// ARGS: write the sequence arguments into guest memory; resume with
    /// their size.
    fn dispatch_args(&self) {
        let PendingArgs::Args { ptr, len } = self.pending_args() else {
            fw_assert!(false);
            return;
        };
        let args = lock(&self.state).args;
        let capacity = args.buffer.0.len() as u64;
        if args.size > capacity {
            self.log_warning_hi_buffer_too_large(
                HostFunction::Args,
                args.size as u32,
                capacity as u32,
            );
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        if args.size > u64::from(len) {
            // Too many argument bytes: the copy would overrun the guest's
            // buffer.
            self.log_warning_hi_buffer_too_small(HostFunction::Args, len, args.size as u32);
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        let bytes = &args.buffer.0[..args.size as usize];
        if !self.write_guest_memory(HostFunction::Args, ptr, bytes) {
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        self.itp().send_signal_hostResumeI32(args.size as i32);
    }

    /// TIME: write the current time into guest memory.
    fn dispatch_time(&self) {
        let PendingArgs::Time { ptr, len } = self.pending_args() else {
            fw_assert!(false);
            return;
        };
        let time = self.evt.time_get();
        fw_assert!(len as usize == Time::SERIALIZED_SIZE, len as i64);
        let mut time_buf = LinearBuffer::<{ Time::SERIALIZED_SIZE }>::new();
        let status = time.serialize_to(&mut time_buf, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        if !self.write_guest_memory(HostFunction::Time, ptr, time_buf.as_slice()) {
            self.itp().send_signal_hostResponseFailure();
            return;
        }
        self.itp().send_signal_hostResume();
    }

    /// SERIAL_OUT: copy the payload out of guest memory and invoke the
    /// serial output port.
    fn dispatch_serial_out(&self) {
        let PendingArgs::SerialOut { index, ptr, len } = self.pending_args() else {
            fw_assert!(false);
            return;
        };
        // Borrow the copy-out buffer for the duration (a move, no
        // allocation), so no lock is held across the port call.
        let mut buffer = std::mem::take(&mut lock(&self.state).serial_out_buffer);
        let len = len as usize;
        fw_assert!(len <= buffer.len(), len as i64);
        let ok = self.read_guest_memory(HostFunction::SerialOut, ptr, &mut buffer[..len]);
        let status = if ok {
            let p = self.serial_out[index as usize].get();
            let mut ser = ExtBuf::with_len(&mut buffer, len);
            Some(p.target.invoke_serial(p.port_num, &mut ser))
        } else {
            None
        };
        lock(&self.state).serial_out_buffer = buffer;
        match status {
            None => self.itp().send_signal_hostResponseFailure(),
            Some(SerializeStatus::Ok) => self.itp().send_signal_hostResume(),
            Some(status) => {
                self.log_warning_hi_serial_port_send_failed(HostFunction::SerialOut, status as i32);
                self.itp().send_signal_hostResponseFailure();
            }
        }
    }

    /// SERIAL_RECV: deliver a queued message now, or block (arming the
    /// host-function timeout), or report an empty queue.
    fn dispatch_serial_recv(&self) {
        let PendingArgs::SerialRecv {
            index,
            blocking_type,
            ..
        } = self.pending_args()
        else {
            fw_assert!(false);
            return;
        };
        let port_num = index as FwIndexType;
        fw_assert!((index as usize) < NUM_SERIAL_IN_PORTS, port_num);
        let available = {
            let serial_in = lock(&self.serial_in);
            serial_in.queues[index as usize]
                .as_ref()
                .is_some_and(|q| q.get_allocated_size() > 0)
        };
        if available {
            // Pull the data and wake up without blocking.
            self.itp().send_signal_serialInMessage(port_num);
            return;
        }
        match blocking_type {
            BlockState::Block => {
                // Wait for a message (or the timeout).
                let now = self.evt.time_get();
                lock(&self.state).host_function_start = Some(now);
            }
            BlockState::NoBlock => {
                self.itp()
                    .send_signal_hostResumeI32(FPRIME_SERIAL_RECV_QUEUE_STATUS_EMPTY);
            }
        }
    }

    /// The `dequeueSerialAndResume` action body: copy the next message of
    /// the pending `serial_recv`'s queue into guest memory and resume with
    /// `OK`. Returns whether it succeeded.
    fn dequeue_serial_and_resume(&self) -> bool {
        let PendingArgs::SerialRecv {
            index,
            data_ptr,
            data_size,
            actual_size_ptr,
            ..
        } = self.pending_args()
        else {
            fw_assert!(false);
            return false;
        };
        fw_assert!((index as usize) < NUM_SERIAL_IN_PORTS, index as i64);
        let mut guard = lock(&self.serial_in);
        let Some(queue) = guard.queues[index as usize].as_mut() else {
            // An unconfigured queue never signals a message.
            fw_assert!(false, index as i64);
            return false;
        };
        let mut msg_size = 0u32;
        let status = queue.peek_u32_be(&mut msg_size, 0);
        fw_assert!(status.is_ok(), status as i32);
        if msg_size > data_size {
            self.log_warning_hi_buffer_too_small(HostFunction::SerialRecv, data_size, msg_size);
            // The frame never fits this guest buffer: drop it so it does
            // not poison every later serial_recv on this port.
            let status = queue.rotate(size_of::<u32>() + msg_size as usize);
            fw_assert!(status.is_ok(), status as i32);
            return false;
        }
        // The queue holds [U32 size][payload]; skip the size prefix.
        let payload_start = size_of::<u32>();
        let frame_size = payload_start + msg_size as usize;
        // The size goes to the guest little-endian.
        if !self.write_guest_memory(
            HostFunction::SerialRecv,
            actual_size_ptr,
            &msg_size.to_le_bytes(),
        ) {
            return false;
        }
        // Copy the payload in chunks.
        let mut scratch = [0u8; SERIAL_RECV_CHUNK_SIZE];
        let mut offset = payload_start;
        while offset + SERIAL_RECV_CHUNK_SIZE <= frame_size {
            let status = queue.peek_bytes(&mut scratch, offset);
            fw_assert!(status.is_ok(), status as i32);
            let addr = data_ptr.wrapping_add((offset - payload_start) as u32);
            if !self.write_guest_memory(HostFunction::SerialRecv, addr, &scratch) {
                return false;
            }
            offset += SERIAL_RECV_CHUNK_SIZE;
        }
        if offset < frame_size {
            let remaining = frame_size - offset;
            fw_assert!(
                remaining < SERIAL_RECV_CHUNK_SIZE,
                frame_size as i64,
                offset as i64,
                SERIAL_RECV_CHUNK_SIZE as i64
            );
            let status = queue.peek_bytes(&mut scratch[..remaining], offset);
            fw_assert!(status.is_ok(), status as i32);
            let addr = data_ptr.wrapping_add((offset - payload_start) as u32);
            if !self.write_guest_memory(HostFunction::SerialRecv, addr, &scratch[..remaining]) {
                return false;
            }
        }
        // Dequeue the message now that it is fully copied.
        let status = queue.rotate(frame_size);
        fw_assert!(status.is_ok(), status as i32);
        drop(guard);
        let resumed =
            self.with_engine(|e| e.resume_value(Value::I32(FPRIME_SERIAL_RECV_QUEUE_STATUS_OK)));
        fw_assert!(resumed.is_ok());
        true
    }
}

#[allow(non_snake_case)]
impl Handlers for WasmSequencer {
    fn action_cmdReplyOK(&self, _sm_id: FwEnumStoreType, _signal: Signal, value: &CommandRequest) {
        self.cmd
            .cmd_response(value.opcode, value.cmd_seq, CmdResponse::Ok);
    }

    fn action_signalEntered(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        self.itp().send_signal_entered();
    }

    /// Run the engine for `INSTRUCTION_FUEL` instructions (at least 1, so
    /// the queue is not spun forever) and signal the outcome.
    fn action_spin(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        let (fuel_param, _valid) = self.param_get_instruction_fuel();
        let fuel = if fuel_param == 0 { 1 } else { fuel_param };
        let fuel = usize::try_from(fuel).unwrap_or(usize::MAX);
        let (run, result) = self.with_engine(|e| {
            let run = e.run(fuel);
            let result = match run {
                Some(InterpreterResult::Finished) => Some(e.get_result(ValType::I32)),
                _ => None,
            };
            (run, result)
        });
        match run {
            Some(InterpreterResult::Finished) => match result {
                // A void main reports success.
                Some(Err(Error::NotFound)) => self.itp().send_signal_interpreterFinished(0),
                Some(Ok(Value::I32(code))) => self.itp().send_signal_interpreterFinished(code),
                _ => fw_assert!(false),
            },
            Some(InterpreterResult::Trap(trap)) => self.signal_trap(Trap::Reason(trap)),
            // An idle engine reports a trap with no reason.
            None => self.signal_trap(Trap::None),
            Some(InterpreterResult::Pause) => {
                self.itp().send_signal_interpreterHostFunctionNeedsPause();
            }
            Some(InterpreterResult::OutOfFuel) => self.itp().send_signal_interpreterOutOfFuel(),
        }
    }

    fn action_reset(&self, _sm_id: FwEnumStoreType, signal: Signal) {
        if self.is_store_created() {
            self.with_engine(|e| e.reset());
        } else {
            fw_assert!(
                signal == Signal::__FPRIME_INITIAL_TRANSITION,
                signal.as_repr()
            );
        }
    }

    fn action_clearExitStatus(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).exit = ExitStatus::default();
    }

    fn action_setExitReason_INTERPRETER_FINISHED(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).exit.reason = ExitReason::InterpreterFinished;
    }

    /// Only when nothing more specific (exit / panic) was recorded.
    fn action_setExitReason_INTERPRETER_TRAP(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        let mut st = lock(&self.state);
        if st.exit.reason == ExitReason::Unknown {
            st.exit.reason = ExitReason::InterpreterTrap;
        }
    }

    fn action_setExitReason_REPLY_TIMEOUT(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).exit.reason = ExitReason::ReplyTimeout;
    }

    fn action_setExitReason_HOST_FAILURE(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).exit.reason = ExitReason::HostFailure;
    }

    fn action_setExitReason_TIMER_INCOMPARABLE(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).exit.reason = ExitReason::TimerIncomparable;
    }

    fn action_setExitReason_UNEXPECTED_REPLY(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).exit.reason = ExitReason::UnexpectedReply;
    }

    fn action_setExitReason_CANCEL(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).exit.reason = ExitReason::Cancel;
    }

    fn action_setExitCode(&self, _sm_id: FwEnumStoreType, _signal: Signal, value: i32) {
        lock(&self.state).exit.code = value;
    }

    fn action_setTrapReason(&self, _sm_id: FwEnumStoreType, _signal: Signal, value: TrapReason) {
        lock(&self.state).exit.last_trap_reason = value;
    }

    fn action_setLastHostFunction(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        let mut st = lock(&self.state);
        st.exit.last_host_function = st.pending.kind;
    }

    /// Tell the controller the engine finished; the outcome is in the exit
    /// status.
    fn action_finish(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        let context = lock(&self.state).executing_context.unwrap_or_default();
        self.ctl().send_signal_engineFinished(&context);
    }

    fn action_reportPaused(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        self.log_activity_hi_sequence_paused();
    }

    fn action_clearPause(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).pending_pause = false;
    }

    fn action_dispatchPendingHostFunction(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        let kind = lock(&self.state).pending.kind;
        match kind {
            HostFunction::None => fw_assert!(false),
            HostFunction::Command => self.dispatch_command(),
            HostFunction::Telemetry => self.dispatch_telemetry(),
            HostFunction::Parameter => self.dispatch_parameter(),
            HostFunction::Event => self.dispatch_event(),
            HostFunction::Rsleep => self.dispatch_relative_sleep(),
            HostFunction::Asleep => self.dispatch_absolute_sleep(),
            HostFunction::Args => self.dispatch_args(),
            HostFunction::Time => self.dispatch_time(),
            HostFunction::SerialOut => self.dispatch_serial_out(),
            HostFunction::SerialRecv => self.dispatch_serial_recv(),
        }
    }

    fn action_clearPendingHostFunction(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        let mut st = lock(&self.state);
        st.pending.kind = HostFunction::None;
        st.pending_timer = None;
        st.host_function_start = None;
    }

    /// Start an execution window: bump the sequence counter so commands
    /// from this program carry a distinct command UID.
    fn action_setContext(&self, _sm_id: FwEnumStoreType, _signal: Signal, value: &RequestContext) {
        let mut st = lock(&self.state);
        fw_assert!(st.executing_context.is_none());
        st.executing_context = Some(*value);
        st.sequences_started = st.sequences_started.wrapping_add(1);
    }

    fn action_clearContext(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        lock(&self.state).executing_context = None;
    }

    fn action_resume(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        self.assert_store();
        let status = self.with_engine(|e| e.resume());
        fw_assert!(status.is_ok());
    }

    fn action_resumeI32(&self, _sm_id: FwEnumStoreType, _signal: Signal, value: i32) {
        self.assert_store();
        let status = self.with_engine(|e| e.resume_value(Value::I32(value)));
        fw_assert!(status.is_ok());
    }

    /// Wake up once the sleep deadline is reached.
    fn action_checkSleepTimers(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        let timer = lock(&self.state).pending_timer;
        let Some(timer) = timer else {
            fw_assert!(false);
            return;
        };
        let now = self.evt.time_get();
        match Time::compare(&now, &timer) {
            TimeComparison::Lt => {}
            TimeComparison::Eq | TimeComparison::Gt => self.itp().send_signal_hostResume(),
            // The time base changed since the timer was set.
            TimeComparison::Incomparable => {
                self.itp().send_signal_hostResponseTimeIncomparable();
            }
        }
    }

    /// Fail a blocking host function (a command, a blocking `serial_recv`)
    /// that has waited longer than `HOST_FUNCTION_TIMEOUT_SECS`. Sleeps
    /// have their own timer.
    fn action_checkTimeout(&self, _sm_id: FwEnumStoreType, _signal: Signal) {
        let start = lock(&self.state).host_function_start;
        let Some(start) = start else {
            return;
        };
        let (timeout_secs, _valid) = self.param_get_host_function_timeout_secs();
        // A non-positive or out-of-range timeout disables the check (NaN
        // passes both comparisons, as in C++, and converts to 0 here).
        if timeout_secs <= 0.0 || timeout_secs >= u32::MAX as f32 {
            return;
        }
        // Round the microseconds up so the timeout is never early.
        let mut seconds = timeout_secs as u32;
        let mut useconds = ((timeout_secs - seconds as f32) * 1_000_000.0 + 0.5) as u32;
        if useconds >= 1_000_000 {
            seconds += useconds / 1_000_000;
            useconds %= 1_000_000;
        }
        let deadline = time_add_wrapping(&start, seconds, useconds);
        let now = self.evt.time_get();
        match Time::compare(&now, &deadline) {
            TimeComparison::Lt => {}
            TimeComparison::Eq | TimeComparison::Gt => {
                self.itp().send_signal_hostResponseTimeout();
            }
            TimeComparison::Incomparable => {
                self.itp().send_signal_hostResponseTimeIncomparable();
            }
        }
    }

    fn action_dequeueSerialAndResume(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        _value: FwIndexType,
    ) {
        lock(&self.state).dequeue_succeeded = false;
        let ok = self.dequeue_serial_and_resume();
        lock(&self.state).dequeue_succeeded = ok;
    }

    fn guard_pendingPause(&self, _sm_id: FwEnumStoreType, _signal: Signal) -> bool {
        lock(&self.state).pending_pause
    }

    fn guard_pendingHostFunction(&self, _sm_id: FwEnumStoreType, _signal: Signal) -> bool {
        lock(&self.state).pending.is_pending()
    }

    fn guard_pendingHostFunctionIsSleep(&self, _sm_id: FwEnumStoreType, _signal: Signal) -> bool {
        matches!(
            lock(&self.state).pending.kind,
            HostFunction::Asleep | HostFunction::Rsleep
        )
    }

    fn guard_blockingSerialIn(
        &self,
        _sm_id: FwEnumStoreType,
        _signal: Signal,
        value: FwIndexType,
    ) -> bool {
        let st = lock(&self.state);
        st.pending.kind == HostFunction::SerialRecv
            && matches!(st.pending.args, PendingArgs::SerialRecv { index, .. } if index == value as u32)
    }

    fn guard_dequeueSucceeded(&self, _sm_id: FwEnumStoreType, _signal: Signal) -> bool {
        lock(&self.state).dequeue_succeeded
    }
}

impl WasmSequencer {
    /// Signal a trap: a host trap that `exit` / `panic` used to stop the
    /// interpreter carries no trap reason.
    fn signal_trap(&self, trap: Trap) {
        let reason = lock(&self.state).exit.reason;
        let reason = if trap == Trap::Reason(fprime_spacewasm::spacewasm::TrapReason::Host)
            && matches!(reason, ExitReason::HostExit | ExitReason::HostPanic)
        {
            TrapReason::None
        } else {
            map_trap_reason(trap)
        };
        self.itp().send_signal_interpreterTrap(reason);
    }
}
