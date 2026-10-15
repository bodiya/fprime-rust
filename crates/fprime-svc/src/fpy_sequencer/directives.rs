//! The directives — port of `FpySequencerDirectives.cpp`: the
//! `directive_*` internal ports, one handler per directive and the stack
//! operations. A handler returns the statement response for the state
//! machine (`stmtResponse_success` / `_failure` / `_beginSleep` /
//! `_keepWaiting`) and reports a failure through its `error` argument;
//! the internal-port handler forwards both (`sendSignal`,
//! `handleDirectiveErrorCode`).
//!
//! Every handler checks the stack before touching it, so a malformed
//! sequence fails with a [`DirectiveErrorCode`] instead of tripping the
//! stack's assertions. The arithmetic follows Python semantics where the
//! Fpy language does (floored `SDIV`, `SMOD`, `FMOD`), and saturates
//! float-to-integer conversions.

use super::{
    DirectiveUnion, FpySequencer, FpyState, MsgDispatchStatus, QUEUE_MSG_SIZE, Signal, StackValue,
    WakeupTime, types::*,
};
use fprime_comp::{QueueFullPolicy, msg};
use fprime_config::fpy_sequencer::{MAX_SERIAL_PORTS, MAX_STACK_SIZE};
use fprime_config::{
    FW_LOG_STRING_MAX_SIZE, FW_SERIALIZE_FALSE_VALUE, FW_SERIALIZE_TRUE_VALUE, FwEnumStoreType,
    FwIndexType, FwOpcodeType, FwPacketDescriptorType, FwTimeBaseStoreType, FwTimeContextStoreType,
};
use fprime_fw::{
    ComBuffer, ComPacketType, Endianness, ExtBuf, LengthMode, LinearBuffer, LogSeverity,
    ParamBuffer, ParamValid, SerBuf, SerBufAny, Serialize, Success, Time, TlmBuffer, TlmValid,
    fw_assert,
};

/// `MAX_STACK_SIZE` as a stack size.
const MAX: StackSizeType = MAX_STACK_SIZE as StackSizeType;

/// `true` / `false` as the stack stores them.
fn bool_byte(value: bool) -> u8 {
    if value {
        FW_SERIALIZE_TRUE_VALUE
    } else {
        FW_SERIALIZE_FALSE_VALUE
    }
}

/// The free bytes above the top of the stack.
fn free(st: &FpyState) -> StackSizeType {
    MAX - st.runtime.stack.size
}

// ---------------------------------------------------------------------------
// The internal ports.
// ---------------------------------------------------------------------------

/// Declares, for each `directive_*` internal port, its invoke function (a
/// queue message of priority 6 carrying the serialized directive), its
/// handler, and its arm of the message dispatch.
macro_rules! directive_ports {
    ($(
        $msg:ident: $variant:ident($ty:ty) => $invoke:ident, $handler:ident, $dir_handler:ident,
            id $id:expr;
    )*) => {
        impl FpySequencer {
            $(
                #[doc = concat!("`directive_", stringify!($variant), "_internalInterfaceInvoke`.")]
                fn $invoke(&self, directive: &$ty) {
                    self.send_directive(Self::$msg, directive);
                }

                #[doc = concat!("`directive_", stringify!($variant), "_internalInterfaceHandler`.")]
                fn $handler(&self, directive: &$ty) {
                    let mut guard = self.state.lock().unwrap();
                    let st = &mut *guard;
                    let mut error = DirectiveError::NoError;
                    let signal = self.$dir_handler(st, directive, &mut error);
                    self.send_signal(signal);
                    let id: fn(&$ty) -> DirectiveId = $id;
                    Self::handle_directive_error_code(st, id(directive), error);
                }
            )*

            /// C++ `dispatchDirective`: send a deserialized directive to its
            /// internal port.
            pub(crate) fn dispatch_directive(&self, directive: &DirectiveUnion, id: DirectiveId) {
                // coding error: INVALID never deserializes
                fw_assert!(id != DirectiveId::Invalid, id.as_repr() as i32);
                match directive {
                    $( DirectiveUnion::$variant(d) => self.$invoke(d), )*
                }
            }

            /// The `directive_*` arms of the message dispatch.
            pub(crate) fn dispatch_directive_message(
                &self,
                msg_type: FwEnumStoreType,
                buf: &mut dyn SerBufAny,
            ) -> MsgDispatchStatus {
                match msg_type {
                    $(
                        Self::$msg => {
                            let mut directive = <$ty>::default();
                            if !buf.deserialize(&mut directive, Endianness::Big).is_ok()
                                || buf.deserialize_size_left() != 0
                            {
                                return MsgDispatchStatus::Error;
                            }
                            self.$handler(&directive);
                            MsgDispatchStatus::Ok
                        }
                    )*
                    _ => MsgDispatchStatus::Error,
                }
            }
        }
    };
}

directive_ports! {
    MSG_TYPE_DIRECTIVE_WAIT_REL: WaitRel(WaitRelDirective)
        => directive_wait_rel_invoke, directive_wait_rel_handler, wait_rel_directive_handler,
        id |_| DirectiveId::WaitRel;
    MSG_TYPE_DIRECTIVE_WAIT_ABS: WaitAbs(WaitAbsDirective)
        => directive_wait_abs_invoke, directive_wait_abs_handler, wait_abs_directive_handler,
        id |_| DirectiveId::WaitAbs;
    MSG_TYPE_DIRECTIVE_GOTO: Goto(GotoDirective)
        => directive_goto_invoke, directive_goto_handler, goto_directive_handler,
        id |_| DirectiveId::Goto;
    MSG_TYPE_DIRECTIVE_IF: If(IfDirective)
        => directive_if_invoke, directive_if_handler, if_directive_handler,
        id |_| DirectiveId::If;
    MSG_TYPE_DIRECTIVE_NO_OP: NoOp(NoOpDirective)
        => directive_no_op_invoke, directive_no_op_handler, no_op_directive_handler,
        id |_| DirectiveId::NoOp;
    MSG_TYPE_DIRECTIVE_PUSH_TLM_VAL: PushTlmVal(PushTlmValDirective)
        => directive_push_tlm_val_invoke, directive_push_tlm_val_handler,
        push_tlm_val_directive_handler, id |_| DirectiveId::PushTlmVal;
    MSG_TYPE_DIRECTIVE_PUSH_TLM_VAL_AND_TIME: PushTlmValAndTime(PushTlmValAndTimeDirective)
        => directive_push_tlm_val_and_time_invoke, directive_push_tlm_val_and_time_handler,
        push_tlm_val_and_time_directive_handler, id |_| DirectiveId::PushTlmValAndTime;
    MSG_TYPE_DIRECTIVE_PUSH_PRM: PushPrm(PushPrmDirective)
        => directive_push_prm_invoke, directive_push_prm_handler, push_prm_directive_handler,
        id |_| DirectiveId::PushPrm;
    MSG_TYPE_DIRECTIVE_CONST_CMD: ConstCmd(ConstCmdDirective)
        => directive_const_cmd_invoke, directive_const_cmd_handler, const_cmd_directive_handler,
        id |_| DirectiveId::ConstCmd;
    // C++ parity: a stack op reports its own opcode.
    MSG_TYPE_DIRECTIVE_STACK_OP: StackOp(StackOpDirective)
        => directive_stack_op_invoke, directive_stack_op_handler, stack_op_directive_handler,
        id |d| d.op;
    MSG_TYPE_DIRECTIVE_EXIT: Exit(ExitDirective)
        => directive_exit_invoke, directive_exit_handler, exit_directive_handler,
        id |_| DirectiveId::Exit;
    MSG_TYPE_DIRECTIVE_ALLOCATE: Allocate(AllocateDirective)
        => directive_allocate_invoke, directive_allocate_handler, allocate_directive_handler,
        id |_| DirectiveId::Allocate;
    MSG_TYPE_DIRECTIVE_STORE_REL_CONST_OFFSET: StoreRelConstOffset(StoreRelConstOffsetDirective)
        => directive_store_rel_const_offset_invoke, directive_store_rel_const_offset_handler,
        store_rel_const_offset_directive_handler, id |_| DirectiveId::StoreRelConstOffset;
    MSG_TYPE_DIRECTIVE_LOAD_REL: LoadRel(LoadRelDirective)
        => directive_load_rel_invoke, directive_load_rel_handler, load_rel_directive_handler,
        id |_| DirectiveId::LoadRel;
    MSG_TYPE_DIRECTIVE_PUSH_VAL: PushVal(PushValDirective)
        => directive_push_val_invoke, directive_push_val_handler, push_val_directive_handler,
        id |_| DirectiveId::PushVal;
    MSG_TYPE_DIRECTIVE_DISCARD: Discard(DiscardDirective)
        => directive_discard_invoke, directive_discard_handler, discard_directive_handler,
        id |_| DirectiveId::Discard;
    MSG_TYPE_DIRECTIVE_MEM_CMP: MemCmp(MemCmpDirective)
        => directive_mem_cmp_invoke, directive_mem_cmp_handler, mem_cmp_directive_handler,
        id |_| DirectiveId::Memcmp;
    MSG_TYPE_DIRECTIVE_STACK_CMD: StackCmd(StackCmdDirective)
        => directive_stack_cmd_invoke, directive_stack_cmd_handler, stack_cmd_directive_handler,
        id |_| DirectiveId::StackCmd;
    MSG_TYPE_DIRECTIVE_PUSH_TIME: PushTime(PushTimeDirective)
        => directive_push_time_invoke, directive_push_time_handler, push_time_directive_handler,
        id |_| DirectiveId::PushTime;
    MSG_TYPE_DIRECTIVE_SET_SEED: SetSeed(SetSeedDirective)
        => directive_set_seed_invoke, directive_set_seed_handler, set_seed_directive_handler,
        id |_| DirectiveId::SetSeed;
    MSG_TYPE_DIRECTIVE_PUSH_RAND: PushRand(PushRandDirective)
        => directive_push_rand_invoke, directive_push_rand_handler, push_rand_directive_handler,
        id |_| DirectiveId::PushRand;
    MSG_TYPE_DIRECTIVE_GET_FIELD: GetField(GetFieldDirective)
        => directive_get_field_invoke, directive_get_field_handler, get_field_directive_handler,
        id |_| DirectiveId::GetField;
    MSG_TYPE_DIRECTIVE_PEEK: Peek(PeekDirective)
        => directive_peek_invoke, directive_peek_handler, peek_directive_handler,
        id |_| DirectiveId::Peek;
    MSG_TYPE_DIRECTIVE_STORE_REL: StoreRel(StoreRelDirective)
        => directive_store_rel_invoke, directive_store_rel_handler, store_rel_directive_handler,
        id |_| DirectiveId::StoreRel;
    MSG_TYPE_DIRECTIVE_CALL: Call(CallDirective)
        => directive_call_invoke, directive_call_handler, call_directive_handler,
        id |_| DirectiveId::Call;
    MSG_TYPE_DIRECTIVE_RETURN: Return(ReturnDirective)
        => directive_return_invoke, directive_return_handler, return_directive_handler,
        id |_| DirectiveId::Return;
    MSG_TYPE_DIRECTIVE_LOAD_ABS: LoadAbs(LoadAbsDirective)
        => directive_load_abs_invoke, directive_load_abs_handler, load_abs_directive_handler,
        id |_| DirectiveId::LoadAbs;
    MSG_TYPE_DIRECTIVE_STORE_ABS: StoreAbs(StoreAbsDirective)
        => directive_store_abs_invoke, directive_store_abs_handler, store_abs_directive_handler,
        id |_| DirectiveId::StoreAbs;
    MSG_TYPE_DIRECTIVE_STORE_ABS_CONST_OFFSET: StoreAbsConstOffset(StoreAbsConstOffsetDirective)
        => directive_store_abs_const_offset_invoke, directive_store_abs_const_offset_handler,
        store_abs_const_offset_directive_handler, id |_| DirectiveId::StoreAbsConstOffset;
    MSG_TYPE_DIRECTIVE_POP_EVENT: PopEvent(PopEventDirective)
        => directive_pop_event_invoke, directive_pop_event_handler, pop_event_directive_handler,
        id |_| DirectiveId::PopEvent;
    MSG_TYPE_DIRECTIVE_POP_SERIALIZABLE: PopSerializable(PopSerializableDirective)
        => directive_pop_serializable_invoke, directive_pop_serializable_handler,
        pop_serializable_directive_handler, id |_| DirectiveId::PopSerializable;
}

impl FpySequencer {
    /// Enqueue a directive on its internal port: `[msg_type][port 0]
    /// [directive]`, priority 6, `assert` on a full queue.
    fn send_directive(&self, msg_type: FwEnumStoreType, directive: &dyn Serialize) {
        let mut buf = LinearBuffer::<QUEUE_MSG_SIZE>::new();
        let status = msg::write_envelope_header(&mut buf, msg_type, 0);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize(directive, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self.active.queued.send_message(
            &buf,
            super::DIRECTIVE_PRIORITY,
            QueueFullPolicy::Assert,
        );
    }

    /// C++ `handleDirectiveErrorCode`: record the last error, and where it
    /// happened if it is one.
    pub(crate) fn handle_directive_error_code(
        st: &mut FpyState,
        id: DirectiveId,
        err: DirectiveError,
    ) {
        st.tlm.last_directive_error = err;
        if err != DirectiveError::NoError {
            st.tlm.directive_error_index = u64::from(st.current_statement_idx());
            st.tlm.directive_error_id = id;
        }
    }

    /// C++ `sendCmd`: build the command packet
    /// `[FW_PACKET_COMMAND][opcode][args]` and send it on `cmdOut` with the
    /// command UID as its context.
    pub(crate) fn send_cmd(&self, st: &FpyState, opcode: FwOpcodeType, arg_buf: &[u8]) -> Success {
        let mut cmd_buf = ComBuffer::new();
        if !cmd_buf
            .serialize_u16_be(ComPacketType::FwPacketCommand.as_repr() as FwPacketDescriptorType)
            .is_ok()
        {
            return Success::Failure;
        }
        if !cmd_buf.serialize_u32_be(opcode).is_ok() {
            return Success::Failure;
        }
        if !cmd_buf
            .serialize_bytes(arg_buf, LengthMode::OmitLength, Endianness::Big)
            .is_ok()
        {
            return Success::Failure;
        }
        // UID: the low 16 bits of the sequence counter, then of the
        // statement counter, so the response can be matched to this
        // statement of this sequence
        let cmd_uid =
            (((st.sequences_started & 0xFFFF) << 16) | (st.statements_dispatched & 0xFFFF)) as u32;
        let p = self.cmd_out.get();
        p.target.invoke(p.port_num, &mut cmd_buf, cmd_uid);
        Success::Success
    }
}

// ---------------------------------------------------------------------------
// Directive handlers.
// ---------------------------------------------------------------------------

impl FpySequencer {
    /// `WAIT_REL`: pops useconds then seconds; sleeps until now + that.
    pub(crate) fn wait_rel_directive_handler(
        &self,
        st: &mut FpyState,
        _directive: &WaitRelDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if st.runtime.stack.size < 8 {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let now = self.evt.time_get();
        let useconds = st.runtime.stack.pop::<u32>();
        let seconds = st.runtime.stack.pop::<u32>();
        // C++ `Time::add(seconds, useconds)`: wrapping seconds, a carry,
        // and FW_ASSERT on a microsecond sum of 1,999,999 or more (a
        // malformed sequence stops the software, as upstream).
        let mut new_seconds = now.get_seconds().wrapping_add(seconds);
        let mut new_useconds = now.get_useconds().wrapping_add(useconds);
        fw_assert!(new_useconds < 1_999_999, new_useconds as i32);
        if new_useconds >= 1_000_000 {
            new_seconds = new_seconds.wrapping_add(1);
            new_useconds -= 1_000_000;
        }
        st.runtime.wakeup_time = WakeupTime {
            time_base: now.get_time_base().as_repr(),
            time_context: now.get_context(),
            seconds: new_seconds,
            useconds: new_useconds,
        };
        Signal::stmtResponse_beginSleep
    }

    /// `WAIT_ABS`: pops useconds, seconds, the time context and the time
    /// base; sleeps until that time.
    pub(crate) fn wait_abs_directive_handler(
        &self,
        st: &mut FpyState,
        _directive: &WaitAbsDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let needed = 2 * size_of::<u32>()
            + size_of::<FwTimeContextStoreType>()
            + size_of::<FwTimeBaseStoreType>();
        if (st.runtime.stack.size as usize) < needed {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let useconds = st.runtime.stack.pop::<u32>();
        let seconds = st.runtime.stack.pop::<u32>();
        let context = st.runtime.stack.pop::<FwTimeContextStoreType>();
        let base = st.runtime.stack.pop::<FwTimeBaseStoreType>();
        // C++ parity: `Fw::Time(base, ctx, s, us)` asserts us < 1,000,000.
        fw_assert!(useconds < 1_000_000, useconds as i32);
        st.runtime.wakeup_time = WakeupTime {
            time_base: base,
            time_context: context,
            seconds,
            useconds,
        };
        Signal::stmtResponse_beginSleep
    }

    /// `GOTO`: jump; the statement count itself is allowed (ends the
    /// sequence).
    pub(crate) fn goto_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &GotoDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if directive.statement_index > u32::from(st.sequence_obj.header.statement_count) {
            *error = DirectiveError::StmtOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        st.runtime.next_statement_index = directive.statement_index;
        Signal::stmtResponse_success
    }

    /// `IF`: pops a byte; jumps when it is zero.
    pub(crate) fn if_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &IfDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if st.runtime.stack.size < 1 {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        if directive.false_goto_stmt_index > u32::from(st.sequence_obj.header.statement_count) {
            *error = DirectiveError::StmtOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        if st.runtime.stack.pop::<u8>() != 0 {
            return Signal::stmtResponse_success;
        }
        st.runtime.next_statement_index = directive.false_goto_stmt_index;
        Signal::stmtResponse_success
    }

    /// `NO_OP`.
    pub(crate) fn no_op_directive_handler(
        &self,
        _st: &mut FpyState,
        _directive: &NoOpDirective,
        _error: &mut DirectiveError,
    ) -> Signal {
        Signal::stmtResponse_success
    }

    /// Fetch a telemetry value through `getTlmChan`.
    fn get_tlm(
        &self,
        chan_id: fprime_config::FwChanIdType,
        error: &mut DirectiveError,
    ) -> Option<(Time, TlmBuffer)> {
        let Some(p) = self.get_tlm_chan.try_get() else {
            *error = DirectiveError::TlmGetNotConnected;
            return None;
        };
        let mut time = Time::default();
        let mut value = TlmBuffer::new();
        if p.target.invoke(p.port_num, chan_id, &mut time, &mut value) != TlmValid::Valid {
            *error = DirectiveError::TlmChanNotFound;
            return None;
        }
        Some((time, value))
    }

    /// `PUSH_TLM_VAL`: push a telemetry value's bytes.
    pub(crate) fn push_tlm_val_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &PushTlmValDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let Some((_time, value)) = self.get_tlm(directive.chan_id, error) else {
            return Signal::stmtResponse_failure;
        };
        if value.get_size() > free(st) as usize {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        st.runtime.stack.push_bytes(value.as_slice());
        Signal::stmtResponse_success
    }

    /// `PUSH_TLM_VAL_AND_TIME`: push a telemetry value's bytes, then its
    /// serialized time (11 bytes).
    pub(crate) fn push_tlm_val_and_time_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &PushTlmValAndTimeDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let Some((time, value)) = self.get_tlm(directive.chan_id, error) else {
            return Signal::stmtResponse_failure;
        };
        let mut time_buf = [0u8; Time::SERIALIZED_SIZE];
        let mut time_esb = ExtBuf::new(&mut time_buf);
        let status = time.serialize_to(&mut time_esb, Endianness::Big);
        // coding error if this failed: the buffer is the serialized size
        fw_assert!(status.is_ok(), status as i32);
        if value.get_size() + time_esb.get_size() > free(st) as usize {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        st.runtime.stack.push_bytes(value.as_slice());
        st.runtime.stack.push_bytes(time_esb.as_slice());
        Signal::stmtResponse_success
    }

    /// `PUSH_PRM`: push a parameter value's bytes.
    pub(crate) fn push_prm_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &PushPrmDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let Some(p) = self.get_param.try_get() else {
            *error = DirectiveError::PrmGetNotConnected;
            return Signal::stmtResponse_failure;
        };
        let mut value = ParamBuffer::new();
        if p.target.invoke(p.port_num, directive.prm_id, &mut value) != ParamValid::Valid {
            *error = DirectiveError::PrmNotFound;
            return Signal::stmtResponse_failure;
        }
        if value.get_size() > free(st) as usize {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        st.runtime.stack.push_bytes(value.as_slice());
        Signal::stmtResponse_success
    }

    /// `CONST_CMD`: dispatch a command with constant arguments, then wait
    /// for its response.
    pub(crate) fn const_cmd_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &ConstCmdDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        // the response code is pushed when the command completes: make sure
        // it will fit before dispatching
        if size_of::<u8>() > free(st) as usize {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        let args = &directive.arg_buf.0
            [..(directive.arg_buf_size as usize).min(directive.arg_buf.0.len())];
        if self.send_cmd(st, directive.op_code, args) == Success::Failure {
            return Signal::stmtResponse_failure;
        }
        // harmless if the response is already back
        Signal::stmtResponse_keepWaiting
    }

    /// The stack operations (`OR` … `ITRUNC_64_32`, `FFLOOR` … `FABS`).
    /// Asserts that `op` is one of them.
    pub(crate) fn stack_op_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &StackOpDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        // coding error: only stack ops are routed here
        fw_assert!(directive.op.is_stack_op(), directive.op.as_repr() as i32);
        *error = match directive.op {
            DirectiveId::Or => st.op_or(),
            DirectiveId::And => st.op_and(),
            DirectiveId::Ieq => st.op_ieq(),
            DirectiveId::Ine => st.op_ine(),
            DirectiveId::Ult => st.op_ult(),
            DirectiveId::Ule => st.op_ule(),
            DirectiveId::Ugt => st.op_ugt(),
            DirectiveId::Uge => st.op_uge(),
            DirectiveId::Slt => st.op_slt(),
            DirectiveId::Sle => st.op_sle(),
            DirectiveId::Sgt => st.op_sgt(),
            DirectiveId::Sge => st.op_sge(),
            DirectiveId::Feq => st.op_feq(),
            DirectiveId::Fne => st.op_fne(),
            DirectiveId::Flt => st.op_flt(),
            DirectiveId::Fle => st.op_fle(),
            DirectiveId::Fgt => st.op_fgt(),
            DirectiveId::Fge => st.op_fge(),
            DirectiveId::Not => st.op_not(),
            DirectiveId::Fpext => st.op_fpext(),
            DirectiveId::Fptrunc => st.op_fptrunc(),
            DirectiveId::Fptosi => st.op_fptosi(),
            DirectiveId::Fptoui => st.op_fptoui(),
            DirectiveId::Sitofp => st.op_sitofp(),
            DirectiveId::Uitofp => st.op_uitofp(),
            DirectiveId::Add => st.op_add(),
            DirectiveId::Sub => st.op_sub(),
            DirectiveId::Mul => st.op_mul(),
            DirectiveId::Udiv => st.op_udiv(),
            DirectiveId::Sdiv => st.op_sdiv(),
            DirectiveId::Umod => st.op_umod(),
            DirectiveId::Smod => st.op_smod(),
            DirectiveId::Fadd => st.op_fadd(),
            DirectiveId::Fsub => st.op_fsub(),
            DirectiveId::Fmul => st.op_fmul(),
            DirectiveId::Fdiv => st.op_fdiv(),
            DirectiveId::Fpow => st.op_fpow(),
            DirectiveId::Flog => st.op_flog(),
            DirectiveId::Fmod => st.op_fmod(),
            DirectiveId::Siext8To64 => st.op_siext_8_64(),
            DirectiveId::Siext16To64 => st.op_siext_16_64(),
            DirectiveId::Siext32To64 => st.op_siext_32_64(),
            DirectiveId::Ziext8To64 => st.op_ziext_8_64(),
            DirectiveId::Ziext16To64 => st.op_ziext_16_64(),
            DirectiveId::Ziext32To64 => st.op_ziext_32_64(),
            DirectiveId::Itrunc64To8 => st.op_itrunc_64_8(),
            DirectiveId::Itrunc64To16 => st.op_itrunc_64_16(),
            DirectiveId::Itrunc64To32 => st.op_itrunc_64_32(),
            DirectiveId::Ffloor => st.op_ffloor(),
            DirectiveId::Iabs => st.op_iabs(),
            DirectiveId::Fabs => st.op_fabs(),
            other => {
                fw_assert!(false, other.as_repr() as i32);
                DirectiveError::NoError
            }
        };
        if *error != DirectiveError::NoError {
            return Signal::stmtResponse_failure;
        }
        Signal::stmtResponse_success
    }

    /// `EXIT`: pops an `I32` code. 0 ends the sequence normally; anything
    /// else fails it with `SequenceExitedWithError`.
    pub(crate) fn exit_directive_handler(
        &self,
        st: &mut FpyState,
        _directive: &ExitDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if (st.runtime.stack.size as usize) < size_of::<i32>() {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let error_code = st.runtime.stack.pop::<i32>();
        if error_code == 0 {
            // just go to the end of the sequence
            st.runtime.next_statement_index = u32::from(st.sequence_obj.header.statement_count);
            return Signal::stmtResponse_success;
        }
        self.log_warning_hi_sequence_exited_with_error(
            st.sequence_file_path.as_bytes(),
            error_code,
        );
        *error = DirectiveError::ExitWithError;
        Signal::stmtResponse_failure
    }

    /// `ALLOCATE`: push zeroed bytes.
    pub(crate) fn allocate_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &AllocateDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if directive.size > free(st) {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        st.runtime.stack.push_zeroes(directive.size);
        Signal::stmtResponse_success
    }

    /// C++ `storeHelper`: pop `size` bytes and store them at `dest_offset`,
    /// which must lie below the popped top.
    pub(crate) fn store_helper(
        st: &mut FpyState,
        dest_offset: StackSizeType,
        size: StackSizeType,
        error: &mut DirectiveError,
    ) -> Signal {
        let stack = &mut st.runtime.stack;
        if stack.size < size {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let new_size = stack.size - size;
        if dest_offset > new_size || size > new_size - dest_offset {
            *error = DirectiveError::StackAccessOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        stack.copy(dest_offset, stack.size - size, size);
        stack.size = new_size;
        Signal::stmtResponse_success
    }

    /// C++ `loadHelper`: push a copy of `size` bytes at `src_offset`.
    pub(crate) fn load_helper(
        st: &mut FpyState,
        src_offset: StackSizeType,
        size: StackSizeType,
        error: &mut DirectiveError,
    ) -> Signal {
        let stack = &mut st.runtime.stack;
        if size > MAX - stack.size {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        if src_offset > stack.size || size > stack.size - src_offset {
            *error = DirectiveError::StackAccessOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        stack.copy(stack.size, src_offset, size);
        stack.size += size;
        Signal::stmtResponse_success
    }

    /// The absolute address of a frame-relative offset, if it is in
    /// `0..=MAX_STACK_SIZE`.
    fn frame_address(st: &FpyState, lvar_offset: SignedStackSizeType) -> Option<StackSizeType> {
        let addr = i64::from(st.runtime.stack.current_frame_start) + i64::from(lvar_offset);
        if addr < 0 || addr > i64::from(MAX) {
            return None;
        }
        Some(addr as StackSizeType)
    }

    /// `STORE_REL_CONST_OFFSET`.
    pub(crate) fn store_rel_const_offset_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &StoreRelConstOffsetDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let Some(addr) = Self::frame_address(st, directive.lvar_offset) else {
            *error = DirectiveError::StackAccessOutOfBounds;
            return Signal::stmtResponse_failure;
        };
        Self::store_helper(st, addr, directive.size, error)
    }

    /// `LOAD_REL`.
    pub(crate) fn load_rel_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &LoadRelDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let Some(addr) = Self::frame_address(st, directive.lvar_offset) else {
            *error = DirectiveError::StackAccessOutOfBounds;
            return Signal::stmtResponse_failure;
        };
        Self::load_helper(st, addr, directive.size, error)
    }

    /// `PUSH_VAL`: push constant bytes.
    pub(crate) fn push_val_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &PushValDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if directive.val_size > u64::from(free(st)) {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        st.runtime
            .stack
            .push_bytes(&directive.val.0[..directive.val_size as usize]);
        Signal::stmtResponse_success
    }

    /// `DISCARD`: drop bytes off the top.
    pub(crate) fn discard_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &DiscardDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if st.runtime.stack.size < directive.size {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        st.runtime.stack.size -= directive.size;
        Signal::stmtResponse_success
    }

    /// `MEMCMP`: pop two `size`-byte arrays, push whether they are equal.
    pub(crate) fn mem_cmp_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &MemCmpDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        // size * 2 cannot overflow once size <= MAX / 2
        if directive.size > MAX / 2 {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let stack = &mut st.runtime.stack;
        if stack.size < directive.size * 2 {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let size = directive.size as usize;
        let lhs = stack.size as usize - size * 2;
        let rhs = stack.size as usize - size;
        // remove both before pushing the result
        stack.size -= directive.size * 2;
        let equal = stack.bytes[lhs..lhs + size] == stack.bytes[rhs..rhs + size];
        stack.push::<u8>(bool_byte(equal));
        Signal::stmtResponse_success
    }

    /// `STACK_CMD`: pop the opcode (on top, after the arguments), then
    /// `args_size` argument bytes, and dispatch the command.
    pub(crate) fn stack_cmd_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &StackCmdDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let opcode_size = size_of::<FwOpcodeType>() as StackSizeType;
        if st.runtime.stack.size < opcode_size
            || st.runtime.stack.size - opcode_size < directive.args_size
        {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let opcode = st.runtime.stack.pop::<FwOpcodeType>();
        let arg_offset = (st.runtime.stack.size - directive.args_size) as usize;
        st.runtime.current_cmd_opcode = opcode;
        st.runtime.stack.size -= directive.args_size;
        // room for the response code (FwOpcodeType is configurable, so the
        // popped opcode may not have freed enough)
        if size_of::<u8>() > free(st) as usize {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        // the argument bytes are still in the array, just above the top
        let n = directive.args_size as usize;
        let args = &st.runtime.stack.bytes[arg_offset..arg_offset + n];
        if self.send_cmd(st, opcode, args) == Success::Failure {
            return Signal::stmtResponse_failure;
        }
        Signal::stmtResponse_keepWaiting
    }

    /// `PUSH_TIME`: push the current time, serialized (11 bytes).
    pub(crate) fn push_time_directive_handler(
        &self,
        st: &mut FpyState,
        _directive: &PushTimeDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if Time::SERIALIZED_SIZE > free(st) as usize {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        let now = self.evt.time_get();
        let mut time_buf = [0u8; Time::SERIALIZED_SIZE];
        let mut time_esb = ExtBuf::new(&mut time_buf);
        let status = now.serialize_to(&mut time_esb, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        st.runtime.stack.push_bytes(time_esb.as_slice());
        Signal::stmtResponse_success
    }

    /// `SET_SEED`: pop a `U32` and seed the generator with it.
    pub(crate) fn set_seed_directive_handler(
        &self,
        st: &mut FpyState,
        _directive: &SetSeedDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if (st.runtime.stack.size as usize) < size_of::<u32>() {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let seed = st.runtime.stack.pop::<u32>();
        st.runtime.rng.seed(seed);
        st.runtime.rng_seeded = true;
        Signal::stmtResponse_success
    }

    /// `PUSH_RAND`: push the next `U32` of the generator, seeding it from
    /// the current time (`std::seed_seq{base, context, seconds,
    /// useconds}`) on first use in a run.
    pub(crate) fn push_rand_directive_handler(
        &self,
        st: &mut FpyState,
        _directive: &PushRandDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if size_of::<u32>() > free(st) as usize {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        if !st.runtime.rng_seeded {
            let now = self.evt.time_get();
            st.runtime.rng.seed_seq(&[
                u32::from(now.get_time_base().as_repr()),
                u32::from(now.get_context()),
                now.get_seconds(),
                now.get_useconds(),
            ]);
            st.runtime.rng_seeded = true;
        }
        let value = st.runtime.rng.next_u32();
        st.runtime.stack.push(value);
        Signal::stmtResponse_success
    }

    /// `GET_FIELD`: pop an offset; keep `member_size` bytes at that offset
    /// of the `parent_size`-byte value below it, moved to the value's start.
    pub(crate) fn get_field_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &GetFieldDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let offset_size = size_of::<StackSizeType>() as StackSizeType;
        let stack = &mut st.runtime.stack;
        if stack.size < offset_size {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        if stack.size - offset_size < directive.parent_size {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let offset = stack.pop::<StackSizeType>();
        if offset > directive.parent_size || directive.member_size > directive.parent_size - offset
        {
            // not quite a stack access, but the closest error code
            *error = DirectiveError::StackAccessOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        let parent_start = stack.size - directive.parent_size;
        stack.r#move(parent_start, parent_start + offset, directive.member_size);
        stack.size -= directive.parent_size - directive.member_size;
        Signal::stmtResponse_success
    }

    /// `PEEK`: pop an offset and a byte count; push a copy of the bytes
    /// that end `offset` bytes below the top.
    pub(crate) fn peek_directive_handler(
        &self,
        st: &mut FpyState,
        _directive: &PeekDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let stack = &mut st.runtime.stack;
        if (stack.size as usize) < size_of::<StackSizeType>() * 2 {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let offset = stack.pop::<StackSizeType>();
        let byte_count = stack.pop::<StackSizeType>();
        // equal is allowed: the byte count may be 0
        if offset > stack.size {
            *error = DirectiveError::StackAccessOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        if byte_count > MAX - stack.size {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        if byte_count > stack.size - offset {
            *error = DirectiveError::StackAccessOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        let src = stack.size - offset - byte_count;
        // source and destination are adjacent, never overlapping
        stack.copy(stack.size, src, byte_count);
        stack.size += byte_count;
        Signal::stmtResponse_success
    }

    /// `STORE_REL`: pop a frame-relative offset, then store `size` bytes
    /// there.
    pub(crate) fn store_rel_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &StoreRelDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let offset_size = size_of::<SignedStackSizeType>() as StackSizeType;
        if st.runtime.stack.size < offset_size
            || st.runtime.stack.size - offset_size < directive.size
        {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let lvar_offset = st.runtime.stack.pop::<SignedStackSizeType>();
        let Some(addr) = Self::frame_address(st, lvar_offset) else {
            *error = DirectiveError::StackAccessOutOfBounds;
            return Signal::stmtResponse_failure;
        };
        Self::store_helper(st, addr, directive.size, error)
    }

    /// `CALL`: pop the target index; push the return address and the frame
    /// pointer; start a new frame.
    pub(crate) fn call_directive_handler(
        &self,
        st: &mut FpyState,
        _directive: &CallDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        if (st.runtime.stack.size as usize) < size_of::<u32>() {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let target = st.runtime.stack.pop::<u32>();
        if st.runtime.stack.size as usize + size_of::<StackSizeType>() + size_of::<u32>()
            > MAX_STACK_SIZE
        {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        if target > u32::from(st.sequence_obj.header.statement_count) {
            *error = DirectiveError::StmtOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        let return_addr = st.runtime.next_statement_index;
        st.runtime.next_statement_index = target;
        st.runtime.stack.push::<u32>(return_addr);
        let frame = st.runtime.stack.current_frame_start;
        st.runtime.stack.push::<StackSizeType>(frame);
        st.runtime.stack.current_frame_start = st.runtime.stack.size;
        Signal::stmtResponse_success
    }

    /// `RETURN`: discard the frame, restore the frame pointer and the
    /// return address, discard the call arguments and push the return value.
    pub(crate) fn return_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &ReturnDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let return_val_size = directive.return_val_size;
        let call_args_size = directive.call_args_size;
        let statement_count = u32::from(st.sequence_obj.header.statement_count);
        let runtime = &mut st.runtime;
        if runtime.stack.size < return_val_size {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        // the return value is moved down below, not copied through a buffer
        let return_val_offset = runtime.stack.size - return_val_size;
        if runtime.stack.current_frame_start > runtime.stack.size {
            *error = DirectiveError::FrameStartOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        runtime.stack.size = runtime.stack.current_frame_start;
        if (runtime.stack.size as usize) < size_of::<StackSizeType>() + size_of::<u32>() {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let saved_frame_ptr = runtime.stack.pop::<StackSizeType>();
        let return_addr = runtime.stack.pop::<u32>();
        if saved_frame_ptr > runtime.stack.size {
            *error = DirectiveError::FrameStartOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        runtime.stack.current_frame_start = saved_frame_ptr;
        if return_addr > statement_count {
            *error = DirectiveError::StmtOutOfBounds;
            return Signal::stmtResponse_failure;
        }
        runtime.next_statement_index = return_addr;
        if runtime.stack.size < call_args_size {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        runtime.stack.size -= call_args_size;
        if return_val_size > MAX - runtime.stack.size {
            *error = DirectiveError::StackOverflow;
            return Signal::stmtResponse_failure;
        }
        if return_val_size > 0 {
            // C++ parity: a raw memmove, not Stack::move — the source lies
            // above the truncated size, which Stack::move rejects. Both
            // regions were bounds-checked against MAX_STACK_SIZE above.
            let top = runtime.stack.size as usize;
            let src = return_val_offset as usize;
            let n = return_val_size as usize;
            runtime.stack.bytes.copy_within(src..src + n, top);
            runtime.stack.size += return_val_size;
        }
        Signal::stmtResponse_success
    }

    /// `LOAD_ABS`.
    pub(crate) fn load_abs_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &LoadAbsDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        Self::load_helper(st, directive.global_offset, directive.size, error)
    }

    /// `STORE_ABS`: pop an absolute offset, then store `size` bytes there.
    pub(crate) fn store_abs_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &StoreAbsDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let offset_size = size_of::<StackSizeType>() as StackSizeType;
        if st.runtime.stack.size < offset_size
            || st.runtime.stack.size - offset_size < directive.size
        {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let global_offset = st.runtime.stack.pop::<StackSizeType>();
        Self::store_helper(st, global_offset, directive.size, error)
    }

    /// `STORE_ABS_CONST_OFFSET`.
    pub(crate) fn store_abs_const_offset_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &StoreAbsConstOffsetDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        Self::store_helper(st, directive.global_offset, directive.size, error)
    }

    /// `POP_EVENT`: pop the message size, the message (at most
    /// `FW_LOG_STRING_MAX_SIZE - 1` bytes are kept; the rest is discarded)
    /// and a `U8` severity; emit the matching `Log*` event.
    pub(crate) fn pop_event_directive_handler(
        &self,
        st: &mut FpyState,
        _directive: &PopEventDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        let stack = &mut st.runtime.stack;
        if (stack.size as usize) < size_of::<StackSizeType>() {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let message_size = stack.pop::<StackSizeType>();
        let severity_size = size_of::<u8>() as StackSizeType;
        if stack.size < severity_size || stack.size - severity_size < message_size {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let mut message_buf = [0u8; FW_LOG_STRING_MAX_SIZE];
        let clamped = message_size.min((FW_LOG_STRING_MAX_SIZE - 1) as StackSizeType);
        if message_size > clamped {
            // drop the end of the message, which is on top
            let excess = message_size - clamped;
            fw_assert!(stack.size >= excess, stack.size as i32, excess as i32);
            stack.size -= excess;
        }
        stack.pop_bytes(&mut message_buf[..clamped as usize]);
        let severity = stack.pop::<u8>();
        // C++ parity: the message is a C string, so it ends at a NUL byte
        let message = &message_buf[..clamped as usize];
        let message = &message[..message
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(message.len())];
        match LogSeverity::try_from(severity) {
            Ok(severity) => {
                self.log_sequence_message(severity, st.sequence_file_path.as_bytes(), message);
                Signal::stmtResponse_success
            }
            Err(_) => {
                *error = DirectiveError::InvalidArg;
                Signal::stmtResponse_failure
            }
        }
    }

    /// `POP_SERIALIZABLE`: send `size` bytes off the top of the stack out
    /// `serialOut[port_index]`, then drop them.
    pub(crate) fn pop_serializable_directive_handler(
        &self,
        st: &mut FpyState,
        directive: &PopSerializableDirective,
        error: &mut DirectiveError,
    ) -> Signal {
        // no size assertion: an oversized size is untrusted sequence
        // content, rejected by the stack check below
        let port_index: FwIndexType = directive.port_index;
        if !(0..MAX_SERIAL_PORTS).contains(&port_index) {
            *error = DirectiveError::SerialPortInvalidIndex;
            return Signal::stmtResponse_failure;
        }
        let Some(p) = self.serial_out[port_index as usize].try_get() else {
            *error = DirectiveError::SerialPortNotConnected;
            return Signal::stmtResponse_failure;
        };
        let stack = &mut st.runtime.stack;
        if stack.size < directive.size {
            *error = DirectiveError::StackUnderflow;
            return Signal::stmtResponse_failure;
        }
        let start = (stack.size - directive.size) as usize;
        let end = stack.size as usize;
        // a buffer over the stack bytes, no copy
        let mut buf = ExtBuf::with_len(&mut stack.bytes[start..end], end - start);
        let status = p.target.invoke_serial(p.port_num, &mut buf);
        fw_assert!(status.is_ok(), status as i32);
        stack.size -= directive.size;
        Signal::stmtResponse_success
    }
}

// ---------------------------------------------------------------------------
// The stack operations (C++ `op_*`).
// ---------------------------------------------------------------------------

/// Pop the right then the left operand of a binary op.
fn pop2<T: StackValue>(st: &mut FpyState) -> (T, T) {
    let rhs = st.runtime.stack.pop::<T>();
    let lhs = st.runtime.stack.pop::<T>();
    (lhs, rhs)
}

impl FpyState {
    fn has(&self, bytes: usize) -> bool {
        self.runtime.stack.size as usize >= bytes
    }

    fn push<T: StackValue>(&mut self, value: T) {
        self.runtime.stack.push(value);
    }

    /// Binary op on two `T`s producing a boolean byte.
    fn cmp_op<T: StackValue>(&mut self, f: impl FnOnce(T, T) -> bool) -> DirectiveError {
        if !self.has(T::SIZE * 2) {
            return DirectiveError::StackUnderflow;
        }
        let (lhs, rhs) = pop2::<T>(self);
        self.push(bool_byte(f(lhs, rhs)));
        DirectiveError::NoError
    }

    /// Unary op from `S` to `D`.
    fn conv_op<S: StackValue, D: StackValue>(&mut self, f: impl FnOnce(S) -> D) -> DirectiveError {
        if !self.has(S::SIZE) {
            return DirectiveError::StackUnderflow;
        }
        let value = self.runtime.stack.pop::<S>();
        self.push(f(value));
        DirectiveError::NoError
    }

    /// Binary float op.
    fn float_op(&mut self, f: impl FnOnce(f64, f64) -> f64) -> DirectiveError {
        if !self.has(8 * 2) {
            return DirectiveError::StackUnderflow;
        }
        let (lhs, rhs) = pop2::<f64>(self);
        self.push(f(lhs, rhs));
        DirectiveError::NoError
    }

    /// `OR` (bitwise on bytes).
    pub fn op_or(&mut self) -> DirectiveError {
        if !self.has(2) {
            return DirectiveError::StackUnderflow;
        }
        let (lhs, rhs) = pop2::<u8>(self);
        self.push(lhs | rhs);
        DirectiveError::NoError
    }

    /// `AND` (bitwise on bytes).
    pub fn op_and(&mut self) -> DirectiveError {
        if !self.has(2) {
            return DirectiveError::StackUnderflow;
        }
        let (lhs, rhs) = pop2::<u8>(self);
        self.push(lhs & rhs);
        DirectiveError::NoError
    }

    /// `IEQ`.
    pub fn op_ieq(&mut self) -> DirectiveError {
        self.cmp_op::<i64>(|l, r| l == r)
    }

    /// `INE`.
    pub fn op_ine(&mut self) -> DirectiveError {
        self.cmp_op::<i64>(|l, r| l != r)
    }

    /// `ULT`.
    pub fn op_ult(&mut self) -> DirectiveError {
        self.cmp_op::<u64>(|l, r| l < r)
    }

    /// `ULE`.
    pub fn op_ule(&mut self) -> DirectiveError {
        self.cmp_op::<u64>(|l, r| l <= r)
    }

    /// `UGT`.
    pub fn op_ugt(&mut self) -> DirectiveError {
        self.cmp_op::<u64>(|l, r| l > r)
    }

    /// `UGE`.
    pub fn op_uge(&mut self) -> DirectiveError {
        self.cmp_op::<u64>(|l, r| l >= r)
    }

    /// `SLT`.
    pub fn op_slt(&mut self) -> DirectiveError {
        self.cmp_op::<i64>(|l, r| l < r)
    }

    /// `SLE`.
    pub fn op_sle(&mut self) -> DirectiveError {
        self.cmp_op::<i64>(|l, r| l <= r)
    }

    /// `SGT`.
    pub fn op_sgt(&mut self) -> DirectiveError {
        self.cmp_op::<i64>(|l, r| l > r)
    }

    /// `SGE`.
    pub fn op_sge(&mut self) -> DirectiveError {
        self.cmp_op::<i64>(|l, r| l >= r)
    }

    /// `FEQ` (false if either is NaN).
    pub fn op_feq(&mut self) -> DirectiveError {
        self.cmp_op::<f64>(|l, r| l == r)
    }

    /// `FNE` (true if either is NaN).
    pub fn op_fne(&mut self) -> DirectiveError {
        self.cmp_op::<f64>(|l, r| l != r)
    }

    /// `FLT` (`std::isless`: false with a NaN).
    pub fn op_flt(&mut self) -> DirectiveError {
        self.cmp_op::<f64>(|l, r| l < r)
    }

    /// `FLE` (`std::islessequal`).
    pub fn op_fle(&mut self) -> DirectiveError {
        self.cmp_op::<f64>(|l, r| l <= r)
    }

    /// `FGT` (`std::isgreater`).
    pub fn op_fgt(&mut self) -> DirectiveError {
        self.cmp_op::<f64>(|l, r| l > r)
    }

    /// `FGE` (`std::isgreaterequal`).
    pub fn op_fge(&mut self) -> DirectiveError {
        self.cmp_op::<f64>(|l, r| l >= r)
    }

    /// `NOT`: true when the byte is zero.
    pub fn op_not(&mut self) -> DirectiveError {
        self.conv_op::<u8, u8>(|v| bool_byte(v == 0))
    }

    /// `FPEXT`: F32 to F64.
    pub fn op_fpext(&mut self) -> DirectiveError {
        self.conv_op::<f32, f64>(f64::from)
    }

    /// `FPTRUNC`: F64 to F32.
    pub fn op_fptrunc(&mut self) -> DirectiveError {
        self.conv_op::<f64, f32>(|v| v as f32)
    }

    /// `FPTOSI`: NaN to 0, out-of-range values clamp, in-range values
    /// truncate toward zero (Rust's `as` saturates exactly so).
    pub fn op_fptosi(&mut self) -> DirectiveError {
        self.conv_op::<f64, i64>(|v| v as i64)
    }

    /// `FPTOUI`: NaN and negatives to 0, values from 2^64 clamp to
    /// `U64` max (Rust's `as` saturates exactly so).
    pub fn op_fptoui(&mut self) -> DirectiveError {
        self.conv_op::<f64, u64>(|v| v as u64)
    }

    /// `SITOFP`.
    pub fn op_sitofp(&mut self) -> DirectiveError {
        self.conv_op::<i64, f64>(|v| v as f64)
    }

    /// `UITOFP`.
    pub fn op_uitofp(&mut self) -> DirectiveError {
        self.conv_op::<u64, f64>(|v| v as f64)
    }

    /// `ADD` with overflow/underflow detection.
    pub fn op_add(&mut self) -> DirectiveError {
        if !self.has(16) {
            return DirectiveError::StackUnderflow;
        }
        let (lhs, rhs) = pop2::<i64>(self);
        if rhs > 0 && lhs > 0 && (i64::MAX - rhs) < lhs {
            return DirectiveError::ArithmeticOverflow;
        } else if rhs < 0 && lhs < 0 && (i64::MIN - rhs) > lhs {
            return DirectiveError::ArithmeticUnderflow;
        }
        self.push(lhs + rhs);
        DirectiveError::NoError
    }

    /// `SUB` with overflow/underflow detection.
    pub fn op_sub(&mut self) -> DirectiveError {
        if !self.has(16) {
            return DirectiveError::StackUnderflow;
        }
        let (lhs, rhs) = pop2::<i64>(self);
        if rhs < 0 && lhs > 0 && (i64::MAX + rhs) < lhs {
            return DirectiveError::ArithmeticOverflow;
        } else if rhs > 0 && lhs < 0 && (i64::MIN + rhs) > lhs {
            return DirectiveError::ArithmeticUnderflow;
        }
        self.push(lhs - rhs);
        DirectiveError::NoError
    }

    /// `MUL` with overflow (same signs) / underflow (differing signs)
    /// detection, without negating `I64` min.
    pub fn op_mul(&mut self) -> DirectiveError {
        if !self.has(16) {
            return DirectiveError::StackUnderflow;
        }
        let (lhs, rhs) = pop2::<i64>(self);
        // same signs overflow; the both-negative check compares without
        // negating, as negating I64 min is undefined in C++
        let overflow = (rhs > 0 && lhs > 0 && (i64::MAX / rhs) < lhs)
            || (rhs < 0 && lhs < 0 && lhs < (i64::MAX / rhs));
        if overflow {
            return DirectiveError::ArithmeticOverflow;
        }
        // differing signs underflow
        let underflow = (rhs < 0 && lhs > 0 && (i64::MIN / lhs) > rhs)
            || (rhs > 0 && lhs < 0 && (i64::MIN / rhs) > lhs);
        if underflow {
            return DirectiveError::ArithmeticUnderflow;
        }
        self.push(lhs * rhs);
        DirectiveError::NoError
    }

    /// `UDIV`; division by zero is a domain error.
    pub fn op_udiv(&mut self) -> DirectiveError {
        if !self.has(16) {
            return DirectiveError::StackUnderflow;
        }
        let (lhs, rhs) = pop2::<u64>(self);
        if rhs == 0 {
            return DirectiveError::DomainError;
        }
        self.push(lhs / rhs);
        DirectiveError::NoError
    }

    /// `SDIV`: floored (Python) division; division by zero is a domain
    /// error and `I64 min / -1` overflows.
    pub fn op_sdiv(&mut self) -> DirectiveError {
        if !self.has(16) {
            return DirectiveError::StackUnderflow;
        }
        let (lhs, rhs) = pop2::<i64>(self);
        if rhs == 0 {
            return DirectiveError::DomainError;
        }
        if lhs == i64::MIN && rhs == -1 {
            return DirectiveError::ArithmeticOverflow;
        }
        let mut quotient = lhs / rhs;
        if lhs % rhs != 0 && ((lhs < 0) != (rhs < 0)) {
            quotient -= 1;
        }
        self.push(quotient);
        DirectiveError::NoError
    }

    /// `UMOD`. C++ parity: the divisor is popped and checked first, so a
    /// zero divisor leaves the dividend on the stack.
    pub fn op_umod(&mut self) -> DirectiveError {
        if !self.has(16) {
            return DirectiveError::StackUnderflow;
        }
        let rhs = self.runtime.stack.pop::<u64>();
        if rhs == 0 {
            return DirectiveError::DomainError;
        }
        let lhs = self.runtime.stack.pop::<u64>();
        self.push(lhs % rhs);
        DirectiveError::NoError
    }

    /// `SMOD`: floored (Python) modulo; `I64 min % -1` is 0. The divisor is
    /// popped and checked first (C++ parity).
    pub fn op_smod(&mut self) -> DirectiveError {
        if !self.has(16) {
            return DirectiveError::StackUnderflow;
        }
        let rhs = self.runtime.stack.pop::<i64>();
        if rhs == 0 {
            return DirectiveError::DomainError;
        }
        let lhs = self.runtime.stack.pop::<i64>();
        if lhs == i64::MIN && rhs == -1 {
            self.push(0i64);
            return DirectiveError::NoError;
        }
        let mut res = lhs % rhs;
        if (res > 0 && rhs < 0) || (res < 0 && rhs > 0) {
            res += rhs;
        }
        self.push(res);
        DirectiveError::NoError
    }

    /// `FADD`.
    pub fn op_fadd(&mut self) -> DirectiveError {
        self.float_op(|l, r| l + r)
    }

    /// `FSUB`.
    pub fn op_fsub(&mut self) -> DirectiveError {
        self.float_op(|l, r| l - r)
    }

    /// `FMUL`.
    pub fn op_fmul(&mut self) -> DirectiveError {
        self.float_op(|l, r| l * r)
    }

    /// `FDIV`.
    pub fn op_fdiv(&mut self) -> DirectiveError {
        self.float_op(|l, r| l / r)
    }

    /// `FPOW`.
    pub fn op_fpow(&mut self) -> DirectiveError {
        self.float_op(f64::powf)
    }

    /// `FLOG`: natural log; a non-positive operand is a domain error.
    pub fn op_flog(&mut self) -> DirectiveError {
        if !self.has(8) {
            return DirectiveError::StackUnderflow;
        }
        let value = self.runtime.stack.pop::<f64>();
        if value <= 0.0 {
            return DirectiveError::DomainError;
        }
        self.push(value.ln());
        DirectiveError::NoError
    }

    /// `FMOD`: floored (Python) float modulo; an exact multiple gives a
    /// zero with the divisor's sign; a zero divisor gives NaN.
    pub fn op_fmod(&mut self) -> DirectiveError {
        if !self.has(16) {
            return DirectiveError::StackUnderflow;
        }
        let (lhs, rhs) = pop2::<f64>(self);
        // `%` on f64 is the exact truncated remainder (C `fmod`)
        let mut res = lhs % rhs;
        if (res > 0.0 && rhs < 0.0) || (res < 0.0 && rhs > 0.0) {
            res += rhs;
        } else if res == 0.0 {
            res = 0.0f64.copysign(rhs);
        }
        self.push(res);
        DirectiveError::NoError
    }

    /// `SIEXT_8_64`.
    pub fn op_siext_8_64(&mut self) -> DirectiveError {
        self.conv_op::<i8, i64>(i64::from)
    }

    /// `SIEXT_16_64`.
    pub fn op_siext_16_64(&mut self) -> DirectiveError {
        self.conv_op::<i16, i64>(i64::from)
    }

    /// `SIEXT_32_64`.
    pub fn op_siext_32_64(&mut self) -> DirectiveError {
        self.conv_op::<i32, i64>(i64::from)
    }

    /// `ZIEXT_8_64`.
    pub fn op_ziext_8_64(&mut self) -> DirectiveError {
        self.conv_op::<u8, u64>(u64::from)
    }

    /// `ZIEXT_16_64`.
    pub fn op_ziext_16_64(&mut self) -> DirectiveError {
        self.conv_op::<u16, u64>(u64::from)
    }

    /// `ZIEXT_32_64`.
    pub fn op_ziext_32_64(&mut self) -> DirectiveError {
        self.conv_op::<u32, u64>(u64::from)
    }

    /// `ITRUNC_64_8`.
    pub fn op_itrunc_64_8(&mut self) -> DirectiveError {
        self.conv_op::<u64, u8>(|v| v as u8)
    }

    /// `ITRUNC_64_16`.
    pub fn op_itrunc_64_16(&mut self) -> DirectiveError {
        self.conv_op::<u64, u16>(|v| v as u16)
    }

    /// `ITRUNC_64_32`.
    pub fn op_itrunc_64_32(&mut self) -> DirectiveError {
        self.conv_op::<u64, u32>(|v| v as u32)
    }

    /// `FFLOOR` (IEEE roundToIntegralTowardNegative: zeros, infinities and
    /// NaN pass through).
    pub fn op_ffloor(&mut self) -> DirectiveError {
        self.conv_op::<f64, f64>(f64::floor)
    }

    /// `IABS`; `abs(I64 min)` overflows.
    pub fn op_iabs(&mut self) -> DirectiveError {
        if !self.has(8) {
            return DirectiveError::StackUnderflow;
        }
        let value = self.runtime.stack.pop::<i64>();
        if value == i64::MIN {
            return DirectiveError::ArithmeticOverflow;
        }
        self.push(value.abs());
        DirectiveError::NoError
    }

    /// `FABS` (clears the sign bit only).
    pub fn op_fabs(&mut self) -> DirectiveError {
        self.conv_op::<f64, f64>(f64::abs)
    }
}
