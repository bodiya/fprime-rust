//! Running a sequence — port of `FpySequencerRunState.cpp`: dispatching
//! the next statement (deserializing its directive and sending it to the
//! directive's internal port), and the two timer checks driven by
//! `checkTimers`.

use super::{
    DirectiveUnion, FpySequencer, FpyState, Signal, Statement, get_event_opcode, types::*,
};
use fprime_config::fpy_sequencer::MAX_DIRECTIVE_SIZE;
use fprime_fw::{Endianness, LengthMode, SerBuf, SerializeStatus, Time, TimeComparison, fw_assert};

impl FpySequencer {
    /// C++ `dispatchStatement`: deserialize and dispatch the next
    /// statement. Returns `result_dispatchStatement_noMoreStatements` at the
    /// end of the sequence, `..._failure` if the directive does not
    /// deserialize, `..._success` otherwise. Asserts that the next index is
    /// at most one past the last statement.
    pub(crate) fn dispatch_statement(&self, st: &mut FpyState) -> Signal {
        let count = u32::from(st.sequence_obj.header.statement_count);
        // out of bounds by at most one, which means end of file
        fw_assert!(
            st.runtime.next_statement_index <= count,
            st.runtime.next_statement_index as i32
        );
        if st.runtime.next_statement_index == count {
            return Signal::result_dispatchStatement_noMoreStatements;
        }
        let idx = st.runtime.next_statement_index as usize;
        st.runtime.next_statement_index += 1;
        let opcode = st.sequence_obj.statements[idx].op_code;
        st.runtime.current_statement_opcode = opcode.as_repr();
        // not known to be a command until deserialized
        st.runtime.current_cmd_opcode = 0;

        let Some(directive) = self.deserialize_directive(st, &st.sequence_obj.statements[idx])
        else {
            return Signal::result_dispatchStatement_failure;
        };
        if let DirectiveUnion::ConstCmd(cmd) = &directive {
            st.runtime.current_cmd_opcode = cmd.op_code;
        }
        self.dispatch_directive(&directive, opcode);
        // the dispatch time is taken right after a successful dispatch
        st.runtime.current_statement_dispatch_time = self.evt.time_get();
        st.statements_dispatched += 1;
        Signal::result_dispatchStatement_success
    }

    /// C++ `deserializeDirective`: decode a statement's argument bytes into
    /// its directive. A decode failure or a leftover byte logs
    /// `DirectiveDeserializeError`; an `INVALID` opcode logs
    /// `UnknownSequencerDirective`. Returns `None` on failure.
    pub(crate) fn deserialize_directive(
        &self,
        st: &FpyState,
        stmt: &Statement,
    ) -> Option<DirectiveUnion> {
        // our own buffer over the statement's bytes, read from the start
        let mut arg_buf = stmt.arg_buf.clone();
        arg_buf.reset_deser();
        let opcode = stmt.op_code.as_repr();
        let idx = st.current_statement_idx();
        let size = arg_buf.get_size() as u64;
        let deser_error = |status: SerializeStatus, left: usize| {
            self.log_warning_hi_directive_deserialize_error(
                opcode,
                idx,
                status as i32,
                left as u64,
                size,
            );
        };

        // directives without arguments: nothing may follow the opcode
        macro_rules! empty {
            ($variant:ident, $ty:ty) => {{
                let left = arg_buf.deserialize_size_left();
                if left != 0 {
                    deser_error(SerializeStatus::DeserSizeMismatch, left);
                    return None;
                }
                Some(DirectiveUnion::$variant(<$ty>::default()))
            }};
        }
        // directives whose arguments are their FPP struct
        macro_rules! fpp {
            ($variant:ident, $ty:ty) => {{
                let mut directive = <$ty>::default();
                let status = arg_buf.deserialize(&mut directive, Endianness::Big);
                let left = arg_buf.deserialize_size_left();
                if status != SerializeStatus::Ok || left != 0 {
                    deser_error(status, left);
                    return None;
                }
                Some(DirectiveUnion::$variant(directive))
            }};
        }

        match stmt.op_code {
            DirectiveId::WaitRel => empty!(WaitRel, WaitRelDirective),
            DirectiveId::WaitAbs => empty!(WaitAbs, WaitAbsDirective),
            DirectiveId::Goto => fpp!(Goto, GotoDirective),
            DirectiveId::If => fpp!(If, IfDirective),
            DirectiveId::NoOp => empty!(NoOp, NoOpDirective),
            DirectiveId::PushTlmVal => fpp!(PushTlmVal, PushTlmValDirective),
            DirectiveId::PushTlmValAndTime => fpp!(PushTlmValAndTime, PushTlmValAndTimeDirective),
            DirectiveId::PushPrm => fpp!(PushPrm, PushPrmDirective),
            DirectiveId::ConstCmd => {
                let mut directive = ConstCmdDirective::default();
                // the opcode, then every remaining byte is the argument buffer
                let status = arg_buf.deserialize_u32_be(&mut directive.op_code);
                if status != SerializeStatus::Ok {
                    deser_error(status, arg_buf.deserialize_size_left());
                    return None;
                }
                let left = arg_buf.deserialize_size_left();
                if left > MAX_DIRECTIVE_SIZE {
                    deser_error(SerializeStatus::DeserFormatError, left);
                    return None;
                }
                let mut len = left;
                let status = arg_buf.deserialize_bytes(
                    &mut directive.arg_buf.0[..left],
                    &mut len,
                    LengthMode::OmitLength,
                    Endianness::Big,
                );
                if status != SerializeStatus::Ok {
                    deser_error(status, arg_buf.deserialize_size_left());
                    return None;
                }
                fw_assert!(
                    arg_buf.deserialize_size_left() == 0,
                    arg_buf.deserialize_size_left() as i32
                );
                directive.arg_buf_size = left as u64;
                Some(DirectiveUnion::ConstCmd(directive))
            }
            op if op.is_stack_op() => {
                let left = arg_buf.deserialize_size_left();
                if left != 0 {
                    deser_error(SerializeStatus::DeserSizeMismatch, left);
                    return None;
                }
                Some(DirectiveUnion::StackOp(StackOpDirective::new(op)))
            }
            DirectiveId::Exit => empty!(Exit, ExitDirective),
            DirectiveId::Allocate => fpp!(Allocate, AllocateDirective),
            DirectiveId::StoreRelConstOffset => {
                fpp!(StoreRelConstOffset, StoreRelConstOffsetDirective)
            }
            DirectiveId::LoadRel => fpp!(LoadRel, LoadRelDirective),
            DirectiveId::PushVal => {
                let mut directive = PushValDirective::default();
                let left = arg_buf.deserialize_size_left();
                if left > MAX_DIRECTIVE_SIZE {
                    deser_error(SerializeStatus::DeserFormatError, left);
                    return None;
                }
                let mut len = left;
                let status = arg_buf.deserialize_bytes(
                    &mut directive.val.0[..left],
                    &mut len,
                    LengthMode::OmitLength,
                    Endianness::Big,
                );
                if status != SerializeStatus::Ok {
                    deser_error(status, arg_buf.deserialize_size_left());
                    return None;
                }
                fw_assert!(
                    arg_buf.deserialize_size_left() == 0,
                    arg_buf.deserialize_size_left() as i32
                );
                directive.val_size = left as u64;
                Some(DirectiveUnion::PushVal(directive))
            }
            DirectiveId::Discard => fpp!(Discard, DiscardDirective),
            DirectiveId::Memcmp => fpp!(MemCmp, MemCmpDirective),
            DirectiveId::StackCmd => fpp!(StackCmd, StackCmdDirective),
            DirectiveId::PushTime => empty!(PushTime, PushTimeDirective),
            DirectiveId::SetSeed => empty!(SetSeed, SetSeedDirective),
            DirectiveId::PushRand => empty!(PushRand, PushRandDirective),
            DirectiveId::GetField => fpp!(GetField, GetFieldDirective),
            DirectiveId::Peek => empty!(Peek, PeekDirective),
            DirectiveId::StoreRel => fpp!(StoreRel, StoreRelDirective),
            DirectiveId::Call => empty!(Call, CallDirective),
            DirectiveId::Return => fpp!(Return, ReturnDirective),
            DirectiveId::LoadAbs => fpp!(LoadAbs, LoadAbsDirective),
            DirectiveId::StoreAbs => fpp!(StoreAbs, StoreAbsDirective),
            DirectiveId::StoreAbsConstOffset => {
                fpp!(StoreAbsConstOffset, StoreAbsConstOffsetDirective)
            }
            DirectiveId::PopEvent => empty!(PopEvent, PopEventDirective),
            DirectiveId::PopSerializable => fpp!(PopSerializable, PopSerializableDirective),
            // INVALID (and, unreachably, a stack op the guard missed): a
            // compiler/sequencer version mismatch
            _ => {
                self.log_warning_hi_unknown_sequencer_directive(
                    opcode,
                    idx,
                    st.sequence_file_path.as_bytes(),
                );
                None
            }
        }
    }

    /// C++ `checkShouldWake`: wake once the current time reaches the wakeup
    /// time; a different time base fails the sequence (the context is not
    /// compared).
    pub(crate) fn check_should_wake(&self, st: &FpyState) -> Signal {
        let current = self.evt.time_get();
        let wakeup = st.runtime.wakeup_time;
        if current.get_time_base().as_repr() != wakeup.time_base {
            self.log_warning_hi_mismatched_time_base(
                i32::from(current.get_time_base().as_repr()),
                i32::from(wakeup.time_base),
            );
            return Signal::result_timeOpFailed;
        }
        if (current.get_seconds(), current.get_useconds()) < (wakeup.seconds, wakeup.useconds) {
            return Signal::result_checkShouldWake_keepSleeping;
        }
        Signal::result_checkShouldWake_wakeup
    }

    /// C++ `checkStatementTimeout`: compare the time since the current
    /// statement was dispatched with `STATEMENT_TIMEOUT_SECS` (no timeout
    /// when it is not positive or exceeds `U32` max seconds).
    pub(crate) fn check_statement_timeout(&self, st: &FpyState) -> Signal {
        let (timeout, _valid) = self.param_get_statement_timeout_secs();
        if timeout <= 0.0 || timeout > u32::MAX as f32 {
            return Signal::result_checkStatementTimeout_noTimeout;
        }
        let current = self.evt.time_get();
        let dispatch = st.runtime.current_statement_dispatch_time;
        if Time::compare(&current, &dispatch) == TimeComparison::Incomparable {
            // the time base changed
            self.log_warning_hi_mismatched_time_base(
                i32::from(current.get_time_base().as_repr()),
                i32::from(dispatch.get_time_base().as_repr()),
            );
            return Signal::result_timeOpFailed;
        }
        // the context is not compared. Time running backwards is ignored.
        if dispatch.get_seconds() > current.get_seconds() {
            return Signal::result_checkStatementTimeout_noTimeout;
        }
        if dispatch.get_seconds() == current.get_seconds()
            && dispatch.get_useconds() > current.get_useconds()
        {
            return Signal::result_checkStatementTimeout_noTimeout;
        }
        let current_us =
            u64::from(current.get_seconds()) * 1_000_000 + u64::from(current.get_useconds());
        let dispatch_us =
            u64::from(dispatch.get_seconds()) * 1_000_000 + u64::from(dispatch.get_useconds());
        // C++ parity: the product is computed in F32, then converted. (A NaN
        // timeout passes the range check in both languages; C++ then
        // converts NaN, which is undefined, and Rust gets 0 — an immediate
        // timeout.)
        let timeout_us = (timeout * 1_000_000.0f32) as u64;
        if current_us - dispatch_us < timeout_us {
            return Signal::result_checkStatementTimeout_noTimeout;
        }
        let opcode = st.runtime.current_statement_opcode;
        if opcode == DirectiveId::ConstCmd.as_repr() || opcode == DirectiveId::StackCmd.as_repr() {
            self.log_warning_hi_command_timed_out(
                get_event_opcode(st.runtime.current_cmd_opcode),
                st.current_statement_idx(),
                st.sequence_file_path.as_bytes(),
            );
        } else {
            self.log_warning_hi_directive_timed_out(
                opcode,
                st.current_statement_idx(),
                st.sequence_file_path.as_bytes(),
            );
        }
        Signal::result_checkStatementTimeout_statementTimeout
    }
}
