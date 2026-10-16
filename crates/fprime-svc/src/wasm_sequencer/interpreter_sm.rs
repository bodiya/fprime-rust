//! `Svc.WasmSequencer.InterpreterStateMachine`
//! (`WasmSequencerInterpreterStateMachine.fppi`): the instruction-execution
//! machine. Its golden traces (from the C++ `fpp-to-cpp` 3.3.0 generates)
//! are checked by `tests/wasm_sequencer_sm_traces.rs`.
#![allow(non_snake_case, non_camel_case_types)]

use super::types::{CommandRequest, HostFunction, RequestContext, TrapReason};
use fprime_config::FwIndexType;

fprime_comp::fpp_state_machine! {
    /// `Svc.WasmSequencer.InterpreterStateMachine`.
    state machine InterpreterStateMachine: u8 {
        actions {
            cmdReplyOK => action_cmdReplyOK (ref CommandRequest),
            signalEntered => action_signalEntered,
            spin => action_spin,
            reset => action_reset,
            clearExitStatus => action_clearExitStatus,
            setExitReason_INTERPRETER_FINISHED => action_setExitReason_INTERPRETER_FINISHED,
            setExitReason_INTERPRETER_TRAP => action_setExitReason_INTERPRETER_TRAP,
            setExitReason_REPLY_TIMEOUT => action_setExitReason_REPLY_TIMEOUT,
            setExitReason_HOST_FAILURE => action_setExitReason_HOST_FAILURE,
            setExitReason_TIMER_INCOMPARABLE => action_setExitReason_TIMER_INCOMPARABLE,
            setExitReason_UNEXPECTED_REPLY => action_setExitReason_UNEXPECTED_REPLY,
            setExitReason_CANCEL => action_setExitReason_CANCEL,
            setExitCode => action_setExitCode (val i32),
            setTrapReason => action_setTrapReason (val TrapReason),
            setLastHostFunction => action_setLastHostFunction,
            finish => action_finish,
            reportPaused => action_reportPaused,
            clearPause => action_clearPause,
            dispatchPendingHostFunction => action_dispatchPendingHostFunction,
            clearPendingHostFunction => action_clearPendingHostFunction,
            setContext => action_setContext (ref RequestContext),
            clearContext => action_clearContext,
            resume => action_resume,
            resumeI32 => action_resumeI32 (val i32),
            checkSleepTimers => action_checkSleepTimers,
            checkTimeout => action_checkTimeout,
            dequeueSerialAndResume => action_dequeueSerialAndResume (val FwIndexType),
        }
        guards {
            pendingPause => guard_pendingPause,
            pendingHostFunction => guard_pendingHostFunction,
            pendingHostFunctionIsSleep => guard_pendingHostFunctionIsSleep,
            blockingSerialIn => guard_blockingSerialIn (val FwIndexType),
            dequeueSucceeded => guard_dequeueSucceeded,
        }
        signals {
            cancel = 1 => send_signal_cancel {
                RUNNING => do { setExitReason_CANCEL } enter IDLE;
            }
            checkTimers = 2 => send_signal_checkTimers {
                RUNNING_AWAITING_RESPONSE_SLEEPING => do { checkSleepTimers };
                RUNNING_AWAITING_RESPONSE => do { checkTimeout };
            }
            cmdCancel = 3 => send_signal_cmdCancel (ref CommandRequest) {
                IDLE => do { cmdReplyOK };
                RUNNING => do { setExitReason_CANCEL, cmdReplyOK } enter IDLE;
            }
            cmd_CONTINUE = 4 => send_signal_cmd_CONTINUE {
                RUNNING_PAUSED => enter RUNNING_PAUSED_RESUME;
            }
            entered = 5 => send_signal_entered {
                RUNNING_SPINNING => do { spin };
            }
            hostResponseFailure = 6 => send_signal_hostResponseFailure {
                RUNNING_AWAITING_RESPONSE => do { setExitReason_HOST_FAILURE } enter IDLE;
            }
            hostResponseTimeIncomparable = 7 => send_signal_hostResponseTimeIncomparable {
                RUNNING_AWAITING_RESPONSE => do { setExitReason_TIMER_INCOMPARABLE } enter IDLE;
            }
            hostResponseTimeout = 8 => send_signal_hostResponseTimeout {
                RUNNING_AWAITING_RESPONSE => do { setExitReason_REPLY_TIMEOUT } enter IDLE;
            }
            hostResponseUnexpected = 9 => send_signal_hostResponseUnexpected (val HostFunction) {
                RUNNING => do { setExitReason_UNEXPECTED_REPLY } enter IDLE;
            }
            hostResume = 10 => send_signal_hostResume {
                RUNNING_AWAITING_RESPONSE => do { resume } enter RUNNING_PAUSE_CHECK;
            }
            hostResumeI32 = 11 => send_signal_hostResumeI32 (val i32) {
                RUNNING_AWAITING_RESPONSE => do { resumeI32 } enter RUNNING_PAUSE_CHECK;
            }
            interpreterFinished = 12 => send_signal_interpreterFinished (val i32) {
                RUNNING_SPINNING => do { setExitCode, setExitReason_INTERPRETER_FINISHED } enter IDLE;
            }
            interpreterHostFunctionNeedsPause = 13 => send_signal_interpreterHostFunctionNeedsPause {
                RUNNING_SPINNING => enter RUNNING_SPINNING_HOST_FUNCTION;
            }
            interpreterOutOfFuel = 14 => send_signal_interpreterOutOfFuel {
                RUNNING_SPINNING => enter RUNNING_PAUSE_CHECK;
            }
            interpreterTrap = 15 => send_signal_interpreterTrap (val TrapReason) {
                RUNNING_SPINNING => do { setTrapReason, setExitReason_INTERPRETER_TRAP } enter IDLE;
            }
            run = 16 => send_signal_run (ref RequestContext) {
                IDLE => do { setContext } enter RUNNING;
            }
            serialInMessage = 17 => send_signal_serialInMessage (val FwIndexType) {
                RUNNING_AWAITING_RESPONSE => if blockingSerialIn do { dequeueSerialAndResume }
                    enter RUNNING_AWAITING_RESPONSE_DEQUEUE_CHECK;
            }
        }
        initial enter IDLE;
        states {
            state IDLE = 1 {
                entry do { clearContext, reset }
            }
            state RUNNING {
                initial enter RUNNING_PAUSE_CHECK;
                entry do { clearExitStatus }
                exit do { clearPendingHostFunction, clearPause, finish }
            }
            state RUNNING_SPINNING = 5 in RUNNING {
                entry do { signalEntered }
            }
            state RUNNING_AWAITING_RESPONSE in RUNNING {
                initial do { setLastHostFunction, dispatchPendingHostFunction }
                    enter RUNNING_AWAITING_RESPONSE_PICK_AWAIT;
                exit do { clearPendingHostFunction }
            }
            state RUNNING_AWAITING_RESPONSE_WAITING = 3 in RUNNING_AWAITING_RESPONSE {}
            state RUNNING_AWAITING_RESPONSE_SLEEPING = 2 in RUNNING_AWAITING_RESPONSE {}
            state RUNNING_PAUSED = 4 in RUNNING {
                entry do { clearPause, reportPaused }
            }
        }
        choices {
            choice RUNNING_PAUSE_CHECK in RUNNING {
                if pendingPause enter RUNNING_PAUSED else enter RUNNING_SPINNING
            }
            choice RUNNING_SPINNING_HOST_FUNCTION in RUNNING_SPINNING {
                if pendingPause enter RUNNING_PAUSED else enter RUNNING_AWAITING_RESPONSE
            }
            choice RUNNING_AWAITING_RESPONSE_PICK_AWAIT in RUNNING_AWAITING_RESPONSE {
                if pendingHostFunctionIsSleep enter RUNNING_AWAITING_RESPONSE_SLEEPING
                else enter RUNNING_AWAITING_RESPONSE_WAITING
            }
            choice RUNNING_AWAITING_RESPONSE_DEQUEUE_CHECK in RUNNING_AWAITING_RESPONSE (val FwIndexType) {
                if dequeueSucceeded enter RUNNING_PAUSE_CHECK
                else do { setExitReason_HOST_FAILURE } enter IDLE
            }
            choice RUNNING_PAUSED_RESUME in RUNNING_PAUSED {
                if pendingHostFunction enter RUNNING_AWAITING_RESPONSE else enter RUNNING_SPINNING
            }
        }
    }
}
