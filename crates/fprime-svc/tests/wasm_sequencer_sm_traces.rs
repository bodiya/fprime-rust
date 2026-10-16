//! Differential test of the two `Svc.WasmSequencer` state machines — the
//! `fpp_state_machine!` definitions the component itself uses
//! (`fprime_svc::wasm_sequencer::{controller_sm, interpreter_sm}`) —
//! against the reference compiler: for every leaf state, signal and guard
//! outcome, the trace of actions and guards and the final state must equal
//! the golden trace that `tools/sm-traces.py` extracted from the C++
//! `fpp-to-cpp` 3.3.0 generates for `WasmSequencerControllerStateMachine.fppi`
//! and `WasmSequencerInterpreterStateMachine.fppi`
//! (`crates/fprime-comp/tests/sm_traces/WasmSequencer{Controller,Interpreter}.txt`).
// The component exists only with the `wasm` feature, and runs sequence
// files: `std`-only.
#![cfg(all(feature = "wasm", feature = "std"))]
#![allow(non_snake_case)]

#[path = "../../fprime-comp/tests/sm_trace_replay/mod.rs"]
mod sm_trace_replay;

use fprime_comp::config::{FwEnumStoreType, FwIndexType};
use fprime_svc::wasm_sequencer::{
    CommandRequest, HostFunction, InvokeRequest, LoadRequest, RequestContext, TrapReason,
    controller_sm, interpreter_sm,
};
use sm_trace_replay::{Recorder, check};

/// The recording handlers: every action and guard by its FPP name.
struct R<'a>(&'a Recorder);

macro_rules! actions {
    ($($fn:ident => $name:literal $(($ty:ty))?;)*) => {
        $(
            fn $fn(&self, _: FwEnumStoreType, _: Signal $(, _: $ty)?) {
                self.0.action($name);
            }
        )*
    };
}

macro_rules! guards {
    ($($fn:ident => $name:literal $(($ty:ty))?;)*) => {
        $(
            fn $fn(&self, _: FwEnumStoreType, _: Signal $(, _: $ty)?) -> bool {
                self.0.guard($name)
            }
        )*
    };
}

mod controller {
    use super::*;
    use controller_sm::{Handlers, Signal, State, StateMachine};

    impl Handlers for R<'_> {
        actions! {
            action_processInvoke => "processInvoke" (&InvokeRequest);
            action_setCancelRequested => "setCancelRequested";
            action_clearCancelRequested => "clearCancelRequested";
            action_cancelPendingRequest => "cancelPendingRequest" (&RequestContext);
            action_respond_noblock_OK => "respond_noblock_OK" (&RequestContext);
            action_respond_ERROR => "respond_ERROR" (&RequestContext);
            action_incrementSequenceFailure => "incrementSequenceFailure";
            action_respondInvoke_BUSY => "respondInvoke_BUSY" (&InvokeRequest);
            action_respondInvoke_ERROR => "respondInvoke_ERROR" (&InvokeRequest);
            action_respondLoad_BUSY => "respondLoad_BUSY" (&LoadRequest);
            action_respond_block_OK => "respond_block_OK" (&RequestContext);
            action_respond_block_ERROR => "respond_block_ERROR" (&RequestContext);
            action_load => "load" (&LoadRequest);
            action_invokeStart => "invokeStart" (&RequestContext);
            action_invokeMain => "invokeMain" (&RequestContext);
            action_reportModuleInvalidMain => "reportModuleInvalidMain" (&RequestContext);
            action_reportModuleMainInvokeFailed => "reportModuleMainInvokeFailed" (&RequestContext);
            action_reportModuleStartInvokeFailed => "reportModuleStartInvokeFailed" (&RequestContext);
            action_reportModuleStarted => "reportModuleStarted" (&RequestContext);
            action_reportModuleSucceeded => "reportModuleSucceeded" (&RequestContext);
            action_reportModuleStartFailed => "reportModuleStartFailed" (&RequestContext);
            action_reportModuleMainFailed => "reportModuleMainFailed" (&RequestContext);
            action_resetStore => "resetStore";
            action_runEngine => "runEngine" (&RequestContext);
        }
        guards! {
            guard_cancelRequested => "cancelRequested";
            guard_moduleHasStart => "moduleHasStart" (&RequestContext);
            guard_moduleHasValidMain => "moduleHasValidMain" (&RequestContext);
            guard_invokeSucceeded => "invokeSucceeded" (&RequestContext);
            guard_interpreterSucceeded => "interpreterSucceeded";
        }
    }

    fn send(sm: &StateMachine, r: &Recorder, signal: &str) {
        let h = &R(r);
        let load = LoadRequest::default();
        let invoke = InvokeRequest::default();
        let ctx = RequestContext::default();
        match signal {
            "cancel" => sm.send_signal_cancel(h),
            "engineFinished" => sm.send_signal_engineFinished(h, &ctx),
            "invoke" => sm.send_signal_invoke(h, &invoke),
            "invokeFailed" => sm.send_signal_invokeFailed(h, &ctx),
            "invoked" => sm.send_signal_invoked(h, &ctx),
            "load" => sm.send_signal_load(h, &load),
            "loadFailed" => sm.send_signal_loadFailed(h, &ctx),
            "loadSucceeded" => sm.send_signal_loadSucceeded(h, &ctx),
            "run" => sm.send_signal_run(h, &load),
            other => panic!("unknown signal {other}"),
        }
    }

    #[test]
    fn wasm_sequencer_controller_matches_fpp_to_cpp() {
        check(
            include_str!("../../fprime-comp/tests/sm_traces/WasmSequencerController.txt"),
            |r| {
                let sm = StateMachine::new();
                sm.init(0, &R(r));
                sm
            },
            send,
            |sm| format!("{:?}", sm.get_state()),
        );
    }

    /// The `State` and `Signal` discriminants are the `fpp-to-cpp` ones
    /// (sorted by name).
    #[test]
    fn controller_enums_match_fpp_to_cpp() {
        assert_eq!(State::IDLE as u8, 1);
        assert_eq!(State::INVOKING as u8, 2);
        assert_eq!(State::LOADING_TO_READY as u8, 3);
        assert_eq!(State::LOADING_TO_RUN as u8, 4);
        assert_eq!(State::LOAD_RUNNING_START as u8, 5);
        assert_eq!(State::READY as u8, 6);
        assert_eq!(State::RUNNING_MAIN as u8, 7);
        assert_eq!(State::RUNNING_START_PENDING_MAIN as u8, 8);
        assert_eq!(State::NUM_CONSTANTS, 9);
        assert_eq!(Signal::cancel.as_repr(), 1);
        assert_eq!(Signal::run.as_repr(), 9);
    }
}

mod interpreter {
    use super::*;
    use interpreter_sm::{Handlers, Signal, State, StateMachine};

    impl Handlers for R<'_> {
        actions! {
            action_cmdReplyOK => "cmdReplyOK" (&CommandRequest);
            action_signalEntered => "signalEntered";
            action_spin => "spin";
            action_reset => "reset";
            action_clearExitStatus => "clearExitStatus";
            action_setExitReason_INTERPRETER_FINISHED => "setExitReason_INTERPRETER_FINISHED";
            action_setExitReason_INTERPRETER_TRAP => "setExitReason_INTERPRETER_TRAP";
            action_setExitReason_REPLY_TIMEOUT => "setExitReason_REPLY_TIMEOUT";
            action_setExitReason_HOST_FAILURE => "setExitReason_HOST_FAILURE";
            action_setExitReason_TIMER_INCOMPARABLE => "setExitReason_TIMER_INCOMPARABLE";
            action_setExitReason_UNEXPECTED_REPLY => "setExitReason_UNEXPECTED_REPLY";
            action_setExitReason_CANCEL => "setExitReason_CANCEL";
            action_setExitCode => "setExitCode" (i32);
            action_setTrapReason => "setTrapReason" (TrapReason);
            action_setLastHostFunction => "setLastHostFunction";
            action_finish => "finish";
            action_reportPaused => "reportPaused";
            action_clearPause => "clearPause";
            action_dispatchPendingHostFunction => "dispatchPendingHostFunction";
            action_clearPendingHostFunction => "clearPendingHostFunction";
            action_setContext => "setContext" (&RequestContext);
            action_clearContext => "clearContext";
            action_resume => "resume";
            action_resumeI32 => "resumeI32" (i32);
            action_checkSleepTimers => "checkSleepTimers";
            action_checkTimeout => "checkTimeout";
            action_dequeueSerialAndResume => "dequeueSerialAndResume" (FwIndexType);
        }
        guards! {
            guard_pendingPause => "pendingPause";
            guard_pendingHostFunction => "pendingHostFunction";
            guard_pendingHostFunctionIsSleep => "pendingHostFunctionIsSleep";
            guard_blockingSerialIn => "blockingSerialIn" (FwIndexType);
            guard_dequeueSucceeded => "dequeueSucceeded";
        }
    }

    fn send(sm: &StateMachine, r: &Recorder, signal: &str) {
        let h = &R(r);
        let ctx = RequestContext::default();
        let cmd = CommandRequest::default();
        match signal {
            "cancel" => sm.send_signal_cancel(h),
            "checkTimers" => sm.send_signal_checkTimers(h),
            "cmdCancel" => sm.send_signal_cmdCancel(h, &cmd),
            "cmd_CONTINUE" => sm.send_signal_cmd_CONTINUE(h),
            "entered" => sm.send_signal_entered(h),
            "hostResponseFailure" => sm.send_signal_hostResponseFailure(h),
            "hostResponseTimeIncomparable" => sm.send_signal_hostResponseTimeIncomparable(h),
            "hostResponseTimeout" => sm.send_signal_hostResponseTimeout(h),
            "hostResponseUnexpected" => {
                sm.send_signal_hostResponseUnexpected(h, HostFunction::None)
            }
            "hostResume" => sm.send_signal_hostResume(h),
            "hostResumeI32" => sm.send_signal_hostResumeI32(h, 0),
            "interpreterFinished" => sm.send_signal_interpreterFinished(h, 0),
            "interpreterHostFunctionNeedsPause" => {
                sm.send_signal_interpreterHostFunctionNeedsPause(h)
            }
            "interpreterOutOfFuel" => sm.send_signal_interpreterOutOfFuel(h),
            "interpreterTrap" => sm.send_signal_interpreterTrap(h, TrapReason::None),
            "run" => sm.send_signal_run(h, &ctx),
            "serialInMessage" => sm.send_signal_serialInMessage(h, 0),
            other => panic!("unknown signal {other}"),
        }
    }

    #[test]
    fn wasm_sequencer_interpreter_matches_fpp_to_cpp() {
        check(
            include_str!("../../fprime-comp/tests/sm_traces/WasmSequencerInterpreter.txt"),
            |r| {
                let sm = StateMachine::new();
                sm.init(1, &R(r));
                sm
            },
            send,
            |sm| format!("{:?}", sm.get_state()),
        );
    }

    #[test]
    fn interpreter_enums_match_fpp_to_cpp() {
        assert_eq!(State::IDLE as u8, 1);
        assert_eq!(State::RUNNING_AWAITING_RESPONSE_SLEEPING as u8, 2);
        assert_eq!(State::RUNNING_AWAITING_RESPONSE_WAITING as u8, 3);
        assert_eq!(State::RUNNING_PAUSED as u8, 4);
        assert_eq!(State::RUNNING_SPINNING as u8, 5);
        assert_eq!(State::NUM_CONSTANTS, 6);
        assert_eq!(Signal::cancel.as_repr(), 1);
        assert_eq!(Signal::serialInMessage.as_repr(), 17);
    }
}
