//! Differential test of the `Svc.FpySequencer` state machine — the
//! `fpp_state_machine!` definition the component itself uses
//! (`fprime_svc::fpy_sequencer::StateMachine`) — against the reference
//! compiler: for every leaf state, signal and guard outcome, the trace of
//! actions and guards and the final state must equal the golden trace that
//! `tools/sm-traces.py` extracted from the C++ `fpp-to-cpp` 3.3.0
//! generates for `FpySequencerStateMachine.fppi`
//! (`crates/fprime-comp/tests/sm_traces/FpySequencer.txt`). The checker is
//! the one `fprime-comp`'s `sm_traces.rs` uses for its machines.
#![allow(non_snake_case)]

#[path = "../../fprime-comp/tests/sm_trace_replay/mod.rs"]
mod sm_trace_replay;

use fprime_comp::config::FwEnumStoreType;
use fprime_svc::fpy_sequencer::{
    BreakpointArgs, Handlers, SequenceExecutionArgs, Signal, State, StateMachine,
};
use sm_trace_replay::{Recorder, check};

/// The recording handlers: every action and guard by its FPP name.
struct R<'a>(&'a Recorder);

macro_rules! actions {
    ($($fn:ident => $name:literal $(($ty:ty))?;)*) => {
        $(
            fn $fn(&self, _: FwEnumStoreType, _: Signal $(, _: &$ty)?) {
                self.0.action($name);
            }
        )*
    };
}

impl Handlers for R<'_> {
    actions! {
        action_signalEntered => "signalEntered";
        action_setSequenceFilePath => "setSequenceFilePath" (SequenceExecutionArgs);
        action_setSequenceBlockState => "setSequenceBlockState" (SequenceExecutionArgs);
        action_setSequenceArguments => "setSequenceArguments" (SequenceExecutionArgs);
        action_validate => "validate";
        action_report_seqSucceeded => "report_seqSucceeded";
        action_report_seqCancelled => "report_seqCancelled";
        action_report_seqFailed => "report_seqFailed";
        action_report_seqStarted => "report_seqStarted";
        action_setGoalState_RUNNING => "setGoalState_RUNNING";
        action_setGoalState_VALID => "setGoalState_VALID";
        action_setGoalState_IDLE => "setGoalState_IDLE";
        action_sendCmdResponse_OK => "sendCmdResponse_OK";
        action_sendCmdResponse_EXECUTION_ERROR => "sendCmdResponse_EXECUTION_ERROR";
        action_clearSequenceFile => "clearSequenceFile";
        action_clearBreakpoint => "clearBreakpoint";
        action_clearSequenceArguments => "clearSequenceArguments";
        action_checkShouldWake => "checkShouldWake";
        action_dispatchStatement => "dispatchStatement";
        action_resetRuntime => "resetRuntime";
        action_checkStatementTimeout => "checkStatementTimeout";
        action_incrementSequenceCounter => "incrementSequenceCounter";
        action_pushArgsToStack => "pushArgsToStack";
        action_report_seqBroken => "report_seqBroken";
        action_setBreakpoint => "setBreakpoint" (BreakpointArgs);
        action_setBreakBeforeNextLine => "setBreakBeforeNextLine";
        action_clearBreakBeforeNextLine => "clearBreakBeforeNextLine";
    }

    fn guard_goalStateIs_RUNNING(&self, _: FwEnumStoreType, _: Signal) -> bool {
        self.0.guard("goalStateIs_RUNNING")
    }

    fn guard_shouldBreak(&self, _: FwEnumStoreType, _: Signal) -> bool {
        self.0.guard("shouldBreak")
    }

    fn guard_breakOnce(&self, _: FwEnumStoreType, _: Signal) -> bool {
        self.0.guard("breakOnce")
    }
}

/// Send a signal by its FPP name.
fn send(sm: &StateMachine, r: &Recorder, signal: &str) {
    let h = &R(r);
    let args = SequenceExecutionArgs::default();
    let breakpoint = BreakpointArgs::default();
    match signal {
        "checkTimersIn" => sm.send_signal_checkTimersIn(h),
        "cmd_BREAK" => sm.send_signal_cmd_BREAK(h),
        "cmd_CANCEL" => sm.send_signal_cmd_CANCEL(h),
        "cmd_CLEAR_BREAKPOINT" => sm.send_signal_cmd_CLEAR_BREAKPOINT(h),
        "cmd_CONTINUE" => sm.send_signal_cmd_CONTINUE(h),
        "cmd_RUN" => sm.send_signal_cmd_RUN(h, &args),
        "cmd_RUN_VALIDATED" => sm.send_signal_cmd_RUN_VALIDATED(h, &args),
        "cmd_SET_BREAKPOINT" => sm.send_signal_cmd_SET_BREAKPOINT(h, &breakpoint),
        "cmd_STEP" => sm.send_signal_cmd_STEP(h),
        "cmd_VALIDATE" => sm.send_signal_cmd_VALIDATE(h, &args),
        "entered" => sm.send_signal_entered(h),
        "result_checkShouldWake_keepSleeping" => {
            sm.send_signal_result_checkShouldWake_keepSleeping(h)
        }
        "result_checkShouldWake_wakeup" => sm.send_signal_result_checkShouldWake_wakeup(h),
        "result_checkStatementTimeout_noTimeout" => {
            sm.send_signal_result_checkStatementTimeout_noTimeout(h)
        }
        "result_checkStatementTimeout_statementTimeout" => {
            sm.send_signal_result_checkStatementTimeout_statementTimeout(h)
        }
        "result_dispatchStatement_failure" => sm.send_signal_result_dispatchStatement_failure(h),
        "result_dispatchStatement_noMoreStatements" => {
            sm.send_signal_result_dispatchStatement_noMoreStatements(h)
        }
        "result_dispatchStatement_success" => sm.send_signal_result_dispatchStatement_success(h),
        "result_failure" => sm.send_signal_result_failure(h),
        "result_success" => sm.send_signal_result_success(h),
        "result_timeOpFailed" => sm.send_signal_result_timeOpFailed(h),
        "stmtResponse_beginSleep" => sm.send_signal_stmtResponse_beginSleep(h),
        "stmtResponse_failure" => sm.send_signal_stmtResponse_failure(h),
        "stmtResponse_keepWaiting" => sm.send_signal_stmtResponse_keepWaiting(h),
        "stmtResponse_success" => sm.send_signal_stmtResponse_success(h),
        "stmtResponse_unexpected" => sm.send_signal_stmtResponse_unexpected(h),
        other => panic!("unknown signal {other}"),
    }
}

#[test]
fn fpy_sequencer_matches_fpp_to_cpp() {
    check(
        include_str!("../../fprime-comp/tests/sm_traces/FpySequencer.txt"),
        |r| {
            let sm = StateMachine::new();
            sm.init(0, &R(r));
            sm
        },
        send,
        |sm| format!("{:?}", sm.get_state()),
    );
}

/// The `State` discriminants are the `fpp-to-cpp` ones (sorted by name,
/// not in definition order), and so are the signals.
#[test]
fn fpy_sequencer_state_enum_matches_fpp_to_cpp() {
    assert_eq!(State::AWAITING_CMD_RUN_VALIDATED as u8, 1);
    assert_eq!(State::IDLE as u8, 2);
    assert_eq!(State::RUNNING_AWAITING_STATEMENT_RESPONSE as u8, 3);
    assert_eq!(State::RUNNING_DISPATCH_STATEMENT as u8, 4);
    assert_eq!(State::RUNNING_PAUSED as u8, 5);
    assert_eq!(State::RUNNING_SLEEPING as u8, 6);
    assert_eq!(State::VALIDATING as u8, 7);
    assert_eq!(State::NUM_CONSTANTS, 8);
    assert_eq!(Signal::checkTimersIn.as_repr(), 1);
    assert_eq!(Signal::stmtResponse_unexpected.as_repr(), 26);
}
