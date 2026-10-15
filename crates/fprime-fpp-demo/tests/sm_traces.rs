//! The state machines `fpp-to-rust` generates from upstream FPP replay
//! the golden call traces of the C++ that `fpp-to-cpp` 3.3.0 generates
//! for the same models (`fprime-comp/tests/sm_traces/`, produced by
//! `tools/sm-traces.py`): every leaf state, signal and guard outcome of
//! `Svc.FpySequencer`'s and `Svc.ComAggregator`'s machines, the FppTest
//! `state_machine/internal` machines and `Probe` (all under `fpp/sm`).
//!
//! fprime-comp's `tests/sm_traces.rs` checks hand-written
//! `fpp_state_machine!` inputs against the same files; here the inputs
//! are the generator's. Each `traced!` names the generated machine's
//! actions, guards and signals (with their value types), so it only
//! compiles if the generated `Handlers` trait and send functions have
//! exactly those signatures, and checks the signal numbering.
#![allow(non_snake_case)]

#[path = "../../fprime-comp/tests/sm_trace_replay/mod.rs"]
mod sm_trace_replay;

use fprime_config::FwEnumStoreType;
use fprime_fpp_demo::sm_models::Svc::ComDataContextPair;
use fprime_fpp_demo::sm_models::Svc::FpySequencer::{BreakpointArgs, SequenceExecutionArgs};
use sm_trace_replay::{Recorder, check};

/// A generated machine under test: a recording `Handlers` impl, signal
/// dispatch by name, and the signal numbering.
macro_rules! traced {
    (
        mod $m:ident = $($seg:ident)::+;
        actions { $( $action:ident => $afn:ident $( ( $amode:ident $aty:ty ) )? ),* $(,)? }
        guards { $( $guard:ident => $gfn:ident $( ( $gmode:ident $gty:ty ) )? ),* $(,)? }
        signals {
            $( $signal:ident = $sdisc:literal => $sfn:ident $( ( $smode:ident $sty:ty ) )? ),* $(,)?
        }
    ) => {
        mod $m {
            #[allow(unused_imports)]
            use super::*;
            pub use ::$($seg)::+::{Handlers, Signal, StateMachine};

            impl Handlers for Recorder {
                $(
                    fn $afn(
                        &self,
                        _: FwEnumStoreType,
                        _: Signal
                        $(, _: fprime_comp::__fpp_sm_param_ty!($amode $aty))?
                    ) {
                        self.action(stringify!($action));
                    }
                )*
                $(
                    fn $gfn(
                        &self,
                        _: FwEnumStoreType,
                        _: Signal
                        $(, _: fprime_comp::__fpp_sm_param_ty!($gmode $gty))?
                    ) -> bool {
                        self.guard(stringify!($guard))
                    }
                )*
            }

            #[allow(unused_variables)]
            pub fn send(sm: &StateMachine, r: &Recorder, signal: &str) {
                match signal {
                    $( stringify!($signal) => traced!(@send sm, r, $sfn [$($smode $sty)?]), )*
                    other => panic!("unknown signal {other}"),
                }
            }

            pub fn state(sm: &StateMachine) -> String {
                format!("{:?}", sm.get_state())
            }

            #[test]
            fn signals_are_numbered_by_name() {
                $( assert_eq!(Signal::$signal.as_repr(), $sdisc); )*
            }
        }
    };
    (@send $sm:ident, $r:ident, $sfn:ident []) => { $sm.$sfn($r) };
    (@send $sm:ident, $r:ident, $sfn:ident [val $ty:ty]) => {
        $sm.$sfn($r, <$ty as Default>::default())
    };
    (@send $sm:ident, $r:ident, $sfn:ident [ref $ty:ty]) => {
        $sm.$sfn($r, &<$ty as Default>::default())
    };
    (@send $sm:ident, $r:ident, $sfn:ident [str $ty:ty]) => { $sm.$sfn($r, "") };
}

macro_rules! check_machine {
    ($test:ident, $m:ident, $file:literal) => {
        #[test]
        fn $test() {
            check(
                include_str!(concat!("../../fprime-comp/tests/sm_traces/", $file)),
                |r| {
                    let sm = $m::StateMachine::new();
                    sm.init(0, r);
                    sm
                },
                $m::send,
                $m::state,
            );
        }
    };
}

traced! {
    mod FpySequencer = fprime_fpp_demo::sm_models::Svc::FpySequencer::SequencerStateMachine;
    actions { signalEntered => action_signalEntered, setSequenceFilePath => action_setSequenceFilePath (ref SequenceExecutionArgs), setSequenceBlockState => action_setSequenceBlockState (ref SequenceExecutionArgs), setSequenceArguments => action_setSequenceArguments (ref SequenceExecutionArgs), validate => action_validate, report_seqSucceeded => action_report_seqSucceeded, report_seqCancelled => action_report_seqCancelled, report_seqFailed => action_report_seqFailed, report_seqStarted => action_report_seqStarted, setGoalState_RUNNING => action_setGoalState_RUNNING, setGoalState_VALID => action_setGoalState_VALID, setGoalState_IDLE => action_setGoalState_IDLE, sendCmdResponse_OK => action_sendCmdResponse_OK, sendCmdResponse_EXECUTION_ERROR => action_sendCmdResponse_EXECUTION_ERROR, clearSequenceFile => action_clearSequenceFile, clearBreakpoint => action_clearBreakpoint, clearSequenceArguments => action_clearSequenceArguments, checkShouldWake => action_checkShouldWake, dispatchStatement => action_dispatchStatement, resetRuntime => action_resetRuntime, checkStatementTimeout => action_checkStatementTimeout, incrementSequenceCounter => action_incrementSequenceCounter, pushArgsToStack => action_pushArgsToStack, report_seqBroken => action_report_seqBroken, setBreakpoint => action_setBreakpoint (ref BreakpointArgs), setBreakBeforeNextLine => action_setBreakBeforeNextLine, clearBreakBeforeNextLine => action_clearBreakBeforeNextLine }
    guards { goalStateIs_RUNNING => guard_goalStateIs_RUNNING, shouldBreak => guard_shouldBreak, breakOnce => guard_breakOnce }
    signals {
        checkTimersIn = 1 => send_signal_checkTimersIn,
        cmd_BREAK = 2 => send_signal_cmd_BREAK,
        cmd_CANCEL = 3 => send_signal_cmd_CANCEL,
        cmd_CLEAR_BREAKPOINT = 4 => send_signal_cmd_CLEAR_BREAKPOINT,
        cmd_CONTINUE = 5 => send_signal_cmd_CONTINUE,
        cmd_RUN = 6 => send_signal_cmd_RUN (ref SequenceExecutionArgs),
        cmd_RUN_VALIDATED = 7 => send_signal_cmd_RUN_VALIDATED (ref SequenceExecutionArgs),
        cmd_SET_BREAKPOINT = 8 => send_signal_cmd_SET_BREAKPOINT (ref BreakpointArgs),
        cmd_STEP = 9 => send_signal_cmd_STEP,
        cmd_VALIDATE = 10 => send_signal_cmd_VALIDATE (ref SequenceExecutionArgs),
        entered = 11 => send_signal_entered,
        result_checkShouldWake_keepSleeping = 12 => send_signal_result_checkShouldWake_keepSleeping,
        result_checkShouldWake_wakeup = 13 => send_signal_result_checkShouldWake_wakeup,
        result_checkStatementTimeout_noTimeout = 14 => send_signal_result_checkStatementTimeout_noTimeout,
        result_checkStatementTimeout_statementTimeout = 15 => send_signal_result_checkStatementTimeout_statementTimeout,
        result_dispatchStatement_failure = 16 => send_signal_result_dispatchStatement_failure,
        result_dispatchStatement_noMoreStatements = 17 => send_signal_result_dispatchStatement_noMoreStatements,
        result_dispatchStatement_success = 18 => send_signal_result_dispatchStatement_success,
        result_failure = 19 => send_signal_result_failure,
        result_success = 20 => send_signal_result_success,
        result_timeOpFailed = 21 => send_signal_result_timeOpFailed,
        stmtResponse_beginSleep = 22 => send_signal_stmtResponse_beginSleep,
        stmtResponse_failure = 23 => send_signal_stmtResponse_failure,
        stmtResponse_keepWaiting = 24 => send_signal_stmtResponse_keepWaiting,
        stmtResponse_success = 25 => send_signal_stmtResponse_success,
        stmtResponse_unexpected = 26 => send_signal_stmtResponse_unexpected,
    }
}
check_machine!(
    fpy_sequencer_generated_matches_fpp_to_cpp,
    FpySequencer,
    "FpySequencer.txt"
);

traced! {
    mod ComAggregator = fprime_fpp_demo::sm_models::Svc::AggregationMachine;
    actions { doClear => action_doClear, doFill => action_doFill (ref ComDataContextPair), doSend => action_doSend, doHold => action_doHold (ref ComDataContextPair), doSplitHold => action_doSplitHold (ref ComDataContextPair), doNoteFailure => action_doNoteFailure, assertNoStatus => action_assertNoStatus }
    guards { isFull => guard_isFull (ref ComDataContextPair), willFill => guard_willFill (ref ComDataContextPair), isNotEmpty => guard_isNotEmpty, isGood => guard_isGood (val ::fprime_fw::Success), isSpanFull => guard_isSpanFull }
    signals {
        fill = 1 => send_signal_fill (ref ComDataContextPair),
        status = 2 => send_signal_status (val ::fprime_fw::Success),
        timeout = 3 => send_signal_timeout,
    }
}
check_machine!(
    com_aggregator_generated_matches_fpp_to_cpp,
    ComAggregator,
    "ComAggregator.txt"
);

traced! {
    mod Probe = fprime_fpp_demo::sm_models::Probe;
    actions { a => action_a, b => action_b, exitS1 => action_exitS1, exitS2 => action_exitS2, enterS1 => action_enterS1, enterS2 => action_enterS2 }
    guards { g => guard_g }
    signals {
        s = 1 => send_signal_s,
        up = 2 => send_signal_up,
        v = 3 => send_signal_v (val u32),
    }
}
check_machine!(probe_generated_matches_fpp_to_cpp, Probe, "Probe.txt");

traced! {
    mod State_BasicGuard = fprime_fpp_demo::sm_models::FppTest::SmState::BasicGuard;
    actions { a => action_a }
    guards { g => guard_g }
    signals {
        s = 1 => send_signal_s,
    }
}
check_machine!(
    state_basic_guard_generated_matches_fpp_to_cpp,
    State_BasicGuard,
    "State_BasicGuard.txt"
);

traced! {
    mod State_BasicInternal = fprime_fpp_demo::sm_models::FppTest::SmState::BasicInternal;
    actions { a => action_a }
    guards {  }
    signals {
        s = 1 => send_signal_s,
    }
}
check_machine!(
    state_basic_internal_generated_matches_fpp_to_cpp,
    State_BasicInternal,
    "State_BasicInternal.txt"
);

traced! {
    mod State_BasicSelf = fprime_fpp_demo::sm_models::FppTest::SmState::BasicSelf;
    actions { a => action_a }
    guards {  }
    signals {
        s = 1 => send_signal_s,
    }
}
check_machine!(
    state_basic_self_generated_matches_fpp_to_cpp,
    State_BasicSelf,
    "State_BasicSelf.txt"
);

traced! {
    mod State_Internal = fprime_fpp_demo::sm_models::FppTest::SmState::Internal;
    actions { a => action_a }
    guards {  }
    signals {
        S1_internal = 1 => send_signal_S1_internal,
        S2_to_S3 = 2 => send_signal_S2_to_S3,
    }
}
check_machine!(
    state_internal_generated_matches_fpp_to_cpp,
    State_Internal,
    "State_Internal.txt"
);

traced! {
    mod State_Polymorphism = fprime_fpp_demo::sm_models::FppTest::SmState::Polymorphism;
    actions {  }
    guards {  }
    signals {
        S2_to_S3 = 1 => send_signal_S2_to_S3,
        poly = 2 => send_signal_poly,
    }
}
check_machine!(
    state_polymorphism_generated_matches_fpp_to_cpp,
    State_Polymorphism,
    "State_Polymorphism.txt"
);

traced! {
    mod State_StateToChild = fprime_fpp_demo::sm_models::FppTest::SmState::StateToChild;
    actions { exitS2 => action_exitS2, exitS3 => action_exitS3, a => action_a, enterS2 => action_enterS2, enterS3 => action_enterS3 }
    guards {  }
    signals {
        S1_to_S2 = 1 => send_signal_S1_to_S2,
        S2_to_S3 = 2 => send_signal_S2_to_S3,
    }
}
check_machine!(
    state_state_to_child_generated_matches_fpp_to_cpp,
    State_StateToChild,
    "State_StateToChild.txt"
);

traced! {
    mod State_StateToChoice = fprime_fpp_demo::sm_models::FppTest::SmState::StateToChoice;
    actions { exitS1 => action_exitS1, exitS2 => action_exitS2, exitS3 => action_exitS3, a => action_a, enterS1 => action_enterS1, enterS2 => action_enterS2, enterS3 => action_enterS3, enterS4 => action_enterS4 }
    guards { g => guard_g }
    signals {
        S1_to_C = 1 => send_signal_S1_to_C,
        S1_to_S4 = 2 => send_signal_S1_to_S4,
        S2_to_S3 = 3 => send_signal_S2_to_S3,
    }
}
check_machine!(
    state_state_to_choice_generated_matches_fpp_to_cpp,
    State_StateToChoice,
    "State_StateToChoice.txt"
);

traced! {
    mod State_StateToSelf = fprime_fpp_demo::sm_models::FppTest::SmState::StateToSelf;
    actions { exitS1 => action_exitS1, exitS2 => action_exitS2, exitS3 => action_exitS3, a => action_a, enterS1 => action_enterS1, enterS2 => action_enterS2, enterS3 => action_enterS3 }
    guards {  }
    signals {
        S1_to_S1 = 1 => send_signal_S1_to_S1,
        S2_to_S3 = 2 => send_signal_S2_to_S3,
    }
}
check_machine!(
    state_state_to_self_generated_matches_fpp_to_cpp,
    State_StateToSelf,
    "State_StateToSelf.txt"
);

traced! {
    mod State_StateToState = fprime_fpp_demo::sm_models::FppTest::SmState::StateToState;
    actions { exitS1 => action_exitS1, exitS2 => action_exitS2, exitS3 => action_exitS3, a => action_a, enterS1 => action_enterS1, enterS2 => action_enterS2, enterS3 => action_enterS3, enterS4 => action_enterS4, enterS5 => action_enterS5 }
    guards {  }
    signals {
        S1_to_S4 = 1 => send_signal_S1_to_S4,
        S1_to_S5 = 2 => send_signal_S1_to_S5,
        S2_to_S3 = 3 => send_signal_S2_to_S3,
    }
}
check_machine!(
    state_state_to_state_generated_matches_fpp_to_cpp,
    State_StateToState,
    "State_StateToState.txt"
);

traced! {
    mod Choice_ChoiceToChoice = fprime_fpp_demo::sm_models::FppTest::SmChoice::ChoiceToChoice;
    actions { exitS1 => action_exitS1, a => action_a, enterS2 => action_enterS2 }
    guards { g1 => guard_g1, g2 => guard_g2 }
    signals {
        s = 1 => send_signal_s,
    }
}
check_machine!(
    choice_choice_to_choice_generated_matches_fpp_to_cpp,
    Choice_ChoiceToChoice,
    "Choice_ChoiceToChoice.txt"
);

traced! {
    mod Choice_ChoiceToState = fprime_fpp_demo::sm_models::FppTest::SmChoice::ChoiceToState;
    actions { exitS1 => action_exitS1, a => action_a, enterS2 => action_enterS2, enterS3 => action_enterS3 }
    guards { g => guard_g }
    signals {
        s = 1 => send_signal_s,
    }
}
check_machine!(
    choice_choice_to_state_generated_matches_fpp_to_cpp,
    Choice_ChoiceToState,
    "Choice_ChoiceToState.txt"
);

traced! {
    mod Choice_InputPairU16U32 = fprime_fpp_demo::sm_models::FppTest::SmChoice::InputPairU16U32;
    actions { a => action_a (val u32) }
    guards { g => guard_g (val u32) }
    signals {
        s1 = 1 => send_signal_s1 (val u16),
        s2 = 2 => send_signal_s2 (val u32),
    }
}
check_machine!(
    choice_input_pair_u16_u32_generated_matches_fpp_to_cpp,
    Choice_InputPairU16U32,
    "Choice_InputPairU16U32.txt"
);

traced! {
    mod Choice_Sequence = fprime_fpp_demo::sm_models::FppTest::SmChoice::Sequence;
    actions { a => action_a, b => action_b }
    guards { g1 => guard_g1, g2 => guard_g2 }
    signals {
        s = 1 => send_signal_s,
    }
}
check_machine!(
    choice_sequence_generated_matches_fpp_to_cpp,
    Choice_Sequence,
    "Choice_Sequence.txt"
);

traced! {
    mod Choice_SequenceU32 = fprime_fpp_demo::sm_models::FppTest::SmChoice::SequenceU32;
    actions { a => action_a (val u32), b => action_b }
    guards { g1 => guard_g1, g2 => guard_g2 (val u32) }
    signals {
        s = 1 => send_signal_s (val u32),
    }
}
check_machine!(
    choice_sequence_u32_generated_matches_fpp_to_cpp,
    Choice_SequenceU32,
    "Choice_SequenceU32.txt"
);

traced! {
    mod Initial_Choice = fprime_fpp_demo::sm_models::FppTest::SmInitial::Choice;
    actions { a => action_a }
    guards { g => guard_g }
    signals {
    }
}
check_machine!(
    initial_choice_generated_matches_fpp_to_cpp,
    Initial_Choice,
    "Initial_Choice.txt"
);

traced! {
    mod Initial_Nested = fprime_fpp_demo::sm_models::FppTest::SmInitial::Nested;
    actions { a => action_a }
    guards {  }
    signals {
    }
}
check_machine!(
    initial_nested_generated_matches_fpp_to_cpp,
    Initial_Nested,
    "Initial_Nested.txt"
);

/// The generated `Svc.AggregationMachine` is the one `fprime-svc`'s
/// `ComAggregator` writes by hand (both replay `ComAggregator.txt`): the
/// same `State` and `Signal` numbering, and the same `fill` value
/// (`Svc.ComDataContextPair`, its `Fw.Buffer` an `EscrowedBuffer`).
#[test]
fn com_aggregator_machine_is_the_hand_written_one() {
    use fprime_fpp_demo::sm_models::Svc::AggregationMachine as generated;
    use fprime_fw::{Endianness, FppSized, LinearBuffer, SerBuf, Serialize};
    use fprime_svc::com_aggregator::aggregation_machine as hand;
    for (g, h) in [
        (generated::State::FILL, hand::State::FILL),
        (generated::State::WAIT_STATUS, hand::State::WAIT_STATUS),
    ] {
        assert_eq!(g.as_repr(), h.as_repr());
    }
    assert_eq!(generated::State::NUM_CONSTANTS, hand::State::NUM_CONSTANTS);
    for (g, h) in [
        (generated::Signal::fill, hand::Signal::fill),
        (generated::Signal::status, hand::Signal::status),
        (generated::Signal::timeout, hand::Signal::timeout),
    ] {
        assert_eq!(g.as_repr(), h.as_repr());
    }
    assert_eq!(
        generated::StateMachine::SIGNAL_MSG_SIZE,
        hand::StateMachine::SIGNAL_MSG_SIZE
    );
    // The fill value: 36-byte Fw::Buffer layout + 15-byte frame context.
    assert_eq!(ComDataContextPair::SERIALIZED_SIZE, 51);
    assert_eq!(
        <ComDataContextPair as FppSized>::SERIALIZED_SIZE,
        <fprime_svc::com_aggregator::ComDataContextPair as FppSized>::SERIALIZED_SIZE
    );
    let mut a = LinearBuffer::<64>::new();
    let mut b = LinearBuffer::<64>::new();
    assert!(
        ComDataContextPair::default()
            .serialize_to(&mut a, Endianness::Big)
            .is_ok()
    );
    assert!(
        fprime_svc::com_aggregator::ComDataContextPair::default()
            .serialize_to(&mut b, Endianness::Big)
            .is_ok()
    );
    assert_eq!(a.as_slice(), b.as_slice());
}

/// The generated `State` enum is numbered as `fpp-to-cpp` numbers it
/// (leaf states sorted by name, not in definition order).
#[test]
fn fpy_sequencer_state_enum_matches_fpp_to_cpp() {
    use FpySequencer::{Signal, StateMachine};
    use fprime_fpp_demo::sm_models::Svc::FpySequencer::SequencerStateMachine::State;
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
    assert_eq!(
        StateMachine::new().get_state(),
        State::__FPRIME_UNINITIALIZED
    );
}
