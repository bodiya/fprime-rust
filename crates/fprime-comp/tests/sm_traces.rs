//! Differential test of `fpp_state_machine!` against the reference
//! compiler: every state machine here is checked, for **every** leaf state,
//! signal and guard outcome, against the call trace of the C++ that
//! `fpp-to-cpp` 3.3.0 (the version upstream F Prime 4dee010 pins) generates
//! for the same FPP model.
//!
//! The golden files in `tests/sm_traces/` were produced by
//! `tools/sm-traces.py`, which reads a generated `<M>StateMachineAc.cpp`
//! and enumerates `initBase` and each `sendSignal_<s>` arm, following the
//! `enter_*` functions and both branches of every guard:
//!
//! ```text
//! init [outcomes] -> trace ; FINAL
//! LEAF signal [outcomes] -> trace ; FINAL
//! ```
//!
//! `outcomes` are the guard results in call order, the trace lists the
//! actions (`a:`) and guards (`g:`) called, and `FINAL` is the state after
//! the step (`-`: unchanged). The test drives the Rust machine to `LEAF`
//! along a path found in the same golden data, scripts the guards, sends
//! the signal and compares.
//!
//! Machines: the upstream `Svc.ComAggregator` machine, the hierarchical
//! `FppTest/state_machine` ones, and `Probe`
//! (an inner guarded transition overriding an outer one, a transition to
//! an ancestor, an internal transition on a typed signal with an untyped
//! guard, nested initial transitions; below). The upstream
//! `Svc.FpySequencer` machine is checked against
//! `sm_traces/FpySequencer.txt` by
//! `crates/fprime-svc/tests/fpy_sequencer_sm_traces.rs`, on the definition
//! the component uses, with the checker in `sm_trace_replay/`.
//!
//! ```text
//! state machine Probe {
//!   action a  action b  action exitS1  action exitS2  action enterS1
//!   action enterS2  guard g  signal s  signal up  signal v: U32
//!   initial enter S1
//!   state S1 {
//!     entry do { enterS1 }  exit do { exitS1 }  initial enter S2
//!     on s do { a } enter S3
//!     state S2 {
//!       entry do { enterS2 }  exit do { exitS2 }
//!       on s if g do { b } enter S3
//!       on up enter S1
//!       on v if g do { a }
//!     }
//!   }
//!   state S3 { initial enter S4  state S4 { initial enter S5  state S5 } }
//! }
//! ```
#![allow(non_snake_case)]

mod sm_harness;

mod sm_trace_replay;

use fprime_comp::config::FwEnumStoreType;
use sm_harness::TestStruct;
use sm_trace_replay::{Recorder, check};

/// One machine under test: `fpp_state_machine!` plus a recording
/// `Handlers` impl and signal dispatch by name.
macro_rules! traced_machine {
    (
        mod $m:ident;
        state machine $name:ident : $repr:ty {
            actions { $( $action:ident => $afn:ident $( ( $amode:ident $aty:ty ) )? ),* $(,)? }
            guards { $( $guard:ident => $gfn:ident $( ( $gmode:ident $gty:ty ) )? ),* $(,)? }
            signals {
                $(
                    $signal:ident = $sdisc:literal => $sfn:ident $( ( $smode:ident $sty:ty ) )?
                    { $($strans:tt)* }
                )*
            }
            $($rest:tt)*
        }
    ) => {
        mod $m {
            #[allow(unused_imports)]
            use super::*;

            fprime_comp::fpp_state_machine! {
                state machine $name : $repr {
                    actions { $( $action => $afn $( ( $amode $aty ) )? ),* }
                    guards { $( $guard => $gfn $( ( $gmode $gty ) )? ),* }
                    signals {
                        $( $signal = $sdisc => $sfn $( ( $smode $sty ) )? { $($strans)* } )*
                    }
                    $($rest)*
                }
            }

            impl Handlers for super::Recorder {
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
            pub fn send(sm: &StateMachine, r: &super::Recorder, signal: &str) {
                match signal {
                    $( stringify!($signal) => traced_machine!(@send sm, r, $sfn [$($smode $sty)?]), )*
                    other => panic!("unknown signal {other}"),
                }
            }

            pub fn state(sm: &StateMachine) -> String {
                format!("{:?}", sm.get_state())
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
                include_str!(concat!("sm_traces/", $file)),
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

// ===========================================================================
// Svc.FpySequencer.SequencerStateMachine: checked in fprime-svc
// (`tests/fpy_sequencer_sm_traces.rs`) against the machine the component
// itself uses, with this file's checker and golden file `FpySequencer.txt`.
// ===========================================================================

// ===========================================================================
// Svc.AggregationMachine (ComAggregator)
// ===========================================================================

/// Stand-in for `Svc.ComDataContextPair` (it carries an `Fw.Buffer`; see
/// `docs/api-notes.md` on buffer-carrying signals).
type ComDataContextPair = TestStruct;

traced_machine! {
    mod ComAggregator;
    state machine AggregationMachine: u8 {
        actions {
            doClear => action_doClear,
            doFill => action_doFill (ref ComDataContextPair),
            doSend => action_doSend,
            doHold => action_doHold (ref ComDataContextPair),
            doSplitHold => action_doSplitHold (ref ComDataContextPair),
            doNoteFailure => action_doNoteFailure,
            assertNoStatus => action_assertNoStatus,
        }
        guards {
            isFull => guard_isFull (ref ComDataContextPair),
            willFill => guard_willFill (ref ComDataContextPair),
            isNotEmpty => guard_isNotEmpty,
            isGood => guard_isGood (val fprime_fw::Success),
            isSpanFull => guard_isSpanFull,
        }
        signals {
            fill = 1 => send_signal_fill (ref ComDataContextPair) {
                WAIT_STATUS => do { doHold };
                FILL => enter IS_FULL_THEN_SEND;
            }
            status = 2 => send_signal_status (val fprime_fw::Success) {
                WAIT_STATUS => enter IS_GOOD_STATUS;
                FILL => do { assertNoStatus };
            }
            timeout = 3 => send_signal_timeout {
                FILL => if isNotEmpty do { doSend } enter WAIT_STATUS;
            }
        }
        initial enter WAIT_STATUS;
        states {
            state FILL = 1 {}
            state WAIT_STATUS = 2 {}
        }
        choices {
            choice IS_FULL_THEN_SEND (ref ComDataContextPair) {
                if isFull do { doSplitHold, doSend } enter WAIT_STATUS
                else enter WILL_FILL_THEN_SEND
            }
            choice WILL_FILL_THEN_SEND (ref ComDataContextPair) {
                if willFill do { doFill, doSend } enter WAIT_STATUS
                else do { doFill } enter FILL
            }
            choice IS_GOOD_STATUS (val fprime_fw::Success) {
                if isGood do { doClear } enter SPAN_CHECK
                else do { doNoteFailure } enter WAIT_STATUS
            }
            choice SPAN_CHECK (val fprime_fw::Success) {
                if isSpanFull do { doSend } enter WAIT_STATUS else enter FILL
            }
        }
    }
}

check_machine!(
    com_aggregator_matches_fpp_to_cpp,
    ComAggregator,
    "ComAggregator.txt"
);

// ===========================================================================
// Probe
// ===========================================================================

traced_machine! {
    mod Probe;
    state machine Probe: u8 {
        actions {
            a => action_a,
            b => action_b,
            exitS1 => action_exitS1,
            exitS2 => action_exitS2,
            enterS1 => action_enterS1,
            enterS2 => action_enterS2,
        }
        guards { g => guard_g }
        signals {
            s = 1 => send_signal_s {
                S1 => do { a } enter S3;
                S1_S2 => if g do { b } enter S3;
            }
            up = 2 => send_signal_up {
                S1_S2 => enter S1;
            }
            v = 3 => send_signal_v (val u32) {
                S1_S2 => if g do { a };
            }
        }
        initial enter S1;
        states {
            state S1 {
                initial enter S1_S2;
                entry do { enterS1 }
                exit do { exitS1 }
            }
            state S1_S2 = 1 in S1 {
                entry do { enterS2 }
                exit do { exitS2 }
            }
            state S3 {
                initial enter S3_S4;
            }
            state S3_S4 in S3 {
                initial enter S3_S4_S5;
            }
            state S3_S4_S5 = 2 in S3_S4 {}
        }
        choices {}
    }
}

check_machine!(probe_matches_fpp_to_cpp, Probe, "Probe.txt");

// ===========================================================================
// FppTest/state_machine/internal
// ===========================================================================

traced_machine! {
    mod StateToChoice;
    state machine StateToChoice: u8 {
        actions {
            exitS1 => action_exitS1, exitS2 => action_exitS2, exitS3 => action_exitS3,
            a => action_a, enterS1 => action_enterS1, enterS2 => action_enterS2,
            enterS3 => action_enterS3, enterS4 => action_enterS4,
        }
        guards { g => guard_g }
        signals {
            S1_to_C = 1 => send_signal_S1_to_C { S1 => do { a } enter S4_C; }
            S1_to_S4 = 2 => send_signal_S1_to_S4 { S1 => do { a } enter S4; }
            S2_to_S3 = 3 => send_signal_S2_to_S3 { S1_S2 => enter S1_S3; }
        }
        initial enter S1;
        states {
            state S1 { initial enter S1_S2; entry do { enterS1 } exit do { exitS1 } }
            state S1_S2 = 1 in S1 { entry do { enterS2 } exit do { exitS2 } }
            state S1_S3 = 2 in S1 { entry do { enterS3 } exit do { exitS3 } }
            state S4 { initial enter S4_C; entry do { enterS4 } }
            state S4_S5 = 3 in S4 {}
            state S4_S6 = 4 in S4 {}
        }
        choices {
            choice S4_C in S4 { if g enter S4_S5 else enter S4_S6 }
        }
    }
}

check_machine!(
    state_to_choice_matches_fpp_to_cpp,
    StateToChoice,
    "State_StateToChoice.txt"
);

traced_machine! {
    mod StateToState;
    state machine StateToState: u8 {
        actions {
            exitS1 => action_exitS1, exitS2 => action_exitS2, exitS3 => action_exitS3,
            a => action_a, enterS1 => action_enterS1, enterS2 => action_enterS2,
            enterS3 => action_enterS3, enterS4 => action_enterS4, enterS5 => action_enterS5,
        }
        guards {}
        signals {
            S1_to_S4 = 1 => send_signal_S1_to_S4 { S1 => do { a } enter S4; }
            S1_to_S5 = 2 => send_signal_S1_to_S5 { S1 => do { a } enter S4_S5; }
            S2_to_S3 = 3 => send_signal_S2_to_S3 { S1_S2 => enter S1_S3; }
        }
        initial enter S1;
        states {
            state S1 { initial enter S1_S2; entry do { enterS1 } exit do { exitS1 } }
            state S1_S2 = 1 in S1 { entry do { enterS2 } exit do { exitS2 } }
            state S1_S3 = 2 in S1 { entry do { enterS3 } exit do { exitS3 } }
            state S4 { initial enter S4_S5; entry do { enterS4 } }
            state S4_S5 = 3 in S4 { entry do { enterS5 } }
        }
        choices {}
    }
}

check_machine!(
    state_to_state_matches_fpp_to_cpp,
    StateToState,
    "State_StateToState.txt"
);

traced_machine! {
    mod StateToSelf;
    state machine StateToSelf: u8 {
        actions {
            exitS1 => action_exitS1, exitS2 => action_exitS2, exitS3 => action_exitS3,
            a => action_a, enterS1 => action_enterS1, enterS2 => action_enterS2,
            enterS3 => action_enterS3,
        }
        guards {}
        signals {
            S1_to_S1 = 1 => send_signal_S1_to_S1 { S1 => do { a } enter S1; }
            S2_to_S3 = 2 => send_signal_S2_to_S3 { S1_S2 => enter S1_S3; }
        }
        initial enter S1;
        states {
            state S1 { initial enter S1_S2; entry do { enterS1 } exit do { exitS1 } }
            state S1_S2 = 1 in S1 { entry do { enterS2 } exit do { exitS2 } }
            state S1_S3 = 2 in S1 { entry do { enterS3 } exit do { exitS3 } }
        }
        choices {}
    }
}

check_machine!(
    state_to_self_matches_fpp_to_cpp,
    StateToSelf,
    "State_StateToSelf.txt"
);

traced_machine! {
    mod StateToChild;
    state machine StateToChild: u8 {
        actions {
            exitS2 => action_exitS2, exitS3 => action_exitS3, a => action_a,
            enterS2 => action_enterS2, enterS3 => action_enterS3,
        }
        guards {}
        signals {
            S1_to_S2 = 1 => send_signal_S1_to_S2 { S1 => do { a } enter S1_S2; }
            S2_to_S3 = 2 => send_signal_S2_to_S3 { S1_S2 => enter S1_S3; }
        }
        initial enter S1;
        states {
            state S1 { initial enter S1_S2; }
            state S1_S2 = 1 in S1 { entry do { enterS2 } exit do { exitS2 } }
            state S1_S3 = 2 in S1 { entry do { enterS3 } exit do { exitS3 } }
        }
        choices {}
    }
}

check_machine!(
    state_to_child_matches_fpp_to_cpp,
    StateToChild,
    "State_StateToChild.txt"
);

traced_machine! {
    mod Polymorphism;
    state machine Polymorphism: u8 {
        actions {}
        guards {}
        signals {
            S2_to_S3 = 1 => send_signal_S2_to_S3 { S1_S2 => enter S1_S3; }
            poly = 2 => send_signal_poly { S1 => enter S4; S1_S3 => enter S5; }
        }
        initial enter S1;
        states {
            state S1 { initial enter S1_S2; }
            state S1_S2 = 1 in S1 {}
            state S1_S3 = 2 in S1 {}
            state S4 = 3 {}
            state S5 = 4 {}
        }
        choices {}
    }
}

check_machine!(
    polymorphism_matches_fpp_to_cpp,
    Polymorphism,
    "State_Polymorphism.txt"
);

traced_machine! {
    mod Internal;
    state machine Internal: u8 {
        actions { a => action_a }
        guards {}
        signals {
            S1_internal = 1 => send_signal_S1_internal { S1 => do { a }; }
            S2_to_S3 = 2 => send_signal_S2_to_S3 { S1_S2 => enter S1_S3; }
        }
        initial enter S1;
        states {
            state S1 { initial enter S1_S2; }
            state S1_S2 = 1 in S1 {}
            state S1_S3 = 2 in S1 {}
        }
        choices {}
    }
}

check_machine!(internal_matches_fpp_to_cpp, Internal, "State_Internal.txt");

traced_machine! {
    mod BasicSelf;
    state machine BasicSelf: u8 {
        actions { a => action_a }
        guards {}
        signals {
            s = 1 => send_signal_s { S => do { a, a, a } enter S; }
        }
        initial enter S;
        states {
            state S = 1 { entry do { a } exit do { a, a } }
        }
        choices {}
    }
}

check_machine!(
    basic_self_matches_fpp_to_cpp,
    BasicSelf,
    "State_BasicSelf.txt"
);

traced_machine! {
    mod BasicInternal;
    state machine BasicInternal: u8 {
        actions { a => action_a }
        guards {}
        signals {
            s = 1 => send_signal_s { S => do { a }; }
        }
        initial enter S;
        states {
            state S = 1 { entry do { a } }
        }
        choices {}
    }
}

check_machine!(
    basic_internal_matches_fpp_to_cpp,
    BasicInternal,
    "State_BasicInternal.txt"
);

traced_machine! {
    mod BasicGuard;
    state machine BasicGuard: u8 {
        actions { a => action_a }
        guards { g => guard_g }
        signals {
            s = 1 => send_signal_s { S => if g do { a, a } enter T; }
        }
        initial enter S;
        states {
            state S = 1 { exit do { a } }
            state T = 2 { entry do { a, a, a } }
        }
        choices {}
    }
}

check_machine!(
    basic_guard_matches_fpp_to_cpp,
    BasicGuard,
    "State_BasicGuard.txt"
);

traced_machine! {
    mod ChoiceToChoice;
    state machine ChoiceToChoice: u8 {
        actions { exitS1 => action_exitS1, a => action_a, enterS2 => action_enterS2 }
        guards { g1 => guard_g1, g2 => guard_g2 }
        signals {
            s = 1 => send_signal_s { S1 => enter S1_C1; }
        }
        initial enter S1;
        states {
            state S1 = 1 { exit do { exitS1 } }
            state S2 { initial enter S2_S3; entry do { enterS2 } }
            state S2_S3 = 2 in S2 {}
            state S2_S4 = 3 in S2 {}
        }
        choices {
            choice S1_C1 in S1 { if g1 do { a } enter S2 else do { a } enter S2_C2 }
            choice S2_C2 in S2 { if g2 enter S2_S3 else enter S2_S4 }
        }
    }
}

check_machine!(
    choice_to_choice_matches_fpp_to_cpp,
    ChoiceToChoice,
    "Choice_ChoiceToChoice.txt"
);

traced_machine! {
    mod ChoiceToState;
    state machine ChoiceToState: u8 {
        actions {
            exitS1 => action_exitS1, a => action_a, enterS2 => action_enterS2,
            enterS3 => action_enterS3,
        }
        guards { g => guard_g }
        signals {
            s = 1 => send_signal_s { S1 => enter S1_C; }
        }
        initial enter S1;
        states {
            state S1 = 1 { exit do { exitS1 } }
            state S2 { initial do { a } enter S2_S3; entry do { enterS2 } }
            state S2_S3 = 2 in S2 { entry do { enterS3 } }
        }
        choices {
            choice S1_C in S1 { if g do { a } enter S2 else do { a } enter S2_S3 }
        }
    }
}

check_machine!(
    choice_to_state_matches_fpp_to_cpp,
    ChoiceToState,
    "Choice_ChoiceToState.txt"
);

traced_machine! {
    mod Sequence;
    state machine Sequence: u8 {
        actions { a => action_a, b => action_b }
        guards { g1 => guard_g1, g2 => guard_g2 }
        signals {
            s = 1 => send_signal_s { S1 => enter C1; }
        }
        initial enter S1;
        states {
            state S1 = 1 {}
            state S2 = 2 {}
            state S3 = 3 {}
            state S4 = 4 {}
        }
        choices {
            choice C1 { if g1 enter S2 else enter C2 }
            choice C2 { if g2 do { a } enter S3 else do { b } enter S4 }
        }
    }
}

check_machine!(sequence_matches_fpp_to_cpp, Sequence, "Choice_Sequence.txt");

traced_machine! {
    mod SequenceU32;
    state machine SequenceU32: u8 {
        actions { a => action_a (val u32), b => action_b }
        guards { g1 => guard_g1, g2 => guard_g2 (val u32) }
        signals {
            s = 1 => send_signal_s (val u32) { S1 => enter C1; }
        }
        initial enter S1;
        states {
            state S1 = 1 {}
            state S2 = 2 {}
            state S3 = 3 {}
            state S4 = 4 {}
        }
        choices {
            choice C1 (val u32) { if g1 enter S2 else enter C2 }
            choice C2 (val u32) { if g2 do { a } enter S3 else do { b } enter S4 }
        }
    }
}

check_machine!(
    sequence_u32_matches_fpp_to_cpp,
    SequenceU32,
    "Choice_SequenceU32.txt"
);

traced_machine! {
    mod InputPairU16U32;
    state machine InputPairU16U32: u8 {
        actions { a => action_a (val u32) }
        guards { g => guard_g (val u32) }
        signals {
            s1 = 1 => send_signal_s1 (val u16) { S1 => enter C; }
            s2 = 2 => send_signal_s2 (val u32) { S1 => enter C; }
        }
        initial enter S1;
        states {
            state S1 = 1 {}
            state S2 = 2 {}
            state S3 = 3 {}
        }
        choices {
            choice C (val u32) { if g do { a } enter S2 else do { a } enter S3 }
        }
    }
}

check_machine!(
    input_pair_matches_fpp_to_cpp,
    InputPairU16U32,
    "Choice_InputPairU16U32.txt"
);

traced_machine! {
    mod InitialChoice;
    state machine Choice: u8 {
        actions { a => action_a }
        guards { g => guard_g }
        signals {}
        initial do { a } enter C;
        states {
            state S = 1 { entry do { a } }
            state T = 2 { entry do { a, a } }
        }
        choices {
            choice C { if g do { a } enter S else do { a, a } enter T }
        }
    }
}

check_machine!(
    initial_choice_matches_fpp_to_cpp,
    InitialChoice,
    "Initial_Choice.txt"
);

traced_machine! {
    mod InitialNested;
    state machine Nested: u8 {
        actions { a => action_a }
        guards {}
        signals {}
        initial do { a } enter S;
        states {
            state S { initial enter S_T; entry do { a, a } }
            state S_T = 1 in S { entry do { a, a, a } }
        }
        choices {}
    }
}

check_machine!(
    initial_nested_matches_fpp_to_cpp,
    InitialNested,
    "Initial_Nested.txt"
);
