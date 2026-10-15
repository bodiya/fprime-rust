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
//! Machines: the upstream `Svc.FpySequencer` and `Svc.ComAggregator`
//! machines, the hierarchical `FppTest/state_machine` ones, and `Probe`
//! (an inner guarded transition overriding an outer one, a transition to
//! an ancestor, an internal transition on a typed signal with an untyped
//! guard, nested initial transitions):
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

use fprime_comp::config::FwEnumStoreType;
use sm_harness::TestStruct;
use std::collections::{HashMap, VecDeque};
use std::sync::Mutex;

/// Records the trace and plays back scripted guard outcomes.
#[derive(Default)]
struct Recorder {
    trace: Mutex<Vec<String>>,
    script: Mutex<VecDeque<bool>>,
}

impl Recorder {
    fn action(&self, name: &str) {
        self.trace.lock().unwrap().push(format!("a:{name}"));
    }

    fn guard(&self, name: &str) -> bool {
        self.trace.lock().unwrap().push(format!("g:{name}"));
        self.script
            .lock()
            .unwrap()
            .pop_front()
            .unwrap_or_else(|| panic!("unscripted guard {name}"))
    }

    fn arm(&self, outcomes: &str) {
        self.trace.lock().unwrap().clear();
        *self.script.lock().unwrap() = outcomes.chars().map(|c| c == '1').collect();
    }

    fn finish(&self) -> String {
        assert!(
            self.script.lock().unwrap().is_empty(),
            "scripted guards left over"
        );
        self.trace.lock().unwrap().join(" ")
    }
}

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

/// One golden line.
struct Step {
    from: Option<(String, String)>,
    outcomes: String,
    trace: String,
    to: String,
}

fn parse(golden: &str) -> Vec<Step> {
    golden
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|line| {
            let (head, rest) = line.split_once(" [").expect("head");
            let (outcomes, rest) = rest.split_once("] -> ").expect("outcomes");
            let (trace, to) = rest.rsplit_once(" ; ").expect("final");
            let from = if head == "init" {
                None
            } else {
                let (leaf, signal) = head.split_once(' ').expect("leaf signal");
                Some((leaf.to_string(), signal.to_string()))
            };
            Step {
                from,
                outcomes: outcomes.to_string(),
                trace: trace.trim().to_string(),
                to: to.to_string(),
            }
        })
        .collect()
}

/// Check one machine against its golden traces. `make` builds a fresh
/// machine and runs `init` with the recorder.
fn check<M>(
    golden: &str,
    make: impl Fn(&Recorder) -> M,
    send: impl Fn(&M, &Recorder, &str),
    state: impl Fn(&M) -> String,
) {
    let steps = parse(golden);
    // Shortest path (in golden steps) from init to every reachable leaf.
    let mut paths: HashMap<String, Vec<usize>> = HashMap::new();
    let mut frontier: VecDeque<String> = VecDeque::new();
    for (i, s) in steps.iter().enumerate() {
        if s.from.is_none() && !paths.contains_key(&s.to) {
            paths.insert(s.to.clone(), vec![i]);
            frontier.push_back(s.to.clone());
        }
    }
    while let Some(leaf) = frontier.pop_front() {
        for (i, s) in steps.iter().enumerate() {
            if let Some((from, _)) = &s.from {
                if *from == leaf && s.to != "-" && !paths.contains_key(&s.to) {
                    let mut p = paths[&leaf].clone();
                    p.push(i);
                    paths.insert(s.to.clone(), p);
                    frontier.push_back(s.to.clone());
                }
            }
        }
    }

    let mut checked = 0;
    for (i, step) in steps.iter().enumerate() {
        let r = Recorder::default();
        let (path, leaf) = match &step.from {
            None => (vec![], None),
            Some((leaf, _)) => match paths.get(leaf) {
                Some(p) => (p.clone(), Some(leaf.clone())),
                None => panic!("leaf {leaf} unreachable in the golden data"),
            },
        };
        // Replay the path (its first step is an init line).
        let mut sm = None;
        for &j in &path {
            let s = &steps[j];
            r.arm(&s.outcomes);
            match &s.from {
                None => sm = Some(make(&r)),
                Some((_, signal)) => send(sm.as_ref().unwrap(), &r, signal),
            }
            r.finish();
        }
        r.arm(&step.outcomes);
        let sm = match &step.from {
            None => make(&r),
            Some((_, signal)) => {
                let sm = sm.unwrap();
                assert_eq!(Some(state(&sm)), leaf);
                send(&sm, &r, signal);
                sm
            }
        };
        let trace = r.finish();
        let line = golden
            .lines()
            .filter(|l| !l.trim().is_empty())
            .nth(i)
            .unwrap();
        assert_eq!(trace, step.trace, "trace of `{line}`");
        let expected = if step.to == "-" {
            leaf.clone().unwrap()
        } else {
            step.to.clone()
        };
        assert_eq!(state(&sm), expected, "final state of `{line}`");
        checked += 1;
    }
    assert_eq!(checked, steps.len());
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
// Svc.FpySequencer.SequencerStateMachine
// ===========================================================================

/// Stand-ins for `Svc.FpySequencer.SequenceExecutionArgs` and
/// `BreakpointArgs` (the values play no part in the transitions).
type SequenceExecutionArgs = TestStruct;
type BreakpointArgs = TestStruct;

traced_machine! {
    mod FpySequencer;
    state machine SequencerStateMachine: u8 {
        actions {
            signalEntered => action_signalEntered,
            setSequenceFilePath => action_setSequenceFilePath (ref SequenceExecutionArgs),
            setSequenceBlockState => action_setSequenceBlockState (ref SequenceExecutionArgs),
            setSequenceArguments => action_setSequenceArguments (ref SequenceExecutionArgs),
            validate => action_validate,
            report_seqSucceeded => action_report_seqSucceeded,
            report_seqCancelled => action_report_seqCancelled,
            report_seqFailed => action_report_seqFailed,
            report_seqStarted => action_report_seqStarted,
            setGoalState_RUNNING => action_setGoalState_RUNNING,
            setGoalState_VALID => action_setGoalState_VALID,
            setGoalState_IDLE => action_setGoalState_IDLE,
            sendCmdResponse_OK => action_sendCmdResponse_OK,
            sendCmdResponse_EXECUTION_ERROR => action_sendCmdResponse_EXECUTION_ERROR,
            clearSequenceFile => action_clearSequenceFile,
            clearBreakpoint => action_clearBreakpoint,
            clearSequenceArguments => action_clearSequenceArguments,
            checkShouldWake => action_checkShouldWake,
            dispatchStatement => action_dispatchStatement,
            resetRuntime => action_resetRuntime,
            checkStatementTimeout => action_checkStatementTimeout,
            incrementSequenceCounter => action_incrementSequenceCounter,
            pushArgsToStack => action_pushArgsToStack,
            report_seqBroken => action_report_seqBroken,
            setBreakpoint => action_setBreakpoint (ref BreakpointArgs),
            setBreakBeforeNextLine => action_setBreakBeforeNextLine,
            clearBreakBeforeNextLine => action_clearBreakBeforeNextLine,
        }
        guards {
            goalStateIs_RUNNING => guard_goalStateIs_RUNNING,
            shouldBreak => guard_shouldBreak,
            breakOnce => guard_breakOnce,
        }
        signals {
            checkTimersIn = 1 => send_signal_checkTimersIn {
                RUNNING_AWAITING_STATEMENT_RESPONSE => do { checkStatementTimeout };
                RUNNING_SLEEPING => do { checkShouldWake, checkStatementTimeout };
            }
            cmd_BREAK = 2 => send_signal_cmd_BREAK {
                RUNNING => do { setBreakBeforeNextLine };
            }
            cmd_CANCEL = 3 => send_signal_cmd_CANCEL {
                VALIDATING => do { report_seqCancelled, sendCmdResponse_EXECUTION_ERROR } enter IDLE;
                AWAITING_CMD_RUN_VALIDATED => do { report_seqCancelled } enter IDLE;
                RUNNING => do { report_seqCancelled, sendCmdResponse_EXECUTION_ERROR } enter IDLE;
            }
            cmd_CLEAR_BREAKPOINT = 4 => send_signal_cmd_CLEAR_BREAKPOINT {
                IDLE => do { clearBreakpoint };
                VALIDATING => do { clearBreakpoint };
                AWAITING_CMD_RUN_VALIDATED => do { clearBreakpoint };
                RUNNING => do { clearBreakpoint };
            }
            cmd_CONTINUE = 5 => send_signal_cmd_CONTINUE {
                RUNNING_PAUSED => enter RUNNING_DISPATCH_STATEMENT;
            }
            cmd_RUN = 6 => send_signal_cmd_RUN (ref SequenceExecutionArgs) {
                IDLE => do {
                    setGoalState_RUNNING, setSequenceFilePath, setSequenceBlockState,
                    setSequenceArguments
                } enter VALIDATING;
            }
            cmd_RUN_VALIDATED = 7 => send_signal_cmd_RUN_VALIDATED (ref SequenceExecutionArgs) {
                AWAITING_CMD_RUN_VALIDATED => do { setSequenceBlockState } enter RUNNING;
            }
            cmd_SET_BREAKPOINT = 8 => send_signal_cmd_SET_BREAKPOINT (ref BreakpointArgs) {
                IDLE => do { setBreakpoint };
                VALIDATING => do { setBreakpoint };
                AWAITING_CMD_RUN_VALIDATED => do { setBreakpoint };
                RUNNING => do { setBreakpoint };
            }
            cmd_STEP = 9 => send_signal_cmd_STEP {
                RUNNING_PAUSED => do { setBreakBeforeNextLine } enter RUNNING_DISPATCH_STATEMENT;
            }
            cmd_VALIDATE = 10 => send_signal_cmd_VALIDATE (ref SequenceExecutionArgs) {
                IDLE => do {
                    setGoalState_VALID, setSequenceFilePath, setSequenceArguments
                } enter VALIDATING;
            }
            entered = 11 => send_signal_entered {
                VALIDATING => do { validate };
                RUNNING_PAUSED => if breakOnce do { clearBreakpoint };
            }
            result_checkShouldWake_keepSleeping = 12 => send_signal_result_checkShouldWake_keepSleeping {}
            result_checkShouldWake_wakeup = 13 => send_signal_result_checkShouldWake_wakeup {
                RUNNING_SLEEPING => enter RUNNING_BREAK_CHECK;
            }
            result_checkStatementTimeout_noTimeout = 14 => send_signal_result_checkStatementTimeout_noTimeout {}
            result_checkStatementTimeout_statementTimeout = 15 => send_signal_result_checkStatementTimeout_statementTimeout {
                RUNNING_AWAITING_STATEMENT_RESPONSE => do { report_seqFailed, sendCmdResponse_EXECUTION_ERROR } enter IDLE;
                RUNNING_SLEEPING => do { report_seqFailed, sendCmdResponse_EXECUTION_ERROR } enter IDLE;
            }
            result_dispatchStatement_failure = 16 => send_signal_result_dispatchStatement_failure {
                RUNNING_DISPATCH_STATEMENT => do { report_seqFailed, sendCmdResponse_EXECUTION_ERROR } enter IDLE;
            }
            result_dispatchStatement_noMoreStatements = 17 => send_signal_result_dispatchStatement_noMoreStatements {
                RUNNING_DISPATCH_STATEMENT => do { report_seqSucceeded, sendCmdResponse_OK } enter IDLE;
            }
            result_dispatchStatement_success = 18 => send_signal_result_dispatchStatement_success {
                RUNNING_DISPATCH_STATEMENT => enter RUNNING_AWAITING_STATEMENT_RESPONSE;
            }
            result_failure = 19 => send_signal_result_failure {
                VALIDATING => do { report_seqFailed, sendCmdResponse_EXECUTION_ERROR } enter IDLE;
            }
            result_success = 20 => send_signal_result_success {
                VALIDATING => enter VALID;
            }
            result_timeOpFailed = 21 => send_signal_result_timeOpFailed {
                RUNNING_AWAITING_STATEMENT_RESPONSE => do { report_seqFailed, sendCmdResponse_EXECUTION_ERROR } enter IDLE;
                RUNNING_SLEEPING => do { report_seqFailed, sendCmdResponse_EXECUTION_ERROR } enter IDLE;
            }
            stmtResponse_beginSleep = 22 => send_signal_stmtResponse_beginSleep {
                RUNNING_AWAITING_STATEMENT_RESPONSE => enter RUNNING_SLEEPING;
            }
            stmtResponse_failure = 23 => send_signal_stmtResponse_failure {
                RUNNING_AWAITING_STATEMENT_RESPONSE => do { report_seqFailed, sendCmdResponse_EXECUTION_ERROR } enter IDLE;
            }
            stmtResponse_keepWaiting = 24 => send_signal_stmtResponse_keepWaiting {}
            stmtResponse_success = 25 => send_signal_stmtResponse_success {
                RUNNING_AWAITING_STATEMENT_RESPONSE => enter RUNNING_BREAK_CHECK;
            }
            stmtResponse_unexpected = 26 => send_signal_stmtResponse_unexpected {
                RUNNING => do { report_seqFailed, sendCmdResponse_EXECUTION_ERROR } enter IDLE;
            }
        }
        initial enter IDLE;
        states {
            state AWAITING_CMD_RUN_VALIDATED = 1 {
                entry do { sendCmdResponse_OK }
            }
            state IDLE = 2 {
                entry do { clearBreakpoint, setGoalState_IDLE, clearSequenceFile, clearSequenceArguments }
            }
            state RUNNING {
                initial enter RUNNING_BREAK_CHECK;
                entry do { resetRuntime, incrementSequenceCounter, pushArgsToStack }
            }
            state RUNNING_AWAITING_STATEMENT_RESPONSE = 3 in RUNNING {}
            state RUNNING_DISPATCH_STATEMENT = 4 in RUNNING {
                entry do { dispatchStatement }
            }
            state RUNNING_PAUSED = 5 in RUNNING {
                entry do { signalEntered, clearBreakBeforeNextLine }
            }
            state RUNNING_SLEEPING = 6 in RUNNING {}
            state VALIDATING = 7 {
                entry do { report_seqStarted, signalEntered }
            }
        }
        choices {
            choice VALID {
                if goalStateIs_RUNNING enter RUNNING else enter AWAITING_CMD_RUN_VALIDATED
            }
            choice RUNNING_BREAK_CHECK in RUNNING {
                if shouldBreak do { report_seqBroken } enter RUNNING_PAUSED
                else enter RUNNING_DISPATCH_STATEMENT
            }
        }
    }
}

check_machine!(
    fpy_sequencer_matches_fpp_to_cpp,
    FpySequencer,
    "FpySequencer.txt"
);

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

/// The `State` discriminants of the FpySequencer machine are the
/// `fpp-to-cpp` ones (sorted by name, not in definition order).
#[test]
fn fpy_sequencer_state_enum_matches_fpp_to_cpp() {
    use FpySequencer::{Signal, State};
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
