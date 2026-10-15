//! Port of `FppTest/state_machine/internal/choice` (transitions out of
//! choices: basic, typed, two inputs of different types, sequences,
//! choice-to-choice and choice-to-state with hierarchy) and
//! `FppTest/state_machine/internal/initial` (initial transitions: basic,
//! to a choice, nested).
#![allow(non_snake_case)]

mod sm_harness;

use fprime_comp::config::FwEnumStoreType;
use fprime_comp::fpp_state_machine;
use sm_harness::{Guard, History, Pick};

/// Histories of untyped actions `a`/`b` and no-arg guards `g`/`g1`/`g2`.
struct Untyped<S: Copy> {
    a: History<S, ()>,
    b: History<S, ()>,
    g1: Guard<S, ()>,
    g2: Guard<S, ()>,
}

impl<S: Copy> Default for Untyped<S> {
    fn default() -> Self {
        Self {
            a: History::new(),
            b: History::new(),
            g1: Guard::new(),
            g2: Guard::new(),
        }
    }
}

/// Histories of a typed action `a: U32`, an untyped `b`, an untyped guard
/// `g1` and a typed guard `g`/`g2: U32`.
struct TypedU32<S: Copy> {
    a: History<S, u32>,
    b: History<S, ()>,
    g1: Guard<S, ()>,
    g2: Guard<S, u32>,
}

impl<S: Copy> Default for TypedU32<S> {
    fn default() -> Self {
        Self {
            a: History::new(),
            b: History::new(),
            g1: Guard::new(),
            g2: Guard::new(),
        }
    }
}

// ===========================================================================
// choice
// ===========================================================================

mod ChoiceBasic {
    super::fpp_state_machine! {
        /// A basic state machine with a choice
        state machine Basic: u8 {
            actions { a => action_a, b => action_b }
            guards { g => guard_g }
            signals {
                s = 1 => send_signal_s {
                    S1 => enter C;
                }
            }
            initial enter S1;
            states {
                state S1 = 1 {}
                state S2 = 2 {}
                state S3 = 3 {}
            }
            choices {
                choice C { if g do { a } enter S2 else do { b } enter S3 }
            }
        }
    }
}

impl ChoiceBasic::Handlers for Untyped<ChoiceBasic::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: ChoiceBasic::Signal) {
        self.a.push(signal, ());
    }
    fn action_b(&self, _: FwEnumStoreType, signal: ChoiceBasic::Signal) {
        self.b.push(signal, ());
    }
    fn guard_g(&self, _: FwEnumStoreType, signal: ChoiceBasic::Signal) -> bool {
        self.g1.call(signal, ())
    }
}

fn choice_basic(guard: bool) {
    use ChoiceBasic::{Signal, State, StateMachine};
    let h = Untyped::default();
    h.g1.set_return_value(guard);
    let sm = StateMachine::new();
    let id = Pick::new(1).state_machine_id();
    sm.init(id, &h);
    assert_eq!(sm.id(), id);
    assert_eq!(sm.get_state(), State::S1);
    assert_eq!(h.g1.history().size(), 0);
    assert_eq!(h.a.size(), 0);
    assert_eq!(h.b.size(), 0);
    sm.send_signal_s(&h);
    sm.send_signal_s(&h);
    assert_eq!(h.g1.history().signals(), vec![Signal::s]);
    if guard {
        assert_eq!(h.a.signals(), vec![Signal::s]);
        assert_eq!(h.b.size(), 0);
        assert_eq!(sm.get_state(), State::S2);
    } else {
        assert_eq!(h.a.size(), 0);
        assert_eq!(h.b.signals(), vec![Signal::s]);
        assert_eq!(sm.get_state(), State::S3);
    }
}

#[test]
fn choice_basic_true() {
    choice_basic(true);
}

#[test]
fn choice_basic_false() {
    choice_basic(false);
}

mod ChoiceBasicU32 {
    super::fpp_state_machine! {
        /// A basic state machine with a U32 choice
        state machine BasicU32: u8 {
            actions { a => action_a (val u32), b => action_b }
            guards { g => guard_g (val u32) }
            signals {
                s = 1 => send_signal_s (val u32) {
                    S1 => enter C;
                }
            }
            initial enter S1;
            states {
                state S1 = 1 {}
                state S2 = 2 {}
                state S3 = 3 {}
            }
            choices {
                choice C (val u32) { if g do { a } enter S2 else do { b } enter S3 }
            }
        }
    }
}

impl ChoiceBasicU32::Handlers for TypedU32<ChoiceBasicU32::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: ChoiceBasicU32::Signal, value: u32) {
        self.a.push(signal, value);
    }
    fn action_b(&self, _: FwEnumStoreType, signal: ChoiceBasicU32::Signal) {
        self.b.push(signal, ());
    }
    fn guard_g(&self, _: FwEnumStoreType, signal: ChoiceBasicU32::Signal, value: u32) -> bool {
        self.g2.call(signal, value)
    }
}

fn choice_basic_u32(guard: bool) {
    use ChoiceBasicU32::{Signal, State, StateMachine};
    let pick = Pick::new(2);
    let h = TypedU32::default();
    h.g2.set_return_value(guard);
    let sm = StateMachine::new();
    sm.init(pick.state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1);
    let value = pick.any();
    sm.send_signal_s(&h, value);
    sm.send_signal_s(&h, value);
    assert_eq!(h.g2.history().items(), vec![(Signal::s, value)]);
    if guard {
        assert_eq!(h.a.items(), vec![(Signal::s, value)]);
        assert_eq!(h.b.size(), 0);
        assert_eq!(sm.get_state(), State::S2);
    } else {
        assert_eq!(h.a.size(), 0);
        assert_eq!(h.b.signals(), vec![Signal::s]);
        assert_eq!(sm.get_state(), State::S3);
    }
}

#[test]
fn choice_basic_u32_true() {
    choice_basic_u32(true);
}

#[test]
fn choice_basic_u32_false() {
    choice_basic_u32(false);
}

mod InputPairU16U32 {
    super::fpp_state_machine! {
        /// A state machine with two inputs to a choice
        state machine InputPairU16U32: u8 {
            actions { a => action_a (val u32) }
            guards { g => guard_g (val u32) }
            signals {
                s1 = 1 => send_signal_s1 (val u16) {
                    S1 => enter C;
                }
                s2 = 2 => send_signal_s2 (val u32) {
                    S1 => enter C;
                }
            }
            initial enter S1;
            states {
                state S1 = 1 {}
                state S2 = 2 {}
                state S3 = 3 {}
            }
            choices {
                // The U16 signal converts to the choice's U32, as in FPP.
                choice C (val u32) { if g do { a } enter S2 else do { a } enter S3 }
            }
        }
    }
}

impl InputPairU16U32::Handlers for TypedU32<InputPairU16U32::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: InputPairU16U32::Signal, value: u32) {
        self.a.push(signal, value);
    }
    fn guard_g(&self, _: FwEnumStoreType, signal: InputPairU16U32::Signal, value: u32) -> bool {
        self.g2.call(signal, value)
    }
}

fn input_pair(use_s1: bool, guard: bool) {
    use InputPairU16U32::{Signal, State, StateMachine};
    let pick = Pick::new(3);
    let h = TypedU32::default();
    h.g2.set_return_value(guard);
    let sm = StateMachine::new();
    sm.init(pick.state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1);
    let (signal, value) = if use_s1 {
        let value = pick.any() as u16;
        sm.send_signal_s1(&h, value);
        sm.send_signal_s1(&h, value);
        (Signal::s1, u32::from(value))
    } else {
        let value = pick.any();
        sm.send_signal_s2(&h, value);
        sm.send_signal_s2(&h, value);
        (Signal::s2, value)
    };
    assert_eq!(h.g2.history().items(), vec![(signal, value)]);
    assert_eq!(h.a.items(), vec![(signal, value)]);
    assert_eq!(sm.get_state(), if guard { State::S2 } else { State::S3 });
}

#[test]
fn input_pair_s1_true() {
    input_pair(true, true);
}

#[test]
fn input_pair_s1_false() {
    input_pair(true, false);
}

#[test]
fn input_pair_s2_true() {
    input_pair(false, true);
}

#[test]
fn input_pair_s2_false() {
    input_pair(false, false);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ActionId {
    ExitS1,
    A,
    EnterS2,
    EnterS3,
}

/// A `SignalValueHistory<Signal, ActionId>` and two no-arg guards.
struct ActionHistory<S: Copy> {
    actions: History<S, ActionId>,
    g1: Guard<S, ()>,
    g2: Guard<S, ()>,
}

impl<S: Copy> Default for ActionHistory<S> {
    fn default() -> Self {
        Self {
            actions: History::new(),
            g1: Guard::new(),
            g2: Guard::new(),
        }
    }
}

mod ChoiceToChoice {
    super::fpp_state_machine! {
        /// A state machine for testing choice-to-choice transitions with
        /// hierarchy
        state machine ChoiceToChoice: u8 {
            actions {
                exitS1 => action_exitS1,
                a => action_a,
                enterS2 => action_enterS2,
            }
            guards { g1 => guard_g1, g2 => guard_g2 }
            signals {
                s = 1 => send_signal_s {
                    S1 => enter S1_C1;
                }
            }
            initial enter S1;
            states {
                state S1 = 1 {
                    exit do { exitS1 }
                }
                state S2 {
                    initial enter S2_S3;
                    entry do { enterS2 }
                }
                state S2_S3 = 2 in S2 {}
                state S2_S4 = 3 in S2 {}
            }
            choices {
                choice S1_C1 in S1 {
                    if g1 do { a } enter S2 else do { a } enter S2_C2
                }
                choice S2_C2 in S2 { if g2 enter S2_S3 else enter S2_S4 }
            }
        }
    }
}

impl ChoiceToChoice::Handlers for ActionHistory<ChoiceToChoice::Signal> {
    fn action_exitS1(&self, _: FwEnumStoreType, signal: ChoiceToChoice::Signal) {
        self.actions.push(signal, ActionId::ExitS1);
    }
    fn action_a(&self, _: FwEnumStoreType, signal: ChoiceToChoice::Signal) {
        self.actions.push(signal, ActionId::A);
    }
    fn action_enterS2(&self, _: FwEnumStoreType, signal: ChoiceToChoice::Signal) {
        self.actions.push(signal, ActionId::EnterS2);
    }
    fn guard_g1(&self, _: FwEnumStoreType, signal: ChoiceToChoice::Signal) -> bool {
        self.g1.call(signal, ())
    }
    fn guard_g2(&self, _: FwEnumStoreType, signal: ChoiceToChoice::Signal) -> bool {
        self.g2.call(signal, ())
    }
}

fn choice_to_choice(g1: bool, g2: bool) {
    use ChoiceToChoice::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    h.g1.set_return_value(g1);
    h.g2.set_return_value(g2);
    let sm = StateMachine::new();
    sm.init(Pick::new(4).state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1);
    assert_eq!(h.actions.size(), 0);
    sm.send_signal_s(&h);
    sm.send_signal_s(&h);
    assert_eq!(h.g1.history().signals(), vec![Signal::s]);
    assert_eq!(h.g2.history().size(), if g1 { 0 } else { 1 });
    assert_eq!(h.actions.signals(), vec![Signal::s; 3]);
    assert_eq!(
        h.actions.values(),
        [ActionId::ExitS1, ActionId::A, ActionId::EnterS2]
    );
    let expected = if g1 || g2 { State::S2_S3 } else { State::S2_S4 };
    assert_eq!(sm.get_state(), expected);
}

#[test]
fn choice_to_choice_g1_true() {
    choice_to_choice(true, false);
}

#[test]
fn choice_to_choice_g1_false_g2_true() {
    choice_to_choice(false, true);
}

#[test]
fn choice_to_choice_g1_false_g2_false() {
    choice_to_choice(false, false);
}

mod ChoiceToState {
    super::fpp_state_machine! {
        /// A state machine for testing choice-to-state transitions with
        /// hierarchy
        state machine ChoiceToState: u8 {
            actions {
                exitS1 => action_exitS1,
                a => action_a,
                enterS2 => action_enterS2,
                enterS3 => action_enterS3,
            }
            guards { g => guard_g }
            signals {
                s = 1 => send_signal_s {
                    S1 => enter S1_C;
                }
            }
            initial enter S1;
            states {
                state S1 = 1 {
                    exit do { exitS1 }
                }
                state S2 {
                    initial do { a } enter S2_S3;
                    entry do { enterS2 }
                }
                state S2_S3 = 2 in S2 {
                    entry do { enterS3 }
                }
            }
            choices {
                choice S1_C in S1 { if g do { a } enter S2 else do { a } enter S2_S3 }
            }
        }
    }
}

impl ChoiceToState::Handlers for ActionHistory<ChoiceToState::Signal> {
    fn action_exitS1(&self, _: FwEnumStoreType, signal: ChoiceToState::Signal) {
        self.actions.push(signal, ActionId::ExitS1);
    }
    fn action_a(&self, _: FwEnumStoreType, signal: ChoiceToState::Signal) {
        self.actions.push(signal, ActionId::A);
    }
    fn action_enterS2(&self, _: FwEnumStoreType, signal: ChoiceToState::Signal) {
        self.actions.push(signal, ActionId::EnterS2);
    }
    fn action_enterS3(&self, _: FwEnumStoreType, signal: ChoiceToState::Signal) {
        self.actions.push(signal, ActionId::EnterS3);
    }
    fn guard_g(&self, _: FwEnumStoreType, signal: ChoiceToState::Signal) -> bool {
        self.g1.call(signal, ())
    }
}

#[test]
fn choice_to_state_true() {
    use ChoiceToState::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    h.g1.set_return_value(true);
    let sm = StateMachine::new();
    sm.init(Pick::new(5).state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1);
    sm.send_signal_s(&h);
    sm.send_signal_s(&h);
    assert_eq!(h.g1.history().signals(), vec![Signal::s]);
    assert_eq!(h.actions.signals(), vec![Signal::s; 5]);
    assert_eq!(
        h.actions.values(),
        [
            ActionId::ExitS1,
            ActionId::A,
            ActionId::EnterS2,
            ActionId::A,
            ActionId::EnterS3
        ]
    );
    assert_eq!(sm.get_state(), State::S2_S3);
}

#[test]
fn choice_to_state_false() {
    use ChoiceToState::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(Pick::new(6).state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1);
    sm.send_signal_s(&h);
    assert_eq!(h.g1.history().signals(), vec![Signal::s]);
    assert_eq!(h.actions.signals(), vec![Signal::s; 4]);
    assert_eq!(
        h.actions.values(),
        [
            ActionId::ExitS1,
            ActionId::A,
            ActionId::EnterS2,
            ActionId::EnterS3
        ]
    );
    assert_eq!(sm.get_state(), State::S2_S3);
}

mod Sequence {
    super::fpp_state_machine! {
        /// A basic state machine with a choice sequence
        state machine Sequence: u8 {
            actions { a => action_a, b => action_b }
            guards { g1 => guard_g1, g2 => guard_g2 }
            signals {
                s = 1 => send_signal_s {
                    S1 => enter C1;
                }
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
}

impl Sequence::Handlers for Untyped<Sequence::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: Sequence::Signal) {
        self.a.push(signal, ());
    }
    fn action_b(&self, _: FwEnumStoreType, signal: Sequence::Signal) {
        self.b.push(signal, ());
    }
    fn guard_g1(&self, _: FwEnumStoreType, signal: Sequence::Signal) -> bool {
        self.g1.call(signal, ())
    }
    fn guard_g2(&self, _: FwEnumStoreType, signal: Sequence::Signal) -> bool {
        self.g2.call(signal, ())
    }
}

fn sequence(g1: bool, g2: bool) {
    use Sequence::{Signal, State, StateMachine};
    let h = Untyped::default();
    h.g1.set_return_value(g1);
    h.g2.set_return_value(g2);
    let sm = StateMachine::new();
    sm.init(Pick::new(7).state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1);
    sm.send_signal_s(&h);
    sm.send_signal_s(&h);
    assert_eq!(h.g1.history().signals(), vec![Signal::s]);
    if g1 {
        assert_eq!(h.g2.history().size(), 0);
        assert_eq!(h.a.size(), 0);
        assert_eq!(h.b.size(), 0);
        assert_eq!(sm.get_state(), State::S2);
    } else if g2 {
        assert_eq!(h.g2.history().signals(), vec![Signal::s]);
        assert_eq!(h.a.signals(), vec![Signal::s]);
        assert_eq!(h.b.size(), 0);
        assert_eq!(sm.get_state(), State::S3);
    } else {
        assert_eq!(h.g2.history().signals(), vec![Signal::s]);
        assert_eq!(h.a.size(), 0);
        assert_eq!(h.b.signals(), vec![Signal::s]);
        assert_eq!(sm.get_state(), State::S4);
    }
}

#[test]
fn sequence_g1_true() {
    sequence(true, false);
}

#[test]
fn sequence_g1_false_g2_true() {
    sequence(false, true);
}

#[test]
fn sequence_g1_false_g2_false() {
    sequence(false, false);
}

mod SequenceU32 {
    super::fpp_state_machine! {
        /// A basic state machine with a U32 choice sequence
        state machine SequenceU32: u8 {
            actions { a => action_a (val u32), b => action_b }
            guards { g1 => guard_g1, g2 => guard_g2 (val u32) }
            signals {
                s = 1 => send_signal_s (val u32) {
                    S1 => enter C1;
                }
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
}

impl SequenceU32::Handlers for TypedU32<SequenceU32::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: SequenceU32::Signal, value: u32) {
        self.a.push(signal, value);
    }
    fn action_b(&self, _: FwEnumStoreType, signal: SequenceU32::Signal) {
        self.b.push(signal, ());
    }
    fn guard_g1(&self, _: FwEnumStoreType, signal: SequenceU32::Signal) -> bool {
        self.g1.call(signal, ())
    }
    fn guard_g2(&self, _: FwEnumStoreType, signal: SequenceU32::Signal, value: u32) -> bool {
        self.g2.call(signal, value)
    }
}

fn sequence_u32(g1: bool, g2: bool) {
    use SequenceU32::{Signal, State, StateMachine};
    let pick = Pick::new(8);
    let h = TypedU32::default();
    h.g1.set_return_value(g1);
    h.g2.set_return_value(g2);
    let sm = StateMachine::new();
    sm.init(pick.state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1);
    let value = pick.any();
    sm.send_signal_s(&h, value);
    sm.send_signal_s(&h, value);
    assert_eq!(h.g1.history().signals(), vec![Signal::s]);
    if g1 {
        assert_eq!(h.g2.history().size(), 0);
        assert_eq!(h.a.size(), 0);
        assert_eq!(h.b.size(), 0);
        assert_eq!(sm.get_state(), State::S2);
    } else if g2 {
        assert_eq!(h.g2.history().items(), vec![(Signal::s, value)]);
        assert_eq!(h.a.items(), vec![(Signal::s, value)]);
        assert_eq!(h.b.size(), 0);
        assert_eq!(sm.get_state(), State::S3);
    } else {
        assert_eq!(h.g2.history().items(), vec![(Signal::s, value)]);
        assert_eq!(h.a.size(), 0);
        assert_eq!(h.b.signals(), vec![Signal::s]);
        assert_eq!(sm.get_state(), State::S4);
    }
}

#[test]
fn sequence_u32_g1_true() {
    sequence_u32(true, false);
}

#[test]
fn sequence_u32_g1_false_g2_true() {
    sequence_u32(false, true);
}

#[test]
fn sequence_u32_g1_false_g2_false() {
    sequence_u32(false, false);
}

// ===========================================================================
// initial
// ===========================================================================

mod InitialBasic {
    super::fpp_state_machine! {
        /// A basic state machine with an initial action
        state machine Basic: u8 {
            actions { a => action_a }
            guards {}
            signals {}
            initial do { a } enter S;
            states {
                state S = 1 {
                    entry do { a, a }
                }
            }
            choices {}
        }
    }
}

impl InitialBasic::Handlers for Untyped<InitialBasic::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: InitialBasic::Signal) {
        self.a.push(signal, ());
    }
}

#[test]
fn initial_basic() {
    use InitialBasic::{Signal, State, StateMachine};
    let h = Untyped::default();
    let sm = StateMachine::new();
    let id = Pick::new(9).state_machine_id();
    sm.init(id, &h);
    assert_eq!(sm.id(), id);
    assert_eq!(sm.get_state(), State::S);
    assert_eq!(h.a.signals(), vec![Signal::__FPRIME_INITIAL_TRANSITION; 3]);
}

mod InitialChoice {
    super::fpp_state_machine! {
        /// A state machine with an initial choice
        state machine Choice: u8 {
            actions { a => action_a }
            guards { g => guard_g }
            signals {}
            initial do { a } enter C;
            states {
                state S = 1 {
                    entry do { a }
                }
                state T = 2 {
                    entry do { a, a }
                }
            }
            choices {
                choice C { if g do { a } enter S else do { a, a } enter T }
            }
        }
    }
}

impl InitialChoice::Handlers for Untyped<InitialChoice::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: InitialChoice::Signal) {
        self.a.push(signal, ());
    }
    fn guard_g(&self, _: FwEnumStoreType, signal: InitialChoice::Signal) -> bool {
        self.g1.call(signal, ())
    }
}

fn initial_choice(guard: bool) {
    use InitialChoice::{Signal, State, StateMachine};
    let h = Untyped::default();
    h.g1.set_return_value(guard);
    let sm = StateMachine::new();
    let id = Pick::new(10).state_machine_id();
    sm.init(id, &h);
    assert_eq!(sm.id(), id);
    let (state, actions) = if guard { (State::S, 3) } else { (State::T, 5) };
    assert_eq!(sm.get_state(), state);
    assert_eq!(
        h.a.signals(),
        vec![Signal::__FPRIME_INITIAL_TRANSITION; actions]
    );
    assert_eq!(
        h.g1.history().signals(),
        vec![Signal::__FPRIME_INITIAL_TRANSITION]
    );
}

#[test]
fn initial_choice_false() {
    initial_choice(false);
}

#[test]
fn initial_choice_true() {
    initial_choice(true);
}

mod InitialNested {
    super::fpp_state_machine! {
        /// A state machine with nested initial transitions
        state machine Nested: u8 {
            actions { a => action_a }
            guards {}
            signals {}
            initial do { a } enter S;
            states {
                state S {
                    initial enter S_T;
                    entry do { a, a }
                }
                state S_T = 1 in S {
                    entry do { a, a, a }
                }
            }
            choices {}
        }
    }
}

impl InitialNested::Handlers for Untyped<InitialNested::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: InitialNested::Signal) {
        self.a.push(signal, ());
    }
}

#[test]
fn initial_nested() {
    use InitialNested::{Signal, State, StateMachine};
    let h = Untyped::default();
    let sm = StateMachine::new();
    let id = Pick::new(11).state_machine_id();
    sm.init(id, &h);
    assert_eq!(sm.id(), id);
    assert_eq!(sm.get_state(), State::S_T);
    assert_eq!(h.a.signals(), vec![Signal::__FPRIME_INITIAL_TRANSITION; 6]);
}

/// `init` again re-runs the initial transition (the C++ testers call
/// `initBase` once per test on a fresh object; re-initializing is allowed).
#[test]
fn initial_reinit_reruns_the_initial_transition() {
    use InitialNested::{State, StateMachine};
    let h = Untyped::default();
    let sm = StateMachine::new();
    sm.init(1, &h);
    sm.init(2, &h);
    assert_eq!(sm.id(), 2);
    assert_eq!(sm.get_state(), State::S_T);
    assert_eq!(h.a.size(), 12);
}
