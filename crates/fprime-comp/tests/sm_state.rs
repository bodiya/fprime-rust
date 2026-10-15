//! Port of `FppTest/state_machine/internal/state`: transitions out of
//! states in FPP internal state machines (basic, guarded, internal, self,
//! typed signals and actions, polymorphism, state-to-child,
//! state-to-choice, state-to-self, state-to-state).
//!
//! Each state machine is the upstream `.fppi` model written for
//! `fpp_state_machine!` (signals and leaf states numbered in name order,
//! as `fpp-to-cpp` does); each test is the upstream
//! test, action for action.
#![allow(non_snake_case)]

mod sm_harness;

use fprime_comp::config::FwEnumStoreType;
use fprime_comp::fpp_state_machine;
use sm_harness::{Guard, History, Pick, TestAbsType, TestArray, TestEnum, TestStruct};

// ---------------------------------------------------------------------------
// Basic
// ---------------------------------------------------------------------------

mod Basic {
    super::fpp_state_machine! {
        /// A basic state machine
        state machine Basic: u8 {
            actions { a => action_a }
            guards {}
            signals {
                s = 1 => send_signal_s {
                    S => do { a, a } enter T;
                }
            }
            initial enter S;
            states {
                state S = 1 {
                    exit do { a }
                }
                state T = 2 {
                    entry do { a, a, a }
                }
            }
            choices {}
        }
    }
}

/// The harness of the untyped one-action machines: a history of `a`.
struct AHistory<S: Copy> {
    a: History<S, ()>,
}

impl<S: Copy> Default for AHistory<S> {
    fn default() -> Self {
        Self { a: History::new() }
    }
}

impl Basic::Handlers for AHistory<Basic::Signal> {
    fn action_a(&self, _sm_id: FwEnumStoreType, signal: Basic::Signal) {
        self.a.push(signal, ());
    }
}

#[test]
fn basic_test() {
    use Basic::{Signal, State, StateMachine};
    let h = AHistory::default();
    let sm = StateMachine::new();
    let id = Pick::new(1).state_machine_id();
    sm.init(id, &h);
    assert_eq!(sm.id(), id);
    assert_eq!(sm.get_state(), State::S);
    assert_eq!(h.a.size(), 0);
    sm.send_signal_s(&h);
    assert_eq!(sm.get_state(), State::T);
    sm.send_signal_s(&h);
    assert_eq!(sm.get_state(), State::T);
    assert_eq!(h.a.signals(), vec![Signal::s; 6]);
}

#[test]
fn basic_enums_have_the_cpp_discriminants() {
    use Basic::{Signal, State};
    assert_eq!(State::__FPRIME_UNINITIALIZED as u8, 0);
    assert_eq!(State::S as u8, 1);
    assert_eq!(State::T as u8, 2);
    assert_eq!(State::SERIALIZED_SIZE, 1);
    assert_eq!(Signal::__FPRIME_INITIAL_TRANSITION.as_repr(), 0);
    assert_eq!(Signal::s.as_repr(), 1);
    assert_eq!(Signal::try_from(1), Ok(Signal::s));
    assert_eq!(Signal::try_from(2), Err(2));
    assert_eq!(Basic::StateMachine::MAX_SIGNAL_DATA_SIZE, 0);
}

#[test]
#[should_panic]
fn basic_signal_before_init_asserts() {
    let h = AHistory::default();
    Basic::StateMachine::new().send_signal_s(&h);
}

/// Re-entering an instance from one of its own actions breaks
/// run-to-completion; it asserts (a deliberate deviation — see
/// `fprime_comp::sm::SmCore`).
#[test]
#[should_panic]
fn basic_reentrant_signal_asserts() {
    struct Reenter<'a>(&'a Basic::StateMachine);
    impl Basic::Handlers for Reenter<'_> {
        fn action_a(&self, _: FwEnumStoreType, _: Basic::Signal) {
            self.0.send_signal_s(self);
        }
    }
    let sm = Basic::StateMachine::new();
    let h = Reenter(&sm);
    sm.init(0, &h);
    sm.send_signal_s(&h);
}

// ---------------------------------------------------------------------------
// BasicGuard
// ---------------------------------------------------------------------------

mod BasicGuard {
    super::fpp_state_machine! {
        /// A basic state machine with a guard
        state machine BasicGuard: u8 {
            actions { a => action_a }
            guards { g => guard_g }
            signals {
                s = 1 => send_signal_s {
                    S => if g do { a, a } enter T;
                }
            }
            initial enter S;
            states {
                state S = 1 {
                    exit do { a }
                }
                state T = 2 {
                    entry do { a, a, a }
                }
            }
            choices {}
        }
    }
}

struct AGHistory<S: Copy> {
    a: History<S, ()>,
    g: Guard<S, ()>,
}

impl<S: Copy> Default for AGHistory<S> {
    fn default() -> Self {
        Self {
            a: History::new(),
            g: Guard::new(),
        }
    }
}

impl BasicGuard::Handlers for AGHistory<BasicGuard::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: BasicGuard::Signal) {
        self.a.push(signal, ());
    }
    fn guard_g(&self, _: FwEnumStoreType, signal: BasicGuard::Signal) -> bool {
        self.g.call(signal, ())
    }
}

fn basic_guard_check(h: &AGHistory<BasicGuard::Signal>, actions: usize, guards: usize) {
    use BasicGuard::Signal;
    assert_eq!(h.a.signals(), vec![Signal::s; actions]);
    assert_eq!(h.g.history().signals(), vec![Signal::s; guards]);
}

#[test]
fn basic_guard_false() {
    use BasicGuard::{State, StateMachine};
    let h = AGHistory::default();
    let sm = StateMachine::new();
    let id = Pick::new(2).state_machine_id();
    sm.init(id, &h);
    assert_eq!(sm.id(), id);
    assert_eq!(sm.get_state(), State::S);
    assert_eq!(h.a.size(), 0);
    sm.send_signal_s(&h);
    assert_eq!(sm.get_state(), State::S);
    basic_guard_check(&h, 0, 1);
}

#[test]
fn basic_guard_true() {
    use BasicGuard::{State, StateMachine};
    let h = AGHistory::default();
    h.g.set_return_value(true);
    let sm = StateMachine::new();
    sm.init(Pick::new(3).state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S);
    sm.send_signal_s(&h);
    assert_eq!(sm.get_state(), State::T);
    sm.send_signal_s(&h);
    assert_eq!(sm.get_state(), State::T);
    basic_guard_check(&h, 6, 1);
}

// ---------------------------------------------------------------------------
// BasicGuard<T>: BasicGuardU32, BasicGuardTestAbsType, BasicGuardTestArray,
// BasicGuardTestEnum, BasicGuardTestStruct, BasicGuardString
// ---------------------------------------------------------------------------

/// `state machine BasicGuard<T> { action a: T; guard g: T; signal s: T;
/// initial enter S; state S { on s if g do { a } enter T }; state T }`
macro_rules! basic_guard_typed {
    ($name:ident, $mode:ident, $ty:ty, $vty:ty, $pick:expr, $arg:ident => $send:expr, $own:ident => $owned:expr) => {
        mod $name {
            #[allow(unused_imports)]
            use super::*;
            fprime_comp::fpp_state_machine! {
                /// A basic state machine with a typed guard
                state machine $name: u8 {
                    actions { a => action_a ($mode $ty) }
                    guards { g => guard_g ($mode $ty) }
                    signals {
                        s = 1 => send_signal_s ($mode $ty) {
                            S => if g do { a } enter T;
                        }
                    }
                    initial enter S;
                    states {
                        state S = 1 {}
                        state T = 2 {}
                    }
                    choices {}
                }
            }

            #[derive(Default)]
            struct Harness {
                a: History<Signal, $vty>,
                g: Guard<Signal, $vty>,
            }

            impl Handlers for Harness {
                fn action_a(
                    &self,
                    _: FwEnumStoreType,
                    signal: Signal,
                    $own: fprime_comp::__fpp_sm_param_ty!($mode $ty),
                ) {
                    self.a.push(signal, $owned);
                }
                fn guard_g(
                    &self,
                    _: FwEnumStoreType,
                    signal: Signal,
                    $own: fprime_comp::__fpp_sm_param_ty!($mode $ty),
                ) -> bool {
                    self.g.call(signal, $owned)
                }
            }

            #[test]
            fn test_false() {
                let pick = Pick::new(11);
                let h = Harness::default();
                let sm = StateMachine::new();
                let id = pick.state_machine_id();
                sm.init(id, &h);
                assert_eq!(sm.id(), id);
                assert_eq!(sm.get_state(), State::S);
                assert_eq!(h.a.size(), 0);
                assert_eq!(h.g.history().size(), 0);
                let value: $vty = $pick(&pick);
                let $arg = &value;
                sm.send_signal_s(&h, $send);
                assert_eq!(sm.get_state(), State::S);
                assert_eq!(h.g.history().items(), vec![(Signal::s, value.clone())]);
                assert_eq!(h.a.size(), 0);
            }

            #[test]
            fn test_true() {
                let pick = Pick::new(12);
                let h = Harness::default();
                h.g.set_return_value(true);
                let sm = StateMachine::new();
                sm.init(pick.state_machine_id(), &h);
                assert_eq!(sm.get_state(), State::S);
                let value: $vty = $pick(&pick);
                let $arg = &value;
                sm.send_signal_s(&h, $send);
                assert_eq!(sm.get_state(), State::T);
                sm.send_signal_s(&h, $send);
                assert_eq!(sm.get_state(), State::T);
                assert_eq!(h.g.history().items(), vec![(Signal::s, value.clone())]);
                assert_eq!(h.a.items(), vec![(Signal::s, value.clone())]);
            }
        }
    };
}

basic_guard_typed!(BasicGuardU32, val, u32, u32, |p: &Pick| p.any(), v => *v, v => v);
basic_guard_typed!(
    BasicGuardTestAbsType, ref, TestAbsType, TestAbsType,
    |p: &Pick| p.test_abs_type(), v => v, v => *v
);
basic_guard_typed!(
    BasicGuardTestArray, ref, TestArray, TestArray,
    |p: &Pick| p.test_array(), v => v, v => *v
);
basic_guard_typed!(
    BasicGuardTestEnum, val, TestEnum, TestEnum,
    |p: &Pick| p.test_enum(), v => *v, v => v
);
basic_guard_typed!(
    BasicGuardTestStruct, ref, TestStruct, TestStruct,
    |p: &Pick| p.test_struct(), v => v, v => *v
);
// `constant basicGuardStringSize = 80`; `signal s: string size 80`, and
// the action and guard take any string.
basic_guard_typed!(
    BasicGuardString, str, fprime_fw::FwString<80>, String,
    |p: &Pick| p.string(80), v => v.as_str(), v => v.to_string()
);

// ---------------------------------------------------------------------------
// BasicInternal, BasicSelf
// ---------------------------------------------------------------------------

mod BasicInternal {
    super::fpp_state_machine! {
        /// A basic state machine with an internal transition
        state machine BasicInternal: u8 {
            actions { a => action_a }
            guards {}
            signals {
                s = 1 => send_signal_s {
                    S => do { a };
                }
            }
            initial enter S;
            states {
                state S = 1 {
                    entry do { a }
                }
            }
            choices {}
        }
    }
}

impl BasicInternal::Handlers for AHistory<BasicInternal::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: BasicInternal::Signal) {
        self.a.push(signal, ());
    }
}

#[test]
fn basic_internal_test() {
    use BasicInternal::{Signal, State, StateMachine};
    let h = AHistory::default();
    let sm = StateMachine::new();
    let id = Pick::new(4).state_machine_id();
    sm.init(id, &h);
    assert_eq!(sm.id(), id);
    assert_eq!(sm.get_state(), State::S);
    assert_eq!(h.a.signals(), vec![Signal::__FPRIME_INITIAL_TRANSITION]);
    h.a.clear();
    sm.send_signal_s(&h);
    assert_eq!(sm.get_state(), State::S);
    assert_eq!(h.a.signals(), vec![Signal::s]);
}

mod BasicSelf {
    super::fpp_state_machine! {
        /// A basic state machine with a self transition
        state machine BasicSelf: u8 {
            actions { a => action_a }
            guards {}
            signals {
                s = 1 => send_signal_s {
                    S => do { a, a, a } enter S;
                }
            }
            initial enter S;
            states {
                state S = 1 {
                    entry do { a }
                    exit do { a, a }
                }
            }
            choices {}
        }
    }
}

impl BasicSelf::Handlers for AHistory<BasicSelf::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: BasicSelf::Signal) {
        self.a.push(signal, ());
    }
}

#[test]
fn basic_self_test() {
    use BasicSelf::{Signal, State, StateMachine};
    let h = AHistory::default();
    let sm = StateMachine::new();
    sm.init(Pick::new(5).state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S);
    assert_eq!(h.a.signals(), vec![Signal::__FPRIME_INITIAL_TRANSITION]);
    h.a.clear();
    sm.send_signal_s(&h);
    assert_eq!(sm.get_state(), State::S);
    // exit (2) + transition (3) + entry (1)
    assert_eq!(h.a.signals(), vec![Signal::s; 6]);
}

// ---------------------------------------------------------------------------
// Basic<T>: BasicU32, BasicTestAbsType, BasicTestArray, BasicTestEnum,
// BasicTestStruct, BasicString
// ---------------------------------------------------------------------------

/// `state machine Basic<T> { action a; action b: T; signal s: T;
/// initial enter S; state S { exit do { a }; on s do { a, b } enter T };
/// state T { entry do { a, a, a } } }`
macro_rules! basic_typed {
    ($name:ident, $mode:ident, $ty:ty, $vty:ty, $pick:expr, $arg:ident => $send:expr, $own:ident => $owned:expr) => {
        mod $name {
            #[allow(unused_imports)]
            use super::*;
            fprime_comp::fpp_state_machine! {
                /// A basic state machine with a typed signal
                state machine $name: u8 {
                    actions {
                        a => action_a,
                        b => action_b ($mode $ty),
                    }
                    guards {}
                    signals {
                        s = 1 => send_signal_s ($mode $ty) {
                            S => do { a, b } enter T;
                        }
                    }
                    initial enter S;
                    states {
                        state S = 1 {
                            exit do { a }
                        }
                        state T = 2 {
                            entry do { a, a, a }
                        }
                    }
                    choices {}
                }
            }

            #[derive(Default)]
            struct Harness {
                a: History<Signal, ()>,
                b: History<Signal, $vty>,
            }

            impl Handlers for Harness {
                fn action_a(&self, _: FwEnumStoreType, signal: Signal) {
                    self.a.push(signal, ());
                }
                fn action_b(
                    &self,
                    _: FwEnumStoreType,
                    signal: Signal,
                    $own: fprime_comp::__fpp_sm_param_ty!($mode $ty),
                ) {
                    self.b.push(signal, $owned);
                }
            }

            #[test]
            fn test() {
                let pick = Pick::new(21);
                let h = Harness::default();
                let sm = StateMachine::new();
                let id = pick.state_machine_id();
                sm.init(id, &h);
                assert_eq!(sm.id(), id);
                assert_eq!(sm.get_state(), State::S);
                assert_eq!(h.a.size(), 0);
                let value: $vty = $pick(&pick);
                let $arg = &value;
                sm.send_signal_s(&h, $send);
                assert_eq!(sm.get_state(), State::T);
                sm.send_signal_s(&h, $send);
                assert_eq!(sm.get_state(), State::T);
                assert_eq!(h.a.signals(), vec![Signal::s; 5]);
                assert_eq!(h.b.items(), vec![(Signal::s, value.clone())]);
            }
        }
    };
}

basic_typed!(BasicU32, val, u32, u32, |p: &Pick| p.any(), v => *v, v => v);
basic_typed!(
    BasicTestAbsType, ref, TestAbsType, TestAbsType,
    |p: &Pick| p.test_abs_type(), v => v, v => *v
);
basic_typed!(
    BasicTestArray, ref, TestArray, TestArray,
    |p: &Pick| p.test_array(), v => v, v => *v
);
basic_typed!(
    BasicTestEnum, val, TestEnum, TestEnum,
    |p: &Pick| p.test_enum(), v => *v, v => v
);
basic_typed!(
    BasicTestStruct, ref, TestStruct, TestStruct,
    |p: &Pick| p.test_struct(), v => v, v => *v
);

/// `BasicString`: `action b: string`, `signal s: string size 200` and an
/// unused `signal s1: string size 100` (signals in name order: `s`, `s1`).
mod BasicString {
    use super::*;
    fprime_comp::fpp_state_machine! {
        /// A basic state machine with string signals
        state machine BasicString: u8 {
            actions {
                a => action_a,
                b => action_b (str fprime_fw::FwDefaultString),
            }
            guards {}
            signals {
                s = 1 => send_signal_s (str fprime_fw::FwString<200>) {
                    S => do { a, b } enter T;
                }
                s1 = 2 => send_signal_s1 (str fprime_fw::FwString<100>) {}
            }
            initial enter S;
            states {
                state S = 1 {
                    exit do { a }
                }
                state T = 2 {
                    entry do { a, a, a }
                }
            }
            choices {}
        }
    }

    #[derive(Default)]
    struct Harness {
        a: History<Signal, ()>,
        b: History<Signal, String>,
    }

    impl Handlers for Harness {
        fn action_a(&self, _: FwEnumStoreType, signal: Signal) {
            self.a.push(signal, ());
        }
        fn action_b(&self, _: FwEnumStoreType, signal: Signal, value: &str) {
            self.b.push(signal, value.to_string());
        }
    }

    #[test]
    fn test() {
        let pick = Pick::new(31);
        let h = Harness::default();
        let sm = StateMachine::new();
        sm.init(pick.state_machine_id(), &h);
        assert_eq!(sm.get_state(), State::S);
        let value = pick.string(256);
        sm.send_signal_s(&h, &value);
        assert_eq!(sm.get_state(), State::T);
        sm.send_signal_s(&h, &value);
        assert_eq!(sm.get_state(), State::T);
        assert_eq!(h.a.signals(), vec![Signal::s; 5]);
        assert_eq!(h.b.items(), vec![(Signal::s, value)]);
        // `s1` has no transitions anywhere: ignored.
        sm.send_signal_s1(&h, "x");
        assert_eq!(sm.get_state(), State::T);
        // The largest signal value is the 200-byte string.
        assert_eq!(StateMachine::MAX_SIGNAL_DATA_SIZE, 202);
    }
}

// ---------------------------------------------------------------------------
// Internal
// ---------------------------------------------------------------------------

mod Internal {
    super::fpp_state_machine! {
        /// A hierarchical state machine with an internal transition
        state machine Internal: u8 {
            actions { a => action_a }
            guards {}
            signals {
                S1_internal = 1 => send_signal_S1_internal {
                    S1 => do { a };
                }
                S2_to_S3 = 2 => send_signal_S2_to_S3 {
                    S1_S2 => enter S1_S3;
                }
            }
            initial enter S1;
            states {
                state S1 {
                    initial enter S1_S2;
                }
                state S1_S2 = 1 in S1 {}
                state S1_S3 = 2 in S1 {}
            }
            choices {}
        }
    }
}

impl Internal::Handlers for AHistory<Internal::Signal> {
    fn action_a(&self, _: FwEnumStoreType, signal: Internal::Signal) {
        self.a.push(signal, ());
    }
}

#[test]
fn internal_init() {
    use Internal::{State, StateMachine};
    let h = AHistory::default();
    let sm = StateMachine::new();
    let id = Pick::new(6).state_machine_id();
    sm.init(id, &h);
    assert_eq!(sm.id(), id);
    assert_eq!(sm.get_state(), State::S1_S2);
    assert_eq!(h.a.size(), 0);
}

#[test]
fn internal_s2_internal() {
    use Internal::{Signal, State, StateMachine};
    let h = AHistory::default();
    let sm = StateMachine::new();
    sm.init(Pick::new(7).state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1_S2);
    sm.send_signal_S1_internal(&h);
    assert_eq!(sm.get_state(), State::S1_S2);
    assert_eq!(h.a.signals(), vec![Signal::S1_internal]);
}

#[test]
fn internal_s2_to_s3() {
    use Internal::{State, StateMachine};
    let h = AHistory::default();
    let sm = StateMachine::new();
    sm.init(Pick::new(8).state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1_S2);
    sm.send_signal_S2_to_S3(&h);
    sm.send_signal_S2_to_S3(&h);
    assert_eq!(sm.get_state(), State::S1_S3);
    assert_eq!(h.a.size(), 0);
}

#[test]
fn internal_s3_internal() {
    use Internal::{Signal, State, StateMachine};
    let h = AHistory::default();
    let sm = StateMachine::new();
    sm.init(Pick::new(9).state_machine_id(), &h);
    sm.send_signal_S2_to_S3(&h);
    assert_eq!(sm.get_state(), State::S1_S3);
    sm.send_signal_S1_internal(&h);
    assert_eq!(sm.get_state(), State::S1_S3);
    assert_eq!(h.a.signals(), vec![Signal::S1_internal]);
}

// ---------------------------------------------------------------------------
// Polymorphism
// ---------------------------------------------------------------------------

mod Polymorphism {
    super::fpp_state_machine! {
        /// A hierarchical state machine with behavioral polymorphism
        state machine Polymorphism: u8 {
            actions {}
            guards {}
            signals {
                S2_to_S3 = 1 => send_signal_S2_to_S3 {
                    S1_S2 => enter S1_S3;
                }
                poly = 2 => send_signal_poly {
                    S1 => enter S4;
                    // This transition overrides the transition in S1.
                    S1_S3 => enter S5;
                }
            }
            initial enter S1;
            states {
                state S1 {
                    initial enter S1_S2;
                }
                state S1_S2 = 1 in S1 {}
                state S1_S3 = 2 in S1 {}
                state S4 = 3 {}
                state S5 = 4 {}
            }
            choices {}
        }
    }
}

struct NoHandlers;
impl Polymorphism::Handlers for NoHandlers {}

#[test]
fn polymorphism_init() {
    use Polymorphism::{State, StateMachine};
    let sm = StateMachine::new();
    let id = Pick::new(10).state_machine_id();
    sm.init(id, &NoHandlers);
    assert_eq!(sm.id(), id);
    assert_eq!(sm.get_state(), State::S1_S2);
}

#[test]
fn polymorphism_s2_poly() {
    use Polymorphism::{State, StateMachine};
    let sm = StateMachine::new();
    sm.init(0, &NoHandlers);
    assert_eq!(sm.get_state(), State::S1_S2);
    sm.send_signal_poly(&NoHandlers);
    sm.send_signal_poly(&NoHandlers);
    sm.send_signal_S2_to_S3(&NoHandlers);
    assert_eq!(sm.get_state(), State::S4);
}

#[test]
fn polymorphism_s2_to_s3() {
    use Polymorphism::{State, StateMachine};
    let sm = StateMachine::new();
    sm.init(0, &NoHandlers);
    sm.send_signal_S2_to_S3(&NoHandlers);
    sm.send_signal_S2_to_S3(&NoHandlers);
    assert_eq!(sm.get_state(), State::S1_S3);
}

#[test]
fn polymorphism_s3_poly() {
    use Polymorphism::{State, StateMachine};
    let sm = StateMachine::new();
    sm.init(0, &NoHandlers);
    sm.send_signal_S2_to_S3(&NoHandlers);
    assert_eq!(sm.get_state(), State::S1_S3);
    sm.send_signal_poly(&NoHandlers);
    sm.send_signal_poly(&NoHandlers);
    sm.send_signal_S2_to_S3(&NoHandlers);
    assert_eq!(sm.get_state(), State::S5);
}

// ---------------------------------------------------------------------------
// Action-id histories for the hierarchy tests
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ActionId {
    ExitS1,
    ExitS2,
    ExitS3,
    A,
    EnterS1,
    EnterS2,
    EnterS3,
    EnterS4,
    EnterS5,
}

/// A `SignalValueHistory<Signal, ActionId>` plus an optional no-arg guard.
struct ActionHistory<S: Copy> {
    actions: History<S, ActionId>,
    g: Guard<S, ()>,
}

impl<S: Copy> Default for ActionHistory<S> {
    fn default() -> Self {
        Self {
            actions: History::new(),
            g: Guard::new(),
        }
    }
}

impl<S: Copy + PartialEq + std::fmt::Debug> ActionHistory<S> {
    /// Every recorded action was triggered by `signal`, and the actions are
    /// `expected`, in order.
    fn check(&self, signal: S, expected: &[ActionId]) {
        assert_eq!(self.actions.signals(), vec![signal; expected.len()]);
        assert_eq!(self.actions.values(), expected);
    }
}

/// Implement a machine's `Handlers` for [`ActionHistory`] from a list of
/// `method => ActionId` pairs (and `guard_g` if the machine has it).
macro_rules! action_history_handlers {
    ($m:ident { $($f:ident => $id:ident),* } $(guard $g:ident)?) => {
        impl $m::Handlers for ActionHistory<$m::Signal> {
            $(
                fn $f(&self, _: FwEnumStoreType, signal: $m::Signal) {
                    self.actions.push(signal, ActionId::$id);
                }
            )*
            $(
                fn $g(&self, _: FwEnumStoreType, signal: $m::Signal) -> bool {
                    self.g.call(signal, ())
                }
            )?
        }
    };
}

// ---------------------------------------------------------------------------
// StateToChild
// ---------------------------------------------------------------------------

mod StateToChild {
    super::fpp_state_machine! {
        /// A state machine for testing state-to-child transitions
        state machine StateToChild: u8 {
            actions {
                exitS2 => action_exitS2,
                exitS3 => action_exitS3,
                a => action_a,
                enterS2 => action_enterS2,
                enterS3 => action_enterS3,
            }
            guards {}
            signals {
                S1_to_S2 = 1 => send_signal_S1_to_S2 {
                    S1 => do { a } enter S1_S2;
                }
                S2_to_S3 = 2 => send_signal_S2_to_S3 {
                    S1_S2 => enter S1_S3;
                }
            }
            initial enter S1;
            states {
                state S1 {
                    initial enter S1_S2;
                }
                state S1_S2 = 1 in S1 {
                    entry do { enterS2 }
                    exit do { exitS2 }
                }
                state S1_S3 = 2 in S1 {
                    entry do { enterS3 }
                    exit do { exitS3 }
                }
            }
            choices {}
        }
    }
}

action_history_handlers!(StateToChild {
    action_exitS2 => ExitS2,
    action_exitS3 => ExitS3,
    action_a => A,
    action_enterS2 => EnterS2,
    action_enterS3 => EnterS3
});

#[test]
fn state_to_child_init() {
    use StateToChild::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    let id = Pick::new(13).state_machine_id();
    sm.init(id, &h);
    assert_eq!(sm.id(), id);
    assert_eq!(sm.get_state(), State::S1_S2);
    h.check(Signal::__FPRIME_INITIAL_TRANSITION, &[ActionId::EnterS2]);
}

#[test]
fn state_to_child_s2_to_s2() {
    use StateToChild::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    h.actions.clear();
    sm.send_signal_S1_to_S2(&h);
    assert_eq!(sm.get_state(), State::S1_S2);
    h.check(
        Signal::S1_to_S2,
        &[ActionId::ExitS2, ActionId::A, ActionId::EnterS2],
    );
}

#[test]
fn state_to_child_s2_to_s3() {
    use StateToChild::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    h.actions.clear();
    sm.send_signal_S2_to_S3(&h);
    sm.send_signal_S2_to_S3(&h);
    assert_eq!(sm.get_state(), State::S1_S3);
    h.check(Signal::S2_to_S3, &[ActionId::ExitS2, ActionId::EnterS3]);
}

#[test]
fn state_to_child_s3_to_s2() {
    use StateToChild::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    sm.send_signal_S2_to_S3(&h);
    h.actions.clear();
    assert_eq!(sm.get_state(), State::S1_S3);
    sm.send_signal_S1_to_S2(&h);
    assert_eq!(sm.get_state(), State::S1_S2);
    h.check(
        Signal::S1_to_S2,
        &[ActionId::ExitS3, ActionId::A, ActionId::EnterS2],
    );
}

// ---------------------------------------------------------------------------
// StateToChoice
// ---------------------------------------------------------------------------

mod StateToChoice {
    super::fpp_state_machine! {
        /// A state machine for testing state-to-choice transitions with
        /// hierarchy
        state machine StateToChoice: u8 {
            actions {
                exitS1 => action_exitS1,
                exitS2 => action_exitS2,
                exitS3 => action_exitS3,
                a => action_a,
                enterS1 => action_enterS1,
                enterS2 => action_enterS2,
                enterS3 => action_enterS3,
                enterS4 => action_enterS4,
            }
            guards { g => guard_g }
            signals {
                S1_to_C = 1 => send_signal_S1_to_C {
                    S1 => do { a } enter S4_C;
                }
                S1_to_S4 = 2 => send_signal_S1_to_S4 {
                    S1 => do { a } enter S4;
                }
                S2_to_S3 = 3 => send_signal_S2_to_S3 {
                    S1_S2 => enter S1_S3;
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
                state S1_S3 = 2 in S1 {
                    entry do { enterS3 }
                    exit do { exitS3 }
                }
                state S4 {
                    initial enter S4_C;
                    entry do { enterS4 }
                }
                state S4_S5 = 3 in S4 {}
                state S4_S6 = 4 in S4 {}
            }
            choices {
                choice S4_C in S4 { if g enter S4_S5 else enter S4_S6 }
            }
        }
    }
}

action_history_handlers!(StateToChoice {
    action_exitS1 => ExitS1,
    action_exitS2 => ExitS2,
    action_exitS3 => ExitS3,
    action_a => A,
    action_enterS1 => EnterS1,
    action_enterS2 => EnterS2,
    action_enterS3 => EnterS3,
    action_enterS4 => EnterS4
} guard guard_g);

#[test]
fn state_to_choice_init() {
    use StateToChoice::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    let id = Pick::new(14).state_machine_id();
    sm.init(id, &h);
    assert_eq!(sm.id(), id);
    assert_eq!(sm.get_state(), State::S1_S2);
    h.check(
        Signal::__FPRIME_INITIAL_TRANSITION,
        &[ActionId::EnterS1, ActionId::EnterS2],
    );
}

#[test]
fn state_to_choice_s2_to_c() {
    use StateToChoice::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    h.g.set_return_value(true);
    let sm = StateMachine::new();
    sm.init(0, &h);
    h.actions.clear();
    sm.send_signal_S1_to_C(&h);
    sm.send_signal_S1_to_C(&h);
    sm.send_signal_S2_to_S3(&h);
    assert_eq!(sm.get_state(), State::S4_S5);
    h.check(
        Signal::S1_to_C,
        &[
            ActionId::ExitS2,
            ActionId::ExitS1,
            ActionId::A,
            ActionId::EnterS4,
        ],
    );
}

#[test]
fn state_to_choice_s2_to_s3() {
    use StateToChoice::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    h.actions.clear();
    sm.send_signal_S2_to_S3(&h);
    sm.send_signal_S2_to_S3(&h);
    assert_eq!(sm.get_state(), State::S1_S3);
    h.check(Signal::S2_to_S3, &[ActionId::ExitS2, ActionId::EnterS3]);
}

#[test]
fn state_to_choice_s2_to_s4() {
    use StateToChoice::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    h.g.set_return_value(true);
    let sm = StateMachine::new();
    sm.init(0, &h);
    h.actions.clear();
    sm.send_signal_S1_to_S4(&h);
    sm.send_signal_S1_to_S4(&h);
    assert_eq!(sm.get_state(), State::S4_S5);
    h.check(
        Signal::S1_to_S4,
        &[
            ActionId::ExitS2,
            ActionId::ExitS1,
            ActionId::A,
            ActionId::EnterS4,
        ],
    );
}

#[test]
fn state_to_choice_s3_to_c() {
    use StateToChoice::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    sm.send_signal_S2_to_S3(&h);
    h.actions.clear();
    sm.send_signal_S1_to_C(&h);
    sm.send_signal_S1_to_C(&h);
    sm.send_signal_S2_to_S3(&h);
    assert_eq!(sm.get_state(), State::S4_S6);
    h.check(
        Signal::S1_to_C,
        &[
            ActionId::ExitS3,
            ActionId::ExitS1,
            ActionId::A,
            ActionId::EnterS4,
        ],
    );
}

#[test]
fn state_to_choice_s3_to_s4() {
    use StateToChoice::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    sm.send_signal_S2_to_S3(&h);
    h.actions.clear();
    sm.send_signal_S1_to_S4(&h);
    sm.send_signal_S1_to_S4(&h);
    assert_eq!(sm.get_state(), State::S4_S6);
    h.check(
        Signal::S1_to_S4,
        &[
            ActionId::ExitS3,
            ActionId::ExitS1,
            ActionId::A,
            ActionId::EnterS4,
        ],
    );
}

// ---------------------------------------------------------------------------
// StateToSelf
// ---------------------------------------------------------------------------

mod StateToSelf {
    super::fpp_state_machine! {
        /// A state machine for testing state-to-self transitions with
        /// hierarchy
        state machine StateToSelf: u8 {
            actions {
                exitS1 => action_exitS1,
                exitS2 => action_exitS2,
                exitS3 => action_exitS3,
                a => action_a,
                enterS1 => action_enterS1,
                enterS2 => action_enterS2,
                enterS3 => action_enterS3,
            }
            guards {}
            signals {
                S1_to_S1 = 1 => send_signal_S1_to_S1 {
                    S1 => do { a } enter S1;
                }
                S2_to_S3 = 2 => send_signal_S2_to_S3 {
                    S1_S2 => enter S1_S3;
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
                state S1_S3 = 2 in S1 {
                    entry do { enterS3 }
                    exit do { exitS3 }
                }
            }
            choices {}
        }
    }
}

action_history_handlers!(StateToSelf {
    action_exitS1 => ExitS1,
    action_exitS2 => ExitS2,
    action_exitS3 => ExitS3,
    action_a => A,
    action_enterS1 => EnterS1,
    action_enterS2 => EnterS2,
    action_enterS3 => EnterS3
});

#[test]
fn state_to_self_init() {
    use StateToSelf::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(Pick::new(15).state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1_S2);
    h.check(
        Signal::__FPRIME_INITIAL_TRANSITION,
        &[ActionId::EnterS1, ActionId::EnterS2],
    );
}

#[test]
fn state_to_self_s2_to_s1() {
    use StateToSelf::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    h.actions.clear();
    sm.send_signal_S1_to_S1(&h);
    assert_eq!(sm.get_state(), State::S1_S2);
    h.check(
        Signal::S1_to_S1,
        &[
            ActionId::ExitS2,
            ActionId::ExitS1,
            ActionId::A,
            ActionId::EnterS1,
            ActionId::EnterS2,
        ],
    );
}

#[test]
fn state_to_self_s2_to_s3() {
    use StateToSelf::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    h.actions.clear();
    sm.send_signal_S2_to_S3(&h);
    sm.send_signal_S2_to_S3(&h);
    assert_eq!(sm.get_state(), State::S1_S3);
    h.check(Signal::S2_to_S3, &[ActionId::ExitS2, ActionId::EnterS3]);
}

#[test]
fn state_to_self_s3_to_s1() {
    use StateToSelf::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    sm.send_signal_S2_to_S3(&h);
    h.actions.clear();
    sm.send_signal_S1_to_S1(&h);
    assert_eq!(sm.get_state(), State::S1_S2);
    h.check(
        Signal::S1_to_S1,
        &[
            ActionId::ExitS3,
            ActionId::ExitS1,
            ActionId::A,
            ActionId::EnterS1,
            ActionId::EnterS2,
        ],
    );
}

// ---------------------------------------------------------------------------
// StateToState
// ---------------------------------------------------------------------------

mod StateToState {
    super::fpp_state_machine! {
        /// A state machine for testing state-to-state transitions with
        /// hierarchy
        state machine StateToState: u8 {
            actions {
                exitS1 => action_exitS1,
                exitS2 => action_exitS2,
                exitS3 => action_exitS3,
                a => action_a,
                enterS1 => action_enterS1,
                enterS2 => action_enterS2,
                enterS3 => action_enterS3,
                enterS4 => action_enterS4,
                enterS5 => action_enterS5,
            }
            guards {}
            signals {
                S1_to_S4 = 1 => send_signal_S1_to_S4 {
                    S1 => do { a } enter S4;
                }
                S1_to_S5 = 2 => send_signal_S1_to_S5 {
                    S1 => do { a } enter S4_S5;
                }
                S2_to_S3 = 3 => send_signal_S2_to_S3 {
                    S1_S2 => enter S1_S3;
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
                state S1_S3 = 2 in S1 {
                    entry do { enterS3 }
                    exit do { exitS3 }
                }
                state S4 {
                    initial enter S4_S5;
                    entry do { enterS4 }
                }
                state S4_S5 = 3 in S4 {
                    entry do { enterS5 }
                }
            }
            choices {}
        }
    }
}

action_history_handlers!(StateToState {
    action_exitS1 => ExitS1,
    action_exitS2 => ExitS2,
    action_exitS3 => ExitS3,
    action_a => A,
    action_enterS1 => EnterS1,
    action_enterS2 => EnterS2,
    action_enterS3 => EnterS3,
    action_enterS4 => EnterS4,
    action_enterS5 => EnterS5
});

const S1_TO_S4_FROM_S2: [ActionId; 5] = [
    ActionId::ExitS2,
    ActionId::ExitS1,
    ActionId::A,
    ActionId::EnterS4,
    ActionId::EnterS5,
];
const S1_TO_S4_FROM_S3: [ActionId; 5] = [
    ActionId::ExitS3,
    ActionId::ExitS1,
    ActionId::A,
    ActionId::EnterS4,
    ActionId::EnterS5,
];

#[test]
fn state_to_state_init() {
    use StateToState::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(Pick::new(16).state_machine_id(), &h);
    assert_eq!(sm.get_state(), State::S1_S2);
    h.check(
        Signal::__FPRIME_INITIAL_TRANSITION,
        &[ActionId::EnterS1, ActionId::EnterS2],
    );
}

#[test]
fn state_to_state_s2_to_s3() {
    use StateToState::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    h.actions.clear();
    sm.send_signal_S2_to_S3(&h);
    sm.send_signal_S2_to_S3(&h);
    assert_eq!(sm.get_state(), State::S1_S3);
    h.check(Signal::S2_to_S3, &[ActionId::ExitS2, ActionId::EnterS3]);
}

#[test]
fn state_to_state_s2_to_s4() {
    use StateToState::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    h.actions.clear();
    sm.send_signal_S1_to_S4(&h);
    sm.send_signal_S1_to_S4(&h);
    assert_eq!(sm.get_state(), State::S4_S5);
    h.check(Signal::S1_to_S4, &S1_TO_S4_FROM_S2);
}

#[test]
fn state_to_state_s2_to_s5() {
    use StateToState::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    h.actions.clear();
    sm.send_signal_S1_to_S5(&h);
    sm.send_signal_S1_to_S5(&h);
    assert_eq!(sm.get_state(), State::S4_S5);
    h.check(Signal::S1_to_S5, &S1_TO_S4_FROM_S2);
}

#[test]
fn state_to_state_s3_to_s4() {
    use StateToState::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    sm.send_signal_S2_to_S3(&h);
    h.actions.clear();
    sm.send_signal_S1_to_S4(&h);
    sm.send_signal_S2_to_S3(&h);
    assert_eq!(sm.get_state(), State::S4_S5);
    h.check(Signal::S1_to_S4, &S1_TO_S4_FROM_S3);
}

#[test]
fn state_to_state_s3_to_s5() {
    use StateToState::{Signal, State, StateMachine};
    let h = ActionHistory::default();
    let sm = StateMachine::new();
    sm.init(0, &h);
    sm.send_signal_S2_to_S3(&h);
    h.actions.clear();
    sm.send_signal_S1_to_S5(&h);
    assert_eq!(sm.get_state(), State::S4_S5);
    h.check(Signal::S1_to_S5, &S1_TO_S4_FROM_S3);
}
