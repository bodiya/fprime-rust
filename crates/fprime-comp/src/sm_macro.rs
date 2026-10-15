//! # `fpp_state_machine!` — FPP internal state machines as a declarative macro
//!
//! The Rust counterpart of what `fpp-to-cpp` generates for an FPP `state
//! machine` definition (`<M>StateMachineAc.{hpp,cpp}` and the implicit
//! `<M>_State` enum), built on the runtime in [`sm`](crate::sm). See
//! [`fpp_state_machine!`] for the input language and the generated
//! surface, and `docs/api-notes.md` ("fprime-comp: state machines") for
//! the contract a code generator targets.

/// Declare an FPP internal state machine.
///
/// Invoke it inside the module that stands for the state machine (the
/// FPP back end puts a state machine's nested definitions in a module of
/// the machine's name); it defines these items there:
///
/// | item | C++ counterpart |
/// |------|-----------------|
/// | `State` (an [`fpp_enum!`](fprime_fw::fpp_enum)) | `<M>_State`: `__FPRIME_UNINITIALIZED = 0` then the leaf states |
/// | `Signal` | `<M>StateMachineBase::Signal`: `__FPRIME_INITIAL_TRANSITION = 0` then the signals |
/// | `Handlers` (trait) | the pure-virtual `action_<a>` / `guard_<g>` functions |
/// | `StateMachine` | `<M>StateMachineBase`: `init` (`initBase`), `get_state`, `id`, one `send_signal_<s>` per signal, plus `dispatch_signal` (the component's `<M>_smDispatch`) |
/// | `Sender` | the component's `<m>_sendSignal_<s>` functions for one instance |
///
/// and hidden helpers (`__SmNode`, `__SmEnter`, `__SmEntry`, `__SmExit`,
/// `__SmAction`, `__SmGuard`), so use one module per state machine.
///
/// ## Input
///
/// ```
/// # #![allow(non_snake_case)]
/// pub mod StateToChoice {
///     fprime_comp::fpp_state_machine! {
///         /// A state machine for testing state-to-choice transitions.
///         state machine StateToChoice: u8 {
///             actions {
///                 exitS1 => action_exitS1,
///                 exitS2 => action_exitS2,
///                 a => action_a,
///                 enterS1 => action_enterS1,
///                 enterS2 => action_enterS2,
///                 enterS4 => action_enterS4,
///             }
///             guards { g => guard_g }
///             signals {
///                 S1_to_C = 1 => send_signal_S1_to_C {
///                     S1 => do { a } enter S4_C;
///                 }
///                 S1_to_S4 = 2 => send_signal_S1_to_S4 {
///                     S1 => do { a } enter S4;
///                 }
///                 S2_to_S3 = 3 => send_signal_S2_to_S3 {
///                     S1_S2 => enter S1_S3;
///                 }
///             }
///             initial enter S1;
///             states {
///                 state S1 {
///                     initial enter S1_S2;
///                     entry do { enterS1 }
///                     exit do { exitS1 }
///                 }
///                 state S1_S2 = 1 in S1 {
///                     entry do { enterS2 }
///                     exit do { exitS2 }
///                 }
///                 state S1_S3 = 2 in S1 {}
///                 state S4 {
///                     initial enter S4_C;
///                     entry do { enterS4 }
///                 }
///                 state S4_S5 = 3 in S4 {}
///                 state S4_S6 = 4 in S4 {}
///             }
///             choices {
///                 choice S4_C in S4 { if g enter S4_S5 else enter S4_S6 }
///             }
///         }
///     }
/// }
///
/// use fprime_comp::config::FwEnumStoreType;
/// use std::cell::RefCell;
/// use StateToChoice::{Handlers, Signal, State, StateMachine};
///
/// #[derive(Default)]
/// struct Recorder(RefCell<Vec<&'static str>>);
/// impl Handlers for Recorder {
///     fn action_exitS1(&self, _: FwEnumStoreType, _: Signal) { self.0.borrow_mut().push("exitS1") }
///     fn action_exitS2(&self, _: FwEnumStoreType, _: Signal) { self.0.borrow_mut().push("exitS2") }
///     fn action_a(&self, _: FwEnumStoreType, _: Signal) { self.0.borrow_mut().push("a") }
///     fn action_enterS1(&self, _: FwEnumStoreType, _: Signal) { self.0.borrow_mut().push("enterS1") }
///     fn action_enterS2(&self, _: FwEnumStoreType, _: Signal) { self.0.borrow_mut().push("enterS2") }
///     fn action_enterS4(&self, _: FwEnumStoreType, _: Signal) { self.0.borrow_mut().push("enterS4") }
///     fn guard_g(&self, _: FwEnumStoreType, _: Signal) -> bool { true }
/// }
///
/// let rec = Recorder::default();
/// let sm = StateMachine::new();
/// sm.init(7, &rec);
/// assert_eq!(sm.get_state(), State::S1_S2);
/// rec.0.borrow_mut().clear();
/// sm.send_signal_S1_to_C(&rec);
/// assert_eq!(sm.get_state(), State::S4_S5);
/// assert_eq!(*rec.0.borrow(), ["exitS2", "exitS1", "a", "enterS4"]);
/// ```
///
/// The language is FPP's, flattened so that `macro_rules!` can read it:
///
/// - **Every name is explicit.** `macro_rules!` cannot build identifiers,
///   so the inputs spell out what the C++ autocoder derives: the
///   `action_<a>` / `guard_<g>` / `send_signal_<s>` method names after
///   `=>`, and each state and choice under its **qualified name with `.`
///   replaced by `_`** (`S1.S2` is `S1_S2`, as in the C++ `State` enum and
///   `enter_S1_S2`).
/// - **Discriminants are explicit**, as everywhere in this workspace:
///   `fpp-to-cpp` sorts both enums by name, so signals are numbered from
///   1 in **name order** and leaf states from 1 in the **order of their
///   `_`-joined names** (`A_A` before `A_B`, whatever the definition
///   order). The representation type after the machine name is the
///   `State` enum's: the smallest unsigned type holding the leaf count
///   plus one (`u8` in practice). The order in which states, choices and
///   signals are *listed* does not matter.
/// - **Types carry a passing mode**, as in the port macros: `(val T)`
///   passes a `Copy` value by value, `(ref T)` by reference, and
///   `(str FwString<N>)` is an FPP string, passed as `&str` (C++
///   `const Fw::StringBase&`; any string converts, as in FPP — `N` is the
///   declared size, used for a signal's queue storage and truncation and
///   ignored elsewhere). Typed
///   actions, guards, signals and choices use it; the same type rules as
///   FPP apply and are checked by the compiler (a typed action in an
///   untyped transition, or a value whose type does not convert, does not
///   compile). A `val` value converts with `Into`, so a `U16` signal can
///   enter a `U32` choice as in FPP.
/// - **States are listed flat**, each with its parent (`in S1`; top-level
///   states have none). A *leaf* state has a discriminant (`= 1`) and no
///   initial transition; a *composite* state has an initial transition
///   and no discriminant (anything else does not compile). A state's body
///   is, in this order, `initial [do { .. }] enter T;`, `entry do { .. }`,
///   `exit do { .. }`, each optional as FPP allows.
/// - **Transitions are listed by signal**, then source state — the shape
///   of the generated `sendSignal_<s>` functions: `S => [if g] [do { .. }]
///   [enter T];` (no `enter` makes an internal transition). A state's
///   transition overrides its ancestors' for the same signal, and a false
///   guard means no transition (there is no fall-through to the parent),
///   exactly as in C++.
/// - **Choices** are listed flat with their parent and optional type:
///   `choice C in S (val U32) { if g [do { .. }] enter T1 else [do { .. }]
///   enter T2 }`.
///
/// Every section is required (write `choices {}` for none).
///
/// ## Semantics
///
/// `init` sets the id and runs the initial transition with signal
/// `__FPRIME_INITIAL_TRANSITION`. A signal delivered in a state with no
/// transition for it (after searching the ancestors) is ignored; a signal
/// before `init` asserts. A transition from source `S` to target `T` runs
/// the exit actions from the current leaf up to the least common ancestor
/// (see [`sm::lca`](crate::sm::lca)), then its actions, then the entry
/// actions of the states between, then enters `T`: a state runs its entry
/// actions, then its initial transition or (a leaf) becomes the current
/// state; a choice evaluates its guard and takes one branch. Each action
/// and guard receives the state machine id and the triggering signal, and
/// a typed one the value. The order of calls is the C++ order, action for
/// action — the ported FppTest `state_machine` tests check it.
///
/// Deliberate deviations: `Handlers` methods take `&self` (components keep
/// state behind their own locks; a test harness uses a `RefCell`) and
/// receive the id as `FwEnumStoreType` (the C++ component passes its
/// `SmId`); and a signal delivered while a transition of the same instance
/// is running asserts (see [`SmCore`](crate::sm::SmCore)).
#[macro_export]
macro_rules! fpp_state_machine {
    (
        $(#[$meta:meta])*
        state machine $machine:ident : $repr:ty {
            actions {
                $( $(#[$am:meta])* $action:ident => $afn:ident $( ( $amode:ident $aty:ty ) )? ),* $(,)?
            }
            guards {
                $( $(#[$gm:meta])* $guard:ident => $gfn:ident $( ( $gmode:ident $gty:ty ) )? ),* $(,)?
            }
            signals {
                $(
                    $(#[$sm:meta])*
                    $signal:ident = $sdisc:literal => $sfn:ident $( ( $smode:ident $sty:ty ) )?
                    { $($strans:tt)* }
                )*
            }
            initial $( do { $($ia:ident),* $(,)? } )? enter $itarget:ident ;
            states {
                $(
                    $(#[$stm:meta])*
                    state $state:ident $( = $disc:literal )? $( in $parent:ident )?
                    { $($sbody:tt)* }
                )*
            }
            choices {
                $(
                    $(#[$cm:meta])*
                    choice $choice:ident $( in $cparent:ident )? $( ( $cmode:ident $cty:ty ) )?
                    { $($cbody:tt)* }
                )*
            }
        }
    ) => {
        $crate::fw::fpp_enum! {
            #[doc = concat!(
                "The state enum of state machine `", stringify!($machine),
                "`: the leaf states plus the uninitialized state (C++ `",
                stringify!($machine), "_State`)."
            )]
            #[allow(non_camel_case_types)]
            pub enum State : $repr {
                /// The state before `init`.
                __FPRIME_UNINITIALIZED = 0,
                $($(
                    #[doc = concat!("Leaf state `", stringify!($state), "` (", stringify!($disc), ").")]
                    $state = $disc,
                )?)*
            }
            default __FPRIME_UNINITIALIZED
        }

        #[doc = concat!(
            "The signals of state machine `", stringify!($machine),
            "` (C++ `Signal`, an `FwEnumStoreType` on the wire)."
        )]
        #[allow(non_camel_case_types)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub enum Signal {
            /// The initial transition.
            __FPRIME_INITIAL_TRANSITION = 0,
            $( $(#[$sm])* $signal = $sdisc, )*
        }

        impl Signal {
            /// The signal's wire value.
            pub const fn as_repr(self) -> $crate::config::FwEnumStoreType {
                self as $crate::config::FwEnumStoreType
            }
        }

        impl ::core::convert::TryFrom<$crate::config::FwEnumStoreType> for Signal {
            type Error = $crate::config::FwEnumStoreType;
            /// Map a wire value to the signal; `Err` carries an unknown value.
            fn try_from(v: $crate::config::FwEnumStoreType) -> ::core::result::Result<Self, Self::Error> {
                match v {
                    0 => Ok(Self::__FPRIME_INITIAL_TRANSITION),
                    $( $sdisc => Ok(Self::$signal), )*
                    other => Err(other),
                }
            }
        }

        impl ::core::convert::From<Signal> for $crate::config::FwEnumStoreType {
            fn from(s: Signal) -> Self {
                s.as_repr()
            }
        }

        #[doc = concat!(
            "The actions and guards of state machine `", stringify!($machine),
            "`, implemented by its owner (C++: the pure virtual `action_*` / \
             `guard_*` functions). Each receives the state machine id and the \
             signal, and a typed one the value."
        )]
        #[allow(non_snake_case)]
        pub trait Handlers {
            $(
                $(#[$am])*
                fn $afn(
                    &self,
                    sm_id: $crate::config::FwEnumStoreType,
                    signal: Signal
                    $(, value: $crate::__fpp_sm_param_ty!($amode $aty))?
                );
            )*
            $(
                $(#[$gm])*
                fn $gfn(
                    &self,
                    sm_id: $crate::config::FwEnumStoreType,
                    signal: Signal
                    $(, value: $crate::__fpp_sm_param_ty!($gmode $gty))?
                ) -> bool;
            )*
        }

        $(#[$meta])*
        #[doc = ""]
        #[doc = concat!(
            "An instance of state machine `", stringify!($machine),
            "` (C++ `", stringify!($machine), "StateMachineBase`)."
        )]
        #[derive(Debug, Default)]
        pub struct StateMachine {
            core: $crate::sm::SmCore,
        }

        #[allow(dead_code, non_snake_case)]
        impl StateMachine {
            /// Largest serialized signal value (0 if no signal carries data).
            pub const MAX_SIGNAL_DATA_SIZE: usize = $crate::sm::max_size(&[
                0 $(, $crate::__fpp_sm_sized!([$($smode $sty)?]))*
            ]);

            /// Queue message size needed to send any signal of this machine
            /// through a component queue: envelope header, state machine id,
            /// signal, largest value.
            pub const SIGNAL_MSG_SIZE: usize = $crate::msg::ENVELOPE_HEADER_SIZE
                + $crate::sm::SIGNAL_HEADER_SIZE
                + Self::MAX_SIGNAL_DATA_SIZE;

            /// An uninitialized instance (state `__FPRIME_UNINITIALIZED`).
            pub const fn new() -> Self {
                Self {
                    core: $crate::sm::SmCore::new(),
                }
            }

            /// Initialize: record the state machine id and run the initial
            /// transition with signal `__FPRIME_INITIAL_TRANSITION` (C++
            /// `initBase`).
            pub fn init<H: Handlers + ?Sized>(
                &self,
                sm_id: $crate::config::FwEnumStoreType,
                handlers: &H,
            ) {
                let _run = self.core.begin_run();
                self.core.set_id(sm_id);
                let signal = Signal::__FPRIME_INITIAL_TRANSITION;
                $($( __SmAction::$ia(handlers, sm_id, signal, &()); )*)?
                $crate::sm::for_each_entry(None, __SmNode::$itarget, |n| {
                    n.__entry(handlers, sm_id, signal)
                });
                __SmEnter::$itarget(self, handlers, signal, &());
            }

            /// The current state (C++ `getState`).
            pub fn get_state(&self) -> State {
                let raw = self.core.raw_state();
                match <State as ::core::convert::TryFrom<$repr>>::try_from(raw as $repr) {
                    Ok(state) => state,
                    Err(_) => {
                        // Only ever written from a `State`.
                        $crate::fw::fw_assert!(false, raw as i32);
                        State::__FPRIME_UNINITIALIZED
                    }
                }
            }

            /// The state machine id given to `init` (C++ `m_id`).
            pub fn id(&self) -> $crate::config::FwEnumStoreType {
                self.core.id()
            }

            $(
                $crate::__fpp_sm_signal_fn! {
                    $(#[$sm])*
                    $signal => $sfn [$($smode $sty)?] { $($strans)* }
                }
            )*

            /// Deliver a signal read from a component queue message (C++
            /// `<M>_smDispatch`): deserialize the value, if the signal
            /// carries one, check that nothing is left over, and call the
            /// signal's `send_signal_*` function. The cursor of `msg` must
            /// be at the signal data (after
            /// [`read_signal_header`]($crate::sm::read_signal_header)). An
            /// unknown signal, a decode failure or leftover bytes assert, as
            /// in C++.
            pub fn dispatch_signal<H: Handlers + ?Sized>(
                &self,
                handlers: &H,
                signal: $crate::config::FwEnumStoreType,
                msg: &mut dyn $crate::fw::SerBufAny,
            ) {
                let sm = self;
                match <Signal as ::core::convert::TryFrom<$crate::config::FwEnumStoreType>>::try_from(signal) {
                    $(
                        Ok(Signal::$signal) => $crate::__fpp_sm_dispatch_one!(
                            sm, handlers, msg, $sfn [$($smode $sty)?]
                        ),
                    )*
                    _ => $crate::fw::fw_assert!(false, signal),
                }
            }

            fn __set_state(&self, state: State) {
                self.core.set_raw_state(state.as_repr() as u32);
            }

            /// The node of the current leaf state; asserts before `init`.
            fn __leaf(&self) -> ::core::option::Option<__SmNode> {
                match self.get_state() {
                    State::__FPRIME_UNINITIALIZED => {
                        // C++: the sendSignal switch has no arm for the
                        // uninitialized state and asserts in `default`.
                        $crate::fw::fw_assert!(false, 0);
                        None
                    }
                    $($(
                        State::$state => {
                            $crate::__fpp_sm_ignore!($disc);
                            Some(__SmNode::$state)
                        }
                    )?)*
                }
            }
        }

        #[doc = concat!(
            "The signal send functions of one instance of state machine `",
            stringify!($machine), "` inside a component (C++ `<m>_sendSignal_<s>`): ",
            "each enqueues a signal message with the instance's priority and \
             queue-full policy. `N` is the component's queue message size."
        )]
        pub struct Sender<'a, const N: usize>(pub $crate::sm::SmSender<'a, N>);

        #[allow(dead_code, non_snake_case)]
        impl<const N: usize> Sender<'_, N> {
            $(
                $crate::__fpp_sm_sender_fn! {
                    $(#[$sm])*
                    $signal => $sfn [$($smode $sty)?]
                }
            )*
        }

        /// The states and choices of the machine (hidden; the hierarchy the
        /// runtime walks).
        #[doc(hidden)]
        #[allow(non_camel_case_types)]
        #[derive(Clone, Copy, PartialEq, Eq, Debug)]
        pub enum __SmNode {
            $( $state, )*
            $( $choice, )*
        }

        impl $crate::sm::SmNode for __SmNode {
            fn parent(self) -> ::core::option::Option<Self> {
                match self {
                    $( Self::$state => $crate::__fpp_sm_opt!($(Self::$parent)?), )*
                    $( Self::$choice => $crate::__fpp_sm_opt!($(Self::$cparent)?), )*
                }
            }
        }

        #[allow(dead_code)]
        impl __SmNode {
            fn __entry<H: Handlers + ?Sized>(
                self,
                h: &H,
                sm_id: $crate::config::FwEnumStoreType,
                signal: Signal,
            ) {
                match self {
                    $( Self::$state => __SmEntry::$state(h, sm_id, signal), )*
                    $( Self::$choice => {} )*
                }
            }

            fn __exit<H: Handlers + ?Sized>(
                self,
                h: &H,
                sm_id: $crate::config::FwEnumStoreType,
                signal: Signal,
            ) {
                match self {
                    $( Self::$state => __SmExit::$state(h, sm_id, signal), )*
                    $( Self::$choice => {} )*
                }
            }
        }

        #[doc(hidden)]
        pub struct __SmAction;

        #[allow(dead_code, non_snake_case)]
        impl __SmAction {
            $( $crate::__fpp_sm_action_helper!($action => $afn [$($amode $aty)?]); )*
        }

        #[doc(hidden)]
        pub struct __SmGuard;

        #[allow(dead_code, non_snake_case)]
        impl __SmGuard {
            $( $crate::__fpp_sm_guard_helper!($guard => $gfn [$($gmode $gty)?]); )*
        }

        #[doc(hidden)]
        pub struct __SmEntry;
        #[doc(hidden)]
        pub struct __SmExit;
        #[doc(hidden)]
        pub struct __SmEnter;

        $(
            $crate::__fpp_sm_state! {
                $(#[$stm])*
                $state [$($disc)?] { $($sbody)* }
            }
        )*

        $(
            $crate::__fpp_sm_choice! {
                $(#[$cm])*
                $choice [$($cmode $cty)?] { $($cbody)* }
            }
        )*
    };
}

/// Parameter type for a typed action/guard/signal; internal to
/// [`fpp_state_machine!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_param_ty {
    (val $ty:ty) => {
        $ty
    };
    (ref $ty:ty) => {
        &$ty
    };
    (str $ty:ty) => {
        &str
    };
}

/// Serialized size of a signal's value; internal to [`fpp_state_machine!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_sized {
    ([]) => {
        0
    };
    ([$mode:ident $ty:ty]) => {
        <$ty as $crate::fw::FppSized>::SERIALIZED_SIZE
    };
}

/// `Some(e)` / `None`; internal to [`fpp_state_machine!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_opt {
    () => {
        ::core::option::Option::None
    };
    ($e:expr) => {
        ::core::option::Option::Some($e)
    };
}

/// Consumes a token so a repetition can be driven by it; internal to
/// [`fpp_state_machine!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_ignore {
    ($($t:tt)*) => {};
}

/// The uniform-signature helper behind one action: transitions call every
/// action as `__SmAction::<a>(h, id, signal, &value)`, and the helper's
/// bounds decide whether the value is passed on (typed action) or dropped
/// (untyped action). A typed action reached without a value of a
/// convertible type does not compile — FPP's static check.
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_action_helper {
    ($action:ident => $afn:ident []) => {
        #[inline]
        fn $action<H: Handlers + ?Sized, V: ?Sized>(
            h: &H,
            sm_id: $crate::config::FwEnumStoreType,
            signal: Signal,
            _value: &V,
        ) {
            h.$afn(sm_id, signal)
        }
    };
    ($action:ident => $afn:ident [val $ty:ty]) => {
        #[inline]
        fn $action<H: Handlers + ?Sized, V: Copy + ::core::convert::Into<$ty>>(
            h: &H,
            sm_id: $crate::config::FwEnumStoreType,
            signal: Signal,
            value: &V,
        ) {
            h.$afn(sm_id, signal, (*value).into())
        }
    };
    ($action:ident => $afn:ident [ref $ty:ty]) => {
        #[inline]
        fn $action<H: Handlers + ?Sized, V: ?Sized + ::core::borrow::Borrow<$ty>>(
            h: &H,
            sm_id: $crate::config::FwEnumStoreType,
            signal: Signal,
            value: &V,
        ) {
            h.$afn(sm_id, signal, value.borrow())
        }
    };
    ($action:ident => $afn:ident [str $ty:ty]) => {
        #[inline]
        fn $action<H: Handlers + ?Sized, V: ?Sized + ::core::convert::AsRef<str>>(
            h: &H,
            sm_id: $crate::config::FwEnumStoreType,
            signal: Signal,
            value: &V,
        ) {
            h.$afn(sm_id, signal, value.as_ref())
        }
    };
}

/// The uniform-signature helper behind one guard (see
/// [`__fpp_sm_action_helper!`]).
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_guard_helper {
    ($guard:ident => $gfn:ident []) => {
        #[inline]
        fn $guard<H: Handlers + ?Sized, V: ?Sized>(
            h: &H,
            sm_id: $crate::config::FwEnumStoreType,
            signal: Signal,
            _value: &V,
        ) -> bool {
            h.$gfn(sm_id, signal)
        }
    };
    ($guard:ident => $gfn:ident [val $ty:ty]) => {
        #[inline]
        fn $guard<H: Handlers + ?Sized, V: Copy + ::core::convert::Into<$ty>>(
            h: &H,
            sm_id: $crate::config::FwEnumStoreType,
            signal: Signal,
            value: &V,
        ) -> bool {
            h.$gfn(sm_id, signal, (*value).into())
        }
    };
    ($guard:ident => $gfn:ident [ref $ty:ty]) => {
        #[inline]
        fn $guard<H: Handlers + ?Sized, V: ?Sized + ::core::borrow::Borrow<$ty>>(
            h: &H,
            sm_id: $crate::config::FwEnumStoreType,
            signal: Signal,
            value: &V,
        ) -> bool {
            h.$gfn(sm_id, signal, value.borrow())
        }
    };
    ($guard:ident => $gfn:ident [str $ty:ty]) => {
        #[inline]
        fn $guard<H: Handlers + ?Sized, V: ?Sized + ::core::convert::AsRef<str>>(
            h: &H,
            sm_id: $crate::config::FwEnumStoreType,
            signal: Signal,
            value: &V,
        ) -> bool {
            h.$gfn(sm_id, signal, value.as_ref())
        }
    };
}

/// One state: its entry actions, exit actions and entry function (C++
/// `enter_<S>`). The two arms are the two legal shapes — a leaf with a
/// discriminant and no initial transition, a composite with an initial
/// transition and no discriminant.
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_state {
    (
        $(#[$m:meta])*
        $state:ident [$disc:literal] {
            $( entry do { $($en:ident),* $(,)? } )?
            $( exit do { $($ex:ident),* $(,)? } )?
        }
    ) => {
        $crate::__fpp_sm_state_actions!($state [$($($en),*)?] [$($($ex),*)?]);

        #[allow(dead_code, non_snake_case)]
        impl __SmEnter {
            $(#[$m])*
            fn $state<H: Handlers + ?Sized, V: ?Sized>(
                sm: &StateMachine,
                h: &H,
                signal: Signal,
                _value: &V,
            ) {
                __SmEntry::$state(h, sm.id(), signal);
                $crate::__fpp_sm_ignore!($disc);
                sm.__set_state(State::$state);
            }
        }
    };
    (
        $(#[$m:meta])*
        $state:ident [] {
            initial $( do { $($ia:ident),* $(,)? } )? enter $it:ident ;
            $( entry do { $($en:ident),* $(,)? } )?
            $( exit do { $($ex:ident),* $(,)? } )?
        }
    ) => {
        $crate::__fpp_sm_state_actions!($state [$($($en),*)?] [$($($ex),*)?]);

        #[allow(dead_code, non_snake_case)]
        impl __SmEnter {
            $(#[$m])*
            fn $state<H: Handlers + ?Sized, V: ?Sized>(
                sm: &StateMachine,
                h: &H,
                signal: Signal,
                _value: &V,
            ) {
                let sm_id = sm.id();
                __SmEntry::$state(h, sm_id, signal);
                // The initial transition: no exits, its actions, the states
                // between, then its target.
                $($( __SmAction::$ia(h, sm_id, signal, &()); )*)?
                $crate::sm::for_each_entry(Some(__SmNode::$state), __SmNode::$it, |n| {
                    n.__entry(h, sm_id, signal)
                });
                __SmEnter::$it(sm, h, signal, &());
            }
        }
    };
}

/// A state's entry and exit action lists; internal to
/// [`fpp_state_machine!`]. Entry and exit actions are untyped (passing
/// `&()` makes a typed one a compile error, as in FPP).
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_state_actions {
    ($state:ident [$($en:ident),*] [$($ex:ident),*]) => {
        #[allow(dead_code, non_snake_case, unused_variables)]
        impl __SmEntry {
            fn $state<H: Handlers + ?Sized>(
                h: &H,
                sm_id: $crate::config::FwEnumStoreType,
                signal: Signal,
            ) {
                $( __SmAction::$en(h, sm_id, signal, &()); )*
            }
        }

        #[allow(dead_code, non_snake_case, unused_variables)]
        impl __SmExit {
            fn $state<H: Handlers + ?Sized>(
                h: &H,
                sm_id: $crate::config::FwEnumStoreType,
                signal: Signal,
            ) {
                $( __SmAction::$ex(h, sm_id, signal, &()); )*
            }
        }
    };
}

/// One choice: its entry function (C++ `enter_<C>`), typed by its value
/// type if it has one.
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_choice {
    ($(#[$m:meta])* $choice:ident [] { $($body:tt)* }) => {
        #[allow(dead_code, non_snake_case)]
        impl __SmEnter {
            $(#[$m])*
            fn $choice<H: Handlers + ?Sized, V: ?Sized>(
                sm: &StateMachine,
                h: &H,
                signal: Signal,
                _value: &V,
            ) {
                let value = &();
                $crate::__fpp_sm_choice_body!((sm, h, signal, value) $choice { $($body)* });
            }
        }
    };
    ($(#[$m:meta])* $choice:ident [val $ty:ty] { $($body:tt)* }) => {
        #[allow(dead_code, non_snake_case)]
        impl __SmEnter {
            $(#[$m])*
            fn $choice<H: Handlers + ?Sized, V: Copy + ::core::convert::Into<$ty>>(
                sm: &StateMachine,
                h: &H,
                signal: Signal,
                value: &V,
            ) {
                let value: $ty = (*value).into();
                let value = &value;
                $crate::__fpp_sm_choice_body!((sm, h, signal, value) $choice { $($body)* });
            }
        }
    };
    ($(#[$m:meta])* $choice:ident [ref $ty:ty] { $($body:tt)* }) => {
        #[allow(dead_code, non_snake_case)]
        impl __SmEnter {
            $(#[$m])*
            fn $choice<H: Handlers + ?Sized, V: ?Sized + ::core::borrow::Borrow<$ty>>(
                sm: &StateMachine,
                h: &H,
                signal: Signal,
                value: &V,
            ) {
                let value: &$ty = value.borrow();
                $crate::__fpp_sm_choice_body!((sm, h, signal, value) $choice { $($body)* });
            }
        }
    };
    ($(#[$m:meta])* $choice:ident [str $ty:ty] { $($body:tt)* }) => {
        #[allow(dead_code, non_snake_case)]
        impl __SmEnter {
            $(#[$m])*
            fn $choice<H: Handlers + ?Sized, V: ?Sized + ::core::convert::AsRef<str>>(
                sm: &StateMachine,
                h: &H,
                signal: Signal,
                value: &V,
            ) {
                let value: &str = value.as_ref();
                $crate::__fpp_sm_choice_body!((sm, h, signal, value) $choice { $($body)* });
            }
        }
    };
}

/// The `if g ... else ...` of a choice; internal to [`fpp_state_machine!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_choice_body {
    (
        ($sm:ident, $h:ident, $signal:ident, $value:ident) $choice:ident {
            if $g:ident $( do { $($ia:ident),* $(,)? } )? enter $it:ident
            else $( do { $($ea:ident),* $(,)? } )? enter $et:ident
        }
    ) => {{
        let sm_id = $sm.id();
        if __SmGuard::$g($h, sm_id, $signal, $value) {
            $crate::__fpp_sm_go!(
                ($sm, $h, sm_id, $signal, $value, __SmNode::$choice)
                $choice [$($($ia),*)?] [$it]
            );
        } else {
            $crate::__fpp_sm_go!(
                ($sm, $h, sm_id, $signal, $value, __SmNode::$choice)
                $choice [$($($ea),*)?] [$et]
            );
        }
    }};
}

/// Take one transition: exits, actions, entries, target. `cur` is the
/// node the configuration currently ends at (the leaf state for a signal,
/// the choice for a choice branch). No target makes an internal
/// transition: just the actions.
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_go {
    (
        ($sm:ident, $h:ident, $sm_id:ident, $signal:ident, $value:ident, $cur:expr)
        $source:ident [$($a:ident),*] []
    ) => {{
        $( __SmAction::$a($h, $sm_id, $signal, $value); )*
    }};
    (
        ($sm:ident, $h:ident, $sm_id:ident, $signal:ident, $value:ident, $cur:expr)
        $source:ident [$($a:ident),*] [$target:ident]
    ) => {{
        let lca = $crate::sm::lca(__SmNode::$source, __SmNode::$target);
        $crate::sm::for_each_exit($cur, lca, |n| n.__exit($h, $sm_id, $signal));
        $( __SmAction::$a($h, $sm_id, $signal, $value); )*
        $crate::sm::for_each_entry(lca, __SmNode::$target, |n| n.__entry($h, $sm_id, $signal));
        __SmEnter::$target($sm, $h, $signal, $value);
    }};
}

/// A transition's guard, `true` if it has none; internal to
/// [`fpp_state_machine!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_guard_call {
    (($h:ident, $sm_id:ident, $signal:ident, $value:ident) []) => {
        true
    };
    (($h:ident, $sm_id:ident, $signal:ident, $value:ident) [$g:ident]) => {
        __SmGuard::$g($h, $sm_id, $signal, $value)
    };
}

/// One `send_signal_<s>` function (C++ `sendSignal_<s>`); internal to
/// [`fpp_state_machine!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_signal_fn {
    ($(#[$m:meta])* $signal:ident => $sfn:ident [] { $($trans:tt)* }) => {
        $(#[$m])*
        #[doc = ""]
        #[doc = concat!("Send signal `", stringify!($signal), "` (C++ `sendSignal_", stringify!($signal), "`).")]
        pub fn $sfn<H: Handlers + ?Sized>(&self, handlers: &H) {
            let sm = self;
            let value = &();
            $crate::__fpp_sm_signal_body!((sm, handlers, value) $signal { $($trans)* });
        }
    };
    ($(#[$m:meta])* $signal:ident => $sfn:ident [val $ty:ty] { $($trans:tt)* }) => {
        $(#[$m])*
        #[doc = ""]
        #[doc = concat!("Send signal `", stringify!($signal), "` (C++ `sendSignal_", stringify!($signal), "`).")]
        pub fn $sfn<H: Handlers + ?Sized>(&self, handlers: &H, value: $ty) {
            let sm = self;
            let value = &value;
            $crate::__fpp_sm_signal_body!((sm, handlers, value) $signal { $($trans)* });
        }
    };
    ($(#[$m:meta])* $signal:ident => $sfn:ident [ref $ty:ty] { $($trans:tt)* }) => {
        $(#[$m])*
        #[doc = ""]
        #[doc = concat!("Send signal `", stringify!($signal), "` (C++ `sendSignal_", stringify!($signal), "`).")]
        pub fn $sfn<H: Handlers + ?Sized>(&self, handlers: &H, value: &$ty) {
            let sm = self;
            $crate::__fpp_sm_signal_body!((sm, handlers, value) $signal { $($trans)* });
        }
    };
    ($(#[$m:meta])* $signal:ident => $sfn:ident [str $ty:ty] { $($trans:tt)* }) => {
        $(#[$m])*
        #[doc = ""]
        #[doc = concat!("Send signal `", stringify!($signal), "` (C++ `sendSignal_", stringify!($signal), "`).")]
        pub fn $sfn<H: Handlers + ?Sized>(&self, handlers: &H, value: &str) {
            let sm = self;
            $crate::__fpp_sm_signal_body!((sm, handlers, value) $signal { $($trans)* });
        }
    };
}

/// The body of a `send_signal_<s>` function: find the innermost state,
/// from the current leaf outwards, with a transition on the signal, and
/// take it if its guard holds.
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_signal_body {
    (
        ($sm:ident, $h:ident, $value:ident) $signal:ident {
            $(
                $src:ident =>
                    $( if $g:ident )?
                    $( do { $($a:ident),* $(,)? } )?
                    $( enter $t:ident )?
                ;
            )*
        }
    ) => {{
        let _run = $sm.core.begin_run();
        let signal = Signal::$signal;
        let sm_id = $sm.id();
        let Some(leaf) = $sm.__leaf() else {
            return;
        };
        let mut cur = leaf;
        loop {
            #[allow(unreachable_patterns)]
            match cur {
                $(
                    __SmNode::$src => {
                        if $crate::__fpp_sm_guard_call!(($h, sm_id, signal, $value) [$($g)?]) {
                            $crate::__fpp_sm_go!(
                                ($sm, $h, sm_id, signal, $value, leaf)
                                $src [$($($a),*)?] [$($t)?]
                            );
                        }
                        return;
                    }
                )*
                _ => {}
            }
            match $crate::sm::SmNode::parent(cur) {
                Some(p) => cur = p,
                None => return,
            }
        }
    }};
}

/// One arm of `dispatch_signal`; internal to [`fpp_state_machine!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_dispatch_one {
    ($sm:ident, $h:ident, $msg:ident, $sfn:ident []) => {{
        let left = $crate::fw::SerBuf::deserialize_size_left(&*$msg);
        $crate::fw::fw_assert!(left == 0, left as i32);
        $sm.$sfn($h)
    }};
    ($sm:ident, $h:ident, $msg:ident, $sfn:ident [val $ty:ty]) => {{
        let mut value = <$ty as ::core::default::Default>::default();
        let status =
            $crate::fw::SerBuf::deserialize(&mut *$msg, &mut value, $crate::fw::Endianness::Big);
        $crate::fw::fw_assert!(status.is_ok(), status as i32);
        let left = $crate::fw::SerBuf::deserialize_size_left(&*$msg);
        $crate::fw::fw_assert!(left == 0, left as i32);
        $sm.$sfn($h, value)
    }};
    ($sm:ident, $h:ident, $msg:ident, $sfn:ident [ref $ty:ty]) => {{
        let mut value = <$ty as ::core::default::Default>::default();
        let status =
            $crate::fw::SerBuf::deserialize(&mut *$msg, &mut value, $crate::fw::Endianness::Big);
        $crate::fw::fw_assert!(status.is_ok(), status as i32);
        let left = $crate::fw::SerBuf::deserialize_size_left(&*$msg);
        $crate::fw::fw_assert!(left == 0, left as i32);
        $sm.$sfn($h, &value)
    }};
    ($sm:ident, $h:ident, $msg:ident, $sfn:ident [str $ty:ty]) => {{
        let mut value = <$ty as ::core::default::Default>::default();
        let status =
            $crate::fw::SerBuf::deserialize(&mut *$msg, &mut value, $crate::fw::Endianness::Big);
        $crate::fw::fw_assert!(status.is_ok(), status as i32);
        let left = $crate::fw::SerBuf::deserialize_size_left(&*$msg);
        $crate::fw::fw_assert!(left == 0, left as i32);
        $sm.$sfn($h, $crate::sm::str_of(&value))
    }};
}

/// One `Sender` method (C++ `<m>_sendSignal_<s>`); internal to
/// [`fpp_state_machine!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_sm_sender_fn {
    ($(#[$m:meta])* $signal:ident => $sfn:ident []) => {
        $(#[$m])*
        #[doc = ""]
        #[doc = concat!("Enqueue signal `", stringify!($signal), "` for this instance.")]
        pub fn $sfn(&self) {
            self.0.send(Signal::$signal.as_repr(), None)
        }
    };
    ($(#[$m:meta])* $signal:ident => $sfn:ident [val $ty:ty]) => {
        $(#[$m])*
        #[doc = ""]
        #[doc = concat!("Enqueue signal `", stringify!($signal), "` with its value for this instance.")]
        pub fn $sfn(&self, value: $ty) {
            self.0.send(Signal::$signal.as_repr(), Some(&value))
        }
    };
    ($(#[$m:meta])* $signal:ident => $sfn:ident [ref $ty:ty]) => {
        $(#[$m])*
        #[doc = ""]
        #[doc = concat!("Enqueue signal `", stringify!($signal), "` with its value for this instance.")]
        pub fn $sfn(&self, value: &$ty) {
            self.0.send(Signal::$signal.as_repr(), Some(value))
        }
    };
    ($(#[$m:meta])* $signal:ident => $sfn:ident [str $ty:ty]) => {
        $(#[$m])*
        #[doc = ""]
        #[doc = concat!("Enqueue signal `", stringify!($signal), "` with its value for this instance.")]
        pub fn $sfn(&self, value: &str) {
            let arg = $crate::sm::StrArg::new(value, <$ty>::CAPACITY);
            self.0.send(Signal::$signal.as_repr(), Some(&arg))
        }
    };
}
