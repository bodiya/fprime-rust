//! Port of `FppTest/state_machine/internal_instance`: state machine
//! instances inside queued and active components. Signals sent through a
//! component are queue messages dispatched on the component's thread, with
//! the instance's priority and queue-full policy; the message is the C++
//! one byte for byte.
//!
//! The components here are written the way the FPP back end will emit them
//! (`component_msg_types!`, `component_sm_ids!`, one `fpp_state_machine!`
//! module per machine, the `INTERNAL_STATE_MACHINE_SIGNAL` arm of
//! `dispatch_message`), so this file is also the reference for that
//! generator. The state machines are the upstream `.fppi` models; like
//! upstream, a component-local copy (`include "...fppi"` inside the
//! component) is a separate machine from the module-level one.
#![allow(non_snake_case)]

mod sm_harness;

use fprime_comp::config::{FwEnumStoreType, FwQueuePriorityType, FwSizeType};
use fprime_comp::fw::{LinearBuffer, SerBuf, SerBufAny};
use fprime_comp::os::queue::{BlockingType, Status as QueueStatus};
use fprime_comp::sm::{self, SmOverflowHook, SmSender};
use fprime_comp::{
    ActiveBase, ActiveComponent, ComponentDispatch, MsgDispatchStatus, QueueFullPolicy, QueuedBase,
    component_msg_types, component_sm_ids, fpp_state_machine, msg,
};
use sm_harness::{Guard, History, Pick, TestAbsType};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

const QUEUE_DEPTH: FwSizeType = 10;

/// Read and check the port number, then the state machine id and signal
/// (the generated `doDispatch` prologue for `INTERNAL_STATE_MACHINE_SIGNAL`).
fn read_sm_message(msg: &mut dyn SerBufAny) -> (FwEnumStoreType, FwEnumStoreType) {
    let mut port_num = 0;
    let status = msg::read_port_num(msg, &mut port_num);
    assert!(status.is_ok());
    sm::read_signal_header(msg)
}

/// Receive the next raw message of a queue: (bytes, priority).
fn take_raw(queue: &QueuedBase, size: usize) -> (Vec<u8>, FwQueuePriorityType) {
    let mut bytes = vec![0u8; size];
    let mut actual: FwSizeType = 0;
    let mut priority: FwQueuePriorityType = 0;
    let status = queue.queue().receive(
        &mut bytes,
        BlockingType::NonBlocking,
        &mut actual,
        &mut priority,
    );
    assert_eq!(status, QueueStatus::OpOk);
    bytes.truncate(actual as usize);
    (bytes, priority)
}

// ===========================================================================
// SmInstanceState.Basic — queued; four instances of two machines
// ===========================================================================

/// `include "../../internal/state/include/Basic.fppi"` inside the
/// component: C++ `Basic_Basic`.
mod Basic_Basic {
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

/// `FppTest.SmState.Basic`: C++ `SmState_Basic`.
mod SmState_Basic {
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

/// `queued component Basic` with `sync input port schedIn` and
/// `basic1: Basic`, `basic2: Basic`, `smStateBasic1: SmState.Basic
/// priority 1 assert`, `smStateBasic2: SmState.Basic priority 2 drop`.
struct StateBasic {
    queued: QueuedBase,
    basic1: Basic_Basic::StateMachine,
    basic2: Basic_Basic::StateMachine,
    sm_state_basic1: SmState_Basic::StateMachine,
    sm_state_basic2: SmState_Basic::StateMachine,
    // Tester state.
    basic1_a: History<Basic_Basic::Signal, ()>,
    basic2_a: History<Basic_Basic::Signal, ()>,
    sm_state_basic1_a: History<SmState_Basic::Signal, ()>,
    sm_state_basic2_a: History<SmState_Basic::Signal, ()>,
}

component_msg_types! {
    impl StateBasic {
        MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
    }
}

component_sm_ids! {
    /// C++ `SmId`: the instances in name order.
    impl StateBasic {
        SM_ID_BASIC1,
        SM_ID_BASIC2,
        SM_ID_SM_STATE_BASIC1,
        SM_ID_SM_STATE_BASIC2,
    }
}

const STATE_BASIC_MSG_SIZE: usize = sm::max_size(&[
    Basic_Basic::StateMachine::SIGNAL_MSG_SIZE,
    SmState_Basic::StateMachine::SIGNAL_MSG_SIZE,
]);

impl StateBasic {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            queued: QueuedBase::new("basic"),
            basic1: Basic_Basic::StateMachine::new(),
            basic2: Basic_Basic::StateMachine::new(),
            sm_state_basic1: SmState_Basic::StateMachine::new(),
            sm_state_basic2: SmState_Basic::StateMachine::new(),
            basic1_a: History::new(),
            basic2_a: History::new(),
            sm_state_basic1_a: History::new(),
            sm_state_basic2_a: History::new(),
        })
    }

    /// C++ `init(queueDepth, instance)`: initialize every state machine
    /// instance with its id (running its initial transition), then create
    /// the queue — in that order, so an initial transition cannot send
    /// signals through the queue.
    fn init(&self, queue_depth: FwSizeType) {
        self.basic1.init(Self::SM_ID_BASIC1, self);
        self.basic2.init(Self::SM_ID_BASIC2, self);
        self.sm_state_basic1.init(Self::SM_ID_SM_STATE_BASIC1, self);
        self.sm_state_basic2.init(Self::SM_ID_SM_STATE_BASIC2, self);
        self.queued
            .create_queue(queue_depth, STATE_BASIC_MSG_SIZE as FwSizeType);
    }

    // `<m>_sendSignal_<s>`: one sender per instance.
    fn basic1_sm(&self) -> Basic_Basic::Sender<'_, STATE_BASIC_MSG_SIZE> {
        Basic_Basic::Sender(SmSender::new(
            &self.queued,
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
            Self::SM_ID_BASIC1,
            0,
            QueueFullPolicy::Assert,
        ))
    }

    fn basic2_sm(&self) -> Basic_Basic::Sender<'_, STATE_BASIC_MSG_SIZE> {
        Basic_Basic::Sender(SmSender::new(
            &self.queued,
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
            Self::SM_ID_BASIC2,
            0,
            QueueFullPolicy::Assert,
        ))
    }

    fn sm_state_basic1_sm(&self) -> SmState_Basic::Sender<'_, STATE_BASIC_MSG_SIZE> {
        SmState_Basic::Sender(SmSender::new(
            &self.queued,
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
            Self::SM_ID_SM_STATE_BASIC1,
            1,
            QueueFullPolicy::Assert,
        ))
    }

    fn sm_state_basic2_sm(&self) -> SmState_Basic::Sender<'_, STATE_BASIC_MSG_SIZE> {
        SmState_Basic::Sender(SmSender::new(
            &self.queued,
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
            Self::SM_ID_SM_STATE_BASIC2,
            2,
            QueueFullPolicy::Drop,
        ))
    }

    fn do_dispatch(&self) -> MsgDispatchStatus {
        self.queued.do_dispatch(self, BlockingType::NonBlocking)
    }
}

impl ComponentDispatch for StateBasic {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        match msg_type {
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL => {
                let (sm_id, signal) = read_sm_message(msg);
                match sm_id {
                    Self::SM_ID_BASIC1 => self.basic1.dispatch_signal(self, signal, msg),
                    Self::SM_ID_BASIC2 => self.basic2.dispatch_signal(self, signal, msg),
                    Self::SM_ID_SM_STATE_BASIC1 => {
                        self.sm_state_basic1.dispatch_signal(self, signal, msg)
                    }
                    Self::SM_ID_SM_STATE_BASIC2 => {
                        self.sm_state_basic2.dispatch_signal(self, signal, msg)
                    }
                    // C++ parity: an unknown state machine id asserts.
                    _ => fprime_comp::fw::fw_assert!(false, sm_id),
                }
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

/// C++ `FppTest_SmInstanceState_Basic_Basic_action_a(smId, signal)`.
impl Basic_Basic::Handlers for StateBasic {
    fn action_a(&self, sm_id: FwEnumStoreType, signal: Basic_Basic::Signal) {
        match sm_id {
            Self::SM_ID_BASIC1 => self.basic1_a.push(signal, ()),
            Self::SM_ID_BASIC2 => self.basic2_a.push(signal, ()),
            _ => panic!("unexpected sm id {sm_id}"),
        }
    }
}

/// C++ `FppTest_SmState_Basic_action_a(smId, signal)`.
impl SmState_Basic::Handlers for StateBasic {
    fn action_a(&self, sm_id: FwEnumStoreType, signal: SmState_Basic::Signal) {
        match sm_id {
            Self::SM_ID_SM_STATE_BASIC1 => self.sm_state_basic1_a.push(signal, ()),
            Self::SM_ID_SM_STATE_BASIC2 => self.sm_state_basic2_a.push(signal, ()),
            _ => panic!("unexpected sm id {sm_id}"),
        }
    }
}

#[test]
fn state_basic_test() {
    let c = StateBasic::new();
    c.init(QUEUE_DEPTH);
    assert_eq!(c.basic1.get_state(), Basic_Basic::State::S);
    assert_eq!(c.sm_state_basic1.get_state(), SmState_Basic::State::S);
    assert_eq!(c.basic1_a.size(), 0);
    assert_eq!(c.basic2_a.size(), 0);
    assert_eq!(c.sm_state_basic1_a.size(), 0);

    c.basic1_sm().send_signal_s();
    // Nothing happens until the message is dispatched.
    assert_eq!(c.basic1.get_state(), Basic_Basic::State::S);
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.basic1.get_state(), Basic_Basic::State::T);
    assert_eq!(c.basic1_a.signals(), vec![Basic_Basic::Signal::s; 6]);

    c.basic2_sm().send_signal_s();
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.basic2.get_state(), Basic_Basic::State::T);
    assert_eq!(c.basic2_a.signals(), vec![Basic_Basic::Signal::s; 6]);

    c.sm_state_basic1_sm().send_signal_s();
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.sm_state_basic1.get_state(), SmState_Basic::State::T);
    assert_eq!(
        c.sm_state_basic1_a.signals(),
        vec![SmState_Basic::Signal::s; 6]
    );

    c.sm_state_basic2_sm().send_signal_s();
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.sm_state_basic2.get_state(), SmState_Basic::State::T);
    assert_eq!(
        c.sm_state_basic2_a.signals(),
        vec![SmState_Basic::Signal::s; 6]
    );
}

#[test]
fn state_basic_signal_message_is_byte_exact() {
    let c = StateBasic::new();
    c.init(QUEUE_DEPTH);
    // C++ SERIALIZATION_SIZE: DATA_OFFSET (4 + 2) + 2 * sizeof(FwEnumStoreType).
    assert_eq!(STATE_BASIC_MSG_SIZE, 14);
    c.sm_state_basic2_sm().send_signal_s();
    let (bytes, priority) = take_raw(&c.queued, STATE_BASIC_MSG_SIZE);
    assert_eq!(priority, 2);
    assert_eq!(
        bytes,
        [
            0, 0, 0, 1, // INTERNAL_STATE_MACHINE_SIGNAL
            0, 0, // port number 0
            0, 0, 0, 3, // SmId::smStateBasic2
            0, 0, 0, 1, // Signal::s
        ]
    );
}

#[test]
fn state_basic_priorities_order_dispatch() {
    let c = StateBasic::new();
    c.init(QUEUE_DEPTH);
    // basic1: priority 0, smStateBasic1: 1, smStateBasic2: 2.
    c.basic1_sm().send_signal_s();
    c.sm_state_basic1_sm().send_signal_s();
    c.sm_state_basic2_sm().send_signal_s();
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.sm_state_basic2.get_state(), SmState_Basic::State::T);
    assert_eq!(c.sm_state_basic1.get_state(), SmState_Basic::State::S);
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.sm_state_basic1.get_state(), SmState_Basic::State::T);
    assert_eq!(c.basic1.get_state(), Basic_Basic::State::S);
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.basic1.get_state(), Basic_Basic::State::T);
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Empty);
}

#[test]
fn state_basic_drop_policy_counts_dropped_signals() {
    let c = StateBasic::new();
    c.init(QUEUE_DEPTH);
    for _ in 0..QUEUE_DEPTH {
        c.sm_state_basic2_sm().send_signal_s();
    }
    assert_eq!(c.queued.get_num_msgs_dropped(), 0);
    c.sm_state_basic2_sm().send_signal_s();
    assert_eq!(c.queued.get_num_msgs_dropped(), 1);
}

#[test]
#[should_panic]
fn state_basic_assert_policy_asserts_on_overflow() {
    let c = StateBasic::new();
    c.init(QUEUE_DEPTH);
    for _ in 0..=QUEUE_DEPTH {
        c.sm_state_basic1_sm().send_signal_s();
    }
}

#[test]
#[should_panic]
fn state_basic_unknown_sm_id_asserts() {
    let c = StateBasic::new();
    c.init(QUEUE_DEPTH);
    // A well-formed message for an id the component does not have.
    SmState_Basic::Sender(SmSender::<'_, STATE_BASIC_MSG_SIZE>::new(
        &c.queued,
        StateBasic::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
        7,
        0,
        QueueFullPolicy::Assert,
    ))
    .send_signal_s();
    let _ = c.do_dispatch();
}

// ===========================================================================
// SmInstanceState.BasicGuardTestAbsType — active, `priority 4 hook`
// ===========================================================================

mod SmState_BasicGuardTestAbsType {
    use super::TestAbsType;
    fprime_comp::fpp_state_machine! {
        /// A basic state machine with a TestAbsType guard
        state machine BasicGuardTestAbsType: u8 {
            actions { a => action_a (ref TestAbsType) }
            guards { g => guard_g (ref TestAbsType) }
            signals {
                s = 1 => send_signal_s (ref TestAbsType) {
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
}

struct GuardAbs {
    active: ActiveBase,
    sm_state_basic_guard_test_abs_type: SmState_BasicGuardTestAbsType::StateMachine,
    a: History<SmState_BasicGuardTestAbsType::Signal, TestAbsType>,
    g: Guard<SmState_BasicGuardTestAbsType::Signal, TestAbsType>,
    value: std::sync::Mutex<TestAbsType>,
    hook_called: AtomicBool,
}

component_msg_types! {
    impl GuardAbs {
        MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
    }
}

component_sm_ids! {
    impl GuardAbs {
        SM_ID_SM_STATE_BASIC_GUARD_TEST_ABS_TYPE,
    }
}

const GUARD_ABS_MSG_SIZE: usize = SmState_BasicGuardTestAbsType::StateMachine::SIGNAL_MSG_SIZE;

impl GuardAbs {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new("guardAbs"),
            sm_state_basic_guard_test_abs_type: SmState_BasicGuardTestAbsType::StateMachine::new(),
            a: History::new(),
            g: Guard::new(),
            value: std::sync::Mutex::new(TestAbsType::default()),
            hook_called: AtomicBool::new(false),
        })
    }

    fn init(&self, queue_depth: FwSizeType) {
        self.sm_state_basic_guard_test_abs_type
            .init(Self::SM_ID_SM_STATE_BASIC_GUARD_TEST_ABS_TYPE, self);
        self.active
            .queued
            .create_queue(queue_depth, GUARD_ABS_MSG_SIZE as FwSizeType);
    }

    fn sm_state_basic_guard_test_abs_type_sm(
        &self,
    ) -> SmState_BasicGuardTestAbsType::Sender<'_, GUARD_ABS_MSG_SIZE> {
        SmState_BasicGuardTestAbsType::Sender(
            SmSender::new(
                &self.active.queued,
                Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
                Self::SM_ID_SM_STATE_BASIC_GUARD_TEST_ABS_TYPE,
                4,
                QueueFullPolicy::Hook,
            )
            .with_overflow_hook(self),
        )
    }

    fn do_dispatch(&self) -> MsgDispatchStatus {
        self.active
            .queued
            .do_dispatch(self, BlockingType::NonBlocking)
    }
}

impl ComponentDispatch for GuardAbs {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        match msg_type {
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL => {
                let (sm_id, signal) = read_sm_message(msg);
                match sm_id {
                    Self::SM_ID_SM_STATE_BASIC_GUARD_TEST_ABS_TYPE => self
                        .sm_state_basic_guard_test_abs_type
                        .dispatch_signal(self, signal, msg),
                    _ => fprime_comp::fw::fw_assert!(false, sm_id),
                }
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for GuardAbs {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

impl SmState_BasicGuardTestAbsType::Handlers for GuardAbs {
    fn action_a(
        &self,
        sm_id: FwEnumStoreType,
        signal: SmState_BasicGuardTestAbsType::Signal,
        value: &TestAbsType,
    ) {
        assert_eq!(sm_id, Self::SM_ID_SM_STATE_BASIC_GUARD_TEST_ABS_TYPE);
        self.a.push(signal, *value);
    }
    fn guard_g(
        &self,
        sm_id: FwEnumStoreType,
        signal: SmState_BasicGuardTestAbsType::Signal,
        value: &TestAbsType,
    ) -> bool {
        assert_eq!(sm_id, Self::SM_ID_SM_STATE_BASIC_GUARD_TEST_ABS_TYPE);
        self.g.call(signal, *value)
    }
}

/// C++ `smStateBasicGuardTestAbsType_stateMachineOverflowHook`.
impl SmOverflowHook for GuardAbs {
    fn sm_overflow_hook(
        &self,
        sm_id: FwEnumStoreType,
        signal: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) {
        self.hook_called.store(true, Ordering::Relaxed);
        assert_eq!(sm_id, Self::SM_ID_SM_STATE_BASIC_GUARD_TEST_ABS_TYPE);
        assert_eq!(signal, SmState_BasicGuardTestAbsType::Signal::s.as_repr());
        let mut value = TestAbsType::default();
        let status = msg.deserialize(&mut value, fprime_comp::fw::Endianness::Big);
        assert!(status.is_ok());
        assert_eq!(msg.deserialize_size_left(), 0);
        assert_eq!(value, *self.value.lock().unwrap());
    }
}

fn guard_abs(guard: bool) {
    use SmState_BasicGuardTestAbsType::{Signal, State};
    let c = GuardAbs::new();
    c.g.set_return_value(guard);
    c.init(QUEUE_DEPTH);
    assert_eq!(c.sm_state_basic_guard_test_abs_type.get_state(), State::S);
    assert_eq!(c.a.size(), 0);
    assert_eq!(c.g.history().size(), 0);
    let value = Pick::new(41).test_abs_type();
    *c.value.lock().unwrap() = value;
    c.sm_state_basic_guard_test_abs_type_sm()
        .send_signal_s(&value);
    assert!(!c.hook_called.load(Ordering::Relaxed));
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.g.history().items(), vec![(Signal::s, value)]);
    if guard {
        assert_eq!(c.sm_state_basic_guard_test_abs_type.get_state(), State::T);
        assert_eq!(c.a.items(), vec![(Signal::s, value)]);
    } else {
        assert_eq!(c.sm_state_basic_guard_test_abs_type.get_state(), State::S);
        assert_eq!(c.a.size(), 0);
    }
}

#[test]
fn guard_abs_false() {
    guard_abs(false);
}

#[test]
fn guard_abs_true() {
    guard_abs(true);
}

#[test]
fn guard_abs_overflow() {
    let c = GuardAbs::new();
    c.init(QUEUE_DEPTH);
    let value = Pick::new(42).test_abs_type();
    *c.value.lock().unwrap() = value;
    for _ in 0..QUEUE_DEPTH {
        c.sm_state_basic_guard_test_abs_type_sm()
            .send_signal_s(&value);
        assert!(!c.hook_called.load(Ordering::Relaxed));
    }
    c.sm_state_basic_guard_test_abs_type_sm()
        .send_signal_s(&value);
    assert!(c.hook_called.load(Ordering::Relaxed));
}

#[test]
fn guard_abs_signal_message_is_byte_exact() {
    let c = GuardAbs::new();
    c.init(QUEUE_DEPTH);
    // DATA_OFFSET + 2 * sizeof(FwEnumStoreType) + TestAbsType::SERIALIZED_SIZE.
    assert_eq!(GUARD_ABS_MSG_SIZE, 6 + 8 + 4);
    c.sm_state_basic_guard_test_abs_type_sm()
        .send_signal_s(&TestAbsType::new(0x0A0B_0C0D));
    let (bytes, priority) = take_raw(&c.active.queued, GUARD_ABS_MSG_SIZE);
    assert_eq!(priority, 4);
    assert_eq!(
        bytes,
        [
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0x0A, 0x0B, 0x0C, 0x0D
        ]
    );
}

/// A truncated signal message is a decode failure, which asserts in the
/// dispatch (C++ parity: `FW_ASSERT(status == FW_SERIALIZE_OK)`).
#[test]
#[should_panic]
fn guard_abs_short_signal_data_asserts() {
    let c = GuardAbs::new();
    c.init(QUEUE_DEPTH);
    let mut buf = LinearBuffer::<GUARD_ABS_MSG_SIZE>::new();
    let status = sm::write_signal_header(
        &mut buf,
        GuardAbs::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
        GuardAbs::SM_ID_SM_STATE_BASIC_GUARD_TEST_ABS_TYPE,
        SmState_BasicGuardTestAbsType::Signal::s.as_repr(),
    );
    assert!(status.is_ok());
    assert!(buf.serialize_u16_be(7).is_ok());
    let _ = c
        .active
        .queued
        .send_message(&buf, 0, QueueFullPolicy::Assert);
    let _ = c.do_dispatch();
}

/// The component thread dispatches the signal (active component started
/// for real).
#[test]
fn guard_abs_dispatches_on_the_component_thread() {
    use SmState_BasicGuardTestAbsType::State;
    let c = GuardAbs::new();
    c.g.set_return_value(true);
    c.init(QUEUE_DEPTH);
    c.active.start(
        &c,
        0,
        fprime_comp::os::task::TASK_DEFAULT,
        fprime_comp::os::task::TASK_DEFAULT,
    );
    c.sm_state_basic_guard_test_abs_type_sm()
        .send_signal_s(&TestAbsType::new(5));
    c.active.exit();
    let _ = c.active.join();
    assert_eq!(c.sm_state_basic_guard_test_abs_type.get_state(), State::T);
    assert_eq!(c.a.values(), vec![TestAbsType::new(5)]);
}

// ===========================================================================
// SmInstanceState.BasicGuardString — active, `priority 3 drop`
// ===========================================================================

mod SmState_BasicGuardString {
    fprime_comp::fpp_state_machine! {
        /// A basic state machine with a string guard
        state machine BasicGuardString: u8 {
            actions { a => action_a (str fprime_fw::FwDefaultString) }
            guards { g => guard_g (str fprime_fw::FwDefaultString) }
            signals {
                s = 1 => send_signal_s (str fprime_fw::FwString<80>) {
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
}

struct GuardString {
    active: ActiveBase,
    sm_state_basic_guard_string: SmState_BasicGuardString::StateMachine,
    a: History<SmState_BasicGuardString::Signal, String>,
    g: Guard<SmState_BasicGuardString::Signal, String>,
}

component_msg_types! {
    impl GuardString {
        MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
    }
}

const GUARD_STRING_MSG_SIZE: usize = SmState_BasicGuardString::StateMachine::SIGNAL_MSG_SIZE;

impl GuardString {
    const SM_ID_SM_STATE_BASIC_GUARD_STRING: FwEnumStoreType = 0;

    fn new() -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new("guardString"),
            sm_state_basic_guard_string: SmState_BasicGuardString::StateMachine::new(),
            a: History::new(),
            g: Guard::new(),
        })
    }

    fn init(&self, queue_depth: FwSizeType) {
        self.sm_state_basic_guard_string
            .init(Self::SM_ID_SM_STATE_BASIC_GUARD_STRING, self);
        self.active
            .queued
            .create_queue(queue_depth, GUARD_STRING_MSG_SIZE as FwSizeType);
    }

    fn sm(&self) -> SmState_BasicGuardString::Sender<'_, GUARD_STRING_MSG_SIZE> {
        SmState_BasicGuardString::Sender(SmSender::new(
            &self.active.queued,
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
            Self::SM_ID_SM_STATE_BASIC_GUARD_STRING,
            3,
            QueueFullPolicy::Drop,
        ))
    }

    fn do_dispatch(&self) -> MsgDispatchStatus {
        self.active
            .queued
            .do_dispatch(self, BlockingType::NonBlocking)
    }
}

impl ComponentDispatch for GuardString {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        match msg_type {
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL => {
                let (sm_id, signal) = read_sm_message(msg);
                fprime_comp::fw::fw_assert!(
                    sm_id == Self::SM_ID_SM_STATE_BASIC_GUARD_STRING,
                    sm_id
                );
                self.sm_state_basic_guard_string
                    .dispatch_signal(self, signal, msg);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl SmState_BasicGuardString::Handlers for GuardString {
    fn action_a(&self, _: FwEnumStoreType, signal: SmState_BasicGuardString::Signal, value: &str) {
        self.a.push(signal, value.to_string());
    }
    fn guard_g(
        &self,
        _: FwEnumStoreType,
        signal: SmState_BasicGuardString::Signal,
        value: &str,
    ) -> bool {
        self.g.call(signal, value.to_string())
    }
}

fn guard_string(guard: bool) {
    use SmState_BasicGuardString::{Signal, State};
    let c = GuardString::new();
    c.g.set_return_value(guard);
    c.init(QUEUE_DEPTH);
    assert_eq!(c.sm_state_basic_guard_string.get_state(), State::S);
    let value = Pick::new(43).string(80);
    c.sm().send_signal_s(&value);
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.g.history().items(), vec![(Signal::s, value.clone())]);
    if guard {
        assert_eq!(c.sm_state_basic_guard_string.get_state(), State::T);
        assert_eq!(c.a.items(), vec![(Signal::s, value)]);
    } else {
        assert_eq!(c.sm_state_basic_guard_string.get_state(), State::S);
        assert_eq!(c.a.size(), 0);
    }
}

#[test]
fn guard_string_false() {
    guard_string(false);
}

#[test]
fn guard_string_true() {
    guard_string(true);
}

#[test]
fn guard_string_is_truncated_to_its_declared_size_on_the_queue() {
    use SmState_BasicGuardString::Signal;
    let c = GuardString::new();
    c.init(QUEUE_DEPTH);
    // C++ `value.serializeTo(buffer, 80)`: [u16 length][at most 80 bytes].
    assert_eq!(GUARD_STRING_MSG_SIZE, 6 + 8 + 2 + 80);
    let long = "x".repeat(100);
    c.sm().send_signal_s(&long);
    let (bytes, priority) = take_raw(&c.active.queued, GUARD_STRING_MSG_SIZE);
    assert_eq!(priority, 3);
    assert_eq!(&bytes[..14], &[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
    assert_eq!(&bytes[14..16], &[0, 80]);
    assert_eq!(bytes.len(), GUARD_STRING_MSG_SIZE);

    c.sm().send_signal_s(&long);
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.g.history().items(), vec![(Signal::s, "x".repeat(80))]);
}

#[test]
fn guard_string_drop_policy_counts_dropped_signals() {
    let c = GuardString::new();
    c.init(QUEUE_DEPTH);
    for _ in 0..QUEUE_DEPTH {
        c.sm().send_signal_s("v");
    }
    c.sm().send_signal_s("v");
    c.sm().send_signal_s("v");
    assert_eq!(c.active.queued.get_num_msgs_dropped(), 2);
}

// ===========================================================================
// SmInstanceChoice.InputPairU16U32 — active, `priority 3 drop`
// ===========================================================================

mod SmChoice_InputPairU16U32 {
    fprime_comp::fpp_state_machine! {
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
                choice C (val u32) { if g do { a } enter S2 else do { a } enter S3 }
            }
        }
    }
}

struct InputPair {
    active: ActiveBase,
    sm_choice_input_pair_u16_u32: SmChoice_InputPairU16U32::StateMachine,
    a: History<SmChoice_InputPairU16U32::Signal, u32>,
    g: Guard<SmChoice_InputPairU16U32::Signal, u32>,
}

component_msg_types! {
    impl InputPair {
        MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
    }
}

const INPUT_PAIR_MSG_SIZE: usize = SmChoice_InputPairU16U32::StateMachine::SIGNAL_MSG_SIZE;

impl InputPair {
    const SM_ID: FwEnumStoreType = 0;

    fn new() -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new("inputPair"),
            sm_choice_input_pair_u16_u32: SmChoice_InputPairU16U32::StateMachine::new(),
            a: History::new(),
            g: Guard::new(),
        })
    }

    fn init(&self, queue_depth: FwSizeType) {
        self.sm_choice_input_pair_u16_u32.init(Self::SM_ID, self);
        self.active
            .queued
            .create_queue(queue_depth, INPUT_PAIR_MSG_SIZE as FwSizeType);
    }

    fn sm(&self) -> SmChoice_InputPairU16U32::Sender<'_, INPUT_PAIR_MSG_SIZE> {
        SmChoice_InputPairU16U32::Sender(SmSender::new(
            &self.active.queued,
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
            Self::SM_ID,
            3,
            QueueFullPolicy::Drop,
        ))
    }

    fn do_dispatch(&self) -> MsgDispatchStatus {
        self.active
            .queued
            .do_dispatch(self, BlockingType::NonBlocking)
    }
}

impl ComponentDispatch for InputPair {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        match msg_type {
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL => {
                let (sm_id, signal) = read_sm_message(msg);
                fprime_comp::fw::fw_assert!(sm_id == Self::SM_ID, sm_id);
                self.sm_choice_input_pair_u16_u32
                    .dispatch_signal(self, signal, msg);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl SmChoice_InputPairU16U32::Handlers for InputPair {
    fn action_a(&self, _: FwEnumStoreType, signal: SmChoice_InputPairU16U32::Signal, value: u32) {
        self.a.push(signal, value);
    }
    fn guard_g(
        &self,
        _: FwEnumStoreType,
        signal: SmChoice_InputPairU16U32::Signal,
        value: u32,
    ) -> bool {
        self.g.call(signal, value)
    }
}

fn input_pair(use_s1: bool, guard: bool) {
    use SmChoice_InputPairU16U32::{Signal, State};
    let pick = Pick::new(44);
    let c = InputPair::new();
    c.g.set_return_value(guard);
    c.init(QUEUE_DEPTH);
    assert_eq!(c.sm_choice_input_pair_u16_u32.get_state(), State::S1);
    let (signal, value) = if use_s1 {
        let value = pick.any() as u16;
        c.sm().send_signal_s1(value);
        (Signal::s1, u32::from(value))
    } else {
        let value = pick.any();
        c.sm().send_signal_s2(value);
        (Signal::s2, value)
    };
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert_eq!(c.g.history().items(), vec![(signal, value)]);
    assert_eq!(c.a.items(), vec![(signal, value)]);
    assert_eq!(
        c.sm_choice_input_pair_u16_u32.get_state(),
        if guard { State::S2 } else { State::S3 }
    );
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

#[test]
fn input_pair_u16_signal_is_two_bytes_on_the_wire() {
    let c = InputPair::new();
    c.init(QUEUE_DEPTH);
    // The message buffer is sized for the largest signal value (U32).
    assert_eq!(INPUT_PAIR_MSG_SIZE, 6 + 8 + 4);
    c.sm().send_signal_s1(0xBEEF);
    let (bytes, _) = take_raw(&c.active.queued, INPUT_PAIR_MSG_SIZE);
    assert_eq!(
        bytes,
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0xBE, 0xEF]
    );
}

// ===========================================================================
// SmInstanceChoice.ChoiceToChoice — active, `priority 4 hook`
// ===========================================================================

mod SmChoice_ChoiceToChoice {
    fprime_comp::fpp_state_machine! {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ActionId {
    ExitS1,
    A,
    EnterS2,
}

struct ChoiceToChoice {
    active: ActiveBase,
    sm_choice_choice_to_choice: SmChoice_ChoiceToChoice::StateMachine,
    actions: History<SmChoice_ChoiceToChoice::Signal, ActionId>,
    g1: Guard<SmChoice_ChoiceToChoice::Signal, ()>,
    g2: Guard<SmChoice_ChoiceToChoice::Signal, ()>,
    hook_called: AtomicBool,
}

component_msg_types! {
    impl ChoiceToChoice {
        MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
    }
}

const CHOICE_TO_CHOICE_MSG_SIZE: usize = SmChoice_ChoiceToChoice::StateMachine::SIGNAL_MSG_SIZE;

impl ChoiceToChoice {
    const SM_ID: FwEnumStoreType = 0;

    fn new() -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new("choiceToChoice"),
            sm_choice_choice_to_choice: SmChoice_ChoiceToChoice::StateMachine::new(),
            actions: History::new(),
            g1: Guard::new(),
            g2: Guard::new(),
            hook_called: AtomicBool::new(false),
        })
    }

    fn init(&self, queue_depth: FwSizeType) {
        self.sm_choice_choice_to_choice.init(Self::SM_ID, self);
        self.active
            .queued
            .create_queue(queue_depth, CHOICE_TO_CHOICE_MSG_SIZE as FwSizeType);
    }

    fn sm(&self) -> SmChoice_ChoiceToChoice::Sender<'_, CHOICE_TO_CHOICE_MSG_SIZE> {
        SmChoice_ChoiceToChoice::Sender(
            SmSender::new(
                &self.active.queued,
                Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
                Self::SM_ID,
                4,
                QueueFullPolicy::Hook,
            )
            .with_overflow_hook(self),
        )
    }

    fn do_dispatch(&self) -> MsgDispatchStatus {
        self.active
            .queued
            .do_dispatch(self, BlockingType::NonBlocking)
    }
}

impl ComponentDispatch for ChoiceToChoice {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        match msg_type {
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL => {
                let (sm_id, signal) = read_sm_message(msg);
                fprime_comp::fw::fw_assert!(sm_id == Self::SM_ID, sm_id);
                self.sm_choice_choice_to_choice
                    .dispatch_signal(self, signal, msg);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl SmChoice_ChoiceToChoice::Handlers for ChoiceToChoice {
    fn action_exitS1(&self, _: FwEnumStoreType, signal: SmChoice_ChoiceToChoice::Signal) {
        self.actions.push(signal, ActionId::ExitS1);
    }
    fn action_a(&self, _: FwEnumStoreType, signal: SmChoice_ChoiceToChoice::Signal) {
        self.actions.push(signal, ActionId::A);
    }
    fn action_enterS2(&self, _: FwEnumStoreType, signal: SmChoice_ChoiceToChoice::Signal) {
        self.actions.push(signal, ActionId::EnterS2);
    }
    fn guard_g1(&self, _: FwEnumStoreType, signal: SmChoice_ChoiceToChoice::Signal) -> bool {
        self.g1.call(signal, ())
    }
    fn guard_g2(&self, _: FwEnumStoreType, signal: SmChoice_ChoiceToChoice::Signal) -> bool {
        self.g2.call(signal, ())
    }
}

impl SmOverflowHook for ChoiceToChoice {
    fn sm_overflow_hook(
        &self,
        sm_id: FwEnumStoreType,
        signal: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) {
        self.hook_called.store(true, Ordering::Relaxed);
        assert_eq!(sm_id, Self::SM_ID);
        assert_eq!(signal, SmChoice_ChoiceToChoice::Signal::s.as_repr());
        // No data: the cursor is at the end of the message.
        assert_eq!(msg.deserialize_size_left(), 0);
    }
}

fn choice_to_choice(g1: bool, g2: bool) {
    use SmChoice_ChoiceToChoice::{Signal, State};
    let c = ChoiceToChoice::new();
    c.g1.set_return_value(g1);
    c.g2.set_return_value(g2);
    c.init(QUEUE_DEPTH);
    assert_eq!(c.sm_choice_choice_to_choice.get_state(), State::S1);
    c.sm().send_signal_s();
    assert_eq!(c.do_dispatch(), MsgDispatchStatus::Ok);
    assert!(!c.hook_called.load(Ordering::Relaxed));
    assert_eq!(c.g1.history().signals(), vec![Signal::s]);
    assert_eq!(c.g2.history().size(), if g1 { 0 } else { 1 });
    assert_eq!(c.actions.signals(), vec![Signal::s; 3]);
    assert_eq!(
        c.actions.values(),
        [ActionId::ExitS1, ActionId::A, ActionId::EnterS2]
    );
    let expected = if g1 || g2 { State::S2_S3 } else { State::S2_S4 };
    assert_eq!(c.sm_choice_choice_to_choice.get_state(), expected);
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

#[test]
fn choice_to_choice_overflow() {
    let c = ChoiceToChoice::new();
    c.init(QUEUE_DEPTH);
    for _ in 0..QUEUE_DEPTH {
        c.sm().send_signal_s();
        assert!(!c.hook_called.load(Ordering::Relaxed));
    }
    c.sm().send_signal_s();
    assert!(c.hook_called.load(Ordering::Relaxed));
}

// ===========================================================================
// SmInstanceInitial.Nested — active; `nested: Nested priority 3 drop`,
// `smInitialNested: SmInitial.Nested priority 4 hook`
// ===========================================================================

mod Nested_Nested {
    fprime_comp::fpp_state_machine! {
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

mod SmInitial_Nested {
    fprime_comp::fpp_state_machine! {
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

struct InitialNested {
    active: ActiveBase,
    nested: Nested_Nested::StateMachine,
    sm_initial_nested: SmInitial_Nested::StateMachine,
    nested_a: History<Nested_Nested::Signal, ()>,
    sm_initial_nested_a: History<SmInitial_Nested::Signal, ()>,
}

component_sm_ids! {
    impl InitialNested {
        SM_ID_NESTED,
        SM_ID_SM_INITIAL_NESTED,
    }
}

impl InitialNested {
    fn init(&self) {
        self.nested.init(Self::SM_ID_NESTED, self);
        self.sm_initial_nested
            .init(Self::SM_ID_SM_INITIAL_NESTED, self);
        self.active.queued.create_queue(QUEUE_DEPTH, 16);
    }
}

impl Nested_Nested::Handlers for InitialNested {
    fn action_a(&self, sm_id: FwEnumStoreType, signal: Nested_Nested::Signal) {
        assert_eq!(sm_id, Self::SM_ID_NESTED);
        self.nested_a.push(signal, ());
    }
}

impl SmInitial_Nested::Handlers for InitialNested {
    fn action_a(&self, sm_id: FwEnumStoreType, signal: SmInitial_Nested::Signal) {
        assert_eq!(sm_id, Self::SM_ID_SM_INITIAL_NESTED);
        self.sm_initial_nested_a.push(signal, ());
    }
}

#[test]
fn initial_nested_test() {
    let c = InitialNested {
        active: ActiveBase::new("nested"),
        nested: Nested_Nested::StateMachine::new(),
        sm_initial_nested: SmInitial_Nested::StateMachine::new(),
        nested_a: History::new(),
        sm_initial_nested_a: History::new(),
    };
    c.init();
    assert_eq!(c.nested.get_state(), Nested_Nested::State::S_T);
    assert_eq!(
        c.sm_initial_nested.get_state(),
        SmInitial_Nested::State::S_T
    );
    assert_eq!(
        c.nested_a.signals(),
        vec![Nested_Nested::Signal::__FPRIME_INITIAL_TRANSITION; 6]
    );
    assert_eq!(
        c.sm_initial_nested_a.signals(),
        vec![SmInitial_Nested::Signal::__FPRIME_INITIAL_TRANSITION; 6]
    );
    assert_eq!(c.nested.id(), 0);
    assert_eq!(c.sm_initial_nested.id(), 1);
}

// ===========================================================================
// Actions that send signals: through the queue, run to completion
// ===========================================================================

mod PingPong {
    fprime_comp::fpp_state_machine! {
        /// Two states that bounce a counter through the component queue.
        state machine PingPong: u8 {
            actions { bounce => action_bounce }
            guards {}
            signals {
                ping = 1 => send_signal_ping {
                    A => do { bounce } enter B;
                    B => do { bounce } enter A;
                }
            }
            initial enter A;
            states {
                state A = 1 {}
                state B = 2 {}
            }
            choices {}
        }
    }
}

struct Bouncer {
    queued: QueuedBase,
    sm_ping_pong: PingPong::StateMachine,
    bounces: AtomicU32,
}

component_msg_types! {
    impl Bouncer {
        MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
    }
}

const BOUNCER_MSG_SIZE: usize = PingPong::StateMachine::SIGNAL_MSG_SIZE;

impl Bouncer {
    fn sm(&self) -> PingPong::Sender<'_, BOUNCER_MSG_SIZE> {
        PingPong::Sender(SmSender::new(
            &self.queued,
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
            0,
            0,
            QueueFullPolicy::Assert,
        ))
    }
}

impl ComponentDispatch for Bouncer {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        if msg_type != Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL {
            return MsgDispatchStatus::Error;
        }
        let (_, signal) = read_sm_message(msg);
        self.sm_ping_pong.dispatch_signal(self, signal, msg);
        MsgDispatchStatus::Ok
    }
}

impl PingPong::Handlers for Bouncer {
    fn action_bounce(&self, _: FwEnumStoreType, _: PingPong::Signal) {
        // The machine's state is readable mid-transition (old state).
        let _ = self.sm_ping_pong.get_state();
        if self.bounces.fetch_add(1, Ordering::Relaxed) < 4 {
            self.sm().send_signal_ping();
        }
    }
}

#[test]
fn actions_send_signals_through_the_queue() {
    let c = Bouncer {
        queued: QueuedBase::new("bouncer"),
        sm_ping_pong: PingPong::StateMachine::new(),
        bounces: AtomicU32::new(0),
    };
    c.queued
        .create_queue(QUEUE_DEPTH, BOUNCER_MSG_SIZE as FwSizeType);
    c.sm_ping_pong.init(0, &c);
    c.sm().send_signal_ping();
    let mut dispatched = 0;
    while c.queued.do_dispatch(&c, BlockingType::NonBlocking) == MsgDispatchStatus::Ok {
        dispatched += 1;
    }
    assert_eq!(dispatched, 5);
    assert_eq!(c.bounces.load(Ordering::Relaxed), 5);
    assert_eq!(c.sm_ping_pong.get_state(), PingPong::State::B);
}
