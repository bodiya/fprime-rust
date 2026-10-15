//! State machine instances in components generated from upstream's FppTest
//! models (`fpp/sm/FppTest/state_machine/internal_instance` and
//! `external_instance`), ported from the FppTest testers like
//! fprime-comp's hand-written `sm_instance.rs` / `sm_external.rs`: the
//! generated `SmId`s, senders (priority, queue-full policy, overflow hook),
//! `init` order and dispatch on the component's queue, and the C++ queue
//! messages byte for byte.

use fprime_comp::os::queue::{BlockingType, Status as QueueStatus};
use fprime_comp::{MsgDispatchStatus, QueuedBase};
use fprime_config::{FwQueuePriorityType, FwSizeType};
use fprime_fpp_demo::fpptest::{Basic, GuardAbs, InputPair, SmTest};
use fprime_fpp_demo::sm_models::FppTest::SmChoice::InputPairU16U32 as SmChoiceInputPair;
use fprime_fpp_demo::sm_models::FppTest::SmInstanceChoice::{
    InputPairU16U32Base, InputPairU16U32Component, InputPairU16U32Handlers,
};
use fprime_fpp_demo::sm_models::FppTest::SmInstanceState::Basic as BasicLocal;
use fprime_fpp_demo::sm_models::FppTest::SmInstanceState::{
    BasicBase, BasicComponent, BasicGuardTestAbsTypeBase, BasicGuardTestAbsTypeComponent,
    BasicGuardTestAbsTypeHandlers, BasicHandlers,
};
use fprime_fpp_demo::sm_models::FppTest::SmState::{
    Basic as SmStateBasic, BasicGuardTestAbsType as SmGuardAbs,
};
use fprime_fpp_demo::sm_models::FppTest::{SmTestBase, SmTestComponent, SmTestHandlers};
use fprime_fpp_demo::sm_support::{
    DeviceSm_Signals, DeviceSm_States, HackSm_Signals, HackSm_States, TestAbsType,
};
use fprime_fw::{SerBuf, SmSignalBuffer};
use std::sync::atomic::Ordering;

const QUEUE_DEPTH: FwSizeType = 10;

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

// ---------------------------------------------------------------------------
// SmInstanceState.Basic (queued)
// ---------------------------------------------------------------------------

fn basic_dispatch(c: &std::sync::Arc<Basic>) -> MsgDispatchStatus {
    c.base().queued.do_dispatch(&**c, BlockingType::NonBlocking)
}

#[test]
fn basic_ids_message_types_and_size_follow_the_cpp_rules() {
    // SmId: every instance sorted by name.
    assert_eq!(BasicBase::SM_ID_BASIC1, 0);
    assert_eq!(BasicBase::SM_ID_BASIC2, 1);
    assert_eq!(BasicBase::SM_ID_SM_STATE_BASIC1, 2);
    assert_eq!(BasicBase::SM_ID_SM_STATE_BASIC2, 3);
    // The only queued message: the internal signal.
    assert_eq!(BasicBase::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL, 1);
    // DATA_OFFSET (4 + 2) + 2 * sizeof(FwEnumStoreType).
    assert_eq!(BasicBase::MSG_SIZE, 14);
}

#[test]
fn basic_signals_run_on_dispatch() {
    let c = Basic::new("basic");
    c.init(QUEUE_DEPTH);
    let b = c.base();
    assert_eq!(b.basic1_get_state(), BasicLocal::Basic::State::S);
    assert_eq!(b.sm_state_basic1_get_state(), SmStateBasic::State::S);
    assert_eq!(c.basic1_a.size(), 0);

    c.basic1_send_signal_s();
    // Nothing happens until the message is dispatched.
    assert_eq!(b.basic1_get_state(), BasicLocal::Basic::State::S);
    assert_eq!(basic_dispatch(&c), MsgDispatchStatus::Ok);
    assert_eq!(b.basic1_get_state(), BasicLocal::Basic::State::T);
    assert_eq!(c.basic1_a.signals(), vec![BasicLocal::Basic::Signal::s; 6]);
    assert_eq!(c.basic2_a.size(), 0);

    c.basic2_send_signal_s();
    c.sm_state_basic1_send_signal_s();
    c.sm_state_basic2_send_signal_s();
    for _ in 0..3 {
        assert_eq!(basic_dispatch(&c), MsgDispatchStatus::Ok);
    }
    assert_eq!(b.basic2_get_state(), BasicLocal::Basic::State::T);
    assert_eq!(b.sm_state_basic1_get_state(), SmStateBasic::State::T);
    assert_eq!(b.sm_state_basic2_get_state(), SmStateBasic::State::T);
    assert_eq!(c.basic2_a.signals(), vec![BasicLocal::Basic::Signal::s; 6]);
    assert_eq!(
        c.sm_state_basic1_a.signals(),
        vec![SmStateBasic::Signal::s; 6]
    );
    assert_eq!(
        c.sm_state_basic2_a.signals(),
        vec![SmStateBasic::Signal::s; 6]
    );
    assert_eq!(basic_dispatch(&c), MsgDispatchStatus::Empty);
}

#[test]
fn basic_signal_message_is_byte_exact() {
    let c = Basic::new("basic");
    c.init(QUEUE_DEPTH);
    c.sm_state_basic2_send_signal_s();
    let (bytes, priority) = take_raw(&c.base().queued, BasicBase::MSG_SIZE);
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
fn basic_priorities_order_dispatch() {
    let c = Basic::new("basic");
    c.init(QUEUE_DEPTH);
    let b = c.base();
    // basic1: priority 0, smStateBasic1: 1, smStateBasic2: 2.
    c.basic1_send_signal_s();
    c.sm_state_basic1_send_signal_s();
    c.sm_state_basic2_send_signal_s();
    assert_eq!(basic_dispatch(&c), MsgDispatchStatus::Ok);
    assert_eq!(b.sm_state_basic2_get_state(), SmStateBasic::State::T);
    assert_eq!(b.sm_state_basic1_get_state(), SmStateBasic::State::S);
    assert_eq!(basic_dispatch(&c), MsgDispatchStatus::Ok);
    assert_eq!(b.sm_state_basic1_get_state(), SmStateBasic::State::T);
    assert_eq!(b.basic1_get_state(), BasicLocal::Basic::State::S);
    assert_eq!(basic_dispatch(&c), MsgDispatchStatus::Ok);
    assert_eq!(b.basic1_get_state(), BasicLocal::Basic::State::T);
}

#[test]
fn basic_drop_policy_counts_dropped_signals() {
    let c = Basic::new("basic");
    c.init(QUEUE_DEPTH);
    for _ in 0..QUEUE_DEPTH {
        c.sm_state_basic2_send_signal_s();
    }
    assert_eq!(c.base().queued.get_num_msgs_dropped(), 0);
    c.sm_state_basic2_send_signal_s();
    assert_eq!(c.base().queued.get_num_msgs_dropped(), 1);
}

#[test]
#[should_panic]
fn basic_assert_policy_asserts_on_overflow() {
    let c = Basic::new("basic");
    c.init(QUEUE_DEPTH);
    for _ in 0..=QUEUE_DEPTH {
        c.sm_state_basic1_send_signal_s();
    }
}

#[test]
#[should_panic]
fn basic_unknown_sm_id_asserts() {
    let c = Basic::new("basic");
    c.init(QUEUE_DEPTH);
    SmStateBasic::Sender(
        fprime_comp::sm::SmSender::<'_, { BasicBase::MSG_SIZE }>::new(
            &c.base().queued,
            BasicBase::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
            7,
            0,
            fprime_comp::QueueFullPolicy::Assert,
        ),
    )
    .send_signal_s();
    let _ = basic_dispatch(&c);
}

// ---------------------------------------------------------------------------
// SmInstanceState.BasicGuardTestAbsType (active, `priority 4 hook`)
// ---------------------------------------------------------------------------

fn guard_abs(guard: bool) {
    let c = GuardAbs::new("guardAbs");
    c.guard.store(guard, Ordering::Relaxed);
    c.init(QUEUE_DEPTH);
    let b = c.base();
    assert_eq!(
        b.sm_state_basic_guard_test_abs_type_get_state(),
        SmGuardAbs::State::S
    );
    let value = TestAbsType::new(41);
    c.sm_state_basic_guard_test_abs_type_send_signal_s(&value);
    assert_eq!(
        b.active.queued.do_dispatch(&*c, BlockingType::NonBlocking),
        MsgDispatchStatus::Ok
    );
    assert_eq!(c.g.signals(), vec![SmGuardAbs::Signal::s]);
    assert_eq!(c.g.values(), vec![value]);
    if guard {
        assert_eq!(
            b.sm_state_basic_guard_test_abs_type_get_state(),
            SmGuardAbs::State::T
        );
        assert_eq!(c.a.values(), vec![value]);
    } else {
        assert_eq!(
            b.sm_state_basic_guard_test_abs_type_get_state(),
            SmGuardAbs::State::S
        );
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
fn guard_abs_overflow_calls_the_hook_with_the_message() {
    let c = GuardAbs::new("guardAbs");
    c.init(QUEUE_DEPTH);
    let value = TestAbsType::new(42);
    for _ in 0..QUEUE_DEPTH {
        c.sm_state_basic_guard_test_abs_type_send_signal_s(&value);
    }
    assert!(c.overflows.lock().unwrap().is_empty());
    c.sm_state_basic_guard_test_abs_type_send_signal_s(&value);
    assert_eq!(
        *c.overflows.lock().unwrap(),
        [(SmGuardAbs::Signal::s.as_repr(), value)]
    );
}

#[test]
fn guard_abs_signal_message_is_byte_exact() {
    let c = GuardAbs::new("guardAbs");
    c.init(QUEUE_DEPTH);
    // DATA_OFFSET + 2 * sizeof(FwEnumStoreType) + TestAbsType::SERIALIZED_SIZE.
    assert_eq!(BasicGuardTestAbsTypeBase::MSG_SIZE, 6 + 8 + 4);
    c.sm_state_basic_guard_test_abs_type_send_signal_s(&TestAbsType::new(0x0A0B_0C0D));
    let (bytes, priority) = take_raw(&c.base().active.queued, BasicGuardTestAbsTypeBase::MSG_SIZE);
    assert_eq!(priority, 4);
    assert_eq!(
        bytes,
        [
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0x0A, 0x0B, 0x0C, 0x0D
        ]
    );
}

/// The component's own thread dispatches the signal.
#[test]
fn guard_abs_dispatches_on_the_component_thread() {
    let c = GuardAbs::new("guardAbs");
    c.guard.store(true, Ordering::Relaxed);
    c.init(QUEUE_DEPTH);
    c.base().active.start(
        &c,
        0,
        fprime_comp::os::task::TASK_DEFAULT,
        fprime_comp::os::task::TASK_DEFAULT,
    );
    c.sm_state_basic_guard_test_abs_type_send_signal_s(&TestAbsType::new(5));
    c.base().active.exit();
    let _ = c.base().active.join();
    assert_eq!(
        c.base().sm_state_basic_guard_test_abs_type_get_state(),
        SmGuardAbs::State::T
    );
    assert_eq!(c.a.values(), vec![TestAbsType::new(5)]);
}

// ---------------------------------------------------------------------------
// SmInstanceChoice.InputPairU16U32 (active, `priority 3 drop`)
// ---------------------------------------------------------------------------

fn input_pair(use_s1: bool, guard: bool) {
    use SmChoiceInputPair::{Signal, State};
    let c = InputPair::new("inputPair");
    c.guard.store(guard, Ordering::Relaxed);
    c.init(QUEUE_DEPTH);
    let b = c.base();
    assert_eq!(b.sm_choice_input_pair_u16_u32_get_state(), State::S1);
    let (signal, value) = if use_s1 {
        c.sm_choice_input_pair_u16_u32_send_signal_s1(0xBEEF);
        (Signal::s1, 0xBEEF_u32)
    } else {
        c.sm_choice_input_pair_u16_u32_send_signal_s2(0x1234_5678);
        (Signal::s2, 0x1234_5678)
    };
    assert_eq!(
        b.active.queued.do_dispatch(&*c, BlockingType::NonBlocking),
        MsgDispatchStatus::Ok
    );
    // The U16 value reaches the U32 choice converted.
    assert_eq!(c.g.signals(), vec![signal]);
    assert_eq!(c.g.values(), vec![value]);
    assert_eq!(c.a.values(), vec![value]);
    assert_eq!(
        b.sm_choice_input_pair_u16_u32_get_state(),
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
    let c = InputPair::new("inputPair");
    c.init(QUEUE_DEPTH);
    // The message is sized for the largest signal value (U32).
    assert_eq!(InputPairU16U32Base::MSG_SIZE, 6 + 8 + 4);
    c.sm_choice_input_pair_u16_u32_send_signal_s1(0xBEEF);
    let (bytes, priority) = take_raw(&c.base().active.queued, InputPairU16U32Base::MSG_SIZE);
    assert_eq!(priority, 3);
    assert_eq!(
        bytes,
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0xBE, 0xEF]
    );
}

// ---------------------------------------------------------------------------
// SmTest (external_instance)
// ---------------------------------------------------------------------------

fn dispatch_all(c: &std::sync::Arc<SmTest>) {
    while c.base().active.queued.queue().get_messages_available() > 0 {
        assert_eq!(
            c.base()
                .active
                .queued
                .do_dispatch(&**c, BlockingType::NonBlocking),
            MsgDispatchStatus::Ok
        );
    }
}

fn states(c: &SmTest) -> [i32; 5] {
    let b = c.base();
    [
        b.device1.state() as i32,
        b.device2.state() as i32,
        b.device3.state() as i32,
        b.device4.state() as i32,
        b.device5.state() as i32,
    ]
}

#[test]
fn external_ids_and_message_types() {
    // schedIn, then the external signal.
    assert_eq!(SmTestBase::MSG_TYPE_SCHED_IN, 1);
    assert_eq!(SmTestBase::MSG_TYPE_EXTERNAL_STATE_MACHINE_SIGNAL, 2);
    assert_eq!(SmTestBase::SM_ID_DEVICE1, 0);
    assert_eq!(SmTestBase::SM_ID_DEVICE5, 4);
    // DATA_OFFSET + 2 * sizeof(FwEnumStoreType) + SmSignalBuffer::SERIALIZED_SIZE.
    assert_eq!(SmTestBase::MSG_SIZE, 6 + 8 + 2 + 128);
}

#[test]
fn external_init_runs_every_machine_before_the_queue() {
    let c = SmTest::new("smTest");
    c.init(QUEUE_DEPTH);
    // SmId order: device1 .. device5, each `turnOff`.
    assert_eq!(
        *c.calls.lock().unwrap(),
        [
            ("DeviceSm_turnOff", 0),
            ("DeviceSm_turnOff", 1),
            ("HackSm_turnOff", 2),
            ("HackSm_turnOff", 3),
            ("HackSm_turnOff", 4),
        ]
    );
    assert_eq!(states(&c), [0; 5]);
}

#[test]
fn external_sched_in_toggles_every_machine() {
    let c = SmTest::new("smTest");
    c.init(QUEUE_DEPTH);
    let sched = c.sched_in(0);
    sched.target.invoke(sched.port_num, 0);
    dispatch_all(&c);
    assert_eq!(c.ticks.load(Ordering::Relaxed), 1);
    assert_eq!(states(&c), [1; 5]);
    sched.target.invoke(sched.port_num, 0);
    dispatch_all(&c);
    assert_eq!(states(&c), [0; 5]);

    let data = SmSignalBuffer::new();
    c.device3_state_machine_invoke(HackSm_Signals::CHECK_SIG, &data);
    dispatch_all(&c);
    assert_eq!(c.base().device3.state(), HackSm_States::DIAG);
    sched.target.invoke(sched.port_num, 0);
    dispatch_all(&c);
    assert_eq!(c.base().device3.state(), HackSm_States::OFF);
}

#[test]
fn external_machines_call_back_with_their_ids() {
    let c = SmTest::new("smTest");
    c.init(QUEUE_DEPTH);
    c.calls.lock().unwrap().clear();
    c.device2_state_machine_invoke(DeviceSm_Signals::RTI_SIG, &SmSignalBuffer::new());
    dispatch_all(&c);
    assert_eq!(
        *c.calls.lock().unwrap(),
        [("DeviceSm_a1", 1), ("DeviceSm_turnOn", 1)]
    );
    assert_eq!(c.base().device2.state(), DeviceSm_States::ON);
}

#[test]
fn external_signal_message_is_byte_exact() {
    let c = SmTest::new("smTest");
    c.init(QUEUE_DEPTH);
    let mut data = SmSignalBuffer::new();
    assert!(data.serialize_u16_be(0xABCD).is_ok());
    c.device3_state_machine_invoke(HackSm_Signals::CHECK_SIG, &data);
    let (bytes, priority) = take_raw(&c.base().active.queued, SmTestBase::MSG_SIZE);
    assert_eq!(priority, 3);
    assert_eq!(
        bytes,
        [
            0, 0, 0, 2, // EXTERNAL_STATE_MACHINE_SIGNAL
            0, 0, // port number 0
            0, 0, 0, 2, // SmId::device3
            0, 0, 0, 1, // CHECK_SIG
            0, 2, 0xAB, 0xCD, // the data buffer, length-prefixed
        ]
    );
}

#[test]
fn external_hook_policy_calls_the_overflow_hook() {
    let c = SmTest::new("smTest");
    c.init(2);
    let mut data = SmSignalBuffer::new();
    for b in [1u8, 2, 3] {
        assert!(data.serialize_u8_be(b).is_ok());
    }
    c.device4_state_machine_invoke(HackSm_Signals::CHECK_SIG, &data);
    c.device4_state_machine_invoke(HackSm_Signals::CHECK_SIG, &data);
    assert!(c.hook_calls.lock().unwrap().is_empty());
    c.device4_state_machine_invoke(HackSm_Signals::CHECK_SIG, &data);
    assert_eq!(*c.hook_calls.lock().unwrap(), [(1, vec![1, 2, 3])]);
}

#[test]
fn external_drop_policy_counts_dropped_signals() {
    let c = SmTest::new("smTest");
    c.init(1);
    let data = SmSignalBuffer::new();
    c.device3_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
    c.device3_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
    assert_eq!(c.base().active.queued.get_num_msgs_dropped(), 1);
}

#[test]
fn unknown_external_sm_id_is_a_dispatch_error() {
    let c = SmTest::new("smTest");
    c.init(2);
    fprime_comp::ExternalSmSender::<'_, { SmTestBase::MSG_SIZE }>::new(
        &c.base().active.queued,
        SmTestBase::MSG_TYPE_EXTERNAL_STATE_MACHINE_SIGNAL,
        9,
        0,
        fprime_comp::QueueFullPolicy::Assert,
    )
    .invoke(0, &SmSignalBuffer::new());
    assert_eq!(
        c.base()
            .active
            .queued
            .do_dispatch(&*c, BlockingType::NonBlocking),
        MsgDispatchStatus::Error
    );
}

/// The handler traits are what the components implement (used above
/// through the generated component traits).
#[allow(dead_code)]
fn handler_traits_are_implemented(
    _: &dyn BasicHandlers,
    _: &dyn BasicGuardTestAbsTypeHandlers,
    _: &dyn InputPairU16U32Handlers,
    _: &dyn SmTestHandlers,
) {
}
