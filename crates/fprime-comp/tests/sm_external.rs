//! Port of `FppTest/state_machine/external_instance`: external (STARS
//! style) state machines `DeviceSm` and `HackSm` instantiated five times in
//! the active component `SmTest`, with every FPP queue-full policy. A
//! signal to an external machine is the `EXTERNAL_STATE_MACHINE_SIGNAL`
//! queue message `[msg_type][port 0][sm_id][signal][u16 size][data]`,
//! dispatched to the machine's `update(id, signal, data)` on the component
//! thread.
#![allow(non_snake_case, non_camel_case_types, clippy::upper_case_acronyms)]

use fprime_comp::config::{FwEnumStoreType, FwIndexType, FwQueuePriorityType, FwSizeType};
use fprime_comp::fw::{SerBufAny, SmSignalBuffer};
use fprime_comp::os::queue::{BlockingType, Status as QueueStatus};
use fprime_comp::sm::{self, ExternalSmOverflowHook, ExternalStateMachine};
use fprime_comp::{
    ActiveBase, ComponentDispatch, ExternalSmSender, MsgDispatchStatus, QueueFullPolicy, SchedPort,
    async_input_port_adapter, component_msg_types, component_sm_ids, msg,
};
use fprime_fw::SerBuf;
use std::sync::atomic::{AtomicI32, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// DeviceSm (DeviceSm.hpp/.cpp, as STARS generates it)
// ---------------------------------------------------------------------------

/// `DeviceSm_Interface`.
trait DeviceSm_Interface {
    fn DeviceSm_g1(&self, sm_id: FwEnumStoreType) -> bool;
    fn DeviceSm_g2(
        &self,
        sm_id: FwEnumStoreType,
        signal: DeviceSm_Signals,
        data: &SmSignalBuffer,
    ) -> bool;
    fn DeviceSm_turnOff(&self, sm_id: FwEnumStoreType);
    fn DeviceSm_a1(&self, sm_id: FwEnumStoreType, signal: DeviceSm_Signals, data: &SmSignalBuffer);
    fn DeviceSm_turnOn(&self, sm_id: FwEnumStoreType);
    fn DeviceSm_a2(&self, sm_id: FwEnumStoreType);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DeviceSm_Signals {
    RTI_SIG = 0,
}

impl From<DeviceSm_Signals> for FwEnumStoreType {
    fn from(s: DeviceSm_Signals) -> Self {
        s as FwEnumStoreType
    }
}

impl TryFrom<FwEnumStoreType> for DeviceSm_Signals {
    type Error = FwEnumStoreType;
    fn try_from(v: FwEnumStoreType) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(Self::RTI_SIG),
            other => Err(other),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DeviceSm_States {
    OFF = 0,
    ON = 1,
}

#[derive(Default)]
struct DeviceSm {
    state: AtomicI32,
}

impl DeviceSm {
    fn state(&self) -> DeviceSm_States {
        match self.state.load(Ordering::Relaxed) {
            0 => DeviceSm_States::OFF,
            _ => DeviceSm_States::ON,
        }
    }

    fn set(&self, s: DeviceSm_States) {
        self.state.store(s as i32, Ordering::Relaxed);
    }
}

impl ExternalStateMachine for DeviceSm {
    type Signal = DeviceSm_Signals;
    type Parent = dyn DeviceSm_Interface;

    fn init(&self, parent: &Self::Parent, sm_id: FwEnumStoreType) {
        parent.DeviceSm_turnOff(sm_id);
        self.set(DeviceSm_States::OFF);
    }

    fn update(
        &self,
        parent: &Self::Parent,
        sm_id: FwEnumStoreType,
        signal: DeviceSm_Signals,
        data: &SmSignalBuffer,
    ) {
        match self.state() {
            DeviceSm_States::OFF => match signal {
                DeviceSm_Signals::RTI_SIG => {
                    if parent.DeviceSm_g1(sm_id) {
                        parent.DeviceSm_a1(sm_id, signal, data);
                        parent.DeviceSm_turnOn(sm_id);
                        self.set(DeviceSm_States::ON);
                    }
                }
            },
            DeviceSm_States::ON => match signal {
                DeviceSm_Signals::RTI_SIG => {
                    if parent.DeviceSm_g2(sm_id, signal, data) {
                        parent.DeviceSm_a2(sm_id);
                        parent.DeviceSm_turnOff(sm_id);
                        self.set(DeviceSm_States::OFF);
                    }
                }
            },
        }
    }
}

// ---------------------------------------------------------------------------
// HackSm (HackSm.hpp/.cpp)
// ---------------------------------------------------------------------------

/// `HackSm_Interface`.
trait HackSm_Interface {
    fn HackSm_turnOff(&self, sm_id: FwEnumStoreType);
    fn HackSm_turnOn(&self, sm_id: FwEnumStoreType);
    fn HackSm_doDiag(&self, sm_id: FwEnumStoreType);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HackSm_Signals {
    RTI_SIG = 0,
    CHECK_SIG = 1,
}

impl From<HackSm_Signals> for FwEnumStoreType {
    fn from(s: HackSm_Signals) -> Self {
        s as FwEnumStoreType
    }
}

impl TryFrom<FwEnumStoreType> for HackSm_Signals {
    type Error = FwEnumStoreType;
    fn try_from(v: FwEnumStoreType) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(Self::RTI_SIG),
            1 => Ok(Self::CHECK_SIG),
            other => Err(other),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HackSm_States {
    OFF = 0,
    ON = 1,
    DIAG = 2,
}

#[derive(Default)]
struct HackSm {
    state: AtomicI32,
}

impl HackSm {
    fn state(&self) -> HackSm_States {
        match self.state.load(Ordering::Relaxed) {
            0 => HackSm_States::OFF,
            1 => HackSm_States::ON,
            _ => HackSm_States::DIAG,
        }
    }

    fn set(&self, s: HackSm_States) {
        self.state.store(s as i32, Ordering::Relaxed);
    }
}

impl ExternalStateMachine for HackSm {
    type Signal = HackSm_Signals;
    type Parent = dyn HackSm_Interface;

    fn init(&self, parent: &Self::Parent, sm_id: FwEnumStoreType) {
        parent.HackSm_turnOff(sm_id);
        self.set(HackSm_States::OFF);
    }

    fn update(
        &self,
        parent: &Self::Parent,
        sm_id: FwEnumStoreType,
        signal: HackSm_Signals,
        _data: &SmSignalBuffer,
    ) {
        match (self.state(), signal) {
            (HackSm_States::OFF, HackSm_Signals::RTI_SIG) => {
                parent.HackSm_turnOn(sm_id);
                self.set(HackSm_States::ON);
            }
            (HackSm_States::OFF | HackSm_States::ON, HackSm_Signals::CHECK_SIG) => {
                parent.HackSm_doDiag(sm_id);
                self.set(HackSm_States::DIAG);
            }
            (HackSm_States::ON | HackSm_States::DIAG, HackSm_Signals::RTI_SIG) => {
                parent.HackSm_turnOff(sm_id);
                self.set(HackSm_States::OFF);
            }
            (HackSm_States::DIAG, HackSm_Signals::CHECK_SIG) => {}
        }
    }
}

// ---------------------------------------------------------------------------
// SmTest
// ---------------------------------------------------------------------------

/// `active component SmTest` with `async input port schedIn: Svc.Sched`
/// and `device1: DeviceSm priority 1 block`, `device2: DeviceSm priority 2
/// assert`, `device3: HackSm priority 3 drop`, `device4: HackSm priority 4
/// hook`, `device5: HackSm`.
struct SmTest {
    active: ActiveBase,
    device1: DeviceSm,
    device2: DeviceSm,
    device3: HackSm,
    device4: HackSm,
    device5: HackSm,
    // Tester state.
    calls: Mutex<Vec<(&'static str, FwEnumStoreType)>>,
    hook_calls: AtomicU32,
}

component_msg_types! {
    impl SmTest {
        MSG_TYPE_SCHEDIN_SCHED,
        MSG_TYPE_EXTERNAL_STATE_MACHINE_SIGNAL,
    }
}

component_sm_ids! {
    impl SmTest {
        SM_ID_DEVICE1,
        SM_ID_DEVICE2,
        SM_ID_DEVICE3,
        SM_ID_DEVICE4,
        SM_ID_DEVICE5,
    }
}

/// max(schedIn: header + U32, the external signal message).
const SM_TEST_MSG_SIZE: usize = sm::EXTERNAL_SIGNAL_MSG_SIZE;

async_input_port_adapter! {
    /// `schedIn` — ASYNC `Svc.Sched` input.
    component: SmTest;
    adapter: SchedInAdapter;
    port: SchedPort;
    input: sched_in;
    deserialize: sched_in_deserialize;
    handler: sched_in_handler;
    base: active.queued;
    msg_type: SmTest::MSG_TYPE_SCHEDIN_SCHED;
    msg_size: SM_TEST_MSG_SIZE;
    priority: 0;
    queue_full: QueueFullPolicy::Assert;
    args { val context: u32 }
}

impl SmTest {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new("smTest"),
            device1: DeviceSm::default(),
            device2: DeviceSm::default(),
            device3: HackSm::default(),
            device4: HackSm::default(),
            device5: HackSm::default(),
            calls: Mutex::new(Vec::new()),
            hook_calls: AtomicU32::new(0),
        })
    }

    fn init(&self, queue_depth: FwSizeType) {
        // C++ order: the machines are initialized before the queue exists.
        self.device1.init(self, Self::SM_ID_DEVICE1);
        self.device2.init(self, Self::SM_ID_DEVICE2);
        self.device3.init(self, Self::SM_ID_DEVICE3);
        self.device4.init(self, Self::SM_ID_DEVICE4);
        self.device5.init(self, Self::SM_ID_DEVICE5);
        self.active
            .queued
            .create_queue(queue_depth, SM_TEST_MSG_SIZE as FwSizeType);
    }

    fn sender(
        &self,
        sm_id: FwEnumStoreType,
        priority: FwQueuePriorityType,
        policy: QueueFullPolicy,
    ) -> ExternalSmSender<'_, SM_TEST_MSG_SIZE> {
        ExternalSmSender::new(
            &self.active.queued,
            Self::MSG_TYPE_EXTERNAL_STATE_MACHINE_SIGNAL,
            sm_id,
            priority,
            policy,
        )
    }

    // C++ `<m>_stateMachineInvoke(signal, data)`.
    fn device1_state_machine_invoke(&self, signal: DeviceSm_Signals, data: &SmSignalBuffer) {
        self.sender(Self::SM_ID_DEVICE1, 1, QueueFullPolicy::Block)
            .invoke(signal.into(), data);
    }

    fn device2_state_machine_invoke(&self, signal: DeviceSm_Signals, data: &SmSignalBuffer) {
        self.sender(Self::SM_ID_DEVICE2, 2, QueueFullPolicy::Assert)
            .invoke(signal.into(), data);
    }

    fn device3_state_machine_invoke(&self, signal: HackSm_Signals, data: &SmSignalBuffer) {
        self.sender(Self::SM_ID_DEVICE3, 3, QueueFullPolicy::Drop)
            .invoke(signal.into(), data);
    }

    fn device4_state_machine_invoke(&self, signal: HackSm_Signals, data: &SmSignalBuffer) {
        self.sender(Self::SM_ID_DEVICE4, 4, QueueFullPolicy::Hook)
            .with_overflow_hook(self)
            .invoke(signal.into(), data);
    }

    fn device5_state_machine_invoke(&self, signal: HackSm_Signals, data: &SmSignalBuffer) {
        self.sender(Self::SM_ID_DEVICE5, 0, QueueFullPolicy::Assert)
            .invoke(signal.into(), data);
    }

    /// `SmTest::schedIn_handler`: RTI to every machine.
    fn sched_in_handler(&self, _port_num: FwIndexType, _context: u32) {
        let data = SmSignalBuffer::new();
        self.device1_state_machine_invoke(DeviceSm_Signals::RTI_SIG, &data);
        self.device2_state_machine_invoke(DeviceSm_Signals::RTI_SIG, &data);
        self.device3_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
        self.device4_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
        self.device5_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
    }

    fn record(&self, what: &'static str, sm_id: FwEnumStoreType) {
        self.calls.lock().unwrap().push((what, sm_id));
    }

    /// `SmTestTester::dispatchAll`.
    fn dispatch_all(&self) {
        while self.active.queued.queue().get_messages_available() > 0 {
            let status = self
                .active
                .queued
                .do_dispatch(self, BlockingType::NonBlocking);
            assert_eq!(status, MsgDispatchStatus::Ok);
        }
    }

    fn states(&self) -> [i32; 5] {
        [
            self.device1.state() as i32,
            self.device2.state() as i32,
            self.device3.state() as i32,
            self.device4.state() as i32,
            self.device5.state() as i32,
        ]
    }
}

impl ComponentDispatch for SmTest {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(msg, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            Self::MSG_TYPE_SCHEDIN_SCHED => match Self::sched_in_deserialize(msg) {
                Some((context,)) => {
                    self.sched_in_handler(port_num, context);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_EXTERNAL_STATE_MACHINE_SIGNAL => {
                let mut data = SmSignalBuffer::new();
                let (sm_id, signal) = sm::read_external_signal(msg, &mut data);
                match sm_id {
                    Self::SM_ID_DEVICE1 => self.device1.dispatch(self, sm_id, signal, &data),
                    Self::SM_ID_DEVICE2 => self.device2.dispatch(self, sm_id, signal, &data),
                    Self::SM_ID_DEVICE3 => self.device3.dispatch(self, sm_id, signal, &data),
                    Self::SM_ID_DEVICE4 => self.device4.dispatch(self, sm_id, signal, &data),
                    Self::SM_ID_DEVICE5 => self.device5.dispatch(self, sm_id, signal, &data),
                    // C++ parity: an unknown external id is a dispatch error.
                    _ => MsgDispatchStatus::Error,
                }
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl DeviceSm_Interface for SmTest {
    fn DeviceSm_g1(&self, _: FwEnumStoreType) -> bool {
        true
    }
    fn DeviceSm_g2(&self, _: FwEnumStoreType, _: DeviceSm_Signals, _: &SmSignalBuffer) -> bool {
        true
    }
    fn DeviceSm_turnOff(&self, sm_id: FwEnumStoreType) {
        self.record("DeviceSm_turnOff", sm_id);
    }
    fn DeviceSm_a1(&self, sm_id: FwEnumStoreType, _: DeviceSm_Signals, _: &SmSignalBuffer) {
        self.record("DeviceSm_a1", sm_id);
    }
    fn DeviceSm_turnOn(&self, sm_id: FwEnumStoreType) {
        self.record("DeviceSm_turnOn", sm_id);
    }
    fn DeviceSm_a2(&self, sm_id: FwEnumStoreType) {
        self.record("DeviceSm_a2", sm_id);
    }
}

impl HackSm_Interface for SmTest {
    fn HackSm_turnOff(&self, sm_id: FwEnumStoreType) {
        self.record("HackSm_turnOff", sm_id);
    }
    fn HackSm_turnOn(&self, sm_id: FwEnumStoreType) {
        self.record("HackSm_turnOn", sm_id);
    }
    fn HackSm_doDiag(&self, sm_id: FwEnumStoreType) {
        self.record("HackSm_doDiag", sm_id);
    }
}

/// C++ `device4_stateMachineOverflowHook(signal, data)`.
impl ExternalSmOverflowHook for SmTest {
    fn external_sm_overflow_hook(
        &self,
        sm_id: FwEnumStoreType,
        signal: FwEnumStoreType,
        data: &SmSignalBuffer,
    ) {
        assert_eq!(sm_id, Self::SM_ID_DEVICE4);
        assert_eq!(signal, HackSm_Signals::CHECK_SIG as i32);
        assert_eq!(data.as_slice(), &[1, 2, 3]);
        self.hook_calls.fetch_add(1, Ordering::Relaxed);
    }
}

const OFF: i32 = 0;
const ON: i32 = 1;
const DIAG: i32 = 2;

#[test]
fn sched_in_ok() {
    let c = SmTest::new();
    c.init(10);
    assert_eq!(c.states(), [OFF; 5]);
    let sched = c.sched_in(0);
    sched.target.invoke(sched.port_num, 0);
    c.dispatch_all();
    assert_eq!(c.states(), [ON; 5]);
    sched.target.invoke(sched.port_num, 0);
    c.dispatch_all();
    assert_eq!(c.states(), [OFF; 5]);

    let data = SmSignalBuffer::new();
    c.device3_state_machine_invoke(HackSm_Signals::CHECK_SIG, &data);
    c.dispatch_all();
    assert_eq!(c.device3.state(), HackSm_States::DIAG);
    assert_eq!(c.device3.state() as i32, DIAG);
    sched.target.invoke(sched.port_num, 0);
    c.dispatch_all();
    assert_eq!(c.device3.state(), HackSm_States::OFF);
}

#[test]
fn external_machines_call_back_in_order() {
    let c = SmTest::new();
    c.init(10);
    c.calls.lock().unwrap().clear();
    c.device1_state_machine_invoke(DeviceSm_Signals::RTI_SIG, &SmSignalBuffer::new());
    c.dispatch_all();
    assert_eq!(
        *c.calls.lock().unwrap(),
        [("DeviceSm_a1", 0), ("DeviceSm_turnOn", 0)]
    );
    assert_eq!(c.device1.state(), DeviceSm_States::ON);
}

#[test]
fn external_signal_message_is_byte_exact() {
    let c = SmTest::new();
    c.init(10);
    // DATA_OFFSET + 2 * sizeof(FwEnumStoreType) + SmSignalBuffer::SERIALIZED_SIZE.
    assert_eq!(SM_TEST_MSG_SIZE, 6 + 8 + 2 + 128);
    let mut data = SmSignalBuffer::new();
    assert!(data.serialize_u16_be(0xABCD).is_ok());
    c.device3_state_machine_invoke(HackSm_Signals::CHECK_SIG, &data);
    let mut bytes = vec![0u8; SM_TEST_MSG_SIZE];
    let mut actual: FwSizeType = 0;
    let mut priority: FwQueuePriorityType = 0;
    let status = c.active.queued.queue().receive(
        &mut bytes,
        BlockingType::NonBlocking,
        &mut actual,
        &mut priority,
    );
    assert_eq!(status, QueueStatus::OpOk);
    assert_eq!(priority, 3);
    assert_eq!(
        &bytes[..actual as usize],
        &[
            0, 0, 0, 2, // EXTERNAL_STATE_MACHINE_SIGNAL
            0, 0, // port number 0
            0, 0, 0, 2, // SmId::device3
            0, 0, 0, 1, // CHECK_SIG
            0, 2, 0xAB, 0xCD, // the data buffer, length-prefixed
        ]
    );
}

#[test]
fn external_signal_data_reaches_update() {
    struct Echo(Mutex<Vec<u8>>);
    impl ExternalStateMachine for Echo {
        type Signal = HackSm_Signals;
        type Parent = ();
        fn init(&self, _: &(), _: FwEnumStoreType) {}
        fn update(&self, _: &(), _: FwEnumStoreType, _: HackSm_Signals, data: &SmSignalBuffer) {
            *self.0.lock().unwrap() = data.as_slice().to_vec();
        }
    }
    let c = SmTest::new();
    c.init(10);
    let mut data = SmSignalBuffer::new();
    assert!(data.serialize_u32_be(0x0102_0304).is_ok());
    c.device5_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
    let mut raw = vec![0u8; SM_TEST_MSG_SIZE];
    let (mut actual, mut priority) = (0, 0);
    let status = c.active.queued.queue().receive(
        &mut raw,
        BlockingType::NonBlocking,
        &mut actual,
        &mut priority,
    );
    assert_eq!(status, QueueStatus::OpOk);
    let mut msg = fprime_fw::ExtBuf::with_len(&mut raw, actual as usize);
    let mut msg_type = 0;
    assert!(msg::read_msg_type(&mut msg, &mut msg_type).is_ok());
    assert_eq!(msg_type, SmTest::MSG_TYPE_EXTERNAL_STATE_MACHINE_SIGNAL);
    let mut out = SmSignalBuffer::new();
    let (sm_id, signal) = sm::read_external_signal(&mut msg, &mut out);
    assert_eq!((sm_id, signal), (SmTest::SM_ID_DEVICE5, 0));
    let echo = Echo(Mutex::new(Vec::new()));
    assert_eq!(
        echo.dispatch(&(), sm_id, signal, &out),
        MsgDispatchStatus::Ok
    );
    assert_eq!(*echo.0.lock().unwrap(), [1, 2, 3, 4]);
    // Not one of the machine's signals: a dispatch error.
    assert_eq!(echo.dispatch(&(), sm_id, 9, &out), MsgDispatchStatus::Error);
}

#[test]
fn external_hook_policy_calls_the_overflow_hook() {
    let c = SmTest::new();
    c.init(2);
    let mut data = SmSignalBuffer::new();
    assert!(data.serialize_u8_be(1).is_ok());
    assert!(data.serialize_u8_be(2).is_ok());
    assert!(data.serialize_u8_be(3).is_ok());
    c.device4_state_machine_invoke(HackSm_Signals::CHECK_SIG, &data);
    c.device4_state_machine_invoke(HackSm_Signals::CHECK_SIG, &data);
    assert_eq!(c.hook_calls.load(Ordering::Relaxed), 0);
    c.device4_state_machine_invoke(HackSm_Signals::CHECK_SIG, &data);
    assert_eq!(c.hook_calls.load(Ordering::Relaxed), 1);
}

#[test]
fn external_drop_policy_counts_dropped_signals() {
    let c = SmTest::new();
    c.init(1);
    let data = SmSignalBuffer::new();
    c.device3_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
    c.device3_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
    assert_eq!(c.active.queued.get_num_msgs_dropped(), 1);
}

#[test]
fn unknown_external_sm_id_is_a_dispatch_error() {
    let c = SmTest::new();
    c.init(2);
    c.sender(9, 0, QueueFullPolicy::Assert)
        .invoke(0, &SmSignalBuffer::new());
    assert_eq!(
        c.active.queued.do_dispatch(&*c, BlockingType::NonBlocking),
        MsgDispatchStatus::Error
    );
}
