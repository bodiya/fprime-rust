//! Implementations of the FppTest state machine components generated
//! from upstream's FPP (`fpp/sm/FppTest/state_machine`, see
//! [`crate::sm_models`]): the `internal_instance` components `Basic`
//! (queued; two machine types, one of them component-local),
//! `BasicGuardTestAbsType` (active; typed guard, `hook` policy) and
//! `InputPairU16U32` (active; a `U16` and a `U32` signal into one typed
//! choice), and the `external_instance` component `SmTest` (five
//! instances of the external machines in [`crate::sm_support`]). Each
//! records what the generated code calls, like upstream's testers;
//! `tests/sm_instances.rs` drives them.

use crate::sm_models::FppTest::SmChoice::InputPairU16U32 as SmChoiceInputPair;
use crate::sm_models::FppTest::SmInstanceChoice::{InputPairU16U32Base, InputPairU16U32Handlers};
use crate::sm_models::FppTest::SmInstanceState::Basic as BasicLocal;
use crate::sm_models::FppTest::SmInstanceState::{
    BasicBase, BasicGuardTestAbsTypeBase, BasicGuardTestAbsTypeHandlers, BasicHandlers,
};
use crate::sm_models::FppTest::SmState::{Basic as SmStateBasic, BasicGuardTestAbsType};
use crate::sm_models::FppTest::{SmTestBase, SmTestHandlers};
use crate::sm_support::{
    DeviceSm_Interface, DeviceSm_Signals, HackSm_Interface, History, TestAbsType,
};
use fprime_config::{FwEnumStoreType, FwIndexType};
use fprime_fw::{Endianness, SerBuf, SerBufAny, SmSignalBuffer};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// SmInstanceState.Basic
// ---------------------------------------------------------------------------

/// `FppTest.SmInstanceState.Basic`.
pub struct Basic {
    base: BasicBase,
    /// `action a` calls of `basic1`.
    pub basic1_a: History<BasicLocal::Basic::Signal, ()>,
    /// `action a` calls of `basic2`.
    pub basic2_a: History<BasicLocal::Basic::Signal, ()>,
    /// `action a` calls of `smStateBasic1`.
    pub sm_state_basic1_a: History<SmStateBasic::Signal, ()>,
    /// `action a` calls of `smStateBasic2`.
    pub sm_state_basic2_a: History<SmStateBasic::Signal, ()>,
}

impl Basic {
    /// Construct.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: BasicBase::new(name),
            basic1_a: History::new(),
            basic2_a: History::new(),
            sm_state_basic1_a: History::new(),
            sm_state_basic2_a: History::new(),
        })
    }
}

impl BasicHandlers for Basic {
    fn base(&self) -> &BasicBase {
        &self.base
    }

    fn sched_in_handler(&self, _port_num: FwIndexType, _context: u32) {}

    fn FppTest_SmInstanceState_Basic_Basic_action_a(
        &self,
        sm_id: FwEnumStoreType,
        signal: BasicLocal::Basic::Signal,
    ) {
        match sm_id {
            BasicBase::SM_ID_BASIC1 => self.basic1_a.push(signal, ()),
            BasicBase::SM_ID_BASIC2 => self.basic2_a.push(signal, ()),
            _ => panic!("unexpected state machine id {sm_id}"),
        }
    }

    fn FppTest_SmState_Basic_action_a(&self, sm_id: FwEnumStoreType, signal: SmStateBasic::Signal) {
        match sm_id {
            BasicBase::SM_ID_SM_STATE_BASIC1 => self.sm_state_basic1_a.push(signal, ()),
            BasicBase::SM_ID_SM_STATE_BASIC2 => self.sm_state_basic2_a.push(signal, ()),
            _ => panic!("unexpected state machine id {sm_id}"),
        }
    }
}

crate::sm_models::FppTest::SmInstanceState::impl_basic_component!(Basic);

// ---------------------------------------------------------------------------
// SmInstanceState.BasicGuardTestAbsType
// ---------------------------------------------------------------------------

/// `FppTest.SmInstanceState.BasicGuardTestAbsType`.
pub struct GuardAbs {
    base: BasicGuardTestAbsTypeBase,
    /// `action a` calls.
    pub a: History<BasicGuardTestAbsType::Signal, TestAbsType>,
    /// `guard g` calls.
    pub g: History<BasicGuardTestAbsType::Signal, TestAbsType>,
    /// What `guard g` returns.
    pub guard: AtomicBool,
    /// Overflow hook calls: (signal, value read from the message).
    pub overflows: Mutex<Vec<(FwEnumStoreType, TestAbsType)>>,
}

impl GuardAbs {
    /// Construct.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: BasicGuardTestAbsTypeBase::new(name),
            a: History::new(),
            g: History::new(),
            guard: AtomicBool::new(false),
            overflows: Mutex::new(Vec::new()),
        })
    }
}

impl BasicGuardTestAbsTypeHandlers for GuardAbs {
    fn base(&self) -> &BasicGuardTestAbsTypeBase {
        &self.base
    }

    fn FppTest_SmState_BasicGuardTestAbsType_action_a(
        &self,
        _sm_id: FwEnumStoreType,
        signal: BasicGuardTestAbsType::Signal,
        value: &TestAbsType,
    ) {
        self.a.push(signal, *value);
    }

    fn FppTest_SmState_BasicGuardTestAbsType_guard_g(
        &self,
        _sm_id: FwEnumStoreType,
        signal: BasicGuardTestAbsType::Signal,
        value: &TestAbsType,
    ) -> bool {
        self.g.push(signal, *value);
        self.guard.load(Ordering::Relaxed)
    }

    fn sm_state_basic_guard_test_abs_type_state_machine_overflow_hook(
        &self,
        sm_id: FwEnumStoreType,
        signal: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) {
        assert_eq!(
            sm_id,
            BasicGuardTestAbsTypeBase::SM_ID_SM_STATE_BASIC_GUARD_TEST_ABS_TYPE
        );
        let mut value = TestAbsType::default();
        assert!(msg.deserialize(&mut value, Endianness::Big).is_ok());
        self.overflows.lock().unwrap().push((signal, value));
    }
}

crate::sm_models::FppTest::SmInstanceState::impl_basic_guard_test_abs_type_component!(GuardAbs);

// ---------------------------------------------------------------------------
// SmInstanceChoice.InputPairU16U32
// ---------------------------------------------------------------------------

/// `FppTest.SmInstanceChoice.InputPairU16U32`.
pub struct InputPair {
    base: InputPairU16U32Base,
    /// `action a` calls.
    pub a: History<SmChoiceInputPair::Signal, u32>,
    /// `guard g` calls.
    pub g: History<SmChoiceInputPair::Signal, u32>,
    /// What `guard g` returns.
    pub guard: AtomicBool,
}

impl InputPair {
    /// Construct.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: InputPairU16U32Base::new(name),
            a: History::new(),
            g: History::new(),
            guard: AtomicBool::new(false),
        })
    }
}

impl InputPairU16U32Handlers for InputPair {
    fn base(&self) -> &InputPairU16U32Base {
        &self.base
    }

    fn FppTest_SmChoice_InputPairU16U32_action_a(
        &self,
        _sm_id: FwEnumStoreType,
        signal: SmChoiceInputPair::Signal,
        value: u32,
    ) {
        self.a.push(signal, value);
    }

    fn FppTest_SmChoice_InputPairU16U32_guard_g(
        &self,
        _sm_id: FwEnumStoreType,
        signal: SmChoiceInputPair::Signal,
        value: u32,
    ) -> bool {
        self.g.push(signal, value);
        self.guard.load(Ordering::Relaxed)
    }
}

crate::sm_models::FppTest::SmInstanceChoice::impl_input_pair_u16_u32_component!(InputPair);

// ---------------------------------------------------------------------------
// SmTest (external_instance)
// ---------------------------------------------------------------------------

/// `FppTest.SmTest`: `schedIn` sends `RTI_SIG` to all five machines
/// (upstream `SmTest.cpp`).
pub struct SmTest {
    base: SmTestBase,
    /// Interface calls: (name, state machine id).
    pub calls: Mutex<Vec<(&'static str, FwEnumStoreType)>>,
    /// Overflow hook calls of `device4`: (signal, data bytes).
    pub hook_calls: Mutex<Vec<(FwEnumStoreType, Vec<u8>)>>,
    /// `schedIn` calls.
    pub ticks: AtomicU32,
}

impl SmTest {
    /// Construct.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: SmTestBase::new(name),
            calls: Mutex::new(Vec::new()),
            hook_calls: Mutex::new(Vec::new()),
            ticks: AtomicU32::new(0),
        })
    }

    fn record(&self, what: &'static str, sm_id: FwEnumStoreType) {
        self.calls.lock().unwrap().push((what, sm_id));
    }
}

impl SmTestHandlers for SmTest {
    fn base(&self) -> &SmTestBase {
        &self.base
    }

    fn sched_in_handler(&self, _port_num: FwIndexType, _context: u32) {
        use crate::sm_models::FppTest::SmTestComponent;
        use crate::sm_support::HackSm_Signals;
        self.ticks.fetch_add(1, Ordering::Relaxed);
        let data = SmSignalBuffer::new();
        self.device1_state_machine_invoke(DeviceSm_Signals::RTI_SIG, &data);
        self.device2_state_machine_invoke(DeviceSm_Signals::RTI_SIG, &data);
        self.device3_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
        self.device4_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
        self.device5_state_machine_invoke(HackSm_Signals::RTI_SIG, &data);
    }

    fn device4_state_machine_overflow_hook(&self, signal: FwEnumStoreType, data: &SmSignalBuffer) {
        self.hook_calls
            .lock()
            .unwrap()
            .push((signal, data.as_slice().to_vec()));
    }
}

#[allow(non_snake_case)]
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

#[allow(non_snake_case)]
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

crate::sm_models::FppTest::impl_sm_test_component!(SmTest);
