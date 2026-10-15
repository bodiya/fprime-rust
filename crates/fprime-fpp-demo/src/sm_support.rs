//! Hand-written items the generated state machine models (`fpp/sm`) are
//! bound to: the FppTest harness's abstract type `TestAbsType` and the
//! external state machines `FppTest.DeviceSm` / `FppTest.HackSm` of the
//! FppTest `external_instance` suite (upstream's `DeviceSm.cpp` /
//! `HackSm.cpp`, as the STARS autocoder generates them), plus a call
//! history for the component implementations in [`crate::fpptest`].

use fprime_config::FwEnumStoreType;
use fprime_fw::{
    Deserialize, Endianness, FppSized, SerBuf, SerBufAny, Serialize, SerializeStatus,
    SmSignalBuffer,
};
use std::sync::Mutex;
use std::sync::atomic::{AtomicI32, Ordering};

/// `FppTest.SmHarness.TestAbsType`: an abstract type wrapping one `U32`
/// (upstream `TestAbsType.hpp`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TestAbsType {
    /// The wrapped value.
    pub data: u32,
}

impl TestAbsType {
    /// A value.
    pub fn new(data: u32) -> Self {
        Self { data }
    }
}

impl FppSized for TestAbsType {
    const SERIALIZED_SIZE: usize = 4;
}

impl Serialize for TestAbsType {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        buf.serialize_u32(self.data, e)
    }
    fn serialized_size(&self) -> usize {
        4
    }
}

impl Deserialize for TestAbsType {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        buf.deserialize_u32(&mut self.data, e)
    }
}

/// A call history (upstream `SmHarness::History`).
#[derive(Debug, Default)]
pub struct History<S, T> {
    items: Mutex<Vec<(S, T)>>,
}

impl<S: Copy, T: Clone> History<S, T> {
    /// An empty history.
    pub fn new() -> Self {
        Self {
            items: Mutex::new(Vec::new()),
        }
    }

    /// Record one call.
    pub fn push(&self, signal: S, value: T) {
        self.items.lock().unwrap().push((signal, value));
    }

    /// Forget every call.
    pub fn clear(&self) {
        self.items.lock().unwrap().clear();
    }

    /// The number of calls.
    pub fn size(&self) -> usize {
        self.items.lock().unwrap().len()
    }

    /// The signals of the calls.
    pub fn signals(&self) -> Vec<S> {
        self.items.lock().unwrap().iter().map(|(s, _)| *s).collect()
    }

    /// The values of the calls.
    pub fn values(&self) -> Vec<T> {
        self.items
            .lock()
            .unwrap()
            .iter()
            .map(|(_, v)| v.clone())
            .collect()
    }
}

// ---------------------------------------------------------------------------
// DeviceSm (DeviceSm.hpp/.cpp)
// ---------------------------------------------------------------------------

/// `DeviceSm_Interface`: what `DeviceSm` calls back into.
#[allow(non_snake_case, non_camel_case_types)]
pub trait DeviceSm_Interface {
    /// Guard `g1`.
    fn DeviceSm_g1(&self, sm_id: FwEnumStoreType) -> bool;
    /// Guard `g2`.
    fn DeviceSm_g2(
        &self,
        sm_id: FwEnumStoreType,
        signal: DeviceSm_Signals,
        data: &SmSignalBuffer,
    ) -> bool;
    /// Action `turnOff`.
    fn DeviceSm_turnOff(&self, sm_id: FwEnumStoreType);
    /// Action `a1`.
    fn DeviceSm_a1(&self, sm_id: FwEnumStoreType, signal: DeviceSm_Signals, data: &SmSignalBuffer);
    /// Action `turnOn`.
    fn DeviceSm_turnOn(&self, sm_id: FwEnumStoreType);
    /// Action `a2`.
    fn DeviceSm_a2(&self, sm_id: FwEnumStoreType);
}

/// `DeviceSm_Interface::DeviceSm_Signals`.
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceSm_Signals {
    /// The real-time-interrupt signal.
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

/// `DeviceSm::DeviceSm_States`.
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceSm_States {
    /// Off.
    OFF = 0,
    /// On.
    ON = 1,
}

/// `DeviceSm`: OFF/ON, toggled by `RTI_SIG` under guards `g1`/`g2`.
#[derive(Debug, Default)]
pub struct DeviceSm {
    state: AtomicI32,
}

impl DeviceSm {
    /// The current state.
    pub fn state(&self) -> DeviceSm_States {
        match self.state.load(Ordering::Relaxed) {
            0 => DeviceSm_States::OFF,
            _ => DeviceSm_States::ON,
        }
    }

    fn set(&self, s: DeviceSm_States) {
        self.state.store(s as i32, Ordering::Relaxed);
    }
}

impl fprime_comp::sm::ExternalStateMachine for DeviceSm {
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
        match (self.state(), signal) {
            (DeviceSm_States::OFF, DeviceSm_Signals::RTI_SIG) => {
                if parent.DeviceSm_g1(sm_id) {
                    parent.DeviceSm_a1(sm_id, signal, data);
                    parent.DeviceSm_turnOn(sm_id);
                    self.set(DeviceSm_States::ON);
                }
            }
            (DeviceSm_States::ON, DeviceSm_Signals::RTI_SIG) => {
                if parent.DeviceSm_g2(sm_id, signal, data) {
                    parent.DeviceSm_a2(sm_id);
                    parent.DeviceSm_turnOff(sm_id);
                    self.set(DeviceSm_States::OFF);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// HackSm (HackSm.hpp/.cpp)
// ---------------------------------------------------------------------------

/// `HackSm_Interface`: what `HackSm` calls back into.
#[allow(non_snake_case, non_camel_case_types)]
pub trait HackSm_Interface {
    /// Action `turnOff`.
    fn HackSm_turnOff(&self, sm_id: FwEnumStoreType);
    /// Action `turnOn`.
    fn HackSm_turnOn(&self, sm_id: FwEnumStoreType);
    /// Action `doDiag`.
    fn HackSm_doDiag(&self, sm_id: FwEnumStoreType);
}

/// `HackSm_Interface::HackSm_Signals`.
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HackSm_Signals {
    /// The real-time-interrupt signal.
    RTI_SIG = 0,
    /// The diagnostics signal.
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

/// `HackSm::HackSm_States`.
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HackSm_States {
    /// Off.
    OFF = 0,
    /// On.
    ON = 1,
    /// Diagnostics.
    DIAG = 2,
}

/// `HackSm`: OFF/ON/DIAG.
#[derive(Debug, Default)]
pub struct HackSm {
    state: AtomicI32,
}

impl HackSm {
    /// The current state.
    pub fn state(&self) -> HackSm_States {
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

impl fprime_comp::sm::ExternalStateMachine for HackSm {
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
