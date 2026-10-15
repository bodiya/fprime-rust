//! Port of `FppTest/state_machine/internal/harness`: the FPP types the
//! state machine tests use (`Harness.fpp`, `TestAbsType`), call histories
//! (`History`, `SignalValueHistory`), configurable guards (`Guard`,
//! `NoArgGuard`) and value picks (`Pick`, deterministic here instead of
//! STest's random picks).
#![allow(dead_code)]

use fprime_comp::config::FwEnumStoreType;
use fprime_fw::{
    Deserialize, Endianness, FppSized, SerBuf, SerBufAny, Serialize, SerializeStatus, fpp_array,
    fpp_enum, fpp_struct,
};
use std::cell::Cell;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

// ---------------------------------------------------------------------------
// module SmHarness (Harness.fpp)
// ---------------------------------------------------------------------------

/// `type TestAbsType`: an abstract type wrapping one `U32`
/// (`TestAbsType.hpp`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TestAbsType {
    pub data: u32,
}

impl TestAbsType {
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

fpp_enum! {
    /// `enum TestEnum { A, B, C }` (default representation I32).
    pub enum TestEnum : i32 {
        /// A
        A = 0,
        /// B
        B = 1,
        /// C
        C = 2,
    }
    default A
}

fpp_array! {
    /// `array TestArray = [3] U32`.
    #[derive(Clone, Copy, Eq)]
    pub array TestArray = [u32; 3]
}

fpp_struct! {
    /// `struct TestStruct { x: U32 }`.
    #[derive(Clone, Copy, Eq)]
    pub struct TestStruct {
        x: u32,
    }
}

// ---------------------------------------------------------------------------
// Histories and guards
// ---------------------------------------------------------------------------

/// A call history of `(signal, value)` pairs (`SignalValueHistory`; a
/// `History` of signals alone is a `History<S, ()>`). Behind a mutex so a
/// component (which must be `Sync`) can own one.
pub struct History<S, T> {
    items: Mutex<Vec<(S, T)>>,
}

impl<S: Copy, T: Clone> History<S, T> {
    pub fn new() -> Self {
        Self {
            items: Mutex::new(Vec::new()),
        }
    }

    pub fn push(&self, signal: S, value: T) {
        self.items.lock().unwrap().push((signal, value));
    }

    pub fn clear(&self) {
        self.items.lock().unwrap().clear();
    }

    pub fn size(&self) -> usize {
        self.items.lock().unwrap().len()
    }

    pub fn items(&self) -> Vec<(S, T)> {
        self.items.lock().unwrap().clone()
    }

    pub fn signals(&self) -> Vec<S> {
        self.items.lock().unwrap().iter().map(|(s, _)| *s).collect()
    }

    pub fn values(&self) -> Vec<T> {
        self.items
            .lock()
            .unwrap()
            .iter()
            .map(|(_, v)| v.clone())
            .collect()
    }
}

impl<S: Copy, T: Clone> Default for History<S, T> {
    fn default() -> Self {
        Self::new()
    }
}

/// A guard with a settable return value and a call history (`Guard`, and
/// `NoArgGuard` as `Guard<S, ()>`).
pub struct Guard<S, T> {
    return_value: AtomicBool,
    history: History<S, T>,
}

impl<S: Copy, T: Clone> Guard<S, T> {
    pub fn new() -> Self {
        Self {
            return_value: AtomicBool::new(false),
            history: History::new(),
        }
    }

    pub fn reset(&self) {
        self.history.clear();
        self.return_value.store(false, Ordering::Relaxed);
    }

    pub fn set_return_value(&self, v: bool) {
        self.return_value.store(v, Ordering::Relaxed);
    }

    pub fn call(&self, signal: S, value: T) -> bool {
        self.history.push(signal, value);
        self.return_value.load(Ordering::Relaxed)
    }

    pub fn history(&self) -> &History<S, T> {
        &self.history
    }
}

impl<S: Copy, T: Clone> Default for Guard<S, T> {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Pick
// ---------------------------------------------------------------------------

/// Deterministic stand-ins for `SmHarness::Pick` (a fixed-seed xorshift
/// instead of STest's random source, so failures reproduce).
pub struct Pick {
    state: Cell<u64>,
}

impl Pick {
    pub fn new(seed: u64) -> Self {
        Self {
            state: Cell::new(seed | 1),
        }
    }

    pub fn any(&self) -> u32 {
        let mut x = self.state.get();
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state.set(x);
        (x >> 16) as u32
    }

    /// `Pick::stateMachineId()`: a non-negative `FwEnumStoreType`.
    pub fn state_machine_id(&self) -> FwEnumStoreType {
        (self.any() & 0x7FFF_FFFF) as FwEnumStoreType
    }

    pub fn test_abs_type(&self) -> TestAbsType {
        TestAbsType::new(self.any())
    }

    pub fn test_array(&self) -> TestArray {
        TestArray::new([self.any(), self.any(), self.any()])
    }

    pub fn test_enum(&self) -> TestEnum {
        TestEnum::VALUES[(self.any() % 3) as usize]
    }

    pub fn test_struct(&self) -> TestStruct {
        TestStruct::new(self.any())
    }

    /// `Pick::string`: printable ASCII, length in `0..=max_len`.
    pub fn string(&self, max_len: usize) -> String {
        let len = (self.any() as usize) % (max_len + 1);
        (0..len)
            .map(|_| char::from(32 + (self.any() % 95) as u8))
            .collect()
    }
}
