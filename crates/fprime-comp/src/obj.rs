//! Port of `Fw::ObjBase` + `Fw::PassiveComponentBase` state
//! (`Fw/Obj/ObjBase.cpp`, `Fw/Comp/PassiveComponentBase.cpp`; analysis:
//! `docs/cpp-analysis/fw-comp.md`).
//!
//! The object registry (`Fw::ObjRegistry` / `SimpleObjRegistry`) is not
//! ported — it exists for ground debugging dumps and holds raw pointers by
//! design; Rust deployments keep their `Arc` handles in the topology struct
//! instead.

use fprime_config::{FwEnumStoreType, FwIdType};
use fprime_fw::ObjectName;
use std::sync::Mutex;
use std::sync::PoisonError;
use std::sync::atomic::{AtomicI32, AtomicU32, Ordering};

/// State every component kind carries: object name, runtime id base, and
/// instance number.
///
/// C++ splits this between `Fw::ObjBase` (name) and
/// `Fw::PassiveComponentBase` (`m_idBase`, `m_instance`); the Rust port
/// collapses them since ports do not derive from this type. Components are
/// shared as `Arc<C>`, so all setters take `&self` — they are meant to be
/// called during topology setup, before tasks start (C++ parity: the same
/// fields are set un-synchronized at init time).
#[derive(Debug)]
pub struct PassiveBase {
    /// Object name, truncated to `FW_OBJ_NAME_BUFFER_SIZE` (C++
    /// `Fw::ObjectName` truncates silently).
    name: Mutex<ObjectName>,
    /// Runtime offset added to opcodes / event ids / channel ids / param ids
    /// (C++ `m_idBase`, `FwIdType` = u32, init 0).
    id_base: AtomicU32,
    /// Instance number (C++ `m_instance`, `FwEnumStoreType` = i32, init 0).
    instance: AtomicI32,
}

impl PassiveBase {
    /// Create with a name. C++ parity: a null name becomes `"NoName"`
    /// (`ObjBase.cpp`); an empty `&str` is kept empty since Rust has no null.
    pub fn new(name: &str) -> Self {
        Self {
            name: Mutex::new(ObjectName::from(name)),
            id_base: AtomicU32::new(0),
            instance: AtomicI32::new(0),
        }
    }

    /// The object name (a copy; the internal copy can be renamed later).
    pub fn get_obj_name(&self) -> ObjectName {
        self.name
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// Set (rename) the object name — C++ `setObjName`, used by topology
    /// code. Truncates silently to the `ObjectName` capacity.
    pub fn set_obj_name(&self, name: &str) {
        self.name
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .set(name);
    }

    /// C++ `setIdBase`.
    pub fn set_id_base(&self, base: FwIdType) {
        self.id_base.store(base, Ordering::Relaxed);
    }

    /// C++ `getIdBase`.
    pub fn get_id_base(&self) -> FwIdType {
        self.id_base.load(Ordering::Relaxed)
    }

    /// C++ `PassiveComponentBase::init(instance)` — stores the instance
    /// number (registry callback not ported).
    pub fn set_instance(&self, instance: FwEnumStoreType) {
        self.instance.store(instance, Ordering::Relaxed);
    }

    /// C++ `getInstance` (protected there; public here — Rust components are
    /// separate structs, not subclasses).
    pub fn get_instance(&self) -> FwEnumStoreType {
        self.instance.load(Ordering::Relaxed)
    }
}

impl Default for PassiveBase {
    /// C++ parity: unnamed objects are `"NoName"` (`ObjBase.cpp`).
    fn default() -> Self {
        Self::new("NoName")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_cpp_init_values() {
        let base = PassiveBase::default();
        assert_eq!(base.get_obj_name(), "NoName");
        assert_eq!(base.get_id_base(), 0);
        assert_eq!(base.get_instance(), 0);
    }

    #[test]
    fn setters_round_trip() {
        let base = PassiveBase::new("comp1");
        assert_eq!(base.get_obj_name(), "comp1");
        base.set_obj_name("renamed");
        assert_eq!(base.get_obj_name(), "renamed");
        base.set_id_base(0x1000);
        assert_eq!(base.get_id_base(), 0x1000);
        base.set_instance(3);
        assert_eq!(base.get_instance(), 3);
    }

    #[test]
    fn long_names_truncate_silently() {
        // C++ parity: Fw::ObjectName truncates to FW_OBJ_NAME_BUFFER_SIZE.
        let long = "x".repeat(200);
        let base = PassiveBase::new(&long);
        assert_eq!(
            base.get_obj_name().len(),
            fprime_config::FW_OBJ_NAME_BUFFER_SIZE
        );
    }
}
