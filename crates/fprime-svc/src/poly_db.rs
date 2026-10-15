//! # PolyDb — port of `Svc::PolyDb` (passive, guarded)
//!
//! C++ sources: `Svc/PolyDb/PolyDb.{cpp,hpp,fpp}`, `Svc/PolyIf/PolyIf.fpp`,
//! `default/config/PolyDbCfg.fpp`, `Svc/PolyDb/docs/sdd.md`.
//! Analysis: `docs/cpp-analysis/svc-core.md` (PolyDb section).
//!
//! A resident table of [`PolyType`] measurements, one entry per
//! [`PolyDbEntry`], each with a time tag and a [`MeasurementStatus`]. Every
//! entry starts `Stale` (with a `NoType` value and a zero time) until its
//! first `setValue`. `getValue` copies an entry out into the caller's `ref`
//! arguments; `setValue` copies the caller's arguments in. Both ports are
//! guarded by the one component mutex.
//!
//! The database never inspects the value: a reader that expects a
//! different type than was written finds out from the [`PolyType`]
//! variant (the C++ `isXX()` checks), not from PolyDb.

use fprime_comp::{PassiveBase, input_port_adapter};
use fprime_config::FwIndexType;
use fprime_config::poly_db::NUM_ENTRIES;
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::{PolyType, Time, fw_assert};

pub use crate::poly_if::{MeasurementStatus, PolyDbEntry, PolyPort};

/// One database entry (C++ `t_dbStruct`).
#[derive(Debug, Clone, Copy, Default)]
struct DbEntry {
    /// Last status of the measurement.
    status: MeasurementStatus,
    /// The last value of the measurement.
    val: PolyType,
    /// The time tag of the last measurement.
    time: Time,
}

/// `Svc::PolyDb` — the polymorphic measurement database.
pub struct PolyDb {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// The database (C++ `m_db`); the mutex is the guarded-port mutex.
    db: Mutex<[DbEntry; NUM_ENTRIES]>,
}

impl PolyDb {
    /// Construct the component with every entry `Stale` (C++ constructor).
    pub fn new(name: &str) -> Arc<Self> {
        let stale = DbEntry {
            status: MeasurementStatus::Stale,
            ..DbEntry::default()
        };
        Arc::new(Self {
            base: PassiveBase::new(name),
            db: Mutex::new([stale; NUM_ENTRIES]),
        })
    }

    /// `getValue_handler` (guarded): copy the entry into the `ref` args.
    fn get_value_handler(
        &self,
        _port_num: FwIndexType,
        entry: PolyDbEntry,
        status: &mut MeasurementStatus,
        time: &mut Time,
        val: &mut PolyType,
    ) {
        // C++ FW_ASSERT(entry.isValid(), entry.e): a typed entry is always
        // valid; the index check stands in for it.
        fw_assert!(entry.index() < NUM_ENTRIES, entry.as_repr());
        let db = self.db.lock();
        let slot = &db[entry.index()];
        *status = slot.status;
        *time = slot.time;
        *val = slot.val;
    }

    /// `setValue_handler` (guarded): copy the `ref` args into the entry.
    fn set_value_handler(
        &self,
        _port_num: FwIndexType,
        entry: PolyDbEntry,
        status: &mut MeasurementStatus,
        time: &mut Time,
        val: &mut PolyType,
    ) {
        fw_assert!(entry.index() < NUM_ENTRIES, entry.as_repr());
        let mut db = self.db.lock();
        let slot = &mut db[entry.index()];
        slot.status = *status;
        slot.time = *time;
        slot.val = *val;
    }
}

input_port_adapter! {
    /// `getValue` — GUARDED `Svc.Poly` input: read an entry.
    component: PolyDb;
    adapter: GetValueAdapter;
    port: PolyPort;
    input: pub get_value;
    handler: get_value_handler;
    args { val entry: PolyDbEntry, mut status: MeasurementStatus, mut time: Time, mut val: PolyType }
}

input_port_adapter! {
    /// `setValue` — GUARDED `Svc.Poly` input: write an entry.
    component: PolyDb;
    adapter: SetValueAdapter;
    port: PolyPort;
    input: pub set_value;
    handler: set_value_handler;
    args { val entry: PolyDbEntry, mut status: MeasurementStatus, mut time: Time, mut val: PolyType }
}

#[cfg(test)]
mod tests {
    //! Port of `Svc/PolyDb/test/ut` (`runNominalReadWrite`, requirements
    //! PDB-001..004) plus the initial-state and isolation checks.

    use super::*;
    use fprime_fw::TimeBase;

    fn get(db: &Arc<PolyDb>, entry: PolyDbEntry) -> (MeasurementStatus, Time, PolyType) {
        let port = db.get_value(0);
        let mut status = MeasurementStatus::Ok;
        let mut time = Time::default();
        let mut val = PolyType::default();
        port.target
            .invoke(port.port_num, entry, &mut status, &mut time, &mut val);
        (status, time, val)
    }

    fn set(
        db: &Arc<PolyDb>,
        entry: PolyDbEntry,
        status: MeasurementStatus,
        time: Time,
        val: PolyType,
    ) {
        let port = db.set_value(0);
        let (mut status, mut time, mut val) = (status, time, val);
        port.target
            .invoke(port.port_num, entry, &mut status, &mut time, &mut val);
    }

    #[test]
    fn entries_start_stale() {
        let db = PolyDb::new("PolyDb");
        for &entry in PolyDbEntry::VALUES {
            let (status, time, val) = get(&db, entry);
            assert_eq!(status, MeasurementStatus::Stale);
            assert_eq!(time, Time::default());
            assert_eq!(val, PolyType::NoType);
        }
    }

    #[test]
    fn nominal_read_write() {
        // One value of each supported type (upstream NUM_TEST_VALS = 12;
        // the C++ `void*` becomes the pointer-cast integer).
        let vals = [
            PolyType::U8(1),
            PolyType::I8(2),
            PolyType::U16(3),
            PolyType::I16(4),
            PolyType::U32(5),
            PolyType::I32(6),
            PolyType::U64(7),
            PolyType::I64(8),
            PolyType::F32(9.0),
            PolyType::F64(10.0),
            PolyType::Bool(true),
            PolyType::Ptr(0x100),
        ];
        let ts = Time::new(TimeBase::TbNone, 0, 6, 7);
        let db = PolyDb::new("PolyDb");
        // Every entry with every status (PDB-001..PDB-004).
        for &mstat in MeasurementStatus::VALUES {
            for (i, &entry) in PolyDbEntry::VALUES.iter().enumerate() {
                let value = vals[i % vals.len()];
                set(&db, entry, mstat, ts, value);
                let (check_stat, check_ts, check) = get(&db, entry);
                // Upstream asserts `check != value` for F32/F64: the C++
                // `PolyType::operator==` returns false for floating-point
                // (and NoType) values by design. The Rust `PolyType`
                // derives `PartialEq`, so a stored float compares equal
                // to itself here; the stored bits are what matter.
                assert_eq!(check, value);
                assert_eq!(check_ts, ts);
                assert_eq!(check_stat, mstat);
            }
        }
    }

    #[test]
    fn entries_are_independent() {
        let db = PolyDb::new("PolyDb");
        let ts = Time::new(TimeBase::TbProcTime, 3, 100, 200);
        set(
            &db,
            PolyDbEntry::PolydbEntry03,
            MeasurementStatus::Failure,
            ts,
            PolyType::I32(-5),
        );
        for &entry in PolyDbEntry::VALUES {
            let (status, time, val) = get(&db, entry);
            if entry == PolyDbEntry::PolydbEntry03 {
                assert_eq!(status, MeasurementStatus::Failure);
                assert_eq!(time, ts);
                assert_eq!(time.get_context(), 3);
                assert_eq!(val, PolyType::I32(-5));
            } else {
                assert_eq!(status, MeasurementStatus::Stale);
                assert_eq!(val, PolyType::NoType);
            }
        }
    }

    #[test]
    fn config_enum_keeps_the_upstream_gap() {
        // POLYDB_ENTRY_02 is absent upstream: names and values diverge.
        assert_eq!(PolyDbEntry::NUM_CONSTANTS, 9);
        assert_eq!(PolyDbEntry::PolydbEntry03.as_repr(), 2);
        assert_eq!(PolyDbEntry::PolydbEntry09.as_repr(), 8);
        assert!(!PolyDbEntry::is_valid_repr(9));
        assert_eq!(MeasurementStatus::Stale.as_repr(), 2);
        assert_eq!(MeasurementStatus::SERIALIZED_SIZE, 1);
        assert_eq!(PolyDbEntry::SERIALIZED_SIZE, 4);
    }
}
