//! # Svc/PolyIf — the `Svc.Poly` port and its types
//!
//! C++ sources: `Svc/PolyIf/PolyIf.fpp` (`Svc.MeasurementStatus`, the
//! `Svc.Poly` port) and `default/config/PolyDbCfg.fpp`
//! (`Svc.PolyDbCfg.PolyDbEntry`, the entry index the port carries).
//! Analysis: `docs/cpp-analysis/svc-core.md` (PolyDb section).
//!
//! Served by [`PolyDb`](crate::poly_db::PolyDb): one component sets an
//! [`fprime_fw::PolyType`] measurement, others read it back with its time
//! tag and [`MeasurementStatus`].

use fprime_config::FwIndexType;
use fprime_fw::{PolyType, Time, fpp_enum};

fpp_enum! {
    /// `Svc.MeasurementStatus` — the state of a stored measurement.
    pub enum MeasurementStatus : u8 {
        /// Measurement was good.
        Ok = 0,
        /// Failure to retrieve measurement.
        Failure = 1,
        /// Measurement is stale.
        Stale = 2,
    }
    default Ok
}

fpp_enum! {
    /// `Svc.PolyDbCfg.PolyDbEntry` (`PolyDbCfg.fpp`): the database entries,
    /// defined per project.
    ///
    /// C++ parity: upstream's default list skips `POLYDB_ENTRY_02`, so the
    /// names and the auto-assigned values diverge from `PolydbEntry03`
    /// (value 2) on. Kept verbatim — the values are what cross ports.
    pub enum PolyDbEntry : u32 {
        /// Entry 0.
        PolydbEntry00 = 0,
        /// Entry 1.
        PolydbEntry01 = 1,
        /// Entry 2 (named `POLYDB_ENTRY_03` upstream).
        PolydbEntry03 = 2,
        /// Entry 4 (upstream comment; value 3).
        PolydbEntry04 = 3,
        /// Entry 5 (upstream comment; value 4).
        PolydbEntry05 = 4,
        /// Entry 6 (upstream comment; value 5).
        PolydbEntry06 = 5,
        /// Entry 7 (upstream comment; value 6).
        PolydbEntry07 = 6,
        /// Entry 8 (upstream comment; value 7).
        PolydbEntry08 = 7,
        /// Entry 9 (upstream comment; value 8).
        PolydbEntry09 = 8,
    }
    default PolydbEntry00
}

// The database size is configured in `fprime-config`; keep the enum and the
// count in step (a project edits both).
const _: () = assert!(PolyDbEntry::NUM_CONSTANTS == fprime_config::poly_db::NUM_ENTRIES);

impl PolyDbEntry {
    /// The entry's index into the database (its FPP value; the constants
    /// are contiguous from 0, as FPP assigns them).
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// `Svc.Poly` — set or get a [`PolyType`] value. All three `ref` arguments
/// are in/out: a setter passes values in, a getter receives them.
pub trait PolyPort: Send + Sync {
    /// Invoke the port.
    fn invoke(
        &self,
        port_num: FwIndexType,
        entry: PolyDbEntry,
        status: &mut MeasurementStatus,
        time: &mut Time,
        val: &mut PolyType,
    );
}
