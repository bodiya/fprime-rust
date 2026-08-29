//! # ApidManager — port of `Svc::Ccsds::ApidManager` (passive, both inputs guarded)
//!
//! C++ sources: `Svc/Ccsds/ApidManager/ApidManager.{cpp,hpp,fpp}`.
//! Analysis: `docs/cpp-analysis/ccsds.md` ("ApidManager").
//!
//! Holds one 14-bit Space Packet sequence counter per APID, shared between
//! the downlink thread (`getApidSeqCountIn`, from
//! [`SpacePacketFramer`](crate::ccsds::space_packet_framer::SpacePacketFramer))
//! and the uplink thread (`validateApidSeqCountIn`, from
//! [`SpacePacketDeframer`](crate::ccsds::space_packet_deframer::SpacePacketDeframer)).
//! Both input ports are `guarded`, and that single component mutex is the
//! only synchronization protecting the table.
//!
//! The table is a fixed-capacity association array sized to the number of
//! [`Apid`] constants, exactly like the C++ `Fw::ArrayMap<Apid::T, U16,
//! MAX_TRACKED_APIDS>`: every key is an enum member (the deframer maps
//! unknown 11-bit APIDs to `INVALID_UNINITIALIZED` before getting here), so
//! the table can never overflow — an insert failure is an assert, not an
//! error return.

use crate::ccsds::types::{ApidSequenceCountPort, space_packet_subfields};
use fprime_comp::{EventGlue, PassiveBase, input_port_adapter};
use fprime_config::{FwEventIdType, FwIdType, FwIndexType};
use fprime_fw::{Apid, LogSeverity, SerBuf, fw_assert, fw_try};
use std::sync::{Arc, Mutex};

/// Sequence-count modulus: the counter is 14 bits (`1 << SeqCountWidth`).
pub const SEQ_COUNT_MODULUS: u32 = 1 << space_packet_subfields::SEQ_COUNT_WIDTH;

/// One `(APID, sequence count)` table slot.
#[derive(Debug, Clone, Copy)]
struct Entry {
    /// The tracked APID.
    apid: Apid,
    /// The NEXT sequence count to hand out for that APID.
    seq_count: u16,
}

/// Guarded state: the association array and its live length.
struct ApidState {
    /// Occupied slots, in insertion order (C++ `Fw::ArrayMap` storage).
    entries: [Option<Entry>; ApidManager::MAX_TRACKED_APIDS],
}

impl ApidState {
    /// Look up `apid`; `None` when it is not tracked yet.
    fn find(&self, apid: Apid) -> Option<u16> {
        self.entries
            .iter()
            .flatten()
            .find(|entry| entry.apid == apid)
            .map(|entry| entry.seq_count)
    }

    /// Insert or overwrite `apid`'s count. Returns `false` when the table is
    /// full (impossible with enum keys — the caller asserts).
    #[must_use]
    fn insert(&mut self, apid: Apid, seq_count: u16) -> bool {
        for slot in self.entries.iter_mut() {
            match slot {
                Some(entry) if entry.apid == apid => {
                    entry.seq_count = seq_count;
                    return true;
                }
                Some(_) => {}
                None => {
                    *slot = Some(Entry { apid, seq_count });
                    return true;
                }
            }
        }
        false
    }
}

/// `Svc::Ccsds::ApidManager` — passive per-APID sequence counter.
pub struct ApidManager {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// The guarded table (the mutex of both `guarded input port`s).
    state: Mutex<ApidState>,
}

impl ApidManager {
    /// Table capacity: `ComCfg::Apid::NUM_CONSTANTS`.
    pub const MAX_TRACKED_APIDS: usize = Apid::NUM_CONSTANTS;

    /// `UnexpectedSequenceCount(transmitted: U16, expected: U16)` —
    /// WARNING_LO, FPP-relative event id 0.
    pub const EVENTID_UNEXPECTED_SEQUENCE_COUNT: FwEventIdType = 0;

    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            state: Mutex::new(ApidState {
                entries: [None; Self::MAX_TRACKED_APIDS],
            }),
        })
    }

    /// The component's event/telemetry id base.
    fn id_base(&self) -> FwIdType {
        self.base.get_id_base()
    }

    /// The next sequence count after `seq_count`, wrapping at 14 bits
    /// (C++ `calculateNextSeqCount`).
    #[must_use]
    pub const fn calculate_next_seq_count(seq_count: u16) -> u16 {
        ((seq_count as u32 + 1) % SEQ_COUNT_MODULUS) as u16
    }

    /// The current count for `apid`, storing the incremented value for next
    /// time (C++ `getAndIncrementSeqCount`). A never-seen APID starts at 0:
    /// the miss leaves 0 and inserts 1.
    ///
    /// The C++ asserts that the insert succeeds; here the capacity invariant
    /// (one slot per [`Apid`] constant) makes that unreachable, and the
    /// assert is kept for the same reason.
    fn get_and_increment_locked(state: &mut ApidState, apid: Apid) -> u16 {
        let seq_count = state.find(apid).unwrap_or(0);
        let inserted = state.insert(apid, Self::calculate_next_seq_count(seq_count));
        fw_assert!(inserted, apid.as_repr() as i32);
        seq_count
    }

    /// `getApidSeqCountIn` handler: hand out the current count for `apid`
    /// and increment it. The second port argument is unused (C++ parity: the
    /// single `ApidSequenceCount` port type serves both directions).
    fn get_apid_seq_count_in_handler(
        &self,
        _port_num: FwIndexType,
        apid: Apid,
        _unused: u16,
    ) -> u16 {
        let mut state = self.state.lock().unwrap();
        Self::get_and_increment_locked(&mut state, apid)
    }

    /// `validateApidSeqCountIn` handler: compare `received_seq_count` with
    /// the expected count, and on a mismatch emit `UnexpectedSequenceCount`
    /// and resynchronize the onboard counter to `received + 1` so counting
    /// can continue. ALWAYS returns the RECEIVED count, never the expected
    /// one (C++ parity — the deframer stores what came in).
    fn validate_apid_seq_count_in_handler(
        &self,
        _port_num: FwIndexType,
        apid: Apid,
        received_seq_count: u16,
    ) -> u16 {
        let mismatch = {
            let mut state = self.state.lock().unwrap();
            let expected = Self::get_and_increment_locked(&mut state, apid);
            if received_seq_count == expected {
                None
            } else {
                // Resync: the next expected count follows what was received.
                let inserted =
                    state.insert(apid, Self::calculate_next_seq_count(received_seq_count));
                fw_assert!(inserted, apid.as_repr() as i32);
                Some(expected)
            }
        };
        // Event outside the state lock (C++ emits under the guarded mutex;
        // behaviorally equivalent, deadlock-safe).
        if let Some(expected) = mismatch {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_UNEXPECTED_SEQUENCE_COUNT,
                LogSeverity::WarningLo,
                &format!(
                    "Unexpected sequence count received. Packets may have been dropped. \
                     Transmitted: {received_seq_count} | Expected on board: {expected}"
                ),
                |buf| {
                    fw_try!(buf.serialize_u16_be(received_seq_count));
                    buf.serialize_u16_be(expected)
                },
            );
        }
        received_seq_count
    }
}

// -- Input-port adapters + factories (generated by the codegen layer) --------

input_port_adapter! {
    /// `getApidSeqCountIn` — GUARDED `Ccsds.ApidSequenceCount` input:
    /// allocate the next sequence count for an APID.
    component: ApidManager;
    adapter: GetApidSeqCountInAdapter;
    port: ApidSequenceCountPort;
    input: pub get_apid_seq_count_in;
    handler: get_apid_seq_count_in_handler;
    returns: u16;
    args { val apid: Apid, val sequence_count: u16 }
}

input_port_adapter! {
    /// `validateApidSeqCountIn` — GUARDED `Ccsds.ApidSequenceCount` input:
    /// validate a received sequence count against the onboard one.
    component: ApidManager;
    adapter: ValidateApidSeqCountInAdapter;
    port: ApidSequenceCountPort;
    input: pub validate_apid_seq_count_in;
    handler: validate_apid_seq_count_in_handler;
    returns: u16;
    args { val apid: Apid, val sequence_count: u16 }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::LogPort;
    use fprime_fw::{Deserialize, Endianness, LogBuffer, Time};
    use std::sync::Mutex as StdMutex;

    /// Records emitted events with their decoded arguments.
    #[derive(Default)]
    struct Recorder {
        events: StdMutex<Vec<(FwEventIdType, LogSeverity, u16, u16)>>,
    }

    impl LogPort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            let mut transmitted = 0u16;
            let mut expected = 0u16;
            assert!(transmitted.deserialize_from(args, Endianness::Big).is_ok());
            assert!(expected.deserialize_from(args, Endianness::Big).is_ok());
            self.events
                .lock()
                .unwrap()
                .push((id, severity, transmitted, expected));
        }
    }

    fn build() -> (Arc<ApidManager>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let mgr = ApidManager::new("apidManager");
        mgr.evt.log_out.connect(rec.clone(), 0);
        (mgr, rec)
    }

    fn get(mgr: &Arc<ApidManager>, apid: Apid) -> u16 {
        let p = mgr.get_apid_seq_count_in(0);
        p.target.invoke(p.port_num, apid, 0)
    }

    fn validate(mgr: &Arc<ApidManager>, apid: Apid, received: u16) -> u16 {
        let p = mgr.validate_apid_seq_count_in(0);
        p.target.invoke(p.port_num, apid, received)
    }

    /// The first packet for a never-seen APID carries count 0, then 1, 2...
    #[test]
    fn first_count_for_new_apid_is_zero() {
        let (mgr, rec) = build();
        assert_eq!(get(&mgr, Apid::FwPacketTelem), 0);
        assert_eq!(get(&mgr, Apid::FwPacketTelem), 1);
        assert_eq!(get(&mgr, Apid::FwPacketTelem), 2);
        assert!(rec.events.lock().unwrap().is_empty());
    }

    /// Each APID counts independently.
    #[test]
    fn counters_are_independent_per_apid() {
        let (mgr, _rec) = build();
        assert_eq!(get(&mgr, Apid::FwPacketTelem), 0);
        assert_eq!(get(&mgr, Apid::FwPacketLog), 0);
        assert_eq!(get(&mgr, Apid::FwPacketTelem), 1);
        assert_eq!(get(&mgr, Apid::FwPacketFile), 0);
        assert_eq!(get(&mgr, Apid::FwPacketLog), 1);
        assert_eq!(get(&mgr, Apid::FwPacketTelem), 2);
        assert_eq!(get(&mgr, Apid::FwPacketFile), 1);
    }

    /// Every APID constant fits the table (capacity = NUM_CONSTANTS).
    #[test]
    fn table_holds_every_apid_constant() {
        let (mgr, _rec) = build();
        assert_eq!(ApidManager::MAX_TRACKED_APIDS, 12);
        for apid in Apid::VALUES {
            assert_eq!(get(&mgr, *apid), 0);
        }
        for apid in Apid::VALUES {
            assert_eq!(get(&mgr, *apid), 1);
        }
    }

    /// The counter is 14 bits: 0x3FFF wraps back to 0.
    #[test]
    fn sequence_count_wraps_at_fourteen_bits() {
        assert_eq!(ApidManager::calculate_next_seq_count(0), 1);
        assert_eq!(ApidManager::calculate_next_seq_count(0x3FFE), 0x3FFF);
        assert_eq!(ApidManager::calculate_next_seq_count(0x3FFF), 0);
        // Values above the 14-bit range still wrap by modulus, never panic.
        assert_eq!(ApidManager::calculate_next_seq_count(0xFFFF), 0);

        let (mgr, _rec) = build();
        // Drive the APID's counter to the top through the validate port.
        assert_eq!(validate(&mgr, Apid::FwPacketCommand, 0x3FFF), 0x3FFF);
        assert_eq!(get(&mgr, Apid::FwPacketCommand), 0);
        assert_eq!(get(&mgr, Apid::FwPacketCommand), 1);
    }

    /// Matching counts validate silently and always return the received one.
    #[test]
    fn matching_sequence_count_emits_nothing() {
        let (mgr, rec) = build();
        assert_eq!(validate(&mgr, Apid::FwPacketCommand, 0), 0);
        assert_eq!(validate(&mgr, Apid::FwPacketCommand, 1), 1);
        assert_eq!(validate(&mgr, Apid::FwPacketCommand, 2), 2);
        assert!(rec.events.lock().unwrap().is_empty());
    }

    /// A gap emits WARNING_LO `UnexpectedSequenceCount(transmitted,
    /// expected)`, returns the RECEIVED count, and resyncs so the next
    /// in-order packet is silent.
    #[test]
    fn mismatch_emits_event_and_resyncs() {
        let (mgr, rec) = build();
        assert_eq!(validate(&mgr, Apid::FwPacketCommand, 0), 0);
        // Ground jumped to 7 (packets 1..6 lost).
        assert_eq!(validate(&mgr, Apid::FwPacketCommand, 7), 7);
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(
                ApidManager::EVENTID_UNEXPECTED_SEQUENCE_COUNT,
                LogSeverity::WarningLo,
                7,
                1
            )]
        );
        // Resynced: 8 is now expected.
        assert_eq!(validate(&mgr, Apid::FwPacketCommand, 8), 8);
        assert_eq!(rec.events.lock().unwrap().len(), 1);
    }

    /// The resync wraps at 14 bits too: after receiving 0x3FFF the next
    /// expected count is 0, not 0x4000.
    #[test]
    fn resync_wraps_at_fourteen_bits() {
        let (mgr, rec) = build();
        // First-ever validate for this APID expects 0, so 0x3FFF mismatches.
        assert_eq!(validate(&mgr, Apid::FwPacketLog, 0x3FFF), 0x3FFF);
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(
                ApidManager::EVENTID_UNEXPECTED_SEQUENCE_COUNT,
                LogSeverity::WarningLo,
                0x3FFF,
                0
            )]
        );
        // Resynced to (0x3FFF + 1) & 0x3FFF == 0: the next packet is silent.
        assert_eq!(validate(&mgr, Apid::FwPacketLog, 0), 0);
        assert_eq!(rec.events.lock().unwrap().len(), 1);
    }

    /// Both ports share one table: a framer allocation advances the count
    /// the deframer validates against.
    #[test]
    fn get_and_validate_share_one_table() {
        let (mgr, rec) = build();
        assert_eq!(get(&mgr, Apid::FwPacketTelem), 0); // stores 1
        assert_eq!(validate(&mgr, Apid::FwPacketTelem, 1), 1); // expects 1: ok
        assert!(rec.events.lock().unwrap().is_empty());
        assert_eq!(get(&mgr, Apid::FwPacketTelem), 2);
    }

    /// The table is shared across threads through the guarded ports: the
    /// counts handed out are a permutation of 0..N with no duplicates.
    #[test]
    fn concurrent_allocation_hands_out_unique_counts() {
        let (mgr, _rec) = build();
        let mut handles = Vec::new();
        for _ in 0..4 {
            let mgr = mgr.clone();
            handles.push(std::thread::spawn(move || {
                let port = mgr.get_apid_seq_count_in(0);
                (0..50)
                    .map(|_| port.target.invoke(port.port_num, Apid::FwPacketDp, 0))
                    .collect::<Vec<_>>()
            }));
        }
        let mut counts: Vec<u16> = handles
            .into_iter()
            .flat_map(|h| h.join().unwrap())
            .collect();
        counts.sort_unstable();
        assert_eq!(counts, (0..200u16).collect::<Vec<_>>());
    }
}
