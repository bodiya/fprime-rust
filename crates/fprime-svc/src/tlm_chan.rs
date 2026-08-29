//! # Svc::TlmChan — channelized telemetry storage (active component)
//!
//! Port of `Svc/TlmChan/TlmChan.{cpp,hpp,fpp}` per
//! `docs/cpp-analysis/svc-core.md` (TlmChan section + gotchas).
//!
//! Latest channel values live in a double-buffered, seeded, chained hash
//! table (guarded `TlmRecv`/`TlmGet` ports). On each async `Run` call the
//! active store is swapped and every `updated && used` entry of the now
//! inactive store is serialized into `FW_PACKET_TELEM` com packets sent via
//! `PktSend`.
//!
//! C++-parity notes carried over from the analysis:
//! - The `Run` port has NO `drop` policy: queue overflow asserts.
//! - Per-boot hash seed = folded `Os::RawTime` bytes XOR a folded stack
//!   address, forced to `0xDEADBEEF` when zero.
//! - `FwChanIdType` is `u32` in this configuration, so only the C++
//!   `sizeof(FwChanIdType) >= 4` Murmur3-finalizer hash path is compiled.
//! - Bucket-pool exhaustion drops the NEW channel with a throttled (10)
//!   WARNING_HI event — it does not assert.
//! - `Run` clears the NEW active store's updated flags right after the
//!   swap, dropping (not requeueing) entries deferred by the per-run cap.
//! - `TlmGet` searches BOTH stores; when both hit, the newer `lastUpdate`
//!   wins, and INCOMPARABLE time bases prefer the inactive entry only if
//!   its updated flag is set.

use fprime_comp::msg;
use fprime_comp::{
    ActiveBase, ActiveComponent, ComPort, ComponentDispatch, EventGlue, EventThrottle,
    MsgDispatchStatus, OutputPort, PingPort, PortRef, QueueFullPolicy, SchedPort, TlmPort,
};
use fprime_config::tlm_chan::{HASH_BUCKETS, MAX_ENTRIES_PER_RUN, NUM_TLM_HASH_SLOTS};
use fprime_config::{
    FwChanIdType, FwEnumStoreType, FwEventIdType, FwIdType, FwIndexType, FwQueuePriorityType,
    FwSizeType,
};
use fprime_fw::{
    Endianness, LinearBuffer, LogSeverity, SerBuf, SerBufAny, SerializeStatus, Time,
    TimeComparison, TlmBuffer, TlmPacket, TlmValid, fw_assert, fw_try,
};
use fprime_os::RawTime;
use std::sync::{Arc, Mutex};

// C++ parity: the static_asserts at the top of TlmChan.cpp.
const _: () = {
    assert!(NUM_TLM_HASH_SLOTS > 0, "must have at least one hash slot");
    assert!(
        HASH_BUCKETS <= u16::MAX as usize,
        "bucket indices must fit u16 links"
    );
    assert!(MAX_ENTRIES_PER_RUN > 0, "per-run cap must be positive");
    assert!(
        MAX_ENTRIES_PER_RUN <= HASH_BUCKETS,
        "per-run cap cannot exceed the bucket count"
    );
};

/// Queue message type for the async `Run` (`Svc.Sched`) input.
pub const MSG_TYPE_RUN: FwEnumStoreType = 1;
/// Queue message type for the async `pingIn` (`Svc.Ping`) input.
pub const MSG_TYPE_PING_IN: FwEnumStoreType = 2;

/// Queue message size: envelope (6) + one u32 argument, rounded up.
pub const QUEUE_MESSAGE_SIZE: FwSizeType = 16;
/// Queue priority for `Run` messages (FPP default — no qualifier).
pub const RUN_PRIORITY: FwQueuePriorityType = 1;
/// Queue priority for `pingIn` messages (FPP default — no qualifier).
pub const PING_IN_PRIORITY: FwQueuePriorityType = 1;

// Murmur3 32-bit finalizer constants (TlmChan.hpp).
const MURMUR3_C1: u32 = 0x85EB_CA6B;
const MURMUR3_C2: u32 = 0xC2B2_AE35;

impl TlmChan {
    /// `TlmChanEpochProcessingCapReached(numDeferred: U32, cumulative: U32)`
    /// — WARNING_HI (FPP auto id 0).
    pub const EVENTID_TLM_CHAN_EPOCH_PROCESSING_CAP_REACHED: FwEventIdType = 0;
    /// `TlmChanBucketPoolExhausted(Id: FwChanIdType)` — WARNING_HI,
    /// `throttle 10` (FPP auto id 1).
    pub const EVENTID_TLM_CHAN_BUCKET_POOL_EXHAUSTED: FwEventIdType = 1;
    /// FPP `throttle 10` on `TlmChanBucketPoolExhausted`.
    pub const BUCKET_POOL_EXHAUSTED_THROTTLE: u32 = 10;
}

/// `Fw.TlmGet` port: look up the latest stored value for a channel.
///
/// This trait does not exist in `fprime-comp`, so it is defined (and
/// exported) here. Signature per `Fw/Tlm/Tlm.fpp`:
/// `TlmGet(id, ref timeTag, ref val) -> Fw.TlmValid`. On a miss `val` is
/// reset to size 0 and `Invalid` is returned.
pub trait TlmGetPort: Send + Sync {
    /// Invoke the port.
    fn invoke(
        &self,
        port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) -> TlmValid;
}

/// One hash bucket (C++ `TlmEntry`, with index links instead of pointers).
struct TlmEntry {
    /// Telemetry id stored in the bucket.
    id: FwChanIdType,
    /// Set whenever a value is written; cleared when downlinked (or by the
    /// deferred-entry drop on buffer swap).
    updated: bool,
    /// Last update time.
    last_update: Time,
    /// Serialized telemetry value.
    buffer: TlmBuffer,
    /// Next bucket in the chain (`None` terminates).
    next: Option<u16>,
    /// Bucket has been allocated from the free list.
    used: bool,
    /// C++ parity: bucket number, "for testing".
    #[allow(dead_code)]
    bucket_no: FwChanIdType,
}

/// One of the two stores (C++ `TlmSet`).
struct TlmSet {
    /// Chain head indices, one per hash slot.
    slots: [Option<u16>; NUM_TLM_HASH_SLOTS],
    /// Bucket pool (exactly `HASH_BUCKETS` entries).
    buckets: Box<[TlmEntry]>,
    /// Next free bucket index (buckets `< free` are allocated).
    free: usize,
}

impl TlmSet {
    fn new() -> Self {
        let mut buckets = Vec::with_capacity(HASH_BUCKETS);
        for bucket_no in 0..HASH_BUCKETS {
            buckets.push(TlmEntry {
                id: 0,
                updated: false,
                last_update: Time::ZERO,
                buffer: TlmBuffer::new(),
                next: None,
                used: false,
                bucket_no: bucket_no as FwChanIdType,
            });
        }
        Self {
            slots: [None; NUM_TLM_HASH_SLOTS],
            buckets: buckets.into_boxed_slice(),
            free: 0,
        }
    }
}

/// Guarded state (the C++ component mutex shared by TlmRecv/TlmGet/Run).
struct TlmChanState {
    /// The double buffer (C++ `m_tlmEntries[2]`).
    sets: [TlmSet; 2],
    /// Index of the active store (C++ `m_activeBuffer`).
    active: usize,
    /// Cumulative count of runs where the per-run cap was hit
    /// (C++ `m_procCapCount`).
    proc_cap_count: u32,
}

/// The TlmChan component. Construct with [`TlmChan::new`], then follow the
/// topology order: `set_id_base` -> connect ports -> `create_queue` ->
/// `start`.
pub struct TlmChan {
    /// Active core: PassiveBase + queue + task.
    pub active: ActiveBase,
    /// Event ports (`eventOut`/`eventOutText`) + `timeCaller`.
    pub evt: EventGlue,
    /// `PktSend` — `Fw.Com` output for telemetry packets.
    pub pkt_send: OutputPort<dyn ComPort>,
    /// `pingOut` — `Svc.Ping` output.
    pub ping_out: OutputPort<dyn PingPort>,
    /// Throttle for `TlmChanBucketPoolExhausted` (`throttle 10`).
    bucket_pool_throttle: EventThrottle,
    /// Per-boot hash seed (C++ `m_hashSeed`).
    hash_seed: u32,
    /// Guarded hash-store state.
    state: Mutex<TlmChanState>,
}

impl TlmChan {
    /// Construct the component (C++ constructor: cleared stores + per-boot
    /// seed).
    pub fn new(name: &str) -> Arc<Self> {
        Self::new_with_seed(name, Self::generate_seed())
    }

    /// Construction with an explicit seed (deterministic tests).
    fn new_with_seed(name: &str, hash_seed: u32) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            evt: EventGlue::new(),
            pkt_send: OutputPort::new(),
            ping_out: OutputPort::new(),
            bucket_pool_throttle: EventThrottle::new(Self::BUCKET_POOL_EXHAUSTED_THROTTLE),
            hash_seed,
            state: Mutex::new(TlmChanState {
                sets: [TlmSet::new(), TlmSet::new()],
                active: 0,
                proc_cap_count: 0,
            }),
        })
    }

    /// Per-boot seed: fold the serialized `Os::RawTime` bytes with a
    /// rotate-8-XOR, XOR a folded stack address (varies per boot with
    /// ASLR — safe to *take*, never dereferenced), and force a non-zero
    /// result (C++ parity: 0 -> 0xDEADBEEF).
    fn generate_seed() -> u32 {
        let mut raw_time = RawTime::new();
        // C++ parity: (void)rawTime.now() — status ignored.
        let _ = raw_time.now();
        let mut time_buf = LinearBuffer::<{ RawTime::SERIALIZED_SIZE }>::new();
        let _ = fprime_fw::Serialize::serialize_to(&raw_time, &mut time_buf, Endianness::Big);

        let mut folded_time: u32 = 0;
        for &byte in time_buf.as_slice() {
            // Rotate-and-XOR each byte to avoid cancellation of equal bytes.
            folded_time = folded_time.rotate_left(8);
            folded_time ^= u32::from(byte);
        }

        // Address substitute for the C++ stack-address fold.
        let local: u32 = 0;
        let raw = std::ptr::from_ref(&local) as usize as u64;
        let folded_stack = (raw ^ (raw >> 32)) as u32;

        let seed = folded_time ^ folded_stack;
        // Keep every hash path keyed (C++ parity).
        if seed == 0 { 0xDEAD_BEEF } else { seed }
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    /// C++ `doHash` for the `sizeof(FwChanIdType) >= 4` configuration:
    /// Murmur3 32-bit finalizer over `id ^ seed`, reduced mod the slot
    /// count. (The 16-bit Wang and 8-bit `% HASH_MOD_VALUE` paths apply
    /// only to narrower `FwChanIdType` configs and are not compiled here.)
    fn do_hash(&self, id: FwChanIdType) -> usize {
        let mut h: u32 = id ^ self.hash_seed;
        h ^= h >> 16;
        h = h.wrapping_mul(MURMUR3_C1);
        h ^= h >> 13;
        h = h.wrapping_mul(MURMUR3_C2);
        h ^= h >> 16;
        (h as usize) % NUM_TLM_HASH_SLOTS
    }

    // -- Input-port factories (topology wiring surface) ---------------------

    /// `TlmRecv` — SYNC GUARDED `Fw.Tlm` input.
    pub fn tlm_recv_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn TlmPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `TlmGet` — SYNC GUARDED `Fw.TlmGet` input.
    pub fn tlm_get_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn TlmGetPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `Run` — ASYNC `Svc.Sched` input. NO `drop` policy: a full queue
    /// asserts (C++ parity: the FPP model has no overflow qualifier).
    pub fn run_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(Arc::new(RunInAdapter { comp: self.clone() }), port_num)
    }

    /// `pingIn` — ASYNC `Svc.Ping` input (default/assert policy).
    pub fn ping_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn PingPort> {
        PortRef::new(Arc::new(PingInAdapter { comp: self.clone() }), port_num)
    }

    // -- Handlers -----------------------------------------------------------

    /// `TlmRecv_handler` (caller thread, guarded): store/overwrite the
    /// channel value in the ACTIVE store, allocating a bucket on first
    /// sight; drop with a throttled WARNING_HI when the pool is exhausted.
    fn tlm_recv_handler(
        &self,
        _port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &Time,
        val: &TlmBuffer,
    ) {
        let mut exhausted = false;
        {
            let mut state = self.state.lock().unwrap();
            let index = self.do_hash(id);
            let active = state.active;
            let set = &mut state.sets[active];

            let mut entry_to_use: Option<u16> = None;
            match set.slots[index] {
                Some(head) => {
                    let mut prev = head;
                    let mut cur = Some(head);
                    // C++ parity: loop one extra time (BUCKETS + 1 bound) so
                    // a full-length chain is not fallen off early.
                    for _ in 0..=HASH_BUCKETS {
                        match cur {
                            Some(i) => {
                                if set.buckets[usize::from(i)].id == id {
                                    entry_to_use = Some(i);
                                    break;
                                }
                                prev = i;
                                cur = set.buckets[usize::from(i)].next;
                            }
                            None => {
                                // Out of buckets: drop the new channel rather
                                // than asserting (ids may arrive from external
                                // sources, e.g. a hub).
                                if set.free >= HASH_BUCKETS {
                                    exhausted = true;
                                    break;
                                }
                                let new_idx = set.free as u16;
                                set.free += 1;
                                set.buckets[usize::from(prev)].next = Some(new_idx);
                                set.buckets[usize::from(new_idx)].next = None;
                                entry_to_use = Some(new_idx);
                                break;
                            }
                        }
                    }
                }
                None => {
                    if set.free >= HASH_BUCKETS {
                        exhausted = true;
                    } else {
                        let new_idx = set.free as u16;
                        set.free += 1;
                        set.slots[index] = Some(new_idx);
                        set.buckets[usize::from(new_idx)].next = None;
                        entry_to_use = Some(new_idx);
                    }
                }
            }

            if let Some(i) = entry_to_use {
                let entry = &mut set.buckets[usize::from(i)];
                entry.used = true;
                entry.id = id;
                entry.updated = true;
                entry.last_update = *time_tag;
                entry.buffer.clone_from(val);
            } else {
                // C++ parity: FW_ASSERT(entryToUse != nullptr) on every
                // non-drop path.
                fw_assert!(exhausted, id);
            }
        }
        // Event emitted after releasing the state lock (the C++ guarded
        // handler holds its mutex here; emitting unlocked is behaviorally
        // equivalent and deadlock-safe).
        if exhausted && self.bucket_pool_throttle.ok_to_emit() {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_TLM_CHAN_BUCKET_POOL_EXHAUSTED,
                LogSeverity::WarningHi,
                &format!("TlmChan bucket pool exhausted: dropping new telemetry ID 0x{id:x}"),
                |buf| buf.serialize_u32_be(id),
            );
        }
    }

    /// `TlmGet_handler` (caller thread, guarded): search BOTH stores; when
    /// both hit, the newer `lastUpdate` wins (GT -> inactive); INCOMPARABLE
    /// prefers the inactive entry only if its updated flag is set. Miss:
    /// `val.reset_ser()` + `Invalid`.
    fn tlm_get_handler(
        &self,
        _port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) -> TlmValid {
        let state = self.state.lock().unwrap();
        let index = self.do_hash(id);

        let active_set = &state.sets[state.active];
        let inactive_set = &state.sets[1 - state.active];
        let active_hit = find_entry(active_set, index, id);
        let inactive_hit = find_entry(inactive_set, index, id);

        let chosen: Option<&TlmEntry> = match (active_hit, inactive_hit) {
            (Some(a), Some(i)) => {
                let active_entry = &active_set.buckets[usize::from(a)];
                let inactive_entry = &inactive_set.buckets[usize::from(i)];
                match Time::compare(&inactive_entry.last_update, &active_entry.last_update) {
                    TimeComparison::Gt => Some(inactive_entry),
                    TimeComparison::Incomparable => {
                        if inactive_entry.updated {
                            Some(inactive_entry)
                        } else {
                            Some(active_entry)
                        }
                    }
                    _ => Some(active_entry),
                }
            }
            (Some(a), None) => Some(&active_set.buckets[usize::from(a)]),
            (None, Some(i)) => Some(&inactive_set.buckets[usize::from(i)]),
            (None, None) => None,
        };

        match chosen {
            Some(entry) => {
                val.clone_from(&entry.buffer);
                *time_tag = entry.last_update;
                TlmValid::Valid
            }
            None => {
                val.reset_ser();
                TlmValid::Invalid
            }
        }
    }

    /// `Run_handler` (component thread): swap the double buffer, clear the
    /// NEW active store's updated flags (dropping entries deferred last
    /// cycle), then drain the inactive store into `FW_PACKET_TELEM` packets.
    fn run_handler(&self, _port_num: FwIndexType, _context: u32) {
        self.run_with_cap(MAX_ENTRIES_PER_RUN);
    }

    /// The `Run_handler` body with the per-run cap as a parameter so tests
    /// can exercise the deferral path (flight code always passes
    /// `MAX_ENTRIES_PER_RUN`; with the default config cap == bucket count,
    /// deferral is unreachable — a config-tuning safety valve).
    fn run_with_cap(&self, cap: usize) {
        // Only write packets if connected (checked before the swap).
        let Some(pkt_port) = self.pkt_send.try_get() else {
            return;
        };

        // Lock long enough to swap the active buffer and clean the new
        // active store; producers then write into it while we drain the
        // inactive one.
        {
            let mut state = self.state.lock().unwrap();
            state.active = 1 - state.active;
            let active = state.active;
            // Deferred-entry drop (C++ parity): entries skipped by the cap
            // last cycle still carry updated=true here and are cleared, not
            // requeued.
            for entry in state.sets[active].buckets.iter_mut() {
                entry.updated = false;
            }
        }

        let mut entries_processed: usize = 0;
        let mut entries_deferred: u32 = 0;

        let mut pkt = TlmPacket::new();
        let reset_stat = pkt.reset_pkt_ser();
        fw_assert!(reset_stat.is_ok(), reset_stat as i32);

        for entry_idx in 0..HASH_BUCKETS {
            // Per-entry locking approximates the C++ granularity (C++ reads
            // the inactive store lock-free and locks only to clear the
            // updated flag; safe Rust locks around both — the inactive store
            // is written by no one else, so contention is unchanged).
            loop {
                let mut state = self.state.lock().unwrap();
                let inactive = 1 - state.active;
                let entry = &mut state.sets[inactive].buckets[entry_idx];
                if !(entry.updated && entry.used) {
                    break;
                }
                if entries_processed >= cap {
                    entries_deferred += 1;
                    break;
                }
                let stat = pkt.add_value(entry.id, &entry.last_update, &entry.buffer);
                match stat {
                    SerializeStatus::Ok => {
                        entry.updated = false;
                        entries_processed += 1;
                        break;
                    }
                    SerializeStatus::NoRoomLeft => {
                        // C++ parity: if a single channel does not fit an
                        // EMPTY packet, the packet is misconfigured — assert.
                        fw_assert!(pkt.get_num_entries() > 0, stat as i32);
                        drop(state);
                        let p = pkt_port;
                        p.target.invoke(p.port_num, pkt.get_buffer_mut(), 0);
                        let reset_stat = pkt.reset_pkt_ser();
                        fw_assert!(reset_stat.is_ok(), reset_stat as i32);
                        // Retry the same entry against the empty packet.
                    }
                    other => {
                        fw_assert!(false, other as i32);
                        break;
                    }
                }
            }
        }

        // Send remnant entries.
        if pkt.get_num_entries() > 0 {
            pkt_port
                .target
                .invoke(pkt_port.port_num, pkt.get_buffer_mut(), 0);
        }

        if entries_deferred > 0 {
            let cumulative = {
                let mut state = self.state.lock().unwrap();
                state.proc_cap_count += 1;
                state.proc_cap_count
            };
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_TLM_CHAN_EPOCH_PROCESSING_CAP_REACHED,
                LogSeverity::WarningHi,
                &format!(
                    "TlmChan epoch processing cap reached: {entries_deferred} entries deferred (cumulative: {cumulative})"
                ),
                |buf| {
                    fw_try!(buf.serialize_u32_be(entries_deferred));
                    buf.serialize_u32_be(cumulative)
                },
            );
        }
    }

    /// `pingIn_handler` (component thread): echo the key on `pingOut`.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }
}

/// Walk a store's chain for `id` (bounded by the bucket count).
fn find_entry(set: &TlmSet, index: usize, id: FwChanIdType) -> Option<u16> {
    let mut cur = set.slots[index];
    for _ in 0..HASH_BUCKETS {
        match cur {
            Some(i) => {
                if set.buckets[usize::from(i)].id == id {
                    return Some(i);
                }
                cur = set.buckets[usize::from(i)].next;
            }
            None => break,
        }
    }
    None
}

// -- Guarded sync ports, implemented directly on the component --------------

impl TlmPort for TlmChan {
    fn invoke(
        &self,
        port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) {
        self.tlm_recv_handler(port_num, id, time_tag, val);
    }
}

impl TlmGetPort for TlmChan {
    fn invoke(
        &self,
        port_num: FwIndexType,
        id: FwChanIdType,
        time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) -> TlmValid {
        self.tlm_get_handler(port_num, id, time_tag, val)
    }
}

// -- Async input adapters ---------------------------------------------------

/// Adapter for the async `Run` port. NO drop policy: `Assert` (queue
/// overflow is an fw_assert, C++ parity with the missing FPP qualifier).
struct RunInAdapter {
    comp: Arc<TlmChan>,
}

impl SchedPort for RunInAdapter {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        let mut buf = LinearBuffer::<{ QUEUE_MESSAGE_SIZE as usize }>::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_RUN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(context);
        fw_assert!(status.is_ok(), status as i32);
        let _ = self
            .comp
            .active
            .queued
            .send_message(&buf, RUN_PRIORITY, QueueFullPolicy::Assert);
    }
}

/// Adapter for the async `pingIn` port (default/assert policy).
struct PingInAdapter {
    comp: Arc<TlmChan>,
}

impl PingPort for PingInAdapter {
    fn invoke(&self, port_num: FwIndexType, key: u32) {
        let mut buf = LinearBuffer::<{ QUEUE_MESSAGE_SIZE as usize }>::new();
        let status = msg::write_envelope_header(&mut buf, MSG_TYPE_PING_IN, port_num);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_u32_be(key);
        fw_assert!(status.is_ok(), status as i32);
        let _ =
            self.comp
                .active
                .queued
                .send_message(&buf, PING_IN_PRIORITY, QueueFullPolicy::Assert);
    }
}

// -- Dispatch ---------------------------------------------------------------

impl ComponentDispatch for TlmChan {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            MSG_TYPE_RUN => {
                let mut context = 0u32;
                if !buf.deserialize_u32_be(&mut context).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.run_handler(port_num, context);
                MsgDispatchStatus::Ok
            }
            MSG_TYPE_PING_IN => {
                let mut key = 0u32;
                if !buf.deserialize_u32_be(&mut key).is_ok() {
                    return MsgDispatchStatus::Error;
                }
                self.ping_in_handler(port_num, key);
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for TlmChan {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{LogPort, LogTextPort, TimePort};
    use fprime_fw::{ComBuffer, LogBuffer, TextLogString, TimeBase};
    use fprime_os::task::{Status as TaskStatus, TASK_DEFAULT};

    const ID_BASE: FwIdType = 0x400;

    /// Records every com packet sent through PktSend.
    #[derive(Default)]
    struct PktRecorder {
        pkts: Mutex<Vec<Vec<u8>>>,
    }

    impl ComPort for PktRecorder {
        fn invoke(&self, _port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
            assert_eq!(context, 0);
            self.pkts.lock().unwrap().push(data.as_slice().to_vec());
        }
    }

    /// Records events (id, severity, raw args) + text events.
    #[derive(Default)]
    struct EventRecorder {
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        texts: Mutex<Vec<(FwEventIdType, String)>>,
    }

    impl LogPort for EventRecorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.events
                .lock()
                .unwrap()
                .push((id, severity, args.as_slice().to_vec()));
        }
    }

    impl LogTextPort for EventRecorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            text: &mut TextLogString,
        ) {
            self.texts
                .lock()
                .unwrap()
                .push((id, text.as_str().unwrap_or_default().to_string()));
        }
    }

    struct TimeStub;
    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 77, 5);
        }
    }

    /// Ping echo recorder.
    #[derive(Default)]
    struct PingRecorder {
        keys: Mutex<Vec<u32>>,
    }
    impl PingPort for PingRecorder {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.keys.lock().unwrap().push(key);
        }
    }

    fn build() -> (Arc<TlmChan>, Arc<PktRecorder>, Arc<EventRecorder>) {
        let comp = TlmChan::new_with_seed("tlmChan", 0xDEAD_BEEF);
        let pkts = Arc::new(PktRecorder::default());
        let events = Arc::new(EventRecorder::default());
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.pkt_send.connect(pkts.clone(), 0);
        comp.evt.log_out.connect(events.clone(), 0);
        comp.evt.text_log_out.connect(events.clone(), 0);
        comp.evt.time_out.connect(Arc::new(TimeStub), 0);
        (comp, pkts, events)
    }

    fn recv(comp: &Arc<TlmChan>, id: FwChanIdType, time: Time, bytes: &[u8]) {
        let mut val = TlmBuffer::new();
        assert!(val.set_buff(bytes).is_ok());
        let mut t = time;
        let port = comp.tlm_recv_in(0);
        port.target.invoke(port.port_num, id, &mut t, &mut val);
    }

    fn get(comp: &Arc<TlmChan>, id: FwChanIdType) -> (TlmValid, Time, Vec<u8>) {
        let mut time = Time::ZERO;
        let mut val = TlmBuffer::new();
        // pre-dirty val to prove the miss path resets it
        assert!(val.set_buff(&[0xAA, 0xBB]).is_ok());
        let port = comp.tlm_get_in(0);
        let valid = port.target.invoke(port.port_num, id, &mut time, &mut val);
        (valid, time, val.as_slice().to_vec())
    }

    fn t(sec: u32) -> Time {
        Time::new(TimeBase::TbWorkstationTime, 0, sec, 0)
    }

    /// Expected on-wire bytes of one packet entry: [id u32][time 11B][raw].
    fn entry_bytes(id: FwChanIdType, time: &Time, value: &[u8]) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&id.to_be_bytes());
        v.extend_from_slice(&(time.get_time_base() as u16).to_be_bytes());
        v.push(time.get_context());
        v.extend_from_slice(&time.get_seconds().to_be_bytes());
        v.extend_from_slice(&time.get_useconds().to_be_bytes());
        v.extend_from_slice(value);
        v
    }

    #[test]
    fn generated_seed_is_nonzero() {
        for _ in 0..10 {
            assert_ne!(TlmChan::generate_seed(), 0);
        }
    }

    #[test]
    fn hash_stays_in_slot_range_and_is_deterministic() {
        let comp = TlmChan::new_with_seed("h", 12345);
        for id in 0..1000u32 {
            let h = comp.do_hash(id);
            assert!(h < NUM_TLM_HASH_SLOTS);
            assert_eq!(h, comp.do_hash(id));
        }
    }

    #[test]
    fn recv_then_get_roundtrip() {
        let (comp, _pkts, _events) = build();
        recv(&comp, 42, t(100), &[1, 2, 3, 4]);
        let (valid, time, bytes) = get(&comp, 42);
        assert_eq!(valid, TlmValid::Valid);
        assert_eq!(time.get_seconds(), 100);
        assert_eq!(bytes, vec![1, 2, 3, 4]);
    }

    #[test]
    fn get_miss_resets_val_and_returns_invalid() {
        let (comp, _pkts, _events) = build();
        let (valid, _time, bytes) = get(&comp, 999);
        assert_eq!(valid, TlmValid::Invalid);
        assert!(bytes.is_empty());
    }

    #[test]
    fn colliding_ids_chain_in_one_slot_and_all_resolve() {
        let (comp, _pkts, _events) = build();
        // Brute-force three ids that hash to the same slot.
        let target = comp.do_hash(0);
        let mut ids = vec![0u32];
        let mut id = 1u32;
        while ids.len() < 3 {
            if comp.do_hash(id) == target {
                ids.push(id);
            }
            id += 1;
        }
        for (i, &cid) in ids.iter().enumerate() {
            recv(&comp, cid, t(10 + i as u32), &[i as u8]);
        }
        // All three retrievable.
        for (i, &cid) in ids.iter().enumerate() {
            let (valid, time, bytes) = get(&comp, cid);
            assert_eq!(valid, TlmValid::Valid);
            assert_eq!(time.get_seconds(), 10 + i as u32);
            assert_eq!(bytes, vec![i as u8]);
        }
        // Structural check: one chain of length 3 hanging off the slot.
        let state = comp.state.lock().unwrap();
        let set = &state.sets[state.active];
        let mut chain = Vec::new();
        let mut cur = set.slots[target];
        while let Some(i) = cur {
            chain.push(set.buckets[usize::from(i)].id);
            cur = set.buckets[usize::from(i)].next;
        }
        assert_eq!(chain, ids);
        assert_eq!(set.free, 3);
    }

    #[test]
    fn overwrite_existing_id_does_not_allocate_a_new_bucket() {
        let (comp, _pkts, _events) = build();
        recv(&comp, 7, t(1), &[1]);
        recv(&comp, 7, t(2), &[2]);
        let (valid, time, bytes) = get(&comp, 7);
        assert_eq!(valid, TlmValid::Valid);
        assert_eq!(time.get_seconds(), 2);
        assert_eq!(bytes, vec![2]);
        assert_eq!(comp.state.lock().unwrap().sets[0].free, 1);
    }

    #[test]
    fn bucket_pool_exhaustion_drops_with_throttled_event() {
        let (comp, _pkts, events) = build();
        // Fill all 500 buckets of the active store.
        for id in 0..HASH_BUCKETS as u32 {
            recv(&comp, id, t(1), &[0xEE]);
        }
        assert_eq!(comp.state.lock().unwrap().sets[0].free, HASH_BUCKETS);
        // 12 new ids: all dropped, only 10 events (throttle 10).
        for id in 0..12u32 {
            recv(&comp, 10_000 + id, t(2), &[0xDD]);
        }
        for id in 0..12u32 {
            let (valid, _t, _b) = get(&comp, 10_000 + id);
            assert_eq!(valid, TlmValid::Invalid);
        }
        let evts = events.events.lock().unwrap();
        assert_eq!(evts.len(), 10);
        let abs_id = ID_BASE + TlmChan::EVENTID_TLM_CHAN_BUCKET_POOL_EXHAUSTED;
        assert_eq!(evts[0].0, abs_id);
        assert_eq!(evts[0].1, LogSeverity::WarningHi);
        assert_eq!(evts[0].2, 10_000u32.to_be_bytes().to_vec());
        drop(evts);
        let texts = events.texts.lock().unwrap();
        assert_eq!(
            texts[0].1,
            format!(
                "TlmChan bucket pool exhausted: dropping new telemetry ID 0x{:x}",
                10_000
            )
        );
        drop(texts);
        // Existing ids still update while the pool is exhausted.
        recv(&comp, 3, t(9), &[0x33]);
        let (valid, time, bytes) = get(&comp, 3);
        assert_eq!(valid, TlmValid::Valid);
        assert_eq!(time.get_seconds(), 9);
        assert_eq!(bytes, vec![0x33]);
    }

    #[test]
    fn run_downlinks_updated_entries_byte_exact() {
        let (comp, pkts, _events) = build();
        let time = t(55);
        recv(&comp, 0x10, time, &[0xCA, 0xFE]);
        recv(&comp, 0x11, time, &[0x01]);
        comp.run_handler(0, 0);
        let sent = pkts.pkts.lock().unwrap();
        assert_eq!(sent.len(), 1);
        // [descriptor 0x0001][entries in bucket-allocation order]
        let mut expected = vec![0x00, 0x01];
        expected.extend(entry_bytes(0x10, &time, &[0xCA, 0xFE]));
        expected.extend(entry_bytes(0x11, &time, &[0x01]));
        assert_eq!(sent[0], expected);
    }

    #[test]
    fn run_with_no_updates_sends_nothing() {
        let (comp, pkts, _events) = build();
        comp.run_handler(0, 0);
        assert!(pkts.pkts.lock().unwrap().is_empty());
        // A drained entry is not re-sent on the next run.
        recv(&comp, 5, t(1), &[9]);
        comp.run_handler(0, 0);
        comp.run_handler(0, 0);
        comp.run_handler(0, 0);
        assert_eq!(pkts.pkts.lock().unwrap().len(), 1);
    }

    #[test]
    fn run_splits_packets_when_full_and_readds_after_reset() {
        let (comp, pkts, _events) = build();
        // 100-byte values: entry = 4 + 11 + 100 = 115 bytes; descriptor 2.
        // 2 + 4*115 = 462 fits; the 5th entry (577) does not -> split 4 + 2.
        let val = [0x5A_u8; 100];
        for id in 0..6u32 {
            recv(&comp, 100 + id, t(1), &val);
        }
        comp.run_handler(0, 0);
        let sent = pkts.pkts.lock().unwrap();
        assert_eq!(sent.len(), 2);
        assert_eq!(sent[0].len(), 2 + 4 * 115);
        assert_eq!(sent[1].len(), 2 + 2 * 115);
    }

    #[test]
    fn run_with_unconnected_pkt_send_returns_without_swapping() {
        let comp = TlmChan::new_with_seed("noPkt", 1);
        recv(&comp, 1, t(1), &[1]);
        comp.run_handler(0, 0);
        let state = comp.state.lock().unwrap();
        assert_eq!(state.active, 0);
        // The entry is untouched (still updated).
        let set = &state.sets[0];
        assert!(set.buckets[0].updated && set.buckets[0].used);
    }

    #[test]
    fn deferred_entries_beyond_cap_are_dropped_not_requeued() {
        let (comp, pkts, events) = build();
        for id in 0..3u32 {
            recv(&comp, id, t(1), &[id as u8]);
        }
        // Cap of 2: two entries downlinked, one deferred.
        comp.run_with_cap(2);
        {
            let sent = pkts.pkts.lock().unwrap();
            assert_eq!(sent.len(), 1);
            // 2 entries of 1-byte values: 2 + 2*16.
            assert_eq!(sent[0].len(), 2 + 2 * 16);
        }
        {
            let evts = events.events.lock().unwrap();
            assert_eq!(evts.len(), 1);
            let abs_id = ID_BASE + TlmChan::EVENTID_TLM_CHAN_EPOCH_PROCESSING_CAP_REACHED;
            assert_eq!(evts[0].0, abs_id);
            assert_eq!(evts[0].1, LogSeverity::WarningHi);
            // args: numDeferred=1, cumulative=1
            assert_eq!(evts[0].2, vec![0, 0, 0, 1, 0, 0, 0, 1]);
        }
        // Gotcha: the deferred entry's updated flag is cleared by the NEXT
        // swap — it is dropped, never downlinked.
        comp.run_with_cap(2); // swap back; nothing new updated
        comp.run_with_cap(2);
        assert_eq!(pkts.pkts.lock().unwrap().len(), 1);
        // Cumulative count increments on the next deferral.
        for id in 0..3u32 {
            recv(&comp, id, t(2), &[id as u8]);
        }
        comp.run_with_cap(2);
        let evts = events.events.lock().unwrap();
        assert_eq!(evts.len(), 2);
        assert_eq!(evts[1].2, vec![0, 0, 0, 1, 0, 0, 0, 2]);
    }

    #[test]
    fn tlm_get_both_buffers_newer_timestamp_wins() {
        let (comp, _pkts, _events) = build();
        recv(&comp, 8, t(100), &[1]);
        comp.run_handler(0, 0); // drains store 0; active -> store 1
        recv(&comp, 8, t(200), &[2]);
        // active (store 1) holds t=200, inactive (store 0) t=100 -> active.
        let (valid, time, bytes) = get(&comp, 8);
        assert_eq!(valid, TlmValid::Valid);
        assert_eq!(time.get_seconds(), 200);
        assert_eq!(bytes, vec![2]);
        // Swap again: now the NEWER value sits in the INACTIVE store.
        comp.run_handler(0, 0); // active -> store 0 (still has t=100 copy)
        let (valid, time, bytes) = get(&comp, 8);
        assert_eq!(valid, TlmValid::Valid);
        assert_eq!(time.get_seconds(), 200);
        assert_eq!(bytes, vec![2]);
    }

    #[test]
    fn tlm_get_incomparable_prefers_inactive_only_when_updated() {
        let (comp, _pkts, _events) = build();
        // Entry in store 0 with TB_PROC_TIME, entry in store 1 with
        // TB_WORKSTATION_TIME -> compare() is INCOMPARABLE.
        recv(&comp, 9, Time::new(TimeBase::TbProcTime, 0, 50, 0), &[0xA0]);
        comp.run_handler(0, 0); // store 0 drained (updated=false), active -> 1
        recv(&comp, 9, t(60), &[0xA1]);
        // inactive (store 0) updated=false -> prefer ACTIVE.
        let (valid, time, bytes) = get(&comp, 9);
        assert_eq!(valid, TlmValid::Valid);
        assert_eq!(time.get_time_base(), TimeBase::TbWorkstationTime);
        assert_eq!(bytes, vec![0xA1]);
        // Force the inactive entry's updated flag on (the deferred-entry
        // situation) and verify the preference flips.
        {
            let mut state = comp.state.lock().unwrap();
            let active = state.active;
            let inactive = 1 - active;
            let idx = comp.do_hash(9);
            let head = state.sets[inactive].slots[idx].unwrap();
            // walk to the id-9 entry
            let mut cur = head;
            loop {
                if state.sets[inactive].buckets[usize::from(cur)].id == 9 {
                    break;
                }
                cur = state.sets[inactive].buckets[usize::from(cur)].next.unwrap();
            }
            state.sets[inactive].buckets[usize::from(cur)].updated = true;
        }
        let (valid, time, bytes) = get(&comp, 9);
        assert_eq!(valid, TlmValid::Valid);
        assert_eq!(time.get_time_base(), TimeBase::TbProcTime);
        assert_eq!(bytes, vec![0xA0]);
    }

    #[test]
    fn run_envelope_bytes_are_byte_exact() {
        let (comp, _pkts, _events) = build();
        comp.active.queued.create_queue(4, QUEUE_MESSAGE_SIZE);
        let run = comp.run_in(0);
        run.target.invoke(run.port_num, 0x0102_0304);
        let mut dest = [0u8; QUEUE_MESSAGE_SIZE as usize];
        let mut size: FwSizeType = 0;
        let mut priority: FwQueuePriorityType = 0;
        let status = comp.active.queued.queue().receive(
            &mut dest,
            fprime_os::queue::BlockingType::NonBlocking,
            &mut size,
            &mut priority,
        );
        assert_eq!(status, fprime_os::queue::Status::OpOk);
        assert_eq!(
            &dest[..size as usize],
            &[0, 0, 0, 1, 0, 0, 0x01, 0x02, 0x03, 0x04]
        );
        assert_eq!(priority, RUN_PRIORITY);
    }

    /// The Run port has NO drop policy — overflowing the queue asserts.
    #[test]
    #[should_panic]
    fn run_queue_overflow_asserts() {
        let (comp, _pkts, _events) = build();
        comp.active.queued.create_queue(1, QUEUE_MESSAGE_SIZE);
        let run = comp.run_in(0);
        run.target.invoke(run.port_num, 1); // fills the queue
        run.target.invoke(run.port_num, 2); // asserts
    }

    #[test]
    fn active_lifecycle_run_and_ping_end_to_end() {
        let (comp, pkts, _events) = build();
        let ping = Arc::new(PingRecorder::default());
        comp.ping_out.connect(ping.clone(), 0);
        comp.active.queued.create_queue(16, QUEUE_MESSAGE_SIZE);

        recv(&comp, 1, t(3), &[0x77]);
        comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);

        let run = comp.run_in(0);
        run.target.invoke(run.port_num, 0);
        let ping_in = comp.ping_in(0);
        ping_in.target.invoke(ping_in.port_num, 0xBEEF);

        comp.active.exit();
        assert_eq!(comp.active.join(), TaskStatus::OpOk);

        assert_eq!(pkts.pkts.lock().unwrap().len(), 1);
        assert_eq!(*ping.keys.lock().unwrap(), vec![0xBEEF]);
    }
}
