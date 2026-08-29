//! # Svc::BufferManager — buffer pool manager (passive component)
//!
//! Port of `Svc/BufferManager/BufferManagerComponentImpl.{cpp,hpp}` and
//! `BufferManager.fpp` per `docs/cpp-analysis/utils-misc.md`
//! (BufferManager section + gotchas).
//!
//! All three input ports (`bufferGetCallee`, `bufferSendIn`, `schedIn`) are
//! guarded by the single component mutex. [`BufferManager::setup`]
//! pre-allocates per-slot storage from a set of size bins (ascending sizes
//! recommended — allocation is a linear first-fit over the flat slot
//! array). The `Fw::Buffer` handed out carries
//! `context = (mgr_id << 16) | slot_index`, the ONLY lookup key on return;
//! a bad return (wrong manager, out-of-range index, unallocated slot,
//! foreign storage, oversize) asserts — it is not an error return.
//!
//! Owned-storage adaptation: the C++ pool is one raw allocation with
//! `Fw::Buffer` views into it; here each slot owns a `BufferStorage`
//! (`Box<[u8]>`) that MOVES into the returned [`Buffer`] on get and moves
//! back on return. The C++ data-pointer range asserts map to a storage
//! capacity check.

use fprime_comp::{
    BufferGetPort, BufferSendPort, EventGlue, EventThrottle, PassiveBase, PortRef, SchedPort,
    TlmGlue,
};
use fprime_config::buffer_manager::MAX_NUM_BINS;
use fprime_config::{FwChanIdType, FwEventIdType, FwIdType, FwIndexType, FwSizeType};
use fprime_fw::{Buffer, BufferStorage, LogSeverity, SerBuf, SerializeStatus, fw_assert};
use std::sync::{Arc, Mutex};

/// One user bin: `num_buffers` buffers of `buffer_size` bytes each
/// (C++ `BufferMgr::BufferBin`). Order bins by ascending `buffer_size` so
/// the first-fit scan behaves as a best-fit.
#[derive(Debug, Clone, Copy)]
pub struct BufferBin {
    /// Size of each buffer in this bin.
    pub buffer_size: FwSizeType,
    /// Number of buffers in this bin (0 = unused bin).
    pub num_buffers: u16,
}

impl BufferManager {
    /// `NoBuffsAvailable(size: FwSizeType)` — WARNING_HI, id 0x00,
    /// `throttle 10` ("No available buffers of size {}").
    pub const EVENTID_NO_BUFFS_AVAILABLE: FwEventIdType = 0x00;
    /// `NullEmptyBuffer` — WARNING_HI, id 0x01, `throttle 10`
    /// ("Received null pointer and zero size buffer").
    pub const EVENTID_NULL_EMPTY_BUFFER: FwEventIdType = 0x01;
    /// FPP `throttle 10` on both events.
    pub const EVENT_THROTTLE: u32 = 10;

    /// `TotalBuffs: U32` — id 0x00, update on change.
    pub const CHANID_TOTAL_BUFFS: FwChanIdType = 0x00;
    /// `CurrBuffs: U32` — id 0x01, update on change.
    pub const CHANID_CURR_BUFFS: FwChanIdType = 0x01;
    /// `HiBuffs: U32` — id 0x02, update on change.
    pub const CHANID_HI_BUFFS: FwChanIdType = 0x02;
    /// `NoBuffs: U32` — id 0x03, update on change.
    pub const CHANID_NO_BUFFS: FwChanIdType = 0x03;
    /// `EmptyBuffs: U32` — id 0x04, update on change.
    pub const CHANID_EMPTY_BUFFS: FwChanIdType = 0x04;
}

/// One pool slot (C++ `AllocatedBuffer`): the storage is `Some` while the
/// slot is free and moves out into the handed-out [`Buffer`].
struct Slot {
    /// The slot's storage; `None` while the buffer is out on loan.
    storage: Option<BufferStorage>,
    /// The slot's fixed capacity (its bin's `buffer_size`).
    size: usize,
    /// Slot is currently allocated (loaned out).
    allocated: bool,
}

/// On-change suppression state for one telemetry channel (the autocoded
/// `update on change` bookkeeping: last sent value, or `None` before the
/// first write).
type LastTlm = Option<u32>;

/// Guarded component state (the single component mutex of the C++ guarded
/// ports).
struct BmState {
    /// Setup has completed and the pool is live.
    setup: bool,
    /// Setup happened at least once (C++ `m_buffers != nullptr`).
    was_setup: bool,
    /// Stored manager id for buffer checking.
    mgr_id: u16,
    /// The flat slot array, in bin order.
    slots: Box<[Slot]>,
    /// High watermark for allocations.
    high_water: u32,
    /// Currently allocated buffers.
    curr_buffs: u32,
    /// Failed allocations.
    no_buffs: u32,
    /// Empty buffers returned.
    empty_buffs: u32,
    /// `update on change` last values, in write order:
    /// HiBuffs, CurrBuffs, TotalBuffs, NoBuffs, EmptyBuffs.
    last_tlm: [LastTlm; 5],
}

/// The BufferManager component. Construct with [`BufferManager::new`],
/// `set_id_base`, connect ports, then call [`BufferManager::setup`] before
/// any port traffic.
pub struct BufferManager {
    /// Passive core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (`eventOut`/`textEventOut`) + `timeCaller`.
    pub evt: EventGlue,
    /// Telemetry port (`tlmOut`).
    pub tlm: TlmGlue,
    /// Throttle for `NoBuffsAvailable` (`throttle 10`).
    no_buffs_throttle: EventThrottle,
    /// Throttle for `NullEmptyBuffer` (`throttle 10`).
    null_empty_throttle: EventThrottle,
    /// Guarded state.
    state: Mutex<BmState>,
}

impl BufferManager {
    /// Construct the component (pool empty until [`BufferManager::setup`]).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            no_buffs_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            null_empty_throttle: EventThrottle::new(Self::EVENT_THROTTLE),
            state: Mutex::new(BmState {
                setup: false,
                was_setup: false,
                mgr_id: 0,
                slots: Box::new([]),
                high_water: 0,
                curr_buffs: 0,
                no_buffs: 0,
                empty_buffs: 0,
                last_tlm: [None; 5],
            }),
        })
    }

    fn id_base(&self) -> FwIdType {
        self.base.get_id_base()
    }

    /// C++ `setup(mgrId, memId, allocator, bins)` — the allocator/memId pair
    /// is replaced by owned per-slot storage. Pre-allocates every slot;
    /// asserts on more than [`MAX_NUM_BINS`] bins, on a total slot count
    /// above `u16::MAX` (slot indices must fit the low half of the context
    /// word), and on double setup without an intervening
    /// [`BufferManager::cleanup`].
    pub fn setup(&self, mgr_id: u16, bins: &[BufferBin]) {
        // C++ has a fixed BUFFERMGR_MAX_NUM_BINS-sized bin array.
        fw_assert!(bins.len() <= MAX_NUM_BINS, bins.len());

        let mut state = self.state.lock().unwrap();
        // Re-setup without cleanup would leak the C++ allocation; here it
        // is simply a misuse assert.
        fw_assert!(!state.setup);

        // Count slots first (C++ parity: total structs bounded by U16::MAX
        // so the index fits the low half of the U32 context).
        let mut num_structs: u32 = 0;
        for bin in bins {
            fw_assert!(
                u32::from(u16::MAX) - num_structs >= u32::from(bin.num_buffers),
                num_structs,
                bin.num_buffers
            );
            num_structs += u32::from(bin.num_buffers);
        }

        let mut slots = Vec::with_capacity(num_structs as usize);
        for bin in bins {
            for _ in 0..bin.num_buffers {
                slots.push(Slot {
                    storage: Some(vec![0u8; bin.buffer_size as usize].into_boxed_slice()),
                    size: bin.buffer_size as usize,
                    allocated: false,
                });
            }
        }

        state.mgr_id = mgr_id;
        state.slots = slots.into_boxed_slice();
        state.setup = true;
        state.was_setup = true;
    }

    /// C++ `cleanup()`: drop the pool. Asserts if setup never happened;
    /// idempotent afterwards. Buffers still out on loan cannot be returned
    /// once cleaned (their slots are gone — the C++ equivalent frees the
    /// arena out from under them).
    pub fn cleanup(&self) {
        let mut state = self.state.lock().unwrap();
        fw_assert!(state.was_setup);
        if state.setup {
            state.slots = Box::new([]);
            state.setup = false;
        }
    }

    // -- Input-port factories (topology wiring surface) ---------------------

    /// `bufferGetCallee` — SYNC GUARDED `Fw.BufferGet` input.
    pub fn buffer_get_callee_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn BufferGetPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `bufferSendIn` — SYNC GUARDED `Fw.BufferSend` input.
    pub fn buffer_send_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn BufferSendPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `schedIn` — SYNC GUARDED `Svc.Sched` input.
    pub fn sched_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SchedPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Handlers (caller thread, under the component mutex) ----------------

    /// `bufferGetCallee_handler`: linear first-fit over the flat slot array
    /// (best-fit only when bins are ascending). Success: mark allocated,
    /// bump counters, hand out the slot storage sized to the request.
    /// Failure: throttled WARNING_HI `NoBuffsAvailable(size)`, `no_buffs`++,
    /// and an invalid (empty) [`Buffer`].
    fn buffer_get_callee_handler(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
        let mut failed = false;
        let result = {
            let mut state = self.state.lock().unwrap();
            // C++ parity: FW_ASSERT(m_setup).
            fw_assert!(state.setup);

            let found = state
                .slots
                .iter()
                .position(|slot| !slot.allocated && size <= slot.size as FwSizeType);
            match found {
                Some(index) => {
                    state.curr_buffs += 1;
                    if state.curr_buffs > state.high_water {
                        state.high_water = state.curr_buffs;
                    }
                    let mgr_id = state.mgr_id;
                    let slot = &mut state.slots[index];
                    slot.allocated = true;
                    // Invariant: a free slot always holds its storage.
                    let storage = slot.storage.take();
                    fw_assert!(storage.is_some(), index);
                    let context = (u32::from(mgr_id) << 16) | (index as u32);
                    let mut buffer = Buffer::from_storage(storage.unwrap_or_default(), context);
                    // Window sized to the request (C++ copy.setSize(size)).
                    buffer.set_size(size as usize);
                    Some(buffer)
                }
                None => {
                    state.no_buffs += 1;
                    failed = true;
                    None
                }
            }
        };
        // Event outside the state lock (C++ emits under the guarded mutex;
        // behaviorally equivalent, deadlock-safe).
        if failed && self.no_buffs_throttle.ok_to_emit() {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_NO_BUFFS_AVAILABLE,
                LogSeverity::WarningHi,
                &format!("No available buffers of size {size}"),
                |buf| buf.serialize_u64_be(size),
            );
        }
        result.unwrap_or_else(Buffer::empty)
    }

    /// `bufferSendIn_handler`: a storage-less, zero-size buffer (the C++
    /// null-and-empty check) is a throttled WARNING_HI `NullEmptyBuffer` +
    /// `empty_buffs`++. Otherwise the context word is decoded and validated
    /// with fw_asserts (manager id, index range, slot allocated, own
    /// storage, size not grown) and the storage moves back into its slot.
    fn buffer_send_in_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        let mut empty_return = false;
        {
            let mut state = self.state.lock().unwrap();
            fw_assert!(state.setup);

            // C++: data == nullptr && size == 0. An empty non-null buffer
            // (size legitimately reduced to 0) takes the normal path.
            if buffer.capacity() == 0 && buffer.size() == 0 {
                state.empty_buffs += 1;
                empty_return = true;
            } else {
                let context = buffer.context();
                let id = (context & 0xFFFF) as usize;
                let mgr_id = (context >> 16) as u16;
                fw_assert!(id < state.slots.len(), id, state.slots.len());
                fw_assert!(mgr_id == state.mgr_id, mgr_id, id, state.mgr_id);
                fw_assert!(state.slots[id].allocated, id, state.mgr_id);
                // Owned-model analog of the C++ data-pointer range asserts:
                // the returned storage must be the slot's own storage.
                fw_assert!(buffer.capacity() == state.slots[id].size, id, state.mgr_id);
                // Users may shrink buffers, never grow them (structurally
                // guaranteed by Buffer::set_size, asserted for C++ parity).
                fw_assert!(buffer.size() <= state.slots[id].size, id, state.mgr_id);
                state.slots[id].storage = Some(buffer.into_storage());
                state.slots[id].allocated = false;
                state.curr_buffs -= 1;
            }
        }
        if empty_return && self.null_empty_throttle.ok_to_emit() {
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_NULL_EMPTY_BUFFER,
                LogSeverity::WarningHi,
                "Received null pointer and zero size buffer",
                |_buf| SerializeStatus::Ok,
            );
        }
    }

    /// `schedIn_handler`: write the five U32 channels, `update on change`,
    /// in the C++ write order HiBuffs, CurrBuffs, TotalBuffs, NoBuffs,
    /// EmptyBuffs.
    fn sched_in_handler(&self, _port_num: FwIndexType, _context: u32) {
        // Snapshot + on-change decision under the lock; port invocations
        // outside it.
        let mut writes: [Option<(FwChanIdType, u32)>; 5] = [None; 5];
        {
            let mut state = self.state.lock().unwrap();
            let values: [(FwChanIdType, u32); 5] = [
                (Self::CHANID_HI_BUFFS, state.high_water),
                (Self::CHANID_CURR_BUFFS, state.curr_buffs),
                (Self::CHANID_TOTAL_BUFFS, state.slots.len() as u32),
                (Self::CHANID_NO_BUFFS, state.no_buffs),
                (Self::CHANID_EMPTY_BUFFS, state.empty_buffs),
            ];
            for (i, &(chan, value)) in values.iter().enumerate() {
                if state.last_tlm[i] != Some(value) {
                    state.last_tlm[i] = Some(value);
                    writes[i] = Some((chan, value));
                }
            }
        }
        let time_tag = self.evt.time_get();
        for write in writes.iter().flatten() {
            let (chan, value) = *write;
            self.tlm.tlm_write(self.id_base(), chan, &value, time_tag);
        }
    }
}

// -- Guarded sync ports, implemented directly on the component --------------

impl BufferGetPort for BufferManager {
    fn invoke(&self, port_num: FwIndexType, size: FwSizeType) -> Buffer {
        self.buffer_get_callee_handler(port_num, size)
    }
}

impl BufferSendPort for BufferManager {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        self.buffer_send_in_handler(port_num, buffer);
    }
}

impl SchedPort for BufferManager {
    fn invoke(&self, port_num: FwIndexType, context: u32) {
        self.sched_in_handler(port_num, context);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{LogPort, LogTextPort, TimePort, TlmPort};
    use fprime_fw::{LogBuffer, TextLogString, Time, TimeBase, TlmBuffer};

    const ID_BASE: FwIdType = 0x800;
    const MGR_ID: u16 = 0x0102;

    #[derive(Default)]
    struct GroundStub {
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        texts: Mutex<Vec<(FwEventIdType, String)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
    }

    impl LogPort for GroundStub {
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

    impl LogTextPort for GroundStub {
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

    impl TlmPort for GroundStub {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            _time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            self.tlm.lock().unwrap().push((id, val.as_slice().to_vec()));
        }
    }

    struct TimeStub;
    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::new(TimeBase::TbWorkstationTime, 0, 9, 0);
        }
    }

    fn build() -> (Arc<BufferManager>, Arc<GroundStub>) {
        let comp = BufferManager::new("bufferManager");
        let ground = Arc::new(GroundStub::default());
        comp.base.set_id_base(ID_BASE);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.evt.time_out.connect(Arc::new(TimeStub), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        (comp, ground)
    }

    /// Two bins: 2 x 16 bytes then 1 x 64 bytes (ascending, as documented).
    fn bins() -> Vec<BufferBin> {
        vec![
            BufferBin {
                buffer_size: 16,
                num_buffers: 2,
            },
            BufferBin {
                buffer_size: 64,
                num_buffers: 1,
            },
        ]
    }

    fn get(comp: &Arc<BufferManager>, size: FwSizeType) -> Buffer {
        let port = comp.buffer_get_callee_in(0);
        port.target.invoke(port.port_num, size)
    }

    fn send_back(comp: &Arc<BufferManager>, buffer: Buffer) {
        let port = comp.buffer_send_in(0);
        port.target.invoke(port.port_num, buffer);
    }

    fn sched(comp: &Arc<BufferManager>) {
        let port = comp.sched_in(0);
        port.target.invoke(port.port_num, 0);
    }

    fn ctx(index: u32) -> u32 {
        (u32::from(MGR_ID) << 16) | index
    }

    #[test]
    fn first_fit_allocation_and_context_encoding() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());

        let b0 = get(&comp, 10);
        assert!(b0.is_valid());
        assert_eq!(b0.size(), 10);
        assert_eq!(b0.capacity(), 16);
        assert_eq!(b0.context(), ctx(0));

        let b1 = get(&comp, 16);
        assert_eq!(b1.capacity(), 16);
        assert_eq!(b1.context(), ctx(1));

        // Small request now spills into the 64-byte slot (first fit).
        let b2 = get(&comp, 1);
        assert_eq!(b2.capacity(), 64);
        assert_eq!(b2.size(), 1);
        assert_eq!(b2.context(), ctx(2));
    }

    #[test]
    fn oversized_request_skips_small_slots() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());
        // 17 bytes cannot use the 16-byte slots: goes straight to slot 2.
        let b = get(&comp, 17);
        assert_eq!(b.capacity(), 64);
        assert_eq!(b.context(), ctx(2));
    }

    #[test]
    fn exhaustion_returns_invalid_buffer_with_throttled_event() {
        let (comp, ground) = build();
        comp.setup(MGR_ID, &bins());
        let _keep: Vec<Buffer> = (0..3).map(|_| get(&comp, 8)).collect();
        // Pool exhausted: 12 failed requests, only 10 events (throttle 10).
        for _ in 0..12 {
            let b = get(&comp, 8);
            assert!(!b.is_valid());
            assert_eq!(b.size(), 0);
            assert_eq!(b.capacity(), 0);
        }
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 10);
        let abs_id = ID_BASE + BufferManager::EVENTID_NO_BUFFS_AVAILABLE;
        assert_eq!(events[0].0, abs_id);
        assert_eq!(events[0].1, LogSeverity::WarningHi);
        // arg is the FwSizeType (u64 BE) requested size
        assert_eq!(events[0].2, 8u64.to_be_bytes().to_vec());
        drop(events);
        let texts = ground.texts.lock().unwrap();
        assert_eq!(texts[0].1, "No available buffers of size 8");
    }

    #[test]
    fn request_larger_than_any_slot_fails_even_when_pool_empty() {
        let (comp, ground) = build();
        comp.setup(MGR_ID, &bins());
        let b = get(&comp, 65);
        assert!(!b.is_valid());
        assert_eq!(ground.events.lock().unwrap().len(), 1);
    }

    #[test]
    fn return_lifecycle_recycles_the_slot() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());
        let mut b = get(&comp, 12);
        assert_eq!(b.context(), ctx(0));
        b.data_mut().fill(0x5A); // user writes through the window
        send_back(&comp, b);
        // Same slot is handed out again (first fit).
        let b_again = get(&comp, 12);
        assert_eq!(b_again.context(), ctx(0));
        // Storage was recycled, not reallocated: user bytes persist.
        assert_eq!(b_again.data(), &[0x5A; 12][..]);
        send_back(&comp, b_again);
        let state = comp.state.lock().unwrap();
        assert_eq!(state.curr_buffs, 0);
        assert_eq!(state.high_water, 1);
    }

    #[test]
    fn shrunk_buffer_return_is_accepted() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());
        let mut b = get(&comp, 16);
        b.set_size(0); // users may shrink, even to zero
        send_back(&comp, b);
        assert_eq!(comp.state.lock().unwrap().curr_buffs, 0);
    }

    #[test]
    fn empty_return_counts_and_emits_null_empty_buffer() {
        let (comp, ground) = build();
        comp.setup(MGR_ID, &bins());
        // 12 empty returns -> emptyBuffs = 12, 10 events (throttle 10).
        for _ in 0..12 {
            send_back(&comp, Buffer::empty());
        }
        assert_eq!(comp.state.lock().unwrap().empty_buffs, 12);
        let events = ground.events.lock().unwrap();
        assert_eq!(events.len(), 10);
        let abs_id = ID_BASE + BufferManager::EVENTID_NULL_EMPTY_BUFFER;
        assert_eq!(events[0].0, abs_id);
        assert_eq!(events[0].1, LogSeverity::WarningHi);
        assert!(events[0].2.is_empty());
        drop(events);
        assert_eq!(
            ground.texts.lock().unwrap()[0].1,
            "Received null pointer and zero size buffer"
        );
    }

    #[test]
    fn telemetry_ids_values_and_on_change_suppression() {
        let (comp, ground) = build();
        comp.setup(MGR_ID, &bins());
        let b = get(&comp, 8);
        let _fail = get(&comp, 100); // no_buffs = 1
        send_back(&comp, Buffer::empty()); // empty_buffs = 1

        sched(&comp);
        {
            let tlm = ground.tlm.lock().unwrap();
            // C++ write order: HiBuffs, CurrBuffs, TotalBuffs, NoBuffs,
            // EmptyBuffs — all first-time writes.
            let expected: Vec<(FwChanIdType, Vec<u8>)> = vec![
                (
                    ID_BASE + BufferManager::CHANID_HI_BUFFS,
                    1u32.to_be_bytes().to_vec(),
                ),
                (
                    ID_BASE + BufferManager::CHANID_CURR_BUFFS,
                    1u32.to_be_bytes().to_vec(),
                ),
                (
                    ID_BASE + BufferManager::CHANID_TOTAL_BUFFS,
                    3u32.to_be_bytes().to_vec(),
                ),
                (
                    ID_BASE + BufferManager::CHANID_NO_BUFFS,
                    1u32.to_be_bytes().to_vec(),
                ),
                (
                    ID_BASE + BufferManager::CHANID_EMPTY_BUFFS,
                    1u32.to_be_bytes().to_vec(),
                ),
            ];
            assert_eq!(*tlm, expected);
        }

        // Nothing changed: on-change suppresses every channel.
        sched(&comp);
        assert_eq!(ground.tlm.lock().unwrap().len(), 5);

        // Return the buffer: only CurrBuffs changes (HiBuffs stays 1).
        send_back(&comp, b);
        sched(&comp);
        let tlm = ground.tlm.lock().unwrap();
        assert_eq!(tlm.len(), 6);
        assert_eq!(
            tlm[5],
            (
                ID_BASE + BufferManager::CHANID_CURR_BUFFS,
                0u32.to_be_bytes().to_vec()
            )
        );
    }

    #[test]
    fn high_water_tracks_peak_allocation() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());
        let b0 = get(&comp, 8);
        let b1 = get(&comp, 8);
        let b2 = get(&comp, 8);
        send_back(&comp, b0);
        send_back(&comp, b1);
        send_back(&comp, b2);
        let state = comp.state.lock().unwrap();
        assert_eq!(state.high_water, 3);
        assert_eq!(state.curr_buffs, 0);
    }

    #[test]
    fn cleanup_then_resetup_works() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());
        comp.cleanup();
        comp.cleanup(); // idempotent
        comp.setup(MGR_ID, &bins());
        assert!(get(&comp, 8).is_valid());
    }

    #[test]
    #[should_panic]
    fn get_before_setup_asserts() {
        let (comp, _ground) = build();
        let _ = get(&comp, 8);
    }

    #[test]
    #[should_panic]
    fn return_before_setup_asserts() {
        let (comp, _ground) = build();
        send_back(&comp, Buffer::allocate(4));
    }

    #[test]
    #[should_panic]
    fn double_setup_asserts() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());
        comp.setup(MGR_ID, &bins());
    }

    #[test]
    #[should_panic]
    fn too_many_bins_asserts() {
        let (comp, _ground) = build();
        let many = vec![
            BufferBin {
                buffer_size: 1,
                num_buffers: 1,
            };
            MAX_NUM_BINS + 1
        ];
        comp.setup(MGR_ID, &many);
    }

    #[test]
    #[should_panic]
    fn total_slots_beyond_u16_max_asserts() {
        let (comp, _ground) = build();
        let bins = [
            BufferBin {
                buffer_size: 1,
                num_buffers: 40_000,
            },
            BufferBin {
                buffer_size: 1,
                num_buffers: 30_000,
            },
        ];
        comp.setup(MGR_ID, &bins);
    }

    #[test]
    #[should_panic]
    fn return_with_wrong_manager_id_asserts() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());
        let mut b = get(&comp, 8);
        b.set_context(u32::from(MGR_ID + 1) << 16);
        send_back(&comp, b);
    }

    #[test]
    #[should_panic]
    fn return_with_out_of_range_index_asserts() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());
        let mut b = get(&comp, 8);
        b.set_context((u32::from(MGR_ID) << 16) | 99);
        send_back(&comp, b);
    }

    #[test]
    #[should_panic]
    fn double_return_asserts() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());
        let b = get(&comp, 8);
        let context = b.context();
        send_back(&comp, b);
        // Forge a second buffer with the same context: the slot is no
        // longer allocated -> assert.
        send_back(
            &comp,
            Buffer::from_storage(vec![0u8; 16].into_boxed_slice(), context),
        );
    }

    #[test]
    #[should_panic]
    fn return_with_foreign_storage_asserts() {
        let (comp, _ground) = build();
        comp.setup(MGR_ID, &bins());
        let b = get(&comp, 8);
        // Forge a buffer with a valid context but the wrong storage size
        // (the owned-model analog of the C++ pointer-range assert).
        send_back(
            &comp,
            Buffer::from_storage(vec![0u8; 5].into_boxed_slice(), b.context()),
        );
    }
}
