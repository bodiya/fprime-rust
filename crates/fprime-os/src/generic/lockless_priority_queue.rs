//! The lockless, ISR-safe priority queue delegate.
//!
//! Port of `Os::Generic::LocklessPriorityQueue`
//! (Os/Generic/LocklessPriorityQueue.{hpp,cpp}, design in
//! Os/Generic/docs/sdd-lockless-queue.md), configured by
//! [`fprime_config::lockless_queue`] (`config/LocklessQueueCfg.hpp`).
//! Selected as the `Os::Queue` delegate by the `lockless-queue` cargo
//! feature — the equivalent of linking upstream's
//! `DefaultLocklessPriorityQueue.cpp` instead of `DefaultPriorityQueue.cpp`
//! — and usable directly as a [`QueueInterface`] either way.
//!
//! Messages live in a fixed pool of slots allocated once at
//! [`create`](QueueInterface::create). Each slot carries one atomic
//! state-and-tag word, the only synchronizing element of the queue:
//! producers move a slot `FREE -> WRITING -> READY`, consumers
//! `READY -> READING -> FREE`, each transition bumping an ABA epoch tag in
//! the word's upper bits. A consumer scans for the READY slot with the
//! highest priority, tie-broken by the smallest (wrap-aware) publication
//! sequence, and claims it with a compare-and-swap.
//!
//! Flight-software properties (C++ parity):
//! - no allocation after `create`;
//! - non-blocking `send`/`receive` take no lock and finish within
//!   `depth * LOCKLESS_QUEUE_MAX_RETRY_PASSES` slot visits, so they may be
//!   called from interrupt context; they may report a spurious
//!   [`Status::Full`] / [`Status::Empty`] under contention (slots held
//!   mid-operation are not claimable);
//! - blocking calls poll with an `Os::Task::delay` of
//!   `LOCKLESS_QUEUE_BLOCKING_BACKOFF_US` between scans (an idle blocked
//!   receiver wakes periodically instead of sleeping until signaled) and
//!   must not be called from an interrupt.
//!
//! Safe-Rust design (deviations from C++, none observable through the
//! queue API):
//! - **Payload storage.** C++ copies payloads with plain `memcpy` into a
//!   `U8` region; the exclusive WRITING/READING ownership of the slot makes
//!   that race-free, but expressing "a plain byte buffer owned by whoever
//!   won the CAS" needs `unsafe` (`UnsafeCell`). Here each slot's payload is
//!   a run of `AtomicUsize` words written and read with `Relaxed` ordering,
//!   one machine word at a time. Atomic accesses cannot data-race, and the
//!   happens-before edges that make the bytes correct are exactly the C++
//!   ones — the producer's release store of READY / the consumer's acquire
//!   CAS out of READY, and the consumer's release store of FREE / the next
//!   producer's acquire CAS out of FREE — so a consumer reads exactly what
//!   the producer wrote. On every supported target relaxed word accesses
//!   compile to the plain loads and stores a `memcpy` would use: still
//!   lock-free, still a bounded copy, still ISR-safe. Each slot's region is
//!   rounded up to whole words (at most one word minus a byte of padding
//!   per slot).
//! - **Stored size.** C++ `m_size` is a plain `FwSizeType` accessed under
//!   exclusive ownership; here it is a `Relaxed` `AtomicUsize` (same
//!   reason; a message is at most `usize` bytes, and `FwSizeType` is
//!   64-bit, which thumbv7em has no atomics for).
//! - **Atomic widths.** The state-tag word is
//!   [`LocklessStateTagType`]
//!   (`u64` where the target has 64-bit atomics, else `u32`: 62 or 30 tag
//!   bits); sequence and counters are `AtomicU32` as in C++; the priority
//!   word is the atomic of `FwQueuePriorityType`. Rust only defines an
//!   atomic type where the target implements it natively, so the C++
//!   runtime `is_lock_free()` asserts become compile-time facts: a width the
//!   target lacks does not compile.
//! - **Slot alignment** comes from
//!   [`LOCKLESS_QUEUE_SLOT_ALIGNMENT`]
//!   through a zero-length array of an aligned marker type (a power of two
//!   from 1 to 4096; anything else does not compile, the C++
//!   `static_assert`). The allocator honors the alignment, so the C++
//!   manual over-allocation and pointer rounding are unnecessary.
//! - **Lifecycle.** The C++ delegate is constructed empty, then `create`d,
//!   and asserts on use before `create`; here `create` constructs the
//!   delegate (the uncreated state exists only in the `Os::Queue` facade).
//!   C++ `teardown()` is `Drop` (the double-free and static-destruction
//!   hazards it guards against cannot arise). Memory comes from the global
//!   allocator rather than an `Fw::MemAllocator` (this port has no
//!   allocator registry); allocation failure is still reported as
//!   [`Status::AllocationFailed`] (fallible reservation), as in C++.
//! - The C++ `create` `id`/`name` arguments are unused by the delegate and
//!   not taken.

use crate::queue::{BlockingType, QueueInterface, Status};
use crate::task::Task;
use alloc::boxed::Box;
use alloc::vec::Vec;
use fprime_config::lockless_queue::{
    LOCKLESS_QUEUE_BLOCKING_BACKOFF_US, LOCKLESS_QUEUE_MAX_RETRY_PASSES,
    LOCKLESS_QUEUE_SLOT_ALIGNMENT, LocklessStateTagType,
};
use fprime_config::{FwQueuePriorityType, FwSizeType};
use fprime_fw::TimeInterval;
use fprime_fw::fw_assert;
use fprime_fw::sync::{AtomicU32, AtomicUsize, Ordering};

// C++ static_asserts on the configuration.
const _: () = assert!(
    LOCKLESS_QUEUE_MAX_RETRY_PASSES >= 1,
    "LOCKLESS_QUEUE_MAX_RETRY_PASSES must be at least 1"
);
// A zero backoff could livelock a high-priority blocking caller against a
// lower-priority thread on a strict-priority scheduler.
const _: () = assert!(
    LOCKLESS_QUEUE_BLOCKING_BACKOFF_US > 0,
    "LOCKLESS_QUEUE_BLOCKING_BACKOFF_US must be greater than 0"
);

/// An unsigned integer width with a native atomic type: the
/// `std::atomic<T>` the C++ queue uses for the state-tag word and the
/// per-slot priority. Implemented only for the widths the target has
/// atomic compare-and-swap for, so configuring
/// `LocklessStateTagType` (or `FwQueuePriorityType`) to a width the target
/// lacks is a compile error naming this trait.
trait LocklessAtomicWord: Copy + Eq {
    /// The matching `core::sync::atomic` type.
    type Atomic: Send + Sync;
    /// Width in bits (C++ `std::numeric_limits<T>::digits`).
    const BITS: u32;
    /// A new atomic holding `value`.
    fn new_atomic(value: Self) -> Self::Atomic;
    /// Atomic load.
    fn load(atomic: &Self::Atomic, order: Ordering) -> Self;
    /// Atomic store.
    fn store(atomic: &Self::Atomic, value: Self, order: Ordering);
    /// Strong compare-and-swap (C++ `compare_exchange_strong`).
    fn compare_exchange(
        atomic: &Self::Atomic,
        current: Self,
        new: Self,
        success: Ordering,
        failure: Ordering,
    ) -> Result<Self, Self>;
    /// Widen to `u64` (for the shared tag arithmetic).
    fn to_u64(self) -> u64;
    /// Narrow from `u64`, discarding high bits (C++ unsigned wrap).
    fn from_u64_wrapping(value: u64) -> Self;
}

macro_rules! lockless_atomic_word {
    ($($width:literal => $int:ty, $atomic:ty;)*) => {$(
        #[cfg(target_has_atomic = $width)]
        impl LocklessAtomicWord for $int {
            type Atomic = $atomic;
            const BITS: u32 = <$int>::BITS;
            fn new_atomic(value: Self) -> Self::Atomic {
                <$atomic>::new(value)
            }
            fn load(atomic: &Self::Atomic, order: Ordering) -> Self {
                atomic.load(order)
            }
            fn store(atomic: &Self::Atomic, value: Self, order: Ordering) {
                atomic.store(value, order);
            }
            fn compare_exchange(
                atomic: &Self::Atomic,
                current: Self,
                new: Self,
                success: Ordering,
                failure: Ordering,
            ) -> Result<Self, Self> {
                atomic.compare_exchange(current, new, success, failure)
            }
            fn to_u64(self) -> u64 {
                u64::from(self)
            }
            fn from_u64_wrapping(value: u64) -> Self {
                // Truncation is the point: the C++ word wraps the same way.
                value as $int
            }
        }
    )*};
}

lockless_atomic_word! {
    "8" => u8, core::sync::atomic::AtomicU8;
    "16" => u16, core::sync::atomic::AtomicU16;
    "32" => u32, core::sync::atomic::AtomicU32;
    "64" => u64, core::sync::atomic::AtomicU64;
}

type TagAtomic = <LocklessStateTagType as LocklessAtomicWord>::Atomic;
type PriorityAtomic = <FwQueuePriorityType as LocklessAtomicWord>::Atomic;

/// Port of `LocklessSlotState`: the slot lifecycle states, held in the low
/// [`STATE_BITS`] of the state-tag word.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocklessSlotState {
    /// Slot contains no data; available to a producer.
    Free = 0,
    /// Producer has reserved the slot and is filling it.
    Writing = 1,
    /// Slot contains a published message available to a consumer.
    Ready = 2,
    /// Consumer has reserved the slot and is draining it.
    Reading = 3,
}

/// Number of low bits used for the state within the state-tag word
/// (C++ `LocklessSlot::STATE_BITS`).
pub const STATE_BITS: u32 = 2;
/// Mask for the state portion of the state-tag word.
const STATE_MASK: u64 = (1 << STATE_BITS) - 1;
/// Bits available for the ABA epoch tag (C++ `LocklessSlot::TAG_BITS`: 62
/// with a `u64` word, 30 with `u32`).
pub const TAG_BITS: u32 = <LocklessStateTagType as LocklessAtomicWord>::BITS - STATE_BITS;

/// Extract the state portion of a packed state-tag word.
fn state_of(packed: LocklessStateTagType) -> u64 {
    packed.to_u64() & STATE_MASK
}

/// Extract the tag portion of a packed state-tag word.
fn tag_of(packed: LocklessStateTagType) -> u64 {
    packed.to_u64() >> STATE_BITS
}

/// Pack a (state, tag) pair into one word; the tag wraps at `TAG_BITS`.
fn pack_state_tag(state: LocklessSlotState, tag: u64) -> LocklessStateTagType {
    LocklessStateTagType::from_u64_wrapping((tag << STATE_BITS) | (state as u64 & STATE_MASK))
}

/// Alignment markers: a zero-length array of `<Align<N> as SlotAlignment>
/// ::Marker` gives a struct alignment `N` (a `repr(align)` literal cannot
/// come from a configuration constant directly).
#[allow(dead_code)] // type-level only
struct Align<const N: usize>;

/// Maps an [`Align`] to its marker type; implemented for the powers of two
/// from 1 to 4096 only.
trait SlotAlignment {
    /// A zero-sized type with alignment `N`.
    type Marker;
}

macro_rules! slot_alignments {
    ($($n:literal => $marker:ident),* $(,)?) => {$(
        #[allow(dead_code)] // type-level only
        #[repr(align($n))]
        struct $marker;
        impl SlotAlignment for Align<$n> {
            type Marker = $marker;
        }
    )*};
}

slot_alignments! {
    1 => Align1, 2 => Align2, 4 => Align4, 8 => Align8, 16 => Align16,
    32 => Align32, 64 => Align64, 128 => Align128, 256 => Align256,
    512 => Align512, 1024 => Align1024, 2048 => Align2048, 4096 => Align4096,
}

/// The configured slot alignment as a `usize`.
const SLOT_ALIGNMENT: usize = LOCKLESS_QUEUE_SLOT_ALIGNMENT as usize;

/// Port of `LocklessSlot`: per-slot metadata. The payload lives in the
/// queue's separate data region, as in C++.
struct LocklessSlot {
    /// Forces `LOCKLESS_QUEUE_SLOT_ALIGNMENT` (C++ `alignas`).
    _align: [<Align<{ SLOT_ALIGNMENT }> as SlotAlignment>::Marker; 0],
    /// Packed `(tag << STATE_BITS) | state` word (C++ `m_stateTag`).
    state_tag: TagAtomic,
    /// Publication sequence, the FIFO tie-breaker (C++ `m_sequence`).
    sequence: AtomicU32,
    /// Stored message size (C++ `m_size`; see the module docs).
    size: AtomicUsize,
    /// Stored message priority (C++ `m_priority`).
    priority: PriorityAtomic,
}

const _: () = assert!(core::mem::align_of::<LocklessSlot>() >= SLOT_ALIGNMENT);

impl LocklessSlot {
    /// A slot in the FREE state with a zero tag.
    fn new() -> Self {
        Self {
            _align: [],
            state_tag: LocklessStateTagType::new_atomic(pack_state_tag(LocklessSlotState::Free, 0)),
            sequence: AtomicU32::new(0),
            size: AtomicUsize::new(0),
            priority: FwQueuePriorityType::new_atomic(0),
        }
    }
}

/// Bytes per payload word.
const WORD: usize = core::mem::size_of::<usize>();

/// Port of `Os::Generic::LocklessPriorityQueue` (with its
/// `LocklessPriorityQueueHandle`); see the module docs.
pub struct LocklessPriorityQueue {
    /// `depth` slots (C++ `m_slots`).
    slots: Box<[LocklessSlot]>,
    /// `depth * words_per_slot` payload words (C++ `m_data`).
    data: Box<[AtomicUsize]>,
    /// Payload words per slot (`message_size` rounded up to words).
    words_per_slot: usize,
    /// Configured maximum message size (C++ `m_messageSize`).
    message_size: usize,
    /// Publication sequence counter; wraps (compared modularly).
    sequence: AtomicU32,
    /// Claimed-or-queued slot count, for the high-water mark only.
    count: AtomicU32,
    /// Receivable message count (incremented after READY is published,
    /// decremented at the READY->READING claim).
    available: AtomicU32,
    /// Maximum value `count` has held.
    high_mark: AtomicU32,
}

impl LocklessPriorityQueue {
    /// Maximum retry passes through the slot array before a non-blocking
    /// operation gives up (C++ `MAX_RETRY_PASSES`).
    pub const MAX_RETRY_PASSES: FwSizeType = LOCKLESS_QUEUE_MAX_RETRY_PASSES;

    /// Whether a candidate `(priority, sequence)` is preferred over the
    /// current best (C++ `isCandidatePreferred`): highest priority wins; on
    /// a tie, the smaller sequence in the wrap-aware modular ordering (the
    /// unsigned difference has its top bit set).
    pub fn is_candidate_preferred(
        candidate_priority: FwQueuePriorityType,
        candidate_sequence: u32,
        best_priority: FwQueuePriorityType,
        best_sequence: u32,
    ) -> bool {
        if candidate_priority > best_priority {
            true
        } else if candidate_priority == best_priority {
            let difference = candidate_sequence.wrapping_sub(best_sequence);
            (difference & (1u32 << (u32::BITS - 1))) != 0
        } else {
            false
        }
    }

    /// Configured queue depth.
    pub fn depth(&self) -> usize {
        self.slots.len()
    }

    /// Configured maximum message size.
    pub fn message_size(&self) -> usize {
        self.message_size
    }

    /// The payload words of slot `index`.
    fn payload(&self, index: usize) -> &[AtomicUsize] {
        let start = index * self.words_per_slot;
        &self.data[start..start + self.words_per_slot]
    }

    /// Copy `buffer` into slot `index` (the C++ `memcpy` into `m_data`).
    /// Caller owns the slot (WRITING).
    fn write_payload(&self, index: usize, buffer: &[u8]) {
        for (word, chunk) in self.payload(index).iter().zip(buffer.chunks(WORD)) {
            let mut bytes = [0u8; WORD];
            bytes[..chunk.len()].copy_from_slice(chunk);
            word.store(usize::from_ne_bytes(bytes), Ordering::Relaxed);
        }
    }

    /// Copy slot `index` into `destination` (exactly the stored size).
    /// Caller owns the slot (READING).
    fn read_payload(&self, index: usize, destination: &mut [u8]) {
        for (word, chunk) in self.payload(index).iter().zip(destination.chunks_mut(WORD)) {
            let bytes = word.load(Ordering::Relaxed).to_ne_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }
    }

    /// The blocking-path backoff (C++ `Os::Task::delay(TimeInterval(0,
    /// us))`; its status is ignored, as in C++ — in a `no_std` build with no
    /// platform registered it returns at once and the caller re-scans).
    fn backoff(useconds: u32) {
        let _ = Task::delay(TimeInterval::new(0, useconds));
    }

    /// Raise the high-water mark to `next_count` with a CAS loop bounded by
    /// the depth (each failure strictly raises the observed mark, which is
    /// capped at depth).
    fn raise_high_mark(&self, next_count: u32) {
        let mut previous = self.high_mark.load(Ordering::Relaxed);
        let mut mark_pass = 0;
        while mark_pass < self.slots.len() && next_count > previous {
            match self.high_mark.compare_exchange(
                previous,
                next_count,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(actual) => previous = actual,
            }
            mark_pass += 1;
        }
    }

    /// One scan for a FREE slot; on a successful claim, publish `buffer`.
    fn try_send(&self, buffer: &[u8], priority: FwQueuePriorityType) -> bool {
        for (index, slot) in self.slots.iter().enumerate() {
            let packed = LocklessStateTagType::load(&slot.state_tag, Ordering::Acquire);
            if state_of(packed) != LocklessSlotState::Free as u64 {
                continue;
            }
            let desired = pack_state_tag(LocklessSlotState::Writing, tag_of(packed) + 1);
            if LocklessStateTagType::compare_exchange(
                &slot.state_tag,
                packed,
                desired,
                Ordering::AcqRel,
                Ordering::Relaxed,
            )
            .is_err()
            {
                continue;
            }
            self.write_payload(index, buffer);
            slot.size.store(buffer.len(), Ordering::Relaxed);
            FwQueuePriorityType::store(&slot.priority, priority, Ordering::Relaxed);
            slot.sequence.store(
                self.sequence.fetch_add(1, Ordering::Relaxed),
                Ordering::Relaxed,
            );
            // Count before publishing READY: a consumer's decrement (which
            // follows a READY observation) never precedes this increment.
            let next_count = self.count.fetch_add(1, Ordering::AcqRel).wrapping_add(1);
            LocklessStateTagType::store(
                &slot.state_tag,
                pack_state_tag(LocklessSlotState::Ready, tag_of(desired) + 1),
                Ordering::Release,
            );
            // Receivable only after READY is published.
            self.available.fetch_add(1, Ordering::AcqRel);
            // After publication, so the message is never invisible while the
            // producer runs this loop.
            self.raise_high_mark(next_count);
            return true;
        }
        false
    }

    /// One scan-and-claim attempt. `None`: nothing READY was found;
    /// `Some(false)`: the claim CAS lost a race; `Some(true)`: received.
    fn try_receive(
        &self,
        destination: &mut [u8],
        actual_size: &mut FwSizeType,
        priority: &mut FwQueuePriorityType,
    ) -> Option<bool> {
        // (index, priority, sequence, packed word) of the best candidate.
        let mut best: Option<(usize, FwQueuePriorityType, u32, LocklessStateTagType)> = None;
        for (index, slot) in self.slots.iter().enumerate() {
            let packed = LocklessStateTagType::load(&slot.state_tag, Ordering::Acquire);
            if state_of(packed) != LocklessSlotState::Ready as u64 {
                continue;
            }
            let candidate_priority = FwQueuePriorityType::load(&slot.priority, Ordering::Relaxed);
            let candidate_sequence = slot.sequence.load(Ordering::Relaxed);
            // If the word changed, the relaxed reads above may belong to a
            // recycled slot: discard the candidate.
            if LocklessStateTagType::load(&slot.state_tag, Ordering::Acquire) != packed {
                continue;
            }
            let preferred = match best {
                None => true,
                Some((_, best_priority, best_sequence, _)) => Self::is_candidate_preferred(
                    candidate_priority,
                    candidate_sequence,
                    best_priority,
                    best_sequence,
                ),
            };
            if preferred {
                best = Some((index, candidate_priority, candidate_sequence, packed));
            }
        }
        let (index, _, _, packed) = best?;
        let slot = &self.slots[index];
        let desired = pack_state_tag(LocklessSlotState::Reading, tag_of(packed) + 1);
        if LocklessStateTagType::compare_exchange(
            &slot.state_tag,
            packed,
            desired,
            Ordering::AcqRel,
            Ordering::Relaxed,
        )
        .is_err()
        {
            return Some(false);
        }
        // This message can no longer complete another receive.
        self.available.fetch_sub(1, Ordering::AcqRel);
        let stored_size = slot.size.load(Ordering::Relaxed);
        fw_assert!(stored_size <= destination.len());
        self.read_payload(index, &mut destination[..stored_size]);
        *actual_size = stored_size as FwSizeType;
        *priority = FwQueuePriorityType::load(&slot.priority, Ordering::Relaxed);
        // Uncount before FREE: keeps `count` (and the high-water mark) at or
        // below the depth.
        self.count.fetch_sub(1, Ordering::AcqRel);
        LocklessStateTagType::store(
            &slot.state_tag,
            pack_state_tag(LocklessSlotState::Free, tag_of(desired) + 1),
            Ordering::Release,
        );
        Some(true)
    }
}

impl QueueInterface for LocklessPriorityQueue {
    /// Allocate the slot pool and the payload region, once (C++
    /// `create`). Asserts `depth > 0`, `message_size > 0`, no size overflow
    /// and `depth < u32::MAX / 2` (the modular sequence comparison is exact
    /// only while queued equal-priority messages span less than half the
    /// `u32` domain); [`Status::AllocationFailed`] if memory is short.
    fn create(depth: usize, message_size: usize) -> Result<Self, Status> {
        fw_assert!(depth > 0);
        fw_assert!(message_size > 0);
        fw_assert!((depth as u64) < u64::from(u32::MAX / 2));
        let words_per_slot = message_size.div_ceil(WORD);
        let total_words = depth.checked_mul(words_per_slot);
        fw_assert!(total_words.is_some());
        let total_words = total_words.unwrap_or(0);

        let mut slots = Vec::new();
        if slots.try_reserve_exact(depth).is_err() {
            return Err(Status::AllocationFailed);
        }
        slots.extend((0..depth).map(|_| LocklessSlot::new()));
        let mut data = Vec::new();
        if data.try_reserve_exact(total_words).is_err() {
            return Err(Status::AllocationFailed);
        }
        data.extend((0..total_words).map(|_| AtomicUsize::new(0)));

        Ok(Self {
            slots: slots.into_boxed_slice(),
            data: data.into_boxed_slice(),
            words_per_slot,
            message_size,
            sequence: AtomicU32::new(0),
            count: AtomicU32::new(0),
            available: AtomicU32::new(0),
            high_mark: AtomicU32::new(0),
        })
    }

    /// C++ `send`: [`Status::SizeMismatch`] for an oversized message;
    /// non-blocking: [`Status::Full`] after `MAX_RETRY_PASSES` scans find
    /// no claimable slot (ISR-safe); blocking: re-scan after each backoff
    /// until a slot is claimed (not ISR-safe).
    fn send(
        &self,
        buffer: &[u8],
        priority: FwQueuePriorityType,
        block_type: BlockingType,
    ) -> Status {
        if buffer.len() > self.message_size {
            return Status::SizeMismatch;
        }
        let blocking = block_type == BlockingType::Blocking;
        // Bounded for non-blocking, unbounded for blocking (explicit user
        // contract).
        let mut pass: FwSizeType = 0;
        while blocking || pass < Self::MAX_RETRY_PASSES {
            pass = pass.saturating_add(1);
            if self.try_send(buffer, priority) {
                return Status::OpOk;
            }
            if blocking {
                Self::backoff(LOCKLESS_QUEUE_BLOCKING_BACKOFF_US);
            }
        }
        Status::Full
    }

    /// C++ `receive`: the highest-priority READY message, oldest first
    /// among equal priorities. Non-blocking: [`Status::Empty`] after
    /// `MAX_RETRY_PASSES` attempts (ISR-safe); blocking: back off and
    /// re-scan until a message is claimed (not ISR-safe). `destination`
    /// must hold the dequeued message (`fw_assert!`, C++ parity; the
    /// configured message size always suffices).
    fn receive(
        &self,
        destination: &mut [u8],
        block_type: BlockingType,
        actual_size: &mut FwSizeType,
        priority: &mut FwQueuePriorityType,
    ) -> Status {
        let blocking = block_type == BlockingType::Blocking;
        let mut pass: FwSizeType = 0;
        while blocking || pass < Self::MAX_RETRY_PASSES {
            pass = pass.saturating_add(1);
            // Fast path: skip the O(depth) scan while nothing is receivable.
            if self.available.load(Ordering::Acquire) == 0 {
                if blocking {
                    Self::backoff(LOCKLESS_QUEUE_BLOCKING_BACKOFF_US);
                }
                continue;
            }
            match self.try_receive(destination, actual_size, priority) {
                Some(true) => return Status::OpOk,
                None => {
                    if blocking {
                        Self::backoff(LOCKLESS_QUEUE_BLOCKING_BACKOFF_US);
                    }
                }
                // Another consumer claimed the slot: yield and rescan.
                Some(false) => {
                    if blocking {
                        Self::backoff(0);
                    }
                }
            }
        }
        Status::Empty
    }

    /// Number of receivable messages: a message counts from its READY
    /// publication until a consumer claims it.
    fn get_messages_available(&self) -> FwSizeType {
        FwSizeType::from(self.available.load(Ordering::Acquire))
    }

    /// Maximum number of messages queued (claimed or published) at once.
    fn get_message_high_water_mark(&self) -> FwSizeType {
        FwSizeType::from(self.high_mark.load(Ordering::Acquire))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn created(depth: usize, message_size: usize) -> LocklessPriorityQueue {
        match LocklessPriorityQueue::create(depth, message_size) {
            Ok(queue) => queue,
            Err(status) => panic!("create failed: {status:?}"),
        }
    }

    fn receive(
        queue: &LocklessPriorityQueue,
        destination: &mut [u8],
    ) -> (Status, FwSizeType, FwQueuePriorityType) {
        let (mut size, mut priority) = (0, 0);
        let status = queue.receive(
            destination,
            BlockingType::NonBlocking,
            &mut size,
            &mut priority,
        );
        (status, size, priority)
    }

    #[test]
    fn slot_layout_follows_configuration() {
        assert_eq!(
            core::mem::align_of::<LocklessSlot>(),
            LOCKLESS_QUEUE_SLOT_ALIGNMENT as usize
        );
        assert_eq!(
            TAG_BITS,
            8 * core::mem::size_of::<LocklessStateTagType>() as u32 - 2
        );
        let queue = created(3, 4);
        for slot in queue.slots.iter() {
            let address = core::ptr::from_ref(slot) as usize;
            assert_eq!(address % LOCKLESS_QUEUE_SLOT_ALIGNMENT as usize, 0);
        }
    }

    #[test]
    fn state_tag_packing_and_wrap() {
        let packed = pack_state_tag(LocklessSlotState::Ready, 5);
        assert_eq!(state_of(packed), LocklessSlotState::Ready as u64);
        assert_eq!(tag_of(packed), 5);
        // The tag wraps at TAG_BITS without disturbing the state bits.
        let max_tag = (1u64 << TAG_BITS) - 1;
        let packed = pack_state_tag(LocklessSlotState::Writing, max_tag + 1);
        assert_eq!(state_of(packed), LocklessSlotState::Writing as u64);
        assert_eq!(tag_of(packed), 0);
    }

    #[test]
    fn every_transition_bumps_the_tag() {
        let queue = created(1, 4);
        let word = |queue: &LocklessPriorityQueue| {
            LocklessStateTagType::load(&queue.slots[0].state_tag, Ordering::Relaxed)
        };
        assert_eq!(state_of(word(&queue)), LocklessSlotState::Free as u64);
        assert_eq!(tag_of(word(&queue)), 0);
        assert_eq!(queue.send(&[1], 0, BlockingType::NonBlocking), Status::OpOk);
        // FREE -> WRITING -> READY.
        assert_eq!(state_of(word(&queue)), LocklessSlotState::Ready as u64);
        assert_eq!(tag_of(word(&queue)), 2);
        let (status, _, _) = receive(&queue, &mut [0u8; 4]);
        assert_eq!(status, Status::OpOk);
        // READY -> READING -> FREE.
        assert_eq!(state_of(word(&queue)), LocklessSlotState::Free as u64);
        assert_eq!(tag_of(word(&queue)), 4);
    }

    // Upstream TEST(Adversarial, SequenceWrapComparison).
    #[test]
    fn sequence_wrap_comparison() {
        let top_bit = 1u32 << 31;
        let is_older = |candidate: u32, best: u32| {
            LocklessPriorityQueue::is_candidate_preferred(0, candidate, 0, best)
        };
        assert!(is_older(5, 10));
        assert!(!is_older(10, 5));
        assert!(!is_older(5, 5));
        assert!(is_older(u32::MAX - 5, 3));
        assert!(!is_older(3, u32::MAX - 5));
        // Exactly half the domain apart: both directions report "older"
        // (harmless: depth is far below 2^31).
        assert!(is_older(0, top_bit));
        assert!(is_older(top_bit, 0));
        assert!(!is_older(0, top_bit + 1));
        assert!(is_older(top_bit + 1, 0));
        assert!(is_older(0, top_bit - 1));
        assert!(!is_older(top_bit - 1, 0));
        // Priority dominates the sequence.
        assert!(LocklessPriorityQueue::is_candidate_preferred(2, 9, 1, 0));
        assert!(!LocklessPriorityQueue::is_candidate_preferred(1, 0, 2, 9));
    }

    #[test]
    fn fifo_tie_break_survives_sequence_wrap() {
        let queue = created(4, 4);
        queue.sequence.store(u32::MAX - 1, Ordering::Relaxed);
        for value in 0u8..4 {
            assert_eq!(
                queue.send(&[value], 7, BlockingType::NonBlocking),
                Status::OpOk
            );
        }
        for value in 0u8..4 {
            let mut buffer = [0u8; 4];
            let (status, size, priority) = receive(&queue, &mut buffer);
            assert_eq!((status, size, priority), (Status::OpOk, 1, 7));
            assert_eq!(buffer[0], value);
        }
    }

    // Upstream TEST(LocklessLifetime, OversizedSendRejectedAtDelegate).
    #[test]
    fn oversized_send_rejected_at_delegate() {
        let queue = created(4, 4);
        assert_eq!(
            queue.send(&[0u8; 5], 0, BlockingType::NonBlocking),
            Status::SizeMismatch
        );
        assert_eq!(queue.get_messages_available(), 0);
    }

    // Upstream TEST(LocklessLifetime, ZeroSizeMessage).
    #[test]
    fn zero_size_message() {
        let queue = created(4, 4);
        assert_eq!(queue.send(&[], 1, BlockingType::NonBlocking), Status::OpOk);
        let (mut size, mut priority) = (99, 0);
        assert_eq!(
            queue.receive(
                &mut [0u8; 4],
                BlockingType::NonBlocking,
                &mut size,
                &mut priority
            ),
            Status::OpOk
        );
        assert_eq!((size, priority), (0, 1));
    }

    // Upstream TEST(LocklessLifetime, MessagesAvailableTracksReceivableCount).
    #[test]
    fn messages_available_tracks_receivable_count() {
        let queue = created(4, 4);
        assert_eq!(queue.get_messages_available(), 0);
        for i in 0..3 {
            assert_eq!(
                queue.send(&[0u8; 4], 0, BlockingType::NonBlocking),
                Status::OpOk
            );
            assert_eq!(queue.get_messages_available(), i + 1);
        }
        assert_eq!(queue.get_message_high_water_mark(), 3);
        for i in 0..3 {
            assert_eq!(receive(&queue, &mut [0u8; 4]).0, Status::OpOk);
            assert_eq!(queue.get_messages_available(), 2 - i);
        }
        assert_eq!(queue.get_message_high_water_mark(), 3);
    }

    #[test]
    fn payload_round_trips_across_word_boundaries() {
        // Message sizes around the word size exercise the partial last word.
        for message_size in [1, WORD - 1, WORD, WORD + 1, 3 * WORD + 2] {
            let queue = created(3, message_size);
            for length in [0, 1, message_size / 2, message_size] {
                let message: Vec<u8> = (0..length).map(|i| (i as u8) ^ 0xA5).collect();
                assert_eq!(
                    queue.send(&message, 0, BlockingType::NonBlocking),
                    Status::OpOk
                );
                let mut buffer = alloc::vec![0xEEu8; message_size];
                let (status, size, _) = receive(&queue, &mut buffer);
                assert_eq!((status, size), (Status::OpOk, length as FwSizeType));
                assert_eq!(&buffer[..length], &message[..]);
                // Bytes past the message are untouched.
                assert!(buffer[length..].iter().all(|&byte| byte == 0xEE));
            }
        }
    }

    #[test]
    fn nonblocking_full_and_empty_after_bounded_passes() {
        let queue = created(2, 4);
        assert_eq!(receive(&queue, &mut [0u8; 4]).0, Status::Empty);
        assert_eq!(queue.send(&[1], 0, BlockingType::NonBlocking), Status::OpOk);
        assert_eq!(queue.send(&[2], 0, BlockingType::NonBlocking), Status::OpOk);
        assert_eq!(queue.send(&[3], 0, BlockingType::NonBlocking), Status::Full);
        assert_eq!(queue.get_message_high_water_mark(), 2);
    }

    #[test]
    #[should_panic]
    fn receive_asserts_destination_holds_message() {
        let queue = created(2, 8);
        assert_eq!(
            queue.send(&[0u8; 8], 0, BlockingType::NonBlocking),
            Status::OpOk
        );
        let _ = receive(&queue, &mut [0u8; 4]);
    }

    #[test]
    #[should_panic]
    fn create_asserts_nonzero_depth() {
        let _ = LocklessPriorityQueue::create(0, 4);
    }

    #[test]
    #[should_panic]
    fn create_asserts_nonzero_message_size() {
        let _ = LocklessPriorityQueue::create(4, 0);
    }

    #[test]
    #[should_panic]
    fn create_asserts_depth_below_half_sequence_domain() {
        let _ = LocklessPriorityQueue::create(u32::MAX as usize / 2, 1);
    }

    #[test]
    fn create_reports_allocation_failure() {
        // A payload region no allocator can satisfy.
        let message_size = isize::MAX as usize / 2;
        assert!(matches!(
            LocklessPriorityQueue::create(4, message_size),
            Err(Status::AllocationFailed)
        ));
    }
}
