//! The lock-based priority queue delegate.
//!
//! Port of `Os::Generic::PriorityQueue` (Os/Generic/PriorityQueue.{hpp,cpp})
//! with its `Types::MaxHeap` stable max-heap — the default `Os::Queue`
//! delegate (upstream `Os/Generic/DefaultPriorityQueue.cpp`); see
//! `docs/cpp-analysis/os.md`.
//!
//! Semantics preserved exactly:
//! - fixed slab storage (`depth * message_size` bytes allocated at
//!   [`PriorityQueue::create`](QueueInterface::create)), a circular
//!   free-index ring, and per-slot sizes;
//! - a stable max-heap: highest priority first, FIFO within equal priority
//!   (the C++ MaxHeap tie-breaks equal priorities by insertion order);
//! - one mutex plus two condition variables — senders wait while full,
//!   receivers wait while empty, and each side notifies one waiter AFTER
//!   releasing the lock.
//!
//! `no_std` (ROADMAP item 13): the data mutex is the `fprime_fw::sync` spin
//! lock and there is no condition variable — a blocking send/receive
//! releases the lock, takes one `platform::Platform::queue_wait` backoff
//! step and re-checks (the polling shape of
//! [`LocklessPriorityQueue`](super::lockless_priority_queue::LocklessPriorityQueue)).
//! Everything else, including the status flow and FIFO-within-priority
//! order, is identical. Because the state is behind a spin lock, this
//! delegate must not be sent to from an interrupt that can preempt a holder
//! of the same queue's lock; select the lockless delegate for that.

use crate::queue::{BlockingType, QueueInterface, Status};
use alloc::boxed::Box;
use alloc::collections::BinaryHeap;
use alloc::vec;
use alloc::vec::Vec;
use core::cmp::Reverse;
use fprime_config::{FwQueuePriorityType, FwSizeType};
use fprime_fw::fw_assert;
use fprime_fw::sync::{Mutex, MutexGuard};

/// One stable max-heap entry (C++ `Types::MaxHeap::Node`). Derived ordering
/// is lexicographic: priority DESC first, then `Reverse(order)` — the oldest
/// entry wins among equal priorities, reproducing the MaxHeap age tie-break
/// (FIFO within one priority).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct HeapNode {
    priority: FwQueuePriorityType,
    order: Reverse<u64>,
    index: usize,
}

/// All queue state guarded by the single data mutex (C++
/// `PriorityQueueHandle`).
struct QueueState {
    /// Unordered per-slot slabs: `depth * message_size` bytes; message `i`
    /// lives at `i * message_size`.
    data: Box<[u8]>,
    /// Per-slot stored message size.
    sizes: Box<[usize]>,
    /// Circular free-index list.
    indices: Box<[usize]>,
    start_index: usize,
    stop_index: usize,
    heap: BinaryHeap<HeapNode>,
    /// Monotonic insertion counter (C++ `MaxHeap::m_order`).
    order: u64,
    /// Maximum simultaneous messages seen (C++ `highMark`).
    high_mark: usize,
}

/// Port of `Os::Generic::PriorityQueue` (see module docs).
pub struct PriorityQueue {
    depth: usize,
    message_size: usize,
    state: Mutex<QueueState>,
    /// Senders wait here while the queue is full (C++ `m_full`).
    not_full: Signal,
    /// Receivers wait here while the queue is empty (C++ `m_empty`).
    not_empty: Signal,
}

fn lock_state(mutex: &Mutex<QueueState>) -> MutexGuard<'_, QueueState> {
    // A poisoned lock means another thread panicked mid-operation; the C++
    // equivalent is a crashed FW_ASSERT. The `fprime_fw::sync` mutex
    // recovers the guard so remaining threads see consistent-enough state
    // to shut down.
    mutex.lock()
}

/// One wait condition of the queue: a `std::sync::Condvar` in the `std`
/// build.
#[cfg(feature = "std")]
struct Signal(std::sync::Condvar);

#[cfg(feature = "std")]
impl Signal {
    fn new() -> Self {
        Self(std::sync::Condvar::new())
    }

    /// Atomically release `guard` and wait for a notification, then
    /// re-acquire the lock (spurious wakeups allowed; callers loop).
    fn wait<'a>(
        &self,
        _mutex: &'a Mutex<QueueState>,
        guard: MutexGuard<'a, QueueState>,
    ) -> MutexGuard<'a, QueueState> {
        self.0
            .wait(guard)
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Wake one waiter.
    fn notify_one(&self) {
        self.0.notify_one();
    }
}

/// One wait condition of the queue: polling over the platform seam in the
/// `no_std` build (see the module docs).
#[cfg(not(feature = "std"))]
struct Signal;

#[cfg(not(feature = "std"))]
impl Signal {
    fn new() -> Self {
        Self
    }

    /// Release the lock, take one platform backoff step, re-acquire. The
    /// caller re-checks its condition, exactly as after a condvar wakeup.
    fn wait<'a>(
        &self,
        mutex: &'a Mutex<QueueState>,
        guard: MutexGuard<'a, QueueState>,
    ) -> MutexGuard<'a, QueueState> {
        drop(guard);
        crate::platform::queue_wait();
        lock_state(mutex)
    }

    /// Nothing to wake: waiters poll.
    fn notify_one(&self) {}
}

impl QueueInterface for PriorityQueue {
    /// Allocate the message slab, free-index ring and heap up front
    /// (C++ `PriorityQueue::create`); steady state never allocates.
    fn create(depth: usize, message_size: usize) -> Result<Self, Status> {
        fw_assert!(depth > 0);
        fw_assert!(message_size > 0);
        let slab_size = depth.checked_mul(message_size);
        fw_assert!(slab_size.is_some());
        Ok(Self {
            depth,
            message_size,
            state: Mutex::new(QueueState {
                data: vec![0u8; slab_size.unwrap_or(0)].into_boxed_slice(),
                sizes: vec![0usize; depth].into_boxed_slice(),
                indices: (0..depth).collect::<Vec<usize>>().into_boxed_slice(),
                start_index: 0,
                stop_index: 0,
                heap: BinaryHeap::with_capacity(depth),
                order: 0,
                high_mark: 0,
            }),
            not_full: Signal::new(),
            not_empty: Signal::new(),
        })
    }

    /// C++ `PriorityQueue::send`: [`Status::Full`] for a non-blocking send
    /// on a full queue; a blocking send waits on the not-full condition.
    /// The receiver notification is issued after the lock is released (C++
    /// parity).
    fn send(
        &self,
        buffer: &[u8],
        priority: FwQueuePriorityType,
        block_type: BlockingType,
    ) -> Status {
        // The `Os::Queue` facade has already rejected oversized messages;
        // the delegate keeps the check so it is safe to use directly.
        if buffer.len() > self.message_size {
            return Status::SizeMismatch;
        }
        let mut state = lock_state(&self.state);
        // Wait for space (senders wait on m_full in C++).
        while state.heap.len() >= self.depth {
            if block_type == BlockingType::NonBlocking {
                return Status::Full;
            }
            state = self.not_full.wait(&self.state, state);
        }
        // find_index(): pop the free-index ring at start_index.
        let index = state.indices[state.start_index];
        state.start_index = (state.start_index + 1) % self.depth;
        // Heap push cannot fail: heap size < depth was just checked under
        // the same lock (C++ asserts the push succeeded).
        let node = HeapNode {
            priority,
            order: Reverse(state.order),
            index,
        };
        state.order = state.order.wrapping_add(1);
        state.heap.push(node);
        // store_data(): copy into the slot slab.
        let offset = index * self.message_size;
        state.data[offset..offset + buffer.len()].copy_from_slice(buffer);
        state.sizes[index] = buffer.len();
        state.high_mark = state.high_mark.max(state.heap.len());
        drop(state);
        // C++ parity: notify one receiver AFTER unlocking.
        self.not_empty.notify_one();
        Status::OpOk
    }

    /// C++ `PriorityQueue::receive`: [`Status::Empty`] for a non-blocking
    /// receive on an empty queue; a blocking receive waits on the not-empty
    /// condition. `destination` must hold the dequeued message (an
    /// `fw_assert!`, C++ parity; the facade guarantees it).
    fn receive(
        &self,
        destination: &mut [u8],
        block_type: BlockingType,
        actual_size: &mut FwSizeType,
        priority: &mut FwQueuePriorityType,
    ) -> Status {
        let mut state = lock_state(&self.state);
        while state.heap.is_empty() {
            if block_type == BlockingType::NonBlocking {
                return Status::Empty;
            }
            state = self.not_empty.wait(&self.state, state);
        }
        // Pop cannot fail: non-empty was just checked under the same lock
        // (C++ asserts the pop succeeded).
        let node = match state.heap.pop() {
            Some(node) => node,
            None => return Status::UnknownError,
        };
        let size = state.sizes[node.index];
        // C++ parity: actualSize <= capacity is an FW_ASSERT inside the
        // delegate, NOT a status return (the facade's configured-size check
        // already guarantees it).
        fw_assert!(size <= destination.len());
        let offset = node.index * self.message_size;
        destination[..size].copy_from_slice(&state.data[offset..offset + size]);
        // return_index(): push the freed slot at stop_index.
        let stop = state.stop_index;
        state.indices[stop] = node.index;
        state.stop_index = (state.stop_index + 1) % self.depth;
        drop(state);
        // C++ parity: notify one sender AFTER unlocking.
        self.not_full.notify_one();
        *actual_size = size as FwSizeType;
        *priority = node.priority;
        Status::OpOk
    }

    /// Number of messages currently queued. C++ parity note: the C++
    /// implementation reads the heap size WITHOUT the lock (racy by design);
    /// this port takes the lock — the value is still stale the moment it is
    /// returned, so the semantics are equivalent.
    fn get_messages_available(&self) -> FwSizeType {
        lock_state(&self.state).heap.len() as FwSizeType
    }

    /// Maximum number of messages ever simultaneously queued (read under
    /// the lock).
    fn get_message_high_water_mark(&self) -> FwSizeType {
        lock_state(&self.state).high_mark as FwSizeType
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // The behavioral suite runs through the `Os::Queue` facade in
    // `crate::queue` (against whichever delegate is selected); these cover
    // the delegate used directly.
    #[test]
    fn delegate_rejects_oversized_send() {
        let queue = PriorityQueue::create(2, 4).unwrap();
        assert_eq!(
            queue.send(&[0u8; 5], 0, BlockingType::NonBlocking),
            Status::SizeMismatch
        );
        assert_eq!(queue.get_messages_available(), 0);
    }

    #[test]
    fn delegate_round_trip_with_short_destination() {
        // Directly, the delegate only needs room for the stored message.
        let queue = PriorityQueue::create(2, 8).unwrap();
        assert_eq!(
            queue.send(&[7, 8], 3, BlockingType::NonBlocking),
            Status::OpOk
        );
        let mut buffer = [0u8; 2];
        let (mut size, mut priority) = (0, 0);
        assert_eq!(
            queue.receive(
                &mut buffer,
                BlockingType::NonBlocking,
                &mut size,
                &mut priority
            ),
            Status::OpOk
        );
        assert_eq!((buffer, size, priority), ([7, 8], 2, 3));
    }
}
