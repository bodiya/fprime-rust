//! The F Prime priority message queue.
//!
//! Port of `Os::Queue` (Os/Queue.{hpp,cpp}) over the default delegate
//! `Os::Generic::PriorityQueue` (Os/Generic/PriorityQueue.{hpp,cpp}) with its
//! `Types::MaxHeap` stable max-heap — see `docs/cpp-analysis/os.md`.
//!
//! Semantics preserved exactly:
//! - fixed slab storage (`depth * message_size` bytes allocated at
//!   [`Queue::create`]), a circular free-index ring, and per-slot sizes;
//! - a stable max-heap: highest priority first, FIFO within equal priority
//!   (the C++ MaxHeap tie-breaks equal priorities by insertion order);
//! - one mutex plus two condition variables — senders wait while full,
//!   receivers wait while empty, and each side notifies one waiter AFTER
//!   releasing the lock;
//! - the `Os::Queue` facade checks: `create` asserts depth/size > 0 and
//!   returns [`Status::AlreadyCreated`] on a second create; `send` with a
//!   message larger than the configured size returns
//!   [`Status::SizeMismatch`]; `receive` with a destination smaller than the
//!   CONFIGURED message size (not the actual message size) returns
//!   [`Status::SizeMismatch`]; non-blocking send on full returns
//!   [`Status::Full`], non-blocking receive on empty returns
//!   [`Status::Empty`].
//!
//! Concurrency model: unlike the C++ front class, [`Queue`] uses interior
//! mutability throughout — every method (including `create`) takes `&self`,
//! so a `Queue` is shared as `Arc<Queue>` across sender and receiver threads
//! with no external locking.

use fprime_config::{FW_QUEUE_NAME_BUFFER_SIZE, FwQueuePriorityType, FwSizeType};
use fprime_fw::fw_assert;
use fprime_fw::{FwString, SerBuf, SerBufAny};
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Condvar, Mutex, MutexGuard, PoisonError};

/// Queue name string (C++ `Os::QueueString`, capacity
/// `FW_QUEUE_NAME_BUFFER_SIZE`).
pub type QueueString = FwString<FW_QUEUE_NAME_BUFFER_SIZE>;

/// Port of `Os::QueueInterface::Status` (Os/Queue.hpp) — exact C++
/// discriminants.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Message sent/received okay.
    OpOk = 0,
    /// Creating an already created queue.
    AlreadyCreated = 1,
    /// If non-blocking, all the messages have been drained.
    Empty = 2,
    /// Queue wasn't initialized successfully.
    Uninitialized = 3,
    /// Attempted to send or receive with buffer too large / too small.
    SizeMismatch = 4,
    /// Message send error.
    SendError = 5,
    /// Message receive error.
    ReceiveError = 6,
    /// Invalid priority requested.
    InvalidPriority = 7,
    /// Queue was full when attempting to send a message.
    Full = 8,
    /// Queue feature is not supported.
    NotSupported = 9,
    /// Required memory could not be allocated.
    AllocationFailed = 10,
    /// Unexpected error; can't match with returns.
    UnknownError = 11,
}

/// Port of `Os::QueueInterface::BlockingType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockingType {
    /// Send blocks until space is available; receive blocks until a message
    /// arrives.
    Blocking = 0,
    /// Send returns [`Status::Full`] / receive returns [`Status::Empty`]
    /// instead of blocking.
    NonBlocking = 1,
}

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

/// All queue state guarded by the single data mutex (C++ `PriorityQueueHandle`
/// plus the `Os::Queue` wrapper fields).
#[derive(Default)]
struct QueueState {
    created: bool,
    name: QueueString,
    depth: usize,
    message_size: usize,
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

/// Global created-queue count (C++ `Os::Queue::s_queueCount`, guarded by a
/// static mutex there; a relaxed atomic here — same observable values).
static QUEUE_COUNT: AtomicU64 = AtomicU64::new(0);

/// The F Prime priority message queue (see module docs).
#[derive(Default)]
pub struct Queue {
    state: Mutex<QueueState>,
    /// Senders wait here while the queue is full (C++ `m_full`).
    not_full: Condvar,
    /// Receivers wait here while the queue is empty (C++ `m_empty`).
    not_empty: Condvar,
}

fn lock_state<'a>(mutex: &'a Mutex<QueueState>) -> MutexGuard<'a, QueueState> {
    // A poisoned lock means another thread panicked mid-operation; the C++
    // equivalent is a crashed FW_ASSERT. Recover the guard so remaining
    // threads see consistent-enough state to shut down.
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

impl Queue {
    /// Construct an uncreated queue. All operations except [`Queue::create`]
    /// return [`Status::Uninitialized`] until `create` succeeds.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create the queue storage (port of `Os::Queue::create` +
    /// `PriorityQueue::create`). Allocates the message slab, free-index ring,
    /// and heap up front; steady state never allocates.
    ///
    /// C++ parity: FW_ASSERTs `depth > 0 && message_size > 0` (crash, not
    /// status); a second create returns [`Status::AlreadyCreated`]. The C++
    /// `id` parameter (used only for the optional QueueRegistry) is not
    /// ported.
    pub fn create(&self, name: &str, depth: FwSizeType, message_size: FwSizeType) -> Status {
        fw_assert!(depth > 0);
        fw_assert!(message_size > 0);
        let depth = depth as usize;
        let message_size = message_size as usize;
        let mut state = lock_state(&self.state);
        if state.created {
            return Status::AlreadyCreated;
        }
        state.name.set(name);
        state.depth = depth;
        state.message_size = message_size;
        state.data = vec![0u8; depth * message_size].into_boxed_slice();
        state.sizes = vec![0usize; depth].into_boxed_slice();
        state.indices = (0..depth).collect::<Vec<usize>>().into_boxed_slice();
        state.start_index = 0;
        state.stop_index = 0;
        state.heap = BinaryHeap::with_capacity(depth);
        state.order = 0;
        state.high_mark = 0;
        state.created = true;
        drop(state);
        QUEUE_COUNT.fetch_add(1, Ordering::Relaxed);
        Status::OpOk
    }

    /// Send a message (port of `Os::Queue::send` +
    /// `PriorityQueue::send`).
    ///
    /// Status flow: [`Status::Uninitialized`] before create;
    /// [`Status::SizeMismatch`] when `buffer.len()` exceeds the configured
    /// message size; [`Status::Full`] for a non-blocking send on a full
    /// queue. A blocking send waits on the not-full condition. The receiver
    /// notification is issued after the lock is released (C++ parity).
    pub fn send(
        &self,
        buffer: &[u8],
        priority: FwQueuePriorityType,
        block_type: BlockingType,
    ) -> Status {
        let mut state = lock_state(&self.state);
        if !state.created {
            return Status::Uninitialized;
        }
        if buffer.len() > state.message_size {
            return Status::SizeMismatch;
        }
        // Wait for space (senders wait on m_full in C++).
        while state.heap.len() >= state.depth {
            if block_type == BlockingType::NonBlocking {
                return Status::Full;
            }
            state = self
                .not_full
                .wait(state)
                .unwrap_or_else(PoisonError::into_inner);
        }
        // find_index(): pop the free-index ring at start_index.
        let index = state.indices[state.start_index];
        state.start_index = (state.start_index + 1) % state.depth;
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
        let offset = index * state.message_size;
        state.data[offset..offset + buffer.len()].copy_from_slice(buffer);
        state.sizes[index] = buffer.len();
        state.high_mark = state.high_mark.max(state.heap.len());
        drop(state);
        // C++ parity: notify one receiver AFTER unlocking.
        self.not_empty.notify_one();
        Status::OpOk
    }

    /// Receive the highest-priority message (port of `Os::Queue::receive` +
    /// `PriorityQueue::receive`). `actual_size` and `priority` are outputs.
    ///
    /// Status flow: [`Status::Uninitialized`] before create;
    /// [`Status::SizeMismatch`] when `destination.len()` is smaller than the
    /// queue's CONFIGURED message size (C++ checks the configured size, not
    /// the pending message's size); [`Status::Empty`] for a non-blocking
    /// receive on an empty queue. A blocking receive waits on the not-empty
    /// condition.
    pub fn receive(
        &self,
        destination: &mut [u8],
        block_type: BlockingType,
        actual_size: &mut FwSizeType,
        priority: &mut FwQueuePriorityType,
    ) -> Status {
        let mut state = lock_state(&self.state);
        if !state.created {
            return Status::Uninitialized;
        }
        if destination.len() < state.message_size {
            return Status::SizeMismatch;
        }
        while state.heap.is_empty() {
            if block_type == BlockingType::NonBlocking {
                return Status::Empty;
            }
            state = self
                .not_empty
                .wait(state)
                .unwrap_or_else(PoisonError::into_inner);
        }
        // Pop cannot fail: non-empty was just checked under the same lock
        // (C++ asserts the pop succeeded).
        let node = match state.heap.pop() {
            Some(node) => node,
            None => return Status::UnknownError,
        };
        let size = state.sizes[node.index];
        // C++ parity: actualSize <= capacity is an FW_ASSERT inside the
        // delegate, NOT a status return (the wrapper's configured-size check
        // above already guarantees it).
        fw_assert!(size <= destination.len());
        let offset = node.index * state.message_size;
        destination[..size].copy_from_slice(&state.data[offset..offset + size]);
        // return_index(): push the freed slot at stop_index.
        let stop = state.stop_index;
        state.indices[stop] = node.index;
        state.stop_index = (state.stop_index + 1) % state.depth;
        drop(state);
        // C++ parity: notify one sender AFTER unlocking.
        self.not_full.notify_one();
        *actual_size = size as FwSizeType;
        *priority = node.priority;
        Status::OpOk
    }

    /// Send the valid region (`[0..ser_loc]`) of a serialization buffer
    /// (port of the C++ `Os::Queue::send(Fw::LinearBufferBase&, ...)`
    /// overload).
    pub fn send_serial(
        &self,
        message: &dyn SerBufAny,
        priority: FwQueuePriorityType,
        block_type: BlockingType,
    ) -> Status {
        self.send(&message.bytes()[..message.ser_loc()], priority, block_type)
    }

    /// Receive into a serialization buffer (port of the C++
    /// `Os::Queue::receive(Fw::LinearBufferBase&, ...)` overload): the buffer
    /// is reset, filled to its full capacity window, and its length set to
    /// the received size. A `set_buff_len` failure maps to
    /// [`Status::SizeMismatch`] (C++ parity).
    pub fn receive_serial(
        &self,
        destination: &mut dyn SerBufAny,
        block_type: BlockingType,
        priority: &mut FwQueuePriorityType,
    ) -> Status {
        let mut actual_size: FwSizeType = 0;
        destination.reset_ser();
        let capacity = destination.capacity();
        let status = self.receive(
            &mut destination.bytes_mut()[..capacity],
            block_type,
            &mut actual_size,
            priority,
        );
        if status == Status::OpOk {
            let ser_status = destination.set_buff_len(actual_size as usize);
            if !ser_status.is_ok() {
                return Status::SizeMismatch;
            }
        }
        status
    }

    /// Number of messages currently queued. C++ parity note: the C++
    /// implementation reads the heap size WITHOUT the lock (racy by design);
    /// this port takes the lock — the value is still stale the moment it is
    /// returned, so the semantics are equivalent.
    pub fn get_messages_available(&self) -> FwSizeType {
        lock_state(&self.state).heap.len() as FwSizeType
    }

    /// Maximum number of messages ever simultaneously queued (C++
    /// `getMessageHighWaterMark`, read under the lock).
    pub fn get_message_high_water_mark(&self) -> FwSizeType {
        lock_state(&self.state).high_mark as FwSizeType
    }

    /// Configured queue depth (0 before create).
    pub fn get_depth(&self) -> FwSizeType {
        lock_state(&self.state).depth as FwSizeType
    }

    /// Configured maximum message size (0 before create).
    pub fn get_message_size(&self) -> FwSizeType {
        lock_state(&self.state).message_size as FwSizeType
    }

    /// Queue name as set at create.
    pub fn get_name(&self) -> QueueString {
        lock_state(&self.state).name.clone()
    }

    /// Number of queues created process-wide (C++ `Os::Queue::getNumQueues`).
    pub fn get_num_queues() -> FwSizeType {
        QUEUE_COUNT.load(Ordering::Relaxed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::sync::mpsc;
    use std::time::Duration;

    fn created_queue(depth: FwSizeType, size: FwSizeType) -> Queue {
        let queue = Queue::new();
        assert_eq!(queue.create("test", depth, size), Status::OpOk);
        queue
    }

    #[test]
    fn statuses_before_create_are_uninitialized() {
        let queue = Queue::new();
        assert_eq!(
            queue.send(&[1, 2], 0, BlockingType::NonBlocking),
            Status::Uninitialized
        );
        let mut size = 0;
        let mut priority = 0;
        assert_eq!(
            queue.receive(
                &mut [0u8; 8],
                BlockingType::NonBlocking,
                &mut size,
                &mut priority
            ),
            Status::Uninitialized
        );
        assert_eq!(queue.get_depth(), 0);
        assert_eq!(queue.get_message_size(), 0);
    }

    #[test]
    fn double_create_returns_already_created() {
        let queue = created_queue(2, 8);
        assert_eq!(queue.create("again", 2, 8), Status::AlreadyCreated);
        // Original configuration untouched.
        assert_eq!(queue.get_depth(), 2);
        assert_eq!(queue.get_name(), "test");
    }

    #[test]
    #[should_panic]
    fn create_asserts_nonzero_depth() {
        let queue = Queue::new();
        let _ = queue.create("bad", 0, 8);
    }

    #[test]
    #[should_panic]
    fn create_asserts_nonzero_message_size() {
        let queue = Queue::new();
        let _ = queue.create("bad", 8, 0);
    }

    #[test]
    fn send_oversized_message_is_size_mismatch() {
        let queue = created_queue(2, 4);
        assert_eq!(
            queue.send(&[0u8; 5], 0, BlockingType::NonBlocking),
            Status::SizeMismatch
        );
        // Exactly message_size is fine.
        assert_eq!(
            queue.send(&[0u8; 4], 0, BlockingType::NonBlocking),
            Status::OpOk
        );
    }

    // Gotcha: receive requires capacity >= the CONFIGURED message size, not
    // the actual pending message's size.
    #[test]
    fn receive_undersized_destination_is_size_mismatch() {
        let queue = created_queue(2, 8);
        assert_eq!(
            queue.send(&[1u8; 2], 0, BlockingType::NonBlocking),
            Status::OpOk
        );
        let mut size = 0;
        let mut priority = 0;
        // 4 >= actual message (2) but < configured size (8): SizeMismatch.
        assert_eq!(
            queue.receive(
                &mut [0u8; 4],
                BlockingType::NonBlocking,
                &mut size,
                &mut priority
            ),
            Status::SizeMismatch
        );
        // Full configured size works and reports the actual stored size.
        assert_eq!(
            queue.receive(
                &mut [0u8; 8],
                BlockingType::NonBlocking,
                &mut size,
                &mut priority
            ),
            Status::OpOk
        );
        assert_eq!(size, 2);
    }

    #[test]
    fn nonblocking_full_and_empty() {
        let queue = created_queue(2, 4);
        let mut size = 0;
        let mut priority = 0;
        assert_eq!(
            queue.receive(
                &mut [0u8; 4],
                BlockingType::NonBlocking,
                &mut size,
                &mut priority
            ),
            Status::Empty
        );
        assert_eq!(queue.send(&[1], 0, BlockingType::NonBlocking), Status::OpOk);
        assert_eq!(queue.send(&[2], 0, BlockingType::NonBlocking), Status::OpOk);
        assert_eq!(queue.send(&[3], 0, BlockingType::NonBlocking), Status::Full);
    }

    #[test]
    fn priority_ordering_with_fifo_within_priority() {
        let queue = created_queue(8, 4);
        // (payload, priority) in send order.
        let sends: [(&[u8], FwQueuePriorityType); 6] = [
            (&[10], 1),
            (&[20], 3),
            (&[21], 3),
            (&[30], 2),
            (&[22], 3),
            (&[11], 1),
        ];
        for (payload, priority) in sends {
            assert_eq!(
                queue.send(payload, priority, BlockingType::NonBlocking),
                Status::OpOk
            );
        }
        // Highest priority first; FIFO within equal priority.
        let expected: [(u8, FwQueuePriorityType); 6] =
            [(20, 3), (21, 3), (22, 3), (30, 2), (10, 1), (11, 1)];
        for (value, expected_priority) in expected {
            let mut buffer = [0u8; 4];
            let mut size = 0;
            let mut priority = 0;
            assert_eq!(
                queue.receive(
                    &mut buffer,
                    BlockingType::NonBlocking,
                    &mut size,
                    &mut priority
                ),
                Status::OpOk
            );
            assert_eq!(size, 1);
            assert_eq!(buffer[0], value);
            assert_eq!(priority, expected_priority);
        }
    }

    #[test]
    fn fifo_within_single_priority_over_many_messages() {
        let queue = created_queue(16, 4);
        for i in 0u8..16 {
            assert_eq!(queue.send(&[i], 7, BlockingType::NonBlocking), Status::OpOk);
        }
        for i in 0u8..16 {
            let mut buffer = [0u8; 4];
            let mut size = 0;
            let mut priority = 0;
            assert_eq!(
                queue.receive(
                    &mut buffer,
                    BlockingType::NonBlocking,
                    &mut size,
                    &mut priority
                ),
                Status::OpOk
            );
            assert_eq!(buffer[0], i);
        }
    }

    // Free-index ring reuse: interleave sends/receives beyond depth to force
    // slot recycling and confirm payload integrity.
    #[test]
    fn slot_recycling_preserves_payloads() {
        let queue = created_queue(3, 8);
        for round in 0u8..10 {
            for lane in 0u8..3 {
                let value = round.wrapping_mul(3).wrapping_add(lane);
                assert_eq!(
                    queue.send(&[value; 5], lane, BlockingType::NonBlocking),
                    Status::OpOk
                );
            }
            // Highest lane priority (2) first.
            for lane in (0u8..3).rev() {
                let value = round.wrapping_mul(3).wrapping_add(lane);
                let mut buffer = [0u8; 8];
                let mut size = 0;
                let mut priority = 0;
                assert_eq!(
                    queue.receive(
                        &mut buffer,
                        BlockingType::NonBlocking,
                        &mut size,
                        &mut priority
                    ),
                    Status::OpOk
                );
                assert_eq!(size, 5);
                assert_eq!(priority, lane);
                assert_eq!(&buffer[..5], &[value; 5]);
            }
        }
    }

    #[test]
    fn blocking_send_unblocks_when_receiver_drains() {
        let queue = Arc::new(created_queue(2, 4));
        assert_eq!(queue.send(&[1], 0, BlockingType::Blocking), Status::OpOk);
        assert_eq!(queue.send(&[2], 0, BlockingType::Blocking), Status::OpOk);
        let sender_queue = Arc::clone(&queue);
        let (tx, rx) = mpsc::channel();
        let sender = std::thread::spawn(move || {
            // Blocks: queue is full.
            let status = sender_queue.send(&[3], 0, BlockingType::Blocking);
            tx.send(()).unwrap();
            status
        });
        // The sender must still be blocked.
        assert!(rx.recv_timeout(Duration::from_millis(100)).is_err());
        let mut buffer = [0u8; 4];
        let mut size = 0;
        let mut priority = 0;
        assert_eq!(
            queue.receive(
                &mut buffer,
                BlockingType::NonBlocking,
                &mut size,
                &mut priority
            ),
            Status::OpOk
        );
        // Now the blocked send completes.
        assert!(rx.recv_timeout(Duration::from_secs(5)).is_ok());
        assert_eq!(sender.join().unwrap(), Status::OpOk);
        assert_eq!(queue.get_messages_available(), 2);
    }

    #[test]
    fn blocking_receive_unblocks_on_send() {
        let queue = Arc::new(created_queue(2, 4));
        let receiver_queue = Arc::clone(&queue);
        let receiver = std::thread::spawn(move || {
            let mut buffer = [0u8; 4];
            let mut size = 0;
            let mut priority = 0;
            let status = receiver_queue.receive(
                &mut buffer,
                BlockingType::Blocking,
                &mut size,
                &mut priority,
            );
            (status, buffer[0], size, priority)
        });
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(
            queue.send(&[42], 9, BlockingType::NonBlocking),
            Status::OpOk
        );
        let (status, value, size, priority) = receiver.join().unwrap();
        assert_eq!(status, Status::OpOk);
        assert_eq!(value, 42);
        assert_eq!(size, 1);
        assert_eq!(priority, 9);
    }

    #[test]
    fn high_water_mark_and_messages_available() {
        let queue = created_queue(4, 4);
        assert_eq!(queue.get_message_high_water_mark(), 0);
        for i in 0..3u8 {
            assert_eq!(queue.send(&[i], 0, BlockingType::NonBlocking), Status::OpOk);
        }
        assert_eq!(queue.get_messages_available(), 3);
        assert_eq!(queue.get_message_high_water_mark(), 3);
        let mut buffer = [0u8; 4];
        let mut size = 0;
        let mut priority = 0;
        assert_eq!(
            queue.receive(
                &mut buffer,
                BlockingType::NonBlocking,
                &mut size,
                &mut priority
            ),
            Status::OpOk
        );
        assert_eq!(queue.get_messages_available(), 2);
        // High-water mark does not go back down.
        assert_eq!(queue.get_message_high_water_mark(), 3);
    }

    #[test]
    fn serial_buffer_overloads_round_trip() {
        use fprime_fw::{Endianness, LinearBuffer};
        let queue = created_queue(2, 16);
        let mut message: LinearBuffer<16> = LinearBuffer::new();
        assert!(message.serialize_u32_be(0xDEAD_BEEF).is_ok());
        assert_eq!(
            queue.send_serial(&message, 1, BlockingType::NonBlocking),
            Status::OpOk
        );
        let mut destination: LinearBuffer<16> = LinearBuffer::new();
        let mut priority = 0;
        assert_eq!(
            queue.receive_serial(&mut destination, BlockingType::NonBlocking, &mut priority),
            Status::OpOk
        );
        assert_eq!(priority, 1);
        let mut value = 0u32;
        assert!(
            destination
                .deserialize_u32(&mut value, Endianness::Big)
                .is_ok()
        );
        assert_eq!(value, 0xDEAD_BEEF);
    }

    #[test]
    fn num_queues_counts_creates() {
        let before = Queue::get_num_queues();
        let _queue = created_queue(1, 1);
        assert!(Queue::get_num_queues() > before);
    }
}
