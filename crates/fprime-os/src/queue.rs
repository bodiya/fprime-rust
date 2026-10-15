//! The F Prime priority message queue.
//!
//! Port of the `Os::Queue` facade (Os/Queue.{hpp,cpp}) and its delegate
//! interface `Os::QueueInterface` — see `docs/cpp-analysis/os.md`. The
//! facade owns the name, the configured depth/message size and the
//! create-once check, and forwards to one delegate chosen at build time,
//! as upstream links exactly one `Default*Queue.cpp`:
//!
//! | build | delegate ([`DefaultQueue`]) | upstream |
//! |---|---|---|
//! | default | [`PriorityQueue`](crate::generic::PriorityQueue): stable max-heap under one mutex, condition-variable blocking | `Os/Generic/DefaultPriorityQueue.cpp` |
//! | feature `lockless-queue` | [`LocklessPriorityQueue`](crate::generic::LocklessPriorityQueue): atomic slot state machine, ISR-safe non-blocking calls, polling blocking calls | `Os/Generic/DefaultLocklessPriorityQueue.cpp` |
//!
//! The feature is the recommended configuration for `no_std` deployments
//! that send to component queues from interrupt handlers: the default
//! delegate's state is behind a (spin) lock, which an interrupt that
//! preempted a holder on the same core would spin on forever. With the
//! lockless delegate, the whole async-port send path — the facade's
//! created check is a once-cell read — takes no lock (send with a
//! non-blocking queue-full policy from an ISR; blocking calls poll and
//! must stay in task context). Without interrupt senders the default is
//! the better choice: blocked receivers sleep until signaled instead of
//! waking every `LOCKLESS_QUEUE_BLOCKING_BACKOFF_US`. Both delegates are
//! always compiled and can also be used directly through
//! [`QueueInterface`].
//!
//! Facade semantics (both delegates): highest priority first, FIFO within
//! equal priority; `create` asserts depth/size > 0 and returns
//! [`Status::AlreadyCreated`] on a second create; `send` with a message
//! larger than the configured size returns [`Status::SizeMismatch`];
//! `receive` with a destination smaller than the CONFIGURED message size
//! (not the actual message size) returns [`Status::SizeMismatch`];
//! non-blocking send on full returns [`Status::Full`], non-blocking
//! receive on empty returns [`Status::Empty`]; everything before `create`
//! returns [`Status::Uninitialized`].
//!
//! Concurrency model: unlike the C++ front class, [`Queue`] uses interior
//! mutability throughout — every method (including `create`) takes `&self`,
//! so a `Queue` is shared as `Arc<Queue>` across sender and receiver threads
//! with no external locking. The created configuration and delegate live in
//! a write-once cell, so reading them takes no lock.
//!
//! C++ `teardown()` is `Drop` (storage is freed with the `Queue`), and the
//! C++ `create` `id` (used only by the optional QueueRegistry) is not
//! ported.

use crate::generic;
use fprime_config::{FW_QUEUE_NAME_BUFFER_SIZE, FwQueuePriorityType, FwSizeType};
use fprime_fw::fw_assert;
use fprime_fw::sync::{AtomicU64, OnceLock, Ordering};
use fprime_fw::{FwString, SerBuf, SerBufAny};

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

/// Port of `Os::QueueInterface`: what an `Os::Queue` delegate implements.
///
/// Deviation: C++ delegates are constructed empty and then `create`d (and
/// assert on use before that); here `create` constructs the delegate, so an
/// uncreated delegate cannot be used — the uncreated state is the
/// [`Queue`] facade's. `teardown` is `Drop`, and the unused `id`/`name`
/// arguments of the C++ `create` are not taken.
pub trait QueueInterface: Send + Sync + Sized {
    /// Allocate storage for `depth` messages of up to `message_size` bytes
    /// (all of it, once). Asserts both are nonzero.
    fn create(depth: usize, message_size: usize) -> Result<Self, Status>;

    /// Send `buffer` at `priority`: [`Status::SizeMismatch`] if it exceeds
    /// the message size, [`Status::Full`] if non-blocking and no space.
    fn send(
        &self,
        buffer: &[u8],
        priority: FwQueuePriorityType,
        block_type: BlockingType,
    ) -> Status;

    /// Receive the highest-priority (oldest among equals) message into
    /// `destination`, which must hold it (asserted); [`Status::Empty`] if
    /// non-blocking and nothing is queued.
    fn receive(
        &self,
        destination: &mut [u8],
        block_type: BlockingType,
        actual_size: &mut FwSizeType,
        priority: &mut FwQueuePriorityType,
    ) -> Status;

    /// Number of messages currently receivable.
    fn get_messages_available(&self) -> FwSizeType;

    /// Maximum number of messages ever queued at once.
    fn get_message_high_water_mark(&self) -> FwSizeType;
}

/// The `Os::Queue` delegate of this build: the lock-based
/// [`generic::PriorityQueue`] (upstream `DefaultPriorityQueue.cpp`).
#[cfg(not(feature = "lockless-queue"))]
pub type DefaultQueue = generic::PriorityQueue;

/// The `Os::Queue` delegate of this build: the ISR-safe
/// [`generic::LocklessPriorityQueue`], selected by the `lockless-queue`
/// feature (upstream `DefaultLocklessPriorityQueue.cpp`).
#[cfg(feature = "lockless-queue")]
pub type DefaultQueue = generic::LocklessPriorityQueue;

/// A created queue: the facade fields C++ `Os::Queue` keeps
/// (`m_name`/`m_depth`/`m_size`) plus the delegate.
struct Created {
    name: QueueString,
    depth: usize,
    message_size: usize,
    delegate: DefaultQueue,
}

/// Global created-queue count (C++ `Os::Queue::s_queueCount`, guarded by a
/// static mutex there; a relaxed atomic here — same observable values).
static QUEUE_COUNT: AtomicU64 = AtomicU64::new(0);

/// The F Prime priority message queue (see module docs).
pub struct Queue {
    created: OnceLock<Created>,
}

impl Default for Queue {
    fn default() -> Self {
        Self::new()
    }
}

/// `FwSizeType` to `usize`; a size the target cannot address is a
/// programming error.
fn to_usize(value: FwSizeType) -> usize {
    let converted = usize::try_from(value);
    fw_assert!(converted.is_ok());
    converted.unwrap_or(0)
}

impl Queue {
    /// Construct an uncreated queue. All operations except [`Queue::create`]
    /// return [`Status::Uninitialized`] until `create` succeeds.
    pub const fn new() -> Self {
        Self {
            created: OnceLock::new(),
        }
    }

    /// Create the queue storage (port of `Os::Queue::create` + the
    /// delegate's `create`). Allocates everything up front; steady state
    /// never allocates.
    ///
    /// C++ parity: FW_ASSERTs `depth > 0 && message_size > 0` (crash, not
    /// status); a second create returns [`Status::AlreadyCreated`]; a
    /// delegate allocation failure is returned as is.
    pub fn create(&self, name: &str, depth: FwSizeType, message_size: FwSizeType) -> Status {
        fw_assert!(depth > 0);
        fw_assert!(message_size > 0);
        if self.created.get().is_some() {
            return Status::AlreadyCreated;
        }
        let depth = to_usize(depth);
        let message_size = to_usize(message_size);
        let delegate = match DefaultQueue::create(depth, message_size) {
            Ok(delegate) => delegate,
            Err(status) => return status,
        };
        let mut queue_name = QueueString::new();
        queue_name.set(name);
        let created = Created {
            name: queue_name,
            depth,
            message_size,
            delegate,
        };
        // A concurrent create that won the race keeps its configuration.
        if self.created.set(created).is_err() {
            return Status::AlreadyCreated;
        }
        QUEUE_COUNT.fetch_add(1, Ordering::Relaxed);
        Status::OpOk
    }

    /// Send a message (port of `Os::Queue::send` + the delegate's `send`).
    ///
    /// Status flow: [`Status::Uninitialized`] before create;
    /// [`Status::SizeMismatch`] when `buffer.len()` exceeds the configured
    /// message size; [`Status::Full`] for a non-blocking send on a full
    /// queue (the lockless delegate may also report it spuriously under
    /// contention). A blocking send waits for space.
    pub fn send(
        &self,
        buffer: &[u8],
        priority: FwQueuePriorityType,
        block_type: BlockingType,
    ) -> Status {
        let Some(created) = self.created.get() else {
            return Status::Uninitialized;
        };
        if buffer.len() > created.message_size {
            return Status::SizeMismatch;
        }
        created.delegate.send(buffer, priority, block_type)
    }

    /// Receive the highest-priority message (port of `Os::Queue::receive` +
    /// the delegate's `receive`). `actual_size` and `priority` are outputs.
    ///
    /// Status flow: [`Status::Uninitialized`] before create;
    /// [`Status::SizeMismatch`] when `destination.len()` is smaller than the
    /// queue's CONFIGURED message size (C++ checks the configured size, not
    /// the pending message's size); [`Status::Empty`] for a non-blocking
    /// receive on an empty queue. A blocking receive waits for a message.
    pub fn receive(
        &self,
        destination: &mut [u8],
        block_type: BlockingType,
        actual_size: &mut FwSizeType,
        priority: &mut FwQueuePriorityType,
    ) -> Status {
        let Some(created) = self.created.get() else {
            return Status::Uninitialized;
        };
        if destination.len() < created.message_size {
            return Status::SizeMismatch;
        }
        created
            .delegate
            .receive(destination, block_type, actual_size, priority)
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

    /// Number of messages currently queued (0 before create; the
    /// delegate's `getMessagesAvailable`).
    pub fn get_messages_available(&self) -> FwSizeType {
        self.created
            .get()
            .map_or(0, |created| created.delegate.get_messages_available())
    }

    /// Maximum number of messages ever simultaneously queued (0 before
    /// create; the delegate's `getMessageHighWaterMark`).
    pub fn get_message_high_water_mark(&self) -> FwSizeType {
        self.created
            .get()
            .map_or(0, |created| created.delegate.get_message_high_water_mark())
    }

    /// Configured queue depth (0 before create).
    pub fn get_depth(&self) -> FwSizeType {
        self.created
            .get()
            .map_or(0, |created| created.depth as FwSizeType)
    }

    /// Configured maximum message size (0 before create).
    pub fn get_message_size(&self) -> FwSizeType {
        self.created
            .get()
            .map_or(0, |created| created.message_size as FwSizeType)
    }

    /// Queue name as set at create (empty before create).
    pub fn get_name(&self) -> QueueString {
        self.created
            .get()
            .map_or_else(QueueString::new, |created| created.name.clone())
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
    fn delegate_follows_the_lockless_queue_feature() {
        let expected = if cfg!(feature = "lockless-queue") {
            "LocklessPriorityQueue"
        } else {
            "PriorityQueue"
        };
        let name = core::any::type_name::<DefaultQueue>();
        assert_eq!(name.rsplit("::").next(), Some(expected));
    }

    // C++ parity: a failed delegate create leaves the facade uncreated (the
    // lockless delegate reports allocation failure as a status).
    #[cfg(feature = "lockless-queue")]
    #[test]
    fn failed_create_leaves_queue_uninitialized() {
        let queue = Queue::new();
        assert_eq!(
            queue.create("huge", 4, isize::MAX as FwSizeType / 2),
            Status::AllocationFailed
        );
        assert_eq!(queue.get_depth(), 0);
        assert_eq!(
            queue.send(&[1], 0, BlockingType::NonBlocking),
            Status::Uninitialized
        );
        assert_eq!(queue.create("ok", 2, 4), Status::OpOk);
    }

    #[test]
    fn num_queues_counts_creates() {
        let before = Queue::get_num_queues();
        let _queue = created_queue(1, 1);
        assert!(Queue::get_num_queues() > before);
    }
}
