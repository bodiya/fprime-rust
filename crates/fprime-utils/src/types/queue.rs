//! Port of `Types::Queue` — a fixed-message-size FIFO/LIFO over the byte ring.
//!
//! C++ sources: `Utils/Types/Queue.cpp`, `Utils/Types/Queue.hpp`.
//! Analysis: `docs/cpp-analysis/utils-misc.md` (Types::Queue section).
//!
//! The ring is sized exactly `depth * message_size` (C++ setup passes
//! `depth * message_size`, NOT the raw storage size, to the internal
//! `CircularBuffer`). Sizes reported by `get_queue_size` and
//! `get_high_water_mark` are in MESSAGE units, not bytes. Not thread safe
//! (C++ parity); callers wrap in concurrency constructs.

use fprime_fw::fw_assert;
use fprime_fw::serial::SerializeStatus;

use crate::types::circular_buffer::CircularBuffer;

/// Queue ordering mode (port of `Types::QueueMode`, exact discriminants).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum QueueMode {
    /// First-In-First-Out: dequeue from the front (`QUEUE_FIFO`).
    #[default]
    Fifo = 0,
    /// Last-In-First-Out: dequeue from the back (`QUEUE_LIFO`).
    Lifo = 1,
}

/// Queue overflow behavior mode (port of `Types::QueueOverflowMode`,
/// exact discriminants).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum QueueOverflowMode {
    /// Drop the newest (incoming) message on overflow (`QUEUE_DROP_NEWEST`).
    #[default]
    DropNewest = 0,
    /// Drop the oldest (front) message on overflow (`QUEUE_DROP_OLDEST`).
    DropOldest = 1,
}

/// Fixed-message-size FIFO/LIFO queue (port of `Types::Queue`).
#[derive(Debug)]
pub struct Queue {
    internal: CircularBuffer,
    message_size: usize,
    mode: QueueMode,
    overflow_mode: QueueOverflowMode,
}

impl Queue {
    /// Construct a queue of `depth` messages of `message_size` bytes each
    /// (C++ `Queue()` + `setup`; storage is owned here, sized exactly
    /// `depth * message_size`).
    ///
    /// C++ parity: fw_asserts (crashes) on a zero depth or message size
    /// (the C++ ring setup asserts `size > 0`, and enqueue/dequeue assert
    /// `m_message_size > 0`).
    pub fn new(
        depth: usize,
        message_size: usize,
        mode: QueueMode,
        overflow_mode: QueueOverflowMode,
    ) -> Self {
        fw_assert!(depth > 0, depth as i64);
        fw_assert!(message_size > 0, message_size as i64);
        Self {
            internal: CircularBuffer::new(depth * message_size),
            message_size,
            mode,
            overflow_mode,
        }
    }

    /// Push one message onto the queue (C++ `enqueue`).
    ///
    /// C++ parity: fw_asserts (crashes) unless
    /// `message.len() == message_size`. When the queue is full:
    /// - `DropNewest`: returns [`SerializeStatus::NoRoomLeft`], queue
    ///   unmodified;
    /// - `DropOldest`: rotates the oldest message out, enqueues the new one,
    ///   and returns [`SerializeStatus::DiscardedExisting`]
    ///   (success-with-note, NOT an error).
    pub fn enqueue(&mut self, message: &[u8]) -> SerializeStatus {
        fw_assert!(
            self.message_size == message.len(),
            message.len() as i64,
            self.message_size as i64
        );
        let status = self.internal.serialize(message);

        if status == SerializeStatus::NoRoomLeft
            && self.overflow_mode == QueueOverflowMode::DropOldest
        {
            // Remove the oldest message by rotating, then retry.
            let rotate_status = self.internal.rotate(self.message_size);
            if rotate_status != SerializeStatus::Ok {
                return rotate_status;
            }
            let retry = self.internal.serialize(message);
            if retry != SerializeStatus::Ok {
                return retry;
            }
            // Let the caller know we deleted data.
            return SerializeStatus::DiscardedExisting;
        }

        status
    }

    /// Pop one message off the queue into `message` (C++ `dequeue`).
    ///
    /// FIFO removes the oldest (front) message; LIFO removes the newest
    /// (back) message. C++ parity: fw_asserts (crashes) unless
    /// `message.len() >= message_size`; only the first `message_size` bytes
    /// are written. Returns a non-Ok status when the queue is empty.
    pub fn dequeue(&mut self, message: &mut [u8]) -> SerializeStatus {
        fw_assert!(
            self.message_size <= message.len(),
            message.len() as i64,
            self.message_size as i64
        );
        match self.mode {
            QueueMode::Fifo => {
                // FIFO: dequeue from the front (oldest message).
                let result = self
                    .internal
                    .peek_bytes(&mut message[..self.message_size], 0);
                if result != SerializeStatus::Ok {
                    return result;
                }
                self.internal.rotate(self.message_size)
            }
            QueueMode::Lifo => {
                // LIFO: dequeue from the back (newest message).
                let current_size = self.internal.get_allocated_size();
                if current_size < self.message_size {
                    return SerializeStatus::DeserBufferEmpty;
                }
                let offset = current_size - self.message_size;
                let result = self
                    .internal
                    .peek_bytes(&mut message[..self.message_size], offset);
                if result != SerializeStatus::Ok {
                    return result;
                }
                self.internal.trim(self.message_size)
            }
        }
    }

    /// Remove and return the oldest (front) message REGARDLESS of the queue
    /// mode (C++ `popFront`) — needed when callers must capture the entry a
    /// `DropOldest` overflow is about to discard.
    ///
    /// C++ parity: fw_asserts (crashes) unless
    /// `message.len() >= message_size`.
    pub fn pop_front(&mut self, message: &mut [u8]) -> SerializeStatus {
        fw_assert!(
            self.message_size <= message.len(),
            message.len() as i64,
            self.message_size as i64
        );
        let result = self
            .internal
            .peek_bytes(&mut message[..self.message_size], 0);
        if result != SerializeStatus::Ok {
            return result;
        }
        self.internal.rotate(self.message_size)
    }

    /// Number of messages currently queued (C++ `getQueueSize`) —
    /// MESSAGE units, not bytes.
    pub fn get_queue_size(&self) -> usize {
        self.internal.get_allocated_size() / self.message_size
    }

    /// The fixed per-message size in bytes this queue was constructed with.
    pub fn get_message_size(&self) -> usize {
        self.message_size
    }

    /// Largest number of messages ever queued (C++ `get_high_water_mark`) —
    /// MESSAGE units, not bytes (documented gotcha).
    pub fn get_high_water_mark(&self) -> usize {
        self.internal.get_high_water_mark() / self.message_size
    }

    /// Clear high-water tracking (C++ `clear_high_water_mark`).
    pub fn clear_high_water_mark(&mut self) {
        self.internal.clear_high_water_mark();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_ok(status: SerializeStatus) {
        assert_eq!(status, SerializeStatus::Ok);
    }

    fn msg(tag: u8) -> [u8; 4] {
        [
            tag,
            tag.wrapping_add(1),
            tag.wrapping_add(2),
            tag.wrapping_add(3),
        ]
    }

    #[test]
    fn fifo_order() {
        let mut q = Queue::new(3, 4, QueueMode::Fifo, QueueOverflowMode::DropNewest);
        assert_ok(q.enqueue(&msg(10)));
        assert_ok(q.enqueue(&msg(20)));
        assert_ok(q.enqueue(&msg(30)));
        assert_eq!(q.get_queue_size(), 3);
        let mut out = [0u8; 4];
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(10));
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(20));
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(30));
        assert_eq!(q.get_queue_size(), 0);
        assert_eq!(q.dequeue(&mut out), SerializeStatus::DeserBufferEmpty);
    }

    #[test]
    fn lifo_order() {
        let mut q = Queue::new(3, 4, QueueMode::Lifo, QueueOverflowMode::DropNewest);
        assert_ok(q.enqueue(&msg(10)));
        assert_ok(q.enqueue(&msg(20)));
        assert_ok(q.enqueue(&msg(30)));
        let mut out = [0u8; 4];
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(30));
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(20));
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(10));
        assert_eq!(q.dequeue(&mut out), SerializeStatus::DeserBufferEmpty);
    }

    #[test]
    fn drop_newest_full_returns_no_room_left() {
        let mut q = Queue::new(2, 4, QueueMode::Fifo, QueueOverflowMode::DropNewest);
        assert_ok(q.enqueue(&msg(1)));
        assert_ok(q.enqueue(&msg(2)));
        assert_eq!(q.enqueue(&msg(3)), SerializeStatus::NoRoomLeft);
        // Queue unmodified: original two messages intact, in order.
        assert_eq!(q.get_queue_size(), 2);
        let mut out = [0u8; 4];
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(1));
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(2));
    }

    #[test]
    fn drop_oldest_full_returns_discarded_existing() {
        let mut q = Queue::new(2, 4, QueueMode::Fifo, QueueOverflowMode::DropOldest);
        assert_ok(q.enqueue(&msg(1)));
        assert_ok(q.enqueue(&msg(2)));
        // Gotcha: DROP_OLDEST overflow returns DiscardedExisting, not Ok.
        assert_eq!(q.enqueue(&msg(3)), SerializeStatus::DiscardedExisting);
        assert_eq!(q.get_queue_size(), 2);
        // Oldest (1) was dropped; front is now 2, back is 3.
        let mut out = [0u8; 4];
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(2));
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(3));
    }

    #[test]
    fn drop_oldest_lifo_still_rotates_front() {
        // DROP_OLDEST always discards the FRONT message even in LIFO mode.
        let mut q = Queue::new(2, 4, QueueMode::Lifo, QueueOverflowMode::DropOldest);
        assert_ok(q.enqueue(&msg(1)));
        assert_ok(q.enqueue(&msg(2)));
        assert_eq!(q.enqueue(&msg(3)), SerializeStatus::DiscardedExisting);
        let mut out = [0u8; 4];
        // LIFO dequeue: newest first -> 3, then 2. 1 was discarded.
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(3));
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(2));
        assert_eq!(q.dequeue(&mut out), SerializeStatus::DeserBufferEmpty);
    }

    #[test]
    fn pop_front_ignores_lifo_mode() {
        let mut q = Queue::new(3, 4, QueueMode::Lifo, QueueOverflowMode::DropNewest);
        assert_ok(q.enqueue(&msg(1)));
        assert_ok(q.enqueue(&msg(2)));
        let mut out = [0u8; 4];
        assert_ok(q.pop_front(&mut out));
        assert_eq!(out, msg(1)); // front, despite LIFO
        assert_ok(q.dequeue(&mut out));
        assert_eq!(out, msg(2));
        assert_eq!(q.pop_front(&mut out), SerializeStatus::DeserBufferEmpty);
    }

    #[test]
    fn dequeue_into_oversized_buffer_writes_message_size_bytes() {
        let mut q = Queue::new(2, 4, QueueMode::Fifo, QueueOverflowMode::DropNewest);
        assert_ok(q.enqueue(&msg(7)));
        let mut out = [0xEEu8; 6];
        assert_ok(q.dequeue(&mut out));
        assert_eq!(&out[..4], &msg(7));
        assert_eq!(&out[4..], &[0xEE, 0xEE]); // untouched tail
    }

    #[test]
    fn sizes_and_high_water_in_message_units() {
        let mut q = Queue::new(4, 3, QueueMode::Fifo, QueueOverflowMode::DropNewest);
        assert_eq!(q.get_message_size(), 3);
        assert_eq!(q.get_queue_size(), 0);
        assert_eq!(q.get_high_water_mark(), 0);
        assert_ok(q.enqueue(&[1, 2, 3]));
        assert_ok(q.enqueue(&[4, 5, 6]));
        assert_ok(q.enqueue(&[7, 8, 9]));
        assert_eq!(q.get_queue_size(), 3);
        assert_eq!(q.get_high_water_mark(), 3);
        let mut out = [0u8; 3];
        assert_ok(q.dequeue(&mut out));
        assert_ok(q.dequeue(&mut out));
        // Gotcha: high-water is in messages, not bytes, and persists.
        assert_eq!(q.get_queue_size(), 1);
        assert_eq!(q.get_high_water_mark(), 3);
        q.clear_high_water_mark();
        assert_eq!(q.get_high_water_mark(), 0);
    }

    #[test]
    fn wrap_around_across_ring_boundary_preserves_messages() {
        // Depth 3 x 4 bytes = 12-byte ring; enqueue/dequeue cycles force the
        // message slots to wrap the store end.
        let mut q = Queue::new(3, 4, QueueMode::Fifo, QueueOverflowMode::DropNewest);
        let mut out = [0u8; 4];
        for round in 0u8..10 {
            let tag = round.wrapping_mul(4);
            assert_ok(q.enqueue(&msg(tag)));
            assert_ok(q.enqueue(&msg(tag.wrapping_add(1))));
            assert_ok(q.dequeue(&mut out));
            assert_eq!(out, msg(tag));
            assert_ok(q.dequeue(&mut out));
            assert_eq!(out, msg(tag.wrapping_add(1)));
        }
    }
}
