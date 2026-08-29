//! Port of `Types::CircularBuffer` — a byte ring that never overwrites.
//!
//! C++ sources: `Utils/Types/CircularBuffer.cpp`, `Utils/Types/CircularBuffer.hpp`.
//! Analysis: `docs/cpp-analysis/utils-misc.md` (Types::CircularBuffer section).
//!
//! The C++ class wraps an externally supplied store with setup-once
//! semantics; the Rust port owns its storage (`Box<[u8]>`, allocated once at
//! construction), which makes the "setup exactly once, before use" contract
//! structural. Gotcha ported: the full store size is usable —
//! `get_capacity() == store size`; no byte is lost to wrap-around tracking
//! (port the code, not the C++ file-header comment). Not thread safe
//! (C++ parity); callers wrap in concurrency constructs.

use fprime_fw::fw_assert;
use fprime_fw::serial::SerializeStatus;

/// Byte ring buffer over owned storage (port of `Types::CircularBuffer`).
#[derive(Debug)]
pub struct CircularBuffer {
    /// Physical store backing this circular buffer.
    store: Box<[u8]>,
    /// Index into `store` of byte zero of the logical store; moves forward
    /// (wrapping) as data is rotated out of the front.
    head_idx: usize,
    /// Allocated size (size of the logical store).
    allocated_size: usize,
    /// Maximum allocated size seen.
    high_water_mark: usize,
}

impl CircularBuffer {
    /// Allocate a circular buffer with a `size`-byte store
    /// (C++ `CircularBuffer(U8*, FwSizeType)` / `setup`).
    ///
    /// C++ parity: fw_asserts (crashes) on `size == 0`.
    pub fn new(size: usize) -> Self {
        fw_assert!(size > 0);
        Self {
            store: vec![0u8; size].into_boxed_slice(),
            head_idx: 0,
            allocated_size: 0,
            high_water_mark: 0,
        }
    }

    /// Wrap-advance an index into the store.
    fn advance_idx(&self, idx: usize, amount: usize) -> usize {
        fw_assert!(idx < self.store.len(), idx as i64);
        (idx + amount) % self.store.len()
    }

    /// Number of bytes currently stored (C++ `get_allocated_size`).
    pub fn get_allocated_size(&self) -> usize {
        self.allocated_size
    }

    /// Number of bytes that can be stored without deleting data
    /// (C++ `get_free_size`).
    pub fn get_free_size(&self) -> usize {
        fw_assert!(
            self.allocated_size <= self.store.len(),
            self.allocated_size as i64
        );
        self.store.len() - self.allocated_size
    }

    /// Logical capacity: the store size — the FULL store is usable
    /// (C++ `get_capacity`; see module gotcha).
    pub fn get_capacity(&self) -> usize {
        self.store.len()
    }

    /// Largest tracked allocated size (C++ `get_high_water_mark`).
    pub fn get_high_water_mark(&self) -> usize {
        self.high_water_mark
    }

    /// Clear tracking of the largest allocated size
    /// (C++ `clear_high_water_mark`).
    pub fn clear_high_water_mark(&mut self) {
        self.high_water_mark = 0;
    }

    /// Append `buffer` to the back of the ring (C++ `serialize(U8*, size)`).
    ///
    /// Never overwrites existing data: returns
    /// [`SerializeStatus::NoRoomLeft`] when `buffer` is larger than the free
    /// space, leaving the ring unmodified.
    pub fn serialize(&mut self, buffer: &[u8]) -> SerializeStatus {
        if buffer.len() > self.get_free_size() {
            return SerializeStatus::NoRoomLeft;
        }
        let start = self.advance_idx(self.head_idx, self.allocated_size);
        // Copy in up to two linear segments (identical bytes to the C++
        // byte-at-a-time loop).
        let first = buffer.len().min(self.store.len() - start);
        self.store[start..start + first].copy_from_slice(&buffer[..first]);
        let rest = buffer.len() - first;
        self.store[..rest].copy_from_slice(&buffer[first..]);

        self.allocated_size += buffer.len();
        fw_assert!(
            self.allocated_size <= self.get_capacity(),
            self.allocated_size as i64
        );
        self.high_water_mark = self.high_water_mark.max(self.allocated_size);
        SerializeStatus::Ok
    }

    /// Read one byte at `offset` from the front without consuming
    /// (C++ `peek(U8&, offset)`).
    ///
    /// Returns [`SerializeStatus::DeserBufferEmpty`] when fewer than
    /// `offset + 1` bytes are stored.
    pub fn peek_u8(&self, value: &mut u8, offset: usize) -> SerializeStatus {
        if offset + 1 > self.allocated_size {
            return SerializeStatus::DeserBufferEmpty;
        }
        *value = self.store[self.advance_idx(self.head_idx, offset)];
        SerializeStatus::Ok
    }

    /// Assemble a big-endian `u32` from the 4 bytes at `offset` without
    /// consuming (C++ `peek(U32&, offset)` — always network byte order).
    ///
    /// Returns [`SerializeStatus::DeserBufferEmpty`] when fewer than
    /// `offset + 4` bytes are stored.
    pub fn peek_u32_be(&self, value: &mut u32, offset: usize) -> SerializeStatus {
        if offset + size_of::<u32>() > self.allocated_size {
            return SerializeStatus::DeserBufferEmpty;
        }
        let mut idx = self.advance_idx(self.head_idx, offset);
        let mut v: u32 = 0;
        for _ in 0..size_of::<u32>() {
            v = (v << 8) | u32::from(self.store[idx]);
            idx = self.advance_idx(idx, 1);
        }
        *value = v;
        SerializeStatus::Ok
    }

    /// Copy `dest.len()` bytes starting at `offset` into `dest` without
    /// consuming (C++ `peek(U8*, size, offset)`).
    ///
    /// Returns [`SerializeStatus::DeserBufferEmpty`] when fewer than
    /// `offset + dest.len()` bytes are stored.
    pub fn peek_bytes(&self, dest: &mut [u8], offset: usize) -> SerializeStatus {
        if dest.len() + offset > self.allocated_size {
            return SerializeStatus::DeserBufferEmpty;
        }
        let start = self.advance_idx(self.head_idx, offset);
        let first = dest.len().min(self.store.len() - start);
        dest[..first].copy_from_slice(&self.store[start..start + first]);
        let rest = dest.len() - first;
        dest[first..].copy_from_slice(&self.store[..rest]);
        SerializeStatus::Ok
    }

    /// Advance the head index, deleting `amount` bytes from the FRONT of the
    /// ring (C++ `rotate`).
    ///
    /// Returns [`SerializeStatus::DeserBufferEmpty`] when `amount` exceeds
    /// the allocated size, leaving the ring unmodified.
    pub fn rotate(&mut self, amount: usize) -> SerializeStatus {
        if amount > self.allocated_size {
            return SerializeStatus::DeserBufferEmpty;
        }
        self.head_idx = self.advance_idx(self.head_idx, amount);
        self.allocated_size -= amount;
        SerializeStatus::Ok
    }

    /// Delete `amount` bytes from the BACK of the ring (most recently added
    /// data), without moving the head (C++ `trim`).
    ///
    /// Returns [`SerializeStatus::DeserBufferEmpty`] when `amount` exceeds
    /// the allocated size, leaving the ring unmodified.
    pub fn trim(&mut self, amount: usize) -> SerializeStatus {
        if amount > self.allocated_size {
            return SerializeStatus::DeserBufferEmpty;
        }
        self.allocated_size -= amount;
        SerializeStatus::Ok
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_ok(status: SerializeStatus) {
        assert_eq!(status, SerializeStatus::Ok);
    }

    #[test]
    fn full_store_is_usable_no_lost_byte() {
        // Gotcha: get_capacity() == store size; no wrap-tracking byte lost.
        let mut cb = CircularBuffer::new(8);
        assert_eq!(cb.get_capacity(), 8);
        assert_eq!(cb.get_free_size(), 8);
        assert_ok(cb.serialize(&[1, 2, 3, 4, 5, 6, 7, 8]));
        assert_eq!(cb.get_allocated_size(), 8);
        assert_eq!(cb.get_free_size(), 0);
    }

    #[test]
    fn serialize_refuses_overwrite() {
        let mut cb = CircularBuffer::new(4);
        assert_ok(cb.serialize(&[1, 2, 3]));
        // Two bytes will not fit in the one remaining byte.
        assert_eq!(cb.serialize(&[4, 5]), SerializeStatus::NoRoomLeft);
        // Ring unmodified by the failed serialize.
        assert_eq!(cb.get_allocated_size(), 3);
        let mut out = [0u8; 3];
        assert_ok(cb.peek_bytes(&mut out, 0));
        assert_eq!(out, [1, 2, 3]);
        // Exactly-fitting data is accepted.
        assert_ok(cb.serialize(&[4]));
        assert_eq!(cb.serialize(&[5]), SerializeStatus::NoRoomLeft);
    }

    #[test]
    fn wrap_around_write_and_peek() {
        let mut cb = CircularBuffer::new(5);
        assert_ok(cb.serialize(&[10, 11, 12, 13]));
        assert_ok(cb.rotate(3)); // head at index 3, one byte (13) stored
        // This write wraps the end of the store: indices 4, 0, 1.
        assert_ok(cb.serialize(&[14, 15, 16]));
        assert_eq!(cb.get_allocated_size(), 4);
        let mut out = [0u8; 4];
        assert_ok(cb.peek_bytes(&mut out, 0));
        assert_eq!(out, [13, 14, 15, 16]);
        // Peek across the wrap with an offset.
        let mut out2 = [0u8; 2];
        assert_ok(cb.peek_bytes(&mut out2, 2));
        assert_eq!(out2, [15, 16]);
    }

    #[test]
    fn peek_u8_and_bounds() {
        let mut cb = CircularBuffer::new(4);
        let mut v = 0u8;
        assert_eq!(cb.peek_u8(&mut v, 0), SerializeStatus::DeserBufferEmpty);
        assert_ok(cb.serialize(&[0xAB, 0xCD]));
        assert_ok(cb.peek_u8(&mut v, 0));
        assert_eq!(v, 0xAB);
        assert_ok(cb.peek_u8(&mut v, 1));
        assert_eq!(v, 0xCD);
        assert_eq!(cb.peek_u8(&mut v, 2), SerializeStatus::DeserBufferEmpty);
    }

    #[test]
    fn peek_u32_is_big_endian_and_wraps() {
        let mut cb = CircularBuffer::new(6);
        assert_ok(cb.serialize(&[0, 0, 0, 0]));
        assert_ok(cb.rotate(4)); // head at index 4
        // Bytes land at store indices 4, 5, 0, 1 — wrapping the store end.
        assert_ok(cb.serialize(&[0xDE, 0xAD, 0xBE, 0xEF]));
        let mut v = 0u32;
        assert_ok(cb.peek_u32_be(&mut v, 0));
        assert_eq!(v, 0xDEAD_BEEF);
        // Insufficient data at offset 1.
        assert_eq!(cb.peek_u32_be(&mut v, 1), SerializeStatus::DeserBufferEmpty);
    }

    #[test]
    fn rotate_and_trim_bounds() {
        let mut cb = CircularBuffer::new(8);
        assert_ok(cb.serialize(&[1, 2, 3, 4, 5]));
        // More than allocated -> DeserBufferEmpty, state unchanged.
        assert_eq!(cb.rotate(6), SerializeStatus::DeserBufferEmpty);
        assert_eq!(cb.trim(6), SerializeStatus::DeserBufferEmpty);
        assert_eq!(cb.get_allocated_size(), 5);
        // rotate consumes from the front...
        assert_ok(cb.rotate(2));
        let mut v = 0u8;
        assert_ok(cb.peek_u8(&mut v, 0));
        assert_eq!(v, 3);
        // ...trim drops from the back.
        assert_ok(cb.trim(2));
        assert_eq!(cb.get_allocated_size(), 1);
        assert_ok(cb.peek_u8(&mut v, 0));
        assert_eq!(v, 3);
        // Emptying exactly is allowed.
        assert_ok(cb.rotate(1));
        assert_eq!(cb.get_allocated_size(), 0);
        assert_eq!(cb.rotate(1), SerializeStatus::DeserBufferEmpty);
        assert_eq!(cb.trim(1), SerializeStatus::DeserBufferEmpty);
        // rotate(0)/trim(0) on empty are OK (0 <= allocated).
        assert_ok(cb.rotate(0));
        assert_ok(cb.trim(0));
    }

    #[test]
    fn trim_then_serialize_reuses_back_space() {
        let mut cb = CircularBuffer::new(4);
        assert_ok(cb.serialize(&[1, 2, 3, 4]));
        assert_ok(cb.trim(2));
        assert_ok(cb.serialize(&[9, 10]));
        let mut out = [0u8; 4];
        assert_ok(cb.peek_bytes(&mut out, 0));
        assert_eq!(out, [1, 2, 9, 10]);
    }

    #[test]
    fn high_water_mark_tracks_and_clears() {
        let mut cb = CircularBuffer::new(8);
        assert_eq!(cb.get_high_water_mark(), 0);
        assert_ok(cb.serialize(&[0; 5]));
        assert_ok(cb.rotate(4));
        assert_eq!(cb.get_high_water_mark(), 5);
        assert_ok(cb.serialize(&[0; 2]));
        // 3 allocated now, high water stays at 5.
        assert_eq!(cb.get_allocated_size(), 3);
        assert_eq!(cb.get_high_water_mark(), 5);
        assert_ok(cb.serialize(&[0; 4]));
        assert_eq!(cb.get_high_water_mark(), 7);
        cb.clear_high_water_mark();
        assert_eq!(cb.get_high_water_mark(), 0);
        // Failed serialize does not move the mark.
        assert_eq!(cb.serialize(&[0; 2]), SerializeStatus::NoRoomLeft);
        assert_eq!(cb.get_high_water_mark(), 0);
    }

    #[test]
    fn serialize_empty_slice_is_ok() {
        let mut cb = CircularBuffer::new(2);
        assert_ok(cb.serialize(&[]));
        assert_eq!(cb.get_allocated_size(), 0);
        assert_ok(cb.serialize(&[1, 2]));
        // Zero-size write also OK when full (0 <= free).
        assert_ok(cb.serialize(&[]));
    }

    #[test]
    fn many_wraps_preserve_fifo_order() {
        let mut cb = CircularBuffer::new(7);
        let mut next_in = 0u8;
        let mut next_out = 0u8;
        for _ in 0..50 {
            for _ in 0..3 {
                if cb.serialize(&[next_in]).is_ok() {
                    next_in = next_in.wrapping_add(1);
                }
            }
            let mut v = 0u8;
            while cb.peek_u8(&mut v, 0).is_ok() {
                assert_eq!(v, next_out);
                next_out = next_out.wrapping_add(1);
                assert_ok(cb.rotate(1));
            }
        }
        assert_eq!(next_in, next_out);
    }
}
