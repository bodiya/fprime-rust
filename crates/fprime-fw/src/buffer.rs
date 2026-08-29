//! The owned data buffer.
//!
//! Port of `Fw/Buffer/Buffer.{hpp,cpp}` with one deliberate change: the C++
//! `Fw::Buffer` is a non-owning descriptor over an allocator's memory; the
//! Rust `Buffer` OWNS its storage (`Box<[u8]>` recycled through pools).
//! The window semantics (`offset`/`size` over a fixed `capacity`, with the
//! C++ bounds-asserting `advance`/`set_size`) are kept exactly. The C++
//! pointer-member serialization of `Fw::Buffer` is NOT ported (in-process
//! only, unsafe by construction); async buffer ports use the escrow
//! mechanism in fprime-comp instead.

use crate::fw_assert;
use crate::serial::ExtBuf;
use fprime_config::FwSignedSizeType;

/// Owned backing storage for a [`Buffer`], recycled through pools.
pub type BufferStorage = Box<[u8]>;

/// An owned data buffer: storage + a `[offset, offset + size)` window +
/// a `u32` context for the allocating manager.
#[derive(Debug)]
pub struct Buffer {
    storage: BufferStorage,
    offset: usize,
    size: usize,
    context: u32,
}

impl Buffer {
    /// Context value meaning "no context" (`Fw::Buffer::NO_CONTEXT`).
    pub const NO_CONTEXT: u32 = 0xFFFF_FFFF;

    /// An empty, invalid buffer (the C++ default constructor).
    pub fn empty() -> Self {
        Self {
            storage: BufferStorage::default(),
            offset: 0,
            size: 0,
            context: Self::NO_CONTEXT,
        }
    }

    /// Allocate zeroed storage of `size` bytes; the window covers all of it
    /// and the context is [`Buffer::NO_CONTEXT`].
    pub fn allocate(size: usize) -> Self {
        Self::from_storage(vec![0u8; size].into_boxed_slice(), Self::NO_CONTEXT)
    }

    /// Wrap existing storage (the C++ `(data, size, context)` constructor:
    /// offset = 0, size = capacity = storage length).
    pub fn from_storage(storage: BufferStorage, context: u32) -> Self {
        let size = storage.len();
        Self {
            storage,
            offset: 0,
            size,
            context,
        }
    }

    /// True when there is storage and a non-empty window
    /// (C++ `isValid`: `data != nullptr && size > 0`).
    #[must_use = "the validity flag should be checked"]
    pub fn is_valid(&self) -> bool {
        !self.storage.is_empty() && self.size > 0
    }

    /// Total capacity of the underlying storage.
    pub fn capacity(&self) -> usize {
        self.storage.len()
    }

    /// Current window size.
    pub fn size(&self) -> usize {
        self.size
    }

    /// Current window offset from the start of the storage.
    pub fn offset(&self) -> usize {
        self.offset
    }

    /// The context value.
    pub fn context(&self) -> u32 {
        self.context
    }

    /// Set the context value.
    pub fn set_context(&mut self, context: u32) {
        self.context = context;
    }

    /// Move the window start by `amount` bytes, keeping the window end fixed
    /// (C++ `advance`). Asserts: storage present; a negative amount within
    /// the current offset; a positive amount within the current size.
    pub fn advance(&mut self, amount: FwSignedSizeType) {
        fw_assert!(!self.storage.is_empty());
        if amount < 0 {
            let back = amount.unsigned_abs() as usize;
            fw_assert!(self.offset >= back, self.offset as i32, amount as i32);
            self.offset -= back;
            self.size += back;
        } else {
            let fwd = amount as usize;
            fw_assert!(fwd <= self.size, amount as i32, self.size as i32);
            self.offset += fwd;
            self.size -= fwd;
        }
        fw_assert!(
            self.offset + self.size <= self.capacity(),
            self.offset as i32,
            self.size as i32
        );
    }

    /// Shrink or grow the window size within the capacity (C++ `setSize`:
    /// asserts `offset + size <= capacity`).
    pub fn set_size(&mut self, size: usize) {
        fw_assert!(
            self.offset + size <= self.capacity(),
            self.offset as i32,
            size as i32
        );
        self.size = size;
    }

    /// The window bytes, `storage[offset..offset + size]` (C++ `getData`).
    pub fn data(&self) -> &[u8] {
        &self.storage[self.offset..self.offset + self.size]
    }

    /// The window bytes, mutable.
    pub fn data_mut(&mut self) -> &mut [u8] {
        &mut self.storage[self.offset..self.offset + self.size]
    }

    /// A serialization view over the window that starts EMPTY (write cursor
    /// at 0 — the C++ `getSerializer` calls `resetSer`). Invalid buffers
    /// yield a zero-capacity view.
    pub fn get_serializer(&mut self) -> ExtBuf<'_> {
        if self.storage.is_empty() || self.size == 0 {
            ExtBuf::new(&mut [])
        } else {
            ExtBuf::new(&mut self.storage[self.offset..self.offset + self.size])
        }
    }

    /// A deserialization view over the window with ALL `size` bytes readable
    /// (the C++ `getDeserializer` calls `setBuffLen(size)`).
    pub fn get_deserializer(&mut self) -> ExtBuf<'_> {
        if self.storage.is_empty() || self.size == 0 {
            ExtBuf::new(&mut [])
        } else {
            let len = self.size;
            ExtBuf::with_len(&mut self.storage[self.offset..self.offset + len], len)
        }
    }

    /// Take back the owned storage (for pool recycling).
    pub fn into_storage(self) -> BufferStorage {
        self.storage
    }
}

impl Default for Buffer {
    fn default() -> Self {
        Self::empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::{SerBuf, SerializeStatus};

    #[test]
    fn empty_buffer_is_invalid() {
        let b = Buffer::empty();
        assert!(!b.is_valid());
        assert_eq!(b.context(), Buffer::NO_CONTEXT);
        assert_eq!(b.size(), 0);
        assert_eq!(b.capacity(), 0);
    }

    #[test]
    fn allocate_covers_whole_storage() {
        let b = Buffer::allocate(16);
        assert!(b.is_valid());
        assert_eq!(b.size(), 16);
        assert_eq!(b.offset(), 0);
        assert_eq!(b.capacity(), 16);
    }

    #[test]
    fn advance_moves_window_keeping_end_fixed() {
        let mut b = Buffer::allocate(10);
        b.data_mut()
            .copy_from_slice(&[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]);
        b.advance(3);
        assert_eq!(b.offset(), 3);
        assert_eq!(b.size(), 7);
        assert_eq!(b.data(), &[3, 4, 5, 6, 7, 8, 9]);
        // negative amount moves back
        b.advance(-2);
        assert_eq!(b.offset(), 1);
        assert_eq!(b.size(), 9);
        assert_eq!(b.data()[0], 1);
    }

    #[test]
    #[should_panic]
    fn advance_asserts_beyond_size() {
        let mut b = Buffer::allocate(4);
        b.advance(5);
    }

    #[test]
    #[should_panic]
    fn advance_asserts_negative_beyond_offset() {
        let mut b = Buffer::allocate(4);
        b.advance(-1);
    }

    #[test]
    #[should_panic]
    fn advance_asserts_on_empty_storage() {
        let mut b = Buffer::empty();
        b.advance(0);
    }

    #[test]
    fn set_size_within_capacity() {
        let mut b = Buffer::allocate(8);
        b.set_size(4);
        assert_eq!(b.size(), 4);
        b.advance(2);
        b.set_size(6); // offset 2 + 6 == 8 == capacity, still legal
        assert_eq!(b.size(), 6);
    }

    #[test]
    #[should_panic]
    fn set_size_asserts_beyond_capacity() {
        let mut b = Buffer::allocate(8);
        b.advance(2);
        b.set_size(7); // 2 + 7 > 8
    }

    #[test]
    fn serializer_starts_empty_and_writes_into_window() {
        let mut b = Buffer::allocate(8);
        b.advance(2); // window = storage[2..8]
        {
            let mut ser = b.get_serializer();
            assert_eq!(ser.get_size(), 0, "serializer starts empty (resetSer)");
            assert_eq!(ser.serialize_u32_be(0xAABB_CCDD), SerializeStatus::Ok);
        }
        assert_eq!(&b.data()[..4], &[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(b.into_storage()[..6], [0, 0, 0xAA, 0xBB, 0xCC, 0xDD]);
    }

    #[test]
    fn deserializer_covers_whole_window() {
        let mut b = Buffer::allocate(6);
        b.data_mut()
            .copy_from_slice(&[0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        b.advance(2);
        let mut deser = b.get_deserializer();
        assert_eq!(deser.get_size(), 4, "whole window readable (setBuffLen)");
        let mut v = 0u32;
        assert_eq!(deser.deserialize_u32_be(&mut v), SerializeStatus::Ok);
        assert_eq!(v, 0x0304_0506);
    }

    #[test]
    fn invalid_buffer_views_are_empty() {
        let mut b = Buffer::empty();
        let mut ser = b.get_serializer();
        assert_eq!(ser.serialize_u8_be(1), SerializeStatus::NoRoomLeft);
        let mut deser = b.get_deserializer();
        let mut v = 0u8;
        assert_eq!(
            deser.deserialize_u8_be(&mut v),
            SerializeStatus::DeserBufferEmpty
        );
    }

    #[test]
    fn storage_recycles() {
        let b = Buffer::from_storage(vec![9u8; 4].into_boxed_slice(), 7);
        assert_eq!(b.context(), 7);
        let storage = b.into_storage();
        assert_eq!(storage.len(), 4);
        let b2 = Buffer::from_storage(storage, Buffer::NO_CONTEXT);
        assert_eq!(b2.size(), 4);
    }
}
