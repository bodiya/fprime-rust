//! `BufferEscrow` — safe replacement for the C++ practice of serializing a
//! raw `Fw::Buffer` (pointer + size) into an async queue message.
//!
//! Rust `Buffer` is owned and cannot be flattened into a byte message, so a
//! component with async buffer-carrying ports owns an escrow: the input
//! adapter deposits the `Buffer` and serializes the returned `u64` token
//! where C++ would serialize the pointer (same 8-byte wire width, so the
//! message layout is unchanged); the dispatch side claims the token back on
//! the component thread.
//!
//! Where an `Fw.Buffer` is a *member of a serialized value* rather than a
//! port argument — a state machine signal value such as
//! `Svc.ComDataContextPair` — the value holds an [`EscrowedBuffer`]: the
//! token plus the buffer's descriptor, serialized exactly as C++
//! `Fw::Buffer::serializeTo` lays the buffer out, with the token in the
//! pointer slot.

use alloc::vec::Vec;
use fprime_config::FwSizeType;
use fprime_fw::sync::Mutex;
use fprime_fw::{
    Buffer, Deserialize, Endianness, FppSized, SerBufAny, Serialize, SerializeStatus, fw_assert,
};

/// Token layout: `[generation: u32][slot index: u32]`. The generation
/// counter catches stale/double-claimed tokens (`fw_assert`), the moral
/// equivalent of the memory-corruption a dangling C++ pointer would cause.
#[derive(Debug)]
struct Slot {
    generation: u32,
    buffer: Option<Buffer>,
}

#[derive(Debug, Default)]
struct State {
    slots: Vec<Slot>,
    free: Vec<usize>,
}

/// A slab of in-flight `Buffer`s keyed by opaque `u64` tokens.
#[derive(Debug, Default)]
pub struct BufferEscrow {
    state: Mutex<State>,
}

impl BufferEscrow {
    /// New, empty escrow.
    pub fn new() -> Self {
        Self::default()
    }

    /// New escrow with `capacity` slots pre-allocated (size it to the queue
    /// depth of the ports it serves to avoid steady-state allocation).
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            state: Mutex::new(State {
                slots: Vec::with_capacity(capacity),
                free: Vec::with_capacity(capacity),
            }),
        }
    }

    /// Deposit a buffer, returning the token to serialize into the queue
    /// message (in the position C++ serializes the buffer pointer).
    pub fn deposit(&self, buffer: Buffer) -> u64 {
        let mut state = self.state.lock();
        let index = match state.free.pop() {
            Some(index) => {
                let slot = &mut state.slots[index];
                // Free-list invariant: a freed slot holds no buffer.
                fw_assert!(slot.buffer.is_none(), index as i32);
                slot.buffer = Some(buffer);
                index
            }
            None => {
                state.slots.push(Slot {
                    generation: 0,
                    buffer: Some(buffer),
                });
                state.slots.len() - 1
            }
        };
        let generation = state.slots[index].generation;
        (u64::from(generation) << 32) | (index as u64)
    }

    /// Claim the buffer back for a token produced by
    /// [`BufferEscrow::deposit`]. `fw_assert`s on an invalid, stale, or
    /// already-claimed token (the safe analogue of dereferencing a bad
    /// pointer in the C++ message).
    pub fn claim(&self, token: u64) -> Buffer {
        let mut state = self.state.lock();
        let index = (token & 0xFFFF_FFFF) as usize;
        let generation = (token >> 32) as u32;
        fw_assert!(index < state.slots.len(), index as i32);
        let slot = &mut state.slots[index];
        fw_assert!(slot.generation == generation, generation as i32);
        let buffer = slot.buffer.take();
        fw_assert!(buffer.is_some(), index as i32);
        // Retire this generation so a duplicate token cannot match again.
        slot.generation = slot.generation.wrapping_add(1);
        state.free.push(index);
        match buffer {
            Some(buffer) => buffer,
            // Unreachable past the assert above; kept fatal (a returning
            // assert hook cannot conjure the buffer back).
            None => unreachable!("escrow slot empty after assert"),
        }
    }

    /// Number of buffers currently held.
    pub fn len(&self) -> usize {
        let state = self.state.lock();
        state.slots.iter().filter(|s| s.buffer.is_some()).count()
    }

    /// True when no buffers are held.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// An `Fw.Buffer` held in a [`BufferEscrow`], as a plain serializable value
/// (ROADMAP item 11's buffer-carrying signal values).
///
/// C++ serializes an `Fw::Buffer` value as its pointer, size, context,
/// offset and capacity (`Fw::Buffer::serializeTo`, `SERIALIZED_SIZE = 3 *
/// sizeof(FwSizeType) + sizeof(U32) + sizeof(U8*)`); this type writes the
/// same fields in the same order with the escrow token in the pointer slot
/// (8 bytes, the 64-bit host pointer width), so a message carrying it is
/// the C++ message byte for byte except for the opaque pointer value. The
/// descriptor fields are a snapshot taken at [`EscrowedBuffer::deposit`];
/// the buffer itself cannot change while it is escrowed.
///
/// Ownership moves exactly once: [`EscrowedBuffer::deposit`] moves the
/// buffer into the escrow, and the one [`EscrowedBuffer::claim`] moves it
/// out. The value is deliberately not `Clone`; a second claim of the same
/// token (through a deserialized copy) asserts in [`BufferEscrow::claim`],
/// and a value never claimed leaves its buffer in the escrow (the C++
/// equivalent leaks the buffer). The default value is the serialized form
/// of a default C++ `Fw::Buffer` (all zero) and is not claimable.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct EscrowedBuffer {
    token: u64,
    size: FwSizeType,
    context: u32,
    offset: FwSizeType,
    capacity: FwSizeType,
}

impl EscrowedBuffer {
    /// Deposit `buffer` in `escrow` and describe it.
    pub fn deposit(escrow: &BufferEscrow, buffer: Buffer) -> Self {
        let size = buffer.size() as FwSizeType;
        let context = buffer.context();
        let offset = buffer.offset() as FwSizeType;
        let capacity = buffer.capacity() as FwSizeType;
        Self {
            token: escrow.deposit(buffer),
            size,
            context,
            offset,
            capacity,
        }
    }

    /// Claim the buffer back from `escrow` (the escrow this value was
    /// deposited in). Asserts on a stale or already-claimed token, and if
    /// the claimed buffer does not match the descriptor.
    pub fn claim(&self, escrow: &BufferEscrow) -> Buffer {
        let buffer = escrow.claim(self.token);
        fw_assert!(
            buffer.size() as FwSizeType == self.size,
            buffer.size() as i32
        );
        fw_assert!(buffer.context() == self.context, buffer.context() as i32);
        fw_assert!(
            buffer.offset() as FwSizeType == self.offset,
            buffer.offset() as i32
        );
        fw_assert!(
            buffer.capacity() as FwSizeType == self.capacity,
            buffer.capacity() as i32
        );
        buffer
    }

    /// The escrow token (serialized in the C++ pointer slot).
    pub fn token(&self) -> u64 {
        self.token
    }

    /// The buffer's window size (C++ `getSize`).
    pub fn size(&self) -> usize {
        self.size as usize
    }

    /// The buffer's context word (C++ `getContext`).
    pub fn context(&self) -> u32 {
        self.context
    }

    /// The buffer's window offset (C++ `m_offset`).
    pub fn offset(&self) -> usize {
        self.offset as usize
    }

    /// The buffer's capacity (C++ `getCapacity`).
    pub fn capacity(&self) -> usize {
        self.capacity as usize
    }
}

impl Serialize for EscrowedBuffer {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let fields: [&dyn Serialize; 5] = [
            &self.token,
            &self.size,
            &self.context,
            &self.offset,
            &self.capacity,
        ];
        for field in fields {
            let status = field.serialize_to(buf, e);
            if !status.is_ok() {
                return status;
            }
        }
        SerializeStatus::Ok
    }

    fn serialized_size(&self) -> usize {
        Self::SERIALIZED_SIZE
    }
}

impl Deserialize for EscrowedBuffer {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let fields: [&mut dyn Deserialize; 5] = [
            &mut self.token,
            &mut self.size,
            &mut self.context,
            &mut self.offset,
            &mut self.capacity,
        ];
        for field in fields {
            let status = field.deserialize_from(buf, e);
            if !status.is_ok() {
                return status;
            }
        }
        SerializeStatus::Ok
    }
}

impl FppSized for EscrowedBuffer {
    /// C++ `Fw::Buffer::SERIALIZED_SIZE` on a 64-bit host: 36.
    const SERIALIZED_SIZE: usize =
        3 * size_of::<FwSizeType>() + size_of::<u32>() + size_of::<u64>();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::panic::{AssertUnwindSafe, catch_unwind};

    fn buffer_with_bytes(bytes: &[u8]) -> Buffer {
        let mut buffer = Buffer::allocate(bytes.len());
        buffer.data_mut().copy_from_slice(bytes);
        buffer
    }

    #[test]
    fn deposit_claim_round_trip() {
        let escrow = BufferEscrow::new();
        let token = escrow.deposit(buffer_with_bytes(&[1, 2, 3]));
        assert_eq!(escrow.len(), 1);
        let buffer = escrow.claim(token);
        assert_eq!(buffer.data(), &[1, 2, 3]);
        assert!(escrow.is_empty());
    }

    #[test]
    fn tokens_distinguish_multiple_buffers() {
        let escrow = BufferEscrow::with_capacity(4);
        let t1 = escrow.deposit(buffer_with_bytes(&[1]));
        let t2 = escrow.deposit(buffer_with_bytes(&[2]));
        let t3 = escrow.deposit(buffer_with_bytes(&[3]));
        assert_ne!(t1, t2);
        assert_ne!(t2, t3);
        // Claim out of order.
        assert_eq!(escrow.claim(t2).data(), &[2]);
        assert_eq!(escrow.claim(t1).data(), &[1]);
        assert_eq!(escrow.claim(t3).data(), &[3]);
    }

    #[test]
    fn freed_slot_reuse_gets_fresh_generation() {
        let escrow = BufferEscrow::new();
        let t1 = escrow.deposit(buffer_with_bytes(&[1]));
        let _ = escrow.claim(t1);
        let t2 = escrow.deposit(buffer_with_bytes(&[2]));
        // Same slot, different generation -> different token.
        assert_ne!(t1, t2);
        assert_eq!(t1 & 0xFFFF_FFFF, t2 & 0xFFFF_FFFF);
        assert_eq!(escrow.claim(t2).data(), &[2]);
    }

    #[test]
    fn stale_token_asserts() {
        let escrow = BufferEscrow::new();
        let t1 = escrow.deposit(buffer_with_bytes(&[1]));
        let _ = escrow.claim(t1);
        let result = catch_unwind(AssertUnwindSafe(|| {
            let _ = escrow.claim(t1); // double claim
        }));
        assert!(result.is_err());
    }

    #[test]
    fn out_of_range_token_asserts() {
        let escrow = BufferEscrow::new();
        let result = catch_unwind(AssertUnwindSafe(|| {
            let _ = escrow.claim(42);
        }));
        assert!(result.is_err());
    }

    #[test]
    fn escrowed_buffer_has_the_cpp_buffer_layout() {
        use fprime_fw::{LinearBuffer, SerBuf};
        let escrow = BufferEscrow::new();
        let mut buffer = Buffer::allocate(10);
        buffer.advance(2);
        buffer.set_size(5);
        buffer.set_context(0xA1B2_C3D4);
        let value = EscrowedBuffer::deposit(&escrow, buffer);
        assert_eq!(EscrowedBuffer::SERIALIZED_SIZE, 36);
        assert_eq!(value.size(), 5);
        let mut msg = LinearBuffer::<64>::new();
        assert!(msg.serialize(&value, Endianness::Big).is_ok());
        let mut expected = value.token().to_be_bytes().to_vec();
        expected.extend_from_slice(&5u64.to_be_bytes()); // size
        expected.extend_from_slice(&0xA1B2_C3D4u32.to_be_bytes()); // context
        expected.extend_from_slice(&2u64.to_be_bytes()); // offset
        expected.extend_from_slice(&10u64.to_be_bytes()); // capacity
        assert_eq!(msg.as_slice(), &expected[..]);
        let mut read_back = EscrowedBuffer::default();
        assert!(msg.deserialize(&mut read_back, Endianness::Big).is_ok());
        assert_eq!(read_back, value);
        let buffer = read_back.claim(&escrow);
        assert_eq!((buffer.offset(), buffer.size()), (2, 5));
        assert!(escrow.is_empty());
        // A second claim of the same token asserts.
        let result = catch_unwind(AssertUnwindSafe(|| {
            let _ = value.claim(&escrow);
        }));
        assert!(result.is_err());
    }

    #[test]
    fn default_escrowed_buffer_is_a_default_cpp_buffer() {
        use fprime_fw::{LinearBuffer, SerBuf};
        let mut msg = LinearBuffer::<64>::new();
        assert!(
            msg.serialize(&EscrowedBuffer::default(), Endianness::Big)
                .is_ok()
        );
        assert_eq!(msg.as_slice(), &[0u8; 36][..]);
    }

    #[test]
    fn token_serializes_as_eight_bytes_like_a_pointer() {
        // The token occupies the same 8-byte wire slot as the C++ pointer.
        use fprime_fw::{LinearBuffer, SerBuf};
        let escrow = BufferEscrow::new();
        let token = escrow.deposit(buffer_with_bytes(&[9]));
        let mut msg = LinearBuffer::<16>::new();
        let status = msg.serialize_u64_be(token);
        assert!(status.is_ok());
        assert_eq!(msg.get_size(), 8);
        let mut read_back = 0u64;
        let status = msg.deserialize_u64_be(&mut read_back);
        assert!(status.is_ok());
        assert_eq!(escrow.claim(read_back).data(), &[9]);
    }
}
