//! `BufferEscrow` — safe replacement for the C++ practice of serializing a
//! raw `Fw::Buffer` (pointer + size) into an async queue message.
//!
//! Rust `Buffer` is owned and cannot be flattened into a byte message, so a
//! component with async buffer-carrying ports owns an escrow: the input
//! adapter deposits the `Buffer` and serializes the returned `u64` token
//! where C++ would serialize the pointer (same 8-byte wire width, so the
//! message layout is unchanged); the dispatch side claims the token back on
//! the component thread.

use fprime_fw::{Buffer, fw_assert};
use std::sync::{Mutex, PoisonError};

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
        let mut state = self.state.lock().unwrap_or_else(PoisonError::into_inner);
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
        let mut state = self.state.lock().unwrap_or_else(PoisonError::into_inner);
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
        let state = self.state.lock().unwrap_or_else(PoisonError::into_inner);
        state.slots.iter().filter(|s| s.buffer.is_some()).count()
    }

    /// True when no buffers are held.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
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
