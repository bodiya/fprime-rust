//! # StaticMemory — port of `Svc::StaticMemory` (passive, guarded)
//!
//! C++ sources: `Svc/StaticMemory/StaticMemoryComponentImpl.{cpp,hpp}`,
//! `Svc/StaticMemory/StaticMemory.fpp`, `default/config/StaticMemoryConfig.hpp`,
//! `Svc/StaticMemory/docs/sdd.md`.
//!
//! A drop-in replacement for [`BufferManager`](crate::buffer_manager) for
//! allocations of known size and usage pattern: each `bufferAllocate` /
//! `bufferDeallocate` port index owns ONE fixed segment of
//! [`ALLOCATION_SIZE`] bytes ([`STATIC_MEMORY_ALLOCATIONS`] segments in
//! all). Allocating hands out the whole segment — the buffer's size is the
//! segment size, whatever was asked for — and a segment must come back on
//! the same index before it can be allocated again.
//!
//! Every misuse is a topology/programmer error and asserts (C++ parity):
//! an out-of-range index, asking for more than [`ALLOCATION_SIZE`],
//! allocating an index that is still out, deallocating one that is not,
//! and returning memory that is not the index's segment.
//!
//! ## Ownership mapping (C++ descriptor to owned `Fw::Buffer`)
//!
//! The C++ component owns `U8 m_static_memory[N][SIZE]` and hands out
//! non-owning `Fw::Buffer` descriptors into it; `bufferDeallocate` only
//! clears the `m_allocated` flag after checking that the descriptor's
//! `[data, data + size)` window lies inside the index's segment. Here a
//! [`Buffer`] owns its storage, so each index owns a
//! [`BufferStorage`] allocated once at construction (no steady-state
//! allocation) which MOVES into the buffer on `bufferAllocate` and moves
//! back on `bufferDeallocate` — the BufferManager convention. The C++
//! range check becomes an identity check: the returned buffer must carry
//! the index's own storage (same base address, same capacity); its window
//! is then inside the segment by construction, so users may still
//! `advance` / `set_size` the buffer before returning it, as upstream's
//! test does. The one behavior C++ allows that this cannot express is
//! deallocating with a *different* descriptor that merely points into the
//! segment: the storage itself has to come back.

use alloc::vec;
use fprime_comp::{BufferGetPort, BufferSendPort, PassiveBase, input_port_adapter};
use fprime_config::{FwIndexType, FwSizeType};
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::{Buffer, BufferStorage, fw_assert};

pub use fprime_config::STATIC_MEMORY_ALLOCATIONS;
pub use fprime_config::static_memory::ALLOCATION_SIZE;

/// One per-index memory segment.
struct Segment {
    /// The segment's storage; `None` while it is allocated (on loan).
    storage: Option<BufferStorage>,
    /// The storage's base address, fixed at construction: the identity the
    /// C++ data-pointer range checks reduce to.
    address: usize,
    /// C++ `m_allocated[i]`.
    allocated: bool,
}

/// `Svc::StaticMemory` (C++ `StaticMemoryComponentImpl`).
pub struct StaticMemory {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// The segments; the mutex is the guarded-port mutex.
    segments: Mutex<[Segment; STATIC_MEMORY_ALLOCATIONS]>,
}

impl StaticMemory {
    /// Construct the component: allocates every segment once, all free.
    pub fn new(name: &str) -> Arc<Self> {
        let segments = core::array::from_fn(|_| {
            let storage: BufferStorage = vec![0u8; ALLOCATION_SIZE].into_boxed_slice();
            Segment {
                address: storage.as_ptr() as usize,
                storage: Some(storage),
                allocated: false,
            }
        });
        Arc::new(Self {
            base: PassiveBase::new(name),
            segments: Mutex::new(segments),
        })
    }

    /// True while the segment of `port_num` is allocated (C++
    /// `m_allocated[portNum]`, which upstream's tester reads as a friend).
    pub fn is_allocated(&self, port_num: FwIndexType) -> bool {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < STATIC_MEMORY_ALLOCATIONS,
            port_num
        );
        self.segments.lock()[port_num as usize].allocated
    }

    /// `bufferAllocate_handler` (guarded): hand out the whole segment of
    /// `port_num`.
    fn buffer_allocate_handler(&self, port_num: FwIndexType, size: FwSizeType) -> Buffer {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < STATIC_MEMORY_ALLOCATIONS,
            port_num
        );
        // It is a topology error to ask for too much from this component.
        fw_assert!(size <= ALLOCATION_SIZE as FwSizeType, size as i32);
        let mut segments = self.segments.lock();
        let segment = &mut segments[port_num as usize];
        // It is also an error to allocate again before returning.
        fw_assert!(!segment.allocated, port_num);
        segment.allocated = true;
        // Invariant: a free segment holds its storage.
        let storage = segment.storage.take();
        fw_assert!(storage.is_some(), port_num);
        // C++ `Fw::Buffer(m_static_memory[portNum], sizeof(m_static_memory[0]))`:
        // the full segment, default (NO_CONTEXT) context.
        Buffer::from_storage(storage.unwrap_or_default(), Buffer::NO_CONTEXT)
    }

    /// `bufferDeallocate_handler` (guarded): take the segment of `port_num`
    /// back.
    fn buffer_deallocate_handler(&self, port_num: FwIndexType, fw_buffer: Buffer) {
        fw_assert!(
            port_num >= 0 && (port_num as usize) < STATIC_MEMORY_ALLOCATIONS,
            port_num
        );
        let mut segments = self.segments.lock();
        let segment = &mut segments[port_num as usize];
        // It is also an error to deallocate before allocating.
        fw_assert!(segment.allocated, port_num);
        // C++ checks that [data, data + size) lies inside the segment. The
        // owned buffer's window always lies inside its own storage, so the
        // check is that the storage IS this segment's: same base address
        // (window address minus its offset) and same capacity.
        let base_address = fw_buffer.data().as_ptr() as usize - fw_buffer.offset();
        fw_assert!(base_address == segment.address, port_num);
        fw_assert!(
            fw_buffer.capacity() == ALLOCATION_SIZE,
            fw_buffer.size() as i32,
            ALLOCATION_SIZE as i32
        );
        segment.storage = Some(fw_buffer.into_storage());
        segment.allocated = false;
    }
}

input_port_adapter! {
    /// `bufferAllocate` — GUARDED `Fw.BufferGet` input, one per segment.
    component: StaticMemory;
    adapter: BufferAllocateAdapter;
    port: BufferGetPort;
    input: pub buffer_allocate;
    handler: buffer_allocate_handler;
    returns: Buffer;
    args { val size: FwSizeType }
}

input_port_adapter! {
    /// `bufferDeallocate` — GUARDED `Fw.BufferSend` input, one per segment.
    component: StaticMemory;
    adapter: BufferDeallocateAdapter;
    port: BufferSendPort;
    input: pub buffer_deallocate;
    handler: buffer_deallocate_handler;
    args { val buffer: Buffer }
}

#[cfg(test)]
mod tests {
    //! Port of `Svc/StaticMemory/test/ut` (`test_allocate`, requirements
    //! STAMEM-001/002) plus every assert path.

    use super::*;
    use std::vec::Vec;

    fn allocate(comp: &Arc<StaticMemory>, port_num: FwIndexType, size: FwSizeType) -> Buffer {
        let port = comp.buffer_allocate(port_num);
        port.target.invoke(port.port_num, size)
    }

    fn deallocate(comp: &Arc<StaticMemory>, port_num: FwIndexType, buffer: Buffer) {
        let port = comp.buffer_deallocate(port_num);
        port.target.invoke(port.port_num, buffer);
    }

    #[test]
    fn allocate_and_deallocate_every_segment() {
        let comp = StaticMemory::new("StaticMemory");
        let mut allocations = Vec::new();
        let mut addresses = Vec::new();
        for i in 0..STATIC_MEMORY_ALLOCATIONS {
            // Upstream picks a random size in [1, ALLOCATION_SIZE]; sweep
            // both ends and the middle instead.
            let size = [1, ALLOCATION_SIZE / 2, ALLOCATION_SIZE, 7][i % 4] as FwSizeType;
            let buffer = allocate(&comp, i as FwIndexType, size);
            assert!(
                comp.is_allocated(i as FwIndexType),
                "did not mark buffer as allocated"
            );
            assert!(
                buffer.size() as FwSizeType >= size,
                "did not allocate enough"
            );
            // The whole segment, whatever was asked for.
            assert_eq!(buffer.size(), ALLOCATION_SIZE);
            assert_eq!(buffer.capacity(), ALLOCATION_SIZE);
            assert_eq!(buffer.context(), Buffer::NO_CONTEXT);
            addresses.push(buffer.data().as_ptr() as usize);
            allocations.push(buffer);
        }
        // One distinct segment per index.
        let mut unique = addresses.clone();
        unique.sort_unstable();
        unique.dedup();
        assert_eq!(unique.len(), STATIC_MEMORY_ALLOCATIONS);

        for (i, mut buffer) in allocations.into_iter().enumerate() {
            // The user may move the window before returning it.
            buffer.advance(1);
            buffer.set_size(0);
            deallocate(&comp, i as FwIndexType, buffer);
            assert!(!comp.is_allocated(i as FwIndexType));
        }

        // Each index hands out the same segment again.
        for (i, &address) in addresses.iter().enumerate() {
            let buffer = allocate(&comp, i as FwIndexType, 1);
            assert_eq!(buffer.data().as_ptr() as usize, address);
            deallocate(&comp, i as FwIndexType, buffer);
        }
    }

    #[test]
    fn written_data_survives_the_round_trip() {
        let comp = StaticMemory::new("StaticMemory");
        let mut buffer = allocate(&comp, 2, 16);
        buffer.data_mut()[..4].copy_from_slice(&[1, 2, 3, 4]);
        deallocate(&comp, 2, buffer);
        let buffer = allocate(&comp, 2, 16);
        assert_eq!(&buffer.data()[..4], &[1, 2, 3, 4]);
    }

    #[test]
    #[should_panic]
    fn oversized_request_asserts() {
        let comp = StaticMemory::new("StaticMemory");
        let _ = allocate(&comp, 0, ALLOCATION_SIZE as FwSizeType + 1);
    }

    #[test]
    #[should_panic]
    fn double_allocation_asserts() {
        let comp = StaticMemory::new("StaticMemory");
        let _first = allocate(&comp, 1, 1);
        let _ = allocate(&comp, 1, 1);
    }

    #[test]
    #[should_panic]
    fn deallocation_before_allocation_asserts() {
        let comp = StaticMemory::new("StaticMemory");
        deallocate(&comp, 0, Buffer::allocate(ALLOCATION_SIZE));
    }

    #[test]
    #[should_panic]
    fn returning_another_segment_asserts() {
        let comp = StaticMemory::new("StaticMemory");
        let _zero = allocate(&comp, 0, 1);
        let one = allocate(&comp, 1, 1);
        // Index 0 is allocated, but this is index 1's segment.
        deallocate(&comp, 0, one);
    }

    #[test]
    #[should_panic]
    fn returning_foreign_storage_asserts() {
        let comp = StaticMemory::new("StaticMemory");
        let _zero = allocate(&comp, 0, 1);
        deallocate(&comp, 0, Buffer::allocate(ALLOCATION_SIZE));
    }

    #[test]
    #[should_panic]
    fn out_of_range_index_asserts() {
        let comp = StaticMemory::new("StaticMemory");
        let _ = allocate(&comp, STATIC_MEMORY_ALLOCATIONS as FwIndexType, 1);
    }
}
