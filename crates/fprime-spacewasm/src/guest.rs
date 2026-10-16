//! Guest linear memory: the bump allocator behind every module's memory.
//!
//! Port of C++ `WasmSequencer::guestAlloc` / `guestRealloc` /
//! `guestDealloc` (and the `spacewasm_allocator_new` C adapter, here a
//! `spacewasm::WasmMemoryAllocator` impl). The pool is one raw allocation
//! made at configuration time and reused by every store; spacewasm reads
//! and writes it through the pointers this module hands out, and safe code
//! only ever sees offsets.

use alloc::boxed::Box;
use alloc::sync::Arc;
use core::alloc::Layout;
use core::ptr::NonNull;

use spacewasm::{AllocError, WasmMemoryAllocator};

use crate::sync::SpinLock;

/// Alignment of the pool's base address, and the largest alignment it
/// serves (C++ `SPACEWASM_MEMORY_ALIGNMENT`).
pub const POOL_ALIGNMENT: usize = 8;

/// Why a guest `memory.grow` was refused (FPP
/// `WasmSequencer.MemoryGrowFailReason`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GrowRejected {
    /// The memory is not the last allocation in the pool, so it cannot
    /// grow in place (`NOT_LAST_ALLOCATION`).
    NotLastAllocation,
    /// The grown size would not fit the pool (`INSUFFICIENT_POOL_SPACE`).
    InsufficientPoolSpace,
}

/// Receives the guest allocator's reports (C++ `guestRealloc` logs
/// `MemoryGrowRejected` itself).
pub trait GuestObserver: Send + Sync {
    /// A `memory.grow` to `requested_size` bytes was refused; the guest sees
    /// `memory.grow` return -1.
    fn memory_grow_rejected(&self, reason: GrowRejected, requested_size: u64);
}

#[derive(Debug, Default)]
struct State {
    /// C++ `m_guestPoolOffset`.
    offset: usize,
    /// Allocations handed to spacewasm and not yet freed. Not in C++: it
    /// lets [`GuestPool::reset`] refuse to recycle memory still in use.
    live: usize,
}

/// The guest memory pool (C++ `m_guestPool`, `m_config.guestMemorySize`)
/// with its bump allocator.
pub struct GuestPool {
    base: NonNull<u8>,
    size: usize,
    state: SpinLock<State>,
    observer: Box<dyn GuestObserver>,
}

// SAFETY: `base` points to memory this pool owns and never reads or writes
// itself; it only hands out disjoint, in-bounds regions of it (the bump
// offset only grows between resets, and a reset requires every region to
// be freed), and the bookkeeping is behind a lock. Sharing or sending the
// pool therefore shares no unsynchronized state.
#[allow(unsafe_code)]
unsafe impl Send for GuestPool {}
// SAFETY: see `Send`.
#[allow(unsafe_code)]
unsafe impl Sync for GuestPool {}

impl core::fmt::Debug for GuestPool {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("GuestPool")
            .field("size", &self.size)
            .field("offset", &self.offset())
            .finish()
    }
}

impl GuestPool {
    fn layout(size: usize) -> Layout {
        Layout::from_size_align(size, POOL_ALIGNMENT)
            .unwrap_or_else(|_| panic!("guest pool too large"))
    }

    /// Allocate a zeroed pool of `size` bytes (C++ `configure`'s guest
    /// memory pool).
    pub fn new(size: usize, observer: Box<dyn GuestObserver>) -> Arc<GuestPool> {
        let base = if size == 0 {
            NonNull::dangling()
        } else {
            let layout = Self::layout(size);
            // SAFETY: `layout` has a non-zero size.
            #[allow(unsafe_code)]
            let ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
            NonNull::new(ptr).unwrap_or_else(|| alloc::alloc::handle_alloc_error(layout))
        };
        Arc::new(GuestPool {
            base,
            size,
            state: SpinLock::new(State::default()),
            observer,
        })
    }

    /// The pool size in bytes.
    pub fn size(&self) -> usize {
        self.size
    }

    /// The current bump offset (C++ `m_guestPoolOffset`).
    pub fn offset(&self) -> usize {
        self.state.lock().offset
    }

    /// Recycle the whole pool (C++ `destroyStore`'s
    /// `m_guestPoolOffset = 0`). Returns `false`, changing nothing, while a
    /// region is still in use by a live store.
    pub fn reset(&self) -> bool {
        let mut state = self.state.lock();
        if state.live != 0 {
            return false;
        }
        state.offset = 0;
        true
    }

    /// C++ `guestAlloc`: reserve `size` bytes aligned to `align`, returning
    /// the region's offset. `None` for an empty request, an alignment above
    /// [`POOL_ALIGNMENT`] or a request that does not fit.
    ///
    /// The offset is bookkeeping only; nothing is handed out. Exposed so
    /// tests can drive the allocator directly, as upstream's white-box
    /// tests do.
    pub fn alloc_offset(&self, size: usize, align: usize) -> Option<usize> {
        self.alloc_impl(size, align, false)
    }

    fn alloc_impl(&self, size: usize, align: usize, live: bool) -> Option<usize> {
        if size == 0 {
            return None;
        }
        // Reject any request that cannot possibly fit up front.
        if size > self.size || align > POOL_ALIGNMENT {
            return None;
        }
        let mut state = self.state.lock();
        // Round the current offset up to the requested alignment.
        let a = align.max(1);
        let start = (state.offset + a - 1) & !(a - 1);
        // Compare against the pre-subtracted bound so `start + size`
        // cannot overflow (`size <= self.size` above).
        if start > self.size - size {
            return None;
        }
        state.offset = start + size;
        if live {
            state.live += 1;
        }
        Some(start)
    }

    /// C++ `guestRealloc` (a `memory.grow`): grow the region at `offset`
    /// from `old_size` to `new_size` bytes in place. That works only for
    /// the last region of the pool, and only if the grown region fits;
    /// otherwise the refusal is reported to the observer and `None`
    /// returned. Bookkeeping only, like [`Self::alloc_offset`].
    ///
    /// Deviation (unreachable from spacewasm, which only grows): a request
    /// to shrink the last region succeeds without giving the tail back,
    /// where C++ moves the offset down — memory a live store still uses
    /// must never be handed out again.
    pub fn realloc_offset(
        &self,
        offset: usize,
        old_size: usize,
        new_size: usize,
        align: usize,
    ) -> Option<usize> {
        let _ = align;
        let mut state = self.state.lock();
        // Check 1: the region is the last allocation of the pool.
        if offset.checked_add(old_size) == Some(state.offset) {
            // Check 2: the grown region fits the pool. Sizes keep their
            // full width: a 4 GiB grow must not alias to a small one.
            if offset
                .checked_add(new_size)
                .is_some_and(|end| end <= self.size)
            {
                if new_size >= old_size {
                    state.offset = offset + new_size;
                }
                return Some(offset);
            }
            drop(state);
            self.observer
                .memory_grow_rejected(GrowRejected::InsufficientPoolSpace, new_size as u64);
            return None;
        }
        drop(state);
        self.observer
            .memory_grow_rejected(GrowRejected::NotLastAllocation, new_size as u64);
        None
    }

    /// The address of `offset` (in bounds by the callers).
    fn ptr(&self, offset: usize) -> NonNull<u8> {
        // `offset <= size`, so this stays inside (or one past) the
        // allocation; `wrapping_add` keeps it safe code.
        NonNull::new(self.base.as_ptr().wrapping_add(offset))
            .unwrap_or_else(|| panic!("null guest pointer"))
    }

    /// The offset of `ptr`, which must lie in the pool (C++ asserts
    /// `ptr - m_guestPool >= 0`).
    fn offset_of(&self, ptr: NonNull<u8>) -> usize {
        let offset = (ptr.as_ptr() as usize).wrapping_sub(self.base.as_ptr() as usize);
        assert!(offset <= self.size, "pointer outside the guest pool");
        offset
    }
}

impl Drop for GuestPool {
    fn drop(&mut self) {
        if self.size > 0 {
            // SAFETY: `base` came from `alloc_zeroed` with this layout and
            // is freed once, here. Every region handed out is owned by a
            // store memory that holds an `Arc` of this pool, so none is live.
            #[allow(unsafe_code)]
            unsafe {
                alloc::alloc::dealloc(self.base.as_ptr(), Self::layout(self.size));
            }
        }
    }
}

/// The `WasmMemoryAllocator` a store's modules allocate their linear memory
/// from (C++ `m_guest_allocator`, the `CAllocator` of
/// `spacewasm_allocator_new`).
pub(crate) struct GuestAllocator {
    pub(crate) pool: Arc<GuestPool>,
}

impl WasmMemoryAllocator for GuestAllocator {
    fn allocate(&self, layout: Layout) -> Result<NonNull<u8>, AllocError> {
        // A NULL result is `AllocationFailed` (the C adapter's mapping).
        self.pool
            .alloc_impl(layout.size(), layout.align(), true)
            .map(|offset| self.pool.ptr(offset))
            .ok_or(AllocError::AllocationFailed)
    }

    fn reallocate(
        &self,
        ptr: NonNull<u8>,
        old_layout: Layout,
        layout: Layout,
    ) -> Result<NonNull<u8>, AllocError> {
        let offset = self.pool.offset_of(ptr);
        self.pool
            .realloc_offset(offset, old_layout.size(), layout.size(), layout.align())
            .map(|offset| self.pool.ptr(offset))
            .ok_or(AllocError::AllocationFailed)
    }

    fn deallocate(&self, _ptr: NonNull<u8>, _layout: Layout) {
        // Bump allocator: individual frees are no-ops (C++ `guestDealloc`);
        // the pool is recycled as a whole once every region is back.
        let mut state = self.pool.state.lock();
        state.live = state.live.saturating_sub(1);
    }
}
