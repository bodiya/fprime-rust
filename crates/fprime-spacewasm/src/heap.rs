//! The interpreter heap: spacewasm's process-wide page allocator, backed by
//! per-sequencer page pools.
//!
//! Port of `Svc/WasmSequencer/fprime_spacewasm/src/lib.rs` (the slot
//! registry, `CURRENT_ALLOCATOR`, register / acquire / release /
//! deregister) and of C++ `WasmSequencer::globalAlloc` /
//! `globalDealloc` (the page bump allocator with its poison flag).
//!
//! spacewasm allocates everything but guest linear memory — the store, the
//! modules, the IR code pages, the operand stack — through one global
//! allocator, the `__spacewasm_alloc` / `__spacewasm_dealloc` symbols.
//! Those forward to the *current* slot: each slot is a
//! `spacewasm::PageAllocator` (which carves small allocations out of whole
//! pages) over a [`PagePool`] of [`PAGE_SIZE`]-byte pages. A sequencer
//! registers a slot once ([`Heap::register`]) and makes it current around
//! every store operation ([`Heap::acquire`], released when the
//! [`HeapGuard`] drops).

use core::alloc::Layout;
use core::cell::Cell;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicUsize, Ordering};

use spacewasm::{AllocError, Allocator, PageAllocator};

use crate::error::Error;
use crate::sync::{SpinGuard, SpinLock};

/// Size in bytes of each heap page (`WASM_SEQ_SPACEWASM_PAGE_SIZE`).
pub const PAGE_SIZE: usize = fprime_config::wasm_sequencer::SPACEWASM_PAGE_SIZE as usize;
/// Maximum number of pages of one heap (`WASM_SEQ_SPACEWASM_MAX_PAGES`):
/// the capacity of each slot's `PageAllocator`.
pub const MAX_PAGES: usize = fprime_config::wasm_sequencer::SPACEWASM_MAX_PAGES as usize;
/// Maximum number of registered heaps (`WASM_SEQ_MAX_SEQUENCERS`).
pub const MAX_SEQUENCERS: usize = fprime_config::wasm_sequencer::MAX_SEQUENCERS;
/// Alignment of every page (C++ `SPACEWASM_MEMORY_ALIGNMENT`; spacewasm's
/// `PageAllocator` requests pages aligned to 8).
pub const PAGE_ALIGNMENT: usize = 8;

/// `CURRENT_ALLOCATOR`: the slot `__spacewasm_alloc` serves, or `NONE`.
static CURRENT: AtomicUsize = AtomicUsize::new(NONE);
const NONE: usize = usize::MAX;

/// A pool of `count` pages, each [`PAGE_SIZE`] bytes and
/// [`PAGE_ALIGNMENT`]-aligned, in one raw allocation that only spacewasm
/// reads and writes (through the pointers [`SlotBackend`] hands out).
struct PagePool {
    base: NonNull<u8>,
    count: usize,
    /// C++ `m_heapPagesUsed`: pages handed out, a bump index.
    used: Cell<usize>,
    /// C++ `m_heapPoisoned`: a page was freed. spacewasm frees pages in
    /// forward-scan rather than LIFO order, so after any free the bump
    /// index may point at a live page; further allocations assert until
    /// every page is back.
    poisoned: Cell<bool>,
}

impl PagePool {
    fn layout(count: usize) -> Layout {
        // `count` is bounded by the caller (`MAX_PAGES`), so this cannot
        // overflow; a failure is a programming error.
        Layout::from_size_align(count * PAGE_SIZE, PAGE_ALIGNMENT)
            .unwrap_or_else(|_| panic!("page pool too large"))
    }

    fn new(count: usize) -> Self {
        let base = if count == 0 {
            NonNull::dangling()
        } else {
            let layout = Self::layout(count);
            // SAFETY: `layout` has a non-zero size (`count > 0`).
            #[allow(unsafe_code)]
            let ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
            NonNull::new(ptr).unwrap_or_else(|| alloc::alloc::handle_alloc_error(layout))
        };
        Self {
            base,
            count,
            used: Cell::new(0),
            poisoned: Cell::new(false),
        }
    }

    fn page(&self, index: usize) -> *mut u8 {
        // In bounds: `index < count`, so the page lies inside the
        // allocation (`wrapping_add` keeps this safe code).
        self.base.as_ptr().wrapping_add(index * PAGE_SIZE)
    }

    fn index_of(&self, ptr: *mut u8) -> Option<usize> {
        (0..self.count).find(|&i| self.page(i) == ptr)
    }

    /// C++ `globalAlloc`: the next page, or `None` when all are out.
    fn alloc(&self, size: usize, align: usize) -> Option<*mut u8> {
        // C++ parity (FW_ASSERTs): spacewasm's PageAllocator only ever
        // requests whole pages, aligned no more than the pool's alignment,
        // and a poisoned pool must not hand out pages.
        assert!(size == PAGE_SIZE, "page request of {size} bytes");
        assert!(align <= PAGE_ALIGNMENT, "page alignment {align}");
        assert!(!self.poisoned.get(), "page allocation after a page free");
        let used = self.used.get();
        if used < self.count {
            self.used.set(used + 1);
            Some(self.page(used))
        } else {
            None
        }
    }

    /// C++ `globalDealloc`.
    fn dealloc(&self, ptr: *mut u8) {
        if ptr.is_null() {
            return;
        }
        // C++ parity: the page must be ours, and one must be out.
        assert!(
            self.index_of(ptr).is_some(),
            "freed page is not in the pool"
        );
        let used = self.used.get();
        assert!(used > 0, "page freed with no page in use");
        self.used.set(used - 1);
        self.poisoned.set(true);
    }
}

impl Drop for PagePool {
    fn drop(&mut self) {
        if self.count > 0 {
            // SAFETY: `base` came from `alloc_zeroed` with this layout and
            // is freed once, here. The pool is only dropped when no page is
            // in use (`Heap::drop`), so spacewasm holds no pointer into it.
            #[allow(unsafe_code)]
            unsafe {
                alloc::alloc::dealloc(self.base.as_ptr(), Self::layout(self.count));
            }
        }
    }
}

/// The page source of one slot's `PageAllocator` (C++
/// `FprimeWasmGlobalAllocator`, whose callbacks are `globalAlloc` /
/// `globalDealloc`).
struct SlotBackend {
    pool: Option<PagePool>,
}

impl SlotBackend {
    const fn new() -> Self {
        Self { pool: None }
    }

    fn pool(&self) -> &PagePool {
        match &self.pool {
            Some(pool) => pool,
            // Upstream panics the same way ("allocator not registered").
            None => panic!("allocator not registered"),
        }
    }
}

// SAFETY: `alloc` returns either an error or a pointer to a whole, unused
// page of the pool — `PAGE_SIZE` bytes, `PAGE_ALIGNMENT`-aligned — and the
// pool's bump index never hands a page out twice (after any free the pool
// is poisoned and refuses to allocate until every page is back, see
// `Heap::clear_poison`); `dealloc` only does bookkeeping. The pool memory
// outlives every page handed out (`Heap::drop` leaks the pool rather than
// freeing it while pages are in use).
#[allow(unsafe_code)]
unsafe impl Allocator for SlotBackend {
    unsafe fn alloc(&self, layout: Layout) -> Result<*mut u8, AllocError> {
        // A missing page is an out-of-memory condition (upstream maps a
        // NULL page to `AllocError::OutOfMemory`).
        self.pool()
            .alloc(layout.size(), layout.align())
            .ok_or(AllocError::OutOfMemory)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, _layout: Layout) {
        self.pool().dealloc(ptr);
    }
}

/// One registry slot.
struct SlotState {
    alloc: PageAllocator<SlotBackend, MAX_PAGES>,
    registered: bool,
}

// SAFETY: the raw pointers inside (the pool base, the `PageAllocator`'s page
// records) all point into the pool this same value owns, and every access to
// the value goes through the slot's `SpinLock`, so moving it between threads
// with the lock is sound.
#[allow(unsafe_code)]
unsafe impl Send for SlotState {}

impl SlotState {
    const fn new() -> Self {
        Self {
            alloc: PageAllocator::new(SlotBackend::new(), PAGE_SIZE),
            registered: false,
        }
    }

    fn pool(&self) -> Option<(usize, usize, bool)> {
        self.alloc.with_inner(|b| {
            b.pool
                .as_ref()
                .map(|p| (p.used.get(), p.count, p.poisoned.get()))
        })
    }
}

/// `ALLOCATORS`.
static SLOTS: [SpinLock<SlotState>; MAX_SEQUENCERS] =
    [const { SpinLock::new(SlotState::new()) }; MAX_SEQUENCERS];

fn slot(index: usize) -> SpinGuard<'static, SlotState> {
    SLOTS[index].lock()
}

/// `SwapableAllocator`: spacewasm's global allocator, forwarding to the
/// current slot.
struct Swapable;

impl Swapable {
    fn current() -> SpinGuard<'static, SlotState> {
        let index = CURRENT.load(Ordering::Acquire);
        if index >= MAX_SEQUENCERS {
            // Upstream: `panic!("no allocator set")`, which its panic hook
            // turns into an FSW assertion.
            panic!("no allocator set");
        }
        slot(index)
    }
}

// SAFETY: forwards to the current slot's `PageAllocator` while holding that
// slot's lock (so the `PageAllocator`'s interior state is never accessed
// concurrently), passing the caller's contract through unchanged.
#[allow(unsafe_code)]
unsafe impl Allocator for Swapable {
    unsafe fn alloc(&self, layout: Layout) -> Result<*mut u8, AllocError> {
        let state = Self::current();
        // SAFETY: the caller guarantees a non-zero layout.
        unsafe { state.alloc.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        let state = Self::current();
        // SAFETY: the caller guarantees `ptr` came from `alloc` with
        // `layout`; the slot serving it is current (a store is only touched
        // with its heap acquired, see `WasmEngine::drop`).
        unsafe { state.alloc.dealloc(ptr, layout) }
    }
}

/// The `__spacewasm_alloc` / `__spacewasm_dealloc` symbols, emitted by
/// spacewasm's own macro. They are sound under the macro's documented
/// contract — no concurrent or re-entrant entry — because `Swapable` is a
/// stateless unit and serializes on the slot lock.
#[allow(unsafe_code)]
mod symbols {
    use super::Swapable;
    spacewasm::global_allocator!(Swapable, Swapable);
}

/// A registered interpreter heap: one slot of the registry with its page
/// pool (C++ `m_heapPages` / `m_config.heapPages`, registered with
/// `spacewasm_fprime_register_global_allocator`).
///
/// Dropping it releases the slot (`spacewasm_fprime_deregister_global_allocator`);
/// if pages are still in use — a store outlived its heap — the slot and its
/// pool are leaked instead, so no live pointer dangles.
#[derive(Debug)]
pub struct Heap {
    slot: usize,
}

impl Heap {
    /// Register a heap of `pages` pages in a free slot. `Error::Capacity`
    /// when all [`MAX_SEQUENCERS`] slots are taken (upstream
    /// `SPACEWASM_ERR_CAPACITY`).
    ///
    /// # Panics
    ///
    /// When `pages` exceeds [`MAX_PAGES`] (the component asserts the C++
    /// bound first).
    pub fn register(pages: usize) -> Result<Heap, Error> {
        assert!(pages <= MAX_PAGES, "{pages} heap pages");
        for (index, lock) in SLOTS.iter().enumerate() {
            let mut state = lock.lock();
            if !state.registered {
                state.registered = true;
                state
                    .alloc
                    .with_inner(|b| b.pool = Some(PagePool::new(pages)));
                return Ok(Heap { slot: index });
            }
        }
        Err(Error::Capacity)
    }

    /// Make this heap the one spacewasm allocates from
    /// (`spacewasm_fprime_acquire_global_allocator`) until the guard drops.
    /// `Error::WrongState` when another heap is current.
    pub fn acquire(&self) -> Result<HeapGuard<'_>, Error> {
        CURRENT
            .compare_exchange(NONE, self.slot, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| HeapGuard { heap: self })
            .map_err(|_| Error::WrongState)
    }

    fn pool_state(&self) -> (usize, usize, bool) {
        slot(self.slot)
            .pool()
            .unwrap_or_else(|| panic!("heap without a pool"))
    }

    /// Pages currently handed out (C++ `m_heapPagesUsed`).
    pub fn pages_used(&self) -> usize {
        self.pool_state().0
    }

    /// Pages in the pool (C++ `m_config.heapPages`).
    pub fn page_count(&self) -> usize {
        self.pool_state().1
    }

    /// Whether a page has been freed since the pool was last fully
    /// returned (C++ `m_heapPoisoned`).
    pub fn is_poisoned(&self) -> bool {
        self.pool_state().2
    }

    /// Clear the poison flag (C++ `destroyStore`'s
    /// `m_heapPoisoned = false`). Only possible once every page is back:
    /// returns `false`, changing nothing, while pages are in use.
    pub fn clear_poison(&self) -> bool {
        let state = slot(self.slot);
        state.alloc.with_inner(|b| match &b.pool {
            Some(pool) if pool.used.get() == 0 => {
                pool.poisoned.set(false);
                true
            }
            _ => false,
        })
    }

    /// Test hook for upstream's `pokePageFreeThenReAlloc` death test: free
    /// the first page through the page callbacks (bypassing the
    /// `PageAllocator`), then request a new one, which trips the poison
    /// assertion. Panics; never returns normally when a page is in use.
    #[doc(hidden)]
    pub fn poke_page_free_then_realloc(&self) {
        let state = slot(self.slot);
        state.alloc.with_inner(|b| {
            let pool = b.pool();
            assert!(pool.used.get() > 0, "no page in use");
            pool.dealloc(pool.page(0));
            let _ = pool.alloc(PAGE_SIZE, PAGE_ALIGNMENT);
        });
    }
}

impl Drop for Heap {
    fn drop(&mut self) {
        // Relinquish the current selection if it is ours (upstream
        // deregister does the same).
        let _ = CURRENT.compare_exchange(self.slot, NONE, Ordering::AcqRel, Ordering::Acquire);
        let mut state = slot(self.slot);
        let in_use = state.pool().is_some_and(|(used, _, _)| used > 0);
        if !in_use {
            state.alloc.with_inner(|b| b.pool = None);
            state.registered = false;
        }
    }
}

/// Proof that a [`Heap`] is current; releases it on drop
/// (`spacewasm_fprime_release_global_allocator`).
#[derive(Debug)]
pub struct HeapGuard<'a> {
    heap: &'a Heap,
}

impl HeapGuard<'_> {
    pub(crate) fn slot(&self) -> usize {
        self.heap.slot
    }
}

impl Drop for HeapGuard<'_> {
    fn drop(&mut self) {
        let _ = CURRENT.compare_exchange(self.heap.slot, NONE, Ordering::AcqRel, Ordering::Acquire);
    }
}

/// Run `f` with heap slot `index` current: reuse it if it already is,
/// otherwise wait for the selection to be free, take it, and give it back
/// afterwards. Used to drop a store that is not dropped under its guard.
pub(crate) fn with_current(index: usize, f: impl FnOnce()) {
    loop {
        match CURRENT.compare_exchange(NONE, index, Ordering::AcqRel, Ordering::Acquire) {
            Ok(_) => {
                f();
                CURRENT.store(NONE, Ordering::Release);
                return;
            }
            Err(current) if current == index => {
                f();
                return;
            }
            Err(_) => core::hint::spin_loop(),
        }
    }
}
