//! # fprime-spacewasm — the spacewasm embedding layer of `Svc::WasmSequencer`
//!
//! Upstream's `Svc::WasmSequencer` (C++) does not call NASA's
//! [`spacewasm`](https://crates.io/crates/spacewasm) interpreter directly: it
//! links `Svc/WasmSequencer/fprime_spacewasm`, a Rust static library made of
//! `spacewasm_c_api` (a C ABI over spacewasm) plus a small registry of
//! per-sequencer global page allocators. This crate is the Rust port of that
//! glue, as a *safe* Rust API, so that `fprime_svc::wasm_sequencer` keeps
//! `#![forbid(unsafe_code)]`:
//!
//! - [`Heap`] / [`HeapGuard`] — port of `fprime_spacewasm/src/lib.rs`: up to
//!   [`MAX_SEQUENCERS`] registered page pools behind spacewasm's
//!   process-wide `__spacewasm_alloc` / `__spacewasm_dealloc` symbols, one
//!   of them *current* at a time (`spacewasm_fprime_register_global_allocator`,
//!   `..._acquire_...`, `..._release_...`, `..._deregister_...`). The page
//!   bump allocator and its poison flag are C++ `WasmSequencer::globalAlloc`
//!   / `globalDealloc`, moved here because the memory they hand out must
//!   stay out of reach of safe code.
//! - [`GuestPool`] — the guest linear-memory bump allocator (C++
//!   `guestAlloc` / `guestRealloc` / `guestDealloc`) behind spacewasm's
//!   `WasmMemoryAllocator`, with the `MemoryGrowRejected` report forwarded
//!   to a [`GuestObserver`].
//! - [`WasmEngine`] / [`HostModules`] — the `spacewasm_c_api` calls the
//!   sequencer makes (`spacewasm_new`, `_load_module`, `_find_module`,
//!   `_find_export_func`, `_module_start`, `_check_func_signature`,
//!   `_find_global`, `_get_global`, `_set_global`, `_invoke`, `_run`,
//!   `_resume`, `_resume_value`, `_reset`, `_get_result`, `_mem_read`,
//!   `_mem_write`, `_host_new`, `_add_host_module`, `_add_host_function`),
//!   with spacewasm's error enums returned as [`Error`] instead of
//!   `spacewasm_status_t` codes (the component maps them onto its FPP
//!   `Status` enum, which carries those codes).
//!
//! ## Why this crate has `unsafe`
//!
//! It is the one crate of the workspace that is not
//! `forbid(unsafe_code)` (it is `deny(unsafe_code)`, with `#[allow]` on the
//! items below, each with a SAFETY comment). Embedding spacewasm needs:
//!
//! 1. the `__spacewasm_alloc` / `__spacewasm_dealloc` C-ABI symbols that
//!    spacewasm's `GlobalAllocator` links against (emitted by spacewasm's
//!    own `global_allocator!` macro) and an `unsafe impl spacewasm::Allocator`
//!    to back them — spacewasm has no safe way to supply its heap;
//! 2. raw page and guest-memory pools: spacewasm writes them through raw
//!    pointers, so they are raw allocations that safe code never sees;
//! 3. `InnerVec::from_raw_parts` for the streamed module chunks (spacewasm's
//!    only populated-chunk constructor);
//! 4. `unsafe impl Send` for [`WasmEngine`]: spacewasm's `Engine` is built
//!    on its own `Rc` (raw pointers, `!Send`), but a component's state must
//!    be `Send`. The impl is sound because the whole `Rc` graph moves
//!    together and this API never hands out an `Rc`, a `&Engine` or a
//!    `&mut Engine` that could leak one.
//!
//! spacewasm itself also contains `unsafe` internally (its allocator-backed
//! data structures, operand stack, linear memory and IR reader).
//!
//! ## Feature
//!
//! Everything is behind the `spacewasm` feature (off by default, enabled
//! by `fprime-svc/wasm`), so the default workspace build — which builds
//! every member — has no third-party dependency; without it the crate is
//! empty.
#![no_std]
#![cfg_attr(not(feature = "spacewasm"), forbid(unsafe_code))]
// The crate docs link the API; without the feature it does not exist.
#![cfg_attr(not(feature = "spacewasm"), allow(rustdoc::broken_intra_doc_links))]

#[cfg(feature = "spacewasm")]
extern crate alloc;

#[cfg(feature = "spacewasm")]
mod engine;
#[cfg(feature = "spacewasm")]
mod error;
#[cfg(feature = "spacewasm")]
mod guest;
#[cfg(feature = "spacewasm")]
mod heap;
#[cfg(feature = "spacewasm")]
mod sync;

#[cfg(feature = "spacewasm")]
pub use engine::{HostCallResult, HostModules, MAX_CONTROL_FRAMES, MAX_STACK_DEPTH, WasmEngine};
#[cfg(feature = "spacewasm")]
pub use error::Error;
#[cfg(feature = "spacewasm")]
pub use guest::{GrowRejected, GuestObserver, GuestPool, POOL_ALIGNMENT};
#[cfg(feature = "spacewasm")]
pub use heap::{Heap, HeapGuard, MAX_PAGES, MAX_SEQUENCERS, PAGE_ALIGNMENT, PAGE_SIZE};
/// The interpreter crate, re-exported so users name its types (`Value`,
/// `ValType`, `TrapReason`, the error enums, `CompilerOptions`, ...)
/// without depending on it directly.
#[cfg(feature = "spacewasm")]
pub use spacewasm;
