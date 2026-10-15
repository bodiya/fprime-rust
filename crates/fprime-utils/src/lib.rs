//! fprime-utils: port of the F Prime `Utils/` support libraries.
//!
//! - [`hash`] — `Utils::Hash` CRC32 (IEEE 802.3) and `HashBuffer`
//! - [`crc_checker`] — `Utils::CRCChecker` file checksum sidecars
//! - [`validate_file`] — `Os::ValidateFile`: validate a file against its
//!   `.CRC32` hash file, or create one (an `Os` facility that lives here
//!   because it is built on [`hash`]; see the module docs for the placement
//!   and for its sidecar encoding next to [`crc_checker`]'s)
//! - [`types`] — `Types::CircularBuffer` byte ring and `Types::Queue`
//!   fixed-message FIFO/LIFO
//! - [`rate_limiter`] — `Utils::RateLimiter`
//! - [`token_bucket`] — `Utils::TokenBucket`
//!
//! Analyses: `docs/cpp-analysis/utils-misc.md` and the Utils::Hash section
//! of `docs/cpp-analysis/svc-comms.md`; `Os::ValidateFile` in
//! `docs/cpp-analysis/os.md`. None of these types are thread safe
//! (C++ parity); callers wrap them in concurrency constructs.
//!
//! # `no_std`
//!
//! With the default `std` feature off (`--no-default-features --features
//! spin`) the crate is `no_std` + `alloc`: everything but the two modules
//! that read and write files through `Os::File` / `Os::FileSystem`
//! ([`crc_checker`], [`validate_file`]), which are `std`-only like those
//! `fprime-os` facilities. See ARCHITECTURE.md "no_std".

// The std build keeps the std prelude (the file-based modules are hosted
// code); only the `no_std` build drops it. Unit tests always run on a
// hosted target, so they keep it too.
#![cfg_attr(not(any(feature = "std", test)), no_std)]
// The crate docs link the std-only modules; in the `no_std` build they are
// absent, so those links cannot resolve there.
#![cfg_attr(not(feature = "std"), allow(rustdoc::broken_intra_doc_links))]

extern crate alloc;

#[cfg(not(any(feature = "std", feature = "spin")))]
compile_error!(
    "fprime-utils needs a lock backend: enable the default `std` feature, or build \
     `no_std` with `--no-default-features --features spin`"
);

pub mod cfdp;
#[cfg(feature = "std")]
pub mod crc_checker;
pub mod hash;
pub mod rate_limiter;
pub mod token_bucket;
pub mod types;
#[cfg(feature = "std")]
pub mod validate_file;

#[cfg(feature = "std")]
pub use crc_checker::{
    CRC_FILE_READ_BLOCK, CrcStat, create_checksum_file, read_crc32_from_file, verify_checksum,
};
pub use hash::{HASH_DIGEST_LENGTH, HASH_EXTENSION_STRING, Hash, HashBuffer};
pub use rate_limiter::RateLimiter;
pub use token_bucket::{MAX_TOKEN_BUCKET_TOKENS, TokenBucket};
pub use types::{CircularBuffer, Queue, QueueMode, QueueOverflowMode};
