//! fprime-utils: port of the F Prime `Utils/` support libraries.
//!
//! - [`hash`] — `Utils::Hash` CRC32 (IEEE 802.3) and `HashBuffer`
//! - [`crc_checker`] — `Utils::CRCChecker` file checksum sidecars
//! - [`types`] — `Types::CircularBuffer` byte ring and `Types::Queue`
//!   fixed-message FIFO/LIFO
//! - [`rate_limiter`] — `Utils::RateLimiter`
//! - [`token_bucket`] — `Utils::TokenBucket`
//!
//! Analyses: `docs/cpp-analysis/utils-misc.md` and the Utils::Hash section
//! of `docs/cpp-analysis/svc-comms.md`. None of these types are thread safe
//! (C++ parity); callers wrap them in concurrency constructs.

pub mod cfdp;
pub mod crc_checker;
pub mod hash;
pub mod rate_limiter;
pub mod token_bucket;
pub mod types;

pub use crc_checker::{
    CRC_FILE_READ_BLOCK, CrcStat, create_checksum_file, read_crc32_from_file, verify_checksum,
};
pub use hash::{HASH_DIGEST_LENGTH, HASH_EXTENSION_STRING, Hash, HashBuffer};
pub use rate_limiter::RateLimiter;
pub use token_bucket::{MAX_TOKEN_BUCKET_TOKENS, TokenBucket};
pub use types::{CircularBuffer, Queue, QueueMode, QueueOverflowMode};
