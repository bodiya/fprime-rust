//! Port of `Utils/Types` — allocation-free byte-level data structures.
//!
//! Not ported (phase 1): `Types::SpscQueue` (no phase-1 consumer).

pub mod circular_buffer;
pub mod queue;

pub use circular_buffer::CircularBuffer;
pub use queue::{Queue, QueueMode, QueueOverflowMode};
