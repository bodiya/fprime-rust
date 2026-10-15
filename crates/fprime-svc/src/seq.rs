//! # Svc/Seq — the sequencer port types
//!
//! C++ source: `Svc/Seq/Seq.fpp` (`Svc.SeqArgs`, `Svc.BlockState`, the
//! `Svc.CmdSeqIn` and `Svc.CmdSeqCancel` ports).
//!
//! Shared by every sequencer and by the components that drive them:
//! [`cmd_sequencer`](crate::cmd_sequencer) and `fpy_sequencer` (both
//! `std`-only, they read sequence files) and
//! [`seq_dispatcher`](crate::seq_dispatcher) (`no_std`), so the types live
//! here rather than in a `std`-gated module. `cmd_sequencer` re-exports
//! them under their original paths.

use fprime_config::{FwIndexType, FwSizeType};
use fprime_fw::{FileNameString, fpp_array, fpp_enum, fpp_struct};

/// `SequenceArgumentsMaxSize` (AcConstants.fpp): 12 at the pinned
/// upstream commit (it was the 255 bytes left over in a command argument
/// buffer before upstream's WasmSequencer change, nasa/fprime#5386).
pub use fprime_config::SEQUENCE_ARGUMENTS_MAX_SIZE;

fpp_array! {
    /// The raw argument bytes of [`SeqArgs`] (`[SequenceArgumentsMaxSize] U8`).
    /// Serializes as `SEQUENCE_ARGUMENTS_MAX_SIZE` consecutive bytes — no
    /// count prefix, no padding.
    #[derive(Clone, Copy, Eq)]
    pub array SeqArgsBuffer = [u8; SEQUENCE_ARGUMENTS_MAX_SIZE]
    default fill 0
}

fpp_struct! {
    /// `Svc.SeqArgs` — arguments handed to a sequence over `Svc.CmdSeqIn`.
    /// Wire format: `[size u64 BE][12 raw bytes]` (20 bytes).
    ///
    /// `CmdSequencer` ignores the arguments it receives and always emits an
    /// all-zero value on `seqStartOut` (C++ `(void)args;`).
    #[derive(Clone, Copy, Eq)]
    pub struct SeqArgs {
        /// Number of valid bytes in `buffer`.
        size: FwSizeType { get_size, set_size },
        /// Raw argument bytes.
        buffer: SeqArgsBuffer { get_buffer, set_buffer },
    }
}

fpp_enum! {
    /// `Svc.BlockState` — whether a sequence-run command (`CS_RUN`,
    /// `SeqDispatcher.RUN`) defers its command response until the sequence
    /// finishes.
    pub enum BlockState : u8 {
        /// Answer the command only when the sequence completes or fails.
        Block = 0,
        /// Answer the command immediately.
        NoBlock = 1,
    }
    default Block
}

/// `Svc.CmdSeqIn` — request that a sequence file be run.
pub trait CmdSeqInPort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType, filename: &FileNameString, args: &SeqArgs);
}

/// `Svc.CmdSeqCancel` — cancel the running sequence (no arguments).
pub trait CmdSeqCancelPort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType);
}
