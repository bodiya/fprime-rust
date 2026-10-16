//! Errors of the embedding API.

use spacewasm::{AllocError, HostFunctionError, InvokeError, MemoryError, ParseError, ResumeError};

/// Why a [`crate::WasmEngine`] / [`crate::Heap`] operation failed: the
/// distinct `spacewasm_status_t` outcomes of the C API calls this crate
/// replaces, with spacewasm's own error values where the C API derives the
/// status from one (the component maps each onto its FPP `Status`).
#[derive(Debug, Clone)]
pub enum Error {
    /// A name or signature was not valid UTF-8 (`SPACEWASM_ERR_BAD_UTF8`).
    BadUtf8,
    /// A host name did not fit its capacity (`SPACEWASM_ERR_NAME_TOO_LONG`).
    NameTooLong,
    /// A value had the wrong tag or an index was out of range
    /// (`SPACEWASM_ERR_BAD_ARG`).
    BadArg,
    /// A fixed-capacity table was full (`SPACEWASM_ERR_CAPACITY`).
    Capacity,
    /// The module, export, function, global or result was not found
    /// (`SPACEWASM_ERR_NOT_FOUND`).
    NotFound,
    /// The engine or allocator registry was in the wrong state
    /// (`SPACEWASM_ERR_WRONG_STATE`).
    WrongState,
    /// More modules were requested than an engine can address
    /// (`SPACEWASM_ERR_VEC_TOO_LONG`).
    VecTooLong,
    /// A signature check failed on the parameter or result count
    /// (`SPACEWASM_ERR_PARAM_LEN_MISMATCH`).
    ParamLenMismatch,
    /// A signature check failed on a parameter or result type
    /// (`SPACEWASM_ERR_PARAM_TYPE_MISMATCH`).
    ParamTypeMismatch,
    /// A global write had the wrong value type
    /// (`SPACEWASM_ERR_GLOBAL_TYPE_MISMATCH`).
    GlobalTypeMismatch,
    /// A global write targeted an immutable global
    /// (`SPACEWASM_ERR_GLOBAL_NOT_MUTABLE`).
    GlobalNotMutable,
    /// The module reader reported an I/O error (`SPACEWASM_ERR_READER_ERROR`).
    ReaderError,
    /// An allocation failed (`alloc_status`).
    Alloc(AllocError),
    /// A memory operation failed (`memory_status`).
    Memory(MemoryError),
    /// A module failed to decode or validate (`parse_status`).
    Parse(ParseError),
    /// An invocation was refused (`invoke_status`).
    Invoke(InvokeError),
    /// A resume was refused (`resume_status`).
    Resume(ResumeError),
    /// A host function definition was invalid (`host_val_list_status`).
    HostFunction(HostFunctionError),
}
