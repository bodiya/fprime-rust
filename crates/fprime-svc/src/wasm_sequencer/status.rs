//! spacewasm outcomes as the FPP `Status` / `TrapReason` values: the
//! `spacewasm_c_api` `status.rs` mapping (error enum → `spacewasm_status_t`
//! code) and C++ `WasmSequencer::mapTrapReason`.

use super::{Status, TrapReason};
use fprime_spacewasm::Error;
use fprime_spacewasm::spacewasm::{
    AllocError, ConstantExprError, HostFunctionError, InvokeError, MemoryError, ResumeError,
    TrapReason as SwTrap, ValidationError,
};

/// The `Status` of an embedding-API error (the code the C API returns).
pub(crate) fn error_status(e: &Error) -> Status {
    match e {
        Error::BadUtf8 => Status::ErrBadUtf8,
        Error::NameTooLong => Status::ErrNameTooLong,
        Error::BadArg => Status::ErrBadArg,
        Error::Capacity => Status::ErrCapacity,
        Error::NotFound => Status::ErrNotFound,
        Error::WrongState => Status::ErrWrongState,
        Error::VecTooLong => Status::ErrVecTooLong,
        Error::ParamLenMismatch => Status::ErrParamLenMismatch,
        Error::ParamTypeMismatch => Status::ErrParamTypeMismatch,
        Error::GlobalTypeMismatch => Status::ErrGlobalTypeMismatch,
        Error::GlobalNotMutable => Status::ErrGlobalNotMutable,
        Error::ReaderError => Status::ErrReaderError,
        Error::Alloc(e) => alloc_status(e),
        Error::Memory(e) => memory_status(e),
        Error::Parse(e) => validation_status(&e.err.err),
        Error::Invoke(e) => invoke_status(e),
        Error::Resume(e) => resume_status(e),
        Error::HostFunction(e) => host_function_status(e),
    }
}

/// `alloc_status`.
fn alloc_status(e: &AllocError) -> Status {
    match e {
        AllocError::AllocationFailed => Status::ErrAllocFailed,
        AllocError::OutOfMemory => Status::ErrOutOfMemory,
        AllocError::PageTooSmall => Status::ErrPageTooSmall,
    }
}

/// `memory_status`.
pub(crate) fn memory_status(e: &MemoryError) -> Status {
    match e {
        MemoryError::OutOfBounds => Status::ErrMemOutOfBounds,
        MemoryError::OutOfMemory => Status::ErrOutOfMemory,
        MemoryError::AllocationFailed => Status::ErrAllocFailed,
        MemoryError::PageTooSmall => Status::ErrPageTooSmall,
    }
}

/// `invoke_status`.
fn invoke_status(e: &InvokeError) -> Status {
    match e {
        InvokeError::ParamLenMismatch => Status::ErrParamLenMismatch,
        InvokeError::ParamTypeMismatch => Status::ErrParamTypeMismatch,
        InvokeError::StackOverflow => Status::ErrStackOverflow,
        InvokeError::Busy => Status::ErrWrongState,
    }
}

/// `resume_status`.
fn resume_status(e: &ResumeError) -> Status {
    match e {
        ResumeError::NotPaused => Status::ErrWrongState,
        ResumeError::ResultTypeMismatch => Status::ErrParamTypeMismatch,
    }
}

/// `host_val_list_status`.
fn host_function_status(e: &HostFunctionError) -> Status {
    match e {
        HostFunctionError::ValListInvalidItem => Status::ErrBadSignature,
        HostFunctionError::ParameterListTooLong => Status::ErrFunctionParametersTooLarge,
        HostFunctionError::MultiReturnNotAllowed => Status::ErrFunctionReturnsTooLarge,
        HostFunctionError::AllocError(ae) => alloc_status(ae),
    }
}

/// `constant_expr_status`.
fn constant_expr_status(e: &ConstantExprError) -> Status {
    match e {
        ConstantExprError::InvalidConstantInstruction => Status::ErrInvalidConstInstruction,
        ConstantExprError::AlreadyHasValue => Status::ErrConstAlreadyHasValue,
        ConstantExprError::NoValue => Status::ErrConstNoValue,
        ConstantExprError::InvalidGlobal => Status::ErrConstInvalidGlobal,
    }
}

/// `validation_status`.
fn validation_status(e: &ValidationError) -> Status {
    use ValidationError as V;
    match e {
        V::Eof => Status::ErrEof,
        V::MalformedInteger => Status::ErrMalformedInteger,
        V::I33IsNegative => Status::ErrI33IsNegative,
        V::MalformedMagic => Status::ErrMalformedMagic,
        V::MalformedVersion => Status::ErrMalformedVersion,
        V::MalformedUtf8 => Status::ErrMalformedUtf8,
        V::DuplicateModuleName => Status::ErrDuplicateModuleName,
        V::DuplicateExportName => Status::ErrDuplicateExportName,
        V::MalformedSectionId(_) => Status::ErrMalformedSectionId,
        V::MalformedValueType(_) => Status::ErrMalformedValueType,
        V::MalformedFunction(_) => Status::ErrMalformedFunction,
        V::MalformedLimit(_) => Status::ErrMalformedLimit,
        V::MalformedElemType(_) => Status::ErrMalformedElemType,
        V::MalformedSectionSize => Status::ErrMalformedSectionSize,
        V::ExpectedConstOrVar(_) => Status::ErrExpectedConstOrVar,
        V::MalformedImportExportDesc(_) => Status::ErrMalformedImportExportDesc,
        V::MalformedMemType(_) => Status::ErrMalformedMemType,
        V::InvalidPageSize(_) => Status::ErrInvalidPageSize,
        V::InvalidSectionOrdering(_, _) => Status::ErrInvalidSectionOrdering,
        V::DuplicateSection(_) => Status::ErrDuplicateSection,
        V::InvalidMaxLimit => Status::ErrInvalidMaxLimit,
        V::ExpectedTerminal(_) => Status::ErrExpectedTerminal,
        V::InvalidOpcode(_) => Status::ErrInvalidOpcode,
        V::MalformedCodeSize => Status::ErrMalformedCodeSize,
        V::InvalidCodeSectionFunctionCount => Status::ErrInvalidCodeSectionFunctionCount,
        V::VecTooLong => Status::ErrVecTooLong,
        V::IdxTooLarge => Status::ErrIdxTooLarge,
        V::ModuleIdxTooLarge => Status::ErrModuleIdxTooLarge,
        V::MemoryTooLarge => Status::ErrMemoryTooLarge,
        V::TableTooLarge => Status::ErrTableTooLarge,
        V::MemoryImportTooLarge => Status::ErrMemoryImportTooLarge,
        V::MemAlignTooLarge => Status::ErrMemAlignTooLarge,
        V::ControlFlowTooDeep => Status::ErrControlFlowTooDeep,
        V::StackUnderflow => Status::ErrStackUnderflow,
        V::StackTooLarge => Status::ErrStackTooLarge,
        V::LabelStackJumpTooDeep => Status::ErrLabelStackJumpTooDeep,
        V::LabelJumpTooLarge => Status::ErrLabelJumpTooLarge,
        V::TypeMismatch => Status::ErrTypeMismatch,
        V::BlockResultTypeMismatch => Status::ErrBlockResultTypeMismatch,
        V::BrTableResultTypeMismatch => Status::ErrBrTableResultTypeMismatch,
        V::FunctionResultTypeMismatch => Status::ErrFunctionResultTypeMismatch,
        V::IllegalMemoryGrow => Status::ErrIllegalMemoryGrow,
        V::InvalidElementOffset => Status::ErrInvalidElementOffset,
        V::InvalidElementOutOfBounds => Status::ErrInvalidElementOutOfBounds,
        V::InvalidTableIndex => Status::ErrInvalidTableIndex,
        V::TableNotDefined => Status::ErrTableNotDefined,
        V::InvalidElementCount => Status::ErrInvalidElementCount,
        V::InvalidMemIndex => Status::ErrInvalidMemIndex,
        V::MemoryNotDefined => Status::ErrMemoryNotDefined,
        V::InvalidMemOffsetType => Status::ErrInvalidMemOffsetType,
        V::InvalidNegativeMemOffset => Status::ErrInvalidNegativeMemOffset,
        V::InvalidMemOffset => Status::ErrInvalidMemOffset,
        V::InvalidLabelIndex => Status::ErrInvalidLabelIndex,
        V::InvalidElseBlock => Status::ErrInvalidElseBlock,
        V::InvalidEndBlock => Status::ErrInvalidEndBlock,
        V::MultipleMemories => Status::ErrMultipleMemories,
        V::MultipleTables => Status::ErrMultipleTables,
        V::PossibleBackpatchCycle => Status::ErrPossibleBackpatchCycle,
        V::PageFault => Status::ErrPageFault,
        V::InstructionOutsideOfFunction => Status::ErrInstructionOutsideOfFunction,
        V::LocalIdxOutOfRange => Status::ErrLocalIdxOutOfRange,
        V::FunctionIdxOutOfRange => Status::ErrFunctionIdxOutOfRange,
        V::TypeIdxOutOfRange => Status::ErrTypeIdxOutOfRange,
        V::FunctionTextOutOfRange => Status::ErrFunctionTextOutOfRange,
        V::GlobalIdxOutOfRange => Status::ErrGlobalIdxOutOfRange,
        V::FunctionImportNotFound => Status::ErrFunctionImportNotFound,
        V::GlobalImportNotFound => Status::ErrGlobalImportNotFound,
        V::MemoryImportNotFound => Status::ErrMemoryImportNotFound,
        V::TableImportNotFound => Status::ErrTableImportNotFound,
        V::FunctionImportOutOfRange => Status::ErrFunctionImportOutOfRange,
        V::FunctionImportTypeMismatch => Status::ErrFunctionImportTypeMismatch,
        V::GlobalNotMutable => Status::ErrGlobalNotMutable,
        V::GlobalImportTypeMismatch => Status::ErrGlobalImportTypeMismatch,
        V::MemoryImportTypeMismatch => Status::ErrMemoryImportTypeMismatch,
        V::TableImportTypeMismatch => Status::ErrTableImportTypeMismatch,
        V::TableImportIncompatibleSize => Status::ErrTableImportIncompatibleSize,
        V::TableRefNotUnique => Status::ErrTableRefNotUnique,
        V::FunctionParametersTooLarge => Status::ErrFunctionParametersTooLarge,
        V::FunctionReturnsTooLarge => Status::ErrFunctionReturnsTooLarge,
        V::TooManyLocals => Status::ErrTooManyLocals,
        V::InvalidConstInstruction => Status::ErrInvalidConstInstruction,
        V::GlobalTypeMismatch => Status::ErrGlobalTypeMismatch,
        V::AlignmentLargerThanType => Status::ErrAlignmentLargerThanType,
        V::InvalidStartFunctionSignature => Status::ErrInvalidStartFunctionSignature,
        V::InvalidHostStartFunction => Status::ErrInvalidHostStartFunction,
        V::InvalidConstantExpr(ce) => constant_expr_status(ce),
        V::GuestMemoryAllocationFailure => Status::ErrGuestMemoryAllocFailed,
        V::AllocError(ae) => alloc_status(ae),
        V::MemoryError(me) => memory_status(me),
        V::ReaderError(_) => Status::ErrReaderError,
    }
}

/// The C API's `spacewasm_trap_t` (`trap_reason_code`): `None` stands for
/// `SPACEWASM_TRAP_NONE`, the "trap" `spacewasm_run` reports for an idle
/// engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trap {
    /// `SPACEWASM_TRAP_NONE`.
    None,
    /// A spacewasm trap.
    Reason(SwTrap),
}

/// C++ `mapTrapReason`: `spacewasm_trap_t` codes 0..14 map one to one onto
/// `TrapReason`; anything else — `SPACEWASM_TRAP_NONE` and
/// `SPACEWASM_TRAP_MALFORMED_IR` (which the C++ switch does not list) —
/// becomes `HOST`.
#[must_use]
pub fn map_trap_reason(trap: Trap) -> TrapReason {
    match trap {
        Trap::Reason(SwTrap::Unreachable) => TrapReason::Unreachable,
        Trap::Reason(SwTrap::Host) => TrapReason::Host,
        Trap::Reason(SwTrap::DivideByZero) => TrapReason::DivideByZero,
        Trap::Reason(SwTrap::InvalidTableIndex) => TrapReason::InvalidTableIndex,
        Trap::Reason(SwTrap::InvalidTableFunctionType) => TrapReason::InvalidTableFunctionType,
        Trap::Reason(SwTrap::UninitializedTableElement) => TrapReason::UninitializedTableElement,
        Trap::Reason(SwTrap::GlobalGetFailed) => TrapReason::GlobalGetFailed,
        Trap::Reason(SwTrap::GlobalSetFailed) => TrapReason::GlobalSetFailed,
        Trap::Reason(SwTrap::OutOfMemory) => TrapReason::OutOfMemory,
        Trap::Reason(SwTrap::MemoryRefNotUnique) => TrapReason::MemoryRefNotUnique,
        Trap::Reason(SwTrap::MemoryOutOfBounds) => TrapReason::MemoryOutOfBounds,
        Trap::Reason(SwTrap::StackOverflow) => TrapReason::StackOverflow,
        Trap::Reason(SwTrap::UnrepresentableResult) => TrapReason::UnrepresentableResult,
        Trap::Reason(SwTrap::IntegerOverflow) => TrapReason::IntegerOverflow,
        Trap::Reason(SwTrap::BadConversionToInteger) => TrapReason::BadConversionToInteger,
        Trap::Reason(SwTrap::MalformedIr) | Trap::None => TrapReason::Host,
    }
}
