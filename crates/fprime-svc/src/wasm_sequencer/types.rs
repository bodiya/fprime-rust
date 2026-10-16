//! The WasmSequencer data types: `WasmSequencerTypes.fppi`, the types of
//! `WasmSequencerControllerStateMachine.fppi`,
//! `WasmSequencerInterpreterStateMachine.fppi` and
//! `WasmSequencerEvents.fppi`.
//!
//! Every struct is an FPP struct, so its wire format is the generated C++
//! one (members in declaration order, big-endian, strings with a `u16`
//! length prefix) — which is what crosses the component queue inside the
//! state machine signals.

use crate::seq::{BlockState, SeqArgs};
use fprime_config::wasm_sequencer::MODULE_NAME_STRING_SIZE;
use fprime_config::{FILE_NAME_STRING_SIZE, FwOpcodeType};
use fprime_fw::{FwString, fpp_enum, fpp_struct};

/// A sequence file name (`string size FileNameStringSize`).
pub type FileName = FwString<FILE_NAME_STRING_SIZE>;

/// A WebAssembly module name (`string size Wasm.MODULE_NAME_STRING_SIZE`).
pub type ModuleName = FwString<MODULE_NAME_STRING_SIZE>;

/// `WasmSequencer.ModuleIdx`: a module's index in the store.
pub type ModuleIdx = u8;

fpp_enum! {
    /// `WasmSequencer.HostFunction`: the host function being executed.
    pub enum HostFunction : u8 {
        /// No host function.
        None = 0,
        /// `tlm`: read a telemetry channel.
        Telemetry = 1,
        /// `prm`: read a parameter.
        Parameter = 2,
        /// `cmd`: dispatch a command.
        Command = 3,
        /// `event`: emit an event.
        Event = 4,
        /// `rsleep`: relative sleep.
        Rsleep = 5,
        /// `asleep`: absolute sleep.
        Asleep = 6,
        /// `args`: read the sequence arguments.
        Args = 7,
        /// `time`: read the current time.
        Time = 8,
        /// `serial_send`: invoke a serial output port.
        SerialOut = 9,
        /// `serial_recv`: read a serial input queue.
        SerialRecv = 10,
    }
    default None
}

fpp_enum! {
    /// `WasmSequencer.SequencePhase`: which phase of a module a completion
    /// event refers to.
    pub enum SequencePhase : u8 {
        /// The module's optional start function.
        Start = 0,
        /// The module's main entrypoint.
        Main = 1,
    }
    default Start
}

fpp_enum! {
    /// `WasmSequencer.MemoryGrowFailReason`: why a guest `memory.grow` was
    /// refused.
    pub enum MemoryGrowFailReason : u8 {
        /// The memory is not the last allocation in the guest pool.
        NotLastAllocation = 0,
        /// The grown size would exceed the guest pool.
        InsufficientPoolSpace = 1,
    }
    default NotLastAllocation
}

fpp_enum! {
    /// `WasmSequencer.Status`: the `spacewasm_status_t` codes.
    #[allow(missing_docs)]
    pub enum Status : i32 {
        Ok = 0,
        ErrNullArg = 1,
        ErrBadArg = 2,
        ErrBadUtf8 = 3,
        ErrNameTooLong = 4,
        ErrBadSignature = 5,
        ErrCapacity = 6,
        ErrNotFound = 7,
        ErrWrongState = 8,
        ErrGuestMemoryAllocFailed = 15,
        ErrAllocFailed = 16,
        ErrOutOfMemory = 17,
        ErrPageTooSmall = 18,
        ErrMemOutOfBounds = 32,
        ErrParamLenMismatch = 48,
        ErrParamTypeMismatch = 49,
        ErrStackOverflow = 50,
        ErrEof = 64,
        ErrMalformedInteger = 65,
        ErrI33IsNegative = 66,
        ErrMalformedMagic = 67,
        ErrMalformedVersion = 68,
        ErrMalformedUtf8 = 69,
        ErrDuplicateModuleName = 70,
        ErrDuplicateExportName = 71,
        ErrMalformedSectionId = 72,
        ErrMalformedValueType = 73,
        ErrMalformedFunction = 74,
        ErrMalformedLimit = 75,
        ErrMalformedElemType = 76,
        ErrMalformedSectionSize = 77,
        ErrExpectedConstOrVar = 78,
        ErrMalformedImportExportDesc = 79,
        ErrMalformedMemType = 80,
        ErrInvalidPageSize = 81,
        ErrInvalidSectionOrdering = 82,
        ErrDuplicateSection = 83,
        ErrInvalidMaxLimit = 84,
        ErrExpectedTerminal = 85,
        ErrInvalidOpcode = 86,
        ErrMalformedCodeSize = 87,
        ErrInvalidCodeSectionFunctionCount = 88,
        ErrVecTooLong = 89,
        ErrIdxTooLarge = 90,
        ErrModuleIdxTooLarge = 91,
        ErrMemoryTooLarge = 92,
        ErrMemoryImportTooLarge = 93,
        ErrMemAlignTooLarge = 94,
        ErrTableTooLarge = 95,
        ErrControlFlowTooDeep = 96,
        ErrStackUnderflow = 97,
        ErrStackTooLarge = 98,
        ErrLabelStackJumpTooDeep = 99,
        ErrLabelJumpTooLarge = 100,
        ErrTypeMismatch = 101,
        ErrBlockResultTypeMismatch = 102,
        ErrFunctionResultTypeMismatch = 103,
        ErrBrTableResultTypeMismatch = 104,
        ErrIllegalMemoryGrow = 112,
        ErrInvalidElementOffset = 113,
        ErrInvalidElementOutOfBounds = 114,
        ErrInvalidTableIndex = 115,
        ErrTableNotDefined = 116,
        ErrInvalidElementCount = 117,
        ErrInvalidMemIndex = 118,
        ErrMemoryNotDefined = 119,
        ErrInvalidMemOffsetType = 120,
        ErrInvalidNegativeMemOffset = 121,
        ErrInvalidMemOffset = 122,
        ErrMultipleMemories = 123,
        ErrMultipleTables = 124,
        ErrInvalidLabelIndex = 128,
        ErrInvalidElseBlock = 129,
        ErrInvalidEndBlock = 130,
        ErrInstructionOutsideOfFunction = 131,
        ErrLocalIdxOutOfRange = 132,
        ErrFunctionIdxOutOfRange = 133,
        ErrTypeIdxOutOfRange = 134,
        ErrFunctionTextOutOfRange = 135,
        ErrGlobalIdxOutOfRange = 136,
        ErrFunctionImportNotFound = 144,
        ErrGlobalImportNotFound = 145,
        ErrMemoryImportNotFound = 146,
        ErrTableImportNotFound = 147,
        ErrFunctionImportOutOfRange = 148,
        ErrFunctionImportTypeMismatch = 149,
        ErrGlobalNotMutable = 150,
        ErrGlobalImportTypeMismatch = 151,
        ErrMemoryImportTypeMismatch = 152,
        ErrTableImportTypeMismatch = 153,
        ErrTableImportIncompatibleSize = 154,
        ErrTableRefNotUnique = 155,
        ErrFunctionParametersTooLarge = 160,
        ErrFunctionReturnsTooLarge = 161,
        ErrTooManyLocals = 162,
        ErrInvalidConstInstruction = 163,
        ErrGlobalTypeMismatch = 164,
        ErrAlignmentLargerThanType = 165,
        ErrInvalidStartFunctionSignature = 166,
        ErrInvalidHostStartFunction = 167,
        ErrConstAlreadyHasValue = 176,
        ErrConstNoValue = 177,
        ErrConstInvalidGlobal = 178,
        ErrPossibleBackpatchCycle = 192,
        ErrPageFault = 193,
        ErrReaderError = 194,
    }
    default Ok
}

fpp_enum! {
    /// `WasmSequencer.SignalSource`: where an asynchronous request came
    /// from.
    pub enum SignalSource : u8 {
        /// The `RUN` command.
        CommandRun = 0,
        /// The `INVOKE` command.
        CommandInvoke = 1,
        /// The `LOAD` command.
        CommandLoad = 2,
        /// The `seqRunIn` port.
        PortRun = 3,
        /// A port invoke (declared; no port produces it).
        PortInvoke = 4,
    }
    default CommandRun
}

fpp_enum! {
    /// `WasmSequencer.TrapReason`: why the interpreter trapped (no
    /// representation type in FPP, so `FwEnumStoreType`).
    pub enum TrapReason : i32 {
        /// No recorded trap.
        None = -1,
        /// An `unreachable` instruction.
        Unreachable = 0,
        /// A host function noted an unrecoverable failure.
        Host = 1,
        /// Integer or floating point division by zero.
        DivideByZero = 2,
        /// An indirect call mapped to a table function out of range.
        InvalidTableIndex = 3,
        /// The function type of an indirect call did not match.
        InvalidTableFunctionType = 4,
        /// An indirect call referenced an uninitialized table element.
        UninitializedTableElement = 5,
        /// An imported global could not be read.
        GlobalGetFailed = 6,
        /// An imported global could not be set.
        GlobalSetFailed = 7,
        /// The interpreter ran out of memory.
        OutOfMemory = 8,
        /// `memory.grow` failed because a host function owns the memory.
        MemoryRefNotUnique = 9,
        /// A memory operation was out of bounds.
        MemoryOutOfBounds = 10,
        /// The operand stack overflowed.
        StackOverflow = 11,
        /// Conversion of an infinity to an integer.
        UnrepresentableResult = 12,
        /// Signed division overflowed.
        IntegerOverflow = 13,
        /// Conversion of a NaN to an integer.
        BadConversionToInteger = 14,
        /// The IR decoder met an opcode or immediate it cannot interpret.
        MalformedIr = 15,
    }
    default None
}

fpp_enum! {
    /// `WasmSequencer.AllocError` (declared by the model; unused upstream).
    pub enum AllocError : u8 {
        /// Not enough pages for the allocation.
        OutOfMemory = 0,
        /// A page was too small for the allocation.
        PageTooSmall = 1,
        /// A generic allocation failure.
        AllocationFailed = 2,
    }
    default OutOfMemory
}

fpp_enum! {
    /// `WasmSequencer.ExitReason`: why the last sequence ended.
    pub enum ExitReason : u8 {
        /// Not set (a bug, if it is ever reported).
        Unknown = 0,
        /// The interpreter finished; the code is main's result.
        InterpreterFinished = 1,
        /// The interpreter trapped.
        InterpreterTrap = 2,
        /// The guest called `exit`.
        HostExit = 3,
        /// The guest called `panic`.
        HostPanic = 4,
        /// A host function reply timed out.
        ReplyTimeout = 5,
        /// A host function replied with an error.
        HostFailure = 6,
        /// An unexpected host function reply arrived.
        UnexpectedReply = 7,
        /// The sequence was cancelled.
        Cancel = 8,
        /// A timer became incomparable (time base change).
        TimerIncomparable = 9,
    }
    default Unknown
}

fpp_struct! {
    /// `WasmSequencer.CommandRequest`: the opcode and sequence number to
    /// answer a command with.
    #[derive(Clone, Copy, Eq)]
    pub struct CommandRequest {
        /// The opcode to reply to.
        opcode: FwOpcodeType,
        /// The command sequence number to reply with.
        cmd_seq: u32,
    }
}

fpp_struct! {
    /// `WasmSequencer.RequestContext`: where a request came from and how
    /// to answer it.
    #[derive(Clone, Copy, Eq)]
    pub struct RequestContext {
        /// The originating signal source.
        source: SignalSource,
        /// The opcode / sequence number when `source` is a command.
        cmd_ctx: CommandRequest,
        /// Answer after initializing (`NO_BLOCK`) or when finished (`BLOCK`).
        block: BlockState,
        /// The module whose start / main function runs (filled in after
        /// the load or the module lookup).
        module_idx: ModuleIdx,
    }
}

fpp_struct! {
    /// `WasmSequencer.LoadRequest`: the value of the `run` and `load`
    /// signals.
    #[derive(Clone)]
    pub struct LoadRequest {
        /// The sequence file.
        file_name: FileName,
        /// The module name (empty for a standalone module).
        module_name: ModuleName,
        /// The sequence arguments.
        args: SeqArgs,
        /// How to answer the request.
        context: RequestContext,
    }
}

fpp_struct! {
    /// `WasmSequencer.InvokeRequest`: the value of the `invoke` signal.
    #[derive(Clone)]
    pub struct InvokeRequest {
        /// The module to invoke.
        module_name: ModuleName,
        /// The sequence arguments.
        args: SeqArgs,
        /// How to answer the request.
        context: RequestContext,
    }
}
