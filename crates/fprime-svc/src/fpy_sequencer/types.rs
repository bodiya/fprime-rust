//! The Fpy data types: `Svc/FpySequencer/FpySequencerTypes.fpp` (module
//! `Svc.Fpy`: the sequence file format, the directive ids and error codes),
//! the component-local types of `FpySequencer.fpp`,
//! `FpySequencerStateMachine.fppi` and `FpySequencerDirectives.fppi` (the
//! argument struct of every directive), and `Fw/Fpy/StatementArgBuffer`.
//!
//! Every struct here is an FPP struct, so its wire format is the generated
//! C++ one: members in declaration order, big-endian, strings and nested
//! buffers with a `u16` length prefix, arrays without a count.

use crate::cmd_sequencer::{BlockState, SeqArgs};
use fprime_config::fpy_sequencer::{
    MAX_DIRECTIVE_SIZE, MAX_SEQUENCE_ARG_COUNT, MAX_SEQUENCE_STATEMENT_COUNT,
};
use fprime_config::{
    FW_STATEMENT_ARG_BUFFER_MAX_SIZE, FwChanIdType, FwIndexType, FwOpcodeType, FwPrmIdType,
    FwSizeType,
};
use fprime_fw::{FileNameString, FwString, LinearBuffer, fpp_array, fpp_enum, fpp_struct};

/// `Fpy.SCHEMA_VERSION`: the sequence file schema this sequencer reads.
pub const SCHEMA_VERSION: u8 = 7;

/// `Fpy.MAX_ARG_SPEC_NAME_LEN`: the maximum length of an argument or type
/// name in an [`ArgSpec`].
pub const MAX_ARG_SPEC_NAME_LEN: usize = 255;

/// `Fpy.StackSizeType`: sizes and offsets on the sequence stack.
pub type StackSizeType = u32;

/// `Fpy.SignedStackSizeType`: relative stack offsets, which can be
/// negative.
pub type SignedStackSizeType = i32;

/// `Fw::StatementArgBuffer`: the argument bytes of one statement
/// (capacity `FW_STATEMENT_ARG_BUFFER_MAX_SIZE`; serialized as a nested
/// buffer, `[u16 size][bytes]`).
pub type StatementArgBuffer = LinearBuffer<FW_STATEMENT_ARG_BUFFER_MAX_SIZE>;

/// An argument or type name of an [`ArgSpec`] (`string size
/// MAX_ARG_SPEC_NAME_LEN`).
pub type ArgSpecName = FwString<MAX_ARG_SPEC_NAME_LEN>;

fpp_enum! {
    /// `Fpy.DirectiveId`: the opcode of a statement.
    ///
    /// The stack operations (`Or` through `Itrunc64To32`, and `Ffloor`
    /// through `Fabs`) all go through one `StackOpDirective`.
    pub enum DirectiveId : u8 {
        /// Not a directive (a compiler/sequencer version mismatch).
        Invalid = 0,
        /// Sleep for a relative duration popped off the stack.
        WaitRel = 1,
        /// Sleep until an absolute time popped off the stack.
        WaitAbs = 2,
        /// Jump to a statement index.
        Goto = 3,
        /// Branch on the top byte of the stack.
        If = 4,
        /// Do nothing.
        NoOp = 5,
        /// Push a telemetry value.
        PushTlmVal = 6,
        /// Push a parameter value.
        PushPrm = 7,
        /// Dispatch a command with constant arguments.
        ConstCmd = 8,
        /// Boolean or.
        Or = 9,
        /// Boolean and.
        And = 10,
        /// Integer equal.
        Ieq = 11,
        /// Integer not equal.
        Ine = 12,
        /// Unsigned less than.
        Ult = 13,
        /// Unsigned less or equal.
        Ule = 14,
        /// Unsigned greater than.
        Ugt = 15,
        /// Unsigned greater or equal.
        Uge = 16,
        /// Signed less than.
        Slt = 17,
        /// Signed less or equal.
        Sle = 18,
        /// Signed greater than.
        Sgt = 19,
        /// Signed greater or equal.
        Sge = 20,
        /// Float equal.
        Feq = 21,
        /// Float not equal.
        Fne = 22,
        /// Float less than.
        Flt = 23,
        /// Float less or equal.
        Fle = 24,
        /// Float greater than.
        Fgt = 25,
        /// Float greater or equal.
        Fge = 26,
        /// Boolean not.
        Not = 27,
        /// Float to signed integer (saturating).
        Fptosi = 28,
        /// Float to unsigned integer (saturating).
        Fptoui = 29,
        /// Signed integer to float.
        Sitofp = 30,
        /// Unsigned integer to float.
        Uitofp = 31,
        /// Integer add.
        Add = 32,
        /// Integer subtract.
        Sub = 33,
        /// Integer multiply.
        Mul = 34,
        /// Unsigned divide.
        Udiv = 35,
        /// Signed (floored) divide.
        Sdiv = 36,
        /// Unsigned modulo.
        Umod = 37,
        /// Signed (floored) modulo.
        Smod = 38,
        /// Float add.
        Fadd = 39,
        /// Float subtract.
        Fsub = 40,
        /// Float multiply.
        Fmul = 41,
        /// Float divide.
        Fdiv = 42,
        /// Float power.
        Fpow = 43,
        /// Float natural logarithm.
        Flog = 44,
        /// Float (floored) modulo.
        Fmod = 45,
        /// F32 to F64.
        Fpext = 46,
        /// F64 to F32.
        Fptrunc = 47,
        /// Sign-extend 8 to 64 bits.
        Siext8To64 = 48,
        /// Sign-extend 16 to 64 bits.
        Siext16To64 = 49,
        /// Sign-extend 32 to 64 bits.
        Siext32To64 = 50,
        /// Zero-extend 8 to 64 bits.
        Ziext8To64 = 51,
        /// Zero-extend 16 to 64 bits.
        Ziext16To64 = 52,
        /// Zero-extend 32 to 64 bits.
        Ziext32To64 = 53,
        /// Truncate 64 to 8 bits.
        Itrunc64To8 = 54,
        /// Truncate 64 to 16 bits.
        Itrunc64To16 = 55,
        /// Truncate 64 to 32 bits.
        Itrunc64To32 = 56,
        /// End the sequence with an exit code popped off the stack.
        Exit = 57,
        /// Push zeroed bytes.
        Allocate = 58,
        /// Store to a frame-relative constant offset.
        StoreRelConstOffset = 59,
        /// Load from a frame-relative offset.
        LoadRel = 60,
        /// Push constant bytes.
        PushVal = 61,
        /// Drop bytes off the stack.
        Discard = 62,
        /// Compare two byte arrays.
        Memcmp = 63,
        /// Dispatch a command built on the stack.
        StackCmd = 64,
        /// Push a telemetry value and its time.
        PushTlmValAndTime = 65,
        /// Push the current time.
        PushTime = 66,
        /// Extract a member of a struct on the stack.
        GetField = 67,
        /// Copy bytes from inside the stack to its top.
        Peek = 68,
        /// Store to a frame-relative offset popped off the stack.
        StoreRel = 69,
        /// Call a function.
        Call = 70,
        /// Return from a function.
        Return = 71,
        /// Load from an absolute offset.
        LoadAbs = 72,
        /// Store to an absolute offset popped off the stack.
        StoreAbs = 73,
        /// Store to an absolute constant offset.
        StoreAbsConstOffset = 74,
        /// Emit an event with a message from the stack.
        PopEvent = 75,
        /// Seed the random number generator.
        SetSeed = 76,
        /// Push a random number.
        PushRand = 77,
        /// Send stack bytes out a serial port.
        PopSerializable = 78,
        /// Float floor.
        Ffloor = 79,
        /// Integer absolute value.
        Iabs = 80,
        /// Float absolute value.
        Fabs = 81,
    }
    default Invalid
}

impl DirectiveId {
    /// True for the directives handled by the generic stack-op directive:
    /// `OR` through `ITRUNC_64_32`, and `FFLOOR` through `FABS` (the ranges
    /// `FpySequencerTypes.fpp` documents).
    #[must_use]
    pub const fn is_stack_op(self) -> bool {
        let raw = self.as_repr();
        (raw >= DirectiveId::Or.as_repr() && raw <= DirectiveId::Itrunc64To32.as_repr())
            || (raw >= DirectiveId::Ffloor.as_repr() && raw <= DirectiveId::Fabs.as_repr())
    }
}

fpp_enum! {
    /// `Fpy.DirectiveErrorCode`: why a directive failed.
    pub enum DirectiveErrorCode : u8 {
        /// No error.
        NoError = 0,
        /// A statement index past the end of the sequence.
        StmtOutOfBounds = 1,
        /// `getTlmChan` is not connected.
        TlmGetNotConnected = 2,
        /// The telemetry channel was not found.
        TlmChanNotFound = 3,
        /// `getParam` is not connected.
        PrmGetNotConnected = 4,
        /// The parameter was not found.
        PrmNotFound = 5,
        /// The command could not be serialized.
        CmdSerializeFailure = 6,
        /// The sequence exited with a non-zero code.
        ExitWithError = 7,
        /// A stack access out of bounds.
        StackAccessOutOfBounds = 8,
        /// The stack would overflow.
        StackOverflow = 9,
        /// A math domain error (division by zero, log of a non-positive).
        DomainError = 10,
        /// An array index out of bounds.
        ArrayOutOfBounds = 11,
        /// Integer overflow.
        ArithmeticOverflow = 12,
        /// Integer underflow.
        ArithmeticUnderflow = 13,
        /// A frame start out of bounds.
        FrameStartOutOfBounds = 14,
        /// Not enough bytes on the stack.
        StackUnderflow = 15,
        /// An invalid argument.
        InvalidArg = 16,
        /// The command failed.
        CmdFail = 17,
        /// The serial output port is not connected.
        SerialPortNotConnected = 18,
        /// The serial output port index is out of range.
        SerialPortInvalidIndex = 19,
    }
    default NoError
}

/// C++ `using DirectiveError = Fpy::DirectiveErrorCode`.
pub type DirectiveError = DirectiveErrorCode;

fpp_enum! {
    /// `Fpy.SerialPortIndex` (`FpySequencerCfg.fpp`): the `serialOut`
    /// indices; projects rename the example members.
    pub enum SerialPortIndex : i16 {
        /// Example serial port 0.
        ExamplePort0 = 0,
        /// Example serial port 1.
        ExamplePort1 = 1,
        /// Example serial port 2.
        ExamplePort2 = 2,
        /// Example serial port 3.
        ExamplePort3 = 3,
        /// Example serial port 4.
        ExamplePort4 = 4,
        /// The number of serial ports (the `serialOut` array size).
        MaxSerialPorts = 5,
    }
    default ExamplePort0
}

const _: () = assert!(
    SerialPortIndex::MaxSerialPorts.as_repr()
        == fprime_config::fpy_sequencer::MAX_SERIAL_PORTS as FwIndexType
);

fpp_enum! {
    /// `FpySequencer.GoalState`: where the sequencer is heading after
    /// validation (see `FpySequencer::m_goalState`).
    pub enum GoalState : u8 {
        /// Run the sequence after validating it.
        Running = 0,
        /// Stop after validating.
        Valid = 1,
        /// No sequence.
        Idle = 2,
    }
    default Running
}

fpp_enum! {
    /// `FpySequencer.FileReadStage`: which part of a sequence file was
    /// being read.
    pub enum FileReadStage : u8 {
        /// The header.
        Header = 0,
        /// The argument specs and statements.
        Body = 1,
        /// The CRC footer.
        Footer = 2,
    }
    default Header
}

fpp_struct! {
    /// `Fpy.ArgSpec`: a sequence argument's name, type and stack size.
    #[derive(Clone)]
    pub struct ArgSpec {
        /// The argument name.
        arg_name: ArgSpecName,
        /// The argument's type name.
        type_name: ArgSpecName,
        /// Size of this argument on the stack in bytes.
        arg_size: StackSizeType,
    }
}

fpp_struct! {
    /// `Fpy.Header`: the 11-byte header of a sequence file.
    #[derive(Clone, Copy, Eq)]
    pub struct Header {
        /// The major version of the FSW.
        major_version: u8,
        /// The minor version of the FSW.
        minor_version: u8,
        /// The patch version of the FSW.
        patch_version: u8,
        /// The schema version of this file.
        schema_version: u8,
        /// The number of input arguments to this sequence.
        argument_count: u8,
        /// The number of statements in the sequence.
        statement_count: u16,
        /// The size of the body in bytes.
        body_size: u32,
    }
}

fpp_struct! {
    /// `Fpy.Footer`: the CRC-32 of the header and body.
    #[derive(Clone, Copy, Eq)]
    pub struct Footer {
        /// CRC-32 (`Utils::Hash`) of everything before the footer.
        crc: u32,
    }
}

fpp_struct! {
    /// `Fpy.Statement`: a directive opcode and its argument bytes.
    #[derive(Clone, Eq)]
    pub struct Statement {
        /// The directive.
        op_code: DirectiveId,
        /// The directive's argument bytes.
        arg_buf: StatementArgBuffer,
    }
}

/// `Fpy.Sequence`: a loaded sequence. Upstream never serializes it as a
/// whole (it is the storage the file is read into), so it is a plain
/// struct with its arrays allocated once, at construction.
#[derive(Debug, Clone, PartialEq)]
pub struct Sequence {
    /// The header.
    pub header: Header,
    /// `[MAX_SEQUENCE_ARG_COUNT] ArgSpec`.
    pub args: Box<[ArgSpec]>,
    /// `[MAX_SEQUENCE_STATEMENT_COUNT] Statement`.
    pub statements: Box<[Statement]>,
    /// The footer.
    pub footer: Footer,
}

impl Sequence {
    /// C++ `Fpy::Sequence::SERIALIZED_SIZE`: the largest possible sequence
    /// (upstream's tests size their buffers with it).
    pub const SERIALIZED_SIZE: usize = Header::SERIALIZED_SIZE
        + MAX_SEQUENCE_ARG_COUNT * ArgSpec::SERIALIZED_SIZE
        + MAX_SEQUENCE_STATEMENT_COUNT * Statement::SERIALIZED_SIZE
        + Footer::SERIALIZED_SIZE;

    /// A default (all-zero) sequence. Allocates the argument and statement
    /// arrays (about 1 MiB with the default configuration).
    #[must_use]
    pub fn new() -> Self {
        Self {
            header: Header::default(),
            args: vec![ArgSpec::default(); MAX_SEQUENCE_ARG_COUNT].into_boxed_slice(),
            statements: vec![Statement::default(); MAX_SEQUENCE_STATEMENT_COUNT].into_boxed_slice(),
            footer: Footer::default(),
        }
    }

    /// Reset to the default value in place (C++ `Fpy::Sequence()`
    /// assignment), without reallocating.
    pub fn clear(&mut self) {
        self.header = Header::default();
        for arg in self.args.iter_mut() {
            *arg = ArgSpec::default();
        }
        for stmt in self.statements.iter_mut() {
            stmt.op_code = DirectiveId::Invalid;
            stmt.arg_buf = StatementArgBuffer::new();
        }
        self.footer = Footer::default();
    }

    /// Copy `other` into `self` without reallocating (C++ assignment).
    pub fn assign(&mut self, other: &Sequence) {
        self.header = other.header;
        self.args.clone_from_slice(&other.args);
        self.statements.clone_from_slice(&other.statements);
        self.footer = other.footer;
    }
}

impl Default for Sequence {
    fn default() -> Self {
        Self::new()
    }
}

fpp_struct! {
    /// `FpySequencer.SequenceExecutionArgs`: the value of the `cmd_RUN`,
    /// `cmd_VALIDATE` and `cmd_RUN_VALIDATED` signals.
    #[derive(Clone)]
    pub struct SequenceExecutionArgs {
        /// The sequence file (relative to `SEQ_BASE_DIR`).
        file_path: FileNameString,
        /// Whether the command response waits for the sequence to end.
        block: BlockState,
        /// The sequence arguments.
        buffer: SeqArgs,
    }
}

fpp_struct! {
    /// `FpySequencer.BreakpointArgs`: the value of `cmd_SET_BREAKPOINT`.
    #[derive(Clone, Copy, Eq)]
    pub struct BreakpointArgs {
        /// Whether or not to break at the breakpoint index.
        break_on_breakpoint: bool,
        /// Whether or not to remove the breakpoint after breaking on it.
        break_only_once_on_breakpoint: bool,
        /// The statement index at which to break, before dispatching.
        breakpoint_index: u32,
    }
}

fpp_array! {
    /// `[Fpy.MAX_DIRECTIVE_SIZE] U8`: the constant bytes of a `CONST_CMD`
    /// or `PUSH_VAL` directive (serialized in full, without a count).
    #[derive(Clone, Eq)]
    pub array DirectiveBytes = [u8; MAX_DIRECTIVE_SIZE]
    default fill 0
}

// ---------------------------------------------------------------------------
// Directive argument structs (FpySequencerDirectives.fppi). These are also
// the payloads of the `directive_*` internal ports.
// ---------------------------------------------------------------------------

fpp_struct! {
    /// `WaitRelDirective`: pops `U32` useconds, then `U32` seconds.
    #[derive(Clone, Copy, Eq)]
    pub struct WaitRelDirective { empty: u8 }
}

fpp_struct! {
    /// `WaitAbsDirective`: pops `U32` useconds, `U32` seconds, the time
    /// context and the time base.
    #[derive(Clone, Copy, Eq)]
    pub struct WaitAbsDirective { empty: u8 }
}

fpp_struct! {
    /// `GotoDirective`: sets the index of the next statement.
    #[derive(Clone, Copy, Eq)]
    pub struct GotoDirective {
        /// The statement index to execute next.
        statement_index: u32,
    }
}

fpp_struct! {
    /// `IfDirective`: pops a `U8`; jumps if it is zero.
    #[derive(Clone, Copy, Eq)]
    pub struct IfDirective {
        /// The statement index to go to if the top of the stack is false.
        false_goto_stmt_index: u32,
    }
}

fpp_struct! {
    /// `NoOpDirective`.
    #[derive(Clone, Copy, Eq)]
    pub struct NoOpDirective { empty: u8 }
}

fpp_struct! {
    /// `PushTlmValDirective`: pushes a telemetry value.
    #[derive(Clone, Copy, Eq)]
    pub struct PushTlmValDirective {
        /// The telemetry channel id.
        chan_id: FwChanIdType,
    }
}

fpp_struct! {
    /// `PushTlmValAndTimeDirective`: pushes a telemetry value, then its
    /// time.
    #[derive(Clone, Copy, Eq)]
    pub struct PushTlmValAndTimeDirective {
        /// The telemetry channel id.
        chan_id: FwChanIdType,
    }
}

fpp_struct! {
    /// `PushPrmDirective`: pushes a parameter value.
    #[derive(Clone, Copy, Eq)]
    pub struct PushPrmDirective {
        /// The parameter id.
        prm_id: FwPrmIdType,
    }
}

fpp_struct! {
    /// `ConstCmdDirective`: dispatches a command with constant arguments.
    #[derive(Clone, Eq)]
    pub struct ConstCmdDirective {
        /// The command opcode.
        op_code: FwOpcodeType,
        /// The argument bytes.
        arg_buf: DirectiveBytes,
        /// The number of valid bytes in `arg_buf`.
        arg_buf_size: FwSizeType,
    }
}

fpp_struct! {
    /// `StackOpDirective`: one of the stack operations.
    #[derive(Clone, Copy, Eq)]
    pub struct StackOpDirective {
        /// Which stack op to perform.
        op: DirectiveId,
    }
}

fpp_struct! {
    /// `ExitDirective`: pops an `I32` exit code.
    #[derive(Clone, Copy, Eq)]
    pub struct ExitDirective { empty: u8 }
}

fpp_struct! {
    /// `AllocateDirective`: pushes zeroed bytes.
    #[derive(Clone, Copy, Eq)]
    pub struct AllocateDirective {
        /// The number of bytes.
        size: StackSizeType,
    }
}

fpp_struct! {
    /// `StoreRelConstOffsetDirective`: stores to a frame-relative constant
    /// offset.
    #[derive(Clone, Copy, Eq)]
    pub struct StoreRelConstOffsetDirective {
        /// Signed byte offset relative to the frame start.
        lvar_offset: SignedStackSizeType,
        /// The number of bytes.
        size: StackSizeType,
    }
}

fpp_struct! {
    /// `LoadRelDirective`: loads from a frame-relative constant offset.
    #[derive(Clone, Copy, Eq)]
    pub struct LoadRelDirective {
        /// Signed byte offset relative to the frame start.
        lvar_offset: SignedStackSizeType,
        /// The number of bytes.
        size: StackSizeType,
    }
}

fpp_struct! {
    /// `PushValDirective`: pushes constant bytes.
    #[derive(Clone, Eq)]
    pub struct PushValDirective {
        /// The bytes.
        val: DirectiveBytes,
        /// The number of valid bytes in `val`.
        val_size: FwSizeType,
    }
}

fpp_struct! {
    /// `DiscardDirective`: drops bytes off the stack.
    #[derive(Clone, Copy, Eq)]
    pub struct DiscardDirective {
        /// The number of bytes.
        size: StackSizeType,
    }
}

fpp_struct! {
    /// `MemCmpDirective`: compares two byte arrays of `size` bytes.
    #[derive(Clone, Copy, Eq)]
    pub struct MemCmpDirective {
        /// The size of one array.
        size: StackSizeType,
    }
}

fpp_struct! {
    /// `StackCmdDirective`: pops an opcode and `args_size` argument bytes
    /// and dispatches the command.
    #[derive(Clone, Copy, Eq)]
    pub struct StackCmdDirective {
        /// The size of the argument bytes.
        args_size: StackSizeType,
    }
}

fpp_struct! {
    /// `PushTimeDirective`.
    #[derive(Clone, Copy, Eq)]
    pub struct PushTimeDirective { empty: u8 }
}

fpp_struct! {
    /// `SetSeedDirective`: pops a `U32` seed.
    #[derive(Clone, Copy, Eq)]
    pub struct SetSeedDirective { empty: u8 }
}

fpp_struct! {
    /// `PushRandDirective`.
    #[derive(Clone, Copy, Eq)]
    pub struct PushRandDirective { empty: u8 }
}

fpp_struct! {
    /// `GetFieldDirective`: pops an offset and keeps `member_size` bytes of
    /// the `parent_size`-byte struct below it.
    #[derive(Clone, Copy, Eq)]
    pub struct GetFieldDirective {
        /// The size of the struct.
        parent_size: StackSizeType,
        /// The size of the member to extract.
        member_size: StackSizeType,
    }
}

fpp_struct! {
    /// `PeekDirective`: pops an offset and a byte count.
    #[derive(Clone, Copy, Eq)]
    pub struct PeekDirective { empty: u8 }
}

fpp_struct! {
    /// `StoreRelDirective`: pops a frame-relative offset, then stores
    /// `size` bytes there.
    #[derive(Clone, Copy, Eq)]
    pub struct StoreRelDirective {
        /// The number of bytes.
        size: StackSizeType,
    }
}

fpp_struct! {
    /// `CallDirective`: pops a `U32` target statement index.
    #[derive(Clone, Copy, Eq)]
    pub struct CallDirective { empty: u8 }
}

fpp_struct! {
    /// `ReturnDirective`.
    #[derive(Clone, Copy, Eq)]
    pub struct ReturnDirective {
        /// Size of the return value in bytes, 0 for void functions.
        return_val_size: StackSizeType,
        /// Total size of the function arguments in bytes.
        call_args_size: StackSizeType,
    }
}

fpp_struct! {
    /// `LoadAbsDirective`: loads from an absolute stack offset.
    #[derive(Clone, Copy, Eq)]
    pub struct LoadAbsDirective {
        /// Absolute byte offset from the start of the stack.
        global_offset: StackSizeType,
        /// The number of bytes.
        size: StackSizeType,
    }
}

fpp_struct! {
    /// `StoreAbsDirective`: pops an absolute offset, then stores `size`
    /// bytes there.
    #[derive(Clone, Copy, Eq)]
    pub struct StoreAbsDirective {
        /// The number of bytes.
        size: StackSizeType,
    }
}

fpp_struct! {
    /// `StoreAbsConstOffsetDirective`: stores to an absolute constant
    /// offset.
    #[derive(Clone, Copy, Eq)]
    pub struct StoreAbsConstOffsetDirective {
        /// Absolute byte offset from the start of the stack.
        global_offset: StackSizeType,
        /// The number of bytes.
        size: StackSizeType,
    }
}

fpp_struct! {
    /// `PopEventDirective`: pops a message size, the message and a
    /// severity, and emits an event.
    #[derive(Clone, Copy, Eq)]
    pub struct PopEventDirective { empty: u8 }
}

fpp_struct! {
    /// `PopSerializableDirective`: sends `size` stack bytes out
    /// `serialOut[port_index]`.
    #[derive(Clone, Copy, Eq)]
    pub struct PopSerializableDirective {
        /// The port index.
        port_index: FwIndexType,
        /// The number of bytes.
        size: StackSizeType,
    }
}

/// C++ `FpySequencer::DirectiveUnion`: one deserialized directive.
#[derive(Debug, Clone, PartialEq)]
#[allow(missing_docs)]
pub enum DirectiveUnion {
    WaitRel(WaitRelDirective),
    WaitAbs(WaitAbsDirective),
    Goto(GotoDirective),
    If(IfDirective),
    NoOp(NoOpDirective),
    PushTlmVal(PushTlmValDirective),
    PushTlmValAndTime(PushTlmValAndTimeDirective),
    PushPrm(PushPrmDirective),
    ConstCmd(ConstCmdDirective),
    StackOp(StackOpDirective),
    Exit(ExitDirective),
    Allocate(AllocateDirective),
    StoreRelConstOffset(StoreRelConstOffsetDirective),
    LoadRel(LoadRelDirective),
    PushVal(PushValDirective),
    Discard(DiscardDirective),
    MemCmp(MemCmpDirective),
    StackCmd(StackCmdDirective),
    PushTime(PushTimeDirective),
    SetSeed(SetSeedDirective),
    PushRand(PushRandDirective),
    GetField(GetFieldDirective),
    Peek(PeekDirective),
    StoreRel(StoreRelDirective),
    Call(CallDirective),
    Return(ReturnDirective),
    LoadAbs(LoadAbsDirective),
    StoreAbs(StoreAbsDirective),
    StoreAbsConstOffset(StoreAbsConstOffsetDirective),
    PopEvent(PopEventDirective),
    PopSerializable(PopSerializableDirective),
}
