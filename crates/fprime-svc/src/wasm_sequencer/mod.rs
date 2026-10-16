//! # WasmSequencer — port of `Svc::WasmSequencer` (active)
//!
//! C++ sources: `Svc/WasmSequencer/WasmSequencer.{fpp,hpp,cpp}`, the
//! `WasmSequencer{Types,Commands,Events,Params,Telemetry}.fppi` interface
//! files, `WasmSequencerControllerStateMachine.fppi` +
//! `WasmSequencerController.cpp` ([`controller`]),
//! `WasmSequencerInterpreterStateMachine.fppi` +
//! `WasmSequencerInterpreter.cpp` ([`interpreter`]),
//! `WasmSequencerHost.cpp` ([`host`]), `WasmSequencerHelpers.cpp`, the
//! `fprime_spacewasm` glue crate (here `fprime-spacewasm`) and
//! `default/config/WasmSequencer{Cfg.fpp,Config.hpp,SpacewasmConfig.h}`
//! ([`fprime_config::wasm_sequencer`]). Documentation:
//! `Svc/WasmSequencer/docs/sdd.md`.
//!
//! Compiled only with the `wasm` feature of `fprime-svc` (it pulls in
//! NASA's `spacewasm` interpreter) and with `std` (sequences are files).
//!
//! A sequence is a WebAssembly 1.0 module. `LOAD` streams a module file
//! into the interpreter's store (decoding and validating it into
//! spacewasm's IR) and runs its start function, if any; `INVOKE` runs the
//! `main` export (`[] -> []` or `[] -> i32`) of a loaded module; `RUN`
//! resets the store and does both for one unnamed module. The guest talks
//! to the flight software through the host module `fprime_v1` ([`host`]):
//! `exit`, `panic`, `args`, `time`, `tlm`, `prm`, `cmd`, `event`,
//! `rsleep`, `asleep`, `serial_send`, `serial_recv`.
//!
//! Two FPP state machines drive it, both through the component queue:
//!
//! ```text
//! controller:  IDLE --load/run--> LOADING_* --> (start?) --> READY / RUNNING_MAIN
//!              READY --invoke--> INVOKING --> RUNNING_MAIN --engineFinished--> READY | IDLE
//! interpreter: IDLE --run--> RUNNING { PAUSE_CHECK -> SPINNING (INSTRUCTION_FUEL
//!              instructions per message) -> AWAITING_RESPONSE { WAITING | SLEEPING } ; PAUSED }
//! ```
//!
//! A host function never runs flight-software code inside the interpreter:
//! it records the call ([`PendingHostFunction`]) and pauses the engine; the
//! interpreter machine then dispatches it on the component thread and
//! resumes the engine with the result (immediately, on a command response,
//! on a serial message, or when a sleep timer expires on `checkTimers`).
//!
//! Commands sent by a sequence carry a command UID in their context:
//! `(sequencesStarted & 0xFFFF) << 16 | (commandsDispatched & 0xFFFF)`;
//! `cmdResponseIn` uses it to tell a late response from an earlier
//! sequence (reported, ignored) from a wrong one (fails the sequence).
//!
//! Memory ([`Config`], allocated once by [`WasmSequencer::configure`]): the
//! interpreter heap (`heap_pages` pages of
//! [`SPACEWASM_PAGE_SIZE`](fprime_config::wasm_sequencer::SPACEWASM_PAGE_SIZE)
//! bytes: the store, the modules, their IR and the operand stack), the guest
//! linear-memory pool, the `serial_send` buffer and the `serialIn` queues.
//! Resetting the store recycles all of it in place.
//!
//! Threading: ACTIVE. All C++ `m_*` members live behind `Mutex`es touched
//! only on the component thread, except the `serialIn` queues (C++
//! `m_serialInMutex`, the guarded `serialIn` port) and the parameters. The
//! store is its own lock (`engine`), held while the interpreter runs; host
//! functions, called from inside the interpreter, take only the state lock.
//! Lock order: global allocator lock → heap → engine → state.

pub mod controller;
pub mod controller_sm;
mod events;
pub mod host;
pub mod interpreter;
pub mod interpreter_sm;
mod status;
mod types;

#[cfg(test)]
mod tests;

pub use controller_sm as controller_state_machine;
pub use interpreter_sm as interpreter_state_machine;
pub use types::*;

use crate::seq::{BlockState, CmdSeqCancelPort, CmdSeqInPort, SeqArgs};
use crate::tlm_chan::TlmGetPort;
use fprime_comp::serial_port::SerializePort;
use fprime_comp::sm::{self, SmSender};
use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, CmdResponsePort, ComPort, ComponentDispatch,
    EventGlue, MsgDispatchStatus, OutputPort, PortRef, PrmGetPort, PrmGlue, QueueFullPolicy,
    SchedPort, TlmGlue, async_input_port_adapter, component_msg_types, component_sm_ids, msg,
};
use fprime_config::wasm_sequencer::{
    DEFAULT_INSTRUCTION_FUEL, DEFAULT_SEQ_BASE_DIR, GLOBAL_NAME_STRING_SIZE,
    MAX_CONCURRENT_WAIT_COMMANDS, MAX_SERIAL_IN_PORTS, MAX_SERIAL_OUT_PORTS, SPACEWASM_MAX_PAGES,
};
use fprime_config::{
    FwChanIdType, FwEnumStoreType, FwEventIdType, FwIdType, FwIndexType, FwOpcodeType, FwPrmIdType,
    FwQueuePriorityType, FwSizeType,
};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, CmdStringArg, Endianness, FileNameString, FwString, LinearBuffer,
    ParamBuffer, ParamString, ParamValid, SerBuf, SerBufAny, Serialize, SerializeStatus, Time,
    fw_assert,
};
use fprime_spacewasm::{GuestPool, Heap, HeapGuard, WasmEngine};
use fprime_utils::CircularBuffer;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError, Weak};

/// The controller machine's state enum (C++
/// `WasmSequencer_ControllerStateMachine_State`).
pub type ControllerState = controller_sm::State;
/// The interpreter machine's state enum (C++
/// `WasmSequencer_InterpreterStateMachine_State`).
pub type InterpreterState = interpreter_sm::State;

/// A global export name (`string size Wasm.GLOBAL_NAME_STRING_SIZE`).
pub type GlobalName = FwString<GLOBAL_NAME_STRING_SIZE>;

// ---------------------------------------------------------------------------
// Dictionary constants and queue layout.
// ---------------------------------------------------------------------------

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel): the async input ports
    /// in FPP declaration order (as the generated `MsgTypeEnum` sorts them),
    /// the async commands, then the state machine signals.
    impl WasmSequencer {
        /// `checkTimers` (assert).
        MSG_TYPE_CHECK_TIMERS,
        /// `cmdResponseIn` (assert).
        MSG_TYPE_CMD_RESPONSE_IN,
        /// `seqRunIn` (assert).
        MSG_TYPE_SEQ_RUN_IN,
        /// `writeTelemetry` (drop).
        MSG_TYPE_WRITE_TELEMETRY,
        /// The async commands.
        MSG_TYPE_CMD_IN,
        /// `INTERNAL_STATE_MACHINE_SIGNAL`.
        MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
    }
}

component_sm_ids! {
    /// C++ `SmId`: the state machine instances, sorted by name.
    impl WasmSequencer {
        /// Instance `controller`.
        SM_ID_CONTROLLER,
        /// Instance `interpreter`.
        SM_ID_INTERPRETER,
    }
}

/// Queue message size: the largest message (an async command, `seqRunIn`
/// or a state machine signal — the `run` / `load` `LoadRequest`).
pub const QUEUE_MSG_SIZE: usize = sm::max_size(&[
    msg::ENVELOPE_HEADER_SIZE + 4 + 4 + CmdArgBuffer::SERIALIZED_SIZE,
    msg::ENVELOPE_HEADER_SIZE + FileNameString::SERIALIZED_SIZE + SeqArgs::SERIALIZED_SIZE,
    controller_sm::StateMachine::SIGNAL_MSG_SIZE,
    interpreter_sm::StateMachine::SIGNAL_MSG_SIZE,
]);

/// Every message has the FPP default priority (none is declared).
const PRIORITY: FwQueuePriorityType = 0;

/// Number of `serialOut` ports.
pub const NUM_SERIAL_OUT_PORTS: usize = MAX_SERIAL_OUT_PORTS as usize;
/// Number of `serialIn` ports (C++ `NUM_SERIALIN_INPUT_PORTS`).
pub const NUM_SERIAL_IN_PORTS: usize = MAX_SERIAL_IN_PORTS as usize;

impl WasmSequencer {
    /// `RUN(fileName, block, seqArgs)`.
    pub const OPCODE_RUN: FwOpcodeType = 0;
    /// `LOAD(fileName, name)`.
    pub const OPCODE_LOAD: FwOpcodeType = 1;
    /// `INVOKE(module, block, seqArgs)`.
    pub const OPCODE_INVOKE: FwOpcodeType = 2;
    /// `WAIT()`.
    pub const OPCODE_WAIT: FwOpcodeType = 3;
    /// `CANCEL()` (sync).
    pub const OPCODE_CANCEL: FwOpcodeType = 4;
    /// `PAUSE()`.
    pub const OPCODE_PAUSE: FwOpcodeType = 5;
    /// `CONTINUE()`.
    pub const OPCODE_CONTINUE: FwOpcodeType = 6;
    /// `GLOBAL_SET_I32(moduleName, name, value)`.
    pub const OPCODE_GLOBAL_SET_I32: FwOpcodeType = 7;
    /// `GLOBAL_SET_I64(moduleName, name, value)`.
    pub const OPCODE_GLOBAL_SET_I64: FwOpcodeType = 8;
    /// `GLOBAL_SET_F32(moduleName, name, value)`.
    pub const OPCODE_GLOBAL_SET_F32: FwOpcodeType = 9;
    /// `GLOBAL_SET_F64(moduleName, name, value)`.
    pub const OPCODE_GLOBAL_SET_F64: FwOpcodeType = 10;
    /// `GLOBAL_GET(moduleName, name)`.
    pub const OPCODE_GLOBAL_GET: FwOpcodeType = 11;
    /// `HOST_FUNCTION_TIMEOUT_SECS_PRM_SET(val: F32)`.
    pub const OPCODE_HOST_FUNCTION_TIMEOUT_SECS_PRM_SET: FwOpcodeType = 12;
    /// `HOST_FUNCTION_TIMEOUT_SECS_PRM_SAVE()`.
    pub const OPCODE_HOST_FUNCTION_TIMEOUT_SECS_PRM_SAVE: FwOpcodeType = 13;
    /// `INSTRUCTION_FUEL_PRM_SET(val: FwSizeType)`.
    pub const OPCODE_INSTRUCTION_FUEL_PRM_SET: FwOpcodeType = 14;
    /// `INSTRUCTION_FUEL_PRM_SAVE()`.
    pub const OPCODE_INSTRUCTION_FUEL_PRM_SAVE: FwOpcodeType = 15;
    /// `SEQ_BASE_DIR_PRM_SET(val: string size 240)`.
    pub const OPCODE_SEQ_BASE_DIR_PRM_SET: FwOpcodeType = 16;
    /// `SEQ_BASE_DIR_PRM_SAVE()`.
    pub const OPCODE_SEQ_BASE_DIR_PRM_SAVE: FwOpcodeType = 17;

    /// `StoreAllocationSucceeded(moduleCount)` — DIAGNOSTIC.
    pub const EVENTID_STORE_ALLOCATION_SUCCEEDED: FwEventIdType = 0;
    /// `ControllerCannotInvoke` — WARNING_LO.
    pub const EVENTID_CONTROLLER_CANNOT_INVOKE: FwEventIdType = 1;
    /// `ControllerBusy(signalSource, state)` — WARNING_LO.
    pub const EVENTID_CONTROLLER_BUSY: FwEventIdType = 2;
    /// `ModuleNotFound(module_name)` — WARNING_LO.
    pub const EVENTID_MODULE_NOT_FOUND: FwEventIdType = 3;
    /// `MemoryGrowRejected(reason, requestedSize)` — WARNING_LO.
    pub const EVENTID_MEMORY_GROW_REJECTED: FwEventIdType = 4;
    /// `GlobalSetFailed(module_name, globalName, reason)` — WARNING_LO.
    pub const EVENTID_GLOBAL_SET_FAILED: FwEventIdType = 5;
    /// `GlobalGetFailed(module_name, globalName, reason)` — WARNING_LO.
    pub const EVENTID_GLOBAL_GET_FAILED: FwEventIdType = 6;
    /// `GlobalValueI32(module_name, globalName, value)` — ACTIVITY_LO.
    pub const EVENTID_GLOBAL_VALUE_I32: FwEventIdType = 7;
    /// `GlobalValueI64(module_name, globalName, value)` — ACTIVITY_LO.
    pub const EVENTID_GLOBAL_VALUE_I64: FwEventIdType = 8;
    /// `GlobalValueF32(module_name, globalName, value)` — ACTIVITY_LO.
    pub const EVENTID_GLOBAL_VALUE_F32: FwEventIdType = 9;
    /// `GlobalValueF64(module_name, globalName, value)` — ACTIVITY_LO.
    pub const EVENTID_GLOBAL_VALUE_F64: FwEventIdType = 10;
    /// `FileOpenError(fileName, status)` — WARNING_HI.
    pub const EVENTID_FILE_OPEN_ERROR: FwEventIdType = 11;
    /// `SequenceFilePathTooLong(baseDir, fileName)` — WARNING_HI.
    pub const EVENTID_SEQUENCE_FILE_PATH_TOO_LONG: FwEventIdType = 12;
    /// `SequenceFilePathNotContained(baseDir, fileName)` — WARNING_HI.
    pub const EVENTID_SEQUENCE_FILE_PATH_NOT_CONTAINED: FwEventIdType = 13;
    /// `ModuleLoadFailed(status)` — WARNING_HI.
    pub const EVENTID_MODULE_LOAD_FAILED: FwEventIdType = 14;
    /// `ModuleMainInvokeFailed(status)` — WARNING_HI.
    pub const EVENTID_MODULE_MAIN_INVOKE_FAILED: FwEventIdType = 15;
    /// `ModuleStartInvokeFailed(status)` — WARNING_HI.
    pub const EVENTID_MODULE_START_INVOKE_FAILED: FwEventIdType = 16;
    /// `TooManyBlockingCommands` — WARNING_HI.
    pub const EVENTID_TOO_MANY_BLOCKING_COMMANDS: FwEventIdType = 17;
    /// `BufferTooLarge(host_function, size, maxSize)` — WARNING_HI.
    pub const EVENTID_BUFFER_TOO_LARGE: FwEventIdType = 18;
    /// `BufferTooSmall(host_function, size, valueSize)` — WARNING_HI.
    pub const EVENTID_BUFFER_TOO_SMALL: FwEventIdType = 19;
    /// `HostFunctionInvalidPointer(host_function, code)` — WARNING_HI.
    pub const EVENTID_HOST_FUNCTION_INVALID_POINTER: FwEventIdType = 20;
    /// `InvalidBlockingTypeValue(invalidValue)` — WARNING_HI.
    pub const EVENTID_INVALID_BLOCKING_TYPE_VALUE: FwEventIdType = 21;
    /// `HostFunctionInvalidPort(host_function, index, numPorts)` — WARNING_HI.
    pub const EVENTID_HOST_FUNCTION_INVALID_PORT: FwEventIdType = 22;
    /// `SerialPortSendFailed(host_function, status)` — WARNING_HI.
    pub const EVENTID_SERIAL_PORT_SEND_FAILED: FwEventIdType = 23;
    /// `SerialInFrameTooLarge(portIndex, size, maxSize)` — WARNING_HI.
    pub const EVENTID_SERIAL_IN_FRAME_TOO_LARGE: FwEventIdType = 24;
    /// `HostFunctionInvalidSeverity(raw, msg)` — WARNING_HI.
    pub const EVENTID_HOST_FUNCTION_INVALID_SEVERITY: FwEventIdType = 25;
    /// `SequenceStarting(index)` — ACTIVITY_HI.
    pub const EVENTID_SEQUENCE_STARTING: FwEventIdType = 26;
    /// `SequenceSucceeded(index)` — ACTIVITY_HI.
    pub const EVENTID_SEQUENCE_SUCCEEDED: FwEventIdType = 27;
    /// `SequenceCancelled(index, phase)` — ACTIVITY_HI.
    pub const EVENTID_SEQUENCE_CANCELLED: FwEventIdType = 28;
    /// `SequenceExited(index, phase, code)` — WARNING_HI.
    pub const EVENTID_SEQUENCE_EXITED: FwEventIdType = 29;
    /// `SequencePanic(index, phase, code)` — WARNING_HI.
    pub const EVENTID_SEQUENCE_PANIC: FwEventIdType = 30;
    /// `SequenceTrapped(index, phase, reason)` — WARNING_HI.
    pub const EVENTID_SEQUENCE_TRAPPED: FwEventIdType = 31;
    /// `SequenceHostFailure(index, phase, reason, hostFunction)` — WARNING_HI.
    pub const EVENTID_SEQUENCE_HOST_FAILURE: FwEventIdType = 32;
    /// `SequencePaused()` — ACTIVITY_HI.
    pub const EVENTID_SEQUENCE_PAUSED: FwEventIdType = 33;
    /// `SequenceNotRunning()` — WARNING_LO.
    pub const EVENTID_SEQUENCE_NOT_RUNNING: FwEventIdType = 34;
    /// `SleepDurationTooLarge(host_function, micros)` — WARNING_HI.
    pub const EVENTID_SLEEP_DURATION_TOO_LARGE: FwEventIdType = 35;
    /// `HostFunctionInvalidId(host_function, id)` — WARNING_HI.
    pub const EVENTID_HOST_FUNCTION_INVALID_ID: FwEventIdType = 36;
    /// `CmdResponseFromOldSequence(opcode, response, oldSequenceIdx,
    /// currentSequenceIdx)` — WARNING_LO.
    pub const EVENTID_CMD_RESPONSE_FROM_OLD_SEQUENCE: FwEventIdType = 37;
    /// `WrongCmdResponseIndex(opcode, response, actualCmdIdx,
    /// expectedCmdIdx)` — WARNING_HI.
    pub const EVENTID_WRONG_CMD_RESPONSE_INDEX: FwEventIdType = 38;
    /// `InvalidModuleEntrypoint(moduleIdx, status)` — WARNING_HI.
    pub const EVENTID_INVALID_MODULE_ENTRYPOINT: FwEventIdType = 39;
    /// `LogWarningHi(msg)` — WARNING_HI.
    pub const EVENTID_LOG_WARNING_HI: FwEventIdType = 40;
    /// `LogWarningLo(msg)` — WARNING_LO.
    pub const EVENTID_LOG_WARNING_LO: FwEventIdType = 41;
    /// `LogActivityHi(msg)` — ACTIVITY_HI.
    pub const EVENTID_LOG_ACTIVITY_HI: FwEventIdType = 42;
    /// `LogActivityLo(msg)` — ACTIVITY_LO.
    pub const EVENTID_LOG_ACTIVITY_LO: FwEventIdType = 43;
    /// `LogDiagnostic(msg)` — DIAGNOSTIC.
    pub const EVENTID_LOG_DIAGNOSTIC: FwEventIdType = 44;

    /// `ControllerState: ControllerStateMachine.State` — update on change.
    pub const CHANID_CONTROLLER_STATE: FwChanIdType = 0;
    /// `InterpreterState: InterpreterStateMachine.State` — update on change.
    pub const CHANID_INTERPRETER_STATE: FwChanIdType = 1;
    /// `SequencesSucceeded: U64` — update on change.
    pub const CHANID_SEQUENCES_SUCCEEDED: FwChanIdType = 2;
    /// `SequencesFailed: U64` — update on change.
    pub const CHANID_SEQUENCES_FAILED: FwChanIdType = 3;
    /// `SequencesCancelled: U64` — update on change.
    pub const CHANID_SEQUENCES_CANCELLED: FwChanIdType = 4;
    /// `CommandsDispatched: U64` — update on change.
    pub const CHANID_COMMANDS_DISPATCHED: FwChanIdType = 5;
    /// `CommandsFailed: U64` — update on change.
    pub const CHANID_COMMANDS_FAILED: FwChanIdType = 6;
    /// `LastTrapReason: TrapReason` — update on change.
    pub const CHANID_LAST_TRAP_REASON: FwChanIdType = 7;
    /// `SeqName: string size FileNameStringSize` — update on change.
    pub const CHANID_SEQ_NAME: FwChanIdType = 8;

    /// `HOST_FUNCTION_TIMEOUT_SECS: F32 default 60`.
    pub const PARAMID_HOST_FUNCTION_TIMEOUT_SECS: FwPrmIdType = 0;
    /// `INSTRUCTION_FUEL: FwSizeType default Wasm.DEFAULT_INSTRUCTION_FUEL`.
    pub const PARAMID_INSTRUCTION_FUEL: FwPrmIdType = 1;
    /// `SEQ_BASE_DIR: string size FileNameStringSize default
    /// Wasm.DEFAULT_SEQ_BASE_DIR`.
    pub const PARAMID_SEQ_BASE_DIR: FwPrmIdType = 2;

    /// The FPP default of `HOST_FUNCTION_TIMEOUT_SECS`.
    pub const HOST_FUNCTION_TIMEOUT_SECS_DEFAULT: f32 = 60.0;
}

// ---------------------------------------------------------------------------
// Configuration.
// ---------------------------------------------------------------------------

/// What `serialIn` does when a frame does not fit its queue (C++
/// `SerialInQueueFullBehavior`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SerialInQueueFullBehavior {
    /// Drop the new frame (`DROP_NEWEST`).
    #[default]
    DropNewest,
    /// Drop queued frames, oldest first, until the new one fits
    /// (`DROP_OLDEST`).
    DropOldest,
    /// Assert (`ASSERT`).
    Assert,
}

/// One `serialIn` queue (C++ `SerialInQueueConfig`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SerialInQueueConfig {
    /// Queue size in bytes; 0 means no queue (every inbound frame on that
    /// index is dropped).
    pub size: FwSizeType,
    /// What to do when a frame does not fit.
    pub full_behavior: SerialInQueueFullBehavior,
}

/// Memory configuration (C++ `WasmSequencer::Config`; see the sizing guide
/// in upstream's `docs/sdd.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Interpreter heap pages (store, modules, IR, operand stack), each
    /// `SPACEWASM_PAGE_SIZE` bytes; at most `SPACEWASM_MAX_PAGES`.
    pub heap_pages: FwSizeType,
    /// Guest linear-memory pool shared by every loaded module, in bytes.
    pub guest_memory_size: FwSizeType,
    /// Wasm operand stack, in 32-bit words (allocated from the heap).
    pub stack_size: FwSizeType,
    /// Capacity of the IR code-page table (512-byte pages are allocated
    /// lazily from the heap).
    pub max_code_pages: FwSizeType,
    /// Maximum number of modules in the store.
    pub max_guest_modules: u8,
    /// Size of the `serial_send` copy-out buffer; 0 disables `serial_send`.
    pub serial_out_max: FwSizeType,
    /// Per-port `serialIn` queues.
    pub serial_in: [SerialInQueueConfig; NUM_SERIAL_IN_PORTS],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            heap_pages: 8,
            guest_memory_size: 8192,
            stack_size: 1024,
            max_code_pages: 256,
            max_guest_modules: 8,
            serial_out_max: 0,
            serial_in: [SerialInQueueConfig::default(); NUM_SERIAL_IN_PORTS],
        }
    }
}

// ---------------------------------------------------------------------------
// Component state.
// ---------------------------------------------------------------------------

/// The arguments of a pending host function call (C++
/// `PendingHostFunction::Args`, a union selected by `kind`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PendingArgs {
    /// No arguments recorded.
    #[default]
    None,
    /// `cmd`: the encoded command in guest memory.
    Command {
        /// Guest address.
        ptr: u32,
        /// Length in bytes.
        len: u32,
    },
    /// `tlm`: the channel and where to write its time and value.
    Telemetry {
        /// Channel id.
        chan_id: FwChanIdType,
        /// Guest address of the time.
        time_ptr: u32,
        /// Size of the time buffer.
        time_len: u32,
        /// Guest address of the value.
        value_ptr: u32,
        /// Size of the value buffer.
        value_len: u32,
    },
    /// `prm`: the parameter and where to write its value.
    Parameter {
        /// Parameter id.
        prm_id: FwPrmIdType,
        /// Guest address.
        ptr: u32,
        /// Size of the buffer.
        len: u32,
    },
    /// `event`: the raw severity and the message.
    Event {
        /// Guest-requested severity, unvalidated.
        raw_severity: u32,
        /// Guest address of the message.
        msg_ptr: u32,
        /// Message length, capped at `FW_LOG_STRING_MAX_SIZE`.
        msg_len: u32,
    },
    /// `rsleep`: a relative duration.
    Rsleep {
        /// Microseconds.
        us: u64,
    },
    /// `asleep`: an absolute time.
    Asleep {
        /// Microseconds since the epoch.
        us: u64,
    },
    /// `args`: where to write the sequence arguments.
    Args {
        /// Guest address.
        ptr: u32,
        /// Size of the buffer.
        len: u32,
    },
    /// `time`: where to write the current time.
    Time {
        /// Guest address.
        ptr: u32,
        /// Size of the buffer.
        len: u32,
    },
    /// `serial_send`: the port and the payload.
    SerialOut {
        /// `serialOut` port index.
        index: u32,
        /// Guest address.
        ptr: u32,
        /// Payload length.
        len: u32,
    },
    /// `serial_recv`: the queue and where to write a message.
    SerialRecv {
        /// `serialIn` port index.
        index: u32,
        /// Guest address of the data buffer.
        data_ptr: u32,
        /// Size of the data buffer.
        data_size: u32,
        /// Guest address of the received size.
        actual_size_ptr: u32,
        /// Block until a message arrives.
        blocking_type: BlockState,
    },
}

/// A host function call the guest made, waiting for dispatch (C++
/// `PendingHostFunction`; the C++ `caller` handle is not needed: guest
/// memory is reached through the store).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PendingHostFunction {
    /// Which host function (`NONE`: nothing pending).
    pub kind: HostFunction,
    /// Its arguments (kept when `kind` is cleared, as the C++ union is).
    pub args: PendingArgs,
}

impl PendingHostFunction {
    /// C++ `isPending`.
    #[must_use]
    pub fn is_pending(&self) -> bool {
        self.kind != HostFunction::None
    }
}

/// C++ `ExitStatus`: why the last sequence ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExitStatus {
    /// Why the program exited.
    pub reason: ExitReason,
    /// The exit code (main's result or the `exit` / `panic` code).
    pub code: i32,
    /// The last host function dispatched.
    pub last_host_function: HostFunction,
    /// The last trap reason.
    pub last_trap_reason: TrapReason,
}

impl Default for ExitStatus {
    fn default() -> Self {
        Self {
            reason: ExitReason::Unknown,
            code: 0,
            last_host_function: HostFunction::None,
            last_trap_reason: TrapReason::None,
        }
    }
}

/// C++ `Telemetry`: the periodically written counters.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Telemetry {
    /// Sequences completed successfully.
    pub sequences_succeeded: u64,
    /// Sequences that failed to validate or execute.
    pub sequences_failed: u64,
    /// Sequences cancelled.
    pub sequences_cancelled: u64,
    /// Commands dispatched.
    pub commands_dispatched: u64,
    /// Commands that failed.
    pub commands_failed: u64,
    /// The running sequence's name.
    pub sequence_name: FileNameString,
}

/// A `WAIT` command waiting for the sequence to finish (C++ `WaitingCmd`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WaitingCmd {
    /// Opcode.
    pub op_code: FwOpcodeType,
    /// Command sequence number.
    pub cmd_seq: u32,
}

/// The C++ `m_*` members touched on the component thread.
#[derive(Debug)]
pub struct WasmState {
    /// C++ `m_waiting` (capacity `MAX_CONCURRENT_WAIT_COMMANDS`).
    pub waiting: VecDeque<WaitingCmd>,
    /// C++ `m_args`.
    pub args: SeqArgs,
    /// C++ `m_lastLoadFileName`.
    pub last_load_file_name: FileNameString,
    /// C++ `m_executingContext` / `m_hasExecutingContext`.
    pub executing_context: Option<RequestContext>,
    /// C++ `m_pendingTimer` / `m_hasPendingTimer`.
    pub pending_timer: Option<Time>,
    /// C++ `m_hostFunctionStart` / `m_hasHostFunctionStart`.
    pub host_function_start: Option<Time>,
    /// C++ `m_dequeueSucceeded`.
    pub dequeue_succeeded: bool,
    /// C++ `m_invokeStatus`.
    pub invoke_status: Status,
    /// C++ `m_pendingPause`.
    pub pending_pause: bool,
    /// C++ `m_cancelRequested`.
    pub cancel_requested: bool,
    /// C++ `m_tlm`.
    pub tlm: Telemetry,
    /// C++ `m_sequencesStarted`.
    pub sequences_started: u32,
    /// C++ `m_exit`.
    pub exit: ExitStatus,
    /// C++ `m_pendingHostFunction`.
    pub pending: PendingHostFunction,
    /// C++ `m_serialOutBuffer` (`serial_out_max` bytes; empty when 0).
    pub serial_out_buffer: Vec<u8>,
}

impl WasmState {
    fn new() -> Self {
        Self {
            waiting: VecDeque::with_capacity(MAX_CONCURRENT_WAIT_COMMANDS),
            args: SeqArgs::default(),
            last_load_file_name: FileNameString::new(),
            executing_context: None,
            pending_timer: None,
            host_function_start: None,
            dequeue_succeeded: false,
            invoke_status: Status::Ok,
            pending_pause: false,
            cancel_requested: false,
            tlm: Telemetry::default(),
            sequences_started: 0,
            exit: ExitStatus::default(),
            pending: PendingHostFunction::default(),
            serial_out_buffer: Vec::new(),
        }
    }

    /// C++ `makeCmdUid`: `XXYY` with the low 16 bits of the sequence and
    /// command counters.
    #[must_use]
    pub fn make_cmd_uid(&self) -> u32 {
        ((self.sequences_started & 0xFFFF) << 16) | (self.tlm.commands_dispatched & 0xFFFF) as u32
    }
}

/// The `serialIn` queues (C++ `m_serialInQueue`, guarded by
/// `m_serialInMutex`). A port configured with size 0 has no queue.
#[derive(Debug, Default)]
pub struct SerialIn {
    /// One queue per port, framed `[U32 length][payload]`.
    pub queues: [Option<CircularBuffer>; NUM_SERIAL_IN_PORTS],
    /// The overflow policy per port (C++ `m_config.serialIn[i].fullBehavior`).
    pub behaviors: [SerialInQueueFullBehavior; NUM_SERIAL_IN_PORTS],
}

/// The parameter store (the generated `m_<param>` members and validity,
/// guarded by the generated `m_paramLock`). Zero until `load_parameters`,
/// as the generated members are.
#[derive(Debug)]
struct WasmParams {
    host_function_timeout_secs: f32,
    host_function_timeout_secs_valid: ParamValid,
    instruction_fuel: FwSizeType,
    instruction_fuel_valid: ParamValid,
    seq_base_dir: ParamString,
    seq_base_dir_valid: ParamValid,
}

/// The "update on change" telemetry cache.
#[derive(Debug, Default)]
struct TlmLast {
    controller_state: Option<ControllerState>,
    interpreter_state: Option<InterpreterState>,
    sequences_succeeded: Option<u64>,
    sequences_failed: Option<u64>,
    sequences_cancelled: Option<u64>,
    commands_dispatched: Option<u64>,
    commands_failed: Option<u64>,
    last_trap_reason: Option<TrapReason>,
    seq_name: Option<FileNameString>,
}

/// Lock a mutex, recovering the data if a thread panicked while holding
/// it: the state is consistent between statements, and teardown after a
/// failed assertion must still be able to release the store.
fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// C++ `getGlobalAllocatorLock`: serializes every store operation of every
/// instance, because spacewasm's heap is process-wide.
static GLOBAL_ALLOCATOR_LOCK: Mutex<()> = Mutex::new(());

// ---------------------------------------------------------------------------
// The component.
// ---------------------------------------------------------------------------

/// `Svc::WasmSequencer` — the WebAssembly sequencer.
pub struct WasmSequencer {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// `cmdRegOut` / `cmdResponseOut`.
    pub cmd: CmdGlue,
    /// `logOut` / `logTextOut` / `timeCaller`.
    pub evt: EventGlue,
    /// `tlmOut`.
    pub tlm: TlmGlue,
    /// `prmGet` / `prmSet`.
    pub prm: PrmGlue,
    /// `cmdOut`: `Fw.Com` out — the commands a sequence sends.
    pub cmd_out: OutputPort<dyn ComPort>,
    /// `getTlmChan`: `Fw.TlmGet` out — the `tlm` host function.
    pub get_tlm_chan: OutputPort<dyn TlmGetPort>,
    /// `getParam`: `Fw.PrmGet` out — the `prm` host function.
    pub get_param: OutputPort<dyn PrmGetPort>,
    /// `serialOut[MAX_SERIAL_OUT_PORTS]`: `serial` out — `serial_send`.
    pub serial_out: [OutputPort<dyn SerializePort>; NUM_SERIAL_OUT_PORTS],
    /// `seqStartOut`: `Svc.CmdSeqIn` out — a `RUN` started.
    pub seq_start_out: OutputPort<dyn CmdSeqInPort>,
    /// `seqDoneOut`: `Fw.CmdResponse` out — a `RUN` finished.
    pub seq_done_out: OutputPort<dyn CmdResponsePort>,
    /// The state machine instance `controller`.
    controller: controller_sm::StateMachine,
    /// The state machine instance `interpreter`.
    interpreter: interpreter_sm::StateMachine,
    /// C++ `m_wasm` (and `m_guest_allocator`): the store. Declared before
    /// the pools so it is dropped first.
    engine: Mutex<Option<WasmEngine>>,
    /// C++ `m_heapPages` and the global-allocator registration.
    heap: Mutex<Option<Heap>>,
    /// C++ `m_guestPool`.
    guest: Mutex<Option<Arc<GuestPool>>>,
    /// The other `m_*` members.
    state: Mutex<WasmState>,
    /// The `serialIn` queues (C++ `m_serialInMutex`).
    serial_in: Mutex<SerialIn>,
    /// C++ `m_config` (`Some` once configured, standing in for
    /// `m_allocator != nullptr`).
    config: Mutex<Option<Config>>,
    params: Mutex<WasmParams>,
    tlm_last: Mutex<TlmLast>,
    /// This component, for the host functions and the guest allocator
    /// observer the store holds.
    weak_self: Weak<WasmSequencer>,
}

impl WasmSequencer {
    /// Construct (C++ constructor). Follow with `set_id_base`, wiring,
    /// [`Self::init`], [`Self::configure`], [`Self::reg_commands`],
    /// [`Self::load_parameters`] and `active.start`.
    #[must_use]
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new_cyclic(|weak| Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            prm: PrmGlue::new(),
            cmd_out: OutputPort::new(),
            get_tlm_chan: OutputPort::new(),
            get_param: OutputPort::new(),
            serial_out: std::array::from_fn(|_| OutputPort::new()),
            seq_start_out: OutputPort::new(),
            seq_done_out: OutputPort::new(),
            controller: controller_sm::StateMachine::new(),
            interpreter: interpreter_sm::StateMachine::new(),
            engine: Mutex::new(None),
            heap: Mutex::new(None),
            guest: Mutex::new(None),
            state: Mutex::new(WasmState::new()),
            serial_in: Mutex::new(SerialIn::default()),
            config: Mutex::new(None),
            params: Mutex::new(WasmParams {
                host_function_timeout_secs: 0.0,
                host_function_timeout_secs_valid: ParamValid::Uninit,
                instruction_fuel: 0,
                instruction_fuel_valid: ParamValid::Uninit,
                seq_base_dir: ParamString::new(),
                seq_base_dir_valid: ParamValid::Uninit,
            }),
            tlm_last: Mutex::new(TlmLast::default()),
            weak_self: weak.clone(),
        })
    }

    /// C++ `init(queueDepth, instance)`: initialize the state machines
    /// (their initial transitions enter `IDLE`; the store does not exist
    /// yet), then create the queue — in that order, as the generated code
    /// does.
    pub fn init(&self, queue_depth: FwSizeType) {
        self.controller.init(Self::SM_ID_CONTROLLER, self);
        self.interpreter.init(Self::SM_ID_INTERPRETER, self);
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_RUN,
                Self::OPCODE_LOAD,
                Self::OPCODE_INVOKE,
                Self::OPCODE_WAIT,
                Self::OPCODE_CANCEL,
                Self::OPCODE_PAUSE,
                Self::OPCODE_CONTINUE,
                Self::OPCODE_GLOBAL_SET_I32,
                Self::OPCODE_GLOBAL_SET_I64,
                Self::OPCODE_GLOBAL_SET_F32,
                Self::OPCODE_GLOBAL_SET_F64,
                Self::OPCODE_GLOBAL_GET,
                Self::OPCODE_HOST_FUNCTION_TIMEOUT_SECS_PRM_SET,
                Self::OPCODE_HOST_FUNCTION_TIMEOUT_SECS_PRM_SAVE,
                Self::OPCODE_INSTRUCTION_FUEL_PRM_SET,
                Self::OPCODE_INSTRUCTION_FUEL_PRM_SAVE,
                Self::OPCODE_SEQ_BASE_DIR_PRM_SET,
                Self::OPCODE_SEQ_BASE_DIR_PRM_SAVE,
            ],
        );
    }

    /// C++ `configure(cfg, allocator)` (required): allocate the interpreter
    /// heap, the guest memory pool, the `serial_send` buffer and the
    /// `serialIn` queues, register the heap with spacewasm's global
    /// allocator, and create the first store. All memory is allocated here
    /// (there is no `Fw::MemAllocator` in this port), once.
    ///
    /// Asserts when called twice, when `heap_pages` is 0 or above
    /// `SPACEWASM_MAX_PAGES`, when no global allocator slot is free, and
    /// when the store does not fit the heap.
    pub fn configure(&self, cfg: &Config) {
        fw_assert!(lock(&self.engine).is_none());
        fw_assert!(lock(&self.config).is_none());
        // C++ parity: the page count is checked against the static pool.
        fw_assert!(
            cfg.heap_pages > 0 && cfg.heap_pages <= SPACEWASM_MAX_PAGES,
            cfg.heap_pages as i64,
            SPACEWASM_MAX_PAGES as i64
        );
        let heap = {
            let _global = lock(&GLOBAL_ALLOCATOR_LOCK);
            Heap::register(cfg.heap_pages as usize)
        };
        let heap = match heap {
            Ok(heap) => heap,
            Err(e) => {
                fw_assert!(false, status::error_status(&e).as_repr());
                return;
            }
        };
        *lock(&self.heap) = Some(heap);
        *lock(&self.guest) = Some(GuestPool::new(
            cfg.guest_memory_size as usize,
            Box::new(host::GuestMemoryObserver {
                component: self.weak_self.clone(),
            }),
        ));
        if cfg.serial_out_max > 0 {
            lock(&self.state).serial_out_buffer = vec![0; cfg.serial_out_max as usize];
        }
        {
            let mut serial_in = lock(&self.serial_in);
            for (i, port) in cfg.serial_in.iter().enumerate() {
                if port.size > 0 {
                    serial_in.queues[i] = Some(CircularBuffer::new(port.size as usize));
                }
                serial_in.behaviors[i] = port.full_behavior;
            }
        }
        *lock(&self.config) = Some(*cfg);
        self.create_store();
    }

    /// C++ `deinit()`: destroy the store, release the pools and the global
    /// allocator slot. Tolerates a never-configured instance.
    pub fn deinit(&self) {
        if lock(&self.engine).is_some() {
            self.destroy_store();
        }
        {
            let _global = lock(&GLOBAL_ALLOCATOR_LOCK);
            drop(lock(&self.heap).take());
        }
        drop(lock(&self.guest).take());
        lock(&self.state).serial_out_buffer = Vec::new();
        *lock(&self.serial_in) = SerialIn::default();
        *lock(&self.config) = None;
    }

    /// The configuration given to [`Self::configure`].
    #[must_use]
    pub fn config(&self) -> Option<Config> {
        *lock(&self.config)
    }

    /// The controller machine's state (C++ `controller_getState`).
    #[must_use]
    pub fn controller_get_state(&self) -> ControllerState {
        self.controller.get_state()
    }

    /// The interpreter machine's state (C++ `interpreter_getState`).
    #[must_use]
    pub fn interpreter_get_state(&self) -> InterpreterState {
        self.interpreter.get_state()
    }

    /// The `controller` signal sender (C++ `controller_sendSignal_*`).
    fn ctl(&self) -> controller_sm::Sender<'_, QUEUE_MSG_SIZE> {
        controller_sm::Sender(SmSender::new(
            &self.active.queued,
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
            Self::SM_ID_CONTROLLER,
            PRIORITY,
            QueueFullPolicy::Assert,
        ))
    }

    /// The `interpreter` signal sender (C++ `interpreter_sendSignal_*`).
    fn itp(&self) -> interpreter_sm::Sender<'_, QUEUE_MSG_SIZE> {
        interpreter_sm::Sender(SmSender::new(
            &self.active.queued,
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL,
            Self::SM_ID_INTERPRETER,
            PRIORITY,
            QueueFullPolicy::Assert,
        ))
    }

    /// The component state (white-box access for tests and diagnostics).
    pub fn state(&self) -> MutexGuard<'_, WasmState> {
        lock(&self.state)
    }

    /// The `serialIn` queues (white-box access for tests).
    pub fn serial_in_queues(&self) -> MutexGuard<'_, SerialIn> {
        lock(&self.serial_in)
    }

    /// Whether a store exists (C++ `m_wasm != nullptr`).
    #[must_use]
    pub fn is_store_created(&self) -> bool {
        lock(&self.engine).is_some()
    }

    /// The interpreter heap's pages in use (C++ `m_heapPagesUsed`).
    #[must_use]
    pub fn heap_pages_used(&self) -> usize {
        lock(&self.heap).as_ref().map_or(0, Heap::pages_used)
    }

    /// The guest memory pool (white-box access for tests: offsets only).
    #[must_use]
    pub fn guest_pool(&self) -> Option<Arc<GuestPool>> {
        lock(&self.guest).clone()
    }

    /// Run `f` with this instance's heap current (C++ `takeAllocatorLock`
    /// … `releaseAllocatorLock`).
    fn with_heap<R>(&self, f: impl FnOnce(&HeapGuard<'_>) -> R) -> R {
        let _global = lock(&GLOBAL_ALLOCATOR_LOCK);
        let heap = lock(&self.heap);
        let Some(heap) = heap.as_ref() else {
            fw_assert!(false);
            unreachable!()
        };
        let guard = match heap.acquire() {
            Ok(guard) => guard,
            Err(e) => {
                fw_assert!(false, status::error_status(&e).as_repr());
                unreachable!()
            }
        };
        f(&guard)
    }

    /// C++ `createStore`: a fresh store with the `fprime_v1` host module.
    fn create_store(&self) {
        fw_assert!(lock(&self.engine).is_none());
        let Some(cfg) = self.config() else {
            fw_assert!(false);
            return;
        };
        let Some(guest) = self.guest_pool() else {
            fw_assert!(false);
            return;
        };
        let result = self.with_heap(|guard| {
            let mut host = match fprime_spacewasm::HostModules::new(guard, 1) {
                Ok(host) => host,
                Err(e) => return Err(e),
            };
            self.host_fprime_v1(&mut host);
            let options = fprime_spacewasm::spacewasm::CompilerOptions {
                // implemented in a restricted way (see the guest pool)
                allow_memory_grow: true,
                max_backpatch_iterations:
                    match fprime_config::wasm_sequencer::MAX_BACKPATCH_ITERATIONS {
                        0 => None,
                        n => Some(n),
                    },
                max_code_pages: cfg.max_code_pages as usize,
            };
            WasmEngine::new(
                guard,
                host,
                cfg.stack_size as usize,
                usize::from(cfg.max_guest_modules),
                options,
                &guest,
            )
        });
        // A failure means the heap is too small for this many modules plus
        // the operand stack (or a page is too small for the code-page
        // table): C++ asserts.
        match result {
            Ok(engine) => *lock(&self.engine) = Some(engine),
            Err(e) => fw_assert!(false, status::error_status(&e).as_repr()),
        }
        self.log_diagnostic_store_allocation_succeeded(u16::from(cfg.max_guest_modules));
    }

    /// C++ `destroyStore`.
    fn destroy_store(&self) {
        let engine = lock(&self.engine).take();
        let Some(engine) = engine else {
            fw_assert!(false);
            return;
        };
        self.with_heap(|guard| engine.destroy(guard));
        // Every heap page must be back.
        let used = self.heap_pages_used();
        fw_assert!(used == 0, used as i64);
        // Recycle the guest pool (every module memory went with the store)
        // and lift the page poison.
        if let Some(guest) = self.guest_pool() {
            fw_assert!(guest.reset());
        }
        if let Some(heap) = lock(&self.heap).as_ref() {
            fw_assert!(heap.clear_poison());
        }
    }

    /// Run `f` on the store, which must exist.
    fn with_engine<R>(&self, f: impl FnOnce(&mut WasmEngine) -> R) -> R {
        let mut engine = lock(&self.engine);
        let Some(engine) = engine.as_mut() else {
            fw_assert!(false);
            unreachable!()
        };
        f(engine)
    }

    /// C++ `FW_ASSERT(this->m_wasm != nullptr)`.
    fn assert_store(&self) {
        fw_assert!(self.is_store_created());
    }

    // -- Parameters ----------------------------------------------------------

    /// `paramGet_HOST_FUNCTION_TIMEOUT_SECS`.
    #[must_use]
    pub fn param_get_host_function_timeout_secs(&self) -> (f32, ParamValid) {
        let p = lock(&self.params);
        (
            p.host_function_timeout_secs,
            p.host_function_timeout_secs_valid,
        )
    }

    /// `paramGet_INSTRUCTION_FUEL`.
    #[must_use]
    pub fn param_get_instruction_fuel(&self) -> (FwSizeType, ParamValid) {
        let p = lock(&self.params);
        (p.instruction_fuel, p.instruction_fuel_valid)
    }

    /// `paramGet_SEQ_BASE_DIR`.
    #[must_use]
    pub fn param_get_seq_base_dir(&self) -> (ParamString, ParamValid) {
        let p = lock(&self.params);
        (p.seq_base_dir.clone(), p.seq_base_dir_valid)
    }

    /// The generated `loadParameters()`: fetch each parameter through
    /// `prmGet` (which must be connected), installing the FPP default when
    /// it is not `VALID` or does not deserialize. (No `parameterUpdated`
    /// override upstream.)
    pub fn load_parameters(&self) {
        let id_base = self.id_base();
        let mut buf = ParamBuffer::new();
        let valid = self
            .prm
            .get_param(id_base, Self::PARAMID_HOST_FUNCTION_TIMEOUT_SECS, &mut buf);
        let mut timeout = 0f32;
        let timeout_ok = valid == ParamValid::Valid && buf.deserialize_f32_be(&mut timeout).is_ok();
        {
            let mut p = lock(&self.params);
            if timeout_ok {
                p.host_function_timeout_secs = timeout;
                p.host_function_timeout_secs_valid = ParamValid::Valid;
            } else {
                p.host_function_timeout_secs = Self::HOST_FUNCTION_TIMEOUT_SECS_DEFAULT;
                p.host_function_timeout_secs_valid = ParamValid::Default;
            }
        }

        let mut buf = ParamBuffer::new();
        let valid = self
            .prm
            .get_param(id_base, Self::PARAMID_INSTRUCTION_FUEL, &mut buf);
        let mut fuel = 0u64;
        let fuel_ok = valid == ParamValid::Valid && buf.deserialize_u64_be(&mut fuel).is_ok();
        {
            let mut p = lock(&self.params);
            if fuel_ok {
                p.instruction_fuel = fuel;
                p.instruction_fuel_valid = ParamValid::Valid;
            } else {
                p.instruction_fuel = DEFAULT_INSTRUCTION_FUEL;
                p.instruction_fuel_valid = ParamValid::Default;
            }
        }

        let mut buf = ParamBuffer::new();
        let valid = self
            .prm
            .get_param(id_base, Self::PARAMID_SEQ_BASE_DIR, &mut buf);
        let mut base_dir = ParamString::new();
        let base_dir_ok =
            valid == ParamValid::Valid && buf.deserialize(&mut base_dir, Endianness::Big).is_ok();
        let mut p = lock(&self.params);
        if base_dir_ok {
            p.seq_base_dir = base_dir;
            p.seq_base_dir_valid = ParamValid::Valid;
        } else {
            p.seq_base_dir.set(DEFAULT_SEQ_BASE_DIR);
            p.seq_base_dir_valid = ParamValid::Default;
        }
    }

    /// The generated `<PARAM>_PRM_SET` handler: deserialize the value (a
    /// leftover byte is a format error) and store it as `VALID`.
    fn param_set(&self, id: FwPrmIdType, args: &mut CmdArgBuffer) -> CmdResponse {
        match id {
            Self::PARAMID_HOST_FUNCTION_TIMEOUT_SECS => {
                let mut value = 0f32;
                if !args.deserialize_f32_be(&mut value).is_ok() || args.deserialize_size_left() != 0
                {
                    return CmdResponse::FormatError;
                }
                let mut p = lock(&self.params);
                p.host_function_timeout_secs = value;
                p.host_function_timeout_secs_valid = ParamValid::Valid;
            }
            Self::PARAMID_INSTRUCTION_FUEL => {
                let mut value = 0u64;
                if !args.deserialize_u64_be(&mut value).is_ok() || args.deserialize_size_left() != 0
                {
                    return CmdResponse::FormatError;
                }
                let mut p = lock(&self.params);
                p.instruction_fuel = value;
                p.instruction_fuel_valid = ParamValid::Valid;
            }
            _ => {
                let mut value = ParamString::new();
                if !args.deserialize(&mut value, Endianness::Big).is_ok()
                    || args.deserialize_size_left() != 0
                {
                    return CmdResponse::FormatError;
                }
                let mut p = lock(&self.params);
                p.seq_base_dir = value;
                p.seq_base_dir_valid = ParamValid::Valid;
            }
        }
        CmdResponse::Ok
    }

    /// The generated `<PARAM>_PRM_SAVE` handler.
    fn param_save(&self, id: FwPrmIdType, args: &CmdArgBuffer) -> CmdResponse {
        if args.deserialize_size_left() != 0 {
            return CmdResponse::FormatError;
        }
        if !self.prm.prm_set_out.is_connected() {
            return CmdResponse::ExecutionError;
        }
        let mut buf = ParamBuffer::new();
        let status = {
            let p = lock(&self.params);
            match id {
                Self::PARAMID_HOST_FUNCTION_TIMEOUT_SECS => {
                    buf.serialize_f32_be(p.host_function_timeout_secs)
                }
                Self::PARAMID_INSTRUCTION_FUEL => buf.serialize_u64_be(p.instruction_fuel),
                _ => p.seq_base_dir.serialize_to(&mut buf, Endianness::Big),
            }
        };
        fw_assert!(status.is_ok(), status as i32);
        self.prm.set_param(self.id_base(), id, &mut buf);
        CmdResponse::Ok
    }
}

// ---------------------------------------------------------------------------
// Commands (WasmSequencer.cpp). They run on the component thread, except
// CANCEL and the parameter commands.
// ---------------------------------------------------------------------------

/// A module or global name as the C API sees it: the bytes up to the first
/// NUL (the string's content already stops there), which must be UTF-8.
pub(crate) fn c_str(bytes: &[u8]) -> Result<&str, Status> {
    std::str::from_utf8(bytes).map_err(|_| Status::ErrBadUtf8)
}

impl WasmSequencer {
    /// `RUN_cmdHandler`.
    fn run_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        file_name: &CmdStringArg,
        block: BlockState,
        seq_args: &SeqArgs,
    ) {
        self.assert_store();
        let mut name = FileName::new();
        name.set_bytes(file_name.as_bytes());
        self.ctl().send_signal_run(&LoadRequest::new(
            name,
            ModuleName::new(),
            *seq_args,
            // module_idx is a placeholder, filled in after the load
            RequestContext::new(
                SignalSource::CommandRun,
                CommandRequest::new(op_code, cmd_seq),
                block,
                0,
            ),
        ));
    }

    /// `WAIT_cmdHandler`.
    fn wait_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        self.assert_store();
        match self.controller_get_state() {
            // Nothing is executing, respond immediately
            ControllerState::IDLE | ControllerState::READY => {
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            }
            _ => {
                let queued = {
                    let mut st = lock(&self.state);
                    if st.waiting.len() < MAX_CONCURRENT_WAIT_COMMANDS {
                        st.waiting.push_back(WaitingCmd { op_code, cmd_seq });
                        true
                    } else {
                        false
                    }
                };
                if !queued {
                    self.log_warning_hi_too_many_blocking_commands();
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
                }
            }
        }
    }

    /// `LOAD_cmdHandler`: always blocks until the load (and start
    /// function) finishes.
    fn load_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        file_name: &CmdStringArg,
        name: &CmdStringArg,
    ) {
        self.assert_store();
        let mut file = FileName::new();
        file.set_bytes(file_name.as_bytes());
        let mut module = ModuleName::new();
        module.set_bytes(name.as_bytes());
        self.ctl().send_signal_load(&LoadRequest::new(
            file,
            module,
            SeqArgs::default(),
            RequestContext::new(
                SignalSource::CommandLoad,
                CommandRequest::new(op_code, cmd_seq),
                BlockState::Block,
                0,
            ),
        ));
    }

    /// `INVOKE_cmdHandler`.
    fn invoke_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        module: &CmdStringArg,
        block: BlockState,
        seq_args: &SeqArgs,
    ) {
        self.assert_store();
        let mut name = ModuleName::new();
        name.set_bytes(module.as_bytes());
        self.ctl().send_signal_invoke(&InvokeRequest::new(
            name,
            *seq_args,
            RequestContext::new(
                SignalSource::CommandInvoke,
                CommandRequest::new(op_code, cmd_seq),
                block,
                0,
            ),
        ));
    }

    /// `CANCEL_cmdHandler` (sync): cancel the controller and the
    /// interpreter; the interpreter answers the command.
    fn cancel_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        self.ctl().send_signal_cancel();
        self.itp()
            .send_signal_cmdCancel(&CommandRequest::new(op_code, cmd_seq));
    }

    /// `PAUSE_cmdHandler`: latch a pause taken before the engine spins
    /// again.
    fn pause_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        if self.interpreter_get_state() == InterpreterState::IDLE {
            self.log_warning_lo_sequence_not_running();
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        lock(&self.state).pending_pause = true;
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `CONTINUE_cmdHandler`.
    fn continue_cmd_handler(&self, op_code: FwOpcodeType, cmd_seq: u32) {
        self.assert_store();
        match self.interpreter_get_state() {
            // Already running
            InterpreterState::RUNNING_AWAITING_RESPONSE_SLEEPING
            | InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING
            | InterpreterState::RUNNING_SPINNING => {
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            }
            InterpreterState::RUNNING_PAUSED => {
                self.itp().send_signal_cmd_CONTINUE();
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            }
            InterpreterState::IDLE => {
                self.log_warning_lo_sequence_not_running();
                self.cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            }
            other => fw_assert!(false, other.as_repr()),
        }
    }

    /// The `GLOBAL_SET_*` handlers.
    fn global_set_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        module_name: &CmdStringArg,
        name: &CmdStringArg,
        value: fprime_spacewasm::spacewasm::Value,
    ) {
        self.assert_store();
        match self.set_global(module_name.as_bytes(), name.as_bytes(), value) {
            Ok(()) => self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok),
            Err(status) => {
                self.log_warning_lo_global_set_failed(
                    module_name.as_bytes(),
                    name.as_bytes(),
                    status,
                );
                self.cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            }
        }
    }

    /// `GLOBAL_GET_cmdHandler`: report the value in an event.
    fn global_get_cmd_handler(
        &self,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        module_name: &CmdStringArg,
        name: &CmdStringArg,
    ) {
        use fprime_spacewasm::spacewasm::Value;
        self.assert_store();
        match self.get_global(module_name.as_bytes(), name.as_bytes()) {
            Ok(value) => {
                let (m, n) = (module_name.as_bytes(), name.as_bytes());
                match value {
                    Value::I32(v) => self.log_activity_lo_global_value_i32(m, n, v),
                    Value::I64(v) => self.log_activity_lo_global_value_i64(m, n, v),
                    Value::F32(v) => self.log_activity_lo_global_value_f32(m, n, v),
                    Value::F64(v) => self.log_activity_lo_global_value_f64(m, n, v),
                }
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            }
            Err(status) => {
                self.log_warning_lo_global_get_failed(
                    module_name.as_bytes(),
                    name.as_bytes(),
                    status,
                );
                self.cmd
                    .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            }
        }
    }

    /// C++ `setGlobal`.
    fn set_global(
        &self,
        module_name: &[u8],
        name: &[u8],
        value: fprime_spacewasm::spacewasm::Value,
    ) -> Result<(), Status> {
        let module_name = c_str(module_name)?;
        let name = c_str(name)?;
        self.with_engine(|e| {
            let module_idx = e.find_module(module_name)?;
            let global_idx = e.find_global(module_idx, name)?;
            e.set_global(module_idx, global_idx, value)
        })
        .map_err(|e| status::error_status(&e))
    }

    /// C++ `getGlobal`.
    fn get_global(
        &self,
        module_name: &[u8],
        name: &[u8],
    ) -> Result<fprime_spacewasm::spacewasm::Value, Status> {
        let module_name = c_str(module_name)?;
        let name = c_str(name)?;
        self.with_engine(|e| {
            let module_idx = e.find_module(module_name)?;
            let global_idx = e.find_global(module_idx, name)?;
            e.get_global(module_idx, global_idx)
        })
        .map_err(|e| status::error_status(&e))
    }

    /// The generated async command dispatch: deserialize the arguments in
    /// declaration order (`FORMAT_ERROR` on a failure — an undeclared
    /// `BlockState` included — or a leftover byte) and call the handler.
    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        use fprime_spacewasm::spacewasm::Value;
        let respond = |response| self.cmd.cmd_response(op_code, cmd_seq, response);
        let local = op_code.wrapping_sub(self.id_base());
        macro_rules! de {
            ($ty:ty, $f:ident) => {{
                let mut v: $ty = Default::default();
                if !args.$f(&mut v).is_ok() {
                    respond(CmdResponse::FormatError);
                    return;
                }
                v
            }};
        }
        macro_rules! de_ser {
            ($ty:ty) => {{
                let mut v = <$ty>::default();
                if !args.deserialize(&mut v, Endianness::Big).is_ok() {
                    respond(CmdResponse::FormatError);
                    return;
                }
                v
            }};
        }
        macro_rules! done {
            () => {
                if args.deserialize_size_left() != 0 {
                    respond(CmdResponse::FormatError);
                    return;
                }
            };
        }
        match local {
            Self::OPCODE_RUN => {
                let file_name = de_ser!(CmdStringArg);
                let block = de_ser!(BlockState);
                let seq_args = de_ser!(SeqArgs);
                done!();
                self.run_cmd_handler(op_code, cmd_seq, &file_name, block, &seq_args);
            }
            Self::OPCODE_LOAD => {
                let file_name = de_ser!(CmdStringArg);
                let name = de_ser!(CmdStringArg);
                done!();
                self.load_cmd_handler(op_code, cmd_seq, &file_name, &name);
            }
            Self::OPCODE_INVOKE => {
                let module = de_ser!(CmdStringArg);
                let block = de_ser!(BlockState);
                let seq_args = de_ser!(SeqArgs);
                done!();
                self.invoke_cmd_handler(op_code, cmd_seq, &module, block, &seq_args);
            }
            Self::OPCODE_WAIT => {
                done!();
                self.wait_cmd_handler(op_code, cmd_seq);
            }
            Self::OPCODE_PAUSE => {
                done!();
                self.pause_cmd_handler(op_code, cmd_seq);
            }
            Self::OPCODE_CONTINUE => {
                done!();
                self.continue_cmd_handler(op_code, cmd_seq);
            }
            Self::OPCODE_GLOBAL_SET_I32
            | Self::OPCODE_GLOBAL_SET_I64
            | Self::OPCODE_GLOBAL_SET_F32
            | Self::OPCODE_GLOBAL_SET_F64 => {
                let module_name = de_ser!(CmdStringArg);
                let name = de_ser!(CmdStringArg);
                let value = match local {
                    Self::OPCODE_GLOBAL_SET_I32 => Value::I32(de!(i32, deserialize_i32_be)),
                    Self::OPCODE_GLOBAL_SET_I64 => Value::I64(de!(i64, deserialize_i64_be)),
                    Self::OPCODE_GLOBAL_SET_F32 => Value::F32(de!(f32, deserialize_f32_be)),
                    _ => Value::F64(de!(f64, deserialize_f64_be)),
                };
                done!();
                self.global_set_cmd_handler(op_code, cmd_seq, &module_name, &name, value);
            }
            Self::OPCODE_GLOBAL_GET => {
                let module_name = de_ser!(CmdStringArg);
                let name = de_ser!(CmdStringArg);
                done!();
                self.global_get_cmd_handler(op_code, cmd_seq, &module_name, &name);
            }
            _ => respond(CmdResponse::InvalidOpcode),
        }
    }

    /// `cmdIn` — the command port factory for wiring.
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(self.clone(), port_num)
    }
}

impl CmdPort for WasmSequencer {
    /// The generated `cmdIn` handler: `CANCEL` and the parameter commands
    /// run here, on the caller's thread; every other command is queued
    /// (priority 0, assert).
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let local = op_code.wrapping_sub(self.id_base());
        let response = match local {
            Self::OPCODE_CANCEL => {
                if args.deserialize_size_left() != 0 {
                    Some(CmdResponse::FormatError)
                } else {
                    self.cancel_cmd_handler(op_code, cmd_seq);
                    None
                }
            }
            Self::OPCODE_HOST_FUNCTION_TIMEOUT_SECS_PRM_SET => {
                Some(self.param_set(Self::PARAMID_HOST_FUNCTION_TIMEOUT_SECS, args))
            }
            Self::OPCODE_HOST_FUNCTION_TIMEOUT_SECS_PRM_SAVE => {
                Some(self.param_save(Self::PARAMID_HOST_FUNCTION_TIMEOUT_SECS, args))
            }
            Self::OPCODE_INSTRUCTION_FUEL_PRM_SET => {
                Some(self.param_set(Self::PARAMID_INSTRUCTION_FUEL, args))
            }
            Self::OPCODE_INSTRUCTION_FUEL_PRM_SAVE => {
                Some(self.param_save(Self::PARAMID_INSTRUCTION_FUEL, args))
            }
            Self::OPCODE_SEQ_BASE_DIR_PRM_SET => {
                Some(self.param_set(Self::PARAMID_SEQ_BASE_DIR, args))
            }
            Self::OPCODE_SEQ_BASE_DIR_PRM_SAVE => {
                Some(self.param_save(Self::PARAMID_SEQ_BASE_DIR, args))
            }
            Self::OPCODE_RUN..=Self::OPCODE_GLOBAL_GET => {
                let mut buf = LinearBuffer::<QUEUE_MSG_SIZE>::new();
                let status = msg::write_envelope_header(&mut buf, Self::MSG_TYPE_CMD_IN, port_num);
                fw_assert!(status.is_ok(), status as i32);
                let status = buf.serialize_u32_be(op_code);
                fw_assert!(status.is_ok(), status as i32);
                let status = buf.serialize_u32_be(cmd_seq);
                fw_assert!(status.is_ok(), status as i32);
                let status = buf.serialize_buffer(args, Endianness::Big);
                fw_assert!(status.is_ok(), status as i32);
                let _ = self
                    .active
                    .queued
                    .send_message(&buf, PRIORITY, QueueFullPolicy::Assert);
                None
            }
            _ => Some(CmdResponse::InvalidOpcode),
        };
        if let Some(response) = response {
            self.cmd.cmd_response(op_code, cmd_seq, response);
        }
    }
}

// ---------------------------------------------------------------------------
// Input ports (WasmSequencer.cpp).
// ---------------------------------------------------------------------------

impl WasmSequencer {
    /// `checkTimers_handler`: drive the sleep-wake / host-function-timeout
    /// checks.
    fn check_timers_handler(&self, _port_num: FwIndexType, _context: u32) {
        self.itp().send_signal_checkTimers();
    }

    /// `cmdResponseIn_handler`: match a command response against the
    /// command the sequence is waiting for, using the command UID echoed
    /// back as `cmd_seq`.
    fn cmd_response_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        self.assert_store();
        let cmd_uid = cmd_seq;
        let sequence_index = ((cmd_uid & 0xFFFF_0000) >> 16) as u16;
        let cmd_index = (cmd_uid & 0xFFFF) as u16;
        let (current_sequence_index, current_cmd_index, pending_kind) = {
            let st = lock(&self.state);
            (
                (st.sequences_started & 0xFFFF) as u16,
                (st.tlm.commands_dispatched & 0xFFFF) as u16,
                st.pending.kind,
            )
        };
        // A response from a previous execution window is a nominal late
        // reply (e.g. a command that returned after a CANCEL): report it
        // without failing the current sequence.
        if sequence_index != current_sequence_index {
            self.log_warning_lo_cmd_response_from_old_sequence(
                op_code,
                response,
                sequence_index,
                current_sequence_index,
            );
            return;
        }
        // From here on the response claims to be from the current
        // sequence, so any inconsistency fails it.
        if self.interpreter_get_state() != InterpreterState::RUNNING_AWAITING_RESPONSE_WAITING
            || pending_kind != HostFunction::Command
        {
            self.itp()
                .send_signal_hostResponseUnexpected(HostFunction::Command);
            return;
        }
        // Is it this exact dispatch, or an earlier one with the same opcode?
        if cmd_index != current_cmd_index {
            self.log_warning_hi_wrong_cmd_response_index(
                op_code,
                response,
                cmd_index,
                current_cmd_index,
            );
            self.itp()
                .send_signal_hostResponseUnexpected(HostFunction::Command);
            return;
        }
        {
            let mut st = lock(&self.state);
            st.pending.kind = HostFunction::None;
            if response != CmdResponse::Ok {
                st.tlm.commands_failed += 1;
            }
        }
        self.itp()
            .send_signal_hostResumeI32(i32::from(response.as_repr()));
    }

    /// `writeTelemetry_handler`: write every channel, time-stamped now.
    fn write_telemetry_handler(&self, _port_num: FwIndexType, _context: u32) {
        self.assert_store();
        let now = self.evt.time_get();
        let (tlm, last_trap) = {
            let st = lock(&self.state);
            (st.tlm.clone(), st.exit.last_trap_reason)
        };
        self.tlm_write_controller_state(self.controller_get_state(), now);
        self.tlm_write_interpreter_state(self.interpreter_get_state(), now);
        self.tlm_write_sequences_succeeded(tlm.sequences_succeeded, now);
        self.tlm_write_sequences_failed(tlm.sequences_failed, now);
        self.tlm_write_sequences_cancelled(tlm.sequences_cancelled, now);
        self.tlm_write_commands_dispatched(tlm.commands_dispatched, now);
        self.tlm_write_commands_failed(tlm.commands_failed, now);
        self.tlm_write_last_trap_reason(last_trap, now);
        self.tlm_write_seq_name(&tlm.sequence_name, now);
    }

    /// `seqRunIn_handler`: a non-blocking `RUN` from a port.
    fn seq_run_in_handler(
        &self,
        _port_num: FwIndexType,
        filename: &FileNameString,
        args: &SeqArgs,
    ) {
        self.assert_store();
        let mut name = FileName::new();
        name.set_bytes(filename.as_bytes());
        self.ctl().send_signal_run(&LoadRequest::new(
            name,
            ModuleName::new(),
            *args,
            RequestContext::new(
                SignalSource::PortRun,
                CommandRequest::new(0, 0),
                BlockState::NoBlock,
                0,
            ),
        ));
    }

    /// `seqCancelIn_handler` (sync).
    fn seq_cancel_in_handler(&self, _port_num: FwIndexType) {
        self.ctl().send_signal_cancel();
        self.itp().send_signal_cancel();
    }

    /// `seqCancelIn` — the port factory for wiring.
    pub fn seq_cancel_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdSeqCancelPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `serialIn` — the port factory for wiring (guarded).
    pub fn serial_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn SerializePort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `serialIn_handler`: frame the bytes onto the port's queue as
    /// `[U32 length][payload]` (applying the overflow policy) and wake a
    /// blocked `serial_recv`.
    pub fn serial_in_handler(&self, port_num: FwIndexType, payload: &[u8]) {
        fw_assert!(
            (0..NUM_SERIAL_IN_PORTS as FwIndexType).contains(&port_num),
            port_num,
            NUM_SERIAL_IN_PORTS as i64
        );
        let index = port_num as usize;
        let mut guard = lock(&self.serial_in);
        let serial_in = &mut *guard;
        let behavior = serial_in.behaviors[index];
        const HEADER: usize = size_of::<u32>();
        let payload_size = payload.len();
        let capacity = serial_in.queues[index]
            .as_ref()
            .map_or(0, CircularBuffer::get_capacity);
        // The queue must be able to hold this frame at all.
        if HEADER + payload_size > capacity {
            // A queue smaller than the header (or an unconfigured one) can
            // hold nothing.
            let max_payload = capacity.saturating_sub(HEADER);
            drop(guard);
            self.log_warning_hi_serial_in_frame_too_large(
                port_num as u32,
                payload_size as u32,
                max_payload as u32,
            );
            return;
        }
        let Some(queue) = serial_in.queues[index].as_mut() else {
            // capacity 0 was handled above
            return;
        };
        let frame_size = HEADER + payload_size;
        if frame_size > queue.get_free_size() {
            match behavior {
                SerialInQueueFullBehavior::DropOldest => {
                    // Drop the oldest messages until this one fits.
                    while frame_size > queue.get_free_size() {
                        let mut next_msg_size = 0u32;
                        let status = queue.peek_u32_be(&mut next_msg_size, 0);
                        fw_assert!(status.is_ok(), port_num, status as i32);
                        let allocated = queue.get_allocated_size();
                        fw_assert!(allocated >= HEADER, port_num, allocated as i64);
                        fw_assert!(
                            next_msg_size as usize <= allocated - HEADER,
                            port_num,
                            next_msg_size as i64,
                            allocated as i64
                        );
                        let status = queue.rotate(HEADER + next_msg_size as usize);
                        fw_assert!(status.is_ok(), port_num, status as i32);
                    }
                }
                SerialInQueueFullBehavior::DropNewest => return,
                SerialInQueueFullBehavior::Assert => {
                    fw_assert!(
                        false,
                        port_num,
                        frame_size as i64,
                        queue.get_free_size() as i64,
                        capacity as i64
                    );
                }
            }
        }
        // Enqueue the raw [U32 size][payload] frame.
        let status = queue.serialize(&(payload_size as u32).to_be_bytes());
        fw_assert!(status.is_ok(), status as i32);
        let status = queue.serialize(payload);
        fw_assert!(status.is_ok(), status as i32);
        drop(guard);
        // Wake any blocked serial_recv.
        self.itp().send_signal_serialInMessage(port_num);
    }
}

impl CmdSeqCancelPort for WasmSequencer {
    fn invoke(&self, port_num: FwIndexType) {
        self.seq_cancel_in_handler(port_num);
    }
}

impl SerializePort for WasmSequencer {
    /// The guarded `serialIn` port: the payload is the whole serialized
    /// buffer (C++ `getBuffAddr()` / `getSize()`).
    fn invoke_serial(&self, port_num: FwIndexType, buffer: &mut dyn SerBufAny) -> SerializeStatus {
        let len = buffer.ser_loc();
        self.serial_in_handler(port_num, &buffer.bytes()[..len]);
        SerializeStatus::Ok
    }
}

async_input_port_adapter! {
    /// `checkTimers` — ASYNC `Svc.Sched` input (assert).
    component: WasmSequencer;
    adapter: CheckTimersAdapter;
    port: SchedPort;
    input: pub check_timers;
    deserialize: check_timers_deserialize;
    handler: check_timers_handler;
    base: active.queued;
    msg_type: WasmSequencer::MSG_TYPE_CHECK_TIMERS;
    msg_size: QUEUE_MSG_SIZE;
    priority: 0;
    queue_full: QueueFullPolicy::Assert;
    args { val context: u32 }
}

async_input_port_adapter! {
    /// `cmdResponseIn` — ASYNC `Fw.CmdResponse` input (assert).
    component: WasmSequencer;
    adapter: CmdResponseInAdapter;
    port: CmdResponsePort;
    input: pub cmd_response_in;
    deserialize: cmd_response_in_deserialize;
    handler: cmd_response_in_handler;
    base: active.queued;
    msg_type: WasmSequencer::MSG_TYPE_CMD_RESPONSE_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: 0;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, val response: CmdResponse }
}

async_input_port_adapter! {
    /// `seqRunIn` — ASYNC `Svc.CmdSeqIn` input (assert).
    component: WasmSequencer;
    adapter: SeqRunInAdapter;
    port: CmdSeqInPort;
    input: pub seq_run_in;
    deserialize: seq_run_in_deserialize;
    handler: seq_run_in_handler;
    base: active.queued;
    msg_type: WasmSequencer::MSG_TYPE_SEQ_RUN_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: 0;
    queue_full: QueueFullPolicy::Assert;
    args { ref filename: FileNameString, ref args: SeqArgs }
}

async_input_port_adapter! {
    /// `writeTelemetry` — ASYNC `Svc.Sched` input (drop).
    component: WasmSequencer;
    adapter: WriteTelemetryAdapter;
    port: SchedPort;
    input: pub write_telemetry;
    deserialize: write_telemetry_deserialize;
    handler: write_telemetry_handler;
    base: active.queued;
    msg_type: WasmSequencer::MSG_TYPE_WRITE_TELEMETRY;
    msg_size: QUEUE_MSG_SIZE;
    priority: 0;
    queue_full: QueueFullPolicy::Drop;
    args { val context: u32 }
}

// ---------------------------------------------------------------------------
// Dispatch (the generated `doDispatch` switch).
// ---------------------------------------------------------------------------

impl ComponentDispatch for WasmSequencer {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            Self::MSG_TYPE_CHECK_TIMERS => match Self::check_timers_deserialize(buf) {
                Some((context,)) => {
                    self.check_timers_handler(port_num, context);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_CMD_RESPONSE_IN => match Self::cmd_response_in_deserialize(buf) {
                Some((op_code, cmd_seq, response)) => {
                    self.cmd_response_in_handler(port_num, op_code, cmd_seq, response);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_SEQ_RUN_IN => match Self::seq_run_in_deserialize(buf) {
                Some((filename, args)) => {
                    self.seq_run_in_handler(port_num, &filename, &args);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_WRITE_TELEMETRY => match Self::write_telemetry_deserialize(buf) {
                Some((context,)) => {
                    self.write_telemetry_handler(port_num, context);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_CMD_IN => {
                let mut op_code: FwOpcodeType = 0;
                let mut cmd_seq = 0u32;
                let mut args = CmdArgBuffer::new();
                if !buf.deserialize_u32_be(&mut op_code).is_ok()
                    || !buf.deserialize_u32_be(&mut cmd_seq).is_ok()
                    || !buf.deserialize_buffer(&mut args, Endianness::Big).is_ok()
                {
                    return MsgDispatchStatus::Error;
                }
                self.cmd_in_handler(port_num, op_code, cmd_seq, &mut args);
                MsgDispatchStatus::Ok
            }
            Self::MSG_TYPE_INTERNAL_STATE_MACHINE_SIGNAL => {
                let (sm_id, signal) = sm::read_signal_header(buf);
                match sm_id {
                    Self::SM_ID_CONTROLLER => self.controller.dispatch_signal(self, signal, buf),
                    Self::SM_ID_INTERPRETER => self.interpreter.dispatch_signal(self, signal, buf),
                    // C++ parity: an unknown state machine id asserts.
                    _ => fw_assert!(false, sm_id),
                }
                MsgDispatchStatus::Ok
            }
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for WasmSequencer {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

// ---------------------------------------------------------------------------
// Telemetry (the generated `tlmWrite_*`, all "update on change").
// ---------------------------------------------------------------------------

macro_rules! tlm_on_change {
    ($( $fn:ident, $field:ident, $chan:ident, $ty:ty; )*) => {
        impl WasmSequencer {
            $(
                #[doc = concat!("`tlmWrite_` for `", stringify!($chan), "` (update on change).")]
                fn $fn(&self, value: $ty, time: Time) {
                    {
                        let mut last = lock(&self.tlm_last);
                        if last.$field == Some(value) {
                            return;
                        }
                        last.$field = Some(value);
                    }
                    self.tlm.tlm_write(self.id_base(), Self::$chan, &value, time);
                }
            )*
        }
    };
}

tlm_on_change! {
    tlm_write_controller_state, controller_state, CHANID_CONTROLLER_STATE, ControllerState;
    tlm_write_interpreter_state, interpreter_state, CHANID_INTERPRETER_STATE, InterpreterState;
    tlm_write_sequences_succeeded, sequences_succeeded, CHANID_SEQUENCES_SUCCEEDED, u64;
    tlm_write_sequences_failed, sequences_failed, CHANID_SEQUENCES_FAILED, u64;
    tlm_write_sequences_cancelled, sequences_cancelled, CHANID_SEQUENCES_CANCELLED, u64;
    tlm_write_commands_dispatched, commands_dispatched, CHANID_COMMANDS_DISPATCHED, u64;
    tlm_write_commands_failed, commands_failed, CHANID_COMMANDS_FAILED, u64;
    tlm_write_last_trap_reason, last_trap_reason, CHANID_LAST_TRAP_REASON, TrapReason;
}

impl WasmSequencer {
    /// `tlmWrite_SeqName` (update on change): the string is truncated to
    /// `FW_TLM_STRING_MAX_SIZE` on the wire, as the generated code does.
    fn tlm_write_seq_name(&self, value: &FileNameString, time: Time) {
        {
            let mut last = lock(&self.tlm_last);
            if last.seq_name.as_ref() == Some(value) {
                return;
            }
            last.seq_name = Some(value.clone());
        }
        let mut truncated = fprime_fw::TlmString::new();
        truncated.set_bytes(value.as_bytes());
        self.tlm
            .tlm_write(self.id_base(), Self::CHANID_SEQ_NAME, &truncated, time);
    }
}
