# fprime-rust architecture

A Rust port of [F Prime (F´)](https://github.com/nasa/fprime), NASA JPL's
component-driven flight-software framework. This document is the binding
design contract for the port. The byte-level and behavioral ground truth is
the C++ implementation, summarized per subsystem in
[`docs/cpp-analysis/`](docs/cpp-analysis/) — when this document and those
analyses disagree on a wire format, the analyses (i.e. the C++ code) win.

## Goals and non-goals

**Goals**

1. **Wire compatibility** with the C++ framework and the fprime-gds: byte-exact
   serialization (big-endian, `0xFF`/`0x00` bools, U16 length prefixes), the
   F Prime frame protocol (`0xdeadbeef` start word + CRC32), and the
   command/event/telemetry packet formats.
2. **Semantic fidelity**: the passive/queued/active component model, priority
   message queues with FIFO-within-priority, guarded ports, rate groups,
   command dispatch/registration, event filtering, telemetry double buffering
   — behaving as the C++ components do, including their documented quirks.
3. **Idiomatic, safe Rust**: `#![forbid(unsafe_code)]` in every crate; status
   enums instead of exceptions (F Prime has no exceptions anyway); ownership
   instead of raw-pointer buffer passing; `std` threading primitives.
4. **Flight-software discipline**: no heap allocation in steady state (allocate
   at initialization, recycle after), bounded loops, explicit handling of every
   status, no `unwrap()` outside tests and init-time invariants.
5. **Zero third-party dependencies.** The framework uses only `std`.

**Non-goals (phase 1)**

- No FPP autocoder. Components hand-implement the contract the C++ autocoder
  generates, following the pattern in this document. A derive/proc-macro layer
  can come later.
- No `no_std` support yet (the OSAL keeps a clean seam for it).
- Not ported yet (see the status matrix in README.md): CmdSequencer,
  FileUplink/FileDownlink/FileManager, data products (Dp*), TlmPacketizer,
  CCSDS stack, GenericHub, ComLogger, state-machine autocoding, UART/I2C/SPI/GPIO
  drivers.

## Workspace layout

```
crates/
  fprime-config   # type aliases + constants from default/config (no deps)
  fprime-fw       # Fw: serialization, types, strings, time, buffers, packets, assert, logger
  fprime-os       # Os: task, mutex, queue (priority), file, filesystem, directory, console, rawtime
  fprime-utils    # Utils: CRC32 hash, CRC checker, circular buffer, types queue, rate limiter, token bucket
  fprime-comp     # Fw/Obj + Fw/Port + Fw/Comp: object/port/component model, cmd/evt/tlm glue
  fprime-svc      # Svc: rate groups, CmdDispatcher, EventManager, TlmChan, Health, comms stack, ...
  fprime-drv      # Drv: byte stream driver model, TCP client/server
  fprime-ref      # Reference deployment binary (Ref-style topology, GDS-compatible)
```

Dependency DAG (arrows point at dependencies):

```
fprime-fw    -> fprime-config
fprime-os    -> fprime-fw, fprime-config
fprime-utils -> fprime-fw, fprime-config
fprime-comp  -> fprime-fw, fprime-os, fprime-config
fprime-svc   -> fprime-comp, fprime-utils, (fw, os, config)
fprime-drv   -> fprime-comp, (fw, os, config)
fprime-ref   -> everything
```

Rust 2024 edition, `rust-version = "1.85"` floor, workspace lints:
`unsafe_code = "forbid"`, `clippy::all = "warn"`.

## fprime-config

One crate holding every project-configurable type alias and constant, exactly
mirroring `default/config/` in C++ (see `docs/cpp-analysis/fw-types.md`).
Everything is `pub type` / `pub const` so a project forks this crate to retune.

```rust
// Type aliases (unix platform defaults)
pub type FwSizeType = u64;          // unsigned, >= u32 range
pub type FwSignedSizeType = i64;
pub type FwIndexType = i16;         // SIGNED; port numbers; -1 = unset
pub type FwAssertArgType = i32;
pub type FwIdType = u32;
pub type FwOpcodeType = FwIdType;
pub type FwChanIdType = FwIdType;
pub type FwEventIdType = FwIdType;
pub type FwPrmIdType = FwIdType;
pub type FwDpIdType = FwIdType;
pub type FwDpPriorityType = u32;
pub type FwEnumStoreType = i32;     // serialization width of plain (non-FPP) enums
pub type FwSizeStoreType = u16;     // on-wire length prefix
pub type FwPacketDescriptorType = u16; // NOTE: U16 in this codebase (legacy fprime used U32)
pub type FwTimeBaseStoreType = u16;
pub type FwTimeContextStoreType = u8;
pub type FwTlmPacketizeIdType = u16;
pub type FwQueuePriorityType = u8;
pub type FwTaskPriorityType = u8;
pub type FwTaskIdType = i32;

// Constants (FpConstants.fpp / AcConstants.fpp defaults)
pub const FW_COM_BUFFER_MAX_SIZE: usize = 512;
pub const FW_CMD_ARG_BUFFER_MAX_SIZE: usize = 506;  // 512 - 4 (opcode) - 2 (descriptor)
pub const FW_LOG_BUFFER_MAX_SIZE: usize = 506;
pub const FW_TLM_BUFFER_MAX_SIZE: usize = 506;
pub const FW_PARAM_BUFFER_MAX_SIZE: usize = 506;
pub const FW_CMD_STRING_MAX_SIZE: usize = 40;
pub const FW_LOG_STRING_MAX_SIZE: usize = 200;
pub const FW_TLM_STRING_MAX_SIZE: usize = 40;
pub const FW_PARAM_STRING_MAX_SIZE: usize = 40;
pub const FW_LOG_TEXT_BUFFER_SIZE: usize = 256;
pub const FW_FIXED_LENGTH_STRING_SIZE: usize = 256;
pub const FW_OBJ_NAME_BUFFER_SIZE: usize = 80;
pub const FW_QUEUE_NAME_BUFFER_SIZE: usize = 80;
pub const FW_TASK_NAME_BUFFER_SIZE: usize = 80;
pub const FW_SERIALIZE_TRUE_VALUE: u8 = 0xFF;
pub const FW_SERIALIZE_FALSE_VALUE: u8 = 0x00;
pub const FW_ASSERT_TEXT_SIZE: usize = 256;
pub const FILE_NAME_STRING_SIZE: usize = 240;
// ... plus per-component config constants used by fprime-svc
//     (CMD_DISPATCHER_DISPATCH_TABLE_SIZE = 150, TLMCHAN_HASH_BUCKETS = 500, etc.)
```

Per-component config constants (from `default/config/*Cfg.hpp`) also live here,
in submodules (`config::cmd_dispatcher`, `config::tlm_chan`, ...), so
`fprime-svc` has a single override point.

## fprime-fw: serialization core

The heart of wire compatibility. See `docs/cpp-analysis/fw-types.md` for the
exact C++ semantics being reproduced.

### Status and mode enums (exact discriminants)

```rust
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerializeStatus {
    Ok = 0,
    FormatError = 1,
    NoRoomLeft = 2,
    DeserBufferEmpty = 3,
    DeserFormatError = 4,
    DeserSizeMismatch = 5,
    DeserTypeMismatch = 6,
    DeserImmutable = 7,        // reserved (const strings); kept for parity
    DeserInvalidData = 8,
    DiscardedExisting = 9,     // returned by drop-oldest queues; NOT an error
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Endianness { #[default] Big, Little }
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LengthMode { #[default] IncludeLength, OmitLength }
```

APIs return `SerializeStatus` directly (marked `#[must_use]`), not `Result` —
this keeps 1:1 porting of C++ status-flow (including `DiscardedExisting`,
which is a success-with-note). A `fw_try!` macro early-returns non-`Ok`
statuses.

### The buffer trait

`SerBuf` is the port of `Fw::SerialBufferBase`/`LinearBufferBase`: a byte
array with a **write cursor** (`ser_loc`, equals current size) and a **read
cursor** (`deser_loc`). Implementors supply storage accessors; everything else
is provided methods. Cursor rules ported exactly:

- every successful write advances `ser_loc` **and resets `deser_loc` to 0**;
- read with `deser_loc == ser_loc` → `DeserBufferEmpty`; with fewer remaining
  bytes than needed → `DeserSizeMismatch`;
- `reset_ser()` zeroes both cursors; `reset_deser()` only the read cursor.

```rust
pub trait SerBuf {
    // storage contract (implementor-supplied)
    fn bytes(&self) -> &[u8];
    fn bytes_mut(&mut self) -> &mut [u8];
    fn capacity(&self) -> usize;
    fn ser_loc(&self) -> usize;   fn set_ser_loc(&mut self, loc: usize);
    fn deser_loc(&self) -> usize; fn set_deser_loc(&mut self, loc: usize);

    // provided: primitives (u8..u64, i8..i64, f32, f64, bool) with Endianness
    fn serialize_u32(&mut self, v: u32, e: Endianness) -> SerializeStatus; // etc.
    fn deserialize_u32(&mut self, v: &mut u32, e: Endianness) -> SerializeStatus; // etc.
    // provided: byte slices (LengthMode), nested SerBuf (u16 size + bytes),
    // sizes (FwSizeType stored as FwSizeStoreType), skip/seek helpers,
    // set_buff/set_buff_len, copy_raw/copy_raw_offset, generic
    // serialize(&impl Serialize)/deserialize(&mut impl Deserialize)
}
```

Wire encodings (all big-endian unless `Little` passed): integers MSB-first;
floats bit-cast then integer rules; bool `0xFF`/`0x00` with **strict** decode
(any other byte → `DeserFormatError`, cursor not advanced); byte slices with
`IncludeLength` get a `FwSizeStoreType` (u16) prefix — the length is silently
truncated to u16 on write (C++ parity) while `serialize_size` range-checks and
returns `FormatError`; strings are u16 length + bytes, no NUL.

Concrete types:

```rust
pub struct LinearBuffer<const N: usize> { /* [u8; N] + cursors */ } // owns storage
pub struct ExtBuf<'a> { /* &'a mut [u8] + cursors */ }             // borrows storage
pub type ComBuffer     = LinearBuffer<FW_COM_BUFFER_MAX_SIZE>;
pub type CmdArgBuffer  = LinearBuffer<FW_CMD_ARG_BUFFER_MAX_SIZE>;
pub type LogBuffer     = LinearBuffer<FW_LOG_BUFFER_MAX_SIZE>;
pub type TlmBuffer     = LinearBuffer<FW_TLM_BUFFER_MAX_SIZE>;
pub type ParamBuffer   = LinearBuffer<FW_PARAM_BUFFER_MAX_SIZE>;
```

### Serialize/Deserialize traits

```rust
pub trait Serialize {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus;
    fn serialized_size(&self) -> usize;
}
pub trait Deserialize {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus;
}
```

(`SerBufAny` is the object-safe core of `SerBuf`; `SerBuf` methods are
implemented over it. Implementations exist for all primitives, `Time`,
strings, FPP-style enums, and packets.)

### Fw value types

- **`FwString<const N: usize>`** — fixed-capacity truncating string
  (`StringTemplate<N>`): assignment truncates silently, wire = u16 len + bytes,
  deserialize rejects lengths > capacity. Aliases: `ObjectName` (80),
  `CmdStringArg` (40), `LogStringArg` (200), `TextLogString` (256),
  `TlmString` (40), `ParamString` (40), `FileNameString` (240),
  `FwDefaultString` (256).
- **`Time`** — `{ time_base: TimeBase, context: u8, seconds: u32, useconds: u32 }`,
  wire = 11 bytes `[base u16][context u8][sec u32][usec u32]`. `TimeBase`
  (repr u16): `TbNone=0, TbProcTime=1, TbWorkstationTime=2, TbScTime=3,
  TbDontCare=0xFFFF`. `compare` ignores context, returns `Incomparable` across
  bases; `add`/`sub` panic (fw_assert) on mismatched bases; deserialize
  rejects `useconds >= 1_000_000` leaving self unmodified. `TimeInterval` =
  `{sec u32, usec u32}`, 8 bytes, commutative absolute `sub`.
- **`Buffer`** (`Fw::Buffer`) — **owned** in Rust: `{ storage: BufferStorage,
  offset: usize, size: usize, context: u32 }` where `BufferStorage` is a
  `Box<[u8]>` recycled through pools. `advance/set_size` keep the C++
  bounds-assert semantics. `NO_CONTEXT = 0xFFFF_FFFF`. The C++ pointer-member
  serialization of `Fw::Buffer` is **not** ported (in-process only, unsafe by
  construction); async buffer ports use the escrow mechanism (see fprime-comp).
- **Small FPP enums** (repr u8 unless noted): `Success {Failure=0, Success=1}`,
  `Enabled {Disabled=0, Enabled=1}`, `Wait {Wait=0, NoWait=1}`,
  `Health {Healthy, Sick, Failed}`, `CmdResponse {Ok=0, InvalidOpcode=1,
  ValidationError=2, FormatError=3, ExecutionError=4, Busy=5}`,
  `LogSeverity {Fatal=1, WarningHi=2, WarningLo=3, Command=4, ActivityHi=5,
  ActivityLo=6, Diagnostic=7}`, `ParamValid {Uninit=0, Valid=1, Invalid=2,
  Default=3}`, `TlmValid {Valid=0, Invalid=1}`, `DeserialStatus` (u8 shadow).
  Each FPP enum serializes at **its representation width** and validates
  exact declared values on deserialize.
- **`ComPacketType` / APID** (repr `FwPacketDescriptorType` = u16):
  `FwPacketCommand=0x0000, FwPacketTelem=0x0001, FwPacketLog=0x0002,
  FwPacketFile=0x0003, FwPacketPacketizedTlm=0x0004, FwPacketDp=0x0005,
  FwPacketIdle=0x0006, FwPacketParam=0x0007, FwPacketHand=0x00FE,
  FwPacketUnknown=0x00FF, SppIdlePacket=0x07FF, InvalidUninitialized=0x0800`.
- **Packets**: `CmdPacket` (deserialize-only: `[u16=0][opcode u32][raw args]`,
  clears the arg buffer for zero-arg commands), `LogPacket`
  (`[u16=2][id u32][time 11B][raw args]`, no severity on the wire),
  `TlmPacket` accumulator (`[u16=1]` + N × `[id u32][time 11B][raw value]`,
  no count, no per-entry length; `add_value` returns `NoRoomLeft` when full).
- **`PolyType`** — tagged union, wire = `[tag: FwEnumStoreType i32][value]`,
  tags `NoType=0, U8=1, I8=2, U16=3, I16=4, U32=5, I32=6, U64=7, I64=8,
  F32=9, F64=10, Bool=11, Ptr=12` (Ptr stored as u64 value in Rust).

### Assert and logger

- `fw_assert!(cond)` / `fw_assert!(cond, arg1, ...)` (up to 6
  `FwAssertArgType` args): on failure formats
  `Assert: "file:line" a1 ... aN`, dispatches to a registered `AssertHook`
  (global `OnceLock`-style swappable hook; register before threads start),
  default prints to stderr and panics. Components like `AssertFatalAdapter`
  can hook this to emit FATAL events.
- `fw::logger` — global diagnostic text logger (`Fw::Logger`):
  `register_logger(&'static dyn FwLogger)`, `fw_log!(fmt, ...)`; silently
  drops when unregistered. Distinct from the event (`Fw.Log`) path.

## fprime-os

Trait-per-facility with one std-backed implementation selected at compile
time (the delegate/placement-new machinery is not ported; the "front type is
concrete, chosen at build time" property is kept via type aliases). Exact
status enums from `docs/cpp-analysis/os.md`. Highlights:

- **`Queue`** — THE critical piece; std has no priority queue with blocking
  semantics. Port of `Os::Generic::PriorityQueue`: fixed slab storage
  (`depth × message_size` bytes allocated at `create`), free-index ring,
  stable max-heap (`BinaryHeap<HeapNode>` with `priority DESC, order ASC`
  tiebreak — FIFO within equal priority), `Mutex` + two `Condvar`s
  (senders wait on full, receivers on empty, notify after unlock).
  Facade checks ported: `create` asserts depth/size > 0, double-create →
  `AlreadyCreated`; `send` with size > message_size → `SizeMismatch`;
  `receive` with capacity < message_size → `SizeMismatch`; non-blocking send
  on full → `Full`, receive on empty → `Empty`.
  `Status { OpOk=0, AlreadyCreated=1, Empty=2, Uninitialized=3,
  SizeMismatch=4, SendError=5, ReceiveError=6, InvalidPriority=7, Full=8,
  NotSupported=9, AllocationFailed=10, UnknownError=11 }`,
  `BlockingType { Blocking, NonBlocking }`.
- **`Task`** — wraps `std::thread` with the C++ state machine
  (`NotStarted/Starting/Running/Exited/...`), `start(Arguments)` /
  `join()` status flow. Priority/affinity/stack are accepted and recorded but
  best-effort no-ops on std (documented divergence — equivalent to the C++
  EPERM silent-degrade path). `Task::delay(TimeInterval)`.
- **`Mutex`/`ConditionVariable`** — thin wrappers over std with F Prime
  status mapping (poisoning maps to panics — a poisoned framework mutex is a
  crashed invariant, matching FW_ASSERT philosophy).
- **`File`** — std::fs with exact `Mode`/`Status` enums, open-mode gating
  (read on write-mode file → `InvalidMode`, `OPEN_CREATE` defaults to
  no-overwrite → `FileExists`), chunked CRC (`calculate_crc` returns the
  **un-complemented** register: `!standard_crc32`, the historical F Prime
  file CRC), `readline` with seek-back-on-error contract.
- **`FileSystem`/`Directory`/`Console`** — std-backed with the C++ composite
  algorithms (copy in 512-byte chunks, moveFile falls back to copy+remove
  only on cross-device errors, etc.).
- **`RawTime`** — `SystemTime`-based (CLOCK_REALTIME semantics), 8-byte wire
  format `[sec u32][nsec u32]` BE; `get_time_interval` is commutative
  absolute difference; `get_diff_usec` saturates at `u32::MAX` with
  `OpOverflow` (~71 min).
- **`IntervalTimer`**, `Os::init()` equivalent (`os::init()` registers the
  console logger).

## fprime-comp: object / port / component model

This crate replaces what the FPP autocoder generates. See
`docs/cpp-analysis/fw-comp.md` and `fpp-autocoder.md`.

### Ports

Each F Prime port *type* is a Rust trait with one `invoke` method matching the
FPP signature (`ref` args become `&mut`). All port traits are
`Send + Sync + 'static`, object-safe. The standard framework ports live here:

```rust
pub trait SchedPort  { fn invoke(&self, port_num: FwIndexType, context: u32); }
pub trait CyclePort  { fn invoke(&self, port_num: FwIndexType, cycle_start: &RawTime); }
pub trait PingPort   { fn invoke(&self, port_num: FwIndexType, key: u32); }
pub trait WatchDogPort { fn invoke(&self, port_num: FwIndexType, code: u32); }
pub trait CmdPort    { fn invoke(&self, port_num: FwIndexType, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer); }
pub trait CmdRegPort { fn invoke(&self, port_num: FwIndexType, op_code: FwOpcodeType); }
pub trait CmdResponsePort { fn invoke(&self, port_num: FwIndexType, op_code: FwOpcodeType, cmd_seq: u32, response: CmdResponse); }
pub trait LogPort    { fn invoke(&self, port_num: FwIndexType, id: FwEventIdType, time_tag: &mut Time, severity: LogSeverity, args: &mut LogBuffer); }
pub trait LogTextPort{ fn invoke(&self, port_num: FwIndexType, id: FwEventIdType, time_tag: &mut Time, severity: LogSeverity, text: &mut TextLogString); }
pub trait TlmPort    { fn invoke(&self, port_num: FwIndexType, id: FwChanIdType, time_tag: &mut Time, val: &mut TlmBuffer); }
pub trait TimePort   { fn invoke(&self, port_num: FwIndexType, time: &mut Time); }
pub trait ComPort    { fn invoke(&self, port_num: FwIndexType, data: &mut ComBuffer, context: u32); }
pub trait BufferSendPort { fn invoke(&self, port_num: FwIndexType, buffer: Buffer); }        // ownership moves
pub trait BufferGetPort  { fn invoke(&self, port_num: FwIndexType, size: FwSizeType) -> Buffer; }
pub trait ComDataWithContextPort { fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) -> Buffer; /* see note */ }
pub trait SuccessConditionPort { fn invoke(&self, port_num: FwIndexType, condition: &mut Success); }
pub trait PrmGetPort { fn invoke(&self, port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) -> ParamValid; }
pub trait PrmSetPort { fn invoke(&self, port_num: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer); }
pub trait FatalEventPort { fn invoke(&self, port_num: FwIndexType, id: FwEventIdType); }
```

> Note on buffer-carrying comms ports: C++ passes `ref Fw::Buffer` and pairs
> every `dataOut` with a `dataReturnOut` return path because buffers are views
> into an allocator's memory. Rust `Buffer` is owned, so buffer-carrying port
> traits **move** the `Buffer` in (and the return-path ports move it back).
> The return-path *ports are kept* (allocators differ per hop), carrying the
> owned buffer upstream: `ComDataWithContextPort::invoke` takes `data: Buffer`
> by move and the receiving component forwards or returns it via its own
> output ports; the C++ call-graph shape is preserved exactly. (The signature
> above returning `Buffer` is NOT used — return travels via the paired
> `dataReturnOut` port, as in C++.)

Connections and output ports:

```rust
pub struct PortRef<P: ?Sized>   { pub target: Arc<P>, pub port_num: FwIndexType }
pub struct OutputPort<P: ?Sized>{ conn: OnceLock<PortRef<P>>, /* + name for tracing */ }

impl<P: ?Sized> OutputPort<P> {
    pub fn connect(&self, target: Arc<P>, port_num: FwIndexType); // once; second call fw_asserts
    pub fn is_connected(&self) -> bool;
    pub fn get(&self) -> &PortRef<P>;  // fw_assert if unconnected (C++ invoke-unconnected assert)
    pub fn try_get(&self) -> Option<&PortRef<P>>;
}
```

Invocation: `let p = self.tlm_out.get(); p.target.invoke(p.port_num, id, &mut time, &mut buf);`
Port arrays are `[OutputPort<P>; N]`. Wiring happens once, before tasks start
(`OnceLock` ⇒ lock-free invocation afterwards).

An *input port* of a component is exposed as a factory method returning an
adapter: `impl CmdDispatcher { pub fn seq_cmd_buff_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComPort> }`.
Adapters are small structs holding `Arc<TheComponent>` and implementing the
port trait by calling the component's handler (sync/guarded) or enqueueing a
message (async). One adapter struct per named input port — exactly the static
thunks the C++ autocoder generates, hand-written.

### Component kinds

Components are `Arc<C>`; mutable state lives in `Mutex<CState>` fields inside
`C` (the component mutex — this is the C++ guarded-port mutex generalized).
Handlers take `&self` and lock internally with the narrowest scope the C++
implementation uses; **output ports are invoked outside the state lock**
unless the C++ component explicitly holds its mutex across the call.

- **Passive**: just the struct + `PassiveBase { name, id_base, instance }`
  (`set_id_base`/`get_id_base` — all opcodes/event/channel/param ids are
  declared component-relative and offset by `id_base` at emission,
  subtracted at dispatch).
- **Queued**: adds `MsgQueue` (an `os::Queue` of fixed-size byte messages).
  `do_dispatch()` (non-blocking receive) and `dispatch_available_messages()`
  ported with `MsgDispatchStatus { Ok=0, Empty=1, Error=2, Exit=3 }`.
  Queued components drain their queue from a sync handler (e.g. schedIn).
- **Active**: adds a task. `start(priority, stack, affinity)` spawns the
  lifecycle loop: `preamble()` → blocking `do_dispatch()` until `Exit` →
  `finalizer()`. `exit()` sends the EXIT message; `join()` joins.

### Async message envelope (byte-exact with C++ generated code)

```
[msg_type: FwEnumStoreType = i32 BE]   // 0 is reserved for EXIT
[port_num: FwIndexType   = i16 BE]     // absent for the EXIT message
[args serialized in declaration order, big-endian]
```

The EXIT message is exactly 4 bytes `[i32 0]`, sent priority 0, non-blocking,
send status ignored (C++ parity — exit can be lost on a full queue).
Each component declares its own `msg_type` discriminants starting at 1 for
its async ports/commands/internal interfaces. Per-port queue-full policy:
`Assert` (default — fw_assert on non-OK send), `Drop` (increment
`msgs_dropped` counter), `Block` (blocking send), `Hook` (call overflow
hook). Message size for `create_queue` = max over async invocations.

Async **buffer-carrying** ports cannot serialize an owned `Buffer` into the
byte message (C++ serializes the raw pointer). Instead each component with
such ports owns a `BufferEscrow`: the adapter deposits the `Buffer` and
serializes the returned `u64` token where C++ would serialize the pointer
(same 8-byte width); the dispatch side claims the token back. Safe, same
message layout.

### Command / event / telemetry / parameter glue

Helper blocks that components embed (hand-written equivalents of the
autocoded base-class glue; see `docs/cpp-analysis/fpp-autocoder.md`):

- `CmdGlue`: `cmd_reg_out`, `cmd_response_out` output ports + a
  `reg_commands(&[local_opcodes])` helper (invokes CmdReg with
  `id_base + opcode`), and dispatch helper mapping absolute opcode →
  local. Handlers must respond exactly once (`FormatError` on arg
  deserialization failure or residual bytes, `ValidationError` on invalid
  enum args).
- `EventGlue`: `log_out` + `text_log_out` ports, `time_get` port;
  `log_event(local_id, severity, |buf| ...)` serializes args into a
  `LogBuffer` (strings truncated to `FW_LOG_STRING_MAX_SIZE`), stamps time,
  invokes both ports; per-event throttle counters (suppress after N until
  `throttle_clear`).
- `TlmGlue`: `tlm_out` port; `tlm_write(local_id, &impl Serialize)` with
  optional on-change suppression.
- `PrmGlue`: `prm_get`/`prm_set` ports + `load_parameters()` pattern.
- `time_get()`: invokes the time port if connected, else `Time::default()`.

### Topology / lifecycle

A deployment hand-writes (Ref-style; see `docs/cpp-analysis/ref-topology.md`)
in this exact phase order:

1. construct instances (`Arc::new`), 2. `set_id_base` per instance,
3. connect ports, 4. configure components, 5. `reg_commands`,
6. `load_parameters`, 7. start tasks (active components + driver tasks),
8. run (e.g. blocking timer loop driving the rate-group driver),
9. teardown: `exit()` all active, `join()` all, then driver stop/join,
then cleanup.

## fprime-utils

- `hash`: CRC32 (IEEE 802.3: table 0xEDB88320 reflected, init `0xFFFFFFFF`,
  `finalize = !register`), `HashBuffer` (4 bytes, `as_big_endian_u32`).
  Test vector: `crc32(b"123456789") == 0xCBF43926`.
- `crc_checker`: file sidecar `.CRC32` (4 raw **native-endian** bytes — do
  not "fix" this; C++ parity), 2048-byte read blocks.
- `types::CircularBuffer`: byte ring over owned storage; `serialize` refuses
  overwrite (`NoRoomLeft`), `peek`(u8/u32-BE/range at offset), `rotate`
  (consume front), `trim` (drop back), high-water mark.
- `types::Queue`: fixed-message FIFO/LIFO over the ring with
  `QueueOverflowMode { DropNewest, DropOldest }` (drop-oldest returns
  `DiscardedExisting`).
- `RateLimiter`, `TokenBucket`: ported as specified in
  `docs/cpp-analysis/utils-misc.md`.

## fprime-svc / fprime-drv: phase-1 components

Ported per their analyses in `docs/cpp-analysis/svc-core.md` and
`svc-comms.md`, with the exact event/telemetry/command IDs, port shapes,
config constants, and quirks listed there:

- `RateGroupDriver` (passive), `ActiveRateGroup`, `PassiveRateGroup`
- `CmdDispatcher` (active), `EventManager` (active), `TlmChan` (active)
- `Health` (queued), `FatalHandler` (passive), `PassiveTextLogger` (passive)
- `PosixTime`-equivalent time source (`SystemTimeSource`, TB_WORKSTATION_TIME)
- Timer cycle source (`IntervalTimerDriver`: blocking loop → CycleOut)
- Comms: `ComQueue` (active), `ComStub`, `FprimeFramer`, `FprimeDeframer`,
  `FrameAccumulator` + `FprimeFrameDetector`, `FprimeRouter`, `BufferManager`
- `fprime-drv`: `ByteStreamStatus { OpOk=0, SendRetry=1, RecvNoData=2,
  OtherError=3 }` + ports, `TcpClient`, `TcpServer` (std::net; read thread +
  reconnect thread; status mapping per the analysis)

The F Prime frame (GDS-compatible): `[0xdeadbeef u32][length u32][payload]
[crc32 u32]`, CRC over header+payload, all BE, total = 12 + length.

## fprime-ref

A Ref-style demo deployment: 1 Hz timer → RateGroupDriver (÷1, ÷2, ÷4) →
three ActiveRateGroups; CmdDispatcher/EventManager/TlmChan/Health/
FatalHandler/text logger; a `SignalGen` demo component (commands, events,
telemetry); comms stack over `TcpClient` (`-a host -p port`, optional — runs
standalone without comms). Connects to the stock fprime-gds.

## Testing strategy

- **Byte-vector tests** for every wire format (primitives, time, strings,
  packets, frame, queue message envelope) with hand-computed expected bytes.
- **Semantic tests** per component covering the quirks called out in the
  analyses (e.g. m_seq increments on invalid opcode; Health fatal-before-warn
  equality checks; TlmChan deferred-entry drop; ComQueue WAITING handshake).
- **Concurrency tests** for the priority queue (FIFO-within-priority,
  blocking send/receive) and active component lifecycle.
- **Integration test**: mini topology wired end-to-end — command in via
  framed bytes → deframer → router → dispatcher → component → response;
  events/telemetry out through framer, validated against expected frames.
