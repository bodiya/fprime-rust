# fprime-rust

A Rust port of [F Prime (F´)](https://github.com/nasa/fprime), NASA JPL's
component-driven framework for spaceflight and embedded software.

F Prime structures flight software as **components** (passive, queued, or
active) connected by typed **ports**, with framework-provided infrastructure
for commands, events, telemetry, and parameters, an OS abstraction layer, and
a ground-system-compatible communications stack. This repository ports that
architecture — and its exact wire formats — to safe, dependency-free Rust.

## Highlights

- **Wire-compatible with the C++ framework and the fprime-gds**: big-endian
  F Prime serialization (`0xFF`/`0x00` bools, u16 length prefixes), the
  F Prime frame protocol (`0xdeadbeef` start word, CRC-32), and byte-exact
  command / event / telemetry packet formats.
- **The F Prime component model in Rust**: passive/queued/active components,
  guarded and async ports with priority message queues (FIFO within
  priority), rate groups, command registration/dispatch, event filtering and
  throttling, double-buffered telemetry storage, health pings.
- **Flight-software discipline**: `#![forbid(unsafe_code)]` everywhere, zero
  third-party dependencies, no steady-state heap allocation (allocate at
  init, recycle after), status enums with the C++ discriminants, `fw_assert`
  for invariants.
- **A runnable reference deployment** (`fprime-ref`) mirroring the C++ `Ref`
  topology: rate groups driving a demo component, the full C&DH stack,
  parameters, file uplink/downlink, a command sequencer, data products, and a
  TCP-based comms chain a ground system can connect to. Its integration tests
  drive real framed bytes through the port graph — uplinking a file, running a
  sequence, saving and reloading parameters, writing a data product.

## Workspace layout

| Crate | Ports (C++ equivalent) |
| --- | --- |
| `fprime-config` | `default/config/` — type aliases (`FwSizeType`, `FwIndexType`, ...) and every project-configurable constant |
| `fprime-fw` | `Fw/` — serialization core, buffers, strings, `Time`, packets, `PolyType`, `fw_assert!`, diagnostic logger |
| `fprime-os` | `Os/` — priority message queue, task lifecycle, mutex/condvar, file (with F Prime CRC quirk), filesystem, console, raw time |
| `fprime-utils` | `Utils/` — CRC-32 hash, file CRC sidecars, circular buffer, fixed-message queue, rate limiter, token bucket |
| `fprime-comp` | `Fw/Obj` + `Fw/Port` + `Fw/Comp` + the FPP autocoder contract — port traits, output-port wiring, async message envelopes, active-component lifecycle, cmd/event/tlm glue |
| `fprime-svc` | `Svc/` — the standard service components (see matrix below) |
| `fprime-drv` | `Drv/` — byte-stream driver model, TCP client/server |
| `fprime-ref` | `Ref` — reference deployment binary + end-to-end integration tests |

Design docs: [`ARCHITECTURE.md`](ARCHITECTURE.md) (binding design contract),
[`CONVENTIONS.md`](CONVENTIONS.md) (coding rules), [`docs/ROADMAP.md`](docs/ROADMAP.md)
(what is next, in order),
[`docs/cpp-analysis/`](docs/cpp-analysis/) (per-subsystem analyses of the C++
implementation — wire formats, exact enum values, threading, gotchas — that
ground the port), [`docs/api-notes.md`](docs/api-notes.md) (implementer notes
on each crate's public API).

## Building and running

```bash
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings

# Run the reference deployment standalone (1 Hz rate loop; type "quit" to stop)
cargo run -p fprime-ref

# Run it against a TCP ground system (e.g. fprime-gds in TCP mode)
cargo run -p fprime-ref -- -a 127.0.0.1 -p 50000

# Choose where the deployment keeps its parameter file, data products,
# uplinked files and com logs (default: a per-process temp directory)
cargo run -p fprime-ref -- --data-dir ./run-data
```

Requires stable Rust (edition 2024). No external crates.

## Porting status

### Ported

| Area | Components / features |
| --- | --- |
| Core types & serialization | `Fw` serialization engine, `LinearBuffer`/`ExtBuf`, `ComBuffer`/`CmdArgBuffer`/`LogBuffer`/`TlmBuffer`/`ParamBuffer`, fixed strings, `Time`/`TimeInterval`, FPP enums, `Fw::Buffer` (owned), `CmdPacket`/`LogPacket`/`TlmPacket`, `FilePacket`, `DpContainer`, `PolyType`, assert hooks, `Fw::Logger` |
| Codegen layer | `fpp_enum!`, `fpp_struct!`, `fpp_array!` (FPP data types) and `component_msg_types!`, `input_port_adapter!`, `async_input_port_adapter!` (component/port scaffolding) — declarative macros replacing the mechanical parts of the C++ autocoder's output |
| Component model | Passive/queued/active bases, typed port traits + `OutputPort` wiring, byte-exact async message envelope + EXIT, queue-full policies (assert/drop/block/hook), command/event/telemetry/parameter glue, event throttling, buffer escrow |
| OSAL | Priority queue (stable max-heap, blocking semantics), task state machine, mutex/condvar, file/filesystem/directory/console, `SandboxedFile` + `FilePathUtils` (lexical path resolution and containment), raw time + interval timer |
| C&DH services | `CmdDispatcher`, `EventManager`, `TlmChan`, `TlmPacketizer`, `Health`, `FatalHandler`, `PassiveTextLogger`, `PosixTime`, `LinuxTimer`, `SystemResources` |
| Rate groups | `RateGroupDriver`, `ActiveRateGroup`, `PassiveRateGroup` |
| Sequencing | `CmdSequencer` with the `FPrimeSequence` binary sequence-file format |
| Parameters | `PrmDb` with the byte-exact parameter file and staged-load state machine |
| File services | `FileUplink`, `FileDownlink`, `FileManager` (including `GenerateDp` file-to-data-product chunking), CFDP checksum |
| Data products | `DpManager`, `DpWriter`, `DpCatalog` (`.fdp` files, catalog transmit) |
| Comms stack (F Prime) | `FprimeFramer`, `FprimeDeframer`, `FrameAccumulator` + `FprimeFrameDetector`, `FprimeRouter`, `ComQueue`, `ComStub`, `BufferManager`, `ComLogger` |
| Comms stack (CCSDS) | CRC-16 frame error control, Space Packet primary header, TM/TC transfer frames, `ApidManager`, `SpacePacketFramer`/`SpacePacketDeframer`, `TmFramer`, `TcDeframer`, `CcsdsTcFrameDetector` (TC uplink frame synchronizer for `FrameAccumulator`) |
| Drivers | `TcpClient`, `TcpServer` (byte-stream model); `LinuxGpioDriver`, `LinuxUartDriver`, `LinuxI2cDriver`, `LinuxSpiDriver` as full component surfaces over backend traits (see the hardware note below) |
| Support | CRC-32 (`Utils::Hash`), CRC sidecar checker, circular buffer, fixed-message queue, `RateLimiter`, `TokenBucket` |

### Hardware drivers: what works without `unsafe`

Linux GPIO (character device), I²C (`I2C_SLAVE`/`I2C_RDWR`), SPI full duplex
and UART termios configuration all require `ioctl(2)`, which safe
zero-dependency `std` Rust cannot issue. Each driver is therefore ported as
the complete component surface — ports, statuses, events, telemetry — over a
**backend trait**, so a downstream crate that permits `libc` or an FFI shim
can supply real hardware access without forking the component:

- **GPIO** ships a sysfs backend (`/sys/class/gpio`, pure file I/O) with
  faithful input/output, and *level-sampled* pseudo-interrupts (`poll(2)` is
  unavailable) — documented on the public API as sampling, not kernel edge
  interrupts.
- **UART** ships device-file I/O over an already-configured port, with an
  opt-in helper that applies settings via `stty`. Requesting a
  configuration without it fails the open with `ConfigError` rather than
  silently running at the wrong baud.
- **I²C and SPI** ship stub backends only; the required ioctls are named in
  the rustdoc.

### Not ported

The FPP *compiler* (there is no `.fpp` parser or build-time generator — the
macro codegen layer covers the mechanical output instead), `FpySequencer`,
`GenericHub`, state-machine autocoding (`Fw/Sm`), `ActiveTextLogger`'s file
logging, the SDLS security layer, and zlib data-product compression
(`DpZLibCompressor`/`DpCompressProc`, which would need a third-party
dependency — `ProcType::ZlibDeflate` is kept for wire parity). `no_std`
targets remain a design goal of the OSAL seam rather than a current feature.
The ordered list of what comes next is in [`docs/ROADMAP.md`](docs/ROADMAP.md).

## How the port maps C++ to Rust

- **Ports**: each FPP port type is an object-safe trait; an input port is a
  small adapter struct holding an `Arc` of its component (the hand-written
  equivalent of the autocoder's static thunks); output ports are
  `OnceLock`-based connections wired once at topology setup.
- **Async ports** serialize their arguments into the same
  `[msg_type i32][port_num i16][args]` big-endian envelope the C++
  autocoder generates, pushed onto the same priority-queue semantics
  (numerically higher priority first, FIFO within a priority).
- **Buffers own their memory**: `Fw::Buffer` is an owned storage handle that
  *moves* through the port graph; the C++ `dataOut`/`dataReturnOut`
  ownership-return convention is preserved port-for-port, so allocators
  (`BufferManager`) still get their storage back.
- **Guarded ports** map to a per-component mutex, locked with the same scope
  the C++ components use; C++'s deliberately racy cross-thread flags map to
  relaxed atomics.
- **`FW_ASSERT`** maps to `fw_assert!` with a swappable hook (default:
  formatted message to stderr + panic), preserving the
  assert-on-programmer-error / status-on-runtime-error split.

Every deliberate behavioral divergence is documented in code comments and in
`docs/api-notes.md` (e.g. task priorities are best-effort on `std` threads,
matching the C++ EPERM-degrade path; the router's context table keys on a
generated token rather than a raw pointer).

## License

Apache-2.0, matching upstream F Prime. This is an independent port; it is not
an official NASA or JPL product.
