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
  topology: rate groups driving a demo component, full C&DH stack, and a
  TCP-based comms chain a ground system can connect to.

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
[`CONVENTIONS.md`](CONVENTIONS.md) (coding rules),
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
```

Requires stable Rust (edition 2024). No external crates.

## Porting status

### Ported

| Area | Components / features |
| --- | --- |
| Core types & serialization | `Fw` serialization engine, `LinearBuffer`/`ExtBuf`, `ComBuffer`/`CmdArgBuffer`/`LogBuffer`/`TlmBuffer`/`ParamBuffer`, fixed strings, `Time`/`TimeInterval`, FPP enums, `Fw::Buffer` (owned), `CmdPacket`/`LogPacket`/`TlmPacket`, `PolyType`, assert hooks, `Fw::Logger` |
| Component model | Passive/queued/active bases, typed port traits + `OutputPort` wiring, byte-exact async message envelope + EXIT, queue-full policies (assert/drop/block/hook), command/event/telemetry/parameter glue, event throttling, buffer escrow |
| OSAL | Priority queue (stable max-heap, blocking semantics), task state machine, mutex/condvar, file/filesystem/directory/console, raw time + interval timer |
| C&DH services | `CmdDispatcher`, `EventManager`, `TlmChan`, `Health`, `FatalHandler`, `PassiveTextLogger`, `PosixTime`, `LinuxTimer` |
| Rate groups | `RateGroupDriver`, `ActiveRateGroup`, `PassiveRateGroup` |
| Comms stack | `FprimeFramer`, `FprimeDeframer`, `FrameAccumulator` + `FprimeFrameDetector`, `FprimeRouter`, `ComQueue`, `ComStub`, `BufferManager` |
| Drivers | `TcpClient`, `TcpServer` (byte-stream driver model) |
| Support | CRC-32 (`Utils::Hash`), CRC sidecar checker, circular buffer, fixed-message queue, `RateLimiter`, `TokenBucket` |

### Not yet ported

`CmdSequencer`, `FpySequencer`, file services (`FileUplink`, `FileDownlink`,
`FileManager`), parameter database (`PrmDb`), data products (`Dp*`),
`TlmPacketizer`, `ComLogger`, the CCSDS stack (`SpacePacket`/`TmTc`/SDLS),
`GenericHub`, state-machine autocoding (`Fw/Sm`), `SystemResources`,
UART/I2C/SPI/GPIO drivers, and the FPP autocoder itself (components are
hand-written against a documented pattern; a derive-macro layer is future
work). `no_std` targets are a design goal of the OSAL seam but not yet
implemented.

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
