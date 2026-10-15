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
  third-party dependencies in the default build (the only exceptions sit
  behind optional, off-by-default cargo features: `fprime-svc/zlib` for
  data-product compression and `fprime-svc/aes-gcm` for SDLS AES-GCM), no steady-state heap allocation (allocate at
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
| `fprime-fpp` | the FPP compiler front end and `fpp-to-rust` back end (see `crates/fprime-fpp/README.md`) |
| `fprime-fpp-demo` | an FPP-modeled demo generated at build time, with end-to-end tests of the generated code |

Generate from FPP: `cargo run -p fprime-fpp --bin fpp-to-rust -- --help`
(see `crates/fprime-fpp/README.md` and the `fprime-fpp-demo` crate's `build.rs`).

Talk to the stock ground system: `crates/fprime-ref/dictionary/RefTopologyDictionary.json`
is the Rust reference deployment's dictionary (regenerate it from an upstream
checkout with `crates/fprime-ref/fpp/generate-dictionary.sh`), and
`tools/gds-crosscheck.py` drives the real `fprime-gds` tools against
`target/debug/fprime-ref` end to end, one pass/fail line per check:
commands, events and telemetry (`fprime-cli`); parameter save, reload
across a restart and the parameter-file tools (`fprime-prm-decode`,
`fprime-prm-write`); file uplink (`fprime-cli file-uplink`) and downlink
(`SendFile`, `SendPartial`), byte for byte; data products downlinked by
`dpCat` and decoded with `fprime-dp`; `fprime-seqgen` sequences run by
`cmdSeq`; and all of that over the GDS's CCSDS framing
(`space-packet-space-data-link`) against `fprime-ref --framing ccsds`.
`--checks commands,files,...` runs a subset.

```bash
python3 -m venv ~/.venvs/fprime-gds && ~/.venvs/fprime-gds/bin/pip install fprime-gds
cargo build -p fprime-ref
tools/gds-crosscheck.py --gds-bin ~/.venvs/fprime-gds/bin \
    --ref-bin target/debug/fprime-ref \
    --dictionary crates/fprime-ref/dictionary/RefTopologyDictionary.json
```

Upstream version: the port tracks one nasa/fprime commit, recorded in
[`FPRIME_UPSTREAM_COMMIT`](FPRIME_UPSTREAM_COMMIT) (currently `4dee010c30`).
`generate-dictionary.sh` refuses any other checkout unless given
`--allow-other-commit`, and `cargo test -p fprime-ref --test upstream_ids`
fails when the dictionary and the Rust components' id constants drift apart;
the sync procedure is in [`docs/ROADMAP.md`](docs/ROADMAP.md#syncing-with-upstream).

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
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps

# Also build/test the zlib data-product compressor (pulls in miniz_oxide)
cargo test --workspace --features fprime-svc/zlib

# Also build/test the SDLS AES-256-GCM components (pulls in RustCrypto
# aes-gcm, getrandom and zeroize)
cargo test --workspace --features fprime-svc/aes-gcm

# no_std + alloc: fprime-config, fprime-fw, fprime-comp and the no_std
# subset of fprime-os without the default `std` feature. Locks then come
# from the `spin` crate, which the `spin` feature enables (cargo cannot turn
# a dependency on when a feature is OFF, so the feature is explicit).
cargo build -p fprime-fw -p fprime-os -p fprime-comp --no-default-features --features spin
cargo test  -p fprime-fw -p fprime-os -p fprime-comp --no-default-features --features spin
# ... and for a bare-metal target, which has no std at all
rustup target add thumbv7em-none-eabihf
cargo build --target thumbv7em-none-eabihf -p fprime-fw -p fprime-os -p fprime-comp \
    --no-default-features --features spin
# All of the above plus clippy, MSRV 1.85 and the dependency-tree checks
tools/check-no-std.sh

# Run the reference deployment standalone (1 Hz rate loop; type "quit" to stop)
cargo run -p fprime-ref

# Run it against a TCP ground system (e.g. fprime-gds in TCP mode)
cargo run -p fprime-ref -- -a 127.0.0.1 -p 50000

# Choose where the deployment keeps its parameter file, data products,
# uplinked files and com logs (default: a per-process temp directory)
cargo run -p fprime-ref -- --data-dir ./run-data

# Frame with CCSDS (space packets in TM/TC transfer frames, the
# ComCcsds stack) instead of F Prime frames; pair with the GDS's
# --framing-selection space-packet-space-data-link
cargo run -p fprime-ref -- -a 127.0.0.1 -p 50000 --framing ccsds
```

Requires stable Rust (edition 2024). No external crates in the default build;
the optional `zlib` feature adds `miniz_oxide` (pure Rust), the optional
`aes-gcm` feature adds RustCrypto `aes-gcm` with `getrandom` and `zeroize`,
and the `no_std` configuration (`--no-default-features --features spin`)
adds `spin` (lock types only).

## Porting status

### Ported

| Area | Components / features |
| --- | --- |
| Core types & serialization | `Fw` serialization engine, `LinearBuffer`/`ExtBuf`, `ComBuffer`/`CmdArgBuffer`/`LogBuffer`/`TlmBuffer`/`ParamBuffer`, fixed strings, `Time`/`TimeInterval`, FPP enums, `Fw::Buffer` (owned), `CmdPacket`/`LogPacket`/`TlmPacket`, `FilePacket`, `DpContainer`, `PolyType`, assert hooks, `Fw::Logger` |
| Codegen layer | `fpp_enum!`, `fpp_struct!`, `fpp_array!` (FPP data types), `component_msg_types!`, `component_sm_ids!`, `input_port_adapter!`, `async_input_port_adapter!`, `serializable_port!` (component/port scaffolding and port serialization) and `fpp_state_machine!` (FPP internal state machines) — declarative macros replacing the mechanical parts of the C++ autocoder's output |
| FPP compiler | `fpp-to-rust` (`crates/fprime-fpp`): the full FPP grammar, include resolution, the reference compiler's semantic analysis (implicit ids/opcodes, interface imports, topology imports, patterns, port numbering, telemetry packet sets), a Rust back end emitting types, port traits (serializable ones via `serializable_port!`), component bases with handler traits (including `serial` ports), topology wiring (typed ↔ serial connections included), the `TlmPacketizer` packet tables of telemetry packet sets, state machines (`fpp_state_machine!`) and state machine instances (internal and external), instance init specifiers (Rust code per topology phase) and instances of hand-written components such as `TlmPacketizer`, and a `--dict` back end writing the `fpp-to-dict` JSON dictionary the stock ground system reads; verified on the whole upstream framework model, the Ref deployment and the reference compiler's dictionary corpus |
| Ground-system cross-compatibility | `crates/fprime-ref/dictionary/RefTopologyDictionary.json` is the Rust Ref's ground dictionary, generated from the pinned upstream commit and checked against the Rust ids by `cargo test`; `tools/gds-crosscheck.py` runs the stock `fprime-gds` tools against the Rust Ref binary: commands, events, telemetry, parameter save/reload and parameter files, file uplink/downlink, data-product downlink and decoding, `fprime-seqgen` sequences, under both F Prime and CCSDS framing (passes against fprime-gds 4.3.1) |
| Component model | Passive/queued/active bases, typed port traits + `OutputPort` wiring, serialize ports (`Fw::InputSerializePort`/`OutputSerializePort`: typed calls carried as bytes and replayed on typed ports), byte-exact async message envelope + EXIT, queue-full policies (assert/drop/block/hook), command/event/telemetry/parameter glue, event throttling, buffer escrow |
| State machines (`Fw/Sm`) | FPP internal state machines (hierarchical states, entry/exit, initial transitions, choices, guards, typed signals) with the `fpp-to-cpp` `State`/`Signal` discriminants and action order — checked against the reference compiler's output for `FpySequencer`, `ComAggregator` and the FppTest machines; state machine instances in components (byte-exact signal messages, per-instance priority and queue-full policy); external (`init`/`update`) machines with `SmSignalBuffer` |
| OSAL | Priority queue (stable max-heap, blocking semantics), task state machine, mutex/condvar, file/filesystem/directory/console, `SandboxedFile` + `FilePathUtils` (lexical path resolution and containment), raw time + interval timer, `Cpu`/`Memory` (Linux procfs backend, upstream stubs elsewhere) |
| `no_std` + `alloc` | `fprime-config`, `fprime-fw`, `fprime-comp` and the `fprime-os` queue/task/raw-time/interval-timer/stub-cpu-memory subset build without `std` (`--no-default-features --features spin`; locks from `spin`), including for `thumbv7em-none-eabihf`; tasks, the clock and blocking-queue backoff come from a deployment-supplied `fprime_os::platform::Platform` (`tools/check-no-std.sh`) |
| C&DH services | `CmdDispatcher`, `EventManager`, `TlmChan`, `TlmPacketizer`, `Health`, `FatalHandler`, `PassiveTextLogger`, `ActiveTextLogger` (console + size-capped log file), `PosixTime`, `LinuxTimer`, `SystemResources` |
| Rate groups | `RateGroupDriver`, `ActiveRateGroup`, `PassiveRateGroup` |
| Sequencing | `CmdSequencer` with the `FPrimeSequence` and `AMPCSSequence` (`.CRC32` sidecar) sequence-file formats; `FpySequencer` (the Fpy bytecode sequencer: sequence files with arguments and CRC, the stack machine and every directive, command dispatch with response matching and timeouts, breakpoints and stepping) |
| Parameters | `PrmDb` with the byte-exact parameter file and staged-load state machine |
| File services | `FileUplink`, `FileDownlink`, `FileManager` (including `GenerateDp` file-to-data-product chunking and the fail-closed command-path sandbox), CFDP checksum |
| Data products | `DpManager`, `BufferAccumulator`, `DpWriter`, `DpCatalog` (`.fdp` files, catalog transmit), `DpCompressProc` (in-place chunked compression into `CompressionRecord`s) and `DpZLibCompressor` (zlib streams via pure-Rust `miniz_oxide`; only with the off-by-default `zlib` feature of `fprime-svc`) |
| Comms stack (F Prime) | `FprimeFramer`, `FprimeDeframer`, `FrameAccumulator` + `FprimeFrameDetector`, `FprimeRouter`, `ComQueue`, `ComStub` (synchronous and asynchronous driver paths), `BufferManager`, `ComLogger` |
| Hub pattern | `GenericHub` (events, telemetry, serialized typed port calls, buffers and remote command dispatch/response between deployments; byte-exact hub wire format) |
| Comms stack (CCSDS) | CRC-16 frame error control, Space Packet primary header, TM/TC transfer frames, `ApidManager`, `SpacePacketFramer`/`SpacePacketDeframer`, `TmFramer` (First Header Pointer from the context), `TcDeframer`, `CcsdsTcFrameDetector` (TC uplink frame synchronizer for `FrameAccumulator`), `ComAggregator` (with optional TM packet spanning); wired as the Ref's `ComCcsds` stack by `fprime-ref --framing ccsds`. AOS: `AosFramer`/`AosDeframer` (M_PDU, SPP/EPP, spanning packets). SDLS: `CcsdsSdlsFramer`/`CcsdsSdlsDeframer`, `SdlsSaRouter`, `SdlsFileKeyManager`, `ClearTextEncryptor`/`ClearTextDecryptor`, and `AesGcmEncryptor`/`AesGcmDecryptor` (RustCrypto `aes-gcm`; only with the off-by-default `aes-gcm` feature of `fprime-svc`) |
| Drivers | `TcpClient`, `TcpServer` (byte-stream model); `AsyncByteStreamBufferAdapter` (async byte-stream driver to passive buffer driver); `LinuxGpioDriver`, `PosixUartDriver`, `LinuxI2cDriver`, `LinuxSpiDriver` as full component surfaces over backend traits (see the hardware note below) |
| Support | CRC-32 (`Utils::Hash`), CRC sidecar checker and `Os::ValidateFile` (both big-endian `.CRC32`), circular buffer, fixed-message queue, `RateLimiter`, `TokenBucket` |

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
- **UART** (`PosixUartDriver`, upstream's former `LinuxUartDriver`) ships
  device-file I/O over an already-configured port, with an opt-in helper
  that applies settings via `stty` (GNU `-F` or BSD `-f`, per host). Requesting a
  configuration without it fails the open with `ConfigError` rather than
  silently running at the wrong baud.
- **I²C and SPI** ship stub backends only; the required ioctls are named in
  the rustdoc.

### Not ported

`Svc::Ccsds::CfdpManager` (CFDP) is not ported. The SDLS security
layer is ported, but `AesGcmEncryptor`/`AesGcmDecryptor` are only compiled
with `fprime-svc`'s `aes-gcm` feature, and the Rust Ref, like upstream's
(which uses `ComCcsds`, not `ComCcsdsSdls`), does not instantiate SDLS or
AOS. Data-product compression is ported, but `DpZLibCompressor` is only
compiled with `fprime-svc`'s `zlib` feature, and the Rust Ref, like
upstream's (where `DpCompression.Subtopology` is commented out), does not
instantiate it. `FpySequencer` is ported, but neither upstream's Ref nor
the Rust Ref instantiates it, and the Fpy compiler (`fprime-fpyc`) is not
part of fprime-gds 4.3.1, so its tests build sequences byte by byte.
`no_std`: `fprime-config`, `fprime-fw`, `fprime-comp` and a subset of
`fprime-os` (queue, task, raw time, interval timer, stub cpu/memory) build
as `no_std` + `alloc`; a deployment supplies tasks, the clock and the
blocking-queue backoff through `fprime_os::platform::Platform`. The
services, drivers, utilities and the file/console half of `fprime-os` are
still `std`-only (see ARCHITECTURE.md "no_std").
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
