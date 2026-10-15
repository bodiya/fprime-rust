# Roadmap

Phase 1 (everything in the README status matrix) is complete and green:
`cargo build`, `cargo test`, `cargo clippy -D warnings` and `cargo fmt --check`
across the workspace. This file is the ordered backlog for what comes next.
Each item names the C++ source it ports and the constraint it has to respect;
`docs/api-notes.md` records the deviations once an item lands.

Items are ordered by value-per-effort for a deployment that talks to the stock
`fprime-gds`: consolidation and small unblocked gaps first, then the larger
subsystems, then the strategic changes.

## Done in phase 2

- **JSON dictionary and ground-system cross-check.** `fpp-to-rust --dict`
  writes the `fpp-to-dict` dictionary (spec 1.0.0) for every deployment
  topology and system: the deep closure of used types and constants, the
  implied framework uses, `dictionary` definitions, commands (including
  the implicit `_PRM_SET`/`_PRM_SAVE`), parameters, events, channels,
  records, containers and telemetry packet sets, keyed by global id. It is
  checked against the reference compiler's `fpp-to-dict` corpus
  (`crates/fprime-fpp/tests/dict`). The Rust Ref's dictionary
  (`crates/fprime-ref/dictionary`) is generated from the upstream model
  plus an FPP model of the Rust instances and the Rust SignalGen, and
  `tools/gds-crosscheck.py` proves it live against fprime-gds 4.3.1:
  commands sent by the GDS execute on the Rust Ref, and its events and
  telemetry decode with the stock decoders.

- **FPP code generation (`fpp-to-rust`).** `crates/fprime-fpp` is a
  stdlib-only port of the reference compiler's front end (lexer, parser,
  includes, the semantic analysis with its implicit-id, port-numbering and
  pattern rules) plus a Rust back end that emits the macro layer for data
  types, port traits, component bases with handler traits, and topology
  wiring. `crates/fprime-fpp-demo` builds a model from FPP at build time
  and tests the generated code end to end; the whole upstream framework
  model and the Ref deployment analyze. Not generated: state machines
  (`Fw/Sm`, item 11 below) and serial ports (item 15). See
  `crates/fprime-fpp/README.md`.

- **Config constants into `fprime-config`** (item 1). `FW_FILE_BUFFER_MAX_SIZE`,
  `FILE_DOWN_COMPLETE_PORTS`, `file_downlink::*`, `file_manager::*` and
  `crc_checker::FILE_READ_BLOCK` now live in `fprime-config`, re-exported
  under their old component paths; `Fw::StringFormatStatus` moved to
  `fprime-fw::enums`. The `PassiveTextLogger`, `TlmPacketizer` and
  `SystemResources` constants are still module-local (see
  `docs/api-notes.md`), and the CCSDS `ComCfg` values stay in
  `ccsds::types` by design.
- **`FileManager` command-path sandbox** (item 2). Port of upstream
  nasa/fprime#5850: `configure(sandboxDir)`, every command path resolved and
  contained before use, `PathOutsideSandbox`, fail-closed when unconfigured.
  The Ref confines it to the deployment's data directory (upstream's
  `FileHandling` subtopology configures `/`, i.e. unrestricted).
- **`Os::ValidateFile`** (item 3), as `fprime_utils::validate_file`;
  `ComLogger` uses it. Porting it showed that upstream's `CRCChecker` has
  written big-endian `.CRC32` sidecars since nasa/fprime#5775, so
  `crc_checker` was corrected to match: both now write the same bytes.
- **`Os::Cpu` / `Os::Memory`** (item 4) in `fprime-os`, with a Linux procfs
  backend and the upstream stubs; `SystemResources` samples through them.
- **`ComStub` async driver path** (item 5). Upstream has no async
  byte-stream driver at 4dee010 (`LinuxUartDriver`, now `PosixUartDriver`, is
  synchronous), so the path is tested against test drivers, as upstream does.
- **`CmdSequencer` `formats/AMPCSSequence`** (item 6), selectable with
  `set_sequence_format`; the Ref still uses `FPrimeSequence`.
- **`Svc::BufferAccumulator`** (item 7), restored between `dpMgr` and
  `dpWriter` in the Ref as in upstream's DataProducts subtopology; the
  dictionary and the GDS cross-check include it.
- **`Svc::ActiveTextLogger`** (item 8). The component itself was missing,
  not just its file path, so it is ported whole: the caller-thread
  filtering and formatting, the `drop`-policy internal queue, and
  `LogFile` (`set_log_file`, the `name0`…`name9` suffix search with
  fallback to truncating the original, and the size cap). Upstream does
  not rotate: a line that would exceed the cap closes the file for good.
  The upstream Ref does not instantiate it, so neither does the Rust Ref.
- **`Os::SandboxedFile` / `Os::FilePathUtils` moved to `fprime-os`**
  (`file_path_utils.rs`, `sandboxed_file.rs`). `FileUplink`, `FileDownlink`
  and `PrmDb` all use the one implementation; `PrmDb`'s private `String`-based
  copy is gone.
- **`Svc::FrameDetectors::CcsdsTcFrameDetector`** in
  `fprime-svc::frame_accumulator`, so a CCSDS uplink has a frame synchronizer
  in front of `TcDeframer`.
- **`FileManager::GenerateDp` chunking**: the paced/immediate loop over
  `productGetOut`/`productSendOut`, `FileChunkHeaderRecord` +
  `FileChunkDataRecord` containers, and the `Ref` topology wiring on `dpMgr`
  index 1.

## Next: small follow-ups found in phase 2

These surfaced while porting items 2–7. None changes the wire format of the
Ref deployment.

- **Restrict the Ref's `FileManager` sandbox** to the deployment's data
  directory, as `prmDb`/`fileDownlink` already are, if fail-open-to-`/`
  (the upstream `FileHandling` default) is not wanted.
- **`cargo doc` warnings.** An ambiguous `[`file`]` link in `fprime-os` and
  `Self::lock` in `tlm_packetizer.rs`; fix them and add `RUSTDOCFLAGS=-D
  warnings` to the checks.

## Then: larger subsystems

9. **`Svc::GenericHub`.** The serialize-port and `ObjRegistry` machinery it
   needs was skipped in phase 1; the async envelope is already
   hub-compatible (see `docs/api-notes.md`, fprime-comp §4).
10. **`Svc::FpySequencer`.** Larger than `CmdSequencer`; needs the Fpy
    bytecode dispatcher and its own state machine (see item 11).
11. **State-machine autocoding (`Fw/Sm`).** A declarative-macro layer in the
    style of `component_msg_types!`: states, signals, guards and actions
    with the generated `init()`/`update()` shape and the `Signal` enum on the
    wire.
12. **CCSDS AOS / `ComAggregator` / SDLS.** `AOSHeader`, `AOSTrailer`,
    `MPduHeader`, `SaMapEntry`, `SdlsStatus` are declared; nothing consumes
    them.

## Strategic

13. **`no_std` OSAL seam.** `fprime-os` keeps the seam; making `fprime-fw`
    and `fprime-comp` `no_std` + `alloc` is the first real test of it.
14. **zlib data-product compression** (`DpZLibCompressor`,
    `DpCompressProc`). Needs a policy decision on a first third-party
    dependency (or a vendored deflate); `ProcType::ZlibDeflate` is already on
    the wire.
15. **FPP back-end gaps.** The front end is complete; the Rust back end
    still lacks state-machine instances (needs item 11) and serial ports.
    Telemetry packet sets are analyzed and written to the dictionary but
    the Rust topology does not yet instantiate a packetizer from them.

## Cross-compatibility with the stock ground system

16. **Broaden `tools/gds-crosscheck.py`.** Today it proves commands
    (no-arg, string, enum, float), events and telemetry channels against
    fprime-gds 4.3.1. Not yet exercised through the real GDS: parameter
    `_PRM_SET`/`_PRM_SAVE` round trips (`prmDb`), file uplink and
    downlink (`fprime-cli file-uplink`, `fileDownlink.SendFile`),
    data-product downlink and `fprime-dp` decoding of the Rust
    `DpContainer` bytes, command sequences (`fprime-seqgen` output run
    by `cmdSeq`), and the CCSDS framing stack (`--framing-selection
    space-packet-space-data-link` against a `ComCcsds`-style topology).
    Each is a few more commands and expectations in the script; the
    script needs a Python environment with fprime-gds installed, so it
    stays a tool rather than a `cargo test`.
17. **Pin the upstream model version.** The Ref dictionary is generated
    from whatever F Prime checkout `generate-dictionary.sh` is given
    (currently commit 4dee010, 2026-09-11) and records it in
    `metadata.frameworkVersion`. When the Rust components are next
    synced with upstream, regenerate the dictionary and rerun the
    cross-check; the static id comparison in `docs/api-notes.md`
    (Rust `OPCODE_*`/`EVENTID_*`/`CHANID_*` vs. the dictionary) is the
    quick way to spot drift.
