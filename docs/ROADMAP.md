# Roadmap

Phase 1 (everything in the README status matrix) is complete and green:
`cargo build`, `cargo test`, `cargo clippy -D warnings` and `cargo fmt --check`
across the workspace. This file is the ordered backlog for what comes next.
Each item names the C++ source it ports and the constraint it has to respect;
`docs/api-notes.md` records the deviations once an item lands.

Items are ordered by value-per-effort for a deployment that talks to the stock
`fprime-gds`: consolidation and small unblocked gaps first, then the larger
subsystems, then the strategic changes.

## Done in phase 2 (this branch)

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

## Next: small, unblocked

1. **Config constants into `fprime-config`.** Several components carry
   module-level consts flagged "migrate when config is next touched":
   `FW_FILE_BUFFER_MAX_SIZE`, `FileDownCompletePorts`, `FILEDOWNLINK_*`,
   `FileManagerConfig::*` (`file_uplink.rs`, `file_downlink.rs`,
   `file_manager.rs`), `CONFIG_CRC_FILE_READ_BLOCK` (`fprime-utils`),
   `Fw::StringFormatStatus` (`file_manager.rs` → `fprime-fw::enums`). Pure
   moves with re-exports; no behavior change.
2. **`FileManager` sandbox (`resolveInSandbox`).** Upstream now resolves every
   command path through `Os::SandboxedFile` before use. Port the
   `configureSandbox()` entry point and the resolve step onto the moved
   `fprime-os` helpers; the `Ref` topology leaves it fail-open as upstream's
   `FileHandling` subtopology does.
3. **`Os::ValidateFile` in `fprime-os`.** `ComLogger` carries an inline copy
   (`create_validation`, big-endian `HashBuffer` sidecar) and `CmdSequencer`
   dropped a dead include of it. Move it next to `crc_checker`'s
   native-endian sidecar so the two encodings are documented side by side.
4. **`Os::Cpu` / `Os::Memory`.** `SystemResources` samples `/proc` behind a
   `ResourceSampler` trait; lift the Linux sampler into `fprime-os` with the
   C++ `Cpu`/`Memory` API shape.
5. **`ComStub` async driver path** (`drvAsyncSendOut` /
   `drvAsyncSendReturnIn`), gated on an async byte-stream driver existing;
   the `LinuxUartDriver` is the natural first consumer.
6. **`CmdSequencer` `formats/AMPCSSequence`.** The `Sequence` trait already
   allows a second format; port the AMPCS record layout and its `.CRC32`
   sidecar.

## Then: larger subsystems

7. **`Svc::BufferAccumulator`.** The `Ref` data-product chain wires
   `dpMgr.productSendOut` straight into `dpWriter`; upstream interposes a
   `BufferAccumulator` with a fill/drain state machine and a
   `BA_NumQueuedBuffers` channel. Port it and restore the upstream topology.
8. **`Svc::ActiveTextLogger` file logging.** Only the console path exists.
   Port `LogFile` (`set_log_file`, max-size rotation, the `%d` suffixing).
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
15. **An FPP front end.** Out of scope by design; if it ever lands it should
    emit the existing macro layer rather than replace it.
