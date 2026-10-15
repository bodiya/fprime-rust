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
  model and the Ref deployment analyze. Serial ports, packet tables, state
  machines, init specifiers and hand-written components in topologies
  landed with item 15. See `crates/fprime-fpp/README.md`.

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
- **`Svc::GenericHub`** (item 9), with the serialize ports it needs:
  `fprime_comp::serial_port` ports `Fw::InputSerializePort` /
  `OutputSerializePort` and the generated `invokeSerial` paths
  (`serializable_port!`, `connect_serial`, `connect_typed`), so any typed
  call without a return value or `Fw::Buffer` can cross a hub and be
  replayed on a typed port. The hub's `[type][port][size][payload]` wire
  format, its buffer moves and the upstream unit tests are ported; an
  integration test joins two deployments (remote commanding through a
  `CmdDispatcher`, its events, a ping round trip, a buffer). Upstream
  undersizes the `eventIn`/`tlmIn` scratch buffers and asserts on 505/506-byte
  argument buffers; the Rust hub forwards them. `Fw::ObjRegistry` turned
  out to be unused by the hub and stays unported. The upstream Ref does not
  instantiate a hub.
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
- **zlib data-product compression** (item 14). `Svc::DpCompressProc` (the
  in-place chunking state machine that rewrites a container as
  `CompressionRecord`s, the `CompressChunk` port, `CompressionAlgorithm` and
  `CompressionMetadata`) is always built; `Svc::DpZLibCompressor` is behind
  `fprime-svc`'s off-by-default `zlib` feature, the workspace's first
  third-party dependency (pure-Rust `miniz_oxide`, so no C toolchain). With
  the feature off the compressor is not compiled, as upstream's CMake skips
  it without libz, and an unwired `DpCompressProc` passes products through.
  The deflate bytes differ from libz's but are standard zlib streams:
  `fprime-dp` (fprime-gds 4.3.1) decodes a Rust-written compressed `.fdp`
  back to the original records. Upstream's Ref leaves
  `DpCompression.Subtopology` commented out, so the Rust Ref and its
  dictionary are unchanged; `fprime_config::dp_compression` carries the
  subtopology constants.

- **FPP back end: serial ports and telemetry packet tables** (item 15,
  first part). `fpp-to-rust` targets item 9's
  `fprime_comp::serial_port` runtime: every generated port definition
  without a return value or an `Fw.Buffer` gets a `serializable_port!`;
  `serial` input ports are `SerializePort` adapters (sync, guarded and
  async, the latter queued as `[type][port][u16 size][bytes]` with the
  overflow hook) whose handlers get the serialized call, `serial` outputs
  are `OutputPort<dyn SerializePort>`s with `<port>_out` returning the
  callee's status, and the topology wires typed → serial with
  `connect_serial` and serial → typed with `connect_typed`; the analysis
  enforces the reference's no-return-value rule. Each topology
  `telemetry packets` set becomes `<Topology>_<Set>TlmPackets` with the
  `PACKET_LIST` / `OMITTED_CHANNELS` that `TlmPacketizer::set_packet_list`
  takes, ordered and checked as `TlmPacketSetCppWriter` does. Like the
  C++ autocoder, the topology does not call `set_packet_list` itself
  (upstream does it in instance init code, see below). `fprime-fpp-demo` proves both
  end to end: a queued `Relay` between typed ports, and a real
  `TlmPacketizer` packetizing the generated sensor's telemetry from the
  generated tables.

- **Broader GDS cross-check** (item 16). `tools/gds-crosscheck.py` runs
  named checks (`--checks` selects a subset): `commands` (as before),
  `params` (`_PRM_SET`/`_PRM_SAVE`/`PRM_SAVE_FILE`, `fprime-prm-decode`,
  the value back in the component after a restart, a `fprime-prm-write`
  file loaded at boot), `files` (`fprime-cli file-uplink` incl. an empty
  file, `SendFile`, `SendPartial`), `dp` (both producers' products
  downlinked by `dpCat`, `fprime-dp validate`/`decode`, the file chunks
  reassembled), `seq` (`fprime-seqgen` binaries, relative and absolute
  time tags, run by `cmdSeq`) and `ccsds` (commands, events, telemetry and
  files over `space-packet-space-data-link`). The last needed a CCSDS Ref:
  `fprime-ref --framing ccsds` wires upstream's `ComCcsds` chain
  (`spacePacketFramer -> tmFramer`, `CcsdsTcFrameDetector -> tcDeframer ->
  spacePacketDeframer`, `apidManager`), minus the then-unported
  `ComAggregator` (item 12 has since inserted it). Every check passed against fprime-gds 4.3.1 without a
  Rust fix; `SendPartial` lands at its absolute file offsets on the
  ground, as upstream's does.
- **Upstream pin** (item 17). `FPRIME_UPSTREAM_COMMIT` names the
  nasa/fprime commit the port tracks (4dee010c30, 2026-09-11);
  `generate-dictionary.sh` refuses any other (or a modified) checkout
  without `--allow-other-commit` and writes `metadata.frameworkVersion`
  from the commit it used. `crates/fprime-ref/tests/upstream_ids.rs` is the
  automated id comparison: the dictionary records the pin, and every
  command, event, channel, parameter, record and container of every Ref
  instance matches a Rust constant (base id + component id) and vice
  versa. See "Syncing with upstream" below.
- **CCSDS AOS / `ComAggregator` / SDLS** (item 12). The CCSDS base was
  first brought up to 4dee010: `FrameContext.firstHeaderPointer`, the full
  TM data field as `AggregationSize`, `TmFramer` taking the First Header
  Pointer from the context and sending a full data field without idle
  fill, `TcDeframer` carrying the VCID in the context, and
  `Utils::IdlePacket`. On top: `Svc::ComAggregator` (the FPP
  `AggregationMachine` declared with item 11's `fpp_state_machine!`, its
  instance's signals on the queue as the generated C++ messages, the
  `fill` buffer escrowed in an `EscrowedBuffer`; optional TM packet
  spanning), now
  between `spacePacketFramer` and `tmFramer` in `fprime-ref --framing
  ccsds` with the timeout on rate group 1; `AosFramer` / `AosDeframer`
  (M_PDU packing and striping, SPP/EPP extraction, VC frame-count gaps,
  spanning-packet reassembly); the SDLS framework (`CcsdsSdlsFramer` /
  `CcsdsSdlsDeframer`, `SdlsSaRouter`, `SdlsFileKeyManager`, the
  `ClearTextEncryptor` / `ClearTextDecryptor` null ciphers, the TM/TC AAD
  masks); and `AesGcmEncryptor` / `AesGcmDecryptor` on RustCrypto
  `aes-gcm` behind `fprime-svc`'s off-by-default `aes-gcm` feature
  (without it they are not compiled). Every upstream unit test is ported,
  plus the AES-256 GCM-spec vectors, and
  `crates/fprime-svc/tests/ccsds_sdls_chain.rs` runs upstream's
  `ComCcsdsSdls` wiring end to end with both cipher pairs.
- **State machines, `Fw/Sm`** (item 11). `fprime_comp::fpp_state_machine!`
  declares an FPP internal state machine — hierarchical states with
  entry/exit actions, initial transitions, choices, guards, typed signals
  and actions — and generates the `State`/`Signal` enums with the
  `fpp-to-cpp` discriminants (both sorted by name), the `Handlers` trait
  and `init`/`get_state`/`send_signal_*`. `fprime_comp::sm` is the runtime
  and the component side: the byte-exact signal message
  `[msg_type][port 0][sm_id][signal][data]` enqueued with the instance's
  priority and queue-full policy and dispatched on the component thread,
  the overflow hook, `component_sm_ids!`, and external (`init`/`update`)
  machines with `Fw::SmSignalBuffer`. The FppTest `state_machine` suites
  are ported, and `tools/sm-traces.py` golden traces check every leaf
  state, signal and guard path of the `FpySequencer`, `ComAggregator` and
  hierarchical FppTest machines against the reference compiler's C++.
  The FPP front end now numbers the implicit `State` enum in name order,
  as `fpp-to-cpp` does.
- **FPP back end: state machines, init specifiers, hand-written
  components** (item 15). The front end checks state machines as the
  reference's `CheckStateMachineSemantics` does (target scoping, initial
  transitions, signal uses, reachability, choice cycles, inferred choice
  types, typed call sites); `fpp-to-rust` emits `fpp_state_machine!` for
  every internal machine and the whole instance side of item 11's contract
  (`SM_ID_*` sorted by name, machines initialized before the queue, the
  signal messages with each instance's priority and policy, overflow
  hooks, `smDispatch`, external machines bound to Rust types); a struct
  member of type `Fw.Buffer` becomes an `EscrowedBuffer`. Instance init
  specifiers carry Rust code, placed per phase where `fpp-to-cpp` places
  the C++ (the phase methods, `new` for `instances`, `T_ConfigConstants` /
  `T_ConfigObjects` modules), with `setup`/`teardown` in the C++ order;
  hand-written components can be bound (`--bind-component`;
  `Svc.TlmPacketizer` by default) and instantiated in generated
  topologies. `fprime-fpp-demo` generates upstream's FpySequencer,
  ComAggregator and FppTest machines and replays all of item 11's
  `fpp-to-cpp` golden traces against them, runs the FppTest instance
  tests on generated components, and its `Ops` topology holds a
  controller with an internal and an external machine and the
  hand-written `TlmPacketizer`, configured by a `configComponents` init
  specifier from the generated packet table. The Ref still wires by hand,
  so its dictionary and wire behavior are unchanged.
- **`Svc::FpySequencer`** (item 10). The Fpy bytecode sequencer, whole:
  the sequence file (header, argument specs, statements, CRC-32 footer)
  read stage by stage into the allocated buffer and validated with every
  upstream event; the `SequencerStateMachine` on `fpp_state_machine!` (the
  definition the component uses is now the one `tools/sm-traces.py`'s
  golden traces check, from `fprime-svc`); the stack machine with all 81
  directives — every stack op, the frame-relative and absolute loads
  and stores, `CALL`/`RETURN`, `GET_FIELD`/`PEEK`, waits, telemetry and
  parameter reads, `POP_EVENT`, `POP_SERIALIZABLE` on the `serialOut`
  serial ports, and `SET_SEED`/`PUSH_RAND` on a port of `std::mt19937`
  and `std::seed_seq` that reproduces libstdc++'s numbers; `CONST_CMD`/
  `STACK_CMD` dispatch with command-UID response matching; statement
  timeouts; breakpoints, `BREAK`/`CONTINUE`/`STEP` and
  `DUMP_STACK_TO_FILE`; both parameters (synchronous `PRM_SET`/`PRM_SAVE`,
  echoed on telemetry); every queue priority of the FPP model. Commands,
  events, channels and parameters carry the upstream ids (checked against
  `fpp-to-rust --dict` on the upstream model). The upstream unit tests are
  ported (about 270 Rust tests), plus a sequence run on the component's
  own thread. Upstream's Ref does not instantiate it, so neither does the
  Rust Ref, and fprime-gds 4.3.1 has no Fpy compiler: sequences are built
  byte by byte, as upstream's tests do. Porting it showed that
  `SequenceArgumentsMaxSize` is 12 at the pin, not 255; `Svc.SeqArgs` was
  corrected for `CmdSequencer` too.

## Next: small follow-ups

These surfaced while porting the items above. None changes the wire format of
the Ref deployment.

- **`Utils::Hash` file-extension helpers.** `getFileExtensionString`,
  `addFileExtension` and `getFileExtensionLength` (exercised by upstream's
  `CRCCheckerTester` GoldenValue case) have no Rust equivalent yet.
- **`Drv::ByteStreamBufferAdapter`.** The synchronous sibling of the ported
  `AsyncByteStreamBufferAdapter`.

## Strategic

13. **`no_std` OSAL seam.** `fprime-os` keeps the seam; making `fprime-fw`
    and `fprime-comp` `no_std` + `alloc` is the first real test of it.

## Syncing with upstream

The port tracks the single nasa/fprime commit in `FPRIME_UPSTREAM_COMMIT`.
To move it:

1. Check the new commit out in an fprime clone and list what changed in
   the ported trees: `git diff <pin>..<new> -- Fw Svc Drv Os Utils
   default/config Ref`.
2. Port the C++ changes into the Rust components (their `docs/api-notes.md`
   sections name the upstream files), including new or renumbered
   `OPCODE_*`/`EVENTID_*`/`CHANID_*`/`PARAMID_*` constants.
3. Regenerate the dictionary from the new checkout with
   `crates/fprime-ref/fpp/generate-dictionary.sh --allow-other-commit
   <checkout>` and run `cargo test -p fprime-ref --test upstream_ids`: it
   lists every id that differs, and every dictionary entry without a Rust
   constant (add new ones to the table in that test). Repeat 2-3 until it
   passes; `upstream_pin` keeps failing until step 4.
4. Write the new full hash into `FPRIME_UPSTREAM_COMMIT`, regenerate
   without the override (which records the new `frameworkVersion`), and
   update the commit named in the docs.
5. Run `cargo test --workspace` and `tools/gds-crosscheck.py` (with the
   fprime-gds release matching the new upstream, if it moved).
