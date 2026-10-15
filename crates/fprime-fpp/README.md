# fprime-fpp — `fpp-to-rust`

An [FPP](https://github.com/nasa/fpp) front end and a Rust back end for
this workspace. Zero third-party dependencies, like everything else here.

```
.fpp files ─lexer─▶ tokens ─parser─▶ AST ─include─▶ AST (spliced, state enums added)
           ─analysis─▶ model (symbols, types, values, components, instances, topologies)
           ─codegen─▶ Rust source against fprime-config / fprime-fw / fprime-comp
```

## Front end

The lexer, parser, include resolution and semantic analysis mirror the
reference compiler production for production (`Lexer.scala`, `Parser.scala`,
`EnterSymbols`, `CheckUses`, `EvalConstantExprs`, `CheckComponentDefs`,
`ResolveTopology`, ...):

- the complete grammar: modules, constants, enums, structs, arrays, aliases,
  abstract types, ports, interfaces, components (every member kind), instances,
  topologies (direct and pattern graphs, imports, topology ports, telemetry
  packet sets), state machines, `locate` specifiers, `dictionary` markers;
- FPP's newline discipline, `\` continuations, `$`-escaped identifiers,
  triple-quoted strings, annotations;
- `include` splicing with cycle detection and chained locations;
- name groups and nested scopes, reopened modules, on-demand resolution with
  cycle detection, the reference's type-conversion and common-type rules,
  the implicit `State` enum of state machines;
- implicit numbering exactly as the reference: opcodes, event/channel/param/
  container/record ids, parameter set/save opcodes;
- interface imports, topology imports (transitive instances and
  connections), pattern expansion (`command`, `event`, `telemetry`,
  `text event`, `time`, `health`, `param`), matched and general port
  numbering, connection direction/type checks.

Verified against every `.fpp` under upstream F Prime (the whole framework
model plus the `Ref` deployment resolve: 85 components, 70 instances, 18
topologies) and the reference compiler's own `fpp-to-cpp` test corpora.

## Back end

| FPP | Rust |
|-----|------|
| `module M` | `pub mod M` (nested; FPP names kept verbatim) |
| `constant` | `pub const` (scalars, strings, anonymous arrays) or a `pub fn` returning the value (named arrays/structs) |
| `enum` | `fprime_fw::fpp_enum!` |
| `struct` | `fprime_fw::fpp_struct!` (`x: [n] T` members get a helper `fpp_array!` type `S_x_Array`) |
| `array` | `fprime_fw::fpp_array!` |
| `type T = U` | `pub type T = U` |
| `type T` | a `pub use` of the bound Rust type (abstract types must be bound) |
| `port P` | `pub trait PPort: Send + Sync { fn invoke(&self, port_num, ..) [-> R] }` |
| `component C` | `CBase`, `CHandlers`, `CComponent`, adapters, `impl_c_component!` (below) |
| `topology T` | `TTopology { instances }` with `set_id_bases`, `init`, `connect`, `reg_commands`, `load_parameters`, `start_tasks`/`exit_tasks`/`join_tasks` |
| `serial` port | input: a `fprime_comp::SerializePort` adapter; output: `OutputPort<dyn SerializePort>`; each serializable `port P` also gets `fprime_comp::serializable_port!` |
| `telemetry packets S` in `T` | `T_STlmPackets` with `PACKET_LIST` / `OMITTED_CHANNELS` for `TlmPacketizer::set_packet_list` |
| `deployment topology T` / `system S` | with `--dict`: `TTopologyDictionary.json` / `SSystemDictionary.json` |

**Bindings.** Framework definitions the workspace hand-writes are never
regenerated: config aliases map to `fprime_config`, `Fw.Time`/`Fw.Buffer`/the
buffers/the `Fw` enums to `fprime_fw`, and `Fw.Cmd`, `Fw.Log`, `Svc.Sched`,
... to the `fprime_comp` port traits. The default table is
`codegen::Bindings::framework()`; `--bind-type FPP=RUST[:kind]` and
`--bind-port FPP=RUST` extend it. A port binding may carry per-parameter
passing modes where a hand-written trait deviates from the default rules
(`Fw.DpGet`'s buffer is an out-parameter; `Fw.BufferSend`'s moves).

**Parameter passing.** FPP `ref` → `&mut T`; a value parameter is by value
for `Copy` types (primitives, enums) and `&T` otherwise; serializable
buffers (`CmdArgBuffer`, ...) are always `&mut T`; `Fw.Buffer` is always
owned and crosses queues by escrow token.

### Components

For `queued component Sensor { ... }` the back end emits, in the
component's module:

- `SensorBase` — the C++ `SensorComponentBase`: the kind-specific core
  (`PassiveBase` / `QueuedBase` / `ActiveBase`), `CmdGlue`/`EventGlue`/
  `TlmGlue`/`PrmGlue` as needed, the data-product output ports, every general
  output port (`OutputPort<dyn P>` or an array of them), event throttles, the
  parameter store, the `update on change` telemetry cache, the guarded-port
  mutex and a `BufferEscrow` when an async port carries `Fw.Buffer`.
  Constants: `OPCODE_*`, `EVENTID_*`, `*_THROTTLE`, `CHANID_*`, `PARAMID_*`,
  `CONTAINER_ID_*`, `CONTAINER_PRIORITY_*`, `RECORD_ID_*`,
  `SIZE_OF_*_RECORD` / `size_of_*_record(n)`, `MSG_TYPE_*`, `MSG_SIZE`.
  Methods: `new`, `init`, `set_id_base`, `id_base`, `reg_commands`,
  `<port>_out(port_num, ..)` / `is_connected_<port>_out`,
  `log_<severity>_<event>` (+ `_throttle_clear`), `tlm_write_<chan>` /
  `tlm_write_<chan>_at`, `param_get_<param>`, `dp_get_<container>`,
  `dp_request_<container>`, `dp_send` / `dp_send_at`,
  `serialize_record_<record>`.
- `SensorHandlers` — what the implementation provides: `base()`, a handler
  per input port (`<port>_handler`), per command (`<cmd>_cmd_handler`; it
  answers through `base().cmd.cmd_response`), per internal port
  (`<port>_internal_interface_handler`), the product-receive handler, and
  hooks with default bodies (`parameter_updated`, `parameters_loaded`,
  `<port>_pre_msg_hook`, `<port>_overflow_hook`).
- `SensorComponent` — a blanket extension over `SensorHandlers`: the
  input-port factories (`<port>(self: &Arc<Self>, port_num) -> PortRef`),
  `<port>_internal_interface_invoke`, `load_parameters`, `cmd_dispatch` and
  `dispatch_message`.
- `impl_sensor_component!(MyImpl);` — implements `ComponentDispatch` (and
  `ActiveComponent`) for the implementation type. With `--include-path`
  the macro names its traits absolutely; otherwise they must be in scope.

Wire formats are those of the hand-written components: queue envelopes
`[msg_type i32][port_num i16][args]`, command messages nesting the argument
buffer with a length prefix, `FORMAT_ERROR` on short/residual command bytes,
`INVALID_OPCODE` for unknown opcodes, parameter `PARAM_SET`/`PARAM_SAVE`
handled by the generated dispatch.

### Topologies

Each instance becomes an `Arc<Impl>` field, where `Impl` is the instance's
`type "path"` clause (interpreted as a Rust path) or
`<impl prefix><Module>::<Component>`. Connections use the resolved port
numbers, so pattern graphs and port arrays behave exactly as in the C++
topology. Every instance must be of a generated component.

### Serial ports

On item 9's runtime (`fprime_comp::serial_port`). A port definition with no
return value and no `Fw.Buffer` parameter is followed by
`serializable_port! { port: PPort; args { val x: U, ref s: S, .. } }`
(the C++ `<P>PortSerializer` and the `FW_PORT_SERIALIZATION` halves of the
typed port: arguments in order, big-endian). In a component:

- a `serial` input port `p` gets `p_handler(&self, port_num, buffer: &mut
  dyn SerBufAny)` and a factory returning `PortRef<dyn SerializePort>`; the
  adapter answers `SerializeStatus::Ok` (C++ `InputSerializePort`). Sync
  and guarded ports call the handler; an async one queues
  `[msg type][port number][u16 size][bytes]` and the dispatch hands the
  handler a view of the bytes. The C++ async serial pre-message hook is
  never called upstream, so only `p_overflow_hook` is generated. A
  component with async serial ports takes `init(queue_depth, msg_size)`
  (C++ `init(queueDepth, msgSize)`: the queue message is
  `max(msg_size + 6, MSG_SIZE)`), and the generated topology passes
  `FW_COM_BUFFER_MAX_SIZE` (C++ passes the instance id, which leaves no
  room for a call);
- a `serial` output port `p` is an `OutputPort<dyn SerializePort>` (or an
  array) with `p_out(port_num, buffer) -> SerializeStatus` (asserts if
  unconnected) and `is_connected_p_out`.

The topology connects a typed (or special) output to a serial input with
`connect_serial`, a serial output to a typed (or special) input with
`connect_typed`, and serial to serial directly. A connection through a port
that returns a value is rejected by the analysis (the reference's
`checkSerialWithTypedInput`, special `prmGetOut`/`productGetOut` included);
one through a port carrying an `Fw.Buffer` is a generation error.

### Telemetry packet sets

`telemetry packets S` of topology `T` becomes a module `T_STlmPackets`
(C++ `T_STlmPacketsAc`): `PACKET_LIST: &[TlmPacketizerPacket]` (packets in
name order, channels in declaration order as `(global id, max serialized
size)`, with id and group), `OMITTED_CHANNELS` (in qualified-name order) and
the C++ file-local tables (`ChannelIds`, `ChannelSizes`, `PacketIds`,
`PacketGroups`, `PacketDataSizes`, `PacketEntryArrays`, `Packets`,
`SizeBounds`) with the `static_assert`s as compile-time assertions. The set
is checked as the dictionary checks it. As in C++, the topology does not
configure a packetizer; a deployment calls
`tlm_send.set_packet_list(T_STlmPackets::PACKET_LIST,
T_STlmPackets::OMITTED_CHANNELS, level)` (upstream does it in
`phase Fpp.ToCpp.Phases.configComponents` code). The types come from
`Options::packetizer_path` (default `::fprime_svc::tlm_packetizer`).

## JSON dictionary (`--dict DIR`)

The second back end writes the ground dictionary the stock F Prime ground
data system reads: the `fpp-to-dict` format, `dictionarySpecVersion`
1.0.0, one `<Topology>TopologyDictionary.json` per deployment topology and
one `<System>SystemDictionary.json` per `system`. Its content follows
`DictionaryJsonEncoder.scala`:

- `typeDefinitions` and `constants`: the deep closure of the types and
  constants named by the commands, events, telemetry channels, parameters,
  records and containers of every instance (`analysis::uses`), plus the
  implied uses of dictionary generation (`FwOpcodeType`, `Fw.DpState`,
  `FW_FIXED_LENGTH_STRING_SIZE`, ...; missing ones are errors) and every
  `dictionary`-marked definition (checked to be displayable);
- `commands` (with the implicit `<PARAM>_PRM_SET`/`_PRM_SAVE`),
  `parameters`, `events` (format, throttle, `every`), `telemetryChannels`
  (limits), `records`, `containers`, keyed by `base id + local id` and
  sorted; duplicate global ids are errors;
- `telemetryPacketSets`: the topology's own packet sets with implicit
  packet ids, and the check that every channel is used or omitted.

`-p`, `-f` and `-l` set the project version, framework version and
library versions of the metadata. `tests/dictionary.rs` compares the
output structurally with the reference compiler's `.ref.json` files for
its whole `fpp-to-dict` test corpus (`tests/dict`, copied from nasa/fpp)
and checks its error cases. `src/json.rs` is the stdlib-only JSON value,
printer and parser.

## Not generated (yet)

State machine instances (`Fw/Sm` autocoding); instance init specifiers
(`phase ... "code"`); instances of hand-written components in a generated
topology. Anonymous struct constants and abstract-type values are emitted
as comments.

## Using it

Command line:

```bash
cargo run -p fprime-fpp --bin fpp-to-rust -- \
    -i default/config/FpConfig.fpp -i cmake/platform/unix/Platform/PlatformTypes.fpp \
    -i Fw/Types/Types.fpp -i Fw/Cmd/Cmd.fpp ... \
    --include-path generated --impl-prefix crate:: \
    -o src/generated.rs  MyComponent.fpp MyTopology.fpp
fpp-to-rust --check ...   # analyze only
fpp-to-rust --syntax ...  # parse only
fpp-to-rust --dict out/ -p 1.0.0 -f 4.0.0 -i ... MyTopology.fpp   # JSON dictionaries
```

From a build script (see `crates/fprime-fpp-demo/build.rs`): parse with
`Session`, `analysis::analyze`, `codegen::generate` with `Options`, write to
`$OUT_DIR`, `include!` it.

## Layout

```
src/lexer.rs        tokens + newline discipline
src/ast.rs          the AST (mirrors Ast.scala)
src/parser.rs       recursive descent (mirrors Parser.scala)
src/include.rs      include splicing
src/transform.rs    implicit state enums
src/analysis/       symbols, types/values, eval, format strings, components, topologies
src/analysis/uses.rs  deep used-symbol closure (dictionary contents)
src/codegen/        bindings + driver, names, types, ports, component, topology,
                    serial (serial-port helpers), packets (telemetry packet tables)
src/codegen/dictionary.rs  the JSON dictionary back end
src/json.rs         JSON value, printer, parser
src/bin/fpp_to_rust.rs
tests/analysis.rs   reference-rule tests on inline models
tests/codegen.rs    back-end tests on inline models (serial wiring, packet tables)
tests/dictionary.rs the reference fpp-to-dict corpus (tests/dict)
```
