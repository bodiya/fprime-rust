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

## Not generated (yet)

State machine instances (`Fw/Sm` autocoding), serial ports, telemetry
packet sets and the JSON dictionary. Anonymous struct constants and
abstract-type values are emitted as comments.

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
src/codegen/        bindings + driver, names, types, ports, component, topology
src/bin/fpp_to_rust.rs
tests/analysis.rs   reference-rule tests on inline models
```
