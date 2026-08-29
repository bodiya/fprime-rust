# fprime-rust coding conventions

Binding rules for all code in this workspace. `ARCHITECTURE.md` defines what
to build; this file defines how the code must look and behave.

## Naming

- C++ `Fw::CamelCase` types keep their names (`ComBuffer`, `CmdArgBuffer`,
  `TlmPacket`, `TimeInterval`, `CmdResponse`). C++ methods/functions map to
  `snake_case` (`serializeTo` → `serialize_to`, `resetSer` → `reset_ser`,
  `getDiffUsec` → `get_diff_usec`).
- Config type aliases keep their exact F Prime names (`FwSizeType`,
  `FwIndexType`, ...) — grep-compatibility with the C++ tree matters more
  than Rust alias style here. Allow `non_camel_case_types` where needed.
- Enum variants: C++ `FW_SERIALIZE_OK` / `MSG_DISPATCH_OK` style becomes Rust
  `Ok` / `Empty` CamelCase variants; the enum name carries the context.
  Discriminant values MUST match the C++ numeric values (write them
  explicitly: `Ok = 0`).
- Component port/handler names keep FPP names snake_cased:
  `seqCmdBuff` port → `seq_cmd_buff_in()` adapter + `seq_cmd_buff_handler()`.
- Event/command/channel ID constants: `EVENTID_*`, `OPCODE_*`, `CHANID_*`
  associated consts on the component, values = the FPP-relative IDs.

## Error handling

- Fallible framework APIs return status enums (`SerializeStatus`,
  `os::queue::Status`, ...) marked `#[must_use]` — NOT `Result` — mirroring
  C++ status flow 1:1. Use `fw_try!(expr)` to early-return non-OK.
- `fw_assert!` is for programmer-error invariants only (the places C++ uses
  `FW_ASSERT`): out-of-range port numbers, invoking unconnected required
  ports, corrupted internal state. Never for recoverable runtime conditions.
- No `panic!`, `unwrap()`, or `expect()` in framework code paths. Allowed:
  inside `fw_assert!` itself, in `#[cfg(test)]`, and for init-time
  invariants that C++ also asserts (document each with a comment referencing
  the C++ behavior). Mutex poisoning: `.lock()` failures may panic (a
  poisoned lock means a thread already panicked — matches FW_ASSERT
  philosophy). Note the C++ FW_ASSERT fail-stop is process-wide: release
  builds set `panic = "abort"` in the workspace profile so any panic
  (fw_assert or poison) takes the whole deployment down like the C++
  `assert(false)`; dev/test builds unwind for testability.

## Memory discipline

- Allocate at initialization (`new`, `create`, `configure`); steady state
  recycles fixed storage. No collections that grow unboundedly at runtime.
  `Vec` is fine when `with_capacity` at init and never grown after; prefer
  `Box<[T]>` / arrays to make that structural.
- `#![forbid(unsafe_code)]` in every crate.
- Buffers move; they are never cloned in steady state except where the C++
  copies too (queue message serialization copies by design).

## Wire-format fidelity

- Big-endian default everywhere; every serialize/deserialize takes
  `Endianness` explicitly (call sites pass `Endianness::Big` or use the
  `_be` convenience wrappers).
- Never invent length prefixes or counts that the C++ format omits
  (LogPacket/TlmPacket args are raw), never omit ones it has (strings,
  nested buffers).
- Every wire format gets a unit test with literal expected bytes.
- When implementing a component, re-read its section in
  `docs/cpp-analysis/*.md` AND the C++ source under `/home/user/fprime/`
  when in doubt. The gotcha lists are normative.

## Concurrency

- Component state: `Mutex<XState>` inside the component struct; handlers take
  `&self`. Lock scope = what the C++ component's guarded mutex / explicit
  lock() covers, no wider. Invoke output ports after dropping the state lock
  unless the C++ code holds its mutex across the call.
- Cross-thread flags that C++ leaves as racy plain fields (e.g.
  ActiveRateGroup `m_cycleStarted`) become relaxed atomics.
- No `std::sync::mpsc` for component queues — use `os::Queue` (priority
  semantics required).

## Documentation & style

- Every public item gets a doc comment; component modules start with a
  `//!` header naming the C++ component they port and its analysis doc.
- Comments explain constraints and C++-parity quirks (with a pointer like
  `// C++ parity: CmdPacket.cpp resets arg buffer for zero-arg commands`),
  not what the next line does.
- rustfmt defaults; clippy clean (`cargo clippy --all-targets -- -D warnings`)
  with narrowly-scoped `#[allow]`s only where fidelity demands
  (e.g. `clippy::too_many_arguments` on port invokes).

## Testing

- Unit tests live in `#[cfg(test)] mod tests` next to the code; integration
  tests in `tests/`. No third-party test crates.
- Test names describe the property: `bool_deserialize_rejects_nonstandard_bytes`.
- Each component test must cover: happy path, every documented gotcha for
  that component, and every error/status branch reachable without OS fault
  injection.

## Commits

- Conventional prefix + scope: `fw: implement serialization core`,
  `svc: port CmdDispatcher`. Reference the C++ component being ported in
  the body.
