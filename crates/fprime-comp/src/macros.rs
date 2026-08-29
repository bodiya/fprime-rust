//! # Component codegen: `component_msg_types!`, `input_port_adapter!`,
//! `async_input_port_adapter!`
//!
//! Declarative (`macro_rules!`) replacements for the most repetitive parts of
//! what the C++ FPP autocoder emits into `<Comp>ComponentBase` — the static
//! input-port thunks, the async queue-message envelope, and the message-type
//! discriminants (see `docs/cpp-analysis/fpp-autocoder.md`, sections
//! "Generated typed-component base: ports" and "Async queue message wire
//! contract + dispatch"). No proc macros and no third-party crates.
//!
//! The hand-written contract these macros generate is exactly the one
//! `tests/example_component.rs` spells out; a component may use the macros
//! for the ports that fit and hand-write the rest, in any mix.
//!
//! ## Vocabulary
//!
//! The macro keys are the FPP model's own words: `component`, `port`,
//! `input`, `handler`, `priority`, `queue_full` (FPP `assert`/`drop`/`block`/
//! `hook`), `pre_msg_hook`, `msg_type`, `msg_size`.
//!
//! Port arguments are declared with a *passing mode* prefix, because it
//! decides both the trait-method signature and the codec:
//!
//! | mode | trait parameter | queue write | queue read |
//! |------|-----------------|-------------|------------|
//! | `val x: T` | `x: T` | `Serialize` | `Deserialize` |
//! | `ref x: T` | `x: &T` | `Serialize` | `Deserialize` |
//! | `mut x: T` | `x: &mut T` | `Serialize` | `Deserialize` |
//! | `buf x: T` | `x: &mut T` | nested buffer (`u16` len + bytes) | nested buffer |
//!
//! `mut` is for the C++ `ref` parameters that are not buffers (`Fw::Time&`,
//! `Fw::Success&`). On an ASYNC port the value is copied into the queue
//! message and the caller's variable is never written back — C++ parity, the
//! generated code serializes it the same way.
//!
//! ## What is deliberately NOT generated
//!
//! - **Async ports carrying an owned `Fw::Buffer`.** C++ serializes the raw
//!   pointer; this port uses [`BufferEscrow`](crate::escrow::BufferEscrow),
//!   whose deposit/claim pairing is component state, not a per-port pattern.
//!   Those adapters stay hand-written.
//! - **The `dispatch_message` switch itself.** It is the component's own
//!   `doDispatch`; the generated `<name>_deserialize` helper is what keeps
//!   the read side byte-identical to the write side.
//! - **Adapters whose handler takes a different argument list than the
//!   port** (e.g. dropping an unused `context`): forwarding is 1:1 or the
//!   macro does not apply.
//! - Command/event/telemetry glue: [`CmdGlue`](crate::glue::CmdGlue),
//!   [`EventGlue`](crate::glue::EventGlue) and [`TlmGlue`](crate::glue::TlmGlue)
//!   already collapse that boilerplate into ordinary calls.

/// Trait-method parameter type for a port argument's passing mode; internal
/// to [`input_port_adapter!`] / [`async_input_port_adapter!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_port_arg_ty {
    (val $ty:ty) => {
        $ty
    };
    (ref $ty:ty) => {
        &$ty
    };
    (mut $ty:ty) => {
        &mut $ty
    };
    (buf $ty:ty) => {
        &mut $ty
    };
}

/// Queue-message write for one port argument; internal to
/// [`async_input_port_adapter!`].
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_port_arg_write {
    ($msg:ident, val $arg:ident) => {
        $msg.serialize(&$arg, $crate::fw::Endianness::Big)
    };
    ($msg:ident, ref $arg:ident) => {
        $msg.serialize($arg, $crate::fw::Endianness::Big)
    };
    ($msg:ident, mut $arg:ident) => {
        $msg.serialize(&*$arg, $crate::fw::Endianness::Big)
    };
    ($msg:ident, buf $arg:ident) => {
        $msg.serialize_buffer($arg, $crate::fw::Endianness::Big)
    };
}

/// Queue-message read for one port argument; internal to
/// [`async_input_port_adapter!`]. Mirrors [`__fpp_port_arg_write`] token for
/// token so the two sides cannot drift.
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_port_arg_read {
    ($msg:ident, val $arg:ident) => {
        $msg.deserialize(&mut $arg, $crate::fw::Endianness::Big)
    };
    ($msg:ident, ref $arg:ident) => {
        $msg.deserialize(&mut $arg, $crate::fw::Endianness::Big)
    };
    ($msg:ident, mut $arg:ident) => {
        $msg.deserialize(&mut $arg, $crate::fw::Endianness::Big)
    };
    ($msg:ident, buf $arg:ident) => {
        $msg.deserialize_buffer(&mut $arg, $crate::fw::Endianness::Big)
    };
}

// ---------------------------------------------------------------------------
// component_msg_types!
// ---------------------------------------------------------------------------

/// Declare a component's async queue-message discriminants as associated
/// consts, numbered from 1 in declaration order.
///
/// `0` is the EXIT sentinel ([`msg::EXIT_MSG_TYPE`](crate::msg::EXIT_MSG_TYPE)),
/// so component discriminants start at 1 — one per async input port, async
/// command, and internal interface, exactly as the C++ generated
/// `<Comp>_MSG_TYPE` enumeration does.
///
/// ```
/// use fprime_comp::component_msg_types;
///
/// struct MyComponent;
///
/// component_msg_types! {
///     /// Queue message types (0 is the EXIT sentinel).
///     impl MyComponent {
///         /// `runIn` async input port.
///         MSG_TYPE_RUN_IN,
///         /// `cmdIn` async command port.
///         MSG_TYPE_CMD_IN,
///     }
/// }
///
/// assert_eq!(MyComponent::MSG_TYPE_RUN_IN, 1);
/// assert_eq!(MyComponent::MSG_TYPE_CMD_IN, 2);
/// ```
#[macro_export]
macro_rules! component_msg_types {
    (
        $(#[$meta:meta])*
        impl $comp:ty { $($body:tt)* }
    ) => {
        $(#[$meta])*
        impl $comp {
            $crate::__component_msg_type_consts!(1; $($body)*);
        }
    };
}

/// Numbering muncher for [`component_msg_types!`]; not part of the API.
#[doc(hidden)]
#[macro_export]
macro_rules! __component_msg_type_consts {
    ($idx:expr;) => {};
    ($idx:expr; $(#[$meta:meta])* $name:ident $(, $($rest:tt)*)?) => {
        $(#[$meta])*
        pub const $name: $crate::config::FwEnumStoreType = $idx;
        $crate::__component_msg_type_consts!($idx + 1; $($($rest)*)?);
    };
}

// ---------------------------------------------------------------------------
// input_port_adapter!
// ---------------------------------------------------------------------------

/// Generate a SYNC or GUARDED input port: the adapter struct (the C++
/// generated static thunk), its port-trait implementation forwarding to a
/// named handler on the caller's thread, and the component's factory method
/// used for topology wiring.
///
/// Guarded semantics are the handler's job — it locks the component's state
/// mutex, per the convention in `CONVENTIONS.md`; the adapter is identical
/// either way, exactly as in C++ where only the generated lock differs.
///
/// ```ignore
/// input_port_adapter! {
///     /// `dataIn` — GUARDED `Svc.ComDataWithContext` input.
///     component: FprimeDeframer;
///     adapter: DataInAdapter;
///     port: ComDataWithContextPort;
///     input: pub data_in;
///     handler: data_in_handler;
///     args { val data: Buffer, ref context: FrameContext }
/// }
/// ```
///
/// generates `data_in(self: &Arc<Self>, port_num) -> PortRef<dyn
/// ComDataWithContextPort>` and forwards to
/// `self.data_in_handler(port_num, data, context)`.
///
/// For a port trait whose `invoke` returns a value, add `returns: T;` after
/// `handler:`; the handler must return the same type.
#[macro_export]
macro_rules! input_port_adapter {
    (
        $(#[$meta:meta])*
        component: $comp:ty;
        adapter: $adapter:ident;
        port: $port:path;
        input: $fvis:vis $input:ident;
        handler: $handler:ident;
        $(returns: $ret:ty;)?
        args { $($kind:ident $arg:ident : $aty:ty),* $(,)? }
    ) => {
        #[doc = concat!(
            "Input-port adapter for `", stringify!($input),
            "` (the hand-written equivalent of the C++ generated static thunk)."
        )]
        struct $adapter {
            comp: ::std::sync::Arc<$comp>,
        }

        impl $port for $adapter {
            fn invoke(
                &self,
                port_num: $crate::config::FwIndexType,
                $($arg: $crate::__fpp_port_arg_ty!($kind $aty)),*
            ) $(-> $ret)? {
                // SYNC/GUARDED: runs on the CALLER's thread.
                self.comp.$handler(port_num, $($arg),*)
            }
        }

        impl $comp {
            $(#[$meta])*
            $fvis fn $input(
                self: &::std::sync::Arc<Self>,
                port_num: $crate::config::FwIndexType,
            ) -> $crate::PortRef<dyn $port> {
                $crate::PortRef::new(
                    ::std::sync::Arc::new($adapter {
                        comp: ::std::sync::Arc::clone(self),
                    }),
                    port_num,
                )
            }
        }
    };
}

// ---------------------------------------------------------------------------
// async_input_port_adapter!
// ---------------------------------------------------------------------------

/// Generate an ASYNC input port: the adapter struct, its port-trait
/// implementation building the byte-exact queue envelope and enqueueing it
/// under a given queue-full policy, the component's factory method, and the
/// matching `<name>_deserialize` helper for `dispatch_message`.
///
/// The envelope is
/// `[msg_type i32 BE][port_num i16 BE][args in declaration order]` — see
/// [`msg`](crate::msg). The write and read sides are expanded from the *same*
/// `args { .. }` list, so they cannot drift.
///
/// ```ignore
/// async_input_port_adapter! {
///     /// `CycleIn` — ASYNC `Svc.Cycle` input with the `drop` policy.
///     component: ActiveRateGroup;
///     adapter: CycleInAdapter;
///     port: CyclePort;
///     input: pub cycle_in;
///     deserialize: cycle_in_deserialize;
///     handler: cycle_in_handler;
///     base: active.queued;
///     msg_type: ActiveRateGroup::MSG_TYPE_CYCLE_IN;
///     msg_size: MSG_SIZE;
///     priority: CYCLE_IN_PRIORITY;
///     queue_full: QueueFullPolicy::Drop;
///     args { ref cycle_start: RawTime }
///     pre_msg_hook |comp, _port_num| {
///         comp.cycle_started.store(true, Ordering::Relaxed);
///     }
/// }
/// ```
///
/// Keys:
///
/// - `base:` — the dotted field path from the component to its
///   [`QueuedBase`](crate::queued::QueuedBase) (`active.queued` for an active
///   component, `queued` for a queued one).
/// - `msg_size:` — the component's queue message size, a `usize` const; the
///   envelope buffer is `LinearBuffer<{msg_size}>` and a serialize failure is
///   a `fw_assert!` (C++ parity: the buffer is sized for the worst case).
/// - `queue_full:` — a [`QueueFullPolicy`](crate::msg::QueueFullPolicy)
///   (FPP `assert` / `drop` / `block` / `hook`).
/// - `pre_msg_hook |comp, port_num| { .. }` (optional) — the C++
///   `<port>_preMsgHook`: runs on the SENDER's thread before the enqueue.
///   `comp` binds to `&Component`; the port arguments are in scope too.
/// - `overflow_hook |comp, port_num| { .. }` (optional) — runs on the
///   SENDER's thread when the send returned `Full`; pair it with
///   `queue_full: QueueFullPolicy::Hook`, which is the policy that reports
///   `Full` back to the adapter instead of asserting.
///
/// The generated helper
/// `fn <deserialize>(msg: &mut dyn SerBufAny) -> Option<(A, B, ..)>` reads
/// the arguments (NOT `msg_type`/`port_num`, which the dispatch loop and
/// `dispatch_message` have already consumed) and returns `None` on any
/// decode failure, for a `MsgDispatchStatus::Error` return.
#[macro_export]
macro_rules! async_input_port_adapter {
    (
        $(#[$meta:meta])*
        component: $comp:ty;
        adapter: $adapter:ident;
        port: $port:path;
        input: $fvis:vis $input:ident;
        deserialize: $dvis:vis $deser:ident;
        handler: $handler:ident;
        base: $($base:ident).+;
        msg_type: $msg_type:expr;
        msg_size: $msg_size:expr;
        priority: $priority:expr;
        queue_full: $policy:expr;
        args { $($kind:ident $arg:ident : $aty:ty),* $(,)? }
        $(pre_msg_hook |$pre_comp:ident, $pre_port:ident| $pre_body:block)?
        $(overflow_hook |$ovf_comp:ident, $ovf_port:ident| $ovf_body:block)?
    ) => {
        #[doc = concat!(
            "Async input-port adapter for `", stringify!($input),
            "`: builds the queue envelope and enqueues it (the hand-written \
             equivalent of the C++ generated static thunk + `<port>_preMsgHook`)."
        )]
        struct $adapter {
            comp: ::std::sync::Arc<$comp>,
        }

        impl $port for $adapter {
            fn invoke(
                &self,
                port_num: $crate::config::FwIndexType,
                $($arg: $crate::__fpp_port_arg_ty!($kind $aty)),*
            ) {
                use $crate::fw::{SerBuf as _, Serialize as _};

                $({
                    // `<port>_preMsgHook`: SENDER's thread, before enqueue.
                    let $pre_comp: &$comp = &self.comp;
                    let $pre_port: $crate::config::FwIndexType = port_num;
                    $pre_body
                })?

                let mut msg = $crate::fw::LinearBuffer::<{ $msg_size }>::new();
                // Envelope: [msg_type i32 BE][port_num i16 BE][args...].
                // A serialize failure is a programmer error (msg_size is the
                // worst case), hence fw_assert — C++ parity.
                let status =
                    $crate::msg::write_envelope_header(&mut msg, $msg_type, port_num);
                $crate::fw::fw_assert!(status.is_ok(), status as i32);
                $(
                    let status = $crate::__fpp_port_arg_write!(msg, $kind $arg);
                    $crate::fw::fw_assert!(status.is_ok(), status as i32);
                )*

                let _send_status = self
                    .comp
                    .$($base).+
                    .send_message(&msg, $priority, $policy);
                $(
                    if _send_status == $crate::os::queue::Status::Full {
                        let $ovf_comp: &$comp = &self.comp;
                        let $ovf_port: $crate::config::FwIndexType = port_num;
                        $ovf_body
                    }
                )?
            }
        }

        impl $comp {
            $(#[$meta])*
            $fvis fn $input(
                self: &::std::sync::Arc<Self>,
                port_num: $crate::config::FwIndexType,
            ) -> $crate::PortRef<dyn $port> {
                $crate::PortRef::new(
                    ::std::sync::Arc::new($adapter {
                        comp: ::std::sync::Arc::clone(self),
                    }),
                    port_num,
                )
            }

            #[doc = concat!(
                "Read the `", stringify!($input),
                "` queue-message arguments, in declaration order, for \
                 `dispatch_message`. `msg_type` and `port_num` must already \
                 be consumed. Returns `None` on any decode failure."
            )]
            #[allow(clippy::type_complexity)]
            #[allow(unused_variables)]
            $dvis fn $deser(
                msg: &mut dyn $crate::fw::SerBufAny,
            ) -> Option<($($aty,)*)> {
                use $crate::fw::SerBuf as _;
                $(
                    let mut $arg = <$aty as Default>::default();
                    if !$crate::__fpp_port_arg_read!(msg, $kind $arg).is_ok() {
                        return None;
                    }
                )*
                Some(($($arg,)*))
            }
        }
    };
}
