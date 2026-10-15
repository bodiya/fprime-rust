//! # State machines — the runtime contract behind `fpp_state_machine!`
//!
//! Port of `Fw/Sm` plus the contract the FPP autocoder generates for state
//! machines and state machine instances (upstream: the
//! `<M>StateMachineAc.{hpp,cpp}` base classes, the `SmId` /
//! `sendSignalStart` / `<m>_sendSignalFinish` / `smDispatch` members of a
//! component base, and the `<m>_stateMachineInvoke` path for external
//! machines; see `docs/user-manual/framework/state-machines.md` upstream).
//!
//! Three pieces:
//!
//! - **[`SmCore`] + the graph walks** ([`SmNode`], [`lca`],
//!   [`for_each_exit`], [`for_each_entry`]) — the non-generic runtime an
//!   [`fpp_state_machine!`](crate::fpp_state_machine) expansion runs on.
//!   The C++ autocoder flattens the hierarchy at code-generation time (one
//!   `switch` arm per leaf state and signal, the exit/entry actions spelled
//!   out); a declarative macro cannot, so the expansion keeps the hierarchy
//!   as a parent table and these functions walk it at run time. Every walk
//!   is bounded by the nesting depth, allocation-free, and produces exactly
//!   the action order the C++ code spells out (see [`lca`]).
//! - **[`SmSender`] / [`ExternalSmSender`]** — the component side of
//!   `<m>_sendSignal_<s>` / `<m>_stateMachineInvoke`: build the byte-exact
//!   queue message and enqueue it with the instance's priority and
//!   queue-full policy.
//! - **[`read_signal_header`] / [`read_external_signal`]** — the dispatch
//!   side (`deserializeSmIdAndSignal`, and the external-signal arm of
//!   `doDispatch`).
//!
//! ## Queue message layout (byte-exact with the C++ generated code)
//!
//! ```text
//! internal: [msg_type i32][port_num i16 = 0][sm_id i32][signal i32][signal data, if any]
//! external: [msg_type i32][port_num i16 = 0][sm_id i32][signal i32][u16 size][size bytes]
//! ```
//!
//! `msg_type` is the component's `INTERNAL_STATE_MACHINE_SIGNAL` /
//! `EXTERNAL_STATE_MACHINE_SIGNAL` discriminant: C++ numbers them after
//! every async input port, async command and internal port, external
//! before internal. `sm_id` is the component's `SmId`: every state machine
//! instance of the component, internal and external together, **sorted by
//! instance name** and numbered from 0. The port number is a constant 0
//! ("fake port number to make message dequeue work"). Internal signal data
//! is the value serialized as is; external data is an
//! [`SmSignalBuffer`] serialized as a nested buffer.

use crate::msg::{self, QueueFullPolicy};
use crate::queued::QueuedBase;
use fprime_config::{FwEnumStoreType, FwQueuePriorityType};
use fprime_fw::{
    Endianness, LinearBuffer, SerBuf, SerBufAny, Serialize, SmSignalBuffer, fw_assert,
};
use fprime_os::queue::Status as QueueStatus;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

/// Size of the state-machine signal header that follows the envelope
/// header: `sm_id` + `signal`, both `FwEnumStoreType` (C++
/// `2 * sizeof(FwEnumStoreType)`).
pub const SIGNAL_HEADER_SIZE: usize = 2 * size_of::<FwEnumStoreType>();

/// Maximum on-wire size of an [`SmSignalBuffer`] (C++
/// `Fw::SmSignalBuffer::SERIALIZED_SIZE`: the u16 length prefix plus the
/// capacity).
pub const SM_SIGNAL_BUFFER_SERIALIZED_SIZE: usize =
    size_of::<fprime_config::FwSizeStoreType>() + fprime_config::FW_SM_SIGNAL_BUFFER_MAX_SIZE;

/// Queue message size needed by an external state machine signal (C++
/// `externalSmBufferSize` member of the generated `BuffUnion`, plus the
/// envelope header).
pub const EXTERNAL_SIGNAL_MSG_SIZE: usize =
    msg::ENVELOPE_HEADER_SIZE + SIGNAL_HEADER_SIZE + SM_SIGNAL_BUFFER_SERIALIZED_SIZE;

/// `max` over a slice of sizes, usable in constants (the macro computes a
/// machine's largest signal payload with it).
pub const fn max_size(sizes: &[usize]) -> usize {
    let mut max = 0;
    let mut i = 0;
    while i < sizes.len() {
        if sizes[i] > max {
            max = sizes[i];
        }
        i += 1;
    }
    max
}

// ---------------------------------------------------------------------------
// String values
// ---------------------------------------------------------------------------

/// A string signal value on its way into a queue message: serialized as
/// `[u16 length][bytes]` with the length truncated to the signal's declared
/// size (C++ `value.serializeTo(buffer, size)`; the truncation is
/// byte-based and stops at the first NUL, as in C++).
#[derive(Debug, Clone, Copy)]
pub struct StrArg<'a> {
    value: &'a str,
    max_len: usize,
}

impl<'a> StrArg<'a> {
    /// `value`, to be truncated to `max_len` bytes on the wire.
    pub fn new(value: &'a str, max_len: usize) -> Self {
        Self { value, max_len }
    }

    fn bytes(&self) -> &'a [u8] {
        let bytes = self.value.as_bytes();
        let nul = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
        &bytes[..nul.min(self.max_len)]
    }
}

impl Serialize for StrArg<'_> {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, _e: Endianness) -> fprime_fw::SerializeStatus {
        // C++ parity: string length prefixes are always big-endian.
        buf.serialize_bytes(
            self.bytes(),
            fprime_fw::LengthMode::IncludeLength,
            Endianness::Big,
        )
    }
    fn serialized_size(&self) -> usize {
        size_of::<fprime_config::FwSizeStoreType>() + self.bytes().len()
    }
}

/// The text of a string signal value read back from a queue message, for
/// the `&str` the actions and guards take. Byte-based truncation on the
/// send side can split a UTF-8 sequence; the incomplete tail is dropped.
pub fn str_of<const N: usize>(s: &fprime_fw::FwString<N>) -> &str {
    let bytes = s.as_bytes();
    match std::str::from_utf8(bytes) {
        Ok(text) => text,
        Err(e) => std::str::from_utf8(&bytes[..e.valid_up_to()]).unwrap_or(""),
    }
}

// ---------------------------------------------------------------------------
// SmCore
// ---------------------------------------------------------------------------

/// The state an `fpp_state_machine!` instance carries: the C++ `m_id` and
/// `m_state` members, plus a run-to-completion flag.
///
/// All fields are atomics so every operation takes `&self`: a component
/// holds its instances directly (no mutex), actions running on the
/// component thread may call back into the component, and
/// `<instance>.get_state()` works from any thread, as the C++ getter does.
///
/// Deliberate deviation: C++ does not detect a signal delivered while
/// another transition of the same instance is still running (re-entry from
/// an action, or two threads at once). That is a programmer error — the
/// FPP semantics are run-to-completion, which is why component actions
/// send signals through the queue — and here it is a `fw_assert!`.
#[derive(Debug)]
pub struct SmCore {
    id: AtomicI32,
    /// The `State` discriminant (0 = `__FPRIME_UNINITIALIZED`).
    state: AtomicU32,
    running: AtomicBool,
}

impl SmCore {
    /// An uninitialized machine: id 0, state `__FPRIME_UNINITIALIZED`.
    pub const fn new() -> Self {
        Self {
            id: AtomicI32::new(0),
            state: AtomicU32::new(0),
            running: AtomicBool::new(false),
        }
    }

    /// The state machine id given to `init` (C++ `m_id`; 0 before `init`).
    pub fn id(&self) -> FwEnumStoreType {
        self.id.load(Ordering::Acquire)
    }

    /// Record the id (C++ `initBase` sets `m_id` first).
    #[doc(hidden)]
    pub fn set_id(&self, id: FwEnumStoreType) {
        self.id.store(id, Ordering::Release);
    }

    /// The raw `State` discriminant.
    pub fn raw_state(&self) -> u32 {
        self.state.load(Ordering::Acquire)
    }

    /// Update the state (the C++ `m_state = State::X` at the end of a leaf
    /// state's entry function).
    #[doc(hidden)]
    pub fn set_raw_state(&self, raw: u32) {
        self.state.store(raw, Ordering::Release);
    }

    /// Begin a run-to-completion step (`init` or one signal). Asserts that
    /// no other step of this instance is in progress; the step ends when
    /// the returned guard drops.
    #[doc(hidden)]
    pub fn begin_run(&self) -> RunGuard<'_> {
        let was_running = self.running.swap(true, Ordering::AcqRel);
        // Run-to-completion: a signal sent to an instance while one of its
        // transitions is still running is a programmer error.
        fw_assert!(!was_running, self.id());
        RunGuard { core: self }
    }
}

impl Default for SmCore {
    fn default() -> Self {
        Self::new()
    }
}

/// Ends a run-to-completion step of an [`SmCore`] when dropped.
#[doc(hidden)]
#[derive(Debug)]
pub struct RunGuard<'a> {
    core: &'a SmCore,
}

impl Drop for RunGuard<'_> {
    fn drop(&mut self) {
        self.core.running.store(false, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// Hierarchy walks
// ---------------------------------------------------------------------------

/// A node of a state machine's hierarchy — a state or a choice. Generated
/// by `fpp_state_machine!` as a fieldless enum with a parent table.
pub trait SmNode: Copy + Eq {
    /// The enclosing state, `None` at the top level.
    fn parent(self) -> Option<Self>;
}

/// Nesting depth: 1 for a top-level node.
fn depth<N: SmNode>(n: N) -> usize {
    let mut d = 1;
    let mut cur = n;
    while let Some(p) = cur.parent() {
        d += 1;
        cur = p;
    }
    d
}

/// Whether `a` is a proper ancestor of `n`.
fn is_proper_ancestor<N: SmNode>(a: N, n: N) -> bool {
    let mut cur = n.parent();
    while let Some(p) = cur {
        if p == a {
            return true;
        }
        cur = p.parent();
    }
    false
}

/// The state a transition from `source` to `target` stays inside: the
/// deepest node that is `source` or an ancestor of it **and** a proper
/// ancestor of `target`; `None` for the top level.
///
/// This is the FPP transition rule as the C++ autocoder flattens it:
///
/// - the exit actions run from the current leaf up to, not including, this
///   node ([`for_each_exit`]);
/// - then the transition's own actions;
/// - then the entry actions of the states strictly between this node and
///   the target, top down ([`for_each_entry`]);
/// - then the target is entered (its entry actions and initial transition,
///   or the choice is evaluated).
///
/// So a transition to a substate of `source` exits only below `source`
/// (`StateToChild`); a transition to `source` itself or to one of its
/// ancestors exits and re-enters it (`StateToSelf`, an *external*
/// self-transition); and a transition to a choice inside another state
/// enters that state without running its initial transition
/// (`StateToChoice`).
pub fn lca<N: SmNode>(source: N, target: N) -> Option<N> {
    let mut cur = Some(source);
    while let Some(a) = cur {
        if is_proper_ancestor(a, target) {
            return Some(a);
        }
        cur = a.parent();
    }
    None
}

/// Visit the nodes to exit, innermost first: `from` and its ancestors up
/// to, not including, `stop` (`None` = exit all the way to the top).
/// `stop` must be `None` or an ancestor of (or equal to) `from`.
pub fn for_each_exit<N: SmNode>(from: N, stop: Option<N>, mut f: impl FnMut(N)) {
    let mut cur = Some(from);
    while let Some(n) = cur {
        if Some(n) == stop {
            return;
        }
        f(n);
        cur = n.parent();
    }
    // Reaching the top without meeting `stop` means the transition tables
    // are inconsistent (stop is not an ancestor of the current state).
    fw_assert!(stop.is_none());
}

/// Visit the states to enter on the way to `target`, outermost first: the
/// proper ancestors of `target` strictly below `start` (`None` = from the
/// top level). `target` itself is not visited — the caller enters it.
pub fn for_each_entry<N: SmNode>(start: Option<N>, target: N, mut f: impl FnMut(N)) {
    let start_depth = start.map_or(0, depth);
    let target_depth = depth(target);
    // Bounded: one pass per level, each at most `target_depth` long.
    for level in (start_depth + 1)..target_depth {
        let mut n = target;
        for _ in level..target_depth {
            match n.parent() {
                Some(p) => n = p,
                None => break,
            }
        }
        f(n);
    }
}

// ---------------------------------------------------------------------------
// Component side: enqueue
// ---------------------------------------------------------------------------

/// Overflow hook of internal state machine instances with the `hook`
/// queue-full policy (C++ `<m>_stateMachineOverflowHook(smId, signal,
/// buffer)`). A component implements it once and dispatches on the state
/// machine id; closures implement it too.
pub trait SmOverflowHook {
    /// Called on the sender's thread when the queue is full: the state
    /// machine id, the signal, and the message with its deserialization
    /// cursor at the start of the signal data (at the end of the message
    /// for a signal without data).
    fn sm_overflow_hook(
        &self,
        sm_id: FwEnumStoreType,
        signal: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    );
}

impl<F: Fn(FwEnumStoreType, FwEnumStoreType, &mut dyn SerBufAny)> SmOverflowHook for F {
    fn sm_overflow_hook(
        &self,
        sm_id: FwEnumStoreType,
        signal: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) {
        self(sm_id, signal, msg)
    }
}

/// One internal state machine instance of a component, as seen by the
/// signal send functions (C++ `sendSignalStart` + `<m>_sendSignalFinish`).
///
/// `N` is the component's queue message size. `fpp_state_machine!` wraps
/// this in the machine's typed `Sender`, which has one `send_signal_<s>`
/// method per signal.
pub struct SmSender<'a, const N: usize> {
    queue: &'a QueuedBase,
    msg_type: FwEnumStoreType,
    sm_id: FwEnumStoreType,
    priority: FwQueuePriorityType,
    queue_full: QueueFullPolicy,
    overflow_hook: Option<&'a dyn SmOverflowHook>,
}

impl<'a, const N: usize> SmSender<'a, N> {
    /// A sender for instance `sm_id` of a component whose queue is `queue`
    /// and whose `INTERNAL_STATE_MACHINE_SIGNAL` message type is
    /// `msg_type`, with the instance's FPP `priority` (default 0) and
    /// queue-full policy (default `assert`).
    pub fn new(
        queue: &'a QueuedBase,
        msg_type: FwEnumStoreType,
        sm_id: FwEnumStoreType,
        priority: FwQueuePriorityType,
        queue_full: QueueFullPolicy,
    ) -> Self {
        Self {
            queue,
            msg_type,
            sm_id,
            priority,
            queue_full,
            overflow_hook: None,
        }
    }

    /// Attach the overflow hook required by [`QueueFullPolicy::Hook`].
    pub fn with_overflow_hook(mut self, hook: &'a dyn SmOverflowHook) -> Self {
        self.overflow_hook = Some(hook);
        self
    }

    /// The instance's state machine id.
    pub fn sm_id(&self) -> FwEnumStoreType {
        self.sm_id
    }

    /// Serialize `[msg_type][port 0][sm_id][signal][data]` and enqueue it
    /// (C++ `<m>_sendSignal_<s>`). Serialization failures assert — the
    /// message size is the component's worst case, as in C++.
    pub fn send(&self, signal: FwEnumStoreType, data: Option<&dyn Serialize>) {
        let mut buf = LinearBuffer::<N>::new();
        let status = write_signal_header(&mut buf, self.msg_type, self.sm_id, signal);
        fw_assert!(status.is_ok(), status as i32);
        if let Some(data) = data {
            let status = buf.serialize(data, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
        }
        let send_status = self
            .queue
            .send_message(&buf, self.priority, self.queue_full);
        if send_status == QueueStatus::Full && self.queue_full == QueueFullPolicy::Hook {
            // C++ parity: deserializeSmIdAndSignal moves the cursor past the
            // envelope header and reads the id and signal back out of the
            // message, leaving the cursor at the signal data for the hook.
            let (sm_id, signal) = read_signal_header(&mut buf);
            match self.overflow_hook {
                Some(hook) => hook.sm_overflow_hook(sm_id, signal, &mut buf),
                // A `hook` instance without a hook is a wiring error.
                None => fw_assert!(false, sm_id),
            }
        }
    }
}

/// Serialize the internal-signal message header
/// `[msg_type i32][port_num i16 = 0][sm_id i32][signal i32]` (C++
/// `sendSignalStart`).
pub fn write_signal_header(
    buf: &mut dyn SerBufAny,
    msg_type: FwEnumStoreType,
    sm_id: FwEnumStoreType,
    signal: FwEnumStoreType,
) -> fprime_fw::SerializeStatus {
    fprime_fw::fw_try!(msg::write_envelope_header(buf, msg_type, 0));
    fprime_fw::fw_try!(buf.serialize_i32_be(sm_id));
    buf.serialize_i32_be(signal)
}

/// Read the state machine id and signal of an internal-signal message (C++
/// `deserializeSmIdAndSignal`): moves the cursor to the end of the envelope
/// header, reads both, and leaves the cursor at the signal data. Decode
/// failures assert, as in C++.
pub fn read_signal_header(buf: &mut dyn SerBufAny) -> (FwEnumStoreType, FwEnumStoreType) {
    let status = buf.move_deser_to_offset(msg::ENVELOPE_HEADER_SIZE);
    fw_assert!(status.is_ok(), status as i32);
    let mut sm_id: FwEnumStoreType = 0;
    let status = buf.deserialize_i32_be(&mut sm_id);
    fw_assert!(status.is_ok(), status as i32);
    let mut signal: FwEnumStoreType = 0;
    let status = buf.deserialize_i32_be(&mut signal);
    fw_assert!(status.is_ok(), status as i32);
    (sm_id, signal)
}

// ---------------------------------------------------------------------------
// External state machines
// ---------------------------------------------------------------------------

/// An external state machine (one whose implementation is not written in
/// FPP — typically STARS-generated): the C++ `init(id)` / `update(id,
/// signal, data)` shape the component base calls.
///
/// Deviation: the C++ class keeps a pointer to its parent (the component,
/// through the machine's `<M>_Interface`); here the parent is passed to
/// each call, as the [`fpp_state_machine!`](crate::fpp_state_machine)
/// handlers are, so the component owns its machines without a reference
/// cycle. Signals cross the queue as their `FwEnumStoreType` value.
pub trait ExternalStateMachine {
    /// The machine's signal enum (C++ `<M>_Interface::<M>_Signals`).
    type Signal: Copy + Into<FwEnumStoreType> + TryFrom<FwEnumStoreType>;

    /// The interface the machine calls back into (C++ `<M>_Interface`):
    /// usually a `dyn` trait the component implements.
    type Parent: ?Sized;

    /// Initialize the machine (run its initial transition).
    fn init(&self, parent: &Self::Parent, sm_id: FwEnumStoreType);

    /// Deliver one signal with its data buffer.
    fn update(
        &self,
        parent: &Self::Parent,
        sm_id: FwEnumStoreType,
        signal: Self::Signal,
        data: &SmSignalBuffer,
    );

    /// Deliver a signal read from a queue message (the per-instance arm of
    /// the generated `EXTERNAL_STATE_MACHINE_SIGNAL` dispatch). A value
    /// that is not one of the machine's signals is a dispatch error —
    /// deviation: C++ `static_cast`s it into the enum and hands it on.
    fn dispatch(
        &self,
        parent: &Self::Parent,
        sm_id: FwEnumStoreType,
        signal: FwEnumStoreType,
        data: &SmSignalBuffer,
    ) -> crate::MsgDispatchStatus {
        match Self::Signal::try_from(signal) {
            Ok(signal) => {
                self.update(parent, sm_id, signal, data);
                crate::MsgDispatchStatus::Ok
            }
            Err(_) => crate::MsgDispatchStatus::Error,
        }
    }
}

/// Overflow hook of external state machine instances with the `hook`
/// policy (C++ `<m>_stateMachineOverflowHook(signal, data)`): the signal
/// and the original data buffer. Deviation: the state machine id is passed
/// too, so one implementation can serve every instance of a component.
pub trait ExternalSmOverflowHook {
    /// Called on the sender's thread when the queue is full.
    fn external_sm_overflow_hook(
        &self,
        sm_id: FwEnumStoreType,
        signal: FwEnumStoreType,
        data: &SmSignalBuffer,
    );
}

impl<F: Fn(FwEnumStoreType, FwEnumStoreType, &SmSignalBuffer)> ExternalSmOverflowHook for F {
    fn external_sm_overflow_hook(
        &self,
        sm_id: FwEnumStoreType,
        signal: FwEnumStoreType,
        data: &SmSignalBuffer,
    ) {
        self(sm_id, signal, data)
    }
}

/// One external state machine instance of a component, as seen by its
/// `<m>_stateMachineInvoke(signal, data)` function.
pub struct ExternalSmSender<'a, const N: usize> {
    queue: &'a QueuedBase,
    msg_type: FwEnumStoreType,
    sm_id: FwEnumStoreType,
    priority: FwQueuePriorityType,
    queue_full: QueueFullPolicy,
    overflow_hook: Option<&'a dyn ExternalSmOverflowHook>,
}

impl<'a, const N: usize> ExternalSmSender<'a, N> {
    /// A sender for external instance `sm_id`; `msg_type` is the
    /// component's `EXTERNAL_STATE_MACHINE_SIGNAL` message type.
    pub fn new(
        queue: &'a QueuedBase,
        msg_type: FwEnumStoreType,
        sm_id: FwEnumStoreType,
        priority: FwQueuePriorityType,
        queue_full: QueueFullPolicy,
    ) -> Self {
        Self {
            queue,
            msg_type,
            sm_id,
            priority,
            queue_full,
            overflow_hook: None,
        }
    }

    /// Attach the overflow hook required by [`QueueFullPolicy::Hook`].
    pub fn with_overflow_hook(mut self, hook: &'a dyn ExternalSmOverflowHook) -> Self {
        self.overflow_hook = Some(hook);
        self
    }

    /// Serialize `[msg_type][port 0][sm_id][signal][u16 size][data]` and
    /// enqueue it (C++ `<m>_stateMachineInvoke`).
    pub fn invoke(&self, signal: FwEnumStoreType, data: &SmSignalBuffer) {
        let mut buf = LinearBuffer::<N>::new();
        let status = write_signal_header(&mut buf, self.msg_type, self.sm_id, signal);
        fw_assert!(status.is_ok(), status as i32);
        let status = buf.serialize_buffer(data, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        let send_status = self
            .queue
            .send_message(&buf, self.priority, self.queue_full);
        if send_status == QueueStatus::Full && self.queue_full == QueueFullPolicy::Hook {
            match self.overflow_hook {
                Some(hook) => hook.external_sm_overflow_hook(self.sm_id, signal, data),
                None => fw_assert!(false, self.sm_id),
            }
        }
    }
}

/// Read an external-signal message after the envelope header (the
/// `EXTERNAL_STATE_MACHINE_SIGNAL` arm of the generated `doDispatch`):
/// the state machine id, the signal, and the data buffer into `data`.
/// Decode failures and leftover bytes assert, as in C++.
pub fn read_external_signal(
    buf: &mut dyn SerBufAny,
    data: &mut SmSignalBuffer,
) -> (FwEnumStoreType, FwEnumStoreType) {
    let (sm_id, signal) = read_signal_header(buf);
    let status = buf.deserialize_buffer(data, Endianness::Big);
    fw_assert!(status.is_ok(), status as i32);
    // C++: "Make sure there was no data left over. That means the buffer
    // size was incorrect."
    let left = buf.deserialize_size_left();
    fw_assert!(left == 0, left as i32);
    (sm_id, signal)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A five-node tree:
    /// ```text
    /// A ─┬─ B ─── C
    ///    └─ D
    /// E
    /// ```
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum N {
        A,
        B,
        C,
        D,
        E,
    }

    impl SmNode for N {
        fn parent(self) -> Option<Self> {
            match self {
                N::A | N::E => None,
                N::B | N::D => Some(N::A),
                N::C => Some(N::B),
            }
        }
    }

    fn exits(from: N, stop: Option<N>) -> Vec<N> {
        let mut v = Vec::new();
        for_each_exit(from, stop, |n| v.push(n));
        v
    }

    fn entries(start: Option<N>, target: N) -> Vec<N> {
        let mut v = Vec::new();
        for_each_entry(start, target, |n| v.push(n));
        v
    }

    #[test]
    fn lca_follows_the_fpp_transition_rule() {
        // To a child: stay inside the source.
        assert_eq!(lca(N::A, N::B), Some(N::A));
        assert_eq!(lca(N::A, N::C), Some(N::A));
        // To self or an ancestor: external, leave the source.
        assert_eq!(lca(N::A, N::A), None);
        assert_eq!(lca(N::B, N::B), Some(N::A));
        assert_eq!(lca(N::C, N::A), None);
        // To a sibling / cousin.
        assert_eq!(lca(N::C, N::D), Some(N::A));
        assert_eq!(lca(N::C, N::E), None);
    }

    #[test]
    fn exit_walk_is_innermost_first_and_stops_below_lca() {
        assert_eq!(exits(N::C, Some(N::A)), vec![N::C, N::B]);
        assert_eq!(exits(N::C, None), vec![N::C, N::B, N::A]);
        assert_eq!(exits(N::C, Some(N::C)), Vec::<N>::new());
        assert_eq!(exits(N::E, None), vec![N::E]);
    }

    #[test]
    fn entry_walk_is_outermost_first_and_excludes_target() {
        assert_eq!(entries(None, N::C), vec![N::A, N::B]);
        assert_eq!(entries(Some(N::A), N::C), vec![N::B]);
        assert_eq!(entries(Some(N::B), N::C), Vec::<N>::new());
        assert_eq!(entries(None, N::E), Vec::<N>::new());
    }

    #[test]
    fn max_size_is_usable_in_constants() {
        const M: usize = max_size(&[3, 9, 4]);
        assert_eq!(M, 9);
        assert_eq!(max_size(&[]), 0);
    }

    #[test]
    fn signal_header_layout_is_byte_exact() {
        let mut buf = LinearBuffer::<32>::new();
        let status = write_signal_header(&mut buf, 5, 2, 3);
        assert!(status.is_ok());
        assert_eq!(buf.as_slice(), &[0, 0, 0, 5, 0, 0, 0, 0, 0, 2, 0, 0, 0, 3]);
        let (sm_id, signal) = read_signal_header(&mut buf);
        assert_eq!((sm_id, signal), (2, 3));
        assert_eq!(buf.deserialize_size_left(), 0);
    }

    #[test]
    fn str_arg_truncates_like_string_base_serialize_to() {
        let mut buf = LinearBuffer::<16>::new();
        assert!(
            buf.serialize(&StrArg::new("abcdef", 4), Endianness::Big)
                .is_ok()
        );
        assert_eq!(buf.as_slice(), &[0, 4, b'a', b'b', b'c', b'd']);
        // Content stops at the first NUL, as a C string does.
        let mut buf = LinearBuffer::<16>::new();
        assert!(
            buf.serialize(&StrArg::new("ab\0cd", 80), Endianness::Big)
                .is_ok()
        );
        assert_eq!(buf.as_slice(), &[0, 2, b'a', b'b']);
        assert_eq!(StrArg::new("abcdef", 4).serialized_size(), 6);
    }

    #[test]
    fn str_of_drops_a_split_utf8_tail() {
        let mut s = fprime_fw::FwString::<3>::new();
        s.set("aé!"); // 4 bytes, truncated to 3 at a character boundary
        assert_eq!(s.as_bytes().len(), 3);
        assert_eq!(str_of(&s), "a\u{e9}");
        let mut t = fprime_fw::FwString::<2>::new();
        t.set("aé"); // truncated inside 'é'
        assert_eq!(str_of(&t), "a");
    }

    #[test]
    fn external_message_size_matches_cpp_buff_union() {
        // DATA_OFFSET (4 + 2) + 2 * sizeof(FwEnumStoreType) + (2 + 128).
        assert_eq!(EXTERNAL_SIGNAL_MSG_SIZE, 6 + 8 + 130);
    }

    #[test]
    fn run_guard_releases_on_drop() {
        let core = SmCore::new();
        {
            let _g = core.begin_run();
        }
        let _g = core.begin_run();
    }

    #[test]
    #[should_panic]
    fn reentrant_run_asserts() {
        let core = SmCore::new();
        let _g = core.begin_run();
        let _h = core.begin_run();
    }
}
