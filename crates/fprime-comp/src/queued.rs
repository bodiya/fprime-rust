//! Port of `Fw::QueuedComponentBase` (`Fw/Comp/QueuedComponentBase.cpp`)
//! plus the generated `doDispatch()` receive/exit-check framing (analysis:
//! `docs/cpp-analysis/fw-comp.md`).
//!
//! The C++ pure-virtual `doDispatch` is split in two: [`QueuedBase`]
//! provides the receive + envelope framing (msg-type read, EXIT detection
//! **before** any `port_num` read — the EXIT message has none), and the
//! component implements [`ComponentDispatch::dispatch_message`] — the
//! per-component `switch (msgType)` body the autocoder would generate.

use crate::msg::{self, QueueFullPolicy};
use crate::obj::PassiveBase;
use fprime_config::{FwEnumStoreType, FwQueuePriorityType, FwSizeType};
use fprime_fw::{ExtBuf, SerBufAny, fw_assert};
use fprime_os::queue::{BlockingType, Queue, Status as QueueStatus};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, PoisonError};

/// C++ `Fw::QueuedComponentBase::MsgDispatchStatus` — exact discriminants.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MsgDispatchStatus {
    /// Dispatched one message okay.
    Ok = 0,
    /// No message in the queue (non-blocking receive).
    Empty = 1,
    /// Message receive or deserialization error.
    Error = 2,
    /// The EXIT message was received (only terminates active components).
    Exit = 3,
}

/// The per-component dispatch contract — what the FPP autocoder would
/// generate as `doDispatch()`'s `switch (msgType)` body plus the active
/// lifecycle hooks.
///
/// `dispatch_message` receives the message with the deserialize cursor
/// positioned **after** `msg_type` (the base already consumed it and
/// checked for EXIT). The component must:
///
/// 1. read `port_num` via [`msg::read_port_num`],
/// 2. match on `msg_type`, deserialize the args in declaration order,
/// 3. call the matching handler and return [`MsgDispatchStatus::Ok`], or
///    [`MsgDispatchStatus::Error`] on an unknown type / deserialize failure.
pub trait ComponentDispatch: Send + Sync {
    /// Dispatch one already-received message (see trait docs).
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        msg: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus;

    /// Active components: runs once on the component's own thread before the
    /// dispatch loop (C++ `preamble()`, default empty).
    fn preamble(&self) {}

    /// Active components: runs once on the component's own thread after the
    /// dispatch loop exits (C++ `finalizer()`, default empty).
    fn finalizer(&self) {}
}

/// Queued-component core: object state + message queue + drop counter.
///
/// All methods take `&self`; the receive scratch buffer is behind a `Mutex`
/// (only the dispatching thread contends for it in practice).
// (No Debug derive: os::Queue is not Debug.)
#[derive(Default)]
pub struct QueuedBase {
    /// The passive-component state (name, id base, instance).
    pub base: PassiveBase,
    queue: Queue,
    /// C++ `m_msgsDropped` — incremented by the `drop` queue-full policy.
    msgs_dropped: AtomicU64,
    /// Receive scratch, sized to the queue message size at `create_queue`
    /// (steady-state no-alloc: allocated once at init).
    msg_buffer: Mutex<Box<[u8]>>,
}

impl QueuedBase {
    /// New queued base with the given object name. The queue is created
    /// later via [`QueuedBase::create_queue`].
    pub fn new(name: &str) -> Self {
        Self {
            base: PassiveBase::new(name),
            queue: Queue::new(),
            msgs_dropped: AtomicU64::new(0),
            msg_buffer: Mutex::new(Box::new([])),
        }
    }

    /// C++ `createQueue(depth, msgSize)` — queue name is the object name
    /// (FW_OBJECT_NAMES==1 behavior). Asserts on failure (the generated
    /// init asserts the create status).
    pub fn create_queue(&self, depth: FwSizeType, msg_size: FwSizeType) {
        let name = self.base.get_obj_name();
        let status = self
            .queue
            .create(name.as_str().unwrap_or(""), depth, msg_size);
        fw_assert!(status == QueueStatus::OpOk, status as i32);
        let mut scratch = self
            .msg_buffer
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        *scratch = vec![0u8; msg_size as usize].into_boxed_slice();
    }

    /// The underlying queue (for direct sends, tests, and serial-level
    /// taps).
    pub fn queue(&self) -> &Queue {
        &self.queue
    }

    /// C++ `getNumMsgsDropped`.
    pub fn get_num_msgs_dropped(&self) -> FwSizeType {
        self.msgs_dropped.load(Ordering::Relaxed)
    }

    /// C++ `incNumMsgDropped` (generated code calls this on the `drop`
    /// policy; [`QueuedBase::send_message`] already does).
    pub fn inc_num_msgs_dropped(&self) {
        self.msgs_dropped.fetch_add(1, Ordering::Relaxed);
    }

    /// Send one serialized envelope (`message.bytes()[0..ser_loc]`) with the
    /// given priority under a queue-full policy — the hand-written
    /// equivalent of the generated async-input send path.
    ///
    /// Behavior per policy (C++ generated code parity):
    /// - `Assert`: non-blocking send, `fw_assert` any non-OK status.
    /// - `Drop`: non-blocking; on `Full` increments `msgs_dropped` and
    ///   returns `Full`; asserts on any other non-OK status.
    /// - `Block`: blocking send, `fw_assert` any non-OK status.
    /// - `Hook`: non-blocking; on `Full` returns `Full` so the adapter can
    ///   call its overflow hook with the original args; asserts otherwise.
    pub fn send_message(
        &self,
        message: &dyn SerBufAny,
        priority: FwQueuePriorityType,
        policy: QueueFullPolicy,
    ) -> QueueStatus {
        let blocking = match policy {
            QueueFullPolicy::Block => BlockingType::Blocking,
            _ => BlockingType::NonBlocking,
        };
        let status = self.queue.send_serial(message, priority, blocking);
        match policy {
            QueueFullPolicy::Assert | QueueFullPolicy::Block => {
                fw_assert!(status == QueueStatus::OpOk, status as i32);
            }
            QueueFullPolicy::Drop => {
                if status == QueueStatus::Full {
                    self.inc_num_msgs_dropped();
                } else {
                    fw_assert!(status == QueueStatus::OpOk, status as i32);
                }
            }
            QueueFullPolicy::Hook => {
                if status != QueueStatus::Full {
                    fw_assert!(status == QueueStatus::OpOk, status as i32);
                }
            }
        }
        status
    }

    /// Receive and dispatch ONE message — the framing half of the generated
    /// `doDispatch()`. Active components pass `Blocking`; queued components
    /// (draining from a sync handler) pass `NonBlocking`.
    ///
    /// EXIT detection happens BEFORE reading `port_num`: the EXIT message is
    /// only 4 bytes (C++ parity — a dispatcher that unconditionally reads
    /// `port_num` after `msg_type` would misparse it).
    pub fn do_dispatch(
        &self,
        component: &dyn ComponentDispatch,
        blocking: BlockingType,
    ) -> MsgDispatchStatus {
        let mut scratch = self
            .msg_buffer
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        let mut actual_size: FwSizeType = 0;
        let mut priority: FwQueuePriorityType = 0;
        let status = self
            .queue
            .receive(&mut scratch, blocking, &mut actual_size, &mut priority);
        if status == QueueStatus::Empty {
            return MsgDispatchStatus::Empty;
        }
        // C++ parity: the generated doDispatch asserts any other receive
        // failure (including receiving from a never-created queue).
        fw_assert!(status == QueueStatus::OpOk, status as i32);

        let len = actual_size as usize;
        let mut view = ExtBuf::with_len(&mut scratch[..], len);
        let mut msg_type: FwEnumStoreType = 0;
        let deser = msg::read_msg_type(&mut view, &mut msg_type);
        if !deser.is_ok() {
            return MsgDispatchStatus::Error;
        }
        if msg_type == msg::EXIT_MSG_TYPE {
            return MsgDispatchStatus::Exit;
        }
        component.dispatch_message(msg_type, &mut view)
    }

    /// C++ `dispatchAvailableMessages()`: snapshots the available count
    /// once and dispatches at most that many messages (bounded work —
    /// messages enqueued during the loop wait for the next call). Breaks
    /// and returns the first non-OK status.
    pub fn dispatch_available_messages(
        &self,
        component: &dyn ComponentDispatch,
    ) -> MsgDispatchStatus {
        let available = self.queue.get_messages_available();
        for _ in 0..available {
            let status = self.do_dispatch(component, BlockingType::NonBlocking);
            if status != MsgDispatchStatus::Ok {
                return status;
            }
        }
        MsgDispatchStatus::Ok
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::{LinearBuffer, SerBuf};
    use std::panic::{AssertUnwindSafe, catch_unwind};
    use std::sync::Mutex as StdMutex;

    /// Records (msg_type, port_num, payload u32) triples; message type 9
    /// re-enqueues a message (for the snapshot-bound test).
    struct TestComp {
        dispatched: StdMutex<Vec<(FwEnumStoreType, i16, u32)>>,
        requeue_into: StdMutex<Option<std::sync::Arc<QueuedBase>>>,
    }

    impl TestComp {
        fn new() -> Self {
            Self {
                dispatched: StdMutex::new(Vec::new()),
                requeue_into: StdMutex::new(None),
            }
        }
    }

    impl ComponentDispatch for TestComp {
        fn dispatch_message(
            &self,
            msg_type: FwEnumStoreType,
            buf: &mut dyn SerBufAny,
        ) -> MsgDispatchStatus {
            let mut port_num = 0i16;
            if !msg::read_port_num(buf, &mut port_num).is_ok() {
                return MsgDispatchStatus::Error;
            }
            match msg_type {
                1 | 9 => {
                    let mut value = 0u32;
                    if !buf.deserialize_u32_be(&mut value).is_ok() {
                        return MsgDispatchStatus::Error;
                    }
                    self.dispatched
                        .lock()
                        .unwrap()
                        .push((msg_type, port_num, value));
                    if msg_type == 9 {
                        // Enqueue another message mid-dispatch.
                        if let Some(base) = self.requeue_into.lock().unwrap().as_ref() {
                            let _ = base.send_message(
                                &envelope(1, 0, value + 100),
                                1,
                                QueueFullPolicy::Assert,
                            );
                        }
                    }
                    MsgDispatchStatus::Ok
                }
                _ => MsgDispatchStatus::Error,
            }
        }
    }

    fn envelope(msg_type: FwEnumStoreType, port_num: i16, value: u32) -> LinearBuffer<32> {
        let mut buf = LinearBuffer::<32>::new();
        let status = msg::write_envelope_header(&mut buf, msg_type, port_num);
        assert!(status.is_ok());
        let status = buf.serialize_u32_be(value);
        assert!(status.is_ok());
        buf
    }

    fn make_base(depth: FwSizeType) -> QueuedBase {
        let base = QueuedBase::new("testq");
        base.create_queue(depth, 32);
        base
    }

    #[test]
    fn queue_name_is_object_name() {
        let base = make_base(4);
        assert_eq!(base.queue().get_name(), "testq");
    }

    #[test]
    fn do_dispatch_routes_message_to_component() {
        let base = make_base(4);
        let comp = TestComp::new();
        let status = base.send_message(&envelope(1, 3, 42), 1, QueueFullPolicy::Assert);
        assert_eq!(status, QueueStatus::OpOk);
        let status = base.do_dispatch(&comp, BlockingType::NonBlocking);
        assert_eq!(status, MsgDispatchStatus::Ok);
        assert_eq!(*comp.dispatched.lock().unwrap(), vec![(1, 3, 42)]);
    }

    #[test]
    fn do_dispatch_empty_queue_returns_empty() {
        let base = make_base(4);
        let comp = TestComp::new();
        let status = base.do_dispatch(&comp, BlockingType::NonBlocking);
        assert_eq!(status, MsgDispatchStatus::Empty);
    }

    #[test]
    fn exit_message_detected_before_port_num_read() {
        // The EXIT message is 4 bytes with NO port_num; if do_dispatch read
        // port_num first it would fail or misparse.
        let base = make_base(4);
        let comp = TestComp::new();
        let mut exit = LinearBuffer::<8>::new();
        let status = msg::write_exit(&mut exit);
        assert!(status.is_ok());
        assert_eq!(exit.as_slice(), &msg::EXIT_MSG_BYTES);
        let status = base.send_message(&exit, 0, QueueFullPolicy::Assert);
        assert_eq!(status, QueueStatus::OpOk);
        let status = base.do_dispatch(&comp, BlockingType::NonBlocking);
        assert_eq!(status, MsgDispatchStatus::Exit);
        assert!(comp.dispatched.lock().unwrap().is_empty());
    }

    #[test]
    fn unknown_msg_type_is_error() {
        let base = make_base(4);
        let comp = TestComp::new();
        let status = base.send_message(&envelope(77, 0, 1), 1, QueueFullPolicy::Assert);
        assert_eq!(status, QueueStatus::OpOk);
        let status = base.do_dispatch(&comp, BlockingType::NonBlocking);
        assert_eq!(status, MsgDispatchStatus::Error);
    }

    #[test]
    fn dispatch_available_messages_bounded_by_snapshot() {
        // A message dispatched during the loop enqueues another; the
        // snapshot bound must leave the new one queued (C++ parity:
        // dispatchAvailableMessages snapshots getMessagesAvailable once).
        let base = std::sync::Arc::new(QueuedBase::new("snap"));
        base.create_queue(8, 32);
        let comp = TestComp::new();
        *comp.requeue_into.lock().unwrap() = Some(base.clone());
        let status = base.send_message(&envelope(9, 0, 5), 1, QueueFullPolicy::Assert);
        assert_eq!(status, QueueStatus::OpOk);
        let status = base.dispatch_available_messages(&comp);
        assert_eq!(status, MsgDispatchStatus::Ok);
        // Only the original message ran; the re-enqueued one is still there.
        assert_eq!(*comp.dispatched.lock().unwrap(), vec![(9, 0, 5)]);
        assert_eq!(base.queue().get_messages_available(), 1);
        // Next call drains it.
        let status = base.dispatch_available_messages(&comp);
        assert_eq!(status, MsgDispatchStatus::Ok);
        assert_eq!(
            *comp.dispatched.lock().unwrap(),
            vec![(9, 0, 5), (1, 0, 105)]
        );
    }

    #[test]
    fn dispatch_available_messages_stops_at_first_non_ok() {
        let base = make_base(8);
        let comp = TestComp::new();
        let s = base.send_message(&envelope(77, 0, 1), 1, QueueFullPolicy::Assert);
        assert_eq!(s, QueueStatus::OpOk);
        let s = base.send_message(&envelope(1, 0, 2), 1, QueueFullPolicy::Assert);
        assert_eq!(s, QueueStatus::OpOk);
        // Priority equal -> FIFO: bad message first, loop breaks.
        let status = base.dispatch_available_messages(&comp);
        assert_eq!(status, MsgDispatchStatus::Error);
        assert!(comp.dispatched.lock().unwrap().is_empty());
        assert_eq!(base.queue().get_messages_available(), 1);
    }

    #[test]
    fn drop_policy_counts_dropped_messages() {
        let base = make_base(2);
        let m = envelope(1, 0, 1);
        assert_eq!(
            base.send_message(&m, 1, QueueFullPolicy::Drop),
            QueueStatus::OpOk
        );
        assert_eq!(
            base.send_message(&m, 1, QueueFullPolicy::Drop),
            QueueStatus::OpOk
        );
        assert_eq!(base.get_num_msgs_dropped(), 0);
        assert_eq!(
            base.send_message(&m, 1, QueueFullPolicy::Drop),
            QueueStatus::Full
        );
        assert_eq!(
            base.send_message(&m, 1, QueueFullPolicy::Drop),
            QueueStatus::Full
        );
        assert_eq!(base.get_num_msgs_dropped(), 2);
    }

    #[test]
    fn assert_policy_panics_on_full_queue() {
        let base = make_base(1);
        let m = envelope(1, 0, 1);
        assert_eq!(
            base.send_message(&m, 1, QueueFullPolicy::Assert),
            QueueStatus::OpOk
        );
        let result = catch_unwind(AssertUnwindSafe(|| {
            let _ = base.send_message(&m, 1, QueueFullPolicy::Assert);
        }));
        assert!(result.is_err());
    }

    #[test]
    fn hook_policy_returns_full_without_asserting() {
        let base = make_base(1);
        let m = envelope(1, 0, 1);
        assert_eq!(
            base.send_message(&m, 1, QueueFullPolicy::Hook),
            QueueStatus::OpOk
        );
        // The adapter would now invoke its overflow hook on Full.
        assert_eq!(
            base.send_message(&m, 1, QueueFullPolicy::Hook),
            QueueStatus::Full
        );
        assert_eq!(base.get_num_msgs_dropped(), 0);
    }

    #[test]
    fn block_policy_waits_for_space() {
        let base = std::sync::Arc::new(QueuedBase::new("blk"));
        base.create_queue(1, 32);
        let m = envelope(1, 0, 1);
        assert_eq!(
            base.send_message(&m, 1, QueueFullPolicy::Block),
            QueueStatus::OpOk
        );
        // Receiver frees a slot after a delay; the blocking send must wait.
        let receiver = {
            let base = base.clone();
            std::thread::spawn(move || {
                std::thread::sleep(std::time::Duration::from_millis(50));
                let mut buf = [0u8; 32];
                let mut size = 0;
                let mut prio = 0;
                let status =
                    base.queue()
                        .receive(&mut buf, BlockingType::Blocking, &mut size, &mut prio);
                assert_eq!(status, QueueStatus::OpOk);
            })
        };
        let status = base.send_message(&m, 1, QueueFullPolicy::Block);
        assert_eq!(status, QueueStatus::OpOk);
        receiver.join().unwrap();
        assert_eq!(base.queue().get_messages_available(), 1);
    }

    #[test]
    fn oversized_message_asserts_size_mismatch() {
        // Os::Queue rejects sends larger than message_size; the Assert
        // policy turns that into fw_assert.
        let base = make_base(4);
        let mut big = LinearBuffer::<64>::new();
        let status = big.serialize_skip(40);
        assert!(status.is_ok());
        let result = catch_unwind(AssertUnwindSafe(|| {
            let _ = base.send_message(&big, 1, QueueFullPolicy::Assert);
        }));
        assert!(result.is_err());
    }
}
