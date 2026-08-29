//! Port of `Fw::ActiveComponentBase` (`Fw/Comp/ActiveComponentBase.cpp`;
//! analysis: `docs/cpp-analysis/fw-comp.md`).
//!
//! The component's task runs the C++ lifecycle: `preamble()` → blocking
//! dispatch loop until `MSG_DISPATCH_EXIT` → `finalizer()` → `DONE`.
//! `exit()` posts the 4-byte EXIT message non-blocking at priority 0 with
//! the send status ignored — C++ parity, which means exit can be silently
//! lost on a full queue, and every pending higher-priority message is
//! processed before the component stops.

use crate::msg;
use crate::queued::{ComponentDispatch, MsgDispatchStatus, QueuedBase};
use fprime_config::{FwSizeType, FwTaskPriorityType};
use fprime_fw::fw_assert;
use fprime_os::queue::BlockingType;
use fprime_os::task::{Arguments, Status as TaskStatus, TASK_IDENTIFIER_DEFAULT, Task};
use std::sync::Arc;
use std::sync::atomic::{AtomicI32, Ordering};

/// C++ `ActiveComponentBase::Lifecycle` — exact discriminants.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lifecycle {
    /// Initialized but not started.
    Created = 0,
    /// Preamble done; dispatching messages.
    Dispatching = 1,
    /// EXIT received; running the finalizer.
    Finalizing = 2,
    /// Terminated.
    Done = 3,
}

/// Implemented by every active component: [`ComponentDispatch`] plus access
/// to its embedded [`ActiveBase`], so [`ActiveBase::start`] can run the
/// lifecycle loop against the component.
pub trait ActiveComponent: ComponentDispatch + 'static {
    /// The component's embedded active base.
    fn active_base(&self) -> &ActiveBase;
}

/// Active-component core: queued base + task + lifecycle stage.
#[derive(Default)]
pub struct ActiveBase {
    /// The queued-component core (which itself embeds the [`crate::PassiveBase`]).
    pub queued: QueuedBase,
    task: Task,
    /// C++ `m_stage`, written only by the component's own task (atomic for
    /// safe observation from other threads).
    stage: AtomicI32,
}

impl ActiveBase {
    /// New active base with the given object name. Create the queue with
    /// [`QueuedBase::create_queue`] before [`ActiveBase::start`].
    pub fn new(name: &str) -> Self {
        Self {
            queued: QueuedBase::new(name),
            task: Task::new(),
            stage: AtomicI32::new(Lifecycle::Created as i32),
        }
    }

    /// Current lifecycle stage.
    pub fn lifecycle(&self) -> Lifecycle {
        match self.stage.load(Ordering::Acquire) {
            0 => Lifecycle::Created,
            1 => Lifecycle::Dispatching,
            2 => Lifecycle::Finalizing,
            _ => Lifecycle::Done,
        }
    }

    /// C++ `start(priority, stackSize, cpuAffinity)`: spawns the component
    /// task (named after the object) running
    /// `preamble()` → blocking dispatch until `Exit` → `finalizer()`.
    ///
    /// `component` must be the component embedding this base (asserted);
    /// pass `&self_arc` from the topology. Task priority/stack/affinity are
    /// recorded but best-effort no-ops on std threads (see fprime-os).
    /// Asserts the task-start status (C++ parity).
    pub fn start<C: ActiveComponent>(
        &self,
        component: &Arc<C>,
        priority: FwTaskPriorityType,
        stack_size: FwSizeType,
        cpu_affinity: FwSizeType,
    ) {
        // Wiring sanity: the loop below dispatches into `component`, which
        // must own this very base.
        fw_assert!(std::ptr::eq(component.active_base(), self));
        let comp = Arc::clone(component);
        let name = self.queued.base.get_obj_name();
        let routine = Box::new(move || {
            let base = comp.active_base();
            comp.preamble();
            base.stage
                .store(Lifecycle::Dispatching as i32, Ordering::Release);
            loop {
                // C++ parity: anything != MSG_DISPATCH_EXIT (including
                // ERROR and the cooperative EMPTY) stays in DISPATCHING.
                let status = base.queued.do_dispatch(&*comp, BlockingType::Blocking);
                if status == MsgDispatchStatus::Exit {
                    break;
                }
            }
            base.stage
                .store(Lifecycle::Finalizing as i32, Ordering::Release);
            comp.finalizer();
            base.stage.store(Lifecycle::Done as i32, Ordering::Release);
        });
        let mut arguments = Arguments::new(name.as_str().unwrap_or(""), routine);
        arguments.priority = priority;
        arguments.stack_size = stack_size;
        arguments.cpu_affinity = cpu_affinity;
        arguments.identifier = TASK_IDENTIFIER_DEFAULT;
        let status = self.task.start(arguments);
        // C++ parity: ActiveComponentBase::start asserts Os::Task::start OK.
        fw_assert!(status == TaskStatus::OpOk, status as i32);
    }

    /// C++ `exit()`: sends the 4-byte EXIT message (`[i32 0]`) at priority
    /// 0, non-blocking, send status ignored (exit can be lost on a full
    /// queue — documented C++ gotcha).
    pub fn exit(&self) {
        let status = self
            .queued
            .queue()
            .send(&msg::EXIT_MSG_BYTES, 0, BlockingType::NonBlocking);
        let _ = status; // C++ parity: status deliberately discarded.
    }

    /// C++ `join()`: joins the component task.
    pub fn join(&self) -> TaskStatus {
        self.task.join()
    }

    /// The underlying task (state inspection).
    pub fn task(&self) -> &Task {
        &self.task
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::QueueFullPolicy;
    use fprime_config::FwEnumStoreType;
    use fprime_fw::{LinearBuffer, SerBuf, SerBufAny};
    use fprime_os::queue::Status as QueueStatus;
    use fprime_os::task::TASK_DEFAULT;
    use std::sync::Mutex;

    /// Records lifecycle callbacks and dispatched values in order.
    struct LifeComp {
        active: ActiveBase,
        journal: Mutex<Vec<String>>,
    }

    impl LifeComp {
        fn new() -> Arc<Self> {
            let comp = Arc::new(Self {
                active: ActiveBase::new("life"),
                journal: Mutex::new(Vec::new()),
            });
            comp.active.queued.create_queue(8, 32);
            comp
        }
    }

    impl ComponentDispatch for LifeComp {
        fn dispatch_message(
            &self,
            msg_type: FwEnumStoreType,
            buf: &mut dyn SerBufAny,
        ) -> MsgDispatchStatus {
            let mut port_num = 0i16;
            if !msg::read_port_num(buf, &mut port_num).is_ok() {
                return MsgDispatchStatus::Error;
            }
            let mut value = 0u32;
            if !buf.deserialize_u32_be(&mut value).is_ok() {
                return MsgDispatchStatus::Error;
            }
            self.journal
                .lock()
                .unwrap()
                .push(format!("msg {msg_type} {value}"));
            MsgDispatchStatus::Ok
        }

        fn preamble(&self) {
            self.journal.lock().unwrap().push("preamble".into());
        }

        fn finalizer(&self) {
            self.journal.lock().unwrap().push("finalizer".into());
        }
    }

    impl ActiveComponent for LifeComp {
        fn active_base(&self) -> &ActiveBase {
            &self.active
        }
    }

    fn envelope(msg_type: FwEnumStoreType, value: u32) -> LinearBuffer<32> {
        let mut buf = LinearBuffer::<32>::new();
        let status = msg::write_envelope_header(&mut buf, msg_type, 0);
        assert!(status.is_ok());
        let status = buf.serialize_u32_be(value);
        assert!(status.is_ok());
        buf
    }

    #[test]
    fn exit_message_on_queue_is_four_bytes_priority_zero() {
        let comp = LifeComp::new();
        comp.active.exit();
        let mut dest = [0u8; 32];
        let mut size = 0;
        let mut priority = 0xAA;
        let status = comp.active.queued.queue().receive(
            &mut dest,
            BlockingType::NonBlocking,
            &mut size,
            &mut priority,
        );
        assert_eq!(status, QueueStatus::OpOk);
        assert_eq!(size, 4);
        assert_eq!(&dest[..4], &[0, 0, 0, 0]);
        assert_eq!(priority, 0);
    }

    #[test]
    fn lifecycle_runs_preamble_messages_finalizer_in_order() {
        let comp = LifeComp::new();
        assert_eq!(comp.active.lifecycle(), Lifecycle::Created);
        // Enqueue work BEFORE start: it must run after preamble.
        let s = comp
            .active
            .queued
            .send_message(&envelope(1, 11), 1, QueueFullPolicy::Assert);
        assert_eq!(s, QueueStatus::OpOk);
        let s = comp
            .active
            .queued
            .send_message(&envelope(2, 22), 1, QueueFullPolicy::Assert);
        assert_eq!(s, QueueStatus::OpOk);
        comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);
        // EXIT is priority 0: all pending priority-1 messages drain first.
        comp.active.exit();
        let status = comp.active.join();
        assert_eq!(status, TaskStatus::OpOk);
        assert_eq!(comp.active.lifecycle(), Lifecycle::Done);
        assert_eq!(
            *comp.journal.lock().unwrap(),
            vec!["preamble", "msg 1 11", "msg 2 22", "finalizer"]
        );
    }

    #[test]
    fn dispatch_error_does_not_exit_the_loop() {
        // C++ parity: DISPATCHING stays in stage for anything != EXIT.
        let comp = LifeComp::new();
        // A malformed message (envelope only, missing the u32 arg).
        let mut bad = LinearBuffer::<32>::new();
        let status = msg::write_envelope_header(&mut bad, 1, 0);
        assert!(status.is_ok());
        let s = comp
            .active
            .queued
            .send_message(&bad, 1, QueueFullPolicy::Assert);
        assert_eq!(s, QueueStatus::OpOk);
        let s = comp
            .active
            .queued
            .send_message(&envelope(1, 7), 1, QueueFullPolicy::Assert);
        assert_eq!(s, QueueStatus::OpOk);
        comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);
        comp.active.exit();
        let status = comp.active.join();
        assert_eq!(status, TaskStatus::OpOk);
        // The good message after the bad one still ran.
        assert_eq!(
            *comp.journal.lock().unwrap(),
            vec!["preamble", "msg 1 7", "finalizer"]
        );
    }

    #[test]
    fn task_is_named_after_object() {
        let comp = LifeComp::new();
        comp.active.start(&comp, 100, TASK_DEFAULT, TASK_DEFAULT);
        assert_eq!(comp.active.task().get_name(), "life");
        comp.active.exit();
        let status = comp.active.join();
        assert_eq!(status, TaskStatus::OpOk);
    }
}
