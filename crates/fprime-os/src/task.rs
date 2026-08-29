//! The F Prime task facility.
//!
//! Port of `Os::Task` (Os/Task.{hpp,cpp}) and `Os::Posix::Task` over
//! `std::thread` — see `docs/cpp-analysis/os.md`.
//!
//! The C++ state machine is kept exactly: `start()` sets
//! [`State::Starting`] BEFORE the thread spawns, so the routine can run (and
//! even finish) before `start()` returns; the spawned wrapper asserts the
//! state is not [`State::NotStarted`] and transitions
//! `Starting -> Running` exactly once; `join()` is legal only from
//! `Starting`/`Running` and otherwise returns [`Status::InvalidState`]
//! without touching the thread handle.
//!
//! Priority and CPU affinity are recorded but are no-ops on the std backend:
//! `std::thread` cannot express SCHED_RR priority or affinity. This is the
//! documented equivalent of the C++ EPERM silent-degrade path (PosixTask
//! retries without priority/affinity after one global log notice — the same
//! one-time notice is emitted here). Stack size IS honored via
//! `std::thread::Builder::stack_size`.

use fprime_config::{FwSizeType, FwTaskIdType, FwTaskPriorityType};
use fprime_fw::TimeInterval;
use fprime_fw::fw_assert;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::Duration;

/// Port of `Os::TaskInterface::Status` (Os/Task.hpp) — exact C++
/// discriminants.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Operation succeeded.
    OpOk = 0,
    /// Task handle invalid.
    InvalidHandle = 1,
    /// Started task with invalid parameters.
    InvalidParams = 2,
    /// Started task with invalid priority.
    InvalidPriority = 3,
    /// Started with invalid stack size.
    InvalidStack = 4,
    /// Unexpected error return value.
    UnknownError = 5,
    /// Unable to set the task affinity.
    InvalidAffinity = 6,
    /// Error trying to delay the task.
    DelayError = 7,
    /// Error trying to join the task.
    JoinError = 8,
    /// Unable to allocate more tasks.
    ErrorResources = 9,
    /// Permissions error setting-up tasks.
    ErrorPermission = 10,
    /// Task feature is not supported.
    NotSupported = 11,
    /// Task is in an invalid state for the operation.
    InvalidState = 12,
}

/// Port of `Os::TaskInterface::State`.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    NotStarted = 0,
    Starting = 1,
    Running = 2,
    SuspendedIntentionally = 3,
    SuspendedUnintentionally = 4,
    Exited = 5,
    Unknown = 6,
}

/// Port of `Os::TaskInterface::SuspensionType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SuspensionType {
    Intentional = 0,
    Unintentional = 1,
}

/// C++ `Os::Task::TASK_DEFAULT` (`FwSizeType` max — the FPP `TASK_DEFAULT
/// = -1` constant cast to unsigned).
pub const TASK_DEFAULT: FwSizeType = FwSizeType::MAX;
/// C++ `Os::Task::TASK_PRIORITY_DEFAULT` (`FwTaskPriorityType` max).
pub const TASK_PRIORITY_DEFAULT: FwTaskPriorityType = FwTaskPriorityType::MAX;
/// Default task identifier (C++ `TASK_DEFAULT` cast to `FwTaskIdType`,
/// i.e. -1).
pub const TASK_IDENTIFIER_DEFAULT: FwTaskIdType = -1;

/// Port of `Os::TaskInterface::Arguments`. The C++ raw `routine + argument`
/// pair becomes a boxed `FnOnce` closure (the closure captures its
/// argument).
pub struct Arguments {
    /// Task name (thread name).
    pub name: String,
    /// The task routine, run once on the new thread.
    pub routine: Box<dyn FnOnce() + Send + 'static>,
    /// Requested priority; recorded but a no-op on the std backend.
    pub priority: FwTaskPriorityType,
    /// Requested stack size in bytes; honored via
    /// `std::thread::Builder::stack_size` unless [`TASK_DEFAULT`].
    pub stack_size: FwSizeType,
    /// Requested CPU affinity; recorded but a no-op on the std backend.
    pub cpu_affinity: FwSizeType,
    /// Task identifier (unused by the framework itself).
    pub identifier: FwTaskIdType,
}

impl Arguments {
    /// Construct arguments with the C++ defaults for priority, stack,
    /// affinity, and identifier.
    pub fn new(name: &str, routine: Box<dyn FnOnce() + Send + 'static>) -> Self {
        Self {
            name: name.to_string(),
            routine,
            priority: TASK_PRIORITY_DEFAULT,
            stack_size: TASK_DEFAULT,
            cpu_affinity: TASK_DEFAULT,
            identifier: TASK_IDENTIFIER_DEFAULT,
        }
    }
}

/// State shared between the `Task` front object and its spawned thread
/// (guarded by the per-instance lock — C++ `Task::m_lock`).
struct Shared {
    state: State,
    name: String,
    priority: FwTaskPriorityType,
}

/// Global started-task count (C++ `Os::Task::s_numTasks`).
static NUM_TASKS: AtomicU64 = AtomicU64::new(0);
/// One-time degraded-permissions notice flag (C++ PosixTask
/// `s_permissions_reported`).
static DEGRADE_NOTICE_REPORTED: AtomicBool = AtomicBool::new(false);

/// The F Prime task (see module docs). Methods take `&self`; a `Task` is
/// shareable as `Arc<Task>`.
pub struct Task {
    shared: Arc<Mutex<Shared>>,
    handle: Mutex<Option<JoinHandle<()>>>,
}

fn lock_shared(shared: &Mutex<Shared>) -> MutexGuard<'_, Shared> {
    shared.lock().unwrap_or_else(PoisonError::into_inner)
}

impl Task {
    /// Construct a task in the [`State::NotStarted`] state.
    pub fn new() -> Self {
        Self {
            shared: Arc::new(Mutex::new(Shared {
                state: State::NotStarted,
                name: String::new(),
                priority: TASK_PRIORITY_DEFAULT,
            })),
            handle: Mutex::new(None),
        }
    }

    /// Start the task (port of `Os::Task::start`).
    ///
    /// Sets the state to [`State::Starting`] before spawning; the wrapper
    /// running on the new thread transitions to [`State::Running`] and then
    /// invokes the routine (C++ `TaskRoutineWrapper::run`). On spawn failure
    /// the state remains `Starting` (C++ parity) and
    /// [`Status::ErrorResources`] is returned.
    pub fn start(&self, arguments: Arguments) -> Status {
        {
            let mut shared = lock_shared(&self.shared);
            shared.name = arguments.name.clone();
            shared.state = State::Starting;
        }
        // C++ parity: PosixTask silently degrades priority/affinity on
        // EPERM with one global notice. std::thread cannot express them at
        // all, so the degrade path is unconditional here.
        if (arguments.priority != TASK_PRIORITY_DEFAULT || arguments.cpu_affinity != TASK_DEFAULT)
            && !DEGRADE_NOTICE_REPORTED.swap(true, Ordering::Relaxed)
        {
            fprime_fw::fw_log!(
                "[WARNING] Task priority and CPU affinity are not supported by the std thread \
                 backend and will be ignored.\n"
            );
        }

        let shared = Arc::clone(&self.shared);
        let routine = arguments.routine;
        let wrapper = move || {
            // C++ TaskRoutineWrapper::run: assert not NOT_STARTED; run-once
            // Starting -> Running transition, then the user routine.
            let state = { lock_shared(&shared).state };
            fw_assert!(state != State::NotStarted);
            if state == State::Starting {
                lock_shared(&shared).state = State::Running;
                // onStart(): no-op on the std backend (Posix parity).
            }
            routine();
        };

        let mut builder = std::thread::Builder::new().name(arguments.name);
        if arguments.stack_size != TASK_DEFAULT {
            builder = builder.stack_size(arguments.stack_size as usize);
        }
        match builder.spawn(wrapper) {
            Ok(handle) => {
                *self.handle.lock().unwrap_or_else(PoisonError::into_inner) = Some(handle);
                lock_shared(&self.shared).priority = arguments.priority;
                NUM_TASKS.fetch_add(1, Ordering::Relaxed);
                Status::OpOk
            }
            // Thread spawn fails on resource exhaustion (EAGAIN in C++ maps
            // to ERROR_RESOURCES).
            Err(_) => Status::ErrorResources,
        }
    }

    /// Wait for the task to exit (port of `Os::Task::join`).
    ///
    /// Legal only from [`State::Starting`] or [`State::Running`]; any other
    /// state returns [`Status::InvalidState`] without touching the thread.
    /// On success the state becomes [`State::Exited`]; on failure
    /// [`State::Unknown`] (C++ parity).
    pub fn join(&self) -> Status {
        let state = self.get_state();
        if state != State::Running && state != State::Starting {
            return Status::InvalidState;
        }
        let handle = self
            .handle
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take();
        let status = match handle {
            // No underlying thread: PosixTask::join returns INVALID_HANDLE.
            None => Status::InvalidHandle,
            Some(handle) => match handle.join() {
                Ok(()) => Status::OpOk,
                // The routine panicked: pthread_join failure equivalent.
                Err(_) => Status::JoinError,
            },
        };
        let mut shared = lock_shared(&self.shared);
        shared.state = if status == Status::OpOk {
            State::Exited
        } else {
            State::Unknown
        };
        status
    }

    /// Current lifecycle state (read under the per-instance lock).
    pub fn get_state(&self) -> State {
        lock_shared(&self.shared).state
    }

    /// Task name as passed to [`Task::start`].
    pub fn get_name(&self) -> String {
        lock_shared(&self.shared).name.clone()
    }

    /// Recorded priority (the requested value; not applied on std).
    pub fn get_priority(&self) -> FwTaskPriorityType {
        lock_shared(&self.shared).priority
    }

    /// Suspend the task. C++ parity: `PosixTask::suspend` is
    /// `FW_ASSERT(false)` — unsupported, a crash by contract.
    pub fn suspend(&self, suspension_type: SuspensionType) {
        fw_assert!(false);
        // Reached only if a registered assert hook returns: keep the C++
        // front-class bookkeeping.
        lock_shared(&self.shared).state = match suspension_type {
            SuspensionType::Intentional => State::SuspendedIntentionally,
            SuspensionType::Unintentional => State::SuspendedUnintentionally,
        };
    }

    /// Resume the task. C++ parity: `PosixTask::resume` is
    /// `FW_ASSERT(false)` — unsupported, a crash by contract.
    pub fn resume(&self) {
        fw_assert!(false);
    }

    /// Whether the task backend is cooperative (always false here — the C++
    /// default; cooperative backends run one unit of work per invoke).
    pub fn is_cooperative(&self) -> bool {
        false
    }

    /// Number of tasks started process-wide (C++ `Os::Task::getNumTasks`).
    pub fn get_num_tasks() -> FwSizeType {
        NUM_TASKS.load(Ordering::Relaxed)
    }

    /// Sleep the calling thread (port of the static `Os::Task::delay`).
    /// C++ parity: `TimeInterval` carries (seconds, MICROseconds); nanosleep
    /// EINTR-resume is handled by `std::thread::sleep` internally.
    pub fn delay(interval: TimeInterval) -> Status {
        std::thread::sleep(Duration::new(
            u64::from(interval.get_seconds()),
            interval.get_useconds().saturating_mul(1000),
        ));
        Status::OpOk
    }
}

impl Default for Task {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::time::Instant;

    #[test]
    fn lifecycle_not_started_to_exited() {
        let task = Task::new();
        assert_eq!(task.get_state(), State::NotStarted);
        // join before start is INVALID_STATE.
        assert_eq!(task.join(), Status::InvalidState);

        let (release_tx, release_rx) = mpsc::channel::<()>();
        let (running_tx, running_rx) = mpsc::channel::<()>();
        let status = task.start(Arguments::new(
            "test_task",
            Box::new(move || {
                running_tx.send(()).unwrap();
                release_rx.recv().unwrap();
            }),
        ));
        assert_eq!(status, Status::OpOk);
        // Wait until the routine is definitely running: the wrapper has
        // performed the Starting -> Running transition by then.
        running_rx.recv().unwrap();
        assert_eq!(task.get_state(), State::Running);
        assert_eq!(task.get_name(), "test_task");

        release_tx.send(()).unwrap();
        assert_eq!(task.join(), Status::OpOk);
        assert_eq!(task.get_state(), State::Exited);
        // join after exit is INVALID_STATE (state no longer Starting or
        // Running).
        assert_eq!(task.join(), Status::InvalidState);
    }

    // Gotcha: STARTING is set before the thread spawns, so the routine can
    // finish before start() returns; join() from Starting is still legal.
    #[test]
    fn join_legal_from_starting_even_if_routine_already_finished() {
        let task = Task::new();
        let status = task.start(Arguments::new("fast_task", Box::new(|| {})));
        assert_eq!(status, Status::OpOk);
        let state = task.get_state();
        assert!(state == State::Starting || state == State::Running);
        assert_eq!(task.join(), Status::OpOk);
        assert_eq!(task.get_state(), State::Exited);
    }

    #[test]
    fn join_on_panicked_routine_is_join_error_and_unknown_state() {
        let task = Task::new();
        let status = task.start(Arguments::new(
            "panicking_task",
            Box::new(|| panic!("intentional test panic")),
        ));
        assert_eq!(status, Status::OpOk);
        assert_eq!(task.join(), Status::JoinError);
        assert_eq!(task.get_state(), State::Unknown);
    }

    #[test]
    fn priority_and_stack_are_recorded() {
        let task = Task::new();
        let mut arguments = Arguments::new("configured_task", Box::new(|| {}));
        arguments.priority = 42;
        arguments.stack_size = 512 * 1024;
        assert_eq!(task.start(arguments), Status::OpOk);
        assert_eq!(task.get_priority(), 42);
        assert_eq!(task.join(), Status::OpOk);
    }

    #[test]
    fn delay_sleeps_at_least_the_interval() {
        let start = Instant::now();
        assert_eq!(Task::delay(TimeInterval::new(0, 20_000)), Status::OpOk);
        assert!(start.elapsed() >= Duration::from_millis(20));
    }

    #[test]
    fn num_tasks_counts_starts() {
        let before = Task::get_num_tasks();
        let task = Task::new();
        assert_eq!(
            task.start(Arguments::new("counted", Box::new(|| {}))),
            Status::OpOk
        );
        assert!(Task::get_num_tasks() > before);
        assert_eq!(task.join(), Status::OpOk);
    }

    #[test]
    #[should_panic]
    fn suspend_asserts_unsupported() {
        let task = Task::new();
        task.suspend(SuspensionType::Intentional);
    }
}
