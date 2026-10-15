//! The `no_std` OSAL seam end to end (ROADMAP item 13): with the `std`
//! feature off, `Os::Task`, `Os::RawTime::now`, blocking `Os::Queue` waits
//! and `Os::Mutex` / `Os::ConditionVariable` waits route through the
//! registered `platform::Platform`, and report `NOT_SUPPORTED` (or spin)
//! before one is registered. Also run with `--features lockless-queue`,
//! where the blocking receive backs off through `Platform::delay` instead
//! of `Platform::queue_wait`.
//!
//! Run with `cargo test -p fprime-os --no-default-features --features spin`.
//! In the default `std` build this file compiles to nothing. One test
//! function, because the platform registry is process-global and
//! write-once.

#![cfg(not(feature = "std"))]

use fprime_config::lockless_queue::LOCKLESS_QUEUE_BLOCKING_BACKOFF_US;
use fprime_fw::TimeInterval;
use fprime_os::condition::{ConditionVariable, Status as ConditionStatus};
use fprime_os::mutex::OsMutex;
use fprime_os::platform::{self, Platform, TaskHandle};
use fprime_os::queue::{BlockingType, Queue, Status as QueueStatus};
use fprime_os::rawtime::{self, RawTime};
use fprime_os::task::{self, Arguments, State, Task};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{LazyLock, Mutex};

/// What the fake platform saw.
static STARTED: Mutex<Vec<(String, u8, u64)>> = Mutex::new(Vec::new());
static DELAYS: Mutex<Vec<(u32, u32)>> = Mutex::new(Vec::new());
static QUEUE_WAITS: AtomicU32 = AtomicU32::new(0);
/// Blocking-receive backoff steps (through `queue_wait`, or through `delay`
/// with the lockless delegate).
static BACKOFFS: AtomicU32 = AtomicU32::new(0);
static SYNC_WAITS: AtomicU32 = AtomicU32::new(0);
/// The condition variable the fake's `sync_wait` notifies, to prove a
/// pend polls and re-checks.
static CONDITION: ConditionVariable = ConditionVariable::new();
static MUTEX: OsMutex = OsMutex::new();
/// The queue the fake's `queue_wait` feeds, to prove a blocked receive
/// re-checks after each backoff step.
static FED: LazyLock<Queue> = LazyLock::new(Queue::new);

struct Fake;

/// Runs the routine inline at `start_task`, so `join` has nothing to wait
/// for (a superloop-style platform).
struct Done;

impl TaskHandle for Done {
    fn join(self: Box<Self>) -> task::Status {
        task::Status::OpOk
    }
}

impl Platform for Fake {
    fn start_task(&self, arguments: Arguments) -> Result<Box<dyn TaskHandle>, task::Status> {
        STARTED.lock().unwrap().push((
            arguments.name.clone(),
            arguments.priority,
            arguments.stack_size,
        ));
        (arguments.routine)();
        Ok(Box::new(Done))
    }

    fn delay(&self, interval: TimeInterval) -> task::Status {
        DELAYS
            .lock()
            .unwrap()
            .push((interval.get_seconds(), interval.get_useconds()));
        if cfg!(feature = "lockless-queue")
            && (interval.get_seconds(), interval.get_useconds())
                == (0, LOCKLESS_QUEUE_BLOCKING_BACKOFF_US)
        {
            backoff_step();
        }
        task::Status::OpOk
    }

    fn raw_time_now(&self) -> Result<RawTime, rawtime::Status> {
        Ok(RawTime::from_parts(1_700_000_000, 5))
    }

    fn queue_wait(&self) {
        QUEUE_WAITS.fetch_add(1, Ordering::SeqCst);
        backoff_step();
    }

    fn sync_wait(&self) {
        // Second poll of the pending wait: notify it.
        if SYNC_WAITS.fetch_add(1, Ordering::SeqCst) == 1 {
            CONDITION.notify();
        }
    }
}

/// Third backoff step: deliver the message the receiver waits for.
fn backoff_step() {
    if BACKOFFS.fetch_add(1, Ordering::SeqCst) == 2 {
        assert_eq!(
            FED.send(&[0xAB], 3, BlockingType::NonBlocking),
            QueueStatus::OpOk
        );
    }
}

static FAKE: Fake = Fake;

#[test]
fn platform_seam_routes_task_time_queue_and_sync_waits() {
    // --- before registration ------------------------------------------------
    assert!(platform::get().is_none());
    let task = Task::new();
    assert_eq!(
        task.start(Arguments::new("early", Box::new(|| {}))),
        task::Status::NotSupported
    );
    assert_eq!(
        Task::delay(TimeInterval::new(1, 0)),
        task::Status::NotSupported
    );
    let mut now = RawTime::new();
    assert_eq!(now.now(), rawtime::Status::NotSupported);
    // Non-blocking queue traffic needs no platform at all.
    let queue = Queue::new();
    assert_eq!(queue.create("q", 2, 4), QueueStatus::OpOk);
    assert_eq!(
        queue.send(&[1, 2], 0, BlockingType::NonBlocking),
        QueueStatus::OpOk
    );
    let (mut buf, mut size, mut prio) = ([0u8; 4], 0, 0);
    assert_eq!(
        queue.receive(&mut buf, BlockingType::NonBlocking, &mut size, &mut prio),
        QueueStatus::OpOk
    );
    assert_eq!(&buf[..size as usize], &[1, 2]);

    // --- registration is write-once -----------------------------------------
    assert!(fprime_os::register_platform(&FAKE));
    assert!(!fprime_os::register_platform(&FAKE));

    // --- Os::Task over the platform -------------------------------------------
    let ran = std::sync::Arc::new(AtomicU32::new(0));
    let ran2 = std::sync::Arc::clone(&ran);
    let task = Task::new();
    let mut arguments = Arguments::new(
        "worker",
        Box::new(move || {
            ran2.fetch_add(1, Ordering::SeqCst);
        }),
    );
    arguments.priority = 7;
    arguments.stack_size = 4096;
    assert_eq!(task.start(arguments), task::Status::OpOk);
    assert_eq!(ran.load(Ordering::SeqCst), 1);
    // The state-machine wrapper ran inside the platform's start_task.
    assert_eq!(task.get_state(), State::Running);
    assert_eq!(task.get_priority(), 7);
    assert_eq!(
        STARTED.lock().unwrap().as_slice(),
        &[("worker".to_string(), 7, 4096)]
    );
    assert_eq!(task.join(), task::Status::OpOk);
    assert_eq!(task.get_state(), State::Exited);

    assert_eq!(Task::delay(TimeInterval::new(2, 30)), task::Status::OpOk);
    assert_eq!(DELAYS.lock().unwrap().as_slice(), &[(2, 30)]);

    // --- Os::RawTime::now over the platform ---------------------------------------
    assert_eq!(now.now(), rawtime::Status::OpOk);
    assert_eq!(now.get_seconds(), 1_700_000_000);
    assert_eq!(now.get_nanoseconds(), 5);

    // --- blocking receive polls through queue_wait --------------------------
    assert_eq!(FED.create("fed", 1, 1), QueueStatus::OpOk);
    let (mut buf, mut size, mut prio) = ([0u8; 1], 0, 0);
    assert_eq!(
        FED.receive(&mut buf, BlockingType::Blocking, &mut size, &mut prio),
        QueueStatus::OpOk
    );
    assert_eq!((buf[0], size, prio), (0xAB, 1, 3));
    assert_eq!(BACKOFFS.load(Ordering::SeqCst), 3);
    if cfg!(feature = "lockless-queue") {
        // LocklessPriorityQueue backs off with Os::Task::delay.
        let backoff = (0, LOCKLESS_QUEUE_BLOCKING_BACKOFF_US);
        assert_eq!(
            DELAYS.lock().unwrap().as_slice(),
            &[(2, 30), backoff, backoff, backoff]
        );
        assert_eq!(QUEUE_WAITS.load(Ordering::SeqCst), 0);
    } else {
        assert_eq!(QUEUE_WAITS.load(Ordering::SeqCst), 3);
    }

    // --- a condition-variable pend polls through sync_wait ------------------
    let lock = MUTEX.lock();
    let (status, lock) = CONDITION.pend(lock);
    assert_eq!(status, ConditionStatus::OpOk);
    // Woken by the notify the second poll issued, mutex re-acquired.
    assert_eq!(SYNC_WAITS.load(Ordering::SeqCst), 2);
    assert!(MUTEX.try_lock().is_err());
    drop(lock);

    // --- a contended mutex lock polls through sync_wait ---------------------
    let held = MUTEX.lock();
    let contender = std::thread::spawn(|| drop(MUTEX.lock()));
    while SYNC_WAITS.load(Ordering::SeqCst) < 3 {
        std::thread::yield_now();
    }
    drop(held);
    contender.join().unwrap();
    assert!(MUTEX.try_lock().is_ok());
}
