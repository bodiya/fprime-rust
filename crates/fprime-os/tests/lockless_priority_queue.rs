//! Concurrency and stress tests for `Os::Generic::LocklessPriorityQueue`,
//! ported from upstream Os/Generic/test/ut/LocklessPriorityQueueTests.cpp
//! (`LocklessConcurrent`, `LocklessLifetime`) and
//! LocklessPriorityQueueTsanTests.cpp (`TsanStress`, `Adversarial`), plus
//! the same multi-producer / multi-consumer harness run through the
//! `Os::Queue` facade and the lock-based delegate.
//!
//! Upstream drives these through `Os::Queue` with the lockless delegate
//! linked in; here they use the delegate directly (always compiled), so
//! they run whichever delegate the `lockless-queue` feature selects for
//! the facade. The facade-only checks (`OversizedSendRejected`, the
//! lifetime tests) use `Queue` and run against the selected delegate.
//! C++ `teardown()` is `Drop`. They run in both the `std` and the `no_std`
//! configuration (the blocking-path test then backs off through a
//! `Task::delay` that returns `NotSupported` with no platform registered,
//! i.e. it re-scans immediately).
//!
//! Thread, message and iteration counts are upstream's.

use fprime_config::{FwQueuePriorityType, FwSizeType};
use fprime_os::generic::{LocklessPriorityQueue, PriorityQueue};
use fprime_os::queue::{BlockingType, Queue, QueueInterface, Status};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Barrier};

/// The queue operations the harness needs, over the facade or a delegate.
trait TestQueue: Send + Sync + 'static {
    fn send(&self, buffer: &[u8], priority: FwQueuePriorityType, block: BlockingType) -> Status;
    fn receive(
        &self,
        destination: &mut [u8],
        block: BlockingType,
    ) -> (Status, FwSizeType, FwQueuePriorityType);
    fn available(&self) -> FwSizeType;
    fn high_mark(&self) -> FwSizeType;
}

struct Delegate<D>(D);

impl<D: QueueInterface + 'static> TestQueue for Delegate<D> {
    fn send(&self, buffer: &[u8], priority: FwQueuePriorityType, block: BlockingType) -> Status {
        self.0.send(buffer, priority, block)
    }
    fn receive(
        &self,
        destination: &mut [u8],
        block: BlockingType,
    ) -> (Status, FwSizeType, FwQueuePriorityType) {
        let (mut size, mut priority) = (0, 0);
        let status = self.0.receive(destination, block, &mut size, &mut priority);
        (status, size, priority)
    }
    fn available(&self) -> FwSizeType {
        self.0.get_messages_available()
    }
    fn high_mark(&self) -> FwSizeType {
        self.0.get_message_high_water_mark()
    }
}

impl TestQueue for Queue {
    fn send(&self, buffer: &[u8], priority: FwQueuePriorityType, block: BlockingType) -> Status {
        Queue::send(self, buffer, priority, block)
    }
    fn receive(
        &self,
        destination: &mut [u8],
        block: BlockingType,
    ) -> (Status, FwSizeType, FwQueuePriorityType) {
        let (mut size, mut priority) = (0, 0);
        let status = Queue::receive(self, destination, block, &mut size, &mut priority);
        (status, size, priority)
    }
    fn available(&self) -> FwSizeType {
        self.get_messages_available()
    }
    fn high_mark(&self) -> FwSizeType {
        self.get_message_high_water_mark()
    }
}

fn lockless(depth: usize) -> Arc<dyn TestQueue> {
    Arc::new(Delegate(
        LocklessPriorityQueue::create(depth, 4).expect("create"),
    ))
}

fn lock_based(depth: usize) -> Arc<dyn TestQueue> {
    Arc::new(Delegate(PriorityQueue::create(depth, 4).expect("create")))
}

fn facade(depth: usize) -> Arc<dyn TestQueue> {
    let queue = Queue::new();
    assert_eq!(queue.create("stress", depth as FwSizeType, 4), Status::OpOk);
    Arc::new(queue)
}

const MAX_PRIORITIES: u32 = 8;

fn priority_mod8(value: u32) -> FwQueuePriorityType {
    (value % MAX_PRIORITIES) as FwQueuePriorityType
}

/// Upstream `ConcurrentConfig`.
#[derive(Clone, Copy)]
struct Config {
    depth: usize,
    producers: u32,
    consumers: u32,
    messages_per_producer: u32,
    /// Messages sent before any thread starts (values `produced..total`).
    prefill: u32,
    /// Per-message priority; `None` sends everything at priority 0.
    priority_of: Option<fn(u32) -> FwQueuePriorityType>,
    /// Expect the high-water mark to equal the depth exactly.
    expect_high_mark_equals_depth: bool,
}

struct State {
    queue: Arc<dyn TestQueue>,
    received: Box<[AtomicU32]>,
    received_by_priority: [AtomicU32; MAX_PRIORITIES as usize],
    consumed: AtomicU32,
    producers_done: AtomicBool,
    expected_total: u32,
    priority_of: Option<fn(u32) -> FwQueuePriorityType>,
}

impl State {
    fn priority_for(&self, value: u32) -> FwQueuePriorityType {
        self.priority_of.map_or(0, |f| f(value))
    }
}

/// Upstream `send_until_ok`: non-blocking send, yielding on FULL.
fn send_until_ok(state: &State, value: u32) {
    let priority = state.priority_for(value);
    loop {
        match state
            .queue
            .send(&value.to_le_bytes(), priority, BlockingType::NonBlocking)
        {
            Status::Full => std::thread::yield_now(),
            status => {
                assert_eq!(status, Status::OpOk);
                return;
            }
        }
    }
}

/// Upstream `concurrent_consumer`: receive until every message is consumed;
/// exits after a bounded number of empty polls once producers finish, so a
/// lost message reports as a shortfall instead of a hang.
fn concurrent_consumer(state: &State) {
    const EMPTY_POLL_LIMIT: u32 = 100_000;
    let mut buffer = [0u8; 4];
    let mut empty_polls = 0;
    while state.consumed.load(Ordering::Acquire) < state.expected_total {
        match state.queue.receive(&mut buffer, BlockingType::NonBlocking) {
            (Status::OpOk, size, priority) => {
                empty_polls = 0;
                assert_eq!(size, 4);
                let value = u32::from_le_bytes(buffer);
                assert!(value < state.expected_total);
                assert!(u32::from(priority) < MAX_PRIORITIES);
                let prior = state.received[value as usize].fetch_add(1, Ordering::AcqRel);
                assert_eq!(prior, 0, "duplicate delivery of value {value}");
                state.received_by_priority[priority as usize].fetch_add(1, Ordering::AcqRel);
                state.consumed.fetch_add(1, Ordering::AcqRel);
            }
            (Status::Empty, _, _) => {
                empty_polls += 1;
                if state.producers_done.load(Ordering::Acquire) && empty_polls > EMPTY_POLL_LIMIT {
                    break;
                }
                std::thread::yield_now();
            }
            (status, _, _) => panic!("unexpected status {status:?}"),
        }
    }
}

/// Upstream `run_concurrent`.
fn run_concurrent(queue: Arc<dyn TestQueue>, config: Config) {
    let produced = config.producers * config.messages_per_producer;
    let total = produced + config.prefill;
    let state = Arc::new(State {
        queue,
        received: (0..total).map(|_| AtomicU32::new(0)).collect(),
        received_by_priority: Default::default(),
        consumed: AtomicU32::new(0),
        producers_done: AtomicBool::new(false),
        expected_total: total,
        priority_of: config.priority_of,
    });

    assert!(config.prefill as usize <= config.depth);
    for i in 0..config.prefill {
        send_until_ok(&state, produced + i);
    }
    if config.prefill > 0 {
        assert_eq!(state.queue.available(), FwSizeType::from(config.prefill));
    }

    let consumers: Vec<_> = (0..config.consumers)
        .map(|_| {
            let state = Arc::clone(&state);
            std::thread::spawn(move || concurrent_consumer(&state))
        })
        .collect();
    let producers: Vec<_> = (0..config.producers)
        .map(|producer| {
            let state = Arc::clone(&state);
            std::thread::spawn(move || {
                for i in 0..config.messages_per_producer {
                    send_until_ok(&state, producer * config.messages_per_producer + i);
                }
            })
        })
        .collect();
    for producer in producers {
        producer.join().expect("producer");
    }
    state.producers_done.store(true, Ordering::Release);
    for consumer in consumers {
        consumer.join().expect("consumer");
    }

    assert_eq!(state.consumed.load(Ordering::Acquire), total);
    for (value, count) in state.received.iter().enumerate() {
        assert_eq!(count.load(Ordering::Relaxed), 1, "value {value}");
    }
    if let Some(priority_of) = config.priority_of {
        let mut expected = [0u32; MAX_PRIORITIES as usize];
        for value in 0..total {
            expected[priority_of(value) as usize] += 1;
        }
        for (priority, expected) in expected.iter().enumerate() {
            assert_eq!(
                state.received_by_priority[priority].load(Ordering::Relaxed),
                *expected,
                "priority={priority}"
            );
        }
    }
    if config.expect_high_mark_equals_depth {
        assert_eq!(state.queue.high_mark(), config.depth as FwSizeType);
    }
    // The count invariant (SDD section 8): never above the depth.
    assert!(state.queue.high_mark() <= config.depth as FwSizeType);
    assert_eq!(state.queue.available(), 0);
}

fn config(
    depth: usize,
    producers: u32,
    consumers: u32,
    messages_per_producer: u32,
    prefill: u32,
    priority_of: Option<fn(u32) -> FwQueuePriorityType>,
    expect_high_mark_equals_depth: bool,
) -> Config {
    Config {
        depth,
        producers,
        consumers,
        messages_per_producer,
        prefill,
        priority_of,
        expect_high_mark_equals_depth,
    }
}

// ---------------------------------------------------------------------------
// LocklessPriorityQueueTests.cpp
// ---------------------------------------------------------------------------

// TEST(LocklessConcurrent, MultiProducerMultiConsumer): 4x4 threads, 4000
// messages through 64 slots, exactly-once delivery.
#[test]
fn concurrent_multi_producer_multi_consumer() {
    run_concurrent(
        lockless(64),
        config(64, 4, 4, 1000, 0, Some(|value| (value & 0x7) as u8), false),
    );
}

// TEST(LocklessConcurrent, PriorityOrderSingleProducer).
#[test]
fn concurrent_priority_order_single_producer() {
    let queue = lockless(32);
    for batch in 0..200u32 {
        for i in 0..16u32 {
            let value = batch * 16 + i;
            assert_eq!(
                queue.send(&value.to_le_bytes(), i as u8, BlockingType::NonBlocking),
                Status::OpOk
            );
        }
        let mut last = FwQueuePriorityType::MAX;
        for _ in 0..16 {
            let (status, _, priority) = queue.receive(&mut [0u8; 4], BlockingType::NonBlocking);
            assert_eq!(status, Status::OpOk);
            assert!(priority <= last);
            last = priority;
        }
    }
}

// TEST(LocklessLifetime, DestructWithoutCreate) / CreateTeardownDestruct /
// TeardownIsIdempotent: storage is released by Drop, exactly once, whether
// or not the queue was created.
#[test]
fn lifetime_drop_with_and_without_create() {
    {
        let queue = Queue::new();
        assert_eq!(queue.get_messages_available(), 0);
    }
    {
        let queue = Queue::new();
        assert_eq!(queue.create("lifetime-test", 4, 4), Status::OpOk);
        assert_eq!(queue.send(&[1], 0, BlockingType::NonBlocking), Status::OpOk);
        // Dropped with a message still queued.
    }
}

// TEST(LocklessLifetime, OversizedSendRejected) (through the facade).
#[test]
fn lifetime_oversized_send_rejected() {
    let queue = Queue::new();
    assert_eq!(queue.create("oversize-test", 4, 4), Status::OpOk);
    assert_eq!(
        queue.send(&[0u8; 5], 0, BlockingType::NonBlocking),
        Status::SizeMismatch
    );
    assert_eq!(queue.get_messages_available(), 0);
}

// ---------------------------------------------------------------------------
// LocklessPriorityQueueTsanTests.cpp: TsanStress
// ---------------------------------------------------------------------------

// 8 producers, 8 consumers, 40k messages through a 32-slot queue.
#[test]
fn tsan_stress_high_thread_concurrent() {
    for _ in 0..20 {
        run_concurrent(
            lockless(32),
            config(32, 8, 8, 5000, 0, Some(priority_mod8), false),
        );
    }
}

// Many threads fighting over a tiny queue.
#[test]
fn tsan_stress_tiny_queue_high_contention() {
    for _ in 0..50 {
        run_concurrent(
            lockless(4),
            config(4, 8, 8, 500, 0, Some(priority_mod8), false),
        );
    }
}

// Many producers, few consumers: the queue lives near FULL.
#[test]
fn tsan_stress_many_producers_few_consumers() {
    for _ in 0..20 {
        run_concurrent(
            lockless(16),
            config(16, 12, 2, 1000, 0, Some(priority_mod8), false),
        );
    }
}

// Few producers, many consumers: the queue lives near EMPTY.
#[test]
fn tsan_stress_few_producers_many_consumers() {
    for _ in 0..20 {
        run_concurrent(
            lockless(16),
            config(16, 2, 12, 5000, 0, Some(priority_mod8), false),
        );
    }
}

// ---------------------------------------------------------------------------
// LocklessPriorityQueueTsanTests.cpp: Adversarial
// ---------------------------------------------------------------------------

// Depth 1: every send/receive pair races for the single slot.
#[test]
fn adversarial_depth_one_ping_pong() {
    for _ in 0..50 {
        run_concurrent(
            lockless(1),
            config(1, 4, 4, 500, 0, Some(priority_mod8), false),
        );
    }
}

// Pre-filled queue, then many producers (all FULL at first) while
// consumers drain.
#[test]
fn adversarial_full_queue_storm() {
    run_concurrent(lockless(16), config(16, 8, 8, 2000, 16, None, false));
}

// One producer, one consumer, depth 1: strict FIFO hand-off.
#[test]
fn adversarial_single_slot_ping_pong() {
    let messages: u32 = 50_000;
    let queue = lockless(1);
    let producer_queue = Arc::clone(&queue);
    let producer = std::thread::spawn(move || {
        for value in 0..messages {
            while producer_queue.send(&value.to_le_bytes(), 0, BlockingType::NonBlocking)
                == Status::Full
            {
                std::thread::yield_now();
            }
        }
    });
    let consumer_queue = Arc::clone(&queue);
    let consumer = std::thread::spawn(move || {
        let mut expected = 0;
        while expected < messages {
            let mut buffer = [0u8; 4];
            match consumer_queue.receive(&mut buffer, BlockingType::NonBlocking) {
                (Status::OpOk, size, _) => {
                    assert_eq!(size, 4);
                    assert_eq!(u32::from_le_bytes(buffer), expected, "out of order");
                    expected += 1;
                }
                _ => std::thread::yield_now(),
            }
        }
    });
    producer.join().expect("producer");
    consumer.join().expect("consumer");
}

// Pre-filled to the depth before any consumer starts: the mark must equal
// the depth and can never exceed it.
#[test]
fn adversarial_high_water_mark_accuracy() {
    run_concurrent(lockless(16), config(16, 8, 1, 2000, 16, None, true));
}

// Send 3, drain, send 1: the mark is exactly 3.
#[test]
fn adversarial_high_water_mark_intermediate() {
    let queue = lockless(8);
    for value in 0..3u32 {
        assert_eq!(
            queue.send(&value.to_le_bytes(), 0, BlockingType::NonBlocking),
            Status::OpOk
        );
    }
    for _ in 0..3 {
        assert_eq!(
            queue.receive(&mut [0u8; 4], BlockingType::NonBlocking).0,
            Status::OpOk
        );
    }
    assert_eq!(
        queue.send(&0u32.to_le_bytes(), 0, BlockingType::NonBlocking),
        Status::OpOk
    );
    assert_eq!(queue.high_mark(), 3);
}

/// Upstream `BlockingPathStress`: blocking sends into a small queue against
/// blocking receives, exercising the backoff control flow.
fn blocking_path_stress(queue: Arc<dyn TestQueue>) {
    const THREADS: u32 = 4;
    let per_producer: u32 = 1000;
    let total = THREADS * per_producer;
    let received: Arc<Box<[AtomicU32]>> = Arc::new((0..total).map(|_| AtomicU32::new(0)).collect());
    let producers: Vec<_> = (0..THREADS)
        .map(|thread| {
            let queue = Arc::clone(&queue);
            std::thread::spawn(move || {
                for i in 0..per_producer {
                    let value = thread * per_producer + i;
                    assert_eq!(
                        queue.send(&value.to_le_bytes(), 0, BlockingType::Blocking),
                        Status::OpOk
                    );
                }
            })
        })
        .collect();
    let consumers: Vec<_> = (0..THREADS)
        .map(|_| {
            let queue = Arc::clone(&queue);
            let received = Arc::clone(&received);
            std::thread::spawn(move || {
                for _ in 0..per_producer {
                    let mut buffer = [0u8; 4];
                    let (status, _, _) = queue.receive(&mut buffer, BlockingType::Blocking);
                    assert_eq!(status, Status::OpOk);
                    let value = u32::from_le_bytes(buffer);
                    assert!(value < total);
                    received[value as usize].fetch_add(1, Ordering::AcqRel);
                }
            })
        })
        .collect();
    for thread in producers.into_iter().chain(consumers) {
        thread.join().expect("worker");
    }
    for (value, count) in received.iter().enumerate() {
        assert_eq!(count.load(Ordering::Relaxed), 1, "value {value}");
    }
    assert_eq!(queue.available(), 0);
}

#[test]
fn adversarial_blocking_path_stress() {
    blocking_path_stress(lockless(4));
}

// Fresh create / use / drop cycles.
#[test]
fn adversarial_create_teardown_recreate() {
    for _ in 0..5 {
        run_concurrent(lockless(8), config(8, 4, 4, 250, 0, None, false));
    }
}

// Concurrent mixed priorities (exactly-once + per-priority counts), then a
// deterministic drain: highest priority first, FIFO within a priority.
#[test]
fn adversarial_mixed_priority_contention() {
    run_concurrent(
        lockless(32),
        config(32, 4, 4, 2000, 0, Some(priority_mod8), false),
    );
    let queue = lockless(32);
    for value in 0..32u32 {
        let priority = ((value * 5) % MAX_PRIORITIES) as u8;
        assert_eq!(
            queue.send(&value.to_le_bytes(), priority, BlockingType::NonBlocking),
            Status::OpOk
        );
    }
    let mut last: Option<(FwQueuePriorityType, u32)> = None;
    for _ in 0..32 {
        let mut buffer = [0u8; 4];
        let (status, _, priority) = queue.receive(&mut buffer, BlockingType::NonBlocking);
        assert_eq!(status, Status::OpOk);
        let value = u32::from_le_bytes(buffer);
        if let Some((last_priority, last_value)) = last {
            if priority == last_priority {
                assert!(last_value < value, "equal-priority FIFO violated");
            } else {
                assert!(priority < last_priority, "priority order violated");
            }
        }
        last = Some((priority, value));
    }
    assert_eq!(queue.available(), 0);
}

// Many consumers race to drain a pre-filled queue, then it is dropped.
#[test]
fn adversarial_rapid_drain_then_teardown() {
    run_concurrent(lockless(64), config(64, 1, 8, 9936, 64, None, false));
}

// A full batch of mixed priorities, drained by consumers released together:
// each consumer's own receive sequence must be non-increasing in priority,
// and the per-priority totals must match the batch.
#[test]
fn adversarial_batched_priority_drain() {
    const DEPTH: u32 = 32;
    const CONSUMERS: usize = 4;
    let queue = lockless(DEPTH as usize);
    for batch in 0..200 {
        let mut sent_by_priority = [0u32; MAX_PRIORITIES as usize];
        for i in 0..DEPTH {
            let priority = ((i * 5 + batch) % MAX_PRIORITIES) as u8;
            sent_by_priority[priority as usize] += 1;
            assert_eq!(
                queue.send(&i.to_le_bytes(), priority, BlockingType::NonBlocking),
                Status::OpOk
            );
        }
        let claimed = Arc::new(AtomicU32::new(0));
        let received_by_priority: Arc<[AtomicU32; MAX_PRIORITIES as usize]> =
            Arc::new(Default::default());
        let go = Arc::new(Barrier::new(CONSUMERS));
        let consumers: Vec<_> = (0..CONSUMERS)
            .map(|_| {
                let (queue, claimed, received_by_priority, go) = (
                    Arc::clone(&queue),
                    Arc::clone(&claimed),
                    Arc::clone(&received_by_priority),
                    Arc::clone(&go),
                );
                std::thread::spawn(move || {
                    go.wait();
                    let mut last: Option<FwQueuePriorityType> = None;
                    while claimed.load(Ordering::Acquire) < DEPTH {
                        match queue.receive(&mut [0u8; 4], BlockingType::NonBlocking) {
                            (Status::OpOk, _, priority) => {
                                if let Some(last) = last {
                                    assert!(priority <= last, "batch={batch}");
                                }
                                last = Some(priority);
                                received_by_priority[priority as usize]
                                    .fetch_add(1, Ordering::AcqRel);
                                claimed.fetch_add(1, Ordering::AcqRel);
                            }
                            _ => std::thread::yield_now(),
                        }
                    }
                })
            })
            .collect();
        for consumer in consumers {
            consumer.join().expect("consumer");
        }
        assert_eq!(claimed.load(Ordering::Acquire), DEPTH);
        for (priority, sent) in sent_by_priority.iter().enumerate() {
            assert_eq!(
                received_by_priority[priority].load(Ordering::Relaxed),
                *sent,
                "batch={batch} priority={priority}"
            );
        }
        assert_eq!(queue.available(), 0);
    }
}

// ---------------------------------------------------------------------------
// The same harness over the facade and the lock-based delegate.
// ---------------------------------------------------------------------------

// Host multi-threaded stress through `Os::Queue` (whichever delegate this
// build selects) and the lock-based delegate: exactly-once delivery,
// per-priority totals and the high-water-mark bound, non-blocking and
// blocking paths.
#[test]
fn facade_and_lock_based_delegate_stress() {
    for make in [facade, lock_based] {
        run_concurrent(
            make(16),
            config(16, 6, 6, 2000, 0, Some(priority_mod8), false),
        );
        run_concurrent(make(1), config(1, 4, 4, 500, 0, Some(priority_mod8), false));
        run_concurrent(make(16), config(16, 8, 1, 1000, 16, None, true));
        blocking_path_stress(make(4));
    }
}
