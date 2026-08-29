# Os abstraction layer (OSAL) — /home/user/fprime/Os

> Analysis of the C++ F Prime implementation (github.com/nasa/fprime) produced to guide this Rust port.
> Source tree analyzed: local clone at commit of 2026-08. File paths refer to that C++ tree.

## Overview

Os/ is F Prime's OS abstraction. Every facility (Task, Mutex, ConditionVariable, CountingSemaphore, Queue, File, FileSystem, Directory, Console, RawTime, Cpu, Memory) follows one pattern: an abstract `XInterface` with a pure-virtual API plus a static `getDelegate(XHandleStorage&)`, and a `final` front class `Os::X` that owns an inline aligned byte array (`alignas(FW_HANDLE_ALIGNMENT) U8[FW_X_HANDLE_MAX_SIZE]`, Os/Os.hpp) into which the platform implementation is placement-new'd. The platform is chosen at LINK time: exactly one `Default*.cpp` translation unit (e.g. Os/Posix/DefaultTask.cpp, Os/Generic/DefaultPriorityQueue.cpp) defines `getDelegate` by calling `Os::Delegate::makeDelegate<Interface, Impl>` (Os/Delegate.hpp), which static-asserts Impl derives from Interface, fits in storage, and alignment divides FW_HANDLE_ALIGNMENT. Front-class methods forward to `m_delegate` after asserting `&m_delegate == &m_handle_storage[0]`; destructors invoke `m_delegate.~XInterface()` explicitly. File/Console/RawTime getDelegate additionally accept a `to_copy` pointer to support copy construction. FileSystem/Console/Cpu/Memory/Task are singletons (function-local statics) initialized by `Os::init()` (Os/Os.cpp: Console, FileSystem, Cpu, Memory, Task). Default queue delegate on POSIX platforms is the OS-agnostic `Os::Generic::PriorityQueue` (mutex + 2 condvars + stable MaxHeap), not an OS message queue. Components (ActiveComponentBase etc.) sit on Os::Queue and Os::Task, so exact status enums, blocking semantics, and FIFO-within-priority ordering are contract-critical. Alternative queue delegates exist: LocklessPriorityQueue (ISR-safe atomic slot state machine) and PriorityMemQueue (per-priority AtomicQueues + counting semaphore, max 32 priorities). Config constants come from default/config (PlatformCfg.fpp, FpConfig, OsCfg.fpp); unix platform types: FwSizeType=U64, FwSignedSizeType=I64, FwTaskPriorityType=U8, FwQueuePriorityType=U8, FwTaskIdType=I32.

## Key items

### Delegate pattern + handle storage

Files: `Os/Delegate.hpp`, `Os/Os.hpp`, `default/config/PlatformCfg.fpp`, `Os/Posix/DefaultTask.cpp`, `Os/Posix/DefaultMutex.cpp`, `Os/Posix/DefaultFile.cpp`, `Os/Posix/DefaultRawTime.cpp`, `Os/Generic/DefaultPriorityQueue.cpp`

Handle storage typedefs: U8[N] with N from PlatformCfg.fpp: CONSOLE=24, TASK=40, FILE=16, MUTEX=72, QUEUE=368, DIRECTORY=16, FILESYSTEM=16, RAW_TIME=56, COUNTING_SEMAPHORE=48, CONDITION_VARIABLE=56, CPU=16, MEMORY=16; FW_HANDLE_ALIGNMENT=8; FW_FILE_CHUNK_SIZE=512; FW_RAW_TIME_SERIALIZATION_MAX_SIZE=8. makeDelegate: static_assert derives/fits/aligns, placement-new (default or copy-ctor when to_copy!=nullptr), FW_ASSERT non-null. Every delegating call asserts delegate address == storage address. Front classes are `final`; copy allowed only for File, Console, RawTime (DelegateRawTime). Os::RawTime itself is a config-time type alias (config/OsDelegateRawTime.hpp defines the alias + OS_RAW_TIME_HEADER; Os/RawTime.hpp include order is load-bearing).

### QueueInterface / Os::Queue

Files: `Os/Queue.hpp`, `Os/Queue.cpp`, `Os/Models/Queue.fpp`

Status (0-based, in order): OP_OK, ALREADY_CREATED, EMPTY, UNINITIALIZED, SIZE_MISMATCH, SEND_ERROR, RECEIVE_ERROR, INVALID_PRIORITY, FULL, NOT_SUPPORTED, ALLOCATION_FAILED, UNKNOWN_ERROR. BlockingType: BLOCKING=0, NONBLOCKING=1. create(id,name,depth,messageSize): FW_ASSERT depth>0 && messageSize>0; if already created (m_depth>0||m_size>0) return ALREADY_CREATED; on OP_OK store name/depth/size, ++s_queueCount under static mutex, optional QueueRegistry callback (FW_QUEUE_REGISTRATION default 1). send: FW_ASSERT buffer!=nullptr; UNINITIALIZED if not created; SIZE_MISMATCH if size > messageSize; else delegate. receive: FW_ASSERT dest!=nullptr; UNINITIALIZED if not created; SIZE_MISMATCH if capacity < configured messageSize (full message size, not actual!). LinearBufferBase overloads: receive resets buffer, then setBuffLen(actualSize) — failure maps to SIZE_MISMATCH. Blocking send blocks on full; blocking receive blocks on empty; NONBLOCKING returns FULL / EMPTY respectively. QueueString capacity = FW_QUEUE_NAME_BUFFER_SIZE = 80.

### Os::Generic::PriorityQueue (default queue delegate)

Files: `Os/Generic/PriorityQueue.hpp`, `Os/Generic/PriorityQueue.cpp`

Handle: MaxHeap (priority ordering), U8* data (depth*maxSize unordered slabs), FwSizeType* indices (circular free-index list with m_startIndex/m_stopIndex mod depth), FwSizeType* sizes (per-slot), highMark, Os::Mutex m_data_lock, condvars m_full & m_empty. create: allocates 4 blocks via Fw::MemAllocatorRegistry::getAnAllocator(MemoryAllocatorType::OS_GENERIC_PRIORITY_QUEUE); overflow-guard FW_ASSERTs (depth*sizeof(FwSizeType), depth*messageSize); nullptr or short allocation => ALLOCATION_FAILED with rollback of prior blocks; indices[i]=i, sizes[i]=0. send(buffer,size,priority,blockType): size>maxSize => SIZE_MISMATCH (before lock); ScopeLock; heap full && NONBLOCKING => FULL; while(full) m_full.wait(lock); index=find_index() (pop free index at startIndex++), heap.push(priority,index) asserted true; memcpy into slab, sizes[index]=size; highMark=max(highMark,heap.getSize()); unlock THEN m_empty.notify(). receive(dest,capacity,...): ScopeLock; empty && NONBLOCKING => EMPTY; while(empty) m_empty.wait(lock); heap.pop asserted; actualSize=sizes[index]; FW_ASSERT(actualSize<=capacity) — an assert, NOT a status; memcpy out; return_index(index) at stopIndex++; unlock then m_full.notify(). getMessagesAvailable(): heap.getSize() with NO lock; getMessageHighWaterMark(): under lock (const_cast). Not ISR safe. teardown deallocates all four blocks and nulls pointers; only when m_data!=nullptr.

### Types::MaxHeap (stable max-heap)

Files: `Os/Generic/Types/MaxHeap.hpp`, `Os/Generic/Types/MaxHeap.cpp`

Array-backed binary max-heap of Node{FwQueuePriorityType value; FwSizeType order; FwSizeType id}; ELEMENT_SIZE=sizeof(Node), ALIGNMENT=alignof(Node). push(value,id): returns false when full; sift-up copies parent down while value > parent.value (STRICT >: an equal-priority push stops, staying below/after existing entries), then writes {value, order=m_order++, id}; O(log n). pop(&value,&id): false when empty; returns root; moves last node to root; iterative heapify-down. Stability: max(a,b) tie-breaks equal values by age = m_order - node.order (unsigned wraparound-safe); larger age (older) wins => strict FIFO within one priority. Loop-bound bit-flip guards: bounded iterations (maxIter=size+1) with FW_ASSERT on exit conditions. create() placement-constructs Node[capacity] into caller-supplied Fw::ByteArray; asserts capacity < FwSizeType::max.

### TaskInterface / Os::Task state machine

Files: `Os/Task.hpp`, `Os/Task.cpp`, `Os/Models/Task.fpp`

Status (in order): OP_OK, INVALID_HANDLE, INVALID_PARAMS, INVALID_PRIORITY, INVALID_STACK, UNKNOWN_ERROR, INVALID_AFFINITY, DELAY_ERROR, JOIN_ERROR, ERROR_RESOURCES, ERROR_PERMISSION, NOT_SUPPORTED, INVALID_STATE. State: NOT_STARTED, STARTING, RUNNING, SUSPENDED_INTENTIONALLY, SUSPENDED_UNINTENTIONALLY, EXITED, UNKNOWN. SuspensionType: INTENTIONAL, UNINTENTIONAL. TASK_DEFAULT = FwSizeType::max(); TASK_PRIORITY_DEFAULT = FwTaskPriorityType::max() (FPP constant TASK_DEFAULT=-1 casts to these). Arguments ctor FW_ASSERTs routine != nullptr. start(): sets m_state=STARTING BEFORE delegate.start; swaps routine/arg for TaskRoutineWrapper::run + &m_wrapper; on OP_OK stores priority, ++s_numTasks under s_taskMutex, registers with TaskRegistry if set (m_registered=true). Wrapper run() (in new thread): asserts state != NOT_STARTED; if STARTING => set RUNNING (under m_lock), call onStart(), then user routine. join(): only from RUNNING or STARTING else INVALID_STATE; delegate join OP_OK => EXITED else UNKNOWN. suspend() no-arg = UNINTENTIONAL; suspend(type) delegates then sets state SUSPENDED_*. isCooperative() default false (cooperative impls run one unit of work per invokeRoutine()). Static delay(interval) uses singleton's _delay. Destructor removes from registry if registered.

### PosixTask

Files: `Os/Posix/Task.cpp`, `Os/Posix/Task.hpp`, `Os/Posix/error.cpp`

pthread-based; SCHED_POLICY=SCHED_RR, PTHREAD_EXPLICIT_SCHED. Handle: pthread_t + m_is_valid + optional 16-char name (POSIX_THREADS_ENABLE_NAMES). Non-default stackSize rounded DOWN to page-size multiple (warn) and clamped up to PTHREAD_STACK_MIN (warn); priority clamped into [sched_get_priority_min, max] with warnings; affinity only on glibc/Linux/_GNU_SOURCE, FW_ASSERT affinity < CPU_SETSIZE. Two-phase start: create with EXPECT_PERMISSION; on ERROR_PERMISSION log 3-option notice once (std::atomic<bool> s_permissions_reported) and retry with EXPECT_NO_PERMISSION (skips priority+affinity). posix_status_to_task_status: 0=>OP_OK, EINVAL=>INVALID_PARAMS, EPERM=>ERROR_PERMISSION, EAGAIN=>ERROR_RESOURCES, else UNKNOWN_ERROR. join: INVALID_HANDLE if !m_is_valid; pthread_join==0 => OP_OK else JOIN_ERROR. suspend/resume: FW_ASSERT(false) (unsupported). _delay: nanosleep loop, EINTR resumes with remaining interval; other error => DELAY_ERROR.

### Mutex + ScopeLock

Files: `Os/Mutex.hpp`, `Os/Mutex.cpp`, `Os/Posix/Mutex.cpp`

Status: OP_OK, ERROR_BUSY, ERROR_DEADLOCK, NOT_SUPPORTED, ERROR_OTHER. take()/release() return Status; lock()/unLock() (alias unlock()) FW_ASSERT OP_OK (args: this-pointer, status). ScopeLock: ctor lock(), dtor unLock(). Posix: pthread_mutex with PTHREAD_MUTEX_ERRORCHECK type and PTHREAD_PRIO_INHERIT protocol; init/destroy failures assert. posix map: 0=>OP_OK, EBUSY=>ERROR_BUSY, EDEADLK=>ERROR_DEADLOCK, else ERROR_OTHER. Created unlocked.

### ConditionVariable

Files: `Os/Condition.hpp`, `Os/Condition.cpp`, `Os/Posix/ConditionVariable.cpp`

Status: OP_OK, ERROR_MUTEX_NOT_HELD, ERROR_DIFFERENT_MUTEX, ERROR_NOT_IMPLEMENTED, NOT_SUPPORTED, ERROR_OTHER. Wrapper stores first mutex used (m_lock ptr); pend() with a different mutex => ERROR_DIFFERENT_MUTEX (association is sticky, never cleared). wait(mutex) = pend + FW_ASSERT(OP_OK). pend atomically unlocks mutex and blocks; caller MUST hold mutex and MUST recheck predicate in a loop. notify/notifyAll intended to be called WITHOUT holding the mutex (PriorityQueue notifies after unlock). Posix: pthread_cond_wait; EPERM=>ERROR_MUTEX_NOT_HELD; signal/broadcast failures assert. No timed wait in the interface.

### CountingSemaphore

Files: `Os/CountingSemaphore.hpp`, `Os/Posix/CountingSemaphore.cpp`

Status: OP_OK, ERROR_TIMEOUT, ERROR_INVALID, ERROR_NOT_IMPLEMENTED, NOT_SUPPORTED, ERROR_OTHER. getDelegate takes U32 initial_count. wait() blocks; tryWait() returns ERROR_TIMEOUT when count 0; waitTimeout(TimeInterval) absolute deadline from CLOCK_REALTIME (sec + usec*1000 nsec, normalized); Posix sem_init(pshared=0); wait/timedwait retry EINTR; errno map: ETIMEDOUT|EAGAIN=>ERROR_TIMEOUT, EINVAL=>ERROR_INVALID, else ERROR_OTHER.

### File (front) semantics

Files: `Os/File.hpp`, `Os/File.cpp`

Mode: OPEN_NO_MODE=0, OPEN_READ=1, OPEN_CREATE=2, OPEN_WRITE=3, OPEN_SYNC_WRITE=4, OPEN_APPEND=5, MAX_OPEN_MODE. Status: OP_OK=0, DOESNT_EXIST, NO_SPACE, NO_PERMISSION, BAD_SIZE, NOT_OPENED, FILE_EXISTS, NOT_SUPPORTED, INVALID_MODE, INVALID_ARGUMENT, NO_MORE_RESOURCES, OTHER_ERROR, OUTSIDE_SANDBOX, MAX_STATUS. OverwriteType{NO_OVERWRITE,OVERWRITE}, SeekType{RELATIVE,ABSOLUTE}, WaitType{NO_WAIT,WAIT}. open on an already-open file => INVALID_MODE; default overwrite=NO_OVERWRITE; success stores mode and re-inits CRC hash. isOpen = mode != OPEN_NO_MODE; destructor closes if open. Gates: size/position/seek require open (NOT_OPENED); read requires mode==OPEN_READ else INVALID_MODE; write/preallocate/flush require open && mode!=OPEN_READ. seek FW_ASSERTs offset>=0 for ABSOLUTE. seek_absolute(FwSizeType): 1 seek if fits FwSignedSizeType else half+half+odd-byte (3 seeks). readline: scans chunks (512) for '\n'; returns size including newline and positions after it; EOF without newline => OP_OK with what was read; newline not found in buffer or any error => size=0, seek back to original position, OTHER_ERROR/error. CRC: incrementalCrc reads NO_WAIT into 512-byte buffer, Utils::Hash::update; calculateCrc loops until short read; finalizeCrc: crc = ~hash.finalize() (undoes hash's final complement for backward compat; effectively CRC32 with init 0xFFFFFFFF and no final XOR), then re-init. Copy ctor/assignment copy delegate via copy-getDelegate (Posix dups fd with fcntl F_DUPFD); operator= closes its own open file first.

### PosixFile

Files: `Os/Posix/File.cpp`, `Os/Posix/error.cpp`, `default/config/OsCfg.fpp`

open flags: READ=>O_RDONLY; WRITE=>O_WRONLY|O_CREAT; SYNC_WRITE=>O_WRONLY|O_CREAT|O_SYNC (falls back to plain write if O_SYNC undefined); CREATE=>O_WRONLY|O_CREAT|O_TRUNC|(NO_OVERWRITE? O_EXCL : 0); APPEND=>O_WRONLY|O_CREAT|O_APPEND. Create permission bits from FILE_DEFAULT_CREATE_MODE = rw for user/group/other (0666); Os::FILE_MODE_* constants (IRUSR=0x040, IWUSR=0x080, IXUSR=0x100, IRGRP=0x008 ... ISUID=0x800, ISGID=0x400, ISVTX=0x200) are portably remapped to platform S_I* bits. errno_to_file_status: ENOSPC|EFBIG=>NO_SPACE, ENOENT=>DOESNT_EXIST, EPERM|EACCES=>NO_PERMISSION, EEXIST=>FILE_EXISTS, EBADF=>NOT_OPENED, ENOSYS|EOPNOTSUPP=>NOT_SUPPORTED, EINVAL=>INVALID_ARGUMENT, else OTHER_ERROR. read: size>SSIZE_MAX=>BAD_SIZE; loop bounded to 2*size iterations; EINTR retries; read_size==0 (EOF) breaks; NO_WAIT breaks after first successful read; size out-param = accumulated. write: same bounding; loops until all written regardless of wait; WAIT additionally fsync()s. size(): position, lseek(END), lseek back. preallocate: posix_fallocate when available; NOT_SUPPORTED result triggers fallback writing zero bytes one at a time from EOF to offset+length, restoring position; range overflow => BAD_SIZE. close(): ignores errors, resets fd to -1.

### FileSystem singleton + composites

Files: `Os/FileSystem.hpp`, `Os/FileSystem.cpp`, `Os/Posix/FileSystem.cpp`

Status: OP_OK, ALREADY_EXISTS, NO_SPACE, NO_PERMISSION, NOT_DIR, IS_DIR, NOT_EMPTY, INVALID_PATH, DOESNT_EXIST, FILE_LIMIT, BUSY, NO_MORE_FILES, BUFFER_TOO_SMALL, EXDEV_ERROR, OVERFLOW_ERROR, NOT_SUPPORTED, OTHER_ERROR. PathType: FILE, DIRECTORY, OTHER, NOT_EXIST. Static API delegates to singleton `_x` methods (rmdir/unlink/rename/statvfs/lstat/getcwd/chdir). errno_to_filesystem_status notable: ELOOP|ENOENT=>DOESNT_EXIST, ENAMETOOLONG=>INVALID_PATH, ENOTDIR=>NOT_DIR, EDQUOT|ENOSPC|EFBIG=>NO_SPACE, EMLINK=>FILE_LIMIT, ERANGE=>BUFFER_TOO_SMALL, EXDEV=>EXDEV_ERROR, EPERM|EACCES|EROFS|EFAULT=>NO_PERMISSION. Composites: createDirectory via Directory::open(CREATE_EXCLUSIVE or CREATE_IF_MISSING); touch = File open OPEN_WRITE+close; exists = getPathType != NOT_EXIST (any error => NOT_EXIST); copyFile/appendFile = open both + chunked copy (512-byte stack buffer, WAIT reads/writes, loop bound 2*size, short-write => OTHER_ERROR, zero-read breaks); moveFile = rename, and ONLY on EXDEV_ERROR falls back to copy+removeFile; getFileSize = open READ + size. handleFileError maps only NO_SPACE/NO_PERMISSION/DOESNT_EXIST, everything else => OTHER_ERROR. _getFreeSpace: statvfs f_frsize*f_bfree/f_blocks with div-by-zero => OTHER_ERROR, mul overflow => OVERFLOW_ERROR. _getWorkingDirectory asserts bufferSize>0.

### Directory

Files: `Os/Directory.hpp`, `Os/Posix/Directory.cpp`

Status: OP_OK, DOESNT_EXIST, NO_PERMISSION, NOT_OPENED, NOT_DIR, NO_MORE_FILES, FILE_LIMIT, BAD_DESCRIPTOR, ALREADY_EXISTS, NOT_SUPPORTED, OTHER_ERROR. OpenMode: READ, CREATE_IF_MISSING, CREATE_EXCLUSIVE. Posix open: CREATE_* first mkdir(path, S_IRWXU); EEXIST tolerated only for CREATE_IF_MISSING; then opendir. read(buf,size): readdir loop skipping "." and ".."; copies name (silent truncation via string_copy); end-of-stream => NO_MORE_FILES; errno!=0 => BAD_DESCRIPTOR. rewind via rewinddir always OP_OK. Front class tracks m_is_open; readDirectory/getFileCount rewind before AND after; destructor closes. errno map: ENOENT=>DOESNT_EXIST, EACCES=>NO_PERMISSION, ENOTDIR=>NOT_DIR, EEXIST=>ALREADY_EXISTS, else OTHER_ERROR.

### Console

Files: `Os/Console.hpp`, `Os/Console.cpp`, `Os/Posix/Console.cpp`

Console : ConsoleInterface + Fw::Logger. Singleton; first getSingleton() registers itself as the global Fw::Logger. static write(msg,size)/write(ConstStringBase) go through singleton writeMessage. Posix impl: fwrite + fflush to a FILE* (default stdout, switchable to stderr via setOutputStream); size capped at size_t max; null message ignored. Copyable via copy-delegate.

### RawTime / IntervalTimer

Files: `Os/RawTimeInterface.hpp`, `Os/RawTime.hpp`, `Os/DelegateRawTime.cpp`, `Os/Posix/RawTime.cpp`, `Os/IntervalTimer.cpp`

Status: OP_OK, OP_OVERFLOW, INVALID_PARAMS, NOT_SUPPORTED, OTHER_ERROR. RawTimeInterface is Fw::Serializable with SERIALIZED_SIZE = FW_RAW_TIME_SERIALIZATION_MAX_SIZE (8). Os::RawTime is an alias chosen by config/OsDelegateRawTime.hpp: either Os::DelegateRawTime (delegate wrapper, copyable, optional RawTimeSource selector param) or a concrete class. now() samples the clock; getTimeInterval(other, out) computes the ABSOLUTE difference (commutative, always non-negative) as Fw::TimeInterval(sec, usec). Default getDiffUsec: from interval; sec*1e6+usec with overflow checks => result=U32::max + OP_OVERFLOW (max measurable ~71 min). operator==: interval computes to exactly 0 sec 0 usec. Posix: CLOCK_REALTIME timespec; interval usec = nsec_diff/1000; errno EINVAL=>INVALID_PARAMS. IntervalTimer: two RawTime members; start()/stop() call now(); getDiffUsec returns U32::max on overflow; getTimeInterval returns Status.

### Cpu / Memory / Generic status

Files: `Os/Cpu.hpp`, `Os/Memory.hpp`, `Os/Os.hpp`, `Os/Os.cpp`

Shared Os::Generic::Status{OP_OK=0, ERROR=1} and Os::Generic::UsedTotal{FwSizeType used; FwSizeType total}. CpuInterface: _getCount(FwSizeType&), _getTicks(Ticks&, cpu_index) — cumulative ticks; caller does sample-to-sample differencing. MemoryInterface: _getUsage(Usage&). Both singletons with static wrappers. Os::init() initializes Console, FileSystem, Cpu, Memory, Task singletons (Queue/Mutex/etc. are per-instance).

### ValidateFile / ValidatedFile / SandboxedFile / FilePathUtils

Files: `Os/ValidateFile.hpp`, `Os/ValidatedFile.hpp`, `Os/SandboxedFile.hpp`, `Os/FilePathUtils.hpp`

ValidateFile::Status: VALIDATION_OK, VALIDATION_FAIL, FILE_DOESNT_EXIST, FILE_NO_PERMISSION, FILE_BAD_SIZE, VALIDATION_FILE_DOESNT_EXIST, VALIDATION_FILE_NO_PERMISSION, VALIDATION_FILE_BAD_SIZE, NO_SPACE, OTHER_ERROR. Free functions validate(file, hashFile[, &hashBuffer]) and createValidation(...); VFILE_HASH_CHUNK_SIZE=256. ValidatedFile wraps fileName + derived hashFileName + Utils::HashBuffer. SandboxedFile: Os::File-shaped wrapper; configure(allowedDirectory) (default sandbox '/'); open validates textual path containment (no realpath/symlink following) and returns File::Status::OUTSIDE_SANDBOX on escape. FilePathUtils::Status: VALID, OUTSIDE_SANDBOX, INVALID_PATH, TOO_LONG; purely textual '.'/'..' resolution against a base dir; MAX_PATH_LENGTH=FileNameStringSize.

### Alternative queue delegates (selectable)

Files: `Os/Generic/LocklessPriorityQueue.hpp`, `Os/Generic/PriorityMemQueue.hpp`, `Os/Generic/docs/sdd-lockless-queue.md`

LocklessPriorityQueue: fixed slot pool; each slot has one atomic packed word: low 2 bits state (FREE=0, WRITING=1, READY=2, READING=3), remaining bits ABA epoch tag incremented per transition; producer FREE->WRITING->READY, consumer READY->READING->FREE; atomic m_sequence U32 FIFO tiebreak (modular compare); consumer scans for best (priority, sequence); slots alignas(LOCKLESS_QUEUE_SLOT_ALIGNMENT=64); config LocklessStateTagType/backoff/retry passes in config/LocklessQueueCfg.hpp; ISR-safe; a 2^TAG_BITS-stale CAS can only cause out-of-priority-order dequeue, never corruption. PriorityMemQueue: MAX_PRIORITIES=32 (priority bitmask in atomic U32), sparse I8 priorityMap[32] (-1 unused), per-priority Types::AtomicQueue, Os::CountingSemaphore signals messages available, per-priority high-water marks; ISR+SMP safe.

## Wire formats

### RawTime serialization (Posix)

Exactly 2 fields, 8 bytes total (must fit FW_RAW_TIME_SERIALIZATION_MAX_SIZE=8): U32 seconds then U32 nanoseconds, each serialized via Fw::SerialBufferBase with Endianness mode defaulting to BIG (big-endian). truncated casts from timespec tv_sec/tv_nsec. Defined in PosixRawTime::serializeTo/deserializeFrom, /home/user/fprime/Os/Posix/RawTime.cpp lines 47-73. Other platforms may use the full FW_RAW_TIME_SERIALIZATION_MAX_SIZE budget.

### Os::File CRC32

calculateCrc/incrementalCrc/finalizeCrc (Os/File.cpp lines 237-281): feed file bytes in FW_FILE_CHUNK_SIZE=512 chunks (NO_WAIT reads) to Utils::Hash (CRC32, INITIAL_CRC=0xFFFFFFFF constant in Os/File.hpp line 587), then final value is bitwise-complemented once more: crc = ~hash.finalize() — i.e. the historical F Prime file CRC OMITS the standard final 1's complement. On error crc out-param = 0. finalizeCrc re-inits the hash.

### Queue message storage (in-memory, not wire)

Messages are opaque byte blobs memcpy'd into per-slot slabs of exactly messageSize bytes at offset maxSize*index (PriorityQueue.cpp store_data/load_data); per-message metadata kept out-of-band (sizes[index]: FwSizeType; priority: FwQueuePriorityType inside MaxHeap Node {FwQueuePriorityType value; FwSizeType order; FwSizeType id} — native layout, never serialized).

## Threading / concurrency

Front classes are not internally synchronized except: Os::Task guards m_state/m_name/m_priority with a per-instance Os::Mutex (m_lock) and class-wide counters with s_taskMutex; Os::Queue guards s_queueCount/registry with a function-local static Mutex. Generic::PriorityQueue is the core concurrency object: one PTHREAD_PRIO_INHERIT + ERRORCHECK mutex, two condition variables (m_full for senders waiting on space, m_empty for receivers waiting on messages); wait loops re-check predicates; notify is issued AFTER unlocking (single notify(), not notifyAll()). getMessagesAvailable is a racy unlocked read. Task lifecycle: STARTING is set before the delegate spawns the pthread; the spawned thread's wrapper transitions STARTING->RUNNING, runs onStart() then the user routine; join() is only legal from STARTING/RUNNING. Posix threads use SCHED_RR with explicit sched, with an automatic silent-degrade retry to default priority/affinity on EPERM. suspend/resume are unimplemented (assert) on Posix. Condition variables record their first mutex and reject any other forever. Singletons are C++11 function-local statics (thread-safe init); Task::start proactively calls Task::init() to avoid concurrent first-init races. Lockless/PriorityMem queues (optional delegates) use std::atomic state machines and are ISR-safe; the default PriorityQueue explicitly is not.

## Porting notes

1) Collapse the delegate/placement-new machinery: in Rust choose the platform implementation with cfg-selected type aliases (e.g. `pub type Task = posix::PosixTask;`) or a sealed trait behind a newtype; the FW_*_HANDLE_MAX_SIZE budgets and the address-equality FW_ASSERTs are C++ artifacts and need no direct port, but keep the "one implementation chosen at build time, front type is concrete" property so no allocation is required. 2) Reproduce every Status enum verbatim with explicit discriminants (they are mirrored in FPP shadow enums serialized into telemetry — e.g. Os/Models/*.fpp, QueueStatus/TaskStatus as U8). 3) Queue MUST be custom: std::sync::mpsc has no priorities. Faithful port = Mutex<QueueState> + two Condvars, where QueueState holds a stable max-heap. `BinaryHeap<(priority, Reverse(order), slot_index)>` with a monotonically increasing u64 order counter reproduces MaxHeap's exact pop order (max priority first, FIFO within priority); keep the fixed slot-slab + free-index-ring design if you want allocation-free steady state. Preserve: SIZE_MISMATCH pre-checks in the wrapper (receive requires capacity >= configured messageSize), FULL/EMPTY for non-blocking, notify-after-unlock, high-water-mark under lock. 4) Task: std::thread cannot express SCHED_RR priority, affinity, or stack rounding — use libc pthread_attr_* (or the `thread-priority` approach) for parity, including the EPERM fallback retry and the STARTING->RUNNING wrapper handshake and join-state machine. 5) Mutex: std::sync::Mutex lacks priority inheritance and errorcheck semantics; for flight parity wrap pthread_mutex with PTHREAD_PRIO_INHERIT+ERRORCHECK; map take/release statuses exactly; lock()/unlock() panic (assert) on failure. Condvar: std::sync::Condvar works but implement the sticky same-mutex check at the wrapper level and keep pend returning Status. 6) File: std::fs::OpenOptions + custom_flags(O_SYNC) covers modes; note OPEN_CREATE defaults to O_EXCL (NO_OVERWRITE) => FILE_EXISTS; map io::Error raw_os_error through the exact errno tables in Os/Posix/error.cpp; reproduce read NO_WAIT single-attempt vs write always-complete (+fsync on WAIT), bounded 2*size retry loops, EINTR retries (std handles EINTR for read/write but not the wait semantics), readline seek-back contract, seek_absolute 3-seek trick (or simply support u64 seeks natively — but keep API shape), and the ~crc quirk. 7) RawTime: keep the 8-byte U32+U32 big-endian serialization and absolute-difference getTimeInterval; use clock_gettime(CLOCK_REALTIME) not Instant if cross-process comparability matters (Instant is CLOCK_MONOTONIC and non-serializable). 8) Keep FwSizeType=u64, FwQueuePriorityType=u8, FwTaskPriorityType=u8 as config-selected type aliases. 9) FileSystem/Directory/Console/Cpu/Memory map cleanly to std/libc; keep the composite algorithms (moveFile EXDEV fallback, copy chunking with 512-byte buffer, touch, createDirectory via Directory open modes) byte-for-byte in behavior since components (FileManager, FileDownlink, PrmDb) depend on the exact status outcomes. 10) Stub/ and Stub/test implementations exist for every interface and are the pattern for a test double layer; replicate as mock trait impls.

## Gotchas

- Os::Queue::receive returns SIZE_MISMATCH unless capacity >= the queue's CONFIGURED messageSize (not the actual message's size); inside Generic::PriorityQueue::receive, actualSize<=capacity is an FW_ASSERT (crash), not an error return.
- MaxHeap push sift-up uses strict '>' (breaks on value <= parent), and pop tie-break picks the OLDER node via age = m_order - node.order computed in wrapping unsigned arithmetic — this is what guarantees FIFO within equal priority; a naive BinaryHeap without an order tiebreaker breaks message ordering for same-priority ports.
- PriorityQueue::getMessagesAvailable() reads the heap size WITHOUT the lock (racy by design); getMessageHighWaterMark() takes the lock via const_cast. Notifications (m_empty/m_full.notify) happen after the ScopeLock is released, and only notify one waiter.
- Os::File::finalizeCrc returns ~(CRC32) — it deliberately undoes the hash's final complement for backward compatibility; matching standard crc32 output will produce wrong file validation results.
- OPEN_CREATE + default NO_OVERWRITE maps to O_EXCL: creating over an existing file returns FILE_EXISTS; opening an already-open Os::File returns INVALID_MODE (not an assert).
- PosixFile::read honors NO_WAIT (returns after first successful partial read) but PosixFile::write ignores wait for looping — it always writes to completion; WAIT only adds fsync. Both loops are bounded to 2*size iterations.
- Task state is set to STARTING before the delegate spawns the thread, so the routine can run (and even finish) before Task::start() returns; the wrapper asserts state != NOT_STARTED and only transitions STARTING->RUNNING once. join() from any state other than STARTING/RUNNING returns INVALID_STATE and does not touch the delegate.
- PosixTask silently retries without priority/affinity on EPERM (one global log notice); a port using pure std::thread would silently lose SCHED_RR/priority entirely — parity requires the two-phase attempt. suspend/resume on Posix are FW_ASSERT(false).
- Task::_delay tv_nsec = getUSeconds()*1000 and EINTR resumes with the remaining interval; TimeInterval is (seconds, MICROseconds) while timespec is nanoseconds — easy unit slip.
- PosixMutex is ERRORCHECK + PRIO_INHERIT; Mutex::lock()/unLock() assert OP_OK, so recursive locking or unlocking a non-held mutex is a crash, not UB. Rust Mutex poisoning semantics differ — must not surface as a different status.
- ConditionVariable permanently binds to the first mutex passed to pend(); passing a different mutex later returns ERROR_DIFFERENT_MUTEX forever (the binding never resets, even after all waiters leave).
- RawTime::getTimeInterval is commutative (absolute difference — it swaps operands so t1>=t2); there is no sign. getDiffUsec saturates: result=U32::max with OP_OVERFLOW beyond ~71.6 minutes; IntervalTimer::getDiffUsec silently returns U32::max on overflow.
- Os::RawTime is not a class but a config-selected alias; Os/RawTime.hpp has load-bearing include order (config alias -> interface -> OS_RAW_TIME_HEADER) and a #error if OS_RAW_TIME_HEADER is unset.
- FileSystem::exists() treats ANY _getPathType error (including permission errors) as NOT_EXIST; moveFile only falls back to copy+delete on EXDEV_ERROR specifically — other rename failures propagate; handleFileError collapses most File statuses to OTHER_ERROR (only NO_SPACE/NO_PERMISSION/DOESNT_EXIST survive).
- Directory::read silently truncates long filenames (string_copy) and returns OP_OK; mkdir uses S_IRWXU (0700) while File create uses 0666; CREATE_IF_MISSING tolerates EEXIST but any other mkdir error aborts before opendir.
- Queue::create FW_ASSERTs depth>0 and messageSize>0 (crash, not status); double-create is detected in the WRAPPER (ALREADY_CREATED) by m_depth/m_size, while Generic::PriorityQueue::create asserts its pointers are null — calling the delegate twice directly is a crash.
- File copy semantics: Os::File copy ctor/assignment duplicate the underlying fd (F_DUPFD) sharing file offset? No — F_DUPFD shares the open file description, so offsets ARE shared between copies; operator= closes its currently open file first to avoid orphaning the handle.
- Console registers itself as the global Fw::Logger on first getSingleton(); porting order matters or early log output is dropped.
- Default queue-priority type is U8 (0-255) but PriorityMemQueue only supports priorities 0-31 (asserts) — the delegates are not drop-in interchangeable for all priority values.

