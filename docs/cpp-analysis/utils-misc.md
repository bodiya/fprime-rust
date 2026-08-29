# Support libraries and persistence formats (Utils, Fw/DataStructures, Fw/Dp, Fw/FilePacket, CFDP, Svc/BufferManager, Svc/PrmDb, Svc/ComLogger, Fw/SerializableFile, STest)

> Analysis of the C++ F Prime implementation (github.com/nasa/fprime) produced to guide this Rust port.
> Source tree analyzed: local clone at commit of 2026-08. File paths refer to that C++ tree.

## Overview

This subsystem covers F Prime's support libraries plus the persistence/wire pieces a Rust port must reproduce byte-for-byte. Utils/ provides the framework-wide hash abstraction (compile-time-pluggable, CRC32 by default) used by data products, file integrity sidecars, ComLogger, and PrmDb; CRCChecker (file checksum sidecars); RateLimiter and TokenBucket (EVR/action throttling); and Types/ (CircularBuffer byte ring, fixed-message Queue built on it, lock-free SpscQueue). Fw/DataStructures supplies bounded, allocation-free generic containers (arrays, array/red-black maps and sets, FIFO queues, stacks) in internal-storage and external-storage variants; PrmDb's database is an ArrayMap of ParamBuffers. Fw/Dp defines DpContainer, the in-memory view over a data-product packet buffer with a 57-byte header (default config), CRC32 header hash, variable data, and CRC32 data hash — this is a ground-visible wire/file format. Fw/FilePacket defines the CFDP-inspired uplink/downlink file transfer packets (START/DATA/END/CANCEL) with the CFDP 32-bit offset-weighted modular checksum. Svc/BufferManager is the passive pooled-buffer allocator (up to 10 bins of N buffers x M bytes, first-fit, ownership tracked via the Fw::Buffer context word). Svc/PrmDb is the active parameter database component with a double-buffered (active/staging) store, a three-state file-load state machine, and an on-disk format of a leading U32 CRC followed by 0xA5-delimited records. Fw::SerializableFile persists any Serializable to a file; STest is the host-test rule/scenario framework. Byte-exact reproduction is required for: the PrmDb file, DpContainer packets, FilePacket family, CRCChecker/ComLogger sidecar and log files, and the CRC32 variants (complemented vs raw-register) each uses.

## Key items

### Utils::Hash (CRC32 implementation) + HashBuffer

Files: `Utils/Hash/Hash.hpp`, `Utils/Hash/Crc32/HashImpl.cpp`, `Utils/Hash/Crc32/Crc32.cpp`, `Utils/Hash/Crc32/Crc32.hpp`, `Utils/Hash/HashBuffer.hpp`, `Utils/Hash/HashBufferCommon.cpp`, `Utils/Hash/HashCommon.cpp`, `Utils/Hash/HashConfig.hpp`

Pluggable hash selected by HashConfig.hpp include; default = CRC32 (IEEE 802.3, reversed poly 0xEDB88320, table-driven Sarwate). HASH_DIGEST_LENGTH=4, HASH_HANDLE_TYPE=U32, HASH_EXTENSION_STRING=".CRC32". init() sets handle=0xFFFFFFFF; update() runs the reflected table update WITHOUT final complement; finalize() returns ~handle (standard CRC-32 value), serialized big-endian into HashBuffer for the buffer overload; setHashValue(v) stores ~v. Static hash() = init+update+finalize. HashBuffer: fixed 4-byte LinearBufferBase; operator== compares size then memcmp; asBigEndianU32() folds bytes 0..3 MSB-first. Utils::Hash::hash on CRC32 of empty input yields 0x00000000... (standard).

### Utils::CRCChecker

Files: `Utils/CRCChecker.hpp`, `Utils/CRCChecker.cpp`, `default/config/CRCCheckerConfig.hpp`

Free functions over Os::File. crc_stat_t enum: PASSED_FILE_CRC_CHECK=0, PASSED_FILE_CRC_WRITE=1, FAILED_FILE_SIZE=2, FAILED_FILE_SIZE_CAST=3, FAILED_FILE_OPEN=4, FAILED_FILE_READ=5, FAILED_FILE_CRC_OPEN=6, FAILED_FILE_CRC_READ=7, FAILED_FILE_CRC_WRITE=8, FAILED_FILE_CRC_CHECK=9. Reads file in CONFIG_CRC_FILE_READ_BLOCK=2048-byte blocks, Hash::finalize (standard complemented CRC-32), writes 4 raw native-endian bytes to '<fname>.CRC32'. verify_checksum recomputes and compares against sidecar; outputs expected (from file) and actual in both pass and fail cases.

### Utils::RateLimiter

Files: `Utils/RateLimiter.hpp`, `Utils/RateLimiter.cpp`

State: m_counterCycle, m_timeCycle (U32 params), m_counter (U32), m_time (Fw::Time), m_timeAtNegativeInfinity (bool). reset() zeroes counter, sets time to Fw::Time() with negative-infinity flag true. trigger(time): if both cycles 0 -> true; else OR of counter-criterion (counter==0) and time-criterion (time >= m_time + Fw::Time(m_timeCycle,0) || negInf); then updates each enabled dimension: counter -> 1-then-wrap on trigger else increment-then-wrap; time -> set m_time=time on trigger, always clear negInf flag. trigger() (no arg) asserts m_timeCycle==0 and calls trigger(Time::zero()). Not thread safe.

### Utils::TokenBucket

Files: `Utils/TokenBucket.hpp`, `Utils/TokenBucket.cpp`

Params: replenishInterval (microseconds, converted to Fw::Time(interval/1e6, interval%1e6)), maxTokens, replenishRate (tokens per interval), startTokens, startTime. 2-arg ctor: rate=1, tokens=maxTokens, time=(0,0), asserts maxTokens<=1000 (MAX_TOKEN_BUCKET_TOKENS). trigger(now): while tokens<max && m_time+interval <= now: tokens += min(rate, max-tokens); m_time += interval; then if tokens>=max && m_time<now: m_time=now; finally consume 1 token if available (return true) else false. replenish() sets tokens=max if below. Not thread safe.

### Types::CircularBuffer

Files: `Utils/Types/CircularBuffer.hpp`, `Utils/Types/CircularBuffer.cpp`, `default/config/CircularBufferCfg.hpp`

Byte ring over external store; state = m_store, m_store_size, m_head_idx, m_allocated_size, m_high_water_mark. setup() once only (asserts if re-setup). serialize(U8*,size): FW_SERIALIZE_NO_ROOM_LEFT if size>free; byte-copy at head+allocated (mod store_size); updates high-water. Serializable/LinearBufferBase overloads write into a fixed-size slot; if the slot wraps the store end it is staged through a stack buffer of CircularBufferCfg::STAGING_BUFFER_SIZE (larger wrapping slots ASSERT). peek overloads: U8/char at offset; U32 assembled big-endian from 4 bytes; U8* range copy; object peek mirrors serialize staging. rotate(n) advances head (frees from front), trim(n) reduces allocated (frees from back); both FW_DESERIALIZE_BUFFER_EMPTY if n>allocated. get_capacity()==store size (no lost byte). Not thread safe.

### Types::Queue and Types::SpscQueue

Files: `Utils/Types/Queue.hpp`, `Utils/Types/Queue.cpp`, `Utils/Types/SpscQueue.hpp`

Queue: fixed-size-message FIFO/LIFO over CircularBuffer. Enums: QueueMode {QUEUE_FIFO=0, QUEUE_LIFO=1}, QueueOverflowMode {QUEUE_DROP_NEWEST=0, QUEUE_DROP_OLDEST=1}. setup asserts storage_size >= depth*message_size and passes depth*message_size (not storage_size) to the ring. enqueue asserts size==message_size; on full: DROP_NEWEST -> FW_SERIALIZE_NO_ROOM_LEFT; DROP_OLDEST -> rotate one message then enqueue, returning FW_SERIALIZE_DISCARDED_EXISTING. dequeue: FIFO = peek(0)+rotate; LIFO = peek(allocated-msgSize)+trim. popFront always front. getQueueSize/high_water in message units. Not thread safe. SpscQueue<E,CAPACITY>: wait-free single-producer/single-consumer; fixed array E[CAPACITY]; two std::atomic<FwSizeType> indices modulo 2*CAPACITY; produce/consume return bool (false = full/empty); peek reads without advancing; ctor asserts atomics are lock-free; static_assert CAPACITY*2 fits index type.

### Fw::DataStructures container inventory

Files: `Fw/DataStructures/ArrayMap.hpp`, `Fw/DataStructures/ArraySetOrMapImpl.hpp`, `Fw/DataStructures/FifoQueue.hpp`, `Fw/DataStructures/RedBlackTreeMap.hpp`, `Fw/DataStructures/CircularIndex.hpp`

Header-only generic containers, each in internal-storage (template capacity C) and External* (caller-supplied storage) variants: Array/ExternalArray, ArrayMap/ArraySet (unsorted linear-scan array; insert overwrites existing key else appends if size<capacity, returns Fw::Success FAILURE when full; find/remove linear), ExternalRedBlackTreeMap/Set (+RedBlackTreeMap/Set wrappers), FifoQueue/ExternalFifoQueue, Stack/ExternalStack, CircularIndex (value modulo modulus helper), SizedContainer, Nil (unit type for sets), Map/Set const iterators. All bounded-capacity, no heap allocation, Fw::Success {SUCCESS=0, FAILURE=1} returns. Rust: map to fixed-capacity generic collections (heapless-style) with the same overwrite-on-insert semantics; PrmDb depends on ArrayMap preserving insertion order for file save iteration.

### Fw::DpContainer (data product packet)

Files: `Fw/Dp/DpContainer.hpp`, `Fw/Dp/DpContainer.cpp`, `Fw/Dp/Dp.fpp`, `default/config/DpCfg.fpp`, `default/config/DpCfg.hpp`

Wraps an Fw::Buffer holding [header | headerHash | data | dataHash]. Constants (with default config): Header::SIZE=57, HEADER_HASH_OFFSET=57, DATA_OFFSET=61, MIN_PACKET_SIZE=65, packetSize=57+dataSize+8, dataHashOffset=61+dataSize. DpState enum (U8): UNTRANSMITTED=0, PARTIAL=1, TRANSMITTED=2, default UNTRANSMITTED. DpCfg::ProcType bitmask enum (U8): PROC_TYPE_NONE=0x00, PROC_TYPE_ZLIB_DEFLATE=0x01, PROC_TYPE_ONE=0x02, PROC_TYPE_TWO=0x04; procTypes field stores the SerialType (U8) mask. serializeHeader() writes all fields (asserting OK) then updateHeaderHash() = CRC32 over bytes [0,57). deserializeHeader() validates descriptor==FW_PACKET_DP (0x0005) -> else FW_SERIALIZE_FORMAT_ERROR; user data read with OMIT_LENGTH and exact-size check (FW_DESERIALIZE_SIZE_MISMATCH). checkHeaderHash/checkDataHash compare stored vs computed (Fw::Success). setBuffer asserts bufferSize>=65, points data sub-buffer at offset 61 with capacity bufferSize-65, resets dataSize=0. shrinkBufferSize() shrinks Fw::Buffer to packetSize (grow asserts). invalidateBuffer clears everything. Filename convention (DpCfg.hpp): "%s/Dp_%08(id)_%08(secs)_%08(usecs).fdp". Ports (Dp.fpp): DpGet (sync, returns Fw.Success), DpRequest, DpResponse, DpSend.

### Fw::FilePacket + CFDP::Checksum

Files: `Fw/FilePacket/FilePacket.hpp`, `Fw/FilePacket/FilePacket.cpp`, `Fw/FilePacket/Header.cpp`, `Fw/FilePacket/PathName.cpp`, `Fw/FilePacket/StartPacket.cpp`, `Fw/FilePacket/DataPacket.cpp`, `Fw/FilePacket/EndPacket.cpp`, `CFDP/Checksum/Checksum.cpp`

C++ models this as a union tagged by Header.m_type: T_START=0, T_DATA=1, T_END=2, T_CANCEL=3, T_NONE=255 (Rust: enum with variants). fromBuffer deserializes header then dispatches by type; Start/End/Cancel demand exact buffer consumption, Data demands remaining==dataSize and keeps a zero-copy pointer to the payload. PathName: max 255 bytes, initialize() measures strlen capped at 255. CFDP::Checksum: U32 modular sum; update(data, fileOffset, length) handles unaligned prefix/suffix relative to the file offset; addByteAtOffset: value += byte << (8*(3-(offset%4))). EndPacket carries checksum.getValue(). These packets ride inside com packets with descriptor FW_PACKET_FILE=0x0003 (added by FileUplink/FileDownlink, not by FilePacket).

### Svc::BufferManager

Files: `Svc/BufferManager/BufferManagerComponentImpl.hpp`, `Svc/BufferManager/BufferManagerComponentImpl.cpp`, `Svc/BufferManager/BufferManager.fpp`, `default/config/BufferManagerComponentImplCfg.hpp`

Passive component, all three input ports guarded (one component mutex): bufferGetCallee(size)->Fw::Buffer, bufferSendIn(Fw::Buffer), schedIn (telemetry: HiBuffs, CurrBuffs, TotalBuffs, NoBuffs, EmptyBuffs). Config: BUFFERMGR_MAX_NUM_BINS=10 (U16). setup(mgrId:U16, memId, allocator&, bins): BufferBin{bufferSize, numBuffers:U16}, bins expected ascending by size, unused bins numBuffers=0; one allocation sized sum(numBuffers*(bufferSize+sizeof(AllocatedBuffer))); AllocatedBuffer structs laid at start, raw buffer memory after; total structs <= U16::MAX (asserted, must fit low half of context). Each Fw::Buffer built with context=(mgrId<<16)|index. Get: linear scan, first free slot with slotSize>=request; marks allocated, bumps currBuffs/highWater, returns copy with setSize(request); none -> WARNING_HI NoBuffsAvailable(size), noBuffs++, returns default (invalid) Fw::Buffer. Return: null-and-empty buffer -> WARNING_HI NullEmptyBuffer, emptyBuffs++, done; else decode context, ASSERT id<numStructs, mgrId matches, slot allocated, data ptr in [memory, memory+size), size<=original; free slot, currBuffs--. cleanup() destructs Fw::Buffer instances and deallocates; called by destructor if needed; allocator must outlive it.

### Svc::PrmDb

Files: `Svc/PrmDb/PrmDbImpl.cpp`, `Svc/PrmDb/PrmDbImpl.hpp`, `Svc/PrmDb/PrmDb.fpp`, `Svc/PrmDb/PrmDbCmdDict.fppi`, `default/config/PrmDbImplCfg.hpp`

Active component; getPrm guarded, setPrm/pingIn/commands async; explicit lock()/unLock() protects the DBs between guarded and queued contexts. Double-buffered stores: two Fw::ArrayMap<FwPrmIdType, Fw::ParamBuffer, PRMDB_NUM_DB_ENTRIES=25> (active + staging), swapped by pointer under lock on PRM_COMMIT_STAGED. State machine PrmDbFileLoadState (U8): IDLE=0, LOADING_FILE_UPDATES=1, FILE_UPDATES_STAGED=2. Commands: PRM_SAVE_FILE opcode 0x00 (rejected BUSY unless IDLE); PRM_LOAD_FILE 0x01 (fileName, Merge:U8 {MERGE=0 copies active->staging first, RESET=1 clears staging}; loads file into staging, success -> OK + FILE_UPDATES_STAGED, failure -> PrmDbFileLoadFailed EVR + clear staging + IDLE + EXECUTION_ERROR); PRM_COMMIT_STAGED 0x02 (VALIDATION_ERROR unless FILE_UPDATES_STAGED; swaps DBs, clears new staging, IDLE). setPrm during non-IDLE is rejected with PrmDbFileLoadInvalidAction EVR. getPrm returns Fw::ParamValid INVALID + PrmIdNotFound EVR when missing. Commanded (staging) loads may be sandboxed via Os::SandboxedFile if configureLoadSandbox set. updateAddPrm returns PARAM_UPDATED/PARAM_ADDED/NO_SLOTS (NO_SLOTS -> PrmDbFull EVR; during load, any drop fails the load). FW_PARAM_BUFFER_MAX_SIZE = 512 - sizeof(FwPrmIdType=4) - sizeof(FwPacketDescriptorType=2) = 506.

### Svc::ComLogger (brief)

Files: `Svc/ComLogger/ComLogger.cpp`, `Svc/ComLogger/ComLogger.hpp`

Logs Fw::ComBuffers to rotating files named '<prefix>_<timeBase>_<seconds>_<useconds 06d>.com'; rotates when projected bytes exceed maxFileSize; on close writes a Utils::Hash sidecar '<file>.CRC32'. Record = optional big-endian U16 length (size truncated to & 0xFFFF) followed by raw com buffer bytes when storeBufferLength=true (default), else raw bytes only. Open errors are throttled (one EVR until success).

### Fw::SerializableFile (brief)

Files: `Fw/SerializableFile/SerializableFile.hpp`, `Fw/SerializableFile/SerializableFile.cpp`

Utility to persist any Fw::Serializable: ctor allocates maxSerializedSize via MemAllocator (must not return smaller); save() serializes to the buffer then writes buffer contents to file; load() reads file into buffer and deserializes. Status enum: OP_OK=0, FILE_OPEN_ERROR=1, FILE_WRITE_ERROR=2, FILE_READ_ERROR=3, DESERIALIZATION_ERROR=4. File contents are exactly the F Prime big-endian serialization of the object — byte format is type-specific.

### STest (brief)

Files: `STest/STest/Rule/Rule.hpp`, `STest/STest/Scenario/`, `STest/STest/Random/Random.cpp`, `STest/STest/Random/bsd_random.c`

Test-only rule-based scenario testing framework: Rule<State> (precondition/action), Scenario combinators (Bounded, Random, Interleaved, Sequence, Selected, Repeated, etc.), and a seeded PRNG wrapping a vendored BSD random() for reproducible runs (seed logged/replayable). Not flight code; a Rust port can replace it with proptest-style tooling but keeping a Rule/Scenario layer preserves existing component test designs.

## Wire formats

### DpContainer packet (data product; also the .fdp file body written by DpWriter)

All multi-byte fields big-endian (F Prime serialization). Offsets with DEFAULT config (default/config/*): [0..2) FwPacketDescriptorType U16 = 0x0005 (ComCfg Apid FW_PACKET_DP); [2..6) container id FwDpIdType U32; [6..10) priority FwDpPriorityType U32; [10..21) Fw::Time = TimeValue struct {timeBase U16, timeContext U8, seconds U32, useconds U32} (11 bytes, field order per Fw/Time/Time.fpp); [21..22) procTypes U8 bitmask (DpCfg::ProcType: NONE=0x00, ZLIB_DEFLATE=0x01, ONE=0x02, TWO=0x04); [22..54) userData 32 raw bytes (DpCfg::CONTAINER_USER_DATA_SIZE, no length token, zero-initialized); [54..55) DpState U8 (UNTRANSMITTED=0, PARTIAL=1, TRANSMITTED=2); [55..57) dataSize as FwSizeStoreType U16; Header::SIZE=57. [57..61) header hash = standard CRC-32 (complemented) over bytes [0..57), big-endian U32. [61..61+dataSize) data. [61+dataSize..65+dataSize) data hash = CRC-32 over data bytes, big-endian U32. MIN_PACKET_SIZE=65; packetSize = 57 + dataSize + 8. Defined by Fw/Dp/DpContainer.hpp Header offsets and DpContainer.cpp serializeHeader/deserializeHeader.

### PrmDb parameter file

[0..4) U32 big-endian CRC = un-complemented CRC-32 register (init 0xFFFFFFFF, reflected poly 0xEDB88320, NO final XOR; equals !standard_crc32) over ALL bytes from offset 4 to EOF. Then up to PRMDB_NUM_DB_ENTRIES (25) records, each: 1 byte delimiter 0xA5 (PRMDB_ENTRY_DELIMITER); U32 BE recordSize = sizeof(FwPrmIdType)(4) + valueLen, validated 4 <= recordSize <= FW_PARAM_BUFFER_MAX_SIZE+4 (=510 default); FwPrmIdType U32 BE parameter id; valueLen = recordSize-4 bytes of the parameter's F Prime-serialized value (opaque here; type-specific). Records written in ArrayMap iteration (insertion) order. Save path: placeholder CRC 0xFFFFFFFF written first, real CRC seek-back-written at offset 0 last. Defined in Svc/PrmDb/PrmDbImpl.cpp PRM_SAVE_FILE_cmdHandler/readParamFileWork + default/config/PrmDbImplCfg.hpp.

### FilePacket family (file uplink/downlink, inside FW_PACKET_FILE=0x0003 com packets)

All big-endian. Common header (5 bytes): U8 type {T_START=0, T_DATA=1, T_END=2, T_CANCEL=3; T_NONE=255 in-memory only} + U32 sequenceIndex. START (seq always 0): header + U32 fileSize + sourcePath + destPath, where PathName = U8 length (max 255) + length raw bytes, NO NUL; deserialization must consume buffer exactly. DATA: header + U32 byteOffset + U16 dataSize + dataSize raw file bytes (fixed part = 11 bytes); remaining bytes must equal dataSize exactly. END: header + U32 checksumValue (CFDP checksum of the whole file); exact-size. CANCEL: header only. CFDP checksum: U32 wrapping sum where the byte at file offset o contributes byte << (8*(3-(o%4))) — i.e., the file is summed as big-endian U32 words aligned to file offsets, short first/last words padded at their true offsets. Defined in Fw/FilePacket/*.cpp and CFDP/Checksum/Checksum.cpp.

### CRCChecker sidecar file (<name>.CRC32)

Exactly 4 bytes: the standard complemented CRC-32 of the whole target file (Utils::Hash finalize) written via raw memcpy of the U32 — NATIVE machine endianness (little-endian on typical targets), not the F Prime big-endian serializer. File read in 2048-byte blocks (CONFIG_CRC_FILE_READ_BLOCK). Defined in Utils/CRCChecker.cpp create_checksum_file/read_crc32_from_file.

### ComLogger log file (<prefix>_<timeBase>_<secs>_<06usecs>.com)

Sequence of records: when storeBufferLength=true (default), each record = U16 big-endian length (comBuffer size truncated with & 0xFFFF) followed by that many raw com-buffer bytes; when false, raw com-buffer bytes only. File rotated when projected size exceeds maxFileSize; on close a '<file>.CRC32' sidecar is written via Utils::Hash (complemented CRC-32, native-endian 4 bytes, same as CRCChecker). Defined in Svc/ComLogger/ComLogger.cpp writeComBufferToFile/openFile.

## Threading / concurrency

Utils/Hash, CRCChecker, RateLimiter, TokenBucket, CircularBuffer, Types::Queue, DataStructures containers, DpContainer, FilePacket, SerializableFile: NOT thread safe; callers wrap in concurrency constructs (Types::Queue explicitly documents this). Types::SpscQueue is the one lock-free primitive: wait-free ISR-safe single-producer/single-consumer, std::atomic (seq_cst) indices, corruption if the SPSC contract is violated; isEmpty()/isFull() are only authoritative from the consumer/producer side respectively. Svc::BufferManager is a passive component whose three input ports (bufferGetCallee, bufferSendIn, schedIn) are all 'guarded' — serialized by the single component mutex; allocation state needs no further protection. Svc::PrmDb is an active (queued, own thread) component: getPrm is guarded (called synchronously from any thread under the component mutex) while setPrm/pingIn and all three commands are async (dispatched on the component thread); the implementation additionally brackets every DB read/mutation and the active/staging pointer swap with this->lock()/unLock() so the guarded getPrm never races the queued handlers; readParamFile() (boot-time load) runs pre-threading at initialization. ComLogger is queued/active with file I/O on its own thread.

## Porting notes

1) Make the hash a compile-time strategy (trait + type alias, digest length as associated const) exactly mirroring HashConfig.hpp; default CRC32. Implement BOTH finalizations: standard CRC-32 (complemented; used in DpContainer hashes, CRCChecker sidecars, ComLogger sidecars) and the raw register form (~crc32; PrmDb file field and Os::File::calculateCrc). In Rust: crc32 crate value = complemented form; PrmDb stores !value. 2) Keep all serialization big-endian via the Fw serialize layer, EXCEPT CRCChecker's sidecar which is a native-endian raw U32 write — reproduce it literally. 3) Model config typedefs as a config module: FwIdType/FwDpIdType/FwPrmIdType=U32, FwDpPriorityType=U32, FwPacketDescriptorType=U16, FwSizeStoreType=U16, FwTimeBaseStoreType=U16, FwTimeContextStoreType=U8, FwSizeType=u64(unix), DpCfg::CONTAINER_USER_DATA_SIZE=32, PRMDB_NUM_DB_ENTRIES=25, PRMDB_ENTRY_DELIMITER=0xA5, CONFIG_CRC_FILE_READ_BLOCK=2048, BUFFERMGR_MAX_NUM_BINS=10, FW_PARAM_BUFFER_MAX_SIZE=506; derive header offsets from these (const fns) rather than hardcoding 57/61/65 so config overrides keep working. 4) DpContainer: implement as a view over &mut [u8] (like Fw::Buffer) with serialize_header/deserialize_header returning the same SerializeStatus discriminants; keep assert-on-misuse semantics as panics for the write path and status codes for the read path exactly as C++ splits them. 5) FilePacket: replace the C++ union with a Rust enum { Start, Data, End, Cancel }; preserve exact-consumption checks and zero-copy payload/path references (lifetimes over the source buffer). 6) Queue/CircularBuffer: straightforward safe Rust over a borrowed or owned byte slice; preserve status-code returns (NO_ROOM_LEFT, DISCARDED_EXISTING, BUFFER_EMPTY). SpscQueue: use atomics with the modulo-2C index scheme; seq_cst to match, or prove acquire/release equivalence. 7) BufferManager: the context word (mgrId<<16|index) is load-bearing across component boundaries — keep it. Replace placement-new pool with a Vec/Box<[AllocatedBuffer]> plus one raw byte arena from the MemAllocator abstraction; asserts on bad returns should remain hard panics (they indicate ownership corruption). 8) PrmDb: reuse the port's ArrayMap (bounded, insertion-ordered) so save-file record order matches C++; implement the state machine as an enum with the exact rejection responses (BUSY for save/load, VALIDATION_ERROR for commit, EVR PrmDbFileLoadInvalidAction with action enum). 9) STest can be ported thinly or replaced; it is not flight code. 10) Byte-for-byte test vectors to lock in CI: a PrmDb file with 2+ params, a DP packet with nonzero user data, each FilePacket type, a .CRC32 sidecar (note native endianness), and a ComLogger file with length prefixes.

## Gotchas

- CRCChecker writes/reads the .CRC32 sidecar file via raw memcpy of a U32 (reinterpret_cast) — NATIVE endianness (little-endian on Linux), NOT F Prime big-endian serialization. PrmDb's file CRC, by contrast, IS big-endian serialized. Do not 'fix' this inconsistency in a byte-faithful port.
- PrmDb file CRC and Os::File::calculateCrc both store the UN-complemented CRC-32 register: Utils::Hash::finalize returns ~register (standard CRC-32), and both call sites re-complement (crc = ~crc) before storing, yielding init=0xFFFFFFFF, reflected poly 0xEDB88320, NO final XOR. Equivalent to !crc32(data) in Rust terms.
- Utils::Hash::setHashValue(v) stores ~v internally (expects an already-complemented value); hash_handle always holds the raw register.
- DpContainer data-size field is serialized via serializeSize() as FwSizeStoreType = U16 big-endian (2 bytes on wire) even though m_dataSize is FwSizeType (U64 on unix); sizes > 65535 return FW_SERIALIZE_FORMAT_ERROR. The 32-byte user-data field is serialized with OMIT_LENGTH (raw bytes, no length token).
- Fw::Time::deserializeFrom rejects useconds >= 1000000 with FW_DESERIALIZE_FORMAT_ERROR (accepting would assert later); DpContainer::deserializeHeader rejects packet descriptor != 0x0005 with FW_SERIALIZE_FORMAT_ERROR.
- CircularBuffer's file header comment claims one byte of the store is lost to track wrap-around; the current implementation tracks m_allocated_size explicitly, so the FULL store size is usable (get_capacity() == store size). Port the code, not the comment.
- RateLimiter counter semantics: triggers when m_counter == 0; on trigger the counter is set to 1 then wrapped (so counterCycle == 1 triggers every call). Time trigger fires when now >= last+cycle OR time is at 'negative infinity' (initial state flag m_timeAtNegativeInfinity). If both cycles are 0, trigger() always returns true; trigger() with no args asserts if timeCycle != 0.
- TokenBucket: replenish loop advances m_time in whole replenishInterval steps (m_time is the last replenish instant, not 'now'); once full and m_time < now, m_time snaps to now. If time moves backwards, no replenish occurs but a stored token can still be consumed. replenish() (manual) sets tokens to maxTokens, it does not add. MAX_TOKEN_BUCKET_TOKENS=1000 asserted only in the 2-arg constructor.
- Types::Queue::get_high_water_mark() returns messages (bytes / message_size), not bytes. DROP_OLDEST overflow returns FW_SERIALIZE_DISCARDED_EXISTING (a distinct SerializeStatus), not OK.
- SpscQueue distinguishes full from empty by keeping produce/consume indices modulo CAPACITY*2 (element index = idx % CAPACITY); count = (p - c + 2C) % 2C. Uses default (seq_cst) atomics; a Rust port with Acquire/Release must preserve the two-thread contract.
- FilePacket StartPacket/EndPacket/CancelPacket deserialization requires the buffer to be EXACTLY consumed (leftover bytes => FW_DESERIALIZE_SIZE_MISMATCH); DataPacket requires remaining == dataSize. PathName strings on the wire have a U8 length and NO NUL terminator; after deserialize m_value points zero-copy into the source buffer.
- CFDP Checksum word alignment is relative to FILE offset (the offset argument), not buffer address: byte at file offset o contributes byte << (8*(3-(o%4))), summed mod 2^32. Update can be called with arbitrary offset/length chunks and must yield the same sum.
- BufferManager allocation search is a linear first-fit over the flat struct array (not per-bin); it only behaves as best-fit if the user orders bins by ascending bufferSize as documented. Returned Fw::Buffer's context field encodes (mgrId << 16) | structIndex and is the ONLY lookup key on return; returning a buffer twice, with wrong mgrId, larger size, or out-of-range pointer asserts (crashes), it is not an error return.
- PrmDb load reads at most PRMDB_NUM_DB_ENTRIES (25) records even if the file has more (silently ignores the rest); clean EOF is detected as a successful delimiter read of size 0 — a failed read also yielding size 0 must map to error, not EOF. Any dropped record (db full) fails the whole load with ERROR after processing.
- PrmDb save writes a 0xFFFFFFFF placeholder CRC first, streams records while accumulating the CRC, then seeks to offset 0 and overwrites the real CRC; the CRC covers every byte after offset 4 (delimiters + record sizes + ids + values) in file order.
- ComLogger length prefix truncates: size = comBuffer.getSize() & 0xFFFF stored as big-endian U16 when storeBufferLength is true.
- FwSizeType is U64 on unix but many wire/stored size fields are FwSizeStoreType (U16); FwPacketDescriptorType is U16 (dictionary type in ComCfg.fpp), historically U32 in older F Prime — packet layouts depend on these config typedefs, so make them project-configurable in the port.

