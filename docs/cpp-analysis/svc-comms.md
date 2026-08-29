# F Prime comms stack (Svc/FprimeProtocol, FprimeFramer, FprimeDeframer, FrameAccumulator, FprimeRouter, ComQueue, ComStub) + byte-stream drivers (Drv/ByteStreamDriverModel, Drv/Interfaces, Drv/Ip, Drv/TcpClient, Drv/TcpServer) + Utils/Hash CRC32

> Analysis of the C++ F Prime implementation (github.com/nasa/fprime) produced to guide this Rust port.
> Source tree analyzed: local clone at commit of 2026-08. File paths refer to that C++ tree.

## Overview

Downlink path: producers (TlmChan/EventManager → Fw.Com ports; FileDownlink/DpCatalog → Fw.Buffer ports) → ComQueue (active, prioritized queues, one-outstanding-send flow control via Fw.SuccessCondition) → FprimeFramer (wraps payload in F Prime frame: 0xDEADBEEF | len | payload | CRC32) → ComStub (com adapter over a ByteStreamDriver) → Drv::TcpClient/TcpServer/Udp (SocketComponentHelper spawns read + reconnect threads over Drv/Ip sockets). Uplink path: driver recv → ComStub.dataOut → FrameAccumulator (accumulates raw bytes in a Types::CircularBuffer, uses a pluggable FrameDetector to find whole frames) → FprimeDeframer (validates start word, length, CRC; strips 8-byte header and 4-byte trailer; extracts APID from first 2 payload bytes into ComCfg::FrameContext) → FprimeRouter (routes by APID: command → Fw.Com to CmdDispatcher, file → Fw.Buffer to FileUplink, else unknownDataOut). Every hop passes `Svc.ComDataWithContext(ref data: Fw.Buffer, context: ComCfg.FrameContext)` and has a mirror dataReturnOut/dataReturnIn port chain returning buffer ownership upstream to the original allocator. Flow control: Fw.SuccessCondition comStatus travels ComStub → Framer → ComQueue; ComQueue sends exactly one buffer per SUCCESS. All framing integers use F Prime serialization, big-endian by default (Fw/Types/Serializable.cpp LinearBufferBase::serializeFrom, Endianness::BIG). CRC is CRC-32/ISO-HDLC (IEEE 802.3): reflected poly 0xEDB88320, init 0xFFFFFFFF, final one's-complement.

## Key items

### FprimeProtocol frame types

Files: `Svc/FprimeProtocol/FprimeProtocol.fpp`, `Svc/FprimeProtocol/docs/sdd.md`

FPP structs: FrameHeader{startWord: U32 (TokenType), lengthField: U32} with default startWord=0xdeadbeef; FrameTrailer{crcField: U32}. SERIALIZED_SIZE: header=8, trailer=4. lengthField counts payload bytes only. Deframer/detector validate startWord against the FPP default-constructed value, not a separate constant — a Rust port should keep the constant single-sourced. Max payload 2^32-1.

### ComCfg::FrameContext / Apid

Files: `default/config/ComCfg.fpp`

dictionary type FwPacketDescriptorType = U16 (NOT U32 — legacy F Prime used U32). enum Apid: FW_PACKET_COMMAND=0x0000, FW_PACKET_TELEM=0x0001, FW_PACKET_LOG=0x0002, FW_PACKET_FILE=0x0003, FW_PACKET_PACKETIZED_TLM=0x0004, FW_PACKET_DP=0x0005, FW_PACKET_IDLE=0x0006, FW_PACKET_PARAM=0x0007, FW_PACKET_HAND=0x00FE, FW_PACKET_UNKNOWN=0x00FF, SPP_IDLE_PACKET=0x07FF, INVALID_UNINITIALIZED=0x0800 (default). struct FrameContext{comQueueIndex:FwIndexType, apid:Apid, hasSecHdr:bool, sequenceFlags:U8, sequenceCount:U16, vcId:U8, pvn:Pvn, sendNow:bool, saIndex:U16} defaults {0, FW_PACKET_UNKNOWN, false, 0x3, 0, 1, Pvn::INVALID_UNINITIALIZED(0x8), false, 0xFFFF}. Fw::ComPacketType is an alias of ComCfg::Apid::T (Fw/Com/ComPacket.hpp). Port: Svc.ComDataWithContext(ref data: Fw.Buffer, context: ComCfg.FrameContext) in Svc/Ports/CommsPorts/CommsPorts.fpp.

### Interfaces (FPP)

Files: `Svc/Interfaces/Framer.fpp`, `Svc/Interfaces/Deframer.fpp`, `Svc/Interfaces/FrameAccumulator.fpp`, `Svc/Interfaces/Router.fpp`, `Svc/Interfaces/Com.fpp`

Framer: sync dataIn, out dataOut, out dataReturnOut (return incoming buffer to sender), sync dataReturnIn (framed buffer back from com adapter), sync comStatusIn, out comStatusOut. Deframer: guarded dataIn, out dataOut, out dataReturnOut, sync dataReturnIn. FrameAccumulator interface: guarded dataIn + same return pair. Router: guarded dataIn, out dataReturnOut, out fileOut(Fw.BufferSend), guarded fileBufferReturnIn(Fw.BufferSend), out commandOut(Fw.Com), sync cmdResponseIn(Fw.CmdResponse). Com (com-adapter interface): sync dataIn(ComDataWithContext), out dataOut, out comStatusOut(Fw.SuccessCondition), out dataReturnOut, sync dataReturnIn. Fw::Success enum U8: FAILURE=0, SUCCESS=1.

### FprimeFramer

Files: `Svc/FprimeFramer/FprimeFramer.cpp`, `Svc/FprimeFramer/FprimeFramer.fpp`

Passive. dataIn: frameSize = 8 + data.size + 4; asserts data.size <= U32::MAX. bufferAllocate_out(frameSize); if buffer invalid or too small: event NoBufferAvailable (throttle 5), deallocate if valid, return original via dataReturnOut, drop. Else serialize header (startWord default + lengthField=data.size), raw payload (OMIT_LENGTH), compute CRC over frameBuffer[0 .. frameSize-4) via Utils::Hash::hash, trailer.crcField = hashBuffer.asBigEndianU32(); trim buffer to frameSize; dataOut_out(frame, same context); dataReturnOut_out(original data, context). comStatusIn: forward to comStatusOut if connected. dataReturnIn (framed buffer back from com adapter): bufferDeallocate_out.

### FprimeDeframer

Files: `Svc/FprimeDeframer/FprimeDeframer.cpp`, `Svc/FprimeDeframer/FprimeDeframer.fpp`

Passive; dataIn guarded. Reject (event + dataReturnOut with ORIGINAL context, drop) when: size < 12 (InvalidBufferReceived); startWord != 0xDEADBEEF (InvalidStartWord); size != 8+len+4 exactly (InvalidLengthReceived); CRC(header+payload) != trailer.crcField (InvalidChecksum). APID extraction: after header, if remaining < 4+sizeof(FwPacketDescriptorType)=6 → WARNING_LO PayloadTooShort, apid left as incoming context's; else read U16 BE descriptor; if ComCfg::Apid::isValid(descriptor) set context.apid=descriptor else apid=INVALID_UNINITIALIZED (0x0800). Success: data.advance(8) (moves pointer/offset, shrinks size), setSize(size-4), dataOut_out(data, contextCopy) — same allocation, so downstream must return the buffer for the original allocator. dataReturnIn → dataReturnOut passthrough.

### FrameAccumulator + FprimeFrameDetector

Files: `Svc/FrameAccumulator/FrameAccumulator.cpp`, `Svc/FrameAccumulator/FrameDetector.hpp`, `Svc/FrameAccumulator/FrameDetector/FprimeFrameDetector.cpp`, `Svc/FrameAccumulator/FrameAccumulator.fpp`

Passive; guarded dataIn. configure(detector, allocationId, allocator, store_size) allocates ring store. dataIn: if buffer valid, processBuffer; always dataReturnOut immediately (accumulator copies bytes). processBuffer: loop copying min(ring_free, remaining) into ring then processRing; asserts remaining==0 || ring full at end. processRing loop (bounded by ring capacity): detect() must not consume; if size_out > ring capacity → event FrameDetectionSizeError(size_out), rotate(1), continue; FRAME_DETECTED → bufferAllocate(size_out); valid → peek into buffer, setSize, rotate(size_out), dataOut(buffer, incoming context); invalid → event NoBufferAvailable, and if ring free==0 also rotate(size_out) + event FrameDetectionValidFrameDropped, break; MORE_DATA_NEEDED (size_out must be > available) → break; NO_FRAME_DETECTED → rotate(1). dataReturnIn → bufferDeallocate_out. FrameDetector::Status enum: FRAME_DETECTED=0, NO_FRAME_DETECTED=1, MORE_DATA_NEEDED=2. FprimeFrameDetector::detect: <12 bytes → MORE_DATA_NEEDED(size_out=12); peek 8-byte header; wrong startWord → NO_FRAME_DETECTED; overflow guard on len; expected=len+12; expected > ring capacity → NO_FRAME_DETECTED (drop via byte-shift); allocated < expected → MORE_DATA_NEEDED(expected); peek trailer at offset 8+len; CRC over first 8+len bytes (peeked byte-by-byte) compared to trailer BE U32; mismatch → NO_FRAME_DETECTED; else FRAME_DETECTED(size_out=expected). CcsdsTcFrameDetector also exists (expects flags|scid token with bypass flag, SpacecraftId=0x0044).

### Types::CircularBuffer

Files: `Utils/Types/CircularBuffer.hpp`, `Utils/Types/CircularBuffer.cpp`

Externally-backed ring: setup(buffer,size) once; serialize(data,len) refuses if len > free (FW_SERIALIZE_NO_ROOM_LEFT; never overwrites); peek(dst,size,offset) non-consuming; rotate(n) consumes from front; trim(n) removes from back; get_allocated_size/get_free_size/get_capacity (capacity == full store size — the header comment about losing one byte is stale); high-water-mark tracking. Serializable-object slot overloads stage through a stack buffer of CircularBufferCfg::STAGING_BUFFER_SIZE across wrap.

### Utils::Hash CRC32

Files: `Utils/Hash/Crc32/Crc32.cpp`, `Utils/Hash/Crc32/HashImpl.cpp`, `Utils/Hash/HashBufferCommon.cpp`, `Utils/Hash/HashConfig.hpp`

HashConfig selects Crc32 implementation framework-wide (framing AND Utils/CRCChecker file checksums use the same Utils::Hash). Algorithm = CRC-32/ISO-HDLC (IEEE 802.3): reflected, table poly 0xEDB88320, init(): handle=0xFFFFFFFF; update(): slice-by-4 table implementation of crc32_ieee802_3_update (byte-order-safe, alignment prologue); finalize(): result = ~handle, serialized big-endian into 4-byte HashBuffer. HashBuffer::asBigEndianU32 reads digest bytes MSB-first, reproducing the U32 CRC value. HASH_DIGEST_LENGTH=4, extension ".CRC32". Equivalent to Rust crc32fast / zlib crc32. Test vector: crc32("123456789")=0xCBF43926.

### ComQueue

Files: `Svc/ComQueue/ComQueue.cpp`, `Svc/ComQueue/ComQueue.hpp`, `Svc/ComQueue/ComQueue.fpp`, `Utils/Types/Queue.cpp`

Active. Defaults (AcConstants.fpp): ComQueueComPorts=2, ComQueueBufferPorts=1, TOTAL=3. Ports: comPacketQueueIn[2] async Fw.Com with `drop` full-behavior; bufferQueueIn[1] async Fw.BufferSend with `hook` (overflowHook → bufferReturnOut); comStatusIn async; dataReturnIn sync; run async drop; dataOut ComDataWithContext; bufferReturnOut[1]. QueueType enum U8: COM_QUEUE=0, BUFFER_QUEUE=1. Config: per-queue {depth>0, priority in [0,TOTAL), Types::QueueMode QUEUE_FIFO=0/QUEUE_LIFO=1, QueueOverflowMode QUEUE_DROP_NEWEST=0/QUEUE_DROP_OLDEST=1}; entries ordered Com then Buffer; configure builds priority-sorted metadata list (stable by entry index within equal priority), allocates one block: sum(depth*msgSize) where msgSize = ComBuffer::SERIALIZED_SIZE for com queues, Buffer::SERIALIZED_SIZE for buffer queues (queues persist serialized objects in Types::Queue over a CircularBuffer). State machine SendState{READY,WAITING}, initial WAITING — nothing is sent until first comStatusIn SUCCESS (emitted by ComStub on driver connect). comStatusIn in WAITING + SUCCESS → READY + processQueue; FAILURE → stay WAITING; receiving status while READY asserts. processQueue: scan prioritized list, first non-empty queue: dequeue; com path wraps m_dequeued_com_buffer into an Fw::Buffer, reads leading U16 descriptor → context.apid, context.comQueueIndex=queueIndex, atomically flips m_buffer_state OWNED→UNOWNED (asserts), dataOut_out, state=WAITING (one outstanding send). Then round-robin: rotate the dispatched entry to the end of its equal-priority run. enqueue(Buffer, DROP_OLDEST): pre-emptively popFront and bufferReturnOut the oldest when full so ownership isn't leaked; enqueue status FW_SERIALIZE_NO_ROOM_LEFT/FW_SERIALIZE_DISCARDED_EXISTING (or pre-emptive drop) → event QueueOverflow(queueType, portNum) once per queue until a successful send clears throttle. bufferQueueIn returns buffer via bufferReturnOut if enqueue rejected. handleEnqueueStatus calls processQueue if READY. dataReturnIn: atomic exchange UNOWNED→OWNED (asserts); bufferReturnPortNum = context.comQueueIndex - COM_PORT_COUNT; if >=0 forward to bufferReturnOut[portNum] (asserts connected); com-buffer returns (negative) are dropped (data was copied). run: emits comQueueDepth/buffQueueDepth high-water-mark telemetry arrays. Commands: FLUSH_QUEUE(queueType,index), FLUSH_ALL_QUEUES (drain; buffer queues return each buffer), SET_QUEUE_PRIORITY (validates, updates, bubble-sorts, event QueuePriorityChanged).

### ComStub

Files: `Svc/ComStub/ComStub.cpp`, `Svc/ComStub/ComStub.hpp`, `Svc/ComStub/ComStub.fpp`

Passive com adapter implementing Svc.Com over either sync (imports Drv.ByteStreamDriverClient: drvConnected, drvReceiveIn, drvReceiveReturnOut, drvSendOut) or async driver (drvAsyncSendOut: Fw.BufferSend, drvAsyncSendReturnIn: Drv.ByteStreamData). RETRY_LIMIT=10. m_reinitialize starts true. drvConnected: if comStatusOut connected and m_reinitialize → emit SUCCESS once, clear flag (this primes ComQueue). dataIn asserts !(m_reinitialize && comStatusOut connected). Sync send: loop up to RETRY_LIMIT while driver returns SEND_RETRY; OP_OK → SUCCESS; retry-exhausted or error → m_reinitialize=true, FAILURE; always dataReturnOut(buffer, context) then comStatusOut. Async send: store context (single outstanding), drvAsyncSendOut; callback SEND_RETRY → resend up to RETRY_LIMIT then FAILURE + reinit; other statuses → dataReturnOut(stored context), m_reinitialize = (status != OP_OK), comStatusOut SUCCESS iff OP_OK. Receive: drvReceiveIn status != OP_OK → drvReceiveReturnOut immediately; OP_OK → dataOut with default-constructed FrameContext. dataReturnIn → drvReceiveReturnOut.

### FprimeRouter

Files: `Svc/FprimeRouter/FprimeRouter.cpp`, `Svc/FprimeRouter/FprimeRouter.fpp`, `default/config/FprimeRouterCfg.fpp`

Passive; guarded dataIn. Switch on context.apid: FW_PACKET_COMMAND → copy packet bytes (descriptor included) into stack Fw::ComBuffer, commandOut_out(0, com, context=0) unconditionally (must be connected), else event SerializationError; dataReturnOut immediately. FW_PACKET_FILE → if fileOut connected: insertContext(buffer→context) into m_bufferContextTable[50] (FprimeRouterCfg::BufferContextTableSize, keyed by data pointer; full → event FileOutContextTableFull, still forwards), fileOut_out(Fw.BufferSend); else dataReturnOut. default → if unknownDataOut connected: same table insert (UnknownDataOutContextTableFull), unknownDataOut_out(buffer, context); else dataReturnOut. fileBufferReturnIn (guarded): takeContext by pointer (miss → event BufferContextNotFound, default context), dataReturnOut(buffer, context). cmdResponseIn: no-op.

### ByteStreamDriver model

Files: `Drv/ByteStreamDriverModel/ByteStreamDriverModel.fpp`, `Drv/Interfaces/ByteStreamDriver.fpp`, `Drv/Interfaces/AsyncByteStreamDriver.fpp`

enum ByteStreamStatus: U8 {OP_OK=0, SEND_RETRY=1, RECV_NO_DATA=2, OTHER_ERROR=3}. Ports: ByteStreamData(ref buffer: Fw.Buffer, status) — recv delivery and async-send callback; ByteStreamSend(ref sendBuffer) -> ByteStreamStatus — sync send, caller retains buffer ownership; ByteStreamReady() — driver ready signal. interface ByteStreamDriver (sync): out ready, out recv(ByteStreamData), guarded input send, guarded input recvReturnIn(Fw.BufferSend). interface AsyncByteStreamDriver: async input send(Fw.BufferSend), out sendReturnOut(ByteStreamData), plus ready/recv/recvReturnIn.

### Drv/Ip sockets

Files: `Drv/Ip/IpSocket.cpp`, `Drv/Ip/IpSocket.hpp`, `Drv/Ip/TcpClientSocket.cpp`, `Drv/Ip/TcpServerSocket.cpp`, `Drv/Ip/UdpSocket.cpp`, `default/config/IpCfg.hpp`

SocketIpStatus enum: SOCK_SUCCESS=0, FAILED_TO_GET_SOCKET=-1, FAILED_TO_GET_HOST_IP=-2, INVALID_IP_ADDRESS=-3, FAILED_TO_CONNECT=-4, FAILED_TO_SET_SOCKET_OPTIONS=-5, INTERRUPTED_TRY_AGAIN=-6, READ_ERROR=-7, DISCONNECTED=-8, FAILED_TO_BIND=-9, FAILED_TO_LISTEN=-10, FAILED_TO_ACCEPT=-11, SEND_ERROR=-13 (note: -12 unused), NOT_STARTED=-14, FAILED_TO_READ_BACK_PORT=-15, NO_DATA_AVAILABLE=-16, ANOTHER_THREAD_OPENING=-17, AUTO_CONNECT_DISABLED=-18, INVALID_CALL=-19. SocketDescriptor{fd=-1, serverFd=-1}. configure: dotted-quad IPv4 only (inet_pton; no DNS), port!=0 for TCP client; SO_SNDTIMEO from configured timeout. IpCfg defaults: send timeout 1s/0us, send/recv flags 0, SOCKET_MAX_ITERATIONS=0xFFFF, SOCKET_MAX_IPV4_ADDRESS_SIZE=256, SOCKET_RETRY_INTERVAL=1s, IP_SOCKET_OPTIONS={SO_REUSEADDR=1}. send(): 0-size no-op; loop ≤ MAX_ITERATIONS accumulating partial sends; EINTR/0 → retry; EBADF/ECONNRESET → SOCK_DISCONNECTED; other -1 → SOCK_SEND_ERROR; incomplete after retries → SOCK_INTERRUPTED_TRY_AGAIN. recv(): >0 → SUCCESS with size updated; 0 → handleZeroReturn (TCP: DISCONNECTED; UDP override: SUCCESS); EAGAIN/EWOULDBLOCK → NO_DATA_AVAILABLE; ECONNRESET/EBADF → DISCONNECTED; EINTR → retry; else READ_ERROR. TcpServerSocket: startup() = socket/bind/getsockname (reads back ephemeral port)/listen(backlog=1) into serverFd; openProtocol() = accept() → fd, single client; terminate() shuts down+closes serverFd. UdpSocket: configureSend/configureRecv separately; send port 0 ⇒ reply-to-last-sender mode (learned from first recvfrom); sendProtocol before peer learned → ENOTCONN error; zero-length datagram send supported.

### SocketComponentHelper + Tcp components

Files: `Drv/Ip/SocketComponentHelper.cpp`, `Drv/Ip/SocketComponentHelper.hpp`, `Drv/TcpClient/TcpClientComponentImpl.cpp`, `Drv/TcpServer/TcpServerComponentImpl.cpp`

Two Os::Tasks per driver: read task (readLoop) and reconnect task (reconnectLoop, poll every 50ms; requesters wait in 10ms steps with 1s default timeout). OpenState{NOT_OPEN,OPENING,OPEN,SKIP}; ReconnectState{NOT_RECONNECTING,REQUEST_RECONNECT,RECONNECT_IN_PROGRESS}. Only the reconnect thread opens sockets; send()/readLoop request reconnect and wait. m_reopen (auto-open) default true; when false, open() must be called manually and readLoop exits on SOCK_AUTO_CONNECT_DISABLED. readLoop: allocate via getBuffer() (component allocate_out with configured buffer_size); recv; on status not in {SUCCESS, INTERRUPTED_TRY_AGAIN, NO_DATA_AVAILABLE} → close + size 0; sendBuffer maps SOCK_SUCCESS→OP_OK, SOCK_NO_DATA_AVAILABLE→RECV_NO_DATA, else OTHER_ERROR, then recv_out port. send_handler (both Tcp components) maps SOCK_INTERRUPTED_TRY_AGAIN→SEND_RETRY, SOCK_SUCCESS→OP_OK, else OTHER_ERROR. connected() fires ready_out (→ComStub.drvConnected). recvReturnIn → deallocate_out. TcpServer overrides readLoop to retry startup() (listen) until success then delegates; terminate() on exit. stop() sets stop flags + shutdown() to break blocking recv; join() joins both tasks.

## Wire formats

### F Prime frame (uplink and downlink, GDS-compatible)

All fields big-endian (F Prime serialization default, Fw/Types/Serializable.cpp). Offset 0: U32 start word = 0xDEADBEEF (FprimeProtocol.fpp default for FrameHeader.startWord). Offset 4: U32 lengthField = payload byte count (excludes header and trailer). Offset 8: payload, lengthField bytes, raw (serialized with OMIT_LENGTH — no length token). Offset 8+len: U32 CRC32 over bytes [0, 8+len) i.e. header+payload, stored big-endian. CRC algorithm: CRC-32/ISO-HDLC (IEEE 802.3; reflected 0xEDB88320 table, init 0xFFFFFFFF, final XOR 0xFFFFFFFF) — Utils/Hash/Crc32. Total frame size = 12 + lengthField; minimum valid frame = 12 bytes. Defined by FprimeFramer.cpp serialization order and FprimeDeframer.cpp/FprimeFrameDetector.cpp validation.

### F Prime packet (frame payload)

Offset 0: FwPacketDescriptorType packet descriptor = U16 big-endian by default (default/config/ComCfg.fpp `dictionary type FwPacketDescriptorType = U16`) holding a ComCfg::Apid value (FW_PACKET_COMMAND=0, FW_PACKET_TELEM=1, FW_PACKET_LOG=2, FW_PACKET_FILE=3, ...). Remainder: packet-type-specific body. FprimeDeframer reads it to fill FrameContext.apid; ComQueue reads the leading descriptor of every outgoing buffer to set context.apid; FprimeRouter dispatches on it. Payloads shorter than 2 bytes yield PayloadTooShort and no APID update.

### CRC digest (HashBuffer)

4-byte buffer containing ~crc serialized big-endian (Crc32/HashImpl.cpp finalize → LinearBufferBase::serializeFrom(U32, BIG)); asBigEndianU32() folds bytes MSB-first back to the U32. Wire CRC field == this U32 serialized big-endian.

## Threading / concurrency

FprimeFramer, FprimeDeframer, FrameAccumulator, FprimeRouter, ComStub: passive components — handlers execute on the caller's thread; `guarded` inputs (deframer/accumulator/router dataIn, router fileBufferReturnIn, driver send/recvReturnIn) take the component mutex, `sync` inputs do not. ComQueue: active component with its own thread + message queue; comPacketQueueIn/run are async-drop (message dropped when queue full), bufferQueueIn is async-hook (overflowHook returns buffer ownership), commands async; dataReturnIn is sync and runs on the returning driver/adapter thread — hence m_buffer_state is std::atomic<BufferState>{OWNED,UNOWNED} with exchange-assert protocol guaranteeing exactly one outstanding dataOut buffer. Flow-control invariant: ComQueue sends one buffer, then WAITs for comStatusIn SUCCESS (delivered async, serialized through its queue). Drivers: TcpClient/TcpServer components are passive but own a SocketComponentHelper with two Os::Tasks — a read task (blocking recv loop, pushes buffers up recv_out from the driver thread, which is what drives the entire uplink chain accumulator→deframer→router synchronously) and a reconnect task (sole owner of socket open; 50ms poll; requesters block up to 1s in 10ms steps). Shared state (m_descriptor, m_open, m_stop) under m_lock; reconnect state under m_reconnectLock. Downlink send happens on ComQueue's thread through framer→ComStub→driver send_handler (guarded).

## Porting notes

1) Model ComDataWithContext as (buffer, FrameContext) with explicit ownership transfer; every component pairs dataOut with dataReturnIn — in Rust, represent buffer ownership by moving an owned Buffer handle through the port graph and back; the C++ 'ref + return port' convention becomes move semantics, but keep the return *ports* because allocators (BufferManager) differ per hop. 2) Single-source the frame constants: START_WORD=0xDEADBEEF u32, HEADER_SIZE=8, TRAILER_SIZE=4; implement header/trailer as plain structs with BE serialization; use crc32fast (ISO-HDLC) — verify with crc32(b\"123456789\")==0xCBF43926. 3) FrameDetector should be a trait `fn detect(&self, ring: &CircularBuffer) -> Status` with Status::{FrameDetected(usize), NoFrameDetected, MoreDataNeeded(usize)} — encoding the C++ size_out contract (MoreDataNeeded carries total bytes needed and MUST exceed available; FrameAccumulator asserts this). 4) Reproduce FrameAccumulator's exact drop policies (rotate(1) on no-detect; whole-frame drop only when ring is full and allocation fails) — these keep the resync loop live under back-pressure. 5) ComQueue: implement the prioritized metadata list as a stable priority sort with post-send rotation of the equal-priority run (round-robin); keep SendState WAITING initial state and the READY-status assert; keep the pre-emptive DROP_OLDEST pop for buffer queues so ownership is returned before overwrite. The atomic OWNED/UNOWNED handshake maps to an AtomicU8/AtomicBool with swap+assert. 6) ComStub's m_reinitialize handshake is the system bootstrap: comStatus SUCCESS is emitted only on driver `ready` after construction or after a failure — port faithfully or ComQueue never transmits. 7) Drv/Ip: keep the two-thread model (read + reconnect) and the rule that only the reconnect task opens sockets; map errno branches exactly (EINTR retry, EAGAIN→NO_DATA, ECONNRESET/EBADF→DISCONNECTED); TCP recv 0 ⇒ disconnected, UDP recvfrom 0 ⇒ success. Rust std TcpStream with SO_SNDTIMEO via socket2; backlog=1 for server; SO_REUSEADDR on. 8) Make FwPacketDescriptorType width a config type parameter (default u16) rather than hardcoding. 9) Events/telemetry/commands listed per component are part of the ground contract — keep names, severities, formats, and the QueueType/Apid enum values for dictionary compatibility.

## Gotchas

- FwPacketDescriptorType defaults to U16 in this repo (ComCfg.fpp), not U32 as in older F Prime — the GDS dictionary must agree; deframer reads 2 bytes for APID.
- The frame lengthField counts payload only; total frame = 12 + len. FprimeDeframer requires buffer size == 12+len EXACTLY (frames must be pre-extracted by FrameAccumulator); oversized buffers are dropped with InvalidLengthReceived.
- Start-word validation compares against the FPP default-constructed FrameHeader, so changing the FPP default changes the protocol everywhere (framer writes it, deframer and detector check it).
- CRC covers header+payload (bytes 0..8+len), NOT just payload, and the trailer stores the post-~ (final XOR) value big-endian. Utils::Hash::update does NOT finalize; finalize applies ~ without mutating state.
- Deframer emits the payload as the SAME allocation advanced by 8 and shrunk by 4 (Fw::Buffer keeps original pointer via offset); FprimeRouter's context table keys on getData() — the advanced pointer — and buffer identity must survive the round trip for return-path deallocation.
- ComQueue starts WAITING: with no ComStub drvConnected→SUCCESS handshake nothing is ever transmitted; conversely a comStatus arriving while READY is an assert (protocol violation).
- ComQueue com-buffer sends alias internal storage m_dequeued_com_buffer (single outstanding send makes this safe); returned com buffers are NOT forwarded anywhere — only buffer-queue returns (comQueueIndex >= COM_PORT_COUNT) go out bufferReturnOut.
- Types::Queue DROP_OLDEST silently discards via ring rotate and returns FW_SERIALIZE_DISCARDED_EXISTING (still counts as accepted); ComQueue adds a pre-emptive popFront for Fw::Buffer queues specifically to return ownership before the drop — omitting this leaks pool buffers.
- FrameAccumulator returns the incoming driver buffer immediately (it copies into the ring); the ring can be smaller than incoming buffers — the outer loop iterates buffer_size times feeding chunks. A detected frame larger than ring capacity is unprocessable: detector returns NO_FRAME_DETECTED (Fprime detector) or accumulator logs FrameDetectionSizeError and slides one byte.
- MORE_DATA_NEEDED contract: size_out is the TOTAL needed (not additional) and the accumulator asserts size_out > available — returning size_out <= available crashes.
- ByteStreamStatus SEND_RETRY drives bounded retry loops: ComStub sync retries the port call up to 10 times inline; async increments m_retry_count across callbacks. TcpClient maps only SOCK_INTERRUPTED_TRY_AGAIN to SEND_RETRY.
- IpSocket::send treats sendProtocol()==0 as a retry (continue), while recv 0 is protocol-dependent (TCP disconnect vs UDP empty datagram via handleZeroReturn override).
- SocketIpStatus skips -12; values are negative except SOCK_SUCCESS=0.
- TcpServer startup binds+listens at configure() time (returns startup status); listen backlog is 1; getsockname reads back the ephemeral port when port 0 is configured (valid for server, invalid for client).
- UDP with send-port 0 learns its send destination from the first received datagram (recvProtocol overwrites m_addr_send once); sends before that fail with ENOTCONN → SOCK_SEND_ERROR.
- ComStub receive path forwards a DEFAULT FrameContext (apid=FW_PACKET_UNKNOWN etc.), not the driver's; the deframer's context is a copy of what FrameAccumulator forwarded, and drop paths return the ORIGINAL context, not the modified copy.
- Deframer sets apid=INVALID_UNINITIALIZED (0x0800) for descriptors not in the Apid enum (despite a comment mentioning FW_PACKET_UNKNOWN); router's default arm then handles them via unknownDataOut.
- FprimeFramer allocation may return a larger buffer than requested — frame is trimmed with setSize; an allocator returning a SMALLER valid buffer must be deallocated and the send dropped (with dataReturnOut still called).

