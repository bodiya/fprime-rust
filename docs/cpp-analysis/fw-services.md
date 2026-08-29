# Fw command/event/telemetry/parameter/time infrastructure (Fw/Cmd, Fw/Log, Fw/Logger, Fw/Tlm, Fw/Prm, Fw/Time, Fw/Com, Fw/FilePacket)

> Analysis of the C++ F Prime implementation (github.com/nasa/fprime) produced to guide this Rust port.
> Source tree analyzed: local clone at commit of 2026-08. File paths refer to that C++ tree.

## Overview

This layer defines the value types, port signatures, and GDS wire packets that all F Prime components use for commanding, events, telemetry, parameters, and time. Ports (defined in .fpp) carry typed buffers: Fw.Cmd(opCode, cmdSeq, CmdArgBuffer), Fw.CmdResponse(opCode, cmdSeq, CmdResponse), Fw.Log(id, Time, LogSeverity, LogBuffer), Fw.LogText(id, Time, LogSeverity, TextLogString), Fw.Tlm(id, Time, TlmBuffer), Fw.TlmGet(...)->TlmValid, Fw.PrmGet(id, ParamBuffer)->ParamValid, Fw.PrmSet(id, ParamBuffer), Fw.Time(ref Time), Fw.Com(ref ComBuffer, context U32). Packet classes (CmdPacket, LogPacket, TlmPacket) derive from ComPacket, which serializes/deserializes a leading packet descriptor of type FwPacketDescriptorType (U16, per default/config/ComCfg.fpp) whose values come from the ComCfg::Apid FPP enum. Buffers are LinearBufferTemplate<N> instances (fixed capacity, serialize/deserialize cursor pair); strings are StringTemplate<N>. All serialization is big-endian by default via LinearBufferBase (Fw/Types/Serializable.cpp), with an optional per-call Endianness::LITTLE mode. Time is an 11-byte struct (TimeBase U16, context U8, seconds U32, useconds U32) autocoded from Fw/Time/Time.fpp; it is embedded in every event and telemetry channel sample. Svc components (CmdDispatcher, EventManager, TlmChan, PrmDb, FileUplink/Downlink) consume/produce these packets; the GDS depends byte-for-byte on the layouts below. Fw::Logger is a separate minimal static-singleton diagnostic text logger, unrelated to the Fw.Log event path.

## Key items

### ComPacket / ComPacketType (Apid enum)

Files: `Fw/Com/ComPacket.hpp`, `Fw/Com/ComPacket.cpp`, `default/config/ComCfg.fpp`

ComPacket holds m_type (ComPacketType = ComCfg::Apid::T, default-constructed to FW_PACKET_UNKNOWN). serializeBase/deserializeBase write/read m_type cast to FwPacketDescriptorType (dictionary type = U16 in ComCfg.fpp). Apid enum (repr FwPacketDescriptorType=U16): FW_PACKET_COMMAND=0x0000, FW_PACKET_TELEM=0x0001, FW_PACKET_LOG=0x0002, FW_PACKET_FILE=0x0003, FW_PACKET_PACKETIZED_TLM=0x0004, FW_PACKET_DP=0x0005, FW_PACKET_IDLE=0x0006, FW_PACKET_PARAM=0x0007, FW_PACKET_HAND=0x00FE, FW_PACKET_UNKNOWN=0x00FF, SPP_IDLE_PACKET=0x07FF, INVALID_UNINITIALIZED=0x0800 (enum default). ComCfg also defines Pvn enum {SPACE_PACKET_PROTOCOL=0x0, ENCAPSULATION_PACKET_PROTOCOL=0x7, INVALID_UNINITIALIZED=0x8} and FrameContext struct for the comms stack.

### CmdPacket + Cmd ports + CmdResponse enum

Files: `Fw/Cmd/CmdPacket.cpp`, `Fw/Cmd/CmdPacket.hpp`, `Fw/Cmd/Cmd.fpp`, `Fw/Cmd/CmdArgBuffer.hpp`, `Fw/Cmd/CmdString.hpp`

Deserialize-only in FSW: serializeTo() does FW_ASSERT(false). deserializeFrom: read descriptor (U16); if m_type != FW_PACKET_COMMAND return FW_DESERIALIZE_TYPE_MISMATCH; read opcode (FwOpcodeType = U32 via FwIdType); then if bytes remain, copyRaw the remainder into m_argBuffer (replacing contents); if no bytes remain, m_argBuffer.resetSer() (explicitly clears stale args for zero-arg commands). CmdResponse enum (U8): OK=0, INVALID_OPCODE=1, VALIDATION_ERROR=2, FORMAT_ERROR=3, EXECUTION_ERROR=4, BUSY=5. Ports: CmdReg(opCode), Cmd(opCode, cmdSeq U32, ref args CmdArgBuffer), CmdResponse(opCode, cmdSeq U32, response CmdResponse). CmdArgBuffer = LinearBufferTemplate<FW_CMD_ARG_BUFFER_MAX_SIZE=512-4-2=506>. CmdStringArg = StringTemplate<40>.

### LogPacket + Log ports + LogSeverity

Files: `Fw/Log/LogPacket.cpp`, `Fw/Log/Log.fpp`, `Fw/Log/LogBuffer.hpp`, `Fw/Log/LogString.hpp`, `Fw/Log/TextLogString.hpp`

LogPacket holds id (FwEventIdType=U32), Fw::Time timeTag, LogBuffer args. serializeTo: descriptor + id + timeTag + raw arg bytes (OMIT_LENGTH). deserializeFrom: descriptor + id + timeTag, then all remaining bytes into logBuffer (FW_DESERIALIZE_SIZE_MISMATCH if remainder > capacity). Constructor sets type FW_PACKET_LOG. Note: severity is NOT in the packet — GDS resolves it from the dictionary by id. LogSeverity (U8): FATAL=1, WARNING_HI=2, WARNING_LO=3, COMMAND=4, ACTIVITY_HI=5, ACTIVITY_LO=6, DIAGNOSTIC=7 (no 0). Ports: Log(id, ref Time, severity, ref LogBuffer), LogText(id, ref Time, severity, ref TextLogString). LogBuffer = LinearBufferTemplate<512-4-2=506>. LogStringArg = StringTemplate<200>; TextLogString = StringTemplate<256>. EventManager (Svc) serializes LogPacket into a ComBuffer and emits via PktSend; FATAL always passes filters.

### TlmPacket + Tlm ports

Files: `Fw/Tlm/TlmPacket.cpp`, `Fw/Tlm/Tlm.fpp`, `Fw/Tlm/TlmBuffer.hpp`, `Fw/Tlm/TlmString.hpp`

TlmPacket is an accumulator around an internal ComBuffer m_tlmBuffer plus m_numEntries (FwSizeType). resetPktSer(): resetSer + assert type==FW_PACKET_TELEM + serialize descriptor into internal buffer; numEntries=0. addValue(id, time, TlmBuffer): checks room for sizeof(FwChanIdType)+Time::SERIALIZED_SIZE+buffer.getSize(), then serializes id (U32), Time (11B), raw value bytes (OMIT_LENGTH); increments numEntries; returns FW_SERIALIZE_NO_ROOM_LEFT when full. extractValue(id&, time&, buf&, bufferSize) requires caller-known value size (no per-entry length on wire); returns FW_DESERIALIZE_BUFFER_EMPTY at end. GDS wire = the internal ComBuffer via getBuffer() (TlmChan sends it directly). TlmPacket::serializeTo/deserializeFrom (a different format: numEntries as FwSizeType followed by raw internal buffer, no descriptor prefix at outer level) is NOT the downlink path. TlmValid enum (U8): VALID=0, INVALID=1. TlmBuffer = LinearBufferTemplate<512-4-2=506>. TlmString = StringTemplate<40>.

### Prm types

Files: `Fw/Prm/Prm.fpp`, `Fw/Prm/PrmBuffer.hpp`, `Fw/Prm/ParamValid.hpp`, `Fw/Prm/PrmString.hpp`

ParamValid enum (U8): UNINIT=0, VALID=1, INVALID=2, DEFAULT=3. FW_PARAM_OK(v) macro := (v==DEFAULT || v==VALID). Ports: PrmGet(id FwPrmIdType=U32, ref val ParamBuffer)->ParamValid (val unmodified if not found); PrmSet(id, ref val). ParamBuffer = LinearBufferTemplate<FW_PARAM_BUFFER_MAX_SIZE=512-4-2=506>; static_assert it fits StringBase::BUFFER_SIZE(FW_PARAM_STRING_MAX_SIZE=40). PrmBuffer.hpp is an alias header ('work around inconsistent spelling' — ParamBuffer.hpp just includes PrmBuffer.hpp). ParamString = StringTemplate<40>.

### Fw::Time / TimeValue / TimeBase / TimeComparison

Files: `Fw/Time/Time.hpp`, `Fw/Time/Time.cpp`, `Fw/Time/Time.fpp`, `default/config/FpConfig.fpp`

Wraps autocoded TimeValue struct {timeBase: TimeBase, timeContext: FwTimeContextStoreType=U8, seconds: U32, useconds: U32}. SERIALIZED_SIZE = sizeof(FwTimeBaseStoreType=U16)+sizeof(U8)+4+4 = 11 bytes. TimeBase enum (repr U16): TB_NONE=0, TB_PROC_TIME=1, TB_WORKSTATION_TIME=2, TB_SC_TIME=3, TB_DONT_CARE=0xFFFF; default TB_NONE. Default ctor = (TB_NONE, 0, 0, 0); Time(sec,usec) ctor also forces TB_NONE, ctx 0. set() FW_ASSERTs useconds < 1000000. deserializeFrom: deserializes into a temp TimeValue, returns FW_DESERIALIZE_FORMAT_ERROR (leaving *this unmodified) if useconds >= 1000000. compare(): differing timeBase => TimeComparison::INCOMPARABLE; context is IGNORED; otherwise lexicographic (seconds, useconds) => LT/GT/EQ. TimeComparison enum: LT=-1, EQ=0, GT=1, INCOMPARABLE=2. add(a,b)/sub(min,sub): FW_ASSERT equal timeBase (crash, not error); sub also FW_ASSERTs minuend>=subtrahend; usec carry/borrow at 1e6 (add asserts summed usec < 1999999); result context = a's context, or 0 if contexts differ. F64 conversions: parseSeconds=trunc; parseUSeconds=round-to-nearest (frac*1e6+0.5); set(F64) carries a rounded-up 1e6 into seconds. ZERO_TIME global const. FW_CONTEXT_DONT_CARE=0xFF (FpConstants.fpp) for sequences.

### Fw::TimeInterval / TimeIntervalValue

Files: `Fw/Time/TimeInterval.hpp`, `Fw/Time/TimeInterval.cpp`

Wraps TimeIntervalValue {seconds U32, useconds U32}; SERIALIZED_SIZE=8. No time base/context. set() asserts usec<1e6. compare(): lexicographic (sec, usec), Comparison typedef LT=-1/EQ=0/GT=1/INCOMPARABLE=2 (INCOMPARABLE unused). add: same carry logic as Time. sub(t1,t2) is COMMUTATIVE absolute difference (orders operands so larger is minuend). TimeInterval(start, end) = |end - start| via that sub. No serialized validation on deserialize (delegates to autocoded struct).

### Serialization core (LinearBufferBase / SerializeStatus / Endianness)

Files: `Fw/Types/Serializable.hpp`, `Fw/Types/Serializable.cpp`, `Fw/Types/LinearBufferTemplate.hpp`, `default/config/FpConstants.fpp`

SerializeStatus enum order (values 0..9): FW_SERIALIZE_OK, FW_SERIALIZE_FORMAT_ERROR, FW_SERIALIZE_NO_ROOM_LEFT, FW_DESERIALIZE_BUFFER_EMPTY, FW_DESERIALIZE_FORMAT_ERROR, FW_DESERIALIZE_SIZE_MISMATCH, FW_DESERIALIZE_TYPE_MISMATCH, FW_DESERIALIZE_IMMUTABLE, FW_DESERIALIZE_INVALID_DATA, FW_SERIALIZE_DISCARDED_EXISTING. Endianness {BIG (default everywhere), LITTLE}; integers written MSB-first for BIG by explicit shifts; F32/F64 bit-copied to U32/U64 then swapped. bool: writes FW_SERIALIZE_TRUE_VALUE=0xFF / FW_SERIALIZE_FALSE_VALUE=0x00; deserializing any other byte => FW_DESERIALIZE_FORMAT_ERROR without advancing. Byte-array serializeFrom(buff,len) DEFAULTS to INCLUDE_LENGTH (prefix FwSizeStoreType=U16); OMIT_LENGTH writes raw. Serializing a LinearBufferBase writes U16 size + bytes; deserializing reads U16, checks capacity and remaining, setBuffLen. Strings: serializeTo = U16 length + chars, no NUL (ConstStringBase.cpp lines 85-89); deserializeFrom reads INCLUDE_LENGTH then NUL-terminates locally. copyRaw(dest,size): replaces dest contents; copyRawOffset appends (OMIT_LENGTH). Buffer state: m_serLoc (bytes valid), m_deserLoc; EVERY serializeFrom sets m_deserLoc=0. deserializeTo returns FW_DESERIALIZE_BUFFER_EMPTY when nothing left, FW_DESERIALIZE_SIZE_MISMATCH when some-but-insufficient bytes left. LinearBufferTemplate<MaxSize> owns U8[MaxSize]; copy ctor/assign copies contents via setBuff and asserts OK; SERIALIZED_SIZE = sizeof(U16)+MaxSize. Buffer constants (FpConstants.fpp): FW_COM_BUFFER_MAX_SIZE=512; cmd/log/tlm/param buffers = 512 - sizeof(id-type=4) - sizeof(descriptor=2) = 506; FW_FILE_BUFFER_MAX_SIZE=512.

### Fw::Logger (diagnostic text log)

Files: `Fw/Logger/Logger.hpp`, `Fw/Logger/Logger.cpp`

Static singleton: Logger* s_current_logger (init nullptr); registerLogger(Logger*) sets it; log(const char* fmt, ...) formats via Fw::String::vformat (printf-style, truncation OK) then log(ConstStringBase&) which forwards to virtual writeMessage() if a logger is registered, else silently drops. No locking, no buffering, not the event (Fw.Log) path — used for framework diagnostics and assert output.

### FilePacket family

Files: `Fw/FilePacket/FilePacket.hpp`, `Fw/FilePacket/Header.cpp`, `Fw/FilePacket/PathName.cpp`, `Fw/FilePacket/DataPacket.cpp`

C++ union of Start/Data/End/Cancel packets sharing a Header. Type enum: T_START=0, T_DATA=1, T_END=2, T_CANCEL=3, T_NONE=255. Header: type as U8 + sequenceIndex U32 (HEADERSIZE=5). PathName: U8 length + bytes (MAX_LENGTH=255); fromSerialBuffer keeps a pointer INTO the source buffer (zero-copy; value lifetime tied to buffer). DataPacket: header + byteOffset U32 + dataSize U16 + raw data (deserialize requires remaining == dataSize, data pointer aliases source buffer). EndPacket: header + checksum U32 (CFDP checksum). StartPacket: header + fileSize U32 + sourcePath + destPath. These bytes ride inside a com packet after the FW_PACKET_FILE descriptor (descriptor added by Svc file components, not by FilePacket).

## Wire formats

### Packet descriptor (all GDS packets)

FwPacketDescriptorType = U16 big-endian (ComCfg.fpp 'dictionary type FwPacketDescriptorType = U16'; ComPacket::serializeBase casts Apid to it). Values: COMMAND=0, TELEM=1, LOG=2, FILE=3, PACKETIZED_TLM=4, DP=5, IDLE=6, PARAM=7, HAND=0xFE, UNKNOWN=0xFF.

### Command packet (uplink)

[descriptor U16 BE = 0x0000][opcode FwOpcodeType = U32 BE][arg bytes = remainder of buffer, raw, no length prefix]. Args are the FPP-serialized command arguments concatenated (strings inside args are U16-len-prefixed). Defined by CmdPacket::deserializeFrom (Fw/Cmd/CmdPacket.cpp:27-55); confirmed by Fw/Cmd/test/ut/CmdPacketTest.cpp. Max total 512 (ComBuffer).

### Event/log packet (downlink)

[descriptor U16 BE = 0x0002][event id FwEventIdType = U32 BE][time tag 11 bytes, see Time][arg bytes = remainder, raw OMIT_LENGTH]. Severity NOT included. LogPacket::serializeTo (Fw/Log/LogPacket.cpp:19-37); EventManager serializes this into a ComBuffer per event (one event per com packet).

### Telemetry packet (downlink, TlmChan path)

[descriptor U16 BE = 0x0001] then N repetitions of [channel id FwChanIdType = U32 BE][time tag 11 bytes][value bytes, raw, length known only from dictionary] filling up to 512 bytes. Built by TlmPacket::resetPktSer + addValue (Fw/Tlm/TlmPacket.cpp:20-94); no entry count on the wire, no per-entry length. TlmPacket::serializeTo is a different (non-GDS) format: [numEntries FwSizeType][internal buffer bytes incl. descriptor].

### Fw::Time / TimeValue

[timeBase U16 BE][timeContext U8][seconds U32 BE][useconds U32 BE] = 11 bytes, field order per Time.fpp TimeValue struct; SERIALIZED_SIZE in Time.hpp:16 confirms widths. useconds must be < 1,000,000 (deserialize rejects with FW_DESERIALIZE_FORMAT_ERROR). TimeBase: TB_NONE=0, TB_PROC_TIME=1, TB_WORKSTATION_TIME=2, TB_SC_TIME=3, TB_DONT_CARE=0xFFFF.

### Fw::TimeInterval

[seconds U32 BE][useconds U32 BE] = 8 bytes (TimeIntervalValue in Time.fpp; TimeInterval.hpp:22).

### Length-prefixed buffer/string element

[FwSizeStoreType = U16 BE length][bytes]. Used when a LinearBuffer or string is serialized as a VALUE (e.g., string command/event/tlm arguments, serialized ParamBuffer inside PrmDb file records). FpConfig.fpp: 'dictionary type FwSizeStoreType = U16'; Serializable.cpp:250-303, ConstStringBase.cpp:85-89. Packet arg regions instead use raw OMIT_LENGTH.

### bool

1 byte: true=0xFF (FW_SERIALIZE_TRUE_VALUE), false=0x00 (FW_SERIALIZE_FALSE_VALUE); any other byte on deserialize => FW_DESERIALIZE_FORMAT_ERROR (Serializable.cpp:212-228, 499-521; FpConstants.fpp:83-86).

### FPP enum serialization widths

FPP enums serialize as their representation type: CmdResponse/LogSeverity/ParamValid/TlmValid/Pvn = U8; Apid = U16; TimeBase = U16; TimeComparison = default I32. Plain C++ enums serialize as FwEnumStoreType = I32 (FpConfig.fpp:82-87).

### File packets (inside FW_PACKET_FILE=0x0003 com packet)

Common header: [type U8 (START=0,DATA=1,END=2,CANCEL=3)][sequenceIndex U32 BE]. START: header + [fileSize U32][srcPath: U8 len + bytes][destPath: U8 len + bytes]. DATA: header + [byteOffset U32][dataSize U16][data bytes; remainder must equal dataSize]. END: header + [checksum U32 (CFDP)]. CANCEL: header only. (Fw/FilePacket/Header.cpp, PathName.cpp, DataPacket.cpp).

## Threading / concurrency

None of these classes are thread-safe or contain synchronization; they are value types owned by one caller at a time. Concurrency comes from the (autocoded) component layer: active components serialize port arguments (including these buffers, copied by value via LinearBufferTemplate copy ctor) into an OS message queue and deserialize on the component thread; passive/sync ports execute in the caller's thread. TlmPacket/CmdPacket/LogPacket instances are used as locals or single-threaded members (e.g., EventManager's m_comBuffer/m_logPacket are used only on its own thread). Fw::Logger uses a bare static pointer with no lock — registration is expected at startup before threads race; log() calls after registration are only as safe as the registered writeMessage implementation. Fw::Time carries no synchronization; time-source components provide it via the Fw.Time port (guarded connection semantics belong to the component layer, not here). LinearBufferBase's cursor invariant (any serializeFrom resets the deserialize cursor to 0) assumes single-owner access.

## Porting notes

1) Model the serialization core first: a Rust `SerializeStatus` enum with the exact 10 variants and a cursor-pair buffer type (ser_loc/deser_loc over a fixed array). Use const generics: `LinearBuffer<const N: usize>` for ComBuffer(512)/CmdArgBuffer(506)/LogBuffer(506)/TlmBuffer(506)/ParamBuffer(506), with capacities sourced from a config module mirroring FpConstants.fpp (they are project-configurable — do not hardcode 512/506 outside config). 2) Make the config type aliases explicit newtypes/type aliases: FwOpcodeType=u32, FwEventIdType=u32, FwChanIdType=u32, FwPrmIdType=u32, FwPacketDescriptorType=u16, FwSizeStoreType=u16, FwTimeBaseStoreType=u16, FwTimeContextStoreType=u8 — all configurable, all affecting wire layout and the derived buffer sizes (506 = 512 - id(4) - descriptor(2)). 3) Implement serialization as explicit big-endian writes with an Endianness parameter defaulted to BIG; preserve the exact status semantics: BUFFER_EMPTY vs SIZE_MISMATCH distinction, bool strictness, INCLUDE_LENGTH default on byte-slice serialization, and serialize-resets-deserialize-cursor. 4) Packets: implement CmdPacket as parse-only (return error or unimplemented for serialize, mirroring the FW_ASSERT(false)), LogPacket as both, TlmPacket as an accumulator API (reset_ser/add_value/extract_value with caller-supplied value sizes) — Rust iterators are tempting but extraction genuinely requires dictionary knowledge of per-channel sizes. 5) Time: store the four fields; implement compare ignoring context, INCOMPARABLE across time bases; make add/sub return Result or panic to mirror FW_ASSERT (F Prime asserts — a faithful port should treat mixed time bases as a programmer error, not silent data); validate useconds<1_000_000 on deserialize returning FORMAT_ERROR and leaving self untouched; replicate context-mismatch→0 rule and the F64 round-to-nearest-microsecond-with-carry. 6) TimeInterval::sub must stay commutative-absolute. 7) Enums: derive wire width from representation (u8/u16), not a universal i32; keep exact discriminants (LogSeverity starts at 1). 8) FilePacket's zero-copy pointers into the source buffer map naturally to Rust borrowed slices (`&[u8]`/`&str` with the buffer's lifetime). 9) Fw::Logger maps to a global `OnceLock`/atomic pointer to a `dyn` writer; keep it decoupled from the event system.

## Gotchas

- The packet descriptor is U16 (ComCfg.fpp FwPacketDescriptorType), not the U32 of older F Prime versions; all packet layouts, the 506-byte derived buffer sizes, and GDS framing depend on it. It is also the ComCfg::Apid enum now (CCSDS APID, 11-bit max), with INVALID_UNINITIALIZED=0x0800 as the enum default rather than FW_PACKET_UNKNOWN.
- Event severity is not serialized in the log packet, and telemetry packets carry no entry count and no per-entry value lengths — the GDS dictionary supplies both. TlmPacket::extractValue requires the caller to pass each value's size.
- TlmPacket has two distinct formats: getBuffer() (descriptor + entries; the actual downlink) vs serializeTo() (numEntries as FwSizeType — a platform-width type — plus raw internal buffer). Porting serializeTo with a fixed width changes a non-GDS but real byte format.
- serializeFrom(const U8*, len) defaults to INCLUDE_LENGTH (writes a U16 length prefix); packet code deliberately passes OMIT_LENGTH for arg/value regions. Confusing the two is the classic corruption bug.
- Every serializeFrom call resets the deserialize cursor to 0 (m_deserLoc=0), so interleaving reads and writes on one buffer restarts reading from the beginning.
- bool deserialization accepts ONLY 0xFF/0x00 and fails FW_DESERIALIZE_FORMAT_ERROR otherwise, without consuming the byte.
- CmdPacket::deserializeFrom clears the arg buffer when a command has zero args (resetSer) because copyRaw only replaces contents when bytes exist — dropping this leaks previous packet args into zero-arg commands (there is an explicit comment about it).
- Time::compare ignores timeContext entirely but returns INCOMPARABLE for differing TimeBase; operator== can thus be true for different contexts. add/sub FW_ASSERT (crash) on mismatched time bases and sub asserts minuend>=subtrahend — they do not return errors. Context mismatch silently yields context 0.
- Time::deserializeFrom validates useconds<1e6 and must leave the target unmodified on rejection (deserialize into a temp, then commit).
- TimeInterval::sub is commutative (absolute difference) — unlike Time::sub which asserts ordering.
- LogSeverity values start at 1 (FATAL=1..DIAGNOSTIC=7); 0 is invalid and EventManager drops events with invalid severity.
- Deserialization distinguishes FW_DESERIALIZE_BUFFER_EMPTY (nothing left) from FW_DESERIALIZE_SIZE_MISMATCH (some bytes but fewer than needed); CmdPacket/loop-termination logic (extractValue until BUFFER_EMPTY) depends on this exact split.
- FPP enums serialize at their representation width (Apid/TimeBase=U16, severities/responses=U8, TimeComparison=I32); plain C++ enums use FwEnumStoreType=I32. A uniform enum width breaks the wire.
- FilePacket::PathName/DataPacket deserialization aliases the source buffer (stores pointers into it); the parsed packet is only valid while the buffer lives. FilePacket itself never writes the FW_PACKET_FILE descriptor — Svc components prepend it.
- String wire format is U16 length + bytes with no NUL; the C++ deserializer NUL-terminates its local storage but never reads a NUL from the wire.
- Time(seconds,useconds) 2-arg constructor forces TB_NONE/context 0 — it does not preserve an existing base, unlike the 2-arg set() which preserves both.

