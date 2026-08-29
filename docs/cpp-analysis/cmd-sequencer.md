# Svc/CmdSequencer + Svc/Seq (F Prime binary command sequencer)

> Analysis of the C++ F Prime implementation (github.com/nasa/fprime) produced to guide this Rust port.
> File paths refer to the C++ tree.

## Overview

`Svc::CmdSequencer` is an ACTIVE component that loads a binary sequence file into a single pre-allocated buffer, validates it (CRC-32, time base/context, record structure), then walks its records emitting `Fw::ComBuffer` command packets on `comCmdOut`, waiting for each `cmdResponseIn` before advancing. Two orthogonal state variables: `RunMode {STOPPED=0, RUNNING=1}` (`m_runMode`) and `StepMode {AUTO=0, MANUAL=1}` (`m_stepMode`). Timing is per-record: ABSOLUTE records wait until `schedIn` observes wall time >= tag; RELATIVE records add current time to the tag first. A separate command-response timeout timer cancels a stuck sequence. The sequence format is pluggable through the abstract `Sequence` base (`setSequenceFormat`); `FPrimeSequence` is the default and the only one the GDS `fprime-seqgen` produces (an `AMPCSSequence` alternative lives in `formats/`, out of scope for the port). Files: /home/user/fprime/Svc/CmdSequencer/{CmdSequencer.fpp,CmdSequencerImpl.hpp,CmdSequencerImpl.cpp,FPrimeSequence.cpp,Sequence.cpp,Events.cpp,Commands.fppi,Events.fppi,Telemetry.fppi}, /home/user/fprime/Svc/Seq/Seq.fpp, /home/user/fprime/Svc/CmdSequencer/docs/sdd.md.

## Key items

### Svc/Seq port + type definitions

Files: `Svc/Seq/Seq.fpp`, `Svc/Ports/FilePorts/FileDispatch.fpp`, `Svc/Sched/Sched.fpp`, `Svc/Ping/Ping.fpp`

struct Svc.SeqArgs { size: FwSizeType (u64); buffer: [SequenceArgumentsMaxSize] U8 } default {size=0, buffer=0}. SequenceArgumentsMaxSize = FW_CMD_ARG_BUFFER_MAX_SIZE(506) - sizeof(FwSizeStoreType)(2) - FileNameStringSize(240) - sizeof(U8)(1) - sizeof(FwSizeType)(8) = 255. FileNameStringSize = 240 (default/config/AcConstants.fpp:52). enum Svc.BlockState : U8 { BLOCK = 0, NO_BLOCK = 1 }. port Svc.CmdSeqIn(filename: string size 240, args: SeqArgs). port Svc.CmdSeqCancel() — no args. port Svc.FileDispatch(ref file_name: string size 240). port Svc.Sched(context: U32). port Svc.Ping(key: U32).

### CmdSequencer port interface

Files: `Svc/CmdSequencer/CmdSequencer.fpp`

ACTIVE component. Special: cmdIn (command recv), cmdRegOut (command reg), cmdResponseOut (command resp), logOut (event), tlmOut (telemetry), LogText (text event), timeCaller (time get). General ports — ASYNC INPUT: seqCancelIn: Svc.CmdSeqCancel; cmdResponseIn: Fw.CmdResponse(opcode:FwOpcodeType, cmdSeq:U32, response:Fw.CmdResponse); pingIn: Svc.Ping; seqRunIn: Svc.CmdSeqIn; seqDispatchIn: Svc.FileDispatch; schedIn: Svc.Sched. OUTPUT: pingOut: Svc.Ping; seqDone: Fw.CmdResponse; comCmdOut: Fw.Com(data: ComBuffer, context: U32); seqStartOut: Svc.CmdSeqIn. All arrays are size 1 (no array declarations). Ref topology (instances.fpp:56) base id 0x10006000, priority 20, default queue/stack; wired rateGroup2Comp.RateGroupMemberOut[0] -> cmdSeq.schedIn, cmdSeq.comCmdOut -> CmdDispatcher.seqCmdBuff, CmdDispatcher.seqCmdStatus -> cmdSeq.cmdResponseIn; RefTopology.cpp: cmdSeq.allocateBuffer(0, mallocator, 5*1024) at setup, deallocateBuffer at teardown.

### Component-declared enums (serialized in events)

Files: `Svc/CmdSequencer/CmdSequencer.fpp`

enum CmdSequencer_SeqMode : U8 { STEP = 0, AUTO = 1 }. enum CmdSequencer_FileReadStage : U8 { READ_HEADER=0, READ_HEADER_SIZE=1, DESER_SIZE=2, DESER_NUM_RECORDS=3, DESER_TIME_BASE=4, DESER_TIME_CONTEXT=5, READ_SEQ_CRC=6, READ_SEQ_DATA=7, READ_SEQ_DATA_SIZE=8 }. NOTE the SeqMode<->StepMode inversion: internal StepMode is {AUTO=0, MANUAL=1} but the reported enum is {STEP=0, AUTO=1}; CS_AUTO logs SeqMode::AUTO(1), CS_MANUAL logs SeqMode::STEP(0).

### Commands (all async, opcode = component base id + offset)

Files: `Svc/CmdSequencer/Commands.fppi`, `Svc/CmdSequencer/CmdSequencerImpl.cpp`

CS_RUN opcode 0, args (fileName: string size 240, block: Svc.BlockState U8). CS_VALIDATE opcode 1, args (fileName: string size 240). CS_CANCEL opcode 2, no args. CS_START opcode 3, no args. CS_STEP opcode 4, no args. CS_AUTO opcode 5, no args. CS_MANUAL opcode 6, no args. CS_JOIN_WAIT opcode 7, no args. Command-arg wire encoding is the usual FPP: string = [u16 length][bytes], enum BlockState = 1 byte.

### CS_RUN handler semantics

Files: `Svc/CmdSequencer/CmdSequencerImpl.cpp`

1) requireRunMode(STOPPED) — if not: if m_join_waiting emit CS_JoinWaitingNotComplete(id 24); (requireRunMode itself already emitted CS_InvalidMode id 11); cmdResponse EXECUTION_ERROR; return. 2) if block==BLOCK && stepMode==MANUAL: CS_InvalidMode(11) + EXECUTION_ERROR; return. 3) store m_blockState=block, m_cmdSeq, m_opCode. 4) loadFile(fileName); on failure reset m_blockState=NO_BLOCK, m_opCode=0, m_cmdSeq=0, cmdResponse EXECUTION_ERROR, return. 5) m_executedCount=0. 6) if stepMode==AUTO: m_runMode=RUNNING; tlmWrite CS_CurrentSequence(stringFileName); if seqStartOut connected emit seqStartOut_out(0, stringFileName, SeqArgs{0,0}); performCmd_Step(). 7) if m_blockState==NO_BLOCK: cmdResponse OK (BLOCK defers the response until sequenceComplete or performCmd_Cancel).

### CS_VALIDATE / CS_CANCEL / CS_START / CS_STEP / CS_AUTO / CS_MANUAL / CS_JOIN_WAIT

Files: `Svc/CmdSequencer/CmdSequencerImpl.cpp`

CS_VALIDATE: requireRunMode(STOPPED) else EXECUTION_ERROR; loadFile else EXECUTION_ERROR; then m_sequence->clear(); emit CS_SequenceValid(19); cmdResponse OK. CS_CANCEL: if RUNNING -> performCmd_Cancel(), CS_SequenceCanceled(1), ++m_cancelCmdCount, tlm CS_CancelCommands; else CS_NoSequenceActive(18); ALWAYS cmdResponse OK. CS_START: if !hasMoreRecords -> CS_NoSequenceActive(18) + EXECUTION_ERROR; if !requireRunMode(STOPPED) -> EXECUTION_ERROR; else m_blockState=NO_BLOCK, m_runMode=RUNNING, tlm CS_CurrentSequence, CS_CmdStarted(22), performCmd_Step(), then seqStartOut (if connected), cmdResponse OK. CS_STEP: requires runMode RUNNING (else CS_InvalidMode + EXECUTION_ERROR); requires stepMode==MANUAL else CS_InvalidMode(11)+EXECUTION_ERROR; requires hasMoreRecords else CS_NoSequenceActive(18)+EXECUTION_ERROR; performCmd_Step(); if m_runMode != STOPPED emit CS_CmdStepped(21, logFileName, m_executedCount); cmdResponse OK. CS_AUTO: requireRunMode(STOPPED) -> m_stepMode=AUTO, CS_ModeSwitched(17, SeqMode::AUTO=1), OK; else EXECUTION_ERROR. CS_MANUAL: requireRunMode(STOPPED) -> m_stepMode=MANUAL, CS_ModeSwitched(17, SeqMode::STEP=0), OK; else EXECUTION_ERROR. CS_JOIN_WAIT: if runMode != RUNNING -> CS_NoSequenceActive(18) + cmdResponse OK; else if (m_blockState==BLOCK || m_join_waiting) -> CS_JoinWaitingNotComplete(24) + EXECUTION_ERROR; else m_join_waiting=true, emit CS_JoinWaiting(23, logFileName, m_cmdSeq, m_opCode) using the PREVIOUS m_cmdSeq/m_opCode, then overwrite m_cmdSeq=cmdSeq, m_opCode=opCode and send NO response now.

### Port handlers

Files: `Svc/CmdSequencer/CmdSequencerImpl.cpp`

seqRunIn_handler(portNum, filename, args) — args IGNORED — calls doSequenceRun(filename). seqDispatchIn_handler(portNum, file_name) -> doSequenceRun(file_name). doSequenceRun: if stepMode==MANUAL -> CS_InvalidMode(11) + seqDone_out(0, 0, 0, EXECUTION_ERROR); return. if !requireRunMode(STOPPED) -> seqDone EXECUTION_ERROR; return. if filename != "": loadFile; on failure seqDone EXECUTION_ERROR + return. else if !hasMoreRecords -> CS_NoSequenceActive(18), error() (++errors + tlm), seqDone EXECUTION_ERROR, return. m_executedCount=0; if AUTO: m_runMode=RUNNING, tlm CS_CurrentSequence, seqStartOut (if connected), performCmd_Step(). Finally emit CS_PortSequenceStarted(15). NOTE seqDone_out is called UNCONDITIONALLY in the error paths of doSequenceRun (no isConnected check), unlike performCmd_Cancel/sequenceComplete which guard with isConnected_seqDone_OutputPort(0). seqCancelIn_handler: same body as CS_CANCEL minus the cmdResponse. pingIn_handler(portNum,key) -> pingOut_out(0,key). schedIn_handler(portNum, order): currTime=getTime(); if m_cmdTimer.isExpiredAt(currTime) { comCmdOut_out(0, m_record.m_command, 0); m_cmdTimer.clear(); setCmdTimeout(currTime); } else if m_cmdTimeoutTimer.isExpiredAt(getTime()) { CS_SequenceTimeout(20, logFileName, m_executedCount); performCmd_Cancel(); }. cmdResponseIn_handler(portNum, opcode, cmdSeq, response): if runMode==STOPPED -> CS_UnexpectedCompletion(16, opcode); else clear m_cmdTimeoutTimer; if response != OK -> commandError(m_executedCount, opcode, response.e) [CS_CommandError(10) + error()] then performCmd_Cancel(); else commandComplete(opcode) [CS_CommandComplete(8, file, m_executedCount, opcode); ++m_executedCount; ++m_totalExecutedCount; tlm CS_CommandsExecuted] then: AUTO -> if !hasMoreRecords {m_runMode=STOPPED; sequenceComplete();} else performCmd_Step(); MANUAL -> if !hasMoreRecords {m_runMode=STOPPED; sequenceComplete();} (otherwise wait for CS_STEP).

### performCmd_Step / performCmd_Cancel / sequenceComplete

Files: `Svc/CmdSequencer/CmdSequencerImpl.cpp`

performCmd_Step(): m_sequence->nextRecord(m_record) (ASSERTS on deserialize failure); then m_record.m_timeTag.setTimeBase(header.m_timeBase); setTimeContext(header.m_timeContext) — record time tags on the wire carry ONLY seconds/useconds; base+context come from the (already canonicalized) header. currentTime=getTime(). switch descriptor: END_OF_SEQUENCE -> m_runMode=STOPPED; sequenceComplete(). RELATIVE -> m_record.m_timeTag.add(currentTime.getSeconds(), currentTime.getUSeconds()) then fall into ABSOLUTE. ABSOLUTE -> if currentTime >= m_record.m_timeTag { comCmdOut_out(0, m_record.m_command, 0); setCmdTimeout(currentTime); } else { m_cmdTimer.set(m_record.m_timeTag); }. performCmd_Cancel(): m_sequence->reset(); m_runMode=STOPPED; m_cmdTimer.clear(); m_cmdTimeoutTimer.clear(); m_executedCount=0; if seqDone connected -> seqDone_out(0,0,0,EXECUTION_ERROR); if (m_blockState==BLOCK || m_join_waiting) { m_join_waiting=false; cmdResponse_out(m_opCode, m_cmdSeq, EXECUTION_ERROR); } m_blockState=NO_BLOCK. NOTE cancel calls reset() (rewinds deser) not clear(), so hasMoreRecords stays true and a later CS_START can restart the same sequence. sequenceComplete(): ++m_sequencesCompletedCount; m_sequence->clear(); CS_SequenceComplete(9); tlm CS_SequencesCompleted; m_executedCount=0; if seqDone connected -> seqDone_out(0,0,0,OK); if (BLOCK || m_join_waiting) cmdResponse_out(m_opCode, m_cmdSeq, OK); m_join_waiting=false; m_blockState=NO_BLOCK; tlmWrite_CS_CurrentSequence("<no seq>").

### Timeout mechanism (two Timers)

Files: `Svc/CmdSequencer/CmdSequencerImpl.hpp`, `Svc/CmdSequencer/CmdSequencerImpl.cpp`

class Timer { State {SET, CLEAR}; Fw::Time expirationTime; set(t){state=SET; exp=t;} clear(){state=CLEAR;} isExpiredAt(t) { if CLEAR return false; if Fw::Time::compare(exp, t)==GT return false; return true; } } — i.e. expired iff exp <= t, and INCOMPARABLE (different time base/context) counts as EXPIRED. Two instances: m_cmdTimer (pending future-time command dispatch, armed by performCmd_Step_ABSOLUTE when the tag is in the future) and m_cmdTimeoutTimer (command response watchdog). setCmdTimeout(currentTime): only if (m_timeout > 0 && stepMode == AUTO): expTime = currentTime; expTime.add(m_timeout, 0); m_cmdTimeoutTimer.set(expTime). m_timeout is U32 SECONDS, default 0 = disabled, set via public setTimeout(U32) at topology setup. Both timers are only evaluated inside schedIn_handler; the else-if means a due command dispatch preempts a timeout check on the same tick.

### FPrimeSequence load/validate path

Files: `Svc/CmdSequencer/FPrimeSequence.cpp`, `Svc/CmdSequencer/Sequence.cpp`

loadFile(name): FW_ASSERT(buffer addr != nullptr); setFileName(name) (sets m_fileName CmdStringArg, m_logFileName LogStringArg, m_stringFileName Fw::String); then readFile() AND validateCRC() AND m_header.validateTime(component) AND validateRecords() — short-circuit &&. readFile(): Os::File::open(name, OPEN_READ); OP_OK -> readOpenFile(); DOESNT_EXIST -> CS_FileNotFound(6); anything else -> CS_FileReadError(2); always close(). readOpenFile(): crc.init(); readHeader(); if ok crc.update(buffAddr, 11); then deserializeHeader() && readRecordsAndCRC() && extractCRC(); if ok crc.update(buffAddr, buffer.getSize()) where getSize() is now fileSize-4. readHeader(): readLen=11; FW_ASSERT(capacity >= 11); file.read(buffAddr, readLen); status!=OP_OK -> CS_FileInvalid(3, READ_HEADER=0, fileStatus); readLen != 11 -> CS_FileInvalid(3, READ_HEADER_SIZE=1, readLen); then setBuffLen(11). deserializeHeader(): U32 fileSize (fail -> CS_FileInvalid stage DESER_SIZE=2); if fileSize > buffer.getCapacity() -> CS_FileSizeError(5, fileSize); U32 numRecords (DESER_NUM_RECORDS=3); TimeBase U16 (DESER_TIME_BASE=4); FwTimeContextStoreType U8 (DESER_TIME_CONTEXT=5). readRecordsAndCRC(): readLen=fileSize; file.read(buffAddr, readLen) — OVERWRITES the header bytes in the buffer; !=OP_OK -> CS_FileInvalid(READ_SEQ_DATA=7, fileStatus); readLen != fileSize -> CS_FileInvalid(READ_SEQ_DATA_SIZE=8, readLen); setBuffLen(fileSize). extractCRC(): if fileSize < 4 -> CS_FileInvalid(READ_SEQ_CRC=6, fileSize); dataSize=fileSize-4; stored CRC = U32 BE at buffAddr[dataSize..]; setBuffLen(dataSize). Header::validateTime(component): validTime=component.getTime(); if (header.timeBase != validTime.getTimeBase() && header.timeBase != TB_DONT_CARE(0xFFFF)) -> CS_TimeBaseMismatch(13, current, seq) return false; if (header.timeContext != validTime.getContext() && header.timeContext != FW_CONTEXT_DONT_CARE(0xFF)) -> CS_TimeContextMismatch(14, current, seq) return false; then CANONICALIZE header.timeBase=validTimeBase, header.timeContext=validContext. validateRecords(): if numRecords==0 -> CS_NoRecords(25) return false; loop recordNumber 0..numRecords-1 deserializeRecord -> on !=FW_SERIALIZE_OK emit CS_RecordInvalid(4, recordNumber, status) return false; if getDeserializeSizeLeft() > 0 -> CS_RecordMismatch(12, numRecords, leftover) return false (NOTE: recordMismatch does NOT bump the error counter — explicit TODO in Events.cpp); finally buffer.resetDeser(). Buffer allocation: Sequence::allocateBuffer(id, allocator, bytes) FW_ASSERTs bytes >= 11 and does m_buffer.setExtBuffer(allocator.allocate(id,bytes,recoverable), bytes). deallocateBuffer(allocator) -> allocator.deallocate(id, addr); buffer.clear(). Component-level loadFile(): on success CS_SequenceLoaded(0) + ++m_loadCmdCount + tlm CS_LoadCommands; on FAILURE it calls m_sequence->clear() (resetSer, so hasMoreRecords becomes false) — this is a deliberate fix so a partial load cannot be started. Os::ValidateFile is #included in CmdSequencerImpl.hpp but NEVER used by CmdSequencer; only AMPCSSequence uses a sidecar '<file>.CRC32' + Os::FileSystem::getFileSize.

### Record deserialization

Files: `Svc/CmdSequencer/FPrimeSequence.cpp`

deserializeRecord(record): deserializeDescriptor -> if OK and descriptor==END_OF_SEQUENCE return OK immediately (no further fields consumed); else deserializeTimeTag, deserializeRecordSize, copyCommand. deserializeDescriptor: read U8 descEntry; if descEntry > 2 return FW_DESERIALIZE_FORMAT_ERROR (=4); cast. deserializeTimeTag: U32 seconds then U32 useconds; timeTag.set(seconds, useconds) (base/context untouched). deserializeRecordSize: U32 recordSize; if recordSize > buffer.getDeserializeSizeLeft() -> FW_DESERIALIZE_SIZE_MISMATCH (=5); if recordSize + sizeof(FwPacketDescriptorType)(2) > FW_COM_BUFFER_MAX_SIZE(512) -> FW_DESERIALIZE_SIZE_MISMATCH. copyCommand(comBuffer, recordSize): comBuffer.resetSer(); setBuffLen(recordSize) (asserts); buffer.deserializeTo(comBuffer.getBuffAddr(), size, OMIT_LENGTH) — raw byte copy, no length prefix. hasMoreRecords() == buffer.getDeserializeSizeLeft() > 0. nextRecord() FW_ASSERTs on any non-OK status. reset() = resetDeser (deserLoc=0). clear() = resetSer (serLoc=0 and deserLoc=0).

### Telemetry channels

Files: `Svc/CmdSequencer/Telemetry.fppi`

CS_LoadCommands: U32 id 0 (incremented on every successful loadFile). CS_CancelCommands: U32 id 1. CS_Errors: U32 id 2 (bumped by every Events:: helper except recordMismatch, plus doSequenceRun's empty-buffer path and commandError). CS_CommandsExecuted: U32 id 3 (m_totalExecutedCount across all sequences). CS_SequencesCompleted: U32 id 4. CS_CurrentSequence: string size 240 id 5, UPDATE ON CHANGE (written with the sequence file name on start, and with the literal "<no seq>" on sequenceComplete).

### Events (id, severity, args in order)

Files: `Svc/CmdSequencer/Events.fppi`, `Svc/CmdSequencer/Events.cpp`

All fileName/filename args are `string size 60` (LogStringArg), NOT 240 — the 240-char name is truncated for events. 0 CS_SequenceLoaded(fileName) ACTIVITY_LO. 1 CS_SequenceCanceled(fileName) ACTIVITY_HI. 2 CS_FileReadError(fileName) WARNING_HI. 3 CS_FileInvalid(fileName, stage: FileReadStage U8, error: I32) WARNING_HI. 4 CS_RecordInvalid(fileName, recordNumber: U32, error: I32) WARNING_HI. 5 CS_FileSizeError(fileName, size: U32) WARNING_HI. 6 CS_FileNotFound(fileName) WARNING_HI. 7 CS_FileCrcFailure(fileName, storedCRC: U32, computedCRC: U32) WARNING_HI. 8 CS_CommandComplete(fileName, recordNumber: U32, opCode: FwOpcodeType U32) ACTIVITY_LO. 9 CS_SequenceComplete(fileName) ACTIVITY_HI. 10 CS_CommandError(fileName, recordNumber: U32, opCode: FwOpcodeType, errorStatus: U32) WARNING_HI. 11 CS_InvalidMode() WARNING_HI. 12 CS_RecordMismatch(fileName, header_records: U32, extra_bytes: U32) WARNING_HI. 13 CS_TimeBaseMismatch(fileName, time_base: U16, seq_time_base: U16) WARNING_HI. 14 CS_TimeContextMismatch(fileName, currTimeBase: U8, seqTimeBase: U8) WARNING_HI. 15 CS_PortSequenceStarted(filename) ACTIVITY_HI. 16 CS_UnexpectedCompletion(opcode: FwOpcodeType) WARNING_HI. 17 CS_ModeSwitched(mode: SeqMode U8) ACTIVITY_HI. 18 CS_NoSequenceActive() WARNING_LO. 19 CS_SequenceValid(filename) ACTIVITY_HI. 20 CS_SequenceTimeout(filename, command: U32) WARNING_HI. 21 CS_CmdStepped(filename, command: U32) ACTIVITY_HI. 22 CS_CmdStarted(filename) ACTIVITY_HI. 23 CS_JoinWaiting(filename, recordNumber: U32, opCode: FwOpcodeType) ACTIVITY_HI. 24 CS_JoinWaitingNotComplete() WARNING_HI. 25 CS_NoRecords(fileName) WARNING_LO. Every opcode reported in an event goes through CmdDispatcherCfg::getEventOpcode(op) — with IncludeCommandOpcodesInEvents=true (default) it is the identity; when false it becomes std::numeric_limits<FwOpcodeType>::max() (0xFFFFFFFF). Applies to CS_CommandComplete, CS_CommandError, CS_UnexpectedCompletion, CS_JoinWaiting.

### Component state variables (constructor defaults)

Files: `Svc/CmdSequencer/CmdSequencerImpl.cpp`

m_sequence=&m_FPrimeSequence; m_loadCmdCount=0; m_cancelCmdCount=0; m_errorCount=0; m_runMode=STOPPED; m_stepMode=AUTO; m_executedCount=0; m_totalExecutedCount=0; m_sequencesCompletedCount=0; m_timeout=0; m_blockState=NO_BLOCK; m_opCode=0; m_cmdSeq=0; m_join_waiting=false; m_cmdTimer/m_cmdTimeoutTimer CLEAR; NO_SEQ = "<no seq>". Public setup API called by topology BEFORE the task is started, in this order: setSequenceFormat(optional) -> allocateBuffer(id, allocator, bytes) -> setTimeout(optional) -> loadSequence(optional, needs event ports wired; FW_ASSERTs m_runMode==STOPPED and clears the sequence if loadFile fails).

## Wire formats

### F Prime sequence file — overall layout

[Header: 11 bytes][Records: fileSize-4 bytes][CRC: 4 bytes]. Total file length on disk = 11 + fileSize. Everything big-endian. The reader does exactly two reads: 11 bytes, then fileSize bytes; extra trailing bytes beyond 11+fileSize are silently ignored, and a short file yields CS_FileInvalid(READ_SEQ_DATA_SIZE=8).

### Header (11 bytes, offset 0)

off 0: fileSize U32 BE — number of bytes AFTER the header, INCLUDING the trailing 4-byte CRC (i.e. records_len + 4). Rejected with CS_FileSizeError if > buffer capacity. off 4: numRecords U32 BE. off 8: timeBase U16 BE (Fw TimeBase enum: TB_NONE=0, TB_PROC_TIME=1, TB_WORKSTATION_TIME=2, TB_SC_TIME=3, TB_DONT_CARE=0xFFFF). off 10: timeContext U8 (0xFF = FW_CONTEXT_DONT_CARE). SERIALIZED_SIZE = sizeof(U32)+sizeof(U32)+sizeof(FwTimeBaseStoreType)+sizeof(FwTimeContextStoreType) = 4+4+2+1 = 11.

### Record — command (ABSOLUTE or RELATIVE)

[descriptor U8: 0=ABSOLUTE, 1=RELATIVE][seconds U32 BE][useconds U32 BE][recordSize U32 BE][command bytes, exactly recordSize bytes]. The command bytes are a complete F Prime command com packet: [FwPacketDescriptorType U16 BE = 0x0000 FW_PACKET_COMMAND][FwOpcodeType U32 BE][serialized args...]. So recordSize == 2 + 4 + args_len (>= 6 for a no-arg command). Standard fixed overhead per command record = 1+4+4+4 = 13 bytes. The time tag on the wire carries NO time base or context — those come from the header and are stamped onto the record at step time.

### Record — end of sequence

[descriptor U8 = 2 (END_OF_SEQUENCE)] and nothing else — 1 byte total. An EOS record IS counted in numRecords by the generator (the file may also simply end after the last command record; both work, since hasMoreRecords is purely 'bytes left').

### CRC (last 4 bytes)

U32 BE, Utils::Hash CRC-32 (standard reflected CRC-32/ISO-HDLC: poly 0x04C11DB7 reflected 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF — already ported as fprime_utils::Hash). It covers the 11 header bytes followed by the records bytes (fileSize - 4), i.e. every byte of the file EXCEPT the 4 CRC bytes themselves. In C++ this is two update() calls over the same buffer address (header first, then the records that overwrite it). Mismatch -> CS_FileCrcFailure(7, stored, computed) + error().

### GDS compatibility (fprime-seqgen)

The GDS `fprime-seqgen` tool compiles a text .seq (lines `R HH:MM:SS[.fff] MNEM args` / `A YYYY-DDDTHH:MM:SS[.fff] MNEM args`, `;` comments, args include simple values, `[a,b]` arrays and `{k: v}` structs) into exactly this binary layout using the topology JSON dictionary for mnemonic->opcode and arg serialization. Output defaults to `<name>.bin`. Relative times cannot exceed 24 hours. The header time base is whatever the sequence was built for; the flight side refuses to run a sequence whose time base/context differs from the live one unless the file uses the don't-care sentinels (0xFFFF / 0xFF). Any Rust reader must accept byte-for-byte what seqgen emits, including fileSize counting the trailing CRC.

## Threading / concurrency

ACTIVE component with one task and one message queue. Async input ports (seqCancelIn, cmdResponseIn, pingIn, seqRunIn, seqDispatchIn, schedIn) and cmdIn (async commands) all serialize onto that queue, so every handler — command handlers, port handlers, load/CRC/validate, and the sequence buffer — runs on the single component thread with no additional locking. `getTime()` is a synchronous output port call (timeCaller) made on the component thread. All output ports (comCmdOut, seqDone, seqStartOut, pingOut, cmdResponseOut, logOut, tlmOut) are invoked synchronously from that thread. There are no mutexes, no guarded ports, and no shared state with other threads. The sequence buffer is a single `Fw::ExternalSerializeBuffer` over memory handed in at init by a `Fw::MemAllocator` (Ref uses MallocAllocator, 5*1024 bytes) and never reallocated; there is no steady-state allocation. Blocking file I/O (Os::File open/read/close) happens inline on the component thread during load. The public setup methods (setTimeout, setSequenceFormat, allocateBuffer, loadSequence, deallocateBuffer) must be called before the task is spawned / after it stops.

## Porting notes

Crate placement: `crates/fprime-svc/src/cmd_sequencer.rs`, following the hand-written active-component pattern used by `cmd_dispatcher.rs`/`com_queue.rs` (ActiveBase + CmdGlue + EventGlue + TlmGlue, `PortRef` input factories, `OutputPort<dyn ...>` outputs). Sequence file record/header parsing belongs in a private submodule (`cmd_sequencer/fprime_sequence.rs`) so an alternative format can be swapped later.

New port traits needed in fprime-svc (Svc/Seq has no Rust equivalent yet): `SeqArgs { size: FwSizeType, buffer: [u8; 255] }` with Serialize/Deserialize (u64 BE then 255 raw bytes; keep the `[u8; N]` FPP array serialization = elements back to back, no length prefix); `#[repr(u8)] enum BlockState { Block = 0, NoBlock = 1 }` with TryFrom<u8> and u8-width Serialize/Deserialize (match the existing enum conventions in api-notes.md line 59-60); `trait CmdSeqInPort { fn invoke(&self, port_num, filename: &FwString<240>, args: &SeqArgs); }`; `trait CmdSeqCancelPort { fn invoke(&self, port_num); }`; `trait FileDispatchPort { fn invoke(&self, port_num, file_name: &mut FwString<240>); }`. `SchedPort`, `PingPort`, `CmdResponsePort`, `ComPort` already exist.

Reuse directly: `fprime_utils::Hash` (`new/init/update/finalize`) is byte-identical to `Utils::Hash` — use `update(&header_bytes)` then `update(&record_bytes)`, compare `finalize()` to the stored u32. `fprime_fw::Time` has `compare`, `add`, `add_duration`, `set`, `set_time_base`, `set_time_context`, `get_time_base`, `get_context` and `TimeBase::TbDontCare = 0xFFFF` — the Timer's `isExpiredAt` must be written as `compare(exp, t) != TimeComparison::GT` so that INCOMPARABLE counts as expired (do NOT use PartialOrd, which returns false for Incomparable). `fprime_fw::ComBuffer` (LinearBuffer<512>) is the command carrier; build it from the raw record bytes with `set_buff`/`ExtBuf::with_len` semantics rather than field-wise serialization (C++ does a raw OMIT_LENGTH copy).

Buffer model: replace `Fw::MemAllocator` + `ExternalSerializeBuffer` with an owned `Box<[u8]>` created in `allocate_buffer(bytes)` (fw_assert bytes >= 11) and dropped in `deallocate_buffer()`, plus explicit `ser_loc`/`deser_loc` cursors so `has_more_records() == ser_loc - deser_loc > 0`, `reset()` sets deser_loc=0, `clear()` sets both to 0, `set_buff_len(n)` sets ser_loc=n and deser_loc=0. Reproduce these three cursor operations exactly — the state machine leans on their differences (cancel uses reset, complete/validate/load-failure use clear).

File I/O: `fprime_os::File` open(OPEN_READ)/read(&mut [u8]) -> (Status, bytes_read)/close. Map `Status::DoesntExist` to CS_FileNotFound and everything else non-OK to CS_FileReadError on open; on read, pass the raw Status discriminant as the `error: I32` argument of CS_FileInvalid (Os::File::Status ordinals: OP_OK=0, DOESNT_EXIST=1, NO_SPACE=2, NO_PERMISSION=3, BAD_SIZE=4, NOT_OPENED=5, FILE_EXISTS=6, NOT_SUPPORTED=7, INVALID_MODE=8, INVALID_ARGUMENT=9, NO_MORE_RESOURCES=10, OTHER_ERROR=11, OUTSIDE_SANDBOX=12). For deserialize failures the `error` argument is the SerializeStatus discriminant (FormatError=1, NoRoomLeft=2, DeserBufferEmpty=3, DeserFormatError=4, DeserSizeMismatch=5) — reuse `SerializeStatus as i32`.

Do NOT port `Os::ValidateFile` (it is a dead include) or `formats/AMPCSSequence` (optional alternate format, needs a `.CRC32` sidecar). Keep the `Sequence` trait (`load_file`, `has_more_records`, `next_record`, `reset`, `clear`, `header()`) so `set_sequence_format` is expressible, but ship only `FPrimeSequence`.

Do NOT port the `#[repr]` inversion away: keep `StepMode { Auto = 0, Manual = 1 }` internal and a separate wire enum `SeqMode { Step = 0, Auto = 1 }` for CS_ModeSwitched.

Follow the fprime-svc convention of declaring event ids/opcodes/channel ids as module consts (`OPCODE_CS_RUN = 0 ... OPCODE_CS_JOIN_WAIT = 7`, `EVENTID_CS_SEQUENCE_LOADED = 0 ... EVENTID_CS_NO_RECORDS = 25`, `CHANID_CS_LOAD_COMMANDS = 0 ... CHANID_CS_CURRENT_SEQUENCE = 5`). CS_CurrentSequence is update-on-change — use the existing TlmGlue on-change helper (cmd_dispatcher already does this for its run tlm).

Queue message size: the largest async envelope is seqRunIn — 6 (msg_type i32 + port_num i16) + 2+240 (string) + 8+255 (SeqArgs) = 511; round to a const `cmd_sequencer::QUEUE_MSG_SIZE` covering also seqDispatchIn (248) and cmdResponseIn (6+4+4+1=15).

## Rust feasibility (safe, zero-dependency std)

Everything here is portable in safe, zero-dependency std Rust. No ioctl, no zlib, no platform-specific calls. Specifics: (1) `Fw::MemAllocator`/`ExternalSerializeBuffer` raw-pointer aliasing becomes an owned `Box<[u8]>` with explicit cursors — behaviorally identical because CmdSequencer is the sole owner and the allocator identity is only used for the matching deallocate; drop the `identifier`/`recoverable` parameters (or keep `identifier` as an inert field for API parity). (2) The C++ reader reads the records *on top of* the header in the same buffer and CRCs the header from a stale view before the overwrite — in Rust just CRC the 11 header bytes as they are read, then CRC the record slice; the result is bit-identical, no aliasing needed. (3) `FW_ASSERT` paths (nextRecord on a bad deserialize, allocateBuffer size, setBuffLen) map onto the existing `fw_assert!`. (4) `Utils::Hash` needs no C dependency — `fprime_utils::Hash` is already the table-driven CRC-32. (5) The AMPCS alternate format needs `Os::FileSystem::getFileSize` and a `.CRC32` sidecar; `fprime-os` has filesystem support and `fprime-utils::crc_checker` already handles sidecars, so it is feasible but explicitly out of scope. (6) Nothing requires `unsafe`; `#![forbid(unsafe_code)]` holds.

## Gotchas

- `fileSize` in the header INCLUDES the trailing 4-byte CRC. Records length = fileSize - 4, total file = 11 + fileSize. Getting this off by 4 breaks every GDS-produced file.
- The CRC covers header(11) + records(fileSize-4) and NOT the stored CRC bytes. It is computed in two chunks over the same buffer address in C++ — do not accidentally CRC the whole 11+fileSize span.
- Record time tags carry ONLY seconds+useconds on the wire (8 bytes). timeBase/timeContext are stamped from the (canonicalized) header inside `performCmd_Step`, AFTER `nextRecord`. Forgetting this makes every `Fw::Time::compare` return INCOMPARABLE and the sequencer fires everything immediately.
- `Header::validateTime` CANONICALIZES the header: after a successful validate, header.timeBase/timeContext hold the LIVE values, not the file's don't-care sentinels. This is what makes 0xFFFF/0xFF sequences work.
- `Timer::isExpiredAt` treats INCOMPARABLE (mismatched time base/context) as EXPIRED, because it only rejects the GT case. Port it as `compare(exp, t) != GT`, never as `exp <= t` via PartialOrd.
- `SeqMode` (event enum) is inverted relative to the internal `StepMode`: CS_AUTO logs SeqMode::AUTO=1, CS_MANUAL logs SeqMode::STEP=0.
- `deserializeRecordSize` rejects `recordSize + 2 > 512` — this is over-strict by 2 bytes because `recordSize` ALREADY includes the 2-byte packet descriptor. Port the check verbatim (max usable recordSize = 510) or GDS-accepted files will diverge.
- `performCmd_Cancel` calls `reset()` (rewind), not `clear()`; the sequence stays loaded and `CS_START` can rerun it from the top. `sequenceComplete`, `CS_VALIDATE`, and the load-failure path call `clear()`.
- Component-level `loadFile` calls `m_sequence->clear()` on EVERY failure — a deliberate fix so a partially-populated buffer cannot make `hasMoreRecords()` true and drive `nextRecord` into its FW_ASSERT.
- `CS_RecordMismatch` (id 12) is the ONLY sequence event that does NOT increment `CS_Errors` (explicit TODO in Events.cpp).
- `CS_JOIN_WAIT` emits `CS_JoinWaiting` with the PREVIOUS `m_cmdSeq`/`m_opCode` (from the in-flight CS_RUN) before overwriting them with its own — the event's recordNumber/opCode fields are the earlier command's, not the join's.
- BLOCK-mode CS_RUN and CS_JOIN_WAIT both defer their command response; `m_opCode`/`m_cmdSeq` is a single slot, hence the explicit guards that reject a second deferred caller with CS_JoinWaitingNotComplete(24). CS_RUN clears the slot to (NO_BLOCK,0,0) when the load fails so a later port-driven run cannot emit a duplicate response.
- `doSequenceRun` (seqRunIn / seqDispatchIn) calls `seqDone_out(0,0,0,EXECUTION_ERROR)` WITHOUT an isConnected guard on its error paths, while `performCmd_Cancel`/`sequenceComplete` DO guard. Reproduce the asymmetry (in Rust an unconnected OutputPort asserts).
- `seqRunIn`'s `args: SeqArgs` is explicitly ignored (`(void)args`). `seqStartOut` is always emitted with `SeqArgs{size: 0, buffer: all-zero}`.
- `schedIn` uses `else if`: a due timed command dispatch suppresses the timeout check on that same tick. It also calls `getTime()` a second time for the timeout branch.
- `setCmdTimeout` arms the watchdog only when `m_timeout > 0` AND stepMode == AUTO; in MANUAL mode commands never time out.
- In MANUAL mode, `cmdResponseIn` with OK does NOT advance the sequence — it only bumps counters and completes the sequence if no records remain; the next record waits for CS_STEP.
- Event filename args are `string size 60` while command/telemetry/port filenames are `string size 240` (FileNameStringSize) — names are truncated for events. `CS_CurrentSequence` telemetry uses the full 240.
- An END_OF_SEQUENCE record is 1 byte total; `deserializeRecord` returns immediately and consumes no time tag or size. `hasMoreRecords()` is purely 'bytes left in buffer', so a file whose last record is a command (no EOS) completes via the `!hasMoreRecords` branch in `cmdResponseIn` instead of the END_OF_SEQUENCE branch.
- `numRecords == 0` is rejected with CS_NoRecords(25) even if the byte stream is otherwise well-formed.
- `Os::ValidateFile` is included in CmdSequencerImpl.hpp but never used — do not port a validate-file dependency.

