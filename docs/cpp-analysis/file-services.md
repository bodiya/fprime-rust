# Svc file transfer services: FileUplink, FileDownlink, FileDownlinkPorts, FileManager (+ Fw/FilePacket, CFDP/Checksum, Os/SandboxedFile, Os/FilePathUtils)

> Analysis of the C++ F Prime implementation (github.com/nasa/fprime) produced to guide this Rust port.
> File paths refer to the C++ tree.

## Overview

Three active components move files over the com link using Fw::FilePacket (START/DATA/END/CANCEL) wrapped in a FwPacketDescriptorType (U16 big-endian) = FW_PACKET_FILE = 0x0003 prefix. FileUplink consumes Fw::Buffer on an async bufferSendIn, decodes one FilePacket per buffer, drives a two-state (START/DATA) receive machine writing to a sandboxed Os file, verifies the CFDP checksum on END and announces the file. FileDownlink is a queued producer: SendFile/SendPartial commands and a guarded SendFile port push FileEntry records into an Os::Queue; a Run (Sched) tick pops one and drives a five-state (IDLE/DOWNLINK/CANCEL/WAIT/COOLDOWN) machine, sending exactly one packet then blocking in WAIT until that exact buffer comes back on bufferReturn (credit-1 flow control). It never allocates: two internal static byte arrays (FILE_PACKET, CANCEL_PACKET) of FILEDOWNLINK_INTERNAL_BUFFER_SIZE = FW_FILE_BUFFER_MAX_SIZE = FW_COM_BUFFER_MAX_SIZE = 512 bytes, identified by the Fw::Buffer context word. FileManager executes 9 async filesystem commands, plus two rate-group-paced state machines (directory listing, data-product generation) driven by a sync schedIn that hops onto the component thread through an `internal port run drop`. FilePacket byte layouts and the CFDP checksum are already documented in docs/cpp-analysis/utils-misc.md §"FilePacket family" and §"Fw::FilePacket + CFDP::Checksum" — reuse verbatim, do not re-derive.

## Key items

### Svc::FileUplink — ports

Files: `Svc/FileUplink/FileUplink.fpp`, `Svc/Ports/FilePorts/FileAnnounce.fpp`

active component. bufferSendIn: async input Fw.BufferSend (array 1). bufferSendOut: output Fw.BufferSend (buffer return/deallocate). pingIn: async input Svc.Ping; pingOut: output Svc.Ping (echo key). fileAnnounce: output Svc.FileAnnounce — port FileAnnounce(ref file_name: string size FileNameStringSize=240); emitted only if isConnected. Role ports: time get timeCaller, telemetry tlmOut, event eventOut, text event LogText. NO commands. Topology (FileHandling): base id 0x05000000+0x00000, queue 10, stack 64KiB, priority 24.

### Svc::FileUplink — telemetry

Files: `Svc/FileUplink/Telemetry.fppi`

FilesReceived U32 id 0; PacketsReceived U32 id 1; Warnings U32 id 2; FilesReceivedFailed U32 id 3. All are monotone counters written on every increment (counter is incremented then written). Warnings counter is bumped by EVERY warning helper.

### Svc::FileUplink — events (id / severity / args / throttle)

Files: `Svc/FileUplink/Events.fppi`, `Svc/FileUplink/Warnings.cpp`

0 BadChecksum(fileName: string 40, computed: U32, read: U32) WARNING_HI. 1 FileOpenError(fileName: string 40) WARNING_HI. 2 FileReceived(fileName: string 40) ACTIVITY_HI. 3 FileWriteError(fileName: string 40) WARNING_HI throttle 5. 4 InvalidReceiveMode(packetType: FwPacketDescriptorType(U16), mode: U32) WARNING_HI throttle 5 — packetType is FilePacket::Type cast (T_START=0,T_DATA=1,T_END=2,T_CANCEL=3), mode is current m_receiveMode (START=0, DATA=1). 5 PacketOutOfBounds(packetIndex: U32, fileName: string 40) WARNING_HI throttle 5. 6 PacketOutOfOrder(packetIndex: U32, lastPacketIndex: U32) WARNING_HI throttle 20. 7 PacketDuplicate(packetIndex: U32) WARNING_HI throttle 20. 8 UplinkCanceled() ACTIVITY_HI. 9 DecodeError(status: I32) WARNING_HI. 10 InvalidPacketReceived(packetType: FwPacketDescriptorType) WARNING_HI.

### Svc::FileUplink — bufferSendIn state machine

Files: `Svc/FileUplink/FileUplink.cpp`, `Svc/FileUplink/File.cpp`, `Svc/FileUplink/docs/sdd.md`

State: m_receiveMode {START=0 (init), DATA=1}, m_lastSequenceIndex: U32 (init 0), m_lastPacketWriteStatus: Os::File::Status (init MAX_STATUS=13), m_file{size:U32, name:LogStringArg, osFile:SandboxedFile, m_checksum:CFDP::Checksum}.
bufferSendIn_handler: (1) if buffer.size < 2 -> log InvalidPacketReceived(FW_PACKET_UNKNOWN=0x00FF), bufferSendOut_out(0,buffer), return. (2) deserialize U16 packetType from buffer start; FW_ASSERT status==OK. (3) if packetType != 0x0003 -> log InvalidPacketReceived(packetType), return buffer, return. (4) build sub-buffer at data+2, size-2; FilePacket::fromBuffer; on != FW_SERIALIZE_OK log DecodeError(status as I32). (5) else dispatch on header type; default -> FW_ASSERT(false). (6) ALWAYS bufferSendOut_out(0, buffer) at the end (even on decode error).
handleStartPacket: clear throttles for FileWriteError, InvalidReceiveMode, PacketOutOfBounds, PacketOutOfOrder (NOT PacketDuplicate); packetsReceived++; if mode!=START -> osFile.close() + warning invalidReceiveMode(T_START); then File::open(startPacket): copy destinationPath (len>=256 -> BAD_SIZE), set name/size, reset checksum, osFile.open(path, OPEN_WRITE). If OP_OK -> goToDataMode(); else warning fileOpen(name) + goToStartMode().
handleDataPacket: packetsReceived++; if mode!=DATA -> warning invalidReceiveMode(T_DATA) and RETURN (mode unchanged — contradicts sdd which says go to START). seq = header.sequenceIndex. If m_lastPacketWriteStatus==OP_OK AND seq==m_lastSequenceIndex -> warning packetDuplicate(seq), return (skip). Then checkSequenceIndex(seq): if seq != m_lastSequenceIndex+1 -> warning packetOutOfOrder(seq, m_lastSequenceIndex); ALWAYS m_lastSequenceIndex = seq. Bounds: if (U32::MAX - byteOffset < dataSize) || (byteOffset+dataSize > m_file.size) -> warning packetOutOfBounds(seq, name), return (m_lastPacketWriteStatus NOT updated). FW_ASSERT(data != null). File::write(data, byteOffset, dataSize): seek(byteOffset, ABSOLUTE); on error return status; write(data,len,NO_WAIT); on error return status; if written != len return NO_SPACE(2); else checksum.update(data, byteOffset, len) and return OP_OK. If status != OP_OK -> warning fileWrite(name). m_lastPacketWriteStatus = status.
handleEndPacket: packetsReceived++; if mode==DATA: checkSequenceIndex(end.sequenceIndex); compareChecksums: computed = file checksum, stored = end packet checksum; equal -> filesReceived++ , log ACTIVITY_HI FileReceived(name), and if fileAnnounce connected fileAnnounce_out(0, name). Not equal -> warning badChecksum(computed, stored) (issued inside compareChecksums) then filesReceivedFailed++ , NO FileReceived event, NO announce. If mode != DATA -> warning invalidReceiveMode(T_END). ALWAYS goToStartMode() afterwards.
handleCancelPacket: packetsReceived++; log ACTIVITY_HI UplinkCanceled; goToStartMode() (no mode check, always closes file).
goToStartMode(): osFile.close(); mode=START; lastSequenceIndex=0; lastPacketWriteStatus=MAX_STATUS. goToDataMode(): mode=DATA; lastSequenceIndex=0; lastPacketWriteStatus=MAX_STATUS (does NOT close file).
configure(dir) delegates to SandboxedFile::configure.

### Svc::FileDownlink — ports

Files: `Svc/FileDownlink/FileDownlink.fpp`, `Svc/FileDownlinkPorts/FileDownlinkPorts.fpp`

active component. Run: async input Svc.Sched (arg context: U32). SendFile: GUARDED (sync, mutex) input Svc.SendFileRequest -> returns Svc.SendFileResponse. FileComplete: output array [FileDownCompletePorts = 1] Svc.SendFileComplete(resp: SendFileResponse). bufferReturn: async input Fw.BufferSend. bufferSendOut: output Fw.BufferSend. pingIn async / pingOut output Svc.Ping. Role ports: timeCaller, cmdRegOut, cmdIn, cmdResponseOut, eventOut, textEventOut, tlmOut.
Svc.SendFileStatus : U8 { STATUS_OK=0, STATUS_ERROR=1, STATUS_INVALID=2, STATUS_BUSY=3 }. Svc.SendFileResponse { status: SendFileStatus (U8), context: U32 } — 5 bytes on the wire. SendFileRequest(sourceFileName: string 100, destFileName: string 100, offset: U32, length: U32) -> SendFileResponse. Topology: base id 0x05001000, queue 10, stack 64KiB, priority 23, configure(cooldown=1000ms, cycleTime=1000ms, fileQueueDepth=10).

### Svc::FileDownlink — commands / telemetry / events

Files: `Svc/FileDownlink/Commands.fppi`, `Svc/FileDownlink/Telemetry.fppi`, `Svc/FileDownlink/Events.fppi`

Commands (all async except behavior of Cancel which just flips a mutexed mode): 0x00 SendFile(sourceFileName: string 100, destFileName: string 100); 0x01 Cancel(); 0x02 SendPartial(sourceFileName: string 100, destFileName: string 100, startOffset: U32, length: U32) — length 0 = to end of file.
Telemetry: FilesSent U32 id 0x00; PacketsSent U32 id 0x01; Warnings U32 id 0x02.
Events: 0x00 FileOpenError(fileName: string 100) WARNING_HI; 0x01 FileReadError(fileName: string 100, status: I32) WARNING_HI; 0x02 FileSent(sourceFileName, destFileName: string 100) ACTIVITY_HI; 0x03 DownlinkCanceled(source, dest) ACTIVITY_HI; (0x04 unused gap) 0x05 DownlinkPartialWarning(startOffset: U32, length: U32, filesize: U32, sourceFileName: string 100, destFileName: string 100) WARNING_LO; 0x06 DownlinkPartialFail(sourceFileName, destFileName, startOffset: U32, filesize: U32) WARNING_HI; 0x07 SendDataFail(sourceFileName: string 100, byteOffset: U32) WARNING_HI; 0x08 SendStarted(fileSize: U32, sourceFileName, destFileName) ACTIVITY_HI; 0x09 DownlinkZeroSizeFile(sourceFileName) WARNING_HI; 0x10 FilenameSourceOverflow() WARNING_HI; 0x11 FilenameDestinationOverflow() WARNING_HI; 0x12 SourceOutOfSandbox(fileName: string 100) WARNING_HI. No throttles anywhere. Warnings tlm counter is bumped only by the four Warnings:: helpers (fileOpenError, fileRead, zeroSize, sourceOutOfSandbox) — the partial/senddata/overflow events do NOT bump it.

### Svc::FileDownlink — request queueing and responses

Files: `Svc/FileDownlink/FileDownlink.cpp`

FileEntry POD {srcFilename: Fw::FileNameString(240), destFilename: Fw::FileNameString(240), offset: U32, length: U32, source: CallerSource{COMMAND=0, PORT=1}, opCode: FwOpcodeType, cmdSeq: U32, context: U32}. Sent raw (memcpy of the struct) into Os::Queue created in configure(): m_fileQueue.create(instance, "fileDownlinkQueue", depth=fileQueueDepth, msgSize=sizeof(FileEntry)), priority 0, NONBLOCKING.
SendFile_cmdHandler / SendPartial_cmdHandler: offset/length = 0/0 for SendFile, startOffset/length for SendPartial; source=COMMAND; opCode/cmdSeq recorded; context = U32::MAX. If sourceFilename.length() >= 240 -> log FilenameSourceOverflow + cmdResponse VALIDATION_ERROR. Else if dest >= 240 -> FilenameDestinationOverflow + VALIDATION_ERROR. Else enqueue; if queue send != OP_OK -> cmdResponse EXECUTION_ERROR. On successful enqueue NO response is sent — the response is deferred until the downlink finishes.
SendFile_handler (guarded port): source=PORT, opCode=0, cmdSeq=0, context = m_cntxId++ (monotone U32). Overflow -> event + return SendFileResponse(STATUS_ERROR, U32::MAX). Queue full/send failure -> SendFileResponse(STATUS_ERROR, U32::MAX). Success -> SendFileResponse(STATUS_OK, context) immediately; final completion is reported later on FileComplete.
Cancel_cmdHandler: if mode is DOWNLINK or WAIT -> mode = CANCEL. Always cmdResponse OK immediately.
sendResponse(status): if m_curEntry.source==COMMAND -> cmdResponse_out(opCode, cmdSeq, statusToCmdResp(status)); else broadcast FileComplete_out(i, SendFileResponse(status, m_curEntry.context)) over every connected FileComplete port. statusToCmdResp: STATUS_OK->OK(0), STATUS_ERROR->EXECUTION_ERROR, STATUS_INVALID->VALIDATION_ERROR, STATUS_BUSY->BUSY; default FW_ASSERT(false).

### Svc::FileDownlink — send state machine

Files: `Svc/FileDownlink/FileDownlink.cpp`, `Svc/FileDownlink/File.cpp`, `default/config/FileDownlinkCfg.hpp`

Mode {IDLE=0, DOWNLINK=1, CANCEL=2, WAIT=3, COOLDOWN=4}, init IDLE, guarded by its own Os::Mutex (set/get lock it) because SendFile is guarded and Cancel/Run/bufferReturn run on the component thread.
Run_handler(context): IDLE -> nonblocking receive one FileEntry from m_fileQueue; if status!=OP_OK or size mismatch return; else sendFile(entry). COOLDOWN -> if m_curTimer >= m_cooldown { m_curTimer=0; mode=IDLE } else m_curTimer += m_cycleTime. WAIT -> m_curTimer += m_cycleTime (accumulates but NOTHING ever compares it: m_timeout is declared and never read — there is NO timeout in this version). DOWNLINK/CANCEL -> no-op.
sendFile(src,dst,startOffset,length): File::open — set source/dest LogStringArg names, reset checksum, SandboxedFile.open(src, OPEN_READ) (path validation here); then size(); if size query fails or the FwSizeType size does not round-trip through U32 -> close + BAD_SIZE. On open != OP_OK: mode=IDLE; if status==OUTSIDE_SANDBOX -> warning sourceOutOfSandbox (event SourceOutOfSandbox) else warning fileOpenError (event FileOpenError); sendResponse(FILEDOWNLINK_COMMAND_FAILURES_DISABLED ? STATUS_OK : STATUS_ERROR); return. If fileSize==0: close, mode=IDLE, warning zeroSize, sendResponse(...STATUS_OK : STATUS_INVALID). Else if startOffset >= fileSize: enterCooldown(), log DownlinkPartialFail(src,dst,startOffset,fileSize), sendResponse(...STATUS_OK : STATUS_INVALID). Else if length > fileSize-startOffset: log WARNING_LO DownlinkPartialWarning(startOffset,length,fileSize,src,dst) and clamp length = fileSize-startOffset.
Then: getBuffer(m_buffer, FILE_PACKET); sendStartPacket(); mode=WAIT; m_sequenceIndex=1; m_curTimer=0; m_byteOffset=startOffset; m_lastCompletedType=T_START; if length>0 { log SendStarted(length, src, dst); m_endOffset = startOffset+length } else { log SendStarted(fileSize-startOffset, src, dst); m_endOffset = fileSize }.
bufferReturn_handler(fwBuffer): if (m_lastBufferId != fwBuffer.getContext()+1) OR mode==IDLE -> ignore (stale buffer). FW_ASSERT(mode==WAIT || mode==CANCEL). If m_lastCompletedType is T_END or T_CANCEL -> finishHelper(cancel = (type==T_CANCEL)) and return. Else if mode==WAIT -> mode=DOWNLINK. Then downlinkPacket().
downlinkPacket(): FW_ASSERT(m_lastCompletedType != T_NONE); FW_ASSERT(mode==CANCEL || mode==DOWNLINK). If mode==CANCEL && lastCompleted==T_START -> sendCancelPacket(); lastCompleted=T_CANCEL. Else if mode==DOWNLINK && lastCompleted==T_START -> sendDataPacket(m_byteOffset); on failure log SendDataFail(src, m_byteOffset), enterCooldown(), sendResponse(...STATUS_OK:STATUS_ERROR), return WITHOUT entering WAIT. Else if lastCompleted==T_DATA -> sendEndPacket(); lastCompleted=T_END. Then mode=WAIT; m_curTimer=0.
sendDataPacket(byteOffset&): maxDataSize = FILEDOWNLINK_INTERNAL_BUFFER_SIZE(512) - DataPacket::HEADERSIZE(11) - sizeof(FwPacketDescriptorType)(2) = 499 bytes of file data per DATA packet. If byteOffset >= m_endOffset return INVALID_ARGUMENT. remaining = m_endOffset - byteOffset; dataSize = min(remaining, 499). If dataSize+byteOffset == m_endOffset -> lastCompleted = T_DATA (marks the final data packet BEFORE it is sent). File::read: seek(byteOffset, ABSOLUTE), read(data, size); short read -> BAD_SIZE; else checksum.update(data, byteOffset, size). On read error: warning fileRead(status) (event FileReadError) and return status. Build DataPacket.initialize(m_sequenceIndex, byteOffset, (U16)dataSize, buffer); ++m_sequenceIndex; sendFilePacket; byteOffset += dataSize.
finishHelper(cancel): if !cancel { filesSent++ ; log FileSent(src,dst) } else { log DownlinkCanceled(src,dst) }; enterCooldown(); sendResponse(STATUS_OK) — a cancel still reports STATUS_OK.
enterCooldown(): close file, mode=COOLDOWN, lastCompleted=T_NONE, m_curTimer=0.
FILEDOWNLINK_COMMAND_FAILURES_DISABLED = true by default (so open/zero-size/partial/senddata failures report STATUS_OK).

### Svc::FileDownlink — buffer/flow-control mechanics

Files: `Svc/FileDownlink/FileDownlink.cpp`

No BufferManager: m_memoryStore[COUNT_PACKET_TYPE=2][512] static arrays, indices FILE_PACKET=0, CANCEL_PACKET=1. getBuffer(buf, type): FW_ASSERT(0 <= type < 2); buf.set(m_memoryStore[type], 512, m_lastBufferId); m_lastBufferId++. Only one packet is in flight at a time; bufferReturn matches on context+1 == m_lastBufferId, so any earlier/stale buffer is silently dropped.
sendFilePacket(fp): bufferSize = fp.bufferSize() + 2; FW_ASSERT(m_buffer.data != null); FW_ASSERT(m_buffer.size >= bufferSize); serialize U16 FW_PACKET_FILE(0x0003) at offset 0; serialize the FilePacket into a sub-buffer at data+2 (size-2); m_buffer.setSize(bufferSize); bufferSendOut_out(0, m_buffer); m_buffer.setSize(512) restore; packetsSent++.
sendCancelPacket(): builds a CancelPacket with the CURRENT m_sequenceIndex, takes a fresh local Fw::Buffer from the CANCEL_PACKET store (bumping m_lastBufferId again), asserts size >= fp.bufferSize()+2, same descriptor+serialize+setSize+bufferSendOut_out(0)+packetsSent++.
sendEndPacket(): EndPacket.initialize(m_sequenceIndex, checksum-from-file) then sendFilePacket via m_buffer (does NOT re-acquire a buffer). sendStartPacket(): StartPacket.initialize(file size, source name, dest name) — StartPacket::initialize always forces header sequenceIndex = 0.

### Svc::FileManager — ports

Files: `Svc/FileManager/FileManager.fpp`, `Fw/Interfaces/DataProductSync.fpp`

active component. pingIn: async input Svc.Ping; pingOut: output Svc.Ping. schedIn: SYNC input Svc.Sched (arg context: U32). `internal port run drop` — an internal message with DROP queue-full policy used to hop schedIn onto the component thread. import Fw.DataProductSync -> productGetOut (product get, output) and productSendOut (product send, output). Role ports: cmdIn, cmdRegOut, cmdResponseOut, eventOut, LogText, timeCaller, tlmOut. Data products: product record FileChunkHeaderRecord: FileManager.FileChunkHeader id 0; product record FileChunkDataRecord: U8 array id 1; product container FileDpContainer id 0 default priority FileManagerCfg.DEFAULT_DP_PRIORITY = 10. struct FileChunkHeader { fileName: string size 240, offset: U64, dataSize: U32 }. Enums: GenerateDpStage { OPEN=0, SIZE=1, SEEK=2, READ=3, SERIALIZE=4, BUSY=5 }; GenerateDpMode { PACED=0, IMMEDIATE=1 }. Topology: base id 0x05002000, queue 10, stack 64KiB, priority 22.

### Svc::FileManager — commands (exact list; NO shell command)

Files: `Svc/FileManager/Commands.fppi`, `Svc/FileManager/FileManager.cpp`

All async. Every path string is `string size FileNameStringSize` = 240. Opcode 0x04 is an intentional GAP (the historic ShellCommand was removed — do NOT port a shell op).
0x00 CreateDirectory(dirName): log CreateDirectoryStarted; Os::FileSystem::createDirectory(dirName, errorIfDirExists=true); on error log DirectoryCreateError(dirName, status as U32) else CreateDirectorySucceeded; emitTelemetry(status); cmdResponse OK/EXECUTION_ERROR.
0x01 MoveFile(sourceFileName, destFileName): log MoveFileStarted; FileSystem::moveFile (rename; on EXDEV_ERROR falls back to copyFile + removeFile); FileMoveError / MoveFileSucceeded.
0x02 RemoveDirectory(dirName): log RemoveDirectoryStarted; FileSystem::removeDirectory; DirectoryRemoveError / RemoveDirectorySucceeded.
0x03 RemoveFile(fileName, ignoreErrors: bool): log RemoveFileStarted; FileSystem::removeFile; on error log FileRemoveError, and if ignoreErrors==true -> errorCount++, tlmWrite Errors, cmdResponse OK, RETURN EARLY (skips emitTelemetry/sendCommandResponse); on success RemoveFileSucceeded.
0x05 AppendFile(source, target): log AppendFileStarted; FileSystem::appendFile(source, target, createMissingDest=true); AppendFileFailed / AppendFileSucceeded.
0x06 FileSize(fileName): log FileSizeStarted; FileSystem::getFileSize -> FwSizeType; FileSizeError / FileSizeSucceeded(fileName, size: FwSizeType).
0x07 ListDirectory(dirName): async, rate-group paced (see state machine item).
0x08 CalculateCrc(filename): log CalculateCrcStarted; Os::File.open(OPEN_READ) then file.calculateCrc(U32&) (Os::File CRC reads in FW_FILE_CHUNK_SIZE=512 blocks, raw-register CRC form); success -> CalculateCrcSucceeded(filename, crc) + cmdResponse OK; failure -> CalculateCrcFailed(filename, status) + EXECUTION_ERROR. NOTE: does NOT touch the CommandsExecuted/Errors telemetry.
0x09 GenerateDp(fileName, chunkSize: U32, beginOffset: U64, endOffset: U64, priority: U32, mode: GenerateDpMode).
Helpers: emitTelemetry(status) -> OP_OK ? ++commandCount + tlmWrite_CommandsExecuted : ++errorCount + tlmWrite_Errors. sendCommandResponse(op,seq,status) -> OP_OK ? Fw::CmdResponse::OK : EXECUTION_ERROR.

### Svc::FileManager — events / telemetry

Files: `Svc/FileManager/Events.fppi`, `Svc/FileManager/Telemetry.fppi`

Telemetry: CommandsExecuted U32 id 0x00; Errors U32 id 0x01.
Events (string args all size 240; status args all U32 unless noted). WARNING_HI: 0x00 DirectoryCreateError(dirName,status); 0x01 DirectoryRemoveError(dirName,status); 0x02 FileMoveError(sourceFileName,destFileName,status); 0x03 FileRemoveError(fileName,status); 0x05 AppendFileFailed(source,target,status); 0x13 FileSizeError(fileName,status); 0x17 ListDirectoryError(dirName,status); 0x1b CalculateCrcFailed(fileName,status); 0x1d FileNameFormatError(fileName, status: Fw.StringFormatStatus); 0x20 GenerateDpFailed(fileName, stage: GenerateDpStage, status); 0x21 GenerateDpBufferFailed(fileName); 0x22 GenerateDpInvalidRange(fileName, beginOffset: U64, endOffset: U64, fileSize: U64).
ACTIVITY_HI: 0x06 AppendFileSucceeded(source,target); 0x08 CreateDirectorySucceeded(dirName); 0x09 RemoveDirectorySucceeded(dirName); 0x0A MoveFileSucceeded(src,dst); 0x0B RemoveFileSucceeded(fileName); 0x0C AppendFileStarted(source,target); 0x0E CreateDirectoryStarted(dirName); 0x0F RemoveDirectoryStarted(dirName); 0x10 MoveFileStarted(src,dst); 0x11 RemoveFileStarted(fileName); 0x12 FileSizeSucceeded(fileName, size: FwSizeType); 0x14 FileSizeStarted(fileName); 0x15 ListDirectoryStarted(dirName); 0x16 ListDirectorySucceeded(dirName, fileCount: U32); 0x18 DirectoryListing(dirName, fileName, fileSize: FwSizeType); 0x19 DirectoryListingSubdir(dirName, subdirName); 0x1a CalculateCrcStarted(fileName); 0x1c CalculateCrcSucceeded(fileName, crc: U32); 0x1e GenerateDpStarted(fileName, bytesToWrite: U64); 0x1f GenerateDpComplete(fileName, chunks: U32). Gaps: 0x04, 0x07, 0x0D. No throttles.

### Svc::FileManager — schedIn / listing / DP state machines

Files: `Svc/FileManager/FileManager.cpp`, `Svc/FileManager/FileManager.hpp`, `default/config/FileManagerConfig.hpp`

schedIn_handler (SYNC, caller's thread): std::atomic<bool> m_runQueued; compare_exchange_strong(expected=false -> true); only if the swap succeeded call run_internalInterfaceInvoke() (async internal message, queue-full policy DROP). run_internalInterfaceHandler (component thread): FW_ASSERT(m_runQueued); m_runQueued=false; then DP pacing then listing pacing, in that order, in the SAME tick.
DP: GenerateDpState {DP_IDLE, DP_IN_PROGRESS}. GenerateDp_cmdHandler: if state != DP_IDLE -> log GenerateDpFailed(file, BUSY, 0) + cmdResponse OK, return. If productGetOut(0) or productSendOut(0) not connected -> GenerateDpBufferFailed + OK. effectiveChunkSize = (chunkSize==0 || chunkSize > GENERATE_DP_MAX_CHUNK_SIZE=1024) ? 1024 : chunkSize. Open(OPEN_READ) fail -> GenerateDpFailed(OPEN,status)+OK. size() fail -> close + GenerateDpFailed(SIZE,status)+OK. effectiveEnd = (endOffset==0 || endOffset > fileSize) ? fileSize : endOffset. badRange = (beginOffset > fileSize) || (fileSize != 0 && beginOffset >= effectiveEnd) -> close + GenerateDpInvalidRange(file, beginOffset, endOffset, fileSize) + OK. If beginOffset>0 seek ABSOLUTE; fail -> close + GenerateDpFailed(SEEK,status)+OK. Store state; m_dpPriority = (priority==0) ? DEFAULT_DP_PRIORITY(10) : priority; state=DP_IN_PROGRESS; log GenerateDpStarted(file, effectiveEnd-beginOffset). If offset >= end (empty range) -> log GenerateDpComplete(file, 0) + finishDpGeneration(). If mode==IMMEDIATE -> processDpChunks(0) (unbounded, whole range in the command handler); PACED -> return, rate group drives it.
processDpChunks(limit): paced = limit>0; loop (chunk<limit or forever): requestedSize = min(m_dpEndOffset-m_dpOffset, m_dpChunkSize); read into m_dpBuffer[1024]; short read or error -> GenerateDpFailed(READ, status) + finishDpGeneration + return. dpSize = SIZE_OF_FileChunkHeaderRecord_RECORD + SIZE_OF_FileChunkDataRecord_RECORD(readSize); dpGet_FileDpContainer(dpSize, container); != SUCCESS -> GenerateDpBufferFailed + finish + return. container.setPriority(m_dpPriority); serializeRecord_FileChunkHeaderRecord(FileChunkHeader{fileName, m_dpOffset, (U32)readSize}) then serializeRecord_FileChunkDataRecord(m_dpBuffer, readSize); either failing -> GenerateDpFailed(SERIALIZE, serializeStatus) + finish + return. dpSend(container); m_dpOffset += readSize; m_dpChunkCount++; if m_dpOffset >= m_dpEndOffset -> GenerateDpComplete(file, chunkCount) + finish + return. Rate tick uses limit = CHUNKS_PER_RATE_TICK = 1.
finishDpGeneration(): close file, state=DP_IDLE, zero offsets/size, cmdResponse_out(m_dpOpCode, m_dpCmdSeq, OK) — ALWAYS OK, even on failure.
Listing: ListDirectoryState {IDLE, LISTING_IN_PROGRESS}. ListDirectory_cmdHandler: if already LISTING_IN_PROGRESS -> log ListDirectoryError(dirName, Os::Directory::OTHER_ERROR=10), emitTelemetry(FileSystem::OTHER_ERROR), cmdResponse EXECUTION_ERROR, return. log ListDirectoryStarted; m_currentDir.open(dirName, OpenMode::READ); on failure -> ListDirectoryError(dirName, status) + emitTelemetry(OTHER_ERROR) + EXECUTION_ERROR. Else set state, m_currentDirName, m_currentOpCode, m_currentCmdSeq, m_totalEntries=0, NO response yet.
Per tick: loop FILES_PER_RATE_TICK (=1) times: m_currentDir.read(filename). NO_MORE_FILES -> close, state=IDLE, log ListDirectorySucceeded(dirName, m_totalEntries), emitTelemetry(OP_OK), cmdResponse OK, break. OP_OK -> fullPath.format("%s/%s", dirName, filename); if format != SUCCESS -> log FileNameFormatError(filename, formatStatus) (pathType treated NOT_EXIST); else pathType = FileSystem::getPathType(fullPath); FILE -> getFileSize(fullPath) then log DirectoryListing(dirName, filename, size or 0 on failure); DIRECTORY -> log DirectoryListingSubdir(dirName, filename); other/NOT_EXIST -> log DirectoryListing(dirName, filename, 0). m_totalEntries++ in all OP_OK cases. Any other status -> close, state=IDLE, log ListDirectoryError(dirName, status), emitTelemetry(OTHER_ERROR), cmdResponse EXECUTION_ERROR, break.

### Os::SandboxedFile / Os::FilePathUtils — path handling

Files: `Os/SandboxedFile.cpp`, `Os/SandboxedFile.hpp`, `Os/FilePathUtils.cpp`

SandboxedFile default-constructs with m_allowedDirectory = "/" and m_configured = true — FAIL-OPEN. configure(dir): FW_ASSERT(dir != null), FW_ASSERT(!isOpen()); resolveFromCwd(dir) into a MAX_PATH_LENGTH=FileNameStringSize=240 buffer, FW_ASSERT(VALID), FW_ASSERT(len>0 && len+2 <= 240); append '/' if missing. open(path, mode, overwrite=NO_OVERWRITE): if !configured -> OUTSIDE_SANDBOX; resolveFromCwd(path) -> non-VALID gives OUTSIDE_SANDBOX; checkContainment(resolved, allowedDir) != VALID -> OUTSIDE_SANDBOX; else Os::File::open(RESOLVED path). Everything else forwards to Os::File.
FilePathUtils::Status { VALID=0, OUTSIDE_SANDBOX=1, INVALID_PATH=2, TOO_LONG=3 }. resolvePath: empty/null path -> INVALID_PATH; relative path requires an absolute baseDir (baseDir[0]=='/'), else INVALID_PATH; buffer overflow -> TOO_LONG; then a purely textual in-place resolve: skip empty and "." segments, ".." backs writePos to the previous '/' (clamped at root), normal segments memmove'd left, trailing '/' trimmed unless the result is "/". No realpath, no symlink following. resolveFromCwd: relative -> FileSystem::getWorkingDirectory into a 240 byte buffer (failure -> INVALID_PATH) then resolvePath; absolute -> resolvePath(path, "/").
checkContainment(resolvedPath, allowedDirectory): both must be valid C strings < 240; allowedDirectory must end with '/'; special case pathLen == allowedLen-1 and memcmp equal -> VALID (path IS the sandbox dir); else pathLen < allowedLen -> OUTSIDE_SANDBOX; memcmp of allowedLen bytes must match.

### Fw::FilePacket / CFDP::Checksum (already documented)

Files: `Fw/FilePacket/FilePacket.hpp`, `CFDP/Checksum/Checksum.cpp`, `/home/user/fprime-rust/docs/cpp-analysis/utils-misc.md`

See docs/cpp-analysis/utils-misc.md lines 60-64 and 106-108 for the full byte layouts and the checksum algorithm — do not re-derive. Facts needed by the components: Header::HEADERSIZE = sizeof(U8)+sizeof(U32) = 5; DataPacket::HEADERSIZE = Header::HEADERSIZE + sizeof(U32) + sizeof(U16) = 11; PathName::MAX_LENGTH = 255 (FileUplink copies into char[256] and returns Os::File::BAD_SIZE if len >= 256). Types: T_START=0, T_DATA=1, T_END=2, T_CANCEL=3, T_NONE=255. StartPacket::initialize always sets sequenceIndex = 0.

## Wire formats

### File packet in a com buffer (both directions)

[FwPacketDescriptorType = U16 big-endian = 0x0003 (FW_PACKET_FILE)] followed immediately by the serialized Fw::FilePacket. FileUplink strips the first 2 bytes before FilePacket::fromBuffer; FileDownlink writes the 2 bytes then serializes the packet into a sub-buffer at data+2. Total buffer size is set to filePacket.bufferSize() + 2.

### FilePacket bodies

Already specified in /home/user/fprime-rust/docs/cpp-analysis/utils-misc.md (§ 'FilePacket family'): header = U8 type + U32 sequenceIndex; START = header + U32 fileSize + PathName(source) + PathName(dest), PathName = U8 len + len raw bytes, no NUL, exact-consumption required; DATA = header + U32 byteOffset + U16 dataSize + dataSize raw bytes (fixed part 11 bytes, remaining must equal dataSize); END = header + U32 checksum, exact-consumption; CANCEL = header only.

### Svc::SendFileResponse

status: SendFileStatus (U8: STATUS_OK=0, STATUS_ERROR=1, STATUS_INVALID=2, STATUS_BUSY=3) then context: U32 big-endian. 5 bytes.

### Svc::FileManager::FileChunkHeader (data product record)

fileName: F Prime string size 240 (U16 length prefix + bytes) + offset: U64 big-endian + dataSize: U32 big-endian. Emitted as FileChunkHeaderRecord (record id 0) immediately followed by FileChunkDataRecord (record id 1, U8 array) carrying dataSize file bytes, inside FileDpContainer (container id 0).

### Downlink chunking sizes

FILEDOWNLINK_INTERNAL_BUFFER_SIZE = FW_FILE_BUFFER_MAX_SIZE = FW_COM_BUFFER_MAX_SIZE = 512. Max DATA payload = 512 - 11 (DataPacket::HEADERSIZE) - 2 (descriptor) = 499 bytes/packet. FW_FILE_CHUNK_SIZE = 512 is NOT used by FileDownlink — it is Os::File's CRC read block and Os::FileSystem's copy chunk size.

## Threading / concurrency

FileUplink: active (own thread, queue 10). bufferSendIn and pingIn are async — all receive-state mutation happens on the component thread; no extra locking. Telemetry/event writes happen from that thread. File I/O (open/seek/write NO_WAIT/close) is blocking on the component thread.

FileDownlink: active (own thread, queue 10). Run, bufferReturn, pingIn are async (component thread). SendFile is GUARDED — it runs on the caller's thread under the component mutex, so it touches only m_cntxId and the Os::Queue (which is itself thread safe). Because of that, Mode is wrapped in its own Os::Mutex (set/get both lock) even though only the component thread ever changes it in practice. Commands are async (component thread). The internal file queue (Os::Queue of raw FileEntry structs) is the producer/consumer boundary between the guarded port / command handlers and the Run tick. Flow control is a strict one-packet credit: exactly one buffer out, then WAIT until that same buffer (matched by context) returns.

FileManager: active (own thread, queue 10) but schedIn is SYNC — it executes on the rate group's thread and only performs a std::atomic<bool> compare_exchange_strong on m_runQueued, then invokes the `internal port run drop` async message. All real listing/DP work happens in run_internalInterfaceHandler on the component thread. The atomic gate prevents queue flooding when the component thread lags behind the rate group; the DROP policy on the internal port means a missed tick is simply skipped. Commands are async (component thread). Blocking filesystem calls (createDirectory, moveFile with cross-device copy fallback, appendFile, CRC over the whole file) run on the component thread and can be long.

## Porting notes

1. Reuse fprime-fw's Fw::Buffer (owned) plus the existing serializer for the U16 descriptor prefix; implement Fw::FilePacket as the enum { Start, Data, End, Cancel } the utils-misc.md analysis already prescribes, with borrowed &[u8] payload/paths.
2. Implement CFDP::Checksum as a small Copy struct { value: u32 } with update(&mut self, data: &[u8], file_offset: u32) reproducing addByteAtOffset; derive PartialEq for the END comparison. Keep getValue() for the BadChecksum event args.
3. FileUplink: model as a passive-state struct behind the existing active-component base; ReceiveMode { Start, Data }. Store last_sequence_index: u32, last_packet_write_status: Option<os::file::Status> (C++ sentinel is MAX_STATUS=13 — use None or the explicit discriminant, but keep the "duplicate skipped only when the previous write succeeded" logic exact). Port the throttle-clear-on-START set exactly (4 of the 5 throttled events).
4. FileDownlink: replace the raw-struct Os::Queue with fprime-utils' fixed-message queue carrying a typed FileEntry (no memcpy of a POD). Keep the U32::MAX context sentinel for command-sourced entries. Keep the two static [u8; 512] packet arenas and the context-word matching (context+1 == last_buffer_id) — this is load-bearing for stale-buffer rejection; do not replace it with an Option<Buffer>.
5. Mode needs interior mutability across the guarded SendFile port and the component thread: use a Mutex<Mode> (matching C++'s per-Mode mutex) rather than an atomic, so the port and thread see the same serialization the C++ does.
6. FileManager: keep the schedIn -> AtomicBool -> internal-message hop; fprime-comp's internal-port/DROP support is the right mechanism. Both paced loops (listing and DP) are driven from the same handler in that order.
7. Path handling: port Os::FilePathUtils as a pure-string module operating on &str/[u8] with MAX_PATH_LENGTH = FileNameStringSize = 240. It is purely textual (no realpath, no symlink resolution) — Rust must NOT "improve" this by calling std::fs::canonicalize, or the sandbox semantics and error codes change. SandboxedFile wraps fprime-os File and must default to allowed_directory = "/" and configured = true (fail-open) to match.
8. Status enums: add Os::File::Status::OUTSIDE_SANDBOX = 12 and MAX_STATUS = 13 to the fprime-os File status enum (discriminants in declaration order: OP_OK=0, DOESNT_EXIST=1, NO_SPACE=2, NO_PERMISSION=3, BAD_SIZE=4, NOT_OPENED=5, FILE_EXISTS=6, NOT_SUPPORTED=7, INVALID_MODE=8, INVALID_ARGUMENT=9, NO_MORE_RESOURCES=10, OTHER_ERROR=11, OUTSIDE_SANDBOX=12, MAX_STATUS=13). FileSystem::Status: OP_OK=0, ALREADY_EXISTS=1, NO_SPACE=2, NO_PERMISSION=3, NOT_DIR=4, IS_DIR=5, NOT_EMPTY=6, INVALID_PATH=7, DOESNT_EXIST=8, FILE_LIMIT=9, BUSY=10, NO_MORE_FILES=11, BUFFER_TOO_SMALL=12, EXDEV_ERROR=13, OVERFLOW_ERROR=14, NOT_SUPPORTED=15, OTHER_ERROR=16. Directory::Status: OP_OK=0, DOESNT_EXIST=1, NO_PERMISSION=2, NOT_OPENED=3, NOT_DIR=4, NO_MORE_FILES=5, FILE_LIMIT=6, BAD_DESCRIPTOR=7, ALREADY_EXISTS=8, NOT_SUPPORTED=9, OTHER_ERROR=10. FileSystem::PathType: FILE=0, DIRECTORY=1, OTHER=2, NOT_EXIST=3. These are the U32 values that go out in the FileManager event `status` args.
9. FileManager's GenerateDp path needs data products (Fw/Dp, DpContainer, product get/send ports) which are NOT yet ported. Recommendation: land FileManager's 8 filesystem commands + listing state machine first and gate GenerateDp (0x09) behind the Dp port, or stub it as "productGetOut not connected" -> GenerateDpBufferFailed + cmdResponse OK, which is a behavior the C++ itself produces.
10. Config constants to add to fprime-config: FW_FILE_BUFFER_MAX_SIZE = FW_COM_BUFFER_MAX_SIZE = 512; FW_FILE_CHUNK_SIZE = 512; FileDownCompletePorts = 1; FileNameStringSize = 240; FILEDOWNLINK_COMMAND_FAILURES_DISABLED = true; FILEDOWNLINK_INTERNAL_BUFFER_SIZE = 512; FileManagerConfig::{FILES_PER_RATE_TICK = 1, GENERATE_DP_MAX_CHUNK_SIZE = 1024, CHUNKS_PER_RATE_TICK = 1}; FileManagerCfg::DEFAULT_DP_PRIORITY = 10; FileHandling subtopology: cooldown 1000ms, cycleTime 1000ms, fileQueueDepth 10, base ids 0x05000000/0x05001000/0x05002000, queue 10, stack 64KiB, priorities 24/23/22.
11. Test vectors worth locking: a 3-packet uplink (START/DATA/END) with a known CFDP checksum; a duplicate DATA after a successful write (must be silently skipped) vs after a failed write (must be re-written); an out-of-bounds DATA (offset+size > fileSize) that must NOT update last_packet_write_status; a downlink of a 1000-byte file producing START + 2 DATA (499 + 501? no — 499 + 499 + 2) + END with the buffer-return handshake between each.

## Rust feasibility (safe, zero-dependency std)

Everything here is portable in safe, zero-dependency std Rust. Specific notes:
- No ioctl, no zlib, no C library dependence anywhere in these components. All file access goes through Os::File / Os::FileSystem / Os::Directory, which fprime-os already provides over std::fs.
- Os::File::calculateCrc (used by CalculateCrc 0x08) is already ported in fprime-utils/fprime-os (the raw-register CRC-32 form, 512-byte blocks). Reuse it; do not add a crc crate.
- Os::FileSystem::moveFile's EXDEV fallback: std::fs::rename returns an io::Error whose raw_os_error() is EXDEV (18) on Unix. Detecting it needs `std::io::Error::raw_os_error()` — available, no unsafe. If a platform-neutral path is preferred, fall back to copy+remove on any rename error, but map the error code faithfully so the FileMoveError status arg matches.
- `Os::File::write(..., WaitType::NO_WAIT)` (FileUplink) versus WAIT: std has no non-flushing distinction beyond simply not calling sync_all/flush. Map NO_WAIT to a plain write with no fsync, WAIT to write + sync_all. This is what fprime-os already does; keep it.
- FileManager's `std::atomic<bool> m_runQueued` maps to std::sync::atomic::AtomicBool with compare_exchange (use SeqCst to match C++'s default seq_cst).
- FileDownlink's `U8 buffer[maxDataSize]` is a C++ VLA-ish stack array of 499 bytes; in Rust use a fixed [u8; 499] const-sized from the config constants (const arithmetic: FILEDOWNLINK_INTERNAL_BUFFER_SIZE - DATA_PACKET_HEADERSIZE - size_of::<FwPacketDescriptorType>()).
- The one genuinely unportable-as-written piece is the raw memcpy of the FileEntry POD into and out of Os::Queue; port it as a typed queue element instead (behavior-identical, and the queue is component-internal so no wire compatibility is at stake).
- FilePathUtils' in-place resolve uses overlapping memmove; in Rust use copy_within on a &mut [u8] (safe) — the algorithm holds because writePos <= readPos always.

## Gotchas

- FileUplink handleDataPacket: when receiveMode != DATA it logs InvalidReceiveMode and RETURNS, leaving the mode unchanged. The sdd.md says 'go to START mode' — the code does NOT. Follow the code.
- FileUplink duplicate detection is a single-slot check (seq == m_lastSequenceIndex) and it is bypassed entirely when the previous write failed (m_lastPacketWriteStatus != OP_OK), which is the deliberate retry path. checkSequenceIndex ALWAYS assigns m_lastSequenceIndex = seq even when it logs PacketOutOfOrder, so a gap resynchronizes rather than aborting.
- FileUplink PacketOutOfBounds returns WITHOUT updating m_lastPacketWriteStatus, so the previous packet's status still governs duplicate suppression for the next packet.
- FileUplink clears 4 throttles on START (FileWriteError, InvalidReceiveMode, PacketOutOfBounds, PacketOutOfOrder) but NOT PacketDuplicate (throttle 20) — that throttle is never cleared for the life of the component.
- FileUplink returns the input buffer on bufferSendOut in every path, including the too-small-buffer and wrong-descriptor early returns and the decode-error path. Never leak it.
- FileUplink goToDataMode does not close the file; only goToStartMode does. On a second START while in DATA mode the file is closed explicitly before reopening.
- FileUplink File::open returns Os::File::BAD_SIZE (4) when the destination path length >= 256 — this surfaces as a FileOpenError event, not a distinct event.
- FileDownlink m_timeout is declared and initialized nowhere and read nowhere: there is NO WAIT-state timeout in this version. m_curTimer accumulates in WAIT forever. A hung consumer stalls the component permanently. Do not invent a timeout.
- FileDownlink's guarded SendFile port returns STATUS_ERROR (not STATUS_BUSY) when the internal queue is full, with context = U32::MAX. STATUS_BUSY exists in the enum and in statusToCmdResp but is never produced by any code path.
- FileDownlink command handlers do NOT respond on successful enqueue — the response is deferred until the transfer finishes (sendResponse from finishHelper or an error path). A queue-send failure responds EXECUTION_ERROR immediately; a filename overflow responds VALIDATION_ERROR immediately.
- FILEDOWNLINK_COMMAND_FAILURES_DISABLED defaults to TRUE, so open failures, zero-size files, start-offset-past-EOF and send-data failures all report Fw::CmdResponse::OK (or SendFileStatus::STATUS_OK on the port) while still emitting the warning event. A cancel also finishes with STATUS_OK.
- FileDownlink sendDataPacket sets m_lastCompletedType = T_DATA BEFORE the read/send, so a read failure on the final chunk leaves the state marked as 'data complete'; the failure path calls enterCooldown() (which resets it to T_NONE) and returns without entering WAIT.
- FileDownlink sendCancelPacket acquires a second buffer from the CANCEL_PACKET arena, bumping m_lastBufferId again — so the FILE_PACKET buffer sent just before is now stale and its return is ignored. Reproduce the id bookkeeping exactly.
- FileDownlink bufferReturn asserts mode is WAIT or CANCEL after the stale/IDLE filter — a return arriving in DOWNLINK mode with a matching context is a hard FW_ASSERT (panic in Rust).
- FileDownlink's DownlinkPartialFail path calls enterCooldown() (leaving the file open? no — enterCooldown closes it) whereas the open-failure and zero-size paths set mode = IDLE directly and skip the cooldown.
- Both sandboxes are FAIL-OPEN: SandboxedFile default-constructs with allowedDirectory = "/" and configured = true. The stock FileHandling subtopology never calls configure(directory) for either component. Preserve this default (with the same warning) rather than defaulting to a restrictive sandbox.
- SandboxedFile::open passes the RESOLVED (canonicalized) path to the underlying open, not the caller's original path. Path resolution is purely textual: symlinks are not followed, so a symlink inside the sandbox escapes it.
- FileManager opcode 0x04 is a deliberate gap (removed ShellCommand); event ids 0x04, 0x07 and 0x0D are also gaps. There is NO shell/exec command in this version — do not add one.
- FileManager RemoveFile with ignoreErrors=true takes an early return that increments Errors telemetry and responds OK, skipping emitTelemetry/sendCommandResponse — so a suppressed removal still counts as an error but never as a CommandsExecuted.
- FileManager CalculateCrc (0x08) and GenerateDp (0x09) do NOT touch the CommandsExecuted/Errors telemetry channels at all, unlike every other command.
- FileManager GenerateDp always responds Fw::CmdResponse::OK, including for BUSY, unconnected DP ports, open/size/seek/read/serialize failures and invalid ranges. Only the warning event distinguishes them.
- FileManager ListDirectory holds the Os::Directory handle open across rate ticks; a second ListDirectory while LISTING_IN_PROGRESS is rejected with ListDirectoryError(dirName, Os::Directory::OTHER_ERROR=10) and EXECUTION_ERROR — note the status arg is a Directory status here while emitTelemetry gets a FileSystem status.
- FileManager's directory listing formats the full path as "%s/%s" (dirName + '/' + entry) even when dirName already ends in '/', producing a double slash — harmless for stat but reproduce it if byte-identical event text matters.
- FileDownlink is limited to files <= 4 GiB: File::open returns BAD_SIZE if the FwSizeType file size does not round-trip through U32; File::read returns BAD_SIZE on any short read.

