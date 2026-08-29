# Svc/Ccsds — CCSDS communications stack (SpacePacketFramer, SpacePacketDeframer, ApidManager, TmFramer, TcDeframer, Types, Ports, Utils/CRC16) + default/config/ComCfg.fpp

> Analysis of the C++ F Prime implementation (github.com/nasa/fprime) produced to guide this Rust port.
> File paths refer to the C++ tree.

## Overview

Two layered protocol stacks over the generic Svc.Framer/Svc.Deframer interfaces. Downlink: ComQueue -> SpacePacketFramer (adds 6-byte SPP primary header, allocates a new buffer) -> ComAggregator -> TmFramer (fixed 1024-byte TM transfer frame, 6-byte header + payload + SPP idle-packet fill + 2-byte CRC16 trailer, internal static frame buffer) -> ComStub/driver. Uplink: driver -> FrameAccumulator with CcsdsTcFrameDetector -> TcDeframer (validates SCID/length/VCID/CRC16, strips 5-byte TC header and 2-byte trailer in place) -> SpacePacketDeframer (validates PVN/length, extracts APID/secHdr/seqFlags/seqCount into FrameContext, advances buffer past 6-byte header) -> FprimeRouter. ApidManager is a passive shared component holding a per-APID 14-bit sequence counter (Fw::ArrayMap, capacity = ComCfg::Apid::NUM_CONSTANTS = 12), queried by the framer (get+increment) and the deframer (validate+resync). All multi-byte fields serialize big-endian via F Prime Serializable (Fw::Buffer::getSerializer/getDeserializer). Every hop uses port Svc.ComDataWithContext(ref data: Fw.Buffer, context: ComCfg.FrameContext) with a mirror dataReturnOut/dataReturnIn ownership chain. Serialized sizes: SpacePacketHeader=6, TMHeader=6, TMTrailer=2, TCHeader=5, TCTrailer=2, M_PDUHeader=2, AOSHeader=6, AOSTrailer=2. Topology: Svc/Subtopologies/ComCcsds/ComCcsds.fpp (topologies SpacePacketFraming, SpacePacket, TmTcFraming); base ids ComCcsdsConfig.BASE_ID + 0x04000 tcDeframer, +0x05000 spacePacketDeframer, +0x06000 aggregator, +0x07000 framer(TmFramer), +0x08000 spacePacketFramer, +0x09000 apidManager.

## Key items

### ComCfg.fpp — Apid / Pvn / FrameContext / constants

Files: `default/config/ComCfg.fpp`

dictionary type FwPacketDescriptorType = U16. constant SpacecraftId = 0x0044 (10-bit). constant TmFrameFixedSize = 1024. constant AosMaxFrameFixedSize = 1536. constant AggregationSize = TmFrameFixedSize-6-6-1-2 = 1009 (2 SPP headers + 1 idle byte + 2 trailer). constant SaIndexUnset = 0xFFFF. enum Pvn:U8 {SPACE_PACKET_PROTOCOL=0x0, ENCAPSULATION_PACKET_PROTOCOL=0x7, INVALID_UNINITIALIZED=0x8} default INVALID_UNINITIALIZED. enum Apid:U16 {FW_PACKET_COMMAND=0x0000, FW_PACKET_TELEM=0x0001, FW_PACKET_LOG=0x0002, FW_PACKET_FILE=0x0003, FW_PACKET_PACKETIZED_TLM=0x0004, FW_PACKET_DP=0x0005, FW_PACKET_IDLE=0x0006, FW_PACKET_PARAM=0x0007, FW_PACKET_HAND=0x00FE, FW_PACKET_UNKNOWN=0x00FF, SPP_IDLE_PACKET=0x07FF, INVALID_UNINITIALIZED=0x0800} default INVALID_UNINITIALIZED; NUM_CONSTANTS=12. struct FrameContext{comQueueIndex:FwIndexType, apid:Apid, hasSecHdr:bool, sequenceFlags:U8, sequenceCount:U16, vcId:U8, pvn:Pvn, sendNow:bool, saIndex:U16} defaults {0, FW_PACKET_UNKNOWN, false, 0x3, 0, 1, INVALID_UNINITIALIZED, false, 0xFFFF}. (Also documented in fprime-rust/docs/cpp-analysis/svc-comms.md lines 18-22.)

### Svc::Ccsds::FrameError / SdlsStatus (Types.fpp)

Files: `Svc/Ccsds/Types/Types.fpp`

enum FrameError:U8 {SP_INVALID_PACKET=0, SP_INVALID_LENGTH=1, TC_INVALID_SCID=2, TC_INVALID_LENGTH=3, TC_INVALID_VCID=4, TC_INVALID_CRC=5, AOS_INVALID_SCID=6, AOS_INVALID_LENGTH=7, AOS_INVALID_VCID=8, AOS_INVALID_CRC=9, AOS_INVALID_VERSION=10, AOS_INVALID_EPP=11, AOS_VC_FRAME_COUNT_GAP=12, SDLS_DECRYPTION_FAILURE=13}. enum SdlsStatus:U8 {SUCCESS=0, UNKNOWN_SA=1, UNKNOWN_PORT=2, ENCRYPTION_FAILURE=3, DECRYPTION_FAILURE=4, KEY_ERROR=5}. enum Tfvn:U8 {TM_TC=0, AOS=1, PROX_ONE=2, USLP=3, INVALID_UNINITIALIZED=4}. struct SaMapEntry{securityAssociationIndex:U16, portIndex:FwIndexType}.

### Svc.Ccsds Ports

Files: `Svc/Ccsds/Ports/Ports.fpp`

port ApidSequenceCount(apid: ComCfg.Apid, sequenceCount: U16) -> U16 (return-value port). port ErrorNotify(errorCode: Ccsds.FrameError) (no return). port CcsdsSdlsEncryption(securityAssociationIndex: U16, ref data: Fw.Buffer, context: ComCfg.FrameContext). port CcsdsSdlsData(status: SdlsStatus, ref data: Fw.Buffer, context: ComCfg.FrameContext). port SdlsKey(ref key: SdlsKeyBuffer) -> SdlsStatus.

### Svc.Framer / Svc.Deframer interfaces

Files: `Svc/Interfaces/Framer.fpp`, `Svc/Interfaces/Deframer.fpp`

Framer: sync input dataIn(ComDataWithContext); output dataOut; output dataReturnOut; sync input dataReturnIn; sync input comStatusIn(Fw.SuccessCondition); output comStatusOut. Deframer: guarded input dataIn; output dataOut; output dataReturnOut; sync input dataReturnIn. No comStatus on Deframer.

### SpacePacketFramer

Files: `Svc/Ccsds/SpacePacketFramer/SpacePacketFramer.fpp`, `Svc/Ccsds/SpacePacketFramer/SpacePacketFramer.cpp`

Passive. imports Framer; extra output ports: bufferAllocate(Fw.BufferGet), bufferDeallocate(Fw.BufferSend), getApidSeqCount(Ccsds.ApidSequenceCount). Events: NoBufferAvailable() severity WARNING_HI, throttle 5, format "Failed to allocate a packet buffer: packet dropped" (relative event id 0). No commands, no telemetry channels. dataIn_handler: frameSize = 6 + data.size; FW_ASSERT(data.size <= U32_MAX-6); FW_ASSERT(data.size > 0). bufferAllocate_out(0, frameSize); if !valid || size<frameSize -> log NoBufferAvailable, deallocate if valid, dataReturnOut(data, context), return. apid = context.apid; FW_ASSERT((apid >> 11) == 0. packetIdentification = (apid & 0x07FF) | (hasSecHdr?1:0)<<11  [PVN=0, PacketType=0 always]. sequenceCount = getApidSeqCount_out(0, apid, 0) (second arg unused). packetSequenceControl = ((seqFlags<<14) & 0xC000) | (sequenceCount & 0x3FFF). FW_ASSERT(data.size <= 0xFFFF); packetDataLength = data.size - 1. Serialize header then raw data with OMIT_LENGTH; setSize(frameSize); dataOut_out(0, frameBuffer, context) [context passed unmodified]; then dataReturnOut_out(0, data, context). comStatusIn_handler: pass-through to comStatusOut_out(portNum, condition) if connected. dataReturnIn_handler: bufferDeallocate_out(0, frameBuffer).

### SpacePacketDeframer

Files: `Svc/Ccsds/SpacePacketDeframer/SpacePacketDeframer.fpp`, `Svc/Ccsds/SpacePacketDeframer/SpacePacketDeframer.cpp`

Passive. imports Deframer; extra output ports validateApidSeqCount(Ccsds.ApidSequenceCount), errorNotify(Ccsds.ErrorNotify). Events (relative ids in decl order): 0 InvalidPacket() WARNING_HI "Malformed packet received refusing to deframe"; 1 InvalidLength(transmitted: FwSizeType, actual: FwSizeType) WARNING_HI "Invalid length received. Header specified packet byte size of {} | Actual received data length: {}". dataIn_handler order: (1) if data.size <= 6 -> InvalidPacket + errorNotify(SP_INVALID_PACKET) if connected + dataReturnOut(original context) + return. (2) deserializeTo(SpacePacketHeader) failure -> same InvalidPacket path. (3) PVN = (packetIdentification & 0xE000)>>13 must == 0 (Pvn::SPACE_PACKET_PROTOCOL) else InvalidPacket path. (4) pkt_length = U32(packetDataLength)+1; if pkt_length > (data.size - 6) -> InvalidLength(pkt_length, data.size-6) + errorNotify(SP_INVALID_LENGTH) + dataReturnOut + return. (5) apidValue = packetIdentification & 0x07FF; contextCopy.apid = isValid(apidValue) ? apidValue : Apid::INVALID_UNINITIALIZED(0x0800); contextCopy.hasSecHdr = (packetIdentification & 0x0800)!=0; contextCopy.sequenceFlags = (packetSequenceControl & 0xC000)>>14; receivedSequenceCount = packetSequenceControl & 0x3FFF; (void)validateApidSeqCount_out(0, apid, receivedSequenceCount) [return value discarded]; contextCopy.sequenceCount = receivedSequenceCount. (6) data.advance(6); data.setSize(pkt_length); dataOut_out(0, data, contextCopy). dataReturnIn_handler: straight pass-through to dataReturnOut_out(0, data, context). Does NOT read/write vcId, pvn, saIndex, comQueueIndex; does NOT check the Packet Type bit.

### ApidManager

Files: `Svc/Ccsds/ApidManager/ApidManager.fpp`, `Svc/Ccsds/ApidManager/ApidManager.cpp`, `Svc/Ccsds/ApidManager/ApidManager.hpp`

Passive; both input ports GUARDED (single mutex; shared by framer and deframer threads). Ports: guarded input validateApidSeqCountIn: Ccsds.ApidSequenceCount; guarded input getApidSeqCountIn: Ccsds.ApidSequenceCount. Only ports timeCaller/logTextOut/logOut (no tlm, no params). Event (relative id 0): UnexpectedSequenceCount(transmitted: U16, expected: U16) severity WARNING_LO "Unexpected sequence count received. Packets may have been dropped. Transmitted: {} | Expected on board: {}". State: Fw::ArrayMap<Apid::T,U16, MAX_TRACKED_APIDS=ComCfg::Apid::NUM_CONSTANTS(12)> m_apidSequences, empty at construction. getAndIncrementSeqCount(apid): seqCount=0; (void)find(apid,&seqCount) (miss leaves 0); insert(apid, calculateNextSeqCount(seqCount)) FW_ASSERT success; return seqCount (the PRE-increment value). calculateNextSeqCount(c) = (c+1) % (1<<14) = (c+1) & 0x3FFF. getApidSeqCountIn_handler(portNum, apid, unused) -> getAndIncrementSeqCount(apid). validateApidSeqCountIn_handler(portNum, apid, receivedSeqCount): expected = getAndIncrementSeqCount(apid); if received != expected -> log_WARNING_LO_UnexpectedSequenceCount(received, expected) and insert(apid, calculateNextSeqCount(received)) (resync onboard counter to received+1); ALWAYS returns receivedSeqCount. Static asserts enforce SPP_IDLE_PACKET==0x07FF, INVALID_UNINITIALIZED==0x0800, and that Apid values match Fw::ComPacketType for COMMAND/TELEM/LOG/FILE/PACKETIZED_TLM/UNKNOWN.

### TmFramer

Files: `Svc/Ccsds/TmFramer/TmFramer.fpp`, `Svc/Ccsds/TmFramer/TmFramer.cpp`, `Svc/Ccsds/TmFramer/TmFramer.hpp`

Passive; imports Framer only (no extra ports). NO events, NO telemetry, NO commands. State: U8 m_frameBuffer[ComCfg::TmFrameFixedSize=1024]; BufferOwnershipState m_bufferState {NOT_OWNED, OWNED} init OWNED; U8 m_masterFrameCount=0; U8 m_virtualFrameCount=0. Constants: IDLE_DATA_PATTERN=0x44; TmPayloadCapacity = 1024-(6+2)=1016; SppOverhead = 2*6+1 = 13; static_asserts TmPayloadCapacity >= FW_COM_BUFFER_MAX_SIZE+13 and >= FW_FILE_BUFFER_MAX_SIZE+13. dataIn_handler: FW_ASSERT(data.size <= 1024-6-2); FW_ASSERT(m_bufferState==OWNED). globalVcId = (context.vcId << 1) | (ComCfg::SpacecraftId << 4) | 0 (OCF flag 0); TFVN bits [15:14]=00 implicitly. dataFieldStatus = 0x3 << 11 (segment length id 0b11); all other bits 0 (sec hdr flag 0, sync flag 0, packet order 0, first header pointer 0). Set header fields, then m_masterFrameCount++ and m_virtualFrameCount++ (U8 wrap mod 256) AFTER use. Wrap m_frameBuffer in Fw::Buffer(size 1024); serialize header (6B), then data with OMIT_LENGTH, then fill_with_idle_packet, then CRC16 over first 1022 bytes of the frame buffer, moveSerToOffset(1024-2), serialize TMTrailer{fecf}. m_bufferState = NOT_OWNED; dataOut_out(0, frameBuffer, context) [context unmodified]; dataReturnOut_out(0, data, context). comStatusIn_handler: pass-through if connected. dataReturnIn_handler: FW_ASSERT returned pointer lies within m_frameBuffer; m_bufferState = OWNED (no deallocation — buffer is reused). fill_with_idle_packet(serializer): endIndex=1022; startIndex = serializer.getSize() (= 6 + data.size); idlePacketSize = endIndex - startIndex; lengthToken = idlePacketSize - 6 - 1; FW_ASSERT(idlePacketSize >= 7) and (<= 1024); SpacePacketHeader{packetIdentification = 0x07FF (SPP_IDLE_PACKET; PVN=0,type=0,secHdr=0), packetSequenceControl = 0x3<<14 = 0xC000, packetDataLength = lengthToken}; then write IDLE_DATA_PATTERN (0x44) for every byte from startIndex+6 to endIndex-1.

### TcDeframer

Files: `Svc/Ccsds/TcDeframer/TcDeframer.fpp`, `Svc/Ccsds/TcDeframer/TcDeframer.cpp`, `Svc/Ccsds/TcDeframer/TcDeframer.hpp`

Passive; imports Deframer; extra output port errorNotify: Ccsds.ErrorNotify. Events (relative ids in decl order): 0 InvalidPacket() WARNING_LO "Invalid packet received refusing to deframe"; 1 InvalidSpacecraftId(transmitted:U16, configured:U16) WARNING_LO; 2 InvalidFrameLength(transmitted:U16, actual:FwSizeType) WARNING_HI; 3 InvalidVcId(transmitted:U16, configured:U16) ACTIVITY_LO; 4 InvalidCrc(transmitted:U16, computed:U16) WARNING_HI. No telemetry/commands. State: U16 m_vcId (uninitialized by ctor), U16 m_spacecraftId = ComCfg::SpacecraftId (0x0044), bool m_acceptAllVcid = true. configure(vcId, spacecraftId, acceptAllVcid) sets all three. dataIn_handler check order: (1) data.size <= 5+2=7 -> log_WARNING_LO_InvalidPacket, dataReturnOut, return (NO errorNotify emitted on this path). (2) deserializeTo(TCHeader) — FW_ASSERT on failure. total_frame_length = (vcIdAndLength & 0x03FF) + 1; vc_id = (vcIdAndLength & 0xFC00) >> 10; spacecraft_id = flagsAndScId & 0x03FF. (3) spacecraft_id != m_spacecraftId -> InvalidSpacecraftId(spacecraft_id, m_spacecraftId) + errorNotify(TC_INVALID_SCID) + drop. (4) data.size < total_frame_length OR total_frame_length < 7 -> InvalidFrameLength(total_frame_length, data.size) + errorNotify(TC_INVALID_LENGTH) + drop. (5) !m_acceptAllVcid && vc_id != m_vcId -> InvalidVcId(vc_id, m_vcId) + errorNotify(TC_INVALID_VCID) + drop. (6) computed_crc = CRC16 over data[0 .. total_frame_length-2); trailer read at offset total_frame_length-2; mismatch -> log_WARNING_HI_InvalidCrc(computed_crc, transmitted_crc) + errorNotify(TC_INVALID_CRC) + drop. (7) success: data.advance(5); data.setSize(total_frame_length - 5 - 2); dataOut_out(0, data, context) with the context UNMODIFIED (does not set vcId/apid). No FARM/sequence-number checks — F Prime uses Type-BD frames, frameSequenceNum ignored. dataReturnIn_handler: pass-through to dataReturnOut.

### Utils::CRC16 (CRC-16/CCITT-FALSE)

Files: `Svc/Ccsds/Utils/CRC16.hpp`, `Utils/Hash/libcrc/lib_crc.c`

class CRC16 { U16 m_crc = 0xFFFF; void update(U8 b); U16 finalize() { return m_crc ^ 0x0000; } static U16 compute(const U8* buf, U32 len); }. Algorithm = libcrc update_crc_ccitt: table-driven, MSB-first (non-reflected), P_CCITT = 0x1021, init 0xFFFF, no input/output reflection, final XOR 0x0000. Byte step: tmp = (crc >> 8) ^ byte; crc = (crc << 8) ^ table[tmp]. This is CRC-16/CCITT-FALSE (a.k.a. CRC-16/IBM-3740); check value CRC("123456789") = 0x29B1. compute() FW_ASSERTs buffer != nullptr.

### CcsdsTcFrameDetector (uplink frame sync)

Files: `Svc/FrameAccumulator/FrameDetector/CcsdsTcFrameDetector.hpp`, `Svc/FrameAccumulator/FrameDetector/CcsdsTcFrameDetector.cpp`

m_expectedFlagsAndScIdToken = (0x1 << TCSubfields::BypassFlagOffset(13)) | ComCfg::SpacecraftId = 0x2000 | 0x0044 = 0x2044 (TFVN=00, bypass=1, ctrl=0, reserved=00, SCID=0x044). detect(): allocated < 7 -> MORE_DATA_NEEDED(size_out=7). peek 5 header bytes, deserialize TCHeader; flagsAndScId != 0x2044 -> NO_FRAME_DETECTED. expected_frame_length = (vcIdAndLength & 0x03FF)+1; allocated < expected -> MORE_DATA_NEEDED(expected); expected < 7 -> NO_FRAME_DETECTED; CRC16 computed byte-by-byte over expected-2 bytes peeked from the ring; peek 2 trailer bytes at offset expected-2; mismatch -> NO_FRAME_DETECTED; else FRAME_DETECTED(size_out = expected_frame_length).

### ComCcsds subtopology wiring

Files: `Svc/Subtopologies/ComCcsds/ComCcsds.fpp`

topology SpacePacketFraming: comQueue.dataOut -> spacePacketFramer.dataIn; spacePacketFramer.dataReturnOut -> comQueue.dataReturnIn; spacePacketFramer.bufferAllocate -> commsBufferManager.bufferGetCallee; .bufferDeallocate -> commsBufferManager.bufferSendIn; .getApidSeqCount -> apidManager.getApidSeqCountIn; spacePacketFramer.dataOut -> aggregator.dataIn; aggregator.dataReturnOut -> spacePacketFramer.dataReturnIn; aggregator.comStatusOut -> spacePacketFramer.comStatusIn -> comQueue.comStatusIn. Uplink: spacePacketDeframer.validateApidSeqCount -> apidManager.validateApidSeqCountIn; spacePacketDeframer.dataOut -> fprimeRouter.dataIn; fprimeRouter.dataReturnOut -> spacePacketDeframer.dataReturnIn. topology TmTcFraming boxes framer(TmFramer) + frameAccumulator + tcDeframer. Aggregator (active) batches multiple space packets up to ComCfg.AggregationSize=1009 before handing one buffer to TmFramer.

## Wire formats

### CCSDS Space Packet Primary Header (6 bytes, big-endian)

Byte0-1 packetIdentification: bits[15:13]=PVN (always 0b000 for SPP), bit[12]=Packet Type (0=TM/report, 1=TC/command; framer always emits 0), bit[11]=Secondary Header Flag, bits[10:0]=APID (11 bits). Byte2-3 packetSequenceControl: bits[15:14]=Sequence Flags (00=continuation, 01=first, 10=last, 11=unsegmented), bits[13:0]=Packet Sequence Count (14 bits, wraps mod 16384). Byte4-5 packetDataLength = (number of octets in packet data field) - 1; total packet size = 6 + packetDataLength + 1. Masks (Types.fpp SpacePacketSubfields): PvnMask=0xE000/off 13, PktTypeMask=0x1000/off 12, SecHdrMask=0x0800/off 11, ApidMask=0x07FF, SeqFlagsMask=0xC000/off 14, SeqCountMask=0x3FFF, ApidWidth=11, SeqCountWidth=14.

### SPP Idle Packet (as emitted by TmFramer fill)

6-byte primary header with packetIdentification = 0x07FF (PVN=0, type=0, secHdr=0, APID=0x7FF = SPP_IDLE_PACKET), packetSequenceControl = 0xC000 (seq flags 0b11, count 0), packetDataLength = idlePacketSize-7. Followed by (idlePacketSize-6) bytes of 0x44. Minimum idle packet size is 7 bytes (6 header + 1 data byte).

### TM Transfer Frame (fixed 1024 bytes = 6 header + data field + 2 trailer)

Primary header, big-endian: Byte0-1 globalVcId: bits[15:14]=Transfer Frame Version Number (00 => 'version 1'), bits[13:4]=Spacecraft ID (10 bits, ComCfg::SpacecraftId=0x044), bits[3:1]=Virtual Channel ID (3 bits, from context.vcId), bit[0]=Operational Control Field Flag (always 0). Byte2 masterFrameCount (U8, wraps mod 256). Byte3 virtualFrameCount (U8, wraps mod 256). Byte4-5 dataFieldStatus: bit[15]=Transfer Frame Secondary Header Flag (0), bit[14]=Synchronization Flag (0), bit[13]=Packet Order Flag (0), bits[12:11]=Segment Length Identifier (0b11 = 0x3<<11 = 0x1800), bits[10:0]=First Header Pointer (0, payload starts at offset 0 of the data field). Data field bytes 6..1021: the payload packet immediately followed by exactly one SPP idle packet filling to offset 1022. Trailer bytes 1022..1023: FECF = CRC-16/CCITT-FALSE over frame bytes [0,1022). Offsets in TMSubfields: frameVersionOffset=14, spacecraftIdOffset=4, virtualChannelIdOffset=1, segLengthOffset=11.

### TC Transfer Frame (5-byte header + data field + 2-byte FECF)

Byte0-1 flagsAndScId: bits[15:14]=Transfer Frame Version Number (00), bit[13]=Bypass Flag (1 for Type-B expedited; the frame detector requires 1), bit[12]=Control Command Flag (0 = Type-D data), bits[11:10]=Reserved Spare (00), bits[9:0]=Spacecraft ID. Byte2-3 vcIdAndLength: bits[15:10]=Virtual Channel ID (6 bits), bits[9:0]=Frame Length (total frame octets minus 1; max 1024). Byte4 frameSequenceNum (U8, unused for Type-B, never checked). Data field bytes 5..(len-3). Trailer last 2 bytes: FECF = CRC-16/CCITT-FALSE over bytes [0, total_frame_length-2). Masks (TCSubfields): FrameVersionMask=0xC000, BypassFlagMask=0x2000 (offset 13), ControlFlagMask=0x1000, ReservedMask=0x0C00, SpacecraftIdMask=0x03FF, VcIdMask=0xFC00 (offset 10), FrameLengthMask=0x03FF.

### Frame Error Control Field (both TM and TC, and AOS)

U16 big-endian. CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, non-reflected input and output, final XOR 0x0000. TM: covers the entire fixed frame except the last 2 bytes (bytes 0..1021 of 1024, i.e. header + payload + idle fill). TC: covers bytes 0..(total_frame_length-3), i.e. header + data field, excluding the 2 FECF bytes.

### AOS header (declared in Types.fpp; consumed by AosFramer/AosDeframer, out of primary scope)

AOSHeader 6 bytes: globalVcId U16 {bits[15:14]=TFVN (01 for AOS), bits[13:6]=SCID LSBs (8 bits), bits[5:0]=VCID}, frameCountAndSignaling U32 {bits[31:8]=24-bit VC frame count, bit[7]=replay flag, bit[6]=VC frame count cycle use flag, bits[5:4]=SCID MSBs, bits[3:0]=VC frame count cycle}. M_PDUHeader U16 firstHeaderPointer, special values FHP_NO_PACKET_START=0xFFFF, FHP_IDLE_DATA_ONLY=0xFFFE. AOSTrailer U16 fecf. EPPSubfields first octet: packetVersionMask=0xE0 (offset 5), protocolIdMask=0x1C (offset 2), lengthOfLengthMask=0x03. PvnBitfield: SPP_MASK=0x01, EPP_MASK=0x80, VALID_MASK=0x81.

## Threading / concurrency

All five components are PASSIVE — no threads, no queues. SpacePacketFramer/TmFramer dataIn and dataReturnIn are `sync` (execute on the caller's thread — normally the ComQueue active thread for dataIn and the driver/ComStub thread for dataReturnIn). SpacePacketDeframer/TcDeframer dataIn is `guarded` (component mutex) and dataReturnIn is `sync`. ApidManager's two input ports are both `guarded`, which is the only synchronization protecting m_apidSequences — it is shared between the downlink thread (getApidSeqCountIn from SpacePacketFramer) and the uplink thread (validateApidSeqCountIn from SpacePacketDeframer). TmFramer's m_frameBuffer/m_bufferState is a single-slot ownership handshake with no atomics: dataIn asserts OWNED then sets NOT_OWNED, dataReturnIn asserts the pointer belongs to m_frameBuffer and restores OWNED. Because dataIn is `sync` and not guarded, the framer relies on the ComQueue one-outstanding-send flow control (comStatus SUCCESS gating) to guarantee only one frame is in flight. ComAggregator (active) sits between SpacePacketFramer and TmFramer in the ComCcsds topology.

## Porting notes

1) Put the header structs in a `fprime-svc` (or new `fprime-ccsds`) module as plain structs implementing the existing fw Serializable/Deserializable traits with big-endian encoding — SpacePacketHeader{u16,u16,u16}, TMHeader{u16,u8,u8,u16}, TMTrailer{u16}, TCHeader{u16,u16,u8}, TCTrailer{u16}, AOSHeader{u16,u32}, M_PDUHeader{u16}, AOSTrailer{u16}. Do NOT hand-roll byte packing; reuse the fw serializer so SERIALIZED_SIZE stays authoritative (6/6/2/5/2/6/2/2).
2) Expose bitfield accessors as const fns over the raw u16 words plus the exact mask/offset constants from Types.fpp — keep the raw word in the struct (that is how C++ stores it) so round-tripping is byte-exact.
3) CRC16: implement as a 256-entry table generated from poly 0x1021 MSB-first, init 0xFFFF, no reflection, xorout 0. Zero-dependency and trivially const-fn generatable at compile time. Unit-test against CRC("123456789") == 0x29B1. Provide both a streaming `Crc16{crc:u16}` with `update(u8)/finalize()` (needed by the frame detector's byte-by-byte ring peek) and a `compute(&[u8])`.
4) ApidManager: a fixed-capacity array map of (Apid, u16) with capacity = number of Apid enum constants (12). `get_and_increment(apid) -> u16` returns the pre-increment value and stores (v+1) & 0x3FFF; `validate(apid, received) -> u16` compares to `get_and_increment`, emits UnexpectedSequenceCount(received, expected) on mismatch, resyncs to (received+1)&0x3FFF, and always returns `received`. Insert failure is an assert in C++ — in Rust make it an unreachable/panic with the same capacity invariant.
5) TmFramer: model the internal frame as `[u8; TM_FRAME_FIXED_SIZE]` plus an `owned: bool`. Preserve the exact ordering: header, payload, idle packet, then CRC over [0, SIZE-2) computed before writing the trailer at SIZE-2. Note the counters increment AFTER being written into the header (first frame carries 0/0).
6) Preserve the exact validation ORDER and the exact drop-path behavior in both deframers: the C++ returns the ORIGINAL (unmodified) context on every drop, and only forwards a mutated context copy on success. SpacePacketDeframer emits errorNotify only when the port is connected; TcDeframer's first size check (`<= 7`) emits NO errorNotify at all.
7) Keep `errorNotify` and `getApidSeqCount`/`validateApidSeqCount` as optional ports (Option<PortHandle>) — connection checks are semantically load-bearing.
8) Reuse the existing fprime-comp Framer/Deframer port-trait shapes; SpacePacketFramer/TmFramer additionally need comStatusIn -> comStatusOut pass-through, gated on comStatusOut being connected at the SAME port index.
9) Config constants (SpacecraftId, TmFrameFixedSize, AggregationSize, the Apid enum, IDLE_DATA_PATTERN=0x44) belong in a single `ComCfg`-equivalent module, ideally generic/const-parameterized so a deployment can change TmFrameFixedSize.
10) Reproduce CcsdsTcFrameDetector as a FrameDetector trait impl over the circular buffer, including its MORE_DATA_NEEDED(7) / MORE_DATA_NEEDED(expected) / NO_FRAME_DETECTED contracts and the underflow guard before computing expected-2.

## Rust feasibility (safe, zero-dependency std)

Everything in this subsystem is portable to safe zero-dependency std Rust. The CRC16 dependency on the vendored C libcrc (`Utils/Hash/libcrc/lib_crc.c` update_crc_ccitt) is only a 256-entry table lookup with polynomial 0x1021 MSB-first — regenerate the table with a `const fn` at compile time; no FFI, no zlib, no ioctl anywhere in Svc/Ccsds's core framing path. The only non-mechanical translations are: (a) Fw::Buffer's advance()/setSize() in-place windowing, which maps to slice ranges or an offset+len view over an owned buffer handle (the deframers mutate the SAME allocation rather than copying, so the Rust type must support a re-slice that the return path can still map back to the original allocator); (b) FW_ASSERT-on-failure paths become panics or debug_asserts — keep them as panics where the C++ asserts, since several of them (TmFramer buffer-ownership, APID width) encode real invariants; (c) TmFramer's static member buffer handed out as an Fw::Buffer is aliasing that Rust will not accept directly — model it as an owned `Box<[u8; 1024]>`/array moved out on dataOut and moved back on dataReturnIn, or an ownership token plus interior mutability guarded by the same OWNED/NOT_OWNED flag. The ArrayMap used by ApidManager is a plain fixed-capacity assoc array; no allocation needed. Note the AOS/SDLS/CFDP siblings in Svc/Ccsds (out of scope here) do have heavier dependencies (crypto, file I/O) but the TM/TC/SPP path does not.

## Gotchas

- TcDeframer::InvalidCrc argument order is INVERTED relative to the FPP declaration: the event is declared `InvalidCrc(transmitted: U16, computed: U16)` but the call site is `log_WARNING_HI_InvalidCrc(computed_crc, transmitted_crc)` (TcDeframer.cpp). Reproduce the C++ behavior only if bit-compatibility with the ground dictionary output matters; otherwise flag it.
- TcDeframer's `m_vcId` is NOT initialized by the constructor (only m_spacecraftId is). It is safe only because m_acceptAllVcid defaults to true; in Rust initialize it to 0 and keep accept_all_vcid=true as the default.
- TmFramer builds globalVcId as `context.vcId << 1` with NO mask — a vcId > 7 silently corrupts the spacecraft-ID field. Also note FrameContext's default vcId is 1 (not 0, despite what TmFramer's sdd.md table claims).
- SpacePacketDeframer rejects size <= 6 (strictly greater required), so a header-only 6-byte packet is dropped as InvalidPacket even though packetDataLength=0 would imply a 7-byte packet.
- SpacePacketDeframer sets apid = INVALID_UNINITIALIZED (0x0800) for any 11-bit APID not present in the ComCfg::Apid enum; SPP_IDLE_PACKET (0x7FF) IS a valid enum member so uplinked idle packets are forwarded to the router rather than dropped.
- The SpacePacketDeframer discards the return value of validateApidSeqCount (`(void)`), and stores the RECEIVED count into context, not the expected one. The validation is purely for the event/resync side effect.
- TmFramer's dataIn FW_ASSERT allows payloads up to 1016 bytes, but fill_with_idle_packet then FW_ASSERTs idlePacketSize >= 7, so the real maximum payload is 1009 bytes (= ComCfg.AggregationSize). Two different asserts guard the same constraint at different limits.
- TmFramer computes CRC over `sizeof(m_frameBuffer) - 2` (the whole fixed frame minus trailer), not over the serializer's current length — correct only because the idle fill always reaches offset 1022. If the idle fill were skipped the CRC would cover stale bytes.
- TmFramer never deallocates: dataReturnIn just flips ownership of the static member buffer back. Downstream consumers MUST copy the frame before returning it. SpacePacketFramer, by contrast, deallocates its allocated frame buffer on dataReturnIn.
- Packet Data Length semantics differ per layer but are both 'minus one': SPP packetDataLength = data-field octets - 1 (total = 6 + len + 1); TC Frame Length = TOTAL frame octets - 1 (including the 5-byte header and 2-byte FECF).
- SpacePacketFramer hardcodes PVN=0 and Packet Type=0 (telemetry) — it can never emit a command packet; there is no field in FrameContext for either.
- The FECF CRC is CRC-16/CCITT-FALSE (init 0xFFFF, non-reflected), which is a different algorithm from the F Prime frame CRC-32/ISO-HDLC used by FprimeFramer — do not share code between them.
- CcsdsTcFrameDetector matches the first header word EXACTLY against 0x2044 (bypass flag set, TFVN 00, control 0, reserved 00, SCID 0x044): Type-A (bypass=0) TC frames will never be detected by the accumulator even though TcDeframer itself does not check the bypass flag.
- ApidManager's counter is 14-bit and wraps at 0x3FFF; the first packet for a never-seen APID carries sequence count 0 (map miss leaves seqCount=0 and inserts 1).

