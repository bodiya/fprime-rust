//! # CmdSequencer — port of `Svc::CmdSequencer` (active)
//!
//! C++ sources: `Svc/CmdSequencer/{CmdSequencerImpl.cpp,CmdSequencerImpl.hpp,
//! FPrimeSequence.cpp,Sequence.cpp,Events.cpp,CmdSequencer.fpp,Commands.fppi,
//! Events.fppi,Telemetry.fppi}`, `Svc/Seq/Seq.fpp`.
//! Analysis: `docs/cpp-analysis/cmd-sequencer.md` (normative).
//!
//! Loads a binary command sequence into one pre-allocated buffer, validates
//! it (CRC-32, time base/context, record structure), then walks the records
//! emitting `Fw::ComBuffer` command packets on `comCmdOut`, waiting for each
//! `cmdResponseIn` before advancing.
//!
//! Two *orthogonal* state variables:
//!
//! - [`RunMode`] `{Stopped = 0, Running = 1}` (`m_runMode`) — is a sequence
//!   executing right now,
//! - [`StepMode`] `{Auto = 0, Manual = 1}` (`m_stepMode`) — does the next
//!   record advance on its own or only on `CS_STEP`.
//!
//! The *reported* mode enum [`SeqMode`] `{Step = 0, Auto = 1}` is inverted
//! relative to `StepMode`: `CS_AUTO` logs `SeqMode::Auto` (1) and
//! `CS_MANUAL` logs `SeqMode::Step` (0). Both are kept, exactly as in C++.
//!
//! Ported quirks (each covered by a unit test):
//! - The header's `fileSize` counts the trailing 4-byte CRC: records length
//!   is `fileSize - 4` and the file is `11 + fileSize` bytes long.
//! - The CRC covers header(11) + records(`fileSize - 4`) and NOT the stored
//!   CRC bytes themselves.
//! - Record time tags carry ONLY seconds/useconds; the time base and
//!   context are stamped from the (canonicalized) header in
//!   `perform_cmd_step`, after `next_record`.
//! - `Header::validateTime` CANONICALIZES the header, which is what makes
//!   `TB_DONT_CARE`/`FW_CONTEXT_DONT_CARE` sequences run.
//! - [`Timer::is_expired_at`] treats INCOMPARABLE (mismatched time base) as
//!   EXPIRED — it only rejects the `Gt` case.
//! - `deserialize_record_size` rejects `recordSize + 2 > 512`, over-strict
//!   by two bytes because `recordSize` already includes the packet
//!   descriptor. Ported verbatim.
//! - `perform_cmd_cancel` calls `reset()` (rewind), not `clear()`, so
//!   `CS_START` can rerun the same sequence.
//! - Component-level `load_file` calls `clear()` on EVERY failure so a
//!   partially populated buffer cannot make `has_more_records()` true.
//! - `CS_RecordMismatch` (12) is the only sequence event that does NOT
//!   bump `CS_Errors`.
//! - `CS_JOIN_WAIT` logs `CS_JoinWaiting` with the PREVIOUS `m_cmdSeq` /
//!   `m_opCode` before overwriting them.
//! - `do_sequence_run` invokes `seqDone` on its error paths WITHOUT an
//!   is-connected guard, while cancel/complete DO guard.
//! - `schedIn` uses `else if`: a due timed dispatch suppresses the timeout
//!   check on that tick, and it calls `getTime()` a second time.
//! - `set_cmd_timeout` arms the watchdog only when `timeout > 0` AND the
//!   step mode is AUTO.
//!
//! Threading: ACTIVE with one task and one queue; every input port is
//! ASYNC, so all handlers run on the component thread. There are no guarded
//! ports; the `Mutex<CmdSequencerState>` is the workspace convention for
//! component state and is held for the whole handler, mirroring the C++
//! single-threaded component.

use fprime_comp::{
    ActiveBase, ActiveComponent, CmdGlue, CmdPort, CmdResponsePort, ComPort, ComponentDispatch,
    EventGlue, MsgDispatchStatus, OutputPort, PingPort, QueueFullPolicy, SchedPort, TlmGlue,
    async_input_port_adapter, component_msg_types, msg,
};
use fprime_config::cmd_dispatcher::INCLUDE_COMMAND_OPCODES_IN_EVENTS;
use fprime_config::{
    FW_COM_BUFFER_MAX_SIZE, FW_CONTEXT_DONT_CARE, FwChanIdType, FwEnumStoreType, FwEventIdType,
    FwIdType, FwIndexType, FwOpcodeType, FwPacketDescriptorType, FwQueuePriorityType, FwSizeType,
};
use fprime_fw::{
    CmdArgBuffer, CmdResponse, CmdStringArg, ComBuffer, Endianness, ExtBuf, FileNameString,
    FwDefaultString, LengthMode, LogBuffer, LogSeverity, LogStringArg, SerBuf, SerBufAny,
    SerializeStatus, Time, TimeBase, TimeComparison, fpp_array, fpp_enum, fpp_struct, fw_assert,
    fw_try,
};
use fprime_os::file::{Mode as FileMode, Status as FileStatus, WaitType};
use fprime_utils::Hash;
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Svc/Seq types and ports (Svc/Seq/Seq.fpp).
// ---------------------------------------------------------------------------

/// `Svc.SequenceArgumentsMaxSize`:
/// `FW_CMD_ARG_BUFFER_MAX_SIZE(506) - sizeof(FwSizeStoreType)(2)
/// - FileNameStringSize(240) - sizeof(U8)(1) - sizeof(FwSizeType)(8)`.
pub const SEQUENCE_ARGUMENTS_MAX_SIZE: usize = 255;

fpp_array! {
    /// The raw argument bytes of [`SeqArgs`] (`[SequenceArgumentsMaxSize] U8`).
    /// Serializes as 255 consecutive bytes — no count prefix, no padding.
    #[derive(Clone, Copy, Eq)]
    pub array SeqArgsBuffer = [u8; SEQUENCE_ARGUMENTS_MAX_SIZE]
    default fill 0
}

fpp_struct! {
    /// `Svc.SeqArgs` — arguments handed to a sequence over `Svc.CmdSeqIn`.
    /// Wire format: `[size u64 BE][255 raw bytes]` (263 bytes).
    ///
    /// `CmdSequencer` ignores the arguments it receives and always emits an
    /// all-zero value on `seqStartOut` (C++ `(void)args;`).
    #[derive(Clone, Copy, Eq)]
    pub struct SeqArgs {
        /// Number of valid bytes in `buffer`.
        size: FwSizeType { get_size, set_size },
        /// Raw argument bytes.
        buffer: SeqArgsBuffer { get_buffer, set_buffer },
    }
}

fpp_enum! {
    /// `Svc.BlockState` — whether `CS_RUN` defers its command response until
    /// the sequence finishes.
    pub enum BlockState : u8 {
        /// Answer the command only when the sequence completes or fails.
        Block = 0,
        /// Answer the command immediately.
        NoBlock = 1,
    }
    default Block
}

/// `Svc.CmdSeqIn` — request that a sequence file be run.
pub trait CmdSeqInPort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType, filename: &FileNameString, args: &SeqArgs);
}

/// `Svc.CmdSeqCancel` — cancel the running sequence (no arguments).
pub trait CmdSeqCancelPort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType);
}

/// `Svc.FileDispatch` — dispatch a file name (by reference, C++ `ref`).
pub trait FileDispatchPort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType, file_name: &mut FileNameString);
}

// ---------------------------------------------------------------------------
// Component-declared enums (CmdSequencer.fpp).
// ---------------------------------------------------------------------------

fpp_enum! {
    /// `CmdSequencer.SeqMode` — the mode reported by `CS_ModeSwitched`.
    ///
    /// NOTE the inversion against [`StepMode`]: `CS_AUTO` reports
    /// `SeqMode::Auto` (1) while the internal `StepMode::Auto` is 0.
    pub enum SeqMode : u8 {
        /// Manual stepping (internal `StepMode::Manual`).
        Step = 0,
        /// Automatic stepping (internal `StepMode::Auto`).
        Auto = 1,
    }
    default Step
}

fpp_enum! {
    /// `CmdSequencer.FileReadStage` — where a sequence file read failed.
    pub enum FileReadStage : u8 {
        /// `Os::File::read` of the 11 header bytes failed.
        ReadHeader = 0,
        /// The header read returned fewer than 11 bytes.
        ReadHeaderSize = 1,
        /// Deserializing the header's `fileSize` failed.
        DeserSize = 2,
        /// Deserializing the header's `numRecords` failed.
        DeserNumRecords = 3,
        /// Deserializing the header's time base failed.
        DeserTimeBase = 4,
        /// Deserializing the header's time context failed.
        DeserTimeContext = 5,
        /// The file held fewer than the four CRC bytes.
        ReadSeqCrc = 6,
        /// `Os::File::read` of the record block failed.
        ReadSeqData = 7,
        /// The record-block read returned fewer bytes than `fileSize`.
        ReadSeqDataSize = 8,
    }
    default ReadHeader
}

/// `CmdSequencerComponentImpl::RunMode` — is a sequence executing?
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(i32)]
pub enum RunMode {
    /// No sequence is executing.
    #[default]
    Stopped = 0,
    /// A sequence is executing.
    Running = 1,
}

/// `CmdSequencerComponentImpl::StepMode` — how the next record advances.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(i32)]
pub enum StepMode {
    /// Records advance automatically on each command response.
    #[default]
    Auto = 0,
    /// Records advance only on `CS_STEP`.
    Manual = 1,
}

// ---------------------------------------------------------------------------
// Sequence format: header, records, and the pluggable `Sequence` trait.
// ---------------------------------------------------------------------------

/// Bytes of `Sequence::Header::SERIALIZED_SIZE`:
/// `U32 fileSize + U32 numRecords + U16 timeBase + U8 timeContext`.
pub const SEQUENCE_HEADER_SIZE: usize = 11;

/// Size of the trailing CRC field.
pub const SEQUENCE_CRC_SIZE: usize = 4;

/// `CmdSequencerComponentImpl::Sequence::Header`.
///
/// Wire layout (big-endian, offset 0 of the file):
/// `[fileSize u32][numRecords u32][timeBase u16][timeContext u8]`.
/// `fileSize` counts the bytes AFTER the header INCLUDING the 4-byte CRC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SequenceHeader {
    /// Bytes after the header, including the trailing CRC.
    pub file_size: u32,
    /// Number of records the generator wrote.
    pub num_records: u32,
    /// Sequence time base (`TbDontCare` = accept any).
    pub time_base: TimeBase,
    /// Sequence time context (`FW_CONTEXT_DONT_CARE` = accept any).
    pub time_context: u8,
}

impl Default for SequenceHeader {
    /// C++ `Header()` — the don't-care sentinels.
    fn default() -> Self {
        Self {
            file_size: 0,
            num_records: 0,
            time_base: TimeBase::TbDontCare,
            time_context: FW_CONTEXT_DONT_CARE,
        }
    }
}

/// `Sequence::Record::Descriptor`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum RecordDescriptor {
    /// Absolute time tag: dispatch when wall time >= tag.
    Absolute = 0,
    /// Relative time tag: current time is added at step time.
    Relative = 1,
    /// End of sequence — a one-byte record with no further fields.
    #[default]
    EndOfSequence = 2,
}

/// `CmdSequencerComponentImpl::Sequence::Record`.
#[derive(Debug, Clone, Default)]
pub struct SequenceRecord {
    /// Record kind.
    pub descriptor: RecordDescriptor,
    /// Time tag; the wire carries only seconds/useconds, the base and
    /// context are stamped from the header at step time.
    pub time_tag: Time,
    /// The complete command com packet to emit.
    pub command: ComBuffer,
}

/// The single event a [`Sequence::load_file`] attempt may report.
///
/// C++ routes these through `Sequence::Events`, which also bumps the
/// component's error counter for every variant except
/// [`SequenceLoadEvent::RecordMismatch`]. Every failing path emits at most
/// one event and returns immediately, so an `Option` is faithful.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SequenceLoadEvent {
    /// `CS_FileNotFound` (6).
    FileNotFound,
    /// `CS_FileReadError` (2).
    FileReadError,
    /// `CS_FileInvalid` (3).
    FileInvalid {
        /// Stage at which the read/deserialize failed.
        stage: FileReadStage,
        /// `Os::File::Status` or `Fw::SerializeStatus` discriminant.
        error: i32,
    },
    /// `CS_FileSizeError` (5).
    FileSizeError {
        /// The rejected `fileSize`.
        size: u32,
    },
    /// `CS_FileCrcFailure` (7).
    FileCrcFailure {
        /// CRC stored in the file.
        stored: u32,
        /// CRC computed over header + records.
        computed: u32,
    },
    /// `CS_RecordInvalid` (4).
    RecordInvalid {
        /// Zero-based record index that failed.
        record_number: u32,
        /// `Fw::SerializeStatus` discriminant.
        error: i32,
    },
    /// `CS_RecordMismatch` (12) — the ONLY load event that does not bump
    /// `CS_Errors`.
    RecordMismatch {
        /// `numRecords` from the header.
        header_records: u32,
        /// Bytes left over after the declared records.
        extra_bytes: u32,
    },
    /// `CS_TimeBaseMismatch` (13).
    TimeBaseMismatch {
        /// The live time base.
        current: TimeBase,
        /// The sequence file's time base.
        seq: TimeBase,
    },
    /// `CS_TimeContextMismatch` (14).
    TimeContextMismatch {
        /// The live time context.
        current: u8,
        /// The sequence file's time context.
        seq: u8,
    },
    /// `CS_NoRecords` (25).
    NoRecords,
}

/// A pluggable sequence file format (C++ `CmdSequencer::Sequence`).
///
/// Only [`FPrimeSequence`] ships; the trait exists so
/// [`CmdSequencer::set_sequence_format`] stays expressible, exactly like the
/// C++ `setSequenceFormat`.
pub trait Sequence: Send {
    /// Reserve the sequence buffer (C++ `allocateBuffer`, minus the
    /// `MemAllocator`). Panics via `fw_assert!` when `bytes` cannot hold a
    /// header.
    fn allocate_buffer(&mut self, identifier: FwEnumStoreType, bytes: usize);
    /// Release the sequence buffer (C++ `deallocateBuffer`).
    fn deallocate_buffer(&mut self);
    /// Capacity of the sequence buffer in bytes.
    fn capacity(&self) -> usize;
    /// C++ `setFileName`: records the command/event/telemetry spellings.
    fn set_file_name(&mut self, file_name: &CmdStringArg);
    /// The command-string spelling of the file name (`Fw::CmdStringArg`).
    fn file_name(&self) -> &CmdStringArg;
    /// The event spelling of the file name (`Fw::LogStringArg`).
    fn log_file_name(&self) -> &LogStringArg;
    /// The telemetry spelling of the file name (`Fw::String`).
    fn string_file_name(&self) -> &FwDefaultString;
    /// The (canonicalized) header of the loaded sequence.
    fn header(&self) -> &SequenceHeader;
    /// Read and fully validate a sequence file.
    ///
    /// `current_time` stands in for the C++ `component.getTime()` call made
    /// inside `Header::validateTime`; the component reads the time port once
    /// and hands the value in. At most one [`SequenceLoadEvent`] is
    /// produced.
    fn load_file(
        &mut self,
        file_name: &CmdStringArg,
        current_time: &Time,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool;
    /// C++ `hasMoreRecords()` — purely "bytes left in the buffer".
    fn has_more_records(&self) -> bool;
    /// C++ `nextRecord()` — `fw_assert!`s on a failed deserialize.
    fn next_record(&mut self, record: &mut SequenceRecord);
    /// C++ `reset()` — rewind the read cursor, keeping the data.
    fn reset(&mut self);
    /// C++ `clear()` — drop the data (read AND write cursors to 0).
    fn clear(&mut self);
}

// ---------------------------------------------------------------------------
// FPrimeSequence — the format `fprime-seqgen` emits.
// ---------------------------------------------------------------------------

/// The default binary sequence format (C++ `FPrimeSequence`).
///
/// File layout: `[header 11][records fileSize-4][CRC 4]`, all big-endian.
/// A command record is
/// `[descriptor u8][seconds u32][useconds u32][recordSize u32][recordSize
/// bytes]` where the payload is a complete command com packet
/// (`[u16 0x0000][opcode u32][args]`). An end-of-sequence record is the
/// single byte `0x02`.
pub struct FPrimeSequence {
    /// Owned replacement for the C++ `ExternalSerializeBuffer` over
    /// `MemAllocator` memory. Empty until `allocate_buffer`.
    storage: Box<[u8]>,
    /// Write cursor (`m_serLoc`): valid bytes in `storage`.
    ser_loc: usize,
    /// Read cursor (`m_deserLoc`).
    deser_loc: usize,
    /// Inert API parity with the C++ allocator identifier.
    allocator_id: FwEnumStoreType,
    header: SequenceHeader,
    crc: Hash,
    crc_stored: u32,
    file_name: CmdStringArg,
    log_file_name: LogStringArg,
    string_file_name: FwDefaultString,
}

impl Default for FPrimeSequence {
    fn default() -> Self {
        Self::new()
    }
}

impl FPrimeSequence {
    /// A sequence with no buffer yet (C++ constructor).
    #[must_use]
    pub fn new() -> Self {
        Self {
            storage: Box::new([]),
            ser_loc: 0,
            deser_loc: 0,
            allocator_id: 0,
            header: SequenceHeader::default(),
            crc: Hash::new(),
            crc_stored: 0,
            file_name: CmdStringArg::new(),
            log_file_name: LogStringArg::new(),
            string_file_name: FwDefaultString::new(),
        }
    }

    /// The allocator identifier passed to [`Sequence::allocate_buffer`].
    #[must_use]
    pub fn allocator_id(&self) -> FwEnumStoreType {
        self.allocator_id
    }

    /// The stored (file) CRC of the last load attempt.
    #[must_use]
    pub fn stored_crc(&self) -> u32 {
        self.crc_stored
    }

    /// C++ `setBuffLen`: write cursor = `len`, read cursor = 0.
    fn set_buff_len(&mut self, len: usize) {
        fw_assert!(
            len <= self.storage.len(),
            len as i32,
            self.storage.len() as i32
        );
        self.ser_loc = len;
        self.deser_loc = 0;
    }

    /// Run `f` over a serialization view of the buffer, committing the read
    /// cursor afterwards.
    fn with_deser<R>(&mut self, f: impl FnOnce(&mut ExtBuf<'_>) -> R) -> R {
        let ser_loc = self.ser_loc;
        let deser_loc = self.deser_loc;
        let mut buf = ExtBuf::with_len(&mut self.storage[..], ser_loc);
        buf.set_deser_loc(deser_loc);
        let result = f(&mut buf);
        let new_deser = buf.deser_loc();
        self.deser_loc = new_deser;
        result
    }

    /// C++ `getDeserializeSizeLeft()`.
    fn deserialize_size_left(&self) -> usize {
        self.ser_loc.saturating_sub(self.deser_loc)
    }

    /// C++ `readFile()`.
    fn read_file(&mut self, event: &mut Option<SequenceLoadEvent>) -> bool {
        // The name is copied out so the borrow does not conflict with the
        // `&mut self` read path below (C++ passes `m_fileName.toChar()`).
        let name = self.file_name.clone();
        let Some(path) = name.as_str() else {
            // A non-UTF-8 path cannot be opened through the Rust OSAL; C++
            // would hand the raw bytes to open(2) and fail there.
            *event = Some(SequenceLoadEvent::FileReadError);
            return false;
        };
        let mut file = fprime_os::File::new();
        let status = file.open(path, FileMode::OpenRead);
        let result = match status {
            FileStatus::OpOk => self.read_open_file(&mut file, event),
            FileStatus::DoesntExist => {
                *event = Some(SequenceLoadEvent::FileNotFound);
                false
            }
            _ => {
                *event = Some(SequenceLoadEvent::FileReadError);
                false
            }
        };
        file.close();
        result
    }

    /// C++ `readOpenFile()`. The CRC is taken over the 11 header bytes as
    /// read, then over the record block — bit-identical to the C++ two
    /// `update()` calls over the same (overwritten) address.
    fn read_open_file(
        &mut self,
        file: &mut fprime_os::File,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        self.crc = Hash::new();
        self.crc.init();
        if !self.read_header(file, event) {
            return false;
        }
        let header_bytes: [u8; SEQUENCE_HEADER_SIZE] = {
            let mut copy = [0u8; SEQUENCE_HEADER_SIZE];
            copy.copy_from_slice(&self.storage[..SEQUENCE_HEADER_SIZE]);
            copy
        };
        self.crc.update(&header_bytes);
        if !(self.deserialize_header(event)
            && self.read_records_and_crc(file, event)
            && self.extract_crc(event))
        {
            return false;
        }
        let buff_len = self.ser_loc;
        let mut crc = std::mem::take(&mut self.crc);
        crc.update(&self.storage[..buff_len]);
        self.crc = crc;
        true
    }

    /// C++ `readHeader()`.
    fn read_header(
        &mut self,
        file: &mut fprime_os::File,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        let capacity = self.storage.len();
        fw_assert!(
            capacity >= SEQUENCE_HEADER_SIZE,
            capacity as i32,
            SEQUENCE_HEADER_SIZE as i32
        );
        let mut read_len: FwSizeType = 0;
        let status = file.read(
            &mut self.storage[..SEQUENCE_HEADER_SIZE],
            &mut read_len,
            WaitType::Wait,
        );
        if status != FileStatus::OpOk {
            *event = Some(SequenceLoadEvent::FileInvalid {
                stage: FileReadStage::ReadHeader,
                error: status as i32,
            });
            return false;
        }
        if read_len != SEQUENCE_HEADER_SIZE as FwSizeType {
            *event = Some(SequenceLoadEvent::FileInvalid {
                stage: FileReadStage::ReadHeaderSize,
                error: read_len as i32,
            });
            return false;
        }
        self.set_buff_len(SEQUENCE_HEADER_SIZE);
        true
    }

    /// C++ `deserializeHeader()`.
    fn deserialize_header(&mut self, event: &mut Option<SequenceLoadEvent>) -> bool {
        let capacity = self.storage.len();
        let mut header = SequenceHeader::default();
        let outcome = self.with_deser(|buf| {
            let mut file_size = 0u32;
            let status = buf.deserialize_u32_be(&mut file_size);
            if !status.is_ok() {
                return Err(SequenceLoadEvent::FileInvalid {
                    stage: FileReadStage::DeserSize,
                    error: status as i32,
                });
            }
            if file_size as usize > capacity {
                return Err(SequenceLoadEvent::FileSizeError { size: file_size });
            }
            header.file_size = file_size;
            let status = buf.deserialize_u32_be(&mut header.num_records);
            if !status.is_ok() {
                return Err(SequenceLoadEvent::FileInvalid {
                    stage: FileReadStage::DeserNumRecords,
                    error: status as i32,
                });
            }
            let mut time_base = TimeBase::default();
            let status = buf.deserialize(&mut time_base, Endianness::Big);
            if !status.is_ok() {
                return Err(SequenceLoadEvent::FileInvalid {
                    stage: FileReadStage::DeserTimeBase,
                    error: status as i32,
                });
            }
            header.time_base = time_base;
            let status = buf.deserialize_u8_be(&mut header.time_context);
            if !status.is_ok() {
                return Err(SequenceLoadEvent::FileInvalid {
                    stage: FileReadStage::DeserTimeContext,
                    error: status as i32,
                });
            }
            Ok(())
        });
        match outcome {
            Ok(()) => {
                self.header = header;
                true
            }
            Err(load_event) => {
                // C++ writes the header members as it goes; the values are
                // never read after a failed load (the component clears the
                // sequence), so committing only on success is equivalent.
                *event = Some(load_event);
                false
            }
        }
    }

    /// C++ `readRecordsAndCRC()` — reads `fileSize` bytes OVER the header.
    fn read_records_and_crc(
        &mut self,
        file: &mut fprime_os::File,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        let size = self.header.file_size as usize;
        let mut read_len: FwSizeType = 0;
        let status = file.read(&mut self.storage[..size], &mut read_len, WaitType::Wait);
        if status != FileStatus::OpOk {
            *event = Some(SequenceLoadEvent::FileInvalid {
                stage: FileReadStage::ReadSeqData,
                error: status as i32,
            });
            return false;
        }
        if read_len != size as FwSizeType {
            *event = Some(SequenceLoadEvent::FileInvalid {
                stage: FileReadStage::ReadSeqDataSize,
                error: read_len as i32,
            });
            return false;
        }
        self.set_buff_len(size);
        true
    }

    /// C++ `extractCRC()` — pulls the trailing four bytes out and shortens
    /// the buffer to the record data.
    fn extract_crc(&mut self, event: &mut Option<SequenceLoadEvent>) -> bool {
        let buff_size = self.ser_loc;
        if buff_size < SEQUENCE_CRC_SIZE {
            *event = Some(SequenceLoadEvent::FileInvalid {
                stage: FileReadStage::ReadSeqCrc,
                error: buff_size as i32,
            });
            return false;
        }
        let data_size = buff_size - SEQUENCE_CRC_SIZE;
        let mut raw = [0u8; SEQUENCE_CRC_SIZE];
        raw.copy_from_slice(&self.storage[data_size..buff_size]);
        self.crc_stored = u32::from_be_bytes(raw);
        self.set_buff_len(data_size);
        true
    }

    /// C++ `validateCRC()`.
    fn validate_crc(&mut self, event: &mut Option<SequenceLoadEvent>) -> bool {
        let computed = self.crc.finalize();
        if self.crc_stored != computed {
            *event = Some(SequenceLoadEvent::FileCrcFailure {
                stored: self.crc_stored,
                computed,
            });
            return false;
        }
        true
    }

    /// C++ `Header::validateTime(component)` — validates AND canonicalizes.
    fn validate_time(
        &mut self,
        current_time: &Time,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        let valid_time_base = current_time.get_time_base();
        if self.header.time_base != valid_time_base && self.header.time_base != TimeBase::TbDontCare
        {
            *event = Some(SequenceLoadEvent::TimeBaseMismatch {
                current: valid_time_base,
                seq: self.header.time_base,
            });
            return false;
        }
        let valid_context = current_time.get_context();
        if self.header.time_context != valid_context
            && self.header.time_context != FW_CONTEXT_DONT_CARE
        {
            *event = Some(SequenceLoadEvent::TimeContextMismatch {
                current: valid_context,
                seq: self.header.time_context,
            });
            return false;
        }
        // Canonicalize: after a successful validate the header carries the
        // LIVE base/context, which is what makes don't-care files run.
        self.header.time_base = valid_time_base;
        self.header.time_context = valid_context;
        true
    }

    /// C++ `validateRecords()`.
    fn validate_records(&mut self, event: &mut Option<SequenceLoadEvent>) -> bool {
        let num_records = self.header.num_records;
        if num_records == 0 {
            *event = Some(SequenceLoadEvent::NoRecords);
            return false;
        }
        let mut record = SequenceRecord::default();
        for record_number in 0..num_records {
            let status = self.deserialize_record(&mut record);
            if status != SerializeStatus::Ok {
                *event = Some(SequenceLoadEvent::RecordInvalid {
                    record_number,
                    error: status as i32,
                });
                return false;
            }
        }
        let left = self.deserialize_size_left();
        if left > 0 {
            *event = Some(SequenceLoadEvent::RecordMismatch {
                header_records: num_records,
                extra_bytes: left as u32,
            });
            return false;
        }
        self.reset();
        true
    }

    /// C++ `deserializeRecord()`.
    fn deserialize_record(&mut self, record: &mut SequenceRecord) -> SerializeStatus {
        self.with_deser(|buf| {
            // Descriptor.
            let mut desc_entry = 0u8;
            let status = buf.deserialize_u8_be(&mut desc_entry);
            if !status.is_ok() {
                return status;
            }
            if desc_entry > RecordDescriptor::EndOfSequence as u8 {
                return SerializeStatus::DeserFormatError;
            }
            record.descriptor = match desc_entry {
                0 => RecordDescriptor::Absolute,
                1 => RecordDescriptor::Relative,
                _ => RecordDescriptor::EndOfSequence,
            };
            if record.descriptor == RecordDescriptor::EndOfSequence {
                // No further fields are consumed (C++ returns here).
                return SerializeStatus::Ok;
            }
            // Time tag: seconds + useconds only. The base and context come
            // from the header at step time.
            let mut seconds = 0u32;
            let mut useconds = 0u32;
            let status = buf.deserialize_u32_be(&mut seconds);
            if !status.is_ok() {
                return status;
            }
            let status = buf.deserialize_u32_be(&mut useconds);
            if !status.is_ok() {
                return status;
            }
            // C++ parity: `Fw::Time::set` FW_ASSERTs useconds < 1e6, so a
            // malformed tag is a fail-stop in both implementations.
            record.time_tag.set(seconds, useconds);
            // Record size.
            let mut record_size = 0u32;
            let status = buf.deserialize_u32_be(&mut record_size);
            if !status.is_ok() {
                return status;
            }
            if record_size as usize > buf.deserialize_size_left() {
                return SerializeStatus::DeserSizeMismatch;
            }
            // C++ parity: this check is over-strict by two bytes because
            // `record_size` already includes the packet descriptor. Ported
            // verbatim — usable record size tops out at 510.
            if record_size as usize + size_of::<FwPacketDescriptorType>() > FW_COM_BUFFER_MAX_SIZE {
                return SerializeStatus::DeserSizeMismatch;
            }
            // C++ `copyCommand`: a raw OMIT_LENGTH copy into the ComBuffer.
            let size = record_size as usize;
            let mut raw = [0u8; FW_COM_BUFFER_MAX_SIZE];
            let mut len = size;
            let status = buf.deserialize_bytes(
                &mut raw[..size],
                &mut len,
                LengthMode::OmitLength,
                Endianness::Big,
            );
            if !status.is_ok() {
                return status;
            }
            record.command.set_buff(&raw[..size])
        })
    }
}

impl Sequence for FPrimeSequence {
    fn allocate_buffer(&mut self, identifier: FwEnumStoreType, bytes: usize) {
        // C++ FW_ASSERT(bytes >= Header::SERIALIZED_SIZE).
        fw_assert!(bytes >= SEQUENCE_HEADER_SIZE, bytes as i32);
        self.allocator_id = identifier;
        self.storage = vec![0u8; bytes].into_boxed_slice();
        self.ser_loc = 0;
        self.deser_loc = 0;
    }

    fn deallocate_buffer(&mut self) {
        self.storage = Box::new([]);
        self.ser_loc = 0;
        self.deser_loc = 0;
    }

    fn capacity(&self) -> usize {
        self.storage.len()
    }

    fn set_file_name(&mut self, file_name: &CmdStringArg) {
        self.file_name = file_name.clone();
        self.log_file_name.set_bytes(file_name.as_bytes());
        self.string_file_name.set_bytes(file_name.as_bytes());
    }

    fn file_name(&self) -> &CmdStringArg {
        &self.file_name
    }

    fn log_file_name(&self) -> &LogStringArg {
        &self.log_file_name
    }

    fn string_file_name(&self) -> &FwDefaultString {
        &self.string_file_name
    }

    fn header(&self) -> &SequenceHeader {
        &self.header
    }

    fn load_file(
        &mut self,
        file_name: &CmdStringArg,
        current_time: &Time,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        // C++ FW_ASSERT(m_buffer.getBuffAddr() != nullptr).
        fw_assert!(!self.storage.is_empty());
        self.set_file_name(file_name);
        // Short-circuit chain, exactly as in C++.
        self.read_file(event)
            && self.validate_crc(event)
            && self.validate_time(current_time, event)
            && self.validate_records(event)
    }

    fn has_more_records(&self) -> bool {
        self.deserialize_size_left() > 0
    }

    fn next_record(&mut self, record: &mut SequenceRecord) {
        let status = self.deserialize_record(record);
        fw_assert!(status == SerializeStatus::Ok, status as i32);
    }

    fn reset(&mut self) {
        self.deser_loc = 0;
    }

    fn clear(&mut self) {
        self.ser_loc = 0;
        self.deser_loc = 0;
    }
}

// ---------------------------------------------------------------------------
// Timer (CmdSequencerImpl.hpp `class Timer`).
// ---------------------------------------------------------------------------

/// A one-shot expiration time. C++ `CmdSequencerComponentImpl::Timer`.
///
/// `is_expired_at` only rejects the `Gt` case, so an INCOMPARABLE
/// comparison (mismatched time base or context) counts as EXPIRED. Do NOT
/// rewrite it with `PartialOrd`, which answers `false` for incomparable
/// times.
#[derive(Debug, Clone, Copy, Default)]
pub struct Timer {
    armed: bool,
    expiration_time: Time,
}

impl Timer {
    /// A cleared timer.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            armed: false,
            expiration_time: Time::ZERO,
        }
    }

    /// Arm the timer for `time` (C++ `set`).
    pub fn set(&mut self, time: Time) {
        self.armed = true;
        self.expiration_time = time;
    }

    /// Disarm the timer (C++ `clear`).
    pub fn clear(&mut self) {
        self.armed = false;
    }

    /// True when armed and `expiration <= time` — or the two are
    /// INCOMPARABLE (C++ parity).
    #[must_use]
    pub fn is_expired_at(&self, time: &Time) -> bool {
        if !self.armed {
            return false;
        }
        Time::compare(&self.expiration_time, time) != TimeComparison::Gt
    }

    /// The armed expiration time (unspecified when disarmed).
    #[must_use]
    pub fn expiration_time(&self) -> Time {
        self.expiration_time
    }

    /// True when armed.
    #[must_use]
    pub fn is_armed(&self) -> bool {
        self.armed
    }
}

// ---------------------------------------------------------------------------
// Dictionary constants (CmdSequencer.fpp / Commands.fppi / Events.fppi /
// Telemetry.fppi).
// ---------------------------------------------------------------------------

component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel), in FPP port order.
    impl CmdSequencer {
        /// `seqCancelIn` async input port.
        MSG_TYPE_SEQ_CANCEL_IN,
        /// `cmdResponseIn` async input port.
        MSG_TYPE_CMD_RESPONSE_IN,
        /// `pingIn` async input port.
        MSG_TYPE_PING_IN,
        /// `seqRunIn` async input port.
        MSG_TYPE_SEQ_RUN_IN,
        /// `seqDispatchIn` async input port.
        MSG_TYPE_SEQ_DISPATCH_IN,
        /// `schedIn` async input port.
        MSG_TYPE_SCHED_IN,
        /// `cmdIn` async command port.
        MSG_TYPE_CMD_IN,
    }
}

/// Queue message size: the maximum over every async invocation.
///
/// The command envelope is the largest: 6 (msg type and port num), 4
/// (opcode), 4 (cmdSeq), 2 + 506 (length-prefixed `CmdArgBuffer`) = 522.
/// `seqRunIn` needs 6, 2 + 240 (file name), 8 + 255 (`SeqArgs`) = 511;
/// `seqDispatchIn` 248; `cmdResponseIn` 15; `schedIn` and `pingIn` 10;
/// `seqCancelIn` 6.
pub const QUEUE_MSG_SIZE: usize = 522;

/// FPP declares no `priority` qualifiers, so every async input shares one
/// queue priority and dispatch is pure FIFO.
const QUEUE_PRIORITY: FwQueuePriorityType = 1;

/// Event `fileName`/`filename` arguments are `string size 60` — shorter
/// than the 240-character command/telemetry/port spelling.
const EVENT_STRING_SIZE: usize = 60;

/// C++ `CmdSequencerComponentImpl::NO_SEQ`.
pub const NO_SEQ: &str = "<no seq>";

impl CmdSequencer {
    /// `CS_RUN(fileName: string size 240, block: Svc.BlockState)`.
    pub const OPCODE_CS_RUN: FwOpcodeType = 0;
    /// `CS_VALIDATE(fileName: string size 240)`.
    pub const OPCODE_CS_VALIDATE: FwOpcodeType = 1;
    /// `CS_CANCEL()`.
    pub const OPCODE_CS_CANCEL: FwOpcodeType = 2;
    /// `CS_START()`.
    pub const OPCODE_CS_START: FwOpcodeType = 3;
    /// `CS_STEP()` — MANUAL step mode only.
    pub const OPCODE_CS_STEP: FwOpcodeType = 4;
    /// `CS_AUTO()`.
    pub const OPCODE_CS_AUTO: FwOpcodeType = 5;
    /// `CS_MANUAL()`.
    pub const OPCODE_CS_MANUAL: FwOpcodeType = 6;
    /// `CS_JOIN_WAIT()`.
    pub const OPCODE_CS_JOIN_WAIT: FwOpcodeType = 7;

    /// `CS_SequenceLoaded(fileName)` — ACTIVITY_LO.
    pub const EVENTID_CS_SEQUENCE_LOADED: FwEventIdType = 0;
    /// `CS_SequenceCanceled(fileName)` — ACTIVITY_HI.
    pub const EVENTID_CS_SEQUENCE_CANCELED: FwEventIdType = 1;
    /// `CS_FileReadError(fileName)` — WARNING_HI.
    pub const EVENTID_CS_FILE_READ_ERROR: FwEventIdType = 2;
    /// `CS_FileInvalid(fileName, stage, error)` — WARNING_HI.
    pub const EVENTID_CS_FILE_INVALID: FwEventIdType = 3;
    /// `CS_RecordInvalid(fileName, recordNumber, error)` — WARNING_HI.
    pub const EVENTID_CS_RECORD_INVALID: FwEventIdType = 4;
    /// `CS_FileSizeError(fileName, size)` — WARNING_HI.
    pub const EVENTID_CS_FILE_SIZE_ERROR: FwEventIdType = 5;
    /// `CS_FileNotFound(fileName)` — WARNING_HI.
    pub const EVENTID_CS_FILE_NOT_FOUND: FwEventIdType = 6;
    /// `CS_FileCrcFailure(fileName, storedCRC, computedCRC)` — WARNING_HI.
    pub const EVENTID_CS_FILE_CRC_FAILURE: FwEventIdType = 7;
    /// `CS_CommandComplete(fileName, recordNumber, opCode)` — ACTIVITY_LO.
    pub const EVENTID_CS_COMMAND_COMPLETE: FwEventIdType = 8;
    /// `CS_SequenceComplete(fileName)` — ACTIVITY_HI.
    pub const EVENTID_CS_SEQUENCE_COMPLETE: FwEventIdType = 9;
    /// `CS_CommandError(fileName, recordNumber, opCode, errorStatus)` —
    /// WARNING_HI.
    pub const EVENTID_CS_COMMAND_ERROR: FwEventIdType = 10;
    /// `CS_InvalidMode()` — WARNING_HI.
    pub const EVENTID_CS_INVALID_MODE: FwEventIdType = 11;
    /// `CS_RecordMismatch(fileName, header_records, extra_bytes)` —
    /// WARNING_HI.
    pub const EVENTID_CS_RECORD_MISMATCH: FwEventIdType = 12;
    /// `CS_TimeBaseMismatch(fileName, time_base, seq_time_base)` —
    /// WARNING_HI.
    pub const EVENTID_CS_TIME_BASE_MISMATCH: FwEventIdType = 13;
    /// `CS_TimeContextMismatch(fileName, currTimeBase, seqTimeBase)` —
    /// WARNING_HI.
    pub const EVENTID_CS_TIME_CONTEXT_MISMATCH: FwEventIdType = 14;
    /// `CS_PortSequenceStarted(filename)` — ACTIVITY_HI.
    pub const EVENTID_CS_PORT_SEQUENCE_STARTED: FwEventIdType = 15;
    /// `CS_UnexpectedCompletion(opcode)` — WARNING_HI.
    pub const EVENTID_CS_UNEXPECTED_COMPLETION: FwEventIdType = 16;
    /// `CS_ModeSwitched(mode)` — ACTIVITY_HI.
    pub const EVENTID_CS_MODE_SWITCHED: FwEventIdType = 17;
    /// `CS_NoSequenceActive()` — WARNING_LO.
    pub const EVENTID_CS_NO_SEQUENCE_ACTIVE: FwEventIdType = 18;
    /// `CS_SequenceValid(filename)` — ACTIVITY_HI.
    pub const EVENTID_CS_SEQUENCE_VALID: FwEventIdType = 19;
    /// `CS_SequenceTimeout(filename, command)` — WARNING_HI.
    pub const EVENTID_CS_SEQUENCE_TIMEOUT: FwEventIdType = 20;
    /// `CS_CmdStepped(filename, command)` — ACTIVITY_HI.
    pub const EVENTID_CS_CMD_STEPPED: FwEventIdType = 21;
    /// `CS_CmdStarted(filename)` — ACTIVITY_HI.
    pub const EVENTID_CS_CMD_STARTED: FwEventIdType = 22;
    /// `CS_JoinWaiting(filename, recordNumber, opCode)` — ACTIVITY_HI.
    pub const EVENTID_CS_JOIN_WAITING: FwEventIdType = 23;
    /// `CS_JoinWaitingNotComplete()` — WARNING_HI.
    pub const EVENTID_CS_JOIN_WAITING_NOT_COMPLETE: FwEventIdType = 24;
    /// `CS_NoRecords(fileName)` — WARNING_LO.
    pub const EVENTID_CS_NO_RECORDS: FwEventIdType = 25;

    /// `CS_LoadCommands: U32`.
    pub const CHANID_CS_LOAD_COMMANDS: FwChanIdType = 0;
    /// `CS_CancelCommands: U32`.
    pub const CHANID_CS_CANCEL_COMMANDS: FwChanIdType = 1;
    /// `CS_Errors: U32`.
    pub const CHANID_CS_ERRORS: FwChanIdType = 2;
    /// `CS_CommandsExecuted: U32`.
    pub const CHANID_CS_COMMANDS_EXECUTED: FwChanIdType = 3;
    /// `CS_SequencesCompleted: U32`.
    pub const CHANID_CS_SEQUENCES_COMPLETED: FwChanIdType = 4;
    /// `CS_CurrentSequence: string size 240` — **update on change**.
    pub const CHANID_CS_CURRENT_SEQUENCE: FwChanIdType = 5;
}

/// `CmdDispatcherCfg::getEventOpcode`.
const fn get_event_opcode(opcode: FwOpcodeType) -> FwOpcodeType {
    if INCLUDE_COMMAND_OPCODES_IN_EVENTS {
        opcode
    } else {
        FwOpcodeType::MAX
    }
}

// ---------------------------------------------------------------------------
// Component state.
// ---------------------------------------------------------------------------

/// Mutable component state (the C++ `m_*` members). Every input port is
/// ASYNC, so this is only ever touched on the component thread; the mutex is
/// the workspace convention, not a C++ guarded-port mutex.
struct CmdSequencerState {
    /// The sequence format in use (C++ `m_sequence`, `&m_FPrimeSequence`).
    sequence: Box<dyn Sequence>,
    load_cmd_count: u32,
    cancel_cmd_count: u32,
    error_count: u32,
    run_mode: RunMode,
    step_mode: StepMode,
    executed_count: u32,
    total_executed_count: u32,
    sequences_completed_count: u32,
    /// Command-response watchdog in SECONDS; 0 disables it.
    timeout: u32,
    block_state: BlockState,
    op_code: FwOpcodeType,
    cmd_seq: u32,
    join_waiting: bool,
    /// The record currently being executed (C++ `m_record`).
    record: SequenceRecord,
    /// Pending future-time command dispatch (C++ `m_cmdTimer`).
    cmd_timer: Timer,
    /// Command-response watchdog (C++ `m_cmdTimeoutTimer`).
    cmd_timeout_timer: Timer,
    /// `update on change` cache for `CS_CurrentSequence`.
    last_current_sequence: Option<FwDefaultString>,
}

impl CmdSequencerState {
    fn new() -> Self {
        Self {
            sequence: Box::new(FPrimeSequence::new()),
            load_cmd_count: 0,
            cancel_cmd_count: 0,
            error_count: 0,
            run_mode: RunMode::Stopped,
            step_mode: StepMode::Auto,
            executed_count: 0,
            total_executed_count: 0,
            sequences_completed_count: 0,
            timeout: 0,
            block_state: BlockState::NoBlock,
            op_code: 0,
            cmd_seq: 0,
            join_waiting: false,
            record: SequenceRecord::default(),
            cmd_timer: Timer::new(),
            cmd_timeout_timer: Timer::new(),
            last_current_sequence: None,
        }
    }
}

// ---------------------------------------------------------------------------
// The component.
// ---------------------------------------------------------------------------

/// `Svc::CmdSequencer` — active binary command sequencer.
pub struct CmdSequencer {
    /// Active core: `PassiveBase` + queue + task.
    pub active: ActiveBase,
    /// Own command registration/response glue (`cmdRegOut`/`cmdResponseOut`).
    pub cmd: CmdGlue,
    /// Event ports (binary + text) and the time port (`timeCaller`).
    pub evt: EventGlue,
    /// Telemetry port (`tlmOut`).
    pub tlm: TlmGlue,
    /// `comCmdOut`: `Fw.Com` out — the sequenced command packets.
    pub com_cmd_out: OutputPort<dyn ComPort>,
    /// `seqDone`: `Fw.CmdResponse` out — sequence completion notification.
    pub seq_done: OutputPort<dyn CmdResponsePort>,
    /// `seqStartOut`: `Svc.CmdSeqIn` out — sequence start notification.
    pub seq_start_out: OutputPort<dyn CmdSeqInPort>,
    /// `pingOut`: `Svc.Ping` out.
    pub ping_out: OutputPort<dyn PingPort>,
    state: Mutex<CmdSequencerState>,
}

impl CmdSequencer {
    /// Construct (topology phase 1). Follow with `set_id_base`, wiring,
    /// [`Self::init`], [`Self::allocate_buffer`], [`Self::set_timeout`],
    /// [`Self::reg_commands`] and `active.start`.
    #[must_use]
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            active: ActiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            com_cmd_out: OutputPort::new(),
            seq_done: OutputPort::new(),
            seq_start_out: OutputPort::new(),
            ping_out: OutputPort::new(),
            state: Mutex::new(CmdSequencerState::new()),
        })
    }

    /// Create the message queue (C++ `init(queueDepth)`).
    pub fn init(&self, queue_depth: FwSizeType) {
        self.active
            .queued
            .create_queue(queue_depth, QUEUE_MSG_SIZE as FwSizeType);
    }

    fn id_base(&self) -> FwIdType {
        self.active.queued.base.get_id_base()
    }

    /// C++ `regCommands()`: register all eight opcodes.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.id_base(),
            &[
                Self::OPCODE_CS_RUN,
                Self::OPCODE_CS_VALIDATE,
                Self::OPCODE_CS_CANCEL,
                Self::OPCODE_CS_START,
                Self::OPCODE_CS_STEP,
                Self::OPCODE_CS_AUTO,
                Self::OPCODE_CS_MANUAL,
                Self::OPCODE_CS_JOIN_WAIT,
            ],
        );
    }

    // -- Public setup API (topology, before the task is started) -----------

    /// C++ `setSequenceFormat(Sequence&)`.
    pub fn set_sequence_format(&self, sequence: Box<dyn Sequence>) {
        self.state.lock().unwrap().sequence = sequence;
    }

    /// C++ `allocateBuffer(identifier, allocator, bytes)`; the Ref topology
    /// uses `5 * 1024`. `fw_assert!`s `bytes >= 11`.
    pub fn allocate_buffer(&self, identifier: FwEnumStoreType, bytes: usize) {
        self.state
            .lock()
            .unwrap()
            .sequence
            .allocate_buffer(identifier, bytes);
    }

    /// C++ `deallocateBuffer(allocator)`.
    pub fn deallocate_buffer(&self) {
        self.state.lock().unwrap().sequence.deallocate_buffer();
    }

    /// C++ `setTimeout(U32)` — command-response watchdog in SECONDS,
    /// 0 (the default) disables it.
    pub fn set_timeout(&self, timeout: u32) {
        self.state.lock().unwrap().timeout = timeout;
    }

    /// C++ `loadSequence(fileName)` — load at topology setup time. Requires
    /// the event ports to be wired and `fw_assert!`s that no sequence is
    /// running; a failed load clears the sequence.
    pub fn load_sequence(&self, file_name: &CmdStringArg) {
        let mut guard = self.state.lock().unwrap();
        let st = &mut *guard;
        fw_assert!(st.run_mode == RunMode::Stopped, st.run_mode as i32);
        if !self.load_file(st, file_name) {
            st.sequence.clear();
        }
    }

    /// Current run mode (test/introspection helper).
    #[must_use]
    pub fn run_mode(&self) -> RunMode {
        self.state.lock().unwrap().run_mode
    }

    /// Current step mode (test/introspection helper).
    #[must_use]
    pub fn step_mode(&self) -> StepMode {
        self.state.lock().unwrap().step_mode
    }

    // -- Input-port factories are generated below by the codegen macros ----
}

// ---------------------------------------------------------------------------
// Event / telemetry helpers (the autocoded `log_*` / `tlmWrite_*`).
// ---------------------------------------------------------------------------

impl CmdSequencer {
    fn log_name_only(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        name: &LogStringArg,
        text: &str,
    ) {
        self.evt.log_event(
            self.id_base(),
            id,
            severity,
            &format!("{text} {name}"),
            |buf: &mut LogBuffer| {
                name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
            },
        );
    }

    fn log_no_args(&self, id: FwEventIdType, severity: LogSeverity, text: &str) {
        self.evt.log_event(
            self.id_base(),
            id,
            severity,
            text,
            |_buf: &mut LogBuffer| SerializeStatus::Ok,
        );
    }

    fn log_sequence_loaded(&self, name: &LogStringArg) {
        self.log_name_only(
            Self::EVENTID_CS_SEQUENCE_LOADED,
            LogSeverity::ActivityLo,
            name,
            "Loaded sequence",
        );
    }

    fn log_sequence_canceled(&self, name: &LogStringArg) {
        self.log_name_only(
            Self::EVENTID_CS_SEQUENCE_CANCELED,
            LogSeverity::ActivityHi,
            name,
            "Sequence file canceled:",
        );
    }

    fn log_file_read_error(&self, name: &LogStringArg) {
        self.log_name_only(
            Self::EVENTID_CS_FILE_READ_ERROR,
            LogSeverity::WarningHi,
            name,
            "Error reading sequence file",
        );
    }

    fn log_file_not_found(&self, name: &LogStringArg) {
        self.log_name_only(
            Self::EVENTID_CS_FILE_NOT_FOUND,
            LogSeverity::WarningHi,
            name,
            "Sequence file not found:",
        );
    }

    fn log_sequence_complete(&self, name: &LogStringArg) {
        self.log_name_only(
            Self::EVENTID_CS_SEQUENCE_COMPLETE,
            LogSeverity::ActivityHi,
            name,
            "Sequence file complete:",
        );
    }

    fn log_port_sequence_started(&self, name: &LogStringArg) {
        self.log_name_only(
            Self::EVENTID_CS_PORT_SEQUENCE_STARTED,
            LogSeverity::ActivityHi,
            name,
            "Local request for sequence started:",
        );
    }

    fn log_sequence_valid(&self, name: &LogStringArg) {
        self.log_name_only(
            Self::EVENTID_CS_SEQUENCE_VALID,
            LogSeverity::ActivityHi,
            name,
            "Sequence is valid:",
        );
    }

    fn log_cmd_started(&self, name: &LogStringArg) {
        self.log_name_only(
            Self::EVENTID_CS_CMD_STARTED,
            LogSeverity::ActivityHi,
            name,
            "Sequence started:",
        );
    }

    fn log_no_records(&self, name: &LogStringArg) {
        self.log_name_only(
            Self::EVENTID_CS_NO_RECORDS,
            LogSeverity::WarningLo,
            name,
            "Sequence file has no records. Ignoring:",
        );
    }

    fn log_invalid_mode(&self) {
        self.log_no_args(
            Self::EVENTID_CS_INVALID_MODE,
            LogSeverity::WarningHi,
            "Invalid mode",
        );
    }

    fn log_no_sequence_active(&self) {
        self.log_no_args(
            Self::EVENTID_CS_NO_SEQUENCE_ACTIVE,
            LogSeverity::WarningLo,
            "No sequence active.",
        );
    }

    fn log_join_waiting_not_complete(&self) {
        self.log_no_args(
            Self::EVENTID_CS_JOIN_WAITING_NOT_COMPLETE,
            LogSeverity::WarningHi,
            "Still waiting for sequence file to complete",
        );
    }

    fn log_file_invalid(&self, name: &LogStringArg, stage: FileReadStage, error: i32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_FILE_INVALID,
            LogSeverity::WarningHi,
            &format!("Sequence file {name} invalid. Stage: {stage:?} Error: {error}"),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u8_be(stage.as_repr()));
                buf.serialize_i32_be(error)
            },
        );
    }

    fn log_record_invalid(&self, name: &LogStringArg, record_number: u32, error: i32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_RECORD_INVALID,
            LogSeverity::WarningHi,
            &format!("Sequence file {name}: Record {record_number} invalid. Err: {error}"),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(record_number));
                buf.serialize_i32_be(error)
            },
        );
    }

    fn log_file_size_error(&self, name: &LogStringArg, size: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_FILE_SIZE_ERROR,
            LogSeverity::WarningHi,
            &format!("Sequence file {name} too large. Size: {size}"),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u32_be(size)
            },
        );
    }

    fn log_file_crc_failure(&self, name: &LogStringArg, stored: u32, computed: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_FILE_CRC_FAILURE,
            LogSeverity::WarningHi,
            &format!(
                "Sequence file {name} had invalid CRC. Stored 0x{stored:x}, Computed 0x{computed:x}."
            ),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(stored));
                buf.serialize_u32_be(computed)
            },
        );
    }

    fn log_record_mismatch(&self, name: &LogStringArg, header_records: u32, extra_bytes: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_RECORD_MISMATCH,
            LogSeverity::WarningHi,
            &format!(
                "Sequence file {name} header records mismatch: {header_records} in header, found {extra_bytes} extra bytes."
            ),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(header_records));
                buf.serialize_u32_be(extra_bytes)
            },
        );
    }

    fn log_time_base_mismatch(&self, name: &LogStringArg, current: TimeBase, seq: TimeBase) {
        let current_raw = current as u16;
        let seq_raw = seq as u16;
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_TIME_BASE_MISMATCH,
            LogSeverity::WarningHi,
            &format!(
                "Sequence file {name}: Current time base doesn't match sequence time: base: {current_raw} seq: {seq_raw}"
            ),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u16_be(current_raw));
                buf.serialize_u16_be(seq_raw)
            },
        );
    }

    fn log_time_context_mismatch(&self, name: &LogStringArg, current: u8, seq: u8) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_TIME_CONTEXT_MISMATCH,
            LogSeverity::WarningHi,
            &format!(
                "Sequence file {name}: Current time context doesn't match sequence context: base: {current} seq: {seq}"
            ),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u8_be(current));
                buf.serialize_u8_be(seq)
            },
        );
    }

    fn log_command_complete(&self, name: &LogStringArg, record_number: u32, op_code: FwOpcodeType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_COMMAND_COMPLETE,
            LogSeverity::ActivityLo,
            &format!("Sequence file {name}: Command {record_number} (opcode {op_code}) complete"),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(record_number));
                buf.serialize_u32_be(op_code)
            },
        );
    }

    fn log_command_error(
        &self,
        name: &LogStringArg,
        record_number: u32,
        op_code: FwOpcodeType,
        error_status: u32,
    ) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_COMMAND_ERROR,
            LogSeverity::WarningHi,
            &format!(
                "Sequence file {name}: Command {record_number} (opcode {op_code}) completed with error {error_status}"
            ),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(record_number));
                fw_try!(buf.serialize_u32_be(op_code));
                buf.serialize_u32_be(error_status)
            },
        );
    }

    fn log_unexpected_completion(&self, op_code: FwOpcodeType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_UNEXPECTED_COMPLETION,
            LogSeverity::WarningHi,
            &format!(
                "Command complete status received while no sequences active. Opcode: {op_code}"
            ),
            |buf: &mut LogBuffer| buf.serialize_u32_be(op_code),
        );
    }

    fn log_mode_switched(&self, mode: SeqMode) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_MODE_SWITCHED,
            LogSeverity::ActivityHi,
            &format!("Sequencer switched to {mode:?} step mode"),
            |buf: &mut LogBuffer| buf.serialize_u8_be(mode.as_repr()),
        );
    }

    fn log_sequence_timeout(&self, name: &LogStringArg, command: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_SEQUENCE_TIMEOUT,
            LogSeverity::WarningHi,
            &format!("Sequence {name} timed out on command {command}"),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u32_be(command)
            },
        );
    }

    fn log_cmd_stepped(&self, name: &LogStringArg, command: u32) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_CMD_STEPPED,
            LogSeverity::ActivityHi,
            &format!("Sequence {name} command {command} stepped"),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_u32_be(command)
            },
        );
    }

    fn log_join_waiting(&self, name: &LogStringArg, record_number: u32, op_code: FwOpcodeType) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CS_JOIN_WAITING,
            LogSeverity::ActivityHi,
            &format!(
                "Start waiting for sequence file {name}: Command {record_number} (opcode {op_code}) to complete"
            ),
            |buf: &mut LogBuffer| {
                fw_try!(name.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                fw_try!(buf.serialize_u32_be(record_number));
                buf.serialize_u32_be(op_code)
            },
        );
    }

    /// `tlmWrite_*` for the five U32 counters.
    fn tlm_u32(&self, chan: FwChanIdType, value: u32) {
        self.tlm
            .tlm_write(self.id_base(), chan, &value, self.evt.time_get());
    }

    /// `tlmWrite_CS_CurrentSequence` — **update on change**, truncated to
    /// `FileNameStringSize` (240) like the autocoded write.
    fn tlm_current_sequence(&self, st: &mut CmdSequencerState, value: &FwDefaultString) {
        if st.last_current_sequence.as_ref() == Some(value) {
            return;
        }
        st.last_current_sequence = Some(value.clone());
        let mut truncated = FileNameString::new();
        truncated.set_bytes(value.as_bytes());
        self.tlm.tlm_write(
            self.id_base(),
            Self::CHANID_CS_CURRENT_SEQUENCE,
            &truncated,
            self.evt.time_get(),
        );
    }
}

// ---------------------------------------------------------------------------
// Core sequencing logic (the C++ private helper methods).
// ---------------------------------------------------------------------------

impl CmdSequencer {
    /// C++ `error()`: bump the counter and emit `CS_Errors`.
    fn error(&self, st: &mut CmdSequencerState) {
        st.error_count = st.error_count.wrapping_add(1);
        self.tlm_u32(Self::CHANID_CS_ERRORS, st.error_count);
    }

    /// C++ `requireRunMode(mode)`: logs `CS_InvalidMode` on mismatch.
    fn require_run_mode(&self, st: &CmdSequencerState, mode: RunMode) -> bool {
        if st.run_mode == mode {
            true
        } else {
            self.log_invalid_mode();
            false
        }
    }

    /// Emit the single event a load attempt produced. C++ `Sequence::Events`
    /// bumps `CS_Errors` for every variant EXCEPT `recordMismatch` (an
    /// explicit TODO in `Events.cpp`).
    fn emit_sequence_event(&self, st: &mut CmdSequencerState, event: SequenceLoadEvent) {
        let name = st.sequence.log_file_name().clone();
        match event {
            SequenceLoadEvent::FileNotFound => self.log_file_not_found(&name),
            SequenceLoadEvent::FileReadError => self.log_file_read_error(&name),
            SequenceLoadEvent::FileInvalid { stage, error } => {
                self.log_file_invalid(&name, stage, error);
            }
            SequenceLoadEvent::FileSizeError { size } => self.log_file_size_error(&name, size),
            SequenceLoadEvent::FileCrcFailure { stored, computed } => {
                self.log_file_crc_failure(&name, stored, computed);
            }
            SequenceLoadEvent::RecordInvalid {
                record_number,
                error,
            } => self.log_record_invalid(&name, record_number, error),
            SequenceLoadEvent::RecordMismatch {
                header_records,
                extra_bytes,
            } => {
                self.log_record_mismatch(&name, header_records, extra_bytes);
                // C++ parity: recordMismatch does NOT call error().
                return;
            }
            SequenceLoadEvent::TimeBaseMismatch { current, seq } => {
                self.log_time_base_mismatch(&name, current, seq);
            }
            SequenceLoadEvent::TimeContextMismatch { current, seq } => {
                self.log_time_context_mismatch(&name, current, seq);
            }
            SequenceLoadEvent::NoRecords => self.log_no_records(&name),
        }
        self.error(st);
    }

    /// C++ component-level `loadFile(fileName)`.
    fn load_file(&self, st: &mut CmdSequencerState, file_name: &CmdStringArg) -> bool {
        // C++ reads the time inside `Header::validateTime`; the component
        // samples the time port once here and hands the value in. Equivalent
        // — the port is synchronous and nothing else runs in between.
        let current_time = self.evt.time_get();
        let mut event = None;
        let status = st.sequence.load_file(file_name, &current_time, &mut event);
        if let Some(event) = event {
            self.emit_sequence_event(st, event);
        }
        if status {
            let name = st.sequence.log_file_name().clone();
            self.log_sequence_loaded(&name);
            st.load_cmd_count = st.load_cmd_count.wrapping_add(1);
            self.tlm_u32(Self::CHANID_CS_LOAD_COMMANDS, st.load_cmd_count);
        } else {
            // Deliberate fix ported from C++: a partial load must not leave
            // `has_more_records()` true, or a later CS_START would drive
            // `next_record` into its assert.
            st.sequence.clear();
        }
        status
    }

    /// `seqStartOut_out(0, stringFileName, SeqArgs{0, 0})` when connected.
    fn emit_seq_start(&self, st: &CmdSequencerState) {
        if let Some(p) = self.seq_start_out.try_get() {
            let mut filename = FileNameString::new();
            filename.set_bytes(st.sequence.string_file_name().as_bytes());
            // C++ parity: always an empty placeholder argument value.
            let args = SeqArgs::default();
            p.target.invoke(p.port_num, &filename, &args);
        }
    }

    /// `seqDone_out(0, 0, 0, response)` when connected (cancel/complete).
    fn seq_done_guarded(&self, response: CmdResponse) {
        if let Some(p) = self.seq_done.try_get() {
            p.target.invoke(p.port_num, 0, 0, response);
        }
    }

    /// `seqDone_out(0, 0, 0, response)` WITHOUT an is-connected guard.
    /// C++ parity: `doSequenceRun`'s error paths call the port
    /// unconditionally (an unconnected `OutputPort` `fw_assert!`s here).
    fn seq_done_unguarded(&self, response: CmdResponse) {
        let p = self.seq_done.get();
        p.target.invoke(p.port_num, 0, 0, response);
    }

    /// C++ `setCmdTimeout(currentTime)` — armed only when a timeout is
    /// configured AND the step mode is AUTO.
    fn set_cmd_timeout(&self, st: &mut CmdSequencerState, current_time: &Time) {
        if st.timeout > 0 && st.step_mode == StepMode::Auto {
            let mut expiration = *current_time;
            expiration.add_duration(st.timeout, 0);
            st.cmd_timeout_timer.set(expiration);
        }
    }

    /// C++ `performCmd_Step()`.
    fn perform_cmd_step(&self, st: &mut CmdSequencerState) {
        st.sequence.next_record(&mut st.record);
        // The record time tag carries only seconds/useconds; the base and
        // context come from the canonicalized header.
        let header = *st.sequence.header();
        st.record.time_tag.set_time_base(header.time_base);
        st.record.time_tag.set_time_context(header.time_context);

        let current_time = self.evt.time_get();
        match st.record.descriptor {
            RecordDescriptor::EndOfSequence => {
                st.run_mode = RunMode::Stopped;
                self.sequence_complete(st);
            }
            RecordDescriptor::Relative => {
                st.record
                    .time_tag
                    .add_duration(current_time.get_seconds(), current_time.get_useconds());
                self.perform_cmd_step_absolute(st, &current_time);
            }
            RecordDescriptor::Absolute => self.perform_cmd_step_absolute(st, &current_time),
        }
    }

    /// C++ `performCmd_Step_ABSOLUTE(currentTime)`.
    fn perform_cmd_step_absolute(&self, st: &mut CmdSequencerState, current_time: &Time) {
        // C++ `currentTime >= m_record.m_timeTag`: GT or EQ only, so
        // INCOMPARABLE defers (unlike Timer::isExpiredAt).
        let comparison = Time::compare(current_time, &st.record.time_tag);
        if comparison == TimeComparison::Gt || comparison == TimeComparison::Eq {
            let p = self.com_cmd_out.get();
            p.target.invoke(p.port_num, &mut st.record.command, 0);
            self.set_cmd_timeout(st, current_time);
        } else {
            let tag = st.record.time_tag;
            st.cmd_timer.set(tag);
        }
    }

    /// C++ `performCmd_Cancel()` — note `reset()` (rewind), not `clear()`.
    fn perform_cmd_cancel(&self, st: &mut CmdSequencerState) {
        st.sequence.reset();
        st.run_mode = RunMode::Stopped;
        st.cmd_timer.clear();
        st.cmd_timeout_timer.clear();
        st.executed_count = 0;
        self.seq_done_guarded(CmdResponse::ExecutionError);
        if st.block_state == BlockState::Block || st.join_waiting {
            st.join_waiting = false;
            self.cmd
                .cmd_response(st.op_code, st.cmd_seq, CmdResponse::ExecutionError);
        }
        st.block_state = BlockState::NoBlock;
    }

    /// C++ `sequenceComplete()`.
    fn sequence_complete(&self, st: &mut CmdSequencerState) {
        st.sequences_completed_count = st.sequences_completed_count.wrapping_add(1);
        st.sequence.clear();
        let name = st.sequence.log_file_name().clone();
        self.log_sequence_complete(&name);
        self.tlm_u32(
            Self::CHANID_CS_SEQUENCES_COMPLETED,
            st.sequences_completed_count,
        );
        st.executed_count = 0;
        self.seq_done_guarded(CmdResponse::Ok);
        if st.block_state == BlockState::Block || st.join_waiting {
            self.cmd
                .cmd_response(st.op_code, st.cmd_seq, CmdResponse::Ok);
        }
        st.join_waiting = false;
        st.block_state = BlockState::NoBlock;
        let mut no_seq = FwDefaultString::new();
        no_seq.set(NO_SEQ);
        self.tlm_current_sequence(st, &no_seq);
    }

    /// C++ `commandComplete(opcode)`.
    fn command_complete(&self, st: &mut CmdSequencerState, op_code: FwOpcodeType) {
        let name = st.sequence.log_file_name().clone();
        self.log_command_complete(&name, st.executed_count, get_event_opcode(op_code));
        st.executed_count = st.executed_count.wrapping_add(1);
        st.total_executed_count = st.total_executed_count.wrapping_add(1);
        self.tlm_u32(Self::CHANID_CS_COMMANDS_EXECUTED, st.total_executed_count);
    }

    /// C++ `commandError(number, opCode, error)`.
    fn command_error(
        &self,
        st: &mut CmdSequencerState,
        number: u32,
        op_code: FwOpcodeType,
        error: u32,
    ) {
        let name = st.sequence.log_file_name().clone();
        self.log_command_error(&name, number, get_event_opcode(op_code), error);
        self.error(st);
    }

    /// Start the current sequence in AUTO mode: the shared tail of
    /// `CS_RUN_cmdHandler` and `doSequenceRun`.
    fn start_auto(&self, st: &mut CmdSequencerState) {
        st.run_mode = RunMode::Running;
        let name = st.sequence.string_file_name().clone();
        self.tlm_current_sequence(st, &name);
        self.emit_seq_start(st);
        self.perform_cmd_step(st);
    }
}

// ---------------------------------------------------------------------------
// Command handlers (all async — component thread).
// ---------------------------------------------------------------------------

impl CmdSequencer {
    /// `CS_RUN(fileName, block)`.
    ///
    /// Deviation from C++ (documented): the trailing `if (NO_BLOCK ==
    /// m_blockState) cmdResponse(OK)` is evaluated against the block state
    /// the command ARRIVED with, not the member variable. In C++ a sequence
    /// that completes synchronously inside `performCmd_Step` (a file whose
    /// first record is END_OF_SEQUENCE) resets `m_blockState` to `NO_BLOCK`
    /// after `sequenceComplete` already answered the BLOCK caller, so the
    /// handler answers a SECOND time. The exactly-once discipline of this
    /// port forbids that; every other path is byte-identical.
    fn cs_run_handler(
        &self,
        st: &mut CmdSequencerState,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        file_name: &CmdStringArg,
        block: BlockState,
    ) {
        if !self.require_run_mode(st, RunMode::Stopped) {
            if st.join_waiting {
                self.log_join_waiting_not_complete();
            }
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        if block == BlockState::Block && st.step_mode == StepMode::Manual {
            // Nothing executes until CS_STEP, so a BLOCK response could
            // never be sent.
            self.log_invalid_mode();
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }

        st.block_state = block;
        st.cmd_seq = cmd_seq;
        st.op_code = op_code;

        if !self.load_file(st, file_name) {
            // Clear the recorded command state so a later port-driven run
            // cannot emit a duplicate response for this answered command.
            st.block_state = BlockState::NoBlock;
            st.op_code = 0;
            st.cmd_seq = 0;
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }

        st.executed_count = 0;

        if st.step_mode == StepMode::Auto {
            self.start_auto(st);
        }

        if block == BlockState::NoBlock {
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
        }
    }

    /// `CS_VALIDATE(fileName)`.
    fn cs_validate_handler(
        &self,
        st: &mut CmdSequencerState,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        file_name: &CmdStringArg,
    ) {
        if !self.require_run_mode(st, RunMode::Stopped) {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        if !self.load_file(st, file_name) {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        // Validation does not leave the sequence runnable.
        st.sequence.clear();
        let name = st.sequence.log_file_name().clone();
        self.log_sequence_valid(&name);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// The shared body of `CS_CANCEL` and `seqCancelIn`.
    fn cancel_sequence(&self, st: &mut CmdSequencerState) {
        if st.run_mode == RunMode::Running {
            self.perform_cmd_cancel(st);
            let name = st.sequence.log_file_name().clone();
            self.log_sequence_canceled(&name);
            st.cancel_cmd_count = st.cancel_cmd_count.wrapping_add(1);
            self.tlm_u32(Self::CHANID_CS_CANCEL_COMMANDS, st.cancel_cmd_count);
        } else {
            self.log_no_sequence_active();
        }
    }

    /// `CS_CANCEL()` — always answers OK.
    fn cs_cancel_handler(&self, st: &mut CmdSequencerState, op_code: FwOpcodeType, cmd_seq: u32) {
        self.cancel_sequence(st);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `CS_START()`.
    fn cs_start_handler(&self, st: &mut CmdSequencerState, op_code: FwOpcodeType, cmd_seq: u32) {
        if !st.sequence.has_more_records() {
            self.log_no_sequence_active();
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        if !self.require_run_mode(st, RunMode::Stopped) {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        st.block_state = BlockState::NoBlock;
        st.run_mode = RunMode::Running;
        let name = st.sequence.string_file_name().clone();
        self.tlm_current_sequence(st, &name);
        let log_name = st.sequence.log_file_name().clone();
        self.log_cmd_started(&log_name);
        self.perform_cmd_step(st);
        // C++ emits seqStartOut AFTER the first step here (unlike CS_RUN).
        self.emit_seq_start(st);
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `CS_STEP()` — MANUAL step mode only.
    fn cs_step_handler(&self, st: &mut CmdSequencerState, op_code: FwOpcodeType, cmd_seq: u32) {
        if !self.require_run_mode(st, RunMode::Running) {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        if st.step_mode != StepMode::Manual {
            self.log_invalid_mode();
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        if !st.sequence.has_more_records() {
            // A sequence with no end-of-sequence record leaves nothing to
            // step; stepping anyway asserts in the sequence reader.
            self.log_no_sequence_active();
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
            return;
        }
        self.perform_cmd_step(st);
        // Special case: an END_OF_SEQUENCE record stopped the sequence.
        if st.run_mode != RunMode::Stopped {
            let name = st.sequence.log_file_name().clone();
            self.log_cmd_stepped(&name, st.executed_count);
        }
        self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
    }

    /// `CS_AUTO()` — reports `SeqMode::Auto` (1).
    fn cs_auto_handler(&self, st: &mut CmdSequencerState, op_code: FwOpcodeType, cmd_seq: u32) {
        if self.require_run_mode(st, RunMode::Stopped) {
            st.step_mode = StepMode::Auto;
            self.log_mode_switched(SeqMode::Auto);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
        } else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
        }
    }

    /// `CS_MANUAL()` — reports `SeqMode::Step` (0), the inverted enum.
    fn cs_manual_handler(&self, st: &mut CmdSequencerState, op_code: FwOpcodeType, cmd_seq: u32) {
        if self.require_run_mode(st, RunMode::Stopped) {
            st.step_mode = StepMode::Manual;
            self.log_mode_switched(SeqMode::Step);
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
        } else {
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
        }
    }

    /// `CS_JOIN_WAIT()` — defers its response until the sequence ends.
    fn cs_join_wait_handler(
        &self,
        st: &mut CmdSequencerState,
        op_code: FwOpcodeType,
        cmd_seq: u32,
    ) {
        if st.run_mode != RunMode::Running {
            self.log_no_sequence_active();
            self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
        } else if st.block_state == BlockState::Block || st.join_waiting {
            // A response is already owed to a BLOCK-mode CS_RUN caller or a
            // previous CS_JOIN_WAIT; the deferred slot holds only one.
            self.log_join_waiting_not_complete();
            self.cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
        } else {
            st.join_waiting = true;
            let name = st.sequence.log_file_name().clone();
            // C++ parity: the event carries the PREVIOUS cmdSeq/opCode.
            self.log_join_waiting(&name, st.cmd_seq, get_event_opcode(st.op_code));
            st.cmd_seq = cmd_seq;
            st.op_code = op_code;
        }
    }
}

// ---------------------------------------------------------------------------
// Port handlers (all async — component thread).
// ---------------------------------------------------------------------------

impl CmdSequencer {
    /// C++ `doSequenceRun(filename)` — shared by `seqRunIn` and
    /// `seqDispatchIn`.
    fn do_sequence_run(&self, st: &mut CmdSequencerState, filename: &FileNameString) {
        if st.step_mode == StepMode::Manual {
            // Nothing executes until CS_STEP, so a port-driven run would
            // wedge.
            self.log_invalid_mode();
            self.seq_done_unguarded(CmdResponse::ExecutionError);
            return;
        }
        if !self.require_run_mode(st, RunMode::Stopped) {
            self.seq_done_unguarded(CmdResponse::ExecutionError);
            return;
        }

        if !filename.is_empty() {
            // C++ `Fw::CmdStringArg cmdStr(filename)` — truncates to 40.
            let mut cmd_str = CmdStringArg::new();
            cmd_str.set_bytes(filename.as_bytes());
            if !self.load_file(st, &cmd_str) {
                self.seq_done_unguarded(CmdResponse::ExecutionError);
                return;
            }
        } else if !st.sequence.has_more_records() {
            self.log_no_sequence_active();
            self.error(st);
            self.seq_done_unguarded(CmdResponse::ExecutionError);
            return;
        }

        st.executed_count = 0;

        if st.step_mode == StepMode::Auto {
            self.start_auto(st);
        }

        let name = st.sequence.log_file_name().clone();
        self.log_port_sequence_started(&name);
    }

    /// `seqRunIn_handler` — the `args` value is explicitly IGNORED.
    fn seq_run_in_handler(
        &self,
        _port_num: FwIndexType,
        filename: &FileNameString,
        _args: &SeqArgs,
    ) {
        let mut guard = self.state.lock().unwrap();
        let st = &mut *guard;
        self.do_sequence_run(st, filename);
    }

    /// `seqDispatchIn_handler`.
    fn seq_dispatch_in_handler(&self, _port_num: FwIndexType, file_name: &mut FileNameString) {
        let mut guard = self.state.lock().unwrap();
        let st = &mut *guard;
        self.do_sequence_run(st, file_name);
    }

    /// `seqCancelIn_handler` — the `CS_CANCEL` body minus the response.
    fn seq_cancel_in_handler(&self, _port_num: FwIndexType) {
        let mut guard = self.state.lock().unwrap();
        let st = &mut *guard;
        self.cancel_sequence(st);
    }

    /// `pingIn_handler` — echo the key on `pingOut`.
    fn ping_in_handler(&self, _port_num: FwIndexType, key: u32) {
        let p = self.ping_out.get();
        p.target.invoke(p.port_num, key);
    }

    /// `cmdResponseIn_handler` — the completion of a sequenced command.
    fn cmd_response_in_handler(
        &self,
        _port_num: FwIndexType,
        opcode: FwOpcodeType,
        _cmd_seq: u32,
        response: CmdResponse,
    ) {
        let mut guard = self.state.lock().unwrap();
        let st = &mut *guard;
        if st.run_mode == RunMode::Stopped {
            self.log_unexpected_completion(get_event_opcode(opcode));
            return;
        }
        st.cmd_timeout_timer.clear();
        if response != CmdResponse::Ok {
            let executed = st.executed_count;
            self.command_error(st, executed, opcode, response as u32);
            self.perform_cmd_cancel(st);
        } else if st.step_mode == StepMode::Auto {
            self.command_complete(st, opcode);
            if st.sequence.has_more_records() {
                self.perform_cmd_step(st);
            } else {
                st.run_mode = RunMode::Stopped;
                self.sequence_complete(st);
            }
        } else {
            // MANUAL: the next record waits for CS_STEP.
            self.command_complete(st, opcode);
            if !st.sequence.has_more_records() {
                st.run_mode = RunMode::Stopped;
                self.sequence_complete(st);
            }
        }
    }

    /// `schedIn_handler` — drives both timers.
    ///
    /// C++ parity: the `else if` means a due timed dispatch suppresses the
    /// timeout check on the same tick, and the timeout branch reads the time
    /// port a SECOND time.
    fn sched_in_handler(&self, _port_num: FwIndexType, _order: u32) {
        let mut guard = self.state.lock().unwrap();
        let st = &mut *guard;
        let curr_time = self.evt.time_get();
        if st.cmd_timer.is_expired_at(&curr_time) {
            let p = self.com_cmd_out.get();
            p.target.invoke(p.port_num, &mut st.record.command, 0);
            st.cmd_timer.clear();
            self.set_cmd_timeout(st, &curr_time);
        } else {
            let timeout_time = self.evt.time_get();
            if st.cmd_timeout_timer.is_expired_at(&timeout_time) {
                let name = st.sequence.log_file_name().clone();
                let executed = st.executed_count;
                self.log_sequence_timeout(&name, executed);
                self.perform_cmd_cancel(st);
            }
        }
    }

    /// `cmdIn` dispatch: opcode decode, argument deserialization and the
    /// exactly-once response discipline.
    fn cmd_in_handler(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        let mut guard = self.state.lock().unwrap();
        let st = &mut *guard;
        match op_code.wrapping_sub(self.id_base()) {
            Self::OPCODE_CS_RUN => {
                let mut file_name = CmdStringArg::new();
                if !args.deserialize(&mut file_name, Endianness::Big).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                let mut raw_block = 0u8;
                if !args.deserialize_u8_be(&mut raw_block).is_ok() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                let Ok(block) = BlockState::try_from(raw_block) else {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::ValidationError);
                    return;
                };
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.cs_run_handler(st, op_code, cmd_seq, &file_name, block);
            }
            Self::OPCODE_CS_VALIDATE => {
                let mut file_name = CmdStringArg::new();
                if !args.deserialize(&mut file_name, Endianness::Big).is_ok()
                    || args.deserialize_size_left() != 0
                {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                self.cs_validate_handler(st, op_code, cmd_seq, &file_name);
            }
            opcode_offset @ (Self::OPCODE_CS_CANCEL
            | Self::OPCODE_CS_START
            | Self::OPCODE_CS_STEP
            | Self::OPCODE_CS_AUTO
            | Self::OPCODE_CS_MANUAL
            | Self::OPCODE_CS_JOIN_WAIT) => {
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                match opcode_offset {
                    Self::OPCODE_CS_CANCEL => self.cs_cancel_handler(st, op_code, cmd_seq),
                    Self::OPCODE_CS_START => self.cs_start_handler(st, op_code, cmd_seq),
                    Self::OPCODE_CS_STEP => self.cs_step_handler(st, op_code, cmd_seq),
                    Self::OPCODE_CS_AUTO => self.cs_auto_handler(st, op_code, cmd_seq),
                    Self::OPCODE_CS_MANUAL => self.cs_manual_handler(st, op_code, cmd_seq),
                    _ => self.cs_join_wait_handler(st, op_code, cmd_seq),
                }
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }
}

// ---------------------------------------------------------------------------
// Async input adapters (generated by the codegen layer).
// ---------------------------------------------------------------------------

async_input_port_adapter! {
    /// `seqCancelIn` — ASYNC `Svc.CmdSeqCancel` input (no arguments).
    component: CmdSequencer;
    adapter: SeqCancelInAdapter;
    port: CmdSeqCancelPort;
    input: pub seq_cancel_in;
    deserialize: seq_cancel_in_deserialize;
    handler: seq_cancel_in_handler;
    base: active.queued;
    msg_type: CmdSequencer::MSG_TYPE_SEQ_CANCEL_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { }
}

async_input_port_adapter! {
    /// `cmdResponseIn` — ASYNC `Fw.CmdResponse` input.
    component: CmdSequencer;
    adapter: CmdResponseInAdapter;
    port: CmdResponsePort;
    input: pub cmd_response_in;
    deserialize: cmd_response_in_deserialize;
    handler: cmd_response_in_handler;
    base: active.queued;
    msg_type: CmdSequencer::MSG_TYPE_CMD_RESPONSE_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, val response: CmdResponse }
}

async_input_port_adapter! {
    /// `pingIn` — ASYNC `Svc.Ping` input.
    component: CmdSequencer;
    adapter: PingInAdapter;
    port: PingPort;
    input: pub ping_in;
    deserialize: ping_in_deserialize;
    handler: ping_in_handler;
    base: active.queued;
    msg_type: CmdSequencer::MSG_TYPE_PING_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val key: u32 }
}

async_input_port_adapter! {
    /// `seqRunIn` — ASYNC `Svc.CmdSeqIn` input. The `args` value rides the
    /// queue (C++ serializes it too) and is then ignored by the handler.
    component: CmdSequencer;
    adapter: SeqRunInAdapter;
    port: CmdSeqInPort;
    input: pub seq_run_in;
    deserialize: seq_run_in_deserialize;
    handler: seq_run_in_handler;
    base: active.queued;
    msg_type: CmdSequencer::MSG_TYPE_SEQ_RUN_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { ref filename: FileNameString, ref args: SeqArgs }
}

async_input_port_adapter! {
    /// `seqDispatchIn` — ASYNC `Svc.FileDispatch` input (C++ `ref` file
    /// name: copied into the message, never written back).
    component: CmdSequencer;
    adapter: SeqDispatchInAdapter;
    port: FileDispatchPort;
    input: pub seq_dispatch_in;
    deserialize: seq_dispatch_in_deserialize;
    handler: seq_dispatch_in_handler;
    base: active.queued;
    msg_type: CmdSequencer::MSG_TYPE_SEQ_DISPATCH_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { mut file_name: FileNameString }
}

async_input_port_adapter! {
    /// `schedIn` — ASYNC `Svc.Sched` input: the timer tick.
    component: CmdSequencer;
    adapter: SchedInAdapter;
    port: SchedPort;
    input: pub sched_in;
    deserialize: sched_in_deserialize;
    handler: sched_in_handler;
    base: active.queued;
    msg_type: CmdSequencer::MSG_TYPE_SCHED_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val order: u32 }
}

async_input_port_adapter! {
    /// `cmdIn` — ASYNC `Fw.Cmd` input for this component's own commands.
    component: CmdSequencer;
    adapter: CmdInAdapter;
    port: CmdPort;
    input: pub cmd_in;
    deserialize: cmd_in_deserialize;
    handler: cmd_in_handler;
    base: active.queued;
    msg_type: CmdSequencer::MSG_TYPE_CMD_IN;
    msg_size: QUEUE_MSG_SIZE;
    priority: QUEUE_PRIORITY;
    queue_full: QueueFullPolicy::Assert;
    args { val op_code: FwOpcodeType, val cmd_seq: u32, buf args: CmdArgBuffer }
}

// ---------------------------------------------------------------------------
// Dispatch (the generated `doDispatch` switch).
// ---------------------------------------------------------------------------

impl ComponentDispatch for CmdSequencer {
    fn dispatch_message(
        &self,
        msg_type: FwEnumStoreType,
        buf: &mut dyn SerBufAny,
    ) -> MsgDispatchStatus {
        let mut port_num: FwIndexType = 0;
        if !msg::read_port_num(buf, &mut port_num).is_ok() {
            return MsgDispatchStatus::Error;
        }
        match msg_type {
            Self::MSG_TYPE_SEQ_CANCEL_IN => match Self::seq_cancel_in_deserialize(buf) {
                Some(()) => {
                    self.seq_cancel_in_handler(port_num);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_CMD_RESPONSE_IN => match Self::cmd_response_in_deserialize(buf) {
                Some((op_code, cmd_seq, response)) => {
                    self.cmd_response_in_handler(port_num, op_code, cmd_seq, response);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_PING_IN => match Self::ping_in_deserialize(buf) {
                Some((key,)) => {
                    self.ping_in_handler(port_num, key);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_SEQ_RUN_IN => match Self::seq_run_in_deserialize(buf) {
                Some((filename, args)) => {
                    self.seq_run_in_handler(port_num, &filename, &args);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_SEQ_DISPATCH_IN => match Self::seq_dispatch_in_deserialize(buf) {
                Some((mut file_name,)) => {
                    self.seq_dispatch_in_handler(port_num, &mut file_name);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_SCHED_IN => match Self::sched_in_deserialize(buf) {
                Some((order,)) => {
                    self.sched_in_handler(port_num, order);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            Self::MSG_TYPE_CMD_IN => match Self::cmd_in_deserialize(buf) {
                Some((op_code, cmd_seq, mut args)) => {
                    self.cmd_in_handler(port_num, op_code, cmd_seq, &mut args);
                    MsgDispatchStatus::Ok
                }
                None => MsgDispatchStatus::Error,
            },
            _ => MsgDispatchStatus::Error,
        }
    }
}

impl ActiveComponent for CmdSequencer {
    fn active_base(&self) -> &ActiveBase {
        &self.active
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdRegPort, LogPort, LogTextPort, TimePort, TlmPort};
    use fprime_fw::{Serialize, TextLogString, TlmBuffer};
    use std::path::{Path, PathBuf};
    use std::sync::atomic::{AtomicU32, Ordering};

    /// Ref topology base id for `cmdSeq` (instances.fpp).
    const ID_BASE: FwIdType = 0x1000_6000;
    /// Ref topology allocation (`RefTopology.cpp`).
    const BUFFER_BYTES: usize = 5 * 1024;

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    /// Command string arguments are `Fw::CmdStringArg` (40 bytes), so test
    /// paths must stay short.
    fn temp_dir() -> PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "fpcs{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&path).unwrap();
        path
    }

    fn write_file(dir: &Path, name: &str, bytes: &[u8]) -> String {
        let mut path = dir.to_path_buf();
        path.push(name);
        std::fs::write(&path, bytes).unwrap();
        path.to_str().unwrap().to_string()
    }

    fn cmd_string(value: &str) -> CmdStringArg {
        let mut out = CmdStringArg::new();
        out.set(value);
        out
    }

    // -- Ground stubs -------------------------------------------------------

    /// Settable clock on `timeCaller`.
    struct TimeStub {
        time: Mutex<Time>,
    }

    impl TimeStub {
        fn new() -> Arc<Self> {
            Arc::new(Self {
                time: Mutex::new(Time::new(TimeBase::TbWorkstationTime, 0, 1000, 0)),
            })
        }
        fn set(&self, seconds: u32, useconds: u32) {
            let mut guard = self.time.lock().unwrap();
            let base = guard.get_time_base();
            let context = guard.get_context();
            *guard = Time::new(base, context, seconds, useconds);
        }
        fn set_full(&self, base: TimeBase, context: u8, seconds: u32, useconds: u32) {
            *self.time.lock().unwrap() = Time::new(base, context, seconds, useconds);
        }
    }

    impl TimePort for TimeStub {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = *self.time.lock().unwrap();
        }
    }

    type EventRecord = (FwEventIdType, LogSeverity, Vec<u8>);

    #[derive(Default)]
    struct Ground {
        regs: Mutex<Vec<FwOpcodeType>>,
        responses: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
        events: Mutex<Vec<EventRecord>>,
        texts: Mutex<Vec<(FwEventIdType, String)>>,
        tlm: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
        pings: Mutex<Vec<u32>>,
        com: Mutex<Vec<(Vec<u8>, u32)>>,
        seq_start: Mutex<Vec<(Vec<u8>, SeqArgs)>>,
    }

    impl Ground {
        fn event_ids(&self) -> Vec<FwEventIdType> {
            self.events.lock().unwrap().iter().map(|e| e.0).collect()
        }
        fn find_event(&self, id: FwEventIdType) -> Option<EventRecord> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .find(|e| e.0 == id)
                .cloned()
        }
        fn count_event(&self, id: FwEventIdType) -> usize {
            self.events
                .lock()
                .unwrap()
                .iter()
                .filter(|e| e.0 == id)
                .count()
        }
        fn last_tlm(&self, chan: FwChanIdType) -> Option<Vec<u8>> {
            self.tlm
                .lock()
                .unwrap()
                .iter()
                .rev()
                .find(|t| t.0 == chan)
                .map(|t| t.1.clone())
        }
        fn tlm_u32(&self, chan: FwChanIdType) -> Option<u32> {
            self.last_tlm(chan).map(|bytes| {
                let mut raw = [0u8; 4];
                raw.copy_from_slice(&bytes[..4]);
                u32::from_be_bytes(raw)
            })
        }
        fn clear(&self) {
            self.regs.lock().unwrap().clear();
            self.responses.lock().unwrap().clear();
            self.events.lock().unwrap().clear();
            self.texts.lock().unwrap().clear();
            self.tlm.lock().unwrap().clear();
            self.pings.lock().unwrap().clear();
            self.com.lock().unwrap().clear();
            self.seq_start.lock().unwrap().clear();
        }
    }

    impl CmdRegPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, op_code: FwOpcodeType) {
            self.regs.lock().unwrap().push(op_code);
        }
    }

    impl CmdResponsePort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            op_code: FwOpcodeType,
            cmd_seq: u32,
            response: CmdResponse,
        ) {
            self.responses
                .lock()
                .unwrap()
                .push((op_code, cmd_seq, response));
        }
    }

    impl LogPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            // Recorded FPP-relative so assertions read like the .fppi.
            self.events.lock().unwrap().push((
                id.wrapping_sub(ID_BASE),
                severity,
                args.as_slice().to_vec(),
            ));
        }
    }

    impl LogTextPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            text: &mut TextLogString,
        ) {
            self.texts.lock().unwrap().push((
                id.wrapping_sub(ID_BASE),
                text.as_str().unwrap_or("").to_string(),
            ));
        }
    }

    impl TlmPort for Ground {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            _time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            self.tlm
                .lock()
                .unwrap()
                .push((id.wrapping_sub(ID_BASE), val.as_slice().to_vec()));
        }
    }

    impl PingPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, key: u32) {
            self.pings.lock().unwrap().push(key);
        }
    }

    impl ComPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, data: &mut ComBuffer, context: u32) {
            self.com
                .lock()
                .unwrap()
                .push((data.as_slice().to_vec(), context));
        }
    }

    impl CmdSeqInPort for Ground {
        fn invoke(&self, _port_num: FwIndexType, filename: &FileNameString, args: &SeqArgs) {
            self.seq_start
                .lock()
                .unwrap()
                .push((filename.as_bytes().to_vec(), *args));
        }
    }

    /// `seqDone` needs its own recorder: it is a `Fw.CmdResponse` port like
    /// `cmdResponseOut` but a distinct destination.
    #[derive(Default)]
    struct SeqDoneStub {
        calls: Mutex<Vec<(FwOpcodeType, u32, CmdResponse)>>,
    }

    impl CmdResponsePort for SeqDoneStub {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            op_code: FwOpcodeType,
            cmd_seq: u32,
            response: CmdResponse,
        ) {
            self.calls
                .lock()
                .unwrap()
                .push((op_code, cmd_seq, response));
        }
    }

    struct Harness {
        comp: Arc<CmdSequencer>,
        ground: Arc<Ground>,
        seq_done: Arc<SeqDoneStub>,
        clock: Arc<TimeStub>,
    }

    fn build_with(connect_seq_done: bool, connect_seq_start: bool) -> Harness {
        let ground = Arc::new(Ground::default());
        let seq_done = Arc::new(SeqDoneStub::default());
        let clock = TimeStub::new();
        let comp = CmdSequencer::new("cmdSeq");
        comp.active.queued.base.set_id_base(ID_BASE);
        comp.cmd.cmd_reg_out.connect(ground.clone(), 0);
        comp.cmd.cmd_response_out.connect(ground.clone(), 0);
        comp.evt.log_out.connect(ground.clone(), 0);
        comp.evt.text_log_out.connect(ground.clone(), 0);
        comp.evt.time_out.connect(clock.clone(), 0);
        comp.tlm.tlm_out.connect(ground.clone(), 0);
        comp.com_cmd_out.connect(ground.clone(), 0);
        comp.ping_out.connect(ground.clone(), 0);
        if connect_seq_done {
            comp.seq_done.connect(seq_done.clone(), 0);
        }
        if connect_seq_start {
            comp.seq_start_out.connect(ground.clone(), 0);
        }
        comp.init(32);
        comp.allocate_buffer(0, BUFFER_BYTES);
        comp.reg_commands();
        Harness {
            comp,
            ground,
            seq_done,
            clock,
        }
    }

    fn build() -> Harness {
        build_with(true, true)
    }

    impl Harness {
        /// Invoke `cmdIn` and drain the queue on the test thread.
        fn command(&self, opcode_offset: FwOpcodeType, cmd_seq: u32, args: &[u8]) {
            let mut buf = CmdArgBuffer::new();
            assert!(
                buf.serialize_bytes(args, LengthMode::OmitLength, Endianness::Big)
                    .is_ok()
            );
            let p = self.comp.cmd_in(0);
            p.target
                .invoke(p.port_num, ID_BASE + opcode_offset, cmd_seq, &mut buf);
            self.drain();
        }

        fn drain(&self) {
            let _ = self
                .comp
                .active
                .queued
                .dispatch_available_messages(self.comp.as_ref());
        }

        fn cmd_response_in(&self, opcode: FwOpcodeType, cmd_seq: u32, response: CmdResponse) {
            let p = self.comp.cmd_response_in(0);
            p.target.invoke(p.port_num, opcode, cmd_seq, response);
            self.drain();
        }

        fn sched_in(&self, order: u32) {
            let p = self.comp.sched_in(0);
            p.target.invoke(p.port_num, order);
            self.drain();
        }

        fn seq_run_in(&self, filename: &str) {
            let mut name = FileNameString::new();
            name.set(filename);
            let args = SeqArgs::default();
            let p = self.comp.seq_run_in(0);
            p.target.invoke(p.port_num, &name, &args);
            self.drain();
        }

        fn seq_dispatch_in(&self, filename: &str) {
            let mut name = FileNameString::new();
            name.set(filename);
            let p = self.comp.seq_dispatch_in(0);
            p.target.invoke(p.port_num, &mut name);
            self.drain();
        }

        fn seq_cancel_in(&self) {
            let p = self.comp.seq_cancel_in(0);
            p.target.invoke(p.port_num);
            self.drain();
        }

        fn responses(&self) -> Vec<(FwOpcodeType, u32, CmdResponse)> {
            self.ground.responses.lock().unwrap().clone()
        }

        fn com_packets(&self) -> Vec<Vec<u8>> {
            self.ground
                .com
                .lock()
                .unwrap()
                .iter()
                .map(|c| c.0.clone())
                .collect()
        }
    }

    // -- Sequence file construction ----------------------------------------

    /// A command com packet: `[u16 0x0000][opcode u32 BE][raw args]`.
    fn command_packet(opcode: u32, args: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&0u16.to_be_bytes());
        out.extend_from_slice(&opcode.to_be_bytes());
        out.extend_from_slice(args);
        out
    }

    /// A command record: `[descriptor][seconds][useconds][recordSize][cmd]`.
    fn command_record(descriptor: u8, seconds: u32, useconds: u32, command: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        out.push(descriptor);
        out.extend_from_slice(&seconds.to_be_bytes());
        out.extend_from_slice(&useconds.to_be_bytes());
        out.extend_from_slice(&(command.len() as u32).to_be_bytes());
        out.extend_from_slice(command);
        out
    }

    /// The one-byte end-of-sequence record.
    fn eos_record() -> Vec<u8> {
        vec![RecordDescriptor::EndOfSequence as u8]
    }

    /// Assemble a complete sequence file. `file_size = records + 4`.
    fn sequence_file(
        num_records: u32,
        time_base: u16,
        time_context: u8,
        records: &[u8],
    ) -> Vec<u8> {
        let mut out = Vec::new();
        let file_size = (records.len() + SEQUENCE_CRC_SIZE) as u32;
        out.extend_from_slice(&file_size.to_be_bytes());
        out.extend_from_slice(&num_records.to_be_bytes());
        out.extend_from_slice(&time_base.to_be_bytes());
        out.push(time_context);
        out.extend_from_slice(records);
        let crc = Hash::hash_u32(&out);
        out.extend_from_slice(&crc.to_be_bytes());
        out
    }

    /// The default two-record file used by most runtime tests: one RELATIVE
    /// command at t+0 then END_OF_SEQUENCE.
    fn simple_sequence(opcode: u32) -> Vec<u8> {
        let mut records = command_record(
            RecordDescriptor::Relative as u8,
            0,
            0,
            &command_packet(opcode, &[]),
        );
        records.extend_from_slice(&eos_record());
        sequence_file(2, TimeBase::TbWorkstationTime as u16, 0, &records)
    }

    // -- Wire format: literal bytes ----------------------------------------

    #[test]
    fn sequence_file_header_is_eleven_big_endian_bytes() {
        let records = eos_record();
        let file = sequence_file(1, TimeBase::TbScTime as u16, 0x07, &records);
        // [fileSize u32][numRecords u32][timeBase u16][timeContext u8]
        assert_eq!(&file[0..4], &[0x00, 0x00, 0x00, 0x05]); // 1 record + 4 CRC
        assert_eq!(&file[4..8], &[0x00, 0x00, 0x00, 0x01]);
        assert_eq!(&file[8..10], &[0x00, 0x03]); // TB_SC_TIME
        assert_eq!(file[10], 0x07);
        assert_eq!(file[11], 0x02); // END_OF_SEQUENCE
        assert_eq!(file.len(), SEQUENCE_HEADER_SIZE + 5);
    }

    #[test]
    fn command_record_layout_is_descriptor_time_size_packet() {
        let packet = command_packet(0x0000_0100, &[0xAA, 0xBB]);
        assert_eq!(packet, vec![0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xAA, 0xBB]);
        let record = command_record(RecordDescriptor::Absolute as u8, 0x1234, 0x5678, &packet);
        assert_eq!(
            record,
            vec![
                0x00, // ABSOLUTE
                0x00, 0x00, 0x12, 0x34, // seconds
                0x00, 0x00, 0x56, 0x78, // useconds
                0x00, 0x00, 0x00, 0x08, // recordSize = 8
                0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xAA, 0xBB,
            ]
        );
        assert_eq!(record.len(), 13 + packet.len());
    }

    #[test]
    fn crc_covers_header_and_records_but_not_the_crc_field() {
        let records = eos_record();
        let file = sequence_file(1, TimeBase::TbWorkstationTime as u16, 0, &records);
        let data_len = file.len() - SEQUENCE_CRC_SIZE;
        let expected = Hash::hash_u32(&file[..data_len]);
        let mut raw = [0u8; 4];
        raw.copy_from_slice(&file[data_len..]);
        assert_eq!(u32::from_be_bytes(raw), expected);
        // The header block alone is not enough.
        assert_ne!(Hash::hash_u32(&file[..SEQUENCE_HEADER_SIZE]), expected);
    }

    #[test]
    fn file_size_counts_the_trailing_crc() {
        let mut records = command_record(
            RecordDescriptor::Relative as u8,
            0,
            0,
            &command_packet(0x2A, &[]),
        );
        records.extend_from_slice(&eos_record());
        let file = sequence_file(2, TimeBase::TbWorkstationTime as u16, 0, &records);
        let mut raw = [0u8; 4];
        raw.copy_from_slice(&file[0..4]);
        let file_size = u32::from_be_bytes(raw) as usize;
        assert_eq!(file_size, records.len() + 4);
        assert_eq!(file.len(), SEQUENCE_HEADER_SIZE + file_size);
    }

    // -- Timer -------------------------------------------------------------

    #[test]
    fn timer_cleared_is_never_expired() {
        let timer = Timer::new();
        assert!(!timer.is_expired_at(&Time::new(TimeBase::TbNone, 0, 100, 0)));
    }

    #[test]
    fn timer_expires_on_equal_and_later_times_only() {
        let mut timer = Timer::new();
        timer.set(Time::new(TimeBase::TbNone, 0, 100, 0));
        assert!(!timer.is_expired_at(&Time::new(TimeBase::TbNone, 0, 99, 999_999)));
        assert!(timer.is_expired_at(&Time::new(TimeBase::TbNone, 0, 100, 0)));
        assert!(timer.is_expired_at(&Time::new(TimeBase::TbNone, 0, 100, 1)));
        timer.clear();
        assert!(!timer.is_expired_at(&Time::new(TimeBase::TbNone, 0, 100, 0)));
    }

    #[test]
    fn timer_treats_incomparable_times_as_expired() {
        // C++ `isExpiredAt` only rejects GT, so a mismatched time base — an
        // INCOMPARABLE comparison — counts as EXPIRED.
        let mut timer = Timer::new();
        timer.set(Time::new(TimeBase::TbScTime, 0, 10_000, 0));
        let now = Time::new(TimeBase::TbWorkstationTime, 0, 1, 0);
        assert_eq!(
            Time::compare(&timer.expiration_time(), &now),
            TimeComparison::Incomparable
        );
        assert!(timer.is_expired_at(&now));
    }

    // -- Load / validate ---------------------------------------------------

    #[test]
    fn validate_accepts_a_well_formed_sequence() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x42));
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 7, &validate_args(&path));
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_VALIDATE,
                7,
                CmdResponse::Ok
            )]
        );
        let ids = h.ground.event_ids();
        assert!(ids.contains(&CmdSequencer::EVENTID_CS_SEQUENCE_LOADED));
        assert!(ids.contains(&CmdSequencer::EVENTID_CS_SEQUENCE_VALID));
        assert_eq!(
            h.ground.tlm_u32(CmdSequencer::CHANID_CS_LOAD_COMMANDS),
            Some(1)
        );
        // CS_VALIDATE clears the sequence, so CS_START finds nothing.
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_START, 8, &[]);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_START,
                8,
                CmdResponse::ExecutionError
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE)
        );
    }

    /// `CS_VALIDATE(fileName)` argument bytes.
    fn validate_args(path: &str) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&(path.len() as u16).to_be_bytes());
        out.extend_from_slice(path.as_bytes());
        out
    }

    /// `CS_RUN(fileName, block)` argument bytes.
    fn run_args(path: &str, block: BlockState) -> Vec<u8> {
        let mut out = validate_args(path);
        out.push(block.as_repr());
        out
    }

    #[test]
    fn load_reports_file_not_found() {
        let h = build();
        h.command(
            CmdSequencer::OPCODE_CS_VALIDATE,
            1,
            &validate_args("/x/none"),
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_VALIDATE,
                1,
                CmdResponse::ExecutionError
            )]
        );
        assert!(
            h.ground
                .find_event(CmdSequencer::EVENTID_CS_FILE_NOT_FOUND)
                .is_some()
        );
        assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
    }

    #[test]
    fn load_reports_crc_failure_with_stored_and_computed() {
        let h = build();
        let dir = temp_dir();
        let mut file = simple_sequence(0x42);
        let last = file.len() - 1;
        file[last] ^= 0xFF;
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 2, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_FILE_CRC_FAILURE)
            .expect("CS_FileCrcFailure");
        assert_eq!(event.1, LogSeverity::WarningHi);
        // [u16 len][name][stored u32][computed u32]
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        let stored = u32::from_be_bytes([tail[0], tail[1], tail[2], tail[3]]);
        let computed = u32::from_be_bytes([tail[4], tail[5], tail[6], tail[7]]);
        let data_len = file.len() - SEQUENCE_CRC_SIZE;
        assert_eq!(computed, Hash::hash_u32(&file[..data_len]));
        assert_ne!(stored, computed);
        assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
    }

    #[test]
    fn load_reports_bad_record_descriptor() {
        let h = build();
        let dir = temp_dir();
        // Descriptor 3 is > END_OF_SEQUENCE.
        let records = vec![0x03u8];
        let file = sequence_file(1, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 3, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_RECORD_INVALID)
            .expect("CS_RecordInvalid");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(
            u32::from_be_bytes([tail[0], tail[1], tail[2], tail[3]]),
            0,
            "record number"
        );
        assert_eq!(
            i32::from_be_bytes([tail[4], tail[5], tail[6], tail[7]]),
            SerializeStatus::DeserFormatError as i32
        );
    }

    #[test]
    fn load_reports_record_size_beyond_remaining_data() {
        let h = build();
        let dir = temp_dir();
        let mut records = vec![RecordDescriptor::Absolute as u8];
        records.extend_from_slice(&0u32.to_be_bytes());
        records.extend_from_slice(&0u32.to_be_bytes());
        records.extend_from_slice(&64u32.to_be_bytes()); // claims 64 bytes
        records.extend_from_slice(&[0u8; 8]); // but only 8 follow
        let file = sequence_file(1, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 4, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_RECORD_INVALID)
            .expect("CS_RecordInvalid");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(
            i32::from_be_bytes([tail[4], tail[5], tail[6], tail[7]]),
            SerializeStatus::DeserSizeMismatch as i32
        );
    }

    #[test]
    fn record_size_check_is_over_strict_by_two_bytes() {
        // C++ rejects `recordSize + sizeof(FwPacketDescriptorType) > 512`
        // even though recordSize already includes the descriptor, so 511 is
        // refused and 510 is accepted. Ported verbatim.
        let h = build();
        let dir = temp_dir();
        for (size, expect_ok) in [(510usize, true), (511usize, false)] {
            h.ground.clear();
            let mut packet = command_packet(0x55, &[]);
            packet.resize(size, 0x5A);
            let mut records = command_record(RecordDescriptor::Relative as u8, 0, 0, &packet);
            records.extend_from_slice(&eos_record());
            let file = sequence_file(2, TimeBase::TbWorkstationTime as u16, 0, &records);
            let path = write_file(&dir, &format!("r{size}.bin"), &file);
            h.command(CmdSequencer::OPCODE_CS_VALIDATE, 5, &validate_args(&path));
            let valid = h
                .ground
                .find_event(CmdSequencer::EVENTID_CS_SEQUENCE_VALID)
                .is_some();
            assert_eq!(valid, expect_ok, "record size {size}");
        }
    }

    #[test]
    fn load_reports_no_records_when_header_declares_zero() {
        let h = build();
        let dir = temp_dir();
        let file = sequence_file(0, TimeBase::TbWorkstationTime as u16, 0, &eos_record());
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 6, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_NO_RECORDS)
            .expect("CS_NoRecords");
        assert_eq!(event.1, LogSeverity::WarningLo);
        assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
    }

    #[test]
    fn record_mismatch_does_not_bump_the_error_counter() {
        let h = build();
        let dir = temp_dir();
        // Two records on the wire, one declared.
        let mut records = eos_record();
        records.extend_from_slice(&eos_record());
        let file = sequence_file(1, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 7, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_RECORD_MISMATCH)
            .expect("CS_RecordMismatch");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(u32::from_be_bytes([tail[0], tail[1], tail[2], tail[3]]), 1);
        assert_eq!(u32::from_be_bytes([tail[4], tail[5], tail[6], tail[7]]), 1);
        // The ONLY sequence event that does not increment CS_Errors.
        assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), None);
    }

    #[test]
    fn load_rejects_a_mismatched_time_base() {
        let h = build();
        let dir = temp_dir();
        let mut records = command_record(
            RecordDescriptor::Relative as u8,
            0,
            0,
            &command_packet(1, &[]),
        );
        records.extend_from_slice(&eos_record());
        let file = sequence_file(2, TimeBase::TbScTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 8, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_TIME_BASE_MISMATCH)
            .expect("CS_TimeBaseMismatch");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(
            u16::from_be_bytes([tail[0], tail[1]]),
            TimeBase::TbWorkstationTime as u16
        );
        assert_eq!(
            u16::from_be_bytes([tail[2], tail[3]]),
            TimeBase::TbScTime as u16
        );
    }

    #[test]
    fn load_rejects_a_mismatched_time_context() {
        let h = build();
        h.clock.set_full(TimeBase::TbWorkstationTime, 2, 1000, 0);
        let dir = temp_dir();
        let mut records = command_record(
            RecordDescriptor::Relative as u8,
            0,
            0,
            &command_packet(1, &[]),
        );
        records.extend_from_slice(&eos_record());
        let file = sequence_file(2, TimeBase::TbWorkstationTime as u16, 9, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 9, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_TIME_CONTEXT_MISMATCH)
            .expect("CS_TimeContextMismatch");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(tail[0], 2, "current context");
        assert_eq!(tail[1], 9, "sequence context");
    }

    #[test]
    fn dont_care_time_base_and_context_are_accepted_and_canonicalized() {
        let h = build();
        h.clock.set_full(TimeBase::TbWorkstationTime, 3, 1000, 0);
        let dir = temp_dir();
        let mut records = command_record(
            RecordDescriptor::Absolute as u8,
            1000,
            0,
            &command_packet(0x77, &[]),
        );
        records.extend_from_slice(&eos_record());
        let file = sequence_file(
            2,
            TimeBase::TbDontCare as u16,
            FW_CONTEXT_DONT_CARE,
            &records,
        );
        let path = write_file(&dir, "s.bin", &file);
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            10,
            &run_args(&path, BlockState::NoBlock),
        );
        // The ABSOLUTE tag is stamped with the LIVE base/context, so it
        // compares equal to "now" and the command goes out immediately.
        assert_eq!(h.com_packets().len(), 1);
        let state = h.comp.state.lock().unwrap();
        assert_eq!(
            state.sequence.header().time_base,
            TimeBase::TbWorkstationTime
        );
        assert_eq!(state.sequence.header().time_context, 3);
        assert_eq!(
            state.record.time_tag.get_time_base(),
            TimeBase::TbWorkstationTime
        );
        assert_eq!(state.record.time_tag.get_context(), 3);
    }

    #[test]
    fn load_reports_file_size_error_when_larger_than_the_buffer() {
        let h = build_with(true, true);
        h.comp.allocate_buffer(0, 64);
        let dir = temp_dir();
        let mut records = Vec::new();
        for _ in 0..8 {
            records.extend_from_slice(&command_record(
                RecordDescriptor::Relative as u8,
                0,
                0,
                &command_packet(1, &[0u8; 8]),
            ));
        }
        let file = sequence_file(8, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 11, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_FILE_SIZE_ERROR)
            .expect("CS_FileSizeError");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(
            u32::from_be_bytes([tail[0], tail[1], tail[2], tail[3]]) as usize,
            records.len() + 4
        );
    }

    #[test]
    fn load_reports_short_header_and_short_record_block() {
        let h = build();
        let dir = temp_dir();

        // Fewer than 11 header bytes.
        let path = write_file(&dir, "short.bin", &[0u8; 5]);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 12, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_FILE_INVALID)
            .expect("CS_FileInvalid");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(tail[0], FileReadStage::ReadHeaderSize.as_repr());
        assert_eq!(i32::from_be_bytes([tail[1], tail[2], tail[3], tail[4]]), 5);

        // A truncated record block.
        h.ground.clear();
        let mut file = simple_sequence(0x42);
        file.truncate(file.len() - 3);
        let path = write_file(&dir, "trunc.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 13, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_FILE_INVALID)
            .expect("CS_FileInvalid");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(tail[0], FileReadStage::ReadSeqDataSize.as_repr());
    }

    #[test]
    fn load_reports_invalid_time_base_value() {
        let h = build();
        let dir = temp_dir();
        let mut file = sequence_file(1, TimeBase::TbWorkstationTime as u16, 0, &eos_record());
        // 0x00AA is not a declared Fw.TimeBase constant.
        file[8] = 0x00;
        file[9] = 0xAA;
        let data_len = file.len() - SEQUENCE_CRC_SIZE;
        let crc = Hash::hash_u32(&file[..data_len]);
        file[data_len..].copy_from_slice(&crc.to_be_bytes());
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 14, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_FILE_INVALID)
            .expect("CS_FileInvalid");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(tail[0], FileReadStage::DeserTimeBase.as_repr());
    }

    #[test]
    fn crc_field_shorter_than_four_bytes_is_read_seq_crc() {
        let h = build();
        let dir = temp_dir();
        // fileSize = 2: the reader gets two bytes, fewer than the CRC width.
        let mut file = Vec::new();
        file.extend_from_slice(&2u32.to_be_bytes());
        file.extend_from_slice(&1u32.to_be_bytes());
        file.extend_from_slice(&(TimeBase::TbWorkstationTime as u16).to_be_bytes());
        file.push(0);
        file.extend_from_slice(&[0xAA, 0xBB]);
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 15, &validate_args(&path));
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_FILE_INVALID)
            .expect("CS_FileInvalid");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(tail[0], FileReadStage::ReadSeqCrc.as_repr());
        assert_eq!(i32::from_be_bytes([tail[1], tail[2], tail[3], tail[4]]), 2);
    }

    #[test]
    fn trailing_bytes_beyond_the_declared_file_size_are_ignored() {
        let h = build();
        let dir = temp_dir();
        let mut file = simple_sequence(0x42);
        file.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 16, &validate_args(&path));
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_VALIDATE,
                16,
                CmdResponse::Ok
            )]
        );
    }

    // -- AUTO-mode execution -----------------------------------------------

    #[test]
    fn auto_run_emits_the_exact_command_packet_and_completes() {
        let h = build();
        let dir = temp_dir();
        let packet = command_packet(0x0000_0123, &[0x01, 0x02, 0x03, 0x04]);
        let mut records = command_record(RecordDescriptor::Relative as u8, 0, 0, &packet);
        records.extend_from_slice(&eos_record());
        let file = sequence_file(2, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);

        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            21,
            &run_args(&path, BlockState::NoBlock),
        );

        // NO_BLOCK answers immediately.
        assert_eq!(
            h.responses(),
            vec![(ID_BASE + CmdSequencer::OPCODE_CS_RUN, 21, CmdResponse::Ok)]
        );
        // The com packet is the raw record payload, byte for byte.
        assert_eq!(h.com_packets(), vec![packet.clone()]);
        assert_eq!(h.ground.com.lock().unwrap()[0].1, 0, "context");
        // seqStartOut carries the file name and an all-zero SeqArgs.
        let starts = h.ground.seq_start.lock().unwrap().clone();
        assert_eq!(starts.len(), 1);
        assert_eq!(starts[0].0, path.as_bytes());
        assert_eq!(starts[0].1, SeqArgs::default());
        assert_eq!(starts[0].1.size, 0);
        assert_eq!(h.comp.run_mode(), RunMode::Running);

        // Command completes -> next record is END_OF_SEQUENCE.
        h.ground.clear();
        h.cmd_response_in(0x0000_0123, 0, CmdResponse::Ok);
        let ids = h.ground.event_ids();
        assert!(ids.contains(&CmdSequencer::EVENTID_CS_COMMAND_COMPLETE));
        assert!(ids.contains(&CmdSequencer::EVENTID_CS_SEQUENCE_COMPLETE));
        assert_eq!(
            h.ground.tlm_u32(CmdSequencer::CHANID_CS_COMMANDS_EXECUTED),
            Some(1)
        );
        assert_eq!(
            h.ground
                .tlm_u32(CmdSequencer::CHANID_CS_SEQUENCES_COMPLETED),
            Some(1)
        );
        assert_eq!(
            *h.seq_done.calls.lock().unwrap(),
            vec![(0, 0, CmdResponse::Ok)]
        );
        assert_eq!(h.comp.run_mode(), RunMode::Stopped);
        // CS_CurrentSequence goes back to "<no seq>".
        let current = h
            .ground
            .last_tlm(CmdSequencer::CHANID_CS_CURRENT_SEQUENCE)
            .unwrap();
        assert_eq!(&current[2..], NO_SEQ.as_bytes());
    }

    #[test]
    fn auto_run_walks_two_commands_in_order() {
        let h = build();
        let dir = temp_dir();
        let first = command_packet(0x11, &[0xAA]);
        let second = command_packet(0x22, &[0xBB, 0xCC]);
        let mut records = command_record(RecordDescriptor::Relative as u8, 0, 0, &first);
        records.extend_from_slice(&command_record(
            RecordDescriptor::Relative as u8,
            0,
            0,
            &second,
        ));
        records.extend_from_slice(&eos_record());
        let file = sequence_file(3, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);

        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            22,
            &run_args(&path, BlockState::NoBlock),
        );
        assert_eq!(h.com_packets(), vec![first.clone()]);
        h.cmd_response_in(0x11, 0, CmdResponse::Ok);
        assert_eq!(h.com_packets(), vec![first, second]);
        h.cmd_response_in(0x22, 0, CmdResponse::Ok);
        assert_eq!(
            h.ground.tlm_u32(CmdSequencer::CHANID_CS_COMMANDS_EXECUTED),
            Some(2)
        );
        assert_eq!(h.comp.run_mode(), RunMode::Stopped);
    }

    #[test]
    fn sequence_without_end_record_completes_on_the_last_response() {
        let h = build();
        let dir = temp_dir();
        let packet = command_packet(0x33, &[]);
        let records = command_record(RecordDescriptor::Relative as u8, 0, 0, &packet);
        let file = sequence_file(1, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            23,
            &run_args(&path, BlockState::NoBlock),
        );
        h.ground.clear();
        h.cmd_response_in(0x33, 0, CmdResponse::Ok);
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_SEQUENCE_COMPLETE)
        );
        assert_eq!(h.comp.run_mode(), RunMode::Stopped);
    }

    #[test]
    fn block_mode_defers_the_command_response_until_completion() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x42));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            24,
            &run_args(&path, BlockState::Block),
        );
        assert!(h.responses().is_empty(), "BLOCK defers the response");
        h.cmd_response_in(0x42, 0, CmdResponse::Ok);
        assert_eq!(
            h.responses(),
            vec![(ID_BASE + CmdSequencer::OPCODE_CS_RUN, 24, CmdResponse::Ok)]
        );
    }

    #[test]
    fn block_mode_answers_exactly_once_for_an_immediately_complete_sequence() {
        // Deviation from C++ (documented on `cs_run_handler`): the stock
        // implementation answers twice here because `sequenceComplete`
        // resets `m_blockState` before the trailing NO_BLOCK check.
        let h = build();
        let dir = temp_dir();
        let file = sequence_file(1, TimeBase::TbWorkstationTime as u16, 0, &eos_record());
        let path = write_file(&dir, "eos.bin", &file);
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            25,
            &run_args(&path, BlockState::Block),
        );
        assert_eq!(
            h.responses(),
            vec![(ID_BASE + CmdSequencer::OPCODE_CS_RUN, 25, CmdResponse::Ok)]
        );
    }

    #[test]
    fn command_error_cancels_the_sequence() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x42));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            26,
            &run_args(&path, BlockState::Block),
        );
        h.ground.clear();
        h.cmd_response_in(0x42, 0, CmdResponse::ExecutionError);
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_COMMAND_ERROR)
            .expect("CS_CommandError");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        assert_eq!(u32::from_be_bytes([tail[0], tail[1], tail[2], tail[3]]), 0);
        assert_eq!(
            u32::from_be_bytes([tail[4], tail[5], tail[6], tail[7]]),
            0x42
        );
        assert_eq!(
            u32::from_be_bytes([tail[8], tail[9], tail[10], tail[11]]),
            CmdResponse::ExecutionError as u32
        );
        assert_eq!(
            *h.seq_done.calls.lock().unwrap(),
            vec![(0, 0, CmdResponse::ExecutionError)]
        );
        // The BLOCK caller is answered with the failure.
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_RUN,
                26,
                CmdResponse::ExecutionError
            )]
        );
        assert_eq!(h.comp.run_mode(), RunMode::Stopped);
    }

    #[test]
    fn command_response_while_stopped_is_unexpected_completion() {
        let h = build();
        h.cmd_response_in(0x99, 3, CmdResponse::Ok);
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_UNEXPECTED_COMPLETION)
            .expect("CS_UnexpectedCompletion");
        assert_eq!(event.1, LogSeverity::WarningHi);
        assert_eq!(
            u32::from_be_bytes([event.2[0], event.2[1], event.2[2], event.2[3]]),
            get_event_opcode(0x99)
        );
    }

    // -- Timing ------------------------------------------------------------

    #[test]
    fn absolute_record_in_the_future_waits_for_sched_in() {
        let h = build();
        let dir = temp_dir();
        let packet = command_packet(0x44, &[]);
        let mut records = command_record(RecordDescriptor::Absolute as u8, 2000, 0, &packet);
        records.extend_from_slice(&eos_record());
        let file = sequence_file(2, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            27,
            &run_args(&path, BlockState::NoBlock),
        );
        assert!(h.com_packets().is_empty(), "tag is in the future");
        assert!(h.comp.state.lock().unwrap().cmd_timer.is_armed());

        // Still early.
        h.clock.set(1999, 999_999);
        h.sched_in(0);
        assert!(h.com_packets().is_empty());

        // Due now.
        h.clock.set(2000, 0);
        h.sched_in(0);
        assert_eq!(h.com_packets(), vec![packet]);
        assert!(!h.comp.state.lock().unwrap().cmd_timer.is_armed());
    }

    #[test]
    fn relative_record_adds_the_current_time_at_step_time() {
        let h = build();
        let dir = temp_dir();
        let packet = command_packet(0x45, &[]);
        let mut records = command_record(RecordDescriptor::Relative as u8, 30, 0, &packet);
        records.extend_from_slice(&eos_record());
        let file = sequence_file(2, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.clock.set(1000, 0);
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            28,
            &run_args(&path, BlockState::NoBlock),
        );
        assert!(h.com_packets().is_empty());
        {
            let state = h.comp.state.lock().unwrap();
            assert_eq!(state.cmd_timer.expiration_time().get_seconds(), 1030);
        }
        h.clock.set(1030, 0);
        h.sched_in(0);
        assert_eq!(h.com_packets(), vec![packet]);
    }

    #[test]
    fn command_response_timeout_cancels_the_sequence() {
        let h = build();
        h.comp.set_timeout(5);
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x46));
        h.clock.set(1000, 0);
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            29,
            &run_args(&path, BlockState::NoBlock),
        );
        assert_eq!(h.com_packets().len(), 1);
        {
            let state = h.comp.state.lock().unwrap();
            assert!(state.cmd_timeout_timer.is_armed());
            assert_eq!(
                state.cmd_timeout_timer.expiration_time().get_seconds(),
                1005
            );
        }
        // Not yet.
        h.clock.set(1004, 0);
        h.sched_in(0);
        assert_eq!(h.comp.run_mode(), RunMode::Running);
        // Expired.
        h.ground.clear();
        h.clock.set(1005, 0);
        h.sched_in(0);
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_SEQUENCE_TIMEOUT)
            .expect("CS_SequenceTimeout");
        assert_eq!(event.1, LogSeverity::WarningHi);
        assert_eq!(h.comp.run_mode(), RunMode::Stopped);
        assert_eq!(
            *h.seq_done.calls.lock().unwrap(),
            vec![(0, 0, CmdResponse::ExecutionError)]
        );
    }

    #[test]
    fn timeout_is_not_armed_when_disabled_or_in_manual_mode() {
        // timeout == 0 (default).
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x47));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            30,
            &run_args(&path, BlockState::NoBlock),
        );
        assert!(!h.comp.state.lock().unwrap().cmd_timeout_timer.is_armed());

        // MANUAL step mode never arms the watchdog.
        let h2 = build();
        h2.comp.set_timeout(5);
        h2.command(CmdSequencer::OPCODE_CS_MANUAL, 31, &[]);
        let dir2 = temp_dir();
        let path2 = write_file(&dir2, "s.bin", &simple_sequence(0x48));
        h2.command(
            CmdSequencer::OPCODE_CS_RUN,
            32,
            &run_args(&path2, BlockState::NoBlock),
        );
        h2.command(CmdSequencer::OPCODE_CS_START, 33, &[]);
        assert_eq!(h2.com_packets().len(), 1);
        assert!(!h2.comp.state.lock().unwrap().cmd_timeout_timer.is_armed());
    }

    #[test]
    fn due_command_dispatch_suppresses_the_timeout_check_on_the_same_tick() {
        let h = build();
        h.comp.set_timeout(5);
        let dir = temp_dir();
        let packet = command_packet(0x49, &[]);
        let mut records = command_record(RecordDescriptor::Absolute as u8, 2000, 0, &packet);
        records.extend_from_slice(&eos_record());
        let file = sequence_file(2, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.clock.set(1000, 0);
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            34,
            &run_args(&path, BlockState::NoBlock),
        );
        // Arm the (stale) timeout timer by hand so both would be expired.
        {
            let mut state = h.comp.state.lock().unwrap();
            state
                .cmd_timeout_timer
                .set(Time::new(TimeBase::TbWorkstationTime, 0, 1, 0));
        }
        h.ground.clear();
        h.clock.set(2000, 0);
        h.sched_in(0);
        // The dispatch branch won; no timeout event, sequence still running.
        assert_eq!(h.com_packets(), vec![packet]);
        assert_eq!(
            h.ground
                .count_event(CmdSequencer::EVENTID_CS_SEQUENCE_TIMEOUT),
            0
        );
        assert_eq!(h.comp.run_mode(), RunMode::Running);
    }

    // -- MANUAL mode -------------------------------------------------------

    #[test]
    fn manual_mode_requires_start_and_step() {
        let h = build();
        h.command(CmdSequencer::OPCODE_CS_MANUAL, 40, &[]);
        let dir = temp_dir();
        let first = command_packet(0x51, &[]);
        let second = command_packet(0x52, &[]);
        let mut records = command_record(RecordDescriptor::Relative as u8, 0, 0, &first);
        records.extend_from_slice(&command_record(
            RecordDescriptor::Relative as u8,
            0,
            0,
            &second,
        ));
        records.extend_from_slice(&eos_record());
        let file = sequence_file(3, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);

        // CS_RUN loads but does not start in MANUAL mode.
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            41,
            &run_args(&path, BlockState::NoBlock),
        );
        assert!(h.com_packets().is_empty());
        assert_eq!(h.comp.run_mode(), RunMode::Stopped);

        // CS_START runs the first record.
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_START, 42, &[]);
        assert_eq!(h.com_packets(), vec![first]);
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_CMD_STARTED)
        );
        assert_eq!(h.comp.run_mode(), RunMode::Running);

        // A response does NOT advance in MANUAL mode.
        h.ground.clear();
        h.cmd_response_in(0x51, 0, CmdResponse::Ok);
        assert!(h.com_packets().is_empty(), "no new command was dispatched");
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_COMMAND_COMPLETE)
        );

        // CS_STEP does.
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_STEP, 43, &[]);
        assert_eq!(h.com_packets(), vec![second]);
        let stepped = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_CMD_STEPPED)
            .expect("CS_CmdStepped");
        let name_len = u16::from_be_bytes([stepped.2[0], stepped.2[1]]) as usize;
        let tail = &stepped.2[2 + name_len..];
        assert_eq!(u32::from_be_bytes([tail[0], tail[1], tail[2], tail[3]]), 1);

        // The END_OF_SEQUENCE record is still pending, so the response
        // alone does not complete the sequence in MANUAL mode.
        h.ground.clear();
        h.cmd_response_in(0x52, 0, CmdResponse::Ok);
        assert!(
            !h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_SEQUENCE_COMPLETE)
        );
        assert_eq!(h.comp.run_mode(), RunMode::Running);

        // One more step reaches END_OF_SEQUENCE and completes.
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_STEP, 48, &[]);
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_SEQUENCE_COMPLETE)
        );
        assert_eq!(h.comp.run_mode(), RunMode::Stopped);
    }

    #[test]
    fn step_onto_end_of_sequence_suppresses_cmd_stepped() {
        let h = build();
        h.command(CmdSequencer::OPCODE_CS_MANUAL, 44, &[]);
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x53));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            45,
            &run_args(&path, BlockState::NoBlock),
        );
        h.command(CmdSequencer::OPCODE_CS_START, 46, &[]);
        h.cmd_response_in(0x53, 0, CmdResponse::Ok);
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_STEP, 47, &[]);
        let ids = h.ground.event_ids();
        assert!(ids.contains(&CmdSequencer::EVENTID_CS_SEQUENCE_COMPLETE));
        assert!(!ids.contains(&CmdSequencer::EVENTID_CS_CMD_STEPPED));
        assert_eq!(
            h.responses(),
            vec![(ID_BASE + CmdSequencer::OPCODE_CS_STEP, 47, CmdResponse::Ok)]
        );
    }

    // -- Cancel ------------------------------------------------------------

    #[test]
    fn cancel_rewinds_so_start_can_rerun_the_sequence() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x60));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            50,
            &run_args(&path, BlockState::NoBlock),
        );
        assert_eq!(h.com_packets().len(), 1);
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_CANCEL, 51, &[]);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_CANCEL,
                51,
                CmdResponse::Ok
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_SEQUENCE_CANCELED)
        );
        assert_eq!(
            h.ground.tlm_u32(CmdSequencer::CHANID_CS_CANCEL_COMMANDS),
            Some(1)
        );
        assert_eq!(h.comp.run_mode(), RunMode::Stopped);
        // `reset()` (not `clear()`): the sequence is still loaded.
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_START, 52, &[]);
        assert_eq!(h.com_packets().len(), 1, "the same record ran again");
        assert_eq!(h.comp.run_mode(), RunMode::Running);
    }

    #[test]
    fn cancel_without_a_running_sequence_still_answers_ok() {
        let h = build();
        h.command(CmdSequencer::OPCODE_CS_CANCEL, 53, &[]);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_CANCEL,
                53,
                CmdResponse::Ok
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE)
        );
        assert_eq!(
            h.ground.tlm_u32(CmdSequencer::CHANID_CS_CANCEL_COMMANDS),
            None
        );
    }

    #[test]
    fn seq_cancel_in_port_cancels_without_a_command_response() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x61));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            54,
            &run_args(&path, BlockState::NoBlock),
        );
        h.ground.clear();
        h.seq_cancel_in();
        assert!(h.responses().is_empty());
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_SEQUENCE_CANCELED)
        );
        assert_eq!(h.comp.run_mode(), RunMode::Stopped);
    }

    // -- Mode guards -------------------------------------------------------

    #[test]
    fn run_while_running_is_invalid_mode() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x70));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            60,
            &run_args(&path, BlockState::NoBlock),
        );
        h.ground.clear();
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            61,
            &run_args(&path, BlockState::NoBlock),
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_RUN,
                61,
                CmdResponse::ExecutionError
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_INVALID_MODE)
        );
        assert_eq!(
            h.ground
                .count_event(CmdSequencer::EVENTID_CS_JOIN_WAITING_NOT_COMPLETE),
            0
        );
    }

    #[test]
    fn run_while_join_waiting_also_reports_join_waiting_not_complete() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x71));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            62,
            &run_args(&path, BlockState::NoBlock),
        );
        h.command(CmdSequencer::OPCODE_CS_JOIN_WAIT, 63, &[]);
        h.ground.clear();
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            64,
            &run_args(&path, BlockState::NoBlock),
        );
        let ids = h.ground.event_ids();
        assert!(ids.contains(&CmdSequencer::EVENTID_CS_INVALID_MODE));
        assert!(ids.contains(&CmdSequencer::EVENTID_CS_JOIN_WAITING_NOT_COMPLETE));
    }

    #[test]
    fn block_run_in_manual_mode_is_rejected() {
        let h = build();
        h.command(CmdSequencer::OPCODE_CS_MANUAL, 65, &[]);
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x72));
        h.ground.clear();
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            66,
            &run_args(&path, BlockState::Block),
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_RUN,
                66,
                CmdResponse::ExecutionError
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_INVALID_MODE)
        );
        // Nothing was loaded.
        assert_eq!(
            h.ground
                .count_event(CmdSequencer::EVENTID_CS_SEQUENCE_LOADED),
            0
        );
    }

    #[test]
    fn validate_start_step_auto_manual_are_mode_guarded() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x73));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            70,
            &run_args(&path, BlockState::NoBlock),
        );
        assert_eq!(h.comp.run_mode(), RunMode::Running);

        for (offset, seq) in [
            (CmdSequencer::OPCODE_CS_VALIDATE, 71u32),
            (CmdSequencer::OPCODE_CS_START, 72),
            (CmdSequencer::OPCODE_CS_AUTO, 73),
            (CmdSequencer::OPCODE_CS_MANUAL, 74),
        ] {
            h.ground.clear();
            let args: Vec<u8> = if offset == CmdSequencer::OPCODE_CS_VALIDATE {
                validate_args(&path)
            } else {
                Vec::new()
            };
            h.command(offset, seq, &args);
            assert_eq!(
                h.responses(),
                vec![(ID_BASE + offset, seq, CmdResponse::ExecutionError)],
                "opcode {offset}"
            );
            assert!(
                h.ground
                    .event_ids()
                    .contains(&CmdSequencer::EVENTID_CS_INVALID_MODE),
                "opcode {offset}"
            );
        }

        // CS_STEP in AUTO while RUNNING is also InvalidMode.
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_STEP, 75, &[]);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_STEP,
                75,
                CmdResponse::ExecutionError
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_INVALID_MODE)
        );
    }

    #[test]
    fn step_while_stopped_is_invalid_mode() {
        let h = build();
        h.command(CmdSequencer::OPCODE_CS_STEP, 76, &[]);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_STEP,
                76,
                CmdResponse::ExecutionError
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_INVALID_MODE)
        );
    }

    #[test]
    fn start_without_a_loaded_sequence_is_no_sequence_active() {
        let h = build();
        h.command(CmdSequencer::OPCODE_CS_START, 77, &[]);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_START,
                77,
                CmdResponse::ExecutionError
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE)
        );
    }

    #[test]
    fn mode_switch_events_use_the_inverted_seq_mode_enum() {
        let h = build();
        h.command(CmdSequencer::OPCODE_CS_MANUAL, 80, &[]);
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_MODE_SWITCHED)
            .expect("CS_ModeSwitched");
        assert_eq!(event.1, LogSeverity::ActivityHi);
        assert_eq!(event.2, vec![SeqMode::Step.as_repr()]);
        assert_eq!(event.2, vec![0u8], "CS_MANUAL logs SeqMode::STEP = 0");
        assert_eq!(h.comp.step_mode(), StepMode::Manual);

        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_AUTO, 81, &[]);
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_MODE_SWITCHED)
            .expect("CS_ModeSwitched");
        assert_eq!(event.2, vec![1u8], "CS_AUTO logs SeqMode::AUTO = 1");
        assert_eq!(h.comp.step_mode(), StepMode::Auto);
    }

    // -- CS_JOIN_WAIT ------------------------------------------------------

    #[test]
    fn join_wait_without_a_running_sequence_answers_ok() {
        let h = build();
        h.command(CmdSequencer::OPCODE_CS_JOIN_WAIT, 90, &[]);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_JOIN_WAIT,
                90,
                CmdResponse::Ok
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE)
        );
    }

    #[test]
    fn join_wait_defers_and_logs_the_previous_command_identity() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x81));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            91,
            &run_args(&path, BlockState::NoBlock),
        );
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_JOIN_WAIT, 92, &[]);
        assert!(h.responses().is_empty(), "the join response is deferred");
        let event = h
            .ground
            .find_event(CmdSequencer::EVENTID_CS_JOIN_WAITING)
            .expect("CS_JoinWaiting");
        let name_len = u16::from_be_bytes([event.2[0], event.2[1]]) as usize;
        let tail = &event.2[2 + name_len..];
        // The PREVIOUS cmdSeq/opCode (the CS_RUN), not the join's own.
        assert_eq!(u32::from_be_bytes([tail[0], tail[1], tail[2], tail[3]]), 91);
        assert_eq!(
            u32::from_be_bytes([tail[4], tail[5], tail[6], tail[7]]),
            get_event_opcode(ID_BASE + CmdSequencer::OPCODE_CS_RUN)
        );
        // Completion answers the join.
        h.cmd_response_in(0x81, 0, CmdResponse::Ok);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_JOIN_WAIT,
                92,
                CmdResponse::Ok
            )]
        );
    }

    #[test]
    fn join_wait_rejects_a_second_deferred_caller() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x82));
        // BLOCK-mode CS_RUN already owns the deferred slot.
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            93,
            &run_args(&path, BlockState::Block),
        );
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_JOIN_WAIT, 94, &[]);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_JOIN_WAIT,
                94,
                CmdResponse::ExecutionError
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_JOIN_WAITING_NOT_COMPLETE)
        );

        // A second CS_JOIN_WAIT is refused too.
        let h2 = build();
        let path2 = write_file(&dir, "s2.bin", &simple_sequence(0x83));
        h2.command(
            CmdSequencer::OPCODE_CS_RUN,
            95,
            &run_args(&path2, BlockState::NoBlock),
        );
        h2.command(CmdSequencer::OPCODE_CS_JOIN_WAIT, 96, &[]);
        h2.ground.clear();
        h2.command(CmdSequencer::OPCODE_CS_JOIN_WAIT, 97, &[]);
        assert_eq!(
            h2.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_JOIN_WAIT,
                97,
                CmdResponse::ExecutionError
            )]
        );
    }

    // -- Port-driven runs --------------------------------------------------

    #[test]
    fn seq_run_in_starts_the_named_sequence_and_ignores_its_args() {
        let h = build();
        let dir = temp_dir();
        let packet = command_packet(0x90, &[]);
        let mut records = command_record(RecordDescriptor::Relative as u8, 0, 0, &packet);
        records.extend_from_slice(&eos_record());
        let file = sequence_file(2, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);
        h.seq_run_in(&path);
        assert_eq!(h.com_packets(), vec![packet]);
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_PORT_SEQUENCE_STARTED)
        );
        // No command response: the port path uses seqDone instead.
        assert!(h.responses().is_empty());
        assert_eq!(h.comp.run_mode(), RunMode::Running);
    }

    #[test]
    fn seq_dispatch_in_starts_the_named_sequence() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x91));
        h.seq_dispatch_in(&path);
        assert_eq!(h.com_packets().len(), 1);
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_PORT_SEQUENCE_STARTED)
        );
    }

    #[test]
    fn port_run_in_manual_mode_reports_invalid_mode_on_seq_done() {
        let h = build();
        h.command(CmdSequencer::OPCODE_CS_MANUAL, 100, &[]);
        h.ground.clear();
        h.seq_run_in("/tmp/whatever");
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_INVALID_MODE)
        );
        assert_eq!(
            *h.seq_done.calls.lock().unwrap(),
            vec![(0, 0, CmdResponse::ExecutionError)]
        );
    }

    #[test]
    fn port_run_with_empty_name_and_no_loaded_sequence_errors() {
        let h = build();
        h.seq_run_in("");
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE)
        );
        assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
        assert_eq!(
            *h.seq_done.calls.lock().unwrap(),
            vec![(0, 0, CmdResponse::ExecutionError)]
        );
    }

    #[test]
    fn port_run_with_empty_name_reruns_the_loaded_sequence() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x92));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            101,
            &run_args(&path, BlockState::NoBlock),
        );
        h.command(CmdSequencer::OPCODE_CS_CANCEL, 102, &[]);
        h.ground.clear();
        h.seq_run_in("");
        assert_eq!(h.com_packets().len(), 1, "the loaded sequence restarted");
        assert_eq!(
            h.ground
                .count_event(CmdSequencer::EVENTID_CS_SEQUENCE_LOADED),
            0,
            "an empty name must not reload"
        );
    }

    #[test]
    fn port_run_load_failure_reports_on_seq_done() {
        let h = build();
        h.seq_dispatch_in("/x/missing.bin");
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_FILE_NOT_FOUND)
        );
        assert_eq!(
            *h.seq_done.calls.lock().unwrap(),
            vec![(0, 0, CmdResponse::ExecutionError)]
        );
    }

    #[test]
    #[should_panic(expected = "Assert")]
    fn port_run_error_path_asserts_when_seq_done_is_unconnected() {
        // C++ parity: `doSequenceRun`'s error paths call `seqDone_out`
        // WITHOUT an is-connected guard, unlike cancel/complete.
        let h = build_with(false, true);
        h.seq_run_in("/x/missing.bin");
    }

    #[test]
    fn cancel_and_complete_tolerate_an_unconnected_seq_done() {
        let h = build_with(false, false);
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0x93));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            103,
            &run_args(&path, BlockState::NoBlock),
        );
        h.cmd_response_in(0x93, 0, CmdResponse::Ok);
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_SEQUENCE_COMPLETE)
        );
        // seqStartOut is likewise guarded.
        assert!(h.ground.seq_start.lock().unwrap().is_empty());
    }

    // -- Ping / telemetry / commands ---------------------------------------

    #[test]
    fn ping_in_echoes_the_key_on_ping_out() {
        let h = build();
        let p = h.comp.ping_in(0);
        p.target.invoke(p.port_num, 0xFEED_BEEF);
        h.drain();
        assert_eq!(*h.ground.pings.lock().unwrap(), vec![0xFEED_BEEF]);
    }

    #[test]
    fn reg_commands_registers_all_eight_opcodes() {
        let h = build();
        assert_eq!(
            *h.ground.regs.lock().unwrap(),
            vec![
                ID_BASE,
                ID_BASE + 1,
                ID_BASE + 2,
                ID_BASE + 3,
                ID_BASE + 4,
                ID_BASE + 5,
                ID_BASE + 6,
                ID_BASE + 7,
            ]
        );
    }

    #[test]
    fn current_sequence_telemetry_is_update_on_change() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0xA0));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            110,
            &run_args(&path, BlockState::NoBlock),
        );
        let writes: Vec<Vec<u8>> = h
            .ground
            .tlm
            .lock()
            .unwrap()
            .iter()
            .filter(|t| t.0 == CmdSequencer::CHANID_CS_CURRENT_SEQUENCE)
            .map(|t| t.1.clone())
            .collect();
        assert_eq!(writes.len(), 1);
        assert_eq!(&writes[0][..2], &(path.len() as u16).to_be_bytes());
        assert_eq!(&writes[0][2..], path.as_bytes());

        // Cancel then restart the SAME name: no second write.
        h.command(CmdSequencer::OPCODE_CS_CANCEL, 111, &[]);
        h.command(CmdSequencer::OPCODE_CS_START, 112, &[]);
        let count = h
            .ground
            .tlm
            .lock()
            .unwrap()
            .iter()
            .filter(|t| t.0 == CmdSequencer::CHANID_CS_CURRENT_SEQUENCE)
            .count();
        assert_eq!(count, 1, "unchanged value is not rewritten");
    }

    #[test]
    fn unknown_opcode_answers_invalid_opcode() {
        let h = build();
        h.command(0x20, 120, &[]);
        assert_eq!(
            h.responses(),
            vec![(ID_BASE + 0x20, 120, CmdResponse::InvalidOpcode)]
        );
    }

    #[test]
    fn short_and_residual_command_arguments_are_format_errors() {
        let h = build();
        // CS_RUN missing the block byte.
        h.command(CmdSequencer::OPCODE_CS_RUN, 121, &validate_args("/tmp/x"));
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_RUN,
                121,
                CmdResponse::FormatError
            )]
        );

        // CS_VALIDATE with a trailing byte.
        h.ground.clear();
        let mut args = validate_args("/tmp/x");
        args.push(0x00);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 122, &args);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_VALIDATE,
                122,
                CmdResponse::FormatError
            )]
        );

        // A no-arg command with residual bytes.
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_CANCEL, 123, &[0xFF]);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_CANCEL,
                123,
                CmdResponse::FormatError
            )]
        );
    }

    #[test]
    fn invalid_block_state_enum_answers_validation_error() {
        let h = build();
        let mut args = validate_args("/tmp/x");
        args.push(0x07); // not BLOCK(0) or NO_BLOCK(1)
        h.command(CmdSequencer::OPCODE_CS_RUN, 124, &args);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_RUN,
                124,
                CmdResponse::ValidationError
            )]
        );
    }

    #[test]
    fn command_file_names_longer_than_forty_bytes_are_format_errors() {
        // FPP declares `string size FileNameStringSize` but the generated
        // handler receives an `Fw::CmdStringArg` (40), so a longer name
        // fails to deserialize.
        let h = build();
        let long_name = "/".repeat(45);
        h.command(
            CmdSequencer::OPCODE_CS_VALIDATE,
            125,
            &validate_args(&long_name),
        );
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_VALIDATE,
                125,
                CmdResponse::FormatError
            )]
        );
    }

    // -- Queue envelope ----------------------------------------------------

    #[test]
    fn queue_message_size_fits_a_full_command_argument_buffer() {
        assert_eq!(QUEUE_MSG_SIZE, 6 + 4 + 4 + 2 + 506);
        const { assert!(QUEUE_MSG_SIZE >= 6 + 2 + 240 + 8 + SEQUENCE_ARGUMENTS_MAX_SIZE) };
        assert_eq!(SeqArgs::SERIALIZED_SIZE, 8 + SEQUENCE_ARGUMENTS_MAX_SIZE);
    }

    #[test]
    fn seq_run_in_envelope_round_trips_the_file_name_and_args() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0xA1));
        let mut name = FileNameString::new();
        name.set(&path);
        let mut args = SeqArgs {
            size: 3,
            ..Default::default()
        };
        args.buffer.0[0] = 0xDE;
        let p = h.comp.seq_run_in(0);
        p.target.invoke(p.port_num, &name, &args);
        // One message queued; dispatch it.
        assert_eq!(h.comp.active.queued.queue().get_messages_available(), 1);
        h.drain();
        // The handler ignores `args` but the sequence still started.
        assert_eq!(h.com_packets().len(), 1);
    }

    // -- Setup API ---------------------------------------------------------

    #[test]
    fn load_sequence_preloads_at_topology_time() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0xA2));
        h.comp.load_sequence(&cmd_string(&path));
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_SEQUENCE_LOADED)
        );
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_START, 130, &[]);
        assert_eq!(h.com_packets().len(), 1);
    }

    #[test]
    fn a_failed_load_clears_the_sequence_so_start_cannot_run_it() {
        let h = build();
        let dir = temp_dir();
        // Valid records but a broken CRC: the buffer is populated before the
        // CRC check fails, so without the clear() `has_more_records()` would
        // still be true.
        let mut file = simple_sequence(0xA3);
        let last = file.len() - 1;
        file[last] ^= 0xFF;
        let path = write_file(&dir, "s.bin", &file);
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 131, &validate_args(&path));
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_START, 132, &[]);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_START,
                132,
                CmdResponse::ExecutionError
            )]
        );
        assert!(
            h.ground
                .event_ids()
                .contains(&CmdSequencer::EVENTID_CS_NO_SEQUENCE_ACTIVE)
        );
    }

    #[test]
    #[should_panic(expected = "Assert")]
    fn allocate_buffer_rejects_a_buffer_smaller_than_the_header() {
        let h = build();
        h.comp.allocate_buffer(0, SEQUENCE_HEADER_SIZE - 1);
    }

    #[test]
    #[should_panic(expected = "Assert")]
    fn loading_without_a_buffer_asserts() {
        let h = build();
        h.comp.deallocate_buffer();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0xA4));
        h.comp.load_sequence(&cmd_string(&path));
    }

    #[test]
    fn set_sequence_format_swaps_the_reader() {
        let h = build();
        let mut replacement = FPrimeSequence::new();
        replacement.allocate_buffer(7, 256);
        assert_eq!(replacement.allocator_id(), 7);
        h.comp.set_sequence_format(Box::new(replacement));
        assert_eq!(h.comp.state.lock().unwrap().sequence.capacity(), 256);
    }

    #[test]
    fn deallocate_buffer_releases_the_storage() {
        let h = build();
        assert_eq!(
            h.comp.state.lock().unwrap().sequence.capacity(),
            BUFFER_BYTES
        );
        h.comp.deallocate_buffer();
        assert_eq!(h.comp.state.lock().unwrap().sequence.capacity(), 0);
    }

    // -- Load counters -----------------------------------------------------

    #[test]
    fn load_command_counter_increments_on_every_successful_load() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0xA5));
        for (n, seq) in [(1u32, 140u32), (2, 141), (3, 142)] {
            h.command(CmdSequencer::OPCODE_CS_VALIDATE, seq, &validate_args(&path));
            assert_eq!(
                h.ground.tlm_u32(CmdSequencer::CHANID_CS_LOAD_COMMANDS),
                Some(n)
            );
        }
    }

    // -- Types on the wire -------------------------------------------------

    #[test]
    fn seq_args_serializes_as_u64_size_then_255_raw_bytes() {
        let mut args = SeqArgs {
            size: 2,
            ..Default::default()
        };
        args.buffer.0[0] = 0xAB;
        args.buffer.0[1] = 0xCD;
        let mut buf = LogBuffer::new();
        assert!(args.serialize_to(&mut buf, Endianness::Big).is_ok());
        let bytes = buf.as_slice();
        assert_eq!(bytes.len(), 8 + SEQUENCE_ARGUMENTS_MAX_SIZE);
        assert_eq!(&bytes[..8], &[0, 0, 0, 0, 0, 0, 0, 2]);
        assert_eq!(&bytes[8..10], &[0xAB, 0xCD]);
        assert!(bytes[10..].iter().all(|b| *b == 0));
    }

    #[test]
    fn enum_representations_match_the_fpp_model() {
        assert_eq!(BlockState::Block.as_repr(), 0);
        assert_eq!(BlockState::NoBlock.as_repr(), 1);
        assert_eq!(SeqMode::Step.as_repr(), 0);
        assert_eq!(SeqMode::Auto.as_repr(), 1);
        assert_eq!(FileReadStage::ReadHeader.as_repr(), 0);
        assert_eq!(FileReadStage::ReadSeqDataSize.as_repr(), 8);
        assert_eq!(RecordDescriptor::Absolute as u8, 0);
        assert_eq!(RecordDescriptor::Relative as u8, 1);
        assert_eq!(RecordDescriptor::EndOfSequence as u8, 2);
        assert_eq!(RunMode::default(), RunMode::Stopped);
        assert_eq!(StepMode::default(), StepMode::Auto);
        assert_eq!(RunMode::Running as i32, 1);
        assert_eq!(StepMode::Manual as i32, 1);
    }

    #[test]
    fn default_header_carries_the_dont_care_sentinels() {
        let header = SequenceHeader::default();
        assert_eq!(header.time_base, TimeBase::TbDontCare);
        assert_eq!(header.time_context, FW_CONTEXT_DONT_CARE);
        assert_eq!(TimeBase::TbDontCare as u16, 0xFFFF);
        assert_eq!(FW_CONTEXT_DONT_CARE, 0xFF);
    }

    // -- FPrimeSequence directly -------------------------------------------

    #[test]
    fn end_of_sequence_record_consumes_exactly_one_byte() {
        let dir = temp_dir();
        let mut records = eos_record();
        records.extend_from_slice(&command_record(
            RecordDescriptor::Relative as u8,
            0,
            0,
            &command_packet(0xB0, &[]),
        ));
        let file = sequence_file(2, TimeBase::TbWorkstationTime as u16, 0, &records);
        let path = write_file(&dir, "s.bin", &file);

        let mut sequence = FPrimeSequence::new();
        sequence.allocate_buffer(0, 512);
        let now = Time::new(TimeBase::TbWorkstationTime, 0, 5, 0);
        let mut event = None;
        assert!(sequence.load_file(&cmd_string(&path), &now, &mut event));
        assert!(event.is_none());

        let mut record = SequenceRecord::default();
        sequence.next_record(&mut record);
        assert_eq!(record.descriptor, RecordDescriptor::EndOfSequence);
        // The command record after it is still readable.
        sequence.next_record(&mut record);
        assert_eq!(record.descriptor, RecordDescriptor::Relative);
        assert_eq!(record.command.as_slice(), command_packet(0xB0, &[]));
        assert!(!sequence.has_more_records());
        // reset() rewinds, clear() drops the data.
        sequence.reset();
        assert!(sequence.has_more_records());
        sequence.clear();
        assert!(!sequence.has_more_records());
    }

    #[test]
    fn record_time_tags_carry_no_base_or_context_on_the_wire() {
        let dir = temp_dir();
        let records = command_record(
            RecordDescriptor::Absolute as u8,
            0x0102_0304,
            0x0005_0607,
            &command_packet(0xB1, &[]),
        );
        let file = sequence_file(1, TimeBase::TbWorkstationTime as u16, 0, &records);
        // 13 bytes of fixed overhead per command record.
        assert_eq!(records.len(), 13 + 6);
        let path = write_file(&dir, "s.bin", &file);

        let mut sequence = FPrimeSequence::new();
        sequence.allocate_buffer(0, 512);
        let now = Time::new(TimeBase::TbWorkstationTime, 0, 5, 0);
        let mut event = None;
        assert!(sequence.load_file(&cmd_string(&path), &now, &mut event));
        let mut record = SequenceRecord::default();
        sequence.next_record(&mut record);
        assert_eq!(record.time_tag.get_seconds(), 0x0102_0304);
        assert_eq!(record.time_tag.get_useconds(), 0x0005_0607);
        // Untouched by the reader: TbNone / context 0 until the component
        // stamps the header values in perform_cmd_step.
        assert_eq!(record.time_tag.get_time_base(), TimeBase::TbNone);
        assert_eq!(record.time_tag.get_context(), 0);
    }

    #[test]
    #[should_panic(expected = "Assert")]
    fn next_record_asserts_on_an_empty_buffer() {
        let mut sequence = FPrimeSequence::new();
        sequence.allocate_buffer(0, 64);
        let mut record = SequenceRecord::default();
        sequence.next_record(&mut record);
    }

    #[test]
    fn opening_a_directory_is_a_file_read_error() {
        let h = build();
        let dir = temp_dir();
        let path = dir.to_str().unwrap().to_string();
        h.command(CmdSequencer::OPCODE_CS_VALIDATE, 150, &validate_args(&path));
        let ids = h.ground.event_ids();
        assert!(
            ids.contains(&CmdSequencer::EVENTID_CS_FILE_READ_ERROR)
                || ids.contains(&CmdSequencer::EVENTID_CS_FILE_INVALID),
            "expected a read failure, got {ids:?}"
        );
        assert_eq!(h.ground.tlm_u32(CmdSequencer::CHANID_CS_ERRORS), Some(1));
    }

    #[test]
    fn start_emits_seq_start_out_after_the_first_step() {
        let h = build();
        h.command(CmdSequencer::OPCODE_CS_MANUAL, 151, &[]);
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0xB2));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            152,
            &run_args(&path, BlockState::NoBlock),
        );
        h.ground.clear();
        h.command(CmdSequencer::OPCODE_CS_START, 153, &[]);
        let starts = h.ground.seq_start.lock().unwrap().clone();
        assert_eq!(starts.len(), 1);
        assert_eq!(starts[0].0, path.as_bytes());
    }

    #[test]
    fn command_error_answers_a_deferred_join_wait_caller() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0xB3));
        h.command(
            CmdSequencer::OPCODE_CS_RUN,
            154,
            &run_args(&path, BlockState::NoBlock),
        );
        h.command(CmdSequencer::OPCODE_CS_JOIN_WAIT, 155, &[]);
        h.ground.clear();
        h.cmd_response_in(0xB3, 0, CmdResponse::ExecutionError);
        assert_eq!(
            h.responses(),
            vec![(
                ID_BASE + CmdSequencer::OPCODE_CS_JOIN_WAIT,
                155,
                CmdResponse::ExecutionError
            )]
        );
        // The slot is released: a new join is accepted next time.
        assert!(!h.comp.state.lock().unwrap().join_waiting);
    }

    // -- Active lifecycle --------------------------------------------------

    #[test]
    fn component_runs_on_its_own_task() {
        let h = build();
        let dir = temp_dir();
        let path = write_file(&dir, "s.bin", &simple_sequence(0xB4));
        h.comp.active.start(
            &h.comp,
            fprime_os::task::TASK_PRIORITY_DEFAULT,
            fprime_os::task::TASK_DEFAULT,
            fprime_os::task::TASK_DEFAULT,
        );

        let mut args = CmdArgBuffer::new();
        assert!(
            args.serialize_bytes(
                &run_args(&path, BlockState::NoBlock),
                LengthMode::OmitLength,
                Endianness::Big,
            )
            .is_ok()
        );
        let p = h.comp.cmd_in(0);
        p.target.invoke(
            p.port_num,
            ID_BASE + CmdSequencer::OPCODE_CS_RUN,
            160,
            &mut args,
        );

        // EXIT is priority 0, so the queued command drains first and join()
        // is a deterministic sync point.
        h.comp.active.exit();
        assert_eq!(h.comp.active.join(), fprime_os::task::Status::OpOk);

        assert_eq!(
            h.responses(),
            vec![(ID_BASE + CmdSequencer::OPCODE_CS_RUN, 160, CmdResponse::Ok)]
        );
        assert_eq!(h.com_packets().len(), 1);
    }
}
