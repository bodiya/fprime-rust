//! # AMPCSSequence — port of `Svc::AMPCSSequence`
//!
//! C++ sources: `Svc/CmdSequencer/formats/{AMPCSSequence.hpp,
//! AMPCSSequence.cpp,README.md}`; test builders in
//! `Svc/CmdSequencer/test/ut/SequenceFiles/AMPCS/`.
//! Analysis: `docs/cpp-analysis/cmd-sequencer.md`.
//!
//! The alternate sequence format produced by the AMPCS ground system. A
//! deployment selects it the way upstream's `setSequenceFormat` does —
//! BEFORE allocating the buffer (`formats/README.md`):
//!
//! ```ignore
//! cmd_seq.set_sequence_format(Box::new(AMPCSSequence::new()));
//! cmd_seq.allocate_buffer(0, 5 * 1024);
//! ```
//!
//! ## Files
//!
//! Two files per sequence, everything big-endian:
//!
//! - `<name>`: `[header 4][records]`. The four header bytes are opaque —
//!   they are read and covered by the CRC but never interpreted.
//! - `<name>.CRC32`: the sidecar, exactly the 4-byte big-endian CRC-32
//!   (`Utils::Hash`) of the WHOLE sequence file (header + records). Only
//!   the first four bytes are read.
//!
//! A record is
//! `[timeFlag u8][time u32][cmdLength u16][cmdLength bytes: opcode u16 + args]`
//! with `timeFlag` `0x00` = ABSOLUTE and `0xFF` = RELATIVE. The time is
//! whole seconds (useconds are always 0). There is no end-of-sequence
//! record and no record count: the sequence ends when the bytes do.
//!
//! The command field is rebuilt into an F Prime command com packet:
//! `[FW_PACKET_COMMAND as FwPacketDescriptorType][sizeof(FwOpcodeType) - 2
//! zero bytes][cmdLength bytes]`, i.e. the two-byte AMPCS opcode is
//! zero-extended to the four-byte `FwOpcodeType` and the arguments are
//! copied raw.
//!
//! ## C++ parity quirks reproduced (each tested)
//!
//! - Every load reads the sidecar FIRST, under the sidecar's name: a missing
//!   or unreadable `.CRC32` reports `CS_FileNotFound` / `CS_FileReadError`
//!   naming `<name>.CRC32`.
//! - The sidecar name is `Fw::CmdStringArg` (40 bytes) `+= ".CRC32"`, which
//!   truncates silently: a 40-byte sequence name makes the sequence file
//!   its own "sidecar".
//! - The sidecar read checks only the read STATUS, never the byte count: a
//!   short sidecar leaves the rest of the four CRC bytes as whatever the
//!   sequence buffer held before (zero on a fresh buffer).
//! - A sequence file shorter than the 4-byte header — or one that does not
//!   exist — is reported by `Os::FileSystem::getFileSize`'s path as
//!   `CS_FileInvalid(READ_HEADER_SIZE, <FileSystem::Status>)`, NOT as
//!   `CS_FileNotFound` (the error is an `Os::FileSystem::Status`
//!   discriminant, e.g. `DOESNT_EXIST` = 8, or `OP_OK` = 0 for a short file).
//! - The header is reset to `TB_DONT_CARE` / `FW_CONTEXT_DONT_CARE` on every
//!   load, so `validateTime` always passes and canonicalizes to the live
//!   time base/context.
//! - `validateRecords` has no `CS_NoRecords` / `CS_RecordMismatch` checks:
//!   it counts records until the bytes run out. A file holding only the
//!   header therefore LOADS (zero records), and running it drives
//!   `nextRecord` into its `FW_ASSERT` exactly as in C++.
//! - `SequenceHeader::file_size` is the record-block size (file size minus
//!   the 4 header bytes), not the F Prime "records + CRC" meaning.

use super::{
    FileReadStage, RecordDescriptor, Sequence, SequenceBase, SequenceHeader, SequenceLoadEvent,
    SequenceRecord,
};
use fprime_config::{
    FW_COM_BUFFER_MAX_SIZE, FW_CONTEXT_DONT_CARE, FwEnumStoreType, FwOpcodeType,
    FwPacketDescriptorType, FwSizeType,
};
use fprime_fw::{
    CmdStringArg, ComBuffer, ComPacketType, Endianness, ExtBuf, FwDefaultString, LengthMode,
    LogStringArg, SerBuf, SerBufAny, SerializeStatus, Time, TimeBase, fw_assert,
};
use fprime_os::file::{Mode as FileMode, Status as FileStatus, WaitType};
use fprime_os::filesystem;
use fprime_utils::Hash;

/// `AMPCSSequence::SequenceHeader::Constants::SIZE` — the opaque header.
pub const SEQUENCE_HEADER_SIZE: usize = 4;

/// The sidecar suffix. C++ appends the literal `".CRC32"` (it does not go
/// through `HASH_EXTENSION_STRING`, although the two agree).
pub const CRC_FILE_EXTENSION: &str = ".CRC32";

/// Size of the CRC held in the sidecar (`sizeof(U32)`).
pub const CRC_SIZE: usize = 4;

/// `AMPCSSequence::Record::Time::t` — whole seconds.
pub type RecordTime = u32;

/// `AMPCSSequence::Record::CmdLength::t`.
pub type CmdLength = u16;

/// `AMPCSSequence::Record::Opcode::t` — the two-byte AMPCS opcode.
pub type Opcode = u16;

/// `AMPCSSequence::Record::TimeFlag::t`; serialized as one byte
/// (`TimeFlag::Serial::t = U8`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TimeFlag {
    /// Absolute time tag (`ABSOLUTE = 0x00`).
    Absolute = 0x00,
    /// Relative time tag (`RELATIVE = 0xFF`).
    Relative = 0xFF,
}

/// Bytes the translation prepends to the command field:
/// the packet descriptor plus the opcode zero-extension.
const TRANSLATION_OVERHEAD: usize =
    size_of::<FwPacketDescriptorType>() + (size_of::<FwOpcodeType>() - size_of::<Opcode>());

// C++ `static_assert(sizeof(FwOpcodeType) >= 2, ...)`.
const _: () = assert!(size_of::<FwOpcodeType>() >= size_of::<Opcode>());

/// `<file_name>.CRC32` as C++ builds it: `Fw::CmdStringArg crcFileName
/// (fileName); crcFileName += ".CRC32";` — silently truncated at the
/// 40-byte `CmdStringArg` capacity.
#[must_use]
pub fn crc_file_name(file_name: &CmdStringArg) -> CmdStringArg {
    let mut crc_name = file_name.clone();
    crc_name.append(CRC_FILE_EXTENSION);
    crc_name
}

/// A sequence in AMPCS format (C++ `Svc::AMPCSSequence`).
///
/// Hand it to [`super::CmdSequencer::set_sequence_format`] before
/// [`super::CmdSequencer::allocate_buffer`].
pub struct AMPCSSequence {
    base: SequenceBase,
    /// C++ `m_sequenceHeader` — the opaque 4-byte header, kept outside the
    /// sequence buffer.
    sequence_header: [u8; SEQUENCE_HEADER_SIZE],
    /// C++ `m_crc.m_computed`.
    crc: Hash,
    /// C++ `m_crc.m_stored` — the value read from the sidecar.
    crc_stored: u32,
}

impl Default for AMPCSSequence {
    fn default() -> Self {
        Self::new()
    }
}

impl AMPCSSequence {
    /// A sequence with no buffer yet (C++ constructor).
    #[must_use]
    pub fn new() -> Self {
        Self {
            base: SequenceBase::new(),
            sequence_header: [0; SEQUENCE_HEADER_SIZE],
            crc: Hash::new(),
            crc_stored: 0,
        }
    }

    /// The allocator identifier passed to [`Sequence::allocate_buffer`].
    #[must_use]
    pub fn allocator_id(&self) -> FwEnumStoreType {
        self.base.allocator_id
    }

    /// The CRC read from the sidecar by the last load attempt.
    #[must_use]
    pub fn stored_crc(&self) -> u32 {
        self.crc_stored
    }

    /// The opaque 4-byte header of the last sequence file read.
    #[must_use]
    pub fn sequence_header(&self) -> [u8; SEQUENCE_HEADER_SIZE] {
        self.sequence_header
    }

    /// C++ `readCRCFile(crcFileName)`.
    fn read_crc_file(
        &mut self,
        crc_file_name: &CmdStringArg,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        self.base.set_file_name(crc_file_name);
        // A name that is not valid UTF-8 cannot be opened through the Rust
        // OSAL; C++ would hand the raw bytes to open(2) and fail there (the
        // same deviation as `FPrimeSequence`).
        let Some(path) = crc_file_name.as_str() else {
            *event = Some(SequenceLoadEvent::FileReadError);
            return false;
        };
        let mut file = fprime_os::File::new();
        let result = match file.open(path, FileMode::OpenRead) {
            FileStatus::OpOk => self.read_crc(&mut file, event) && self.deserialize_crc(event),
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

    /// C++ `readCRC()` — reads the sidecar INTO THE SEQUENCE BUFFER.
    fn read_crc(
        &mut self,
        file: &mut fprime_os::File,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        // C++ `setBuffLen(sizeof(U32))` (asserts) BEFORE the read.
        self.base.set_buff_len(CRC_SIZE);
        let mut read_len: FwSizeType = 0;
        let status = file.read(
            &mut self.base.storage[..CRC_SIZE],
            &mut read_len,
            WaitType::Wait,
        );
        // C++ parity: only the status is checked. A short sidecar leaves
        // the unread CRC bytes as the buffer's previous contents.
        if status != FileStatus::OpOk {
            *event = Some(SequenceLoadEvent::FileInvalid {
                stage: FileReadStage::ReadSeqCrc,
                error: status as i32,
            });
            return false;
        }
        true
    }

    /// C++ `deserializeCRC()`: the stored CRC is a big-endian `U32`.
    fn deserialize_crc(&mut self, event: &mut Option<SequenceLoadEvent>) -> bool {
        let mut stored = 0u32;
        let status = self
            .base
            .with_deser(|buf| buf.deserialize_u32_be(&mut stored));
        if status != SerializeStatus::Ok {
            // Unreachable with a four-byte buffer length; ported for parity.
            *event = Some(SequenceLoadEvent::FileInvalid {
                stage: FileReadStage::ReadSeqCrc,
                error: status as i32,
            });
            return false;
        }
        self.crc_stored = stored;
        true
    }

    /// C++ `getFileSize(seqFileName)`: records size = file size - header.
    fn get_file_size(
        &mut self,
        seq_file_name: &CmdStringArg,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        self.base.set_file_name(seq_file_name);
        let mut file_size: FwSizeType = 0;
        let file_status = match seq_file_name.as_str() {
            Some(path) => filesystem::get_file_size(path, &mut file_size),
            // Unreachable: the sidecar name shares this prefix and failed
            // first. Reported as the closest FileSystem status.
            None => filesystem::Status::InvalidPath,
        };
        if file_status == filesystem::Status::OpOk
            && file_size >= SEQUENCE_HEADER_SIZE as FwSizeType
        {
            // C++ `static_cast<U32>(fileSize - sizeof(m_sequenceHeader))`.
            self.base.header.file_size = (file_size - SEQUENCE_HEADER_SIZE as FwSizeType) as u32;
            true
        } else {
            *event = Some(SequenceLoadEvent::FileInvalid {
                stage: FileReadStage::ReadHeaderSize,
                error: file_status as i32,
            });
            false
        }
    }

    /// C++ `readSequenceFile(seqFileName)`.
    fn read_sequence_file(
        &mut self,
        seq_file_name: &CmdStringArg,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        self.base.set_file_name(seq_file_name);
        let Some(path) = seq_file_name.as_str() else {
            *event = Some(SequenceLoadEvent::FileReadError);
            return false;
        };
        let mut file = fprime_os::File::new();
        let result = match file.open(path, FileMode::OpenRead) {
            FileStatus::OpOk => self.read_open_sequence_file(&mut file, event),
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

    /// C++ `readOpenSequenceFile()`: the CRC covers the header, then the
    /// record block — i.e. every byte of the file.
    fn read_open_sequence_file(
        &mut self,
        file: &mut fprime_os::File,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        self.base.clear();
        self.crc = Hash::new();
        self.crc.init();
        if !self.read_sequence_header(file, event) {
            return false;
        }
        self.crc.update(&self.sequence_header);
        if !self.read_records(file, event) {
            return false;
        }
        let buff_len = self.base.ser_loc;
        fw_assert!(
            buff_len == self.base.header.file_size as usize,
            buff_len as i32,
            self.base.header.file_size as i32
        );
        self.crc.update(&self.base.storage[..buff_len]);
        true
    }

    /// C++ `readSequenceHeader()`.
    fn read_sequence_header(
        &mut self,
        file: &mut fprime_os::File,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        let mut read_len: FwSizeType = 0;
        let status = file.read(&mut self.sequence_header, &mut read_len, WaitType::Wait);
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
        true
    }

    /// C++ `readRecords()`.
    fn read_records(
        &mut self,
        file: &mut fprime_os::File,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        let size = self.base.header.file_size as usize;
        if size > self.base.storage.len() {
            *event = Some(SequenceLoadEvent::FileSizeError { size: size as u32 });
            return false;
        }
        let mut read_len: FwSizeType = 0;
        let status = file.read(
            &mut self.base.storage[..size],
            &mut read_len,
            WaitType::Wait,
        );
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
        self.base.set_buff_len(size);
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

    /// C++ `validateRecords()`: deserialize and COUNT the records. There is
    /// no record-count field to check against, hence no `CS_NoRecords` /
    /// `CS_RecordMismatch` here.
    fn validate_records(&mut self, event: &mut Option<SequenceLoadEvent>) -> bool {
        let mut record = SequenceRecord::default();
        let loop_bound = self.base.deserialize_size_left() as u32;
        let mut num_records = 0u32;
        while num_records < loop_bound {
            if self.base.deserialize_size_left() == 0 {
                break;
            }
            let status = self.deserialize_record(&mut record);
            if status != SerializeStatus::Ok {
                *event = Some(SequenceLoadEvent::RecordInvalid {
                    record_number: num_records,
                    error: status as i32,
                });
                return false;
            }
            num_records += 1;
        }
        self.base.header.num_records = num_records;
        self.base.reset();
        true
    }

    /// C++ `deserializeRecord()`.
    fn deserialize_record(&mut self, record: &mut SequenceRecord) -> SerializeStatus {
        self.base.with_deser(|buf| {
            // Time flag -> descriptor.
            let mut time_flag = 0u8;
            let status = buf.deserialize_u8_be(&mut time_flag);
            if status != SerializeStatus::Ok {
                return status;
            }
            record.descriptor = match time_flag {
                f if f == TimeFlag::Absolute as u8 => RecordDescriptor::Absolute,
                f if f == TimeFlag::Relative as u8 => RecordDescriptor::Relative,
                _ => return SerializeStatus::DeserFormatError,
            };
            // Time: whole seconds; `Fw::Time::set(time, 0)` keeps the base
            // and context, which come from the header at step time.
            let mut time: RecordTime = 0;
            let status = buf.deserialize_u32_be(&mut time);
            if status != SerializeStatus::Ok {
                return status;
            }
            record.time_tag.set(time, 0);
            // Command length (C++ `deserializeCmdLength`).
            let mut cmd_length: CmdLength = 0;
            let status = buf.deserialize_u16_be(&mut cmd_length);
            if status != SerializeStatus::Ok {
                return status;
            }
            let cmd_length = cmd_length as usize;
            if cmd_length > buf.deserialize_size_left() {
                // Not enough data left.
                return SerializeStatus::DeserSizeMismatch;
            }
            // The translated packet must fit the ComBuffer capacity
            // (descriptor + opcode zero-extension + command field).
            if TRANSLATION_OVERHEAD + cmd_length > FW_COM_BUFFER_MAX_SIZE {
                return SerializeStatus::DeserSizeMismatch;
            }
            translate_command(&mut record.command, buf, cmd_length)
        })
    }
}

/// C++ `translateCommand(comBuffer, cmdLength)`: rebuild the AMPCS command
/// field as an F Prime command com packet.
fn translate_command(
    com_buffer: &mut ComBuffer,
    buf: &mut ExtBuf<'_>,
    cmd_length: usize,
) -> SerializeStatus {
    com_buffer.reset_ser();
    // Command packet descriptor.
    let descriptor = ComPacketType::FwPacketCommand as FwPacketDescriptorType;
    let status = com_buffer.serialize_u16(descriptor, Endianness::Big);
    fw_assert!(status == SerializeStatus::Ok, status as i32);
    // Zero-extend the two-byte AMPCS opcode to `FwOpcodeType`.
    for _ in 0..size_of::<FwOpcodeType>() - size_of::<Opcode>() {
        let status = com_buffer.serialize_u8_be(0);
        fw_assert!(status == SerializeStatus::Ok, status as i32);
    }
    let fixed_buff_len = com_buffer.get_size();
    fw_assert!(
        fixed_buff_len == TRANSLATION_OVERHEAD,
        fixed_buff_len as i32
    );
    let total_buff_len = fixed_buff_len + cmd_length;
    let status = com_buffer.set_buff_len(total_buff_len);
    fw_assert!(status == SerializeStatus::Ok, status as i32);
    // Raw OMIT_LENGTH copy of the opcode and argument bytes.
    let mut size = cmd_length;
    buf.deserialize_bytes(
        &mut com_buffer.bytes_mut()[fixed_buff_len..total_buff_len],
        &mut size,
        LengthMode::OmitLength,
        Endianness::Big,
    )
}

impl Sequence for AMPCSSequence {
    fn allocate_buffer(&mut self, identifier: FwEnumStoreType, bytes: usize) {
        self.base.allocate_buffer(identifier, bytes);
    }

    fn deallocate_buffer(&mut self) {
        self.base.deallocate_buffer();
    }

    fn capacity(&self) -> usize {
        self.base.storage.len()
    }

    fn set_file_name(&mut self, file_name: &CmdStringArg) {
        self.base.set_file_name(file_name);
    }

    fn file_name(&self) -> &CmdStringArg {
        &self.base.file_name
    }

    fn log_file_name(&self) -> &LogStringArg {
        &self.base.log_file_name
    }

    fn string_file_name(&self) -> &FwDefaultString {
        &self.base.string_file_name
    }

    fn header(&self) -> &SequenceHeader {
        &self.base.header
    }

    /// C++ `AMPCSSequence::loadFile`. The events of the sidecar stage name
    /// the sidecar; every later stage names the sequence file.
    fn load_file(
        &mut self,
        file_name: &CmdStringArg,
        current_time: &Time,
        event: &mut Option<SequenceLoadEvent>,
    ) -> bool {
        // C++ FW_ASSERT(m_buffer.getBuffAddr() != nullptr).
        fw_assert!(!self.base.storage.is_empty());
        let crc_name = crc_file_name(file_name);
        self.base.header.time_base = TimeBase::TbDontCare;
        self.base.header.time_context = FW_CONTEXT_DONT_CARE;
        // Short-circuit chain, exactly as in C++.
        self.read_crc_file(&crc_name, event)
            && self.get_file_size(file_name, event)
            && self.read_sequence_file(file_name, event)
            && self.validate_crc(event)
            && self.base.validate_time(current_time, event)
            && self.validate_records(event)
    }

    fn has_more_records(&self) -> bool {
        self.base.deserialize_size_left() > 0
    }

    fn next_record(&mut self, record: &mut SequenceRecord) {
        let status = self.deserialize_record(record);
        fw_assert!(status == SerializeStatus::Ok, status as i32);
    }

    fn reset(&mut self) {
        self.base.reset();
    }

    fn clear(&mut self) {
        self.base.clear();
    }
}

/// Sequence-file builders shared by the format tests here and the
/// component tests (`cmd_sequencer/tests/ampcs.rs`). Ports of the upstream
/// `test/ut/SequenceFiles/AMPCS/{Headers,Records,CRCs}.cpp` helpers and of
/// the `serializeAMPCS` bodies of the `SequenceFiles::*File` classes.
#[cfg(test)]
pub(super) mod test_files {
    use super::*;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    /// A fresh temp directory with a SHORT path: the sequence name plus
    /// `.CRC32` must fit the 40-byte `Fw::CmdStringArg`.
    pub(crate) fn temp_dir() -> PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "fpam{}_{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&path).unwrap();
        path
    }

    /// `AMPCS::Headers::serialize(buffer)` — the opaque `0x11223344`.
    pub(crate) fn header() -> Vec<u8> {
        0x1122_3344u32.to_be_bytes().to_vec()
    }

    /// `AMPCS::Records::serialize(timeFlag, time, cmdField, dest)`:
    /// `[flag u8][time u32][cmdLength u16][cmd field]`.
    pub(crate) fn record(time_flag: u8, time: RecordTime, cmd_field: &[u8]) -> Vec<u8> {
        let mut out = vec![time_flag];
        out.extend_from_slice(&time.to_be_bytes());
        out.extend_from_slice(&(cmd_field.len() as CmdLength).to_be_bytes());
        out.extend_from_slice(cmd_field);
        out
    }

    /// `AMPCS::Records::serialize(timeFlag, time, opcode, argument, dest)`:
    /// the command field is `[opcode u16][argument u32]`.
    pub(crate) fn command_record(
        time_flag: u8,
        time: RecordTime,
        opcode: Opcode,
        argument: u32,
    ) -> Vec<u8> {
        let mut field = opcode.to_be_bytes().to_vec();
        field.extend_from_slice(&argument.to_be_bytes());
        record(time_flag, time, &field)
    }

    /// `CommandBuffers::create(comBuff, opcode, argument)` — the F Prime
    /// com packet the sequencer must emit: `[u16 0][opcode u32][arg u32]`.
    pub(crate) fn command_buffer(opcode: FwOpcodeType, argument: u32) -> Vec<u8> {
        let mut out = (ComPacketType::FwPacketCommand as FwPacketDescriptorType)
            .to_be_bytes()
            .to_vec();
        out.extend_from_slice(&opcode.to_be_bytes());
        out.extend_from_slice(&argument.to_be_bytes());
        out
    }

    /// Write `bytes` to `<dir>/<name>` and return the path.
    pub(crate) fn write_sequence(dir: &std::path::Path, name: &str, bytes: &[u8]) -> String {
        let mut path = dir.to_path_buf();
        path.push(name);
        std::fs::write(&path, bytes).unwrap();
        path.to_str().unwrap().to_string()
    }

    /// `AMPCS::CRCs::writeCRC(crc, fileName)`: the sidecar holds the CRC
    /// serialized BIG-endian (`Fw::SerialBuffer::serializeFrom(U32)`).
    pub(crate) fn write_crc(path: &str, crc: u32) {
        std::fs::write(format!("{path}{CRC_FILE_EXTENSION}"), crc.to_be_bytes()).unwrap();
    }

    /// `File::write()` + `AMPCS::CRCs::createFile`: the sequence file and a
    /// sidecar holding the CRC of every byte of it.
    pub(crate) fn write_with_crc(dir: &std::path::Path, name: &str, bytes: &[u8]) -> String {
        let path = write_sequence(dir, name, bytes);
        write_crc(&path, Hash::hash_u32(bytes));
        path
    }

    /// `ImmediateFile::serializeAMPCS`: `n` RELATIVE t+0 records, opcode
    /// `i`, argument `i + 1`.
    pub(crate) fn immediate_file(n: u32) -> Vec<u8> {
        let mut out = header();
        for i in 0..n {
            out.extend(command_record(
                TimeFlag::Relative as u8,
                0,
                i as Opcode,
                i + 1,
            ));
        }
        out
    }

    /// `RelativeFile::serializeAMPCS`: `n` RELATIVE t+2 s records.
    pub(crate) fn relative_file(n: u32) -> Vec<u8> {
        let mut out = header();
        for i in 0..n {
            out.extend(command_record(
                TimeFlag::Relative as u8,
                2,
                i as Opcode,
                i + 1,
            ));
        }
        out
    }

    /// `MixedFile::serializeAMPCS`: absolute t=2 (op 0), immediate (op 2),
    /// relative t+1 (op 4), immediate (op 6).
    pub(crate) fn mixed_file() -> Vec<u8> {
        let mut out = header();
        out.extend(command_record(TimeFlag::Absolute as u8, 2, 0, 1));
        out.extend(command_record(TimeFlag::Relative as u8, 0, 2, 3));
        out.extend(command_record(TimeFlag::Relative as u8, 1, 4, 5));
        out.extend(command_record(TimeFlag::Relative as u8, 0, 6, 7));
        out
    }

    /// `BadDescriptorFile::serializeAMPCS`: time flag 10.
    pub(crate) fn bad_descriptor_file(n: u32) -> Vec<u8> {
        let mut out = header();
        for i in 0..n {
            out.extend(command_record(10, 0, i as Opcode, i + 1));
        }
        out
    }

    /// `BadCRCFile::serializeAMPCS`: header + `U32 0x10`; the sidecar holds
    /// `computed + 1`. Returns `(path, stored, computed)`.
    pub(crate) fn write_bad_crc_file(dir: &std::path::Path, name: &str) -> (String, u32, u32) {
        let mut bytes = header();
        bytes.extend_from_slice(&0x10u32.to_be_bytes());
        let path = write_sequence(dir, name, &bytes);
        let computed = Hash::hash_u32(&bytes);
        let stored = computed.wrapping_add(1);
        write_crc(&path, stored);
        (path, stored, computed)
    }

    /// `TooLargeFile::serializeAMPCS`: one RELATIVE record whose zeroed
    /// command field makes the record block exactly `2 * buffer_size`.
    pub(crate) fn too_large_file(buffer_size: usize) -> Vec<u8> {
        let data_size = 2 * buffer_size;
        let cmd_field_size = data_size - 1 - size_of::<RecordTime>() - size_of::<CmdLength>();
        let mut out = header();
        out.extend(record(
            TimeFlag::Relative as u8,
            0,
            &vec![0u8; cmd_field_size],
        ));
        assert_eq!(out.len(), SEQUENCE_HEADER_SIZE + data_size);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::test_files::*;
    use super::*;
    use fprime_config::FW_CMD_STRING_MAX_SIZE;

    /// Upstream tester allocation (`CmdSequencerTester.hpp` `BUFFER_SIZE`).
    const BUFFER_SIZE: usize = 1024;

    fn cmd_string(value: &str) -> CmdStringArg {
        let mut out = CmdStringArg::new();
        out.set(value);
        out
    }

    fn sequence() -> AMPCSSequence {
        let mut seq = AMPCSSequence::new();
        seq.allocate_buffer(100, BUFFER_SIZE);
        seq
    }

    fn live_time() -> Time {
        Time::new(TimeBase::TbWorkstationTime, 7, 1, 1)
    }

    fn load(seq: &mut AMPCSSequence, path: &str) -> (bool, Option<SequenceLoadEvent>) {
        let mut event = None;
        let ok = seq.load_file(&cmd_string(path), &live_time(), &mut event);
        (ok, event)
    }

    fn next(seq: &mut AMPCSSequence) -> SequenceRecord {
        let mut record = SequenceRecord::default();
        seq.next_record(&mut record);
        record
    }

    // -- Wire format: literal bytes ----------------------------------------

    #[test]
    fn record_layout_is_flag_seconds_length_then_command_field() {
        let bytes = command_record(TimeFlag::Relative as u8, 0x0102_0304, 0xABCD, 0x1122_3344);
        assert_eq!(
            bytes,
            [
                0xFF, // time flag RELATIVE
                0x01, 0x02, 0x03, 0x04, // time (seconds)
                0x00, 0x06, // cmdLength
                0xAB, 0xCD, // AMPCS opcode u16
                0x11, 0x22, 0x33, 0x44, // argument
            ]
        );
        assert_eq!(TimeFlag::Absolute as u8, 0x00);
        assert_eq!(TimeFlag::Relative as u8, 0xFF);
    }

    #[test]
    fn translation_zero_extends_the_opcode_behind_the_command_descriptor() {
        let dir = temp_dir();
        let mut bytes = header();
        bytes.extend(command_record(
            TimeFlag::Absolute as u8,
            9,
            0xABCD,
            0x0102_0304,
        ));
        let path = write_with_crc(&dir, "a.bin", &bytes);
        let mut seq = sequence();
        assert_eq!(load(&mut seq, &path), (true, None));
        let record = next(&mut seq);
        assert_eq!(record.descriptor, RecordDescriptor::Absolute);
        assert_eq!(
            record.command.as_slice(),
            [
                0x00, 0x00, // FW_PACKET_COMMAND
                0x00, 0x00, 0xAB, 0xCD, // opcode zero-extended to u32
                0x01, 0x02, 0x03, 0x04, // argument copied raw
            ]
        );
        assert_eq!(
            record.command.as_slice(),
            command_buffer(0xABCD, 0x0102_0304)
        );
        assert_eq!(
            (
                record.time_tag.get_seconds(),
                record.time_tag.get_useconds()
            ),
            (9, 0)
        );
        assert!(!seq.has_more_records());
    }

    #[test]
    fn zero_length_command_field_translates_to_the_bare_prefix() {
        let dir = temp_dir();
        let mut bytes = header();
        bytes.extend(record(TimeFlag::Relative as u8, 0, &[]));
        let path = write_with_crc(&dir, "z.bin", &bytes);
        let mut seq = sequence();
        assert_eq!(load(&mut seq, &path), (true, None));
        assert_eq!(seq.header().num_records, 1);
        assert_eq!(next(&mut seq).command.as_slice(), [0u8; 4]);
    }

    #[test]
    fn sidecar_crc_is_big_endian_over_the_whole_file() {
        let dir = temp_dir();
        let bytes = immediate_file(2);
        let path = write_with_crc(&dir, "c.bin", &bytes);
        let sidecar = std::fs::read(format!("{path}.CRC32")).unwrap();
        assert_eq!(sidecar, Hash::hash_u32(&bytes).to_be_bytes());
        let mut seq = sequence();
        assert_eq!(load(&mut seq, &path), (true, None));
        assert_eq!(seq.stored_crc(), Hash::hash_u32(&bytes));
        // A little-endian sidecar is a CRC failure unless
        // the CRC happens to be a byte palindrome.
        let crc = Hash::hash_u32(&bytes);
        std::fs::write(format!("{path}.CRC32"), crc.to_le_bytes()).unwrap();
        let (ok, event) = load(&mut seq, &path);
        assert!(!ok);
        assert_eq!(
            event,
            Some(SequenceLoadEvent::FileCrcFailure {
                stored: crc.swap_bytes(),
                computed: crc,
            })
        );
    }

    #[test]
    fn opaque_header_is_crc_covered_but_not_interpreted() {
        let dir = temp_dir();
        let mut bytes = immediate_file(1);
        bytes[..4].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
        let path = write_with_crc(&dir, "h.bin", &bytes);
        let mut seq = sequence();
        assert_eq!(load(&mut seq, &path), (true, None));
        assert_eq!(seq.sequence_header(), [0xDE, 0xAD, 0xBE, 0xEF]);
        // Flip a header byte without updating the sidecar: CRC failure.
        let mut corrupt = bytes.clone();
        corrupt[0] ^= 1;
        write_sequence(&dir, "h.bin", &corrupt);
        let (ok, event) = load(&mut seq, &path);
        assert!(!ok);
        assert!(matches!(
            event,
            Some(SequenceLoadEvent::FileCrcFailure { .. })
        ));
    }

    // -- Header semantics --------------------------------------------------

    #[test]
    fn header_counts_records_and_canonicalizes_to_the_live_time() {
        let dir = temp_dir();
        let bytes = immediate_file(5);
        let path = write_with_crc(&dir, "n.bin", &bytes);
        let mut seq = sequence();
        assert_eq!(load(&mut seq, &path), (true, None));
        let header = *seq.header();
        assert_eq!(header.num_records, 5);
        // file_size is the record block: file length minus the 4-byte header.
        assert_eq!(
            header.file_size as usize,
            bytes.len() - SEQUENCE_HEADER_SIZE
        );
        // Don't-care on every load, canonicalized to the live time.
        assert_eq!(header.time_base, TimeBase::TbWorkstationTime);
        assert_eq!(header.time_context, 7);
        // Records walk in order, then the buffer is exhausted.
        for i in 0..5u32 {
            let record = next(&mut seq);
            assert_eq!(record.descriptor, RecordDescriptor::Relative);
            assert_eq!(record.command.as_slice(), command_buffer(i, i + 1));
        }
        assert!(!seq.has_more_records());
        seq.reset();
        assert!(seq.has_more_records());
        seq.clear();
        assert!(!seq.has_more_records());
    }

    #[test]
    fn any_live_time_base_is_accepted() {
        let dir = temp_dir();
        let path = write_with_crc(&dir, "t.bin", &immediate_file(1));
        let mut seq = sequence();
        for (base, context) in [(TimeBase::TbNone, 0u8), (TimeBase::TbProcTime, 3)] {
            let mut event = None;
            let now = Time::new(base, context, 0, 0);
            assert!(seq.load_file(&cmd_string(&path), &now, &mut event));
            assert_eq!(event, None);
            assert_eq!(seq.header().time_base, base);
            assert_eq!(seq.header().time_context, context);
        }
    }

    #[test]
    fn header_only_file_loads_with_zero_records() {
        // C++ parity: no CS_NoRecords check in the AMPCS format.
        let dir = temp_dir();
        let path = write_with_crc(&dir, "e.bin", &header());
        let mut seq = sequence();
        assert_eq!(load(&mut seq, &path), (true, None));
        assert_eq!(seq.header().num_records, 0);
        assert_eq!(seq.header().file_size, 0);
        assert!(!seq.has_more_records());
    }

    #[test]
    #[should_panic(expected = "Assert")]
    fn next_record_on_a_zero_record_sequence_asserts() {
        let dir = temp_dir();
        let path = write_with_crc(&dir, "e.bin", &header());
        let mut seq = sequence();
        assert_eq!(load(&mut seq, &path), (true, None));
        let _ = next(&mut seq);
    }

    // -- Record errors -----------------------------------------------------

    #[test]
    fn unknown_time_flag_is_a_format_error() {
        let dir = temp_dir();
        for flag in [0x01u8, 0x02, 10, 0xFE] {
            let mut bytes = header();
            bytes.extend(command_record(TimeFlag::Relative as u8, 0, 1, 2));
            bytes.extend(command_record(flag, 0, 3, 4));
            let path = write_with_crc(&dir, "f.bin", &bytes);
            let mut seq = sequence();
            assert_eq!(
                load(&mut seq, &path),
                (
                    false,
                    Some(SequenceLoadEvent::RecordInvalid {
                        record_number: 1,
                        error: SerializeStatus::DeserFormatError as i32,
                    })
                ),
                "flag {flag:#04x}"
            );
        }
    }

    #[test]
    fn command_length_beyond_the_remaining_bytes_is_a_size_mismatch() {
        let dir = temp_dir();
        let mut bytes = header();
        let mut rec = record(TimeFlag::Absolute as u8, 0, &[1, 2, 3, 4]);
        rec[6] = 5; // claim 5 bytes, only 4 follow
        bytes.extend(rec);
        let path = write_with_crc(&dir, "l.bin", &bytes);
        let mut seq = sequence();
        assert_eq!(
            load(&mut seq, &path),
            (
                false,
                Some(SequenceLoadEvent::RecordInvalid {
                    record_number: 0,
                    error: SerializeStatus::DeserSizeMismatch as i32,
                })
            )
        );
    }

    #[test]
    fn truncated_record_fields_report_their_status() {
        // Flag present, time short: DeserSizeMismatch from the u32 read.
        let dir = temp_dir();
        let mut bytes = header();
        bytes.extend_from_slice(&[0xFF, 0x00, 0x00]);
        let path = write_with_crc(&dir, "r.bin", &bytes);
        let mut seq = sequence();
        let (ok, event) = load(&mut seq, &path);
        assert!(!ok);
        let Some(SequenceLoadEvent::RecordInvalid {
            record_number,
            error,
        }) = event
        else {
            panic!("expected RecordInvalid, got {event:?}");
        };
        assert_eq!(record_number, 0);
        assert_ne!(error, SerializeStatus::Ok as i32);
    }

    #[test]
    fn translated_packet_must_fit_the_com_buffer() {
        // descriptor(2) + zero extension(2) + cmdLength <= 512.
        let dir = temp_dir();
        for (len, expect_ok) in [(508usize, true), (509usize, false)] {
            let mut bytes = header();
            bytes.extend(record(TimeFlag::Relative as u8, 0, &vec![0x5A; len]));
            let path = write_with_crc(&dir, "b.bin", &bytes);
            let mut seq = sequence();
            let (ok, event) = load(&mut seq, &path);
            assert_eq!(ok, expect_ok, "cmdLength {len}");
            if expect_ok {
                let command = next(&mut seq).command;
                assert_eq!(command.as_slice().len(), FW_COM_BUFFER_MAX_SIZE);
                assert_eq!(&command.as_slice()[..4], &[0, 0, 0, 0]);
            } else {
                assert_eq!(
                    event,
                    Some(SequenceLoadEvent::RecordInvalid {
                        record_number: 0,
                        error: SerializeStatus::DeserSizeMismatch as i32,
                    })
                );
            }
        }
    }

    // -- File errors -------------------------------------------------------

    #[test]
    fn missing_sidecar_is_file_not_found_under_the_sidecar_name() {
        let dir = temp_dir();
        let path = write_sequence(&dir, "m.bin", &immediate_file(1));
        let mut seq = sequence();
        assert_eq!(
            load(&mut seq, &path),
            (false, Some(SequenceLoadEvent::FileNotFound))
        );
        assert_eq!(
            seq.log_file_name().as_bytes(),
            format!("{path}.CRC32").as_bytes()
        );
    }

    #[test]
    fn missing_sequence_file_is_a_header_size_error_with_the_filesystem_status() {
        let dir = temp_dir();
        let path = write_with_crc(&dir, "m.bin", &immediate_file(1));
        std::fs::remove_file(&path).unwrap();
        let mut seq = sequence();
        assert_eq!(
            load(&mut seq, &path),
            (
                false,
                Some(SequenceLoadEvent::FileInvalid {
                    stage: FileReadStage::ReadHeaderSize,
                    error: filesystem::Status::DoesntExist as i32,
                })
            )
        );
        assert_eq!(filesystem::Status::DoesntExist as i32, 8);
        assert_eq!(seq.log_file_name().as_bytes(), path.as_bytes());
    }

    #[test]
    fn file_shorter_than_the_header_is_a_header_size_error_with_op_ok() {
        let dir = temp_dir();
        for len in [0usize, 3] {
            let path = write_with_crc(&dir, "s.bin", &vec![0x11; len]);
            let mut seq = sequence();
            assert_eq!(
                load(&mut seq, &path),
                (
                    false,
                    Some(SequenceLoadEvent::FileInvalid {
                        stage: FileReadStage::ReadHeaderSize,
                        error: filesystem::Status::OpOk as i32,
                    })
                ),
                "length {len}"
            );
        }
    }

    #[test]
    fn record_block_larger_than_the_buffer_is_a_file_size_error() {
        let dir = temp_dir();
        let path = write_with_crc(&dir, "x.bin", &too_large_file(BUFFER_SIZE));
        let mut seq = sequence();
        assert_eq!(
            load(&mut seq, &path),
            (
                false,
                Some(SequenceLoadEvent::FileSizeError {
                    size: 2 * BUFFER_SIZE as u32
                })
            )
        );
        // Exactly the capacity is accepted.
        let mut bytes = header();
        bytes.extend(record(
            TimeFlag::Relative as u8,
            0,
            &vec![0u8; BUFFER_SIZE - 7],
        ));
        // ...but the 1017-byte command field cannot be translated.
        let path = write_with_crc(&dir, "y.bin", &bytes);
        let (ok, event) = load(&mut seq, &path);
        assert!(!ok);
        assert_eq!(
            event,
            Some(SequenceLoadEvent::RecordInvalid {
                record_number: 0,
                error: SerializeStatus::DeserSizeMismatch as i32,
            })
        );
    }

    #[test]
    fn crc_mismatch_reports_stored_and_computed() {
        let dir = temp_dir();
        let (path, stored, computed) = write_bad_crc_file(&dir, "k.bin");
        let mut seq = sequence();
        assert_eq!(
            load(&mut seq, &path),
            (
                false,
                Some(SequenceLoadEvent::FileCrcFailure { stored, computed })
            )
        );
    }

    #[test]
    fn short_sidecar_keeps_the_stale_buffer_bytes_as_the_crc() {
        // C++ parity: readCRC checks only the read status, so a short
        // sidecar leaves the rest of the four CRC bytes as the buffer held.
        let dir = temp_dir();
        let mut seq = sequence();
        // Fresh buffer: a zero-length sidecar reads as CRC 0.
        let path = write_sequence(&dir, "q.bin", &immediate_file(1));
        std::fs::write(format!("{path}.CRC32"), []).unwrap();
        let (ok, event) = load(&mut seq, &path);
        assert!(!ok);
        let computed = Hash::hash_u32(&immediate_file(1));
        assert_eq!(
            event,
            Some(SequenceLoadEvent::FileCrcFailure {
                stored: 0,
                computed
            })
        );
        // After a load the buffer starts with the first record's bytes
        // (`FF 00 00 00` ...): a two-byte sidecar keeps the last two.
        std::fs::write(format!("{path}.CRC32"), [0xAA, 0xBB]).unwrap();
        let (ok, event) = load(&mut seq, &path);
        assert!(!ok);
        assert_eq!(
            event,
            Some(SequenceLoadEvent::FileCrcFailure {
                stored: 0xAABB_0000,
                computed
            })
        );
        // Bytes past the first four are ignored.
        let mut long = computed.to_be_bytes().to_vec();
        long.extend_from_slice(&[1, 2, 3]);
        std::fs::write(format!("{path}.CRC32"), long).unwrap();
        assert_eq!(load(&mut seq, &path), (true, None));
    }

    #[test]
    fn sidecar_name_is_truncated_at_the_command_string_capacity() {
        let name = "x".repeat(FW_CMD_STRING_MAX_SIZE - 3);
        let crc = crc_file_name(&cmd_string(&name));
        assert_eq!(crc.as_bytes(), format!("{name}.CR").as_bytes());
        let full = "y".repeat(FW_CMD_STRING_MAX_SIZE);
        assert_eq!(
            crc_file_name(&cmd_string(&full)).as_bytes(),
            full.as_bytes()
        );
    }

    #[test]
    fn a_full_length_name_makes_the_sequence_its_own_sidecar() {
        // With a 40-byte name `+= ".CRC32"` appends nothing, so the first
        // four bytes of the sequence file are read as its CRC.
        let dir = temp_dir();
        let prefix = format!("{}/", dir.to_str().unwrap());
        assert!(prefix.len() < FW_CMD_STRING_MAX_SIZE);
        let name = "n".repeat(FW_CMD_STRING_MAX_SIZE - prefix.len());
        let path = write_sequence(&dir, &name, &immediate_file(1));
        assert_eq!(path.len(), FW_CMD_STRING_MAX_SIZE);
        let mut seq = sequence();
        let (ok, event) = load(&mut seq, &path);
        assert!(!ok);
        assert_eq!(
            event,
            Some(SequenceLoadEvent::FileCrcFailure {
                stored: 0x1122_3344,
                computed: Hash::hash_u32(&immediate_file(1)),
            })
        );
    }

    #[test]
    fn unreadable_sidecar_is_not_file_not_found() {
        // A directory named like the sidecar: open or read fails, but not
        // with DOESNT_EXIST.
        let dir = temp_dir();
        let path = write_sequence(&dir, "d.bin", &immediate_file(1));
        std::fs::create_dir(format!("{path}.CRC32")).unwrap();
        let mut seq = sequence();
        let (ok, event) = load(&mut seq, &path);
        assert!(!ok);
        assert!(
            matches!(
                event,
                Some(SequenceLoadEvent::FileReadError)
                    | Some(SequenceLoadEvent::FileInvalid {
                        stage: FileReadStage::ReadSeqCrc,
                        ..
                    })
            ),
            "got {event:?}"
        );
    }

    // -- Buffer management -------------------------------------------------

    #[test]
    #[should_panic(expected = "Assert")]
    fn allocate_buffer_rejects_a_buffer_smaller_than_the_base_header() {
        // The C++ check lives in the `Sequence` base class: 11 bytes, even
        // though the AMPCS header is 4.
        AMPCSSequence::new().allocate_buffer(0, 10);
    }

    #[test]
    #[should_panic(expected = "Assert")]
    fn loading_without_a_buffer_asserts() {
        let mut seq = AMPCSSequence::new();
        let _ = load(&mut seq, "/nonexistent");
    }

    #[test]
    fn allocate_and_deallocate_track_the_capacity() {
        let mut seq = AMPCSSequence::default();
        assert_eq!(seq.capacity(), 0);
        seq.allocate_buffer(42, 64);
        assert_eq!(seq.allocator_id(), 42);
        assert_eq!(seq.capacity(), 64);
        seq.deallocate_buffer();
        assert_eq!(seq.capacity(), 0);
    }
}
