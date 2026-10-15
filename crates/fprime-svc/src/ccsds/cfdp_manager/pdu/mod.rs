//! CFDP PDU encoding and decoding — port of
//! `Svc/Ccsds/CfdpManager/Types/{PduBase,PduHeader}.{hpp,cpp}` and the
//! concrete PDU classes in the submodules (CCSDS 727.0-B-5, section 5).
//!
//! These are new F´ implementations upstream (not CF-derived; see
//! `ATTRIBUTION.md`): object-style PDUs serialized through an
//! `Fw::SerialBufferBase`. Here a PDU is written into any
//! [`SerBufAny`] (an `ExtBuf` over the outgoing `Fw::Buffer` in flight
//! code) and read from a byte slice through [`PduReader`], which applies
//! the exact `Fw::SerialBuffer` cursor rules (`DeserBufferEmpty` when no
//! byte is left, `DeserSizeMismatch` when some but too few are), because
//! deserialization statuses are reported verbatim in the
//! `Fail*PduDeserialization` events.
//!
//! Wire layout of the fixed header (section 5.1, all big-endian):
//!
//! ```text
//! byte 0   version(3)=1 | pdu_type(1) | direction(1) | txm_mode(1) | crc_flag(1) | large_file(1)
//! byte 1-2 PDU data field length
//! byte 3   seg_ctrl(1) | eid_len-1(3) | seg_meta(1) | tsn_len-1(3)
//! then     source EID, transaction sequence number, destination EID
//! ```
//!
//! The EID and TSN widths are the minimum byte counts that hold the
//! values (both EIDs share the width of the larger one).

mod ack;
mod eof;
mod file_data;
mod fin;
mod metadata;
mod nak;
mod tlv;

pub use ack::AckPdu;
pub use eof::EofPdu;
pub use file_data::FileDataPdu;
pub use fin::FinPdu;
pub use metadata::MetadataPdu;
pub use nak::{NakPdu, SegmentRequest};
pub use tlv::{Tlv, TlvData, TlvList, TlvType};

use super::types::{Class, EntityId, FileDirective, PduTypeEnum, TransactionSeq};
use fprime_fw::{SerBuf, SerBufAny, SerializeStatus, fw_assert, fw_try};

/// C++ `PduType`: the header's PDU type bit.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PduType {
    /// File directive PDU.
    #[default]
    Directive = 0,
    /// File data PDU.
    FileData = 1,
}

/// C++ `PduDirection`: the header's direction bit.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PduDirection {
    /// Toward the file receiver.
    #[default]
    TowardReceiver = 0,
    /// Toward the file sender.
    TowardSender = 1,
}

/// C++ `CrcFlag`: whether a PDU CRC trails the PDU (never set here).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CrcFlag {
    /// No PDU CRC.
    #[default]
    NotPresent = 0,
    /// PDU CRC present.
    Present = 1,
}

/// C++ `LargeFileFlag`: 32- or 64-bit file sizes and offsets.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LargeFileFlag {
    /// 32-bit file sizes.
    #[default]
    LargeFile32Bit = 0,
    /// 64-bit file sizes.
    LargeFile64Bit = 1,
}

/// A read cursor over a received PDU: the `Fw::SerialBuffer` that upstream
/// builds over `buffer.getData()` with `setBuffLen(buffer.getSize())`.
#[derive(Debug)]
pub struct PduReader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> PduReader<'a> {
    /// A reader with every byte of `data` readable.
    #[must_use]
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    /// Bytes left to read (C++ `getDeserializeSizeLeft`).
    #[must_use]
    pub fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }

    /// The unread bytes (C++ `getBuffAddrLeft`).
    #[must_use]
    pub fn rest(&self) -> &'a [u8] {
        &self.data[self.pos..]
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], SerializeStatus> {
        let left = self.remaining();
        if left == 0 && n > 0 {
            return Err(SerializeStatus::DeserBufferEmpty);
        }
        if left < n {
            return Err(SerializeStatus::DeserSizeMismatch);
        }
        let out = &self.data[self.pos..self.pos + n];
        self.pos += n;
        Ok(out)
    }

    /// Read a `U8`.
    pub fn read_u8(&mut self, value: &mut u8) -> SerializeStatus {
        match self.take(1) {
            Ok(b) => {
                *value = b[0];
                SerializeStatus::Ok
            }
            Err(status) => status,
        }
    }

    /// Read a big-endian `U16`.
    pub fn read_u16(&mut self, value: &mut u16) -> SerializeStatus {
        match self.take(2) {
            Ok(b) => {
                *value = u16::from_be_bytes([b[0], b[1]]);
                SerializeStatus::Ok
            }
            Err(status) => status,
        }
    }

    /// Read a big-endian `U32`.
    pub fn read_u32(&mut self, value: &mut u32) -> SerializeStatus {
        match self.take(4) {
            Ok(b) => {
                *value = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
                SerializeStatus::Ok
            }
            Err(status) => status,
        }
    }

    /// Read `n` raw bytes (C++ `deserializeTo(buff, n, OMIT_LENGTH)` /
    /// `popBytes`): `DeserSizeMismatch` when fewer than `n` are left.
    pub fn read_bytes(&mut self, n: usize) -> Result<&'a [u8], SerializeStatus> {
        if n > self.remaining() {
            return Err(SerializeStatus::DeserSizeMismatch);
        }
        let out = &self.data[self.pos..self.pos + n];
        self.pos += n;
        Ok(out)
    }
}

/// C++ `PduHeader`: the fixed header common to every PDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PduHeader {
    /// The PDU type derived from the directive code (or `FILE_DATA`).
    pub(crate) type_: PduTypeEnum,
    version: u8,
    pub(crate) pdu_type: PduType,
    direction: PduDirection,
    class: Class,
    crc_flag: CrcFlag,
    pub(crate) large_file_flag: LargeFileFlag,
    segmentation_control: u8,
    segment_metadata_flag: u8,
    pdu_data_length: u16,
    source_eid: EntityId,
    transaction_seq: TransactionSeq,
    dest_eid: EntityId,
}

impl Default for PduHeader {
    /// C++ leaves a fresh header's members uninitialized; here they start
    /// zeroed, with the type `NONE`.
    fn default() -> Self {
        Self {
            type_: PduTypeEnum::None,
            version: 0,
            pdu_type: PduType::Directive,
            direction: PduDirection::TowardReceiver,
            class: Class::Class2,
            crc_flag: CrcFlag::NotPresent,
            large_file_flag: LargeFileFlag::LargeFile32Bit,
            segmentation_control: 0,
            segment_metadata_flag: 0,
            pdu_data_length: 0,
            source_eid: 0,
            transaction_seq: 0,
            dest_eid: 0,
        }
    }
}

/// C++ `encodeIntegerInSize`: `value` as `size` big-endian bytes.
fn encode_integer_in_size(buf: &mut dyn SerBufAny, value: u64, size: u8) -> SerializeStatus {
    for i in 0..size {
        let shift = u32::from(size - 1 - i) * 8;
        fw_try!(buf.serialize_u8_be((value >> shift) as u8));
    }
    SerializeStatus::Ok
}

/// C++ `decodeIntegerInSize`.
fn decode_integer_in_size(
    reader: &mut PduReader<'_>,
    size: u8,
    status: &mut SerializeStatus,
) -> u64 {
    let mut value: u64 = 0;
    for _ in 0..size {
        let mut byte = 0u8;
        *status = reader.read_u8(&mut byte);
        if *status != SerializeStatus::Ok {
            return 0;
        }
        value = (value << 8) | u64::from(byte);
    }
    value
}

impl PduHeader {
    /// Minimum encoded header size (one-byte EIDs and TSN).
    pub const MIN_HEADERSIZE: usize = 7;

    /// C++ `initialize`: version 1, the PDU-type bit from `type_`, no CRC,
    /// 32-bit sizes, no segmentation, data length 0 (set at encode time).
    pub fn initialize(
        &mut self,
        type_: PduTypeEnum,
        direction: PduDirection,
        txm_mode: Class,
        source_eid: EntityId,
        transaction_seq: TransactionSeq,
        dest_eid: EntityId,
    ) {
        self.type_ = type_;
        self.version = 1;
        self.pdu_type = if type_ == PduTypeEnum::FileData {
            PduType::FileData
        } else {
            PduType::Directive
        };
        self.direction = direction;
        self.class = txm_mode;
        self.crc_flag = CrcFlag::NotPresent;
        self.large_file_flag = LargeFileFlag::LargeFile32Bit;
        self.segmentation_control = 0;
        self.segment_metadata_flag = 0;
        self.pdu_data_length = 0;
        self.source_eid = source_eid;
        self.transaction_seq = transaction_seq;
        self.dest_eid = dest_eid;
    }

    /// C++ `getValueEncodedSize`: bytes (1..=8) needed to hold `value`.
    #[must_use]
    pub fn get_value_encoded_size(value: u64) -> u8 {
        let mut min_size: u8 = 1;
        let mut limit: u64 = 0x100;
        while min_size < 8 && value >= limit {
            limit <<= 8;
            min_size += 1;
        }
        min_size
    }

    fn eid_size(&self) -> u8 {
        Self::get_value_encoded_size(u64::from(self.source_eid.max(self.dest_eid)))
    }

    fn tsn_size(&self) -> u8 {
        Self::get_value_encoded_size(u64::from(self.transaction_seq))
    }

    /// C++ `getBufferSize`: the encoded header size.
    #[must_use]
    pub fn get_buffer_size(&self) -> u32 {
        4 + 2 * u32::from(self.eid_size()) + u32::from(self.tsn_size())
    }

    /// C++ `toSerialBuffer`.
    pub fn to_serial_buffer(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        let eid_size = self.eid_size();
        let tsn_size = self.tsn_size();
        let flags = ((self.version & 0x07) << 5)
            | ((self.pdu_type as u8 & 0x01) << 4)
            | ((self.direction as u8 & 0x01) << 3)
            | ((self.class.as_repr() & 0x01) << 2)
            | ((self.crc_flag as u8 & 0x01) << 1)
            | (self.large_file_flag as u8 & 0x01);
        fw_try!(buf.serialize_u8_be(flags));
        fw_try!(buf.serialize_u16_be(self.pdu_data_length));
        let eid_tsn_lengths = ((self.segmentation_control & 0x01) << 7)
            | (((eid_size - 1) & 0x07) << 4)
            | ((self.segment_metadata_flag & 0x01) << 3)
            | ((tsn_size - 1) & 0x07);
        fw_try!(buf.serialize_u8_be(eid_tsn_lengths));
        fw_try!(encode_integer_in_size(
            buf,
            u64::from(self.source_eid),
            eid_size
        ));
        fw_try!(encode_integer_in_size(
            buf,
            u64::from(self.transaction_seq),
            tsn_size
        ));
        encode_integer_in_size(buf, u64::from(self.dest_eid), eid_size)
    }

    /// C++ `fromSerialBuffer`. Leaves the type `FILE_DATA` for a file data
    /// PDU and `NONE` for a directive (the directive code sets it).
    pub fn from_serial_buffer(&mut self, reader: &mut PduReader<'_>) -> SerializeStatus {
        let mut flags = 0u8;
        fw_try!(reader.read_u8(&mut flags));
        self.version = (flags >> 5) & 0x07;
        self.pdu_type = if (flags >> 4) & 0x01 == 1 {
            PduType::FileData
        } else {
            PduType::Directive
        };
        self.direction = if (flags >> 3) & 0x01 == 1 {
            PduDirection::TowardSender
        } else {
            PduDirection::TowardReceiver
        };
        // Both values of the bit are declared classes (CLASS_2 = 0, CLASS_1 = 1).
        self.class = if (flags >> 2) & 0x01 == 1 {
            Class::Class1
        } else {
            Class::Class2
        };
        self.crc_flag = if (flags >> 1) & 0x01 == 1 {
            CrcFlag::Present
        } else {
            CrcFlag::NotPresent
        };
        self.large_file_flag = if flags & 0x01 == 1 {
            LargeFileFlag::LargeFile64Bit
        } else {
            LargeFileFlag::LargeFile32Bit
        };
        fw_try!(reader.read_u16(&mut self.pdu_data_length));
        let mut eid_tsn_lengths = 0u8;
        fw_try!(reader.read_u8(&mut eid_tsn_lengths));
        self.segmentation_control = (eid_tsn_lengths >> 7) & 0x01;
        let eid_size = ((eid_tsn_lengths >> 4) & 0x07) + 1;
        self.segment_metadata_flag = (eid_tsn_lengths >> 3) & 0x01;
        let tsn_size = (eid_tsn_lengths & 0x07) + 1;
        // C++ parity: asserted (always true for a 3-bit field plus one).
        fw_assert!((1..=8).contains(&eid_size), i32::from(eid_size));
        fw_assert!((1..=8).contains(&tsn_size), i32::from(tsn_size));
        let mut status = SerializeStatus::Ok;
        // C++ parity: wider encodings are truncated to the configured widths.
        self.source_eid = decode_integer_in_size(reader, eid_size, &mut status) as EntityId;
        fw_try!(status);
        self.transaction_seq =
            decode_integer_in_size(reader, tsn_size, &mut status) as TransactionSeq;
        fw_try!(status);
        self.dest_eid = decode_integer_in_size(reader, eid_size, &mut status) as EntityId;
        fw_try!(status);
        self.type_ = if self.pdu_type == PduType::FileData {
            PduTypeEnum::FileData
        } else {
            PduTypeEnum::None
        };
        SerializeStatus::Ok
    }

    /// The PDU type (C++ `getType`).
    #[must_use]
    pub fn get_type(&self) -> PduTypeEnum {
        self.type_
    }

    /// The direction (C++ `getDirection`).
    #[must_use]
    pub fn get_direction(&self) -> PduDirection {
        self.direction
    }

    /// The transmission mode (C++ `getTxmMode`).
    #[must_use]
    pub fn get_txm_mode(&self) -> Class {
        self.class
    }

    /// The source entity id.
    #[must_use]
    pub fn get_source_eid(&self) -> EntityId {
        self.source_eid
    }

    /// The transaction sequence number.
    #[must_use]
    pub fn get_transaction_seq(&self) -> TransactionSeq {
        self.transaction_seq
    }

    /// The destination entity id.
    #[must_use]
    pub fn get_dest_eid(&self) -> EntityId {
        self.dest_eid
    }

    /// The PDU data field length.
    #[must_use]
    pub fn get_pdu_data_length(&self) -> u16 {
        self.pdu_data_length
    }

    /// Set the PDU data field length (used while encoding).
    pub fn set_pdu_data_length(&mut self, length: u16) {
        self.pdu_data_length = length;
    }

    /// The large-file flag.
    #[must_use]
    pub fn get_large_file_flag(&self) -> LargeFileFlag {
        self.large_file_flag
    }

    /// Set the large-file flag (tests and configuration).
    pub fn set_large_file_flag(&mut self, flag: LargeFileFlag) {
        self.large_file_flag = flag;
    }

    /// Whether the segment-metadata flag is set.
    #[must_use]
    pub fn has_segment_metadata(&self) -> bool {
        self.segment_metadata_flag != 0
    }

    /// The CFDP version field.
    #[must_use]
    pub fn get_version(&self) -> u8 {
        self.version
    }

    /// The CRC flag.
    #[must_use]
    pub fn get_crc_flag(&self) -> CrcFlag {
        self.crc_flag
    }

    /// Set the raw segment-metadata flag (tests: upstream reaches it by
    /// patching the encoded byte).
    pub fn set_segment_metadata_flag(&mut self, flag: u8) {
        self.segment_metadata_flag = flag;
    }
}

/// C++ `PduBase`: what the engine needs to encode any PDU.
pub trait Pdu {
    /// The header.
    fn header(&self) -> &PduHeader;
    /// The encoded size (C++ `getBufferSize`).
    fn get_buffer_size(&self) -> u32;
    /// Encode (C++ `serializeTo`).
    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus;
    /// The PDU type (C++ `getType`).
    fn get_type(&self) -> PduTypeEnum {
        self.header().get_type()
    }
}

/// Encode `header` with its data length set to `total - header size`
/// (the `headerCopy` step every concrete `toSerialBuffer` begins with).
fn write_header(header: &PduHeader, total_size: u32, buf: &mut dyn SerBufAny) -> SerializeStatus {
    let mut copy = *header;
    copy.set_pdu_data_length((total_size - header.get_buffer_size()) as u16);
    copy.to_serial_buffer(buf)
}

/// Decode a directive header and check its directive code (the common
/// prologue of every directive PDU's `deserializeFrom`).
fn read_directive_header(
    header: &mut PduHeader,
    reader: &mut PduReader<'_>,
    expected: FileDirective,
    type_: PduTypeEnum,
) -> SerializeStatus {
    fw_try!(header.from_serial_buffer(reader));
    if header.pdu_type != PduType::Directive {
        return SerializeStatus::DeserTypeMismatch;
    }
    let mut directive_code = 0u8;
    fw_try!(reader.read_u8(&mut directive_code));
    if directive_code != expected.0 {
        return SerializeStatus::DeserTypeMismatch;
    }
    header.type_ = type_;
    SerializeStatus::Ok
}

/// C++ `peekPduType`: the PDU type of an encoded PDU without decoding it;
/// `NONE` for anything shorter than a minimal header, a directive code
/// beyond the buffer, or an unknown directive code.
#[must_use]
pub fn peek_pdu_type(data: &[u8]) -> PduTypeEnum {
    if data.len() < PduHeader::MIN_HEADERSIZE {
        return PduTypeEnum::None;
    }
    if (data[0] >> 4) & 0x01 == PduType::FileData as u8 {
        return PduTypeEnum::FileData;
    }
    let eid_tsn_lengths = data[3];
    let eid_size = usize::from(((eid_tsn_lengths >> 4) & 0x07) + 1);
    let tsn_size = usize::from((eid_tsn_lengths & 0x07) + 1);
    let directive_code_offset = 4 + 2 * eid_size + tsn_size;
    if directive_code_offset >= data.len() {
        return PduTypeEnum::None;
    }
    match FileDirective(data[directive_code_offset]) {
        FileDirective::METADATA => PduTypeEnum::Metadata,
        FileDirective::END_OF_FILE => PduTypeEnum::EndOfFile,
        FileDirective::FIN => PduTypeEnum::Finished,
        FileDirective::ACK => PduTypeEnum::Acknowledgment,
        FileDirective::NAK => PduTypeEnum::NegativeAck,
        _ => PduTypeEnum::None,
    }
}
