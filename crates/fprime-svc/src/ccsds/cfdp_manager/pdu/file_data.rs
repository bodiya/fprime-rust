//! File Data PDU — port of `Svc/Ccsds/CfdpManager/Types/FileDataPdu.{hpp,cpp}`
//! (CCSDS 727.0-B-5 section 5.3).
//!
//! ```text
//! header (pdu_type bit = 1) | offset u32 (u64 with the large-file flag) | data
//! ```
//!
//! The PDU borrows its data: on transmit, the file bytes just read; on
//! receive, the bytes inside the received buffer (upstream keeps a raw
//! pointer into the buffer).

use super::{LargeFileFlag, Pdu, PduDirection, PduHeader, PduReader, PduType, write_header};
use crate::ccsds::cfdp_manager::types::{Class, EntityId, FileSize, PduTypeEnum, TransactionSeq};
use fprime_config::cfdp::MAX_PDU_SIZE;
use fprime_fw::{
    Endianness, ExtBuf, LengthMode, SerBuf, SerBufAny, SerializeStatus, fw_assert, fw_try,
};

/// C++ `FileDataPdu`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FileDataPdu<'a> {
    header: PduHeader,
    offset: FileSize,
    data_size: u16,
    data: &'a [u8],
}

impl<'a> FileDataPdu<'a> {
    /// An uninitialized PDU.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// C++ `initialize`. `data` holds at least `data_size` bytes.
    #[allow(clippy::too_many_arguments)]
    pub fn initialize(
        &mut self,
        direction: PduDirection,
        txm_mode: Class,
        source_eid: EntityId,
        transaction_seq: TransactionSeq,
        dest_eid: EntityId,
        offset: FileSize,
        data_size: u16,
        data: &'a [u8],
    ) {
        self.header.initialize(
            PduTypeEnum::FileData,
            direction,
            txm_mode,
            source_eid,
            transaction_seq,
            dest_eid,
        );
        self.offset = offset;
        self.data_size = data_size;
        self.data = data;
    }

    /// The header (C++ `asHeader`).
    #[must_use]
    pub fn as_header(&self) -> &PduHeader {
        &self.header
    }

    /// Mutable header access (tests set the large-file and segment-metadata
    /// flags through it).
    pub fn header_mut(&mut self) -> &mut PduHeader {
        &mut self.header
    }

    /// The file offset.
    #[must_use]
    pub fn get_offset(&self) -> FileSize {
        self.offset
    }

    /// The number of data bytes.
    #[must_use]
    pub fn get_data_size(&self) -> u16 {
        self.data_size
    }

    /// The data bytes.
    #[must_use]
    pub fn get_data(&self) -> &'a [u8] {
        &self.data[..usize::from(self.data_size).min(self.data.len())]
    }

    fn offset_size(&self) -> u32 {
        if self.header.large_file_flag == LargeFileFlag::LargeFile64Bit {
            size_of::<u64>() as u32
        } else {
            size_of::<u32>() as u32
        }
    }

    /// C++ `getMaxFileDataSize`: the data bytes that fit a `MaxPduSize`
    /// PDU with this header.
    #[must_use]
    pub fn get_max_file_data_size(&self) -> u32 {
        MAX_PDU_SIZE as u32 - (self.header.get_buffer_size() + self.offset_size())
    }

    /// C++ `toBuffer`: encode into `buffer`, returning the encoded length
    /// in `size` on success (upstream shrinks the `Fw::Buffer`).
    pub fn to_buffer(&self, buffer: &mut [u8], size: &mut usize) -> SerializeStatus {
        let mut sb = ExtBuf::new(buffer);
        let status = self.write_pdu(&mut sb);
        if status == SerializeStatus::Ok {
            *size = sb.ser_loc();
        }
        status
    }

    /// C++ `fromBuffer`: decode, rejecting a directive PDU with
    /// `DeserTypeMismatch`.
    pub fn from_buffer(&mut self, buffer: &'a [u8]) -> SerializeStatus {
        let mut reader = PduReader::new(buffer);
        fw_try!(self.header.from_serial_buffer(&mut reader));
        if self.header.pdu_type != PduType::FileData {
            return SerializeStatus::DeserTypeMismatch;
        }
        self.header.type_ = PduTypeEnum::FileData;
        self.read_body(&mut reader)
    }

    /// C++ `deserializeFrom`. Unlike `from_buffer` it does not check the
    /// PDU type first: decoding a directive asserts (C++ parity — callers
    /// peek the type before decoding).
    pub fn deserialize_from(&mut self, reader: &mut PduReader<'a>) -> SerializeStatus {
        fw_try!(self.header.from_serial_buffer(reader));
        self.read_body(reader)
    }

    /// The body: a 32-bit offset (upstream reads `sizeof(FileSize)` bytes
    /// whatever the large-file flag says), then `pdu data length - 4` data
    /// bytes, which must all be present.
    fn read_body(&mut self, reader: &mut PduReader<'a>) -> SerializeStatus {
        fw_assert!(self.header.type_ == PduTypeEnum::FileData);
        fw_try!(reader.read_u32(&mut self.offset));
        let offset_size = size_of::<FileSize>() as u16;
        let pdu_data_length = self.header.get_pdu_data_length();
        if pdu_data_length < offset_size {
            return SerializeStatus::DeserSizeMismatch;
        }
        self.data_size = pdu_data_length - offset_size;
        self.data = reader.rest();
        if reader.remaining() < usize::from(self.data_size) {
            return SerializeStatus::DeserSizeMismatch;
        }
        match reader.read_bytes(usize::from(self.data_size)) {
            Ok(_) => SerializeStatus::Ok,
            Err(status) => status,
        }
    }

    fn write_pdu(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        fw_assert!(self.header.type_ == PduTypeEnum::FileData);
        fw_try!(write_header(&self.header, self.get_buffer_size(), buf));
        if self.header.large_file_flag == LargeFileFlag::LargeFile64Bit {
            fw_try!(buf.serialize_u64_be(u64::from(self.offset)));
        } else {
            fw_try!(buf.serialize_u32_be(self.offset));
        }
        if self.data_size > 0 {
            fw_try!(buf.serialize_bytes(
                &self.data[..usize::from(self.data_size)],
                LengthMode::OmitLength,
                Endianness::Big
            ));
        }
        SerializeStatus::Ok
    }
}

impl Pdu for FileDataPdu<'_> {
    fn header(&self) -> &PduHeader {
        &self.header
    }

    /// Header + offset (4 or 8 bytes) + data.
    fn get_buffer_size(&self) -> u32 {
        self.header.get_buffer_size() + self.offset_size() + u32::from(self.data_size)
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        self.write_pdu(buf)
    }
}
