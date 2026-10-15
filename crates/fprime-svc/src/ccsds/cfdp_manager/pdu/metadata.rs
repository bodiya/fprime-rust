//! Metadata PDU — port of `Svc/Ccsds/CfdpManager/Types/MetadataPdu.{hpp,cpp}`
//! (CCSDS 727.0-B-5 section 5.2.5).
//!
//! ```text
//! header | 0x07 | closure(1) rsvd(3) checksum_type(4) | file size u32
//!        | src name len u8 | src name | dst name len u8 | dst name
//! ```

use super::{Pdu, PduDirection, PduHeader, PduReader, read_directive_header, write_header};
use crate::ccsds::cfdp_manager::types::{
    ChecksumType, Class, EntityId, FileDirective, FileSize, PathString, PduTypeEnum, TransactionSeq,
};
use fprime_config::cfdp::MAX_FILE_PATH_SIZE;
use fprime_fw::{Endianness, LengthMode, SerBuf, SerBufAny, SerializeStatus, fw_assert, fw_try};

/// C++ `MetadataPdu`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MetadataPdu {
    header: PduHeader,
    closure_requested: u8,
    checksum_type: ChecksumType,
    file_size: FileSize,
    source_filename: PathString,
    dest_filename: PathString,
}

impl Default for MetadataPdu {
    fn default() -> Self {
        Self {
            header: PduHeader::default(),
            closure_requested: 0,
            checksum_type: ChecksumType::MODULAR,
            file_size: 0,
            source_filename: PathString::new(),
            dest_filename: PathString::new(),
        }
    }
}

impl MetadataPdu {
    /// An uninitialized PDU.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// C++ `initialize`. Asserts that both names fit `MaxFilePathSize`.
    #[allow(clippy::too_many_arguments)]
    pub fn initialize(
        &mut self,
        direction: PduDirection,
        txm_mode: Class,
        source_eid: EntityId,
        transaction_seq: TransactionSeq,
        dest_eid: EntityId,
        file_size: FileSize,
        source_filename: &[u8],
        dest_filename: &[u8],
        checksum_type: ChecksumType,
        closure_requested: u8,
    ) {
        self.header.initialize(
            PduTypeEnum::Metadata,
            direction,
            txm_mode,
            source_eid,
            transaction_seq,
            dest_eid,
        );
        self.file_size = file_size;
        fw_assert!(
            source_filename.len() <= MAX_FILE_PATH_SIZE,
            source_filename.len() as i32,
            MAX_FILE_PATH_SIZE as i32
        );
        self.source_filename.set_bytes(source_filename);
        fw_assert!(
            dest_filename.len() <= MAX_FILE_PATH_SIZE,
            dest_filename.len() as i32,
            MAX_FILE_PATH_SIZE as i32
        );
        self.dest_filename.set_bytes(dest_filename);
        self.checksum_type = checksum_type;
        self.closure_requested = closure_requested;
    }

    /// The header (C++ `asHeader`).
    #[must_use]
    pub fn as_header(&self) -> &PduHeader {
        &self.header
    }

    /// The declared file size.
    #[must_use]
    pub fn get_file_size(&self) -> FileSize {
        self.file_size
    }

    /// The source file name.
    #[must_use]
    pub fn get_source_filename(&self) -> &PathString {
        &self.source_filename
    }

    /// The destination file name.
    #[must_use]
    pub fn get_dest_filename(&self) -> &PathString {
        &self.dest_filename
    }

    /// The checksum type.
    #[must_use]
    pub fn get_checksum_type(&self) -> ChecksumType {
        self.checksum_type
    }

    /// The closure-requested flag.
    #[must_use]
    pub fn get_closure_requested(&self) -> u8 {
        self.closure_requested
    }

    /// The directive code.
    #[must_use]
    pub fn get_directive_code(&self) -> FileDirective {
        FileDirective::METADATA
    }

    /// C++ `deserializeFrom`: header, directive code, then the body.
    pub fn deserialize_from(&mut self, reader: &mut PduReader<'_>) -> SerializeStatus {
        fw_try!(read_directive_header(
            &mut self.header,
            reader,
            FileDirective::METADATA,
            PduTypeEnum::Metadata
        ));
        self.read_body(reader)
    }

    /// Decode one length-prefixed file name: an empty name or one longer
    /// than `MaxFilePathSize` is a `DeserSizeMismatch`.
    fn read_name(reader: &mut PduReader<'_>, name: &mut PathString) -> SerializeStatus {
        let mut length = 0u8;
        fw_try!(reader.read_u8(&mut length));
        if usize::from(length) > MAX_FILE_PATH_SIZE || length == 0 {
            return SerializeStatus::DeserSizeMismatch;
        }
        match reader.read_bytes(usize::from(length)) {
            Ok(bytes) => {
                // C++ parity: assigned as a C string (stops at an embedded NUL).
                name.set_bytes(bytes);
                SerializeStatus::Ok
            }
            Err(status) => status,
        }
    }

    fn read_body(&mut self, reader: &mut PduReader<'_>) -> SerializeStatus {
        fw_assert!(self.header.type_ == PduTypeEnum::Metadata);
        let mut segmentation_control = 0u8;
        fw_try!(reader.read_u8(&mut segmentation_control));
        self.closure_requested = (segmentation_control >> 7) & 0x01;
        self.checksum_type = ChecksumType(segmentation_control & 0x0F);
        fw_try!(reader.read_u32(&mut self.file_size));
        fw_try!(Self::read_name(reader, &mut self.source_filename));
        Self::read_name(reader, &mut self.dest_filename)
    }

    fn write_pdu(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        fw_assert!(self.header.type_ == PduTypeEnum::Metadata);
        fw_try!(write_header(&self.header, self.get_buffer_size(), buf));
        fw_try!(buf.serialize_u8_be(FileDirective::METADATA.0));
        let segmentation_control =
            ((self.closure_requested & 0x01) << 7) | (self.checksum_type.0 & 0x0F);
        fw_try!(buf.serialize_u8_be(segmentation_control));
        fw_try!(buf.serialize_u32_be(self.file_size));
        for name in [&self.source_filename, &self.dest_filename] {
            fw_try!(buf.serialize_u8_be(name.len() as u8));
            fw_try!(buf.serialize_bytes(name.as_bytes(), LengthMode::OmitLength, Endianness::Big));
        }
        SerializeStatus::Ok
    }
}

impl Pdu for MetadataPdu {
    fn header(&self) -> &PduHeader {
        &self.header
    }

    /// Header + directive code + segmentation control + file size + the
    /// two length-value names.
    fn get_buffer_size(&self) -> u32 {
        self.header.get_buffer_size()
            + 1
            + 1
            + size_of::<FileSize>() as u32
            + 1
            + self.source_filename.len() as u32
            + 1
            + self.dest_filename.len() as u32
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        self.write_pdu(buf)
    }
}
