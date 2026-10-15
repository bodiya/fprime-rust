//! End-of-File PDU — port of `Svc/Ccsds/CfdpManager/Types/EofPdu.{hpp,cpp}`
//! (CCSDS 727.0-B-5 section 5.2.2).
//!
//! ```text
//! header | 0x04 | condition code(4) spare(4) | checksum u32 | file size u32 | TLVs
//! ```

use super::{
    Pdu, PduDirection, PduHeader, PduReader, Tlv, TlvList, read_directive_header, write_header,
};
use crate::ccsds::cfdp_manager::types::{
    Class, ConditionCode, EntityId, FileDirective, FileSize, PduTypeEnum, TransactionSeq,
};
use fprime_fw::{SerBuf, SerBufAny, SerializeStatus, fw_assert, fw_try};

/// C++ `EofPdu`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EofPdu {
    header: PduHeader,
    condition_code: ConditionCode,
    checksum: u32,
    file_size: FileSize,
    tlv_list: TlvList,
}

impl EofPdu {
    /// An uninitialized PDU.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// C++ `initialize` (clears the TLV list).
    #[allow(clippy::too_many_arguments)]
    pub fn initialize(
        &mut self,
        direction: PduDirection,
        txm_mode: Class,
        source_eid: EntityId,
        transaction_seq: TransactionSeq,
        dest_eid: EntityId,
        condition_code: ConditionCode,
        checksum: u32,
        file_size: FileSize,
    ) {
        self.header.initialize(
            PduTypeEnum::EndOfFile,
            direction,
            txm_mode,
            source_eid,
            transaction_seq,
            dest_eid,
        );
        self.condition_code = condition_code;
        self.checksum = checksum;
        self.file_size = file_size;
        self.tlv_list.clear();
    }

    /// The header (C++ `asHeader`).
    #[must_use]
    pub fn as_header(&self) -> &PduHeader {
        &self.header
    }

    /// The condition code.
    #[must_use]
    pub fn get_condition_code(&self) -> ConditionCode {
        self.condition_code
    }

    /// The file checksum.
    #[must_use]
    pub fn get_checksum(&self) -> u32 {
        self.checksum
    }

    /// The file size.
    #[must_use]
    pub fn get_file_size(&self) -> FileSize {
        self.file_size
    }

    /// The directive code.
    #[must_use]
    pub fn get_directive_code(&self) -> FileDirective {
        FileDirective::END_OF_FILE
    }

    /// Append a TLV; false when the list is full.
    pub fn append_tlv(&mut self, tlv: &Tlv) -> bool {
        self.tlv_list.append_tlv(tlv)
    }

    /// The TLV list.
    #[must_use]
    pub fn get_tlv_list(&self) -> &TlvList {
        &self.tlv_list
    }

    /// Number of TLVs.
    #[must_use]
    pub fn get_num_tlv(&self) -> u8 {
        self.tlv_list.get_num_tlv()
    }

    /// C++ `deserializeFrom`. The TLV list runs to the end of the buffer.
    pub fn deserialize_from(&mut self, reader: &mut PduReader<'_>) -> SerializeStatus {
        fw_try!(read_directive_header(
            &mut self.header,
            reader,
            FileDirective::END_OF_FILE,
            PduTypeEnum::EndOfFile
        ));
        fw_assert!(self.header.type_ == PduTypeEnum::EndOfFile);
        let mut cc = 0u8;
        fw_try!(reader.read_u8(&mut cc));
        self.condition_code = ConditionCode((cc >> 4) & 0x0F);
        fw_try!(reader.read_u32(&mut self.checksum));
        fw_try!(reader.read_u32(&mut self.file_size));
        self.tlv_list.from_serial_buffer(reader)
    }
}

impl Pdu for EofPdu {
    fn header(&self) -> &PduHeader {
        &self.header
    }

    /// Header + directive code + condition code + checksum + file size + TLVs.
    fn get_buffer_size(&self) -> u32 {
        self.header.get_buffer_size()
            + 1
            + 1
            + 4
            + size_of::<FileSize>() as u32
            + self.tlv_list.get_encoded_size()
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        fw_assert!(self.header.type_ == PduTypeEnum::EndOfFile);
        fw_try!(write_header(&self.header, self.get_buffer_size(), buf));
        fw_try!(buf.serialize_u8_be(FileDirective::END_OF_FILE.0));
        fw_try!(buf.serialize_u8_be((self.condition_code.0 & 0x0F) << 4));
        fw_try!(buf.serialize_u32_be(self.checksum));
        fw_try!(buf.serialize_u32_be(self.file_size));
        self.tlv_list.to_serial_buffer(buf)
    }
}
