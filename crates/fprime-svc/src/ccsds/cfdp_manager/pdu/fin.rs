//! Finished PDU — port of `Svc/Ccsds/CfdpManager/Types/FinPdu.{hpp,cpp}`
//! (CCSDS 727.0-B-5 section 5.2.3).
//!
//! ```text
//! header | 0x05 | condition code(4) spare(1) delivery code(1) file status(2) | TLVs
//! ```

use super::{
    Pdu, PduDirection, PduHeader, PduReader, Tlv, TlvList, read_directive_header, write_header,
};
use crate::ccsds::cfdp_manager::types::{
    Class, ConditionCode, EntityId, FileDirective, FinDeliveryCode, FinFileStatus, PduTypeEnum,
    TransactionSeq,
};
use fprime_fw::{SerBuf, SerBufAny, SerializeStatus, fw_assert, fw_try};

/// C++ `FinPdu`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FinPdu {
    header: PduHeader,
    condition_code: ConditionCode,
    delivery_code: FinDeliveryCode,
    file_status: FinFileStatus,
    tlv_list: TlvList,
}

impl FinPdu {
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
        delivery_code: FinDeliveryCode,
        file_status: FinFileStatus,
    ) {
        self.header.initialize(
            PduTypeEnum::Finished,
            direction,
            txm_mode,
            source_eid,
            transaction_seq,
            dest_eid,
        );
        self.condition_code = condition_code;
        self.delivery_code = delivery_code;
        self.file_status = file_status;
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

    /// The delivery code.
    #[must_use]
    pub fn get_delivery_code(&self) -> FinDeliveryCode {
        self.delivery_code
    }

    /// The file status.
    #[must_use]
    pub fn get_file_status(&self) -> FinFileStatus {
        self.file_status
    }

    /// The directive code.
    #[must_use]
    pub fn get_directive_code(&self) -> FileDirective {
        FileDirective::FIN
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
            FileDirective::FIN,
            PduTypeEnum::Finished
        ));
        fw_assert!(self.header.type_ == PduTypeEnum::Finished);
        let mut flags = 0u8;
        fw_try!(reader.read_u8(&mut flags));
        self.condition_code = ConditionCode((flags >> 4) & 0x0F);
        self.delivery_code = FinDeliveryCode::from_bit((flags >> 2) & 0x01);
        self.file_status = FinFileStatus::from_bits(flags & 0x03);
        self.tlv_list.from_serial_buffer(reader)
    }
}

impl Pdu for FinPdu {
    fn header(&self) -> &PduHeader {
        &self.header
    }

    /// Header + directive code + flags + TLVs.
    fn get_buffer_size(&self) -> u32 {
        self.header.get_buffer_size() + 1 + 1 + self.tlv_list.get_encoded_size()
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        fw_assert!(self.header.type_ == PduTypeEnum::Finished);
        fw_try!(write_header(&self.header, self.get_buffer_size(), buf));
        fw_try!(buf.serialize_u8_be(FileDirective::FIN.0));
        let flags = ((self.condition_code.0 & 0x0F) << 4)
            | ((self.delivery_code as u8 & 0x01) << 2)
            | (self.file_status as u8 & 0x03);
        fw_try!(buf.serialize_u8_be(flags));
        self.tlv_list.to_serial_buffer(buf)
    }
}
