//! Acknowledgment PDU — port of `Svc/Ccsds/CfdpManager/Types/AckPdu.{hpp,cpp}`
//! (CCSDS 727.0-B-5 section 5.2.4).
//!
//! ```text
//! header | 0x06 | acked directive(4) subtype(4) | condition code(4) spare(2) txn status(2)
//! ```

use super::{Pdu, PduDirection, PduHeader, PduReader, read_directive_header, write_header};
use crate::ccsds::cfdp_manager::types::{
    AckTxnStatus, Class, ConditionCode, EntityId, FileDirective, PduTypeEnum, TransactionSeq,
};
use fprime_fw::{SerBuf, SerBufAny, SerializeStatus, fw_assert, fw_try};

/// C++ `AckPdu`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AckPdu {
    header: PduHeader,
    directive_code: FileDirective,
    directive_subtype_code: u8,
    condition_code: ConditionCode,
    transaction_status: AckTxnStatus,
}

impl AckPdu {
    /// An uninitialized PDU.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// C++ `initialize`.
    #[allow(clippy::too_many_arguments)]
    pub fn initialize(
        &mut self,
        direction: PduDirection,
        txm_mode: Class,
        source_eid: EntityId,
        transaction_seq: TransactionSeq,
        dest_eid: EntityId,
        directive_code: FileDirective,
        directive_subtype_code: u8,
        condition_code: ConditionCode,
        transaction_status: AckTxnStatus,
    ) {
        self.header.initialize(
            PduTypeEnum::Acknowledgment,
            direction,
            txm_mode,
            source_eid,
            transaction_seq,
            dest_eid,
        );
        self.directive_code = directive_code;
        self.directive_subtype_code = directive_subtype_code;
        self.condition_code = condition_code;
        self.transaction_status = transaction_status;
    }

    /// The header (C++ `asHeader`).
    #[must_use]
    pub fn as_header(&self) -> &PduHeader {
        &self.header
    }

    /// The directive being acknowledged.
    #[must_use]
    pub fn get_directive_code(&self) -> FileDirective {
        self.directive_code
    }

    /// The directive subtype code.
    #[must_use]
    pub fn get_directive_subtype_code(&self) -> u8 {
        self.directive_subtype_code
    }

    /// The condition code.
    #[must_use]
    pub fn get_condition_code(&self) -> ConditionCode {
        self.condition_code
    }

    /// The transaction status.
    #[must_use]
    pub fn get_transaction_status(&self) -> AckTxnStatus {
        self.transaction_status
    }

    /// C++ `deserializeFrom`.
    pub fn deserialize_from(&mut self, reader: &mut PduReader<'_>) -> SerializeStatus {
        fw_try!(read_directive_header(
            &mut self.header,
            reader,
            FileDirective::ACK,
            PduTypeEnum::Acknowledgment
        ));
        fw_assert!(self.header.type_ == PduTypeEnum::Acknowledgment);
        let mut directive_and_subtype = 0u8;
        fw_try!(reader.read_u8(&mut directive_and_subtype));
        self.directive_code = FileDirective((directive_and_subtype >> 4) & 0x0F);
        self.directive_subtype_code = directive_and_subtype & 0x0F;
        let mut cc_and_status = 0u8;
        fw_try!(reader.read_u8(&mut cc_and_status));
        self.condition_code = ConditionCode((cc_and_status >> 4) & 0x0F);
        self.transaction_status = AckTxnStatus(cc_and_status & 0x03);
        SerializeStatus::Ok
    }
}

impl Pdu for AckPdu {
    fn header(&self) -> &PduHeader {
        &self.header
    }

    /// Header + directive code + two packed bytes.
    fn get_buffer_size(&self) -> u32 {
        self.header.get_buffer_size() + 3
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        fw_assert!(self.header.type_ == PduTypeEnum::Acknowledgment);
        fw_try!(write_header(&self.header, self.get_buffer_size(), buf));
        fw_try!(buf.serialize_u8_be(FileDirective::ACK.0));
        fw_try!(buf.serialize_u8_be(
            ((self.directive_code.0 & 0x0F) << 4) | (self.directive_subtype_code & 0x0F)
        ));
        buf.serialize_u8_be(
            ((self.condition_code.0 & 0x0F) << 4) | (self.transaction_status.0 & 0x03),
        )
    }
}
