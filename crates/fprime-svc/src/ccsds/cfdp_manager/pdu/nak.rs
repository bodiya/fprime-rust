//! Negative Acknowledgment PDU — port of
//! `Svc/Ccsds/CfdpManager/Types/NakPdu.{hpp,cpp}` (CCSDS 727.0-B-5 section
//! 5.2.6).
//!
//! ```text
//! header | 0x08 | scope start u32 | scope end u32 | N x (start u32, end u32)
//! ```
//!
//! The segment count is implied by the PDU length; decoding takes every
//! whole 8-byte segment left in the buffer, truncated to `U8` and then
//! clamped to `NakMaxSegments` (C++ parity: the truncation comes first).

use super::{Pdu, PduDirection, PduHeader, PduReader, read_directive_header, write_header};
use crate::ccsds::cfdp_manager::types::{
    Class, EntityId, FileDirective, FileSize, PduTypeEnum, TransactionSeq,
};
use fprime_config::cfdp::NAK_MAX_SEGMENTS;
use fprime_fw::{SerBuf, SerBufAny, SerializeStatus, fw_assert, fw_try};

/// C++ `SegmentRequest`: a missing range `[offset_start, offset_end)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SegmentRequest {
    /// Start offset of the missing data.
    pub offset_start: FileSize,
    /// End offset of the missing data.
    pub offset_end: FileSize,
}

/// C++ `NakPdu`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NakPdu {
    header: PduHeader,
    scope_start: FileSize,
    scope_end: FileSize,
    num_segments: u8,
    segments: [SegmentRequest; NAK_MAX_SEGMENTS],
}

impl Default for NakPdu {
    fn default() -> Self {
        Self {
            header: PduHeader::default(),
            scope_start: 0,
            scope_end: 0,
            num_segments: 0,
            segments: [SegmentRequest::default(); NAK_MAX_SEGMENTS],
        }
    }
}

impl NakPdu {
    /// An uninitialized PDU.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// C++ `initialize` (no segments).
    #[allow(clippy::too_many_arguments)]
    pub fn initialize(
        &mut self,
        direction: PduDirection,
        txm_mode: Class,
        source_eid: EntityId,
        transaction_seq: TransactionSeq,
        dest_eid: EntityId,
        scope_start: FileSize,
        scope_end: FileSize,
    ) {
        self.header.initialize(
            PduTypeEnum::NegativeAck,
            direction,
            txm_mode,
            source_eid,
            transaction_seq,
            dest_eid,
        );
        self.scope_start = scope_start;
        self.scope_end = scope_end;
        self.num_segments = 0;
    }

    /// The header (C++ `asHeader`).
    #[must_use]
    pub fn as_header(&self) -> &PduHeader {
        &self.header
    }

    /// The scope start.
    #[must_use]
    pub fn get_scope_start(&self) -> FileSize {
        self.scope_start
    }

    /// The scope end.
    #[must_use]
    pub fn get_scope_end(&self) -> FileSize {
        self.scope_end
    }

    /// Number of segment requests.
    #[must_use]
    pub fn get_num_segments(&self) -> u8 {
        self.num_segments
    }

    /// Segment request `index` (unchecked against the count, as upstream;
    /// asserts only the array bound).
    #[must_use]
    pub fn get_segment(&self, index: u8) -> &SegmentRequest {
        fw_assert!(usize::from(index) < NAK_MAX_SEGMENTS, i32::from(index));
        &self.segments[usize::from(index)]
    }

    /// Add a segment request; false when `NakMaxSegments` are present.
    pub fn add_segment(&mut self, offset_start: FileSize, offset_end: FileSize) -> bool {
        if usize::from(self.num_segments) >= NAK_MAX_SEGMENTS {
            return false;
        }
        self.segments[usize::from(self.num_segments)] = SegmentRequest {
            offset_start,
            offset_end,
        };
        self.num_segments += 1;
        true
    }

    /// Remove every segment request.
    pub fn clear_segments(&mut self) {
        self.num_segments = 0;
    }

    /// The directive code.
    #[must_use]
    pub fn get_directive_code(&self) -> FileDirective {
        FileDirective::NAK
    }

    /// C++ `deserializeFrom`.
    pub fn deserialize_from(&mut self, reader: &mut PduReader<'_>) -> SerializeStatus {
        fw_try!(read_directive_header(
            &mut self.header,
            reader,
            FileDirective::NAK,
            PduTypeEnum::NegativeAck
        ));
        fw_assert!(self.header.type_ == PduTypeEnum::NegativeAck);
        fw_try!(reader.read_u32(&mut self.scope_start));
        fw_try!(reader.read_u32(&mut self.scope_end));
        let segment_size = 2 * size_of::<FileSize>();
        // C++ parity: `static_cast<U8>` of the count before the clamp.
        self.num_segments = (reader.remaining() / segment_size) as u8;
        if usize::from(self.num_segments) > NAK_MAX_SEGMENTS {
            self.num_segments = NAK_MAX_SEGMENTS as u8;
        }
        for segment in &mut self.segments[..usize::from(self.num_segments)] {
            fw_try!(reader.read_u32(&mut segment.offset_start));
            fw_try!(reader.read_u32(&mut segment.offset_end));
        }
        SerializeStatus::Ok
    }
}

impl Pdu for NakPdu {
    fn header(&self) -> &PduHeader {
        &self.header
    }

    /// Header + directive code + scope + segments.
    fn get_buffer_size(&self) -> u32 {
        self.header.get_buffer_size()
            + 1
            + 2 * size_of::<FileSize>() as u32
            + u32::from(self.num_segments) * 2 * size_of::<FileSize>() as u32
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        fw_assert!(self.header.type_ == PduTypeEnum::NegativeAck);
        fw_try!(write_header(&self.header, self.get_buffer_size(), buf));
        fw_try!(buf.serialize_u8_be(FileDirective::NAK.0));
        fw_try!(buf.serialize_u32_be(self.scope_start));
        fw_try!(buf.serialize_u32_be(self.scope_end));
        for segment in &self.segments[..usize::from(self.num_segments)] {
            fw_try!(buf.serialize_u32_be(segment.offset_start));
            fw_try!(buf.serialize_u32_be(segment.offset_end));
        }
        SerializeStatus::Ok
    }
}
