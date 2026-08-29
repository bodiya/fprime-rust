//! # CCSDS types — port of `Svc/Ccsds/Types/Types.fpp` and the CCSDS part of
//! `default/config/ComCfg.fpp`
//!
//! C++ sources: `Svc/Ccsds/Types/Types.fpp`, `Svc/Ccsds/Ports/Ports.fpp`,
//! `default/config/ComCfg.fpp`.
//! Analysis: `docs/cpp-analysis/ccsds.md` ("ComCfg.fpp", "Svc::Ccsds::
//! FrameError", "Svc.Ccsds Ports" and the "Wire formats" section, which is
//! normative for every bit position below).
//!
//! Three groups live here:
//!
//! 1. **Config constants** ([`SPACECRAFT_ID`], [`TM_FRAME_FIXED_SIZE`], ...)
//!    — the `ComCfg` values the CCSDS components are built against. They are
//!    kept in this module (rather than `fprime-config`) so a deployment has
//!    one CCSDS override point, mirroring `ComCfg.fpp`.
//! 2. **Frame header/trailer structs**, declared with `fpp_struct!` so their
//!    `SERIALIZED_SIZE` and big-endian member-order encoding come from the
//!    same generator the rest of the port uses (6/6/2/5/2/6/2/2 bytes). Each
//!    holds the RAW header words exactly as the C++ `Serializable` does; the
//!    bit fields packed inside those words are exposed as `const fn`
//!    accessors and builders alongside the mask/offset constants from
//!    `Types.fpp`, so round-tripping is byte-exact.
//! 3. **The CCSDS port traits** ([`ApidSequenceCountPort`],
//!    [`ErrorNotifyPort`]) from `Svc/Ccsds/Ports/Ports.fpp`. They live here
//!    for the same reason `com_stub` defines the byte-stream port traits
//!    locally: they are CCSDS-specific and `fprime-comp` has no CCSDS
//!    knowledge.

use fprime_config::FwIndexType;
use fprime_fw::{Apid, fpp_enum, fpp_struct};

// ---------------------------------------------------------------------------
// ComCfg constants (default/config/ComCfg.fpp)
// ---------------------------------------------------------------------------

/// Spacecraft identifier (`ComCfg::SpacecraftId`), 10 bits on the wire.
pub const SPACECRAFT_ID: u16 = 0x0044;

/// Fixed TM transfer-frame size in bytes (`ComCfg::TmFrameFixedSize`).
pub const TM_FRAME_FIXED_SIZE: usize = 1024;

/// Maximum AOS transfer-frame size (`ComCfg::AosMaxFrameFixedSize`); declared
/// for parity — the AOS components are out of scope for this port.
pub const AOS_MAX_FRAME_FIXED_SIZE: usize = 1536;

/// Maximum payload the ComAggregator hands to the [`TmFramer`] —
/// `TmFrameFixedSize - 6 - 6 - 1 - 2` (two Space Packet headers, one idle
/// data byte and the TM trailer). Equals 1009.
///
/// [`TmFramer`]: crate::ccsds::tm_framer::TmFramer
pub const AGGREGATION_SIZE: usize = TM_FRAME_FIXED_SIZE
    - SpacePacketHeader::SERIALIZED_SIZE
    - SpacePacketHeader::SERIALIZED_SIZE
    - 1
    - TMTrailer::SERIALIZED_SIZE;

// ---------------------------------------------------------------------------
// Enums (Types.fpp)
// ---------------------------------------------------------------------------

fpp_enum! {
    /// Error reported over [`ErrorNotifyPort`] during CCSDS framing or
    /// deframing (`Svc::Ccsds::FrameError`).
    pub enum FrameError : u8 {
        /// Space Packet: malformed packet (size, deserialization or PVN).
        SpInvalidPacket = 0,
        /// Space Packet: header length exceeds the received data.
        SpInvalidLength = 1,
        /// TC: spacecraft ID mismatch.
        TcInvalidScid = 2,
        /// TC: frame length invalid or larger than the received data.
        TcInvalidLength = 3,
        /// TC: virtual channel ID mismatch.
        TcInvalidVcid = 4,
        /// TC: frame error control field mismatch.
        TcInvalidCrc = 5,
        /// AOS: spacecraft ID mismatch (CCSDS 732.0-B-5 4.1.2.2).
        AosInvalidScid = 6,
        /// AOS: frame length insufficient.
        AosInvalidLength = 7,
        /// AOS: virtual channel ID mismatch (4.1.2.3).
        AosInvalidVcid = 8,
        /// AOS: frame error control field mismatch (4.1.6).
        AosInvalidCrc = 9,
        /// AOS: transfer frame version number mismatch (4.1.2.2.2).
        AosInvalidVersion = 10,
        /// AOS: encapsulation packet protocol error (CCSDS 133.1-B-3).
        AosInvalidEpp = 11,
        /// AOS: virtual channel frame count discontinuity.
        AosVcFrameCountGap = 12,
        /// SDLS decryption failed.
        SdlsDecryptionFailure = 13,
    }
    default SpInvalidPacket
}

fpp_enum! {
    /// Status of an SDLS (Space Data Link Security) request
    /// (`Svc::Ccsds::SdlsStatus`). Declared for parity; the SDLS components
    /// are out of scope for this port.
    pub enum SdlsStatus : u8 {
        /// Request completed successfully.
        Success = 0,
        /// Security association index has no known mapping.
        UnknownSa = 1,
        /// Mapped port index is out of range or unconnected.
        UnknownPort = 2,
        /// Encryption operation failed.
        EncryptionFailure = 3,
        /// Decryption operation failed.
        DecryptionFailure = 4,
        /// Key retrieval failed.
        KeyError = 5,
    }
    default Success
}

fpp_enum! {
    /// Transfer Frame Version Number (`Svc::Ccsds::Tfvn`). CCSDS counts
    /// versions from one, so TM/TC "version 1" is `0b00` on the wire.
    pub enum Tfvn : u8 {
        /// Telemetry and Telecommand space data links.
        TmTc = 0x0,
        /// Advanced Orbiting Systems space data link.
        Aos = 0x1,
        /// Proximity-1 space data link.
        ProxOne = 0x2,
        /// Unified Space Data Link Protocol.
        Uslp = 0x3,
        /// Anything of equal or higher value is invalid (the FPP default).
        InvalidUninitialized = 0x4,
    }
    default InvalidUninitialized
}

// ---------------------------------------------------------------------------
// Space Packet
// ---------------------------------------------------------------------------

/// Bit masks, offsets and widths for the Space Packet primary header
/// (`Types.fpp` module `SpacePacketSubfields`).
pub mod space_packet_subfields {
    /// Packet Version Number mask, `packetIdentification` bits `[15:13]`.
    pub const PVN_MASK: u16 = 0xE000;
    /// Packet Type mask, `packetIdentification` bit `[12]`.
    pub const PKT_TYPE_MASK: u16 = 0x1000;
    /// Secondary Header Flag mask, `packetIdentification` bit `[11]`.
    pub const SEC_HDR_MASK: u16 = 0x0800;
    /// APID mask, `packetIdentification` bits `[10:0]`.
    pub const APID_MASK: u16 = 0x07FF;
    /// Packet Version Number shift.
    pub const PVN_OFFSET: u32 = 13;
    /// Packet Type shift.
    pub const PKT_TYPE_OFFSET: u32 = 12;
    /// Secondary Header Flag shift.
    pub const SEC_HDR_OFFSET: u32 = 11;
    /// Sequence Flags mask, `packetSequenceControl` bits `[15:14]`.
    pub const SEQ_FLAGS_MASK: u16 = 0xC000;
    /// Packet Sequence Count mask, `packetSequenceControl` bits `[13:0]`.
    pub const SEQ_COUNT_MASK: u16 = 0x3FFF;
    /// Sequence Flags shift.
    pub const SEQ_FLAGS_OFFSET: u32 = 14;
    /// APID field width in bits.
    pub const APID_WIDTH: u32 = 11;
    /// Packet Sequence Count field width in bits.
    pub const SEQ_COUNT_WIDTH: u32 = 14;
}

fpp_struct! {
    /// CCSDS Space Packet primary header, 6 bytes big-endian
    /// (`Svc::Ccsds::SpacePacketHeader`).
    ///
    /// Wire layout:
    ///
    /// | bytes | field | contents |
    /// |-------|-------|----------|
    /// | 0..2 | `packet_identification` | 3b PVN, 1b packet type, 1b secondary header flag, 11b APID |
    /// | 2..4 | `packet_sequence_control` | 2b sequence flags, 14b sequence count |
    /// | 4..6 | `packet_data_length` | data-field octets **minus one** |
    #[derive(Clone, Copy, Eq)]
    pub struct SpacePacketHeader {
        /// 3b PVN | 1b packet type | 1b secondary header flag | 11b APID.
        packet_identification: u16 { get_packet_identification, set_packet_identification },
        /// 2b sequence flags | 14b packet sequence count.
        packet_sequence_control: u16 { get_packet_sequence_control, set_packet_sequence_control },
        /// Number of octets in the packet data field, minus one.
        packet_data_length: u16 { get_packet_data_length, set_packet_data_length },
    }
}

impl SpacePacketHeader {
    /// Pack a `packetIdentification` word.
    ///
    /// Every field is masked into place, exactly as the C++ framer does for
    /// the APID and the secondary-header flag (it hardcodes PVN = 0 and
    /// packet type = 0).
    #[must_use]
    pub const fn build_packet_identification(
        pvn: u8,
        packet_type: u8,
        has_sec_hdr: bool,
        apid: u16,
    ) -> u16 {
        use space_packet_subfields as sp;
        let sec_hdr_flag = if has_sec_hdr { 1u16 } else { 0u16 };
        (((pvn as u16) << sp::PVN_OFFSET) & sp::PVN_MASK)
            | (((packet_type as u16) << sp::PKT_TYPE_OFFSET) & sp::PKT_TYPE_MASK)
            | ((sec_hdr_flag << sp::SEC_HDR_OFFSET) & sp::SEC_HDR_MASK)
            | (apid & sp::APID_MASK)
    }

    /// Pack a `packetSequenceControl` word from 2-bit sequence flags and a
    /// 14-bit sequence count.
    #[must_use]
    pub const fn build_packet_sequence_control(sequence_flags: u8, sequence_count: u16) -> u16 {
        use space_packet_subfields as sp;
        (((sequence_flags as u16) << sp::SEQ_FLAGS_OFFSET) & sp::SEQ_FLAGS_MASK)
            | (sequence_count & sp::SEQ_COUNT_MASK)
    }

    /// The 3-bit Packet Version Number (0 for the Space Packet Protocol).
    #[must_use]
    pub const fn pvn(&self) -> u8 {
        ((self.packet_identification & space_packet_subfields::PVN_MASK)
            >> space_packet_subfields::PVN_OFFSET) as u8
    }

    /// The packet type bit (0 = telemetry/report, 1 = telecommand).
    #[must_use]
    pub const fn packet_type(&self) -> u8 {
        ((self.packet_identification & space_packet_subfields::PKT_TYPE_MASK)
            >> space_packet_subfields::PKT_TYPE_OFFSET) as u8
    }

    /// The secondary header flag.
    #[must_use]
    pub const fn has_sec_hdr(&self) -> bool {
        (self.packet_identification & space_packet_subfields::SEC_HDR_MASK) != 0
    }

    /// The raw 11-bit APID (not validated against [`Apid`]).
    #[must_use]
    pub const fn apid_value(&self) -> u16 {
        self.packet_identification & space_packet_subfields::APID_MASK
    }

    /// The 2-bit sequence flags.
    #[must_use]
    pub const fn sequence_flags(&self) -> u8 {
        ((self.packet_sequence_control & space_packet_subfields::SEQ_FLAGS_MASK)
            >> space_packet_subfields::SEQ_FLAGS_OFFSET) as u8
    }

    /// The 14-bit packet sequence count.
    #[must_use]
    pub const fn sequence_count(&self) -> u16 {
        self.packet_sequence_control & space_packet_subfields::SEQ_COUNT_MASK
    }

    /// Number of octets in the packet data field: the length token plus one.
    /// Widened to `u32` before the increment, as the C++ deframer does, so a
    /// token of `0xFFFF` yields 65536 rather than wrapping to zero.
    #[must_use]
    pub const fn data_field_length(&self) -> u32 {
        self.packet_data_length as u32 + 1
    }

    /// The length token for a data field of `data_field_length` octets
    /// (octets minus one). The caller guarantees a non-empty data field.
    #[must_use]
    pub const fn length_token(data_field_length: u16) -> u16 {
        data_field_length - 1
    }
}

// ---------------------------------------------------------------------------
// TM transfer frame
// ---------------------------------------------------------------------------

/// Bit offsets for the TM transfer frame primary header (`Types.fpp` module
/// `TMSubfields`), plus the masks implied by them.
pub mod tm_subfields {
    /// Transfer Frame Version Number shift, `globalVcId` bits `[15:14]`.
    pub const FRAME_VERSION_OFFSET: u32 = 14;
    /// Spacecraft ID shift, `globalVcId` bits `[13:4]`.
    pub const SPACECRAFT_ID_OFFSET: u32 = 4;
    /// Virtual Channel ID shift, `globalVcId` bits `[3:1]`.
    pub const VIRTUAL_CHANNEL_ID_OFFSET: u32 = 1;
    /// Segment Length Identifier shift, `dataFieldStatus` bits `[12:11]`.
    pub const SEG_LENGTH_OFFSET: u32 = 11;
    /// Transfer Frame Version Number mask.
    pub const FRAME_VERSION_MASK: u16 = 0xC000;
    /// Spacecraft ID mask (10 bits).
    pub const SPACECRAFT_ID_MASK: u16 = 0x3FF0;
    /// Virtual Channel ID mask (3 bits).
    pub const VIRTUAL_CHANNEL_ID_MASK: u16 = 0x000E;
    /// Operational Control Field flag mask, `globalVcId` bit `[0]`.
    pub const OCF_FLAG_MASK: u16 = 0x0001;
    /// Transfer Frame Secondary Header flag, `dataFieldStatus` bit `[15]`.
    pub const SEC_HDR_FLAG_MASK: u16 = 0x8000;
    /// Synchronization flag, `dataFieldStatus` bit `[14]`.
    pub const SYNC_FLAG_MASK: u16 = 0x4000;
    /// Packet Order flag, `dataFieldStatus` bit `[13]`.
    pub const PACKET_ORDER_FLAG_MASK: u16 = 0x2000;
    /// Segment Length Identifier mask, `dataFieldStatus` bits `[12:11]`.
    pub const SEG_LENGTH_MASK: u16 = 0x1800;
    /// First Header Pointer mask, `dataFieldStatus` bits `[10:0]`.
    pub const FIRST_HEADER_POINTER_MASK: u16 = 0x07FF;
}

fpp_struct! {
    /// TM transfer frame primary header, 6 bytes big-endian
    /// (`Svc::Ccsds::TMHeader`).
    ///
    /// | bytes | field | contents |
    /// |-------|-------|----------|
    /// | 0..2 | `global_vc_id` | 2b frame version, 10b spacecraft ID, 3b virtual channel ID, 1b OCF flag |
    /// | 2..3 | `master_frame_count` | master channel frame count (wraps mod 256) |
    /// | 3..4 | `virtual_frame_count` | virtual channel frame count (wraps mod 256) |
    /// | 4..6 | `data_field_status` | 1b secondary header, 1b sync, 1b packet order, 2b segment length id, 11b first header pointer |
    #[derive(Clone, Copy, Eq)]
    pub struct TMHeader {
        /// 2b frame version | 10b spacecraft ID | 3b virtual channel ID | 1b OCF flag.
        global_vc_id: u16 { get_global_vc_id, set_global_vc_id },
        /// Master channel frame count.
        master_frame_count: u8 { get_master_frame_count, set_master_frame_count },
        /// Virtual channel frame count.
        virtual_frame_count: u8 { get_virtual_frame_count, set_virtual_frame_count },
        /// 1b secondary header | 1b sync | 1b packet order | 2b segment length id | 11b first header pointer.
        data_field_status: u16 { get_data_field_status, set_data_field_status },
    }
}

impl TMHeader {
    /// Pack a `globalVcId` word exactly as `TmFramer` does.
    ///
    /// **C++ parity gotcha** (`TmFramer.cpp`): neither the virtual channel
    /// ID nor the spacecraft ID is masked before being shifted into place,
    /// so a `vc_id` above 7 silently corrupts the spacecraft-ID field and a
    /// spacecraft ID above 10 bits corrupts the frame version. Reproduced
    /// deliberately — see the unit tests.
    #[must_use]
    pub const fn build_global_vc_id(spacecraft_id: u16, vc_id: u8, ocf_flag: bool) -> u16 {
        let ocf = if ocf_flag { 1u16 } else { 0u16 };
        ((vc_id as u16) << tm_subfields::VIRTUAL_CHANNEL_ID_OFFSET)
            | (spacecraft_id << tm_subfields::SPACECRAFT_ID_OFFSET)
            | ocf
    }

    /// Pack a `dataFieldStatus` word. `TmFramer` passes
    /// `(false, false, false, 0b11, 0)`: every flag clear, the segment
    /// length identifier `0b11` required by CCSDS 4.1.2.7.5, and a first
    /// header pointer of 0 (the payload starts at the data field's origin).
    #[must_use]
    pub const fn build_data_field_status(
        sec_hdr_flag: bool,
        sync_flag: bool,
        packet_order_flag: bool,
        segment_length_id: u8,
        first_header_pointer: u16,
    ) -> u16 {
        let sec = if sec_hdr_flag {
            tm_subfields::SEC_HDR_FLAG_MASK
        } else {
            0
        };
        let sync = if sync_flag {
            tm_subfields::SYNC_FLAG_MASK
        } else {
            0
        };
        let order = if packet_order_flag {
            tm_subfields::PACKET_ORDER_FLAG_MASK
        } else {
            0
        };
        sec | sync
            | order
            | (((segment_length_id as u16) << tm_subfields::SEG_LENGTH_OFFSET)
                & tm_subfields::SEG_LENGTH_MASK)
            | (first_header_pointer & tm_subfields::FIRST_HEADER_POINTER_MASK)
    }

    /// The 2-bit Transfer Frame Version Number (`0b00` for TM).
    #[must_use]
    pub const fn frame_version(&self) -> u8 {
        ((self.global_vc_id & tm_subfields::FRAME_VERSION_MASK)
            >> tm_subfields::FRAME_VERSION_OFFSET) as u8
    }

    /// The 10-bit spacecraft ID.
    #[must_use]
    pub const fn spacecraft_id(&self) -> u16 {
        (self.global_vc_id & tm_subfields::SPACECRAFT_ID_MASK) >> tm_subfields::SPACECRAFT_ID_OFFSET
    }

    /// The 3-bit virtual channel ID.
    #[must_use]
    pub const fn vc_id(&self) -> u8 {
        ((self.global_vc_id & tm_subfields::VIRTUAL_CHANNEL_ID_MASK)
            >> tm_subfields::VIRTUAL_CHANNEL_ID_OFFSET) as u8
    }

    /// The Operational Control Field flag.
    #[must_use]
    pub const fn ocf_flag(&self) -> bool {
        (self.global_vc_id & tm_subfields::OCF_FLAG_MASK) != 0
    }

    /// The 2-bit segment length identifier.
    #[must_use]
    pub const fn segment_length_id(&self) -> u8 {
        ((self.data_field_status & tm_subfields::SEG_LENGTH_MASK)
            >> tm_subfields::SEG_LENGTH_OFFSET) as u8
    }

    /// The 11-bit first header pointer.
    #[must_use]
    pub const fn first_header_pointer(&self) -> u16 {
        self.data_field_status & tm_subfields::FIRST_HEADER_POINTER_MASK
    }
}

fpp_struct! {
    /// TM transfer frame trailer, 2 bytes big-endian (`Svc::Ccsds::TMTrailer`).
    #[derive(Clone, Copy, Eq)]
    pub struct TMTrailer {
        /// Frame Error Control Field: CRC-16/CCITT-FALSE over the frame
        /// minus these two bytes.
        fecf: u16 { get_fecf, set_fecf },
    }
}

// ---------------------------------------------------------------------------
// TC transfer frame
// ---------------------------------------------------------------------------

/// Bit masks and offsets for the TC transfer frame primary header
/// (`Types.fpp` module `TCSubfields`).
pub mod tc_subfields {
    /// Transfer Frame Version Number mask, `flagsAndScId` bits `[15:14]`.
    pub const FRAME_VERSION_MASK: u16 = 0xC000;
    /// Bypass flag mask, `flagsAndScId` bit `[13]`.
    pub const BYPASS_FLAG_MASK: u16 = 0x2000;
    /// Control Command flag mask, `flagsAndScId` bit `[12]`.
    pub const CONTROL_FLAG_MASK: u16 = 0x1000;
    /// Reserved spare mask, `flagsAndScId` bits `[11:10]`.
    pub const RESERVED_MASK: u16 = 0x0C00;
    /// Spacecraft ID mask, `flagsAndScId` bits `[9:0]`.
    pub const SPACECRAFT_ID_MASK: u16 = 0x03FF;
    /// Bypass flag shift.
    pub const BYPASS_FLAG_OFFSET: u32 = 13;
    /// Virtual Channel ID mask, `vcIdAndLength` bits `[15:10]`.
    pub const VC_ID_MASK: u16 = 0xFC00;
    /// Frame Length mask, `vcIdAndLength` bits `[9:0]`.
    pub const FRAME_LENGTH_MASK: u16 = 0x03FF;
    /// Virtual Channel ID shift.
    pub const VC_ID_OFFSET: u32 = 10;
}

fpp_struct! {
    /// TC transfer frame primary header, 5 bytes big-endian
    /// (`Svc::Ccsds::TCHeader`).
    ///
    /// | bytes | field | contents |
    /// |-------|-------|----------|
    /// | 0..2 | `flags_and_sc_id` | 2b frame version, 1b bypass, 1b control command, 2b reserved, 10b spacecraft ID |
    /// | 2..4 | `vc_id_and_length` | 6b virtual channel ID, 10b frame length (total octets **minus one**) |
    /// | 4..5 | `frame_sequence_num` | frame sequence number (unused for Type-B) |
    #[derive(Clone, Copy, Eq)]
    pub struct TCHeader {
        /// 2b frame version | 1b bypass | 1b control | 2b reserved | 10b spacecraft ID.
        flags_and_sc_id: u16 { get_flags_and_sc_id, set_flags_and_sc_id },
        /// 6b virtual channel ID | 10b frame length (total octets minus one).
        vc_id_and_length: u16 { get_vc_id_and_length, set_vc_id_and_length },
        /// Frame sequence number; unused (and never checked) for Type-B frames.
        frame_sequence_num: u8 { get_frame_sequence_num, set_frame_sequence_num },
    }
}

impl TCHeader {
    /// Pack a `flagsAndScId` word (frame version `0b00`, reserved spare
    /// `0b00`).
    #[must_use]
    pub const fn build_flags_and_sc_id(
        bypass_flag: bool,
        control_command_flag: bool,
        spacecraft_id: u16,
    ) -> u16 {
        let bypass = if bypass_flag {
            tc_subfields::BYPASS_FLAG_MASK
        } else {
            0
        };
        let control = if control_command_flag {
            tc_subfields::CONTROL_FLAG_MASK
        } else {
            0
        };
        bypass | control | (spacecraft_id & tc_subfields::SPACECRAFT_ID_MASK)
    }

    /// Pack a `vcIdAndLength` word from a virtual channel ID and the TOTAL
    /// frame length in octets (the stored token is that length minus one).
    #[must_use]
    pub const fn build_vc_id_and_length(vc_id: u8, total_frame_length: u16) -> u16 {
        (((vc_id as u16) << tc_subfields::VC_ID_OFFSET) & tc_subfields::VC_ID_MASK)
            | ((total_frame_length - 1) & tc_subfields::FRAME_LENGTH_MASK)
    }

    /// The 2-bit Transfer Frame Version Number.
    #[must_use]
    pub const fn frame_version(&self) -> u8 {
        ((self.flags_and_sc_id & tc_subfields::FRAME_VERSION_MASK) >> 14) as u8
    }

    /// The bypass flag (1 = Type-B, FARM checks bypassed).
    #[must_use]
    pub const fn bypass_flag(&self) -> bool {
        (self.flags_and_sc_id & tc_subfields::BYPASS_FLAG_MASK) != 0
    }

    /// The control command flag (0 = Type-D data).
    #[must_use]
    pub const fn control_command_flag(&self) -> bool {
        (self.flags_and_sc_id & tc_subfields::CONTROL_FLAG_MASK) != 0
    }

    /// The 10-bit spacecraft ID.
    #[must_use]
    pub const fn spacecraft_id(&self) -> u16 {
        self.flags_and_sc_id & tc_subfields::SPACECRAFT_ID_MASK
    }

    /// The 6-bit virtual channel ID.
    #[must_use]
    pub const fn vc_id(&self) -> u8 {
        ((self.vc_id_and_length & tc_subfields::VC_ID_MASK) >> tc_subfields::VC_ID_OFFSET) as u8
    }

    /// The TOTAL frame length in octets: the 10-bit length token plus one
    /// (header + data field + FECF).
    #[must_use]
    pub const fn total_frame_length(&self) -> u16 {
        (self.vc_id_and_length & tc_subfields::FRAME_LENGTH_MASK) + 1
    }
}

fpp_struct! {
    /// TC transfer frame trailer, 2 bytes big-endian (`Svc::Ccsds::TCTrailer`).
    #[derive(Clone, Copy, Eq)]
    pub struct TCTrailer {
        /// Frame Error Control Field: CRC-16/CCITT-FALSE over
        /// `[0, total_frame_length - 2)`.
        fecf: u16 { get_fecf, set_fecf },
    }
}

// ---------------------------------------------------------------------------
// AOS / M_PDU (declared for parity; the AOS components are out of scope)
// ---------------------------------------------------------------------------

/// Special First Header Pointer values (`Types.fpp` module `M_PDUSubfields`).
pub mod m_pdu_subfields {
    /// No packet starts in this frame.
    pub const FHP_NO_PACKET_START: u16 = 0xFFFF;
    /// The frame contains only idle data.
    pub const FHP_IDLE_DATA_ONLY: u16 = 0xFFFE;
}

fpp_struct! {
    /// AOS transfer frame primary header, 6 bytes big-endian
    /// (`Svc::Ccsds::AOSHeader`). Declared for parity with `Types.fpp`.
    #[derive(Clone, Copy, Eq)]
    pub struct AOSHeader {
        /// 2b frame version | 8 LSBs of spacecraft ID | 6b virtual channel ID.
        global_vc_id: u16 { get_global_vc_id, set_global_vc_id },
        /// 24b VC frame count | 1b replay | 1b cycle-use | 2 MSBs of SCID | 4b cycle.
        frame_count_and_signaling: u32 { get_frame_count_and_signaling, set_frame_count_and_signaling },
    }
}

fpp_struct! {
    /// AOS M_PDU header, 2 bytes big-endian (`Svc::Ccsds::M_PDUHeader`;
    /// renamed to a Rust-conventional identifier).
    #[derive(Clone, Copy, Eq)]
    pub struct MPduHeader {
        /// Bytes to the header of the first new CCSDS packet.
        first_header_pointer: u16 { get_first_header_pointer, set_first_header_pointer },
    }
    default {
        first_header_pointer = m_pdu_subfields::FHP_NO_PACKET_START,
    }
}

fpp_struct! {
    /// AOS transfer frame trailer, 2 bytes big-endian
    /// (`Svc::Ccsds::AOSTrailer`).
    #[derive(Clone, Copy, Eq)]
    pub struct AOSTrailer {
        /// Frame Error Control Field.
        fecf: u16 { get_fecf, set_fecf },
    }
}

fpp_struct! {
    /// One security-association-index to port-index mapping entry
    /// (`Svc::Ccsds::SaMapEntry`). Declared for parity; SDLS is out of scope.
    #[derive(Clone, Copy, Eq)]
    pub struct SaMapEntry {
        /// Security association index.
        security_association_index: u16 { get_security_association_index, set_security_association_index },
        /// Port index.
        port_index: FwIndexType { get_port_index, set_port_index },
    }
}

// ---------------------------------------------------------------------------
// Ports (Svc/Ccsds/Ports/Ports.fpp)
// ---------------------------------------------------------------------------

/// `Ccsds.ApidSequenceCount` — request or validate the sequence count of an
/// APID; returns a 16-bit count.
///
/// The `sequence_count` argument is the received count on the validation
/// port and unused (`0`) on the request port, mirroring the single FPP port
/// type used for both directions.
pub trait ApidSequenceCountPort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType, apid: Apid, sequence_count: u16) -> u16;
}

/// `Ccsds.ErrorNotify` — notify a listener of a framing/deframing error.
pub trait ErrorNotifyPort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType, error_code: FrameError);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::{Deserialize, Endianness, LinearBuffer, SerBuf, Serialize};

    /// Serialize `value` and return its bytes.
    fn ser(value: &dyn Serialize) -> Vec<u8> {
        let mut buf: LinearBuffer<32> = LinearBuffer::new();
        assert!(buf.serialize(value, Endianness::Big).is_ok());
        buf.as_slice().to_vec()
    }

    #[test]
    fn serialized_sizes_match_cpp() {
        assert_eq!(SpacePacketHeader::SERIALIZED_SIZE, 6);
        assert_eq!(TMHeader::SERIALIZED_SIZE, 6);
        assert_eq!(TMTrailer::SERIALIZED_SIZE, 2);
        assert_eq!(TCHeader::SERIALIZED_SIZE, 5);
        assert_eq!(TCTrailer::SERIALIZED_SIZE, 2);
        assert_eq!(AOSHeader::SERIALIZED_SIZE, 6);
        assert_eq!(MPduHeader::SERIALIZED_SIZE, 2);
        assert_eq!(AOSTrailer::SERIALIZED_SIZE, 2);
    }

    #[test]
    fn com_cfg_constants_match_cpp() {
        assert_eq!(SPACECRAFT_ID, 0x0044);
        assert_eq!(TM_FRAME_FIXED_SIZE, 1024);
        assert_eq!(AOS_MAX_FRAME_FIXED_SIZE, 1536);
        assert_eq!(AGGREGATION_SIZE, 1009);
    }

    /// Literal bytes for a telemetry Space Packet header: PVN 0, type 0, no
    /// secondary header, APID 0x0002 (LOG), unsegmented (0b11), count 5,
    /// data field 100 bytes.
    #[test]
    fn space_packet_header_literal_bytes() {
        let header = SpacePacketHeader::new(
            SpacePacketHeader::build_packet_identification(0, 0, false, 0x0002),
            SpacePacketHeader::build_packet_sequence_control(0b11, 5),
            SpacePacketHeader::length_token(100),
        );
        assert_eq!(ser(&header), vec![0x00, 0x02, 0xC0, 0x05, 0x00, 0x63]);
    }

    /// Every bit field of the identification word lands where the standard
    /// says: PVN 0b101, type 1, sec hdr 1, APID 0x2AA.
    #[test]
    fn space_packet_identification_bit_packing() {
        let word = SpacePacketHeader::build_packet_identification(0b101, 1, true, 0x2AA);
        assert_eq!(word, 0b1011_1010_1010_1010);
        let header = SpacePacketHeader::new(word, 0, 0);
        assert_eq!(header.pvn(), 0b101);
        assert_eq!(header.packet_type(), 1);
        assert!(header.has_sec_hdr());
        assert_eq!(header.apid_value(), 0x2AA);
        // Out-of-range inputs are masked, never bleeding into a neighbour.
        assert_eq!(
            SpacePacketHeader::build_packet_identification(0xFF, 0xFF, true, 0xFFFF),
            0xFFFF
        );
    }

    /// Sequence flags occupy bits `[15:14]` and the count the low 14 bits.
    #[test]
    fn space_packet_sequence_control_bit_packing() {
        assert_eq!(
            SpacePacketHeader::build_packet_sequence_control(0b01, 0x3FFF),
            0x7FFF
        );
        let header = SpacePacketHeader::new(0, 0xC001, 0);
        assert_eq!(header.sequence_flags(), 0b11);
        assert_eq!(header.sequence_count(), 1);
        // The count is masked to 14 bits, the flags to 2.
        assert_eq!(
            SpacePacketHeader::build_packet_sequence_control(0xFF, 0xFFFF),
            0xFFFF
        );
    }

    /// The length field is "octets minus one" in both directions, and the
    /// maximum token widens instead of wrapping.
    #[test]
    fn space_packet_data_length_semantics() {
        assert_eq!(SpacePacketHeader::length_token(1), 0);
        let header = SpacePacketHeader::new(0, 0, 0xFFFF);
        assert_eq!(header.data_field_length(), 65536);
        let header = SpacePacketHeader::new(0, 0, 0);
        assert_eq!(header.data_field_length(), 1);
    }

    /// Round trip through the wire keeps every raw word.
    #[test]
    fn space_packet_header_round_trip() {
        let bytes = [0x08u8, 0x01, 0x4A, 0xBC, 0x01, 0xFF];
        let mut buf: LinearBuffer<16> = LinearBuffer::new();
        assert!(
            buf.serialize_bytes(&bytes, fprime_fw::LengthMode::OmitLength, Endianness::Big)
                .is_ok()
        );
        let mut header = SpacePacketHeader::default();
        assert!(header.deserialize_from(&mut buf, Endianness::Big).is_ok());
        assert_eq!(header.packet_identification, 0x0801);
        assert!(header.has_sec_hdr());
        assert_eq!(header.apid_value(), 0x0001);
        assert_eq!(header.sequence_flags(), 0b01);
        assert_eq!(header.sequence_count(), 0x0ABC);
        assert_eq!(header.data_field_length(), 0x0200);
        assert_eq!(ser(&header), bytes.to_vec());
    }

    /// Literal bytes for the TM primary header the framer emits with the
    /// default context (vcId 1) on the first frame.
    #[test]
    fn tm_header_literal_bytes() {
        let header = TMHeader::new(
            TMHeader::build_global_vc_id(SPACECRAFT_ID, 1, false),
            0,
            0,
            TMHeader::build_data_field_status(false, false, false, 0b11, 0),
        );
        // 0b00 | 0b0001000100 (0x044) | 0b001 | 0b0 = 0x0442
        assert_eq!(ser(&header), vec![0x04, 0x42, 0x00, 0x00, 0x18, 0x00]);
        assert_eq!(header.frame_version(), 0);
        assert_eq!(header.spacecraft_id(), SPACECRAFT_ID);
        assert_eq!(header.vc_id(), 1);
        assert!(!header.ocf_flag());
        assert_eq!(header.segment_length_id(), 0b11);
        assert_eq!(header.first_header_pointer(), 0);
    }

    /// C++ gotcha: `vcId` is shifted without masking, so a value above 7
    /// corrupts the spacecraft ID field.
    #[test]
    fn tm_global_vc_id_does_not_mask_vc_id() {
        let word = TMHeader::build_global_vc_id(SPACECRAFT_ID, 8, false);
        assert_eq!(word, (8u16 << 1) | (SPACECRAFT_ID << 4));
        let header = TMHeader::new(word, 0, 0, 0);
        assert_eq!(header.spacecraft_id(), SPACECRAFT_ID + 1); // corrupted
        assert_eq!(header.vc_id(), 0);
    }

    /// The OCF flag is bit 0 and every data-field-status flag is distinct.
    #[test]
    fn tm_data_field_status_bit_packing() {
        assert_eq!(TMHeader::build_global_vc_id(0, 0, true), 0x0001);
        assert_eq!(
            TMHeader::build_data_field_status(true, false, false, 0, 0),
            0x8000
        );
        assert_eq!(
            TMHeader::build_data_field_status(false, true, false, 0, 0),
            0x4000
        );
        assert_eq!(
            TMHeader::build_data_field_status(false, false, true, 0, 0),
            0x2000
        );
        assert_eq!(
            TMHeader::build_data_field_status(false, false, false, 0b11, 0),
            0x1800
        );
        assert_eq!(
            TMHeader::build_data_field_status(false, false, false, 0, 0x7FF),
            0x07FF
        );
    }

    /// Literal bytes for the TC header the ground segment sends: bypass set,
    /// SCID 0x044, VCID 0, total frame length 20.
    #[test]
    fn tc_header_literal_bytes() {
        let header = TCHeader::new(
            TCHeader::build_flags_and_sc_id(true, false, SPACECRAFT_ID),
            TCHeader::build_vc_id_and_length(0, 20),
            0,
        );
        assert_eq!(ser(&header), vec![0x20, 0x44, 0x00, 0x13, 0x00]);
        assert_eq!(header.frame_version(), 0);
        assert!(header.bypass_flag());
        assert!(!header.control_command_flag());
        assert_eq!(header.spacecraft_id(), SPACECRAFT_ID);
        assert_eq!(header.vc_id(), 0);
        assert_eq!(header.total_frame_length(), 20);
    }

    /// The frame-detector token: TFVN 00, bypass 1, control 0, reserved 00,
    /// SCID 0x044 => 0x2044.
    #[test]
    fn tc_flags_and_sc_id_matches_frame_detector_token() {
        assert_eq!(
            TCHeader::build_flags_and_sc_id(true, false, SPACECRAFT_ID),
            0x2044
        );
        assert_eq!(TCHeader::build_flags_and_sc_id(false, true, 0x03FF), 0x13FF);
    }

    /// VCID occupies bits `[15:10]` and the length token the low 10 bits;
    /// the token is the TOTAL length minus one, maxing out at 1024.
    #[test]
    fn tc_vc_id_and_length_bit_packing() {
        assert_eq!(TCHeader::build_vc_id_and_length(0x3F, 1024), 0xFFFF);
        let header = TCHeader::new(0, 0xFFFF, 0);
        assert_eq!(header.vc_id(), 0x3F);
        assert_eq!(header.total_frame_length(), 1024);
        // vc_id is masked to 6 bits.
        assert_eq!(TCHeader::build_vc_id_and_length(0xFF, 1), 0xFC00);
    }

    /// Trailers are a bare big-endian u16.
    #[test]
    fn trailers_are_two_big_endian_bytes() {
        assert_eq!(ser(&TMTrailer::new(0x29B1)), vec![0x29, 0xB1]);
        assert_eq!(ser(&TCTrailer::new(0xBEEF)), vec![0xBE, 0xEF]);
        assert_eq!(ser(&AOSTrailer::new(0x0001)), vec![0x00, 0x01]);
    }

    /// AOS header and M_PDU header wire layout (parity declarations).
    #[test]
    fn aos_types_literal_bytes() {
        let header = AOSHeader::new(0x4123, 0x0012_3456);
        assert_eq!(ser(&header), vec![0x41, 0x23, 0x00, 0x12, 0x34, 0x56]);
        assert_eq!(
            MPduHeader::default().first_header_pointer,
            m_pdu_subfields::FHP_NO_PACKET_START
        );
        assert_eq!(ser(&MPduHeader::default()), vec![0xFF, 0xFF]);
    }

    #[test]
    fn frame_error_discriminants_match_cpp() {
        assert_eq!(FrameError::SpInvalidPacket as u8, 0);
        assert_eq!(FrameError::SpInvalidLength as u8, 1);
        assert_eq!(FrameError::TcInvalidScid as u8, 2);
        assert_eq!(FrameError::TcInvalidLength as u8, 3);
        assert_eq!(FrameError::TcInvalidVcid as u8, 4);
        assert_eq!(FrameError::TcInvalidCrc as u8, 5);
        assert_eq!(FrameError::AosInvalidScid as u8, 6);
        assert_eq!(FrameError::SdlsDecryptionFailure as u8, 13);
        assert_eq!(FrameError::NUM_CONSTANTS, 14);
        assert_eq!(ser(&FrameError::TcInvalidCrc), vec![5]);
    }

    #[test]
    fn sdls_and_tfvn_discriminants_match_cpp() {
        assert_eq!(SdlsStatus::Success as u8, 0);
        assert_eq!(SdlsStatus::KeyError as u8, 5);
        assert_eq!(Tfvn::TmTc as u8, 0);
        assert_eq!(Tfvn::Aos as u8, 1);
        assert_eq!(Tfvn::ProxOne as u8, 2);
        assert_eq!(Tfvn::Uslp as u8, 3);
        assert_eq!(Tfvn::default(), Tfvn::InvalidUninitialized);
    }

    /// The idle packet header the TM framer writes: APID 0x7FF, sequence
    /// flags 0b11, count 0.
    #[test]
    fn idle_packet_identification_is_all_ones_apid() {
        let ident = Apid::SppIdlePacket as u16;
        assert_eq!(ident, 0x07FF);
        let header = SpacePacketHeader::new(
            ident,
            SpacePacketHeader::build_packet_sequence_control(0b11, 0),
            10,
        );
        assert_eq!(ser(&header), vec![0x07, 0xFF, 0xC0, 0x00, 0x00, 0x0A]);
        assert_eq!(header.pvn(), 0);
        assert_eq!(header.packet_type(), 0);
        assert!(!header.has_sec_hdr());
    }
}
