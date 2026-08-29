//! Com packet types and comms-stack context.
//!
//! Port of `Fw/Com/ComPacket.{hpp,cpp}` and `default/config/ComCfg.fpp`
//! (the `Apid` / `Pvn` enums and the `FrameContext` FPP struct). See
//! `docs/cpp-analysis/fw-services.md`.

use crate::enums::fpp_enum;
use crate::fw_try;
use crate::serial::{Deserialize, Endianness, SerBuf, SerBufAny, Serialize, SerializeStatus};
use fprime_config::FwIndexType;

fpp_enum! {
    /// Packet descriptor / CCSDS APID (`ComCfg::Apid`, repr
    /// `FwPacketDescriptorType` = u16). APIDs are 11 bits, max 0x7FF.
    pub enum ComPacketType : u16 { serialize_u16, deserialize_u16 } {
        /// Command packet type - incoming.
        FwPacketCommand = 0x0000,
        /// Telemetry packet type - outgoing.
        FwPacketTelem = 0x0001,
        /// Log type - outgoing.
        FwPacketLog = 0x0002,
        /// File type - incoming and outgoing.
        FwPacketFile = 0x0003,
        /// Packetized telemetry packet type.
        FwPacketPacketizedTlm = 0x0004,
        /// Data Product packet type.
        FwPacketDp = 0x0005,
        /// F Prime idle packet.
        FwPacketIdle = 0x0006,
        /// Parameter value type - outgoing.
        FwPacketParam = 0x0007,
        /// F Prime handshake.
        FwPacketHand = 0x00FE,
        /// F Prime unknown packet.
        FwPacketUnknown = 0x00FF,
        /// Per the Space Packet standard, all 1s (11 bits) is reserved for
        /// idle packets.
        SppIdlePacket = 0x07FF,
        /// Anything of equal or higher value is invalid and should not be
        /// used (the FPP enum default).
        InvalidUninitialized = 0x0800,
    }
    default InvalidUninitialized
}

/// The `ComCfg::Apid` name for the same enum.
pub type Apid = ComPacketType;

fpp_enum! {
    /// CCSDS Packet Version Number (`ComCfg::Pvn`, repr u8; 3 bits with only
    /// two valid values).
    pub enum Pvn : u8 { serialize_u8, deserialize_u8 } {
        /// Fully featured CCSDS Space Packet Protocol.
        SpacePacketProtocol = 0x0,
        /// Bare-bones CCSDS Encapsulation Packet Protocol.
        EncapsulationPacketProtocol = 0x7,
        /// Anything of equal or higher value is invalid (the FPP default).
        InvalidUninitialized = 0x8,
    }
    default InvalidUninitialized
}

/// Reserved SA-index sentinel meaning "unset" (`ComCfg::SaIndexUnset`).
pub const SA_INDEX_UNSET: u16 = 0xFFFF;

/// Context info passed between components during framing/deframing
/// (`ComCfg::FrameContext` FPP struct). Serializes its fields in declaration
/// order (13 bytes total).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameContext {
    /// Queue index used by the ComQueue; other components shall not modify.
    pub com_queue_index: FwIndexType,
    /// 11-bit APID in CCSDS.
    pub apid: Apid,
    /// Secondary header flag for a SpacePacketFramer.
    pub has_sec_hdr: bool,
    /// 2-bit sequence flags (0b00=continuation, 0b01=first, 0b10=last,
    /// 0b11=unsegmented).
    pub sequence_flags: u8,
    /// 14-bit sequence count, incremented per APID.
    pub sequence_count: u16,
    /// 6-bit virtual channel ID (AOS, TC, TM protocols).
    pub vc_id: u8,
    /// Packet Version Number (AOS deframing packet-type identification).
    pub pvn: Pvn,
    /// Flag to an AOS framer that this packet's frame should be sent ASAP.
    pub send_now: bool,
    /// Security Association index (set by SDLS deframers, read by framers).
    pub sa_index: u16,
}

impl FrameContext {
    /// On-wire size: i16 + u16 + 1 + 1 + u16 + 1 + 1 + 1 + u16 = 13 bytes.
    pub const SERIALIZED_SIZE: usize = 13;
}

impl Default for FrameContext {
    /// The FPP struct defaults from `ComCfg.fpp`.
    fn default() -> Self {
        Self {
            com_queue_index: 0,
            apid: Apid::FwPacketUnknown,
            has_sec_hdr: false,
            sequence_flags: 0x3,
            sequence_count: 0,
            vc_id: 1,
            pvn: Pvn::InvalidUninitialized,
            send_now: false,
            sa_index: SA_INDEX_UNSET,
        }
    }
}

impl Serialize for FrameContext {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        fw_try!(buf.serialize_i16(self.com_queue_index, e));
        fw_try!(self.apid.serialize_to(buf, e));
        fw_try!(buf.serialize_bool(self.has_sec_hdr, e));
        fw_try!(buf.serialize_u8(self.sequence_flags, e));
        fw_try!(buf.serialize_u16(self.sequence_count, e));
        fw_try!(buf.serialize_u8(self.vc_id, e));
        fw_try!(self.pvn.serialize_to(buf, e));
        fw_try!(buf.serialize_bool(self.send_now, e));
        buf.serialize_u16(self.sa_index, e)
    }
    fn serialized_size(&self) -> usize {
        Self::SERIALIZED_SIZE
    }
}

impl Deserialize for FrameContext {
    /// Deserializes into a temporary and commits only on full success
    /// (enum members are validated per FPP rules).
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let mut tmp = FrameContext::default();
        fw_try!(buf.deserialize_i16(&mut tmp.com_queue_index, e));
        fw_try!(tmp.apid.deserialize_from(buf, e));
        fw_try!(buf.deserialize_bool(&mut tmp.has_sec_hdr, e));
        fw_try!(buf.deserialize_u8(&mut tmp.sequence_flags, e));
        fw_try!(buf.deserialize_u16(&mut tmp.sequence_count, e));
        fw_try!(buf.deserialize_u8(&mut tmp.vc_id, e));
        fw_try!(tmp.pvn.deserialize_from(buf, e));
        fw_try!(buf.deserialize_bool(&mut tmp.send_now, e));
        fw_try!(buf.deserialize_u16(&mut tmp.sa_index, e));
        *self = tmp;
        SerializeStatus::Ok
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::LinearBuffer;

    #[test]
    fn apid_discriminants_match_cpp() {
        assert_eq!(ComPacketType::FwPacketCommand as u16, 0x0000);
        assert_eq!(ComPacketType::FwPacketTelem as u16, 0x0001);
        assert_eq!(ComPacketType::FwPacketLog as u16, 0x0002);
        assert_eq!(ComPacketType::FwPacketFile as u16, 0x0003);
        assert_eq!(ComPacketType::FwPacketPacketizedTlm as u16, 0x0004);
        assert_eq!(ComPacketType::FwPacketDp as u16, 0x0005);
        assert_eq!(ComPacketType::FwPacketIdle as u16, 0x0006);
        assert_eq!(ComPacketType::FwPacketParam as u16, 0x0007);
        assert_eq!(ComPacketType::FwPacketHand as u16, 0x00FE);
        assert_eq!(ComPacketType::FwPacketUnknown as u16, 0x00FF);
        assert_eq!(ComPacketType::SppIdlePacket as u16, 0x07FF);
        assert_eq!(ComPacketType::InvalidUninitialized as u16, 0x0800);
        assert_eq!(Apid::default(), Apid::InvalidUninitialized);
    }

    #[test]
    fn apid_serializes_as_u16() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(
            ComPacketType::SppIdlePacket.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0x07, 0xFF]);
    }

    #[test]
    fn pvn_values_and_default() {
        assert_eq!(Pvn::SpacePacketProtocol as u8, 0);
        assert_eq!(Pvn::EncapsulationPacketProtocol as u8, 7);
        assert_eq!(Pvn::InvalidUninitialized as u8, 8);
        assert_eq!(Pvn::default(), Pvn::InvalidUninitialized);
    }

    #[test]
    fn frame_context_default_wire_format() {
        let ctx = FrameContext::default();
        let mut buf = LinearBuffer::<32>::new();
        assert_eq!(
            ctx.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(
            buf.as_slice(),
            &[
                0x00, 0x00, // comQueueIndex = 0 (i16)
                0x00, 0xFF, // apid = FW_PACKET_UNKNOWN
                0x00, // hasSecHdr = false
                0x03, // sequenceFlags = 0b11 unsegmented
                0x00, 0x00, // sequenceCount = 0
                0x01, // vcId = 1
                0x08, // pvn = INVALID_UNINITIALIZED
                0x00, // sendNow = false
                0xFF, 0xFF, // saIndex = SaIndexUnset
            ]
        );
        assert_eq!(FrameContext::SERIALIZED_SIZE, 13);
        assert_eq!(ctx.serialized_size(), 13);
    }

    #[test]
    fn frame_context_roundtrip() {
        let ctx = FrameContext {
            com_queue_index: -1,
            apid: Apid::FwPacketTelem,
            has_sec_hdr: true,
            sequence_flags: 0x1,
            sequence_count: 0x3FFF,
            vc_id: 5,
            pvn: Pvn::SpacePacketProtocol,
            send_now: true,
            sa_index: 2,
        };
        let mut buf = LinearBuffer::<32>::new();
        assert_eq!(
            ctx.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        let mut out = FrameContext::default();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(out, ctx);
    }

    #[test]
    fn frame_context_deserialize_rejects_invalid_enum_leaving_self_unmodified() {
        let mut buf = LinearBuffer::<32>::new();
        // valid i16 index, then an APID value that is not declared (0x0300)
        assert_eq!(buf.serialize_i16_be(0), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u16_be(0x0300), SerializeStatus::Ok);
        let mut ctx = FrameContext {
            com_queue_index: 9,
            ..FrameContext::default()
        };
        assert_eq!(
            ctx.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserFormatError
        );
        assert_eq!(ctx.com_queue_index, 9, "commit only on full success");
    }
}
