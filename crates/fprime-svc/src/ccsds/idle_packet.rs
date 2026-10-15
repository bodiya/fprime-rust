//! # IdlePacket — port of `Svc::Ccsds::Utils::IdlePacket`
//!
//! C++ sources: `Svc/Ccsds/Utils/IdlePacket.{hpp,cpp}`.
//!
//! The Space Packet Protocol idle packet (APID `0x7FF`) shared by the CCSDS
//! framers and the `ComAggregator` to fill unused frame data-field space:
//! a 6-byte primary header (PVN 0, packet type 0, no secondary header, APID
//! `SPP_IDLE_PACKET`, sequence flags `0b11`, count 0, length token
//! `size - 7`) followed by [`DATA_PATTERN`] bytes.

use crate::ccsds::types::{SpacePacketHeader, space_packet_subfields};
use fprime_fw::{Apid, Endianness, SerBuf, SerBufAny, SerializeStatus, fw_assert};

/// Fill pattern for idle packet data (`IdlePacket::DATA_PATTERN`).
pub const DATA_PATTERN: u8 = 0x44;

/// Minimum idle packet size: header plus one byte of idle data
/// (`IdlePacket::MIN_SIZE`).
pub const MIN_SIZE: usize = SpacePacketHeader::SERIALIZED_SIZE + 1;

/// Serialize an idle packet of exactly `size` bytes (header and idle data)
/// into `serializer` (`IdlePacket::serialize`).
///
/// Asserts `size >= MIN_SIZE` and that the length token fits a `u16` (C++
/// parity). Stops at, and returns, the first non-OK serialization status.
pub fn serialize(serializer: &mut dyn SerBufAny, size: usize) -> SerializeStatus {
    fw_assert!(size >= MIN_SIZE, size as i32);
    // The length token is the number of data bytes minus one.
    let length_token = size - SpacePacketHeader::SERIALIZED_SIZE - 1;
    fw_assert!(length_token <= usize::from(u16::MAX), length_token as i32);

    let header = SpacePacketHeader::new(
        Apid::SppIdlePacket.as_repr(),
        // Sequence flags 0b11 (unsegmented), unused sequence count.
        0x3 << space_packet_subfields::SEQ_FLAGS_OFFSET,
        length_token as u16,
    );
    let mut status = serializer.serialize(&header, Endianness::Big);
    let mut i = SpacePacketHeader::SERIALIZED_SIZE;
    while status.is_ok() && i < size {
        status = serializer.serialize_u8_be(DATA_PATTERN);
        i += 1;
    }
    status
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::{LinearBuffer, SerBuf};

    /// The minimum idle packet, byte for byte.
    #[test]
    fn minimum_idle_packet_bytes() {
        let mut buf: LinearBuffer<16> = LinearBuffer::new();
        assert!(serialize(&mut buf, MIN_SIZE).is_ok());
        assert_eq!(
            buf.as_slice(),
            &[0x07, 0xFF, 0xC0, 0x00, 0x00, 0x00, DATA_PATTERN]
        );
    }

    /// A larger idle packet carries `size - 7` as its length token.
    #[test]
    fn length_token_is_size_minus_seven() {
        let mut buf: LinearBuffer<64> = LinearBuffer::new();
        assert!(serialize(&mut buf, 40).is_ok());
        assert_eq!(buf.get_size(), 40);
        assert_eq!(&buf.as_slice()[..6], &[0x07, 0xFF, 0xC0, 0x00, 0x00, 33]);
        assert!(buf.as_slice()[6..].iter().all(|&b| b == DATA_PATTERN));
    }

    /// Running out of room returns the serialization status.
    #[test]
    fn no_room_is_reported() {
        let mut buf: LinearBuffer<8> = LinearBuffer::new();
        assert_eq!(serialize(&mut buf, 9), SerializeStatus::NoRoomLeft);
    }

    /// Below the minimum size is a programmer error.
    #[test]
    #[should_panic(expected = "Assert:")]
    fn below_minimum_asserts() {
        let mut buf: LinearBuffer<8> = LinearBuffer::new();
        let _ = serialize(&mut buf, MIN_SIZE - 1);
    }
}
