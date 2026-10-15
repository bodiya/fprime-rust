//! # SdlsAuthMask — port of `Svc::Ccsds::Utils::SdlsTcAuthMask` /
//! `SdlsTmAuthMask`
//!
//! C++ source: `Svc/Ccsds/Utils/SdlsAuthMask.hpp`.
//!
//! AES-GCM authenticates, but does not encrypt, a block of frame fields
//! supplied as additional authenticated data (AAD). SDLS uses it to bind a
//! frame to its virtual channel and security association: the block is a
//! masked copy of the frame primary header (fields that differ between the
//! two ends zeroed), then the SPI verbatim, then a zeroed 12-byte IV field.

/// AAD for an SDLS-protected TC (uplink) transfer frame: 19 bytes — the
/// masked 5-byte primary header, the SPI, then a zeroed IV field.
///
/// Assumes no TC segment header; a link using one inserts an extra `0xFF`
/// mask byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdlsTcAuthMask {
    /// The AAD bytes.
    pub bytes: [u8; SdlsTcAuthMask::SIZE],
}

impl SdlsTcAuthMask {
    /// TC primary header size.
    pub const TC_PRIMARY_HEADER_SIZE: usize = 5;
    /// Security Parameter Index size.
    pub const SPI_SIZE: usize = 2;
    /// IV field size.
    pub const IV_SIZE: usize = 12;
    /// Total AAD size.
    pub const SIZE: usize = Self::TC_PRIMARY_HEADER_SIZE + Self::SPI_SIZE + Self::IV_SIZE;
    /// Byte 2 holds the 6-bit VCID in bits 7..2, above the frame length.
    pub const VCID_MASK: u8 = 0xFC;
    /// Index of the VCID byte.
    pub const VCID_BYTE_INDEX: usize = 2;
    /// Shift of the VCID within its byte.
    pub const VCID_SHIFT: u32 = 2;

    /// Build the AAD for `vc_id` and `security_association_index`.
    #[must_use]
    pub fn new(vc_id: u8, security_association_index: u16) -> Self {
        let mut bytes = [0u8; Self::SIZE];
        bytes[Self::VCID_BYTE_INDEX] = (vc_id << Self::VCID_SHIFT) & Self::VCID_MASK;
        bytes[Self::TC_PRIMARY_HEADER_SIZE..Self::TC_PRIMARY_HEADER_SIZE + Self::SPI_SIZE]
            .copy_from_slice(&security_association_index.to_be_bytes());
        Self { bytes }
    }
}

/// AAD for an SDLS-protected TM (downlink) transfer frame: 20 bytes — the
/// masked 6-byte primary header, the SPI, then a zeroed IV field.
///
/// Assumes no transfer frame secondary header; a link using one inserts
/// that many additional zero bytes before the SPI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdlsTmAuthMask {
    /// The AAD bytes.
    pub bytes: [u8; SdlsTmAuthMask::SIZE],
}

impl SdlsTmAuthMask {
    /// TM primary header size.
    pub const TM_PRIMARY_HEADER_SIZE: usize = 6;
    /// Security Parameter Index size.
    pub const SPI_SIZE: usize = 2;
    /// IV field size.
    pub const IV_SIZE: usize = 12;
    /// Total AAD size.
    pub const SIZE: usize = Self::TM_PRIMARY_HEADER_SIZE + Self::SPI_SIZE + Self::IV_SIZE;
    /// Byte 1 holds the 3-bit VCID in bits 3..1, between the low spacecraft
    /// ID bits and the OCF flag.
    pub const VCID_MASK: u8 = 0x0E;
    /// Index of the VCID byte.
    pub const VCID_BYTE_INDEX: usize = 1;
    /// Shift of the VCID within its byte.
    pub const VCID_SHIFT: u32 = 1;

    /// Build the AAD for `vc_id` and `security_association_index`.
    #[must_use]
    pub fn new(vc_id: u8, security_association_index: u16) -> Self {
        let mut bytes = [0u8; Self::SIZE];
        bytes[Self::VCID_BYTE_INDEX] = (vc_id << Self::VCID_SHIFT) & Self::VCID_MASK;
        bytes[Self::TM_PRIMARY_HEADER_SIZE..Self::TM_PRIMARY_HEADER_SIZE + Self::SPI_SIZE]
            .copy_from_slice(&security_association_index.to_be_bytes());
        Self { bytes }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tc_mask_layout() {
        let m = SdlsTcAuthMask::new(0x3F, 0x1234);
        assert_eq!(SdlsTcAuthMask::SIZE, 19);
        let mut expected = [0u8; 19];
        expected[2] = 0xFC;
        expected[5] = 0x12;
        expected[6] = 0x34;
        assert_eq!(m.bytes, expected);
        // VCID bits above 6 are masked off.
        assert_eq!(SdlsTcAuthMask::new(0x41, 0).bytes[2], 0x04);
    }

    #[test]
    fn tm_mask_layout() {
        let m = SdlsTmAuthMask::new(5, 0xABCD);
        assert_eq!(SdlsTmAuthMask::SIZE, 20);
        let mut expected = [0u8; 20];
        expected[1] = 0x0A;
        expected[6] = 0xAB;
        expected[7] = 0xCD;
        assert_eq!(m.bytes, expected);
        // Only 3 VCID bits survive.
        assert_eq!(SdlsTmAuthMask::new(0x0F, 0).bytes[1], 0x0E);
    }
}
