//! `CFDP::Checksum` — the CCSDS File Delivery Protocol modular checksum.
//!
//! Port of `CFDP/Checksum/Checksum.{hpp,cpp}`. Analysis:
//! `docs/cpp-analysis/utils-misc.md` (§ "Fw::FilePacket + CFDP::Checksum").
//!
//! The checksum is a wrapping `u32` sum in which the byte at **file offset**
//! `o` contributes `byte << (8 * (3 - (o % 4)))` — i.e. the file is summed
//! as big-endian 32-bit words aligned to file offsets, with short first and
//! last words padded at their true offsets. Because the contribution depends
//! only on the absolute file offset, chunks may be checksummed in any order
//! and at any alignment and still produce the same value; this is what lets
//! `Svc::FileUplink` accumulate over out-of-order DATA packets and
//! `Svc::FileDownlink` accumulate as it reads.
//!
//! `Svc::FileUplink` compares its accumulated value against the one carried
//! in the `Fw::FilePacket` END packet and emits `BadChecksum(computed, read)`
//! on a mismatch.

/// The CFDP modular checksum accumulator (`CFDP::Checksum`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct Checksum {
    value: u32,
}

impl Checksum {
    /// A zeroed checksum (C++ default constructor).
    #[must_use]
    pub const fn new() -> Self {
        Self { value: 0 }
    }

    /// A checksum initialized to a stored value (C++ `Checksum(U32)`, used
    /// by `EndPacket::getChecksum`).
    #[must_use]
    pub const fn from_value(value: u32) -> Self {
        Self { value }
    }

    /// The accumulated value (C++ `getValue`).
    #[must_use]
    pub const fn get_value(&self) -> u32 {
        self.value
    }

    /// C++ `Checksum::update(data, offset, length)`: fold `data` into the
    /// checksum as the file bytes at `[file_offset, file_offset + len)`.
    ///
    /// Reproduces the three-phase C++ walk exactly — an unaligned prefix (up
    /// to the next 4-byte file-offset boundary), the whole aligned words in
    /// the middle, then an unaligned suffix — so an unaligned call yields the
    /// same value as the equivalent aligned calls.
    pub fn update(&mut self, data: &[u8], file_offset: u32) {
        let length = data.len();
        let mut index: usize = 0;

        // Add the first word unaligned if necessary.
        let offset_mod_4 = (file_offset % 4) as usize;
        if offset_mod_4 != 0 {
            let word_length = core::cmp::min(length, 4 - offset_mod_4);
            // C++ casts (offset + index) to U8 before taking % 4; 256 is a
            // multiple of 4, so the truncation cannot change the result.
            self.add_word_unaligned(
                &data[index..index + word_length],
                (file_offset.wrapping_add(index as u32) % 4) as u8,
            );
            index += word_length;
        }

        // Add the middle words aligned.
        let aligned_end = index + (((length - index) / 4) * 4);
        while index < aligned_end {
            self.add_word_aligned(&data[index..index + 4]);
            index += 4;
        }

        // Add the last word unaligned if necessary.
        if index < length {
            self.add_word_unaligned(
                &data[index..length],
                (file_offset.wrapping_add(index as u32) % 4) as u8,
            );
        }
    }

    /// C++ `addWordAligned`: four bytes at word offsets 0..=3.
    fn add_word_aligned(&mut self, word: &[u8]) {
        for (i, byte) in word.iter().enumerate().take(4) {
            self.add_byte_at_offset(*byte, i as u8);
        }
    }

    /// C++ `addWordUnaligned`: fewer than four bytes starting at word offset
    /// `position % 4`, wrapping back to 0 at the word boundary.
    fn add_word_unaligned(&mut self, word: &[u8], position: u8) {
        let mut offset = position % 4;
        for byte in word {
            self.add_byte_at_offset(*byte, offset);
            offset += 1;
            if offset == 4 {
                offset = 0;
            }
        }
    }

    /// C++ `addByteAtOffset`: `value += byte << (8 * (3 - offset))`, wrapping.
    pub fn add_byte_at_offset(&mut self, byte: u8, offset: u8) {
        debug_assert!(offset < 4);
        let addend = u32::from(byte) << (8 * (3 - u32::from(offset % 4)));
        self.value = self.value.wrapping_add(addend);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The reference definition, byte by byte at absolute file offsets.
    fn reference(data: &[u8], file_offset: u32) -> u32 {
        let mut value: u32 = 0;
        for (i, byte) in data.iter().enumerate() {
            let o = file_offset.wrapping_add(i as u32) % 4;
            value = value.wrapping_add(u32::from(*byte) << (8 * (3 - o)));
        }
        value
    }

    #[test]
    fn new_checksum_is_zero() {
        assert_eq!(Checksum::new().get_value(), 0);
        assert_eq!(Checksum::default().get_value(), 0);
        assert_eq!(Checksum::from_value(0xDEAD_BEEF).get_value(), 0xDEAD_BEEF);
    }

    #[test]
    fn aligned_single_word_is_the_big_endian_word() {
        let mut c = Checksum::new();
        c.update(&[0x01, 0x02, 0x03, 0x04], 0);
        assert_eq!(c.get_value(), 0x0102_0304);
    }

    #[test]
    fn aligned_two_words_sum() {
        let mut c = Checksum::new();
        c.update(&[0x01, 0x02, 0x03, 0x04, 0x10, 0x20, 0x30, 0x40], 0);
        assert_eq!(c.get_value(), 0x0102_0304u32.wrapping_add(0x1020_3040));
    }

    #[test]
    fn short_trailing_word_pads_at_its_true_offset() {
        // "abc" at offset 0 -> 0x61_62_63_00 (the missing byte contributes 0).
        let mut c = Checksum::new();
        c.update(b"abc", 0);
        assert_eq!(c.get_value(), 0x6162_6300);
    }

    #[test]
    fn unaligned_start_shifts_by_file_offset() {
        // One byte at file offset 1 lands in the second-most-significant byte.
        let mut c = Checksum::new();
        c.update(&[0xAB], 1);
        assert_eq!(c.get_value(), 0x00AB_0000);

        // Three bytes at file offset 3 straddle two words:
        // offset 3 -> 0x000000AA, offset 4 -> 0xBB000000, offset 5 -> 0x00CC0000
        let mut c = Checksum::new();
        c.update(&[0xAA, 0xBB, 0xCC], 3);
        assert_eq!(c.get_value(), 0x0000_00AAu32 + 0xBB00_0000 + 0x00CC_0000);
    }

    #[test]
    fn unaligned_prefix_middle_and_suffix_matches_reference() {
        let data: Vec<u8> = (0u8..=200).collect();
        for offset in 0u32..8 {
            for len in [0usize, 1, 2, 3, 4, 5, 7, 8, 9, 63, 64, 65, 200] {
                let mut c = Checksum::new();
                c.update(&data[..len], offset);
                assert_eq!(
                    c.get_value(),
                    reference(&data[..len], offset),
                    "offset {offset} len {len}"
                );
            }
        }
    }

    #[test]
    fn chunked_updates_equal_a_single_update() {
        let data: Vec<u8> = (0u8..=255).cycle().take(1000).collect();
        let mut whole = Checksum::new();
        whole.update(&data, 0);

        // Split at every conceivable alignment; the file offset drives the
        // shift, so chunking must not change the result.
        for chunk in [1usize, 2, 3, 5, 7, 499, 512] {
            let mut parts = Checksum::new();
            let mut offset = 0u32;
            for piece in data.chunks(chunk) {
                parts.update(piece, offset);
                offset += piece.len() as u32;
            }
            assert_eq!(parts.get_value(), whole.get_value(), "chunk {chunk}");
        }
    }

    #[test]
    fn out_of_order_chunks_produce_the_same_value() {
        let data: Vec<u8> = (0u8..=255).cycle().take(300).collect();
        let mut whole = Checksum::new();
        whole.update(&data, 0);

        let mut shuffled = Checksum::new();
        // Deliberately out of order and unaligned, as an uplink may arrive.
        shuffled.update(&data[100..255], 100);
        shuffled.update(&data[0..100], 0);
        shuffled.update(&data[255..300], 255);
        assert_eq!(shuffled.get_value(), whole.get_value());
    }

    #[test]
    fn update_with_empty_slice_is_a_no_op() {
        let mut c = Checksum::from_value(0x1234_5678);
        c.update(&[], 0);
        c.update(&[], 3);
        assert_eq!(c.get_value(), 0x1234_5678);
    }

    #[test]
    fn value_wraps_modulo_2_to_the_32() {
        let mut c = Checksum::from_value(0xFFFF_FFFF);
        c.update(&[0x00, 0x00, 0x00, 0x02], 0);
        assert_eq!(c.get_value(), 1);
    }

    #[test]
    fn add_byte_at_offset_matches_the_shift_table() {
        for (offset, expected) in [
            (0u8, 0xFF00_0000u32),
            (1, 0x00FF_0000),
            (2, 0x0000_FF00),
            (3, 0x0000_00FF),
        ] {
            let mut c = Checksum::new();
            c.add_byte_at_offset(0xFF, offset);
            assert_eq!(c.get_value(), expected);
        }
    }

    #[test]
    fn equality_is_by_value() {
        assert_eq!(Checksum::from_value(7), Checksum::from_value(7));
        assert_ne!(Checksum::from_value(7), Checksum::from_value(8));
    }
}
