//! Port of `Utils::Hash` (CRC32 implementation) and `Utils::HashBuffer`.
//!
//! C++ sources: `Utils/Hash/Crc32/Crc32.cpp`, `Utils/Hash/Crc32/HashImpl.cpp`,
//! `Utils/Hash/HashCommon.cpp` (the file-extension helpers),
//! `Utils/Hash/HashBuffer.hpp`, `Utils/Hash/HashBufferCommon.cpp`.
//! Analysis: `docs/cpp-analysis/utils-misc.md` (Utils::Hash section) and
//! `docs/cpp-analysis/svc-comms.md` (Utils::Hash CRC32 section).
//!
//! Algorithm: CRC-32/ISO-HDLC (IEEE 802.3) — reflected polynomial
//! `0xEDB88320`, init `0xFFFFFFFF`, final one's complement. `update` runs the
//! table-driven register update WITHOUT the final complement; `finalize`
//! returns `!register` (the standard CRC-32 value) without mutating state.

use fprime_config::FwSizeType;
use fprime_fw::serial::{
    Deserialize, Endianness, LinearBuffer, SerBuf, SerBufAny, Serialize, SerializeStatus,
};
use fprime_fw::{FwString, StringFormatStatus, fw_assert};

/// Size of a hash digest in bytes (C++ `HASH_DIGEST_LENGTH`).
pub const HASH_DIGEST_LENGTH: usize = 4;

/// Filename extension for hash sidecar files (C++ `HASH_EXTENSION_STRING`).
pub const HASH_EXTENSION_STRING: &str = ".CRC32";

/// The 256-entry Sarwate lookup table for the reflected IEEE 802.3
/// polynomial `0xEDB88320`, generated at compile time.
///
/// Equivalent to the C++ `crc32_ieee802_3_lookup0` table in
/// `Utils/Hash/Crc32/Crc32.cpp` (the slice-by-4 tables 1..3 are a pure
/// speed optimization; the byte-at-a-time result is identical).
const fn generate_crc32_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc = i as u32;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

const CRC32_TABLE: [u32; 256] = generate_crc32_table();

/// A 4-byte buffer holding a hash digest (port of `Utils::HashBuffer`,
/// a fixed 4-byte `Fw::LinearBufferBase`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct HashBuffer {
    buf: LinearBuffer<HASH_DIGEST_LENGTH>,
}

impl HashBuffer {
    /// Serialized size as a value: `[u16 size][digest bytes]`.
    pub const SERIALIZED_SIZE: usize = LinearBuffer::<HASH_DIGEST_LENGTH>::SERIALIZED_SIZE;

    /// A new, empty (zeroed) hash buffer.
    pub const fn new() -> Self {
        Self {
            buf: LinearBuffer::new(),
        }
    }

    /// Construct from raw digest bytes (C++ `HashBuffer(const U8*, FwSizeType)`).
    ///
    /// C++ parity: fw_asserts (crashes) if `data` exceeds the digest length.
    pub fn from_bytes(data: &[u8]) -> Self {
        let mut out = Self::new();
        let status = out.buf.set_buff(data);
        fw_assert!(status.is_ok(), status as i32);
        out
    }

    /// Fold digest bytes 0..4 MSB-first into a `u32`
    /// (C++ `HashBuffer::asBigEndianU32`).
    ///
    /// C++ parity: reads the raw storage array regardless of the current
    /// valid size (unset bytes are zero).
    pub fn as_big_endian_u32(&self) -> u32 {
        let mut result: u32 = 0;
        for &byte in &self.buf.bytes()[..HASH_DIGEST_LENGTH] {
            result = (result << 8) | u32::from(byte);
        }
        result
    }
}

// SerBufAny by delegation to the inner LinearBuffer (gives HashBuffer the
// full SerBuf provided-method surface, matching the C++ LinearBufferBase base
// class).
impl SerBufAny for HashBuffer {
    fn bytes(&self) -> &[u8] {
        self.buf.bytes()
    }
    fn bytes_mut(&mut self) -> &mut [u8] {
        self.buf.bytes_mut()
    }
    fn capacity(&self) -> usize {
        self.buf.capacity()
    }
    fn ser_loc(&self) -> usize {
        self.buf.ser_loc()
    }
    fn set_ser_loc(&mut self, loc: usize) {
        self.buf.set_ser_loc(loc);
    }
    fn deser_loc(&self) -> usize {
        self.buf.deser_loc()
    }
    fn set_deser_loc(&mut self, loc: usize) {
        self.buf.set_deser_loc(loc);
    }
    fn as_ser_buf_any(&mut self) -> &mut dyn SerBufAny {
        self
    }
}

impl Serialize for HashBuffer {
    /// Serialized as a value: `[u16 size][digest bytes]`.
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        self.buf.serialize_to(buf, e)
    }
    fn serialized_size(&self) -> usize {
        self.buf.serialized_size()
    }
}

impl Deserialize for HashBuffer {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        self.buf.deserialize_from(buf, e)
    }
}

/// Incremental CRC32 hash (port of `Utils::Hash`, CRC32 implementation).
///
/// The internal handle always holds the raw (un-complemented) CRC register.
/// Not thread safe (C++ parity).
#[derive(Debug, Clone)]
pub struct Hash {
    /// The raw CRC register (C++ `hash_handle`).
    handle: u32,
}

impl Hash {
    /// A new hash, initialized (C++ constructor calls `init()`).
    pub const fn new() -> Self {
        Self {
            handle: 0xFFFF_FFFF,
        }
    }

    /// Reset the register to `0xFFFFFFFF` (C++ `init`).
    pub fn init(&mut self) {
        self.handle = 0xFFFF_FFFF;
    }

    /// Run the reflected table update over `data` WITHOUT the final
    /// complement (C++ `update` -> `crc32_ieee802_3_update`).
    pub fn update(&mut self, data: &[u8]) {
        let mut crc = self.handle;
        for &byte in data {
            crc = (crc >> 8) ^ CRC32_TABLE[((crc ^ u32::from(byte)) & 0xFF) as usize];
        }
        self.handle = crc;
    }

    /// Return the standard (complemented) CRC-32 value `!register`
    /// (C++ `finalize(U32&)`). Does not mutate state.
    pub fn finalize(&self) -> u32 {
        !self.handle
    }

    /// Return the standard CRC-32 value serialized big-endian into a
    /// [`HashBuffer`] (C++ `finalize(HashBuffer&)`).
    pub fn finalize_buffer(&self) -> HashBuffer {
        let mut out = HashBuffer::new();
        // C++ parity: HashImpl.cpp asserts the serialize status.
        let status = out.serialize_u32_be(!self.handle);
        fw_assert!(status.is_ok(), status as i32);
        out
    }

    /// Store an already-complemented (standard) CRC-32 value: the register
    /// becomes `!value` (C++ `setHashValue(U32)`).
    pub fn set_hash_value(&mut self, value: u32) {
        self.handle = !value;
    }

    /// Store an already-complemented value from a hash buffer, reading a
    /// big-endian `u32` at the buffer's read cursor
    /// (C++ `setHashValue(HashBuffer&)` — note it advances the cursor).
    pub fn set_hash_value_buffer(&mut self, value: &mut HashBuffer) {
        let mut v: u32 = 0;
        // C++ parity: HashImpl.cpp asserts the deserialize status.
        let status = value.deserialize_u32_be(&mut v);
        fw_assert!(status.is_ok(), status as i32);
        self.handle = !v;
    }

    /// One-shot hash: init + update + finalize into a [`HashBuffer`]
    /// (C++ static `Hash::hash`).
    pub fn hash(data: &[u8]) -> HashBuffer {
        let mut h = Hash::new();
        h.update(data);
        h.finalize_buffer()
    }

    /// One-shot standard CRC-32 value of `data` (convenience over
    /// [`Hash::hash`] + [`HashBuffer::as_big_endian_u32`]).
    pub fn hash_u32(data: &[u8]) -> u32 {
        let mut h = Hash::new();
        h.update(data);
        h.finalize()
    }

    /// The file extension of the supported hash type, `".CRC32"`
    /// (C++ static `Hash::getFileExtensionString`, `HashCommon.cpp`).
    pub const fn get_file_extension_string() -> &'static str {
        HASH_EXTENSION_STRING
    }

    /// Write `<base_name><extension>` into `extended_name`
    /// (C++ static `Hash::addFileExtension`, `HashCommon.cpp`:
    /// `extendedName.format("%s%s", baseName.toChar(), HASH_EXTENSION_STRING)`).
    ///
    /// `base_name` is the content of the C++ `const Fw::ConstStringBase&`
    /// (pass `s.as_bytes()` for an [`FwString`]). Like `%s` on `toChar()`,
    /// it is read up to its first NUL byte.
    ///
    /// C++ parity (`Fw::stringFormat` over `vsnprintf`): when the result does
    /// not fit in `extended_name` (more than `N` bytes), the status is
    /// [`StringFormatStatus::Overflowed`] and `extended_name` holds the
    /// result TRUNCATED to `N` bytes — not an empty string; otherwise it is
    /// [`StringFormatStatus::Success`]. The other `FormatStatus` values are
    /// unreachable here (no null destination, null format string or
    /// `vsnprintf` error exist in this port).
    pub fn add_file_extension<const N: usize>(
        base_name: &[u8],
        extended_name: &mut FwString<N>,
    ) -> StringFormatStatus {
        let base_len = base_name
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(base_name.len());
        // `set_bytes`/`append` truncate byte-wise at capacity, as vsnprintf
        // does at `capacity - 1` characters plus the terminator.
        extended_name.set_bytes(&base_name[..base_len]);
        extended_name.append(HASH_EXTENSION_STRING);
        if base_len + HASH_EXTENSION_STRING.len() > FwString::<N>::max_length() {
            StringFormatStatus::Overflowed
        } else {
            StringFormatStatus::Success
        }
    }

    /// The length of the file extension string, without a terminator
    /// (C++ static `Hash::getFileExtensionLength`:
    /// `sizeof(HASH_EXTENSION_STRING) - 1`).
    pub const fn get_file_extension_length() -> FwSizeType {
        HASH_EXTENSION_STRING.len() as FwSizeType
    }
}

impl Default for Hash {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_check_vector() {
        // The canonical CRC-32/ISO-HDLC check value.
        assert_eq!(Hash::hash_u32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn crc32_empty_input_is_zero() {
        assert_eq!(Hash::hash_u32(b""), 0x0000_0000);
    }

    #[test]
    fn table_first_entries_match_cpp_lookup0() {
        // First rows of crc32_ieee802_3_lookup0 in Utils/Hash/Crc32/Crc32.cpp.
        assert_eq!(CRC32_TABLE[0], 0x0000_0000);
        assert_eq!(CRC32_TABLE[1], 0x7707_3096);
        assert_eq!(CRC32_TABLE[2], 0xEE0E_612C);
        assert_eq!(CRC32_TABLE[3], 0x9909_51BA);
        assert_eq!(CRC32_TABLE[128], 0xEDB8_8320);
        assert_eq!(CRC32_TABLE[255], 0x2D02_EF8D);
    }

    #[test]
    fn incremental_update_matches_one_shot() {
        let mut h = Hash::new();
        h.update(b"1234");
        h.update(b"5");
        h.update(b"6789");
        assert_eq!(h.finalize(), 0xCBF4_3926);
    }

    #[test]
    fn update_without_final_complement_finalize_complements() {
        let mut h = Hash::new();
        h.update(b"123456789");
        // The register is the un-complemented value; finalize applies !.
        assert_eq!(h.finalize(), 0xCBF4_3926);
        assert_eq!(!h.finalize(), !0xCBF4_3926u32);
        // finalize does not mutate state: calling twice yields the same value.
        assert_eq!(h.finalize(), 0xCBF4_3926);
    }

    #[test]
    fn finalize_buffer_is_big_endian_digest() {
        let mut h = Hash::new();
        h.update(b"123456789");
        let buf = h.finalize_buffer();
        // Wire format: standard CRC value serialized big-endian.
        assert_eq!(buf.bytes()[..4], [0xCB, 0xF4, 0x39, 0x26]);
        assert_eq!(buf.ser_loc(), 4);
        assert_eq!(buf.as_big_endian_u32(), 0xCBF4_3926);
    }

    #[test]
    fn set_hash_value_stores_complement_for_further_updates() {
        // Gotcha: setHashValue(v) stores ~v; the handle always holds the raw
        // register, so update() can continue from a restored standard value.
        let mut reference = Hash::new();
        reference.update(b"hello ");
        let midpoint = reference.finalize();
        reference.update(b"world");
        let expected = reference.finalize();

        let mut restored = Hash::new();
        restored.set_hash_value(midpoint);
        restored.update(b"world");
        assert_eq!(restored.finalize(), expected);
    }

    #[test]
    fn set_hash_value_buffer_roundtrip() {
        let mut h = Hash::new();
        h.update(b"abc");
        let mut digest = h.finalize_buffer();

        let mut restored = Hash::new();
        restored.set_hash_value_buffer(&mut digest);
        assert_eq!(restored.finalize(), Hash::hash_u32(b"abc"));
    }

    #[test]
    fn hash_buffer_equality_compares_content() {
        let a = HashBuffer::from_bytes(&[1, 2, 3, 4]);
        let b = HashBuffer::from_bytes(&[1, 2, 3, 4]);
        let c = HashBuffer::from_bytes(&[1, 2, 3, 5]);
        let short = HashBuffer::from_bytes(&[1, 2]);
        assert_eq!(a, b);
        assert_ne!(a, c);
        assert_ne!(a, short);
    }

    #[test]
    fn hash_buffer_serialize_wire_format() {
        use fprime_fw::ComBuffer;
        let digest = Hash::hash(b"123456789");
        let mut com = ComBuffer::new();
        assert!(com.serialize(&digest, Endianness::Big).is_ok());
        // [u16 size = 4][digest bytes big-endian]
        assert_eq!(com.as_slice(), &[0x00, 0x04, 0xCB, 0xF4, 0x39, 0x26]);
    }

    #[test]
    fn extension_and_digest_constants() {
        assert_eq!(HASH_EXTENSION_STRING, ".CRC32");
        assert_eq!(HASH_DIGEST_LENGTH, 4);
    }

    #[test]
    fn file_extension_string_and_length() {
        assert_eq!(Hash::get_file_extension_string(), HASH_EXTENSION_STRING);
        // sizeof(HASH_EXTENSION_STRING) - 1: the terminator is not counted.
        assert_eq!(Hash::get_file_extension_length(), 6);
    }

    #[test]
    fn add_file_extension_appends_extension() {
        let mut extended = fprime_fw::FileNameString::new();
        assert_eq!(
            Hash::add_file_extension(b"seq/file.bin", &mut extended),
            StringFormatStatus::Success
        );
        assert_eq!(extended, "seq/file.bin.CRC32");

        // The previous content is replaced, not appended to (format()).
        assert_eq!(
            Hash::add_file_extension(b"", &mut extended),
            StringFormatStatus::Success
        );
        assert_eq!(extended, ".CRC32");
    }

    #[test]
    fn add_file_extension_exact_fit_succeeds() {
        // 4 + 6 == capacity 10: fits exactly (vsnprintf needs 10 < 11).
        let mut extended = FwString::<10>::new();
        assert_eq!(
            Hash::add_file_extension(b"abcd", &mut extended),
            StringFormatStatus::Success
        );
        assert_eq!(extended, "abcd.CRC32");
    }

    #[test]
    fn add_file_extension_overflow_truncates() {
        // C++ parity: vsnprintf keeps the first capacity characters and
        // stringFormat reports OVERFLOWED; the name is NOT emptied.
        let mut extended = FwString::<10>::new();
        assert_eq!(
            Hash::add_file_extension(b"abcde", &mut extended),
            StringFormatStatus::Overflowed
        );
        assert_eq!(extended, "abcde.CRC3");

        // A base longer than the destination keeps only its prefix.
        assert_eq!(
            Hash::add_file_extension(b"0123456789AB", &mut extended),
            StringFormatStatus::Overflowed
        );
        assert_eq!(extended, "0123456789");
    }

    #[test]
    fn add_file_extension_stops_at_nul() {
        // `%s` on `toChar()` stops at the first NUL; only the prefix counts
        // toward the length.
        let mut extended = FwString::<10>::new();
        assert_eq!(
            Hash::add_file_extension(b"abcd\0zzzzzzzz", &mut extended),
            StringFormatStatus::Success
        );
        assert_eq!(extended, "abcd.CRC32");
    }
}
