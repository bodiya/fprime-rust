//! Fixed-capacity truncating strings.
//!
//! Port of `Fw/Types/StringBase` / `StringTemplate<N>` and the framework
//! string aliases. Assignment and append truncate silently; the wire format
//! is a `FwSizeStoreType` (u16) length prefix + bytes with NO NUL
//! terminator; deserialization rejects lengths above the capacity, leaving
//! the prior content in place. See `docs/cpp-analysis/fw-types.md`.
//!
//! C++ parity quirks reproduced here:
//! - `ConstStringBase::serializeTo` / `StringBase::deserializeFrom` never
//!   forward their `Endianness` mode to the buffer calls, so the u16 length
//!   prefix is ALWAYS big-endian; the `Endianness` parameters below are
//!   ignored the same way.
//! - Content is C-string based: `length()` is a bounded NUL scan, so bytes
//!   at and after the first `0x00` are invisible. Every content-setting
//!   path here commits only the bytes before the first NUL.

use crate::serial::{
    Deserialize, Endianness, LengthMode, SerBuf, SerBufAny, Serialize, SerializeStatus,
};
use fprime_config::{
    FILE_NAME_STRING_SIZE, FW_CMD_STRING_MAX_SIZE, FW_FIXED_LENGTH_STRING_SIZE,
    FW_LOG_STRING_MAX_SIZE, FW_LOG_TEXT_BUFFER_SIZE, FW_OBJ_NAME_BUFFER_SIZE,
    FW_PARAM_STRING_MAX_SIZE, FW_TLM_STRING_MAX_SIZE, FwSizeStoreType,
};
use std::fmt;

/// Effective C-string length of `bytes`: index of the first NUL, or the full
/// slice length (the C++ `StringUtils::string_length` bounded scan).
fn c_str_len(bytes: &[u8]) -> usize {
    bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len())
}

/// Fixed-capacity truncating string holding at most `N` bytes
/// (port of `Fw::StringTemplate<N>`, whose max length is also `N`).
///
/// Content is stored as raw bytes: like the C++ classes, deserialization can
/// produce arbitrary (non-UTF-8) bytes, so `as_str` is fallible and
/// `Display` renders lossily.
#[derive(Debug, Clone)]
pub struct FwString<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

impl<const N: usize> FwString<N> {
    /// Size when serialized at full capacity: `N + sizeof(FwSizeStoreType)`
    /// (the C++ `STATIC_SERIALIZED_SIZE` used to size autocoded buffers).
    pub const SERIALIZED_SIZE: usize = N + size_of::<FwSizeStoreType>();

    /// A new empty string.
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
        }
    }

    /// Assign from `s`, silently truncating to `N` bytes (C++ `operator=`).
    /// Truncation is byte-based like the C++ code and may cut a UTF-8
    /// character; the stored value is bytes, so this is safe here.
    pub fn set(&mut self, s: &str) {
        self.set_bytes(s.as_bytes());
    }

    /// Assign from raw bytes, silently truncating to `N` bytes. C++ parity:
    /// `operator=` copies via `string_copy`, which stops at the first NUL of
    /// the source, so bytes at and after a `0x00` are dropped.
    pub fn set_bytes(&mut self, src: &[u8]) {
        let n = c_str_len(src).min(N);
        self.bytes[..n].copy_from_slice(&src[..n]);
        self.len = n;
    }

    /// Append `s`, silently truncating at capacity (C++ `operator+=`).
    pub fn append(&mut self, s: &str) {
        self.append_bytes(s.as_bytes());
    }

    /// Append raw bytes, silently truncating at capacity. C++ parity:
    /// `operator+=` appends via `strncat`, which stops at the first NUL of
    /// the source.
    pub fn append_bytes(&mut self, src: &[u8]) {
        let room = N - self.len;
        let n = c_str_len(src).min(room);
        self.bytes[self.len..self.len + n].copy_from_slice(&src[..n]);
        self.len += n;
    }

    /// Reset to the empty string.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Current length in bytes.
    pub fn len(&self) -> usize {
        self.len
    }

    /// True when empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Maximum storable length (`maxLength()`; == `N`).
    pub const fn max_length() -> usize {
        N
    }

    /// The content bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    /// The content as `&str` when it is valid UTF-8.
    pub fn as_str(&self) -> Option<&str> {
        std::str::from_utf8(self.as_bytes()).ok()
    }

    /// Replace the content with formatted text, truncating at capacity
    /// (the C++ `format`/`vformat` path via `core::fmt`).
    pub fn format(&mut self, args: fmt::Arguments<'_>) {
        self.clear();
        // fmt::Write on FwString never errors; truncation is silent
        let _ = fmt::Write::write_fmt(self, args);
    }

    /// Size when serialized: `sizeof(FwSizeStoreType) + len`.
    pub fn serialized_size(&self) -> usize {
        size_of::<FwSizeStoreType>() + self.len
    }

    /// Serialize with the length truncated to `max_len` first
    /// (C++ `serializeTo(buffer, maxLength)`, used e.g. to cap event string
    /// arguments at `FW_LOG_STRING_MAX_SIZE`).
    ///
    /// C++ parity: the endianness mode is ignored — `ConstStringBase`
    /// never forwards it, so the u16 prefix is always big-endian.
    pub fn serialize_to_truncated(
        &self,
        buf: &mut dyn SerBufAny,
        max_len: usize,
        _e: Endianness,
    ) -> SerializeStatus {
        let n = self.len.min(max_len);
        buf.serialize_bytes(&self.bytes[..n], LengthMode::IncludeLength, Endianness::Big)
    }

    /// Size when serialized truncated to `max_len`.
    pub fn serialized_truncated_size(&self, max_len: usize) -> usize {
        size_of::<FwSizeStoreType>() + self.len.min(max_len)
    }
}

impl<const N: usize> Default for FwString<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> From<&str> for FwString<N> {
    fn from(s: &str) -> Self {
        let mut out = Self::new();
        out.set(s);
        out
    }
}

impl<const N: usize> PartialEq for FwString<N> {
    fn eq(&self, other: &Self) -> bool {
        self.as_bytes() == other.as_bytes()
    }
}
impl<const N: usize> Eq for FwString<N> {}

impl<const N: usize> PartialEq<&str> for FwString<N> {
    fn eq(&self, other: &&str) -> bool {
        self.as_bytes() == other.as_bytes()
    }
}

impl<const N: usize> fmt::Display for FwString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", String::from_utf8_lossy(self.as_bytes()))
    }
}

/// Appends with silent truncation and never reports an error, matching the
/// C++ truncating `format` semantics.
impl<const N: usize> fmt::Write for FwString<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.append(s);
        Ok(())
    }
}

impl<const N: usize> Serialize for FwString<N> {
    /// Wire format: `[u16 length][length bytes]`, no NUL terminator.
    /// C++ parity: `ConstStringBase::serializeTo` never forwards its
    /// endianness mode, so the u16 prefix is always big-endian and the
    /// `Endianness` argument is ignored.
    fn serialize_to(&self, buf: &mut dyn SerBufAny, _e: Endianness) -> SerializeStatus {
        buf.serialize_bytes(self.as_bytes(), LengthMode::IncludeLength, Endianness::Big)
    }
    fn serialized_size(&self) -> usize {
        FwString::serialized_size(self)
    }
}

impl<const N: usize> Deserialize for FwString<N> {
    /// Reads `[u16 length][bytes]`. A stored length above the capacity or
    /// the remaining bytes is `DeserSizeMismatch` with the prefix consumed
    /// and the prior content kept (C++ approximately: the failure path never
    /// copies, it only re-NUL-terminates its local buffer).
    ///
    /// C++ parity: `StringBase::deserializeFrom` never forwards its
    /// endianness mode, so the u16 prefix is always read big-endian and the
    /// `Endianness` argument is ignored. The full stored count is consumed
    /// from `buf`, but the committed content stops at the first NUL byte —
    /// the C++ copy is NUL-terminated storage whose `length()` is a bounded
    /// scan, so bytes after an interior `0x00` are invisible.
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, _e: Endianness) -> SerializeStatus {
        let mut scratch = [0u8; N];
        let mut len = N;
        let status = buf.deserialize_bytes(
            &mut scratch,
            &mut len,
            LengthMode::IncludeLength,
            Endianness::Big,
        );
        if status == SerializeStatus::Ok {
            let n = c_str_len(&scratch[..len]);
            self.bytes[..n].copy_from_slice(&scratch[..n]);
            self.len = n;
        }
        status
    }
}

/// Object name string (`Fw::ObjectName`, capacity `FW_OBJ_NAME_BUFFER_SIZE`).
pub type ObjectName = FwString<FW_OBJ_NAME_BUFFER_SIZE>;
/// Command string argument (`Fw::CmdStringArg`).
pub type CmdStringArg = FwString<FW_CMD_STRING_MAX_SIZE>;
/// Event log string argument (`Fw::LogStringArg`).
pub type LogStringArg = FwString<FW_LOG_STRING_MAX_SIZE>;
/// Text log string (`Fw::TextLogString`).
pub type TextLogString = FwString<FW_LOG_TEXT_BUFFER_SIZE>;
/// Telemetry string argument (`Fw::TlmString`).
pub type TlmString = FwString<FW_TLM_STRING_MAX_SIZE>;
/// Parameter string argument (`Fw::ParamString`).
pub type ParamString = FwString<FW_PARAM_STRING_MAX_SIZE>;
/// File name string (`Fw::FileNameString`).
pub type FileNameString = FwString<FILE_NAME_STRING_SIZE>;
/// General-purpose string (`Fw::String`).
pub type FwDefaultString = FwString<FW_FIXED_LENGTH_STRING_SIZE>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::LinearBuffer;

    #[test]
    fn wire_format_is_u16_length_plus_bytes_no_nul() {
        let s: FwString<40> = "ABC".into();
        let mut buf = LinearBuffer::<64>::new();
        assert_eq!(
            s.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0x00, 0x03, b'A', b'B', b'C']);
        assert_eq!(Serialize::serialized_size(&s), 5);
    }

    #[test]
    fn assignment_truncates_silently() {
        let long = "x".repeat(45);
        let s: FwString<40> = long.as_str().into();
        assert_eq!(s.len(), 40);
        assert_eq!(s.as_bytes(), "x".repeat(40).as_bytes());
    }

    #[test]
    fn append_truncates_at_capacity() {
        let mut s: FwString<8> = "abcde".into();
        s.append("fghij");
        assert_eq!(s, "abcdefgh");
        assert_eq!(s.len(), 8);
    }

    #[test]
    fn roundtrip() {
        let s: FwString<40> = "hello world".into();
        let mut buf = LinearBuffer::<64>::new();
        assert_eq!(
            s.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        let mut out = FwString::<40>::new();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(out, s);
    }

    #[test]
    fn deserialize_rejects_length_over_capacity_keeping_prior_content() {
        // 10-byte string into an 8-byte-capacity target
        let src: FwString<40> = "0123456789".into();
        let mut buf = LinearBuffer::<64>::new();
        assert_eq!(
            src.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );

        let mut out: FwString<8> = "keepme".into();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserSizeMismatch
        );
        assert_eq!(out, "keepme");
        // C++ parity: the u16 prefix was consumed by the failed attempt
        assert_eq!(buf.deser_loc(), 2);
    }

    #[test]
    fn deserialize_rejects_length_over_remaining_bytes() {
        let mut buf = LinearBuffer::<64>::new();
        // prefix says 5 bytes but only 2 follow
        assert_eq!(buf.serialize_u16_be(5), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u8_be(b'a'), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u8_be(b'b'), SerializeStatus::Ok);
        let mut out: FwString<40> = "old".into();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserSizeMismatch
        );
        assert_eq!(out, "old");
    }

    #[test]
    fn format_truncates() {
        let mut s = FwString::<10>::new();
        s.format(format_args!("value={}", 1234567890u64));
        assert_eq!(s, "value=1234");
        // reformat replaces content
        s.format(format_args!("x={}", 1));
        assert_eq!(s, "x=1");
    }

    #[test]
    fn serialize_truncated_caps_length() {
        let s: FwString<200> = "abcdefgh".into();
        let mut buf = LinearBuffer::<32>::new();
        assert_eq!(
            s.serialize_to_truncated(&mut buf, 4, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0x00, 0x04, b'a', b'b', b'c', b'd']);
        assert_eq!(s.serialized_truncated_size(4), 6);
    }

    #[test]
    fn length_prefix_is_big_endian_regardless_of_mode() {
        // C++ parity: ConstStringBase::serializeTo / StringBase::deserializeFrom
        // never forward their endianness mode; the u16 prefix stays big-endian
        // even when the caller passes Little.
        let s: FwString<40> = "ABC".into();
        let mut buf = LinearBuffer::<64>::new();
        assert_eq!(
            s.serialize_to(&mut buf, Endianness::Little),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0x00, 0x03, b'A', b'B', b'C']);

        let mut out = FwString::<40>::new();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Little),
            SerializeStatus::Ok
        );
        assert_eq!(out, "ABC");
    }

    #[test]
    fn truncated_serialize_prefix_is_big_endian_regardless_of_mode() {
        let s: FwString<200> = "abcdefgh".into();
        let mut buf = LinearBuffer::<32>::new();
        assert_eq!(
            s.serialize_to_truncated(&mut buf, 4, Endianness::Little),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0x00, 0x04, b'a', b'b', b'c', b'd']);
    }

    #[test]
    fn deserialized_content_stops_at_first_nul() {
        // C++ parity: length() is a bounded NUL scan, so wire content
        // [00 03]['a' 00 'b'] deserializes to an effective length of 1 and
        // re-serializes as [00 01]['a'], while the full stored count is
        // still consumed from the source buffer.
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(
            buf.serialize_bytes(
                &[b'a', 0x00, b'b'],
                LengthMode::IncludeLength,
                Endianness::Big
            ),
            SerializeStatus::Ok
        );
        let mut out = FwString::<8>::new();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(out.len(), 1);
        assert_eq!(out.as_bytes(), b"a");
        assert_eq!(
            buf.deserialize_size_left(),
            0,
            "all 3 stored bytes consumed"
        );

        let mut re = LinearBuffer::<16>::new();
        assert_eq!(
            out.serialize_to(&mut re, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(re.as_slice(), &[0x00, 0x01, b'a']);
    }

    #[test]
    fn set_and_append_stop_at_first_nul() {
        // C++ parity: operator= (string_copy) and operator+= (strncat) both
        // stop at the source's first NUL.
        let mut s = FwString::<16>::new();
        s.set_bytes(b"a\x00b");
        assert_eq!(s.len(), 1);
        assert_eq!(s, "a");
        s.append_bytes(b"cd\x00e");
        assert_eq!(s, "acd");

        let mut t = FwString::<16>::new();
        t.set_bytes(b"\x00xyz");
        assert!(t.is_empty());
    }

    #[test]
    fn empty_string_wire_format() {
        let s = FwString::<40>::new();
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(
            s.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0x00, 0x00]);
    }

    #[test]
    fn alias_capacities() {
        assert_eq!(ObjectName::max_length(), 80);
        assert_eq!(CmdStringArg::max_length(), 40);
        assert_eq!(LogStringArg::max_length(), 200);
        assert_eq!(TextLogString::max_length(), 256);
        assert_eq!(TlmString::max_length(), 40);
        assert_eq!(ParamString::max_length(), 40);
        assert_eq!(FileNameString::max_length(), 240);
        assert_eq!(FwDefaultString::max_length(), 256);
        assert_eq!(CmdStringArg::SERIALIZED_SIZE, 42);
    }
}
