//! Polymorphic value type.
//!
//! Port of `Fw/Types/PolyType.{hpp,cpp}`: a tagged union whose wire format
//! is `[tag: FwEnumStoreType i32][value in its natural encoding]`. Tag
//! values are fixed to the all-widths-enabled C++ numbering (0..=12); the
//! C++ pointer member is stored as a `u64` (the 64-bit
//! `PlatformPointerCastType` wire width).

use crate::fw_try;
use crate::serial::{Deserialize, Endianness, SerBuf, SerBufAny, Serialize, SerializeStatus};
use fprime_config::FwEnumStoreType;

/// A polymorphic value (`Fw::PolyType`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum PolyType {
    /// No value set (`TYPE_NOTYPE`); serializing it writes the tag and then
    /// fails with `FormatError` (C++ parity).
    #[default]
    NoType,
    /// u8 value.
    U8(u8),
    /// i8 value.
    I8(i8),
    /// u16 value.
    U16(u16),
    /// i16 value.
    I16(i16),
    /// u32 value.
    U32(u32),
    /// i32 value.
    I32(i32),
    /// u64 value.
    U64(u64),
    /// i64 value.
    I64(i64),
    /// f32 value.
    F32(f32),
    /// f64 value.
    F64(f64),
    /// bool value (0xFF/0x00 on the wire).
    Bool(bool),
    /// Pointer value, stored as the 64-bit pointer-cast integer.
    Ptr(u64),
}

impl PolyType {
    /// Maximum size when serialized: tag + widest value
    /// (the C++ `SERIALIZED_SIZE` enum, an upper bound).
    pub const SERIALIZED_SIZE: usize = size_of::<FwEnumStoreType>() + 8;

    /// The exact C++ tag for this variant (`TYPE_*` values, all widths on).
    pub fn wire_tag(&self) -> FwEnumStoreType {
        match self {
            PolyType::NoType => 0,
            PolyType::U8(_) => 1,
            PolyType::I8(_) => 2,
            PolyType::U16(_) => 3,
            PolyType::I16(_) => 4,
            PolyType::U32(_) => 5,
            PolyType::I32(_) => 6,
            PolyType::U64(_) => 7,
            PolyType::I64(_) => 8,
            PolyType::F32(_) => 9,
            PolyType::F64(_) => 10,
            PolyType::Bool(_) => 11,
            PolyType::Ptr(_) => 12,
        }
    }
}

impl Serialize for PolyType {
    /// C++ parity: the tag is written FIRST; serializing `NoType` then
    /// returns `FormatError` with the 4 tag bytes already committed.
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        fw_try!(buf.serialize_i32(self.wire_tag(), e));
        match *self {
            PolyType::NoType => SerializeStatus::FormatError,
            PolyType::U8(v) => buf.serialize_u8(v, e),
            PolyType::I8(v) => buf.serialize_i8(v, e),
            PolyType::U16(v) => buf.serialize_u16(v, e),
            PolyType::I16(v) => buf.serialize_i16(v, e),
            PolyType::U32(v) => buf.serialize_u32(v, e),
            PolyType::I32(v) => buf.serialize_i32(v, e),
            PolyType::U64(v) => buf.serialize_u64(v, e),
            PolyType::I64(v) => buf.serialize_i64(v, e),
            PolyType::F32(v) => buf.serialize_f32(v, e),
            PolyType::F64(v) => buf.serialize_f64(v, e),
            PolyType::Bool(v) => buf.serialize_bool(v, e),
            PolyType::Ptr(v) => buf.serialize_u64(v, e),
        }
    }

    /// Actual wire size for this variant: 4 (tag) + the value's width.
    fn serialized_size(&self) -> usize {
        let value = match self {
            PolyType::NoType => 0,
            PolyType::U8(_) | PolyType::I8(_) | PolyType::Bool(_) => 1,
            PolyType::U16(_) | PolyType::I16(_) => 2,
            PolyType::U32(_) | PolyType::I32(_) | PolyType::F32(_) => 4,
            PolyType::U64(_) | PolyType::I64(_) | PolyType::F64(_) | PolyType::Ptr(_) => 8,
        };
        size_of::<FwEnumStoreType>() + value
    }
}

impl Deserialize for PolyType {
    /// An unknown (or `NOTYPE`) tag is `DeserFormatError`. Rust deviation
    /// from C++: `self` is left unmodified on any failure (the C++ code
    /// overwrites its tag field before rejecting; a tagged enum cannot
    /// represent that half-state).
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let mut tag: FwEnumStoreType = 0;
        fw_try!(buf.deserialize_i32(&mut tag, e));
        macro_rules! read {
            ($variant:ident, $ty:ty, $deser:ident) => {{
                let mut v: $ty = Default::default();
                fw_try!(buf.$deser(&mut v, e));
                PolyType::$variant(v)
            }};
        }
        let value = match tag {
            1 => read!(U8, u8, deserialize_u8),
            2 => read!(I8, i8, deserialize_i8),
            3 => read!(U16, u16, deserialize_u16),
            4 => read!(I16, i16, deserialize_i16),
            5 => read!(U32, u32, deserialize_u32),
            6 => read!(I32, i32, deserialize_i32),
            7 => read!(U64, u64, deserialize_u64),
            8 => read!(I64, i64, deserialize_i64),
            9 => read!(F32, f32, deserialize_f32),
            10 => read!(F64, f64, deserialize_f64),
            11 => read!(Bool, bool, deserialize_bool),
            12 => read!(Ptr, u64, deserialize_u64),
            // TYPE_NOTYPE (0) has no switch case in C++ either -> format error
            _ => return SerializeStatus::DeserFormatError,
        };
        *self = value;
        SerializeStatus::Ok
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::LinearBuffer;

    fn ser(p: &PolyType) -> (Vec<u8>, SerializeStatus) {
        let mut buf = LinearBuffer::<16>::new();
        let status = p.serialize_to(&mut buf, Endianness::Big);
        (buf.as_slice().to_vec(), status)
    }

    #[test]
    fn tags_match_cpp() {
        assert_eq!(PolyType::NoType.wire_tag(), 0);
        assert_eq!(PolyType::U8(0).wire_tag(), 1);
        assert_eq!(PolyType::I8(0).wire_tag(), 2);
        assert_eq!(PolyType::U16(0).wire_tag(), 3);
        assert_eq!(PolyType::I16(0).wire_tag(), 4);
        assert_eq!(PolyType::U32(0).wire_tag(), 5);
        assert_eq!(PolyType::I32(0).wire_tag(), 6);
        assert_eq!(PolyType::U64(0).wire_tag(), 7);
        assert_eq!(PolyType::I64(0).wire_tag(), 8);
        assert_eq!(PolyType::F32(0.0).wire_tag(), 9);
        assert_eq!(PolyType::F64(0.0).wire_tag(), 10);
        assert_eq!(PolyType::Bool(false).wire_tag(), 11);
        assert_eq!(PolyType::Ptr(0).wire_tag(), 12);
    }

    #[test]
    fn wire_format_tag_plus_natural_value() {
        let (bytes, st) = ser(&PolyType::U8(0x42));
        assert_eq!(st, SerializeStatus::Ok);
        assert_eq!(bytes, [0, 0, 0, 1, 0x42]);

        let (bytes, st) = ser(&PolyType::U16(0x1234));
        assert_eq!(st, SerializeStatus::Ok);
        assert_eq!(bytes, [0, 0, 0, 3, 0x12, 0x34]);

        let (bytes, st) = ser(&PolyType::I32(-2));
        assert_eq!(st, SerializeStatus::Ok);
        assert_eq!(bytes, [0, 0, 0, 6, 0xFF, 0xFF, 0xFF, 0xFE]);

        let (bytes, st) = ser(&PolyType::F32(1.0));
        assert_eq!(st, SerializeStatus::Ok);
        assert_eq!(bytes, [0, 0, 0, 9, 0x3F, 0x80, 0x00, 0x00]);

        let (bytes, st) = ser(&PolyType::F64(1.0));
        assert_eq!(st, SerializeStatus::Ok);
        assert_eq!(
            bytes,
            [0, 0, 0, 10, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        );

        let (bytes, st) = ser(&PolyType::Bool(true));
        assert_eq!(st, SerializeStatus::Ok);
        assert_eq!(bytes, [0, 0, 0, 11, 0xFF], "bool serializes 0xFF, not 1");

        let (bytes, st) = ser(&PolyType::Ptr(0x1122_3344_5566_7788));
        assert_eq!(st, SerializeStatus::Ok);
        assert_eq!(
            bytes,
            [0, 0, 0, 12, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
            "ptr serializes at 64-bit pointer width"
        );
    }

    #[test]
    fn notype_serialize_writes_tag_then_format_error() {
        // C++ parity: the tag goes out before the default-case rejection
        let (bytes, st) = ser(&PolyType::NoType);
        assert_eq!(st, SerializeStatus::FormatError);
        assert_eq!(bytes, [0, 0, 0, 0], "tag bytes committed before the error");
    }

    #[test]
    fn deserialize_roundtrip_all_variants() {
        let values = [
            PolyType::U8(1),
            PolyType::I8(-1),
            PolyType::U16(2),
            PolyType::I16(-2),
            PolyType::U32(3),
            PolyType::I32(-3),
            PolyType::U64(4),
            PolyType::I64(-4),
            PolyType::F32(1.5),
            PolyType::F64(-2.5),
            PolyType::Bool(true),
            PolyType::Bool(false),
            PolyType::Ptr(0xDEAD_BEEF),
        ];
        for v in values {
            let mut buf = LinearBuffer::<16>::new();
            assert_eq!(
                v.serialize_to(&mut buf, Endianness::Big),
                SerializeStatus::Ok
            );
            let mut out = PolyType::NoType;
            assert_eq!(
                out.deserialize_from(&mut buf, Endianness::Big),
                SerializeStatus::Ok
            );
            assert_eq!(out, v);
        }
    }

    #[test]
    fn deserialize_rejects_unknown_and_notype_tags() {
        for tag in [0i32, 13, -1] {
            let mut buf = LinearBuffer::<16>::new();
            assert_eq!(buf.serialize_i32_be(tag), SerializeStatus::Ok);
            let mut out = PolyType::U8(7);
            assert_eq!(
                out.deserialize_from(&mut buf, Endianness::Big),
                SerializeStatus::DeserFormatError
            );
            assert_eq!(out, PolyType::U8(7), "Rust keeps self unmodified");
        }
    }

    #[test]
    fn serialized_size_is_tag_plus_value_width() {
        assert_eq!(PolyType::U8(0).serialized_size(), 5);
        assert_eq!(PolyType::U16(0).serialized_size(), 6);
        assert_eq!(PolyType::F64(0.0).serialized_size(), 12);
        assert_eq!(PolyType::Ptr(0).serialized_size(), 12);
        assert_eq!(PolyType::SERIALIZED_SIZE, 12);
    }
}
