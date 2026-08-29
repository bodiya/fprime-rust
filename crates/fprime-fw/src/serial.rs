//! The F Prime serialization core.
//!
//! Port of `Fw/Types/Serializable.{hpp,cpp}` (`SerialBufferBase` /
//! `LinearBufferBase` / `LinearBufferTemplate`) and the buffer type aliases.
//! See `docs/cpp-analysis/fw-types.md` for the exact C++ semantics being
//! reproduced; the cursor rules and status distinctions here are normative
//! wire behavior that everything else in the framework trusts.

use fprime_config::{
    FW_CMD_ARG_BUFFER_MAX_SIZE, FW_COM_BUFFER_MAX_SIZE, FW_LOG_BUFFER_MAX_SIZE,
    FW_PARAM_BUFFER_MAX_SIZE, FW_SERIALIZE_FALSE_VALUE, FW_SERIALIZE_TRUE_VALUE,
    FW_TLM_BUFFER_MAX_SIZE, FwSizeStoreType, FwSizeType,
};

/// Serialization status codes; exact C++ discriminants (`Fw::SerializeStatus`).
///
/// APIs return this directly (not `Result`) to keep 1:1 porting of C++ status
/// flow; note `DiscardedExisting` is a success-with-note, not an error.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerializeStatus {
    /// Serialization/deserialization operation was successful.
    Ok = 0,
    /// Data was the wrong format (e.g. wrong packet type).
    FormatError = 1,
    /// No room left in the serialization buffer to serialize data.
    NoRoomLeft = 2,
    /// Deserialization buffer was empty when trying to read more data.
    DeserBufferEmpty = 3,
    /// Deserialization data had incorrect values (e.g. enum out of range).
    DeserFormatError = 4,
    /// Data was left in the buffer, but not enough to deserialize.
    DeserSizeMismatch = 5,
    /// Deserialized type ID didn't match.
    DeserTypeMismatch = 6,
    /// Attempted to deserialize into an immutable value; kept for parity.
    DeserImmutable = 7,
    /// Deserialization data contained invalid data.
    DeserInvalidData = 8,
    /// Serialization discarded existing data (drop-oldest queues); NOT an error.
    DiscardedExisting = 9,
}

impl SerializeStatus {
    /// True when the status is [`SerializeStatus::Ok`].
    #[must_use = "the success flag should be checked"]
    pub fn is_ok(self) -> bool {
        self == SerializeStatus::Ok
    }
}

/// Byte order for a serialize/deserialize call (`Fw::Endianness`).
/// F Prime is big-endian by default everywhere.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Endianness {
    /// MSB first (the F Prime / GDS wire default).
    #[default]
    Big,
    /// LSB first.
    Little,
}

/// Whether a raw-byte region carries a `FwSizeStoreType` (u16) length prefix
/// (`Fw::Serialization::t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LengthMode {
    /// Write/read a u16 length prefix before the bytes (the C++ default).
    #[default]
    IncludeLength,
    /// Raw bytes only; the count is known out-of-band.
    OmitLength,
}

/// Early-return a non-[`SerializeStatus::Ok`] status from the enclosing
/// function (the moral equivalent of the C++ `if (stat != FW_SERIALIZE_OK)
/// return stat;` ladder).
#[macro_export]
macro_rules! fw_try {
    ($e:expr) => {
        match $e {
            $crate::serial::SerializeStatus::Ok => {}
            other => return other,
        }
    };
}

// ---------------------------------------------------------------------------
// Storage contract
// ---------------------------------------------------------------------------

/// Object-safe storage core of a serialization buffer: a flat byte array with
/// a write cursor (`ser_loc`, equals the current size) and a read cursor
/// (`deser_loc`). Implementors supply storage; all encoding logic lives in
/// the blanket [`SerBuf`] implementation.
pub trait SerBufAny {
    /// Full backing storage (capacity bytes).
    fn bytes(&self) -> &[u8];
    /// Full backing storage, mutable.
    fn bytes_mut(&mut self) -> &mut [u8];
    /// Total capacity in bytes.
    fn capacity(&self) -> usize;
    /// Current write cursor == number of valid bytes.
    fn ser_loc(&self) -> usize;
    /// Set the write cursor (used by the provided methods only).
    fn set_ser_loc(&mut self, loc: usize);
    /// Current read cursor.
    fn deser_loc(&self) -> usize;
    /// Set the read cursor (used by the provided methods only).
    fn set_deser_loc(&mut self, loc: usize);
    /// Upcast to a `dyn SerBufAny` (implementors return `self`).
    fn as_ser_buf_any(&mut self) -> &mut dyn SerBufAny;
}

// Core write: room check, copy, advance write cursor, and — C++ parity —
// reset the read cursor to 0 on EVERY successful write.
fn write_raw(b: &mut dyn SerBufAny, data: &[u8]) -> SerializeStatus {
    let ser = b.ser_loc();
    // C++ room check `serLoc + n - 1 >= capacity` == `serLoc + n > capacity`.
    if ser + data.len() > b.capacity() {
        return SerializeStatus::NoRoomLeft;
    }
    b.bytes_mut()[ser..ser + data.len()].copy_from_slice(data);
    b.set_ser_loc(ser + data.len());
    b.set_deser_loc(0);
    SerializeStatus::Ok
}

// Core read: BUFFER_EMPTY only when the read cursor sits exactly at the write
// cursor; any partial remainder is SIZE_MISMATCH (C++ parity).
fn read_raw(b: &mut dyn SerBufAny, dest: &mut [u8]) -> SerializeStatus {
    let ser = b.ser_loc();
    let deser = b.deser_loc();
    if ser == deser {
        return SerializeStatus::DeserBufferEmpty;
    }
    if ser - deser < dest.len() {
        return SerializeStatus::DeserSizeMismatch;
    }
    dest.copy_from_slice(&b.bytes()[deser..deser + dest.len()]);
    b.set_deser_loc(deser + dest.len());
    SerializeStatus::Ok
}

macro_rules! prim_rw {
    ($(#[$m:meta])* $ty:ty, $ser:ident, $ser_be:ident, $deser:ident, $deser_be:ident) => {
        $(#[$m])*
        fn $ser(&mut self, v: $ty, e: Endianness) -> SerializeStatus {
            let bytes = match e {
                Endianness::Big => v.to_be_bytes(),
                Endianness::Little => v.to_le_bytes(),
            };
            write_raw(self.as_ser_buf_any(), &bytes)
        }
        /// Big-endian convenience wrapper.
        fn $ser_be(&mut self, v: $ty) -> SerializeStatus {
            self.$ser(v, Endianness::Big)
        }
        $(#[$m])*
        fn $deser(&mut self, v: &mut $ty, e: Endianness) -> SerializeStatus {
            let mut bytes = [0u8; size_of::<$ty>()];
            let status = read_raw(self.as_ser_buf_any(), &mut bytes);
            if status == SerializeStatus::Ok {
                *v = match e {
                    Endianness::Big => <$ty>::from_be_bytes(bytes),
                    Endianness::Little => <$ty>::from_le_bytes(bytes),
                };
            }
            status
        }
        /// Big-endian convenience wrapper.
        fn $deser_be(&mut self, v: &mut $ty) -> SerializeStatus {
            self.$deser(v, Endianness::Big)
        }
    };
}

/// The serialization buffer API (port of `Fw::SerialBufferBase` /
/// `LinearBufferBase`): every method is provided over the [`SerBufAny`]
/// storage contract via a blanket impl, so any storage — owned
/// ([`LinearBuffer`]) or borrowed ([`ExtBuf`]) — gets the identical codec.
///
/// Cursor rules (C++ parity, normative):
/// - every successful write advances `ser_loc` and **resets `deser_loc` to 0**;
/// - read with `deser_loc == ser_loc` yields [`SerializeStatus::DeserBufferEmpty`];
///   fewer remaining bytes than needed yields [`SerializeStatus::DeserSizeMismatch`];
/// - [`SerBuf::reset_ser`] zeroes both cursors; [`SerBuf::reset_deser`] only
///   the read cursor; [`SerBuf::serialize_skip`] does NOT reset the read cursor.
pub trait SerBuf: SerBufAny {
    prim_rw!(
        /// 8-bit unsigned; endianness ignored (single byte).
        u8, serialize_u8, serialize_u8_be, deserialize_u8, deserialize_u8_be
    );
    prim_rw!(
        /// 8-bit signed; endianness ignored (single byte).
        i8, serialize_i8, serialize_i8_be, deserialize_i8, deserialize_i8_be
    );
    prim_rw!(
        /// 16-bit unsigned integer.
        u16, serialize_u16, serialize_u16_be, deserialize_u16, deserialize_u16_be
    );
    prim_rw!(
        /// 16-bit signed integer (two's complement bytes).
        i16, serialize_i16, serialize_i16_be, deserialize_i16, deserialize_i16_be
    );
    prim_rw!(
        /// 32-bit unsigned integer.
        u32, serialize_u32, serialize_u32_be, deserialize_u32, deserialize_u32_be
    );
    prim_rw!(
        /// 32-bit signed integer (two's complement bytes).
        i32, serialize_i32, serialize_i32_be, deserialize_i32, deserialize_i32_be
    );
    prim_rw!(
        /// 64-bit unsigned integer.
        u64, serialize_u64, serialize_u64_be, deserialize_u64, deserialize_u64_be
    );
    prim_rw!(
        /// 64-bit signed integer (two's complement bytes).
        i64, serialize_i64, serialize_i64_be, deserialize_i64, deserialize_i64_be
    );

    /// IEEE-754 f32, bit-cast then integer endianness rules.
    fn serialize_f32(&mut self, v: f32, e: Endianness) -> SerializeStatus {
        self.serialize_u32(v.to_bits(), e)
    }
    /// Big-endian convenience wrapper.
    fn serialize_f32_be(&mut self, v: f32) -> SerializeStatus {
        self.serialize_f32(v, Endianness::Big)
    }
    /// IEEE-754 f32 decode.
    fn deserialize_f32(&mut self, v: &mut f32, e: Endianness) -> SerializeStatus {
        let mut bits = 0u32;
        let status = self.deserialize_u32(&mut bits, e);
        if status == SerializeStatus::Ok {
            *v = f32::from_bits(bits);
        }
        status
    }
    /// Big-endian convenience wrapper.
    fn deserialize_f32_be(&mut self, v: &mut f32) -> SerializeStatus {
        self.deserialize_f32(v, Endianness::Big)
    }
    /// IEEE-754 f64, bit-cast then integer endianness rules.
    fn serialize_f64(&mut self, v: f64, e: Endianness) -> SerializeStatus {
        self.serialize_u64(v.to_bits(), e)
    }
    /// Big-endian convenience wrapper.
    fn serialize_f64_be(&mut self, v: f64) -> SerializeStatus {
        self.serialize_f64(v, Endianness::Big)
    }
    /// IEEE-754 f64 decode.
    fn deserialize_f64(&mut self, v: &mut f64, e: Endianness) -> SerializeStatus {
        let mut bits = 0u64;
        let status = self.deserialize_u64(&mut bits, e);
        if status == SerializeStatus::Ok {
            *v = f64::from_bits(bits);
        }
        status
    }
    /// Big-endian convenience wrapper.
    fn deserialize_f64_be(&mut self, v: &mut f64) -> SerializeStatus {
        self.deserialize_f64(v, Endianness::Big)
    }

    /// bool: one byte, `0xFF` = true / `0x00` = false.
    fn serialize_bool(&mut self, v: bool, _e: Endianness) -> SerializeStatus {
        let byte = if v {
            FW_SERIALIZE_TRUE_VALUE
        } else {
            FW_SERIALIZE_FALSE_VALUE
        };
        write_raw(self.as_ser_buf_any(), &[byte])
    }
    /// Big-endian convenience wrapper (endianness is irrelevant for bool).
    fn serialize_bool_be(&mut self, v: bool) -> SerializeStatus {
        self.serialize_bool(v, Endianness::Big)
    }
    /// Strict bool decode: any byte other than `0xFF`/`0x00` is
    /// [`SerializeStatus::DeserFormatError`] and the cursor does NOT advance
    /// (C++ parity).
    fn deserialize_bool(&mut self, v: &mut bool, _e: Endianness) -> SerializeStatus {
        let ser = self.ser_loc();
        let deser = self.deser_loc();
        if ser == deser {
            return SerializeStatus::DeserBufferEmpty;
        }
        // one byte always fits when non-empty, so no SIZE_MISMATCH branch
        match self.bytes()[deser] {
            FW_SERIALIZE_TRUE_VALUE => *v = true,
            FW_SERIALIZE_FALSE_VALUE => *v = false,
            _ => return SerializeStatus::DeserFormatError,
        }
        self.set_deser_loc(deser + 1);
        SerializeStatus::Ok
    }
    /// Big-endian convenience wrapper (endianness is irrelevant for bool).
    fn deserialize_bool_be(&mut self, v: &mut bool) -> SerializeStatus {
        self.deserialize_bool(v, Endianness::Big)
    }

    /// Serialize a raw byte region. With [`LengthMode::IncludeLength`] a
    /// `FwSizeStoreType` (u16) prefix is written first, and — C++ parity —
    /// the prefix value is the length **silently truncated** to u16 (no range
    /// check) while all `data.len()` bytes are still copied. Note the prefix
    /// is committed before the room check for the body, so a body that does
    /// not fit leaves the prefix in the buffer (exact C++ behavior).
    fn serialize_bytes(&mut self, data: &[u8], mode: LengthMode, e: Endianness) -> SerializeStatus {
        if mode == LengthMode::IncludeLength {
            // C++: static_cast<FwSizeStoreType>(length) — silent truncation.
            #[allow(clippy::cast_possible_truncation)]
            let prefix = data.len() as FwSizeStoreType;
            fw_try!(self.serialize_u16(prefix, e));
        }
        let ser = self.ser_loc();
        if ser + data.len() > self.capacity() {
            return SerializeStatus::NoRoomLeft;
        }
        self.bytes_mut()[ser..ser + data.len()].copy_from_slice(data);
        self.set_ser_loc(ser + data.len());
        self.set_deser_loc(0);
        SerializeStatus::Ok
    }

    /// Deserialize a raw byte region into `dest`. `len` is in/out exactly as
    /// in C++: on entry the maximum ([`LengthMode::IncludeLength`]) or exact
    /// ([`LengthMode::OmitLength`]) byte count (must be <= `dest.len()`);
    /// on success it is set to the count actually stored.
    ///
    /// C++ parity notes: with `IncludeLength` a stored length larger than the
    /// caller max or the remaining bytes is `DeserSizeMismatch` with the
    /// prefix already consumed; the `OmitLength` path has NO empty check —
    /// reading from an empty buffer yields `DeserSizeMismatch`, not
    /// `DeserBufferEmpty` (and a zero-length read succeeds).
    fn deserialize_bytes(
        &mut self,
        dest: &mut [u8],
        len: &mut usize,
        mode: LengthMode,
        e: Endianness,
    ) -> SerializeStatus {
        crate::fw_assert!(*len <= dest.len(), *len as i32, dest.len() as i32);
        match mode {
            LengthMode::IncludeLength => {
                let mut stored: FwSizeStoreType = 0;
                fw_try!(self.deserialize_u16(&mut stored, e));
                let stored = stored as usize;
                if stored > self.deserialize_size_left() || stored > *len {
                    return SerializeStatus::DeserSizeMismatch;
                }
                let deser = self.deser_loc();
                dest[..stored].copy_from_slice(&self.bytes()[deser..deser + stored]);
                self.set_deser_loc(deser + stored);
                *len = stored;
            }
            LengthMode::OmitLength => {
                if *len > self.deserialize_size_left() {
                    return SerializeStatus::DeserSizeMismatch;
                }
                let deser = self.deser_loc();
                dest[..*len].copy_from_slice(&self.bytes()[deser..deser + *len]);
                self.set_deser_loc(deser + *len);
            }
        }
        SerializeStatus::Ok
    }

    /// Serialize another buffer's content as a value: `[u16 size][size bytes]`.
    /// The total (size + 2) is room-checked BEFORE the prefix is written, so a
    /// failed call leaves this buffer untouched — unlike
    /// [`SerBuf::serialize_bytes`] with `IncludeLength` (C++ parity).
    fn serialize_buffer(&mut self, val: &dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let size = val.ser_loc();
        if self.ser_loc() + size + size_of::<FwSizeStoreType>() > self.capacity() {
            return SerializeStatus::NoRoomLeft;
        }
        #[allow(clippy::cast_possible_truncation)]
        let prefix = size as FwSizeStoreType;
        fw_try!(self.serialize_u16(prefix, e));
        let ser = self.ser_loc();
        self.bytes_mut()[ser..ser + size].copy_from_slice(&val.bytes()[..size]);
        self.set_ser_loc(ser + size);
        self.set_deser_loc(0);
        SerializeStatus::Ok
    }

    /// Deserialize a `[u16 size][bytes]` value into `val` (replacing its
    /// content, `set_buff_len` semantics). A stored length exceeding `val`'s
    /// capacity or the remaining bytes is `DeserSizeMismatch` with the prefix
    /// already consumed (C++ parity).
    fn deserialize_buffer(&mut self, val: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let mut stored: FwSizeStoreType = 0;
        fw_try!(self.deserialize_u16(&mut stored, e));
        let stored = stored as usize;
        if stored > val.capacity() || stored > self.deserialize_size_left() {
            return SerializeStatus::DeserSizeMismatch;
        }
        let deser = self.deser_loc();
        val.bytes_mut()[..stored].copy_from_slice(&self.bytes()[deser..deser + stored]);
        fw_try!(val.set_buff_len(stored));
        self.set_deser_loc(deser + stored);
        SerializeStatus::Ok
    }

    /// Serialize a `FwSizeType` as its on-wire `FwSizeStoreType` (u16).
    /// Unlike the raw-slice path this DOES range-check: a value outside the
    /// u16 range returns [`SerializeStatus::FormatError`] (C++ parity — two
    /// different behaviors for the same wire field).
    fn serialize_size(&mut self, size: FwSizeType, e: Endianness) -> SerializeStatus {
        if size > FwSizeType::from(FwSizeStoreType::MAX) {
            return SerializeStatus::FormatError;
        }
        #[allow(clippy::cast_possible_truncation)]
        self.serialize_u16(size as FwSizeStoreType, e)
    }

    /// Deserialize an on-wire `FwSizeStoreType` (u16) and widen to `FwSizeType`.
    fn deserialize_size(&mut self, size: &mut FwSizeType, e: Endianness) -> SerializeStatus {
        let mut stored: FwSizeStoreType = 0;
        let status = self.deserialize_u16(&mut stored, e);
        if status == SerializeStatus::Ok {
            *size = FwSizeType::from(stored);
        }
        status
    }

    /// Advance the write cursor leaving the bytes unwritten. C++ parity:
    /// room-checked, and does NOT reset the read cursor.
    fn serialize_skip(&mut self, num_bytes: usize) -> SerializeStatus {
        let new_ser = self.ser_loc() + num_bytes;
        if new_ser <= self.capacity() {
            self.set_ser_loc(new_ser);
            SerializeStatus::Ok
        } else {
            SerializeStatus::NoRoomLeft
        }
    }

    /// Skip bytes on the read side, with the exact empty/mismatch checks —
    /// so `deserialize_skip(0)` on a fully consumed buffer returns
    /// [`SerializeStatus::DeserBufferEmpty`], not Ok (C++ parity gotcha).
    fn deserialize_skip(&mut self, num_bytes: usize) -> SerializeStatus {
        let ser = self.ser_loc();
        let deser = self.deser_loc();
        if ser == deser {
            return SerializeStatus::DeserBufferEmpty;
        }
        if ser - deser < num_bytes {
            return SerializeStatus::DeserSizeMismatch;
        }
        self.set_deser_loc(deser + num_bytes);
        SerializeStatus::Ok
    }

    /// `resetSer` + `serializeSkip(offset)`.
    fn move_ser_to_offset(&mut self, offset: usize) -> SerializeStatus {
        self.reset_ser();
        self.serialize_skip(offset)
    }

    /// `resetDeser` + `deserializeSkip(offset)`.
    fn move_deser_to_offset(&mut self, offset: usize) -> SerializeStatus {
        self.reset_deser();
        self.deserialize_skip(offset)
    }

    /// Zero both cursors.
    fn reset_ser(&mut self) {
        self.set_ser_loc(0);
        self.set_deser_loc(0);
    }

    /// Zero only the read cursor.
    fn reset_deser(&mut self) {
        self.set_deser_loc(0);
    }

    /// Number of valid bytes (== the write cursor).
    fn get_size(&self) -> usize {
        self.ser_loc()
    }

    /// Bytes remaining to read (`getDeserializeSizeLeft`).
    fn deserialize_size_left(&self) -> usize {
        crate::fw_assert!(
            self.ser_loc() >= self.deser_loc(),
            self.ser_loc() as i32,
            self.deser_loc() as i32
        );
        self.ser_loc() - self.deser_loc()
    }

    /// Room remaining to write (`getSerializeSizeLeft`).
    fn serialize_size_left(&self) -> usize {
        crate::fw_assert!(
            self.capacity() >= self.ser_loc(),
            self.capacity() as i32,
            self.ser_loc() as i32
        );
        self.capacity() - self.ser_loc()
    }

    /// Replace the buffer content with `src` (`setBuff`): capacity check
    /// yields [`SerializeStatus::NoRoomLeft`], then the write cursor is set to
    /// `src.len()` and the read cursor to 0.
    fn set_buff(&mut self, src: &[u8]) -> SerializeStatus {
        if self.capacity() < src.len() {
            return SerializeStatus::NoRoomLeft;
        }
        self.bytes_mut()[..src.len()].copy_from_slice(src);
        self.set_ser_loc(src.len());
        self.set_deser_loc(0);
        SerializeStatus::Ok
    }

    /// Declare `len` bytes of the existing storage valid without copying
    /// (`setBuffLen`): write cursor = `len`, read cursor = 0.
    fn set_buff_len(&mut self, len: usize) -> SerializeStatus {
        if self.capacity() < len {
            return SerializeStatus::NoRoomLeft;
        }
        self.set_ser_loc(len);
        self.set_deser_loc(0);
        SerializeStatus::Ok
    }

    /// Copy `size` unread bytes into `dest`, REPLACING dest's content
    /// (`copyRaw`). Checks dest's total capacity (`NoRoomLeft`) and this
    /// buffer's remaining bytes (`DeserSizeMismatch`); advances the read
    /// cursor only on success.
    fn copy_raw(&mut self, dest: &mut dyn SerBufAny, size: usize) -> SerializeStatus {
        if dest.capacity() < size {
            return SerializeStatus::NoRoomLeft;
        }
        if self.deserialize_size_left() < size {
            return SerializeStatus::DeserSizeMismatch;
        }
        let deser = self.deser_loc();
        // setBuff replaces dest content and resets its cursors
        dest.bytes_mut()[..size].copy_from_slice(&self.bytes()[deser..deser + size]);
        dest.set_ser_loc(size);
        dest.set_deser_loc(0);
        self.set_deser_loc(deser + size);
        SerializeStatus::Ok
    }

    /// Copy `size` unread bytes APPENDING to `dest` (`copyRawOffset`); checks
    /// `dest.capacity - dest.size` (append semantics) and this buffer's
    /// remaining bytes; advances the read cursor only on success.
    fn copy_raw_offset(&mut self, dest: &mut dyn SerBufAny, size: usize) -> SerializeStatus {
        if dest.capacity() < size + dest.ser_loc() {
            return SerializeStatus::NoRoomLeft;
        }
        if self.deserialize_size_left() < size {
            return SerializeStatus::DeserSizeMismatch;
        }
        let deser = self.deser_loc();
        let dest_ser = dest.ser_loc();
        dest.bytes_mut()[dest_ser..dest_ser + size]
            .copy_from_slice(&self.bytes()[deser..deser + size]);
        // append via the write path: advance dest write cursor, reset its read cursor
        dest.set_ser_loc(dest_ser + size);
        dest.set_deser_loc(0);
        self.set_deser_loc(deser + size);
        SerializeStatus::Ok
    }

    /// Serialize a [`Serialize`] value into this buffer.
    fn serialize(&mut self, val: &dyn Serialize, e: Endianness) -> SerializeStatus {
        val.serialize_to(self.as_ser_buf_any(), e)
    }

    /// Deserialize a [`Deserialize`] value out of this buffer.
    fn deserialize(&mut self, val: &mut dyn Deserialize, e: Endianness) -> SerializeStatus {
        val.deserialize_from(self.as_ser_buf_any(), e)
    }

    /// The valid content, `bytes()[0..ser_loc]`.
    fn as_slice(&self) -> &[u8] {
        &self.bytes()[..self.ser_loc()]
    }

    /// The not-yet-deserialized remainder, `bytes()[deser_loc..ser_loc]`
    /// (`getBuffAddrLeft` equivalent).
    fn remaining_slice(&self) -> &[u8] {
        &self.bytes()[self.deser_loc()..self.ser_loc()]
    }
}

impl<T: SerBufAny + ?Sized> SerBuf for T {}

// ---------------------------------------------------------------------------
// Serialize / Deserialize value traits
// ---------------------------------------------------------------------------

/// A value that knows how to write itself into a serialization buffer
/// (port of the `Fw::Serializable` serialize half).
pub trait Serialize {
    /// Serialize this value into `buf`.
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus;
    /// The number of bytes `serialize_to` will write for this value.
    fn serialized_size(&self) -> usize;
}

/// A value that knows how to read itself from a serialization buffer
/// (port of the `Fw::Serializable` deserialize half).
pub trait Deserialize {
    /// Deserialize this value out of `buf`.
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus;
}

macro_rules! prim_value_impl {
    ($ty:ty, $ser:ident, $deser:ident) => {
        impl Serialize for $ty {
            fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
                buf.$ser(*self, e)
            }
            fn serialized_size(&self) -> usize {
                size_of::<$ty>()
            }
        }
        impl Deserialize for $ty {
            fn deserialize_from(
                &mut self,
                buf: &mut dyn SerBufAny,
                e: Endianness,
            ) -> SerializeStatus {
                buf.$deser(self, e)
            }
        }
    };
}

prim_value_impl!(u8, serialize_u8, deserialize_u8);
prim_value_impl!(i8, serialize_i8, deserialize_i8);
prim_value_impl!(u16, serialize_u16, deserialize_u16);
prim_value_impl!(i16, serialize_i16, deserialize_i16);
prim_value_impl!(u32, serialize_u32, deserialize_u32);
prim_value_impl!(i32, serialize_i32, deserialize_i32);
prim_value_impl!(u64, serialize_u64, deserialize_u64);
prim_value_impl!(i64, serialize_i64, deserialize_i64);
prim_value_impl!(f32, serialize_f32, deserialize_f32);
prim_value_impl!(f64, serialize_f64, deserialize_f64);

impl Serialize for bool {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        buf.serialize_bool(*self, e)
    }
    fn serialized_size(&self) -> usize {
        1
    }
}
impl Deserialize for bool {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        buf.deserialize_bool(self, e)
    }
}

// ---------------------------------------------------------------------------
// Concrete buffers
// ---------------------------------------------------------------------------

/// Owned fixed-capacity serialization buffer
/// (port of `Fw::LinearBufferTemplate<N>`).
#[derive(Debug, Clone)]
pub struct LinearBuffer<const N: usize> {
    data: [u8; N],
    ser: usize,
    deser: usize,
}

impl<const N: usize> LinearBuffer<N> {
    /// Size when serialized as a value: `N + sizeof(FwSizeStoreType)`.
    pub const SERIALIZED_SIZE: usize = N + size_of::<FwSizeStoreType>();

    /// A new, empty buffer.
    pub const fn new() -> Self {
        Self {
            data: [0; N],
            ser: 0,
            deser: 0,
        }
    }
}

impl<const N: usize> Default for LinearBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

// Content equality: same valid length and same valid bytes (the C++ BUILD_UT
// operator==); cursor position of the read side is not part of identity.
impl<const N: usize> PartialEq for LinearBuffer<N> {
    fn eq(&self, other: &Self) -> bool {
        self.ser == other.ser && self.data[..self.ser] == other.data[..other.ser]
    }
}
impl<const N: usize> Eq for LinearBuffer<N> {}

impl<const N: usize> SerBufAny for LinearBuffer<N> {
    fn bytes(&self) -> &[u8] {
        &self.data
    }
    fn bytes_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }
    fn capacity(&self) -> usize {
        N
    }
    fn ser_loc(&self) -> usize {
        self.ser
    }
    fn set_ser_loc(&mut self, loc: usize) {
        self.ser = loc;
    }
    fn deser_loc(&self) -> usize {
        self.deser
    }
    fn set_deser_loc(&mut self, loc: usize) {
        self.deser = loc;
    }
    fn as_ser_buf_any(&mut self) -> &mut dyn SerBufAny {
        self
    }
}

impl<const N: usize> Serialize for LinearBuffer<N> {
    /// Serialized as a value: `[u16 size][content bytes]`.
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        buf.serialize_buffer(self, e)
    }
    fn serialized_size(&self) -> usize {
        size_of::<FwSizeStoreType>() + self.ser
    }
}

impl<const N: usize> Deserialize for LinearBuffer<N> {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        buf.deserialize_buffer(self, e)
    }
}

/// Serialization buffer borrowing caller-owned storage
/// (port of `Fw::ExternalSerializeBuffer`).
#[derive(Debug)]
pub struct ExtBuf<'a> {
    data: &'a mut [u8],
    ser: usize,
    deser: usize,
}

impl<'a> ExtBuf<'a> {
    /// Wrap `data` with both cursors reset (an empty buffer for writing).
    pub fn new(data: &'a mut [u8]) -> Self {
        Self {
            data,
            ser: 0,
            deser: 0,
        }
    }

    /// Wrap `data` with `len` bytes declared valid (`setBuffLen` semantics;
    /// the whole window readable when `len == data.len()`).
    pub fn with_len(data: &'a mut [u8], len: usize) -> Self {
        crate::fw_assert!(len <= data.len(), len as i32, data.len() as i32);
        Self {
            data,
            ser: len,
            deser: 0,
        }
    }
}

impl SerBufAny for ExtBuf<'_> {
    fn bytes(&self) -> &[u8] {
        self.data
    }
    fn bytes_mut(&mut self) -> &mut [u8] {
        self.data
    }
    fn capacity(&self) -> usize {
        self.data.len()
    }
    fn ser_loc(&self) -> usize {
        self.ser
    }
    fn set_ser_loc(&mut self, loc: usize) {
        self.ser = loc;
    }
    fn deser_loc(&self) -> usize {
        self.deser
    }
    fn set_deser_loc(&mut self, loc: usize) {
        self.deser = loc;
    }
    fn as_ser_buf_any(&mut self) -> &mut dyn SerBufAny {
        self
    }
}

/// Com packet buffer (`Fw::ComBuffer`).
pub type ComBuffer = LinearBuffer<FW_COM_BUFFER_MAX_SIZE>;
/// Command argument buffer (`Fw::CmdArgBuffer`).
pub type CmdArgBuffer = LinearBuffer<FW_CMD_ARG_BUFFER_MAX_SIZE>;
/// Event log argument buffer (`Fw::LogBuffer`).
pub type LogBuffer = LinearBuffer<FW_LOG_BUFFER_MAX_SIZE>;
/// Telemetry value buffer (`Fw::TlmBuffer`).
pub type TlmBuffer = LinearBuffer<FW_TLM_BUFFER_MAX_SIZE>;
/// Parameter value buffer (`Fw::ParamBuffer`).
pub type ParamBuffer = LinearBuffer<FW_PARAM_BUFFER_MAX_SIZE>;

#[cfg(test)]
mod tests {
    use super::*;

    // ------------------------------------------------------------ primitives

    #[test]
    fn u8_i8_single_byte() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.serialize_u8_be(0xAB), SerializeStatus::Ok);
        assert_eq!(buf.serialize_i8_be(-1), SerializeStatus::Ok);
        assert_eq!(buf.as_slice(), &[0xAB, 0xFF]);
        let mut u = 0u8;
        let mut i = 0i8;
        assert_eq!(buf.deserialize_u8_be(&mut u), SerializeStatus::Ok);
        assert_eq!(buf.deserialize_i8_be(&mut i), SerializeStatus::Ok);
        assert_eq!((u, i), (0xAB, -1));
    }

    #[test]
    fn integers_big_endian_msb_first() {
        let mut buf = LinearBuffer::<32>::new();
        assert_eq!(buf.serialize_u16_be(0x1234), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u32_be(0xDEAD_BEEF), SerializeStatus::Ok);
        assert_eq!(
            buf.serialize_u64_be(0x0102_0304_0506_0708),
            SerializeStatus::Ok
        );
        assert_eq!(
            buf.as_slice(),
            &[
                0x12, 0x34, //
                0xDE, 0xAD, 0xBE, 0xEF, //
                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
            ]
        );
    }

    #[test]
    fn signed_integers_twos_complement_bytes() {
        let mut buf = LinearBuffer::<32>::new();
        assert_eq!(buf.serialize_i16_be(-2), SerializeStatus::Ok);
        assert_eq!(buf.serialize_i32_be(-2), SerializeStatus::Ok);
        assert_eq!(buf.serialize_i64_be(-2), SerializeStatus::Ok);
        assert_eq!(
            buf.as_slice(),
            &[
                0xFF, 0xFE, //
                0xFF, 0xFF, 0xFF, 0xFE, //
                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
            ]
        );
        let mut a = 0i16;
        let mut b = 0i32;
        let mut c = 0i64;
        assert_eq!(buf.deserialize_i16_be(&mut a), SerializeStatus::Ok);
        assert_eq!(buf.deserialize_i32_be(&mut b), SerializeStatus::Ok);
        assert_eq!(buf.deserialize_i64_be(&mut c), SerializeStatus::Ok);
        assert_eq!((a, b, c), (-2, -2, -2));
    }

    #[test]
    fn little_endian_reverses_per_call() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(
            buf.serialize_u32(0xDEAD_BEEF, Endianness::Little),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0xEF, 0xBE, 0xAD, 0xDE]);
        let mut v = 0u32;
        assert_eq!(
            buf.deserialize_u32(&mut v, Endianness::Little),
            SerializeStatus::Ok
        );
        assert_eq!(v, 0xDEAD_BEEF);
    }

    #[test]
    fn floats_bit_cast_then_integer_rules() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(buf.serialize_f32_be(1.0), SerializeStatus::Ok);
        assert_eq!(buf.serialize_f64_be(1.0), SerializeStatus::Ok);
        assert_eq!(
            buf.as_slice(),
            &[
                0x3F, 0x80, 0x00, 0x00, //
                0x3F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ]
        );
        let mut f = 0f32;
        let mut d = 0f64;
        assert_eq!(buf.deserialize_f32_be(&mut f), SerializeStatus::Ok);
        assert_eq!(buf.deserialize_f64_be(&mut d), SerializeStatus::Ok);
        assert_eq!((f, d), (1.0, 1.0));
    }

    #[test]
    fn bool_wire_values_ff_and_00() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.serialize_bool_be(true), SerializeStatus::Ok);
        assert_eq!(buf.serialize_bool_be(false), SerializeStatus::Ok);
        assert_eq!(buf.as_slice(), &[0xFF, 0x00]);
        let mut a = false;
        let mut b = true;
        assert_eq!(buf.deserialize_bool_be(&mut a), SerializeStatus::Ok);
        assert_eq!(buf.deserialize_bool_be(&mut b), SerializeStatus::Ok);
        assert!(a);
        assert!(!b);
    }

    #[test]
    fn bool_deserialize_rejects_nonstandard_bytes_without_advancing() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.serialize_u8_be(0x01), SerializeStatus::Ok);
        let mut v = false;
        assert_eq!(
            buf.deserialize_bool_be(&mut v),
            SerializeStatus::DeserFormatError
        );
        // cursor did not advance: the byte is still readable
        assert_eq!(buf.deser_loc(), 0);
        let mut raw = 0u8;
        assert_eq!(buf.deserialize_u8_be(&mut raw), SerializeStatus::Ok);
        assert_eq!(raw, 0x01);
    }

    // ---------------------------------------------------------- cursor rules

    #[test]
    fn every_write_resets_the_read_cursor() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(buf.serialize_u16_be(0x0102), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u16_be(0x0304), SerializeStatus::Ok);
        let mut v = 0u16;
        assert_eq!(buf.deserialize_u16_be(&mut v), SerializeStatus::Ok);
        assert_eq!(v, 0x0102);
        assert_eq!(buf.deser_loc(), 2);
        // a write in between loses the read progress by design
        assert_eq!(buf.serialize_u16_be(0x0506), SerializeStatus::Ok);
        assert_eq!(buf.deser_loc(), 0);
        assert_eq!(buf.deserialize_u16_be(&mut v), SerializeStatus::Ok);
        assert_eq!(v, 0x0102, "reading restarts from the beginning");
    }

    #[test]
    fn empty_vs_size_mismatch_distinction() {
        let mut buf = LinearBuffer::<16>::new();
        let mut v = 0u32;
        // nothing written at all -> BUFFER_EMPTY
        assert_eq!(
            buf.deserialize_u32_be(&mut v),
            SerializeStatus::DeserBufferEmpty
        );
        // partial remainder -> SIZE_MISMATCH
        assert_eq!(buf.serialize_u16_be(1), SerializeStatus::Ok);
        assert_eq!(
            buf.deserialize_u32_be(&mut v),
            SerializeStatus::DeserSizeMismatch
        );
        // consume exactly, then BUFFER_EMPTY again
        let mut w = 0u16;
        assert_eq!(buf.deserialize_u16_be(&mut w), SerializeStatus::Ok);
        assert_eq!(
            buf.deserialize_u8_be(&mut [0u8; 1][0]),
            SerializeStatus::DeserBufferEmpty
        );
    }

    #[test]
    fn deserialize_skip_checks_empty_and_mismatch() {
        let mut buf = LinearBuffer::<16>::new();
        // skip(0) on a fully consumed buffer is BUFFER_EMPTY, not Ok (gotcha)
        assert_eq!(buf.deserialize_skip(0), SerializeStatus::DeserBufferEmpty);
        assert_eq!(buf.serialize_u32_be(7), SerializeStatus::Ok);
        assert_eq!(buf.deserialize_skip(5), SerializeStatus::DeserSizeMismatch);
        assert_eq!(buf.deserialize_skip(3), SerializeStatus::Ok);
        assert_eq!(buf.deser_loc(), 3);
        let mut v = 0u8;
        assert_eq!(buf.deserialize_u8_be(&mut v), SerializeStatus::Ok);
        assert_eq!(v, 7);
        assert_eq!(buf.deserialize_skip(0), SerializeStatus::DeserBufferEmpty);
    }

    #[test]
    fn serialize_skip_room_checked_and_keeps_read_cursor() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.serialize_u32_be(0xAABBCCDD), SerializeStatus::Ok);
        let mut v = 0u16;
        assert_eq!(buf.deserialize_u16_be(&mut v), SerializeStatus::Ok);
        assert_eq!(buf.deser_loc(), 2);
        // C++ parity: serializeSkip does NOT reset the read cursor
        assert_eq!(buf.serialize_skip(2), SerializeStatus::Ok);
        assert_eq!(buf.deser_loc(), 2);
        assert_eq!(buf.ser_loc(), 6);
        assert_eq!(buf.serialize_skip(3), SerializeStatus::NoRoomLeft);
        assert_eq!(buf.ser_loc(), 6);
    }

    #[test]
    fn move_to_offset_helpers() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.move_ser_to_offset(4), SerializeStatus::Ok);
        assert_eq!(buf.ser_loc(), 4);
        assert_eq!(buf.serialize_u16_be(0x0102), SerializeStatus::Ok);
        assert_eq!(buf.move_deser_to_offset(4), SerializeStatus::Ok);
        let mut v = 0u16;
        assert_eq!(buf.deserialize_u16_be(&mut v), SerializeStatus::Ok);
        assert_eq!(v, 0x0102);
        assert_eq!(buf.move_ser_to_offset(9), SerializeStatus::NoRoomLeft);
    }

    #[test]
    fn room_check_boundary_exact_fit() {
        let mut buf = LinearBuffer::<4>::new();
        assert_eq!(buf.serialize_u32_be(1), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u8_be(1), SerializeStatus::NoRoomLeft);
        buf.reset_ser();
        assert_eq!(buf.serialize_u16_be(1), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u32_be(1), SerializeStatus::NoRoomLeft);
    }

    #[test]
    fn reset_ser_and_reset_deser() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.serialize_u32_be(9), SerializeStatus::Ok);
        let mut v = 0u32;
        assert_eq!(buf.deserialize_u32_be(&mut v), SerializeStatus::Ok);
        buf.reset_deser();
        assert_eq!(buf.deser_loc(), 0);
        assert_eq!(buf.ser_loc(), 4);
        buf.reset_ser();
        assert_eq!((buf.ser_loc(), buf.deser_loc()), (0, 0));
    }

    #[test]
    fn size_left_accessors() {
        let mut buf = LinearBuffer::<10>::new();
        assert_eq!(buf.serialize_u32_be(1), SerializeStatus::Ok);
        assert_eq!(buf.get_size(), 4);
        assert_eq!(buf.serialize_size_left(), 6);
        assert_eq!(buf.deserialize_size_left(), 4);
        let mut v = 0u16;
        assert_eq!(buf.deserialize_u16_be(&mut v), SerializeStatus::Ok);
        assert_eq!(buf.deserialize_size_left(), 2);
    }

    // ------------------------------------------------------------ byte slices

    #[test]
    fn serialize_bytes_include_length_prefix() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(
            buf.serialize_bytes(
                &[0xAA, 0xBB, 0xCC],
                LengthMode::IncludeLength,
                Endianness::Big
            ),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0x00, 0x03, 0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn serialize_bytes_omit_length_raw() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(
            buf.serialize_bytes(&[0xAA, 0xBB], LengthMode::OmitLength, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0xAA, 0xBB]);
    }

    #[test]
    fn serialize_bytes_include_length_commits_prefix_before_body_room_check() {
        // C++ parity gotcha: the prefix goes out first; a too-big body then
        // fails NoRoomLeft with the prefix already written.
        let mut buf = LinearBuffer::<4>::new();
        assert_eq!(
            buf.serialize_bytes(&[1, 2, 3, 4], LengthMode::IncludeLength, Endianness::Big),
            SerializeStatus::NoRoomLeft
        );
        assert_eq!(buf.ser_loc(), 2, "prefix committed");
        assert_eq!(buf.as_slice(), &[0x00, 0x04]);
    }

    #[test]
    fn serialize_bytes_prefix_silently_truncates_to_u16() {
        // 65537 bytes: the prefix wraps to 1 while all bytes are written
        let mut storage = vec![0u8; 70_000];
        let mut buf = ExtBuf::new(&mut storage);
        let data = vec![0x5Au8; 65_537];
        assert_eq!(
            buf.serialize_bytes(&data, LengthMode::IncludeLength, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.ser_loc(), 2 + 65_537);
        assert_eq!(
            &buf.bytes()[..2],
            &[0x00, 0x01],
            "prefix truncated, not checked"
        );
    }

    #[test]
    fn serialize_size_range_checks_unlike_slice_prefix() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(
            buf.serialize_size(65_536, Endianness::Big),
            SerializeStatus::FormatError
        );
        assert_eq!(buf.ser_loc(), 0, "nothing written on FormatError");
        assert_eq!(
            buf.serialize_size(65_535, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0xFF, 0xFF]);
        let mut size: FwSizeType = 0;
        assert_eq!(
            buf.deserialize_size(&mut size, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(size, 65_535);
    }

    #[test]
    fn deserialize_bytes_include_length() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(
            buf.serialize_bytes(&[9, 8, 7], LengthMode::IncludeLength, Endianness::Big),
            SerializeStatus::Ok
        );
        let mut dest = [0u8; 8];
        let mut len = dest.len();
        assert_eq!(
            buf.deserialize_bytes(
                &mut dest,
                &mut len,
                LengthMode::IncludeLength,
                Endianness::Big
            ),
            SerializeStatus::Ok
        );
        assert_eq!(len, 3);
        assert_eq!(&dest[..3], &[9, 8, 7]);
    }

    #[test]
    fn deserialize_bytes_include_length_rejects_over_max_after_consuming_prefix() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(
            buf.serialize_bytes(&[1, 2, 3, 4, 5], LengthMode::IncludeLength, Endianness::Big),
            SerializeStatus::Ok
        );
        let mut dest = [0u8; 3];
        let mut len = dest.len();
        assert_eq!(
            buf.deserialize_bytes(
                &mut dest,
                &mut len,
                LengthMode::IncludeLength,
                Endianness::Big
            ),
            SerializeStatus::DeserSizeMismatch
        );
        assert_eq!(buf.deser_loc(), 2, "prefix consumed by the failed attempt");
    }

    #[test]
    fn deserialize_bytes_include_length_rejects_prefix_beyond_remaining() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(buf.serialize_u16_be(9), SerializeStatus::Ok); // prefix says 9
        assert_eq!(buf.serialize_u8_be(1), SerializeStatus::Ok); // only 1 byte follows
        let mut dest = [0u8; 16];
        let mut len = dest.len();
        assert_eq!(
            buf.deserialize_bytes(
                &mut dest,
                &mut len,
                LengthMode::IncludeLength,
                Endianness::Big
            ),
            SerializeStatus::DeserSizeMismatch
        );
    }

    #[test]
    fn deserialize_bytes_omit_length_no_empty_check() {
        // C++ parity: the raw path checks only remaining >= len, so an empty
        // buffer yields SIZE_MISMATCH (not BUFFER_EMPTY), and len 0 succeeds.
        let mut buf = LinearBuffer::<16>::new();
        let mut dest = [0u8; 4];
        let mut len = 4usize;
        assert_eq!(
            buf.deserialize_bytes(&mut dest, &mut len, LengthMode::OmitLength, Endianness::Big),
            SerializeStatus::DeserSizeMismatch
        );
        let mut len0 = 0usize;
        assert_eq!(
            buf.deserialize_bytes(
                &mut dest,
                &mut len0,
                LengthMode::OmitLength,
                Endianness::Big
            ),
            SerializeStatus::Ok
        );
        assert_eq!(buf.serialize_u32_be(0x01020304), SerializeStatus::Ok);
        let mut len2 = 2usize;
        assert_eq!(
            buf.deserialize_bytes(
                &mut dest,
                &mut len2,
                LengthMode::OmitLength,
                Endianness::Big
            ),
            SerializeStatus::Ok
        );
        assert_eq!(&dest[..2], &[0x01, 0x02]);
    }

    // -------------------------------------------------------- nested buffers

    #[test]
    fn nested_buffer_serialize_u16_size_plus_bytes() {
        let mut inner = LinearBuffer::<8>::new();
        assert_eq!(inner.serialize_u16_be(0xBEEF), SerializeStatus::Ok);
        let mut outer = LinearBuffer::<16>::new();
        assert_eq!(
            outer.serialize_buffer(&inner, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(outer.as_slice(), &[0x00, 0x02, 0xBE, 0xEF]);
        // via the Serialize impl too
        let mut outer2 = LinearBuffer::<16>::new();
        assert_eq!(
            outer2.serialize(&inner, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(outer2.as_slice(), &[0x00, 0x02, 0xBE, 0xEF]);
        assert_eq!(Serialize::serialized_size(&inner), 4);
    }

    #[test]
    fn nested_buffer_serialize_prechecks_room_without_writing_prefix() {
        // unlike serialize_bytes, the (size + 2) precheck fires BEFORE the
        // prefix write, leaving the destination untouched
        let mut inner = LinearBuffer::<8>::new();
        assert_eq!(inner.serialize_u32_be(1), SerializeStatus::Ok);
        let mut outer = LinearBuffer::<5>::new(); // needs 6
        assert_eq!(
            outer.serialize_buffer(&inner, Endianness::Big),
            SerializeStatus::NoRoomLeft
        );
        assert_eq!(outer.ser_loc(), 0, "no prefix committed");
    }

    #[test]
    fn nested_buffer_deserialize_roundtrip_and_checks() {
        let mut inner = LinearBuffer::<8>::new();
        assert_eq!(inner.serialize_u32_be(0xCAFEBABE), SerializeStatus::Ok);
        let mut outer = LinearBuffer::<16>::new();
        assert_eq!(
            outer.serialize_buffer(&inner, Endianness::Big),
            SerializeStatus::Ok
        );

        let mut out = LinearBuffer::<8>::new();
        assert_eq!(
            outer.deserialize_buffer(&mut out, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(out.as_slice(), &[0xCA, 0xFE, 0xBA, 0xBE]);
        assert_eq!(out.deser_loc(), 0, "destination readable from the start");

        // stored length > destination capacity -> SIZE_MISMATCH, prefix consumed
        outer.reset_deser();
        let mut small = LinearBuffer::<2>::new();
        assert_eq!(
            outer.deserialize_buffer(&mut small, Endianness::Big),
            SerializeStatus::DeserSizeMismatch
        );
        assert_eq!(outer.deser_loc(), 2);

        // stored length > remaining -> SIZE_MISMATCH
        let mut lying = LinearBuffer::<16>::new();
        assert_eq!(lying.serialize_u16_be(10), SerializeStatus::Ok);
        assert_eq!(lying.serialize_u8_be(1), SerializeStatus::Ok);
        let mut dest = LinearBuffer::<16>::new();
        assert_eq!(
            lying.deserialize_buffer(&mut dest, Endianness::Big),
            SerializeStatus::DeserSizeMismatch
        );
    }

    // ------------------------------------------------- set_buff and copy_raw

    #[test]
    fn set_buff_copies_and_sets_cursors() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.set_buff(&[1, 2, 3]), SerializeStatus::Ok);
        assert_eq!(buf.as_slice(), &[1, 2, 3]);
        assert_eq!((buf.ser_loc(), buf.deser_loc()), (3, 0));
        assert_eq!(buf.set_buff(&[0; 9]), SerializeStatus::NoRoomLeft);
        assert_eq!(buf.as_slice(), &[1, 2, 3], "unchanged on NoRoomLeft");
    }

    #[test]
    fn set_buff_len_declares_valid_without_copying() {
        let mut buf = LinearBuffer::<8>::new();
        buf.bytes_mut()[..2].copy_from_slice(&[9, 8]);
        assert_eq!(buf.set_buff_len(2), SerializeStatus::Ok);
        assert_eq!(buf.as_slice(), &[9, 8]);
        assert_eq!(buf.set_buff_len(9), SerializeStatus::NoRoomLeft);
    }

    #[test]
    fn copy_raw_replaces_dest_and_advances_source() {
        let mut src = LinearBuffer::<16>::new();
        assert_eq!(src.set_buff(&[1, 2, 3, 4, 5]), SerializeStatus::Ok);
        let mut dest = LinearBuffer::<8>::new();
        assert_eq!(dest.set_buff(&[9, 9, 9, 9, 9, 9]), SerializeStatus::Ok);
        assert_eq!(src.copy_raw(&mut dest, 3), SerializeStatus::Ok);
        assert_eq!(dest.as_slice(), &[1, 2, 3], "dest content REPLACED");
        assert_eq!(src.deser_loc(), 3);
        // remaining source bytes still readable
        let mut v = 0u16;
        assert_eq!(src.deserialize_u16_be(&mut v), SerializeStatus::Ok);
        assert_eq!(v, 0x0405);
    }

    #[test]
    fn copy_raw_checks_dest_capacity_then_source_remaining() {
        let mut src = LinearBuffer::<16>::new();
        assert_eq!(src.set_buff(&[1, 2]), SerializeStatus::Ok);
        let mut small = LinearBuffer::<2>::new();
        assert_eq!(src.copy_raw(&mut small, 3), SerializeStatus::NoRoomLeft);
        assert_eq!(src.deser_loc(), 0, "source cursor untouched on failure");
        let mut dest = LinearBuffer::<8>::new();
        assert_eq!(
            src.copy_raw(&mut dest, 3),
            SerializeStatus::DeserSizeMismatch
        );
        assert_eq!(src.deser_loc(), 0);
    }

    #[test]
    fn copy_raw_offset_appends_to_dest() {
        let mut src = LinearBuffer::<16>::new();
        assert_eq!(src.set_buff(&[1, 2, 3, 4]), SerializeStatus::Ok);
        let mut dest = LinearBuffer::<6>::new();
        assert_eq!(dest.set_buff(&[9, 9]), SerializeStatus::Ok);
        assert_eq!(src.copy_raw_offset(&mut dest, 2), SerializeStatus::Ok);
        assert_eq!(dest.as_slice(), &[9, 9, 1, 2], "appended, not replaced");
        assert_eq!(src.deser_loc(), 2);
        // append room check uses capacity - current size
        assert_eq!(
            src.copy_raw_offset(&mut dest, 3),
            SerializeStatus::NoRoomLeft
        );
        assert_eq!(src.copy_raw_offset(&mut dest, 2), SerializeStatus::Ok);
        assert_eq!(dest.as_slice(), &[9, 9, 1, 2, 3, 4]);
    }

    // ----------------------------------------------- ExtBuf and generic path

    #[test]
    fn ext_buf_borrows_external_storage() {
        let mut storage = [0u8; 8];
        {
            let mut buf = ExtBuf::new(&mut storage);
            assert_eq!(buf.capacity(), 8);
            assert_eq!(buf.serialize_u32_be(0x01020304), SerializeStatus::Ok);
        }
        assert_eq!(&storage[..4], &[1, 2, 3, 4]);
    }

    #[test]
    fn ext_buf_with_len_is_readable() {
        let mut storage = [0xAB, 0xCD, 0, 0];
        let mut buf = ExtBuf::with_len(&mut storage, 2);
        assert_eq!(buf.get_size(), 2);
        let mut v = 0u16;
        assert_eq!(buf.deserialize_u16_be(&mut v), SerializeStatus::Ok);
        assert_eq!(v, 0xABCD);
    }

    #[test]
    fn generic_serialize_and_deserialize_through_dyn() {
        let mut buf = LinearBuffer::<32>::new();
        let dyn_buf: &mut dyn SerBufAny = &mut buf;
        assert_eq!(
            dyn_buf.serialize(&0x1234u16, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(
            dyn_buf.serialize(&true, Endianness::Big),
            SerializeStatus::Ok
        );
        let mut v = 0u16;
        let mut b = false;
        assert_eq!(
            dyn_buf.deserialize(&mut v, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(
            dyn_buf.deserialize(&mut b, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(v, 0x1234);
        assert!(b);
    }

    #[test]
    fn fw_try_early_returns_non_ok() {
        fn inner(fail: bool) -> SerializeStatus {
            let mut buf = LinearBuffer::<2>::new();
            if fail {
                fw_try!(buf.serialize_u32_be(1)); // NoRoomLeft
                unreachable!();
            }
            fw_try!(buf.serialize_u16_be(1));
            SerializeStatus::Ok
        }
        assert_eq!(inner(true), SerializeStatus::NoRoomLeft);
        assert_eq!(inner(false), SerializeStatus::Ok);
    }

    #[test]
    fn status_discriminants_match_cpp() {
        assert_eq!(SerializeStatus::Ok as i32, 0);
        assert_eq!(SerializeStatus::FormatError as i32, 1);
        assert_eq!(SerializeStatus::NoRoomLeft as i32, 2);
        assert_eq!(SerializeStatus::DeserBufferEmpty as i32, 3);
        assert_eq!(SerializeStatus::DeserFormatError as i32, 4);
        assert_eq!(SerializeStatus::DeserSizeMismatch as i32, 5);
        assert_eq!(SerializeStatus::DeserTypeMismatch as i32, 6);
        assert_eq!(SerializeStatus::DeserImmutable as i32, 7);
        assert_eq!(SerializeStatus::DeserInvalidData as i32, 8);
        assert_eq!(SerializeStatus::DiscardedExisting as i32, 9);
    }

    #[test]
    fn buffer_alias_capacities_and_serialized_size() {
        assert_eq!(ComBuffer::new().capacity(), 512);
        assert_eq!(CmdArgBuffer::new().capacity(), 506);
        assert_eq!(LogBuffer::new().capacity(), 506);
        assert_eq!(TlmBuffer::new().capacity(), 506);
        assert_eq!(ParamBuffer::new().capacity(), 506);
        assert_eq!(ComBuffer::SERIALIZED_SIZE, 514);
    }

    #[test]
    fn linear_buffer_clone_copies_content_and_cursors() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.serialize_u32_be(0xA1B2C3D4), SerializeStatus::Ok);
        let mut v = 0u16;
        assert_eq!(buf.deserialize_u16_be(&mut v), SerializeStatus::Ok);
        let clone = buf.clone();
        assert_eq!(clone.ser_loc(), 4);
        assert_eq!(clone.deser_loc(), 2, "cursors copied (C++ copyFrom)");
        assert_eq!(clone, buf);
    }
}
