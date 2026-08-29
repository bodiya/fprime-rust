//! `Fw::FilePacket` — the file-transfer packet family.
//!
//! Port of `Fw/FilePacket/{FilePacket,Header,PathName,StartPacket,
//! DataPacket,EndPacket,CancelPacket}.cpp`. Byte layouts are normative in
//! `docs/cpp-analysis/utils-misc.md` (§ "FilePacket family") and
//! `docs/cpp-analysis/file-services.md`.
//!
//! Wire format (all big-endian — the C++ `SerialBuffer` calls never pass an
//! endianness mode, so the F Prime default applies):
//!
//! ```text
//! header  = [u8 type][u32 sequenceIndex]                       (5 bytes)
//! START   = header + [u32 fileSize][PathName src][PathName dst]
//! DATA    = header + [u32 byteOffset][u16 dataSize][dataSize raw bytes]
//! END     = header + [u32 checksum]
//! CANCEL  = header
//! PathName = [u8 length (<= 255)][length raw bytes]  (no NUL)
//! ```
//!
//! `START`, `END` and `CANCEL` require the source buffer to be consumed
//! EXACTLY; `DATA` requires the bytes remaining after its fixed part to
//! equal `dataSize` exactly. These packets ride inside com packets with
//! descriptor `FW_PACKET_FILE` (0x0003), which is added and stripped by
//! `Svc::FileUplink` / `Svc::FileDownlink`, never here.
//!
//! ## Deviation from C++
//!
//! C++ models the family as a union tagged by `Header::m_type` (with an
//! in-memory-only `T_NONE`) and fills it in place via
//! `SerializeStatus fromBuffer(const Buffer&)`. Rust models it as an enum
//! over a borrowed source buffer, so parsing produces a value instead of
//! filling one: [`FilePacket::from_buffer`] returns
//! `Result<FilePacket<'_>, SerializeStatus>` and the `Err` carries the very
//! `SerializeStatus` the C++ call returns (`Svc::FileUplink` logs it as
//! `DecodeError(status)`). The `T_NONE` state is expressed as
//! [`FilePacketType::None`], which is reachable only through the type enum
//! (`FileDownlink::m_lastCompletedType`), never as a `FilePacket` variant.
//!
//! The path and payload slices borrow the source buffer, reproducing the
//! C++ zero-copy aliasing (`m_value` / `m_data` point into the buffer).

use crate::fpp_enum;
use crate::serial::{Endianness, LengthMode, SerBuf, SerBufAny, SerializeStatus};

fpp_enum! {
    /// `Fw::FilePacket::Type`. `None` (`T_NONE = 255`) is an in-memory
    /// sentinel only — it never appears on the wire.
    pub enum FilePacketType : u8 {
        /// START packet: opens a file transfer.
        Start = 0,
        /// DATA packet: one chunk of file content.
        Data = 1,
        /// END packet: closes a transfer and carries the CFDP checksum.
        End = 2,
        /// CANCEL packet: aborts a transfer.
        Cancel = 3,
        /// In-memory sentinel: no packet (`T_NONE`).
        None = 255,
    }
    default None
}

/// `Fw::FilePacket::Header::HEADERSIZE` — `sizeof(U8) + sizeof(U32)`.
pub const HEADER_SIZE: usize = 5;

/// `Fw::FilePacket::DataPacket::HEADERSIZE` — header + `U32` offset +
/// `U16` size.
pub const DATA_PACKET_HEADER_SIZE: usize = HEADER_SIZE + 4 + 2;

/// `Fw::FilePacket::PathName::MAX_LENGTH` — the length field is a `U8`.
pub const PATH_NAME_MAX_LENGTH: usize = 255;

/// The common packet header: `[u8 type][u32 sequenceIndex]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Header {
    /// The packet type.
    pub packet_type: FilePacketType,
    /// The packet's sequence index (always 0 for START).
    pub sequence_index: u32,
}

impl Header {
    /// C++ `Header::initialize`.
    #[must_use]
    pub const fn new(packet_type: FilePacketType, sequence_index: u32) -> Self {
        Self {
            packet_type,
            sequence_index,
        }
    }

    /// C++ `Header::bufferSize` — always [`HEADER_SIZE`].
    #[must_use]
    pub const fn buffer_size(&self) -> usize {
        HEADER_SIZE
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        let status = buf.serialize_u8(self.packet_type.as_repr(), Endianness::Big);
        if !status.is_ok() {
            return status;
        }
        buf.serialize_u32_be(self.sequence_index)
    }
}

/// START packet: `header + [u32 fileSize][PathName src][PathName dst]`.
///
/// The path slices borrow the source buffer when parsed (C++ aliases the
/// buffer through `PathName::m_value`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StartPacket<'a> {
    /// Header (`sequence_index` is always 0 — C++ `StartPacket::initialize`
    /// hard-codes it).
    pub header: Header,
    /// The size of the file being transferred.
    pub file_size: u32,
    /// The source (on-board) path, at most [`PATH_NAME_MAX_LENGTH`] bytes.
    pub source_path: &'a [u8],
    /// The destination path, at most [`PATH_NAME_MAX_LENGTH`] bytes.
    pub destination_path: &'a [u8],
}

impl<'a> StartPacket<'a> {
    /// C++ `StartPacket::initialize`: the sequence index is ALWAYS forced to
    /// 0 and each path is truncated to [`PATH_NAME_MAX_LENGTH`] bytes (the
    /// C++ `string_length(value, MAX_LENGTH)` cap).
    #[must_use]
    pub fn initialize(file_size: u32, source_path: &'a [u8], destination_path: &'a [u8]) -> Self {
        Self {
            header: Header::new(FilePacketType::Start, 0),
            file_size,
            source_path: truncate_path(source_path),
            destination_path: truncate_path(destination_path),
        }
    }

    /// C++ `StartPacket::bufferSize`.
    #[must_use]
    pub fn buffer_size(&self) -> usize {
        HEADER_SIZE + 4 + (1 + self.source_path.len()) + (1 + self.destination_path.len())
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        let status = self.header.serialize_to(buf);
        if !status.is_ok() {
            return status;
        }
        let status = buf.serialize_u32_be(self.file_size);
        if !status.is_ok() {
            return status;
        }
        let status = serialize_path(buf, self.source_path);
        if !status.is_ok() {
            return status;
        }
        serialize_path(buf, self.destination_path)
    }
}

/// DATA packet: `header + [u32 byteOffset][u16 dataSize][dataSize bytes]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataPacket<'a> {
    /// Header.
    pub header: Header,
    /// Offset of this chunk within the file.
    pub byte_offset: u32,
    /// Declared payload length; always equal to `data.len()`.
    pub data_size: u16,
    /// The payload, borrowed from the source buffer when parsed.
    pub data: &'a [u8],
}

impl<'a> DataPacket<'a> {
    /// C++ `DataPacket::initialize`. `data_size` is taken from the C++
    /// argument, and the payload slice is trimmed to it so the struct can
    /// never claim more bytes than it carries.
    #[must_use]
    pub fn initialize(
        sequence_index: u32,
        byte_offset: u32,
        data_size: u16,
        data: &'a [u8],
    ) -> Self {
        let len = core::cmp::min(data_size as usize, data.len());
        Self {
            header: Header::new(FilePacketType::Data, sequence_index),
            byte_offset,
            data_size: len as u16,
            data: &data[..len],
        }
    }

    /// C++ `DataPacket::fixedLengthSize` — [`DATA_PACKET_HEADER_SIZE`].
    #[must_use]
    pub const fn fixed_length_size(&self) -> usize {
        DATA_PACKET_HEADER_SIZE
    }

    /// C++ `DataPacket::bufferSize`.
    #[must_use]
    pub fn buffer_size(&self) -> usize {
        DATA_PACKET_HEADER_SIZE + self.data_size as usize
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        let status = self.header.serialize_to(buf);
        if !status.is_ok() {
            return status;
        }
        let status = buf.serialize_u32_be(self.byte_offset);
        if !status.is_ok() {
            return status;
        }
        let status = buf.serialize_u16_be(self.data_size);
        if !status.is_ok() {
            return status;
        }
        buf.serialize_bytes(self.data, LengthMode::OmitLength, Endianness::Big)
    }
}

/// END packet: `header + [u32 checksum]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndPacket {
    /// Header.
    pub header: Header,
    /// The CFDP checksum of the whole file.
    pub checksum_value: u32,
}

impl EndPacket {
    /// C++ `EndPacket::initialize` (which takes a `CFDP::Checksum` and
    /// stores `checksum.getValue()`).
    #[must_use]
    pub const fn initialize(sequence_index: u32, checksum_value: u32) -> Self {
        Self {
            header: Header::new(FilePacketType::End, sequence_index),
            checksum_value,
        }
    }

    /// C++ `EndPacket::bufferSize`.
    #[must_use]
    pub const fn buffer_size(&self) -> usize {
        HEADER_SIZE + 4
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        let status = self.header.serialize_to(buf);
        if !status.is_ok() {
            return status;
        }
        buf.serialize_u32_be(self.checksum_value)
    }
}

/// CANCEL packet: header only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CancelPacket {
    /// Header.
    pub header: Header,
}

impl CancelPacket {
    /// C++ `CancelPacket::initialize`.
    #[must_use]
    pub const fn initialize(sequence_index: u32) -> Self {
        Self {
            header: Header::new(FilePacketType::Cancel, sequence_index),
        }
    }

    /// C++ `CancelPacket::bufferSize`.
    #[must_use]
    pub const fn buffer_size(&self) -> usize {
        HEADER_SIZE
    }

    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        self.header.serialize_to(buf)
    }
}

/// One file packet, borrowing its variable-length fields from the buffer it
/// was parsed out of.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilePacket<'a> {
    /// START packet.
    Start(StartPacket<'a>),
    /// DATA packet.
    Data(DataPacket<'a>),
    /// END packet.
    End(EndPacket),
    /// CANCEL packet.
    Cancel(CancelPacket),
}

impl<'a> FilePacket<'a> {
    /// The packet's header (C++ `asHeader`).
    #[must_use]
    pub fn header(&self) -> Header {
        match self {
            FilePacket::Start(p) => p.header,
            FilePacket::Data(p) => p.header,
            FilePacket::End(p) => p.header,
            FilePacket::Cancel(p) => p.header,
        }
    }

    /// The packet's type (C++ `asHeader().getType()`).
    #[must_use]
    pub fn packet_type(&self) -> FilePacketType {
        self.header().packet_type
    }

    /// C++ `FilePacket::bufferSize` — the number of bytes
    /// [`serialize_to`](Self::serialize_to) writes.
    #[must_use]
    pub fn buffer_size(&self) -> usize {
        match self {
            FilePacket::Start(p) => p.buffer_size(),
            FilePacket::Data(p) => p.buffer_size(),
            FilePacket::End(p) => p.buffer_size(),
            FilePacket::Cancel(p) => p.buffer_size(),
        }
    }

    /// C++ `FilePacket::toBuffer` / `toSerialBuffer`: append the packet to
    /// `buf` (big-endian). Returns `NoRoomLeft` when the destination is too
    /// small; nothing is rolled back on a partial write (C++ parity).
    pub fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        match self {
            FilePacket::Start(p) => p.serialize_to(buf),
            FilePacket::Data(p) => p.serialize_to(buf),
            FilePacket::End(p) => p.serialize_to(buf),
            FilePacket::Cancel(p) => p.serialize_to(buf),
        }
    }

    /// C++ `FilePacket::fromBuffer` — parse exactly one packet out of
    /// `data`, borrowing its path/payload slices from it.
    ///
    /// Error statuses reproduce the C++ `SerialBuffer` ladder:
    /// `DeserBufferEmpty` when a read starts at the end of the buffer,
    /// `DeserSizeMismatch` when too few bytes remain, and
    /// `DeserSizeMismatch` when a packet that must consume the buffer
    /// exactly leaves residual bytes. An unknown type byte is
    /// `DeserFormatError` (the C++ `FW_ASSERT(false)` in the caller's
    /// dispatch switch — Rust rejects it as a decode error instead of
    /// crashing on ground-supplied data).
    pub fn from_buffer(data: &'a [u8]) -> Result<FilePacket<'a>, SerializeStatus> {
        let mut cursor = Cursor::new(data);
        let raw_type = cursor.read_u8()?;
        let sequence_index = cursor.read_u32()?;
        let packet_type =
            FilePacketType::try_from(raw_type).map_err(|_| SerializeStatus::DeserFormatError)?;
        let header = Header::new(packet_type, sequence_index);
        match packet_type {
            FilePacketType::Start => {
                let file_size = cursor.read_u32()?;
                let source_path = cursor.read_path()?;
                let destination_path = cursor.read_path()?;
                cursor.expect_empty()?;
                Ok(FilePacket::Start(StartPacket {
                    header,
                    file_size,
                    source_path,
                    destination_path,
                }))
            }
            FilePacketType::Data => {
                let byte_offset = cursor.read_u32()?;
                let data_size = cursor.read_u16()?;
                // C++ DataPacket::fromSerialBuffer: the REMAINING bytes must
                // equal dataSize exactly (not merely be enough).
                if cursor.remaining() != data_size as usize {
                    return Err(SerializeStatus::DeserSizeMismatch);
                }
                let payload = cursor.take_rest();
                Ok(FilePacket::Data(DataPacket {
                    header,
                    byte_offset,
                    data_size,
                    data: payload,
                }))
            }
            FilePacketType::End => {
                let checksum_value = cursor.read_u32()?;
                cursor.expect_empty()?;
                Ok(FilePacket::End(EndPacket {
                    header,
                    checksum_value,
                }))
            }
            FilePacketType::Cancel => {
                cursor.expect_empty()?;
                Ok(FilePacket::Cancel(CancelPacket { header }))
            }
            // T_NONE never appears on the wire.
            FilePacketType::None => Err(SerializeStatus::DeserFormatError),
        }
    }
}

/// C++ `PathName::initialize` caps the measured length at `MAX_LENGTH`.
fn truncate_path(path: &[u8]) -> &[u8] {
    if path.len() > PATH_NAME_MAX_LENGTH {
        &path[..PATH_NAME_MAX_LENGTH]
    } else {
        path
    }
}

/// C++ `PathName::toSerialBuffer`: `[u8 length][length raw bytes]`.
fn serialize_path(buf: &mut dyn SerBufAny, path: &[u8]) -> SerializeStatus {
    let path = truncate_path(path);
    let status = buf.serialize_u8(path.len() as u8, Endianness::Big);
    if !status.is_ok() {
        return status;
    }
    buf.serialize_bytes(path, LengthMode::OmitLength, Endianness::Big)
}

/// A read cursor over the borrowed source buffer, reproducing the
/// `Fw::SerializeBufferBase` deserialization statuses exactly: a read that
/// starts with nothing left is `DeserBufferEmpty`; a read with too few bytes
/// left is `DeserSizeMismatch`. Raw byte pops (`popBytes`, i.e.
/// `OMIT_LENGTH`) skip the empty check, so a short pop is always
/// `DeserSizeMismatch`.
struct Cursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }

    fn read_primitive(&mut self, len: usize) -> Result<&'a [u8], SerializeStatus> {
        if self.remaining() == 0 {
            return Err(SerializeStatus::DeserBufferEmpty);
        }
        if self.remaining() < len {
            return Err(SerializeStatus::DeserSizeMismatch);
        }
        let out = &self.data[self.pos..self.pos + len];
        self.pos += len;
        Ok(out)
    }

    fn read_u8(&mut self) -> Result<u8, SerializeStatus> {
        Ok(self.read_primitive(1)?[0])
    }

    fn read_u16(&mut self) -> Result<u16, SerializeStatus> {
        let bytes = self.read_primitive(2)?;
        Ok(u16::from_be_bytes([bytes[0], bytes[1]]))
    }

    fn read_u32(&mut self) -> Result<u32, SerializeStatus> {
        let bytes = self.read_primitive(4)?;
        Ok(u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    /// C++ `PathName::fromSerialBuffer`: `U8` length then `popBytes` of that
    /// many bytes, aliasing the buffer.
    fn read_path(&mut self) -> Result<&'a [u8], SerializeStatus> {
        let len = self.read_u8()? as usize;
        if self.remaining() < len {
            return Err(SerializeStatus::DeserSizeMismatch);
        }
        let out = &self.data[self.pos..self.pos + len];
        self.pos += len;
        Ok(out)
    }

    fn take_rest(&mut self) -> &'a [u8] {
        let out = &self.data[self.pos..];
        self.pos = self.data.len();
        out
    }

    /// The C++ `getDeserializeSizeLeft() != 0` check.
    fn expect_empty(&self) -> Result<(), SerializeStatus> {
        if self.remaining() == 0 {
            Ok(())
        } else {
            Err(SerializeStatus::DeserSizeMismatch)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::LinearBuffer;

    fn ser(packet: &FilePacket<'_>) -> Vec<u8> {
        let mut buf = LinearBuffer::<1024>::new();
        assert_eq!(packet.serialize_to(&mut buf), SerializeStatus::Ok);
        assert_eq!(buf.as_slice().len(), packet.buffer_size());
        buf.as_slice().to_vec()
    }

    #[test]
    fn start_packet_serializes_to_literal_bytes() {
        let packet = FilePacket::Start(StartPacket::initialize(1234, b"a.bin", b"/dest/b.bin"));
        let bytes = ser(&packet);
        let expected: Vec<u8> = [
            0x00, // T_START
            0x00, 0x00, 0x00, 0x00, // sequenceIndex (always 0)
            0x00, 0x00, 0x04, 0xD2, // fileSize = 1234
            0x05, b'a', b'.', b'b', b'i', b'n', // source path
            0x0B, b'/', b'd', b'e', b's', b't', b'/', b'b', b'.', b'b', b'i', b'n',
        ]
        .to_vec();
        assert_eq!(bytes, expected);
        assert_eq!(bytes.len(), 27);
    }

    #[test]
    fn data_packet_serializes_to_literal_bytes() {
        let payload = [0xAAu8, 0xBB, 0xCC];
        let packet = FilePacket::Data(DataPacket::initialize(7, 0x10, 3, &payload));
        assert_eq!(
            ser(&packet),
            vec![
                0x01, // T_DATA
                0x00, 0x00, 0x00, 0x07, // sequenceIndex
                0x00, 0x00, 0x00, 0x10, // byteOffset
                0x00, 0x03, // dataSize
                0xAA, 0xBB, 0xCC,
            ]
        );
    }

    #[test]
    fn end_packet_serializes_to_literal_bytes() {
        let packet = FilePacket::End(EndPacket::initialize(9, 0xDEAD_BEEF));
        assert_eq!(
            ser(&packet),
            vec![0x02, 0x00, 0x00, 0x00, 0x09, 0xDE, 0xAD, 0xBE, 0xEF]
        );
    }

    #[test]
    fn cancel_packet_serializes_to_literal_bytes() {
        let packet = FilePacket::Cancel(CancelPacket::initialize(5));
        assert_eq!(ser(&packet), vec![0x03, 0x00, 0x00, 0x00, 0x05]);
    }

    #[test]
    fn start_packet_round_trips() {
        let bytes = ser(&FilePacket::Start(StartPacket::initialize(
            77,
            b"src/path",
            b"dst",
        )));
        match FilePacket::from_buffer(&bytes).unwrap() {
            FilePacket::Start(p) => {
                assert_eq!(p.header.packet_type, FilePacketType::Start);
                assert_eq!(p.header.sequence_index, 0);
                assert_eq!(p.file_size, 77);
                assert_eq!(p.source_path, b"src/path");
                assert_eq!(p.destination_path, b"dst");
            }
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn data_packet_round_trips_with_borrowed_payload() {
        let payload: Vec<u8> = (0u8..40).collect();
        let bytes = ser(&FilePacket::Data(DataPacket::initialize(
            3, 100, 40, &payload,
        )));
        match FilePacket::from_buffer(&bytes).unwrap() {
            FilePacket::Data(p) => {
                assert_eq!(p.header.sequence_index, 3);
                assert_eq!(p.byte_offset, 100);
                assert_eq!(p.data_size, 40);
                assert_eq!(p.data, &payload[..]);
                // Zero-copy: the payload aliases the source buffer.
                assert!(std::ptr::eq(p.data.as_ptr(), bytes[11..].as_ptr()));
            }
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn end_and_cancel_round_trip() {
        let bytes = ser(&FilePacket::End(EndPacket::initialize(4, 0x0102_0304)));
        match FilePacket::from_buffer(&bytes).unwrap() {
            FilePacket::End(p) => {
                assert_eq!(p.header.sequence_index, 4);
                assert_eq!(p.checksum_value, 0x0102_0304);
            }
            other => panic!("wrong variant: {other:?}"),
        }
        let bytes = ser(&FilePacket::Cancel(CancelPacket::initialize(11)));
        match FilePacket::from_buffer(&bytes).unwrap() {
            FilePacket::Cancel(p) => assert_eq!(p.header.sequence_index, 11),
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn empty_buffer_deserialize_is_buffer_empty() {
        assert_eq!(
            FilePacket::from_buffer(&[]).unwrap_err(),
            SerializeStatus::DeserBufferEmpty
        );
    }

    #[test]
    fn short_header_deserialize_is_size_mismatch() {
        assert_eq!(
            FilePacket::from_buffer(&[0x03, 0x00, 0x00]).unwrap_err(),
            SerializeStatus::DeserSizeMismatch
        );
    }

    #[test]
    fn unknown_packet_type_is_format_error() {
        assert_eq!(
            FilePacket::from_buffer(&[0x09, 0, 0, 0, 0]).unwrap_err(),
            SerializeStatus::DeserFormatError
        );
        // T_NONE (255) is in-memory only and never valid on the wire.
        assert_eq!(
            FilePacket::from_buffer(&[0xFF, 0, 0, 0, 0]).unwrap_err(),
            SerializeStatus::DeserFormatError
        );
    }

    #[test]
    fn start_end_and_cancel_demand_exact_consumption() {
        let mut start = ser(&FilePacket::Start(StartPacket::initialize(1, b"a", b"b")));
        start.push(0x00);
        assert_eq!(
            FilePacket::from_buffer(&start).unwrap_err(),
            SerializeStatus::DeserSizeMismatch
        );

        let mut end = ser(&FilePacket::End(EndPacket::initialize(1, 2)));
        end.push(0x00);
        assert_eq!(
            FilePacket::from_buffer(&end).unwrap_err(),
            SerializeStatus::DeserSizeMismatch
        );

        let mut cancel = ser(&FilePacket::Cancel(CancelPacket::initialize(1)));
        cancel.push(0x00);
        assert_eq!(
            FilePacket::from_buffer(&cancel).unwrap_err(),
            SerializeStatus::DeserSizeMismatch
        );
    }

    #[test]
    fn data_packet_remaining_must_equal_data_size() {
        let payload = [1u8, 2, 3];
        let good = ser(&FilePacket::Data(DataPacket::initialize(1, 0, 3, &payload)));

        let mut too_long = good.clone();
        too_long.push(4);
        assert_eq!(
            FilePacket::from_buffer(&too_long).unwrap_err(),
            SerializeStatus::DeserSizeMismatch
        );

        let too_short = &good[..good.len() - 1];
        assert_eq!(
            FilePacket::from_buffer(too_short).unwrap_err(),
            SerializeStatus::DeserSizeMismatch
        );

        // A zero-length DATA packet is legal (remaining == 0 == dataSize).
        let empty = ser(&FilePacket::Data(DataPacket::initialize(1, 0, 0, &[])));
        assert_eq!(empty.len(), DATA_PACKET_HEADER_SIZE);
        match FilePacket::from_buffer(&empty).unwrap() {
            FilePacket::Data(p) => assert_eq!(p.data.len(), 0),
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn path_name_length_beyond_buffer_is_size_mismatch() {
        // START claiming a 200-byte source path with only 3 bytes present.
        let bytes = [
            0x00, 0x00, 0x00, 0x00, 0x00, // header
            0x00, 0x00, 0x00, 0x01, // fileSize
            0xC8, b'a', b'b', b'c', // path length 200, 3 bytes present
        ];
        assert_eq!(
            FilePacket::from_buffer(&bytes).unwrap_err(),
            SerializeStatus::DeserSizeMismatch
        );
    }

    #[test]
    fn empty_path_names_round_trip() {
        let bytes = ser(&FilePacket::Start(StartPacket::initialize(0, b"", b"")));
        assert_eq!(bytes, vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        match FilePacket::from_buffer(&bytes).unwrap() {
            FilePacket::Start(p) => {
                assert!(p.source_path.is_empty());
                assert!(p.destination_path.is_empty());
            }
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn path_names_truncate_at_255_bytes() {
        let long = vec![b'x'; 300];
        let packet = StartPacket::initialize(1, &long, &long);
        assert_eq!(packet.source_path.len(), PATH_NAME_MAX_LENGTH);
        assert_eq!(packet.destination_path.len(), PATH_NAME_MAX_LENGTH);
        assert_eq!(packet.buffer_size(), 5 + 4 + 256 + 256);
        let bytes = ser(&FilePacket::Start(packet));
        assert_eq!(bytes[9], 0xFF);
        assert_eq!(bytes[9 + 256], 0xFF);
    }

    #[test]
    fn serialize_into_short_buffer_is_no_room_left() {
        let mut buf = LinearBuffer::<4>::new();
        let packet = FilePacket::Cancel(CancelPacket::initialize(1));
        assert_eq!(packet.serialize_to(&mut buf), SerializeStatus::NoRoomLeft);
    }

    #[test]
    fn data_packet_initialize_trims_payload_to_data_size() {
        let payload = [1u8, 2, 3, 4, 5];
        let packet = DataPacket::initialize(1, 0, 2, &payload);
        assert_eq!(packet.data, &[1, 2]);
        assert_eq!(packet.data_size, 2);
        assert_eq!(packet.buffer_size(), DATA_PACKET_HEADER_SIZE + 2);
    }

    #[test]
    fn header_and_packet_type_accessors() {
        let packet = FilePacket::End(EndPacket::initialize(6, 0));
        assert_eq!(packet.packet_type(), FilePacketType::End);
        assert_eq!(packet.header().sequence_index, 6);
        assert_eq!(FilePacketType::None.as_repr(), 255);
        assert_eq!(HEADER_SIZE, 5);
        assert_eq!(DATA_PACKET_HEADER_SIZE, 11);
    }
}
