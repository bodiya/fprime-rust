//! Data products: `Fw::DpContainer` and the `Fw/Dp/Dp.fpp` port traits.
//!
//! Port of `Fw/Dp/DpContainer.{hpp,cpp}`, `Fw/Dp/Dp.fpp` and
//! `default/config/DpCfg.fpp`. Analysis:
//! `docs/cpp-analysis/utils-misc.md` (section "Fw::DpContainer (data product
//! packet)" and wire-format section "DpContainer packet") and
//! `docs/cpp-analysis/data-products.md`.
//!
//! A data-product packet is
//! `[header (57 B) | header hash (4 B) | data | data hash (4 B)]`, all
//! big-endian; the `.fdp` file written by `Svc::DpWriter` *is* this packet
//! with no extra framing. Every offset here is derived from the config
//! constants (`FwPacketDescriptorType`, `FwDpIdType`, `FwDpPriorityType`,
//! `Time::SERIALIZED_SIZE`, [`CONTAINER_USER_DATA_SIZE`],
//! `FwSizeStoreType`), never written as a magic number, so a config
//! override keeps working.
//!
//! # Deviations from C++
//!
//! * The C++ `Fw::Buffer` is a non-owning descriptor, so `DpContainer`
//!   holds a copy of it and the caller keeps the storage alive. The Rust
//!   `Buffer` owns its storage, so the container OWNS the buffer:
//!   [`DpContainer::set_buffer`] takes it by value and
//!   [`DpContainer::take_buffer`] (the Rust form of C++ `invalidateBuffer`)
//!   gives it back.
//! * C++ keeps an `ExternalSerializeBuffer` member pointing into the packet
//!   data region (`m_dataBuffer`); Rust cannot hold that alias, so the data
//!   region is reached on demand through [`DpContainer::data_serializer`] /
//!   [`DpContainer::data_region_mut`] and the data size is tracked
//!   explicitly with [`DpContainer::set_data_size`], exactly as the C++
//!   `m_dataSize` is.
//! * Hashes are exchanged as `u32` rather than `Utils::HashBuffer`: the
//!   digest is 4 bytes stored big-endian, so `HashBuffer::asBigEndianU32()`
//!   (what every event argument uses) is the same number. The CRC-32
//!   implementation is duplicated here because `fprime-fw` may not depend on
//!   `fprime-utils` (the dependency DAG points the other way) — the same
//!   duplication `fprime-os` already carries for `File::calculate_crc`.

use crate::buffer::Buffer;
use crate::com::ComPacketType;
use crate::enums::Success;
use crate::serial::{Endianness, ExtBuf, LengthMode, SerBuf, SerializeStatus};
use crate::time::Time;
use crate::{fpp_enum, fw_assert};
use fprime_config::{
    FwDpIdType, FwDpPriorityType, FwIndexType, FwPacketDescriptorType, FwSizeStoreType, FwSizeType,
};

// ---------------------------------------------------------------------------
// Configuration (default/config/DpCfg.fpp, Utils/Hash/HashConfig.hpp)
// ---------------------------------------------------------------------------

/// `Fw::DpCfg::CONTAINER_USER_DATA_SIZE` — the size in bytes of the
/// user-configurable data in the container packet header.
pub const CONTAINER_USER_DATA_SIZE: usize = 32;

/// `HASH_DIGEST_LENGTH` for the default CRC-32 hash (mirrors
/// `fprime_utils::hash::HASH_DIGEST_LENGTH`; duplicated because `fprime-fw`
/// cannot depend on `fprime-utils`).
pub const HASH_DIGEST_LENGTH: usize = 4;

fpp_enum! {
    /// `Fw.DpState` — the transmission state of a data product.
    pub enum DpState : u8 {
        /// The untransmitted state.
        Untransmitted = 0,
        /// The partially transmitted state: from the start of transmission
        /// until transmission is complete.
        Partial = 1,
        /// The transmitted state.
        Transmitted = 2,
    }
    default Untransmitted
}

fpp_enum! {
    /// `Fw.DpCfg.ProcType` — a **bit mask** selecting the processing to
    /// perform on a container before writing it to disk.
    ///
    /// The `procTypes` header field stores the mask (`SerialType` = `u8`),
    /// not a single enum value, so it is serialized as a raw `u8`; this enum
    /// names the bits. `Svc::DpWriter` fans out on bit *index*: bit 0
    /// ([`ProcType::ZlibDeflate`]) selects `procBufferSendOut` port 0, bit 1
    /// ([`ProcType::One`]) port 1, and so on.
    pub enum ProcType : u8 {
        /// No processing.
        None = 0x00,
        /// Processing type 0 (zlib deflate; the compressor itself is out of
        /// scope for this port — see the module notes in `Svc::DpWriter`).
        ZlibDeflate = 0x01,
        /// Processing type 1.
        One = 0x02,
        /// Processing type 2.
        Two = 0x04,
    }
    default None
}

// ---------------------------------------------------------------------------
// Header layout
// ---------------------------------------------------------------------------

/// Offsets of the `Fw::DpContainer::Header` fields, derived from the config
/// types (C++ `DpContainer::Header`).
pub struct Header;

impl Header {
    /// Offset of the packet descriptor field (`FwPacketDescriptorType`).
    pub const PACKET_DESCRIPTOR_OFFSET: usize = 0;
    /// Offset of the container id field (`FwDpIdType`).
    pub const ID_OFFSET: usize =
        Self::PACKET_DESCRIPTOR_OFFSET + size_of::<FwPacketDescriptorType>();
    /// Offset of the priority field (`FwDpPriorityType`).
    pub const PRIORITY_OFFSET: usize = Self::ID_OFFSET + size_of::<FwDpIdType>();
    /// Offset of the time tag field (`Fw::Time`, 11 bytes).
    pub const TIME_TAG_OFFSET: usize = Self::PRIORITY_OFFSET + size_of::<FwDpPriorityType>();
    /// Offset of the processing-types bit mask (`ProcType::SerialType`).
    pub const PROC_TYPES_OFFSET: usize = Self::TIME_TAG_OFFSET + Time::SERIALIZED_SIZE;
    /// Offset of the user data field.
    pub const USER_DATA_OFFSET: usize = Self::PROC_TYPES_OFFSET + size_of::<u8>();
    /// Offset of the data-product state field.
    pub const DP_STATE_OFFSET: usize = Self::USER_DATA_OFFSET + CONTAINER_USER_DATA_SIZE;
    /// Offset of the data size field (`FwSizeStoreType` on the wire).
    pub const DATA_SIZE_OFFSET: usize = Self::DP_STATE_OFFSET + DpState::SERIALIZED_SIZE;
    /// The header size (57 bytes with the default config).
    pub const SIZE: usize = Self::DATA_SIZE_OFFSET + size_of::<FwSizeStoreType>();
}

/// A data-product packet view over an owned [`Buffer`]
/// (C++ `Fw::DpContainer`).
#[derive(Debug)]
pub struct DpContainer {
    /// The user data field (`m_userData`), 32 raw bytes with no length
    /// token. Public exactly as in C++.
    pub user_data: [u8; CONTAINER_USER_DATA_SIZE],
    id: FwDpIdType,
    priority: FwDpPriorityType,
    time_tag: Time,
    proc_types: u8,
    dp_state: DpState,
    data_size: FwSizeType,
    buffer: Buffer,
}

impl Default for DpContainer {
    fn default() -> Self {
        Self::new()
    }
}

impl DpContainer {
    /// Offset of the header hash (immediately after the header).
    pub const HEADER_HASH_OFFSET: usize = Header::SIZE;
    /// Offset of the packet data.
    pub const DATA_OFFSET: usize = Self::HEADER_HASH_OFFSET + HASH_DIGEST_LENGTH;
    /// The minimum packet size = the number of non-data bytes in a packet
    /// (header + header hash + data hash = 65 with the default config).
    pub const MIN_PACKET_SIZE: usize = Header::SIZE + 2 * HASH_DIGEST_LENGTH;

    /// The packet size for a given data size (C++
    /// `getPacketSizeForDataSize`).
    pub const fn packet_size_for_data_size(data_size: FwSizeType) -> FwSizeType {
        Header::SIZE as FwSizeType + data_size + 2 * HASH_DIGEST_LENGTH as FwSizeType
    }

    /// A container with default initialization and no buffer (C++ default
    /// constructor).
    pub fn new() -> Self {
        Self {
            user_data: [0u8; CONTAINER_USER_DATA_SIZE],
            id: 0,
            priority: 0,
            time_tag: Time::ZERO,
            proc_types: ProcType::None.as_repr(),
            dp_state: DpState::Untransmitted,
            data_size: 0,
            buffer: Buffer::empty(),
        }
    }

    /// A container with an id and a packet buffer (C++
    /// `DpContainer(id, buffer)`); `fw_assert`s that the buffer can hold a
    /// zero-data packet.
    pub fn with_buffer(id: FwDpIdType, buffer: Buffer) -> Self {
        let mut container = Self::new();
        container.id = id;
        container.set_buffer(buffer);
        container
    }

    // -- Field accessors ---------------------------------------------------

    /// The container id.
    pub fn id(&self) -> FwDpIdType {
        self.id
    }

    /// Set the container id.
    pub fn set_id(&mut self, id: FwDpIdType) {
        self.id = id;
    }

    /// The priority.
    pub fn priority(&self) -> FwDpPriorityType {
        self.priority
    }

    /// Set the priority.
    pub fn set_priority(&mut self, priority: FwDpPriorityType) {
        self.priority = priority;
    }

    /// The time tag.
    pub fn time_tag(&self) -> Time {
        self.time_tag
    }

    /// Set the time tag.
    pub fn set_time_tag(&mut self, time_tag: Time) {
        self.time_tag = time_tag;
    }

    /// The processing-types bit mask (see [`ProcType`]).
    pub fn proc_types(&self) -> u8 {
        self.proc_types
    }

    /// Set the processing-types bit mask.
    pub fn set_proc_types(&mut self, proc_types: u8) {
        self.proc_types = proc_types;
    }

    /// The data-product state (C++ `getState` / `getDpState`).
    pub fn state(&self) -> DpState {
        self.dp_state
    }

    /// Set the data-product state.
    pub fn set_dp_state(&mut self, dp_state: DpState) {
        self.dp_state = dp_state;
    }

    /// The data size.
    pub fn data_size(&self) -> FwSizeType {
        self.data_size
    }

    /// Set the data size (the number of valid bytes in the data region).
    pub fn set_data_size(&mut self, data_size: FwSizeType) {
        self.data_size = data_size;
    }

    /// The packet size corresponding to the current data size.
    pub fn packet_size(&self) -> FwSizeType {
        Self::packet_size_for_data_size(self.data_size)
    }

    /// The offset of the data hash (after the header, header hash and data).
    pub fn data_hash_offset(&self) -> FwSizeType {
        (Header::SIZE + HASH_DIGEST_LENGTH) as FwSizeType + self.data_size
    }

    // -- Buffer management -------------------------------------------------

    /// The packet buffer.
    pub fn buffer(&self) -> &Buffer {
        &self.buffer
    }

    /// The packet buffer, mutable.
    pub fn buffer_mut(&mut self) -> &mut Buffer {
        &mut self.buffer
    }

    /// Set the packet buffer (C++ `setBuffer`): `fw_assert`s that the buffer
    /// holds at least [`Self::MIN_PACKET_SIZE`] bytes and resets the data
    /// size to 0 (`Svc::DpWriter` relies on that reset before reading the
    /// header back).
    pub fn set_buffer(&mut self, buffer: Buffer) {
        let buffer_size = buffer.size();
        fw_assert!(
            buffer_size >= Self::MIN_PACKET_SIZE,
            buffer_size as i32,
            Self::MIN_PACKET_SIZE as i32
        );
        self.buffer = buffer;
        self.data_size = 0;
    }

    /// Take the packet buffer back, leaving the container without one
    /// (the Rust form of C++ `invalidateBuffer`, which cannot return the
    /// borrowed storage).
    pub fn take_buffer(&mut self) -> Buffer {
        self.data_size = 0;
        std::mem::take(&mut self.buffer)
    }

    /// Shrink the packet buffer to the packet size implied by the header
    /// (C++ `shrinkBufferSize`); `fw_assert`s that this is a shrink and that
    /// a minimum packet still fits (growing an `Fw::Buffer` is not safe).
    pub fn shrink_buffer_size(&mut self) {
        let new_size = self.packet_size();
        fw_assert!(
            new_size >= Self::MIN_PACKET_SIZE as FwSizeType,
            new_size as i32
        );
        fw_assert!(
            new_size <= self.buffer.size() as FwSizeType,
            new_size as i32,
            self.buffer.size() as i32
        );
        self.buffer.set_size(new_size as usize);
    }

    /// The capacity of the data region (`bufferSize - MIN_PACKET_SIZE`).
    pub fn data_capacity(&self) -> FwSizeType {
        (self.buffer.size().saturating_sub(Self::MIN_PACKET_SIZE)) as FwSizeType
    }

    /// The valid data bytes (`dataSize` bytes at [`Self::DATA_OFFSET`]).
    pub fn data(&self) -> &[u8] {
        let end = Self::DATA_OFFSET + self.data_size as usize;
        fw_assert!(
            end <= self.buffer.size(),
            end as i32,
            self.buffer.size() as i32
        );
        &self.buffer.data()[Self::DATA_OFFSET..end]
    }

    /// The whole data region (capacity bytes at [`Self::DATA_OFFSET`]),
    /// mutable — the C++ `m_dataBuffer` storage.
    pub fn data_region_mut(&mut self) -> &mut [u8] {
        let end = Self::DATA_OFFSET + self.data_capacity() as usize;
        &mut self.buffer.data_mut()[Self::DATA_OFFSET..end]
    }

    /// A serializer over the data region, write cursor at 0 (the C++
    /// `m_dataBuffer`). The caller updates [`Self::set_data_size`] with the
    /// number of bytes written, as the C++ record-append code does.
    pub fn data_serializer(&mut self) -> ExtBuf<'_> {
        ExtBuf::new(self.data_region_mut())
    }

    // -- Header serialization ----------------------------------------------

    /// Serialize the header into the packet buffer and update the header
    /// hash (C++ `serializeHeader`). Buffer must be valid and at least
    /// [`Self::MIN_PACKET_SIZE`] bytes; serialization failures are
    /// programmer errors and `fw_assert`.
    pub fn serialize_header(&mut self) {
        fw_assert!(self.buffer.is_valid());
        {
            let mut ser = self.buffer.get_serializer();
            let status = ser.serialize_u16_be(ComPacketType::FwPacketDp.as_repr());
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize_u32_be(self.id);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize_u32_be(self.priority);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize(&self.time_tag, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize_u8_be(self.proc_types);
            fw_assert!(status.is_ok(), status as i32);
            let status =
                ser.serialize_bytes(&self.user_data, LengthMode::OmitLength, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize(&self.dp_state, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize_size(self.data_size, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
        }
        self.update_header_hash();
    }

    /// Deserialize the header from the packet buffer (C++
    /// `deserializeHeader`). Call [`Self::check_header_hash`] first. Like
    /// C++, fields are committed as they are read, so a mid-header failure
    /// leaves the earlier fields updated.
    pub fn deserialize_header(&mut self) -> SerializeStatus {
        fw_assert!(self.buffer.is_valid());
        let mut de = self.buffer.get_deserializer();
        let mut status = de.move_deser_to_offset(Header::PACKET_DESCRIPTOR_OFFSET);
        if status.is_ok() {
            let mut descriptor: FwPacketDescriptorType = 0;
            status = de.deserialize_u16_be(&mut descriptor);
            if status.is_ok() && descriptor != ComPacketType::FwPacketDp.as_repr() {
                status = SerializeStatus::FormatError;
            }
        }
        if status.is_ok() {
            status = de.deserialize_u32_be(&mut self.id);
        }
        if status.is_ok() {
            status = de.deserialize_u32_be(&mut self.priority);
        }
        if status.is_ok() {
            status = de.deserialize(&mut self.time_tag, Endianness::Big);
        }
        if status.is_ok() {
            status = de.deserialize_u8_be(&mut self.proc_types);
        }
        if status.is_ok() {
            let requested = CONTAINER_USER_DATA_SIZE;
            let mut received = requested;
            status = de.deserialize_bytes(
                &mut self.user_data,
                &mut received,
                LengthMode::OmitLength,
                Endianness::Big,
            );
            if received != requested {
                status = SerializeStatus::DeserSizeMismatch;
            }
        }
        if status.is_ok() {
            status = de.deserialize(&mut self.dp_state, Endianness::Big);
        }
        if status.is_ok() {
            status = de.deserialize_size(&mut self.data_size, Endianness::Big);
        }
        status
    }

    // -- Hashes ------------------------------------------------------------

    /// The stored header hash (4 bytes big-endian at
    /// [`Self::HEADER_HASH_OFFSET`]).
    pub fn header_hash(&self) -> u32 {
        let min = Self::HEADER_HASH_OFFSET + HASH_DIGEST_LENGTH;
        fw_assert!(
            self.buffer.size() >= min,
            self.buffer.size() as i32,
            min as i32
        );
        read_u32_be(&self.buffer.data()[Self::HEADER_HASH_OFFSET..])
    }

    /// The CRC-32 of the header bytes `[0, Header::SIZE)`.
    pub fn compute_header_hash(&self) -> u32 {
        fw_assert!(
            self.buffer.size() >= Header::SIZE,
            self.buffer.size() as i32,
            Header::SIZE as i32
        );
        crc32(&self.buffer.data()[..Header::SIZE])
    }

    /// Store a header hash.
    pub fn set_header_hash(&mut self, hash: u32) {
        let min = Self::HEADER_HASH_OFFSET + HASH_DIGEST_LENGTH;
        fw_assert!(
            self.buffer.size() >= min,
            self.buffer.size() as i32,
            min as i32
        );
        let bytes = hash.to_be_bytes();
        self.buffer.data_mut()[Self::HEADER_HASH_OFFSET..min].copy_from_slice(&bytes);
    }

    /// Compute and store the header hash.
    pub fn update_header_hash(&mut self) {
        let hash = self.compute_header_hash();
        self.set_header_hash(hash);
    }

    /// Compare the stored and computed header hashes
    /// (C++ `checkHeaderHash`); returns `(status, stored, computed)`.
    pub fn check_header_hash(&self) -> (Success, u32, u32) {
        let stored = self.header_hash();
        let computed = self.compute_header_hash();
        let status = if stored == computed {
            Success::Success
        } else {
            Success::Failure
        };
        (status, stored, computed)
    }

    /// The stored data hash (4 bytes big-endian at
    /// [`Self::data_hash_offset`]).
    pub fn data_hash(&self) -> u32 {
        let offset = self.data_hash_offset() as usize;
        let end = offset + HASH_DIGEST_LENGTH;
        fw_assert!(
            end <= self.buffer.size(),
            end as i32,
            self.buffer.size() as i32
        );
        read_u32_be(&self.buffer.data()[offset..])
    }

    /// The CRC-32 of the data bytes.
    pub fn compute_data_hash(&self) -> u32 {
        crc32(self.data())
    }

    /// Store a data hash.
    pub fn set_data_hash(&mut self, hash: u32) {
        let offset = self.data_hash_offset() as usize;
        let end = offset + HASH_DIGEST_LENGTH;
        fw_assert!(
            end <= self.buffer.size(),
            end as i32,
            self.buffer.size() as i32
        );
        let bytes = hash.to_be_bytes();
        self.buffer.data_mut()[offset..end].copy_from_slice(&bytes);
    }

    /// Compute and store the data hash.
    pub fn update_data_hash(&mut self) {
        let hash = self.compute_data_hash();
        self.set_data_hash(hash);
    }

    /// Compare the stored and computed data hashes (C++ `checkDataHash`);
    /// returns `(status, stored, computed)`.
    pub fn check_data_hash(&self) -> (Success, u32, u32) {
        let stored = self.data_hash();
        let computed = self.compute_data_hash();
        let status = if stored == computed {
            Success::Success
        } else {
            Success::Failure
        };
        (status, stored, computed)
    }
}

/// Read a big-endian `u32` from the first four bytes of `data`.
fn read_u32_be(data: &[u8]) -> u32 {
    fw_assert!(data.len() >= HASH_DIGEST_LENGTH, data.len() as i32);
    u32::from_be_bytes([data[0], data[1], data[2], data[3]])
}

// ---------------------------------------------------------------------------
// CRC-32 (duplicated from fprime-utils; see the module header)
// ---------------------------------------------------------------------------

/// The 256-entry Sarwate table for the reflected IEEE 802.3 polynomial
/// `0xEDB88320` (identical to `fprime_utils::hash`'s table).
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

/// Standard (complemented) CRC-32 over `data`, the value
/// `Utils::Hash::hash()` stores big-endian in a `HashBuffer`.
pub fn crc32(data: &[u8]) -> u32 {
    let mut register = 0xFFFF_FFFFu32;
    for &byte in data {
        register = (register >> 8) ^ CRC32_TABLE[((register ^ u32::from(byte)) & 0xFF) as usize];
    }
    !register
}

// ---------------------------------------------------------------------------
// Fw/Dp/Dp.fpp ports
// ---------------------------------------------------------------------------
//
// These four port traits live here (rather than in `fprime-comp`'s port set)
// because they are declared in `Fw/Dp/Dp.fpp` alongside `DpState` and are
// only meaningful together with `DpContainer`. `Svc::DpWritten` lives with
// its emitter in `fprime_svc::dp_writer`.

/// `Fw.DpGet` — synchronously get a data-product buffer.
///
/// On return the buffer is valid and large enough to hold a packet with the
/// requested data size (status `Success`), or invalid (status `Failure`).
/// The FPP `ref buffer: Fw.Buffer` output parameter is a `&mut Buffer`.
pub trait DpGetPort: Send + Sync {
    /// Invoke the port.
    fn invoke(
        &self,
        port_num: FwIndexType,
        id: FwDpIdType,
        data_size: FwSizeType,
        buffer: &mut Buffer,
    ) -> Success;
}

/// `Fw.DpRequest` — asynchronously request a data-product buffer; the
/// answer arrives on [`DpResponsePort`].
pub trait DpRequestPort: Send + Sync {
    /// Invoke the port.
    fn invoke(&self, port_num: FwIndexType, id: FwDpIdType, data_size: FwSizeType);
}

/// `Fw.DpResponse` — the response to a [`DpRequestPort`] request.
pub trait DpResponsePort: Send + Sync {
    /// Invoke the port. The buffer moves to the callee; on `Failure` it is
    /// an invalid (empty) buffer.
    fn invoke(&self, port_num: FwIndexType, id: FwDpIdType, buffer: Buffer, status: Success);
}

/// `Fw.DpSend` — send a filled data-product buffer.
pub trait DpSendPort: Send + Sync {
    /// Invoke the port; the buffer moves to the callee.
    fn invoke(&self, port_num: FwIndexType, id: FwDpIdType, buffer: Buffer);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::TimeBase;

    /// The 57-byte header of the reference packet used below.
    const REF_HEADER: [u8; 57] = [
        0x00, 0x05, // descriptor = FW_PACKET_DP
        0x00, 0x00, 0x00, 0x01, // id = 1
        0x00, 0x00, 0x00, 0x02, // priority = 2
        0x00, 0x02, // time base = TB_WORKSTATION_TIME
        0x03, // time context = 3
        0x00, 0x00, 0x00, 0x04, // seconds = 4
        0x00, 0x00, 0x00, 0x05, // useconds = 5
        0x03, // procTypes = ZLIB_DEFLATE | ONE
        0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // userData[0..8]
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // userData[8..16]
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // userData[16..24]
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB, // userData[24..32]
        0x01, // DpState = PARTIAL
        0x00, 0x04, // dataSize = 4 (FwSizeStoreType)
    ];
    /// CRC-32 of [`REF_HEADER`].
    const REF_HEADER_HASH: u32 = 0x3779_D5A5;
    /// CRC-32 of `DE AD BE EF`.
    const REF_DATA_HASH: u32 = 0x7C9C_A35A;

    fn reference_container() -> DpContainer {
        let mut container = DpContainer::with_buffer(1, Buffer::allocate(69));
        container.set_priority(2);
        container.set_time_tag(Time::new(TimeBase::TbWorkstationTime, 3, 4, 5));
        container.set_proc_types(ProcType::ZlibDeflate.as_repr() | ProcType::One.as_repr());
        container.user_data[0] = 0xAA;
        container.user_data[CONTAINER_USER_DATA_SIZE - 1] = 0xBB;
        container.set_dp_state(DpState::Partial);
        container.data_region_mut()[..4].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
        container.set_data_size(4);
        container
    }

    #[test]
    fn header_offsets_derive_from_the_default_config() {
        assert_eq!(Header::PACKET_DESCRIPTOR_OFFSET, 0);
        assert_eq!(Header::ID_OFFSET, 2);
        assert_eq!(Header::PRIORITY_OFFSET, 6);
        assert_eq!(Header::TIME_TAG_OFFSET, 10);
        assert_eq!(Header::PROC_TYPES_OFFSET, 21);
        assert_eq!(Header::USER_DATA_OFFSET, 22);
        assert_eq!(Header::DP_STATE_OFFSET, 54);
        assert_eq!(Header::DATA_SIZE_OFFSET, 55);
        assert_eq!(Header::SIZE, 57);
        assert_eq!(DpContainer::HEADER_HASH_OFFSET, 57);
        assert_eq!(DpContainer::DATA_OFFSET, 61);
        assert_eq!(DpContainer::MIN_PACKET_SIZE, 65);
        assert_eq!(DpContainer::packet_size_for_data_size(0), 65);
        assert_eq!(DpContainer::packet_size_for_data_size(4), 69);
    }

    #[test]
    fn proc_type_values_are_a_bit_mask() {
        assert_eq!(ProcType::None.as_repr(), 0x00);
        assert_eq!(ProcType::ZlibDeflate.as_repr(), 0x01);
        assert_eq!(ProcType::One.as_repr(), 0x02);
        assert_eq!(ProcType::Two.as_repr(), 0x04);
        assert_eq!(DpState::Untransmitted.as_repr(), 0);
        assert_eq!(DpState::Partial.as_repr(), 1);
        assert_eq!(DpState::Transmitted.as_repr(), 2);
    }

    #[test]
    fn crc32_matches_the_standard_vectors() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
        assert_eq!(crc32(&REF_HEADER), REF_HEADER_HASH);
    }

    #[test]
    fn serialize_header_writes_the_exact_packet_bytes() {
        let mut container = reference_container();
        container.serialize_header();
        container.update_data_hash();
        let packet = container.buffer().data();
        assert_eq!(&packet[..57], &REF_HEADER[..]);
        assert_eq!(&packet[57..61], &REF_HEADER_HASH.to_be_bytes());
        assert_eq!(&packet[61..65], &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(&packet[65..69], &REF_DATA_HASH.to_be_bytes());
        assert_eq!(container.packet_size(), 69);
        assert_eq!(container.data_hash_offset(), 65);
    }

    #[test]
    fn minimum_packet_has_a_zero_data_size_and_both_hashes() {
        let mut container = DpContainer::with_buffer(0, Buffer::allocate(65));
        container.serialize_header();
        container.update_data_hash();
        let packet = container.buffer().data();
        assert_eq!(packet.len(), 65);
        assert_eq!(&packet[..2], &[0x00, 0x05]);
        assert_eq!(&packet[2..57], &[0u8; 55]);
        // CRC-32 of 57 zero-ish header bytes (descriptor 0x0005 only).
        assert_eq!(&packet[57..61], &0xB610_8297u32.to_be_bytes());
        // CRC-32 of no data at all is 0.
        assert_eq!(&packet[61..65], &[0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn deserialize_header_round_trips_every_field() {
        let mut container = reference_container();
        container.serialize_header();
        let buffer = container.take_buffer();

        let mut read_back = DpContainer::new();
        read_back.set_buffer(buffer);
        assert_eq!(read_back.data_size(), 0); // set_buffer resets it
        let status = read_back.deserialize_header();
        assert_eq!(status, SerializeStatus::Ok);
        assert_eq!(read_back.id(), 1);
        assert_eq!(read_back.priority(), 2);
        assert_eq!(read_back.time_tag().get_seconds(), 4);
        assert_eq!(read_back.time_tag().get_useconds(), 5);
        assert_eq!(
            read_back.time_tag().get_time_base(),
            TimeBase::TbWorkstationTime
        );
        assert_eq!(read_back.time_tag().get_context(), 3);
        assert_eq!(read_back.proc_types(), 0x03);
        assert_eq!(read_back.user_data[0], 0xAA);
        assert_eq!(read_back.user_data[CONTAINER_USER_DATA_SIZE - 1], 0xBB);
        assert_eq!(read_back.state(), DpState::Partial);
        assert_eq!(read_back.data_size(), 4);
        assert_eq!(read_back.data(), &[0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn deserialize_header_rejects_a_bad_packet_descriptor() {
        let mut container = reference_container();
        container.serialize_header();
        container.buffer_mut().data_mut()[1] = 0x06;
        let status = container.deserialize_header();
        assert_eq!(status, SerializeStatus::FormatError);
    }

    #[test]
    fn deserialize_header_rejects_out_of_range_useconds() {
        let mut container = reference_container();
        container.serialize_header();
        // useconds = 1_000_000 is rejected by Fw::Time.
        container.buffer_mut().data_mut()[17..21].copy_from_slice(&1_000_000u32.to_be_bytes());
        let status = container.deserialize_header();
        assert_eq!(status, SerializeStatus::DeserFormatError);
    }

    #[test]
    fn deserialize_header_rejects_an_invalid_dp_state() {
        let mut container = reference_container();
        container.serialize_header();
        container.buffer_mut().data_mut()[54] = 0x07;
        let status = container.deserialize_header();
        assert_eq!(status, SerializeStatus::DeserFormatError);
    }

    #[test]
    fn check_header_hash_detects_corruption() {
        let mut container = reference_container();
        container.serialize_header();
        let (status, stored, computed) = container.check_header_hash();
        assert_eq!(status, Success::Success);
        assert_eq!(stored, REF_HEADER_HASH);
        assert_eq!(computed, REF_HEADER_HASH);

        container.buffer_mut().data_mut()[Header::ID_OFFSET] = 0xFF;
        let (status, stored, computed) = container.check_header_hash();
        assert_eq!(status, Success::Failure);
        assert_eq!(stored, REF_HEADER_HASH);
        assert_ne!(computed, REF_HEADER_HASH);
    }

    #[test]
    fn check_data_hash_detects_corruption() {
        let mut container = reference_container();
        container.serialize_header();
        container.update_data_hash();
        let (status, stored, computed) = container.check_data_hash();
        assert_eq!(status, Success::Success);
        assert_eq!(stored, REF_DATA_HASH);
        assert_eq!(computed, REF_DATA_HASH);

        container.buffer_mut().data_mut()[DpContainer::DATA_OFFSET] = 0x00;
        let (status, ..) = container.check_data_hash();
        assert_eq!(status, Success::Failure);
    }

    #[test]
    fn shrink_buffer_size_matches_the_packet_size() {
        let mut container = DpContainer::with_buffer(7, Buffer::allocate(128));
        container.set_data_size(4);
        assert_eq!(container.data_capacity(), 63);
        container.shrink_buffer_size();
        assert_eq!(container.buffer().size(), 69);
        assert_eq!(container.data_capacity(), 4);
    }

    #[test]
    fn data_serializer_writes_into_the_data_region() {
        let mut container = DpContainer::with_buffer(1, Buffer::allocate(80));
        {
            let mut ser = container.data_serializer();
            assert_eq!(ser.serialize_u32_be(0x0102_0304), SerializeStatus::Ok);
            assert_eq!(ser.get_size(), 4);
        }
        container.set_data_size(4);
        assert_eq!(container.data(), &[0x01, 0x02, 0x03, 0x04]);
    }

    #[test]
    fn take_buffer_returns_the_storage_and_clears_the_data_size() {
        let mut container = reference_container();
        assert_eq!(container.data_size(), 4);
        let buffer = container.take_buffer();
        assert_eq!(buffer.size(), 69);
        assert_eq!(container.data_size(), 0);
        assert!(!container.buffer().is_valid());
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn set_buffer_asserts_on_a_buffer_smaller_than_the_minimum_packet() {
        let mut container = DpContainer::new();
        container.set_buffer(Buffer::allocate(64));
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn serialize_header_asserts_when_the_data_size_overflows_the_store_type() {
        // FwSizeStoreType is U16: serializeSize range-checks and the
        // generated C++ code FW_ASSERTs the failure.
        let mut container = DpContainer::with_buffer(1, Buffer::allocate(128));
        container.set_data_size(70_000);
        container.serialize_header();
    }
}
