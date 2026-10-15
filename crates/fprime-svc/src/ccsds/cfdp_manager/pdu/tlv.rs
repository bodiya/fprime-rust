//! CFDP TLVs (type-length-value) — port of
//! `Svc/Ccsds/CfdpManager/Types/Tlv.{hpp,cpp}` (CCSDS 727.0-B-5 section
//! 5.4, table 5-3).
//!
//! On the wire a TLV is `[type u8][length u8][value]`. An entity-ID TLV
//! always encodes its value as a big-endian `EntityId` (4 bytes) with
//! length 4, and decodes up to 4 value bytes into an id — so a decoded
//! short EID re-encodes at full width (C++ parity).

use super::PduReader;
use crate::ccsds::cfdp_manager::types::EntityId;
use fprime_config::cfdp::MAX_TLV;
use fprime_fw::{SerBuf, SerBufAny, SerializeStatus, fw_assert, fw_try};

/// C++ `TlvType`. A newtype rather than a Rust enum: the type byte comes
/// off the wire and the C++ `enum class` holds any value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct TlvType(pub u8);

impl TlvType {
    /// Filestore request.
    pub const FILESTORE_REQUEST: Self = Self(0);
    /// Filestore response.
    pub const FILESTORE_RESPONSE: Self = Self(1);
    /// Message to user.
    pub const MESSAGE_TO_USER: Self = Self(2);
    /// Fault handler override.
    pub const FAULT_HANDLER_OVERRIDE: Self = Self(4);
    /// Flow label.
    pub const FLOW_LABEL: Self = Self(5);
    /// Entity ID.
    pub const ENTITY_ID: Self = Self(6);
}

const EID_SIZE: usize = size_of::<EntityId>();

/// C++ `TlvData`: the value storage. Upstream overlays the entity id and
/// the raw bytes in a union; here the id lives in the first bytes of the
/// raw storage in host byte order, so reading one view after writing the
/// other gives what the C++ union gives on the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TlvData {
    raw: [u8; 256],
    length: u8,
}

impl Default for TlvData {
    fn default() -> Self {
        Self {
            raw: [0; 256],
            length: 0,
        }
    }
}

impl TlvData {
    /// A zeroed value.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Store an entity id (length becomes `sizeof(EntityId)`).
    pub fn set_entity_id(&mut self, eid: EntityId) {
        self.raw[..EID_SIZE].copy_from_slice(&eid.to_ne_bytes());
        self.length = EID_SIZE as u8;
    }

    /// Store raw bytes (at most 255).
    pub fn set_data(&mut self, data: &[u8]) {
        fw_assert!(data.len() <= usize::from(u8::MAX), data.len() as i32);
        self.raw[..data.len()].copy_from_slice(data);
        self.length = data.len() as u8;
    }

    /// The entity id view.
    #[must_use]
    pub fn get_entity_id(&self) -> EntityId {
        let mut bytes = [0u8; EID_SIZE];
        bytes.copy_from_slice(&self.raw[..EID_SIZE]);
        EntityId::from_ne_bytes(bytes)
    }

    /// The raw value bytes (`get_length()` of them are meaningful).
    #[must_use]
    pub fn get_data(&self) -> &[u8; 256] {
        &self.raw
    }

    /// The value length.
    #[must_use]
    pub fn get_length(&self) -> u8 {
        self.length
    }
}

/// C++ `Tlv`: one TLV.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tlv {
    tlv_type: TlvType,
    data: TlvData,
}

impl Default for Tlv {
    /// C++ default: an empty entity-ID TLV.
    fn default() -> Self {
        Self {
            tlv_type: TlvType::ENTITY_ID,
            data: TlvData::new(),
        }
    }
}

impl Tlv {
    /// An empty entity-ID TLV.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Initialize as an entity-ID TLV.
    pub fn initialize_entity_id(&mut self, eid: EntityId) {
        self.tlv_type = TlvType::ENTITY_ID;
        self.data.set_entity_id(eid);
    }

    /// Initialize with a type and raw value bytes.
    pub fn initialize(&mut self, tlv_type: TlvType, data: &[u8]) {
        self.tlv_type = tlv_type;
        self.data.set_data(data);
    }

    /// The type.
    #[must_use]
    pub fn get_type(&self) -> TlvType {
        self.tlv_type
    }

    /// The value.
    #[must_use]
    pub fn get_data(&self) -> &TlvData {
        &self.data
    }

    /// Encoded size: type + length + value.
    #[must_use]
    pub fn get_encoded_size(&self) -> u32 {
        2 + u32::from(self.data.get_length())
    }

    /// C++ `toSerialBuffer`.
    pub fn to_serial_buffer(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        fw_try!(buf.serialize_u8_be(self.tlv_type.0));
        let length = self.data.get_length();
        fw_try!(buf.serialize_u8_be(length));
        if self.tlv_type == TlvType::ENTITY_ID {
            buf.serialize_u32_be(self.data.get_entity_id())
        } else {
            for byte in &self.data.raw[..usize::from(length)] {
                fw_try!(buf.serialize_u8_be(*byte));
            }
            SerializeStatus::Ok
        }
    }

    /// C++ `fromSerialBuffer`. An entity-ID value longer than
    /// `sizeof(EntityId)` is a `DeserFormatError`.
    pub fn from_serial_buffer(&mut self, reader: &mut PduReader<'_>) -> SerializeStatus {
        let mut type_val = 0u8;
        fw_try!(reader.read_u8(&mut type_val));
        self.tlv_type = TlvType(type_val);
        let mut length = 0u8;
        fw_try!(reader.read_u8(&mut length));
        if self.tlv_type == TlvType::ENTITY_ID {
            if usize::from(length) > EID_SIZE {
                return SerializeStatus::DeserFormatError;
            }
            let mut raw = [0u8; EID_SIZE];
            for byte in raw.iter_mut().take(usize::from(length)) {
                fw_try!(reader.read_u8(byte));
            }
            let mut eid: EntityId = 0;
            for byte in raw.iter().take(usize::from(length)) {
                eid = (eid << 8) | EntityId::from(*byte);
            }
            self.data.set_entity_id(eid);
        } else {
            let mut raw = [0u8; 256];
            for byte in raw.iter_mut().take(usize::from(length)) {
                fw_try!(reader.read_u8(byte));
            }
            self.data.set_data(&raw[..usize::from(length)]);
        }
        SerializeStatus::Ok
    }
}

/// C++ `TlvList`: up to `MaxTlv` TLVs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TlvList {
    num_tlv: u8,
    tlvs: [Tlv; MAX_TLV],
}

impl Default for TlvList {
    fn default() -> Self {
        Self {
            num_tlv: 0,
            tlvs: [Tlv::default(); MAX_TLV],
        }
    }
}

impl TlvList {
    /// An empty list.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a TLV; false when the list is full.
    pub fn append_tlv(&mut self, tlv: &Tlv) -> bool {
        if usize::from(self.num_tlv) >= MAX_TLV {
            return false;
        }
        self.tlvs[usize::from(self.num_tlv)] = *tlv;
        self.num_tlv += 1;
        true
    }

    /// Empty the list.
    pub fn clear(&mut self) {
        self.num_tlv = 0;
    }

    /// Number of TLVs.
    #[must_use]
    pub fn get_num_tlv(&self) -> u8 {
        self.num_tlv
    }

    /// The TLV at `index` (asserts `index < get_num_tlv()`).
    #[must_use]
    pub fn get_tlv(&self, index: u8) -> &Tlv {
        fw_assert!(
            index < self.num_tlv,
            i32::from(index),
            i32::from(self.num_tlv)
        );
        &self.tlvs[usize::from(index)]
    }

    /// Total encoded size.
    #[must_use]
    pub fn get_encoded_size(&self) -> u32 {
        self.tlvs[..usize::from(self.num_tlv)]
            .iter()
            .map(Tlv::get_encoded_size)
            .sum()
    }

    /// Encode every TLV.
    pub fn to_serial_buffer(&self, buf: &mut dyn SerBufAny) -> SerializeStatus {
        for tlv in &self.tlvs[..usize::from(self.num_tlv)] {
            fw_try!(tlv.to_serial_buffer(buf));
        }
        SerializeStatus::Ok
    }

    /// C++ `fromSerialBuffer`: decode TLVs until the buffer is exhausted or
    /// the list is full. A failure on the first TLV is returned; a failure
    /// on a later one ends the list with what was decoded (C++ parity).
    pub fn from_serial_buffer(&mut self, reader: &mut PduReader<'_>) -> SerializeStatus {
        self.num_tlv = 0;
        while reader.remaining() > 0 && usize::from(self.num_tlv) < MAX_TLV {
            let status = self.tlvs[usize::from(self.num_tlv)].from_serial_buffer(reader);
            if status != SerializeStatus::Ok {
                if self.num_tlv == 0 {
                    return status;
                }
                break;
            }
            self.num_tlv += 1;
        }
        SerializeStatus::Ok
    }
}
