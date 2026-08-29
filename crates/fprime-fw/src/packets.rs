//! GDS packet types.
//!
//! Port of `Fw/Cmd/CmdPacket.cpp`, `Fw/Log/LogPacket.cpp`,
//! `Fw/Tlm/TlmPacket.cpp` (see `docs/cpp-analysis/fw-services.md`). Every
//! packet begins with a `FwPacketDescriptorType` (u16) descriptor whose
//! values are the [`ComPacketType`] discriminants. The GDS depends
//! byte-for-byte on these layouts.

use crate::com::ComPacketType;
use crate::serial::{
    CmdArgBuffer, ComBuffer, Deserialize, Endianness, LengthMode, SerBuf, SerBufAny, Serialize,
    SerializeStatus,
};
use crate::time::Time;
use crate::{fw_assert, fw_try};
use fprime_config::{
    FwChanIdType, FwEventIdType, FwOpcodeType, FwPacketDescriptorType, FwSizeType,
};

// LogBuffer/TlmBuffer are used via the serial aliases
use crate::serial::{LogBuffer, TlmBuffer};

/// Command packet (uplink): `[descriptor u16 = 0][opcode u32][raw args]`.
///
/// Deserialize-only in flight software — the C++ `serializeTo` is
/// `FW_ASSERT(false)`, so no `Serialize` impl exists here.
#[derive(Debug, Default)]
pub struct CmdPacket {
    opcode: FwOpcodeType,
    arg_buffer: CmdArgBuffer,
}

impl CmdPacket {
    /// A new packet with opcode 0 and an empty argument buffer.
    pub fn new() -> Self {
        Self::default()
    }

    /// The deserialized opcode.
    pub fn get_opcode(&self) -> FwOpcodeType {
        self.opcode
    }

    /// The deserialized argument bytes.
    pub fn get_arg_buffer(&self) -> &CmdArgBuffer {
        &self.arg_buffer
    }

    /// The deserialized argument bytes, mutable (handlers deserialize the
    /// FPP arguments out of this).
    pub fn get_arg_buffer_mut(&mut self) -> &mut CmdArgBuffer {
        &mut self.arg_buffer
    }
}

impl Deserialize for CmdPacket {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        // descriptor: raw u16 compare, no enum validation (C++ deserializeBase)
        let mut descriptor: FwPacketDescriptorType = 0;
        fw_try!(buf.deserialize_u16(&mut descriptor, e));
        if descriptor != ComPacketType::FwPacketCommand as FwPacketDescriptorType {
            return SerializeStatus::DeserTypeMismatch;
        }
        fw_try!(buf.deserialize_u32(&mut self.opcode, e));
        let left = buf.deserialize_size_left();
        if left > 0 {
            // copy the serialized arguments (replaces the arg buffer content)
            buf.copy_raw(&mut self.arg_buffer, left)
        } else {
            // C++ parity: copyRaw only replaces content when bytes exist, so a
            // zero-arg command must explicitly clear stale args from a
            // previously parsed packet.
            self.arg_buffer.reset_ser();
            SerializeStatus::Ok
        }
    }
}

/// Event/log packet (downlink):
/// `[descriptor u16 = 2][id u32][time 11B][raw args, OMIT_LENGTH]`.
/// The severity is NOT on the wire — the GDS resolves it by id.
#[derive(Debug, Default)]
pub struct LogPacket {
    id: FwEventIdType,
    time_tag: Time,
    log_buffer: LogBuffer,
}

impl LogPacket {
    /// A new packet with id 0, zero time, empty args.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the event id.
    pub fn set_id(&mut self, id: FwEventIdType) {
        self.id = id;
    }

    /// Set the time tag.
    pub fn set_time_tag(&mut self, time_tag: Time) {
        self.time_tag = time_tag;
    }

    /// Set the argument buffer (copied, as the C++ copy-assign does).
    pub fn set_log_buffer(&mut self, buffer: &LogBuffer) {
        self.log_buffer = buffer.clone();
    }

    /// The event id.
    pub fn get_id(&self) -> FwEventIdType {
        self.id
    }

    /// The time tag.
    pub fn get_time_tag(&self) -> &Time {
        &self.time_tag
    }

    /// The argument buffer.
    pub fn get_log_buffer(&self) -> &LogBuffer {
        &self.log_buffer
    }

    /// The argument buffer, mutable.
    pub fn get_log_buffer_mut(&mut self) -> &mut LogBuffer {
        &mut self.log_buffer
    }
}

impl Serialize for LogPacket {
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        fw_try!(buf.serialize_u16(ComPacketType::FwPacketLog as FwPacketDescriptorType, e));
        fw_try!(buf.serialize_u32(self.id, e));
        fw_try!(self.time_tag.serialize_to(buf, e));
        // data without a length prefix for the ground software (C++ parity)
        buf.serialize_bytes(self.log_buffer.as_slice(), LengthMode::OmitLength, e)
    }
    fn serialized_size(&self) -> usize {
        size_of::<FwPacketDescriptorType>()
            + size_of::<FwEventIdType>()
            + Time::SERIALIZED_SIZE
            + self.log_buffer.get_size()
    }
}

impl Deserialize for LogPacket {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        let mut descriptor: FwPacketDescriptorType = 0;
        fw_try!(buf.deserialize_u16(&mut descriptor, e));
        if descriptor != ComPacketType::FwPacketLog as FwPacketDescriptorType {
            return SerializeStatus::DeserTypeMismatch;
        }
        fw_try!(buf.deserialize_u32(&mut self.id, e));
        fw_try!(self.time_tag.deserialize_from(buf, e));
        // the remainder of the buffer is the argument bytes
        let size = buf.deserialize_size_left();
        if size > self.log_buffer.capacity() {
            return SerializeStatus::DeserSizeMismatch;
        }
        let deser = buf.deser_loc();
        let status = self.log_buffer.set_buff(&buf.bytes()[deser..deser + size]);
        fw_assert!(status == SerializeStatus::Ok, status as i32);
        buf.set_deser_loc(deser + size);
        SerializeStatus::Ok
    }
}

/// Telemetry packet accumulator (downlink, the TlmChan path):
/// `[descriptor u16 = 1]` then N repetitions of
/// `[id u32][time 11B][raw value bytes]` — no entry count, no per-entry
/// length (the GDS dictionary supplies value sizes).
///
/// All accumulator operations use the big-endian wire default, matching the
/// defaulted C++ calls.
#[derive(Debug, Default)]
pub struct TlmPacket {
    tlm_buffer: ComBuffer,
    num_entries: FwSizeType,
}

impl TlmPacket {
    /// A new, un-reset packet; call [`TlmPacket::reset_pkt_ser`] before
    /// adding values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset the internal buffer and serialize the `FW_PACKET_TELEM`
    /// descriptor into it; zeroes the entry count.
    pub fn reset_pkt_ser(&mut self) -> SerializeStatus {
        self.tlm_buffer.reset_ser();
        self.num_entries = 0;
        self.tlm_buffer
            .serialize_u16_be(ComPacketType::FwPacketTelem as FwPacketDescriptorType)
    }

    /// Reset the read cursor and verify the descriptor is `FW_PACKET_TELEM`
    /// (else `DeserTypeMismatch`), for extraction.
    pub fn reset_pkt_deser(&mut self) -> SerializeStatus {
        self.tlm_buffer.reset_deser();
        let mut descriptor: FwPacketDescriptorType = 0;
        fw_try!(self.tlm_buffer.deserialize_u16_be(&mut descriptor));
        if descriptor != ComPacketType::FwPacketTelem as FwPacketDescriptorType {
            return SerializeStatus::DeserTypeMismatch;
        }
        SerializeStatus::Ok
    }

    /// Number of channel values added since the last reset.
    pub fn get_num_entries(&self) -> FwSizeType {
        self.num_entries
    }

    /// The accumulated wire bytes (what TlmChan downlinks via `getBuffer`).
    pub fn get_buffer(&self) -> &ComBuffer {
        &self.tlm_buffer
    }

    /// The accumulated wire bytes, mutable.
    pub fn get_buffer_mut(&mut self) -> &mut ComBuffer {
        &mut self.tlm_buffer
    }

    /// Replace the internal buffer (for the extraction path).
    pub fn set_buffer(&mut self, buffer: ComBuffer) {
        self.tlm_buffer = buffer;
    }

    /// Append one channel sample: `[id u32][time 11B][value bytes raw]`.
    /// Room for the whole entry is pre-checked; a full packet returns
    /// [`SerializeStatus::NoRoomLeft`] without partial writes.
    pub fn add_value(
        &mut self,
        id: FwChanIdType,
        time_tag: &Time,
        buffer: &TlmBuffer,
    ) -> SerializeStatus {
        let left = self.tlm_buffer.capacity() - self.tlm_buffer.get_size();
        if size_of::<FwChanIdType>() + Time::SERIALIZED_SIZE + buffer.get_size() > left {
            return SerializeStatus::NoRoomLeft;
        }
        fw_try!(self.tlm_buffer.serialize_u32_be(id));
        fw_try!(time_tag.serialize_to(&mut self.tlm_buffer, Endianness::Big));
        fw_try!(self.tlm_buffer.serialize_bytes(
            buffer.as_slice(),
            LengthMode::OmitLength,
            Endianness::Big
        ));
        self.num_entries += 1;
        SerializeStatus::Ok
    }

    /// Extract the next channel sample. `buffer_size` is the value's byte
    /// count, which the caller must know from the dictionary (there is no
    /// per-entry length on the wire). Returns
    /// [`SerializeStatus::DeserBufferEmpty`] at the end of the packet.
    pub fn extract_value(
        &mut self,
        id: &mut FwChanIdType,
        time_tag: &mut Time,
        buffer: &mut TlmBuffer,
        buffer_size: usize,
    ) -> SerializeStatus {
        fw_try!(self.tlm_buffer.deserialize_u32_be(id));
        fw_try!(time_tag.deserialize_from(&mut self.tlm_buffer, Endianness::Big));
        let mut len = buffer_size;
        let mut scratch = [0u8; fprime_config::FW_TLM_BUFFER_MAX_SIZE];
        fw_assert!(buffer_size <= scratch.len(), buffer_size as i32);
        fw_try!(self.tlm_buffer.deserialize_bytes(
            &mut scratch,
            &mut len,
            LengthMode::OmitLength,
            Endianness::Big
        ));
        buffer.set_buff(&scratch[..buffer_size])
    }
}

impl Serialize for TlmPacket {
    /// NOTE (C++ parity): this is the NON-GDS format — `[numEntries as
    /// FwSizeType (u64)][internal buffer bytes incl. descriptor]`. The
    /// downlink path is [`TlmPacket::get_buffer`].
    fn serialize_to(&self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        fw_try!(buf.serialize_u64(self.num_entries, e));
        buf.serialize_bytes(self.tlm_buffer.as_slice(), LengthMode::OmitLength, e)
    }
    fn serialized_size(&self) -> usize {
        size_of::<FwSizeType>() + self.tlm_buffer.get_size()
    }
}

impl Deserialize for TlmPacket {
    fn deserialize_from(&mut self, buf: &mut dyn SerBufAny, e: Endianness) -> SerializeStatus {
        fw_try!(buf.deserialize_u64(&mut self.num_entries, e));
        let size = buf.deserialize_size_left();
        if size > self.tlm_buffer.capacity() {
            return SerializeStatus::DeserSizeMismatch;
        }
        let deser = buf.deser_loc();
        let status = self.tlm_buffer.set_buff(&buf.bytes()[deser..deser + size]);
        fw_assert!(status == SerializeStatus::Ok, status as i32);
        buf.set_deser_loc(deser + size);
        SerializeStatus::Ok
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serial::LinearBuffer;
    use crate::time::TimeBase;

    fn test_time() -> Time {
        Time::new(TimeBase::TbWorkstationTime, 0, 0x0000_0001, 0x0000_0002)
    }

    const TEST_TIME_BYTES: [u8; 11] = [
        0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02,
    ];

    // ------------------------------------------------------------------ Cmd

    #[test]
    fn cmd_packet_deserializes_descriptor_opcode_args() {
        let mut buf = ComBuffer::new();
        let bytes = [
            0x00, 0x00, // descriptor = FW_PACKET_COMMAND
            0x00, 0x00, 0x00, 0x05, // opcode 5
            0xAA, 0xBB, // args
        ];
        assert_eq!(buf.set_buff(&bytes), SerializeStatus::Ok);
        let mut pkt = CmdPacket::new();
        assert_eq!(
            pkt.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(pkt.get_opcode(), 5);
        assert_eq!(pkt.get_arg_buffer().as_slice(), &[0xAA, 0xBB]);
    }

    #[test]
    fn cmd_packet_zero_args_clears_stale_arg_buffer() {
        let mut pkt = CmdPacket::new();
        // first: a command with args
        let mut buf = ComBuffer::new();
        assert_eq!(
            buf.set_buff(&[0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xDE, 0xAD]),
            SerializeStatus::Ok
        );
        assert_eq!(
            pkt.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(pkt.get_arg_buffer().get_size(), 2);
        // then: a zero-arg command must not inherit the previous args
        let mut buf = ComBuffer::new();
        assert_eq!(
            buf.set_buff(&[0x00, 0x00, 0x00, 0x00, 0x00, 0x02]),
            SerializeStatus::Ok
        );
        assert_eq!(
            pkt.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(pkt.get_opcode(), 2);
        assert_eq!(
            pkt.get_arg_buffer().get_size(),
            0,
            "stale args must be cleared"
        );
    }

    #[test]
    fn cmd_packet_rejects_wrong_descriptor() {
        let mut buf = ComBuffer::new();
        assert_eq!(
            buf.set_buff(&[0x00, 0x01, 0x00, 0x00, 0x00, 0x05]),
            SerializeStatus::Ok
        );
        let mut pkt = CmdPacket::new();
        assert_eq!(
            pkt.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserTypeMismatch
        );
    }

    // ------------------------------------------------------------------ Log

    #[test]
    fn log_packet_wire_format() {
        let mut pkt = LogPacket::new();
        pkt.set_id(0x0000_0042);
        pkt.set_time_tag(test_time());
        let mut args = LogBuffer::new();
        assert_eq!(args.serialize_u16_be(0xCAFE), SerializeStatus::Ok);
        pkt.set_log_buffer(&args);

        let mut buf = ComBuffer::new();
        assert_eq!(
            pkt.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );

        let mut expected = vec![0x00, 0x02, 0x00, 0x00, 0x00, 0x42];
        expected.extend_from_slice(&TEST_TIME_BYTES);
        expected.extend_from_slice(&[0xCA, 0xFE]); // raw args, NO length prefix
        assert_eq!(buf.as_slice(), expected.as_slice());
        assert_eq!(pkt.serialized_size(), expected.len());
    }

    #[test]
    fn log_packet_roundtrip() {
        let mut pkt = LogPacket::new();
        pkt.set_id(7);
        pkt.set_time_tag(test_time());
        let mut args = LogBuffer::new();
        assert_eq!(args.serialize_u32_be(0xDEAD_BEEF), SerializeStatus::Ok);
        pkt.set_log_buffer(&args);

        let mut buf = ComBuffer::new();
        assert_eq!(
            pkt.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );

        let mut out = LogPacket::new();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(out.get_id(), 7);
        assert_eq!(out.get_time_tag(), &test_time());
        assert_eq!(out.get_log_buffer().as_slice(), &[0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn log_packet_empty_args_roundtrip() {
        let mut pkt = LogPacket::new();
        pkt.set_id(1);
        pkt.set_time_tag(test_time());
        let mut buf = ComBuffer::new();
        assert_eq!(
            pkt.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.get_size(), 2 + 4 + 11);
        let mut out = LogPacket::new();
        // pre-fill stale args to prove they are replaced
        assert_eq!(
            out.get_log_buffer_mut().serialize_u8_be(9),
            SerializeStatus::Ok
        );
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(out.get_log_buffer().get_size(), 0);
    }

    #[test]
    fn log_packet_rejects_wrong_descriptor() {
        let mut buf = ComBuffer::new();
        assert_eq!(buf.serialize_u16_be(0x0001), SerializeStatus::Ok);
        let mut out = LogPacket::new();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserTypeMismatch
        );
    }

    #[test]
    fn log_packet_rejects_oversized_remainder() {
        // remainder larger than the LogBuffer capacity (506): use a big ExtBuf
        let mut raw = vec![0u8; 600];
        raw[1] = 0x02; // descriptor FW_PACKET_LOG
        // id = 0, then a valid time
        raw[6..17].copy_from_slice(&TEST_TIME_BYTES);
        let mut src = crate::serial::ExtBuf::with_len(&mut raw, 600); // 583 arg bytes remain
        let mut out = LogPacket::new();
        assert_eq!(
            out.deserialize_from(&mut src, Endianness::Big),
            SerializeStatus::DeserSizeMismatch
        );
    }

    // ------------------------------------------------------------------ Tlm

    #[test]
    fn tlm_packet_accumulates_entries_with_descriptor() {
        let mut pkt = TlmPacket::new();
        assert_eq!(pkt.reset_pkt_ser(), SerializeStatus::Ok);
        assert_eq!(pkt.get_buffer().as_slice(), &[0x00, 0x01]);
        assert_eq!(pkt.get_num_entries(), 0);

        let mut value = TlmBuffer::new();
        assert_eq!(value.serialize_u16_be(0x1234), SerializeStatus::Ok);
        assert_eq!(
            pkt.add_value(0x0000_0099, &test_time(), &value),
            SerializeStatus::Ok
        );
        assert_eq!(pkt.get_num_entries(), 1);

        let mut expected = vec![0x00, 0x01, 0x00, 0x00, 0x00, 0x99];
        expected.extend_from_slice(&TEST_TIME_BYTES);
        expected.extend_from_slice(&[0x12, 0x34]); // raw value, no length
        assert_eq!(pkt.get_buffer().as_slice(), expected.as_slice());
    }

    #[test]
    fn tlm_packet_add_value_no_room_left_when_full() {
        let mut pkt = TlmPacket::new();
        assert_eq!(pkt.reset_pkt_ser(), SerializeStatus::Ok);
        // entry size = 4 + 11 + 2 = 17; capacity after descriptor = 510
        let mut value = TlmBuffer::new();
        assert_eq!(value.serialize_u16_be(0), SerializeStatus::Ok);
        let time = test_time();
        let mut added = 0usize;
        loop {
            match pkt.add_value(1, &time, &value) {
                SerializeStatus::Ok => added += 1,
                SerializeStatus::NoRoomLeft => break,
                other => panic!("unexpected status {other:?}"),
            }
            assert!(added < 100, "must fill up eventually");
        }
        assert_eq!(added, 30); // floor(510 / 17)
        assert_eq!(pkt.get_num_entries() as usize, added);
        // NoRoomLeft must not have partially written an entry
        assert_eq!(pkt.get_buffer().get_size(), 2 + added * 17);
    }

    #[test]
    fn tlm_packet_extract_roundtrip_until_buffer_empty() {
        let mut pkt = TlmPacket::new();
        assert_eq!(pkt.reset_pkt_ser(), SerializeStatus::Ok);
        let mut v1 = TlmBuffer::new();
        assert_eq!(v1.serialize_u32_be(0xAABB_CCDD), SerializeStatus::Ok);
        let mut v2 = TlmBuffer::new();
        assert_eq!(v2.serialize_u8_be(0x7E), SerializeStatus::Ok);
        assert_eq!(pkt.add_value(10, &test_time(), &v1), SerializeStatus::Ok);
        assert_eq!(pkt.add_value(20, &test_time(), &v2), SerializeStatus::Ok);

        // extraction path: hand the buffer to a fresh packet
        let mut rx = TlmPacket::new();
        rx.set_buffer(pkt.get_buffer().clone());
        assert_eq!(rx.reset_pkt_deser(), SerializeStatus::Ok);

        let mut id = 0;
        let mut time = Time::default();
        let mut value = TlmBuffer::new();
        assert_eq!(
            rx.extract_value(&mut id, &mut time, &mut value, 4),
            SerializeStatus::Ok
        );
        assert_eq!(id, 10);
        assert_eq!(time, test_time());
        assert_eq!(value.as_slice(), &[0xAA, 0xBB, 0xCC, 0xDD]);

        assert_eq!(
            rx.extract_value(&mut id, &mut time, &mut value, 1),
            SerializeStatus::Ok
        );
        assert_eq!(id, 20);
        assert_eq!(value.as_slice(), &[0x7E]);

        // loop termination contract: BUFFER_EMPTY exactly at the end
        assert_eq!(
            rx.extract_value(&mut id, &mut time, &mut value, 1),
            SerializeStatus::DeserBufferEmpty
        );
    }

    #[test]
    fn tlm_packet_reset_deser_rejects_wrong_descriptor() {
        let mut pkt = TlmPacket::new();
        let mut buf = ComBuffer::new();
        assert_eq!(buf.serialize_u16_be(0x0002), SerializeStatus::Ok);
        pkt.set_buffer(buf);
        assert_eq!(pkt.reset_pkt_deser(), SerializeStatus::DeserTypeMismatch);
    }

    #[test]
    fn tlm_packet_value_serialization_is_non_gds_format() {
        // C++ parity: serializeTo writes numEntries as FwSizeType (u64) then
        // the raw internal buffer (including its descriptor)
        let mut pkt = TlmPacket::new();
        assert_eq!(pkt.reset_pkt_ser(), SerializeStatus::Ok);
        let mut value = TlmBuffer::new();
        assert_eq!(value.serialize_u8_be(0x55), SerializeStatus::Ok);
        assert_eq!(pkt.add_value(1, &test_time(), &value), SerializeStatus::Ok);

        let mut out = LinearBuffer::<600>::new();
        assert_eq!(
            pkt.serialize_to(&mut out, Endianness::Big),
            SerializeStatus::Ok
        );
        let bytes = out.as_slice();
        assert_eq!(&bytes[..8], &[0, 0, 0, 0, 0, 0, 0, 1], "u64 numEntries");
        assert_eq!(&bytes[8..10], &[0x00, 0x01], "internal buffer descriptor");
        assert_eq!(bytes.len(), 8 + pkt.get_buffer().get_size());

        let mut rx = TlmPacket::new();
        assert_eq!(
            rx.deserialize_from(&mut out, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(rx.get_num_entries(), 1);
        assert_eq!(rx.get_buffer(), pkt.get_buffer());
    }
}
