//! # SpacePacketDeframer — port of `Svc::Ccsds::SpacePacketDeframer`
//! (passive, guarded `dataIn`)
//!
//! C++ sources: `Svc/Ccsds/SpacePacketDeframer/SpacePacketDeframer.{cpp,fpp}`.
//! Analysis: `docs/cpp-analysis/ccsds.md` ("SpacePacketDeframer" + gotchas).
//!
//! Validates one Space Packet per `dataIn` invocation and strips its 6-byte
//! primary header, extracting the APID, the secondary-header flag, the
//! sequence flags and the sequence count into a COPY of the incoming
//! [`FrameContext`]. The payload is emitted as the SAME allocation advanced
//! past the header and shrunk to the header's data-field length, so
//! downstream must return the buffer for the original allocator.
//!
//! Validation order (C++ parity — every drop returns the ORIGINAL,
//! unmodified context):
//!
//! 1. `size <= 6` → `InvalidPacket` (a header-only 6-byte packet is dropped
//!    even though a length token of 0 would describe a 7-byte packet).
//! 2. header deserialization failure → `InvalidPacket`.
//! 3. PVN != 0 (`Pvn::SPACE_PACKET_PROTOCOL`) → `InvalidPacket`. The packet
//!    TYPE bit is deliberately not checked.
//! 4. `packetDataLength + 1 > size - 6` → `InvalidLength`.
//!
//! `errorNotify` is invoked only when connected; `validateApidSeqCount` is
//! invoked unconditionally (its return value is discarded — the validation
//! exists purely for the `UnexpectedSequenceCount` event and the onboard
//! resync, and the context carries the RECEIVED count, not the expected
//! one).

use crate::ccsds::types::{
    ApidSequenceCountPort, ErrorNotifyPort, FrameError, SpacePacketHeader, space_packet_subfields,
};
use fprime_comp::{ComDataWithContextPort, EventGlue, OutputPort, PassiveBase, input_port_adapter};
use fprime_config::{FwEventIdType, FwIndexType, FwSignedSizeType, FwSizeType};
use fprime_fw::{
    Apid, Buffer, Endianness, FrameContext, LogSeverity, Pvn, SerBuf, SerializeStatus, fw_assert,
    fw_try,
};
use std::sync::{Arc, Mutex};

/// `InvalidPacket` event id (FPP declaration order, relative).
pub const EVENTID_INVALID_PACKET: FwEventIdType = 0;
/// `InvalidLength(transmitted, actual)` event id.
pub const EVENTID_INVALID_LENGTH: FwEventIdType = 1;

/// `Svc::Ccsds::SpacePacketDeframer` — passive Space Packet deframer.
pub struct SpacePacketDeframer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// `dataOut` — the deframed packet data field downstream (to the router).
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — buffer ownership back upstream (to the TC deframer).
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `validateApidSeqCount` — checks the received count against the
    /// onboard one (return value discarded).
    pub validate_apid_seq_count: OutputPort<dyn ApidSequenceCountPort>,
    /// `errorNotify` — optional deframing-error notification.
    pub error_notify: OutputPort<dyn ErrorNotifyPort>,
    /// The guarded-port mutex (C++ `guarded input port dataIn`).
    guard: Mutex<()>,
}

impl SpacePacketDeframer {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            validate_apid_seq_count: OutputPort::new(),
            error_notify: OutputPort::new(),
            guard: Mutex::new(()),
        })
    }

    /// Emit `errorNotify` when the port is connected (C++
    /// `isConnected_errorNotify_OutputPort` check).
    fn error_notify(&self, error: FrameError) {
        if let Some(p) = self.error_notify.try_get() {
            p.target.invoke(p.port_num, error);
        }
    }

    /// Drop path shared by every rejection: return the buffer upstream with
    /// the ORIGINAL context.
    fn data_return_out(&self, data: Buffer, context: &FrameContext) {
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    /// `InvalidPacket` (WARNING_HI), `errorNotify(SP_INVALID_PACKET)` and
    /// the drop — the three C++ rejection sites share this body verbatim.
    fn reject_invalid_packet(&self, data: Buffer, context: &FrameContext) {
        self.evt.log_event(
            self.base.get_id_base(),
            EVENTID_INVALID_PACKET,
            LogSeverity::WarningHi,
            "Malformed packet received refusing to deframe",
            |_buf| SerializeStatus::Ok,
        );
        self.error_notify(FrameError::SpInvalidPacket);
        self.data_return_out(data, context);
    }

    /// `dataIn` handler (guarded — runs under the component mutex).
    fn data_in_handler(&self, _port_num: FwIndexType, mut data: Buffer, context: &FrameContext) {
        let _guard = self.guard.lock().unwrap();

        // (1) Strictly greater than the header size is required.
        if data.size() <= SpacePacketHeader::SERIALIZED_SIZE {
            self.reject_invalid_packet(data, context);
            return;
        }

        // (2) Deserialize the primary header. Unreachable after the size
        // check, but the C++ keeps the branch and so do we.
        let mut header = SpacePacketHeader::default();
        let header_ok = {
            let mut deser = data.get_deserializer();
            deser.deserialize(&mut header, Endianness::Big).is_ok()
        };
        if !header_ok {
            self.reject_invalid_packet(data, context);
            return;
        }

        // (3) The Packet Version Number must be 0 (Space Packet Protocol).
        // The packet TYPE bit is deliberately NOT checked (C++ parity).
        if u16::from(header.pvn()) != u16::from(Pvn::SpacePacketProtocol.as_repr()) {
            self.reject_invalid_packet(data, context);
            return;
        }

        // (4) The data field must fit in what was actually received. The
        // length token is widened to u32 before the +1 so 0xFFFF does not
        // wrap to zero.
        let pkt_length = header.data_field_length();
        let max_data_available = data.size() - SpacePacketHeader::SERIALIZED_SIZE;
        if pkt_length as u64 > max_data_available as u64 {
            let transmitted = pkt_length as FwSizeType;
            let actual = max_data_available as FwSizeType;
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_LENGTH,
                LogSeverity::WarningHi,
                &format!(
                    "Invalid length received. Header specified packet byte size of \
                     {transmitted} | Actual received data length: {actual}"
                ),
                |buf| {
                    fw_try!(buf.serialize_u64_be(transmitted));
                    buf.serialize_u64_be(actual)
                },
            );
            self.error_notify(FrameError::SpInvalidLength);
            self.data_return_out(data, context);
            return;
        }

        // (5) Fill the context copy from the header.
        let apid_value = header.apid_value();
        // An 11-bit APID that is not a declared constant becomes
        // INVALID_UNINITIALIZED (0x0800) for the router's default arm. Note
        // SPP_IDLE_PACKET (0x7FF) IS declared, so uplinked idle packets are
        // forwarded rather than dropped.
        let apid = Apid::try_from(apid_value).unwrap_or(Apid::InvalidUninitialized);
        let mut context_copy = *context;
        context_copy.apid = apid;
        context_copy.has_sec_hdr = header.has_sec_hdr();
        context_copy.sequence_flags = header.sequence_flags();
        let received_sequence_count = header.sequence_count();
        // Return value deliberately discarded: the call exists for the
        // event/resync side effect inside the ApidManager.
        let p = self.validate_apid_seq_count.get();
        let _ = p.target.invoke(p.port_num, apid, received_sequence_count);
        // The RECEIVED count is stored, never the expected one.
        context_copy.sequence_count = received_sequence_count;

        // (6) Window the same allocation onto the packet data field.
        data.advance(SpacePacketHeader::SERIALIZED_SIZE as FwSignedSizeType);
        // pkt_length <= max_data_available <= u32::MAX, so this fits usize.
        fw_assert!(pkt_length as u64 <= usize::MAX as u64, pkt_length as i32);
        data.set_size(pkt_length as usize);
        let p = self.data_out.get();
        p.target.invoke(p.port_num, data, &context_copy);
    }

    /// `dataReturnIn` handler: straight pass-through upstream.
    fn data_return_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.data_return_out(data, context);
    }
}

/// The APID width is 11 bits, so a masked value can never exceed it.
const _: () = assert!(space_packet_subfields::APID_MASK == 0x07FF);

// -- Input-port adapters + factories (generated by the codegen layer) --------

input_port_adapter! {
    /// `dataIn` — GUARDED `Svc.ComDataWithContext` input: one Space Packet.
    component: SpacePacketDeframer;
    adapter: DataInAdapter;
    port: ComDataWithContextPort;
    input: pub data_in;
    handler: data_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

input_port_adapter! {
    /// `dataReturnIn` — SYNC input: the packet data field coming back from
    /// downstream; passed through to `dataReturnOut`.
    component: SpacePacketDeframer;
    adapter: DataReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub data_return_in;
    handler: data_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ccsds::apid_manager::ApidManager;
    use fprime_comp::LogPort;
    use fprime_fw::{Deserialize, LogBuffer, Time};
    use std::sync::Mutex as StdMutex;

    #[derive(Default)]
    struct Recorder {
        /// dataOut records: (bytes, context).
        data_out: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        /// dataReturnOut records.
        data_return: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        /// events: (id, severity, decoded u64 args).
        events: StdMutex<Vec<(FwEventIdType, LogSeverity, Vec<u64>)>>,
        /// errorNotify records.
        errors: StdMutex<Vec<FrameError>>,
        /// validateApidSeqCount calls.
        validations: StdMutex<Vec<(Apid, u16)>>,
    }

    impl ComDataWithContextPort for Recorder {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            let rec = (data.data().to_vec(), *context);
            if port_num == 0 {
                self.data_out.lock().unwrap().push(rec);
            } else {
                self.data_return.lock().unwrap().push(rec);
            }
        }
    }

    impl LogPort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            let mut decoded = Vec::new();
            loop {
                let mut value = 0u64;
                if value.deserialize_from(args, Endianness::Big).is_ok() {
                    decoded.push(value);
                } else {
                    break;
                }
            }
            self.events.lock().unwrap().push((id, severity, decoded));
        }
    }

    impl ErrorNotifyPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, error_code: FrameError) {
            self.errors.lock().unwrap().push(error_code);
        }
    }

    impl ApidSequenceCountPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, apid: Apid, sequence_count: u16) -> u16 {
            self.validations
                .lock()
                .unwrap()
                .push((apid, sequence_count));
            // A deliberately different value: the deframer must discard it.
            0xBEEF
        }
    }

    fn build() -> (Arc<SpacePacketDeframer>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let deframer = SpacePacketDeframer::new("spacePacketDeframer");
        deframer.data_out.connect(rec.clone(), 0);
        deframer.data_return_out.connect(rec.clone(), 1);
        deframer.validate_apid_seq_count.connect(rec.clone(), 0);
        deframer.error_notify.connect(rec.clone(), 0);
        deframer.evt.log_out.connect(rec.clone(), 0);
        (deframer, rec)
    }

    fn buffer_of(bytes: &[u8]) -> Buffer {
        let mut b = Buffer::allocate(bytes.len().max(1));
        b.data_mut()[..bytes.len()].copy_from_slice(bytes);
        b.set_size(bytes.len());
        b
    }

    fn feed(deframer: &Arc<SpacePacketDeframer>, bytes: &[u8], context: &FrameContext) {
        let p = deframer.data_in(0);
        p.target.invoke(p.port_num, buffer_of(bytes), context);
    }

    /// Build a Space Packet from raw header words plus a payload.
    fn packet(ident: u16, seq_control: u16, length_token: u16, payload: &[u8]) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&ident.to_be_bytes());
        bytes.extend_from_slice(&seq_control.to_be_bytes());
        bytes.extend_from_slice(&length_token.to_be_bytes());
        bytes.extend_from_slice(payload);
        bytes
    }

    /// Happy path: header stripped, context filled from the header, payload
    /// emitted as the same allocation.
    #[test]
    fn valid_packet_emits_payload_and_context() {
        let (deframer, rec) = build();
        // APID 0x0000 (COMMAND), sec hdr set, seq flags 0b10, count 0x1234.
        let bytes = packet(0x0800, 0x9234, 3, &[0xAA, 0xBB, 0xCC, 0xDD]);
        feed(&deframer, &bytes, &FrameContext::default());

        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        // The length token (3) means 4 data octets: the trailing byte is
        // included, nothing more.
        assert_eq!(out[0].0, vec![0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(out[0].1.apid, Apid::FwPacketCommand);
        assert!(out[0].1.has_sec_hdr);
        assert_eq!(out[0].1.sequence_flags, 0b10);
        assert_eq!(out[0].1.sequence_count, 0x1234);
        // The received count went to the ApidManager, and its return value
        // (0xBEEF) was discarded.
        assert_eq!(
            *rec.validations.lock().unwrap(),
            vec![(Apid::FwPacketCommand, 0x1234)]
        );
        assert!(rec.events.lock().unwrap().is_empty());
        assert!(rec.errors.lock().unwrap().is_empty());
        assert!(rec.data_return.lock().unwrap().is_empty());
    }

    /// A short length token windows the buffer down: trailing bytes are cut.
    #[test]
    fn trailing_bytes_beyond_the_length_token_are_trimmed() {
        let (deframer, rec) = build();
        let bytes = packet(0x0001, 0xC000, 0, &[0x11, 0x22, 0x33]);
        feed(&deframer, &bytes, &FrameContext::default());
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out[0].0, vec![0x11]);
        assert_eq!(out[0].1.apid, Apid::FwPacketTelem);
    }

    /// The other context members survive untouched (the deframer never
    /// writes vcId, pvn, saIndex or comQueueIndex).
    #[test]
    fn untouched_context_members_are_preserved() {
        let (deframer, rec) = build();
        let ctx = FrameContext {
            com_queue_index: 7,
            vc_id: 5,
            pvn: Pvn::EncapsulationPacketProtocol,
            sa_index: 0x1234,
            send_now: true,
            ..FrameContext::default()
        };
        feed(&deframer, &packet(0x0002, 0x0000, 0, &[0x01]), &ctx);
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out[0].1.com_queue_index, 7);
        assert_eq!(out[0].1.vc_id, 5);
        assert_eq!(out[0].1.pvn, Pvn::EncapsulationPacketProtocol);
        assert_eq!(out[0].1.sa_index, 0x1234);
        assert!(out[0].1.send_now);
    }

    /// Gotcha: a 6-byte header-only packet is rejected (strictly greater
    /// than the header size is required).
    #[test]
    fn header_only_packet_is_rejected() {
        let (deframer, rec) = build();
        let ctx = FrameContext {
            apid: Apid::FwPacketHand,
            ..FrameContext::default()
        };
        feed(&deframer, &packet(0x0000, 0x0000, 0, &[]), &ctx);
        assert!(rec.data_out.lock().unwrap().is_empty());
        let ret = rec.data_return.lock().unwrap();
        assert_eq!(ret.len(), 1);
        assert_eq!(ret[0].1, ctx); // original context
        assert_eq!(rec.events.lock().unwrap()[0].0, EVENTID_INVALID_PACKET);
        assert_eq!(rec.events.lock().unwrap()[0].1, LogSeverity::WarningHi);
        assert_eq!(
            *rec.errors.lock().unwrap(),
            vec![FrameError::SpInvalidPacket]
        );
        // No sequence-count validation happens on a drop.
        assert!(rec.validations.lock().unwrap().is_empty());
    }

    /// An empty buffer is rejected the same way.
    #[test]
    fn tiny_buffer_is_rejected() {
        let (deframer, rec) = build();
        feed(&deframer, &[0x00], &FrameContext::default());
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert_eq!(rec.data_return.lock().unwrap().len(), 1);
        assert_eq!(
            *rec.errors.lock().unwrap(),
            vec![FrameError::SpInvalidPacket]
        );
    }

    /// A non-zero Packet Version Number is rejected.
    #[test]
    fn non_zero_pvn_is_rejected() {
        let (deframer, rec) = build();
        // PVN = 0b111 in bits [15:13].
        feed(
            &deframer,
            &packet(0xE001, 0xC000, 0, &[0x42]),
            &FrameContext::default(),
        );
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert_eq!(rec.events.lock().unwrap()[0].0, EVENTID_INVALID_PACKET);
        assert_eq!(
            *rec.errors.lock().unwrap(),
            vec![FrameError::SpInvalidPacket]
        );
    }

    /// The packet TYPE bit is deliberately not validated (C++ parity): a
    /// telecommand packet is deframed like any other.
    #[test]
    fn packet_type_bit_is_not_checked() {
        let (deframer, rec) = build();
        feed(
            &deframer,
            &packet(0x1003, 0xC000, 0, &[0x42]),
            &FrameContext::default(),
        );
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].1.apid, Apid::FwPacketFile);
    }

    /// A length token longer than the received data is `InvalidLength`
    /// carrying (transmitted, actual) as FwSizeType args.
    #[test]
    fn over_long_length_token_is_rejected() {
        let (deframer, rec) = build();
        let ctx = FrameContext::default();
        // Token 9 => 10 octets claimed, only 2 present.
        feed(&deframer, &packet(0x0001, 0xC000, 9, &[0x01, 0x02]), &ctx);
        assert!(rec.data_out.lock().unwrap().is_empty());
        let events = rec.events.lock().unwrap();
        assert_eq!(events[0].0, EVENTID_INVALID_LENGTH);
        assert_eq!(events[0].1, LogSeverity::WarningHi);
        assert_eq!(events[0].2, vec![10, 2]);
        assert_eq!(
            *rec.errors.lock().unwrap(),
            vec![FrameError::SpInvalidLength]
        );
        assert_eq!(rec.data_return.lock().unwrap()[0].1, ctx);
    }

    /// The maximum length token widens instead of wrapping to zero (the C++
    /// undefined-behavior guard).
    #[test]
    fn maximum_length_token_does_not_wrap() {
        let (deframer, rec) = build();
        feed(
            &deframer,
            &packet(0x0001, 0xC000, 0xFFFF, &[0x01, 0x02, 0x03]),
            &FrameContext::default(),
        );
        let events = rec.events.lock().unwrap();
        assert_eq!(events[0].0, EVENTID_INVALID_LENGTH);
        assert_eq!(events[0].2, vec![65536, 3]);
    }

    /// Gotcha: an 11-bit APID that is not a declared constant becomes
    /// INVALID_UNINITIALIZED, while the idle APID 0x7FF IS declared and is
    /// forwarded.
    #[test]
    fn undeclared_apid_becomes_invalid_uninitialized() {
        let (deframer, rec) = build();
        feed(
            &deframer,
            &packet(0x0123, 0xC000, 0, &[0x01]),
            &FrameContext::default(),
        );
        assert_eq!(
            rec.data_out.lock().unwrap()[0].1.apid,
            Apid::InvalidUninitialized
        );
        assert_eq!(
            *rec.validations.lock().unwrap(),
            vec![(Apid::InvalidUninitialized, 0)]
        );
    }

    /// Uplinked idle packets are forwarded, not dropped.
    #[test]
    fn idle_apid_is_forwarded() {
        let (deframer, rec) = build();
        feed(
            &deframer,
            &packet(0x07FF, 0xC000, 0, &[0x44]),
            &FrameContext::default(),
        );
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out[0].1.apid, Apid::SppIdlePacket);
        assert_eq!(out[0].0, vec![0x44]);
    }

    /// `errorNotify` is optional: an unconnected port simply emits nothing
    /// (the event and the drop still happen).
    #[test]
    fn error_notify_is_optional() {
        let rec = Arc::new(Recorder::default());
        let deframer = SpacePacketDeframer::new("spacePacketDeframer");
        deframer.data_out.connect(rec.clone(), 0);
        deframer.data_return_out.connect(rec.clone(), 1);
        deframer.validate_apid_seq_count.connect(rec.clone(), 0);
        deframer.evt.log_out.connect(rec.clone(), 0);
        feed(&deframer, &[0x00, 0x01], &FrameContext::default());
        assert!(rec.errors.lock().unwrap().is_empty());
        assert_eq!(rec.events.lock().unwrap().len(), 1);
        assert_eq!(rec.data_return.lock().unwrap().len(), 1);
    }

    /// dataReturnIn passes straight through to dataReturnOut.
    #[test]
    fn data_return_in_passthrough() {
        let (deframer, rec) = build();
        let ctx = FrameContext {
            apid: Apid::FwPacketFile,
            ..FrameContext::default()
        };
        let p = deframer.data_return_in(0);
        p.target.invoke(p.port_num, buffer_of(b"pay"), &ctx);
        let ret = rec.data_return.lock().unwrap();
        assert_eq!(ret.len(), 1);
        assert_eq!(ret[0].0, b"pay");
        assert_eq!(ret[0].1, ctx);
    }

    /// With a real ApidManager, an out-of-order count is reported but the
    /// packet is still forwarded carrying the RECEIVED count.
    #[test]
    fn sequence_gap_is_reported_but_packet_is_forwarded() {
        let rec = Arc::new(Recorder::default());
        let deframer = SpacePacketDeframer::new("spacePacketDeframer");
        let mgr = ApidManager::new("apidManager");
        mgr.evt.log_out.connect(rec.clone(), 1);
        deframer.data_out.connect(rec.clone(), 0);
        deframer.data_return_out.connect(rec.clone(), 1);
        deframer
            .validate_apid_seq_count
            .connect_to(mgr.validate_apid_seq_count_in(0));
        deframer.evt.log_out.connect(rec.clone(), 0);

        // count 0 is expected first: silent.
        feed(
            &deframer,
            &packet(0x0000, 0xC000, 0, &[0x01]),
            &FrameContext::default(),
        );
        assert!(rec.events.lock().unwrap().is_empty());
        // Jump to 5: the ApidManager reports it.
        feed(
            &deframer,
            &packet(0x0000, 0xC005, 0, &[0x02]),
            &FrameContext::default(),
        );
        let events = rec.events.lock().unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].0, ApidManager::EVENTID_UNEXPECTED_SEQUENCE_COUNT);
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 2);
        assert_eq!(out[1].1.sequence_count, 5);
    }
}
