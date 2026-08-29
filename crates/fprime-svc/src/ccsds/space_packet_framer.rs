//! # SpacePacketFramer — port of `Svc::Ccsds::SpacePacketFramer` (passive)
//!
//! C++ sources: `Svc/Ccsds/SpacePacketFramer/SpacePacketFramer.{cpp,fpp}`.
//! Analysis: `docs/cpp-analysis/ccsds.md` ("SpacePacketFramer" + the
//! "CCSDS Space Packet Primary Header" wire format).
//!
//! Prepends a 6-byte Space Packet primary header to every payload:
//! `[packetIdentification u16][packetSequenceControl u16][packetDataLength
//! u16][payload]`. The APID, the secondary-header flag and the sequence
//! flags come from the incoming [`FrameContext`]; the 14-bit sequence count
//! is allocated by
//! [`ApidManager`](crate::ccsds::apid_manager::ApidManager) over the
//! `getApidSeqCount` port. PVN and the packet-type bit are hardcoded to 0
//! (C++ parity: this component can only emit telemetry packets — there is no
//! field in `FrameContext` for either).
//!
//! The framed packet lives in a NEWLY ALLOCATED buffer; the incoming payload
//! buffer is handed straight back over `dataReturnOut`. When the frame
//! buffer comes back on `dataReturnIn` it is released to `bufferDeallocate`
//! (contrast [`TmFramer`](crate::ccsds::tm_framer::TmFramer), which owns a
//! static frame buffer and never deallocates).
//!
//! All input ports are `sync` — handlers run on the caller's thread with no
//! component mutex, exactly as the FPP `Framer` interface declares.

use crate::ccsds::types::{ApidSequenceCountPort, SpacePacketHeader, space_packet_subfields};
use fprime_comp::{
    BufferGetPort, BufferSendPort, ComDataWithContextPort, EventGlue, EventThrottle, OutputPort,
    PassiveBase, PortRef, SuccessConditionPort, input_port_adapter,
};
use fprime_config::{FwEventIdType, FwIndexType, FwSizeType};
use fprime_fw::{
    Buffer, Endianness, FrameContext, LengthMode, LogSeverity, SerBuf, SerializeStatus, Success,
    fw_assert,
};
use std::sync::Arc;

/// `NoBufferAvailable` event id (FPP declaration order, relative).
pub const EVENTID_NO_BUFFER_AVAILABLE: FwEventIdType = 0;
/// FPP `throttle 5` on `NoBufferAvailable`.
pub const EVENTID_NO_BUFFER_AVAILABLE_THROTTLE: u32 = 5;

/// `Svc::Ccsds::SpacePacketFramer` — passive Space Packet framer.
pub struct SpacePacketFramer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// `bufferAllocate` — allocates the packet buffer (`Fw.BufferGet`).
    pub buffer_allocate: OutputPort<dyn BufferGetPort>,
    /// `bufferDeallocate` — returns packet buffers to the allocator.
    pub buffer_deallocate: OutputPort<dyn BufferSendPort>,
    /// `getApidSeqCount` — allocates the sequence count for an APID.
    pub get_apid_seq_count: OutputPort<dyn ApidSequenceCountPort>,
    /// `dataOut` — the framed Space Packet downstream.
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — returns the incoming payload buffer upstream.
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `comStatusOut` — forwards com status upstream (to ComQueue).
    pub com_status_out: OutputPort<dyn SuccessConditionPort>,
    /// Throttle for `NoBufferAvailable` (FPP `throttle 5`).
    no_buffer_available_throttle: EventThrottle,
}

impl SpacePacketFramer {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            buffer_allocate: OutputPort::new(),
            buffer_deallocate: OutputPort::new(),
            get_apid_seq_count: OutputPort::new(),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            com_status_out: OutputPort::new(),
            no_buffer_available_throttle: EventThrottle::new(EVENTID_NO_BUFFER_AVAILABLE_THROTTLE),
        })
    }

    /// Clear the `NoBufferAvailable` throttle (generated `..._ThrottleClear`).
    pub fn clear_no_buffer_available_throttle(&self) {
        self.no_buffer_available_throttle.clear();
    }

    /// `comStatusIn` — SYNC `Fw.SuccessCondition` input: forwarded upstream.
    pub fn com_status_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn SuccessConditionPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `dataReturnIn` — SYNC input: the framed packet coming back from the
    /// com adapter; released to `bufferDeallocate`.
    pub fn data_return_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(
            Arc::new(DataReturnInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    // -- Handlers ------------------------------------------------------------

    /// `dataIn` handler: allocate, write the primary header and the payload,
    /// emit the packet, return the payload buffer to its sender.
    fn data_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        let frame_size = SpacePacketHeader::SERIALIZED_SIZE + data.size();
        // C++ parity: the frame size must fit Fw::Buffer::SizeType (U32)...
        fw_assert!(
            data.size() as u64 <= u32::MAX as u64 - SpacePacketHeader::SERIALIZED_SIZE as u64,
            data.size() as i32
        );
        // ... and the standard requires at least one octet of packet data.
        fw_assert!(data.size() > 0, data.size() as i32);

        // Allocate the packet buffer.
        let p = self.buffer_allocate.get();
        let mut frame_buffer = p.target.invoke(p.port_num, frame_size as FwSizeType);
        // The allocator may return an invalid or smaller-than-requested
        // buffer: drop the packet (with a throttled event), deallocating a
        // valid-but-small buffer, and return the original payload.
        if !frame_buffer.is_valid() || frame_buffer.size() < frame_size {
            if self.no_buffer_available_throttle.ok_to_emit() {
                self.evt.log_event(
                    self.base.get_id_base(),
                    EVENTID_NO_BUFFER_AVAILABLE,
                    LogSeverity::WarningHi,
                    "Failed to allocate a packet buffer: packet dropped",
                    |_buf| SerializeStatus::Ok,
                );
            }
            if frame_buffer.is_valid() {
                let p = self.buffer_deallocate.get();
                p.target.invoke(p.port_num, frame_buffer);
            }
            let p = self.data_return_out.get();
            p.target.invoke(p.port_num, data, context);
            return;
        }

        // ---------------- Primary header ----------------
        let apid = context.apid;
        // The APID must fit in 11 bits (C++ FW_ASSERT).
        fw_assert!(
            (apid.as_repr() >> space_packet_subfields::APID_WIDTH) == 0,
            apid.as_repr() as i32
        );
        // PVN is always 0 per the standard; packet type is 0 for telemetry.
        let packet_identification = SpacePacketHeader::build_packet_identification(
            0,
            0,
            context.has_sec_hdr,
            apid.as_repr(),
        );
        // Allocate this APID's sequence count (the second argument is unused
        // on the request direction of the shared port type).
        let p = self.get_apid_seq_count.get();
        let sequence_count = p.target.invoke(p.port_num, apid, 0);
        let packet_sequence_control = SpacePacketHeader::build_packet_sequence_control(
            context.sequence_flags,
            sequence_count,
        );
        fw_assert!(data.size() <= u16::MAX as usize, data.size() as i32);
        // The standard's length token is the octet count minus one.
        let packet_data_length = SpacePacketHeader::length_token(data.size() as u16);
        let header = SpacePacketHeader::new(
            packet_identification,
            packet_sequence_control,
            packet_data_length,
        );

        // ---------------- Serialize the packet ----------------
        {
            let mut ser = frame_buffer.get_serializer();
            let status = ser.serialize(&header, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize_bytes(data.data(), LengthMode::OmitLength, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
        }
        // Trim to the actual frame size (the allocator may have handed out a
        // larger buffer).
        frame_buffer.set_size(frame_size);

        // Packet out with the context UNMODIFIED, then the payload buffer
        // back to its sender.
        let p = self.data_out.get();
        p.target.invoke(p.port_num, frame_buffer, context);
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    /// `dataReturnIn` handler: the framed packet is ours — release it.
    fn data_return_in_handler(&self, _port_num: FwIndexType, frame_buffer: Buffer) {
        let p = self.buffer_deallocate.get();
        p.target.invoke(p.port_num, frame_buffer);
    }
}

/// `comStatusIn` handler — forward when `comStatusOut` is connected
/// (C++ parity: the only connection-checked out-port in this component).
impl SuccessConditionPort for SpacePacketFramer {
    fn invoke(&self, _port_num: FwIndexType, condition: &mut Success) {
        if let Some(p) = self.com_status_out.try_get() {
            p.target.invoke(p.port_num, condition);
        }
    }
}

input_port_adapter! {
    /// `dataIn` — SYNC `Svc.ComDataWithContext` input: the payload to frame.
    component: SpacePacketFramer;
    adapter: DataInAdapter;
    port: ComDataWithContextPort;
    input: pub data_in;
    handler: data_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

/// Adapter for the sync `dataReturnIn` port. Hand-written because the
/// handler drops the unused `context` argument (the codegen macro forwards
/// 1:1 only).
struct DataReturnInAdapter {
    comp: Arc<SpacePacketFramer>,
}

impl ComDataWithContextPort for DataReturnInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, _context: &FrameContext) {
        self.comp.data_return_in_handler(port_num, data);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ccsds::apid_manager::ApidManager;
    use fprime_comp::LogPort;
    use fprime_fw::{Apid, LogBuffer, Time};
    use std::sync::Mutex;

    /// Records everything the framer can emit.
    #[derive(Default)]
    struct Recorder {
        /// dataOut records: (bytes, context).
        data_out: Mutex<Vec<(Vec<u8>, FrameContext)>>,
        /// dataReturnOut records.
        data_return: Mutex<Vec<(Vec<u8>, FrameContext)>>,
        /// bufferDeallocate records.
        deallocated: Mutex<Vec<Vec<u8>>>,
        /// comStatusOut records.
        statuses: Mutex<Vec<Success>>,
        /// events (id, severity).
        events: Mutex<Vec<(FwEventIdType, LogSeverity)>>,
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

    impl BufferSendPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.deallocated
                .lock()
                .unwrap()
                .push(buffer.data().to_vec());
        }
    }

    impl SuccessConditionPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, condition: &mut Success) {
            self.statuses.lock().unwrap().push(*condition);
        }
    }

    impl LogPort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            _args: &mut LogBuffer,
        ) {
            self.events.lock().unwrap().push((id, severity));
        }
    }

    /// Allocator stub: `sizes` empty means "exactly what was asked for",
    /// a size of 0 means an invalid buffer.
    struct Allocator {
        sizes: Mutex<Vec<usize>>,
        requests: Mutex<Vec<FwSizeType>>,
    }

    impl Allocator {
        fn exact() -> Arc<Self> {
            Arc::new(Self {
                sizes: Mutex::new(vec![]),
                requests: Mutex::new(vec![]),
            })
        }
        fn fixed(size: usize) -> Arc<Self> {
            Arc::new(Self {
                sizes: Mutex::new(vec![size]),
                requests: Mutex::new(vec![]),
            })
        }
    }

    impl BufferGetPort for Allocator {
        fn invoke(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
            self.requests.lock().unwrap().push(size);
            let n = match self.sizes.lock().unwrap().last() {
                Some(&n) => n,
                None => size as usize,
            };
            if n == 0 {
                Buffer::empty()
            } else {
                Buffer::allocate(n)
            }
        }
    }

    /// Sequence-count stub returning a scripted value and recording calls.
    struct SeqCounter {
        next: Mutex<u16>,
        calls: Mutex<Vec<(Apid, u16)>>,
    }

    impl SeqCounter {
        fn new(start: u16) -> Arc<Self> {
            Arc::new(Self {
                next: Mutex::new(start),
                calls: Mutex::new(Vec::new()),
            })
        }
    }

    impl ApidSequenceCountPort for SeqCounter {
        fn invoke(&self, _port_num: FwIndexType, apid: Apid, sequence_count: u16) -> u16 {
            self.calls.lock().unwrap().push((apid, sequence_count));
            let mut next = self.next.lock().unwrap();
            let value = *next;
            *next = value.wrapping_add(1);
            value
        }
    }

    fn build(
        alloc: &Arc<Allocator>,
        seq: &Arc<SeqCounter>,
    ) -> (Arc<SpacePacketFramer>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let framer = SpacePacketFramer::new("spacePacketFramer");
        framer.buffer_allocate.connect(alloc.clone(), 0);
        framer.buffer_deallocate.connect(rec.clone(), 0);
        framer.get_apid_seq_count.connect(seq.clone(), 0);
        framer.data_out.connect(rec.clone(), 0);
        framer.data_return_out.connect(rec.clone(), 1);
        framer.evt.log_out.connect(rec.clone(), 0);
        (framer, rec)
    }

    fn payload(bytes: &[u8]) -> Buffer {
        let mut b = Buffer::allocate(bytes.len().max(1));
        b.data_mut()[..bytes.len()].copy_from_slice(bytes);
        b.set_size(bytes.len());
        b
    }

    fn feed(framer: &Arc<SpacePacketFramer>, bytes: &[u8], context: &FrameContext) {
        let p = framer.data_in(0);
        p.target.invoke(p.port_num, payload(bytes), context);
    }

    /// Literal wire bytes: APID 0x0002 (LOG), no secondary header,
    /// unsegmented (0b11), sequence count 3, 4-byte payload.
    #[test]
    fn packet_bytes_are_byte_exact() {
        let alloc = Allocator::exact();
        let seq = SeqCounter::new(3);
        let (framer, rec) = build(&alloc, &seq);
        let ctx = FrameContext {
            apid: Apid::FwPacketLog,
            sequence_flags: 0b11,
            ..FrameContext::default()
        };
        feed(&framer, &[0xDE, 0xAD, 0xBE, 0xEF], &ctx);

        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(
            out[0].0,
            vec![0x00, 0x02, 0xC0, 0x03, 0x00, 0x03, 0xDE, 0xAD, 0xBE, 0xEF]
        );
        // The context is forwarded UNMODIFIED (the framer never writes back
        // the allocated sequence count).
        assert_eq!(out[0].1, ctx);
        assert_eq!(*alloc.requests.lock().unwrap(), vec![10]);
        // The sequence count came from the ApidManager port, called with the
        // context APID and an unused 0.
        assert_eq!(*seq.calls.lock().unwrap(), vec![(Apid::FwPacketLog, 0)]);
        // Payload buffer returned with the same context.
        let ret = rec.data_return.lock().unwrap();
        assert_eq!(ret.len(), 1);
        assert_eq!(ret[0].0, vec![0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(ret[0].1, ctx);
    }

    /// The secondary-header flag lands in bit 11 and the sequence flags in
    /// bits `[15:14]`.
    #[test]
    fn sec_hdr_flag_and_sequence_flags_are_packed() {
        let alloc = Allocator::exact();
        let seq = SeqCounter::new(0x3FFF);
        let (framer, rec) = build(&alloc, &seq);
        let ctx = FrameContext {
            apid: Apid::SppIdlePacket,
            has_sec_hdr: true,
            sequence_flags: 0b01,
            ..FrameContext::default()
        };
        feed(&framer, &[0x01], &ctx);
        let out = rec.data_out.lock().unwrap();
        // 0x0800 (sec hdr) | 0x07FF (apid) = 0x0FFF; 0b01 << 14 | 0x3FFF.
        assert_eq!(out[0].0, vec![0x0F, 0xFF, 0x7F, 0xFF, 0x00, 0x00, 0x01]);
    }

    /// The length token is octets minus one for a single-byte payload.
    #[test]
    fn single_byte_payload_has_zero_length_token() {
        let alloc = Allocator::exact();
        let seq = SeqCounter::new(0);
        let (framer, rec) = build(&alloc, &seq);
        feed(&framer, &[0xAA], &FrameContext::default());
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out[0].0.len(), 7);
        assert_eq!(&out[0].0[4..6], &[0x00, 0x00]);
    }

    /// A real ApidManager drives the counts: per-APID, monotonic, and the
    /// packets carry them in order.
    #[test]
    fn sequence_counts_come_from_the_apid_manager() {
        let alloc = Allocator::exact();
        let rec = Arc::new(Recorder::default());
        let framer = SpacePacketFramer::new("spacePacketFramer");
        let mgr = ApidManager::new("apidManager");
        framer.buffer_allocate.connect(alloc.clone(), 0);
        framer.buffer_deallocate.connect(rec.clone(), 0);
        framer
            .get_apid_seq_count
            .connect_to(mgr.get_apid_seq_count_in(0));
        framer.data_out.connect(rec.clone(), 0);
        framer.data_return_out.connect(rec.clone(), 1);

        let telem = FrameContext {
            apid: Apid::FwPacketTelem,
            ..FrameContext::default()
        };
        let log = FrameContext {
            apid: Apid::FwPacketLog,
            ..FrameContext::default()
        };
        feed(&framer, &[0x00], &telem);
        feed(&framer, &[0x00], &log);
        feed(&framer, &[0x00], &telem);

        let out = rec.data_out.lock().unwrap();
        let counts: Vec<u16> = out
            .iter()
            .map(|(bytes, _)| u16::from_be_bytes([bytes[2], bytes[3]]) & 0x3FFF)
            .collect();
        assert_eq!(counts, vec![0, 0, 1]);
    }

    /// An invalid allocation drops the packet: throttled event, payload
    /// returned, nothing deallocated.
    #[test]
    fn invalid_allocation_drops_packet() {
        let alloc = Allocator::fixed(0);
        let seq = SeqCounter::new(0);
        let (framer, rec) = build(&alloc, &seq);
        let ctx = FrameContext {
            apid: Apid::FwPacketTelem,
            ..FrameContext::default()
        };
        feed(&framer, &[0x11, 0x22], &ctx);
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert!(rec.deallocated.lock().unwrap().is_empty());
        let ret = rec.data_return.lock().unwrap();
        assert_eq!(ret.len(), 1);
        assert_eq!(ret[0].0, vec![0x11, 0x22]);
        assert_eq!(ret[0].1, ctx);
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(EVENTID_NO_BUFFER_AVAILABLE, LogSeverity::WarningHi)]
        );
        // No sequence count is consumed on the drop path.
        assert!(seq.calls.lock().unwrap().is_empty());
    }

    /// A valid-but-too-small allocation is deallocated and the packet
    /// dropped.
    #[test]
    fn small_allocation_is_deallocated_and_packet_dropped() {
        let alloc = Allocator::fixed(4);
        let seq = SeqCounter::new(0);
        let (framer, rec) = build(&alloc, &seq);
        feed(&framer, &[0x11, 0x22], &FrameContext::default());
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert_eq!(rec.deallocated.lock().unwrap().len(), 1);
        assert_eq!(rec.deallocated.lock().unwrap()[0].len(), 4);
        assert_eq!(rec.data_return.lock().unwrap().len(), 1);
        assert_eq!(rec.events.lock().unwrap().len(), 1);
    }

    /// An oversized allocation is trimmed to the packet size.
    #[test]
    fn oversized_allocation_is_trimmed() {
        let alloc = Allocator::fixed(64);
        let seq = SeqCounter::new(0);
        let (framer, rec) = build(&alloc, &seq);
        feed(&framer, &[0x01, 0x02, 0x03], &FrameContext::default());
        assert_eq!(rec.data_out.lock().unwrap()[0].0.len(), 9);
    }

    /// `NoBufferAvailable` throttles at 5 and the clear helper re-enables it.
    #[test]
    fn no_buffer_available_event_throttles_at_five() {
        let alloc = Allocator::fixed(0);
        let seq = SeqCounter::new(0);
        let (framer, rec) = build(&alloc, &seq);
        for _ in 0..8 {
            feed(&framer, &[0x01], &FrameContext::default());
        }
        assert_eq!(rec.events.lock().unwrap().len(), 5);
        framer.clear_no_buffer_available_throttle();
        feed(&framer, &[0x01], &FrameContext::default());
        assert_eq!(rec.events.lock().unwrap().len(), 6);
        // Every drop still returned the payload buffer.
        assert_eq!(rec.data_return.lock().unwrap().len(), 9);
    }

    /// comStatusIn forwards only when comStatusOut is connected.
    #[test]
    fn com_status_forwarding() {
        let alloc = Allocator::exact();
        let seq = SeqCounter::new(0);
        let (framer, rec) = build(&alloc, &seq);
        let p = framer.com_status_in(0);
        let mut cond = Success::Success;
        p.target.invoke(p.port_num, &mut cond);
        assert!(rec.statuses.lock().unwrap().is_empty());
        framer.com_status_out.connect(rec.clone(), 0);
        let mut cond = Success::Failure;
        p.target.invoke(p.port_num, &mut cond);
        assert_eq!(*rec.statuses.lock().unwrap(), vec![Success::Failure]);
    }

    /// dataReturnIn releases the framed packet to the allocator.
    #[test]
    fn data_return_in_deallocates_the_frame_buffer() {
        let alloc = Allocator::exact();
        let seq = SeqCounter::new(0);
        let (framer, rec) = build(&alloc, &seq);
        let p = framer.data_return_in(0);
        p.target
            .invoke(p.port_num, payload(b"packet"), &FrameContext::default());
        assert_eq!(rec.deallocated.lock().unwrap().len(), 1);
        assert_eq!(rec.deallocated.lock().unwrap()[0], b"packet");
    }

    /// The standard requires at least one octet of packet data (C++
    /// FW_ASSERT).
    #[test]
    #[should_panic(expected = "Assert:")]
    fn empty_payload_asserts() {
        let alloc = Allocator::exact();
        let seq = SeqCounter::new(0);
        let (framer, _rec) = build(&alloc, &seq);
        feed(&framer, &[], &FrameContext::default());
    }

    /// An APID wider than 11 bits asserts (C++ FW_ASSERT); the only such
    /// enum value is INVALID_UNINITIALIZED (0x0800).
    #[test]
    #[should_panic(expected = "Assert:")]
    fn apid_wider_than_eleven_bits_asserts() {
        let alloc = Allocator::exact();
        let seq = SeqCounter::new(0);
        let (framer, _rec) = build(&alloc, &seq);
        let ctx = FrameContext {
            apid: Apid::InvalidUninitialized,
            ..FrameContext::default()
        };
        feed(&framer, &[0x01], &ctx);
    }
}
