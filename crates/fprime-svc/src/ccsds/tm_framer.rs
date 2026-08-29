//! # TmFramer — port of `Svc::Ccsds::TmFramer` (passive)
//!
//! C++ sources: `Svc/Ccsds/TmFramer/TmFramer.{cpp,hpp,fpp}`.
//! Analysis: `docs/cpp-analysis/ccsds.md` ("TmFramer" + the "TM Transfer
//! Frame" and "SPP Idle Packet" wire formats).
//!
//! Wraps one payload in a FIXED-SIZE ([`TM_FRAME_FIXED_SIZE`], 1024 byte) TM
//! transfer frame:
//!
//! ```text
//! [ 6-byte primary header ][ payload ][ SPP idle packet ][ 2-byte FECF ]
//! 0                        6                            1022        1024
//! ```
//!
//! The data field is always filled to offset 1022 with exactly one Space
//! Packet idle packet (APID `0x7FF`, sequence flags `0b11`, `0x44` idle
//! data) as CCSDS TM 4.2.2.5 requires, and the Frame Error Control Field is
//! the CRC-16/CCITT-FALSE over frame bytes `[0, 1022)` — computed over the
//! whole fixed frame, which is correct only because the idle fill always
//! reaches 1022.
//!
//! Like the C++ component this framer owns ONE frame buffer and never
//! deallocates: `dataIn` moves it out (asserting it is currently owned) and
//! `dataReturnIn` moves it back. Downstream consumers must copy the frame
//! before returning it. The C++ `BufferOwnershipState` plus static array
//! becomes `Option<BufferStorage>` here — `Some` is `OWNED` — and the C++
//! "returned pointer lies within m_frameBuffer" assert becomes a capacity +
//! not-currently-owned check.
//!
//! The master and virtual channel frame counts are written into the header
//! and incremented AFTERWARDS, so the first frame carries 0/0; both wrap
//! mod 256.

use crate::ccsds::crc16::Crc16;
use crate::ccsds::types::{
    SPACECRAFT_ID, SpacePacketHeader, TM_FRAME_FIXED_SIZE, TMHeader, TMTrailer,
};
use fprime_comp::{
    ComDataWithContextPort, OutputPort, PassiveBase, PortRef, SuccessConditionPort,
    input_port_adapter,
};
use fprime_config::FwIndexType;
use fprime_fw::{
    Apid, Buffer, BufferStorage, Endianness, FrameContext, LengthMode, SerBuf, SerBufAny, Success,
    fw_assert,
};
use std::sync::{Arc, Mutex};

/// Idle-data fill pattern (`TmFramer::IDLE_DATA_PATTERN`).
pub const IDLE_DATA_PATTERN: u8 = 0x44;

/// Offset of the frame trailer: the CRC covers `[0, TRAILER_OFFSET)`.
pub const TRAILER_OFFSET: usize = TM_FRAME_FIXED_SIZE - TMTrailer::SERIALIZED_SIZE;

/// Largest payload the `dataIn` assert permits
/// (`TmFrameFixedSize - header - trailer` = 1016).
///
/// **Gotcha**: the idle fill then requires at least 7 bytes of its own, so
/// the real maximum payload is
/// [`MAX_PAYLOAD_SIZE`] = 1009 — two different asserts guard the same
/// constraint at different limits, exactly as in C++.
pub const TM_PAYLOAD_CAPACITY: usize =
    TM_FRAME_FIXED_SIZE - TMHeader::SERIALIZED_SIZE - TMTrailer::SERIALIZED_SIZE;

/// Smallest legal Space Packet idle packet: 6-byte header + 1 idle byte.
pub const MIN_IDLE_PACKET_SIZE: usize = SpacePacketHeader::SERIALIZED_SIZE + 1;

/// Largest payload that actually leaves room for the mandatory idle packet
/// (equal to `ComCfg::AggregationSize`, 1009).
pub const MAX_PAYLOAD_SIZE: usize = TM_PAYLOAD_CAPACITY - MIN_IDLE_PACKET_SIZE;

/// Segment Length Identifier `0b11`, required by CCSDS TM 4.1.2.7.5.
pub const SEGMENT_LENGTH_ID: u8 = 0x3;

/// Sequence flags `0b11` (unsegmented) used by the idle packet.
pub const IDLE_SEQUENCE_FLAGS: u8 = 0x3;

/// Framer state: the single frame buffer plus the two frame counters.
struct TmState {
    /// The frame storage. `Some` is the C++ `BufferOwnershipState::OWNED`;
    /// `None` means the frame is downstream awaiting `dataReturnIn`.
    frame_storage: Option<BufferStorage>,
    /// Master channel frame count (wraps mod 256).
    master_frame_count: u8,
    /// Virtual channel frame count (wraps mod 256).
    virtual_frame_count: u8,
}

/// `Svc::Ccsds::TmFramer` — passive fixed-size TM transfer frame framer.
pub struct TmFramer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `dataOut` — the completed TM transfer frame.
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — returns the incoming payload buffer upstream.
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `comStatusOut` — forwards com status upstream.
    pub com_status_out: OutputPort<dyn SuccessConditionPort>,
    /// The frame buffer and frame counters.
    state: Mutex<TmState>,
}

impl TmFramer {
    /// Construct the component with its frame buffer owned and both frame
    /// counts at zero.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            com_status_out: OutputPort::new(),
            state: Mutex::new(TmState {
                frame_storage: Some(vec![0u8; TM_FRAME_FIXED_SIZE].into_boxed_slice()),
                master_frame_count: 0,
                virtual_frame_count: 0,
            }),
        })
    }

    /// True while the frame buffer is held by this component (the C++
    /// `BufferOwnershipState::OWNED`).
    pub fn owns_frame_buffer(&self) -> bool {
        self.state.lock().unwrap().frame_storage.is_some()
    }

    /// `comStatusIn` — SYNC `Fw.SuccessCondition` input: forwarded upstream.
    pub fn com_status_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn SuccessConditionPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `dataReturnIn` — SYNC input: the frame buffer coming back from the
    /// com adapter. NOT deallocated — ownership simply returns here.
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

    /// `dataIn` handler: build the whole fixed-size frame and emit it.
    fn data_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        // C++ parity: the payload must fit header + data field + trailer.
        fw_assert!(data.size() <= TM_PAYLOAD_CAPACITY, data.size() as i32);

        let mut frame_buffer = {
            let mut state = self.state.lock().unwrap();
            // C++ parity: FW_ASSERT(m_bufferState == OWNED). Exactly one
            // frame may be in flight; ComQueue's one-outstanding-send flow
            // control is what guarantees it.
            let storage = state.frame_storage.take();
            fw_assert!(storage.is_some());
            let storage = match storage {
                Some(storage) => storage,
                // Unreachable: the assert above fails first.
                None => return,
            };

            // ---------------- Primary header ----------------
            let header = TMHeader::new(
                // Global Virtual Channel ID (CCSDS 4.1.2.2/4.1.2.3) with the
                // Operational Control Field flag clear (4.1.2.4).
                TMHeader::build_global_vc_id(SPACECRAFT_ID, context.vc_id, false),
                state.master_frame_count,
                state.virtual_frame_count,
                // Every flag clear except the segment length identifier; the
                // first header pointer is 0 because a single whole packet is
                // always wrapped at the data field's origin.
                TMHeader::build_data_field_status(false, false, false, SEGMENT_LENGTH_ID, 0),
            );
            // A single virtual channel is used, so both counts advance
            // together — AFTER being written into the header.
            state.master_frame_count = state.master_frame_count.wrapping_add(1);
            state.virtual_frame_count = state.virtual_frame_count.wrapping_add(1);

            let mut frame_buffer = Buffer::from_storage(storage, Buffer::NO_CONTEXT);
            {
                let mut ser = frame_buffer.get_serializer();
                let status = ser.serialize(&header, Endianness::Big);
                fw_assert!(status.is_ok(), status as i32);
                let status =
                    ser.serialize_bytes(data.data(), LengthMode::OmitLength, Endianness::Big);
                fw_assert!(status.is_ok(), status as i32);
                // CCSDS TM 4.2.2.5: pad the data field with an idle packet.
                Self::fill_with_idle_packet(&mut ser);
            }
            frame_buffer
        };

        // ---------------- Trailer (FECF) ----------------
        // The CRC covers the entire fixed frame minus the trailer, which is
        // correct only because the idle fill always reaches TRAILER_OFFSET.
        let crc = Crc16::compute(&frame_buffer.data()[..TRAILER_OFFSET]);
        {
            let mut ser = frame_buffer.get_serializer();
            let status = ser.move_ser_to_offset(TRAILER_OFFSET);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize(&TMTrailer::new(crc), Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
        }

        // Frame out with the context UNMODIFIED, then the payload buffer
        // back to its sender.
        let p = self.data_out.get();
        p.target.invoke(p.port_num, frame_buffer, context);
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    /// Fill the rest of the data field (up to [`TRAILER_OFFSET`]) with one
    /// Space Packet idle packet: a 6-byte header with APID `SPP_IDLE_PACKET`
    /// followed by [`IDLE_DATA_PATTERN`] bytes.
    fn fill_with_idle_packet(ser: &mut dyn SerBufAny) {
        let start_index = ser.get_size();
        let idle_packet_size = TRAILER_OFFSET - start_index;
        // C++ parity: at least 6 header bytes + 1 idle byte, and never more
        // than one frame.
        fw_assert!(
            idle_packet_size >= MIN_IDLE_PACKET_SIZE,
            idle_packet_size as i32
        );
        fw_assert!(
            idle_packet_size <= TM_FRAME_FIXED_SIZE,
            idle_packet_size as i32
        );
        // The length token is the data-field octet count minus one.
        let length_token = (idle_packet_size - SpacePacketHeader::SERIALIZED_SIZE - 1) as u16;

        let header = SpacePacketHeader::new(
            // PVN 0, packet type 0, no secondary header, APID 0x7FF.
            Apid::SppIdlePacket.as_repr(),
            SpacePacketHeader::build_packet_sequence_control(IDLE_SEQUENCE_FLAGS, 0),
            length_token,
        );
        let status = ser.serialize(&header, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
        for _ in (start_index + SpacePacketHeader::SERIALIZED_SIZE)..TRAILER_OFFSET {
            let status = ser.serialize_u8_be(IDLE_DATA_PATTERN);
            fw_assert!(status.is_ok(), status as i32);
        }
    }

    /// `dataReturnIn` handler: take the frame buffer back. No deallocation —
    /// the storage is reused for the next frame.
    fn data_return_in_handler(&self, _port_num: FwIndexType, frame_buffer: Buffer) {
        let mut state = self.state.lock().unwrap();
        // C++ parity: the returned buffer must be OUR frame buffer. The
        // pointer-range assert maps to "we are not currently holding it and
        // it has the frame capacity".
        fw_assert!(state.frame_storage.is_none());
        fw_assert!(
            frame_buffer.capacity() == TM_FRAME_FIXED_SIZE,
            frame_buffer.capacity() as i32
        );
        state.frame_storage = Some(frame_buffer.into_storage());
    }
}

/// `comStatusIn` handler — forward when `comStatusOut` is connected.
impl SuccessConditionPort for TmFramer {
    fn invoke(&self, _port_num: FwIndexType, condition: &mut Success) {
        if let Some(p) = self.com_status_out.try_get() {
            p.target.invoke(p.port_num, condition);
        }
    }
}

input_port_adapter! {
    /// `dataIn` — SYNC `Svc.ComDataWithContext` input: the payload to frame.
    component: TmFramer;
    adapter: DataInAdapter;
    port: ComDataWithContextPort;
    input: pub data_in;
    handler: data_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

/// Adapter for the sync `dataReturnIn` port. Hand-written because the
/// handler drops the unused `context` argument.
struct DataReturnInAdapter {
    comp: Arc<TmFramer>,
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
    use std::sync::Mutex as StdMutex;

    #[derive(Default)]
    struct Recorder {
        /// dataOut records: (frame bytes, context).
        data_out: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        /// dataReturnOut records.
        data_return: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        /// comStatusOut records.
        statuses: StdMutex<Vec<Success>>,
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

    impl SuccessConditionPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, condition: &mut Success) {
            self.statuses.lock().unwrap().push(*condition);
        }
    }

    fn build() -> (Arc<TmFramer>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let framer = TmFramer::new("tmFramer");
        framer.data_out.connect(rec.clone(), 0);
        framer.data_return_out.connect(rec.clone(), 1);
        (framer, rec)
    }

    fn payload(bytes: &[u8]) -> Buffer {
        let mut b = Buffer::allocate(bytes.len().max(1));
        b.data_mut()[..bytes.len()].copy_from_slice(bytes);
        b.set_size(bytes.len());
        b
    }

    fn feed(framer: &Arc<TmFramer>, bytes: &[u8], context: &FrameContext) {
        let p = framer.data_in(0);
        p.target.invoke(p.port_num, payload(bytes), context);
    }

    /// Return the frame to the framer (what the com adapter does).
    fn give_back(framer: &Arc<TmFramer>, frame: &[u8]) {
        let mut buffer = Buffer::allocate(TM_FRAME_FIXED_SIZE);
        buffer.data_mut().copy_from_slice(frame);
        let p = framer.data_return_in(0);
        p.target
            .invoke(p.port_num, buffer, &FrameContext::default());
    }

    /// The exact 1024 bytes of the first frame, field by field.
    #[test]
    fn frame_bytes_are_byte_exact() {
        let (framer, rec) = build();
        let ctx = FrameContext::default(); // vcId = 1 per ComCfg
        feed(&framer, &[0x01, 0x02, 0x03, 0x04], &ctx);

        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        let frame = &out[0].0;
        assert_eq!(frame.len(), TM_FRAME_FIXED_SIZE);

        // Primary header: TFVN 00 | SCID 0x044 | VCID 1 | OCF 0 = 0x0442,
        // master/virtual counts 0, data field status 0x1800.
        assert_eq!(&frame[0..6], &[0x04, 0x42, 0x00, 0x00, 0x18, 0x00]);
        // Payload at the data field origin.
        assert_eq!(&frame[6..10], &[0x01, 0x02, 0x03, 0x04]);
        // Idle packet header: APID 0x7FF, seq flags 0b11 count 0, length
        // token = idle size - 7 = (1022 - 10) - 7 = 1005 = 0x03ED.
        assert_eq!(&frame[10..16], &[0x07, 0xFF, 0xC0, 0x00, 0x03, 0xED]);
        // Idle data all the way to the trailer.
        assert!(
            frame[16..TRAILER_OFFSET]
                .iter()
                .all(|&b| b == IDLE_DATA_PATTERN)
        );
        // FECF over [0, 1022).
        let crc = Crc16::compute(&frame[..TRAILER_OFFSET]);
        assert_eq!(
            &frame[TRAILER_OFFSET..],
            &crc.to_be_bytes(),
            "trailer must hold the CRC big-endian"
        );
        // The whole frame checksums to zero (the FECF residue property).
        assert_eq!(Crc16::compute(frame), 0);
        // The context is forwarded unmodified and the payload returned.
        assert_eq!(out[0].1, ctx);
        let ret = rec.data_return.lock().unwrap();
        assert_eq!(ret[0].0, vec![0x01, 0x02, 0x03, 0x04]);
        assert_eq!(ret[0].1, ctx);
    }

    /// The virtual channel id comes from the context and lands in bits
    /// `[3:1]` of the first header word.
    #[test]
    fn virtual_channel_id_comes_from_context() {
        let (framer, rec) = build();
        let ctx = FrameContext {
            vc_id: 5,
            ..FrameContext::default()
        };
        feed(&framer, &[0xFF], &ctx);
        let out = rec.data_out.lock().unwrap();
        let word = u16::from_be_bytes([out[0].0[0], out[0].0[1]]);
        assert_eq!(word, (SPACECRAFT_ID << 4) | (5 << 1));
        assert_eq!(word, 0x044A);
    }

    /// The counts are written BEFORE being incremented, so the first frame
    /// carries 0/0, and both wrap mod 256.
    #[test]
    fn frame_counts_increment_after_use_and_wrap() {
        let (framer, rec) = build_with_returner();
        for _ in 0..258 {
            feed(&framer, &[0xAB], &FrameContext::default());
        }
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 258);
        assert_eq!((out[0].0[2], out[0].0[3]), (0, 0));
        assert_eq!((out[1].0[2], out[1].0[3]), (1, 1));
        assert_eq!((out[255].0[2], out[255].0[3]), (255, 255));
        assert_eq!((out[256].0[2], out[256].0[3]), (0, 0)); // wrapped
        assert_eq!((out[257].0[2], out[257].0[3]), (1, 1));
    }

    /// A framer wired to a recorder that also returns the frame buffer.
    fn build_with_returner() -> (Arc<TmFramer>, Arc<Recorder>) {
        struct Both {
            rec: Arc<Recorder>,
            framer: StdMutex<Option<Arc<TmFramer>>>,
        }
        impl ComDataWithContextPort for Both {
            fn invoke(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
                self.rec
                    .data_out
                    .lock()
                    .unwrap()
                    .push((data.data().to_vec(), *context));
                let framer = self.framer.lock().unwrap().clone();
                if let Some(framer) = framer {
                    let p = framer.data_return_in(0);
                    p.target.invoke(p.port_num, data, context);
                }
            }
        }
        let rec = Arc::new(Recorder::default());
        let framer = TmFramer::new("tmFramer");
        let both = Arc::new(Both {
            rec: rec.clone(),
            framer: StdMutex::new(Some(framer.clone())),
        });
        framer.data_out.connect(both, 0);
        framer.data_return_out.connect(rec.clone(), 1);
        (framer, rec)
    }

    /// The idle packet shrinks as the payload grows, down to its 7-byte
    /// minimum at the true maximum payload (1009 bytes).
    #[test]
    fn idle_packet_fills_exactly_to_the_trailer() {
        let (framer, rec) = build_with_returner();
        for size in [1usize, 100, 500, MAX_PAYLOAD_SIZE] {
            feed(&framer, &vec![0x5A; size], &FrameContext::default());
        }
        let out = rec.data_out.lock().unwrap();
        for (i, size) in [1usize, 100, 500, MAX_PAYLOAD_SIZE].iter().enumerate() {
            let frame = &out[i].0;
            let idle_start = TMHeader::SERIALIZED_SIZE + size;
            let idle_size = TRAILER_OFFSET - idle_start;
            assert_eq!(&frame[idle_start..idle_start + 2], &[0x07, 0xFF]);
            assert_eq!(&frame[idle_start + 2..idle_start + 4], &[0xC0, 0x00]);
            let token = u16::from_be_bytes([frame[idle_start + 4], frame[idle_start + 5]]);
            assert_eq!(usize::from(token), idle_size - 7);
            assert!(
                frame[idle_start + 6..TRAILER_OFFSET]
                    .iter()
                    .all(|&b| b == IDLE_DATA_PATTERN)
            );
            assert_eq!(Crc16::compute(frame), 0);
        }
        // The largest payload leaves exactly the 7-byte minimum idle packet.
        assert_eq!(
            TRAILER_OFFSET - (TMHeader::SERIALIZED_SIZE + MAX_PAYLOAD_SIZE),
            MIN_IDLE_PACKET_SIZE
        );
        assert_eq!(MAX_PAYLOAD_SIZE, 1009);
    }

    /// Ownership handshake: the frame must be returned before the next one.
    #[test]
    fn ownership_returns_on_data_return_in() {
        let (framer, rec) = build();
        assert!(framer.owns_frame_buffer());
        feed(&framer, &[0x01], &FrameContext::default());
        assert!(!framer.owns_frame_buffer());
        let frame = rec.data_out.lock().unwrap()[0].0.clone();
        give_back(&framer, &frame);
        assert!(framer.owns_frame_buffer());
        feed(&framer, &[0x02], &FrameContext::default());
        assert_eq!(rec.data_out.lock().unwrap().len(), 2);
    }

    /// A second frame before the first came back asserts (C++ FW_ASSERT on
    /// the ownership state).
    #[test]
    #[should_panic(expected = "Assert:")]
    fn framing_without_the_buffer_asserts() {
        let (framer, _rec) = build();
        feed(&framer, &[0x01], &FrameContext::default());
        feed(&framer, &[0x02], &FrameContext::default());
    }

    /// Returning a buffer that is not the frame buffer asserts.
    #[test]
    #[should_panic(expected = "Assert:")]
    fn returning_a_foreign_buffer_asserts() {
        let (framer, _rec) = build();
        feed(&framer, &[0x01], &FrameContext::default());
        let p = framer.data_return_in(0);
        p.target
            .invoke(p.port_num, Buffer::allocate(16), &FrameContext::default());
    }

    /// Returning a buffer while the framer already owns one asserts.
    #[test]
    #[should_panic(expected = "Assert:")]
    fn returning_while_owned_asserts() {
        let (framer, _rec) = build();
        let p = framer.data_return_in(0);
        p.target.invoke(
            p.port_num,
            Buffer::allocate(TM_FRAME_FIXED_SIZE),
            &FrameContext::default(),
        );
    }

    /// A payload above the header/trailer capacity asserts on `dataIn`.
    #[test]
    #[should_panic(expected = "Assert:")]
    fn oversized_payload_asserts() {
        let (framer, _rec) = build();
        feed(
            &framer,
            &vec![0u8; TM_PAYLOAD_CAPACITY + 1],
            &FrameContext::default(),
        );
    }

    /// Gotcha: a payload that passes the `dataIn` assert but leaves less
    /// than 7 bytes for the idle packet asserts inside the idle fill.
    #[test]
    #[should_panic(expected = "Assert:")]
    fn payload_leaving_no_room_for_the_idle_packet_asserts() {
        let (framer, _rec) = build();
        // MAX_PAYLOAD_SIZE + 1 is still within TM_PAYLOAD_CAPACITY, so the
        // dataIn assert passes and the idle-fill assert is the one that
        // fires.
        feed(
            &framer,
            &vec![0u8; MAX_PAYLOAD_SIZE + 1],
            &FrameContext::default(),
        );
    }

    /// comStatusIn forwards only when comStatusOut is connected.
    #[test]
    fn com_status_forwarding() {
        let (framer, rec) = build();
        let p = framer.com_status_in(0);
        let mut cond = Success::Success;
        p.target.invoke(p.port_num, &mut cond);
        assert!(rec.statuses.lock().unwrap().is_empty());
        framer.com_status_out.connect(rec.clone(), 0);
        let mut cond = Success::Success;
        p.target.invoke(p.port_num, &mut cond);
        assert_eq!(*rec.statuses.lock().unwrap(), vec![Success::Success]);
    }
}
