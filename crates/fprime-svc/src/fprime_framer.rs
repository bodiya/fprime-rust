//! # FprimeFramer — port of `Svc::FprimeFramer` (passive)
//!
//! C++ sources: `Svc/FprimeFramer/FprimeFramer.{cpp,hpp,fpp}` and
//! `Svc/FprimeProtocol/FprimeProtocol.fpp`.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (FprimeFramer + wire formats).
//!
//! Wraps each outgoing payload in an F Prime frame:
//! `[start word u32 BE][length u32 BE][payload][CRC32 u32 BE]` where the CRC
//! covers header + payload. This module also single-sources the frame
//! protocol constants ([`START_WORD`], [`HEADER_SIZE`], [`TRAILER_SIZE`])
//! used by the deframer and the frame detector — mirroring the C++ rule that
//! everything validates against the FPP `FrameHeader` default value.
//!
//! All input ports are `sync` (handlers run on the caller's thread, no
//! component mutex — C++ parity: none of the framer inputs are guarded).

use fprime_comp::{
    BufferGetPort, BufferSendPort, ComDataWithContextPort, EventGlue, EventThrottle, OutputPort,
    PassiveBase, PortRef, SuccessConditionPort,
};
use fprime_config::{FwEventIdType, FwIndexType, FwSizeType};
use fprime_fw::{Buffer, FrameContext, LengthMode, SerBuf, Success, fw_assert};
use fprime_utils::Hash;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// F Prime frame protocol constants (Svc/FprimeProtocol).
// ---------------------------------------------------------------------------

/// Frame start word (`FprimeProtocol::FrameHeader.startWord` FPP default).
pub const START_WORD: u32 = 0xdead_beef;
/// Serialized frame header size: `[startWord u32][lengthField u32]`.
pub const HEADER_SIZE: usize = 8;
/// Serialized frame trailer size: `[crcField u32]`.
pub const TRAILER_SIZE: usize = 4;
/// Minimum valid frame size (header + trailer, zero-length payload).
pub const MIN_FRAME_SIZE: usize = HEADER_SIZE + TRAILER_SIZE;

/// `NoBufferAvailable` event id (FPP declaration order, relative).
pub const EVENTID_NO_BUFFER_AVAILABLE: FwEventIdType = 0;
/// FPP `throttle 5` on `NoBufferAvailable`.
pub const EVENTID_NO_BUFFER_AVAILABLE_THROTTLE: u32 = 5;

/// `Svc::FprimeFramer` — passive framer for the F Prime protocol.
pub struct FprimeFramer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// `bufferAllocate` — allocates the frame buffer (`Fw.BufferGet`).
    pub buffer_allocate: OutputPort<dyn BufferGetPort>,
    /// `bufferDeallocate` — returns frame buffers to the allocator.
    pub buffer_deallocate: OutputPort<dyn BufferSendPort>,
    /// `dataOut` — framed data to the com adapter.
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — returns the incoming (unframed) buffer upstream.
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `comStatusOut` — forwards com status upstream (to ComQueue).
    pub com_status_out: OutputPort<dyn SuccessConditionPort>,
    /// Throttle for `NoBufferAvailable` (FPP `throttle 5`).
    no_buffer_available_throttle: EventThrottle,
}

impl FprimeFramer {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            buffer_allocate: OutputPort::new(),
            buffer_deallocate: OutputPort::new(),
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

    // -- Input-port factories -----------------------------------------------

    /// `dataIn` — SYNC `Svc.ComDataWithContext` input: payload to frame.
    pub fn data_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(Arc::new(DataInAdapter { comp: self.clone() }), port_num)
    }

    /// `dataReturnIn` — SYNC input: framed buffer coming back from the com
    /// adapter; goes to `bufferDeallocate`.
    pub fn data_return_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(
            Arc::new(DataReturnInAdapter { comp: self.clone() }),
            port_num,
        )
    }

    /// `comStatusIn` — SYNC `Fw.SuccessCondition` input: forwarded upstream.
    pub fn com_status_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn SuccessConditionPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Handlers ------------------------------------------------------------

    /// `dataIn` handler: allocate, serialize header + payload + CRC trailer,
    /// emit the frame, return the original payload buffer.
    fn data_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        let frame_size = HEADER_SIZE + data.size() + TRAILER_SIZE;
        // C++ parity: payload must fit the u32 lengthField.
        fw_assert!(data.size() as u64 <= u32::MAX as u64, frame_size as i32);

        // Allocate the frame buffer.
        let p = self.buffer_allocate.get();
        let mut frame_buffer = p.target.invoke(p.port_num, frame_size as FwSizeType);
        // The allocator may return an invalid or smaller-than-requested
        // buffer: drop the frame (with an event), deallocating a valid-but-
        // small buffer, and return the original data.
        if !frame_buffer.is_valid() || frame_buffer.size() < frame_size {
            if self.no_buffer_available_throttle.ok_to_emit() {
                self.evt.log_event(
                    self.base.get_id_base(),
                    EVENTID_NO_BUFFER_AVAILABLE,
                    fprime_fw::LogSeverity::WarningHi,
                    "Failed to allocate a frame buffer: frame dropped",
                    |_buf| fprime_fw::SerializeStatus::Ok,
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

        // Serialize header (start word + length) then the raw payload
        // (OMIT_LENGTH — no length token, C++ parity).
        {
            let mut ser = frame_buffer.get_serializer();
            let status = ser.serialize_u32_be(START_WORD);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize_u32_be(data.size() as u32);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize_bytes(
                data.data(),
                LengthMode::OmitLength,
                fprime_fw::Endianness::Big,
            );
            fw_assert!(status.is_ok(), status as i32);
        }
        // CRC over header + payload, stored big-endian in the trailer.
        let crc = Hash::hash_u32(&frame_buffer.data()[..frame_size - TRAILER_SIZE]);
        {
            let mut ser = frame_buffer.get_serializer();
            let status = ser.move_ser_to_offset(frame_size - TRAILER_SIZE);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize_u32_be(crc);
            fw_assert!(status.is_ok(), status as i32);
        }
        // Trim to the actual frame size (the allocator may have handed out a
        // larger buffer).
        frame_buffer.set_size(frame_size);

        // Frame out (always connected), then return the original payload
        // buffer to its sender (always connected).
        let p = self.data_out.get();
        p.target.invoke(p.port_num, frame_buffer, context);
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    fn data_return_in_handler(&self, _port_num: FwIndexType, frame_buffer: Buffer) {
        // The framed buffer coming back from the com adapter is ours: return
        // it to the allocator.
        let p = self.buffer_deallocate.get();
        p.target.invoke(p.port_num, frame_buffer);
    }
}

/// `comStatusIn` handler — forward when `comStatusOut` is connected
/// (C++ parity: the only optional out-port invocation in this component).
impl SuccessConditionPort for FprimeFramer {
    fn invoke(&self, _port_num: FwIndexType, condition: &mut Success) {
        if let Some(p) = self.com_status_out.try_get() {
            p.target.invoke(p.port_num, condition);
        }
    }
}

/// Adapter for the sync `dataIn` port.
struct DataInAdapter {
    comp: Arc<FprimeFramer>,
}

impl ComDataWithContextPort for DataInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.comp.data_in_handler(port_num, data, context);
    }
}

/// Adapter for the sync `dataReturnIn` port.
struct DataReturnInAdapter {
    comp: Arc<FprimeFramer>,
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
    use fprime_comp::{LogPort, TimePort};
    use fprime_config::FwChanIdType;
    use fprime_fw::{LogBuffer, LogSeverity, Time};
    use std::sync::Mutex;

    /// Records everything the framer can emit.
    #[derive(Default)]
    struct Recorder {
        /// (buffer bytes, context apid) for dataOut.
        data_out: Mutex<Vec<(Vec<u8>, FrameContext)>>,
        /// dataReturnOut records.
        data_return: Mutex<Vec<(Vec<u8>, FrameContext)>>,
        /// bufferDeallocate records (window bytes).
        deallocated: Mutex<Vec<Vec<u8>>>,
        /// comStatusOut records.
        statuses: Mutex<Vec<Success>>,
        /// events (id, severity).
        events: Mutex<Vec<(FwChanIdType, LogSeverity)>>,
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
            id: fprime_config::FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            _args: &mut LogBuffer,
        ) {
            self.events.lock().unwrap().push((id, severity));
        }
    }

    impl TimePort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, time: &mut Time) {
            *time = Time::default();
        }
    }

    /// Allocator stub: hands out buffers of a configurable size (`0` =>
    /// invalid buffer).
    struct Allocator {
        /// Per-call sizes; last entry repeats.
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
            let sizes = self.sizes.lock().unwrap();
            let n = match sizes.last() {
                Some(&n) => n,
                None => size as usize, // exact allocation
            };
            if n == 0 {
                Buffer::empty()
            } else {
                Buffer::allocate(n)
            }
        }
    }

    fn build(alloc: &Arc<Allocator>) -> (Arc<FprimeFramer>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let framer = FprimeFramer::new("framer");
        framer.buffer_allocate.connect(alloc.clone(), 0);
        framer.buffer_deallocate.connect(rec.clone(), 0);
        framer.data_out.connect(rec.clone(), 0);
        framer.data_return_out.connect(rec.clone(), 1);
        framer.evt.log_out.connect(rec.clone(), 0);
        (framer, rec)
    }

    fn payload_buffer(bytes: &[u8]) -> Buffer {
        let mut b = Buffer::allocate(bytes.len().max(1));
        b.data_mut()[..bytes.len()].copy_from_slice(bytes);
        b.set_size(bytes.len());
        b
    }

    /// Literal wire bytes: "123456789" payload. CRC computed over
    /// header+payload with an independent implementation (python zlib).
    #[test]
    fn frame_bytes_are_byte_exact() {
        let alloc = Allocator::exact();
        let (framer, rec) = build(&alloc);
        let ctx = FrameContext {
            apid: fprime_fw::Apid::FwPacketTelem,
            ..FrameContext::default()
        };
        let din = framer.data_in(0);
        din.target
            .invoke(din.port_num, payload_buffer(b"123456789"), &ctx);

        let data_out = rec.data_out.lock().unwrap();
        assert_eq!(data_out.len(), 1);
        let mut expected = vec![0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x09];
        expected.extend_from_slice(b"123456789");
        expected.extend_from_slice(&0xDD59_71FAu32.to_be_bytes()); // zlib.crc32
        assert_eq!(data_out[0].0, expected);
        assert_eq!(data_out[0].1, ctx); // same context forwarded
        // Original buffer returned with the same context.
        let ret = rec.data_return.lock().unwrap();
        assert_eq!(ret.len(), 1);
        assert_eq!(ret[0].0, b"123456789");
        assert_eq!(ret[0].1, ctx);
        assert_eq!(*alloc.requests.lock().unwrap(), vec![21]);
    }

    /// Zero-length payload produces the 12-byte minimum frame.
    #[test]
    fn empty_payload_frame_is_minimum_size() {
        let alloc = Allocator::exact();
        let (framer, rec) = build(&alloc);
        let din = framer.data_in(0);
        // A valid zero-size window: use a 1-byte storage with size 0.
        din.target
            .invoke(din.port_num, payload_buffer(&[]), &FrameContext::default());
        let data_out = rec.data_out.lock().unwrap();
        let mut expected = vec![0xde, 0xad, 0xbe, 0xef, 0, 0, 0, 0];
        expected.extend_from_slice(&0xDA8D_2BE2u32.to_be_bytes()); // zlib.crc32
        assert_eq!(data_out[0].0, expected);
    }

    /// The allocator may hand out a larger buffer; the frame is trimmed.
    #[test]
    fn oversized_allocation_is_trimmed() {
        let alloc = Allocator::fixed(64);
        let (framer, rec) = build(&alloc);
        let din = framer.data_in(0);
        din.target.invoke(
            din.port_num,
            payload_buffer(b"abc"),
            &FrameContext::default(),
        );
        let data_out = rec.data_out.lock().unwrap();
        assert_eq!(data_out[0].0.len(), MIN_FRAME_SIZE + 3);
    }

    /// Invalid allocation: event + dataReturnOut of the original, no dataOut,
    /// no deallocation.
    #[test]
    fn invalid_allocation_drops_frame() {
        let alloc = Allocator::fixed(0);
        let (framer, rec) = build(&alloc);
        let din = framer.data_in(0);
        din.target.invoke(
            din.port_num,
            payload_buffer(b"xy"),
            &FrameContext::default(),
        );
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert_eq!(rec.data_return.lock().unwrap().len(), 1);
        assert_eq!(rec.data_return.lock().unwrap()[0].0, b"xy");
        assert!(rec.deallocated.lock().unwrap().is_empty());
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(EVENTID_NO_BUFFER_AVAILABLE, LogSeverity::WarningHi)]
        );
    }

    /// Valid-but-too-small allocation: the small buffer is deallocated and
    /// the frame dropped (gotcha: allocator returning SMALLER valid buffer).
    #[test]
    fn small_allocation_is_deallocated_and_frame_dropped() {
        let alloc = Allocator::fixed(4);
        let (framer, rec) = build(&alloc);
        let din = framer.data_in(0);
        din.target.invoke(
            din.port_num,
            payload_buffer(b"hello"),
            &FrameContext::default(),
        );
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert_eq!(rec.deallocated.lock().unwrap().len(), 1);
        assert_eq!(rec.deallocated.lock().unwrap()[0].len(), 4);
        assert_eq!(rec.data_return.lock().unwrap()[0].0, b"hello");
        assert_eq!(rec.events.lock().unwrap().len(), 1);
    }

    /// NoBufferAvailable throttles at 5 (FPP `throttle 5`), and the clear
    /// helper re-enables it.
    #[test]
    fn no_buffer_available_event_throttles_at_five() {
        let alloc = Allocator::fixed(0);
        let (framer, rec) = build(&alloc);
        let din = framer.data_in(0);
        for _ in 0..8 {
            din.target
                .invoke(din.port_num, payload_buffer(b"z"), &FrameContext::default());
        }
        assert_eq!(rec.events.lock().unwrap().len(), 5);
        framer.clear_no_buffer_available_throttle();
        din.target
            .invoke(din.port_num, payload_buffer(b"z"), &FrameContext::default());
        assert_eq!(rec.events.lock().unwrap().len(), 6);
        // Every drop still returned the original buffer.
        assert_eq!(rec.data_return.lock().unwrap().len(), 9);
    }

    /// comStatusIn forwards only when comStatusOut is connected.
    #[test]
    fn com_status_forwarding() {
        let alloc = Allocator::exact();
        let (framer, rec) = build(&alloc);
        let sin = framer.com_status_in(0);
        // Unconnected: silently dropped.
        let mut cond = Success::Success;
        sin.target.invoke(sin.port_num, &mut cond);
        assert!(rec.statuses.lock().unwrap().is_empty());
        // Connected: forwarded.
        framer.com_status_out.connect(rec.clone(), 0);
        let mut cond = Success::Failure;
        sin.target.invoke(sin.port_num, &mut cond);
        assert_eq!(*rec.statuses.lock().unwrap(), vec![Success::Failure]);
    }

    /// dataReturnIn (framed buffer back from the com adapter) deallocates.
    #[test]
    fn data_return_in_deallocates() {
        let alloc = Allocator::exact();
        let (framer, rec) = build(&alloc);
        let drin = framer.data_return_in(0);
        drin.target.invoke(
            drin.port_num,
            payload_buffer(b"frame"),
            &FrameContext::default(),
        );
        assert_eq!(rec.deallocated.lock().unwrap().len(), 1);
        assert_eq!(rec.deallocated.lock().unwrap()[0], b"frame");
    }
}
