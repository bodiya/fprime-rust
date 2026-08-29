//! # FrameAccumulator — port of `Svc::FrameAccumulator` (passive, guarded)
//! plus the `FrameDetector` trait and `Svc::FrameDetectors::FprimeFrameDetector`.
//!
//! C++ sources: `Svc/FrameAccumulator/FrameAccumulator.{cpp,hpp,fpp}`,
//! `Svc/FrameAccumulator/FrameDetector.hpp`,
//! `Svc/FrameAccumulator/FrameDetector/FprimeFrameDetector.cpp`.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (FrameAccumulator + gotchas).
//!
//! Accumulates raw byte-stream chunks into a [`CircularBuffer`] and uses a
//! pluggable [`FrameDetector`] to find whole frames, which are copied into
//! allocated buffers and sent downstream. The incoming driver buffer is
//! ALWAYS returned immediately (the accumulator copies). Resync policy is
//! exact C++: rotate one byte on `NoFrameDetected`, drop a whole detected
//! frame only when allocation fails while the ring is full.

use crate::fprime_framer::{HEADER_SIZE, MIN_FRAME_SIZE, START_WORD};
use fprime_comp::{
    BufferGetPort, BufferSendPort, ComDataWithContextPort, EventGlue, OutputPort, PassiveBase,
    PortRef,
};
use fprime_config::{FwEventIdType, FwIndexType, FwSizeType};
use fprime_fw::{Buffer, FrameContext, LogSeverity, SerBuf, fw_assert};
use fprime_utils::{CircularBuffer, Hash};
use std::sync::{Arc, Mutex};

/// Relative event ids (FPP declaration order).
pub const EVENTID_NO_BUFFER_AVAILABLE: FwEventIdType = 0;
pub const EVENTID_FRAME_DETECTION_SIZE_ERROR: FwEventIdType = 1;
pub const EVENTID_FRAME_DETECTION_VALID_FRAME_DROPPED: FwEventIdType = 2;

/// Detection outcome (`Svc::FrameDetector::Status` with the C++ `size_out`
/// output parameter folded into the variants).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectorStatus {
    /// A frame is available at the current ring offset; the payload is the
    /// TOTAL frame size from that offset.
    FrameDetected(usize),
    /// No frame is possible at the current offset (e.g. wrong start word).
    NoFrameDetected,
    /// A frame might be possible but more data is needed. The payload is the
    /// TOTAL number of bytes needed and MUST exceed the bytes currently
    /// available (the accumulator `fw_assert`s this — C++ parity).
    MoreDataNeeded(usize),
}

/// Frame-detection strategy (`Svc::FrameDetector`). `detect` must not
/// consume data from the ring.
pub trait FrameDetector: Send + Sync {
    /// Detect whether a frame is available at the ring's current offset.
    fn detect(&self, ring: &CircularBuffer) -> DetectorStatus;
}

/// `Svc::FrameDetectors::FprimeFrameDetector` — detects F Prime protocol
/// frames (`[0xdeadbeef][len][payload][crc32]`).
#[derive(Debug, Default, Clone, Copy)]
pub struct FprimeFrameDetector;

impl FprimeFrameDetector {
    /// Construct the detector.
    pub const fn new() -> Self {
        Self
    }
}

impl FrameDetector for FprimeFrameDetector {
    fn detect(&self, ring: &CircularBuffer) -> DetectorStatus {
        let available = ring.get_allocated_size();
        // Not enough for header + trailer yet.
        if available < MIN_FRAME_SIZE {
            return DetectorStatus::MoreDataNeeded(MIN_FRAME_SIZE);
        }
        // Peek the header.
        let mut start_word = 0u32;
        if !ring.peek_u32_be(&mut start_word, 0).is_ok() {
            return DetectorStatus::NoFrameDetected;
        }
        if start_word != START_WORD {
            return DetectorStatus::NoFrameDetected;
        }
        let mut length_field = 0u32;
        if !ring.peek_u32_be(&mut length_field, 4).is_ok() {
            return DetectorStatus::NoFrameDetected;
        }
        // Overflow guard (C++ parity; cannot trip with usize >= 64 bits but
        // kept structural).
        let expected = match (length_field as usize).checked_add(MIN_FRAME_SIZE) {
            Some(v) => v,
            None => return DetectorStatus::NoFrameDetected,
        };
        // A frame that can never fit the ring is unprocessable: report
        // NO_FRAME_DETECTED so the accumulator resyncs byte by byte.
        if ring.get_capacity() < expected {
            return DetectorStatus::NoFrameDetected;
        }
        // Frame fits but has not fully arrived.
        if available < expected {
            return DetectorStatus::MoreDataNeeded(expected);
        }
        // Peek the trailer CRC and compute over header + payload
        // (byte-by-byte peek, C++ parity).
        let mut transmitted_crc = 0u32;
        if !ring
            .peek_u32_be(&mut transmitted_crc, HEADER_SIZE + length_field as usize)
            .is_ok()
        {
            return DetectorStatus::NoFrameDetected;
        }
        let mut hash = Hash::new();
        hash.init();
        for i in 0..(HEADER_SIZE + length_field as usize) {
            let mut byte = 0u8;
            let status = ring.peek_u8(&mut byte, i);
            fw_assert!(status.is_ok(), status as i32);
            hash.update(&[byte]);
        }
        if transmitted_crc != hash.finalize() {
            // Corruption: the F Prime protocol has no recovery, drop via
            // byte-by-byte resync.
            return DetectorStatus::NoFrameDetected;
        }
        DetectorStatus::FrameDetected(expected)
    }
}

/// Guarded mutable state: the accumulation ring and the detector, both set
/// by [`FrameAccumulator::configure`].
struct AccumulatorState {
    ring: Option<CircularBuffer>,
    detector: Option<Box<dyn FrameDetector>>,
}

/// `Svc::FrameAccumulator` — passive frame accumulator.
pub struct FrameAccumulator {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// `bufferAllocate` — allocates buffers to hold extracted frames.
    pub buffer_allocate: OutputPort<dyn BufferGetPort>,
    /// `bufferDeallocate` — returns extracted-frame buffers to the allocator.
    pub buffer_deallocate: OutputPort<dyn BufferSendPort>,
    /// `dataOut` — extracted frames downstream (to the deframer).
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — incoming driver buffers back upstream (immediately).
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// Guarded state (the C++ guarded-dataIn component mutex).
    state: Mutex<AccumulatorState>,
}

impl FrameAccumulator {
    /// Construct the component (must be [`configure`](Self::configure)d
    /// before data arrives).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            buffer_allocate: OutputPort::new(),
            buffer_deallocate: OutputPort::new(),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            state: Mutex::new(AccumulatorState {
                ring: None,
                detector: None,
            }),
        })
    }

    /// C++ `configure(detector, allocationId, allocator, store_size)`:
    /// allocates the accumulation ring (the Rust `CircularBuffer` owns its
    /// storage, so the allocator/id parameters are dropped).
    pub fn configure(&self, detector: Box<dyn FrameDetector>, store_size: usize) {
        let mut state = self.state.lock().unwrap();
        state.ring = Some(CircularBuffer::new(store_size));
        state.detector = Some(detector);
    }

    // -- Input-port factories -----------------------------------------------

    /// `dataIn` — GUARDED `Svc.ComDataWithContext` input: raw byte chunks.
    pub fn data_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(Arc::new(DataInAdapter { comp: self.clone() }), port_num)
    }

    /// `dataReturnIn` — SYNC input: extracted-frame buffer coming back from
    /// downstream; goes to `bufferDeallocate`.
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

    /// `dataIn` handler (guarded). Copies data into the ring in chunks, then
    /// ALWAYS returns the incoming buffer immediately.
    fn data_in_handler(&self, _port_num: FwIndexType, buffer: Buffer, context: &FrameContext) {
        {
            let mut state = self.state.lock().unwrap();
            if buffer.is_valid() {
                self.process_buffer(&mut state, buffer.data(), context);
            }
        }
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, buffer, context);
    }

    /// C++ `processBuffer`: outer loop feeding min(ring free, remaining)
    /// chunks — the ring can be smaller than incoming buffers (gotcha).
    fn process_buffer(&self, state: &mut AccumulatorState, data: &[u8], context: &FrameContext) {
        let buffer_size = data.len();
        let mut offset = 0usize;
        let mut remaining = buffer_size;
        for _ in 0..buffer_size {
            // C++ parity: processRing FW_ASSERTs the detector/ring are
            // configured; an unconfigured accumulator is a coding error.
            let ring = state.ring.as_mut().expect("configure() must be called");
            if remaining == 0 || ring.get_free_size() == 0 {
                break;
            }
            let ring_free = ring.get_free_size();
            let ser_size = ring_free.min(remaining);
            let status = ring.serialize(&data[offset..offset + ser_size]);
            // If data does not fit, there is a coding error (C++ parity).
            fw_assert!(
                status.is_ok(),
                status as i32,
                offset as i32,
                ser_size as i32
            );
            self.process_ring(state, context);
            offset += ser_size;
            remaining -= ser_size;
        }
        // Either everything was processed, or the ring is full (back
        // pressure with an undersized ring).
        // C++ parity: configuration is an init-time invariant (FW_ASSERT).
        let ring = state.ring.as_mut().expect("configure() must be called");
        fw_assert!(
            remaining == 0 || ring.get_free_size() == 0,
            remaining as i32
        );
    }

    fn log(&self, id: FwEventIdType, text: &str) {
        self.evt.log_event(
            self.base.get_id_base(),
            id,
            LogSeverity::WarningHi,
            text,
            |_buf| fprime_fw::SerializeStatus::Ok,
        );
    }

    /// C++ `processRing`: the detect/extract/resync loop.
    fn process_ring(&self, state: &mut AccumulatorState, context: &FrameContext) {
        // C++ parity: FW_ASSERT(this->m_detector != nullptr) — configure()
        // is an init-time invariant.
        let detector = state.detector.as_ref().expect("configure() must be called");
        let ring = state.ring.as_mut().expect("configure() must be called");
        let ring_capacity = ring.get_capacity();

        for _ in 0..ring_capacity {
            let remaining = ring.get_allocated_size();
            if remaining == 0 {
                break;
            }
            let status = detector.detect(ring);
            // Detect must not consume data (C++ parity assert).
            fw_assert!(
                ring.get_allocated_size() == remaining,
                ring.get_allocated_size() as i32,
                remaining as i32
            );

            // Drop frames too large to ever fit the accumulation ring.
            let size_out = match status {
                DetectorStatus::FrameDetected(s) | DetectorStatus::MoreDataNeeded(s) => s,
                DetectorStatus::NoFrameDetected => 0,
            };
            if size_out > ring_capacity {
                // Emit FrameDetectionSizeError(size_out: FwSizeType) and
                // slide one byte to resync.
                self.evt.log_event(
                    self.base.get_id_base(),
                    EVENTID_FRAME_DETECTION_SIZE_ERROR,
                    LogSeverity::WarningHi,
                    &format!("Reported size_out={size_out} exceeds accumulation buffer capacity"),
                    |buf| buf.serialize_u64_be(size_out as FwSizeType),
                );
                let _ = ring.rotate(1);
                fw_assert!(ring.get_allocated_size() == remaining - 1);
                continue;
            }

            match status {
                DetectorStatus::FrameDetected(size_out) => {
                    fw_assert!(size_out != 0);
                    fw_assert!(size_out <= remaining, size_out as i32, remaining as i32);
                    let p = self.buffer_allocate.get();
                    let mut buffer = p.target.invoke(p.port_num, size_out as FwSizeType);
                    if buffer.is_valid() {
                        // C++ peeks size_out bytes into the buffer without a
                        // size check (UB on a small allocation); safe Rust
                        // asserts instead.
                        fw_assert!(buffer.size() >= size_out, buffer.size() as i32);
                        let serialize_status =
                            ring.peek_bytes(&mut buffer.data_mut()[..size_out], 0);
                        fw_assert!(serialize_status.is_ok());
                        buffer.set_size(size_out);
                        let serialize_status = ring.rotate(size_out);
                        fw_assert!(serialize_status.is_ok());
                        fw_assert!(ring.get_allocated_size() == remaining - size_out);
                        // Context is passed through unchanged.
                        let p = self.data_out.get();
                        p.target.invoke(p.port_num, buffer, context);
                    } else {
                        self.log(
                            EVENTID_NO_BUFFER_AVAILABLE,
                            "Could not allocate a valid buffer to fit the detected frame",
                        );
                        // With no free space AND no buffer, drop the whole
                        // frame to keep the resync loop alive (gotcha).
                        if ring.get_free_size() == 0 {
                            let serialize_status = ring.rotate(size_out);
                            fw_assert!(serialize_status.is_ok());
                            fw_assert!(ring.get_allocated_size() == remaining - size_out);
                            self.log(
                                EVENTID_FRAME_DETECTION_VALID_FRAME_DROPPED,
                                "A valid frame was detected but dropped",
                            );
                        }
                        break;
                    }
                }
                DetectorStatus::MoreDataNeeded(size_out) => {
                    // MoreDataNeeded carries the TOTAL bytes needed and must
                    // exceed what is available (contract assert, C++ parity).
                    fw_assert!(size_out > remaining, size_out as i32, remaining as i32);
                    break;
                }
                DetectorStatus::NoFrameDetected => {
                    // Discard a single byte and try again.
                    let _ = ring.rotate(1);
                    fw_assert!(ring.get_allocated_size() == remaining - 1);
                }
            }
        }
    }

    fn data_return_in_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        // The extracted-frame buffer was allocated through bufferAllocate:
        // hand it back to the allocator.
        let p = self.buffer_deallocate.get();
        p.target.invoke(p.port_num, buffer);
    }
}

/// Adapter for the guarded `dataIn` port.
struct DataInAdapter {
    comp: Arc<FrameAccumulator>,
}

impl ComDataWithContextPort for DataInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.comp.data_in_handler(port_num, data, context);
    }
}

/// Adapter for the sync `dataReturnIn` port.
struct DataReturnInAdapter {
    comp: Arc<FrameAccumulator>,
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
    use fprime_comp::LogPort;
    use fprime_fw::{LogBuffer, Time};
    use std::sync::Mutex as StdMutex;

    // -- FprimeFrameDetector unit tests -------------------------------------

    fn ring_with(bytes: &[u8], capacity: usize) -> CircularBuffer {
        let mut ring = CircularBuffer::new(capacity);
        assert!(ring.serialize(bytes).is_ok());
        ring
    }

    fn make_frame(payload: &[u8]) -> Vec<u8> {
        let mut frame = Vec::new();
        frame.extend_from_slice(&START_WORD.to_be_bytes());
        frame.extend_from_slice(&(payload.len() as u32).to_be_bytes());
        frame.extend_from_slice(payload);
        let crc = Hash::hash_u32(&frame);
        frame.extend_from_slice(&crc.to_be_bytes());
        frame
    }

    #[test]
    fn detector_needs_twelve_bytes_minimum() {
        let det = FprimeFrameDetector::new();
        let ring = ring_with(&[0xde, 0xad], 64);
        assert_eq!(det.detect(&ring), DetectorStatus::MoreDataNeeded(12));
    }

    #[test]
    fn detector_rejects_wrong_start_word() {
        let det = FprimeFrameDetector::new();
        let ring = ring_with(&[0u8; 16], 64);
        assert_eq!(det.detect(&ring), DetectorStatus::NoFrameDetected);
    }

    #[test]
    fn detector_reports_total_needed_for_partial_frame() {
        let det = FprimeFrameDetector::new();
        let frame = make_frame(b"hello");
        let ring = ring_with(&frame[..frame.len() - 2], 64);
        assert_eq!(det.detect(&ring), DetectorStatus::MoreDataNeeded(17));
    }

    #[test]
    fn detector_rejects_frame_larger_than_ring_capacity() {
        let det = FprimeFrameDetector::new();
        // Header declaring a 100-byte payload in a 32-byte ring.
        let mut bytes = START_WORD.to_be_bytes().to_vec();
        bytes.extend_from_slice(&100u32.to_be_bytes());
        bytes.extend_from_slice(&[0u8; 8]);
        let ring = ring_with(&bytes, 32);
        assert_eq!(det.detect(&ring), DetectorStatus::NoFrameDetected);
    }

    #[test]
    fn detector_rejects_bad_crc() {
        let det = FprimeFrameDetector::new();
        let mut frame = make_frame(b"xyz");
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        let ring = ring_with(&frame, 64);
        assert_eq!(det.detect(&ring), DetectorStatus::NoFrameDetected);
    }

    #[test]
    fn detector_detects_valid_frame() {
        let det = FprimeFrameDetector::new();
        let frame = make_frame(b"payload");
        let mut ring = ring_with(&frame, 64);
        assert_eq!(
            det.detect(&ring),
            DetectorStatus::FrameDetected(frame.len())
        );
        // Detection does not consume.
        assert_eq!(ring.get_allocated_size(), frame.len());
        // And the ring can then rotate past it.
        assert!(ring.rotate(frame.len()).is_ok());
    }

    // -- FrameAccumulator tests ---------------------------------------------

    #[derive(Default)]
    struct Recorder {
        /// dataOut frames.
        frames: StdMutex<Vec<Vec<u8>>>,
        /// dataReturnOut records (bytes, apid).
        returned: StdMutex<Vec<Vec<u8>>>,
        deallocated: StdMutex<Vec<Vec<u8>>>,
        events: StdMutex<Vec<FwEventIdType>>,
        /// When true the allocator returns invalid buffers.
        starve: StdMutex<bool>,
    }

    impl ComDataWithContextPort for Recorder {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, _context: &FrameContext) {
            if port_num == 0 {
                self.frames.lock().unwrap().push(data.data().to_vec());
            } else {
                self.returned.lock().unwrap().push(data.data().to_vec());
            }
        }
    }

    impl BufferGetPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
            if *self.starve.lock().unwrap() {
                Buffer::empty()
            } else {
                Buffer::allocate(size as usize)
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

    impl LogPort for Recorder {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            _args: &mut LogBuffer,
        ) {
            self.events.lock().unwrap().push(id);
        }
    }

    fn build(ring_size: usize) -> (Arc<FrameAccumulator>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let acc = FrameAccumulator::new("accumulator");
        acc.configure(Box::new(FprimeFrameDetector::new()), ring_size);
        acc.buffer_allocate.connect(rec.clone(), 0);
        acc.buffer_deallocate.connect(rec.clone(), 0);
        acc.data_out.connect(rec.clone(), 0);
        acc.data_return_out.connect(rec.clone(), 1);
        acc.evt.log_out.connect(rec.clone(), 0);
        (acc, rec)
    }

    fn feed(acc: &Arc<FrameAccumulator>, bytes: &[u8]) {
        let mut b = Buffer::allocate(bytes.len().max(1));
        b.data_mut()[..bytes.len()].copy_from_slice(bytes);
        b.set_size(bytes.len());
        let din = acc.data_in(0);
        din.target.invoke(din.port_num, b, &FrameContext::default());
    }

    /// One whole frame in one chunk; incoming buffer returned immediately.
    #[test]
    fn single_frame_extracted() {
        let (acc, rec) = build(128);
        let frame = make_frame(b"data!");
        feed(&acc, &frame);
        assert_eq!(*rec.frames.lock().unwrap(), vec![frame.clone()]);
        assert_eq!(rec.returned.lock().unwrap().len(), 1);
        assert_eq!(rec.returned.lock().unwrap()[0], frame);
    }

    /// Resync: garbage, then a valid frame, then a partial frame; completing
    /// the partial frame in a later chunk emits it.
    #[test]
    fn resync_across_garbage_and_chunk_boundaries() {
        let (acc, rec) = build(128);
        let f1 = make_frame(b"first");
        let f2 = make_frame(b"second");
        let mut chunk1 = vec![0x01, 0x02, 0x03]; // garbage
        chunk1.extend_from_slice(&f1);
        chunk1.extend_from_slice(&f2[..5]); // partial second frame
        feed(&acc, &chunk1);
        assert_eq!(*rec.frames.lock().unwrap(), vec![f1.clone()]);
        feed(&acc, &f2[5..]);
        assert_eq!(*rec.frames.lock().unwrap(), vec![f1, f2]);
        // Every incoming buffer was returned.
        assert_eq!(rec.returned.lock().unwrap().len(), 2);
    }

    /// Garbage containing 0xde bytes (partial start-word lookalikes) still
    /// resyncs to the real frame.
    #[test]
    fn resync_past_start_word_lookalikes() {
        let (acc, rec) = build(128);
        let frame = make_frame(b"ok");
        let mut bytes = vec![0xde, 0xad, 0x00, 0xde, 0xad, 0xbe];
        bytes.extend_from_slice(&frame);
        feed(&acc, &bytes);
        assert_eq!(*rec.frames.lock().unwrap(), vec![frame]);
    }

    /// The ring can be smaller than the incoming buffer: the outer loop
    /// feeds it in chunks (gotcha).
    #[test]
    fn incoming_buffer_larger_than_ring_is_chunked() {
        let (acc, rec) = build(16); // tiny ring
        let f1 = make_frame(b"a");
        let f2 = make_frame(b"b");
        let mut bytes = f1.clone();
        bytes.extend_from_slice(&f2);
        feed(&acc, &bytes); // 26 bytes through a 16-byte ring
        assert_eq!(*rec.frames.lock().unwrap(), vec![f1, f2]);
    }

    /// Allocation failure with a non-full ring: event, frame kept for retry.
    #[test]
    fn allocation_failure_keeps_frame_for_retry() {
        let (acc, rec) = build(128);
        let frame = make_frame(b"keep");
        *rec.starve.lock().unwrap() = true;
        feed(&acc, &frame);
        assert!(rec.frames.lock().unwrap().is_empty());
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![EVENTID_NO_BUFFER_AVAILABLE]
        );
        // Allocation recovers: feeding zero more bytes... feed one garbage
        // byte to re-trigger processing.
        *rec.starve.lock().unwrap() = false;
        feed(&acc, &[0x00]);
        assert_eq!(*rec.frames.lock().unwrap(), vec![frame]);
    }

    /// Allocation failure with a FULL ring: the valid frame is dropped to
    /// keep the resync loop alive (gotcha).
    #[test]
    fn allocation_failure_with_full_ring_drops_frame() {
        let frame = make_frame(b"drop"); // 16 bytes
        let (acc, rec) = build(frame.len()); // ring exactly one frame
        *rec.starve.lock().unwrap() = true;
        feed(&acc, &frame);
        assert!(rec.frames.lock().unwrap().is_empty());
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![
                EVENTID_NO_BUFFER_AVAILABLE,
                EVENTID_FRAME_DETECTION_VALID_FRAME_DROPPED
            ]
        );
        // The ring is empty again; a later frame goes through.
        *rec.starve.lock().unwrap() = false;
        let f2 = make_frame(b"next");
        feed(&acc, &f2);
        assert_eq!(*rec.frames.lock().unwrap(), vec![f2]);
    }

    /// A detector reporting a size larger than the ring capacity triggers
    /// FrameDetectionSizeError and a one-byte slide (stub detector — the
    /// F Prime detector never reports such sizes).
    #[test]
    fn oversized_detection_reports_size_error_and_slides() {
        struct HugeDetector;
        impl FrameDetector for HugeDetector {
            fn detect(&self, ring: &CircularBuffer) -> DetectorStatus {
                if ring.get_allocated_size() > 0 {
                    DetectorStatus::FrameDetected(10_000)
                } else {
                    DetectorStatus::MoreDataNeeded(1)
                }
            }
        }
        let rec = Arc::new(Recorder::default());
        let acc = FrameAccumulator::new("acc");
        acc.configure(Box::new(HugeDetector), 8);
        acc.buffer_allocate.connect(rec.clone(), 0);
        acc.data_out.connect(rec.clone(), 0);
        acc.data_return_out.connect(rec.clone(), 1);
        acc.evt.log_out.connect(rec.clone(), 0);
        feed(&acc, &[0xAA, 0xBB]);
        // One size error per byte in the ring (each slide re-detects).
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![
                EVENTID_FRAME_DETECTION_SIZE_ERROR,
                EVENTID_FRAME_DETECTION_SIZE_ERROR
            ]
        );
        assert!(rec.frames.lock().unwrap().is_empty());
    }

    /// The FrameDetectionSizeError event serializes size_out as FwSizeType
    /// (u64 BE).
    #[test]
    fn size_error_event_arg_bytes() {
        struct ArgRecorder(StdMutex<Vec<Vec<u8>>>);
        impl LogPort for ArgRecorder {
            fn invoke(
                &self,
                _port_num: FwIndexType,
                _id: FwEventIdType,
                _time_tag: &mut Time,
                _severity: LogSeverity,
                args: &mut LogBuffer,
            ) {
                self.0.lock().unwrap().push(args.as_slice().to_vec());
            }
        }
        struct HugeDetector;
        impl FrameDetector for HugeDetector {
            fn detect(&self, _ring: &CircularBuffer) -> DetectorStatus {
                DetectorStatus::FrameDetected(9_999)
            }
        }
        let rec = Arc::new(Recorder::default());
        let args = Arc::new(ArgRecorder(StdMutex::new(vec![])));
        let acc = FrameAccumulator::new("acc");
        acc.configure(Box::new(HugeDetector), 4);
        acc.data_return_out.connect(rec.clone(), 1);
        acc.evt.log_out.connect(args.clone(), 0);
        feed(&acc, &[0x01]);
        assert_eq!(args.0.lock().unwrap()[0], 9_999u64.to_be_bytes().to_vec());
    }

    /// MORE_DATA_NEEDED contract: size_out <= available is a coding error
    /// and asserts (gotcha).
    #[test]
    #[should_panic]
    fn more_data_needed_not_exceeding_available_asserts() {
        struct BadDetector;
        impl FrameDetector for BadDetector {
            fn detect(&self, ring: &CircularBuffer) -> DetectorStatus {
                DetectorStatus::MoreDataNeeded(ring.get_allocated_size())
            }
        }
        let rec = Arc::new(Recorder::default());
        let acc = FrameAccumulator::new("acc");
        acc.configure(Box::new(BadDetector), 16);
        acc.data_return_out.connect(rec.clone(), 1);
        feed(&acc, &[0x01, 0x02]);
    }

    /// dataReturnIn deallocates via bufferDeallocate.
    #[test]
    fn data_return_in_deallocates() {
        let (acc, rec) = build(32);
        let drin = acc.data_return_in(0);
        let mut b = Buffer::allocate(3);
        b.data_mut().copy_from_slice(b"abc");
        drin.target
            .invoke(drin.port_num, b, &FrameContext::default());
        assert_eq!(*rec.deallocated.lock().unwrap(), vec![b"abc".to_vec()]);
    }

    /// An invalid incoming buffer is not processed but still returned.
    #[test]
    fn invalid_incoming_buffer_only_returned() {
        let (acc, rec) = build(32);
        let din = acc.data_in(0);
        din.target
            .invoke(din.port_num, Buffer::empty(), &FrameContext::default());
        assert!(rec.frames.lock().unwrap().is_empty());
        assert_eq!(rec.returned.lock().unwrap().len(), 1);
    }
}
