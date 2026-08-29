//! # FprimeDeframer — port of `Svc::FprimeDeframer` (passive, guarded dataIn)
//!
//! C++ sources: `Svc/FprimeDeframer/FprimeDeframer.{cpp,hpp,fpp}`.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (FprimeDeframer + gotchas).
//!
//! Validates one already-extracted frame per `dataIn` invocation:
//! size >= 12, start word, exact size == 12 + lengthField, CRC over
//! header + payload. Any rejection emits its event and returns the buffer
//! via `dataReturnOut` with the ORIGINAL context. APID extraction happens
//! between the length and CRC checks (exact C++ order — a bad-CRC frame with
//! a payload < 2 bytes emits BOTH `PayloadTooShort` and `InvalidChecksum`).
//! On success the payload is emitted as the SAME allocation advanced by the
//! header and shrunk by the trailer; downstream must return the buffer so
//! the original allocator gets it back.

use crate::fprime_framer::{HEADER_SIZE, MIN_FRAME_SIZE, START_WORD, TRAILER_SIZE};
use fprime_comp::{ComDataWithContextPort, EventGlue, OutputPort, PassiveBase, PortRef};
use fprime_config::{FwEventIdType, FwIndexType, FwPacketDescriptorType};
use fprime_fw::{Apid, Buffer, FrameContext, LogSeverity, SerBuf, SerializeStatus, fw_assert};
use fprime_utils::Hash;
use std::sync::{Arc, Mutex};

/// Relative event ids (FPP declaration order).
pub const EVENTID_INVALID_BUFFER_RECEIVED: FwEventIdType = 0;
pub const EVENTID_INVALID_START_WORD: FwEventIdType = 1;
pub const EVENTID_INVALID_LENGTH_RECEIVED: FwEventIdType = 2;
pub const EVENTID_INVALID_CHECKSUM: FwEventIdType = 3;
pub const EVENTID_PAYLOAD_TOO_SHORT: FwEventIdType = 4;

/// `Svc::FprimeDeframer` — passive deframer for the F Prime protocol.
pub struct FprimeDeframer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// `dataOut` — deframed payload downstream (to the router).
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — buffer ownership back upstream (to the accumulator).
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// The guarded-port mutex (C++ `guarded input port dataIn`; no other
    /// mutable state exists, so the guard protects handler serialization
    /// only).
    guard: Mutex<()>,
}

impl FprimeDeframer {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            guard: Mutex::new(()),
        })
    }

    // -- Input-port factories -----------------------------------------------

    /// `dataIn` — GUARDED `Svc.ComDataWithContext` input: one frame.
    pub fn data_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ComDataWithContextPort> {
        PortRef::new(Arc::new(DataInAdapter { comp: self.clone() }), port_num)
    }

    /// `dataReturnIn` — SYNC input: payload buffer coming back from
    /// downstream; passed through to `dataReturnOut`.
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

    fn log(&self, id: FwEventIdType, severity: LogSeverity, text: &str) {
        self.evt
            .log_event(self.base.get_id_base(), id, severity, text, |_buf| {
                SerializeStatus::Ok
            });
    }

    fn data_return_out(&self, data: Buffer, context: &FrameContext) {
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    /// `dataIn` handler (guarded — runs under the component mutex).
    fn data_in_handler(&self, _port_num: FwIndexType, mut data: Buffer, context: &FrameContext) {
        let _guard = self.guard.lock().unwrap();

        if data.size() < MIN_FRAME_SIZE {
            // Too short to hold header + trailer.
            self.log(
                EVENTID_INVALID_BUFFER_RECEIVED,
                LogSeverity::WarningHi,
                "Frame dropped: The received buffer is not long enough to contain a valid frame (header + trailer)",
            );
            self.data_return_out(data, context);
            return;
        }

        // ---------------- Validate frame header ----------------
        let mut start_word = 0u32;
        let mut length_field = 0u32;
        {
            let mut deser = data.get_deserializer();
            let status = deser.deserialize_u32_be(&mut start_word);
            fw_assert!(status.is_ok(), status as i32);
            let status = deser.deserialize_u32_be(&mut length_field);
            fw_assert!(status.is_ok(), status as i32);
        }
        if start_word != START_WORD {
            self.log(
                EVENTID_INVALID_START_WORD,
                LogSeverity::WarningHi,
                "Frame dropped: The received buffer does not start with the F Prime start word",
            );
            self.data_return_out(data, context);
            return;
        }
        // Frames must be pre-extracted: the buffer must hold the frame
        // EXACTLY (gotcha — oversized buffers are dropped).
        let expected_frame_size = HEADER_SIZE as u64 + length_field as u64 + TRAILER_SIZE as u64;
        if data.size() as u64 != expected_frame_size {
            self.log(
                EVENTID_INVALID_LENGTH_RECEIVED,
                LogSeverity::WarningHi,
                "Frame dropped: The received buffer size cannot hold a frame of specified payload length",
            );
            self.data_return_out(data, context);
            return;
        }
        let length_field = length_field as usize;

        // -------- Attempt to extract APID from the payload --------
        // C++ order parity: this runs BEFORE the CRC check, so the
        // PayloadTooShort event can precede an InvalidChecksum drop.
        let mut context_copy = *context;
        let remaining_after_header = data.size() - HEADER_SIZE;
        if remaining_after_header < TRAILER_SIZE + core::mem::size_of::<FwPacketDescriptorType>() {
            self.log(
                EVENTID_PAYLOAD_TOO_SHORT,
                LogSeverity::WarningLo,
                "The received buffer is too short to contain a valid FwPacketDescriptor",
            );
        } else {
            let mut descriptor: FwPacketDescriptorType = 0;
            {
                let mut deser = data.get_deserializer();
                let status = deser.move_deser_to_offset(HEADER_SIZE);
                fw_assert!(status.is_ok(), status as i32);
                let status = deser.deserialize_u16_be(&mut descriptor);
                fw_assert!(status.is_ok(), status as i32);
            }
            // Valid descriptors update the APID; anything else becomes
            // INVALID_UNINITIALIZED (0x0800) for the router's default arm.
            context_copy.apid = Apid::try_from(descriptor).unwrap_or(Apid::InvalidUninitialized);
        }

        // ---------------- Validate frame trailer (CRC) ----------------
        let mut transmitted_crc = 0u32;
        {
            let mut deser = data.get_deserializer();
            let status = deser.move_deser_to_offset(HEADER_SIZE + length_field);
            fw_assert!(status.is_ok(), status as i32);
            let status = deser.deserialize_u32_be(&mut transmitted_crc);
            fw_assert!(status.is_ok(), status as i32);
        }
        let computed_crc = Hash::hash_u32(&data.data()[..HEADER_SIZE + length_field]);
        if transmitted_crc != computed_crc {
            // Drop with the ORIGINAL context (not the APID-updated copy).
            self.log(
                EVENTID_INVALID_CHECKSUM,
                LogSeverity::WarningHi,
                "Frame dropped: The transmitted frame checksum does not match that computed by the receiver",
            );
            self.data_return_out(data, context);
            return;
        }

        // ---------------- Extract payload from the frame ----------------
        // Same allocation: advance past the header, shrink off the trailer.
        data.advance(HEADER_SIZE as fprime_config::FwSignedSizeType);
        data.set_size(data.size() - TRAILER_SIZE);
        let p = self.data_out.get();
        p.target.invoke(p.port_num, data, &context_copy);
    }

    fn data_return_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        // Passthrough: the payload buffer belongs to the upstream allocator.
        self.data_return_out(data, context);
    }
}

/// Adapter for the guarded `dataIn` port.
struct DataInAdapter {
    comp: Arc<FprimeDeframer>,
}

impl ComDataWithContextPort for DataInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.comp.data_in_handler(port_num, data, context);
    }
}

/// Adapter for the sync `dataReturnIn` port.
struct DataReturnInAdapter {
    comp: Arc<FprimeDeframer>,
}

impl ComDataWithContextPort for DataReturnInAdapter {
    fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.comp.data_return_in_handler(port_num, data, context);
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

    #[derive(Default)]
    struct Recorder {
        /// dataOut records: (payload bytes, context).
        data_out: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        /// dataReturnOut records.
        data_return: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        events: StdMutex<Vec<(FwEventIdType, LogSeverity)>>,
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
            _args: &mut LogBuffer,
        ) {
            self.events.lock().unwrap().push((id, severity));
        }
    }

    fn build() -> (Arc<FprimeDeframer>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let deframer = FprimeDeframer::new("deframer");
        deframer.data_out.connect(rec.clone(), 0);
        deframer.data_return_out.connect(rec.clone(), 1);
        deframer.evt.log_out.connect(rec.clone(), 0);
        (deframer, rec)
    }

    /// Build a valid F Prime frame around `payload`.
    fn make_frame(payload: &[u8]) -> Vec<u8> {
        let mut frame = Vec::new();
        frame.extend_from_slice(&START_WORD.to_be_bytes());
        frame.extend_from_slice(&(payload.len() as u32).to_be_bytes());
        frame.extend_from_slice(payload);
        let crc = Hash::hash_u32(&frame);
        frame.extend_from_slice(&crc.to_be_bytes());
        frame
    }

    fn buffer_of(bytes: &[u8]) -> Buffer {
        let mut b = Buffer::allocate(bytes.len().max(1));
        b.data_mut()[..bytes.len()].copy_from_slice(bytes);
        b.set_size(bytes.len());
        b
    }

    fn feed(deframer: &Arc<FprimeDeframer>, bytes: &[u8], context: &FrameContext) {
        let din = deframer.data_in(0);
        din.target.invoke(din.port_num, buffer_of(bytes), context);
    }

    /// Happy path: command frame is deframed, payload identical, APID set.
    #[test]
    fn valid_frame_emits_payload_with_apid() {
        let (deframer, rec) = build();
        let payload = [0x00u8, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x2A]; // cmd
        let frame = make_frame(&payload);
        let ctx = FrameContext::default();
        feed(&deframer, &frame, &ctx);

        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].0, payload); // header stripped, trailer stripped
        assert_eq!(out[0].1.apid, Apid::FwPacketCommand);
        assert!(rec.data_return.lock().unwrap().is_empty());
        assert!(rec.events.lock().unwrap().is_empty());
    }

    /// The payload window is the same allocation, advanced+shrunk: offset 8.
    #[test]
    fn payload_is_same_allocation_advanced() {
        let (deframer, rec) = build();
        let frame = make_frame(&[0x00, 0x03, 0xAA]); // FILE apid
        feed(&deframer, &frame, &FrameContext::default());
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out[0].0, vec![0x00, 0x03, 0xAA]);
        assert_eq!(out[0].1.apid, Apid::FwPacketFile);
    }

    /// Unknown descriptor -> APID becomes INVALID_UNINITIALIZED (gotcha).
    #[test]
    fn invalid_descriptor_becomes_invalid_uninitialized() {
        let (deframer, rec) = build();
        let frame = make_frame(&[0x12, 0x34, 0x00]);
        feed(&deframer, &frame, &FrameContext::default());
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out[0].1.apid, Apid::InvalidUninitialized);
    }

    /// Payload of 0 or 1 bytes: WARNING_LO PayloadTooShort, incoming APID
    /// kept, frame still emitted (CRC valid).
    #[test]
    fn short_payload_keeps_incoming_apid() {
        let (deframer, rec) = build();
        let ctx = FrameContext {
            apid: Apid::FwPacketTelem,
            ..FrameContext::default()
        };
        feed(&deframer, &make_frame(&[0x99]), &ctx);
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].0, vec![0x99]);
        assert_eq!(out[0].1.apid, Apid::FwPacketTelem); // untouched
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(EVENTID_PAYLOAD_TOO_SHORT, LogSeverity::WarningLo)]
        );
    }

    /// Rejection: buffer shorter than 12 bytes.
    #[test]
    fn too_short_buffer_rejected() {
        let (deframer, rec) = build();
        let ctx = FrameContext::default();
        feed(&deframer, &[0xde, 0xad, 0xbe, 0xef, 0, 0], &ctx);
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert_eq!(rec.data_return.lock().unwrap().len(), 1);
        assert_eq!(rec.data_return.lock().unwrap()[0].1, ctx); // original ctx
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(EVENTID_INVALID_BUFFER_RECEIVED, LogSeverity::WarningHi)]
        );
    }

    /// Rejection: wrong start word.
    #[test]
    fn wrong_start_word_rejected() {
        let (deframer, rec) = build();
        let mut frame = make_frame(&[0x00, 0x00]);
        frame[0] = 0xCA;
        feed(&deframer, &frame, &FrameContext::default());
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(EVENTID_INVALID_START_WORD, LogSeverity::WarningHi)]
        );
        assert_eq!(rec.data_return.lock().unwrap().len(), 1);
    }

    /// Rejection: buffer size != 12 + lengthField exactly (oversized buffer
    /// carrying a valid frame + extra byte is dropped — gotcha).
    #[test]
    fn oversized_buffer_rejected_exact_size_required() {
        let (deframer, rec) = build();
        let mut frame = make_frame(&[0x00, 0x00, 0x01]);
        frame.push(0xFF); // extra trailing byte
        feed(&deframer, &frame, &FrameContext::default());
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(EVENTID_INVALID_LENGTH_RECEIVED, LogSeverity::WarningHi)]
        );
    }

    /// Rejection: corrupted CRC; drop uses the ORIGINAL context even though
    /// the APID was extracted (gotcha).
    #[test]
    fn bad_crc_rejected_with_original_context() {
        let (deframer, rec) = build();
        let mut frame = make_frame(&[0x00, 0x00, 0x55]);
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        let ctx = FrameContext {
            apid: Apid::FwPacketTelem,
            ..FrameContext::default()
        };
        feed(&deframer, &frame, &ctx);
        assert!(rec.data_out.lock().unwrap().is_empty());
        let ret = rec.data_return.lock().unwrap();
        assert_eq!(ret[0].1.apid, Apid::FwPacketTelem); // original, not COMMAND
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![(EVENTID_INVALID_CHECKSUM, LogSeverity::WarningHi)]
        );
    }

    /// C++ order parity: short payload + bad CRC emits PayloadTooShort THEN
    /// InvalidChecksum (APID extraction precedes CRC validation).
    #[test]
    fn short_payload_with_bad_crc_emits_both_events() {
        let (deframer, rec) = build();
        let mut frame = make_frame(&[0x77]);
        let last = frame.len() - 1;
        frame[last] ^= 0x01;
        feed(&deframer, &frame, &FrameContext::default());
        assert_eq!(
            *rec.events.lock().unwrap(),
            vec![
                (EVENTID_PAYLOAD_TOO_SHORT, LogSeverity::WarningLo),
                (EVENTID_INVALID_CHECKSUM, LogSeverity::WarningHi),
            ]
        );
        assert!(rec.data_out.lock().unwrap().is_empty());
    }

    /// dataReturnIn passes through to dataReturnOut.
    #[test]
    fn data_return_in_passthrough() {
        let (deframer, rec) = build();
        let drin = deframer.data_return_in(0);
        let ctx = FrameContext {
            apid: Apid::FwPacketFile,
            ..FrameContext::default()
        };
        drin.target.invoke(drin.port_num, buffer_of(b"pay"), &ctx);
        let ret = rec.data_return.lock().unwrap();
        assert_eq!(ret.len(), 1);
        assert_eq!(ret[0].0, b"pay");
        assert_eq!(ret[0].1.apid, Apid::FwPacketFile);
    }

    /// Round trip: framer output -> deframer -> identical payload.
    #[test]
    fn framer_deframer_round_trip() {
        use crate::fprime_framer::FprimeFramer;
        use fprime_comp::BufferGetPort;
        use fprime_config::FwSizeType;

        struct Alloc;
        impl BufferGetPort for Alloc {
            fn invoke(&self, _p: FwIndexType, size: FwSizeType) -> Buffer {
                Buffer::allocate(size as usize)
            }
        }
        /// Bridge: framer dataOut -> deframer dataIn.
        struct Bridge(PortRef<dyn ComDataWithContextPort>);
        impl ComDataWithContextPort for Bridge {
            fn invoke(&self, _p: FwIndexType, data: Buffer, context: &FrameContext) {
                self.0.target.invoke(self.0.port_num, data, context);
            }
        }
        struct Sink;
        impl ComDataWithContextPort for Sink {
            fn invoke(&self, _p: FwIndexType, _data: Buffer, _c: &FrameContext) {}
        }
        impl fprime_comp::BufferSendPort for Sink {
            fn invoke(&self, _p: FwIndexType, _b: Buffer) {}
        }

        let (deframer, rec) = build();
        let framer = FprimeFramer::new("framer");
        framer.buffer_allocate.connect(Arc::new(Alloc), 0);
        framer.buffer_deallocate.connect(Arc::new(Sink), 0);
        framer
            .data_out
            .connect(Arc::new(Bridge(deframer.data_in(0))), 0);
        framer.data_return_out.connect(Arc::new(Sink), 0);

        let payload = [0x00u8, 0x01, 0xDE, 0xCA, 0xFB, 0xAD]; // TELEM apid
        let din = framer.data_in(0);
        din.target
            .invoke(din.port_num, buffer_of(&payload), &FrameContext::default());

        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].0, payload);
        assert_eq!(out[0].1.apid, Apid::FwPacketTelem);
        assert!(rec.events.lock().unwrap().is_empty());
    }
}
