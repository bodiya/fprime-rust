//! # TcDeframer — port of `Svc::Ccsds::TcDeframer` (passive, guarded `dataIn`)
//!
//! C++ sources: `Svc/Ccsds/TcDeframer/TcDeframer.{cpp,hpp,fpp}`.
//! Analysis: `docs/cpp-analysis/ccsds.md` ("TcDeframer" + the "TC Transfer
//! Frame" wire format and the gotcha list).
//!
//! Validates one TC transfer frame per `dataIn` invocation and strips its
//! 5-byte primary header and 2-byte Frame Error Control Field in place:
//!
//! ```text
//! [ 5-byte primary header ][ data field ][ 2-byte FECF ]
//! ```
//!
//! Validation order (C++ parity — every drop returns the buffer upstream
//! with the context UNMODIFIED, and the success path forwards the context
//! unmodified too: this component sets neither `vcId` nor `apid`):
//!
//! 1. `size <= 7` → `InvalidPacket` (WARNING_LO). **No `errorNotify` on this
//!    path** — the only rejection that stays silent on that port.
//! 2. spacecraft ID mismatch → `InvalidSpacecraftId` + `TC_INVALID_SCID`.
//! 3. `size < total_frame_length` or `total_frame_length < 7` →
//!    `InvalidFrameLength` + `TC_INVALID_LENGTH`.
//! 4. virtual channel mismatch (unless [`TcDeframer::configure`] accepts all)
//!    → `InvalidVcId` + `TC_INVALID_VCID`.
//! 5. FECF mismatch → `InvalidCrc` + `TC_INVALID_CRC`.
//!
//! F Prime uses Type-BD frames, so there are no FARM checks and the frame
//! sequence number is never examined; the bypass and control flags are not
//! checked either (only `CcsdsTcFrameDetector` matches on them).

use crate::ccsds::crc16::Crc16;
use crate::ccsds::types::{ErrorNotifyPort, FrameError, SPACECRAFT_ID, TCHeader, TCTrailer};
use fprime_comp::{ComDataWithContextPort, EventGlue, OutputPort, PassiveBase, input_port_adapter};
use fprime_config::{FwEventIdType, FwIndexType, FwSignedSizeType, FwSizeType};
use fprime_fw::{
    Buffer, Endianness, FrameContext, LogSeverity, SerBuf, SerializeStatus, fw_assert, fw_try,
};
use std::sync::{Arc, Mutex};

/// `InvalidPacket` event id (FPP declaration order, relative).
pub const EVENTID_INVALID_PACKET: FwEventIdType = 0;
/// `InvalidSpacecraftId(transmitted, configured)` event id.
pub const EVENTID_INVALID_SPACECRAFT_ID: FwEventIdType = 1;
/// `InvalidFrameLength(transmitted, actual)` event id.
pub const EVENTID_INVALID_FRAME_LENGTH: FwEventIdType = 2;
/// `InvalidVcId(transmitted, configured)` event id.
pub const EVENTID_INVALID_VC_ID: FwEventIdType = 3;
/// `InvalidCrc(transmitted, computed)` event id.
pub const EVENTID_INVALID_CRC: FwEventIdType = 4;

/// Smallest structurally valid TC frame: header + trailer.
pub const MIN_TC_FRAME_SIZE: usize = TCHeader::SERIALIZED_SIZE + TCTrailer::SERIALIZED_SIZE;

/// Configuration, guarded by the component mutex (`dataIn` is `guarded`).
struct TcState {
    /// The accepted virtual channel ID.
    ///
    /// C++ leaves `m_vcId` UNINITIALIZED in the constructor, which is safe
    /// only because `m_acceptAllVcid` defaults to true; here it starts at 0.
    vc_id: u16,
    /// The accepted spacecraft ID (default `ComCfg::SpacecraftId`).
    spacecraft_id: u16,
    /// Accept frames on any virtual channel (default `true`).
    accept_all_vcid: bool,
}

/// `Svc::Ccsds::TcDeframer` — passive TC transfer frame deframer.
pub struct TcDeframer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (binary + text) and the time port.
    pub evt: EventGlue,
    /// `dataOut` — the frame's data field downstream (to the Space Packet
    /// deframer).
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — buffer ownership back upstream (to the frame
    /// accumulator).
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `errorNotify` — optional deframing-error notification.
    pub error_notify: OutputPort<dyn ErrorNotifyPort>,
    /// Configuration + the guarded-port mutex.
    state: Mutex<TcState>,
}

impl TcDeframer {
    /// Construct the component with the `ComCfg` spacecraft ID and
    /// all virtual channels accepted.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            error_notify: OutputPort::new(),
            state: Mutex::new(TcState {
                vc_id: 0,
                spacecraft_id: SPACECRAFT_ID,
                accept_all_vcid: true,
            }),
        })
    }

    /// Restrict the deframer to one virtual channel and/or another
    /// spacecraft ID (C++ `configure`).
    pub fn configure(&self, vc_id: u16, spacecraft_id: u16, accept_all_vcid: bool) {
        let mut state = self.state.lock().unwrap();
        state.vc_id = vc_id;
        state.spacecraft_id = spacecraft_id;
        state.accept_all_vcid = accept_all_vcid;
    }

    /// Emit `errorNotify` when the port is connected (C++
    /// `errorNotifyHelper`).
    fn error_notify(&self, error: FrameError) {
        if let Some(p) = self.error_notify.try_get() {
            p.target.invoke(p.port_num, error);
        }
    }

    /// Drop path: return the buffer upstream with the original context.
    fn data_return_out(&self, data: Buffer, context: &FrameContext) {
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    /// `dataIn` handler (guarded — runs under the component mutex).
    fn data_in_handler(&self, _port_num: FwIndexType, mut data: Buffer, context: &FrameContext) {
        let state = self.state.lock().unwrap();

        // (1) Too short to hold a header and a trailer. C++ parity: this is
        // the ONLY rejection that does not raise errorNotify.
        if data.size() <= MIN_TC_FRAME_SIZE {
            drop(state);
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_PACKET,
                LogSeverity::WarningLo,
                "Invalid packet received refusing to deframe",
                |_buf| SerializeStatus::Ok,
            );
            self.data_return_out(data, context);
            return;
        }

        // (2) Primary header. The size check above guarantees the read.
        let mut header = TCHeader::default();
        {
            let mut deser = data.get_deserializer();
            let status = deser.deserialize(&mut header, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
        }
        let total_frame_length = header.total_frame_length();
        let vc_id = u16::from(header.vc_id());
        let spacecraft_id = header.spacecraft_id();
        let configured_scid = state.spacecraft_id;
        let configured_vcid = state.vc_id;
        let accept_all_vcid = state.accept_all_vcid;
        drop(state);

        // (3) Spacecraft ID.
        if spacecraft_id != configured_scid {
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_SPACECRAFT_ID,
                LogSeverity::WarningLo,
                &format!(
                    "Invalid Spacecraft ID Received. Received: {spacecraft_id} | \
                     Deframer configured with: {configured_scid}"
                ),
                |buf| {
                    fw_try!(buf.serialize_u16_be(spacecraft_id));
                    buf.serialize_u16_be(configured_scid)
                },
            );
            self.error_notify(FrameError::TcInvalidScid);
            self.data_return_out(data, context);
            return;
        }

        // (4) The declared length must be structurally valid and no larger
        // than what was actually received.
        if data.size() < usize::from(total_frame_length)
            || usize::from(total_frame_length) < MIN_TC_FRAME_SIZE
        {
            let actual = data.size() as FwSizeType;
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_FRAME_LENGTH,
                LogSeverity::WarningHi,
                &format!(
                    "Not enough data received. Header length specified: \
                     {total_frame_length} | Received data length: {actual}"
                ),
                |buf| {
                    fw_try!(buf.serialize_u16_be(total_frame_length));
                    buf.serialize_u64_be(actual)
                },
            );
            self.error_notify(FrameError::TcInvalidLength);
            self.data_return_out(data, context);
            return;
        }

        // (5) Virtual channel.
        if !accept_all_vcid && vc_id != configured_vcid {
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_VC_ID,
                LogSeverity::ActivityLo,
                &format!(
                    "Invalid Virtual Channel ID Received. Header token specified: \
                     {vc_id} | Deframer configured with: {configured_vcid}"
                ),
                |buf| {
                    fw_try!(buf.serialize_u16_be(vc_id));
                    buf.serialize_u16_be(configured_vcid)
                },
            );
            self.error_notify(FrameError::TcInvalidVcid);
            self.data_return_out(data, context);
            return;
        }

        // (6) Frame Error Control Field over [0, total_frame_length - 2).
        let crc_len = usize::from(total_frame_length) - TCTrailer::SERIALIZED_SIZE;
        let computed_crc = Crc16::compute(&data.data()[..crc_len]);
        let mut trailer = TCTrailer::default();
        {
            let mut deser = data.get_deserializer();
            let status = deser.move_deser_to_offset(crc_len);
            fw_assert!(status.is_ok(), status as i32);
            let status = deser.deserialize(&mut trailer, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
        }
        let transmitted_crc = trailer.fecf;
        if transmitted_crc != computed_crc {
            // C++ GOTCHA reproduced verbatim: the event is DECLARED
            // `InvalidCrc(transmitted, computed)` but `TcDeframer.cpp` calls
            // `log_WARNING_HI_InvalidCrc(computed_crc, transmitted_crc)`, so
            // the first wire argument is the COMPUTED value. Kept for
            // byte-compatibility with the C++ flight software and the ground
            // dictionary it was built against.
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_CRC,
                LogSeverity::WarningHi,
                &format!(
                    "Invalid checksum received. Trailer specified: {computed_crc} | \
                     Computed on board: {transmitted_crc}"
                ),
                |buf| {
                    fw_try!(buf.serialize_u16_be(computed_crc));
                    buf.serialize_u16_be(transmitted_crc)
                },
            );
            self.error_notify(FrameError::TcInvalidCrc);
            self.data_return_out(data, context);
            return;
        }

        // (7) Window the same allocation onto the data field.
        data.advance(TCHeader::SERIALIZED_SIZE as FwSignedSizeType);
        data.set_size(
            usize::from(total_frame_length)
                - TCHeader::SERIALIZED_SIZE
                - TCTrailer::SERIALIZED_SIZE,
        );
        // The context is forwarded UNMODIFIED (no vcId or apid is set).
        let p = self.data_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    /// `dataReturnIn` handler: straight pass-through upstream.
    fn data_return_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        self.data_return_out(data, context);
    }
}

// -- Input-port adapters + factories (generated by the codegen layer) --------

input_port_adapter! {
    /// `dataIn` — GUARDED `Svc.ComDataWithContext` input: one TC frame.
    component: TcDeframer;
    adapter: DataInAdapter;
    port: ComDataWithContextPort;
    input: pub data_in;
    handler: data_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

input_port_adapter! {
    /// `dataReturnIn` — SYNC input: the data field coming back from
    /// downstream; passed through to `dataReturnOut`.
    component: TcDeframer;
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
    use fprime_comp::LogPort;
    use fprime_fw::{Apid, Deserialize, LogBuffer, Time};
    use std::sync::Mutex as StdMutex;

    #[derive(Default)]
    struct Recorder {
        data_out: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        data_return: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        /// events: (id, severity, first two u16 args when present).
        events: StdMutex<Vec<(FwEventIdType, LogSeverity, Vec<u16>)>>,
        errors: StdMutex<Vec<FrameError>>,
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
                let mut value = 0u16;
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

    fn build() -> (Arc<TcDeframer>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let deframer = TcDeframer::new("tcDeframer");
        deframer.data_out.connect(rec.clone(), 0);
        deframer.data_return_out.connect(rec.clone(), 1);
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

    fn feed(deframer: &Arc<TcDeframer>, bytes: &[u8], context: &FrameContext) {
        let p = deframer.data_in(0);
        p.target.invoke(p.port_num, buffer_of(bytes), context);
    }

    /// Build a well-formed TC frame around `payload` (bypass set, control
    /// clear, valid FECF).
    fn tc_frame(spacecraft_id: u16, vc_id: u8, payload: &[u8]) -> Vec<u8> {
        let total = (MIN_TC_FRAME_SIZE + payload.len()) as u16;
        let mut frame = Vec::new();
        frame.extend_from_slice(
            &TCHeader::build_flags_and_sc_id(true, false, spacecraft_id).to_be_bytes(),
        );
        frame.extend_from_slice(&TCHeader::build_vc_id_and_length(vc_id, total).to_be_bytes());
        frame.push(0); // frame sequence number, never checked
        frame.extend_from_slice(payload);
        let crc = Crc16::compute(&frame);
        frame.extend_from_slice(&crc.to_be_bytes());
        frame
    }

    /// A well-formed frame's literal bytes, and the deframed data field.
    #[test]
    fn valid_frame_bytes_and_deframing() {
        let (deframer, rec) = build();
        let frame = tc_frame(SPACECRAFT_ID, 0, &[0x01, 0x02, 0x03]);
        // 0x2044 | vcid 0 + length token 9 | seq 0 | payload | CRC
        assert_eq!(&frame[..5], &[0x20, 0x44, 0x00, 0x09, 0x00]);
        assert_eq!(frame.len(), 10);
        assert_eq!(Crc16::compute(&frame), 0); // FECF residue

        let ctx = FrameContext {
            apid: Apid::FwPacketHand,
            vc_id: 3,
            ..FrameContext::default()
        };
        feed(&deframer, &frame, &ctx);
        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].0, vec![0x01, 0x02, 0x03]);
        // The context is forwarded UNMODIFIED: no vcId, no apid.
        assert_eq!(out[0].1, ctx);
        assert!(rec.events.lock().unwrap().is_empty());
        assert!(rec.errors.lock().unwrap().is_empty());
    }

    /// A frame carrying exactly one data byte is the smallest accepted one.
    #[test]
    fn minimum_length_frame_is_accepted() {
        let (deframer, rec) = build();
        feed(
            &deframer,
            &tc_frame(SPACECRAFT_ID, 0, &[0xAA]),
            &FrameContext::default(),
        );
        assert_eq!(rec.data_out.lock().unwrap()[0].0, vec![0xAA]);
    }

    /// Gotcha: a 7-byte buffer (header + trailer, no data) is rejected as
    /// `InvalidPacket` WARNING_LO — and raises NO errorNotify.
    #[test]
    fn too_short_frame_is_rejected_without_error_notify() {
        let (deframer, rec) = build();
        let ctx = FrameContext {
            vc_id: 2,
            ..FrameContext::default()
        };
        feed(&deframer, &tc_frame(SPACECRAFT_ID, 0, &[]), &ctx);
        assert!(rec.data_out.lock().unwrap().is_empty());
        let events = rec.events.lock().unwrap();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].0, EVENTID_INVALID_PACKET);
        assert_eq!(events[0].1, LogSeverity::WarningLo);
        assert!(rec.errors.lock().unwrap().is_empty()); // the gotcha
        let ret = rec.data_return.lock().unwrap();
        assert_eq!(ret[0].1, ctx);
    }

    /// A wrong spacecraft ID is rejected with both arguments in declaration
    /// order.
    #[test]
    fn wrong_spacecraft_id_is_rejected() {
        let (deframer, rec) = build();
        feed(
            &deframer,
            &tc_frame(0x0123, 0, &[0x01]),
            &FrameContext::default(),
        );
        assert!(rec.data_out.lock().unwrap().is_empty());
        let events = rec.events.lock().unwrap();
        assert_eq!(events[0].0, EVENTID_INVALID_SPACECRAFT_ID);
        assert_eq!(events[0].1, LogSeverity::WarningLo);
        assert_eq!(events[0].2, vec![0x0123, SPACECRAFT_ID]);
        assert_eq!(*rec.errors.lock().unwrap(), vec![FrameError::TcInvalidScid]);
    }

    /// `configure` changes the accepted spacecraft ID.
    #[test]
    fn configure_changes_the_accepted_spacecraft_id() {
        let (deframer, rec) = build();
        deframer.configure(0, 0x0123, true);
        feed(
            &deframer,
            &tc_frame(0x0123, 0, &[0x01]),
            &FrameContext::default(),
        );
        assert_eq!(rec.data_out.lock().unwrap().len(), 1);
        assert!(rec.events.lock().unwrap().is_empty());
    }

    /// A declared length longer than the received data is `InvalidFrameLength`
    /// (WARNING_HI) with args (u16 transmitted, u64 actual).
    #[test]
    fn declared_length_beyond_the_buffer_is_rejected() {
        let (deframer, rec) = build();
        let mut frame = tc_frame(SPACECRAFT_ID, 0, &[0x01, 0x02]);
        // Claim 100 total octets.
        let word = TCHeader::build_vc_id_and_length(0, 100);
        frame[2..4].copy_from_slice(&word.to_be_bytes());
        feed(&deframer, &frame, &FrameContext::default());
        assert!(rec.data_out.lock().unwrap().is_empty());
        let events = rec.events.lock().unwrap();
        assert_eq!(events[0].0, EVENTID_INVALID_FRAME_LENGTH);
        assert_eq!(events[0].1, LogSeverity::WarningHi);
        // [u16 transmitted][u64 actual] decoded as u16 words.
        assert_eq!(events[0].2, vec![100, 0, 0, 0, 9]);
        assert_eq!(
            *rec.errors.lock().unwrap(),
            vec![FrameError::TcInvalidLength]
        );
    }

    /// A structurally impossible length (< header + trailer) is rejected
    /// even when the buffer is big enough.
    #[test]
    fn structurally_short_declared_length_is_rejected() {
        let (deframer, rec) = build();
        let mut frame = tc_frame(SPACECRAFT_ID, 0, &[0x01, 0x02, 0x03, 0x04]);
        let word = TCHeader::build_vc_id_and_length(0, 6);
        frame[2..4].copy_from_slice(&word.to_be_bytes());
        feed(&deframer, &frame, &FrameContext::default());
        assert!(rec.data_out.lock().unwrap().is_empty());
        assert_eq!(
            rec.events.lock().unwrap()[0].0,
            EVENTID_INVALID_FRAME_LENGTH
        );
        assert_eq!(
            *rec.errors.lock().unwrap(),
            vec![FrameError::TcInvalidLength]
        );
    }

    /// A declared length SHORTER than the buffer is accepted, and the data
    /// field is windowed to it (trailing bytes ignored).
    #[test]
    fn declared_length_shorter_than_the_buffer_windows_the_data_field() {
        let (deframer, rec) = build();
        let mut frame = tc_frame(SPACECRAFT_ID, 0, &[0x01, 0x02]);
        frame.extend_from_slice(&[0xFF, 0xFF]); // extra trailing bytes
        feed(&deframer, &frame, &FrameContext::default());
        assert_eq!(rec.data_out.lock().unwrap()[0].0, vec![0x01, 0x02]);
    }

    /// All virtual channels are accepted by default; `configure` restricts
    /// them, and a mismatch is ACTIVITY_LO.
    #[test]
    fn virtual_channel_filtering() {
        let (deframer, rec) = build();
        // Default: accept all.
        feed(
            &deframer,
            &tc_frame(SPACECRAFT_ID, 0x3F, &[0x01]),
            &FrameContext::default(),
        );
        assert_eq!(rec.data_out.lock().unwrap().len(), 1);

        deframer.configure(2, SPACECRAFT_ID, false);
        feed(
            &deframer,
            &tc_frame(SPACECRAFT_ID, 2, &[0x02]),
            &FrameContext::default(),
        );
        assert_eq!(rec.data_out.lock().unwrap().len(), 2);
        feed(
            &deframer,
            &tc_frame(SPACECRAFT_ID, 5, &[0x03]),
            &FrameContext::default(),
        );
        assert_eq!(rec.data_out.lock().unwrap().len(), 2);
        let events = rec.events.lock().unwrap();
        assert_eq!(events[0].0, EVENTID_INVALID_VC_ID);
        assert_eq!(events[0].1, LogSeverity::ActivityLo);
        assert_eq!(events[0].2, vec![5, 2]);
        assert_eq!(*rec.errors.lock().unwrap(), vec![FrameError::TcInvalidVcid]);
    }

    /// A corrupted FECF is rejected — and the event arguments are INVERTED
    /// relative to the FPP declaration, exactly as the C++ call site does.
    #[test]
    fn bad_crc_is_rejected_with_inverted_event_arguments() {
        let (deframer, rec) = build();
        let mut frame = tc_frame(SPACECRAFT_ID, 0, &[0x01, 0x02]);
        let computed = Crc16::compute(&frame[..frame.len() - 2]);
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        let transmitted = u16::from_be_bytes([frame[last - 1], frame[last]]);
        feed(&deframer, &frame, &FrameContext::default());
        assert!(rec.data_out.lock().unwrap().is_empty());
        let events = rec.events.lock().unwrap();
        assert_eq!(events[0].0, EVENTID_INVALID_CRC);
        assert_eq!(events[0].1, LogSeverity::WarningHi);
        // Declared (transmitted, computed) but emitted (computed, transmitted).
        assert_eq!(events[0].2, vec![computed, transmitted]);
        assert_eq!(*rec.errors.lock().unwrap(), vec![FrameError::TcInvalidCrc]);
    }

    /// The bypass and control flags and the frame sequence number are never
    /// examined (F Prime uses Type-BD frames).
    #[test]
    fn flags_and_sequence_number_are_not_checked() {
        let (deframer, rec) = build();
        let payload = [0x01u8, 0x02];
        let total = (MIN_TC_FRAME_SIZE + payload.len()) as u16;
        let mut frame = Vec::new();
        // Bypass CLEAR, control command SET.
        frame.extend_from_slice(
            &TCHeader::build_flags_and_sc_id(false, true, SPACECRAFT_ID).to_be_bytes(),
        );
        frame.extend_from_slice(&TCHeader::build_vc_id_and_length(0, total).to_be_bytes());
        frame.push(0xAB); // arbitrary frame sequence number
        frame.extend_from_slice(&payload);
        let crc = Crc16::compute(&frame);
        frame.extend_from_slice(&crc.to_be_bytes());
        feed(&deframer, &frame, &FrameContext::default());
        assert_eq!(rec.data_out.lock().unwrap()[0].0, payload.to_vec());
        assert!(rec.events.lock().unwrap().is_empty());
    }

    /// `errorNotify` is optional.
    #[test]
    fn error_notify_is_optional() {
        let rec = Arc::new(Recorder::default());
        let deframer = TcDeframer::new("tcDeframer");
        deframer.data_out.connect(rec.clone(), 0);
        deframer.data_return_out.connect(rec.clone(), 1);
        deframer.evt.log_out.connect(rec.clone(), 0);
        feed(
            &deframer,
            &tc_frame(0x0001, 0, &[0x01]),
            &FrameContext::default(),
        );
        assert!(rec.errors.lock().unwrap().is_empty());
        assert_eq!(rec.events.lock().unwrap().len(), 1);
        assert_eq!(rec.data_return.lock().unwrap().len(), 1);
    }

    /// dataReturnIn passes straight through to dataReturnOut.
    #[test]
    fn data_return_in_passthrough() {
        let (deframer, rec) = build();
        let ctx = FrameContext {
            apid: Apid::FwPacketCommand,
            ..FrameContext::default()
        };
        let p = deframer.data_return_in(0);
        p.target.invoke(p.port_num, buffer_of(b"field"), &ctx);
        let ret = rec.data_return.lock().unwrap();
        assert_eq!(ret[0].0, b"field");
        assert_eq!(ret[0].1, ctx);
    }

    // -----------------------------------------------------------------
    // Full-stack round trip
    // -----------------------------------------------------------------

    /// Downlink `SpacePacketFramer` -> `TmFramer`, then uplink
    /// `TcDeframer` -> `SpacePacketDeframer`, recovering the original
    /// payload and the context fields the Space Packet header carries.
    #[test]
    fn ccsds_round_trip_recovers_payload_and_context() {
        use crate::ccsds::apid_manager::ApidManager;
        use crate::ccsds::space_packet_deframer::SpacePacketDeframer;
        use crate::ccsds::space_packet_framer::SpacePacketFramer;
        use crate::ccsds::tm_framer::{TRAILER_OFFSET, TmFramer};
        use crate::ccsds::types::{SpacePacketHeader, TMHeader};
        use fprime_comp::{BufferGetPort, BufferSendPort};

        /// Captures one (bytes, context) pair per port.
        #[derive(Default)]
        struct Capture {
            frames: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        }
        impl ComDataWithContextPort for Capture {
            fn invoke(&self, _p: FwIndexType, data: Buffer, context: &FrameContext) {
                self.frames
                    .lock()
                    .unwrap()
                    .push((data.data().to_vec(), *context));
            }
        }
        impl BufferSendPort for Capture {
            fn invoke(&self, _p: FwIndexType, _b: Buffer) {}
        }
        struct Alloc;
        impl BufferGetPort for Alloc {
            fn invoke(&self, _p: FwIndexType, size: FwSizeType) -> Buffer {
                Buffer::allocate(size as usize)
            }
        }
        /// Bridges one component's output to another's input port.
        struct Bridge(fprime_comp::PortRef<dyn ComDataWithContextPort>);
        impl ComDataWithContextPort for Bridge {
            fn invoke(&self, _p: FwIndexType, data: Buffer, context: &FrameContext) {
                self.0.target.invoke(self.0.port_num, data, context);
            }
        }

        let payload = b"HELLO CCSDS";
        let downlink_ctx = FrameContext {
            apid: Apid::FwPacketTelem,
            has_sec_hdr: true,
            sequence_flags: 0b11,
            vc_id: 1,
            ..FrameContext::default()
        };

        // ---- Downlink: payload -> Space Packet -> TM transfer frame ----
        let apid_mgr = ApidManager::new("apidManager");
        let sp_framer = SpacePacketFramer::new("spacePacketFramer");
        let tm_framer = TmFramer::new("tmFramer");
        let tm_out = Arc::new(Capture::default());
        let sink = Arc::new(Capture::default());
        sp_framer.buffer_allocate.connect(Arc::new(Alloc), 0);
        sp_framer.buffer_deallocate.connect(sink.clone(), 0);
        sp_framer
            .get_apid_seq_count
            .connect_to(apid_mgr.get_apid_seq_count_in(0));
        sp_framer
            .data_out
            .connect(Arc::new(Bridge(tm_framer.data_in(0))), 0);
        sp_framer.data_return_out.connect(sink.clone(), 0);
        tm_framer.data_out.connect(tm_out.clone(), 0);
        tm_framer.data_return_out.connect(sink.clone(), 0);

        // Burn one sequence count so the round trip carries a non-zero one.
        let p = apid_mgr.get_apid_seq_count_in(0);
        assert_eq!(p.target.invoke(p.port_num, Apid::FwPacketTelem, 0), 0);

        let p = sp_framer.data_in(0);
        let mut payload_buffer = Buffer::allocate(payload.len());
        payload_buffer.data_mut().copy_from_slice(payload);
        p.target.invoke(p.port_num, payload_buffer, &downlink_ctx);

        let frames = tm_out.frames.lock().unwrap();
        assert_eq!(frames.len(), 1);
        let tm_frame = frames[0].0.clone();
        drop(frames);
        assert_eq!(tm_frame.len(), 1024);
        assert_eq!(Crc16::compute(&tm_frame), 0);

        // The Space Packet sits at the TM data field origin.
        let space_packet_len = SpacePacketHeader::SERIALIZED_SIZE + payload.len();
        let space_packet =
            &tm_frame[TMHeader::SERIALIZED_SIZE..TMHeader::SERIALIZED_SIZE + space_packet_len];
        // Everything after it up to the trailer is the idle packet.
        assert_eq!(
            tm_frame[TMHeader::SERIALIZED_SIZE + space_packet_len
                ..TMHeader::SERIALIZED_SIZE + space_packet_len + 2],
            [0x07, 0xFF]
        );
        assert!(
            tm_frame[TMHeader::SERIALIZED_SIZE + space_packet_len + 6..TRAILER_OFFSET]
                .iter()
                .all(|&b| b == 0x44)
        );

        // ---- Uplink: wrap the same Space Packet in a TC frame ----
        let tc = tc_frame(SPACECRAFT_ID, 1, space_packet);

        let uplink_mgr = ApidManager::new("uplinkApidManager");
        let tc_deframer = TcDeframer::new("tcDeframer");
        let sp_deframer = SpacePacketDeframer::new("spacePacketDeframer");
        let recovered = Arc::new(Capture::default());
        tc_deframer
            .data_out
            .connect(Arc::new(Bridge(sp_deframer.data_in(0))), 0);
        tc_deframer.data_return_out.connect(sink.clone(), 0);
        sp_deframer.data_out.connect(recovered.clone(), 0);
        sp_deframer.data_return_out.connect(sink.clone(), 0);
        sp_deframer
            .validate_apid_seq_count
            .connect_to(uplink_mgr.validate_apid_seq_count_in(0));

        let p = tc_deframer.data_in(0);
        p.target
            .invoke(p.port_num, buffer_of(&tc), &FrameContext::default());

        let out = recovered.frames.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].0, payload);
        // The context fields the Space Packet header carries survive the
        // round trip.
        assert_eq!(out[0].1.apid, downlink_ctx.apid);
        assert_eq!(out[0].1.has_sec_hdr, downlink_ctx.has_sec_hdr);
        assert_eq!(out[0].1.sequence_flags, downlink_ctx.sequence_flags);
        assert_eq!(out[0].1.sequence_count, 1);
    }
}
