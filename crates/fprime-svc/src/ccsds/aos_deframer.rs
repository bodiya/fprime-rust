//! # AosDeframer — port of `Svc::Ccsds::AosDeframer` (passive, guarded `dataIn`)
//!
//! C++ sources: `Svc/Ccsds/AosDeframer/AosDeframer.{cpp,hpp,fpp}`, the
//! `AosDeframerEvents.fppi` / `AosDeframerTelem.fppi` includes and its
//! `docs/sdd.md` (CCSDS 732.0-B-5; SPP per 133.0-B-2, EPP per 133.1-B-3).
//!
//! Receives fixed-size AOS transfer frames and extracts the packets in the
//! M_PDU packet zone. Validation order per frame (every rejection returns
//! the frame on `dataReturnOut` with the ORIGINAL context):
//!
//! 1. `size < fixedFrameSize` → `InvalidFrameLength` + `AOS_INVALID_LENGTH`.
//! 2. FECF (when enabled) → `InvalidFecf` + `AOS_INVALID_CRC`, `CrcErrorCount`.
//! 3. TFVN != `0b01` → `InvalidTfvn` + `AOS_INVALID_VERSION`.
//! 4. SCID (8 LSBs + 2 MSBs) → `InvalidSpacecraftId` + `AOS_INVALID_SCID`.
//! 5. VCID not accepted → `InvalidVcId` + `AOS_INVALID_VCID`.
//! 6. VC frame count (extended by the 4-bit cycle when its flag is set)
//!    discontinuous after the first frame → `VcFrameCountGap` +
//!    `AOS_VC_FRAME_COUNT_GAP`, abandoning any spanning packet; the frame is
//!    still processed.
//!
//! The First Header Pointer then locates packets: `0xFFFE` is an idle-only
//! frame, `0xFFFF` continues the spanning packet, an out-of-zone FHP is
//! `InvalidFhp`, and bytes before the FHP finish (or abandon) the spanning
//! packet. Every packet — even one wholly inside a frame — is copied into a
//! buffer from `allocate` and emitted on `dataOut` with the context's `vcId`
//! and `pvn` set; `dataReturnIn` deallocates it. SPP and EPP idle packets
//! end extraction for the frame, as does a disabled PVN (`DisabledPvn`).
//!
//! Event and telemetry ids are the FPP declaration order of the `.fppi`
//! includes. `errorNotify` is invoked only when connected.
//!
//! ## Deviations (documented)
//!
//! - `dataIn` is guarded: the component state is a `Mutex` held for the
//!   whole handler, including its port calls (the C++ guarded-port mutex).
//!   `dataReturnIn` is sync and touches no state.
//! - The spanning packet's `Fw::Buffer` handle is an `Option<Buffer>`
//!   (`Some` = C++ `isValid()`); an invalid allocation is held as `None`.
//! - `FwSizeType` is 64-bit here, so the EPP length overflow guard can never
//!   fire; a ~4 GB EPP length reaches the allocator, which refuses it, as
//!   on upstream's 64-bit hosts.

use crate::ccsds::crc16::Crc16;
use crate::ccsds::types::{
    AOS_MAX_FRAME_FIXED_SIZE, AOSHeader, AOSTrailer, EppLengthOfLength, EppProtocolId,
    ErrorNotifyPort, FrameError, MPduHeader, SPACECRAFT_ID, SpacePacketHeader, Tfvn,
    aos_header_subfields, epp_subfields, m_pdu_subfields, pvn_bitfield, space_packet_subfields,
};
use fprime_comp::{
    BufferGetPort, BufferSendPort, ComDataWithContextPort, EventGlue, OutputPort, PassiveBase,
    TlmGlue, input_port_adapter,
};
use fprime_config::{FwChanIdType, FwEventIdType, FwIndexType, FwSizeType};
use fprime_fw::{
    Apid, Buffer, Deserialize, Endianness, ExtBuf, FrameContext, LogSeverity, Pvn, SerBuf,
    SerializeStatus, fw_assert, fw_try,
};
use std::sync::{Arc, Mutex};

/// Number of accepted virtual channels (`AosDeframer.NumVcs`).
pub const NUM_VCS: usize = 1;

/// Max header bytes needed to size a packet (the 8-byte EPP header).
pub const HEADER_BUF_SIZE: usize = 8;

/// `InvalidSpacecraftId(transmitted: U16, configured: U16)` — WARNING_LO.
pub const EVENTID_INVALID_SPACECRAFT_ID: FwEventIdType = 0;
/// `InvalidFrameLength(actual: FwSizeType, expected: U32)` — WARNING_HI.
pub const EVENTID_INVALID_FRAME_LENGTH: FwEventIdType = 1;
/// `InvalidVcId(transmitted: U8, configured: U8)` — ACTIVITY_LO.
pub const EVENTID_INVALID_VC_ID: FwEventIdType = 2;
/// `InvalidFecf(transmitted: U16, computed: U16)` — WARNING_HI.
pub const EVENTID_INVALID_FECF: FwEventIdType = 3;
/// `InvalidTfvn(transmitted: U8, expected: U8)` — WARNING_HI.
pub const EVENTID_INVALID_TFVN: FwEventIdType = 4;
/// `DisabledPvn(vcId: U8, pvn: U8)` — WARNING_HI.
pub const EVENTID_DISABLED_PVN: FwEventIdType = 5;
/// `IdleFrame(vcId: U8)` — ACTIVITY_LO.
pub const EVENTID_IDLE_FRAME: FwEventIdType = 6;
/// `SpanningPacketAllocFailed(vcId: U8, pvn: ComCfg.Pvn, packetSize:
/// FwSizeType)` — WARNING_HI.
pub const EVENTID_SPANNING_PACKET_ALLOC_FAILED: FwEventIdType = 7;
/// `VcFrameCountGap(vcId: U8, received: U32, expected: U32)` — WARNING_HI.
pub const EVENTID_VC_FRAME_COUNT_GAP: FwEventIdType = 8;
/// `SpanningPacketAbandoned(vcId: U8, pvn: ComCfg.Pvn, bytesReceived:
/// FwSizeType, bytesExpected: FwSizeType)` — WARNING_HI.
pub const EVENTID_SPANNING_PACKET_ABANDONED: FwEventIdType = 9;
/// `InvalidFhp(vcId: U8, fhpReceived: FwSizeType, dataZoneSize:
/// FwSizeType)` — WARNING_HI.
pub const EVENTID_INVALID_FHP: FwEventIdType = 10;

/// `LatestVcFrameCount: U32`.
pub const CHANID_LATEST_VC_FRAME_COUNT: FwChanIdType = 0;
/// `FramesProcessed: U32`.
pub const CHANID_FRAMES_PROCESSED: FwChanIdType = 1;
/// `PacketsExtracted: U32`.
pub const CHANID_PACKETS_EXTRACTED: FwChanIdType = 2;
/// `CrcErrorCount: U32`.
pub const CHANID_CRC_ERROR_COUNT: FwChanIdType = 3;

/// Spanning packet state (CCSDS 732.0-B-5 4.1.4.2.2.3).
#[derive(Debug, Default)]
struct SpanningPacketState {
    /// Header bytes accumulated before allocation.
    header_buf: [u8; HEADER_BUF_SIZE],
    /// The allocated packet buffer (`Some` = valid).
    buffer: Option<Buffer>,
    /// Bytes received so far.
    bytes_received: usize,
    /// Context sent with the packet.
    context: FrameContext,
}

/// Per-virtual-channel state (`AosDeframer::AosDeframerVc`).
#[derive(Debug)]
struct AosDeframerVc {
    virtual_channel_id: u8,
    pvn_mask: u8,
    frames_processed: u32,
    packets_extracted: u32,
    vc_frame_count: u32,
    spanning_packet: SpanningPacketState,
}

/// Deframer state (the C++ members), guarded by the `dataIn` mutex.
struct DeframerState {
    fixed_frame_size: u32,
    fecf_enabled: bool,
    spacecraft_id: u16,
    crc_error_count: u32,
    vcs: [AosDeframerVc; NUM_VCS],
}

/// `Svc::Ccsds::AosDeframer` — passive AOS (M_PDU) deframer.
pub struct AosDeframer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports and the time port.
    pub evt: EventGlue,
    /// Telemetry port.
    pub tlm: TlmGlue,
    /// `dataOut` — extracted packets.
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — frame buffers back to their sender.
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `errorNotify` — optional deframing-error notification.
    pub error_notify: OutputPort<dyn ErrorNotifyPort>,
    /// `allocate` — packet buffers (`Svc.BufferAllocation`).
    pub allocate: OutputPort<dyn BufferGetPort>,
    /// `deallocate` — packet buffers back to the allocator.
    pub deallocate: OutputPort<dyn BufferSendPort>,
    state: Mutex<DeframerState>,
}

impl AosDeframer {
    /// Construct the component with the C++ constructor defaults (maximum
    /// frame size, FECF on, `ComCfg::SpacecraftId`, VCID 0, SPP + EPP).
    /// [`configure`](Self::configure) before processing frames.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            error_notify: OutputPort::new(),
            allocate: OutputPort::new(),
            deallocate: OutputPort::new(),
            state: Mutex::new(DeframerState {
                fixed_frame_size: AOS_MAX_FRAME_FIXED_SIZE as u32,
                fecf_enabled: true,
                spacecraft_id: SPACECRAFT_ID,
                crc_error_count: 0,
                vcs: core::array::from_fn(|_| AosDeframerVc {
                    virtual_channel_id: 0,
                    pvn_mask: pvn_bitfield::SPP_MASK | pvn_bitfield::EPP_MASK,
                    frames_processed: 0,
                    packets_extracted: 0,
                    vc_frame_count: 0,
                    spanning_packet: SpanningPacketState::default(),
                }),
            }),
        })
    }

    /// C++ `configure(fixedFrameSize, frameErrorControlField, spacecraftId,
    /// vcId, pvnMask)`; the C++ defaults are `ComCfg::SpacecraftId`, VCID 0
    /// and SPP + EPP. Must be called before frames are processed, after the
    /// `allocate`/`deallocate` ports are connected (asserted). Clears the
    /// FECF error counter, the VC statistics and any spanning packet.
    pub fn configure(
        &self,
        fixed_frame_size: u32,
        frame_error_control_field: bool,
        spacecraft_id: u16,
        vc_id: u8,
        pvn_mask: u8,
    ) {
        fw_assert!(
            fixed_frame_size as usize <= AOS_MAX_FRAME_FIXED_SIZE,
            fixed_frame_size as i32,
            AOS_MAX_FRAME_FIXED_SIZE as i32
        );
        let min_size = AOSHeader::SERIALIZED_SIZE
            + MPduHeader::SERIALIZED_SIZE
            + if frame_error_control_field {
                AOSTrailer::SERIALIZED_SIZE
            } else {
                0
            };
        fw_assert!(
            fixed_frame_size as usize > min_size,
            fixed_frame_size as i32,
            min_size as i32
        );
        fw_assert!(spacecraft_id & 0xFC00 == 0, i32::from(spacecraft_id));
        fw_assert!(vc_id & 0xC0 == 0, i32::from(vc_id));
        fw_assert!(
            pvn_mask & pvn_bitfield::VALID_MASK != 0,
            i32::from(pvn_mask)
        );
        fw_assert!(
            pvn_mask & !pvn_bitfield::VALID_MASK == 0,
            i32::from(pvn_mask)
        );
        // Spanning packet reassembly requires the allocator ports.
        fw_assert!(self.allocate.is_connected());
        fw_assert!(self.deallocate.is_connected());

        let mut st = self.state.lock().unwrap();
        st.fixed_frame_size = fixed_frame_size;
        st.fecf_enabled = frame_error_control_field;
        st.spacecraft_id = spacecraft_id;
        st.crc_error_count = 0;
        st.vcs[0].virtual_channel_id = vc_id;
        st.vcs[0].pvn_mask = pvn_mask;
        for index in 0..NUM_VCS {
            st.vcs[index].frames_processed = 0;
            st.vcs[index].packets_extracted = 0;
            st.vcs[index].vc_frame_count = 0;
            self.abandon_spanning_packet(&mut st.vcs[index]);
        }
    }

    // -- Handlers ---------------------------------------------------------------

    /// `dataIn` (guarded): the VC_RECEIVE.indication service primitive.
    fn data_in_handler(&self, _port_num: FwIndexType, mut data: Buffer, context: &FrameContext) {
        let mut st = self.state.lock().unwrap();
        fw_assert!(st.fixed_frame_size > 0, st.fixed_frame_size as i32);

        if data.size() < st.fixed_frame_size as usize {
            let actual = data.size() as FwSizeType;
            let expected = st.fixed_frame_size;
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_FRAME_LENGTH,
                LogSeverity::WarningHi,
                &format!("Frame length mismatch. Received: {actual} | Expected: {expected} "),
                |buf| {
                    fw_try!(buf.serialize_u64_be(actual));
                    buf.serialize_u32_be(expected)
                },
            );
            self.notify_error_if_connected(FrameError::AosInvalidLength);
            self.data_return_out(data, context);
            return;
        }

        // Validate the FECF (4.1.6); whole frames are enforced upstream.
        if st.fecf_enabled && !self.validate_fecf(&mut st, &mut data) {
            self.data_return_out(data, context);
            return;
        }

        let mut packet_context = *context;
        let Some(vc_index) =
            self.parse_and_validate_header(&mut st, &mut data, &mut packet_context)
        else {
            self.data_return_out(data, context);
            return;
        };

        // Keep the PVN tracker of an in-progress spanning packet.
        if st.vcs[vc_index].spanning_packet.buffer.is_none() {
            st.vcs[vc_index].spanning_packet.context = packet_context;
        }

        self.extract_packets(&mut st, vc_index, &data);

        self.data_return_out(data, context);

        let vc = &mut st.vcs[vc_index];
        vc.frames_processed = vc.frames_processed.wrapping_add(1);
        let frames = vc.frames_processed;
        self.tlm_write(CHANID_FRAMES_PROCESSED, frames);
    }

    /// `dataReturnIn` (sync): deallocate an emitted packet buffer.
    fn data_return_in_handler(
        &self,
        _port_num: FwIndexType,
        data: Buffer,
        _context: &FrameContext,
    ) {
        let p = self.deallocate.get();
        p.target.invoke(p.port_num, data);
    }

    // -- Helpers ------------------------------------------------------------------

    fn data_return_out(&self, data: Buffer, context: &FrameContext) {
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    fn notify_error_if_connected(&self, error: FrameError) {
        if let Some(p) = self.error_notify.try_get() {
            p.target.invoke(p.port_num, error);
        }
    }

    fn tlm_write(&self, id: FwChanIdType, value: u32) {
        self.tlm
            .tlm_write(self.base.get_id_base(), id, &value, self.evt.time_get());
    }

    /// Abandon an in-progress spanning packet, deallocating its buffer.
    fn abandon_spanning_packet(&self, vc: &mut AosDeframerVc) {
        if let Some(buffer) = vc.spanning_packet.buffer.take() {
            let vc_id = vc.virtual_channel_id;
            let pvn = vc.spanning_packet.context.pvn;
            let received = vc.spanning_packet.bytes_received as FwSizeType;
            let expected = buffer.size() as FwSizeType;
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_SPANNING_PACKET_ABANDONED,
                LogSeverity::WarningHi,
                &format!(
                    "Spanning packet on VC {vc_id} ({pvn:?}) abandoned after receiving \
                     {received} of {expected} expected bytes"
                ),
                |buf| {
                    fw_try!(buf.serialize_u8_be(vc_id));
                    fw_try!(buf.serialize(&pvn, Endianness::Big));
                    fw_try!(buf.serialize_u64_be(received));
                    buf.serialize_u64_be(expected)
                },
            );
            let p = self.deallocate.get();
            p.target.invoke(p.port_num, buffer);
        }
        vc.spanning_packet.bytes_received = 0;
        vc.spanning_packet.context.pvn = Pvn::InvalidUninitialized;
    }

    /// The VC struct accepting `vc_id`, if any.
    fn get_vc_struct(st: &DeframerState, vc_id: u8) -> Option<usize> {
        st.vcs.iter().position(|vc| vc.virtual_channel_id == vc_id)
    }

    /// Parse and validate the primary header (4.1.2), emitting the warning
    /// events and `errorNotify` for header failures.
    fn parse_and_validate_header(
        &self,
        st: &mut DeframerState,
        data: &mut Buffer,
        context: &mut FrameContext,
    ) -> Option<usize> {
        let mut header = AOSHeader::default();
        {
            let mut de = data.get_deserializer();
            let status = header.deserialize_from(&mut de, Endianness::Big);
            // A header fits the configured size, which the frame reached.
            fw_assert!(status.is_ok(), status as i32);
        }

        // Transfer Frame Version Number (4.1.2.2.2).
        let tfvn = ((header.global_vc_id & aos_header_subfields::FRAME_VERSION_MASK)
            >> aos_header_subfields::FRAME_VERSION_OFFSET) as u8;
        let aos = Tfvn::Aos.as_repr();
        if tfvn != aos {
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_TFVN,
                LogSeverity::WarningHi,
                &format!(
                    "Invalid CCSDS Transfer Frame Version Number. Received: {tfvn} | Expected: {aos}"
                ),
                |buf| {
                    fw_try!(buf.serialize_u8_be(tfvn));
                    buf.serialize_u8_be(aos)
                },
            );
            self.notify_error_if_connected(FrameError::AosInvalidVersion);
            return None;
        }

        // Spacecraft ID (4.1.2.2): 8 LSBs in globalVcId, 2 MSBs in signaling.
        let spacecraft_id = ((header.global_vc_id & aos_header_subfields::SPACECRAFT_ID_LSB_MASK)
            >> aos_header_subfields::SPACECRAFT_ID_LSB_OFFSET)
            | (((header.frame_count_and_signaling & aos_header_subfields::SPACECRAFT_ID_MSB_MASK)
                << (8 - aos_header_subfields::SPACECRAFT_ID_MSB_OFFSET)) as u16);
        if spacecraft_id != st.spacecraft_id {
            let configured = st.spacecraft_id;
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_SPACECRAFT_ID,
                LogSeverity::WarningLo,
                &format!(
                    "Invalid Spacecraft ID Received. Received: {spacecraft_id} | Deframer \
                     configured with: {configured}"
                ),
                |buf| {
                    fw_try!(buf.serialize_u16_be(spacecraft_id));
                    buf.serialize_u16_be(configured)
                },
            );
            self.notify_error_if_connected(FrameError::AosInvalidScid);
            return None;
        }

        // Virtual Channel ID (4.1.2.3).
        let vc_id = (header.global_vc_id & aos_header_subfields::VIRTUAL_CHANNEL_ID_MASK) as u8;
        let Some(vc_index) = Self::get_vc_struct(st, vc_id) else {
            let accepted = st.vcs[0].virtual_channel_id;
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_VC_ID,
                LogSeverity::ActivityLo,
                &format!(
                    "Invalid Virtual Channel ID. Frame contained: {vc_id} | Accepted VCIDs: {accepted}"
                ),
                |buf| {
                    fw_try!(buf.serialize_u8_be(vc_id));
                    buf.serialize_u8_be(accepted)
                },
            );
            self.notify_error_if_connected(FrameError::AosInvalidVcid);
            return None;
        };

        // Virtual Channel Frame Count (4.1.2.4), 24 bits.
        let signaling = header.frame_count_and_signaling;
        let mut rx_vc_frame_count = (signaling & aos_header_subfields::VC_FRAME_COUNT_MASK)
            >> aos_header_subfields::VC_FRAME_COUNT_OFFSET;
        let mut frame_count_mask: u32 = 0x00FF_FFFF;
        // Extended by the 4-bit cycle when in use (4.1.2.5.3).
        if signaling & aos_header_subfields::CYCLE_COUNT_FLAG_MASK != 0 {
            let cycle = signaling & aos_header_subfields::VC_FRAME_COUNT_CYCLE_MASK;
            rx_vc_frame_count |= cycle << 24;
            frame_count_mask |= 0x0F00_0000;
        }

        // Gap detection after the first accepted frame on a VC.
        if st.vcs[vc_index].frames_processed > 0 {
            let expected = st.vcs[vc_index].vc_frame_count.wrapping_add(1);
            if rx_vc_frame_count != (expected & frame_count_mask) {
                self.evt.log_event(
                    self.base.get_id_base(),
                    EVENTID_VC_FRAME_COUNT_GAP,
                    LogSeverity::WarningHi,
                    &format!(
                        "VC frame count gap on VC {vc_id}. Received {rx_vc_frame_count} | \
                         Expected next count {expected}"
                    ),
                    |buf| {
                        fw_try!(buf.serialize_u8_be(vc_id));
                        fw_try!(buf.serialize_u32_be(rx_vc_frame_count));
                        buf.serialize_u32_be(expected)
                    },
                );
                self.notify_error_if_connected(FrameError::AosVcFrameCountGap);
                // Other errors drop the spanning packet implicitly once a
                // valid frame is locked onto again.
                self.abandon_spanning_packet(&mut st.vcs[vc_index]);
            }
        }

        st.vcs[vc_index].vc_frame_count = rx_vc_frame_count;
        self.tlm_write(CHANID_LATEST_VC_FRAME_COUNT, rx_vc_frame_count);

        context.vc_id = vc_id;
        Some(vc_index)
    }

    /// Validate the Frame Error Control Field (4.1.6); counts failures per
    /// physical channel.
    fn validate_fecf(&self, st: &mut DeframerState, data: &mut Buffer) -> bool {
        let crc_data_len = st.fixed_frame_size as usize - AOSTrailer::SERIALIZED_SIZE;
        let computed = Crc16::compute(&data.data()[..crc_data_len]);
        let mut trailer = AOSTrailer::default();
        {
            let mut de = data.get_deserializer();
            let status = de.move_deser_to_offset(crc_data_len);
            fw_assert!(status.is_ok(), status as i32);
            let status = trailer.deserialize_from(&mut de, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
        }
        let transmitted = trailer.fecf;
        if transmitted != computed {
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_FECF,
                LogSeverity::WarningHi,
                &format!(
                    "Invalid FECF (CRC) received. Trailer specified: {transmitted} | Computed \
                     on board: {computed}"
                ),
                |buf| {
                    fw_try!(buf.serialize_u16_be(transmitted));
                    buf.serialize_u16_be(computed)
                },
            );
            self.notify_error_if_connected(FrameError::AosInvalidCrc);
            st.crc_error_count = st.crc_error_count.wrapping_add(1);
            let count = st.crc_error_count;
            self.tlm_write(CHANID_CRC_ERROR_COUNT, count);
            return false;
        }
        true
    }

    /// Append bytes to the spanning packet, sizing and allocating it once
    /// its header is known and emitting it when complete. Returns how far
    /// the caller should seek in the frame, or 0 when done with the frame.
    fn append_to_spanning_packet(&self, vc: &mut AosDeframerVc, input: &[u8]) -> usize {
        fw_assert!(!input.is_empty(), input.len() as i32);
        // `pos` is the C++ `data` pointer, relative to the input.
        let mut pos = 0usize;
        let mut seek_forward = 0usize;

        if vc.spanning_packet.buffer.is_none() {
            // Keep packing the header buffer until the size is known.
            let received = vc.spanning_packet.bytes_received;
            let to_header = input.len().min(HEADER_BUF_SIZE - received);
            if to_header > 0 {
                fw_assert!(received < HEADER_BUF_SIZE, received as i32);
                vc.spanning_packet.header_buf[received..received + to_header]
                    .copy_from_slice(&input[..to_header]);
                vc.spanning_packet.bytes_received += to_header;
                pos += to_header;
                seek_forward += to_header;
            }

            let header = vc.spanning_packet.header_buf;
            let header_len = vc.spanning_packet.bytes_received;
            let packet_size = self.size_packet(vc, &header[..header_len]);
            if packet_size == 0 {
                return 0;
            }

            // A packet may declare fewer bytes than the header buffer holds:
            // give the surplus back as the next packet.
            if (packet_size as usize) < vc.spanning_packet.bytes_received {
                let surplus = vc.spanning_packet.bytes_received - packet_size as usize;
                fw_assert!(surplus <= to_header, surplus as i32, to_header as i32);
                vc.spanning_packet.bytes_received = packet_size as usize;
                pos -= surplus;
                seek_forward -= surplus;
            }

            // Allocate the whole packet; an invalid or undersized buffer is
            // handled below.
            let p = self.allocate.get();
            let buffer = p.target.invoke(p.port_num, packet_size);
            let ok = buffer.is_valid() && buffer.size() as FwSizeType >= packet_size;
            vc.spanning_packet.buffer = if buffer.is_valid() {
                Some(buffer)
            } else {
                None
            };
            if !ok {
                let vc_id = vc.virtual_channel_id;
                let pvn = vc.spanning_packet.context.pvn;
                self.evt.log_event(
                    self.base.get_id_base(),
                    EVENTID_SPANNING_PACKET_ALLOC_FAILED,
                    LogSeverity::WarningHi,
                    &format!(
                        "Spanning packet allocation failed on VC {vc_id} for {pvn:?} packet of \
                         {packet_size} bytes; packet dropped"
                    ),
                    |buf| {
                        fw_try!(buf.serialize_u8_be(vc_id));
                        fw_try!(buf.serialize(&pvn, Endianness::Big));
                        buf.serialize_u64_be(packet_size)
                    },
                );
                // Saved before abandon clears it: the seek past the packet.
                let remaining_body = packet_size - vc.spanning_packet.bytes_received as FwSizeType;
                self.abandon_spanning_packet(vc);
                let remaining_length = seek_forward as FwSizeType + remaining_body;
                return if remaining_body > (input.len() - pos) as FwSizeType {
                    0
                } else {
                    remaining_length as usize
                };
            }

            // Load the header into the packet buffer.
            let received = vc.spanning_packet.bytes_received;
            fw_assert!(received <= HEADER_BUF_SIZE, received as i32);
            if let Some(buffer) = vc.spanning_packet.buffer.as_mut() {
                fw_assert!(
                    received <= buffer.size(),
                    received as i32,
                    buffer.size() as i32
                );
                buffer.data_mut()[..received].copy_from_slice(&header[..received]);
            }
        }

        // Fill the packet buffer.
        let received = vc.spanning_packet.bytes_received;
        let buffer_size = vc.spanning_packet.buffer.as_ref().map_or(0, Buffer::size);
        let to_body = (input.len() - pos).min(buffer_size - received);
        if to_body > 0 {
            if let Some(buffer) = vc.spanning_packet.buffer.as_mut() {
                buffer.data_mut()[received..received + to_body]
                    .copy_from_slice(&input[pos..pos + to_body]);
            }
            vc.spanning_packet.bytes_received += to_body;
            seek_forward += to_body;
        }

        // Emit the packet once complete.
        if buffer_size > 0 && vc.spanning_packet.bytes_received >= buffer_size {
            if let Some(buffer) = vc.spanning_packet.buffer.take() {
                let context = vc.spanning_packet.context;
                let p = self.data_out.get();
                p.target.invoke(p.port_num, buffer, &context);
            }
            vc.packets_extracted = vc.packets_extracted.wrapping_add(1);
            let count = vc.packets_extracted;
            self.tlm_write(CHANID_PACKETS_EXTRACTED, count);
            // Ownership moved downstream; the handle is cleared, so this
            // only resets the state.
            self.abandon_spanning_packet(vc);
        }

        seek_forward
    }

    /// Parse the M_PDU header and extract packets (4.1.4.2).
    fn extract_packets(&self, st: &mut DeframerState, vc_index: usize, data: &Buffer) {
        let frame = data.data();
        let fhp = u16::from_be_bytes([
            frame[AOSHeader::SERIALIZED_SIZE],
            frame[AOSHeader::SERIALIZED_SIZE + 1],
        ]);
        let zone_start = AOSHeader::SERIALIZED_SIZE + MPduHeader::SERIALIZED_SIZE;
        let zone_end = st.fixed_frame_size as usize
            - if st.fecf_enabled {
                AOSTrailer::SERIALIZED_SIZE
            } else {
                0
            };
        let zone = &frame[zone_start..zone_end];
        let zone_size = zone.len();
        let vc = &mut st.vcs[vc_index];

        if fhp == m_pdu_subfields::FHP_IDLE_DATA_ONLY {
            let vc_id = vc.virtual_channel_id;
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_IDLE_FRAME,
                LogSeverity::ActivityLo,
                &format!("Received frame containing only idle data on VC {vc_id}"),
                |buf| buf.serialize_u8_be(vc_id),
            );
            return;
        } else if fhp == m_pdu_subfields::FHP_NO_PACKET_START {
            // The whole zone continues the previous packet; without one the
            // continuation is unusable.
            if vc.spanning_packet.bytes_received > 0 {
                let _ = self.append_to_spanning_packet(vc, zone);
            }
            return;
        }

        // Untrusted input: the FHP must point inside the zone.
        if usize::from(fhp) >= zone_size {
            let vc_id = vc.virtual_channel_id;
            let fhp_received = FwSizeType::from(fhp);
            let zone_len = zone_size as FwSizeType;
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INVALID_FHP,
                LogSeverity::WarningHi,
                &format!(
                    "Invalid First Header Pointer (FHP) received on VC {vc_id}: FHP was \
                     {fhp_received} bytes, data zone is only {zone_len} bytes"
                ),
                |buf| {
                    fw_try!(buf.serialize_u8_be(vc_id));
                    fw_try!(buf.serialize_u64_be(fhp_received));
                    buf.serialize_u64_be(zone_len)
                },
            );
            self.notify_error_if_connected(FrameError::AosInvalidLength);
            self.abandon_spanning_packet(vc);
            return;
        }

        // Continuation bytes before the first header finish the previous
        // packet, which must end here.
        if fhp > 0 && vc.spanning_packet.bytes_received > 0 {
            let _ = self.append_to_spanning_packet(vc, &zone[..usize::from(fhp)]);
            self.abandon_spanning_packet(vc);
        }

        let mut current = usize::from(fhp);
        // Bound: a run of 1-byte EPP idle packets.
        let max_iters = zone_size - usize::from(fhp);
        let mut iter = 0;
        while iter < max_iters && current < zone_size {
            self.abandon_spanning_packet(vc);
            let packet_size = self.append_to_spanning_packet(vc, &zone[current..]);
            if packet_size == 0 {
                return;
            }
            current += packet_size;
            iter += 1;
        }
    }

    /// Validity, size and idle status of the packet starting at `packet`
    /// (`sizePacket`): its total size, or 0 when unknown yet, idle, or of a
    /// disabled PVN. Sets the spanning context's PVN.
    fn size_packet(&self, vc: &mut AosDeframerVc, packet: &[u8]) -> FwSizeType {
        fw_assert!(!packet.is_empty(), packet.len() as i32);
        let pvn = get_packet_version(packet[0]);
        vc.spanning_packet.context.pvn = Pvn::InvalidUninitialized;
        if !vc.pvn_mask & (1u8 << pvn) != 0 {
            let vc_id = vc.virtual_channel_id;
            // C++ parity: the FPP format fills its placeholders in argument
            // order, so the text reads "version {vcId} ... VC {pvn}".
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_DISABLED_PVN,
                LogSeverity::WarningHi,
                &format!(
                    "Packet version number {vc_id} encountered on VC {pvn} but it is invalid or \
                     disabled by configuration"
                ),
                |buf| {
                    fw_try!(buf.serialize_u8_be(vc_id));
                    buf.serialize_u8_be(pvn)
                },
            );
            return 0;
        }
        let pvn_enum = match Pvn::try_from(pvn) {
            Ok(p) => p,
            Err(_) => {
                // The mask only admits PVN 0 and 7.
                fw_assert!(false, i32::from(pvn));
                return 0;
            }
        };
        vc.spanning_packet.context.pvn = pvn_enum;
        match pvn_enum {
            Pvn::SpacePacketProtocol => size_spp_packet(packet),
            Pvn::EncapsulationPacketProtocol => size_epp_packet(packet),
            Pvn::InvalidUninitialized => {
                fw_assert!(false, i32::from(pvn));
                0
            }
        }
    }
}

/// The PVN: the upper 3 bits of the first byte (SPP and EPP alike).
fn get_packet_version(first_byte: u8) -> u8 {
    first_byte >> epp_subfields::PACKET_VERSION_OFFSET
}

/// Size a Space Packet (133.0-B-2): `6 + length token + 1`, or 0 when the
/// header is incomplete or the packet is an idle packet (APID `0x7FF`,
/// which ends the frame).
fn size_spp_packet(payload: &[u8]) -> FwSizeType {
    if payload.len() < SpacePacketHeader::SERIALIZED_SIZE {
        return 0;
    }
    let mut header = SpacePacketHeader::default();
    let mut bytes = [0u8; SpacePacketHeader::SERIALIZED_SIZE];
    bytes.copy_from_slice(&payload[..SpacePacketHeader::SERIALIZED_SIZE]);
    let mut de = ExtBuf::with_len(&mut bytes, SpacePacketHeader::SERIALIZED_SIZE);
    if header.deserialize_from(&mut de, Endianness::Big) != SerializeStatus::Ok {
        return 0;
    }
    let total = SpacePacketHeader::SERIALIZED_SIZE as FwSizeType
        + FwSizeType::from(header.packet_data_length)
        + 1;
    let apid = header.packet_identification & space_packet_subfields::APID_MASK;
    if apid == Apid::SppIdlePacket.as_repr() {
        return 0;
    }
    total
}

/// Size an Encapsulation Packet (133.1-B-3 4.1.2): header length from the
/// length-of-length code plus the big-endian length field; 0 when
/// incomplete or an idle packet (protocol id 0).
fn size_epp_packet(payload: &[u8]) -> FwSizeType {
    fw_assert!(!payload.is_empty(), payload.len() as i32);
    let first = payload[0];
    let protocol_id =
        (first & epp_subfields::PROTOCOL_ID_MASK) >> epp_subfields::PROTOCOL_ID_OFFSET;
    if protocol_id == EppProtocolId::Idle.as_repr() {
        return 0;
    }
    let mut length_of_length = first & epp_subfields::LENGTH_OF_LENGTH_MASK;
    let mut length_offset = 1usize;
    // Length of length >= 2 adds a user-defined/extension byte.
    if length_of_length >= EppLengthOfLength::Two.as_repr() {
        length_offset += 1;
    }
    // Length of length "4" (0b11) adds two CCSDS-reserved bytes.
    if length_of_length == EppLengthOfLength::Four.as_repr() {
        length_offset += 2;
        length_of_length = 4;
    }
    let header_length = length_offset + usize::from(length_of_length);
    if payload.len() < header_length {
        return 0;
    }
    let mut packet_data_length: u32 = 0;
    for i in 0..usize::from(length_of_length) {
        packet_data_length = (packet_data_length << 8) | u32::from(payload[length_offset + i]);
    }
    // C++ overflow guard (dead with a 64-bit FwSizeType).
    if FwSizeType::from(packet_data_length) > FwSizeType::MAX - header_length as FwSizeType {
        return 0;
    }
    header_length as FwSizeType + FwSizeType::from(packet_data_length)
}

input_port_adapter! {
    /// `dataIn` — GUARDED `Svc.ComDataWithContext`: one AOS frame.
    component: AosDeframer;
    adapter: DataInAdapter;
    port: ComDataWithContextPort;
    input: pub data_in;
    handler: data_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

input_port_adapter! {
    /// `dataReturnIn` — SYNC: an emitted packet buffer coming back.
    component: AosDeframer;
    adapter: DataReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub data_return_in;
    handler: data_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

// ---------------------------------------------------------------------------
// Tests — port of AosDeframerTester (+ AosDeframerTestSupport.cpp)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{LogPort, TlmPort};
    use fprime_fw::{LogBuffer, Time, TlmBuffer};
    use std::sync::Mutex as StdMutex;

    const TEST_FRAME_SIZE: u32 = 256;
    const ALLOC_BUF_SIZE: FwSizeType = 65536;
    const TEST_DATA_ZONE_SIZE: usize = TEST_FRAME_SIZE as usize
        - AOSHeader::SERIALIZED_SIZE
        - MPduHeader::SERIALIZED_SIZE
        - AOSTrailer::SERIALIZED_SIZE;

    #[derive(Default)]
    struct Harness {
        data_out: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        data_return: StdMutex<Vec<FrameContext>>,
        errors: StdMutex<Vec<FrameError>>,
        /// (relative id, raw args)
        events: StdMutex<Vec<(FwEventIdType, Vec<u8>)>>,
        /// (relative id, value)
        tlm: StdMutex<Vec<(FwChanIdType, u32)>>,
        deallocated: StdMutex<Vec<usize>>,
        fail_next_alloc: StdMutex<bool>,
    }

    impl ComDataWithContextPort for Harness {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            if port_num == 0 {
                self.data_out
                    .lock()
                    .unwrap()
                    .push((data.data().to_vec(), *context));
            } else {
                self.data_return.lock().unwrap().push(*context);
            }
        }
    }

    impl ErrorNotifyPort for Harness {
        fn invoke(&self, _port_num: FwIndexType, error_code: FrameError) {
            self.errors.lock().unwrap().push(error_code);
        }
    }

    impl LogPort for Harness {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.events
                .lock()
                .unwrap()
                .push((id, args.as_slice().to_vec()));
        }
    }

    impl TlmPort for Harness {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            _time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            let mut v = 0u32;
            assert!(val.deserialize_u32_be(&mut v).is_ok());
            self.tlm.lock().unwrap().push((id, v));
        }
    }

    impl BufferGetPort for Harness {
        fn invoke(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
            let mut fail = self.fail_next_alloc.lock().unwrap();
            if *fail {
                *fail = false;
                return Buffer::empty();
            }
            if size <= ALLOC_BUF_SIZE {
                Buffer::allocate(size as usize)
            } else {
                Buffer::empty()
            }
        }
    }

    impl BufferSendPort for Harness {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.deallocated.lock().unwrap().push(buffer.size());
        }
    }

    impl Harness {
        fn clear(&self) {
            self.data_out.lock().unwrap().clear();
            self.data_return.lock().unwrap().clear();
            self.errors.lock().unwrap().clear();
            self.events.lock().unwrap().clear();
            self.tlm.lock().unwrap().clear();
            self.deallocated.lock().unwrap().clear();
        }
        fn n_out(&self) -> usize {
            self.data_out.lock().unwrap().len()
        }
        fn n_ret(&self) -> usize {
            self.data_return.lock().unwrap().len()
        }
        fn out(&self, i: usize) -> (Vec<u8>, FrameContext) {
            self.data_out.lock().unwrap()[i].clone()
        }
        fn errors(&self) -> Vec<FrameError> {
            self.errors.lock().unwrap().clone()
        }
        fn events_of(&self, id: FwEventIdType) -> Vec<Vec<u8>> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .filter(|(i, _)| *i == id)
                .map(|(_, a)| a.clone())
                .collect()
        }
        fn n_events(&self) -> usize {
            self.events.lock().unwrap().len()
        }
        fn tlm_of(&self, id: FwChanIdType) -> Vec<u32> {
            self.tlm
                .lock()
                .unwrap()
                .iter()
                .filter(|(i, _)| *i == id)
                .map(|(_, v)| *v)
                .collect()
        }
        fn assert_data_out_vc_id(&self, vc_id: u8) {
            for (_, ctx) in self.data_out.lock().unwrap().iter() {
                assert_eq!(ctx.vc_id, vc_id);
            }
        }
    }

    fn build() -> (Arc<AosDeframer>, Arc<Harness>) {
        let h = Arc::new(Harness::default());
        let comp = AosDeframer::new("AosDeframer");
        comp.data_out.connect(h.clone(), 0);
        comp.data_return_out.connect(h.clone(), 1);
        comp.error_notify.connect(h.clone(), 0);
        comp.allocate.connect(h.clone(), 0);
        comp.deallocate.connect(h.clone(), 0);
        comp.evt.log_out.connect(h.clone(), 0);
        comp.tlm.tlm_out.connect(h.clone(), 0);
        (comp, h)
    }

    fn configure_default(comp: &AosDeframer) {
        comp.configure(
            TEST_FRAME_SIZE,
            true,
            SPACECRAFT_ID,
            0,
            pvn_bitfield::SPP_MASK | pvn_bitfield::EPP_MASK,
        );
    }

    /// `assembleFrameBuffer`: a frame around `payload` with an EPP fill byte
    /// after it.
    #[allow(clippy::too_many_arguments)]
    fn assemble(
        payload: &[u8],
        fhp: u16,
        scid: u16,
        vcid: u8,
        vc_count: u32,
        tfvn: u8,
        include_fecf: bool,
    ) -> Vec<u8> {
        let frame_size = if include_fecf {
            TEST_FRAME_SIZE as usize
        } else {
            TEST_FRAME_SIZE as usize - AOSTrailer::SERIALIZED_SIZE
        };
        let mut f = vec![0u8; frame_size];
        let global_vc_id =
            ((u16::from(tfvn) & 0x3) << 14) | ((scid & 0xFF) << 6) | u16::from(vcid & 0x3F);
        f[0..2].copy_from_slice(&global_vc_id.to_be_bytes());
        f[2] = (vc_count >> 16) as u8;
        f[3] = (vc_count >> 8) as u8;
        f[4] = vc_count as u8;
        f[5] = (1 << 6) | ((((scid >> 8) & 0x3) as u8) << 4) | (((vc_count >> 24) & 0x0F) as u8);
        f[6..8].copy_from_slice(&fhp.to_be_bytes());
        let zone_start = 8;
        let zone_end = frame_size - if include_fecf { 2 } else { 0 };
        let copy = payload.len().min(zone_end - zone_start);
        f[zone_start..zone_start + copy].copy_from_slice(&payload[..copy]);
        if zone_start + copy < zone_end {
            let mut fill = Vec::new();
            create_epp(
                &mut fill,
                EppProtocolId::Idle.as_repr(),
                EppLengthOfLength::Zero,
                0,
            );
            f[zone_start + copy] = fill[0];
        }
        if include_fecf {
            let crc = Crc16::compute(&f[..frame_size - 2]);
            f[frame_size - 2..].copy_from_slice(&crc.to_be_bytes());
        }
        f
    }

    fn frame(payload: &[u8], fhp: u16) -> Vec<u8> {
        assemble(payload, fhp, SPACECRAFT_ID, 0, 0, 1, true)
    }

    fn frame_n(payload: &[u8], fhp: u16, vcid: u8, vc_count: u32) -> Vec<u8> {
        assemble(payload, fhp, SPACECRAFT_ID, vcid, vc_count, 1, true)
    }

    /// `createSppPacket`: header + `data_length` counting bytes.
    fn create_spp(out: &mut Vec<u8>, apid: u16, data_length: u16, seq_count: u16) -> usize {
        let ident = apid & space_packet_subfields::APID_MASK;
        let seq = (0x3 << 14) | (seq_count & 0x3FFF);
        out.extend_from_slice(&ident.to_be_bytes());
        out.extend_from_slice(&seq.to_be_bytes());
        out.extend_from_slice(&(data_length - 1).to_be_bytes());
        for i in 0..data_length {
            out.push(i as u8);
        }
        SpacePacketHeader::SERIALIZED_SIZE + usize::from(data_length)
    }

    fn spp(apid: u16, data_length: u16) -> Vec<u8> {
        let mut v = Vec::new();
        create_spp(&mut v, apid, data_length, 0);
        v
    }

    /// `createEppPacket`.
    fn create_epp(
        out: &mut Vec<u8>,
        protocol_id: u8,
        lol: EppLengthOfLength,
        data_length: usize,
    ) -> usize {
        let start = out.len();
        out.push(
            (7 << epp_subfields::PACKET_VERSION_OFFSET)
                | ((protocol_id << epp_subfields::PROTOCOL_ID_OFFSET)
                    & epp_subfields::PROTOCOL_ID_MASK)
                | (lol.as_repr() & epp_subfields::LENGTH_OF_LENGTH_MASK),
        );
        if lol == EppLengthOfLength::Zero {
            return 1;
        }
        let mut n = lol.as_repr() as usize;
        if lol.as_repr() >= EppLengthOfLength::Two.as_repr() {
            out.push(0);
        }
        if lol == EppLengthOfLength::Four {
            out.push(0);
            out.push(0);
            n = 4;
        }
        for i in 0..n {
            out.push((data_length >> (8 * (n - i - 1))) as u8);
        }
        out.extend(std::iter::repeat_n(0x55, data_length));
        out.len() - start
    }

    fn epp(protocol_id: u8, lol: EppLengthOfLength, data_length: usize) -> Vec<u8> {
        let mut v = Vec::new();
        create_epp(&mut v, protocol_id, lol, data_length);
        v
    }

    fn feed(comp: &Arc<AosDeframer>, bytes: &[u8]) {
        let mut b = Buffer::allocate(bytes.len());
        b.data_mut().copy_from_slice(bytes);
        let p = comp.data_in(0);
        p.target.invoke(p.port_num, b, &FrameContext::default());
    }

    // -- Basic validation -----------------------------------------------------------

    #[test]
    fn nominal_deframing() {
        let (comp, h) = build();
        let vc = 7;
        comp.configure(
            TEST_FRAME_SIZE,
            true,
            SPACECRAFT_ID,
            vc,
            pvn_bitfield::VALID_MASK,
        );
        let packet = spp(0x001, 50);
        feed(&comp, &frame_n(&packet, 0, vc, 0));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(vc);
        assert_eq!(h.n_ret(), 1);
        let (bytes, ctx) = h.out(0);
        assert_eq!(bytes, packet);
        assert_eq!(ctx.pvn, Pvn::SpacePacketProtocol);
        assert_eq!(h.tlm.lock().unwrap().len(), 3);
        assert_eq!(h.tlm_of(CHANID_FRAMES_PROCESSED), vec![1]);
        assert_eq!(h.tlm_of(CHANID_PACKETS_EXTRACTED), vec![1]);
        assert_eq!(h.tlm_of(CHANID_LATEST_VC_FRAME_COUNT), vec![0]);
    }

    #[test]
    fn data_return_deallocates() {
        let (comp, h) = build();
        configure_default(&comp);
        let p = comp.data_return_in(0);
        p.target
            .invoke(p.port_num, Buffer::allocate(1), &FrameContext::default());
        assert_eq!(*h.deallocated.lock().unwrap(), vec![1]);
    }

    #[test]
    fn invalid_scid() {
        let (comp, h) = build();
        configure_default(&comp);
        let wrong = (SPACECRAFT_ID + 1) & 0x3FF;
        feed(&comp, &assemble(&spp(1, 20), 0, wrong, 0, 0, 1, true));
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.errors(), vec![FrameError::AosInvalidScid]);
        assert_eq!(h.n_events(), 1);
        assert_eq!(
            h.events_of(EVENTID_INVALID_SPACECRAFT_ID),
            vec![vec![0x00, 0x45, 0x00, 0x44]]
        );
    }

    #[test]
    fn invalid_vcid() {
        let (comp, h) = build();
        comp.configure(
            TEST_FRAME_SIZE,
            true,
            SPACECRAFT_ID,
            0,
            pvn_bitfield::VALID_MASK,
        );
        feed(&comp, &frame_n(&spp(1, 20), 0, 1, 0));
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.errors(), vec![FrameError::AosInvalidVcid]);
        assert_eq!(h.n_events(), 1);
        assert_eq!(h.events_of(EVENTID_INVALID_VC_ID), vec![vec![1, 0]]);
    }

    #[test]
    fn invalid_frame_length() {
        let (comp, h) = build();
        configure_default(&comp);
        feed(&comp, &[0u8; 50]);
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.errors(), vec![FrameError::AosInvalidLength]);
        assert_eq!(h.n_events(), 1);
        let mut args = 50u64.to_be_bytes().to_vec();
        args.extend_from_slice(&TEST_FRAME_SIZE.to_be_bytes());
        assert_eq!(h.events_of(EVENTID_INVALID_FRAME_LENGTH), vec![args]);
    }

    #[test]
    fn invalid_fecf() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut f = frame(&spp(1, 20), 0);
        f[TEST_FRAME_SIZE as usize - 1] ^= 0xFF;
        feed(&comp, &f);
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.errors(), vec![FrameError::AosInvalidCrc]);
        assert_eq!(h.n_events(), 1);
        assert_eq!(h.events_of(EVENTID_INVALID_FECF).len(), 1);
        assert_eq!(h.tlm_of(CHANID_CRC_ERROR_COUNT), vec![1]);
        h.clear();
        feed(&comp, &f);
        assert_eq!(h.tlm_of(CHANID_CRC_ERROR_COUNT), vec![2]);
    }

    #[test]
    fn invalid_tfvn() {
        let (comp, h) = build();
        configure_default(&comp);
        feed(
            &comp,
            &assemble(&spp(1, 20), 0, SPACECRAFT_ID, 0, 0, 0, true),
        );
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.errors(), vec![FrameError::AosInvalidVersion]);
        assert_eq!(h.n_events(), 1);
        assert_eq!(h.events_of(EVENTID_INVALID_TFVN), vec![vec![0, 1]]);
    }

    #[test]
    fn vc_frame_count_gap() {
        let (comp, h) = build();
        configure_default(&comp);
        let packet = spp(1, 20);
        feed(&comp, &frame_n(&packet, 0, 0, 0));
        assert!(h.errors().is_empty());
        assert!(h.events_of(EVENTID_VC_FRAME_COUNT_GAP).is_empty());
        h.clear();
        feed(&comp, &frame_n(&packet, 0, 0, 2));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.errors(), vec![FrameError::AosVcFrameCountGap]);
        assert_eq!(
            h.events_of(EVENTID_VC_FRAME_COUNT_GAP),
            vec![vec![0, 0, 0, 0, 2, 0, 0, 0, 1]]
        );
        assert_eq!(h.tlm_of(CHANID_LATEST_VC_FRAME_COUNT), vec![2]);
    }

    // -- M_PDU processing -------------------------------------------------------------

    #[test]
    fn fhp_at_offset() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload = vec![0xAA; 30];
        let size = create_spp(&mut payload, 0x003, 50, 0);
        feed(&comp, &frame(&payload, 30));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.out(0).0.len(), size);
    }

    #[test]
    fn fhp_no_packet_start() {
        let (comp, h) = build();
        configure_default(&comp);
        // Orphan continuation: silently dropped.
        feed(
            &comp,
            &frame(
                &[0xAA; TEST_DATA_ZONE_SIZE],
                m_pdu_subfields::FHP_NO_PACKET_START,
            ),
        );
        assert_eq!(h.n_out(), 0);
        assert!(h.errors().is_empty());
        h.clear();
        // A packet spanning two frames.
        let packet = spp(0x004, 250);
        feed(&comp, &frame_n(&packet[..TEST_DATA_ZONE_SIZE], 0, 0, 1));
        assert_eq!(h.n_out(), 0);
        h.clear();
        feed(
            &comp,
            &frame_n(
                &packet[TEST_DATA_ZONE_SIZE..],
                m_pdu_subfields::FHP_NO_PACKET_START,
                0,
                2,
            ),
        );
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.out(0).0, packet);
    }

    #[test]
    fn fhp_idle_data_only() {
        let (comp, h) = build();
        configure_default(&comp);
        feed(
            &comp,
            &frame(&[0x55; 100], m_pdu_subfields::FHP_IDLE_DATA_ONLY),
        );
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.n_events(), 1);
        assert_eq!(h.events_of(EVENTID_IDLE_FRAME), vec![vec![0]]);
        assert_eq!(h.tlm_of(CHANID_FRAMES_PROCESSED), vec![1]);
    }

    #[test]
    fn multiple_packets_in_frame() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload = Vec::new();
        create_spp(&mut payload, 0x010, 20, 0);
        create_spp(&mut payload, 0x011, 25, 0);
        create_spp(&mut payload, 0x012, 30, 0);
        feed(&comp, &frame(&payload, 0));
        assert_eq!(h.n_out(), 3);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.tlm_of(CHANID_PACKETS_EXTRACTED), vec![1, 2, 3]);
        // Every packet is a separate allocated copy, byte for byte.
        assert_eq!(h.out(0).0, payload[..26].to_vec());
        assert_eq!(h.out(2).0, payload[26 + 31..].to_vec());
    }

    #[test]
    fn minimum_size_spp_packets() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload = Vec::new();
        let first = create_spp(&mut payload, 0x030, 1, 0);
        let second = create_spp(&mut payload, 0x031, 10, 0);
        feed(&comp, &frame(&payload, 0));
        assert_eq!(h.n_out(), 2);
        assert_eq!(h.out(0).0.len(), first);
        assert_eq!(h.out(1).0.len(), second);
    }

    // -- Spanning packets -------------------------------------------------------------

    #[test]
    fn spanning_packet_two_frames() {
        let (comp, h) = build();
        configure_default(&comp);
        let packet = spp(0x020, (TEST_DATA_ZONE_SIZE + 50) as u16);
        feed(&comp, &frame(&packet[..TEST_DATA_ZONE_SIZE], 0));
        assert_eq!(h.n_out(), 0);
        h.clear();
        let remaining = packet.len() - TEST_DATA_ZONE_SIZE;
        let mut payload2 = packet[TEST_DATA_ZONE_SIZE..].to_vec();
        create_spp(&mut payload2, 0x021, 20, 0);
        feed(&comp, &frame_n(&payload2, remaining as u16, 0, 1));
        assert_eq!(h.n_out(), 2);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.out(0).0, packet);
    }

    #[test]
    fn spanning_packet_four_frames() {
        let (comp, h) = build();
        configure_default(&comp);
        let packet = spp(0x031, 900);
        let z = TEST_DATA_ZONE_SIZE;
        assert!(packet.len() > 3 * z && packet.len() <= 4 * z);
        feed(&comp, &frame_n(&packet[..z], 0, 0, 0));
        feed(
            &comp,
            &frame_n(
                &packet[z..2 * z],
                m_pdu_subfields::FHP_NO_PACKET_START,
                0,
                1,
            ),
        );
        feed(
            &comp,
            &frame_n(
                &packet[2 * z..3 * z],
                m_pdu_subfields::FHP_NO_PACKET_START,
                0,
                2,
            ),
        );
        assert_eq!(h.n_out(), 0);
        let remaining = packet.len() - 3 * z;
        feed(&comp, &frame_n(&packet[3 * z..], remaining as u16, 0, 3));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.out(0).0, packet);
    }

    #[test]
    fn spanning_packet_continuation() {
        let (comp, h) = build();
        configure_default(&comp);
        let packet = spp(0x040, 280);
        feed(&comp, &frame(&packet[..TEST_DATA_ZONE_SIZE], 0));
        assert_eq!(h.n_out(), 0);
        h.clear();
        let continuation = packet.len() - TEST_DATA_ZONE_SIZE;
        let mut payload2 = packet[TEST_DATA_ZONE_SIZE..].to_vec();
        create_spp(&mut payload2, 0x041, 30, 0);
        feed(&comp, &frame_n(&payload2, continuation as u16, 0, 1));
        assert_eq!(h.n_out(), 2);
        h.assert_data_out_vc_id(0);
    }

    #[test]
    fn spanning_packet_alloc_failure_event() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload = [0u8; 64];
        payload[0] = (7 << epp_subfields::PACKET_VERSION_OFFSET)
            | (EppProtocolId::MissionSpecific.as_repr() << epp_subfields::PROTOCOL_ID_OFFSET)
            | 0x02;
        payload[2] = 0xFF;
        payload[3] = 0xFF; // 65535 + 4 > the 65536-byte allocator
        feed(&comp, &frame(&payload, 0));
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        let events = h.events_of(EVENTID_SPANNING_PACKET_ALLOC_FAILED);
        assert_eq!(events.len(), 1);
        // vcId 0, pvn EPP (7), size 65539.
        let mut args = vec![0, 7];
        args.extend_from_slice(&65539u64.to_be_bytes());
        assert_eq!(events[0], args);
    }

    #[test]
    fn spanning_packet_abandoned_on_vc_gap() {
        let (comp, h) = build();
        configure_default(&comp);
        let packet = spp(0x050, 280);
        feed(&comp, &frame_n(&packet[..TEST_DATA_ZONE_SIZE], 0, 0, 0));
        assert_eq!(h.n_out(), 0);
        h.clear();
        let fresh = spp(0x051, 20);
        feed(&comp, &frame_n(&fresh, 0, 0, 2));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.out(0).0.len(), fresh.len());
        assert_eq!(h.events_of(EVENTID_VC_FRAME_COUNT_GAP).len(), 1);
        let abandoned = h.events_of(EVENTID_SPANNING_PACKET_ABANDONED);
        let mut args = vec![0, 0];
        args.extend_from_slice(&(TEST_DATA_ZONE_SIZE as u64).to_be_bytes());
        args.extend_from_slice(&286u64.to_be_bytes());
        assert_eq!(abandoned, vec![args]);
        // The partial packet's buffer went back to the allocator.
        assert_eq!(*h.deallocated.lock().unwrap(), vec![286]);
    }

    #[test]
    fn spanning_packet_maintained_on_idle_frame() {
        let (comp, h) = build();
        configure_default(&comp);
        let packet = spp(0x060, 280);
        feed(&comp, &frame_n(&packet[..TEST_DATA_ZONE_SIZE], 0, 0, 0));
        h.clear();
        feed(
            &comp,
            &frame_n(&[0], m_pdu_subfields::FHP_IDLE_DATA_ONLY, 0, 1),
        );
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.events_of(EVENTID_IDLE_FRAME), vec![vec![0]]);
        assert!(h.events_of(EVENTID_SPANNING_PACKET_ABANDONED).is_empty());
        // The packet still completes afterwards.
        feed(
            &comp,
            &frame_n(
                &packet[TEST_DATA_ZONE_SIZE..],
                m_pdu_subfields::FHP_NO_PACKET_START,
                0,
                2,
            ),
        );
        assert_eq!(h.out(0).0, packet);
    }

    #[test]
    fn spanning_packet_abandoned_on_premature_fhp() {
        let (comp, h) = build();
        configure_default(&comp);
        let packet_a = spp(0x070, 394);
        feed(&comp, &frame_n(&packet_a[..TEST_DATA_ZONE_SIZE], 0, 0, 0));
        assert_eq!(h.n_out(), 0);
        h.clear();
        let fhp = 50;
        let mut payload2 = packet_a[TEST_DATA_ZONE_SIZE..TEST_DATA_ZONE_SIZE + fhp].to_vec();
        let size_b = create_spp(&mut payload2, 0x071, 20, 0);
        feed(&comp, &frame_n(&payload2, fhp as u16, 0, 1));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.out(0).0.len(), size_b);
        let mut args = vec![0, 0];
        args.extend_from_slice(&((TEST_DATA_ZONE_SIZE + fhp) as u64).to_be_bytes());
        args.extend_from_slice(&400u64.to_be_bytes());
        assert_eq!(h.events_of(EVENTID_SPANNING_PACKET_ABANDONED), vec![args]);
    }

    #[test]
    fn spp_header_spans_frame() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload1 = Vec::new();
        let first = create_spp(&mut payload1, 0x072, 237, 0);
        let second = spp(0x073, 20);
        let split = TEST_DATA_ZONE_SIZE - first;
        assert_eq!(split, 3);
        payload1.extend_from_slice(&second[..split]);
        feed(&comp, &frame_n(&payload1, 0, 0, 0));
        assert_eq!(h.n_out(), 1);
        assert_eq!(h.out(0).0.len(), first);
        h.clear();
        feed(
            &comp,
            &frame_n(&second[split..], m_pdu_subfields::FHP_NO_PACKET_START, 0, 1),
        );
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.out(0).0, second);
    }

    #[test]
    fn epp_header_spans_frame() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload1 = Vec::new();
        let first = create_spp(&mut payload1, 0x074, 238, 0);
        let epp_packet = epp(
            EppProtocolId::MissionSpecific.as_repr(),
            EppLengthOfLength::Two,
            20,
        );
        let split = TEST_DATA_ZONE_SIZE - first;
        assert_eq!(split, 2);
        payload1.extend_from_slice(&epp_packet[..split]);
        feed(&comp, &frame_n(&payload1, 0, 0, 0));
        assert_eq!(h.n_out(), 1);
        assert_eq!(h.out(0).0.len(), first);
        h.clear();
        feed(
            &comp,
            &frame_n(
                &epp_packet[split..],
                m_pdu_subfields::FHP_NO_PACKET_START,
                0,
                1,
            ),
        );
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        let (bytes, ctx) = h.out(0);
        assert_eq!(bytes, epp_packet);
        assert_eq!(ctx.pvn, Pvn::EncapsulationPacketProtocol);
    }

    #[test]
    fn alloc_failure_next_packet_extracted() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload = Vec::new();
        create_spp(&mut payload, 0x080, 20, 0);
        let size_b = create_spp(&mut payload, 0x081, 20, 0);
        *h.fail_next_alloc.lock().unwrap() = true;
        feed(&comp, &frame(&payload, 0));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.out(0).0.len(), size_b);
        assert_eq!(h.events_of(EVENTID_SPANNING_PACKET_ALLOC_FAILED).len(), 1);
    }

    // -- SPP / EPP extraction ---------------------------------------------------------

    #[test]
    fn spp_idle_packet_filtering() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload = Vec::new();
        let s0 = create_spp(&mut payload, 0x101, 20, 0);
        let s1 = create_spp(&mut payload, 0x102, 25, 0);
        create_spp(&mut payload, Apid::SppIdlePacket.as_repr(), 30, 0);
        feed(&comp, &frame(&payload, 0));
        assert_eq!(h.n_out(), 2);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.out(0).0.len(), s0);
        assert_eq!(h.out(0).1.pvn, Pvn::SpacePacketProtocol);
        assert_eq!(h.out(1).0.len(), s1);
        assert_eq!(h.out(1).1.pvn, Pvn::SpacePacketProtocol);
    }

    #[test]
    fn epp_extraction() {
        let (comp, h) = build();
        let vc = 11;
        comp.configure(
            TEST_FRAME_SIZE,
            true,
            SPACECRAFT_ID,
            vc,
            pvn_bitfield::VALID_MASK,
        );
        let packet = epp(
            EppProtocolId::MissionSpecific.as_repr(),
            EppLengthOfLength::One,
            50,
        );
        feed(&comp, &frame_n(&packet, 0, vc, 0));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(vc);
        assert_eq!(h.out(0).0, packet);
        assert_eq!(h.out(0).1.pvn, Pvn::EncapsulationPacketProtocol);
    }

    #[test]
    fn epp_length_of_length() {
        let (comp, h) = build();
        let vc = 11;
        comp.configure(
            TEST_FRAME_SIZE,
            true,
            SPACECRAFT_ID,
            vc,
            pvn_bitfield::VALID_MASK,
        );
        let ms = EppProtocolId::MissionSpecific.as_repr();
        let p1 = epp(ms, EppLengthOfLength::One, 11);
        assert_eq!(p1.len(), 13);
        let p2 = epp(ms, EppLengthOfLength::Two, 277);
        assert_eq!(p2.len(), 281);
        let p4 = epp(ms, EppLengthOfLength::Four, 1367);
        assert_eq!(p4.len(), 1375);
        let z = TEST_DATA_ZONE_SIZE;
        let nps = m_pdu_subfields::FHP_NO_PACKET_START;
        let mut count = 0;
        let mut next = || {
            count += 1;
            count - 1
        };

        feed(&comp, &frame_n(&p1, 0, vc, next()));
        assert_eq!(h.n_out(), 1);
        assert_eq!(h.out(0).0, p1);
        h.clear();

        feed(&comp, &frame_n(&p2[..z], 0, vc, next()));
        assert_eq!(h.n_out(), 0);
        feed(&comp, &frame_n(&p2[z..], nps, vc, next()));
        assert_eq!(h.n_out(), 1);
        assert_eq!(h.out(0).0, p2);
        h.clear();

        feed(&comp, &frame_n(&p4[..z], 0, vc, next()));
        assert_eq!(h.n_out(), 0);
        for i in 1..=4 {
            feed(&comp, &frame_n(&p4[i * z..(i + 1) * z], nps, vc, next()));
            assert_eq!(h.n_out(), 0);
        }
        feed(&comp, &frame_n(&p4[5 * z..], nps, vc, next()));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(vc);
        assert_eq!(h.out(0).0, p4);
    }

    #[test]
    fn epp_idle_packet() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload = Vec::new();
        create_spp(&mut payload, 0x104, 20, 0);
        create_spp(&mut payload, 0x105, 15, 0);
        create_epp(
            &mut payload,
            EppProtocolId::Idle.as_repr(),
            EppLengthOfLength::Two,
            10,
        );
        feed(&comp, &frame(&payload, 0));
        assert_eq!(h.n_out(), 2);
        h.assert_data_out_vc_id(0);
    }

    #[test]
    fn epp_fill_packet() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload = Vec::new();
        create_spp(&mut payload, 0x106, 30, 0);
        create_epp(
            &mut payload,
            EppProtocolId::Idle.as_repr(),
            EppLengthOfLength::Zero,
            0,
        );
        payload.resize(100, 0x55);
        feed(&comp, &frame(&payload, 0));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
    }

    #[test]
    fn invalid_pvn_version() {
        let (comp, h) = build();
        configure_default(&comp);
        let mut payload = vec![0x60 | 0x02, 0x00, 0x10];
        payload.extend_from_slice(&[0xAA; 16]);
        feed(&comp, &frame(&payload, 0));
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.events_of(EVENTID_DISABLED_PVN), vec![vec![0, 3]]);
        assert_eq!(h.tlm_of(CHANID_FRAMES_PROCESSED), vec![1]);
    }

    // -- Configuration ----------------------------------------------------------------

    #[test]
    fn fecf_disabled() {
        let (comp, h) = build();
        comp.configure(
            TEST_FRAME_SIZE - AOSTrailer::SERIALIZED_SIZE as u32,
            false,
            SPACECRAFT_ID,
            0,
            pvn_bitfield::VALID_MASK,
        );
        feed(
            &comp,
            &assemble(&spp(0x200, 50), 0, SPACECRAFT_ID, 0, 0, 1, false),
        );
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert!(h.events_of(EVENTID_INVALID_FECF).is_empty());
    }

    #[test]
    fn pvn_mask_spp_only() {
        let (comp, h) = build();
        comp.configure(
            TEST_FRAME_SIZE,
            true,
            SPACECRAFT_ID,
            0,
            pvn_bitfield::SPP_MASK,
        );
        let mut payload = Vec::new();
        create_spp(&mut payload, 0x201, 20, 0);
        create_epp(&mut payload, 0x02, EppLengthOfLength::One, 20);
        create_spp(&mut payload, 0x202, 15, 0);
        feed(&comp, &frame(&payload, 0));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.events_of(EVENTID_DISABLED_PVN), vec![vec![0, 7]]);
    }

    #[test]
    fn pvn_mask_epp_only() {
        let (comp, h) = build();
        comp.configure(
            TEST_FRAME_SIZE,
            true,
            SPACECRAFT_ID,
            0,
            pvn_bitfield::EPP_MASK,
        );
        let packet = epp(
            EppProtocolId::MissionSpecific.as_repr(),
            EppLengthOfLength::One,
            30,
        );
        feed(&comp, &frame(&packet, 0));
        assert_eq!(h.n_out(), 1);
        h.assert_data_out_vc_id(0);
        assert_eq!(h.out(0).1.pvn, Pvn::EncapsulationPacketProtocol);
    }

    // -- Telemetry --------------------------------------------------------------------

    #[test]
    fn frame_count_telemetry() {
        let (comp, h) = build();
        let vc = 5;
        comp.configure(
            TEST_FRAME_SIZE,
            true,
            SPACECRAFT_ID,
            vc,
            pvn_bitfield::VALID_MASK,
        );
        let packet = spp(0x300, 20);
        for i in 0..3u32 {
            h.clear();
            feed(&comp, &frame_n(&packet, 0, vc, i));
            h.assert_data_out_vc_id(vc);
            assert_eq!(h.tlm_of(CHANID_FRAMES_PROCESSED), vec![i + 1]);
            assert_eq!(h.tlm_of(CHANID_PACKETS_EXTRACTED), vec![i + 1]);
            assert_eq!(h.tlm_of(CHANID_LATEST_VC_FRAME_COUNT), vec![i]);
        }
    }

    // -- Robustness against untrusted input --------------------------------------------

    #[test]
    fn untrusted_fhp() {
        let (comp, h) = build();
        configure_default(&comp);
        feed(&comp, &frame(&[0u8; 50], TEST_DATA_ZONE_SIZE as u16));
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.events_of(EVENTID_INVALID_FHP).len(), 1);
        assert_eq!(h.errors(), vec![FrameError::AosInvalidLength]);
    }

    fn malicious_epp_header() -> Vec<u8> {
        let mut header = epp(
            EppProtocolId::MissionSpecific.as_repr(),
            EppLengthOfLength::Four,
            0,
        );
        header[4..8].copy_from_slice(&[0xFF, 0xFF, 0xFF, 0xFC]);
        header
    }

    #[test]
    fn epp_size_overflow_rejected() {
        let (comp, h) = build();
        configure_default(&comp);
        feed(&comp, &frame(&malicious_epp_header(), 0));
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.events_of(EVENTID_SPANNING_PACKET_ALLOC_FAILED).len(), 1);
    }

    #[test]
    fn epp_size_overflow_header_spans_frame() {
        let (comp, h) = build();
        configure_default(&comp);
        let header = malicious_epp_header();
        let split = TEST_DATA_ZONE_SIZE - 4;
        let mut payload1 = vec![0u8; TEST_DATA_ZONE_SIZE];
        payload1[split..].copy_from_slice(&header[..4]);
        feed(&comp, &frame_n(&payload1, split as u16, 0, 0));
        assert_eq!(h.n_out(), 0);
        assert!(h.events_of(EVENTID_SPANNING_PACKET_ALLOC_FAILED).is_empty());
        h.clear();
        feed(
            &comp,
            &frame_n(&header[4..], m_pdu_subfields::FHP_NO_PACKET_START, 0, 1),
        );
        assert_eq!(h.n_out(), 0);
        assert_eq!(h.n_ret(), 1);
        assert_eq!(h.events_of(EVENTID_SPANNING_PACKET_ALLOC_FAILED).len(), 1);
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn configure_requires_the_allocator() {
        let comp = AosDeframer::new("AosDeframer");
        comp.configure(
            TEST_FRAME_SIZE,
            true,
            SPACECRAFT_ID,
            0,
            pvn_bitfield::SPP_MASK,
        );
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn configure_rejects_invalid_pvn_bits() {
        let (comp, _h) = build();
        comp.configure(TEST_FRAME_SIZE, true, SPACECRAFT_ID, 0, 0x02);
    }
}
