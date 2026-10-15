//! # AosFramer — port of `Svc::Ccsds::AosFramer` (passive)
//!
//! C++ sources: `Svc/Ccsds/AosFramer/AosFramer.{cpp,hpp,fpp}` and its
//! `docs/sdd.md` (CCSDS 732.0-B-5, the AOS Space Data Link Protocol).
//!
//! Packs payload packets (Space Packets or Encapsulation Packets) into
//! fixed-size AOS transfer frames using the Virtual Channel Packet service
//! with Multiplexing PDUs (M_PDU):
//!
//! ```text
//! [ AOS primary header 6 ][ M_PDU header 2 ][ packet zone ][ FECF 2 (optional) ]
//! ```
//!
//! Several packets share one frame, a packet longer than the remaining zone
//! is striped across frames (the M_PDU First Header Pointer is `0xFFFF` on a
//! continuation-only frame), and a context with `sendNow` set pads the rest
//! of the frame with an SPP idle packet and sends it. An idle packet that
//! would be shorter than the 7-byte SPP minimum is staged in its own 7-byte
//! store and striped into the next frame. A frame is only sent once full;
//! while room remains the framer asks upstream for more with a
//! `comStatusOut` SUCCESS.
//!
//! Header fields (4.1.2): TFVN `0b01`, the 10-bit spacecraft ID split into
//! 8 LSBs (`globalVcId`) and 2 MSBs (signaling field), the 6-bit VCID, a
//! 24-bit VC frame count extended by the 4-bit VC frame count cycle (cycle
//! use flag set), replay flag clear. The count is written into a frame's
//! header and incremented when the header is built; both wrap mod 2^28.
//!
//! Like the C++ component the framer owns one frame store per virtual
//! channel, handed downstream on `dataOut` and taken back on
//! `dataReturnIn`, which also resumes packing any striped packet.
//!
//! ## Deviations (documented)
//!
//! - The C++ frame store is a member array lent out as an `Fw::Buffer`;
//!   here it is owned storage moved out and back (the C++ "returned buffer
//!   lies within the backer" assert becomes "not currently owned + capacity
//!   of `AosMaxFrameFixedSize`").
//! - The C++ component has no mutex: `dataIn` and `dataReturnIn` run on
//!   their callers' threads and recurse when downstream returns the frame
//!   synchronously. Here the per-VC state is behind a `Mutex`, so the
//!   output ports are invoked after the lock is released, in the C++ order
//!   (`dataReturnOut`, then `dataOut` or `comStatusOut`).
//! - The striped packet held between frames is a [`PacketSource`]: either
//!   the caller's owned [`Buffer`] or the framer's own staged idle packet
//!   (the C++ `Fw::Buffer` pointing at `spp_idle.backer`).
//! - The FPP declares the standard time/event/telemetry/parameter ports but
//!   the component uses none of them; they are not ported.
//! - C++ parity gotcha, reproduced: with EPP enabled in the idle-PVN mask
//!   (`configure` requires SPP and allows EPP too), `sendNow` takes the
//!   unimplemented EPP idle branch and pads nothing, so the frame is not
//!   sent until more data fills it.

use crate::ccsds::crc16::Crc16;
use crate::ccsds::idle_packet;
use crate::ccsds::types::{
    AOS_MAX_FRAME_FIXED_SIZE, AOSHeader, AOSTrailer, MPduHeader, SPACECRAFT_ID, Tfvn,
    aos_header_subfields, pvn_bitfield,
};
use fprime_comp::{
    ComDataWithContextPort, OutputPort, PassiveBase, PortRef, SuccessConditionPort,
    input_port_adapter,
};
use fprime_config::FwIndexType;
use fprime_fw::buffer::BufferStorage;
use fprime_fw::{Buffer, Endianness, ExtBuf, FrameContext, SerBuf, Success, fw_assert};
use std::sync::{Arc, Mutex};

/// SPP idle data pattern (`AosFramer::SPP_IDLE_DATA_PATTERN`).
pub const SPP_IDLE_DATA_PATTERN: u8 = idle_packet::DATA_PATTERN;

/// Minimum SPP idle packet length (`AosFramer::MIN_SPP_LENGTH`).
pub const MIN_SPP_LENGTH: usize = idle_packet::MIN_SIZE;

/// Offset of the M_PDU packet zone (`AosFramer::START_OF_PAYLOAD`).
pub const START_OF_PAYLOAD: usize = AOSHeader::SERIALIZED_SIZE + MPduHeader::SERIALIZED_SIZE;

/// M_PDU First Header Pointer of a frame in which no packet starts.
const FHP_NO_FRESH: u16 = 0xFFFF;

/// Number of virtual channels (`m_vcs[1]`).
const NUM_VCS: usize = 1;

/// The packet being striped across frames.
#[derive(Debug)]
pub enum PacketSource {
    /// A caller's packet buffer, returned on `dataReturnOut` once packed.
    User(Buffer),
    /// The framer's staged minimum-size SPP idle packet (never returned).
    SppIdle,
}

/// User packet to be spread across M_PDUs (`AosFramer::PDU`).
#[derive(Debug, Default)]
struct Pdu {
    packet: Option<PacketSource>,
    context: FrameContext,
    offset: usize,
}

/// One virtual channel (`AosFramer::AosVc`).
struct AosVc {
    /// Index into the VC array (`0xFF` until configured).
    vc_struct_index: u8,
    /// VCID of this virtual channel.
    virtual_channel_id: u8,
    /// Virtual frame count (24 bits + 4-bit cycle, wraps).
    virtual_frame_count: u32,
    /// Configured fixed frame size.
    frame_size: usize,
    /// The frame store; `Some` is the C++ `BufferOwnershipState::OWNED`.
    frame: Option<BufferStorage>,
    /// Multi-frame-per-PDU support.
    outstanding: Pdu,
    /// Bitfield of PVNs to use for inserted idle packets.
    idle_packet_types: u8,
    /// The staged SPP idle packet backstop.
    spp_idle: [u8; MIN_SPP_LENGTH],
    /// How far into the current M_PDU packet zone we are.
    current_payload_offset: u16,
    /// Past the first fresh packet in the current M_PDU.
    past_first_fresh_packet: bool,
}

/// Framer state (the C++ members).
struct AosFramerState {
    fecf: bool,
    spacecraft_id: u16,
    vcs: [AosVc; NUM_VCS],
}

/// Port calls deferred until the state lock is released, in C++ order.
#[derive(Default)]
struct Outputs {
    data_return: Option<(Buffer, FrameContext)>,
    frame_out: Option<(Buffer, FrameContext)>,
    status_out: Option<FwIndexType>,
}

/// `Svc::Ccsds::AosFramer` — passive AOS (M_PDU) framer.
pub struct AosFramer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `dataOut` — completed AOS frames.
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — fully packed payload buffers back upstream.
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `comStatusOut` — status upstream (pass-through and requests for more).
    pub com_status_out: OutputPort<dyn SuccessConditionPort>,
    state: Mutex<AosFramerState>,
}

impl AosFramer {
    /// Construct the component configured for the maximum frame size with
    /// the FECF on (the C++ constructor calls
    /// `configure(AosMaxFrameFixedSize, true)`).
    pub fn new(name: &str) -> Arc<Self> {
        let comp = Arc::new(Self {
            base: PassiveBase::new(name),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            com_status_out: OutputPort::new(),
            state: Mutex::new(AosFramerState {
                fecf: true,
                spacecraft_id: 0,
                vcs: core::array::from_fn(|_| AosVc {
                    vc_struct_index: 0xFF,
                    virtual_channel_id: 0,
                    virtual_frame_count: 0,
                    frame_size: 0,
                    frame: Some(vec![0u8; AOS_MAX_FRAME_FIXED_SIZE].into_boxed_slice()),
                    outstanding: Pdu::default(),
                    idle_packet_types: pvn_bitfield::SPP_MASK,
                    spp_idle: [0; MIN_SPP_LENGTH],
                    current_payload_offset: 0,
                    past_first_fresh_packet: false,
                }),
            }),
        });
        comp.configure(
            AOS_MAX_FRAME_FIXED_SIZE as u32,
            true,
            SPACECRAFT_ID,
            1,
            pvn_bitfield::SPP_MASK,
        );
        comp
    }

    /// C++ `configure(fixedFrameSize, frameErrorControlField, spacecraftId =
    /// ComCfg::SpacecraftId, vcId = 1, idlePvns = SPP)`.
    ///
    /// Asserts the frame size fits `AosMaxFrameFixedSize` and exceeds the
    /// header/trailer overhead, a 10-bit SCID, a 6-bit VCID, and that the
    /// idle-PVN mask includes SPP.
    pub fn configure(
        &self,
        fixed_frame_size: u32,
        frame_error_control_field: bool,
        spacecraft_id: u16,
        vc_id: u8,
        idle_pvns: u8,
    ) {
        let size = fixed_frame_size as usize;
        fw_assert!(size <= AOS_MAX_FRAME_FIXED_SIZE, fixed_frame_size as i32);
        fw_assert!(
            size > START_OF_PAYLOAD
                + if frame_error_control_field {
                    AOSTrailer::SERIALIZED_SIZE
                } else {
                    0
                },
            fixed_frame_size as i32
        );
        fw_assert!(spacecraft_id & 0xFC00 == 0, i32::from(spacecraft_id));
        fw_assert!(vc_id & 0xC0 == 0, i32::from(vc_id));
        fw_assert!(
            idle_pvns & pvn_bitfield::SPP_MASK != 0,
            i32::from(idle_pvns)
        );

        let mut st = self.state.lock().unwrap();
        // The FECF is constant for a physical channel in a mission phase.
        st.fecf = frame_error_control_field;
        st.spacecraft_id = spacecraft_id;
        for (index, vc) in st.vcs.iter_mut().enumerate() {
            vc.vc_struct_index = index as u8;
            vc.virtual_channel_id = vc_id;
            vc.frame_size = size;
            vc.idle_packet_types = idle_pvns;
        }
    }

    /// True while VC 0's frame store is held by this component.
    pub fn owns_frame_buffer(&self) -> bool {
        self.state.lock().unwrap().vcs[0].frame.is_some()
    }

    /// VC 0's virtual frame count (the count the next header will carry).
    pub fn virtual_frame_count(&self) -> u32 {
        self.state.lock().unwrap().vcs[0].virtual_frame_count
    }

    /// Set VC 0's virtual frame count (test hook mirroring the C++ tester's
    /// direct member access).
    pub fn set_virtual_frame_count(&self, count: u32) {
        self.state.lock().unwrap().vcs[0].virtual_frame_count = count;
    }

    /// `comStatusIn` — SYNC: forwarded upstream when connected.
    pub fn com_status_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn SuccessConditionPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Handlers ---------------------------------------------------------------

    /// `dataIn`: the CCSDS AOS VCP.request service.
    fn data_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        let mut out = Outputs::default();
        {
            let mut st = self.state.lock().unwrap();
            let vc_index = Self::get_vc_struct(&st, context);
            // The rest of the com stack must comply with the communication
            // adapter interface: one frame in flight.
            fw_assert!(st.vcs[vc_index].frame.is_some());
            if !st.vcs[vc_index].past_first_fresh_packet {
                Self::setup_header(&mut st, vc_index, context);
                Self::setup_m_pdu_header(&mut st, vc_index, false);
                st.vcs[vc_index].past_first_fresh_packet = true;
            }
            self.pack_pad_send(&mut st, PacketSource::User(data), context, 0, &mut out);
        }
        self.flush(out);
    }

    /// `dataReturnIn`: the frame store coming back; resume a striped packet.
    fn data_return_in_handler(
        &self,
        _port_num: FwIndexType,
        frame: Buffer,
        context: &FrameContext,
    ) {
        let mut out = Outputs::default();
        {
            let mut st = self.state.lock().unwrap();
            let vc_index = Self::get_vc_struct(&st, context);
            let vc = &mut st.vcs[vc_index];
            fw_assert!(vc.frame.is_none());
            fw_assert!(
                frame.capacity() == AOS_MAX_FRAME_FIXED_SIZE,
                frame.capacity() as i32
            );
            vc.frame = Some(frame.into_storage());
            if let Some(packet) = vc.outstanding.packet.take() {
                let ctx = vc.outstanding.context;
                let offset = vc.outstanding.offset;
                self.pack_pad_send(&mut st, packet, &ctx, offset, &mut out);
            }
        }
        self.flush(out);
    }

    /// Invoke the deferred output ports in C++ order.
    fn flush(&self, out: Outputs) {
        if let Some((data, context)) = out.data_return {
            let p = self.data_return_out.get();
            p.target.invoke(p.port_num, data, &context);
        }
        if let Some((frame, context)) = out.frame_out {
            let p = self.data_out.get();
            p.target.invoke(p.port_num, frame, &context);
        }
        if let Some(port) = out.status_out {
            if let Some(p) = self.com_status_out_at(port) {
                let mut condition = Success::Success;
                p.target.invoke(p.port_num, &mut condition);
            }
        }
    }

    /// `comStatusOut` at `port` when connected (one port: index 0).
    fn com_status_out_at(&self, port: FwIndexType) -> Option<&PortRef<dyn SuccessConditionPort>> {
        if port == 0 {
            self.com_status_out.try_get()
        } else {
            None
        }
    }

    /// Map a context onto its VC struct (single VC: index 0); asserts
    /// configuration and the context's VCID.
    fn get_vc_struct(st: &AosFramerState, context: &FrameContext) -> usize {
        let index = 0;
        fw_assert!(usize::from(st.vcs[index].vc_struct_index) == index);
        fw_assert!(
            st.vcs[index].virtual_channel_id == context.vc_id,
            i32::from(context.vc_id)
        );
        index
    }

    fn min_size(st: &AosFramerState) -> usize {
        START_OF_PAYLOAD
            + if st.fecf {
                AOSTrailer::SERIALIZED_SIZE
            } else {
                0
            }
    }

    /// The writable frame store window (`frame_size` bytes).
    fn frame_mut(vc: &mut AosVc) -> &mut [u8] {
        let size = vc.frame_size;
        fw_assert!(vc.frame.is_some());
        match vc.frame.as_mut() {
            Some(storage) => &mut storage[..size],
            None => &mut [],
        }
    }

    /// Fill out the Transfer Frame Primary Header (4.1.2) and advance the
    /// VC frame count.
    fn setup_header(st: &mut AosFramerState, vc_index: usize, context: &FrameContext) {
        let scid = st.spacecraft_id;
        let vc = &mut st.vcs[vc_index];
        let global_vc_id = (u16::from(context.vc_id)
            << aos_header_subfields::VIRTUAL_CHANNEL_ID_OFFSET)
            | ((scid & 0x00FF) << aos_header_subfields::SPACECRAFT_ID_LSB_OFFSET)
            | ((u16::from(Tfvn::Aos.as_repr()) & 0x3)
                << aos_header_subfields::FRAME_VERSION_OFFSET);
        // Virtual Channel Frame Count (4.1.2.4).
        let mut signaling =
            (vc.virtual_frame_count & 0x00FF_FFFF) << aos_header_subfields::VC_FRAME_COUNT_OFFSET;
        // Virtual Channel Frame Count Cycle Use Flag (4.1.2.5.3).
        signaling |= 1 << aos_header_subfields::CYCLE_COUNT_FLAG_OFFSET;
        // Spacecraft ID MSBs (4.1.2.5.4).
        signaling |=
            u32::from(scid & 0x0300) >> (8 - aos_header_subfields::SPACECRAFT_ID_MSB_OFFSET);
        // Virtual Channel Frame Count Cycle (4.1.2.5.5).
        signaling |= (vc.virtual_frame_count & 0x0F00_0000) >> 24;
        let header = AOSHeader::new(global_vc_id, signaling);
        // The modulo is applied at serialization time.
        vc.virtual_frame_count = vc.virtual_frame_count.wrapping_add(1);

        let mut ser = ExtBuf::new(Self::frame_mut(vc));
        let status = ser.move_ser_to_offset(0);
        fw_assert!(status.is_ok(), status as i32);
        let status = ser.serialize(&header, Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
    }

    /// Write the M_PDU first header pointer (4.1.4.2.2): the current zone
    /// offset (the first fresh packet starts here) or all ones.
    fn setup_m_pdu_header(st: &mut AosFramerState, vc_index: usize, no_fresh: bool) {
        let vc = &mut st.vcs[vc_index];
        let fhp = if no_fresh {
            FHP_NO_FRESH
        } else {
            vc.current_payload_offset
        };
        let mut ser = ExtBuf::new(Self::frame_mut(vc));
        let status = ser.move_ser_to_offset(AOSHeader::SERIALIZED_SIZE);
        fw_assert!(status.is_ok(), status as i32);
        let status = ser.serialize(&MPduHeader::new(fhp), Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
    }

    /// Compute and write the trailing Frame Error Control Field (4.1.6).
    fn compute_and_inject_fecf(vc: &mut AosVc) {
        let frame = Self::frame_mut(vc);
        let fecf_start = frame.len() - AOSTrailer::SERIALIZED_SIZE;
        let crc = Crc16::compute(&frame[..fecf_start]);
        let mut ser = ExtBuf::new(frame);
        let status = ser.move_ser_to_offset(fecf_start);
        fw_assert!(status.is_ok(), status as i32);
        let status = ser.serialize(&AOSTrailer::new(crc), Endianness::Big);
        fw_assert!(status.is_ok(), status as i32);
    }

    /// Pack, idle pad (on `sendNow`) and send (when full) one packet.
    fn pack_pad_send(
        &self,
        st: &mut AosFramerState,
        data: PacketSource,
        context: &FrameContext,
        data_offset: usize,
        out: &mut Outputs,
    ) {
        Self::pack_packet(st, data, context, data_offset, out);
        let vc_index = Self::get_vc_struct(st, context);
        if context.send_now {
            let min_size = Self::min_size(st);
            let frame_size = st.vcs[vc_index].frame_size;
            fw_assert!(frame_size >= min_size, frame_size as i32, min_size as i32);
            let max_payload = frame_size - min_size;
            if usize::from(st.vcs[vc_index].current_payload_offset) < max_payload {
                // CCSDS 4.2.2.5: fill the rest of the data field with idle.
                Self::fill_with_idle_packet(st, vc_index, context, out);
            }
        }
        Self::check_and_send_vc(st, vc_index, out);
    }

    /// Write a packet (from `data_offset`) into the VC frame, keeping any
    /// bytes that do not fit as the outstanding striped packet.
    fn pack_packet(
        st: &mut AosFramerState,
        data: PacketSource,
        context: &FrameContext,
        data_offset: usize,
        out: &mut Outputs,
    ) {
        let min_size = Self::min_size(st);
        let vc_index = Self::get_vc_struct(st, context);
        let vc = &mut st.vcs[vc_index];
        let max_payload = vc.frame_size - min_size;
        let offset = usize::from(vc.current_payload_offset);
        // C++ computes this unsigned and asserts it did not wrap.
        fw_assert!(offset <= max_payload, (min_size + offset) as i32);
        let bytes_available = max_payload - offset;

        let total = match &data {
            PacketSource::User(buffer) => buffer.size(),
            PacketSource::SppIdle => MIN_SPP_LENGTH,
        };
        let mut data_size = total - data_offset;
        let complete = data_size <= bytes_available;
        if complete {
            vc.outstanding.offset = 0;
        } else {
            // Clamp the write; pick serialization up here later.
            data_size = bytes_available;
            vc.outstanding.offset = data_offset + data_size;
        }

        {
            let idle = vc.spp_idle;
            let src: &[u8] = match &data {
                PacketSource::User(buffer) => &buffer.data()[data_offset..data_offset + data_size],
                PacketSource::SppIdle => &idle[data_offset..data_offset + data_size],
            };
            let start = START_OF_PAYLOAD + offset;
            let frame = Self::frame_mut(vc);
            fw_assert!(start + data_size <= frame.len(), start as i32);
            frame[start..start + data_size].copy_from_slice(src);
        }
        vc.current_payload_offset = (offset + data_size) as u16;
        // Context must always be present once payload bytes are written.
        vc.outstanding.context = *context;

        if complete {
            // Return the buffer unless it is the staged idle packet.
            if let PacketSource::User(buffer) = data {
                out.data_return = Some((buffer, *context));
            }
            vc.outstanding.packet = None;
            vc.outstanding.offset = 0;
        } else {
            // C++ overwrites any previous outstanding packet; by protocol
            // there is none (the striped packet is resumed on return).
            fw_assert!(vc.outstanding.packet.is_none());
            vc.outstanding.packet = Some(data);
        }
    }

    /// Send the frame if full; otherwise ask upstream for more bytes.
    fn check_and_send_vc(st: &mut AosFramerState, vc_index: usize, out: &mut Outputs) {
        let min_size = Self::min_size(st);
        let fecf = st.fecf;
        let max_payload = st.vcs[vc_index].frame_size - min_size;
        if usize::from(st.vcs[vc_index].current_payload_offset) == max_payload {
            // Continuing packet only: write the headers right before sending.
            if !st.vcs[vc_index].past_first_fresh_packet {
                let ctx = st.vcs[vc_index].outstanding.context;
                Self::setup_header(st, vc_index, &ctx);
                Self::setup_m_pdu_header(st, vc_index, true);
            }
            let vc = &mut st.vcs[vc_index];
            if fecf {
                Self::compute_and_inject_fecf(vc);
            }
            // Ensure we are not double sending.
            let storage = vc.frame.take();
            fw_assert!(storage.is_some());
            let Some(storage) = storage else { return };
            vc.current_payload_offset = 0;
            vc.past_first_fresh_packet = false;
            let mut frame = Buffer::from_storage(storage, Buffer::NO_CONTEXT);
            frame.set_size(vc.frame_size);
            out.frame_out = Some((frame, vc.outstanding.context));
        } else {
            // Room left: ask for more (comStatusOut[vc_struct_index]).
            out.status_out = Some(FwIndexType::from(st.vcs[vc_index].vc_struct_index));
        }
    }

    /// Fill the frame from the current offset with an idle packet (4.2.2.5),
    /// striping a minimum SPP idle packet into the next frame if needed.
    fn fill_with_idle_packet(
        st: &mut AosFramerState,
        vc_index: usize,
        context: &FrameContext,
        out: &mut Outputs,
    ) {
        let overhead = Self::min_size(st);
        let frame_size = st.vcs[vc_index].frame_size;
        fw_assert!(frame_size > 0, frame_size as i32);
        fw_assert!(frame_size < usize::from(u16::MAX), frame_size as i32);
        let pdu_size = frame_size - overhead;
        let idle_packet_size = pdu_size - usize::from(st.vcs[vc_index].current_payload_offset);

        // The idle packet is the fresh packet if none preceded it.
        if !st.vcs[vc_index].past_first_fresh_packet {
            Self::setup_header(st, vc_index, context);
            Self::setup_m_pdu_header(st, vc_index, false);
            st.vcs[vc_index].past_first_fresh_packet = true;
        }

        if st.vcs[vc_index].idle_packet_types & pvn_bitfield::EPP_MASK != 0 {
            // EPP idle fill is not yet supported upstream (TODO there): no
            // fill is written. Reproduced; see the module docs.
        } else if idle_packet_size < MIN_SPP_LENGTH {
            // Stage a minimum idle packet and stripe it into the next frame.
            let vc = &mut st.vcs[vc_index];
            fw_assert!(vc.outstanding.packet.is_none());
            {
                let mut ser = ExtBuf::new(&mut vc.spp_idle);
                let status = idle_packet::serialize(&mut ser, MIN_SPP_LENGTH);
                fw_assert!(status.is_ok(), status as i32);
            }
            // The idle fragment spilling onto the next frame need not be
            // sent now.
            let filtered = FrameContext {
                send_now: false,
                ..*context
            };
            Self::pack_packet(st, PacketSource::SppIdle, &filtered, 0, out);
        } else {
            let vc = &mut st.vcs[vc_index];
            let start = START_OF_PAYLOAD + usize::from(vc.current_payload_offset);
            {
                let frame = Self::frame_mut(vc);
                let mut ser = ExtBuf::new(&mut frame[start..]);
                let status = idle_packet::serialize(&mut ser, idle_packet_size);
                fw_assert!(status.is_ok(), status as i32);
            }
            vc.current_payload_offset =
                (usize::from(vc.current_payload_offset) + idle_packet_size) as u16;
        }
    }
}

/// `comStatusIn` handler — upstream decides which VCs get more data; the
/// status is forwarded when `comStatusOut` is connected.
impl SuccessConditionPort for AosFramer {
    fn invoke(&self, port_num: FwIndexType, condition: &mut Success) {
        if let Some(p) = self.com_status_out_at(port_num) {
            p.target.invoke(p.port_num, condition);
        }
    }
}

input_port_adapter! {
    /// `dataIn` — SYNC `Svc.ComDataWithContext`: a packet to frame.
    component: AosFramer;
    adapter: DataInAdapter;
    port: ComDataWithContextPort;
    input: pub data_in;
    handler: data_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

input_port_adapter! {
    /// `dataReturnIn` — SYNC: the frame store coming back.
    component: AosFramer;
    adapter: DataReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub data_return_in;
    handler: data_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

// ---------------------------------------------------------------------------
// Tests — port of AosFramerTester
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ccsds::types::{SpacePacketHeader, space_packet_subfields};
    use fprime_fw::{Apid, Deserialize};
    use std::sync::Mutex as StdMutex;

    #[derive(Default)]
    struct Recorder {
        data_out: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        frames: StdMutex<Vec<Buffer>>,
        data_return: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        statuses: StdMutex<Vec<Success>>,
    }

    impl ComDataWithContextPort for Recorder {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            if port_num == 0 {
                self.data_out
                    .lock()
                    .unwrap()
                    .push((data.data().to_vec(), *context));
                self.frames.lock().unwrap().push(data);
            } else {
                self.data_return
                    .lock()
                    .unwrap()
                    .push((data.data().to_vec(), *context));
            }
        }
    }

    impl SuccessConditionPort for Recorder {
        fn invoke(&self, _port_num: FwIndexType, condition: &mut Success) {
            self.statuses.lock().unwrap().push(*condition);
        }
    }

    fn build() -> (Arc<AosFramer>, Arc<Recorder>) {
        let rec = Arc::new(Recorder::default());
        let framer = AosFramer::new("AosFramer");
        framer.data_out.connect(rec.clone(), 0);
        framer.data_return_out.connect(rec.clone(), 1);
        framer.com_status_out.connect(rec.clone(), 0);
        framer.configure(
            AOS_MAX_FRAME_FIXED_SIZE as u32,
            true,
            SPACECRAFT_ID,
            1,
            pvn_bitfield::SPP_MASK,
        );
        (framer, rec)
    }

    fn feed(framer: &Arc<AosFramer>, bytes: &[u8], context: &FrameContext) {
        let mut b = Buffer::allocate(bytes.len());
        b.data_mut().copy_from_slice(bytes);
        let p = framer.data_in(0);
        p.target.invoke(p.port_num, b, context);
    }

    fn return_frame(framer: &Arc<AosFramer>, rec: &Recorder, context: &FrameContext) {
        let frame = rec.frames.lock().unwrap().remove(0);
        let p = framer.data_return_in(0);
        p.target.invoke(p.port_num, frame, context);
    }

    fn send_now() -> FrameContext {
        FrameContext {
            send_now: true,
            ..FrameContext::default()
        }
    }

    // Tester helpers (AosFramerTester::getFrame*).
    fn tfvn(f: &[u8]) -> u8 {
        (f[0] & 0xC0) >> 6
    }
    fn scid(f: &[u8]) -> u16 {
        ((u16::from(f[1]) >> 6) & 0x0003)
            | ((u16::from(f[0]) << 2) & 0x00FC)
            | ((u16::from(f[5]) << 4) & 0x0300)
    }
    fn vcid(f: &[u8]) -> u8 {
        f[1] & 0x3F
    }
    fn vc_count(f: &[u8]) -> u32 {
        let mut c = (u32::from(f[2]) << 16) | (u32::from(f[3]) << 8) | u32::from(f[4]);
        if f[5] & 0x40 != 0 {
            c |= u32::from(f[5] & 0x0F) << 24;
        }
        c
    }
    fn packet_pointer(f: &[u8]) -> u16 {
        u16::from_be_bytes([f[6], f[7]])
    }

    #[test]
    fn com_status_passthrough() {
        let (framer, rec) = build();
        let p = framer.com_status_in(0);
        p.target.invoke(p.port_num, &mut Success::Success);
        p.target.invoke(p.port_num, &mut Success::Failure);
        assert_eq!(
            *rec.statuses.lock().unwrap(),
            vec![Success::Success, Success::Failure]
        );
    }

    #[test]
    fn nominal_framing() {
        let (framer, rec) = build();
        let payload: Vec<u8> = (0..100u8).collect();
        let ctx = send_now();
        feed(&framer, &payload, &ctx);

        let out = rec.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        let (frame, out_ctx) = &out[0];
        assert_eq!(frame.len(), AOS_MAX_FRAME_FIXED_SIZE);
        assert_eq!(out_ctx.vc_id, ctx.vc_id);
        assert_eq!(tfvn(frame), 0b01);
        assert_eq!(scid(frame), SPACECRAFT_ID);
        assert_eq!(vcid(frame), ctx.vc_id);
        assert_eq!(vc_count(frame), 0);
        assert_eq!(framer.virtual_frame_count(), 1);
        // Literal header: TFVN 01 | SCID LSBs 0x44 | VCID 1; count 0;
        // cycle-use flag, SCID MSBs 00, cycle 0; FHP 0.
        assert_eq!(
            &frame[..8],
            &[0x51, 0x01, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00]
        );
        assert_eq!(&frame[8..108], &payload[..]);
        // The idle SPP fills to the trailer.
        let idle_data = START_OF_PAYLOAD + payload.len() + SpacePacketHeader::SERIALIZED_SIZE;
        let end = AOS_MAX_FRAME_FIXED_SIZE - AOSTrailer::SERIALIZED_SIZE;
        assert!(
            frame[idle_data..end]
                .iter()
                .all(|&b| b == SPP_IDLE_DATA_PATTERN)
        );
        // FECF residue.
        assert_eq!(Crc16::compute(frame), 0);
        // The payload buffer came back.
        assert_eq!(rec.data_return.lock().unwrap()[0].0, payload);
    }

    #[test]
    fn seq_count_wrap_around() {
        let (framer, rec) = build();
        let payload: Vec<u8> = (0..100u8).collect();
        let ctx = send_now();
        framer.set_virtual_frame_count((1 << 28) - 5);
        let mut expected: u32 = (1 << 28) - 5;
        for iter in 0..10 {
            feed(&framer, &payload, &ctx);
            {
                let out = rec.data_out.lock().unwrap();
                assert_eq!(out.len(), iter + 1);
                assert_eq!(vc_count(&out[iter].0), expected & 0x0FFF_FFFF);
            }
            return_frame(&framer, &rec, &ctx);
            expected = expected.wrapping_add(1);
        }
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn returning_a_foreign_buffer_asserts() {
        let (framer, _rec) = build();
        // The framer still owns its store, so any return is foreign.
        let p = framer.data_return_in(0);
        p.target
            .invoke(p.port_num, Buffer::allocate(10), &send_now());
    }

    #[test]
    fn data_return_restores_ownership() {
        let (framer, rec) = build();
        feed(&framer, &[1u8; 10], &send_now());
        assert!(!framer.owns_frame_buffer());
        return_frame(&framer, &rec, &send_now());
        assert!(framer.owns_frame_buffer());
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn buffer_ownership_state_asserts() {
        let (framer, _rec) = build();
        feed(&framer, &[1u8; 10], &send_now());
        assert!(!framer.owns_frame_buffer());
        // A second packet while the frame is downstream asserts.
        feed(&framer, &[1u8; 10], &send_now());
    }

    /// Many frames for one packet (upstream `testLongPacket`).
    #[test]
    fn long_packet() {
        let (framer, rec) = build();
        const FRAME_SIZE: usize = 1024;
        framer.configure(
            FRAME_SIZE as u32,
            true,
            SPACECRAFT_ID,
            1,
            pvn_bitfield::SPP_MASK,
        );
        let payload: Vec<u8> = (0..2048u32).map(|i| i as u8).collect();
        let ctx = send_now();
        assert!(rec.data_out.lock().unwrap().is_empty());
        feed(&framer, &payload, &ctx);

        let per_frame = FRAME_SIZE - START_OF_PAYLOAD - AOSTrailer::SERIALIZED_SIZE;
        let mut payload_ind = 0usize;
        for frame_no in 0..3 {
            let (frame, out_ctx) = {
                let out = rec.data_out.lock().unwrap();
                assert_eq!(out.len(), frame_no + 1);
                out[frame_no].clone()
            };
            assert_eq!(frame.len(), FRAME_SIZE);
            assert_eq!(out_ctx.vc_id, ctx.vc_id);
            assert_eq!(tfvn(&frame), 0b01);
            assert_eq!(scid(&frame), SPACECRAFT_ID);
            assert_eq!(vcid(&frame), ctx.vc_id);
            assert_eq!(vc_count(&frame), frame_no as u32);
            assert_eq!(framer.virtual_frame_count(), frame_no as u32 + 1);
            assert_eq!(Crc16::compute(&frame), 0);
            let fhp = packet_pointer(&frame);
            let (mut start, mut stop) =
                (START_OF_PAYLOAD, FRAME_SIZE - AOSTrailer::SERIALIZED_SIZE);
            match frame_no {
                0 => assert_eq!(fhp, 0),
                1 => assert_eq!(fhp, 0xFFFF),
                _ => {
                    let expected_fhp = (payload.len() % per_frame) as u16;
                    assert_eq!(fhp, expected_fhp);
                    let idle_spp = START_OF_PAYLOAD + usize::from(fhp);
                    let mut header = SpacePacketHeader::default();
                    let mut bytes = frame.clone();
                    let mut de = ExtBuf::with_len(&mut bytes[idle_spp..], 6);
                    assert!(header.deserialize_from(&mut de, Endianness::Big).is_ok());
                    assert_eq!(header.packet_identification, Apid::SppIdlePacket as u16);
                    assert_eq!(
                        header.packet_sequence_control,
                        0x3 << space_packet_subfields::SEQ_FLAGS_OFFSET
                    );
                    let idle_data = idle_spp + SpacePacketHeader::SERIALIZED_SIZE;
                    assert_eq!(usize::from(header.packet_data_length), stop - idle_data - 1);
                    assert!(
                        frame[idle_data..stop]
                            .iter()
                            .all(|&b| b == SPP_IDLE_DATA_PATTERN)
                    );
                    stop = idle_spp;
                    start = START_OF_PAYLOAD;
                }
            }
            for &byte in &frame[start..stop] {
                assert_eq!(byte, payload_ind as u8);
                payload_ind += 1;
            }
            return_frame(&framer, &rec, &out_ctx);
        }
        assert_eq!(payload_ind, payload.len());
        assert_eq!(rec.data_out.lock().unwrap().len(), 3);
        // The packet buffer was returned once fully packed.
        assert_eq!(rec.data_return.lock().unwrap().len(), 1);
    }

    /// Many packets for one frame, ending with fewer than 7 bytes so the
    /// idle packet stripes into the next frame (upstream `testShortPackets`).
    #[test]
    fn short_packets() {
        let (framer, rec) = build();
        const FRAME_SIZE: usize = 1024;
        framer.configure(
            FRAME_SIZE as u32,
            true,
            SPACECRAFT_ID,
            1,
            pvn_bitfield::SPP_MASK,
        );
        let payload: Vec<u8> = (0..1008u32).map(|i| i as u8).collect();
        let packets = 10;
        let packet_size = payload.len() / packets;
        let mut ctx = FrameContext::default();
        for packet in 0..packets {
            assert!(rec.data_out.lock().unwrap().is_empty());
            let mut size = packet_size;
            if packet + 1 == packets {
                ctx.send_now = true;
                size = payload.len() - packet * packet_size;
            }
            feed(
                &framer,
                &payload[packet * packet_size..packet * packet_size + size],
                &ctx,
            );
        }
        // While room remained, the framer asked for more each time.
        assert_eq!(rec.statuses.lock().unwrap().len(), packets - 1);
        let (frame, out_ctx) = rec.data_out.lock().unwrap()[0].clone();
        assert_eq!(frame.len(), FRAME_SIZE);
        assert_eq!(vc_count(&frame), 0);
        assert_eq!(framer.virtual_frame_count(), 1);
        assert_eq!(packet_pointer(&frame), 0);
        assert_eq!(
            &frame[START_OF_PAYLOAD..START_OF_PAYLOAD + payload.len()],
            &payload[..]
        );
        // 1014 - 1008 = 6 bytes left: the leading 6 bytes of a 7-byte idle
        // packet (its header, token 0) — the last byte stripes onward.
        let idle_spp = START_OF_PAYLOAD + payload.len();
        assert_eq!(
            &frame[idle_spp..idle_spp + 6],
            &[0x07, 0xFF, 0xC0, 0x00, 0x00, 0x00]
        );
        assert_eq!(Crc16::compute(&frame), 0);
        // The sent context is the staged idle fragment's (sendNow cleared).
        assert!(!out_ctx.send_now);
        return_frame(&framer, &rec, &out_ctx);
        // The idle remainder was packed into the next frame (no new frame
        // yet; the framer asks for more) and no extra buffer was returned.
        assert_eq!(rec.data_out.lock().unwrap().len(), 1);
        assert_eq!(rec.data_return.lock().unwrap().len(), packets);

        // The next packet's frame: the idle byte first, then the packet,
        // with the FHP pointing past the idle remainder.
        feed(&framer, &[0xAB; 20], &send_now());
        let (frame2, _) = rec.data_out.lock().unwrap()[1].clone();
        assert_eq!(packet_pointer(&frame2), 1);
        assert_eq!(frame2[START_OF_PAYLOAD], SPP_IDLE_DATA_PATTERN);
        assert_eq!(
            &frame2[START_OF_PAYLOAD + 1..START_OF_PAYLOAD + 21],
            &[0xAB; 20]
        );
        assert_eq!(vc_count(&frame2), 1);
    }

    /// Without the FECF the packet zone runs to the end of the frame.
    #[test]
    fn fecf_disabled_frame_has_no_trailer() {
        let (framer, rec) = build();
        framer.configure(64, false, SPACECRAFT_ID, 1, pvn_bitfield::SPP_MASK);
        feed(&framer, &[0x11; 56], &FrameContext::default());
        let (frame, _) = rec.data_out.lock().unwrap()[0].clone();
        assert_eq!(frame.len(), 64);
        assert_eq!(&frame[8..], &[0x11; 56]);
    }

    /// A 10-bit spacecraft ID splits across the header (8 LSBs + 2 MSBs).
    #[test]
    fn spacecraft_id_msbs_land_in_the_signaling_field() {
        let (framer, rec) = build();
        framer.configure(64, true, 0x3A5, 1, pvn_bitfield::SPP_MASK);
        feed(&framer, &[0; 10], &send_now());
        let (frame, _) = rec.data_out.lock().unwrap()[0].clone();
        assert_eq!(scid(&frame), 0x3A5);
        assert_eq!(frame[5] & 0x30, 0x30);
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn wrong_vcid_in_context_asserts() {
        let (framer, _rec) = build();
        let ctx = FrameContext {
            vc_id: 2,
            ..FrameContext::default()
        };
        feed(&framer, &[0; 10], &ctx);
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn configure_rejects_oversized_frames() {
        let (framer, _rec) = build();
        framer.configure(
            AOS_MAX_FRAME_FIXED_SIZE as u32 + 1,
            true,
            SPACECRAFT_ID,
            1,
            pvn_bitfield::SPP_MASK,
        );
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn configure_requires_spp_idle() {
        let (framer, _rec) = build();
        framer.configure(256, true, SPACECRAFT_ID, 1, pvn_bitfield::EPP_MASK);
    }
}
