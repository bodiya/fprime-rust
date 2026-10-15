//! # CcsdsSdlsFramer — port of `Svc::Ccsds::CcsdsSdlsFramer` (passive)
//!
//! C++ sources: `Svc/Ccsds/CcsdsSdlsFramer/CcsdsSdlsFramer.{cpp,hpp,fpp}`
//! and its `docs/sdd.md`.
//!
//! Frames buffers into CCSDS SDLS frames: a framer in the downlink pipeline
//! (`Svc.Framer`) and a client of an SDLS encryption helper
//! (`Svc.Ccsds.CcsdsSdlsEncryptClient`, typically an `SdlsSaRouter`):
//!
//! 1. The SA index is the context's `saIndex` when set, else the `SA_INDEX`
//!    parameter (default 1); it is recorded in a context copy and the data
//!    goes out on `encryptOut`.
//! 2. On `encryptIn` a non-SUCCESS status raises `EncryptionFailed`, returns
//!    the buffer on `encryptReturnOut` and emits a ready-for-more
//!    `comStatusOut` SUCCESS so a ComQueue-driven downlink does not stall.
//! 3. Otherwise a frame buffer of `2 + size` bytes is allocated, the
//!    big-endian SA index prepended to the encrypted data, the encrypted
//!    buffer returned on `encryptReturnOut` and the frame sent on `dataOut`.
//!    An invalid or undersized allocation raises `BufferAllocationFailed`
//!    and drops the frame the same way.
//!
//! Frames returning on `dataReturnIn` are deallocated; original buffers
//! the helper returns on `bufferReturnIn` go upstream on `dataReturnOut`.
//!
//! Commands/parameters: the implicit `SA_INDEX_PRM_SET` / `SA_INDEX_PRM_SAVE`
//! commands and [`CcsdsSdlsFramer::load_parameters`] are hand-written
//! following `fpp-to-rust`'s generated parameter code. Reading the
//! parameter before it has been loaded asserts (the C++ `FW_PARAM_OK`
//! check on an `UNINIT` parameter).

use crate::ccsds::types::{CcsdsSdlsDataPort, CcsdsSdlsEncryptionPort, SdlsStatus};
use alloc::format;
use fprime_comp::{
    BufferGetPort, BufferSendPort, CmdGlue, CmdPort, ComDataWithContextPort, EventGlue, OutputPort,
    PassiveBase, PortRef, PrmGlue, SuccessConditionPort, input_port_adapter,
};
use fprime_config::{FwEventIdType, FwIndexType, FwOpcodeType, FwPrmIdType, FwSizeType};
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::{
    Buffer, CmdArgBuffer, CmdResponse, Endianness, FrameContext, LengthMode, LogSeverity,
    ParamBuffer, ParamValid, SA_INDEX_UNSET, SerBuf, Serialize, Success, fw_assert,
};

/// `SA_INDEX_PRM_SET` opcode (implicit parameter-set command).
pub const OPCODE_SA_INDEX_PRM_SET: FwOpcodeType = 0;
/// `SA_INDEX_PRM_SAVE` opcode (implicit parameter-save command).
pub const OPCODE_SA_INDEX_PRM_SAVE: FwOpcodeType = 1;

/// `EncryptionFailed(status: Svc.Ccsds.SdlsStatus)` — WARNING_HI.
pub const EVENTID_ENCRYPTION_FAILED: FwEventIdType = 0;
/// `BufferAllocationFailed(allocationSize: FwSizeType)` — WARNING_HI.
pub const EVENTID_BUFFER_ALLOCATION_FAILED: FwEventIdType = 1;

/// `SA_INDEX: U16` parameter id.
pub const PARAMID_SA_INDEX: FwPrmIdType = 0;
/// FPP default of `SA_INDEX`.
pub const SA_INDEX_DEFAULT: u16 = 1;

/// Size of the prepended SA index.
const SA_INDEX_SIZE: usize = size_of::<u16>();

/// The `SA_INDEX` parameter and its validity (the autocoded
/// `m_SA_INDEX` / `m_param_SA_INDEX_valid`).
struct Params {
    sa_index: u16,
    sa_index_valid: ParamValid,
}

/// `Svc::Ccsds::CcsdsSdlsFramer` — passive SDLS framer.
pub struct CcsdsSdlsFramer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Command registration + response ports.
    pub cmd: CmdGlue,
    /// Event ports and the time port.
    pub evt: EventGlue,
    /// Parameter ports.
    pub prm: PrmGlue,
    /// `dataOut` — SDLS frames downstream.
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — original data buffers back upstream.
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `comStatusOut` — status upstream.
    pub com_status_out: OutputPort<dyn SuccessConditionPort>,
    /// `encryptOut` — the SA index and data to the encryption helper.
    pub encrypt_out: OutputPort<dyn CcsdsSdlsEncryptionPort>,
    /// `encryptReturnOut` — encrypted buffers back to the helper.
    pub encrypt_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `bufferAllocate` — frame buffers.
    pub buffer_allocate: OutputPort<dyn BufferGetPort>,
    /// `bufferDeallocate` — frame buffers back to the allocator.
    pub buffer_deallocate: OutputPort<dyn BufferSendPort>,
    params: Mutex<Params>,
}

impl CcsdsSdlsFramer {
    /// Construct the component (parameters unloaded).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            cmd: CmdGlue::new(),
            evt: EventGlue::new(),
            prm: PrmGlue::new(),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            com_status_out: OutputPort::new(),
            encrypt_out: OutputPort::new(),
            encrypt_return_out: OutputPort::new(),
            buffer_allocate: OutputPort::new(),
            buffer_deallocate: OutputPort::new(),
            params: Mutex::new(Params {
                sa_index: SA_INDEX_DEFAULT,
                sa_index_valid: ParamValid::Uninit,
            }),
        })
    }

    /// C++ `regCommands()`.
    pub fn reg_commands(&self) {
        self.cmd.reg_commands(
            self.base.get_id_base(),
            &[OPCODE_SA_INDEX_PRM_SET, OPCODE_SA_INDEX_PRM_SAVE],
        );
    }

    /// C++ generated `loadParameters()`: read `SA_INDEX` through
    /// `prmGetOut`, falling back to the FPP default (validity `DEFAULT`)
    /// when the database has no value. Requires `prmGetOut` connected.
    pub fn load_parameters(&self) {
        let mut buf = ParamBuffer::new();
        let valid = self
            .prm
            .get_param(self.base.get_id_base(), PARAMID_SA_INDEX, &mut buf);
        let mut params = self.params.lock();
        match valid {
            ParamValid::Valid | ParamValid::Default => {
                let mut value = 0u16;
                if buf.deserialize_u16_be(&mut value).is_ok() {
                    params.sa_index = value;
                    params.sa_index_valid = valid;
                } else {
                    params.sa_index_valid = ParamValid::Invalid;
                }
            }
            _ => {
                params.sa_index = SA_INDEX_DEFAULT;
                params.sa_index_valid = ParamValid::Default;
            }
        }
    }

    /// The generated `paramGet_SA_INDEX(valid)`.
    pub fn param_get_sa_index(&self) -> (u16, ParamValid) {
        let params = self.params.lock();
        (params.sa_index, params.sa_index_valid)
    }

    // -- Input ports ------------------------------------------------------------

    /// `comStatusIn` — SYNC: forwarded when `comStatusOut` is connected.
    pub fn com_status_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn SuccessConditionPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `encryptIn` — SYNC `Svc.Ccsds.CcsdsSdlsData`: the helper's result.
    pub fn encrypt_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CcsdsSdlsDataPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `cmdIn` — the command input (sync on this passive component).
    pub fn cmd_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CmdPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Handlers ---------------------------------------------------------------

    fn data_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        // The context's SA index when set, otherwise the SA_INDEX parameter.
        let mut sa_index = context.sa_index;
        if sa_index == SA_INDEX_UNSET {
            let (value, valid) = self.param_get_sa_index();
            // C++ FW_PARAM_OK: VALID or DEFAULT.
            fw_assert!(
                matches!(valid, ParamValid::Valid | ParamValid::Default),
                valid as i32
            );
            sa_index = value;
        }
        let new_context = FrameContext {
            sa_index,
            ..*context
        };
        let p = self.encrypt_out.get();
        p.target.invoke(p.port_num, sa_index, data, &new_context);
    }

    fn data_return_in_handler(
        &self,
        _port_num: FwIndexType,
        data: Buffer,
        _context: &FrameContext,
    ) {
        // The allocated frame buffer coming back from dataOut.
        let p = self.buffer_deallocate.get();
        p.target.invoke(p.port_num, data);
    }

    fn buffer_return_in_handler(
        &self,
        _port_num: FwIndexType,
        data: Buffer,
        context: &FrameContext,
    ) {
        // The helper returned the original data buffer: send it upstream.
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    fn encrypt_in_handler(
        &self,
        _port_num: FwIndexType,
        status: SdlsStatus,
        data: Buffer,
        context: &FrameContext,
    ) {
        if status != SdlsStatus::Success {
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_ENCRYPTION_FAILED,
                LogSeverity::WarningHi,
                &format!("Failed to encrypt frame: {status:?}"),
                |buf| status.serialize_to(buf, Endianness::Big),
            );
            // Drop the frame: ownership back to the encryption subsystem.
            self.encrypt_return_out(data, context);
            self.send_com_status_on_drop();
            return;
        }

        let frame_size = data.size() + SA_INDEX_SIZE;
        let p = self.buffer_allocate.get();
        let mut frame = p.target.invoke(p.port_num, frame_size as FwSizeType);
        if !frame.is_valid() || frame.size() < frame_size {
            let allocation_size = frame_size as FwSizeType;
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_BUFFER_ALLOCATION_FAILED,
                LogSeverity::WarningHi,
                &format!("Failed to allocate frame buffer of size {allocation_size}"),
                |buf| buf.serialize_u64_be(allocation_size),
            );
            // Drop: return the undersized allocation (when valid) and the
            // encrypted buffer.
            if frame.is_valid() {
                let p = self.buffer_deallocate.get();
                p.target.invoke(p.port_num, frame);
            }
            self.encrypt_return_out(data, context);
            self.send_com_status_on_drop();
            return;
        }

        {
            let mut ser = frame.get_serializer();
            let status = ser.serialize_u16_be(context.sa_index);
            fw_assert!(status.is_ok(), status as i32);
            let status = ser.serialize_bytes(data.data(), LengthMode::OmitLength, Endianness::Big);
            fw_assert!(status.is_ok(), status as i32);
        }
        // Trim in case the allocator returned a larger buffer.
        frame.set_size(frame_size);

        // Encrypted data back to the helper, then the frame downstream.
        self.encrypt_return_out(data, context);
        let p = self.data_out.get();
        p.target.invoke(p.port_num, frame, context);
    }

    fn encrypt_return_out(&self, data: Buffer, context: &FrameContext) {
        let p = self.encrypt_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    /// Report ready-for-more on a dropped frame so a ComQueue-driven
    /// downlink does not stall.
    fn send_com_status_on_drop(&self) {
        if let Some(p) = self.com_status_out.try_get() {
            let mut status = Success::Success;
            p.target.invoke(p.port_num, &mut status);
        }
    }

    // -- Commands ---------------------------------------------------------------

    fn handle_command(&self, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer) {
        match op_code.wrapping_sub(self.base.get_id_base()) {
            OPCODE_SA_INDEX_PRM_SET => {
                let mut value = 0u16;
                if !args.deserialize_u16_be(&mut value).is_ok() || args.deserialize_size_left() != 0
                {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                {
                    let mut params = self.params.lock();
                    params.sa_index = value;
                    params.sa_index_valid = ParamValid::Valid;
                }
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            }
            OPCODE_SA_INDEX_PRM_SAVE => {
                if args.deserialize_size_left() != 0 {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::FormatError);
                    return;
                }
                if !self.prm.prm_set_out.is_connected() {
                    self.cmd
                        .cmd_response(op_code, cmd_seq, CmdResponse::ExecutionError);
                    return;
                }
                let value = self.params.lock().sa_index;
                let mut buf = ParamBuffer::new();
                let status = buf.serialize_u16_be(value);
                fw_assert!(status.is_ok(), status as i32);
                self.prm
                    .set_param(self.base.get_id_base(), PARAMID_SA_INDEX, &mut buf);
                self.cmd.cmd_response(op_code, cmd_seq, CmdResponse::Ok);
            }
            _ => self
                .cmd
                .cmd_response(op_code, cmd_seq, CmdResponse::InvalidOpcode),
        }
    }
}

impl SuccessConditionPort for CcsdsSdlsFramer {
    fn invoke(&self, port_num: FwIndexType, condition: &mut Success) {
        // Pass-through on the same port index when connected.
        if port_num == 0 {
            if let Some(p) = self.com_status_out.try_get() {
                p.target.invoke(p.port_num, condition);
            }
        }
    }
}

impl CcsdsSdlsDataPort for CcsdsSdlsFramer {
    fn invoke(
        &self,
        port_num: FwIndexType,
        status: SdlsStatus,
        data: Buffer,
        context: &FrameContext,
    ) {
        self.encrypt_in_handler(port_num, status, data, context);
    }
}

impl CmdPort for CcsdsSdlsFramer {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        args: &mut CmdArgBuffer,
    ) {
        self.handle_command(op_code, cmd_seq, args);
    }
}

input_port_adapter! {
    /// `dataIn` — SYNC `Svc.ComDataWithContext`: data to frame.
    component: CcsdsSdlsFramer;
    adapter: DataInAdapter;
    port: ComDataWithContextPort;
    input: pub data_in;
    handler: data_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

input_port_adapter! {
    /// `dataReturnIn` — SYNC: frames sent on `dataOut` coming back.
    component: CcsdsSdlsFramer;
    adapter: DataReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub data_return_in;
    handler: data_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

input_port_adapter! {
    /// `bufferReturnIn` — SYNC: the data buffer sent on `encryptOut`
    /// coming back from the helper.
    component: CcsdsSdlsFramer;
    adapter: BufferReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub buffer_return_in;
    handler: buffer_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

// ---------------------------------------------------------------------------
// Tests — port of CcsdsSdlsFramerTester (Rules/Frame.cpp, Rules/DataFlow.cpp)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{CmdResponsePort, LogPort, PrmGetPort, PrmSetPort};
    use fprime_fw::{LogBuffer, Time};
    use std::sync::Mutex as StdMutex;

    const TEST_BUFFER_SIZE: usize = 64;
    const TEST_PARAM_SA_INDEX: u16 = 0x1234;

    #[derive(Default)]
    struct Harness {
        encrypt_out: StdMutex<Vec<(u16, Vec<u8>, FrameContext)>>,
        encrypt_return: StdMutex<Vec<(Vec<u8>, u32, FrameContext)>>,
        data_out: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        data_return: StdMutex<Vec<(Vec<u8>, u32, FrameContext)>>,
        status: StdMutex<Vec<Success>>,
        allocs: StdMutex<Vec<FwSizeType>>,
        deallocs: StdMutex<Vec<u32>>,
        events: StdMutex<Vec<(FwEventIdType, Vec<u8>)>>,
        responses: StdMutex<Vec<(FwOpcodeType, CmdResponse)>>,
        prm_sets: StdMutex<Vec<(FwPrmIdType, Vec<u8>)>>,
        /// 0 = normal, 1 = invalid, 2 = undersized.
        alloc_mode: StdMutex<u8>,
        prm_value: StdMutex<Option<u16>>,
    }

    impl CcsdsSdlsEncryptionPort for Harness {
        fn invoke(&self, _p: FwIndexType, sa: u16, data: Buffer, context: &FrameContext) {
            self.encrypt_out
                .lock()
                .unwrap()
                .push((sa, data.data().to_vec(), *context));
        }
    }

    impl ComDataWithContextPort for Harness {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            match port_num {
                0 => self
                    .data_out
                    .lock()
                    .unwrap()
                    .push((data.data().to_vec(), *context)),
                1 => self.data_return.lock().unwrap().push((
                    data.data().to_vec(),
                    data.context(),
                    *context,
                )),
                _ => self.encrypt_return.lock().unwrap().push((
                    data.data().to_vec(),
                    data.context(),
                    *context,
                )),
            }
        }
    }

    impl SuccessConditionPort for Harness {
        fn invoke(&self, _p: FwIndexType, condition: &mut Success) {
            self.status.lock().unwrap().push(*condition);
        }
    }

    impl BufferGetPort for Harness {
        fn invoke(&self, _p: FwIndexType, size: FwSizeType) -> Buffer {
            self.allocs.lock().unwrap().push(size);
            match *self.alloc_mode.lock().unwrap() {
                1 => Buffer::empty(),
                2 => {
                    let mut b = Buffer::allocate(size as usize - 1);
                    b.set_context(99);
                    b
                }
                _ => {
                    let mut b = Buffer::allocate(size as usize);
                    b.set_context(42);
                    b
                }
            }
        }
    }

    impl BufferSendPort for Harness {
        fn invoke(&self, _p: FwIndexType, buffer: Buffer) {
            self.deallocs.lock().unwrap().push(buffer.context());
        }
    }

    impl LogPort for Harness {
        fn invoke(
            &self,
            _p: FwIndexType,
            id: FwEventIdType,
            _t: &mut Time,
            _s: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.events
                .lock()
                .unwrap()
                .push((id, args.as_slice().to_vec()));
        }
    }

    impl CmdResponsePort for Harness {
        fn invoke(&self, _p: FwIndexType, op_code: FwOpcodeType, _seq: u32, response: CmdResponse) {
            self.responses.lock().unwrap().push((op_code, response));
        }
    }

    impl PrmGetPort for Harness {
        fn invoke(&self, _p: FwIndexType, _id: FwPrmIdType, val: &mut ParamBuffer) -> ParamValid {
            match *self.prm_value.lock().unwrap() {
                Some(v) => {
                    assert!(val.serialize_u16_be(v).is_ok());
                    ParamValid::Valid
                }
                None => ParamValid::Invalid,
            }
        }
    }

    impl PrmSetPort for Harness {
        fn invoke(&self, _p: FwIndexType, id: FwPrmIdType, val: &mut ParamBuffer) {
            self.prm_sets
                .lock()
                .unwrap()
                .push((id, val.as_slice().to_vec()));
        }
    }

    /// The tester: stage `SA_INDEX` and load it, as the C++ tester does.
    fn build() -> (Arc<CcsdsSdlsFramer>, Arc<Harness>) {
        let h = Arc::new(Harness::default());
        *h.prm_value.lock().unwrap() = Some(TEST_PARAM_SA_INDEX);
        let c = CcsdsSdlsFramer::new("CcsdsSdlsFramer");
        c.data_out.connect(h.clone(), 0);
        c.data_return_out.connect(h.clone(), 1);
        c.encrypt_return_out.connect(h.clone(), 2);
        c.encrypt_out.connect(h.clone(), 0);
        c.com_status_out.connect(h.clone(), 0);
        c.buffer_allocate.connect(h.clone(), 0);
        c.buffer_deallocate.connect(h.clone(), 0);
        c.evt.log_out.connect(h.clone(), 0);
        c.cmd.cmd_response_out.connect(h.clone(), 0);
        c.prm.prm_get_out.connect(h.clone(), 0);
        c.prm.prm_set_out.connect(h.clone(), 0);
        c.load_parameters();
        (c, h)
    }

    fn buf(bytes: &[u8], ctx: u32) -> Buffer {
        let mut b = Buffer::allocate(bytes.len());
        b.data_mut().copy_from_slice(bytes);
        b.set_context(ctx);
        b
    }

    fn data_in(c: &Arc<CcsdsSdlsFramer>, data: Buffer, context: &FrameContext) {
        let p = c.data_in(0);
        p.target.invoke(p.port_num, data, context);
    }

    fn encrypt_in(
        c: &Arc<CcsdsSdlsFramer>,
        status: SdlsStatus,
        data: Buffer,
        context: &FrameContext,
    ) {
        let p = c.encrypt_in(0);
        p.target.invoke(p.port_num, status, data, context);
    }

    /// Frame.ContextSa: the context's SA index reaches the helper.
    #[test]
    fn frame_context_sa() {
        let (c, h) = build();
        for sa in [0u16, 7, 0xFFFE] {
            let context = FrameContext {
                sa_index: sa,
                ..FrameContext::default()
            };
            data_in(&c, buf(&[1, 2, 3], 0), &context);
            let out = h.encrypt_out.lock().unwrap().pop().unwrap();
            assert_eq!(out.0, sa);
            assert_eq!(out.2.sa_index, sa);
        }
        assert!(h.events.lock().unwrap().is_empty());
        assert!(h.data_return.lock().unwrap().is_empty());
    }

    /// Frame.ParameterSa: an unset context SA index falls back to the
    /// parameter.
    #[test]
    fn frame_parameter_sa() {
        let (c, h) = build();
        let context = FrameContext::default();
        data_in(&c, buf(&[9; 10], 0), &context);
        let out = h.encrypt_out.lock().unwrap()[0].clone();
        assert_eq!(out.0, TEST_PARAM_SA_INDEX);
        assert_eq!(out.2.sa_index, TEST_PARAM_SA_INDEX);
        assert_eq!(out.1, vec![9; 10]);
        assert!(h.events.lock().unwrap().is_empty());
    }

    /// Frame.EncryptFailure.
    #[test]
    fn frame_encrypt_failure() {
        for status in [
            SdlsStatus::UnknownSa,
            SdlsStatus::UnknownPort,
            SdlsStatus::EncryptionFailure,
        ] {
            let (c, h) = build();
            let context = FrameContext::default();
            encrypt_in(&c, status, buf(&[0; TEST_BUFFER_SIZE], 5), &context);
            assert_eq!(
                *h.events.lock().unwrap(),
                vec![(EVENTID_ENCRYPTION_FAILED, vec![status as u8])]
            );
            assert!(h.allocs.lock().unwrap().is_empty());
            assert!(h.data_out.lock().unwrap().is_empty());
            let returned = h.encrypt_return.lock().unwrap();
            assert_eq!(returned.len(), 1);
            assert_eq!(returned[0].1, 5);
            assert!(h.data_return.lock().unwrap().is_empty());
            assert_eq!(*h.status.lock().unwrap(), vec![Success::Success]);
        }
    }

    /// DataFlow.EncryptedData: SA index (big-endian) + encrypted data.
    #[test]
    fn data_flow_encrypted_data() {
        let (c, h) = build();
        let sa = 0xA1B2;
        let data: Vec<u8> = (0..TEST_BUFFER_SIZE as u8).collect();
        let context = FrameContext {
            sa_index: sa,
            ..FrameContext::default()
        };
        encrypt_in(&c, SdlsStatus::Success, buf(&data, 5), &context);
        assert_eq!(
            *h.allocs.lock().unwrap(),
            vec![(data.len() + 2) as FwSizeType]
        );
        let out = h.data_out.lock().unwrap();
        assert_eq!(out.len(), 1);
        let mut expected = vec![0xA1, 0xB2];
        expected.extend_from_slice(&data);
        assert_eq!(out[0].0, expected);
        let returned = h.encrypt_return.lock().unwrap();
        assert_eq!(returned.len(), 1);
        assert_eq!((returned[0].0.clone(), returned[0].1), (data, 5));
        assert!(h.events.lock().unwrap().is_empty());
        assert!(h.deallocs.lock().unwrap().is_empty());
    }

    /// DataFlow.AllocationFailure: invalid and undersized allocations.
    #[test]
    fn data_flow_allocation_failure() {
        for (mode, deallocated) in [(1u8, 0usize), (2, 1)] {
            let (c, h) = build();
            *h.alloc_mode.lock().unwrap() = mode;
            encrypt_in(
                &c,
                SdlsStatus::Success,
                buf(&[0; TEST_BUFFER_SIZE], 5),
                &FrameContext::default(),
            );
            let size = (TEST_BUFFER_SIZE + 2) as u64;
            assert_eq!(
                *h.events.lock().unwrap(),
                vec![(
                    EVENTID_BUFFER_ALLOCATION_FAILED,
                    size.to_be_bytes().to_vec()
                )]
            );
            assert!(h.data_out.lock().unwrap().is_empty());
            assert_eq!(h.deallocs.lock().unwrap().len(), deallocated);
            assert_eq!(h.encrypt_return.lock().unwrap().len(), 1);
            assert_eq!(*h.status.lock().unwrap(), vec![Success::Success]);
        }
    }

    /// DataFlow.DataReturn: the frame buffer is deallocated.
    #[test]
    fn data_flow_data_return() {
        let (c, h) = build();
        let p = c.data_return_in(0);
        p.target
            .invoke(p.port_num, buf(&[0; 8], 77), &FrameContext::default());
        assert_eq!(*h.deallocs.lock().unwrap(), vec![77]);
    }

    /// DataFlow.BufferReturn: the original data buffer goes upstream.
    #[test]
    fn data_flow_buffer_return() {
        let (c, h) = build();
        let context = FrameContext::default();
        let p = c.buffer_return_in(0);
        p.target.invoke(p.port_num, buf(&[3; 8], 13), &context);
        assert_eq!(
            *h.data_return.lock().unwrap(),
            vec![(vec![3; 8], 13, context)]
        );
    }

    /// DataFlow.ComStatus: pass-through, both values.
    #[test]
    fn data_flow_com_status() {
        let (c, h) = build();
        let p = c.com_status_in(0);
        p.target.invoke(p.port_num, &mut Success::Success);
        p.target.invoke(p.port_num, &mut Success::Failure);
        assert_eq!(
            *h.status.lock().unwrap(),
            vec![Success::Success, Success::Failure]
        );
    }

    /// The FPP default applies when the database has no value.
    #[test]
    fn parameter_defaults_to_one() {
        let (c, h) = build();
        *h.prm_value.lock().unwrap() = None;
        c.load_parameters();
        assert_eq!(c.param_get_sa_index(), (1, ParamValid::Default));
        data_in(&c, buf(&[1], 0), &FrameContext::default());
        assert_eq!(h.encrypt_out.lock().unwrap()[0].0, 1);
    }

    /// Reading the parameter before loading asserts (C++ FW_PARAM_OK).
    #[test]
    #[should_panic(expected = "Assert:")]
    fn unloaded_parameter_asserts() {
        let h = Arc::new(Harness::default());
        let c = CcsdsSdlsFramer::new("CcsdsSdlsFramer");
        c.encrypt_out.connect(h.clone(), 0);
        data_in(&c, buf(&[1], 0), &FrameContext::default());
    }

    /// The implicit parameter commands.
    #[test]
    fn sa_index_prm_set_and_save() {
        let (c, h) = build();
        c.base.set_id_base(0x100);
        let p = c.cmd_in(0);
        let mut args = CmdArgBuffer::new();
        assert!(args.serialize_u16_be(3).is_ok());
        p.target.invoke(p.port_num, 0x100, 1, &mut args);
        assert_eq!(c.param_get_sa_index(), (3, ParamValid::Valid));
        let mut args = CmdArgBuffer::new();
        p.target.invoke(p.port_num, 0x101, 2, &mut args);
        assert_eq!(*h.prm_sets.lock().unwrap(), vec![(0x100, vec![0, 3])]);
        let mut args = CmdArgBuffer::new();
        p.target.invoke(p.port_num, 0x100, 3, &mut args);
        let mut args = CmdArgBuffer::new();
        p.target.invoke(p.port_num, 0x102, 4, &mut args);
        assert_eq!(
            *h.responses.lock().unwrap(),
            vec![
                (0x100, CmdResponse::Ok),
                (0x101, CmdResponse::Ok),
                (0x100, CmdResponse::FormatError),
                (0x102, CmdResponse::InvalidOpcode),
            ]
        );
    }
}
