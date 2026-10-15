//! # CcsdsSdlsDeframer — port of `Svc::Ccsds::CcsdsSdlsDeframer` (passive)
//!
//! C++ sources: `Svc/Ccsds/CcsdsSdlsDeframer/CcsdsSdlsDeframer.{cpp,hpp,fpp}`
//! and its `docs/sdd.md`.
//!
//! Deframes CCSDS SDLS frames: a deframer in the uplink pipeline
//! (`Svc.Deframer`) and a client of an SDLS decryption helper
//! (`Svc.Ccsds.CcsdsSdlsDecryptClient`, typically an `SdlsSaRouter`):
//!
//! 1. A frame shorter than the 16-bit SA index raises `InsufficientLength`
//!    and is returned on `dataReturnOut` with the original context.
//! 2. Otherwise the big-endian SA index is read, recorded in a context
//!    copy, stripped from the buffer (`advance(2)` — the same allocation),
//!    and the rest goes to the helper on `decryptOut`.
//! 3. On `decryptIn` a non-SUCCESS status raises `DecryptionFailed`,
//!    notifies `errorNotify` (when connected) with `SDLS_DECRYPTION_FAILURE`
//!    and returns the buffer on `decryptReturnOut`; success forwards the
//!    data on `dataOut`.
//!
//! Buffers returning on `dataReturnIn` go back to the helper on
//! `decryptReturnOut`; the frame buffers the helper returns on
//! `bufferReturnIn` go upstream on `dataReturnOut`. `dataIn` is guarded
//! (the component mutex is held across the call, C++ parity).

use crate::ccsds::types::{
    CcsdsSdlsDataPort, CcsdsSdlsEncryptionPort, ErrorNotifyPort, FrameError, SdlsStatus,
};
use fprime_comp::{
    ComDataWithContextPort, EventGlue, OutputPort, PassiveBase, PortRef, input_port_adapter,
};
use fprime_config::{FwEventIdType, FwIndexType, FwSignedSizeType};
use fprime_fw::{
    Buffer, Endianness, FrameContext, LogSeverity, SerBuf, Serialize, SerializeStatus, fw_assert,
};
use std::sync::{Arc, Mutex};

/// `InsufficientLength()` — WARNING_HI.
pub const EVENTID_INSUFFICIENT_LENGTH: FwEventIdType = 0;
/// `DecryptionFailed(status: Svc.Ccsds.SdlsStatus)` — WARNING_HI.
pub const EVENTID_DECRYPTION_FAILED: FwEventIdType = 1;

/// Size of the leading SA index.
const SA_INDEX_SIZE: usize = size_of::<u16>();

/// `Svc::Ccsds::CcsdsSdlsDeframer` — passive SDLS deframer.
pub struct CcsdsSdlsDeframer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports and the time port.
    pub evt: EventGlue,
    /// `dataOut` — decrypted data downstream.
    pub data_out: OutputPort<dyn ComDataWithContextPort>,
    /// `dataReturnOut` — frame buffers back upstream.
    pub data_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `decryptOut` — the SA index and payload to the decryption helper.
    pub decrypt_out: OutputPort<dyn CcsdsSdlsEncryptionPort>,
    /// `decryptReturnOut` — decrypted buffers back to the helper.
    pub decrypt_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `errorNotify` — optional deframing-error notification.
    pub error_notify: OutputPort<dyn ErrorNotifyPort>,
    guard: Mutex<()>,
}

impl CcsdsSdlsDeframer {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            data_out: OutputPort::new(),
            data_return_out: OutputPort::new(),
            decrypt_out: OutputPort::new(),
            decrypt_return_out: OutputPort::new(),
            error_notify: OutputPort::new(),
            guard: Mutex::new(()),
        })
    }

    /// `decryptIn` — SYNC `Svc.Ccsds.CcsdsSdlsData`: the helper's result.
    pub fn decrypt_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn CcsdsSdlsDataPort> {
        PortRef::new(self.clone(), port_num)
    }

    fn data_in_handler(&self, _port_num: FwIndexType, mut data: Buffer, context: &FrameContext) {
        let _guard = self.guard.lock().unwrap();
        if data.size() < SA_INDEX_SIZE {
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_INSUFFICIENT_LENGTH,
                LogSeverity::WarningHi,
                "Failed to deframe security association index: insufficient length",
                |_buf| SerializeStatus::Ok,
            );
            let p = self.data_return_out.get();
            p.target.invoke(p.port_num, data, context);
            return;
        }
        let mut sa_index = 0u16;
        {
            let mut de = data.get_deserializer();
            let status = de.deserialize_u16_be(&mut sa_index);
            fw_assert!(status.is_ok(), status as i32);
        }
        let new_context = FrameContext {
            sa_index,
            ..*context
        };
        // The helper receives only the IV/data.
        data.advance(SA_INDEX_SIZE as FwSignedSizeType);
        let p = self.decrypt_out.get();
        p.target.invoke(p.port_num, sa_index, data, &new_context);
    }

    fn data_return_in_handler(&self, _port_num: FwIndexType, data: Buffer, context: &FrameContext) {
        // Back to the decryption subsystem, which owns decrypted buffers.
        let p = self.decrypt_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    fn buffer_return_in_handler(
        &self,
        _port_num: FwIndexType,
        data: Buffer,
        context: &FrameContext,
    ) {
        // The helper returned the original frame buffer: send it upstream.
        let p = self.data_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    fn decrypt_in_handler(
        &self,
        _port_num: FwIndexType,
        status: SdlsStatus,
        data: Buffer,
        context: &FrameContext,
    ) {
        if status != SdlsStatus::Success {
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_DECRYPTION_FAILED,
                LogSeverity::WarningHi,
                &format!("Failed to decrypt frame: {status:?}"),
                |buf| status.serialize_to(buf, Endianness::Big),
            );
            if let Some(p) = self.error_notify.try_get() {
                p.target
                    .invoke(p.port_num, FrameError::SdlsDecryptionFailure);
            }
            let p = self.decrypt_return_out.get();
            p.target.invoke(p.port_num, data, context);
        } else {
            let p = self.data_out.get();
            p.target.invoke(p.port_num, data, context);
        }
    }
}

impl CcsdsSdlsDataPort for CcsdsSdlsDeframer {
    fn invoke(
        &self,
        port_num: FwIndexType,
        status: SdlsStatus,
        data: Buffer,
        context: &FrameContext,
    ) {
        self.decrypt_in_handler(port_num, status, data, context);
    }
}

input_port_adapter! {
    /// `dataIn` — GUARDED `Svc.ComDataWithContext`: one SDLS frame.
    component: CcsdsSdlsDeframer;
    adapter: DataInAdapter;
    port: ComDataWithContextPort;
    input: pub data_in;
    handler: data_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

input_port_adapter! {
    /// `dataReturnIn` — SYNC: buffers sent on `dataOut` coming back.
    component: CcsdsSdlsDeframer;
    adapter: DataReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub data_return_in;
    handler: data_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

input_port_adapter! {
    /// `bufferReturnIn` — SYNC: the IV/data buffer sent on `decryptOut`
    /// coming back from the helper.
    component: CcsdsSdlsDeframer;
    adapter: BufferReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub buffer_return_in;
    handler: buffer_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

// ---------------------------------------------------------------------------
// Tests — port of CcsdsSdlsDeframerTester (Rules/Deframe.cpp, DataFlow.cpp)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::LogPort;
    use fprime_fw::{LogBuffer, Time};
    use std::sync::Mutex as StdMutex;

    type DecryptRec = (u16, Vec<u8>, usize, FrameContext);

    #[derive(Default)]
    struct Harness {
        decrypt_out: StdMutex<Vec<DecryptRec>>,
        data_out: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        data_return: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        decrypt_return: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        errors: StdMutex<Vec<FrameError>>,
        events: StdMutex<Vec<(FwEventIdType, Vec<u8>)>>,
    }

    impl CcsdsSdlsEncryptionPort for Harness {
        fn invoke(&self, _p: FwIndexType, sa: u16, data: Buffer, context: &FrameContext) {
            self.decrypt_out.lock().unwrap().push((
                sa,
                data.data().to_vec(),
                data.offset(),
                *context,
            ));
        }
    }

    impl ComDataWithContextPort for Harness {
        fn invoke(&self, port_num: FwIndexType, data: Buffer, context: &FrameContext) {
            let rec = (data.data().to_vec(), *context);
            match port_num {
                0 => self.data_out.lock().unwrap().push(rec),
                1 => self.data_return.lock().unwrap().push(rec),
                _ => self.decrypt_return.lock().unwrap().push(rec),
            }
        }
    }

    impl ErrorNotifyPort for Harness {
        fn invoke(&self, _p: FwIndexType, error_code: FrameError) {
            self.errors.lock().unwrap().push(error_code);
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

    fn build() -> (Arc<CcsdsSdlsDeframer>, Arc<Harness>) {
        let h = Arc::new(Harness::default());
        let c = CcsdsSdlsDeframer::new("CcsdsSdlsDeframer");
        c.data_out.connect(h.clone(), 0);
        c.data_return_out.connect(h.clone(), 1);
        c.decrypt_return_out.connect(h.clone(), 2);
        c.decrypt_out.connect(h.clone(), 0);
        c.error_notify.connect(h.clone(), 0);
        c.evt.log_out.connect(h.clone(), 0);
        (c, h)
    }

    fn buf(bytes: &[u8]) -> Buffer {
        let mut b = Buffer::allocate(bytes.len().max(1));
        b.data_mut()[..bytes.len()].copy_from_slice(bytes);
        b.set_size(bytes.len());
        b
    }

    /// Deframe.Nominal: the SA index is read, recorded and stripped.
    #[test]
    fn deframe_nominal() {
        for payload_len in [0usize, 1, 40] {
            let (c, h) = build();
            let payload: Vec<u8> = (0..payload_len as u8).map(|b| b ^ 0x5A).collect();
            let mut frame = vec![0xC3, 0x21];
            frame.extend_from_slice(&payload);
            let p = c.data_in(0);
            p.target
                .invoke(p.port_num, buf(&frame), &FrameContext::default());
            let out = h.decrypt_out.lock().unwrap();
            assert_eq!(out.len(), 1);
            assert_eq!(out[0].0, 0xC321);
            assert_eq!(out[0].3.sa_index, 0xC321);
            assert_eq!(out[0].1, payload);
            // Same allocation, advanced past the SA index.
            assert_eq!(out[0].2, 2);
            assert!(h.events.lock().unwrap().is_empty());
            assert!(h.errors.lock().unwrap().is_empty());
            assert!(h.data_return.lock().unwrap().is_empty());
        }
    }

    /// Deframe.InsufficientLength.
    #[test]
    fn deframe_insufficient_length() {
        for len in [0usize, 1] {
            let (c, h) = build();
            let context = FrameContext {
                vc_id: 3,
                ..FrameContext::default()
            };
            let p = c.data_in(0);
            p.target.invoke(p.port_num, buf(&vec![7; len]), &context);
            assert_eq!(
                *h.events.lock().unwrap(),
                vec![(EVENTID_INSUFFICIENT_LENGTH, vec![])]
            );
            assert!(h.decrypt_out.lock().unwrap().is_empty());
            assert_eq!(
                *h.data_return.lock().unwrap(),
                vec![(vec![7; len], context)]
            );
        }
    }

    /// Deframe.DecryptFailure.
    #[test]
    fn deframe_decrypt_failure() {
        for status in [
            SdlsStatus::UnknownSa,
            SdlsStatus::UnknownPort,
            SdlsStatus::DecryptionFailure,
            SdlsStatus::MacVerificationFailure,
        ] {
            let (c, h) = build();
            let context = FrameContext::default();
            let p = c.decrypt_in(0);
            p.target.invoke(p.port_num, status, buf(&[1; 16]), &context);
            assert_eq!(
                *h.events.lock().unwrap(),
                vec![(EVENTID_DECRYPTION_FAILED, vec![status as u8])]
            );
            assert_eq!(
                *h.errors.lock().unwrap(),
                vec![FrameError::SdlsDecryptionFailure]
            );
            assert!(h.data_out.lock().unwrap().is_empty());
            assert_eq!(
                *h.decrypt_return.lock().unwrap(),
                vec![(vec![1; 16], context)]
            );
            assert!(h.data_return.lock().unwrap().is_empty());
        }
    }

    /// `errorNotify` is optional.
    #[test]
    fn decrypt_failure_without_error_notify() {
        let h = Arc::new(Harness::default());
        let c = CcsdsSdlsDeframer::new("CcsdsSdlsDeframer");
        c.decrypt_return_out.connect(h.clone(), 2);
        let p = c.decrypt_in(0);
        p.target.invoke(
            p.port_num,
            SdlsStatus::DecryptionFailure,
            buf(&[1]),
            &FrameContext::default(),
        );
        assert_eq!(h.decrypt_return.lock().unwrap().len(), 1);
    }

    /// DataFlow.DecryptedData.
    #[test]
    fn data_flow_decrypted_data() {
        let (c, h) = build();
        let context = FrameContext::default();
        let p = c.decrypt_in(0);
        p.target
            .invoke(p.port_num, SdlsStatus::Success, buf(&[4; 32]), &context);
        assert_eq!(*h.data_out.lock().unwrap(), vec![(vec![4; 32], context)]);
    }

    /// DataFlow.DataReturn.
    #[test]
    fn data_flow_data_return() {
        let (c, h) = build();
        let context = FrameContext::default();
        let p = c.data_return_in(0);
        p.target.invoke(p.port_num, buf(&[5; 8]), &context);
        assert_eq!(
            *h.decrypt_return.lock().unwrap(),
            vec![(vec![5; 8], context)]
        );
    }

    /// DataFlow.BufferReturn.
    #[test]
    fn data_flow_buffer_return() {
        let (c, h) = build();
        let context = FrameContext::default();
        let p = c.buffer_return_in(0);
        p.target.invoke(p.port_num, buf(&[6; 8]), &context);
        assert_eq!(*h.data_return.lock().unwrap(), vec![(vec![6; 8], context)]);
    }
}
