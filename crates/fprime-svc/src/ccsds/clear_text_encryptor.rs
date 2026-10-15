//! # ClearTextEncryptor — port of `Svc::Ccsds::ClearTextEncryptor` (passive)
//!
//! C++ sources: `Svc/Ccsds/ClearTextEncryptor/ClearTextEncryptor.{cpp,hpp,fpp}`.
//!
//! **WARNING: this component provides NO security** — no confidentiality,
//! no integrity, no authentication. It implements
//! `Svc.Ccsds.CcsdsSdlsEncrypt` by passing buffers and contexts through
//! unmodified with status `SUCCESS`, for clear-mode operation and testing.
//! It is the default encryptor of the `ComCcsdsSdls` subtopology, reached
//! through the `SdlsSaRouter` `PLAINTEXT` port. Every frame it handles
//! raises `NullCipherInUse` (WARNING_HI, throttled at 5) so that its
//! presence in a configuration is visible.
//!
//! `encryptIn` is guarded (a component mutex held across the call, C++
//! parity); `encryptReturnIn` is sync.

use crate::ccsds::types::{CcsdsSdlsDataPort, CcsdsSdlsEncryptionPort, SdlsStatus};
use fprime_comp::{
    ComDataWithContextPort, EventGlue, EventThrottle, OutputPort, PassiveBase, PortRef,
    input_port_adapter,
};
use fprime_config::{FwEventIdType, FwIndexType};
use fprime_fw::{Buffer, FrameContext, LogSeverity, SerBuf};
use std::sync::{Arc, Mutex};

/// `NullCipherInUse(securityAssociationIndex: U16)` — WARNING_HI,
/// throttle 5.
pub const EVENTID_NULL_CIPHER_IN_USE: FwEventIdType = 0;

/// Throttle limit of `NullCipherInUse`.
pub const NULL_CIPHER_IN_USE_THROTTLE: u32 = 5;

/// `Svc::Ccsds::ClearTextEncryptor` — the null cipher.
pub struct ClearTextEncryptor {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports and the time port.
    pub evt: EventGlue,
    /// `encryptOut` — the status and (unmodified) data.
    pub encrypt_out: OutputPort<dyn CcsdsSdlsDataPort>,
    /// `bufferReturnOut` — returned buffers back for deallocation.
    pub buffer_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `NullCipherInUse` throttle.
    pub null_cipher_throttle: EventThrottle,
    guard: Mutex<()>,
}

impl ClearTextEncryptor {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            encrypt_out: OutputPort::new(),
            buffer_return_out: OutputPort::new(),
            null_cipher_throttle: EventThrottle::new(NULL_CIPHER_IN_USE_THROTTLE),
            guard: Mutex::new(()),
        })
    }

    /// `encryptIn` — GUARDED `Svc.Ccsds.CcsdsSdlsEncryption` input.
    pub fn encrypt_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn CcsdsSdlsEncryptionPort> {
        PortRef::new(self.clone(), port_num)
    }

    fn encrypt_in_handler(
        &self,
        _port_num: FwIndexType,
        security_association_index: u16,
        data: Buffer,
        context: &FrameContext,
    ) {
        let _guard = self.guard.lock().unwrap();
        // Make the absence of security visible: a flight configuration must
        // never route traffic here.
        if self.null_cipher_throttle.ok_to_emit() {
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_NULL_CIPHER_IN_USE,
                LogSeverity::WarningHi,
                &format!(
                    "ClearTextEncryptor handled a frame on SA {security_association_index} with NO \
                     authentication or encryption"
                ),
                |buf| buf.serialize_u16_be(security_association_index),
            );
        }
        let p = self.encrypt_out.get();
        p.target
            .invoke(p.port_num, SdlsStatus::Success, data, context);
    }

    fn encrypt_return_in_handler(
        &self,
        _port_num: FwIndexType,
        data: Buffer,
        context: &FrameContext,
    ) {
        let p = self.buffer_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }
}

impl CcsdsSdlsEncryptionPort for ClearTextEncryptor {
    fn invoke(
        &self,
        port_num: FwIndexType,
        security_association_index: u16,
        data: Buffer,
        context: &FrameContext,
    ) {
        self.encrypt_in_handler(port_num, security_association_index, data, context);
    }
}

input_port_adapter! {
    /// `encryptReturnIn` — SYNC: ownership of a buffer sent on `encryptOut`
    /// coming back.
    component: ClearTextEncryptor;
    adapter: EncryptReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub encrypt_return_in;
    handler: encrypt_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::LogPort;
    use fprime_fw::{LogBuffer, Time};
    use std::sync::Mutex as StdMutex;

    type OutRec = (SdlsStatus, Vec<u8>, u32, FrameContext);

    #[derive(Default)]
    struct Harness {
        out: StdMutex<Vec<OutRec>>,
        returned: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        events: StdMutex<Vec<(FwEventIdType, Vec<u8>)>>,
    }

    impl CcsdsSdlsDataPort for Harness {
        fn invoke(
            &self,
            _p: FwIndexType,
            status: SdlsStatus,
            data: Buffer,
            context: &FrameContext,
        ) {
            self.out
                .lock()
                .unwrap()
                .push((status, data.data().to_vec(), data.context(), *context));
        }
    }

    impl ComDataWithContextPort for Harness {
        fn invoke(&self, _p: FwIndexType, data: Buffer, context: &FrameContext) {
            self.returned
                .lock()
                .unwrap()
                .push((data.data().to_vec(), *context));
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

    fn build() -> (Arc<ClearTextEncryptor>, Arc<Harness>) {
        let h = Arc::new(Harness::default());
        let c = ClearTextEncryptor::new("ClearTextEncryptor");
        c.encrypt_out.connect(h.clone(), 0);
        c.buffer_return_out.connect(h.clone(), 0);
        c.evt.log_out.connect(h.clone(), 0);
        (c, h)
    }

    /// Upstream `testEncryptPassThrough`: buffer and context unmodified,
    /// SUCCESS, one `NullCipherInUse(sa)` event.
    #[test]
    fn encrypt_pass_through() {
        let (c, h) = build();
        let bytes: Vec<u8> = (0..64u8).map(|i| i.wrapping_mul(37)).collect();
        let mut buffer = Buffer::allocate(bytes.len());
        buffer.data_mut().copy_from_slice(&bytes);
        buffer.set_context(77);
        let context = FrameContext {
            vc_id: 0xA5,
            ..FrameContext::default()
        };
        let p = c.encrypt_in(0);
        p.target.invoke(p.port_num, 0xBEEF, buffer, &context);
        let out = h.out.lock().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0], (SdlsStatus::Success, bytes, 77, context));
        assert!(h.returned.lock().unwrap().is_empty());
        assert_eq!(
            *h.events.lock().unwrap(),
            vec![(EVENTID_NULL_CIPHER_IN_USE, vec![0xBE, 0xEF])]
        );
    }

    /// Upstream `testReturnPassThrough`.
    #[test]
    fn return_pass_through() {
        let (c, h) = build();
        let context = FrameContext::default();
        let p = c.encrypt_return_in(0);
        p.target.invoke(p.port_num, Buffer::allocate(8), &context);
        assert_eq!(*h.returned.lock().unwrap(), vec![(vec![0u8; 8], context)]);
        assert!(h.out.lock().unwrap().is_empty());
    }

    /// The `NullCipherInUse` event is throttled after 5 emissions.
    #[test]
    fn null_cipher_event_is_throttled() {
        let (c, h) = build();
        for _ in 0..8 {
            let p = c.encrypt_in(0);
            p.target
                .invoke(p.port_num, 1, Buffer::allocate(1), &FrameContext::default());
        }
        assert_eq!(h.out.lock().unwrap().len(), 8);
        assert_eq!(h.events.lock().unwrap().len(), 5);
    }
}
