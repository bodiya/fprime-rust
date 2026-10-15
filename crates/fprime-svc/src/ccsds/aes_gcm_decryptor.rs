//! # AesGcmDecryptor — port of `Svc::Ccsds::AesGcmDecryptor` (passive;
//! feature `aes-gcm`)
//!
//! C++ sources: `Svc/Ccsds/AesGcmDecryptor/AesGcmDecryptor.{cpp,hpp,fpp}`
//! and its `docs/sdd.md` (CCSDS 355.0-B-2).
//!
//! Decrypts and authenticates uplink data under AES-256-GCM
//! (`Svc.Ccsds.CcsdsSdlsDecrypt` toward a decryption client,
//! `Svc.Ccsds.SdlsKeyInterfaceClient` toward a key source). The buffer is
//! `IV (12) | ciphertext | MAC (16)` (the SPI already stripped by
//! `CcsdsSdlsDeframer`):
//!
//! 1. Shorter than IV + MAC → `DECRYPTION_FAILURE`, before any key request
//!    (the uplink is attacker-influenced).
//! 2. Key failure or a key that is not 32 bytes → `KEY_ERROR`.
//! 3. The masked TC primary header + SPI + zeroed IV ([`SdlsTcAuthMask`],
//!    VC from the context, which `TcDeframer` sets) is the AAD; a MAC that
//!    does not verify → `MAC_VERIFICATION_FAILURE` (distinct from
//!    `DECRYPTION_FAILURE`); the component keeps decrypting later frames.
//! 4. Success decrypts in place and narrows the SAME buffer to the
//!    plaintext (`advance(12)`, `setSize(n)`), keeping its allocation
//!    context for the issuing `BufferManager`.
//!
//! Every outcome is one `decryptOut` with its status; there are no events.
//! `decryptReturnIn` passes buffers upstream on `bufferReturnOut`. There is
//! no anti-replay (as upstream: see its SDD's security considerations).
//!
//! ## Deviations (documented)
//!
//! - RustCrypto `aes-gcm` instead of OpenSSL EVP; the stack key copy is
//!   wiped with `zeroize` (C++ `OPENSSL_cleanse`).
//! - RustCrypto verifies the tag before decrypting, so a frame failing its
//!   MAC check is passed on with its ciphertext intact; OpenSSL decrypts in
//!   place first and leaves unauthenticated plaintext in the buffer. The
//!   status is the same and the client drops the buffer either way.

use crate::ccsds::aes_gcm_cipher::{AES_256_KEY_LEN, GCM_IV_LEN, GCM_TAG_LEN, open};
use crate::ccsds::sdls_auth_mask::SdlsTcAuthMask;
use crate::ccsds::types::{
    CcsdsSdlsDataPort, CcsdsSdlsEncryptionPort, SdlsKeyBuffer, SdlsKeyPort, SdlsStatus,
};
use fprime_comp::{ComDataWithContextPort, OutputPort, PassiveBase, PortRef, input_port_adapter};
use fprime_config::{FwIndexType, FwSignedSizeType};
use fprime_fw::{Buffer, FrameContext, SerBuf, SerBufAny};
use std::sync::{Arc, Mutex};
use zeroize::Zeroize;

/// The AAD cache (C++ `m_aad`, `m_aadVcId`, `m_aadSaIndex`), touched only
/// under the `decryptIn` guard.
struct AadCache {
    aad: SdlsTcAuthMask,
    vc_id: u8,
    sa_index: u16,
}

/// `Svc::Ccsds::AesGcmDecryptor` — AES-256-GCM SDLS decryptor.
pub struct AesGcmDecryptor {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `decryptOut` — the status and (on success) the plaintext.
    pub decrypt_out: OutputPort<dyn CcsdsSdlsDataPort>,
    /// `bufferReturnOut` — returned buffers upstream for deallocation.
    pub buffer_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `keyGet` — the AES-256 key for a security association.
    pub key_get: OutputPort<dyn SdlsKeyPort>,
    /// The `decryptIn` guard and the AAD cache it protects.
    guard: Mutex<AadCache>,
}

impl AesGcmDecryptor {
    /// Construct the component.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            decrypt_out: OutputPort::new(),
            buffer_return_out: OutputPort::new(),
            key_get: OutputPort::new(),
            guard: Mutex::new(AadCache {
                aad: SdlsTcAuthMask::new(0, 0),
                vc_id: 0,
                sa_index: 0,
            }),
        })
    }

    /// `decryptIn` — GUARDED `Svc.Ccsds.CcsdsSdlsEncryption`.
    pub fn decrypt_in(
        self: &Arc<Self>,
        port_num: FwIndexType,
    ) -> PortRef<dyn CcsdsSdlsEncryptionPort> {
        PortRef::new(self.clone(), port_num)
    }

    fn decrypt_in_handler(
        &self,
        _port_num: FwIndexType,
        security_association_index: u16,
        mut data: Buffer,
        context: &FrameContext,
    ) {
        let mut cache = self.guard.lock().unwrap();

        // Too short to hold an IV and a MAC.
        if data.size() < GCM_IV_LEN + GCM_TAG_LEN {
            self.decrypt_out(SdlsStatus::DecryptionFailure, data, context);
            return;
        }

        let mut key = SdlsKeyBuffer::new();
        let p = self.key_get.get();
        let key_status = p
            .target
            .invoke(p.port_num, security_association_index, &mut key);
        if key_status != SdlsStatus::Success || key.get_size() != AES_256_KEY_LEN {
            // A wrong-sized key would decrypt under the wrong material.
            key.bytes_mut().zeroize();
            self.decrypt_out(SdlsStatus::KeyError, data, context);
            return;
        }

        // Authenticated but not encrypted; the VC travels in the context.
        let vc_id = context.vc_id;
        if vc_id != cache.vc_id || security_association_index != cache.sa_index {
            cache.aad = SdlsTcAuthMask::new(vc_id, security_association_index);
            cache.vc_id = vc_id;
            cache.sa_index = security_association_index;
        }

        let cipher_len = data.size() - GCM_IV_LEN - GCM_TAG_LEN;
        let verified = {
            let frame = data.data_mut();
            let mut iv = [0u8; GCM_IV_LEN];
            iv.copy_from_slice(&frame[..GCM_IV_LEN]);
            let mut tag = [0u8; GCM_TAG_LEN];
            tag.copy_from_slice(&frame[GCM_IV_LEN + cipher_len..]);
            let ciphertext = &mut frame[GCM_IV_LEN..GCM_IV_LEN + cipher_len];
            open(key.as_slice(), &iv, &cache.aad.bytes, ciphertext, &tag)
        };
        // The cipher held the key schedule only for this call: wipe the
        // stack copy.
        key.bytes_mut().zeroize();
        if !verified {
            self.decrypt_out(SdlsStatus::MacVerificationFailure, data, context);
            return;
        }

        // Narrow the same buffer to the plaintext.
        data.advance(GCM_IV_LEN as FwSignedSizeType);
        data.set_size(cipher_len);
        self.decrypt_out(SdlsStatus::Success, data, context);
    }

    fn decrypt_return_in_handler(
        &self,
        _port_num: FwIndexType,
        data: Buffer,
        context: &FrameContext,
    ) {
        let p = self.buffer_return_out.get();
        p.target.invoke(p.port_num, data, context);
    }

    fn decrypt_out(&self, status: SdlsStatus, data: Buffer, context: &FrameContext) {
        let p = self.decrypt_out.get();
        p.target.invoke(p.port_num, status, data, context);
    }
}

impl CcsdsSdlsEncryptionPort for AesGcmDecryptor {
    fn invoke(
        &self,
        port_num: FwIndexType,
        security_association_index: u16,
        data: Buffer,
        context: &FrameContext,
    ) {
        self.decrypt_in_handler(port_num, security_association_index, data, context);
    }
}

input_port_adapter! {
    /// `decryptReturnIn` — SYNC: a buffer sent on `decryptOut` coming back.
    component: AesGcmDecryptor;
    adapter: DecryptReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub decrypt_return_in;
    handler: decrypt_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

// ---------------------------------------------------------------------------
// Tests — port of AesGcmDecryptorTester
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ccsds::aes_gcm_cipher::seal;
    use std::sync::Mutex as StdMutex;

    const TEST_VC_ID: u8 = 5;
    const TEST_SPI: u16 = 0x1234;
    const KAT_KEY: [u8; 32] = [
        0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E,
        0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D,
        0x5E, 0x5F,
    ];
    const KAT_IV: [u8; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0x0A, 0x0B];
    /// "SDLS AES-256-GCM KAT payload".
    const KAT_PLAINTEXT: [u8; 28] = *b"SDLS AES-256-GCM KAT payload";
    const KAT_CIPHERTEXT: [u8; 28] = [
        0x69, 0x41, 0x20, 0x2B, 0x57, 0xCD, 0x01, 0xF3, 0xEF, 0x20, 0xB5, 0xF4, 0x95, 0xAA, 0x8F,
        0xF6, 0xA3, 0x8F, 0x6C, 0x5D, 0x4C, 0x3C, 0xCC, 0x2A, 0x82, 0x4F, 0x89, 0xF2,
    ];
    /// The MAC over the above with the TC AAD for VC 5, SPI 0x1234.
    const KAT_MAC: [u8; 16] = [
        0xC4, 0xB0, 0x91, 0x03, 0x7F, 0xA7, 0xA4, 0xAB, 0xD7, 0x25, 0xCB, 0xA2, 0xE8, 0x24, 0x08,
        0xA6,
    ];

    /// The ground segment's TC AAD layout, built independently.
    fn build_tc_aad(vc_id: u8, spi: u16) -> [u8; 19] {
        let mut aad = [0u8; 19];
        aad[2] = (vc_id << 2) & 0xFC;
        aad[5] = (spi >> 8) as u8;
        aad[6] = spi as u8;
        aad
    }

    /// (status, data, offset, context)
    type OutRec = (SdlsStatus, Vec<u8>, usize, u32);

    #[derive(Default)]
    struct Harness {
        decrypt_out: StdMutex<Vec<OutRec>>,
        buffer_return: StdMutex<Vec<(Vec<u8>, FrameContext)>>,
        key_requests: StdMutex<Vec<u16>>,
        key: StdMutex<(Vec<u8>, SdlsStatus)>,
    }

    impl CcsdsSdlsDataPort for Harness {
        fn invoke(&self, _p: FwIndexType, status: SdlsStatus, data: Buffer, _c: &FrameContext) {
            self.decrypt_out.lock().unwrap().push((
                status,
                data.data().to_vec(),
                data.offset(),
                data.context(),
            ));
        }
    }

    impl ComDataWithContextPort for Harness {
        fn invoke(&self, _p: FwIndexType, data: Buffer, context: &FrameContext) {
            self.buffer_return
                .lock()
                .unwrap()
                .push((data.data().to_vec(), *context));
        }
    }

    impl SdlsKeyPort for Harness {
        fn invoke(&self, _p: FwIndexType, sa: u16, key: &mut SdlsKeyBuffer) -> SdlsStatus {
            self.key_requests.lock().unwrap().push(sa);
            let (bytes, status) = self.key.lock().unwrap().clone();
            if status != SdlsStatus::Success {
                assert!(key.set_buff_len(0).is_ok());
                return status;
            }
            key.bytes_mut()[..bytes.len()].copy_from_slice(&bytes);
            assert!(key.set_buff_len(bytes.len()).is_ok());
            SdlsStatus::Success
        }
    }

    fn build() -> (Arc<AesGcmDecryptor>, Arc<Harness>) {
        let h = Arc::new(Harness::default());
        *h.key.lock().unwrap() = (KAT_KEY.to_vec(), SdlsStatus::Success);
        let c = AesGcmDecryptor::new("AesGcmDecryptor");
        c.decrypt_out.connect(h.clone(), 0);
        c.buffer_return_out.connect(h.clone(), 0);
        c.key_get.connect(h.clone(), 0);
        (c, h)
    }

    /// `buildFrame`: IV | ciphertext | MAC under `vc_id` / `spi`.
    fn build_frame(plain: &[u8], vc_id: u8, spi: u16, iv_seed: u8) -> Vec<u8> {
        let iv: [u8; GCM_IV_LEN] = core::array::from_fn(|i| (i as u8).wrapping_mul(31) ^ iv_seed);
        let mut ct = plain.to_vec();
        let tag = seal(&KAT_KEY, &iv, &build_tc_aad(vc_id, spi), &mut ct).unwrap();
        let mut frame = iv.to_vec();
        frame.extend_from_slice(&ct);
        frame.extend_from_slice(&tag);
        frame
    }

    fn send(c: &Arc<AesGcmDecryptor>, h: &Harness, frame: &[u8], spi: u16, vc_id: u8, ctx: u32) {
        h.decrypt_out.lock().unwrap().clear();
        h.key_requests.lock().unwrap().clear();
        let mut b = Buffer::allocate(frame.len().max(1));
        b.data_mut()[..frame.len()].copy_from_slice(frame);
        b.set_size(frame.len());
        b.set_context(ctx);
        let context = FrameContext {
            vc_id,
            ..FrameContext::default()
        };
        let p = c.decrypt_in(0);
        p.target.invoke(p.port_num, spi, b, &context);
    }

    fn last(h: &Harness) -> OutRec {
        let out = h.decrypt_out.lock().unwrap();
        // Every path through decryptIn ends in exactly one decryptOut.
        assert_eq!(out.len(), 1);
        out[0].clone()
    }

    /// The upstream known-answer vector (VC 5 / SPI 0x1234).
    #[test]
    fn known_answer() {
        let (c, h) = build();
        let mut frame = KAT_IV.to_vec();
        frame.extend_from_slice(&KAT_CIPHERTEXT);
        frame.extend_from_slice(&KAT_MAC);
        send(&c, &h, &frame, TEST_SPI, TEST_VC_ID, 0);
        let (status, plain, _, _) = last(&h);
        assert_eq!(status, SdlsStatus::Success);
        assert_eq!(plain, KAT_PLAINTEXT);
        assert_eq!(*h.key_requests.lock().unwrap(), vec![TEST_SPI]);
    }

    #[test]
    fn auth_mask_layout() {
        for vc in [0u8, 1, TEST_VC_ID, 0x3F] {
            for spi in [0u16, 0x1234, 0xFFFF] {
                assert_eq!(SdlsTcAuthMask::new(vc, spi).bytes, build_tc_aad(vc, spi));
            }
        }
    }

    #[test]
    fn decrypt_nominal() {
        let (c, h) = build();
        let plain: Vec<u8> = (0..64u8).map(|b| b ^ 0xC3).collect();
        send(
            &c,
            &h,
            &build_frame(&plain, TEST_VC_ID, TEST_SPI, 1),
            TEST_SPI,
            TEST_VC_ID,
            0,
        );
        let (status, out, offset, _) = last(&h);
        assert_eq!(status, SdlsStatus::Success);
        assert_eq!(out, plain);
        // In place: the plaintext starts one IV into the same storage.
        assert_eq!(offset, GCM_IV_LEN);
    }

    #[test]
    fn empty_ciphertext() {
        let (c, h) = build();
        let frame = build_frame(&[], TEST_VC_ID, TEST_SPI, 2);
        assert_eq!(frame.len(), GCM_IV_LEN + GCM_TAG_LEN);
        send(&c, &h, &frame, TEST_SPI, TEST_VC_ID, 0);
        let (status, out, _, _) = last(&h);
        assert_eq!(status, SdlsStatus::Success);
        assert!(out.is_empty());
    }

    #[test]
    fn allocation_context_preserved() {
        let (c, h) = build();
        send(
            &c,
            &h,
            &build_frame(&[0xA5; 32], TEST_VC_ID, TEST_SPI, 3),
            TEST_SPI,
            TEST_VC_ID,
            0xBEEF,
        );
        let (status, _, offset, context) = last(&h);
        assert_eq!(status, SdlsStatus::Success);
        assert_eq!(context, 0xBEEF);
        assert_eq!(offset, GCM_IV_LEN);
    }

    fn tampered(index: usize) {
        let (c, h) = build();
        let mut frame = build_frame(&[0x5A; 48], TEST_VC_ID, TEST_SPI, 4);
        frame[index] ^= 0x01;
        send(&c, &h, &frame, TEST_SPI, TEST_VC_ID, 0);
        assert_eq!(last(&h).0, SdlsStatus::MacVerificationFailure);
    }

    #[test]
    fn tampered_ciphertext() {
        for i in [0, 17, 47] {
            tampered(GCM_IV_LEN + i);
        }
    }

    #[test]
    fn tampered_mac() {
        for i in [0, 9, 15] {
            tampered(GCM_IV_LEN + 48 + i);
        }
    }

    #[test]
    fn tampered_iv() {
        for i in [0, 6, 11] {
            tampered(i);
        }
    }

    #[test]
    fn wrong_vc_id() {
        let (c, h) = build();
        let frame = build_frame(&[0x11; 32], TEST_VC_ID + 1, TEST_SPI, 5);
        send(&c, &h, &frame, TEST_SPI, TEST_VC_ID, 0);
        assert_eq!(last(&h).0, SdlsStatus::MacVerificationFailure);
    }

    #[test]
    fn wrong_security_association() {
        let (c, h) = build();
        let frame = build_frame(&[0x22; 32], TEST_VC_ID, TEST_SPI, 6);
        send(&c, &h, &frame, TEST_SPI + 1, TEST_VC_ID, 0);
        assert_eq!(last(&h).0, SdlsStatus::MacVerificationFailure);
    }

    #[test]
    fn vc_from_context() {
        let (c, h) = build();
        let other = TEST_VC_ID + 1;
        let plain = [0x66; 32];
        let frame = build_frame(&plain, other, TEST_SPI, 7);
        send(&c, &h, &frame, TEST_SPI, other, 0);
        assert_eq!(
            last(&h),
            (SdlsStatus::Success, plain.to_vec(), GCM_IV_LEN, 0)
        );
        send(&c, &h, &frame, TEST_SPI, TEST_VC_ID, 0);
        assert_eq!(last(&h).0, SdlsStatus::MacVerificationFailure);
    }

    #[test]
    fn recovers_after_mac_failure() {
        let (c, h) = build();
        let plain = [0x77; 40];
        let mut bad = build_frame(&plain, TEST_VC_ID, TEST_SPI, 8);
        bad[GCM_IV_LEN] ^= 1;
        send(&c, &h, &bad, TEST_SPI, TEST_VC_ID, 0);
        assert_eq!(last(&h).0, SdlsStatus::MacVerificationFailure);
        send(
            &c,
            &h,
            &build_frame(&plain, TEST_VC_ID, TEST_SPI, 8),
            TEST_SPI,
            TEST_VC_ID,
            0,
        );
        assert_eq!(last(&h).1, plain.to_vec());
    }

    #[test]
    fn short_buffer() {
        let (c, h) = build();
        send(
            &c,
            &h,
            &[0u8; GCM_IV_LEN + GCM_TAG_LEN - 1],
            TEST_SPI,
            TEST_VC_ID,
            0,
        );
        assert_eq!(last(&h).0, SdlsStatus::DecryptionFailure);
        // Rejected on shape alone, before the key is fetched.
        assert!(h.key_requests.lock().unwrap().is_empty());
    }

    #[test]
    fn key_unavailable() {
        let (c, h) = build();
        *h.key.lock().unwrap() = (Vec::new(), SdlsStatus::KeyError);
        send(
            &c,
            &h,
            &build_frame(&[0x33; 32], TEST_VC_ID, TEST_SPI, 9),
            TEST_SPI,
            TEST_VC_ID,
            0,
        );
        assert_eq!(last(&h).0, SdlsStatus::KeyError);
    }

    #[test]
    fn wrong_key_size() {
        let (c, h) = build();
        *h.key.lock().unwrap() = (KAT_KEY[..16].to_vec(), SdlsStatus::Success);
        send(
            &c,
            &h,
            &build_frame(&[0x44; 32], TEST_VC_ID, TEST_SPI, 10),
            TEST_SPI,
            TEST_VC_ID,
            0,
        );
        assert_eq!(last(&h).0, SdlsStatus::KeyError);
    }

    #[test]
    fn buffer_return() {
        let (c, h) = build();
        let context = FrameContext {
            vc_id: 4,
            ..FrameContext::default()
        };
        let p = c.decrypt_return_in(0);
        p.target.invoke(p.port_num, Buffer::allocate(20), &context);
        assert_eq!(
            *h.buffer_return.lock().unwrap(),
            vec![(vec![0u8; 20], context)]
        );
        assert!(h.decrypt_out.lock().unwrap().is_empty());
    }
}
