//! # AesGcmEncryptor — port of `Svc::Ccsds::AesGcmEncryptor` (passive;
//! feature `aes-gcm`)
//!
//! C++ sources: `Svc/Ccsds/AesGcmEncryptor/AesGcmEncryptor.{cpp,hpp,fpp}`
//! and its `docs/sdd.md` (CCSDS 355.0-B-2, NIST SP 800-38D).
//!
//! Encrypts and authenticates downlink data under AES-256-GCM
//! (`Svc.Ccsds.CcsdsSdlsEncrypt` toward an encryption client,
//! `Svc.Ccsds.SdlsKeyInterfaceClient` toward a key source):
//!
//! 1. A frame arriving while the previous output is still downstream is
//!    dropped with `ENCRYPTION_FAILURE` and `OutputBufferBusy`.
//! 2. Plaintext + IV + MAC beyond [`MAX_OUTPUT_SIZE`] (the TM data field
//!    less the 2-byte SA index) is `ENCRYPTION_FAILURE`.
//! 3. The key for the SA comes from `keyGet`; a failure or a key that is
//!    not 32 bytes is `KEY_ERROR`.
//! 4. A fresh 12-byte IV is drawn from the OS CSPRNG per frame (failure:
//!    `ENCRYPTION_FAILURE`); the plaintext is encrypted into the
//!    per-instance output store as `IV (12) | ciphertext | MAC (16)`, with
//!    the masked TM primary header + SPI + zeroed IV
//!    ([`SdlsTmAuthMask`], VC from the context) as AAD. The stack key copy
//!    is wiped once the cipher is keyed (C++ `OPENSSL_cleanse`).
//! 5. The plaintext always goes back on `bufferReturnOut`; a failed frame
//!    reports its status on `encryptOut` with an empty buffer.
//!
//! The output store returns on `encryptReturnIn` (an empty buffer from a
//! failed frame is ignored). It is marked in flight BEFORE being emitted,
//! so a synchronous return from inside `encryptOut` (as
//! `CcsdsSdlsFramer` does) releases it correctly.
//!
//! ## Deviations (documented)
//!
//! - The crypto is RustCrypto `aes-gcm` instead of OpenSSL EVP; the IV
//!   comes from `getrandom` (the OS CSPRNG) instead of `RAND_bytes`, and
//!   the key is wiped with `zeroize` instead of `OPENSSL_cleanse`. The
//!   cipher is keyed per frame on the stack, so nothing is allocated per
//!   frame (the C++ reuses one heap `EVP_CIPHER_CTX` for the same reason).
//! - The output store is owned storage moved out on `encryptOut` and back
//!   on `encryptReturnIn` (the C++ "returned pointer is `m_outBuf`" assert
//!   becomes "store in flight + capacity of `TmFrameFixedSize`").
//! - Like upstream, nothing counts invocations against the NIST SP 800-38D
//!   2^32 random-IV bound per key; key rotation is the key source's job.
//! - `no_std`: the IV still comes from `getrandom`, built with its
//!   `custom` feature (turned on by this crate's `spin` feature). Hosted
//!   targets keep the OS CSPRNG; on a target without one (bare metal) the
//!   deployment registers its hardware RNG with
//!   `getrandom::register_custom_getrandom!`, and a failing source fails
//!   the frame with `ENCRYPTION_FAILURE`, as a failing `RAND_bytes` does.

use crate::ccsds::aes_gcm_cipher::{AES_256_KEY_LEN, GCM_IV_LEN, GCM_TAG_LEN, seal};
use crate::ccsds::sdls_auth_mask::SdlsTmAuthMask;
use crate::ccsds::types::{
    CcsdsSdlsDataPort, CcsdsSdlsEncryptionPort, SdlsKeyBuffer, SdlsKeyPort, SdlsStatus,
    TM_FRAME_FIXED_SIZE, TMHeader, TMTrailer,
};
use alloc::vec;
use fprime_comp::{
    ComDataWithContextPort, EventGlue, OutputPort, PassiveBase, PortRef, input_port_adapter,
};
use fprime_config::{FwEventIdType, FwIndexType};
use fprime_fw::buffer::BufferStorage;
use fprime_fw::sync::{Arc, Mutex};
use fprime_fw::{Buffer, FrameContext, LogSeverity, SerBuf, SerBufAny, SerializeStatus, fw_assert};
use zeroize::Zeroize;

/// Largest output frame the downstream chain carries: the TM data field
/// less the 2-byte SA index (`AesGcmEncryptor::MAX_OUTPUT_SIZE`, 1014).
pub const MAX_OUTPUT_SIZE: usize =
    TM_FRAME_FIXED_SIZE - TMHeader::SERIALIZED_SIZE - TMTrailer::SERIALIZED_SIZE - size_of::<u16>();

/// `OutputBufferBusy()` — WARNING_HI.
pub const EVENTID_OUTPUT_BUFFER_BUSY: FwEventIdType = 0;

/// The AAD cache (C++ `m_aad`, `m_aadVcId`, `m_aadSaIndex`), touched only
/// under the `encryptIn` guard.
struct AadCache {
    aad: SdlsTmAuthMask,
    vc_id: u8,
    sa_index: u16,
}

/// `Svc::Ccsds::AesGcmEncryptor` — AES-256-GCM SDLS encryptor.
pub struct AesGcmEncryptor {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports and the time port.
    pub evt: EventGlue,
    /// `encryptOut` — the status and `IV | ciphertext | MAC` downstream.
    pub encrypt_out: OutputPort<dyn CcsdsSdlsDataPort>,
    /// `bufferReturnOut` — the plaintext back to its sender.
    pub buffer_return_out: OutputPort<dyn ComDataWithContextPort>,
    /// `keyGet` — the AES-256 key for a security association.
    pub key_get: OutputPort<dyn SdlsKeyPort>,
    /// The output store; `Some` is the C++ `BufferOwnershipState::OWNED`.
    store: Mutex<Option<BufferStorage>>,
    /// The `encryptIn` guard and the AAD cache it protects.
    guard: Mutex<AadCache>,
}

impl AesGcmEncryptor {
    /// Construct the component; the output store is allocated here, so
    /// encrypting a frame allocates nothing.
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            encrypt_out: OutputPort::new(),
            buffer_return_out: OutputPort::new(),
            key_get: OutputPort::new(),
            store: Mutex::new(Some(vec![0u8; TM_FRAME_FIXED_SIZE].into_boxed_slice())),
            guard: Mutex::new(AadCache {
                aad: SdlsTmAuthMask::new(0, 0),
                vc_id: 0,
                sa_index: 0,
            }),
        })
    }

    /// True while the output store is free for the next frame.
    pub fn owns_output_buffer(&self) -> bool {
        self.store.lock().is_some()
    }

    /// `encryptIn` — GUARDED `Svc.Ccsds.CcsdsSdlsEncryption`.
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
        let mut cache = self.guard.lock();

        // The previous frame is still downstream and the store holds
        // unsent ciphertext: overwriting it would let the new MAC cover a
        // substitution. Drop this frame instead.
        let Some(mut storage) = self.store.lock().take() else {
            self.evt.log_event(
                self.base.get_id_base(),
                EVENTID_OUTPUT_BUFFER_BUSY,
                LogSeverity::WarningHi,
                "Output buffer still in flight; frame dropped",
                |_buf| SerializeStatus::Ok,
            );
            self.fail_frame(data, context, SdlsStatus::EncryptionFailure);
            return;
        };

        // Store layout: IV (12) | ciphertext (N) | MAC (16).
        let plain_len = data.size();
        if plain_len + GCM_IV_LEN + GCM_TAG_LEN > MAX_OUTPUT_SIZE {
            *self.store.lock() = Some(storage);
            self.fail_frame(data, context, SdlsStatus::EncryptionFailure);
            return;
        }

        let mut key = SdlsKeyBuffer::new();
        let p = self.key_get.get();
        let key_status = p
            .target
            .invoke(p.port_num, security_association_index, &mut key);
        if key_status != SdlsStatus::Success || key.get_size() != AES_256_KEY_LEN {
            key.bytes_mut().zeroize();
            *self.store.lock() = Some(storage);
            self.fail_frame(data, context, SdlsStatus::KeyError);
            return;
        }

        // A repeated IV under one key breaks GCM: a fresh one per frame.
        let mut iv = [0u8; GCM_IV_LEN];
        if getrandom::getrandom(&mut iv).is_err() {
            key.bytes_mut().zeroize();
            *self.store.lock() = Some(storage);
            self.fail_frame(data, context, SdlsStatus::EncryptionFailure);
            return;
        }

        // The AAD depends only on the VC and the SA: rebuilt on change.
        let vc_id = context.vc_id;
        if vc_id != cache.vc_id || security_association_index != cache.sa_index {
            cache.aad = SdlsTmAuthMask::new(vc_id, security_association_index);
            cache.vc_id = vc_id;
            cache.sa_index = security_association_index;
        }

        storage[..GCM_IV_LEN].copy_from_slice(&iv);
        let ciphertext = &mut storage[GCM_IV_LEN..GCM_IV_LEN + plain_len];
        ciphertext.copy_from_slice(data.data());
        let tag = seal(key.as_slice(), &iv, &cache.aad.bytes, ciphertext);
        // The cipher has its key schedule; wipe the stack copy.
        key.bytes_mut().zeroize();
        let Some(tag) = tag else {
            *self.store.lock() = Some(storage);
            self.fail_frame(data, context, SdlsStatus::EncryptionFailure);
            return;
        };
        // The MAC goes straight after the ciphertext.
        let tag_start = GCM_IV_LEN + plain_len;
        storage[tag_start..tag_start + GCM_TAG_LEN].copy_from_slice(&tag);

        let out_len = GCM_IV_LEN + plain_len + GCM_TAG_LEN;
        let mut cipher_buf = Buffer::from_storage(storage, Buffer::NO_CONTEXT);
        cipher_buf.set_size(out_len);
        // The store is already marked in flight (taken), so a return from
        // inside encryptOut releases it.

        // The plaintext was copied: hand it back before the ciphertext goes
        // downstream.
        let p = self.buffer_return_out.get();
        p.target.invoke(p.port_num, data, context);
        let p = self.encrypt_out.get();
        p.target
            .invoke(p.port_num, SdlsStatus::Success, cipher_buf, context);
    }

    fn encrypt_return_in_handler(
        &self,
        _port_num: FwIndexType,
        data: Buffer,
        _context: &FrameContext,
    ) {
        // A failed frame reported its status with an empty buffer.
        if !data.is_valid() {
            return;
        }
        // Only the output store is ever emitted: anything else is a wiring
        // error.
        let mut store = self.store.lock();
        fw_assert!(store.is_none());
        fw_assert!(
            data.capacity() == TM_FRAME_FIXED_SIZE,
            data.capacity() as i32
        );
        *store = Some(data.into_storage());
    }

    /// Return the plaintext and report `status` against an empty buffer
    /// (`CcsdsSdlsFramer` still needs a status to release the com status).
    fn fail_frame(&self, data: Buffer, context: &FrameContext, status: SdlsStatus) {
        let p = self.buffer_return_out.get();
        p.target.invoke(p.port_num, data, context);
        let p = self.encrypt_out.get();
        p.target
            .invoke(p.port_num, status, Buffer::empty(), context);
    }
}

impl CcsdsSdlsEncryptionPort for AesGcmEncryptor {
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
    /// `encryptReturnIn` — SYNC: the output store coming back.
    component: AesGcmEncryptor;
    adapter: EncryptReturnInAdapter;
    port: ComDataWithContextPort;
    input: pub encrypt_return_in;
    handler: encrypt_return_in_handler;
    args { val data: Buffer, ref context: FrameContext }
}

// ---------------------------------------------------------------------------
// Tests — port of AesGcmEncryptorTester
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ccsds::aes_gcm_cipher::open;
    use fprime_comp::LogPort;
    use fprime_fw::{LogBuffer, Time};
    use std::sync::Mutex as StdMutex;
    use std::sync::Weak;

    const TEST_VC_ID: u8 = 3;
    const TEST_SPI: u16 = 0x00AB;
    const TEST_KEY: [u8; 32] = [
        0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E,
        0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D,
        0x5E, 0x5F,
    ];

    /// The ground segment's TM AAD layout, built independently.
    fn build_tm_aad(vc_id: u8, spi: u16) -> [u8; 20] {
        let mut aad = [0u8; 20];
        aad[1] = (vc_id << 1) & 0x0E;
        aad[6] = (spi >> 8) as u8;
        aad[7] = spi as u8;
        aad
    }

    type OutRec = (SdlsStatus, Vec<u8>, bool);

    #[derive(Default)]
    struct Harness {
        encrypt_out: StdMutex<Vec<OutRec>>,
        /// Emitted stores kept for a later return.
        stores: StdMutex<Vec<Buffer>>,
        buffer_return: StdMutex<Vec<(Vec<u8>, u32)>>,
        key_requests: StdMutex<Vec<u16>>,
        events: StdMutex<Vec<FwEventIdType>>,
        key: StdMutex<(Vec<u8>, SdlsStatus)>,
        return_synchronously: StdMutex<bool>,
        comp: StdMutex<Weak<AesGcmEncryptor>>,
    }

    impl CcsdsSdlsDataPort for Harness {
        fn invoke(
            &self,
            _p: FwIndexType,
            status: SdlsStatus,
            data: Buffer,
            context: &FrameContext,
        ) {
            self.encrypt_out
                .lock()
                .unwrap()
                .push((status, data.data().to_vec(), data.is_valid()));
            if !data.is_valid() {
                return;
            }
            if *self.return_synchronously.lock().unwrap() {
                // CcsdsSdlsFramer copies the ciphertext and returns the
                // buffer before encryptOut returns.
                let comp = self.comp.lock().unwrap().upgrade().unwrap();
                let p = comp.encrypt_return_in(0);
                p.target.invoke(p.port_num, data, context);
            } else {
                self.stores.lock().unwrap().push(data);
            }
        }
    }

    impl ComDataWithContextPort for Harness {
        fn invoke(&self, _p: FwIndexType, data: Buffer, _c: &FrameContext) {
            self.buffer_return
                .lock()
                .unwrap()
                .push((data.data().to_vec(), data.context()));
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

    impl LogPort for Harness {
        fn invoke(
            &self,
            _p: FwIndexType,
            id: FwEventIdType,
            _t: &mut Time,
            _s: LogSeverity,
            _a: &mut LogBuffer,
        ) {
            self.events.lock().unwrap().push(id);
        }
    }

    impl Harness {
        fn clear(&self) {
            self.encrypt_out.lock().unwrap().clear();
            self.buffer_return.lock().unwrap().clear();
            self.events.lock().unwrap().clear();
            self.key_requests.lock().unwrap().clear();
        }
        fn last_out(&self) -> OutRec {
            let out = self.encrypt_out.lock().unwrap();
            // Every path through encryptIn ends in exactly one encryptOut.
            assert_eq!(out.len(), 1);
            out[0].clone()
        }
    }

    fn build() -> (Arc<AesGcmEncryptor>, Arc<Harness>) {
        let h = Arc::new(Harness::default());
        *h.key.lock().unwrap() = (TEST_KEY.to_vec(), SdlsStatus::Success);
        let c = AesGcmEncryptor::new("AesGcmEncryptor");
        *h.comp.lock().unwrap() = Arc::downgrade(&c);
        c.encrypt_out.connect(h.clone(), 0);
        c.buffer_return_out.connect(h.clone(), 0);
        c.key_get.connect(h.clone(), 0);
        c.evt.log_out.connect(h.clone(), 0);
        (c, h)
    }

    /// `sendEncrypt`: a patterned plaintext with the VC on the context.
    fn send(c: &Arc<AesGcmEncryptor>, h: &Harness, len: usize, spi: u16, vc_id: u8) -> Vec<u8> {
        h.clear();
        let plain: Vec<u8> = (0..len).map(|i| (i * 7 + 3) as u8).collect();
        let mut b = Buffer::allocate(len.max(1));
        b.data_mut()[..len].copy_from_slice(&plain);
        b.set_size(len);
        b.set_context(0x55);
        let context = FrameContext {
            vc_id,
            ..FrameContext::default()
        };
        let p = c.encrypt_in(0);
        p.target.invoke(p.port_num, spi, b, &context);
        plain
    }

    fn return_store(c: &Arc<AesGcmEncryptor>, h: &Harness) {
        let store = h.stores.lock().unwrap().remove(0);
        let p = c.encrypt_return_in(0);
        p.target.invoke(p.port_num, store, &FrameContext::default());
    }

    /// Decrypt an emitted frame with the reference AAD.
    fn decrypts_to(frame: &[u8], vc_id: u8, spi: u16) -> Option<Vec<u8>> {
        let iv: [u8; GCM_IV_LEN] = frame[..GCM_IV_LEN].try_into().unwrap();
        let n = frame.len() - GCM_IV_LEN - GCM_TAG_LEN;
        let tag: [u8; GCM_TAG_LEN] = frame[GCM_IV_LEN + n..].try_into().unwrap();
        let mut buf = frame[GCM_IV_LEN..GCM_IV_LEN + n].to_vec();
        open(&TEST_KEY, &iv, &build_tm_aad(vc_id, spi), &mut buf, &tag).then_some(buf)
    }

    #[test]
    fn encrypt_nominal() {
        let (c, h) = build();
        let plain = send(&c, &h, 64, TEST_SPI, TEST_VC_ID);
        let (status, frame, _) = h.last_out();
        assert_eq!(status, SdlsStatus::Success);
        assert_eq!(frame.len(), GCM_IV_LEN + 64 + GCM_TAG_LEN);
        assert_eq!(decrypts_to(&frame, TEST_VC_ID, TEST_SPI).unwrap(), plain);
        assert_eq!(*h.key_requests.lock().unwrap(), vec![TEST_SPI]);
    }

    /// The ciphertext and MAC match an independently computed reference.
    #[test]
    fn ciphertext_and_mac_match() {
        let (c, h) = build();
        let plain = send(&c, &h, 48, TEST_SPI, TEST_VC_ID);
        let (_, frame, _) = h.last_out();
        let iv: [u8; GCM_IV_LEN] = frame[..GCM_IV_LEN].try_into().unwrap();
        let mut expected = plain.clone();
        let tag = seal(
            &TEST_KEY,
            &iv,
            &build_tm_aad(TEST_VC_ID, TEST_SPI),
            &mut expected,
        )
        .unwrap();
        assert_eq!(&frame[GCM_IV_LEN..GCM_IV_LEN + 48], &expected[..]);
        assert_eq!(&frame[GCM_IV_LEN + 48..], &tag[..]);
    }

    #[test]
    fn auth_mask_layout() {
        for vc in [0u8, 1, TEST_VC_ID, 0x07] {
            for spi in [0u16, 0x1234, 0xFFFF] {
                assert_eq!(SdlsTmAuthMask::new(vc, spi).bytes, build_tm_aad(vc, spi));
            }
        }
    }

    #[test]
    fn context_vc_id_is_authenticated() {
        let (c, h) = build();
        let other = TEST_VC_ID + 1;
        let plain = send(&c, &h, 32, TEST_SPI, other);
        let (_, frame, _) = h.last_out();
        assert_eq!(decrypts_to(&frame, other, TEST_SPI).unwrap(), plain);
        assert!(decrypts_to(&frame, TEST_VC_ID, TEST_SPI).is_none());
    }

    #[test]
    fn iv_is_fresh_per_frame() {
        let (c, h) = build();
        send(&c, &h, 32, TEST_SPI, TEST_VC_ID);
        let (_, first, _) = h.last_out();
        return_store(&c, &h);
        send(&c, &h, 32, TEST_SPI, TEST_VC_ID);
        let (_, second, _) = h.last_out();
        assert_ne!(&first[..GCM_IV_LEN], &second[..GCM_IV_LEN]);
        assert_ne!(
            &first[GCM_IV_LEN..GCM_IV_LEN + 32],
            &second[GCM_IV_LEN..GCM_IV_LEN + 32]
        );
    }

    #[test]
    fn empty_plaintext() {
        let (c, h) = build();
        send(&c, &h, 0, TEST_SPI, TEST_VC_ID);
        let (status, frame, _) = h.last_out();
        assert_eq!(status, SdlsStatus::Success);
        assert_eq!(frame.len(), GCM_IV_LEN + GCM_TAG_LEN);
        assert_eq!(
            decrypts_to(&frame, TEST_VC_ID, TEST_SPI).unwrap(),
            Vec::<u8>::new()
        );
    }

    #[test]
    fn output_capacity_boundary() {
        let (c, h) = build();
        assert_eq!(MAX_OUTPUT_SIZE, 1014);
        let largest = MAX_OUTPUT_SIZE - GCM_IV_LEN - GCM_TAG_LEN;
        send(&c, &h, largest, TEST_SPI, TEST_VC_ID);
        let (status, frame, _) = h.last_out();
        assert_eq!(status, SdlsStatus::Success);
        assert_eq!(frame.len(), MAX_OUTPUT_SIZE);
        return_store(&c, &h);
        // One byte more has nowhere to go.
        let plain = send(&c, &h, largest + 1, TEST_SPI, TEST_VC_ID);
        let (status, _, valid) = h.last_out();
        assert_eq!(status, SdlsStatus::EncryptionFailure);
        assert!(!valid);
        assert_eq!(*h.buffer_return.lock().unwrap(), vec![(plain, 0x55)]);
        // Refused on size, the store stays free.
        assert!(c.owns_output_buffer());
    }

    #[test]
    fn key_unavailable() {
        let (c, h) = build();
        *h.key.lock().unwrap() = (Vec::new(), SdlsStatus::KeyError);
        send(&c, &h, 32, TEST_SPI, TEST_VC_ID);
        assert_eq!(h.last_out().0, SdlsStatus::KeyError);
        assert!(c.owns_output_buffer());
    }

    #[test]
    fn wrong_key_size() {
        let (c, h) = build();
        // An AES-128 key: usable, and silently wrong if it were used.
        *h.key.lock().unwrap() = (TEST_KEY[..16].to_vec(), SdlsStatus::Success);
        send(&c, &h, 32, TEST_SPI, TEST_VC_ID);
        assert_eq!(h.last_out().0, SdlsStatus::KeyError);
    }

    #[test]
    fn plaintext_returned_unmodified() {
        let (c, h) = build();
        let plain = send(&c, &h, 40, TEST_SPI, TEST_VC_ID);
        assert_eq!(h.last_out().0, SdlsStatus::Success);
        assert_eq!(*h.buffer_return.lock().unwrap(), vec![(plain, 0x55)]);
    }

    #[test]
    fn output_buffer_reclaimed() {
        let (c, h) = build();
        send(&c, &h, 32, TEST_SPI, TEST_VC_ID);
        assert!(!c.owns_output_buffer());
        h.clear();
        return_store(&c, &h);
        assert!(h.buffer_return.lock().unwrap().is_empty());
        assert!(c.owns_output_buffer());
        send(&c, &h, 32, TEST_SPI, TEST_VC_ID);
        assert_eq!(h.last_out().0, SdlsStatus::Success);
        // An empty buffer from a failed frame is a no-op.
        h.clear();
        let p = c.encrypt_return_in(0);
        p.target
            .invoke(p.port_num, Buffer::empty(), &FrameContext::default());
        assert!(h.buffer_return.lock().unwrap().is_empty());
        assert!(!c.owns_output_buffer());
    }

    #[test]
    fn synchronous_return() {
        let (c, h) = build();
        *h.return_synchronously.lock().unwrap() = true;
        for _ in 0..3 {
            send(&c, &h, 32, TEST_SPI, TEST_VC_ID);
            assert_eq!(h.last_out().0, SdlsStatus::Success);
        }
        assert!(h.events.lock().unwrap().is_empty());
        assert!(c.owns_output_buffer());
    }

    #[test]
    fn output_buffer_busy() {
        let (c, h) = build();
        send(&c, &h, 48, TEST_SPI, TEST_VC_ID);
        let (_, first, _) = h.last_out();
        let plain = send(&c, &h, 48, TEST_SPI, TEST_VC_ID);
        let (status, _, valid) = h.last_out();
        assert_eq!(status, SdlsStatus::EncryptionFailure);
        assert!(!valid);
        assert_eq!(*h.events.lock().unwrap(), vec![EVENTID_OUTPUT_BUFFER_BUSY]);
        assert_eq!(*h.buffer_return.lock().unwrap(), vec![(plain, 0x55)]);
        // The in-flight frame was not overwritten.
        assert_eq!(h.stores.lock().unwrap()[0].data(), &first[..]);
        return_store(&c, &h);
        send(&c, &h, 48, TEST_SPI, TEST_VC_ID);
        assert_eq!(h.last_out().0, SdlsStatus::Success);
    }

    #[test]
    #[should_panic(expected = "Assert:")]
    fn returning_a_foreign_buffer_asserts() {
        let (c, h) = build();
        send(&c, &h, 8, TEST_SPI, TEST_VC_ID);
        let p = c.encrypt_return_in(0);
        p.target
            .invoke(p.port_num, Buffer::allocate(64), &FrameContext::default());
    }
}
