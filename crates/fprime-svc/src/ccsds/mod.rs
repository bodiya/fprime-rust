//! CCSDS communications stack (`Svc/Ccsds`): Space Packet framing/deframing,
//! TM/TC and AOS transfer frames, APID sequence management, and the SDLS
//! (Space Data Link Security) framework.
//!
//! See `docs/cpp-analysis/ccsds.md` and, for AOS and SDLS, the module
//! headers (ROADMAP item 12).

// AES-256-GCM SDLS crypto: only with the off-by-default `aes-gcm` feature
// (third-party RustCrypto crates). Without it these components are not
// compiled at all — there is deliberately no stub that could be wired in
// and silently provide no security; `ClearTextEncryptor`/`Decryptor` are
// the explicit no-security choice.
#[cfg(feature = "aes-gcm")]
pub mod aes_gcm_cipher;
#[cfg(feature = "aes-gcm")]
pub mod aes_gcm_decryptor;
#[cfg(feature = "aes-gcm")]
pub mod aes_gcm_encryptor;
pub mod aos_deframer;
pub mod aos_framer;
pub mod apid_manager;
pub mod ccsds_sdls_deframer;
pub mod ccsds_sdls_framer;
pub mod clear_text_decryptor;
pub mod clear_text_encryptor;
pub mod crc16;
pub mod idle_packet;
pub mod sdls_auth_mask;
pub mod sdls_file_key_manager;
pub mod sdls_sa_router;
pub mod space_packet_deframer;
pub mod space_packet_framer;
pub mod tc_deframer;
pub mod tm_framer;
pub mod types;
