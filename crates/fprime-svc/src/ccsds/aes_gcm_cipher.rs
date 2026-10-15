//! AES-256-GCM primitives shared by [`AesGcmEncryptor`] and
//! [`AesGcmDecryptor`] (feature `aes-gcm`).
//!
//! The cryptography is RustCrypto's `aes-gcm` crate — nothing here is
//! hand-rolled; this module only fixes the SDLS parameters the upstream
//! components hard-code (`EVP_aes_256_gcm`, a 12-byte IV, a 16-byte tag)
//! and adapts the detached in-place API to the frame layouts. The cipher
//! is keyed per frame on the stack (no allocation), as the C++ re-keys its
//! `EVP_CIPHER_CTX` per frame.
//!
//! [`AesGcmEncryptor`]: crate::ccsds::aes_gcm_encryptor::AesGcmEncryptor
//! [`AesGcmDecryptor`]: crate::ccsds::aes_gcm_decryptor::AesGcmDecryptor

use aes_gcm::aead::AeadInPlace;
use aes_gcm::{Aes256Gcm, KeyInit, Nonce, Tag};

/// Length of the AES-GCM initialization vector (`GCM_IV_LEN`).
pub const GCM_IV_LEN: usize = 12;
/// Length of the AES-GCM authentication tag, the SDLS MAC (`GCM_TAG_LEN`).
pub const GCM_TAG_LEN: usize = 16;
/// Length of an AES-256 key (`AES_256_KEY_LEN`).
pub const AES_256_KEY_LEN: usize = 32;

/// Encrypt `buffer` in place under `key`/`iv`, authenticating `aad`;
/// returns the tag, or `None` if the cipher rejects the inputs.
pub fn seal(
    key: &[u8],
    iv: &[u8; GCM_IV_LEN],
    aad: &[u8],
    buffer: &mut [u8],
) -> Option<[u8; GCM_TAG_LEN]> {
    let cipher = Aes256Gcm::new_from_slice(key).ok()?;
    let tag = cipher
        .encrypt_in_place_detached(Nonce::from_slice(iv), aad, buffer)
        .ok()?;
    let mut out = [0u8; GCM_TAG_LEN];
    out.copy_from_slice(&tag);
    Some(out)
}

/// Verify `tag` over `aad` and the ciphertext in `buffer`, then decrypt it
/// in place. Returns false — leaving the ciphertext untouched — when the
/// tag does not verify or the cipher rejects the inputs.
pub fn open(
    key: &[u8],
    iv: &[u8; GCM_IV_LEN],
    aad: &[u8],
    buffer: &mut [u8],
    tag: &[u8; GCM_TAG_LEN],
) -> bool {
    let Ok(cipher) = Aes256Gcm::new_from_slice(key) else {
        return false;
    };
    cipher
        .decrypt_in_place_detached(Nonce::from_slice(iv), aad, buffer, Tag::from_slice(tag))
        .is_ok()
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Decode a hex string (test vectors).
    pub(crate) fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    /// A NIST/GCM-spec AES-256 test case: (key, iv, plaintext, aad,
    /// ciphertext, tag). Test cases 13-16 of McGrew & Viega, "The
    /// Galois/Counter Mode of Operation (GCM)", the AES-256 vectors NIST
    /// SP 800-38D validation draws on.
    pub(crate) const NIST_VECTORS: [(&str, &str, &str, &str, &str, &str); 4] = [
        (
            "0000000000000000000000000000000000000000000000000000000000000000",
            "000000000000000000000000",
            "",
            "",
            "",
            "530f8afbc74536b9a963b4f1c4cb738b",
        ),
        (
            "0000000000000000000000000000000000000000000000000000000000000000",
            "000000000000000000000000",
            "00000000000000000000000000000000",
            "",
            "cea7403d4d606b6e074ec5d3baf39d18",
            "d0d1c8a799996bf0265b98b5d48ab919",
        ),
        (
            "feffe9928665731c6d6a8f9467308308feffe9928665731c6d6a8f9467308308",
            "cafebabefacedbaddecaf888",
            "d9313225f88406e5a55909c5aff5269a86a7a9531534f7da2e4c303d8a318a72\
             1c3c0c95956809532fcf0e2449a6b525b16aedf5aa0de657ba637b391aafd255",
            "",
            "522dc1f099567d07f47f37a32a84427d643a8cdcbfe5c0c97598a2bd2555d1aa\
             8cb08e48590dbb3da7b08b1056828838c5f61e6393ba7a0abcc9f662898015ad",
            "b094dac5d93471bdec1a502270e3cc6c",
        ),
        (
            "feffe9928665731c6d6a8f9467308308feffe9928665731c6d6a8f9467308308",
            "cafebabefacedbaddecaf888",
            "d9313225f88406e5a55909c5aff5269a86a7a9531534f7da2e4c303d8a318a72\
             1c3c0c95956809532fcf0e2449a6b525b16aedf5aa0de657ba637b39",
            "feedfacedeadbeeffeedfacedeadbeefabaddad2",
            "522dc1f099567d07f47f37a32a84427d643a8cdcbfe5c0c97598a2bd2555d1aa\
             8cb08e48590dbb3da7b08b1056828838c5f61e6393ba7a0abcc9f662",
            "76fc6ece0f4e1768cddf8853bb2d551b",
        ),
    ];

    #[test]
    fn nist_vectors_seal() {
        for (key, iv, pt, aad, ct, tag) in NIST_VECTORS {
            let mut buf = hex(pt);
            let iv: [u8; GCM_IV_LEN] = hex(iv).try_into().unwrap();
            let got = seal(&hex(key), &iv, &hex(aad), &mut buf).unwrap();
            assert_eq!(buf, hex(ct));
            assert_eq!(got.to_vec(), hex(tag));
        }
    }

    #[test]
    fn nist_vectors_open() {
        for (key, iv, pt, aad, ct, tag) in NIST_VECTORS {
            let mut buf = hex(ct);
            let iv: [u8; GCM_IV_LEN] = hex(iv).try_into().unwrap();
            let tag: [u8; GCM_TAG_LEN] = hex(tag).try_into().unwrap();
            assert!(open(&hex(key), &iv, &hex(aad), &mut buf, &tag));
            assert_eq!(buf, hex(pt));
            // Any flipped tag bit fails and leaves the ciphertext alone.
            let mut bad = tag;
            bad[0] ^= 1;
            let mut buf = hex(ct);
            assert!(!open(&hex(key), &iv, &hex(aad), &mut buf, &bad));
            assert_eq!(buf, hex(ct));
        }
    }

    #[test]
    fn wrong_key_length_is_rejected() {
        let iv = [0u8; GCM_IV_LEN];
        assert!(seal(&[0u8; 16], &iv, &[], &mut []).is_none());
        assert!(!open(&[0u8; 16], &iv, &[], &mut [], &[0; GCM_TAG_LEN]));
    }
}
