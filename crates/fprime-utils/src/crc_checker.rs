//! Port of `Utils::CRCChecker` — file checksum sidecar creation/verification.
//!
//! C++ sources: `Utils/CRCChecker.cpp`, `Utils/CRCChecker.hpp`,
//! `default/config/CRCCheckerConfig.hpp`.
//! Analysis: `docs/cpp-analysis/utils-misc.md` (Utils::CRCChecker section).
//!
//! Sidecar format (`<name>.CRC32`): exactly 4 bytes — the standard
//! complemented CRC-32 of the whole target file, written via a raw copy of
//! the `u32` in NATIVE machine endianness (`to_ne_bytes`). This is C++
//! parity: `CRCChecker.cpp` `reinterpret_cast`s the `U32`, so the sidecar is
//! little-endian on typical targets, NOT F Prime big-endian serialization.
//! Do not "fix" it.

use std::fs;
use std::io::{Read, Write};

use crate::hash::{HASH_EXTENSION_STRING, Hash};

/// Block size used when reading files for CRC calculation
/// (C++ `CONFIG_CRC_FILE_READ_BLOCK`, `default/config/CRCCheckerConfig.hpp`).
pub const CRC_FILE_READ_BLOCK: usize = 2048;

/// CRC checker status (port of `Utils::crc_stat_t`, exact discriminants).
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcStat {
    /// `PASSED_FILE_CRC_CHECK`
    PassedFileCrcCheck = 0,
    /// `PASSED_FILE_CRC_WRITE`
    PassedFileCrcWrite = 1,
    /// `FAILED_FILE_SIZE`
    FailedFileSize = 2,
    /// `FAILED_FILE_SIZE_CAST` (unused by the current C++ code paths;
    /// kept for parity)
    FailedFileSizeCast = 3,
    /// `FAILED_FILE_OPEN`
    FailedFileOpen = 4,
    /// `FAILED_FILE_READ`
    FailedFileRead = 5,
    /// `FAILED_FILE_CRC_OPEN`
    FailedFileCrcOpen = 6,
    /// `FAILED_FILE_CRC_READ`
    FailedFileCrcRead = 7,
    /// `FAILED_FILE_CRC_WRITE`
    FailedFileCrcWrite = 8,
    /// `FAILED_FILE_CRC_CHECK`
    FailedFileCrcCheck = 9,
}

/// Compute the standard CRC-32 of the file at `fname`, reading in
/// [`CRC_FILE_READ_BLOCK`]-byte blocks (the shared body of the C++
/// `create_checksum_file` / `verify_checksum`).
fn compute_file_crc(fname: &str) -> Result<u32, CrcStat> {
    // C++: Os::FileSystem::getFileSize failure -> FAILED_FILE_SIZE.
    let filesize = match fs::metadata(fname) {
        Ok(meta) => meta.len() as usize,
        Err(_) => return Err(CrcStat::FailedFileSize),
    };

    let mut file = match fs::File::open(fname) {
        Ok(f) => f,
        Err(_) => return Err(CrcStat::FailedFileOpen),
    };

    // C++ parity: read filesize/BLOCK full blocks, then the remainder; a
    // short read (file shrank underneath us) is FAILED_FILE_READ.
    let mut hash = Hash::new();
    let mut block = [0u8; CRC_FILE_READ_BLOCK];
    let blocks = filesize / CRC_FILE_READ_BLOCK;
    for _ in 0..blocks {
        if file.read_exact(&mut block).is_err() {
            return Err(CrcStat::FailedFileRead);
        }
        hash.update(&block);
    }
    let remaining = filesize % CRC_FILE_READ_BLOCK;
    if remaining > 0 {
        if file.read_exact(&mut block[..remaining]).is_err() {
            return Err(CrcStat::FailedFileRead);
        }
        hash.update(&block[..remaining]);
    }

    Ok(hash.finalize())
}

/// The sidecar path for `fname`: `<fname>.CRC32`.
fn sidecar_name(fname: &str) -> String {
    format!("{fname}{HASH_EXTENSION_STRING}")
}

/// Compute the CRC-32 of the file at `fname` and write the 4-byte
/// native-endian sidecar `<fname>.CRC32`
/// (port of `Utils::create_checksum_file`).
///
/// Returns [`CrcStat::PassedFileCrcWrite`] on success.
pub fn create_checksum_file(fname: &str) -> CrcStat {
    let checksum = match compute_file_crc(fname) {
        Ok(crc) => crc,
        Err(stat) => return stat,
    };

    let mut crc_file = match fs::File::create(sidecar_name(fname)) {
        Ok(f) => f,
        Err(_) => return CrcStat::FailedFileCrcOpen,
    };
    // C++ parity gotcha: raw memcpy of the U32 -> NATIVE endianness.
    if crc_file.write_all(&checksum.to_ne_bytes()).is_err() {
        return CrcStat::FailedFileCrcWrite;
    }

    CrcStat::PassedFileCrcWrite
}

/// Read the stored CRC-32 out of the sidecar `<fname>.CRC32` (4 raw
/// native-endian bytes) into `checksum_from_file`
/// (port of `Utils::read_crc32_from_file`).
///
/// Returns [`CrcStat::PassedFileCrcCheck`] on success; on failure
/// `checksum_from_file` is left unmodified.
pub fn read_crc32_from_file(fname: &str, checksum_from_file: &mut u32) -> CrcStat {
    let mut crc_file = match fs::File::open(sidecar_name(fname)) {
        Ok(f) => f,
        Err(_) => return CrcStat::FailedFileCrcOpen,
    };

    let mut bytes = [0u8; 4];
    if crc_file.read_exact(&mut bytes).is_err() {
        return CrcStat::FailedFileCrcRead;
    }
    // C++ parity gotcha: raw read into the U32 -> NATIVE endianness.
    *checksum_from_file = u32::from_ne_bytes(bytes);
    CrcStat::PassedFileCrcCheck
}

/// Recompute the CRC-32 of `fname` and compare against the sidecar
/// (port of `Utils::verify_checksum`).
///
/// C++ parity: `expected` (the sidecar value) and `actual` (the recomputed
/// value) are written in BOTH the pass and fail cases; on an earlier failure
/// (size/open/read) they are left unmodified.
pub fn verify_checksum(fname: &str, expected: &mut u32, actual: &mut u32) -> CrcStat {
    let checksum = match compute_file_crc(fname) {
        Ok(crc) => crc,
        Err(stat) => return stat,
    };

    let mut checksum_from_file: u32 = 0;
    let stat = read_crc32_from_file(fname, &mut checksum_from_file);
    if stat != CrcStat::PassedFileCrcCheck {
        return stat;
    }

    *expected = checksum_from_file;
    *actual = checksum;
    if checksum != checksum_from_file {
        return CrcStat::FailedFileCrcCheck;
    }
    CrcStat::PassedFileCrcCheck
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    /// A per-test scratch file under the system temp dir, removed (with its
    /// sidecar) on drop.
    struct TempFile {
        path: PathBuf,
    }

    impl TempFile {
        fn create(name: &str, contents: &[u8]) -> Self {
            let mut path = std::env::temp_dir();
            path.push(format!("fprime_utils_crc_{}_{name}", std::process::id()));
            fs::write(&path, contents).unwrap();
            Self { path }
        }

        fn path_str(&self) -> &str {
            self.path.to_str().unwrap()
        }
    }

    impl Drop for TempFile {
        fn drop(&mut self) {
            let _ = fs::remove_file(&self.path);
            let _ = fs::remove_file(sidecar_name(self.path.to_str().unwrap()));
        }
    }

    #[test]
    fn create_then_verify_roundtrip() {
        let f = TempFile::create("roundtrip.bin", b"The quick brown fox");
        assert_eq!(
            create_checksum_file(f.path_str()),
            CrcStat::PassedFileCrcWrite
        );

        let (mut expected, mut actual) = (0u32, 0u32);
        assert_eq!(
            verify_checksum(f.path_str(), &mut expected, &mut actual),
            CrcStat::PassedFileCrcCheck
        );
        assert_eq!(expected, actual);
        assert_eq!(actual, Hash::hash_u32(b"The quick brown fox"));
    }

    #[test]
    fn sidecar_holds_native_endian_standard_crc32() {
        let f = TempFile::create("endian.bin", b"123456789");
        assert_eq!(
            create_checksum_file(f.path_str()),
            CrcStat::PassedFileCrcWrite
        );

        // C++ parity gotcha: NATIVE endianness, not big-endian.
        let sidecar = fs::read(sidecar_name(f.path_str())).unwrap();
        assert_eq!(sidecar, 0xCBF4_3926u32.to_ne_bytes());
        assert_eq!(sidecar.len(), 4);

        let mut stored = 0u32;
        assert_eq!(
            read_crc32_from_file(f.path_str(), &mut stored),
            CrcStat::PassedFileCrcCheck
        );
        assert_eq!(stored, 0xCBF4_3926);
    }

    #[test]
    fn verify_detects_corruption() {
        let f = TempFile::create("corrupt.bin", b"original contents");
        assert_eq!(
            create_checksum_file(f.path_str()),
            CrcStat::PassedFileCrcWrite
        );

        // Corrupt the target file after the sidecar was written.
        fs::write(&f.path, b"tampered contents!").unwrap();

        let (mut expected, mut actual) = (0u32, 0u32);
        assert_eq!(
            verify_checksum(f.path_str(), &mut expected, &mut actual),
            CrcStat::FailedFileCrcCheck
        );
        // C++ parity: expected/actual are populated in the fail case too.
        assert_eq!(expected, Hash::hash_u32(b"original contents"));
        assert_eq!(actual, Hash::hash_u32(b"tampered contents!"));
        assert_ne!(expected, actual);
    }

    #[test]
    fn multi_block_file_crc_is_correct() {
        // Larger than one 2048-byte read block, not a multiple of it.
        let data: Vec<u8> = (0..5000u32).map(|i| (i % 251) as u8).collect();
        let f = TempFile::create("blocks.bin", &data);
        assert_eq!(
            create_checksum_file(f.path_str()),
            CrcStat::PassedFileCrcWrite
        );

        let (mut expected, mut actual) = (0u32, 0u32);
        assert_eq!(
            verify_checksum(f.path_str(), &mut expected, &mut actual),
            CrcStat::PassedFileCrcCheck
        );
        assert_eq!(actual, Hash::hash_u32(&data));
    }

    #[test]
    fn missing_target_file_reports_failed_size() {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "fprime_utils_crc_{}_missing.bin",
            std::process::id()
        ));
        let name = path.to_str().unwrap();
        assert_eq!(create_checksum_file(name), CrcStat::FailedFileSize);
        let (mut e, mut a) = (7u32, 8u32);
        assert_eq!(
            verify_checksum(name, &mut e, &mut a),
            CrcStat::FailedFileSize
        );
        // Untouched on early failure (C++ parity).
        assert_eq!((e, a), (7, 8));
    }

    #[test]
    fn missing_sidecar_reports_crc_open_failure() {
        let f = TempFile::create("nosidecar.bin", b"data");
        let mut stored = 42u32;
        assert_eq!(
            read_crc32_from_file(f.path_str(), &mut stored),
            CrcStat::FailedFileCrcOpen
        );
        assert_eq!(stored, 42); // untouched
        let (mut e, mut a) = (0u32, 0u32);
        assert_eq!(
            verify_checksum(f.path_str(), &mut e, &mut a),
            CrcStat::FailedFileCrcOpen
        );
    }

    #[test]
    fn short_sidecar_reports_crc_read_failure() {
        let f = TempFile::create("shortsidecar.bin", b"data");
        fs::write(sidecar_name(f.path_str()), [0xAAu8, 0xBB]).unwrap();
        let mut stored = 0u32;
        assert_eq!(
            read_crc32_from_file(f.path_str(), &mut stored),
            CrcStat::FailedFileCrcRead
        );
    }

    #[test]
    fn empty_file_checksum() {
        let f = TempFile::create("empty.bin", b"");
        assert_eq!(
            create_checksum_file(f.path_str()),
            CrcStat::PassedFileCrcWrite
        );
        let mut stored = 1u32;
        assert_eq!(
            read_crc32_from_file(f.path_str(), &mut stored),
            CrcStat::PassedFileCrcCheck
        );
        assert_eq!(stored, 0x0000_0000);
    }

    #[test]
    fn crc_stat_discriminants_match_cpp() {
        assert_eq!(CrcStat::PassedFileCrcCheck as i32, 0);
        assert_eq!(CrcStat::PassedFileCrcWrite as i32, 1);
        assert_eq!(CrcStat::FailedFileSize as i32, 2);
        assert_eq!(CrcStat::FailedFileSizeCast as i32, 3);
        assert_eq!(CrcStat::FailedFileOpen as i32, 4);
        assert_eq!(CrcStat::FailedFileRead as i32, 5);
        assert_eq!(CrcStat::FailedFileCrcOpen as i32, 6);
        assert_eq!(CrcStat::FailedFileCrcRead as i32, 7);
        assert_eq!(CrcStat::FailedFileCrcWrite as i32, 8);
        assert_eq!(CrcStat::FailedFileCrcCheck as i32, 9);
    }
}
