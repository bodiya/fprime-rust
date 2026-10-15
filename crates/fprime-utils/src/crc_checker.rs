//! Port of `Utils::CRCChecker` — file checksum sidecar creation/verification.
//!
//! C++ sources: `Utils/CRCChecker.cpp`, `Utils/CRCChecker.hpp`,
//! `default/config/CRCCheckerConfig.hpp`; tests
//! `Utils/test/ut/CRCCheckerTester.cpp` (ported below, except
//! `ChecksumFileMatchesFrameworkFormat`, which lives next to
//! [`crate::validate_file`]).
//! Analysis: `docs/cpp-analysis/utils-misc.md` (Utils::CRCChecker section).
//!
//! Like upstream, the functions run over `Os::File` / `Os::FileSystem`
//! ([`fprime_os::file`], [`fprime_os::filesystem`]), so open modes, read/write
//! wait semantics and size queries are the OSAL's, and the sidecar name is
//! built in a fixed-capacity `Fw::FileNameString` (no heap allocation).
//!
//! Sidecar format (`<name>.CRC32`): exactly 4 bytes — the standard
//! complemented CRC-32 of the whole target file, serialized BIG-ENDIAN
//! (`to_be_bytes`). This is the same file [`crate::validate_file`]
//! (`Os::ValidateFile`) writes and reads; the two encodings are compared
//! side by side in that module's documentation. The sidecar is opened
//! `OPEN_WRITE` (created if missing, NOT truncated), as upstream does: a
//! pre-existing longer file keeps its tail, and readers only look at the
//! first 4 bytes.
//!
//! C++ parity: upstream serializes the `U32` through `Fw::SerialBuffer`
//! since nasa/fprime#5775 (in the 4dee010 model this port tracks). Older
//! CRCChecker builds `reinterpret_cast` the `U32`, i.e. wrote NATIVE-endian
//! sidecars that were not interchangeable with `Utils::Hash` /
//! `Os::ValidateFile` on little-endian hosts; this port used to reproduce
//! that and now follows the fix.
//!
//! Overlong names (nasa/fprime#5803): when `<fname>.CRC32` does not fit in an
//! `Fw::FileNameString` (`FileNameStringSize` = 240 characters), the
//! functions that need the sidecar name return
//! [`CrcStat::FailedFileNameTooLong`] instead of asserting. The name is
//! built only once the sidecar is needed, so [`create_checksum_file`] and
//! [`verify_checksum`] report data-file failures (size/open/read) first.

use fprime_config::FwSizeType;
use fprime_fw::FileNameString;
use fprime_os::file::{self, File, Mode, WaitType};
use fprime_os::filesystem;

use crate::hash::{HASH_EXTENSION_STRING, Hash};

/// Block size used when reading files for CRC calculation
/// (C++ `CONFIG_CRC_FILE_READ_BLOCK`, `default/config/CRCCheckerConfig.hpp`);
/// configured in [`fprime_config::crc_checker`].
pub use fprime_config::crc_checker::FILE_READ_BLOCK as CRC_FILE_READ_BLOCK;

/// Size of the stored checksum (`sizeof(U32)`).
const CHECKSUM_SIZE: usize = size_of::<u32>();

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
    /// `FAILED_FILE_NAME_TOO_LONG` — the file name plus
    /// [`HASH_EXTENSION_STRING`] does not fit in an `Fw::FileNameString`
    /// (nasa/fprime#5803).
    FailedFileNameTooLong = 10,
}

/// Compute the standard CRC-32 of the file at `fname`, reading in
/// [`CRC_FILE_READ_BLOCK`]-byte blocks (the body C++ `create_checksum_file`
/// and `verify_checksum` each repeat inline).
fn compute_file_crc(fname: &str) -> Result<u32, CrcStat> {
    let mut filesize: FwSizeType = 0;
    if filesystem::get_file_size(fname, &mut filesize) != filesystem::Status::OpOk {
        return Err(CrcStat::FailedFileSize);
    }

    let mut f = File::new();
    if f.open(fname, Mode::OpenRead) != file::Status::OpOk {
        return Err(CrcStat::FailedFileOpen);
    }

    // C++ parity: filesize/BLOCK full blocks, then the remainder, with the
    // two-argument `read` (WAIT: fill the buffer or hit EOF). A short read
    // (the file shrank underneath us) is FAILED_FILE_READ; a file that grew
    // is hashed only up to the size taken above.
    let block_size = CRC_FILE_READ_BLOCK as FwSizeType;
    let mut hash = Hash::new();
    let mut block_data = [0u8; CRC_FILE_READ_BLOCK];
    let blocks = filesize / block_size;
    for _ in 0..blocks {
        let mut bytes_to_read: FwSizeType = 0;
        let stat = f.read(&mut block_data, &mut bytes_to_read, WaitType::Wait);
        if stat != file::Status::OpOk || bytes_to_read != block_size {
            f.close();
            return Err(CrcStat::FailedFileRead);
        }
        hash.update(&block_data);
    }

    // Less than one block, so it fits in usize.
    let remaining_bytes = (filesize % block_size) as usize;
    if remaining_bytes > 0 {
        let mut bytes_to_read: FwSizeType = 0;
        let stat = f.read(
            &mut block_data[..remaining_bytes],
            &mut bytes_to_read,
            WaitType::Wait,
        );
        if stat != file::Status::OpOk || bytes_to_read != remaining_bytes as FwSizeType {
            f.close();
            return Err(CrcStat::FailedFileRead);
        }
        hash.update(&block_data[..remaining_bytes]);
    }

    f.close();
    Ok(hash.finalize())
}

/// Build the sidecar name `<fname>.CRC32` in `hash_filename` (C++
/// `hashFilename.format("%s%s", fname, HASH_EXTENSION_STRING)`).
///
/// Returns [`CrcStat::FailedFileNameTooLong`] when the result does not fit
/// in an `Fw::FileNameString` (C++ `FormatStatus::OVERFLOWED`, reported
/// since nasa/fprime#5803 instead of asserted).
fn hash_file_name<'a>(
    fname: &str,
    hash_filename: &'a mut FileNameString,
) -> Result<&'a str, CrcStat> {
    // C++ parity: `%s` stops at the first NUL, as `FwString::set` does, so
    // the length that must fit is the C-string length of `fname`.
    let fname_len = fname.bytes().position(|b| b == 0).unwrap_or(fname.len());
    if fname_len + HASH_EXTENSION_STRING.len() > FileNameString::max_length() {
        return Err(CrcStat::FailedFileNameTooLong);
    }
    hash_filename.set(fname);
    hash_filename.append(HASH_EXTENSION_STRING);
    // Unreachable in practice: a `&str` cut at a NUL (an ASCII byte) plus an
    // ASCII extension is valid UTF-8. The OSAL takes `&str`, so a non-UTF-8
    // name could not be opened; report it as the open failure it would be.
    hash_filename.as_str().ok_or(CrcStat::FailedFileCrcOpen)
}

/// Compute the CRC-32 of the file at `fname` and write the 4-byte
/// big-endian sidecar `<fname>.CRC32`
/// (port of `Utils::create_checksum_file`).
///
/// Returns [`CrcStat::PassedFileCrcWrite`] on success. The data file is read
/// before the sidecar name is checked, so a missing data file reports
/// [`CrcStat::FailedFileSize`] even when its name is too long.
///
/// C++ parity: the sidecar is opened `OPEN_WRITE` — created if missing,
/// NOT truncated — and written with the two-argument `write` (WAIT, i.e.
/// synced to disk).
pub fn create_checksum_file(fname: &str) -> CrcStat {
    let checksum = match compute_file_crc(fname) {
        Ok(crc) => crc,
        Err(stat) => return stat,
    };

    // The filename is caller-supplied input: an overlong name is a
    // reportable failure, not a coding error (nasa/fprime#5803).
    let mut hash_filename = FileNameString::new();
    let hash_path = match hash_file_name(fname, &mut hash_filename) {
        Ok(path) => path,
        Err(stat) => return stat,
    };

    let mut f = File::new();
    if f.open(hash_path, Mode::OpenWrite) != file::Status::OpOk {
        return CrcStat::FailedFileCrcOpen;
    }

    // C++ parity: serialized (big-endian) through Fw::SerialBuffer, so the
    // file does not depend on the host's byte order; `to_be_bytes` is the
    // same 4 bytes.
    let checksum_data = checksum.to_be_bytes();
    let mut bytes_to_write: FwSizeType = 0;
    let stat = f.write(&checksum_data, &mut bytes_to_write, WaitType::Wait);
    if stat != file::Status::OpOk || bytes_to_write != CHECKSUM_SIZE as FwSizeType {
        f.close();
        return CrcStat::FailedFileCrcWrite;
    }

    f.close();
    CrcStat::PassedFileCrcWrite
}

/// Read the stored CRC-32 out of the sidecar `<fname>.CRC32` (its first 4
/// bytes, big-endian) into `checksum_from_file`
/// (port of `Utils::read_crc32_from_file`).
///
/// Returns [`CrcStat::PassedFileCrcCheck`] on success,
/// [`CrcStat::FailedFileNameTooLong`] when the sidecar name does not fit in
/// an `Fw::FileNameString`, [`CrcStat::FailedFileCrcOpen`] /
/// [`CrcStat::FailedFileCrcRead`] (fewer than 4 bytes) otherwise. Trailing
/// bytes are ignored. On failure `checksum_from_file` is left unmodified.
pub fn read_crc32_from_file(fname: &str, checksum_from_file: &mut u32) -> CrcStat {
    // See create_checksum_file(): overlong names are reported, not asserted.
    let mut hash_filename = FileNameString::new();
    let hash_path = match hash_file_name(fname, &mut hash_filename) {
        Ok(path) => path,
        Err(stat) => return stat,
    };

    let mut f = File::new();
    if f.open(hash_path, Mode::OpenRead) != file::Status::OpOk {
        return CrcStat::FailedFileCrcOpen;
    }

    let mut checksum_data = [0u8; CHECKSUM_SIZE];
    let mut checksum_from_file_size: FwSizeType = 0;
    let stat = f.read(
        &mut checksum_data,
        &mut checksum_from_file_size,
        WaitType::Wait,
    );
    if stat != file::Status::OpOk || checksum_from_file_size != CHECKSUM_SIZE as FwSizeType {
        f.close();
        return CrcStat::FailedFileCrcRead;
    }

    // C++ parity: deserialized to match create_checksum_file (big-endian).
    *checksum_from_file = u32::from_be_bytes(checksum_data);
    f.close();
    CrcStat::PassedFileCrcCheck
}

/// Recompute the CRC-32 of `fname` and compare against the sidecar
/// (port of `Utils::verify_checksum`).
///
/// C++ parity: `expected` (the sidecar value) and `actual` (the recomputed
/// value) are written in BOTH the pass and fail cases; on an earlier failure
/// (size/open/read of the data file, then [`read_crc32_from_file`]'s
/// statuses, including [`CrcStat::FailedFileNameTooLong`]) they are left
/// unmodified.
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
    use fprime_config::FILE_NAME_STRING_SIZE;
    use fprime_fw::SerBuf;
    use std::fs;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    /// A unique, empty scratch directory removed on drop.
    struct TempDir {
        path: PathBuf,
    }

    impl TempDir {
        fn new(tag: &str) -> Self {
            static COUNTER: AtomicU32 = AtomicU32::new(0);
            let path = std::env::temp_dir().join(format!(
                "fprime_utils_crc_{}_{}_{tag}",
                std::process::id(),
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            let _ = fs::remove_dir_all(&path);
            fs::create_dir_all(&path).unwrap();
            Self { path }
        }

        /// The upstream `TEST_FILE` inside this directory.
        fn test_file(&self) -> String {
            self.file("crc_checker_test_file.bin")
        }

        fn file(&self, name: &str) -> String {
            self.path.join(name).to_str().unwrap().to_string()
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.path);
        }
    }

    fn sidecar(fname: &str) -> String {
        format!("{fname}{HASH_EXTENSION_STRING}")
    }

    /// Upstream `writeFile` (`OPEN_CREATE`, `OVERWRITE`).
    fn write_file(fname: &str, data: &[u8]) {
        fs::write(fname, data).unwrap();
    }

    /// Upstream `removeTestFiles`.
    fn remove_test_files(fname: &str) {
        let _ = fs::remove_file(fname);
        let _ = fs::remove_file(sidecar(fname));
    }

    /// Deterministic stand-in for `STest::Pick` (xorshift32, fixed seed).
    struct Pick(u32);

    impl Pick {
        fn new() -> Self {
            Self(0x2545_F491)
        }

        /// A value in `lower..=upper` (`STest::Pick::lowerUpper`).
        fn lower_upper(&mut self, lower: u32, upper: u32) -> u32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            lower + self.0 % (upper - lower + 1)
        }

        /// Upstream `randomData`.
        fn random_data(&mut self, size: usize) -> Vec<u8> {
            (0..size).map(|_| self.lower_upper(0, 255) as u8).collect()
        }
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
        assert_eq!(CrcStat::FailedFileNameTooLong as i32, 10);
        assert_eq!(CRC_FILE_READ_BLOCK, 2048);
    }

    /// Port of `CRCCheckerTest.NominalRoundTripSizes`.
    #[test]
    fn nominal_round_trip_sizes() {
        let dir = TempDir::new("nominal");
        let test_file = dir.test_file();
        let mut pick = Pick::new();
        let block = CRC_FILE_READ_BLOCK;
        for size in [0, 1, block - 1, block, block + 1, (3 * block) + 7] {
            let data = pick.random_data(size);
            write_file(&test_file, &data);

            assert_eq!(
                create_checksum_file(&test_file),
                CrcStat::PassedFileCrcWrite,
                "size {size}"
            );
            let (mut expected, mut actual) = (0u32, 0u32);
            assert_eq!(
                verify_checksum(&test_file, &mut expected, &mut actual),
                CrcStat::PassedFileCrcCheck,
                "size {size}"
            );
            assert_eq!(expected, actual);
            assert_eq!(actual, Hash::hash_u32(&data));
            remove_test_files(&test_file);
        }
    }

    /// Port of `CRCCheckerTest.GoldenValue`, plus the literal sidecar bytes.
    #[test]
    fn golden_value() {
        let dir = TempDir::new("golden");
        let test_file = dir.test_file();
        // Standard CRC-32 (IEEE 802.3) check value for "123456789".
        write_file(&test_file, b"123456789");
        assert_eq!(
            create_checksum_file(&test_file),
            CrcStat::PassedFileCrcWrite
        );

        let mut from_file = 0u32;
        assert_eq!(
            read_crc32_from_file(&test_file, &mut from_file),
            CrcStat::PassedFileCrcCheck
        );
        assert_eq!(from_file, 0xCBF4_3926);

        // The static hash interface produces the same digest...
        let buffer = Hash::hash(b"123456789");
        assert_eq!(buffer.as_big_endian_u32(), 0xCBF4_3926);
        assert_eq!(buffer, Hash::hash(b"123456789"));
        // ...and the sidecar holds exactly its serialized (big-endian) bytes.
        let on_disk = fs::read(sidecar(&test_file)).unwrap();
        assert_eq!(on_disk, [0xCB, 0xF4, 0x39, 0x26]);
        assert_eq!(on_disk, buffer.as_slice());

        // File extension helpers
        assert_eq!(Hash::get_file_extension_string(), HASH_EXTENSION_STRING);
        assert_eq!(
            Hash::get_file_extension_length(),
            HASH_EXTENSION_STRING.len() as FwSizeType
        );
        let mut extended = FileNameString::new();
        // C++ ignores the status here; the name fits, so it is SUCCESS.
        assert_eq!(
            Hash::add_file_extension(
                FileNameString::from(test_file.as_str()).as_bytes(),
                &mut extended
            ),
            fprime_fw::StringFormatStatus::Success
        );
        assert_eq!(extended.as_str(), Some(sidecar(&test_file).as_str()));
    }

    /// Port of `CRCCheckerTest.MissingDataFile`.
    #[test]
    fn missing_data_file() {
        let dir = TempDir::new("missing");
        let test_file = dir.test_file();
        assert_eq!(create_checksum_file(&test_file), CrcStat::FailedFileSize);
        let (mut expected, mut actual) = (7u32, 8u32);
        assert_eq!(
            verify_checksum(&test_file, &mut expected, &mut actual),
            CrcStat::FailedFileSize
        );
        // C++ parity: untouched on an early failure.
        assert_eq!((expected, actual), (7, 8));
        assert!(fs::metadata(sidecar(&test_file)).is_err());
    }

    /// Port of `CRCCheckerTest.MissingCrcFile`.
    #[test]
    fn missing_crc_file() {
        let dir = TempDir::new("nocrc");
        let test_file = dir.test_file();
        write_file(&test_file, &Pick::new().random_data(64));

        let mut from_file = 42u32;
        assert_eq!(
            read_crc32_from_file(&test_file, &mut from_file),
            CrcStat::FailedFileCrcOpen
        );
        assert_eq!(from_file, 42); // untouched
        let (mut expected, mut actual) = (7u32, 8u32);
        assert_eq!(
            verify_checksum(&test_file, &mut expected, &mut actual),
            CrcStat::FailedFileCrcOpen
        );
        assert_eq!((expected, actual), (7, 8));
    }

    /// Port of `CRCCheckerTest.TruncatedCrcFile`.
    #[test]
    fn truncated_crc_file() {
        let dir = TempDir::new("truncated");
        let test_file = dir.test_file();
        write_file(&test_file, &Pick::new().random_data(64));
        // A checksum file shorter than a U32.
        write_file(&sidecar(&test_file), &[0xAB, 0xCD]);

        let mut from_file = 42u32;
        assert_eq!(
            read_crc32_from_file(&test_file, &mut from_file),
            CrcStat::FailedFileCrcRead
        );
        assert_eq!(from_file, 42); // untouched
        let (mut expected, mut actual) = (7u32, 8u32);
        assert_eq!(
            verify_checksum(&test_file, &mut expected, &mut actual),
            CrcStat::FailedFileCrcRead
        );
        assert_eq!((expected, actual), (7, 8));
    }

    /// Port of `CRCCheckerTest.CorruptedDataFails`.
    #[test]
    fn corrupted_data_fails() {
        let dir = TempDir::new("corrupt");
        let test_file = dir.test_file();
        let mut pick = Pick::new();
        let mut data = pick.random_data(256);
        write_file(&test_file, &data);
        assert_eq!(
            create_checksum_file(&test_file),
            CrcStat::PassedFileCrcWrite
        );
        let original = Hash::hash_u32(&data);

        // Corrupt one byte of the data file.
        let index = pick.lower_upper(0, (data.len() - 1) as u32) as usize;
        data[index] ^= 0xFF;
        write_file(&test_file, &data);

        let (mut expected, mut actual) = (0u32, 0u32);
        assert_eq!(
            verify_checksum(&test_file, &mut expected, &mut actual),
            CrcStat::FailedFileCrcCheck
        );
        assert_ne!(expected, actual);
        // C++ parity: expected/actual are populated in the fail case too.
        assert_eq!(expected, original);
        assert_eq!(actual, Hash::hash_u32(&data));
    }

    /// Port of `CRCCheckerTest.RandomizedSoak`.
    #[test]
    fn randomized_soak() {
        let dir = TempDir::new("soak");
        let test_file = dir.test_file();
        let mut pick = Pick::new();
        let block = CRC_FILE_READ_BLOCK as u32;
        for _ in 0..25 {
            let size = pick.lower_upper(0, 4 * block) as usize;
            let mut data = pick.random_data(size);
            write_file(&test_file, &data);

            assert_eq!(
                create_checksum_file(&test_file),
                CrcStat::PassedFileCrcWrite
            );
            let (mut expected, mut actual) = (0u32, 0u32);
            assert_eq!(
                verify_checksum(&test_file, &mut expected, &mut actual),
                CrcStat::PassedFileCrcCheck
            );

            if size > 0 {
                // Mutate a random bit: verification must fail.
                let index = pick.lower_upper(0, (size - 1) as u32) as usize;
                data[index] ^= 1u8 << pick.lower_upper(0, 7);
                write_file(&test_file, &data);
                assert_eq!(
                    verify_checksum(&test_file, &mut expected, &mut actual),
                    CrcStat::FailedFileCrcCheck
                );
                assert_ne!(expected, actual);
            }
            remove_test_files(&test_file);
        }
    }

    /// A path of exactly `len` bytes inside `dir`.
    fn name_of_len(dir: &TempDir, len: usize, fill: char) -> String {
        let prefix = format!("{}/", dir.path.to_str().unwrap());
        assert!(prefix.len() < len, "temp dir path too long for this test");
        let name = format!("{prefix}{}", fill.to_string().repeat(len - prefix.len()));
        assert_eq!(name.len(), len);
        name
    }

    /// Port of `CRCCheckerTest.FileNameTooLong` (nasa/fprime#5803), with
    /// absolute names in a scratch directory instead of the working
    /// directory.
    #[test]
    fn file_name_too_long() {
        let dir = TempDir::new("toolong");
        let ext = Hash::get_file_extension_length() as usize;
        // Fw::FileNameString holds FileNameStringSize characters.
        assert_eq!(FileNameString::max_length(), FILE_NAME_STRING_SIZE);
        let longest_ok = FILE_NAME_STRING_SIZE - ext;
        let data = b"123456789";

        for len in [longest_ok + 1, FILE_NAME_STRING_SIZE] {
            let name = name_of_len(&dir, len, 'x');
            write_file(&name, data);

            assert_eq!(
                create_checksum_file(&name),
                CrcStat::FailedFileNameTooLong,
                "len {len}"
            );
            let mut from_file = 42u32;
            assert_eq!(
                read_crc32_from_file(&name, &mut from_file),
                CrcStat::FailedFileNameTooLong,
                "len {len}"
            );
            assert_eq!(from_file, 42);
            let (mut expected, mut actual) = (7u32, 8u32);
            assert_eq!(
                verify_checksum(&name, &mut expected, &mut actual),
                CrcStat::FailedFileNameTooLong,
                "len {len}"
            );
            assert_eq!((expected, actual), (7, 8));

            let _ = fs::remove_file(&name);
        }

        // The longest name that fits round-trips normally.
        let ok_name = name_of_len(&dir, longest_ok, 'y');
        write_file(&ok_name, data);
        assert_eq!(create_checksum_file(&ok_name), CrcStat::PassedFileCrcWrite);
        assert_eq!(sidecar(&ok_name).len(), FILE_NAME_STRING_SIZE);
        let (mut expected, mut actual) = (0u32, 0u32);
        assert_eq!(
            verify_checksum(&ok_name, &mut expected, &mut actual),
            CrcStat::PassedFileCrcCheck
        );
        assert_eq!(expected, 0xCBF4_3926);
        remove_test_files(&ok_name);
    }

    /// C++ parity: the sidecar name is formatted only after the data file has
    /// been read, so data-file failures take precedence over
    /// `FAILED_FILE_NAME_TOO_LONG` in create/verify.
    #[test]
    fn data_file_checks_precede_name_length_check() {
        let dir = TempDir::new("order");
        let name = name_of_len(&dir, FILE_NAME_STRING_SIZE, 'z'); // not created
        assert_eq!(create_checksum_file(&name), CrcStat::FailedFileSize);
        let (mut expected, mut actual) = (0u32, 0u32);
        assert_eq!(
            verify_checksum(&name, &mut expected, &mut actual),
            CrcStat::FailedFileSize
        );
        let mut from_file = 0u32;
        assert_eq!(
            read_crc32_from_file(&name, &mut from_file),
            CrcStat::FailedFileNameTooLong
        );
    }

    /// C++ parity: `%s` stops at the first NUL, so a name with an embedded
    /// NUL addresses the sidecar of its prefix and only the prefix counts
    /// toward the length limit.
    #[test]
    fn sidecar_name_stops_at_nul() {
        let dir = TempDir::new("nul");
        let test_file = dir.test_file();
        write_file(&sidecar(&test_file), &0x0102_0304u32.to_be_bytes());

        let name = format!("{test_file}\0{}", "g".repeat(FILE_NAME_STRING_SIZE));
        let mut from_file = 0u32;
        assert_eq!(
            read_crc32_from_file(&name, &mut from_file),
            CrcStat::PassedFileCrcCheck
        );
        assert_eq!(from_file, 0x0102_0304);
    }

    /// C++ parity: the sidecar is opened `OPEN_WRITE` (no truncate), so a
    /// longer pre-existing file keeps its tail; readers use the first 4
    /// bytes only.
    #[test]
    fn sidecar_is_not_truncated() {
        let dir = TempDir::new("notrunc");
        let test_file = dir.test_file();
        write_file(&test_file, b"123456789");
        write_file(
            &sidecar(&test_file),
            &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
        );

        assert_eq!(
            create_checksum_file(&test_file),
            CrcStat::PassedFileCrcWrite
        );
        assert_eq!(
            fs::read(sidecar(&test_file)).unwrap(),
            [0xCB, 0xF4, 0x39, 0x26, 0x55, 0x66, 0x77, 0x88]
        );
        let (mut expected, mut actual) = (0u32, 0u32);
        assert_eq!(
            verify_checksum(&test_file, &mut expected, &mut actual),
            CrcStat::PassedFileCrcCheck
        );
        assert_eq!(expected, 0xCBF4_3926);
    }

    /// A sidecar path that cannot be opened for writing is
    /// `FAILED_FILE_CRC_OPEN`.
    #[test]
    fn unopenable_sidecar_reports_crc_open_failure() {
        let dir = TempDir::new("crcopen");
        let test_file = dir.test_file();
        write_file(&test_file, b"data");
        // A directory where the sidecar should go.
        fs::create_dir(sidecar(&test_file)).unwrap();
        assert_eq!(create_checksum_file(&test_file), CrcStat::FailedFileCrcOpen);
    }
}
