//! Port of `Os::ValidateFile` — validate a file against its `.CRC32` hash
//! file, or create that hash file — and of `Os::ValidatedFile`
//! ([`ValidatedFile`]), the object that pairs a file with its
//! `<file>.CRC32` hash file name.
//!
//! C++ sources: `Os/ValidateFile.hpp`, `Os/ValidateFileCommon.cpp`,
//! `Os/ValidatedFile.{hpp,cpp}`; tests `Os/test/ut/OsValidateFileTest.cpp`
//! and the `ValidateFile` interop case in `Utils/test/ut/CRCCheckerTester.cpp`
//! (upstream has no `ValidatedFile` unit test; `Svc::BufferLogger`'s tests
//! exercise it, and so do the ones below).
//! Analysis: `docs/cpp-analysis/os.md` (ValidateFile section) and the
//! ComLogger sidecar section of `docs/cpp-analysis/svc-misc.md`.
//!
//! # Placement
//!
//! Upstream this lives in `Os`, but it is built on `Utils::Hash`. In the C++
//! tree that is a library-level dependency in both directions (`Os` links
//! `Utils/Hash` for this file, `Utils/CRCChecker` links `Os::File`); a Rust
//! crate graph has to pick one direction. It lives here, in `fprime-utils`,
//! with `fprime-utils -> fprime-os`:
//! - `fprime-os` stays the OSAL leaf over `fprime-fw`/`fprime-config` only
//!   (the seam the `no_std` work targets), and does not grow a dependency on
//!   a support library for a single file;
//! - the opposite edge (`fprime-os -> fprime-utils`) would forbid
//!   `fprime-utils` from ever reaching `Os::File`, which `Utils::CRCChecker`
//!   uses upstream;
//! - it sits next to [`crate::crc_checker`], the other producer/consumer of
//!   the same `.CRC32` sidecar, so the two are documented side by side
//!   below.
//!
//! The C++ namespace is kept in the names: the module is `validate_file`,
//! its status enum is [`Status`] (`Os::ValidateFile::Status`), and the free
//! functions keep the C++ names in `snake_case`.
//!
//! # The `.CRC32` sidecar: `ValidateFile` and `crc_checker` side by side
//!
//! Both write and read the same 4-byte file, so files are interchangeable
//! between them (tested below, as upstream's `CRCCheckerTester` does):
//!
//! | | `validate_file` (`Os::ValidateFile`) | [`crc_checker`](crate::crc_checker) (`Utils::CRCChecker`) |
//! |---|---|---|
//! | Contents | standard CRC-32 (IEEE 802.3, complemented) | same |
//! | Byte order | **big-endian**: [`Hash::finalize_buffer`] serializes the `u32` through the F Prime serializer | **big-endian**: `Fw::SerialBuffer` serialization of the `u32` |
//! | Size | exactly [`HASH_DIGEST_LENGTH`] (4) bytes | exactly 4 bytes |
//! | Name | caller passes `hash_file_name` explicitly (callers use `<file>` + [`HASH_EXTENSION_STRING`]) | always `<fname>.CRC32` |
//! | Read chunk | [`VFILE_HASH_CHUNK_SIZE`] (256) bytes, `NO_WAIT` reads, bound from the size at open | `CRC_FILE_READ_BLOCK` (2048) bytes, exact-size reads |
//! | Reading the sidecar | first 4 bytes, trailing bytes ignored; short => [`Status::ValidationFileBadSize`] | first 4 bytes, trailing bytes ignored; short => `FailedFileCrcRead` |
//! | Writing the sidecar | `OPEN_WRITE` (create, **no truncate**) | same |
//! | Overlong name | not checked (the caller passes the name) | `<fname>.CRC32` longer than a `FileNameString` => `FailedFileNameTooLong` |
//!
//! History: `Utils::CRCChecker` used to `reinterpret_cast` the `U32`, so its
//! sidecar was NATIVE-endian (little-endian on typical targets) while this
//! module's was big-endian. Upstream fixed that in nasa/fprime#5775, which is
//! in the 4dee010 model this port tracks; a `.CRC32` written by an older
//! CRCChecker build on a little-endian host reads back byte-swapped through
//! either API here.

use fprime_config::FwSizeType;
use fprime_fw::{FwDefaultString, SerBuf, StringFormatStatus, fw_assert};
use fprime_os::file::{self, File, Mode, WaitType};
use fprime_os::filesystem;

#[cfg(doc)]
use crate::hash::HASH_EXTENSION_STRING;
use crate::hash::{HASH_DIGEST_LENGTH, Hash, HashBuffer};

/// Chunk size used to read the file being hashed (C++ `VFILE_HASH_CHUNK_SIZE`
/// `#define` in `Os/ValidateFile.hpp`).
///
/// Not in `fprime-config`: upstream defines it in the `Os` header itself, not
/// in a `config/` header, so it is not project-configurable.
pub const VFILE_HASH_CHUNK_SIZE: usize = 256;

/// Validation status (port of `Os::ValidateFile::Status`, exact C++
/// discriminants; a plain C++ enum, hence `i32`).
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// `VALIDATION_OK` — the validation of the file passed (or the hash file
    /// was written).
    ValidationOk = 0,
    /// `VALIDATION_FAIL` — the validation of the file did not pass.
    ValidationFail = 1,
    /// `FILE_DOESNT_EXIST` — file doesn't exist (for read).
    FileDoesntExist = 2,
    /// `FILE_NO_PERMISSION` — no permission to read/write the file.
    FileNoPermission = 3,
    /// `FILE_BAD_SIZE` — invalid size parameter on the file.
    FileBadSize = 4,
    /// `VALIDATION_FILE_DOESNT_EXIST` — validation file doesn't exist (for
    /// read).
    ValidationFileDoesntExist = 5,
    /// `VALIDATION_FILE_NO_PERMISSION` — no permission to read/write the
    /// validation file.
    ValidationFileNoPermission = 6,
    /// `VALIDATION_FILE_BAD_SIZE` — invalid size parameter on the validation
    /// file.
    ValidationFileBadSize = 7,
    /// `NO_SPACE` — no space left on the device for writing.
    NoSpace = 8,
    /// `OTHER_ERROR` — catch-all for other errors.
    OtherError = 9,
}

/// Which file a [`file::Status`] came from (C++ `StatusFileType`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StatusFileType {
    /// The data file being hashed (C++ `FileType`).
    File,
    /// The `.CRC32` hash file (C++ `HashFileType`).
    HashFile,
}

/// Port of `Os::translateStatus`.
///
/// C++ parity: statuses the table does not list (`NOT_SUPPORTED`,
/// `INVALID_ARGUMENT`, ...) report [`Status::OtherError`] rather than assert.
/// The C++ `FW_ASSERT` on an unknown `StatusFileType` is unrepresentable
/// here.
fn translate_status(status: file::Status, file_type: StatusFileType) -> Status {
    match file_type {
        StatusFileType::File => match status {
            file::Status::OpOk => Status::ValidationOk,
            file::Status::DoesntExist => Status::FileDoesntExist,
            file::Status::NoSpace => Status::NoSpace,
            file::Status::NoPermission => Status::FileNoPermission,
            file::Status::BadSize => Status::FileBadSize,
            _ => Status::OtherError,
        },
        StatusFileType::HashFile => match status {
            file::Status::OpOk => Status::ValidationOk,
            file::Status::DoesntExist => Status::ValidationFileDoesntExist,
            file::Status::NoSpace => Status::NoSpace,
            file::Status::NoPermission => Status::ValidationFileNoPermission,
            file::Status::BadSize => Status::ValidationFileBadSize,
            _ => Status::OtherError,
        },
    }
}

/// Port of `Os::computeHash`: the CRC-32 of `file_name`, read in
/// [`VFILE_HASH_CHUNK_SIZE`] chunks. `hash_buffer` is written only on
/// success.
fn compute_hash(file_name: &str, hash_buffer: &mut HashBuffer) -> file::Status {
    let mut file = File::new();
    let status = file.open(file_name, Mode::OpenRead);
    if status != file::Status::OpOk {
        return status;
    }

    // C++ parity: any FileSystem::getFileSize failure is reported as the
    // file status BAD_SIZE.
    let mut file_size: FwSizeType = 0;
    if filesystem::get_file_size(file_name, &mut file_size) != filesystem::Status::OpOk {
        return file::Status::BadSize;
    }
    // C++ parity: the iteration bound is computed from the size at open; a
    // file that grows while being hashed exhausts it with data still unread,
    // which is a BAD_SIZE error, not a hang.
    let max_itr = file_size / (VFILE_HASH_CHUNK_SIZE as FwSizeType) + 1;

    let mut hash = Hash::new();
    hash.init();
    let mut buffer = [0u8; VFILE_HASH_CHUNK_SIZE];
    let mut size: FwSizeType = 0;
    let mut cnt: FwSizeType = 0;
    while cnt <= max_itr {
        size = buffer.len() as FwSizeType;
        let status = file.read(&mut buffer, &mut size, WaitType::NoWait);
        if status != file::Status::OpOk {
            return status;
        }
        if size == 0 {
            break;
        }
        hash.update(&buffer[..size as usize]);
        cnt += 1;
    }
    file.close();

    if size != 0 {
        return file::Status::BadSize;
    }

    *hash_buffer = hash.finalize_buffer();
    file::Status::OpOk
}

/// Port of `Os::readHash`: the first [`HASH_DIGEST_LENGTH`] bytes of
/// `hash_file_name`. Fewer bytes is `BAD_SIZE`; trailing bytes are ignored.
/// `hash_buffer` is written only on success.
fn read_hash(hash_file_name: &str, hash_buffer: &mut HashBuffer) -> file::Status {
    let mut hash_file = File::new();
    let status = hash_file.open(hash_file_name, Mode::OpenRead);
    if status != file::Status::OpOk {
        return status;
    }

    let mut saved_hash = [0u8; HASH_DIGEST_LENGTH];
    let mut size = HASH_DIGEST_LENGTH as FwSizeType;
    // C++ parity: the two-argument `read` overload defaults to WAIT.
    let status = hash_file.read(&mut saved_hash, &mut size, WaitType::Wait);
    if status != file::Status::OpOk {
        return status;
    }
    if size != HASH_DIGEST_LENGTH as FwSizeType {
        return file::Status::BadSize;
    }
    hash_file.close();

    *hash_buffer = HashBuffer::from_bytes(&saved_hash[..size as usize]);
    file::Status::OpOk
}

/// Port of `Os::writeHash`: the valid bytes of `hash_buffer` (its serialize
/// length, 4 for any finalized digest) to `hash_file_name`.
///
/// C++ parity: opened `OPEN_WRITE` — created if missing but NOT truncated, so
/// a pre-existing longer file keeps its tail (reads only ever look at the
/// first 4 bytes).
fn write_hash(hash_file_name: &str, hash_buffer: &HashBuffer) -> file::Status {
    let mut hash_file = File::new();
    let status = hash_file.open(hash_file_name, Mode::OpenWrite);
    if status != file::Status::OpOk {
        return status;
    }

    let digest = hash_buffer.as_slice();
    let mut size = digest.len() as FwSizeType;
    let status = hash_file.write(digest, &mut size, WaitType::NoWait);
    if status != file::Status::OpOk {
        return status;
    }
    if size != digest.len() as FwSizeType {
        return file::Status::BadSize;
    }
    hash_file.close();

    file::Status::OpOk
}

/// Validate the contents of `file_name` against the hash stored in
/// `hash_file_name` (C++ `validate(fileName, hashFileName)`).
pub fn validate(file_name: &str, hash_file_name: &str) -> Status {
    // C++ parity: the hash value is computed and discarded.
    let mut hash_buffer = HashBuffer::new();
    validate_with_hash_buffer(file_name, hash_file_name, &mut hash_buffer)
}

/// Validate `file_name` against `hash_file_name` and also return the hash
/// (C++ `validate(fileName, hashFileName, HashBuffer&)`).
///
/// The hash file is read FIRST, so a missing hash file reports
/// [`Status::ValidationFileDoesntExist`] even when the data file is missing
/// too. `hash_buffer` receives the stored (= computed) digest only on
/// [`Status::ValidationOk`]; on every other status it is left unmodified.
pub fn validate_with_hash_buffer(
    file_name: &str,
    hash_file_name: &str,
    hash_buffer: &mut HashBuffer,
) -> Status {
    let mut saved_hash = HashBuffer::new();
    let status = read_hash(hash_file_name, &mut saved_hash);
    if status != file::Status::OpOk {
        return translate_status(status, StatusFileType::HashFile);
    }

    let mut computed_hash = HashBuffer::new();
    let status = compute_hash(file_name, &mut computed_hash);
    if status != file::Status::OpOk {
        return translate_status(status, StatusFileType::File);
    }

    if saved_hash != computed_hash {
        return Status::ValidationFail;
    }

    *hash_buffer = saved_hash;
    Status::ValidationOk
}

/// Hash `file_name` and store the digest in `hash_file_name`
/// (C++ `createValidation(fileName, hashFileName)`).
pub fn create_validation(file_name: &str, hash_file_name: &str) -> Status {
    // C++ parity: the hash value is computed and discarded.
    let mut hash_buffer = HashBuffer::new();
    create_validation_with_hash_buffer(file_name, hash_file_name, &mut hash_buffer)
}

/// Hash `file_name`, store the digest in `hash_file_name`, and also return it
/// (C++ `createValidation(fileName, hash, HashBuffer&)`).
///
/// C++ parity: `hash_buffer` is written as soon as the data file has been
/// hashed, so it holds the digest even when writing the hash file then fails
/// (`ValidationFile*`/[`Status::NoSpace`]/[`Status::OtherError`]); it is left
/// unmodified only when hashing the data file fails.
pub fn create_validation_with_hash_buffer(
    file_name: &str,
    hash_file_name: &str,
    hash_buffer: &mut HashBuffer,
) -> Status {
    let status = compute_hash(file_name, hash_buffer);
    if status != file::Status::OpOk {
        return translate_status(status, StatusFileType::File);
    }

    let status = write_hash(hash_file_name, hash_buffer);
    if status != file::Status::OpOk {
        return translate_status(status, StatusFileType::HashFile);
    }

    Status::ValidationOk
}

/// Write a pre-computed hash to `hash_file_name`
/// (C++ `createValidation(hashFileName, const HashBuffer&)`).
///
/// Writes exactly the valid bytes of `hash_buffer` (its serialize length);
/// errors are reported as hash-file statuses.
pub fn create_validation_from_hash_buffer(
    hash_file_name: &str,
    hash_buffer: &HashBuffer,
) -> Status {
    let status = write_hash(hash_file_name, hash_buffer);
    if status != file::Status::OpOk {
        return translate_status(status, StatusFileType::HashFile);
    }
    Status::ValidationOk
}

/// A file paired with its `<file>.CRC32` hash file (port of
/// `Os::ValidatedFile`, `Os/ValidatedFile.{hpp,cpp}`).
///
/// A thin object wrapper over the free functions above: it fixes the hash
/// file name to `<file name>` + [`Hash::get_file_extension_string`] (built
/// with [`Hash::add_file_extension`], as upstream) and keeps the hash value
/// from the last validation or hash-file creation.
///
/// It lives in this module for the same reason the free functions do (see
/// "Placement" above): upstream it is an `Os` class built on `Utils::Hash`.
/// Like them it is `std`-only. The C++ `Fw::String` members are
/// [`FwDefaultString`]s (`FW_FIXED_LENGTH_STRING_SIZE`, 256 bytes).
#[derive(Debug, Clone)]
pub struct ValidatedFile {
    /// C++ `m_fileName`.
    file_name: FwDefaultString,
    /// C++ `m_hashFileName`.
    hash_file_name: FwDefaultString,
    /// C++ `m_hashBuffer`: the hash after creating or loading a validation
    /// file.
    hash_buffer: HashBuffer,
}

impl ValidatedFile {
    /// Construct a validated file (C++ `ValidatedFile(const char*)`).
    ///
    /// The file name is truncated to the string capacity as the C++
    /// `Fw::String` assignment does. C++ parity: the hash file name not
    /// fitting (`<file name>.CRC32` longer than 256 bytes) is an
    /// `FW_ASSERT` — a truncated hash file name would silently validate
    /// against the wrong file.
    #[must_use]
    pub fn new(file_name: &str) -> Self {
        let mut name = FwDefaultString::new();
        name.set(file_name);
        let mut hash_file_name = FwDefaultString::new();
        let status = Hash::add_file_extension(name.as_bytes(), &mut hash_file_name);
        fw_assert!(status == StringFormatStatus::Success, status as i32);
        Self {
            file_name: name,
            hash_file_name,
            hash_buffer: HashBuffer::new(),
        }
    }

    /// Validate the file against its hash file (C++ `validate()`); on
    /// [`Status::ValidationOk`] the stored hash is kept
    /// ([`get_hash_buffer`](Self::get_hash_buffer)), on any other status it
    /// is left unmodified.
    pub fn validate(&mut self) -> Status {
        validate_with_hash_buffer(
            self.file_name.as_str().unwrap_or_default(),
            self.hash_file_name.as_str().unwrap_or_default(),
            &mut self.hash_buffer,
        )
    }

    /// Hash the file and write the hash file (C++ `createHashFile()`). The
    /// hash is kept even when writing the hash file fails (see
    /// [`create_validation_with_hash_buffer`]).
    pub fn create_hash_file(&mut self) -> Status {
        create_validation_with_hash_buffer(
            self.file_name.as_str().unwrap_or_default(),
            self.hash_file_name.as_str().unwrap_or_default(),
            &mut self.hash_buffer,
        )
    }

    /// Write a pre-computed hash to the hash file (C++
    /// `createHashFile(const HashBuffer&)`). The hash is stored before the
    /// write, so it is kept whatever the status.
    pub fn create_hash_file_from(&mut self, hash_buffer: &HashBuffer) -> Status {
        self.hash_buffer = hash_buffer.clone();
        create_validation_from_hash_buffer(
            self.hash_file_name.as_str().unwrap_or_default(),
            &self.hash_buffer,
        )
    }

    /// The file name (C++ `getFileName()`).
    #[must_use]
    pub fn get_file_name(&self) -> &FwDefaultString {
        &self.file_name
    }

    /// The hash file name, `<file name>.CRC32` (C++ `getHashFileName()`).
    #[must_use]
    pub fn get_hash_file_name(&self) -> &FwDefaultString {
        &self.hash_file_name
    }

    /// The hash from the last successful validation or hash-file creation
    /// (C++ `getHashBuffer()`); empty until then.
    #[must_use]
    pub fn get_hash_buffer(&self) -> &HashBuffer {
        &self.hash_buffer
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc_checker::{self, CrcStat};
    use crate::hash::HASH_EXTENSION_STRING;
    use fprime_fw::SerBufAny;
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
                "fprime_utils_vf_{}_{}_{tag}",
                std::process::id(),
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            let _ = fs::remove_dir_all(&path);
            fs::create_dir_all(&path).unwrap();
            Self { path }
        }

        fn file(&self, name: &str) -> String {
            self.path.join(name).to_str().unwrap().to_string()
        }

        /// Write `contents` to `name` in this directory; return its path.
        fn write(&self, name: &str, contents: &[u8]) -> String {
            let path = self.file(name);
            fs::write(&path, contents).unwrap();
            path
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.path);
        }
    }

    fn sidecar(name: &str) -> String {
        format!("{name}{HASH_EXTENSION_STRING}")
    }

    #[test]
    fn status_discriminants_match_cpp() {
        assert_eq!(Status::ValidationOk as i32, 0);
        assert_eq!(Status::ValidationFail as i32, 1);
        assert_eq!(Status::FileDoesntExist as i32, 2);
        assert_eq!(Status::FileNoPermission as i32, 3);
        assert_eq!(Status::FileBadSize as i32, 4);
        assert_eq!(Status::ValidationFileDoesntExist as i32, 5);
        assert_eq!(Status::ValidationFileNoPermission as i32, 6);
        assert_eq!(Status::ValidationFileBadSize as i32, 7);
        assert_eq!(Status::NoSpace as i32, 8);
        assert_eq!(Status::OtherError as i32, 9);
        assert_eq!(VFILE_HASH_CHUNK_SIZE, 256);
    }

    /// Port of `Os/test/ut/OsValidateFileTest.cpp` (`testValidateFile`).
    #[test]
    fn upstream_validate_file_sequence() {
        let dir = TempDir::new("upstream");
        let file_name = dir.write("data.bin", b"some file contents to validate\n");
        let nonexistent_file_name = dir.file("thisfiledoesnotexist");
        let hash_file_name = dir.file("hashed.hashed");
        let hardtoaccess_hash_file_name = dir.file("thisdirdoesnotexist/hashed.hashed");

        assert_eq!(
            create_validation(&file_name, &hash_file_name),
            Status::ValidationOk
        );
        assert_eq!(
            create_validation(&nonexistent_file_name, &hash_file_name),
            Status::FileDoesntExist
        );
        assert_eq!(
            create_validation(&file_name, &hardtoaccess_hash_file_name),
            Status::ValidationFileDoesntExist
        );

        let mut file_size: FwSizeType = 0;
        assert_eq!(
            filesystem::get_file_size(&hash_file_name, &mut file_size),
            filesystem::Status::OpOk
        );
        assert_eq!(file_size, HashBuffer::new().capacity() as FwSizeType);

        assert_eq!(validate(&file_name, &hash_file_name), Status::ValidationOk);
        // The data file's first 4 bytes are not its CRC.
        assert_eq!(validate(&file_name, &file_name), Status::ValidationFail);

        assert_eq!(
            filesystem::remove_file(&hash_file_name),
            filesystem::Status::OpOk
        );
    }

    #[test]
    fn sidecar_is_the_big_endian_standard_crc32() {
        let dir = TempDir::new("vector");
        let name = dir.write("check.bin", b"123456789");
        let mut digest = HashBuffer::new();
        assert_eq!(
            create_validation_with_hash_buffer(&name, &sidecar(&name), &mut digest),
            Status::ValidationOk
        );
        assert_eq!(fs::read(sidecar(&name)).unwrap(), [0xCB, 0xF4, 0x39, 0x26]);
        assert_eq!(digest.as_slice(), &[0xCB, 0xF4, 0x39, 0x26]);
        assert_eq!(digest.as_big_endian_u32(), 0xCBF4_3926);
    }

    #[test]
    fn hashing_crosses_chunk_boundaries() {
        let dir = TempDir::new("chunks");
        let sizes = [
            0,
            1,
            VFILE_HASH_CHUNK_SIZE - 1,
            VFILE_HASH_CHUNK_SIZE,
            VFILE_HASH_CHUNK_SIZE + 1,
            3 * VFILE_HASH_CHUNK_SIZE + 7,
        ];
        for size in sizes {
            let data: Vec<u8> = (0..size).map(|i| (i % 251) as u8).collect();
            let name = dir.write("data.bin", &data);
            let mut digest = HashBuffer::new();
            assert_eq!(
                create_validation_with_hash_buffer(&name, &sidecar(&name), &mut digest),
                Status::ValidationOk,
                "size {size}"
            );
            assert_eq!(digest, Hash::hash(&data), "size {size}");
            let mut validated = HashBuffer::new();
            assert_eq!(
                validate_with_hash_buffer(&name, &sidecar(&name), &mut validated),
                Status::ValidationOk,
                "size {size}"
            );
            assert_eq!(validated, digest, "size {size}");
            fs::remove_file(sidecar(&name)).unwrap();
        }
    }

    #[test]
    fn validate_reports_mismatch_and_leaves_the_hash_buffer_alone() {
        let dir = TempDir::new("mismatch");
        let name = dir.write("data.bin", b"original contents");
        assert_eq!(
            create_validation(&name, &sidecar(&name)),
            Status::ValidationOk
        );
        fs::write(&name, b"tampered contents").unwrap();

        let untouched = HashBuffer::from_bytes(&[1, 2, 3]);
        let mut out = untouched.clone();
        assert_eq!(
            validate_with_hash_buffer(&name, &sidecar(&name), &mut out),
            Status::ValidationFail
        );
        assert_eq!(out, untouched);
    }

    #[test]
    fn validate_reads_the_hash_file_before_the_data_file() {
        let dir = TempDir::new("order");
        let missing = dir.file("missing.bin");
        // Both missing: the hash file is checked first.
        assert_eq!(
            validate(&missing, &sidecar(&missing)),
            Status::ValidationFileDoesntExist
        );
        // Hash file present, data file missing: now it is the file status.
        let hash_name = dir.write("only.CRC32", &[0, 0, 0, 0]);
        let untouched = HashBuffer::from_bytes(&[9]);
        let mut out = untouched.clone();
        assert_eq!(
            validate_with_hash_buffer(&missing, &hash_name, &mut out),
            Status::FileDoesntExist
        );
        assert_eq!(out, untouched);
    }

    #[test]
    fn short_hash_file_is_a_validation_file_bad_size() {
        let dir = TempDir::new("short");
        let name = dir.write("data.bin", b"123456789");
        fs::write(sidecar(&name), [0xCB, 0xF4, 0x39]).unwrap();
        assert_eq!(
            validate(&name, &sidecar(&name)),
            Status::ValidationFileBadSize
        );
        fs::write(sidecar(&name), []).unwrap();
        assert_eq!(
            validate(&name, &sidecar(&name)),
            Status::ValidationFileBadSize
        );
    }

    #[test]
    fn trailing_bytes_in_the_hash_file_are_ignored() {
        let dir = TempDir::new("trailing");
        let name = dir.write("data.bin", b"123456789");
        fs::write(sidecar(&name), [0xCB, 0xF4, 0x39, 0x26, 0xEE, 0xEE]).unwrap();
        assert_eq!(validate(&name, &sidecar(&name)), Status::ValidationOk);
    }

    #[test]
    fn writing_the_hash_file_does_not_truncate() {
        // C++ parity: writeHash opens OPEN_WRITE (O_CREAT, no O_TRUNC).
        let dir = TempDir::new("notrunc");
        let name = dir.write("data.bin", b"123456789");
        fs::write(sidecar(&name), [0u8; 8]).unwrap();
        assert_eq!(
            create_validation(&name, &sidecar(&name)),
            Status::ValidationOk
        );
        assert_eq!(
            fs::read(sidecar(&name)).unwrap(),
            [0xCB, 0xF4, 0x39, 0x26, 0, 0, 0, 0]
        );
        assert_eq!(validate(&name, &sidecar(&name)), Status::ValidationOk);
    }

    #[test]
    fn create_validation_reports_hash_file_errors_but_still_returns_the_hash() {
        let dir = TempDir::new("hasherr");
        let name = dir.write("data.bin", b"123456789");
        let bad_hash_name = dir.file("no/such/dir/data.bin.CRC32");
        let mut digest = HashBuffer::new();
        assert_eq!(
            create_validation_with_hash_buffer(&name, &bad_hash_name, &mut digest),
            Status::ValidationFileDoesntExist
        );
        // C++ parity: computeHash already wrote the out-parameter.
        assert_eq!(digest.as_big_endian_u32(), 0xCBF4_3926);

        // A missing data file leaves the out-parameter alone.
        let untouched = HashBuffer::from_bytes(&[7, 7]);
        let mut out = untouched.clone();
        assert_eq!(
            create_validation_with_hash_buffer(&dir.file("missing"), &sidecar(&name), &mut out),
            Status::FileDoesntExist
        );
        assert_eq!(out, untouched);
    }

    #[test]
    fn create_validation_from_hash_buffer_writes_the_buffer_verbatim() {
        let dir = TempDir::new("precomputed");
        let name = dir.write("data.bin", b"123456789");
        let digest = Hash::hash(b"123456789");
        assert_eq!(
            create_validation_from_hash_buffer(&sidecar(&name), &digest),
            Status::ValidationOk
        );
        assert_eq!(fs::read(sidecar(&name)).unwrap(), [0xCB, 0xF4, 0x39, 0x26]);
        assert_eq!(validate(&name, &sidecar(&name)), Status::ValidationOk);

        // Only the valid bytes are written.
        let other = dir.file("partial.CRC32");
        assert_eq!(
            create_validation_from_hash_buffer(&other, &HashBuffer::from_bytes(&[0xAB, 0xCD])),
            Status::ValidationOk
        );
        assert_eq!(fs::read(&other).unwrap(), [0xAB, 0xCD]);

        assert_eq!(
            create_validation_from_hash_buffer(&dir.file("nodir/x.CRC32"), &digest),
            Status::ValidationFileDoesntExist
        );
    }

    #[test]
    fn a_directory_as_the_data_file_is_reported_not_asserted() {
        let dir = TempDir::new("isdir");
        let data_dir = dir.path.to_str().unwrap().to_string();
        let hash_name = dir.file("dir.CRC32");
        // Opening a directory read-only succeeds; the read fails with
        // EISDIR, which the File layer maps to OTHER_ERROR.
        assert_eq!(create_validation(&data_dir, &hash_name), Status::OtherError);
        assert!(!std::path::Path::new(&hash_name).exists());
    }

    #[test]
    fn translate_status_table_matches_cpp() {
        use file::Status as F;
        let cases = [
            (F::OpOk, Status::ValidationOk, Status::ValidationOk),
            (
                F::DoesntExist,
                Status::FileDoesntExist,
                Status::ValidationFileDoesntExist,
            ),
            (F::NoSpace, Status::NoSpace, Status::NoSpace),
            (
                F::NoPermission,
                Status::FileNoPermission,
                Status::ValidationFileNoPermission,
            ),
            (
                F::BadSize,
                Status::FileBadSize,
                Status::ValidationFileBadSize,
            ),
            (F::NotOpened, Status::OtherError, Status::OtherError),
            (F::OtherError, Status::OtherError, Status::OtherError),
            (F::FileExists, Status::OtherError, Status::OtherError),
            (F::NotSupported, Status::OtherError, Status::OtherError),
            (F::InvalidMode, Status::OtherError, Status::OtherError),
            (F::InvalidArgument, Status::OtherError, Status::OtherError),
            (F::NoMoreResources, Status::OtherError, Status::OtherError),
            (F::OutsideSandbox, Status::OtherError, Status::OtherError),
        ];
        for (input, as_file, as_hash_file) in cases {
            assert_eq!(translate_status(input, StatusFileType::File), as_file);
            assert_eq!(
                translate_status(input, StatusFileType::HashFile),
                as_hash_file
            );
        }
    }

    /// Port of `CRCCheckerTest.ChecksumFileMatchesFrameworkFormat`
    /// (`Utils/test/ut/CRCCheckerTester.cpp`): the two sidecar writers agree.
    #[test]
    fn sidecars_interoperate_with_crc_checker() {
        let dir = TempDir::new("interop");
        let name = dir.write("crc_checker_test_file.bin", b"123456789");

        // crc_checker writes, ValidateFile reads.
        assert_eq!(
            crc_checker::create_checksum_file(&name),
            CrcStat::PassedFileCrcWrite
        );
        assert_eq!(
            fs::read(sidecar(&name)).unwrap(),
            Hash::hash(b"123456789").as_slice()
        );
        assert_eq!(validate(&name, &sidecar(&name)), Status::ValidationOk);

        // ValidateFile writes, crc_checker reads.
        fs::remove_file(sidecar(&name)).unwrap();
        assert_eq!(
            create_validation(&name, &sidecar(&name)),
            Status::ValidationOk
        );
        let mut from_file = 0u32;
        assert_eq!(
            crc_checker::read_crc32_from_file(&name, &mut from_file),
            CrcStat::PassedFileCrcCheck
        );
        assert_eq!(from_file, 0xCBF4_3926);
        let (mut expected, mut actual) = (0u32, 0u32);
        assert_eq!(
            crc_checker::verify_checksum(&name, &mut expected, &mut actual),
            CrcStat::PassedFileCrcCheck
        );
    }

    #[test]
    fn validated_file_names_its_hash_file_with_the_extension() {
        let vf = ValidatedFile::new("buf/log.buf");
        assert_eq!(vf.get_file_name().as_bytes(), b"buf/log.buf");
        assert_eq!(vf.get_hash_file_name().as_bytes(), b"buf/log.buf.CRC32");
        assert_eq!(vf.get_hash_buffer(), &HashBuffer::new());
    }

    #[test]
    fn validated_file_creates_then_validates_and_keeps_the_hash() {
        let dir = TempDir::new("validated");
        let name = dir.write("data.bin", b"123456789");
        let mut vf = ValidatedFile::new(&name);

        // Missing hash file: the hash file is read first.
        assert_eq!(vf.validate(), Status::ValidationFileDoesntExist);
        assert_eq!(vf.get_hash_buffer(), &HashBuffer::new());

        assert_eq!(vf.create_hash_file(), Status::ValidationOk);
        assert_eq!(vf.get_hash_buffer().as_big_endian_u32(), 0xCBF4_3926);
        assert_eq!(fs::read(sidecar(&name)).unwrap(), [0xCB, 0xF4, 0x39, 0x26]);

        // A fresh object loads the hash through validate().
        let mut other = ValidatedFile::new(&name);
        assert_eq!(other.validate(), Status::ValidationOk);
        assert_eq!(other.get_hash_buffer().as_big_endian_u32(), 0xCBF4_3926);

        // A mismatch leaves the previously loaded hash in place.
        fs::write(&name, b"tampered").unwrap();
        assert_eq!(other.validate(), Status::ValidationFail);
        assert_eq!(other.get_hash_buffer().as_big_endian_u32(), 0xCBF4_3926);
    }

    #[test]
    fn validated_file_writes_a_precomputed_hash_and_keeps_it_on_failure() {
        let dir = TempDir::new("validated_pre");
        let name = dir.write("data.bin", b"123456789");
        let mut vf = ValidatedFile::new(&name);
        let digest = Hash::hash(b"123456789");
        assert_eq!(vf.create_hash_file_from(&digest), Status::ValidationOk);
        assert_eq!(vf.get_hash_buffer(), &digest);
        assert_eq!(vf.validate(), Status::ValidationOk);

        // The hash is stored before the write, so a failed write keeps it.
        let mut missing = ValidatedFile::new(&dir.file("no/such/dir/data.bin"));
        assert_eq!(
            missing.create_hash_file_from(&digest),
            Status::ValidationFileDoesntExist
        );
        assert_eq!(missing.get_hash_buffer(), &digest);
    }

    #[test]
    fn validated_file_accepts_the_longest_name_whose_hash_name_fits() {
        let longest = "f".repeat(FwDefaultString::max_length() - HASH_EXTENSION_STRING.len());
        let vf = ValidatedFile::new(&longest);
        assert_eq!(vf.get_hash_file_name().len(), FwDefaultString::max_length());
    }

    #[test]
    #[should_panic]
    fn validated_file_asserts_when_the_hash_name_does_not_fit() {
        // C++ parity: FW_ASSERT on the addFileExtension format status.
        let too_long = "f".repeat(FwDefaultString::max_length() - HASH_EXTENSION_STRING.len() + 1);
        let _ = ValidatedFile::new(&too_long);
    }
}
