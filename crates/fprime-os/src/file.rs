//! The F Prime file facility.
//!
//! Port of `Os::File` (Os/File.{hpp,cpp}) and `Os::Posix::File`
//! (Os/Posix/File.cpp) over `std::fs` — see `docs/cpp-analysis/os.md`.
//!
//! Mode gating (C++ parity):
//! - open on an already-open file returns [`Status::InvalidMode`] (not an
//!   assert);
//! - [`Mode::OpenCreate`] defaults to NO_OVERWRITE, which maps to O_EXCL:
//!   creating over an existing file returns [`Status::FileExists`];
//! - read requires [`Mode::OpenRead`]; write/flush require an open file in
//!   any non-read mode;
//! - size/position/seek require an open file ([`Status::NotOpened`]).
//!
//! Read honors [`WaitType::NoWait`] (returns after the first successful
//! read); write ALWAYS loops to completion regardless of wait — WAIT only
//! adds an fsync (C++ parity). Both loops are bounded to `2 * size`
//! iterations.
//!
//! The CRC functions reproduce the historical F Prime file CRC:
//! [`File::finalize_crc`] returns the UN-complemented CRC32 register
//! (`!standard_crc32`) — see the gotcha list in the analysis doc.

use fprime_config::{FwSignedSizeType, FwSizeType};
use fprime_fw::fw_assert;
use std::io::{Read, Seek, SeekFrom, Write};

/// C++ `FW_FILE_CHUNK_SIZE` (PlatformCfg.fpp): the chunk size for CRC and
/// readline scans.
pub const FW_FILE_CHUNK_SIZE: usize = 512;

/// C++ `Os::File::INITIAL_CRC` (Os/File.hpp): the CRC32 register seed.
pub const INITIAL_CRC: u32 = 0xFFFF_FFFF;

/// Port of `Os::FileInterface::Mode` (Os/File.hpp) — exact C++
/// discriminants.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// File not open.
    #[default]
    OpenNoMode = 0,
    /// Open for reading (O_RDONLY).
    OpenRead = 1,
    /// Open for writing, truncating; NO_OVERWRITE adds O_EXCL
    /// (O_WRONLY|O_CREAT|O_TRUNC[|O_EXCL]).
    OpenCreate = 2,
    /// Open for writing (O_WRONLY|O_CREAT).
    OpenWrite = 3,
    /// Open for synchronous writing (O_WRONLY|O_CREAT|O_SYNC).
    OpenSyncWrite = 4,
    /// Open for appending (O_WRONLY|O_CREAT|O_APPEND).
    OpenAppend = 5,
}

/// Port of `Os::FileInterface::Status` (Os/File.hpp) — exact C++
/// discriminants.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Operation was successful.
    OpOk = 0,
    /// File doesn't exist (for read).
    DoesntExist = 1,
    /// No space left on device.
    NoSpace = 2,
    /// No permission to read/write file.
    NoPermission = 3,
    /// Invalid size parameter.
    BadSize = 4,
    /// File hasn't been opened yet.
    NotOpened = 5,
    /// File already exists.
    FileExists = 6,
    /// Kernel or file system does not support operation.
    NotSupported = 7,
    /// Mode for file operation is invalid.
    InvalidMode = 8,
    /// Invalid argument passed in.
    InvalidArgument = 9,
    /// Too many files or handles open.
    NoMoreResources = 10,
    /// Other error not captured above.
    OtherError = 11,
    /// Path outside of sandbox (SandboxedFile only).
    OutsideSandbox = 12,
}

/// Port of `Os::FileInterface::OverwriteType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverwriteType {
    /// Do not overwrite an existing file (OPEN_CREATE default: O_EXCL).
    NoOverwrite = 0,
    /// Overwrite an existing file.
    Overwrite = 1,
}

/// Port of `Os::FileInterface::SeekType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeekType {
    /// Seek relative to the current position.
    Relative = 0,
    /// Seek from the start of the file (offset must be >= 0: FW_ASSERT).
    Absolute = 1,
}

/// Port of `Os::FileInterface::WaitType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitType {
    /// Read returns after the first successful read (single read-call
    /// semantics); write is unaffected (always to completion, no fsync).
    NoWait = 0,
    /// Read fills the buffer or hits EOF; write additionally fsyncs.
    Wait = 1,
}

// --------------------------------------------------------------------------
// CRC32 (IEEE 802.3 reflected, table-driven). Private duplicate of the
// Utils::Hash CRC32 — fprime-os may not depend on fprime-utils (the DAG runs
// the other way), so the small table lives here. A later refactor can unify.
// --------------------------------------------------------------------------

const fn make_crc32_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc = i as u32;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

const CRC32_TABLE: [u32; 256] = make_crc32_table();

fn crc32_update(register: u32, data: &[u8]) -> u32 {
    let mut crc = register;
    for &byte in data {
        crc = (crc >> 8) ^ CRC32_TABLE[((crc ^ u32::from(byte)) & 0xFF) as usize];
    }
    crc
}

/// Map a std I/O error to the C++ `errno_to_file_status` table
/// (Os/Posix/error.cpp).
fn map_io_error(error: &std::io::Error) -> Status {
    use std::io::ErrorKind;
    match error.kind() {
        // ENOSPC | EFBIG => NO_SPACE
        ErrorKind::StorageFull | ErrorKind::FileTooLarge => Status::NoSpace,
        // ENOENT => DOESNT_EXIST
        ErrorKind::NotFound => Status::DoesntExist,
        // EPERM | EACCES => NO_PERMISSION
        ErrorKind::PermissionDenied => Status::NoPermission,
        // EEXIST => FILE_EXISTS
        ErrorKind::AlreadyExists => Status::FileExists,
        // ENOSYS | EOPNOTSUPP => NOT_SUPPORTED
        ErrorKind::Unsupported => Status::NotSupported,
        // EINVAL => INVALID_ARGUMENT
        ErrorKind::InvalidInput => Status::InvalidArgument,
        // Unlike errno_to_filesystem_status, the C++ file table has NO
        // EROFS or EDQUOT cases: ReadOnlyFilesystem and QuotaExceeded
        // deliberately fall through to OTHER_ERROR here.
        _ => Status::OtherError,
    }
}

/// The F Prime file (see module docs). Closes on drop (C++ destructor
/// parity).
pub struct File {
    fd: Option<std::fs::File>,
    mode: Mode,
    /// Running CRC32 register (C++ `m_hash`), re-initialized on open and
    /// after finalize.
    crc: u32,
    /// CRC chunk staging buffer (C++ `m_crc_buffer`).
    crc_buffer: [u8; FW_FILE_CHUNK_SIZE],
}

impl Default for File {
    fn default() -> Self {
        Self::new()
    }
}

impl File {
    /// Construct a closed file.
    pub fn new() -> Self {
        Self {
            fd: None,
            mode: Mode::OpenNoMode,
            crc: INITIAL_CRC,
            crc_buffer: [0u8; FW_FILE_CHUNK_SIZE],
        }
    }

    /// Open with the default NO_OVERWRITE (port of the two-argument C++
    /// `open` overload).
    pub fn open(&mut self, filepath: &str, requested_mode: Mode) -> Status {
        self.open_with_overwrite(filepath, requested_mode, OverwriteType::NoOverwrite)
    }

    /// Open the file (port of `Os::File::open`). Opening an already-open
    /// file returns [`Status::InvalidMode`]; success stores the mode and
    /// re-initializes the CRC register.
    pub fn open_with_overwrite(
        &mut self,
        filepath: &str,
        requested_mode: Mode,
        overwrite: OverwriteType,
    ) -> Status {
        // C++ parity: FW_ASSERT(OPEN_NO_MODE < requested_mode < MAX_OPEN_MODE).
        fw_assert!(requested_mode != Mode::OpenNoMode);
        if self.is_open() {
            return Status::InvalidMode;
        }
        let mut options = std::fs::OpenOptions::new();
        match requested_mode {
            Mode::OpenNoMode => return Status::InvalidMode, // unreachable: asserted above
            // READ => O_RDONLY
            Mode::OpenRead => {
                options.read(true);
            }
            // WRITE / SYNC_WRITE => O_WRONLY | O_CREAT (no truncate).
            // O_SYNC is approximated: every write in SYNC_WRITE mode is
            // followed by sync_all (see write()).
            Mode::OpenWrite | Mode::OpenSyncWrite => {
                options.write(true).create(true);
            }
            // CREATE => O_WRONLY | O_CREAT | O_TRUNC | (NO_OVERWRITE ? O_EXCL : 0)
            Mode::OpenCreate => {
                options.write(true);
                if overwrite == OverwriteType::NoOverwrite {
                    options.create_new(true);
                } else {
                    options.create(true).truncate(true);
                }
            }
            // APPEND => O_WRONLY | O_CREAT | O_APPEND
            Mode::OpenAppend => {
                options.append(true).create(true);
            }
        }
        match options.open(filepath) {
            Ok(fd) => {
                self.fd = Some(fd);
                self.mode = requested_mode;
                // C++ parity: reset any open CRC calculation.
                self.crc = INITIAL_CRC;
                Status::OpOk
            }
            Err(error) => map_io_error(&error),
        }
    }

    /// Close the file (idempotent; errors ignored, C++ parity).
    pub fn close(&mut self) {
        self.fd = None;
        self.mode = Mode::OpenNoMode;
    }

    /// Whether the file is open (`mode != OPEN_NO_MODE`).
    pub fn is_open(&self) -> bool {
        self.mode != Mode::OpenNoMode
    }

    /// Current open mode.
    pub fn get_mode(&self) -> Mode {
        self.mode
    }

    /// Get the file size without disturbing the file position (port of
    /// `size`). [`Status::NotOpened`] when closed.
    pub fn size(&mut self, size_result: &mut FwSizeType) -> Status {
        let fd = match &mut self.fd {
            None => return Status::NotOpened,
            Some(fd) => fd,
        };
        match fd.metadata() {
            Ok(metadata) => {
                *size_result = metadata.len();
                Status::OpOk
            }
            Err(error) => map_io_error(&error),
        }
    }

    /// Get the current file position (port of `position`).
    pub fn position(&mut self, position_result: &mut FwSizeType) -> Status {
        let fd = match &mut self.fd {
            None => return Status::NotOpened,
            Some(fd) => fd,
        };
        match fd.stream_position() {
            Ok(position) => {
                *position_result = position;
                Status::OpOk
            }
            Err(error) => map_io_error(&error),
        }
    }

    /// Seek (port of `seek`). C++ parity: FW_ASSERTs `offset >= 0` for
    /// [`SeekType::Absolute`].
    pub fn seek(&mut self, offset: FwSignedSizeType, seek_type: SeekType) -> Status {
        fw_assert!(seek_type == SeekType::Relative || offset >= 0);
        let fd = match &mut self.fd {
            None => return Status::NotOpened,
            Some(fd) => fd,
        };
        let seek_from = match seek_type {
            SeekType::Absolute => SeekFrom::Start(offset as u64),
            SeekType::Relative => SeekFrom::Current(offset),
        };
        match fd.seek(seek_from) {
            Ok(_) => Status::OpOk,
            Err(error) => map_io_error(&error),
        }
    }

    /// Absolute seek over the full `FwSizeType` range (port of
    /// `seek_absolute`). The C++ 3-seek trick for offsets above
    /// `FwSignedSizeType::MAX` is unnecessary here — `SeekFrom::Start` takes
    /// a u64 natively — but the API shape is kept.
    pub fn seek_absolute(&mut self, offset: FwSizeType) -> Status {
        let fd = match &mut self.fd {
            None => return Status::NotOpened,
            Some(fd) => fd,
        };
        match fd.seek(SeekFrom::Start(offset)) {
            Ok(_) => Status::OpOk,
            Err(error) => map_io_error(&error),
        }
    }

    /// Flush written data to disk (port of `flush`). Requires an open,
    /// non-read-mode file.
    pub fn flush(&mut self) -> Status {
        if self.mode == Mode::OpenNoMode {
            return Status::NotOpened;
        }
        if self.mode == Mode::OpenRead {
            return Status::InvalidMode;
        }
        let fd = match &mut self.fd {
            None => return Status::NotOpened,
            Some(fd) => fd,
        };
        match fd.sync_all() {
            Ok(()) => Status::OpOk,
            Err(error) => map_io_error(&error),
        }
    }

    /// Read up to `buffer.len()` bytes (port of `read`). `size` is an
    /// output: the number of bytes actually read.
    ///
    /// [`WaitType::NoWait`] returns after the first successful read call;
    /// [`WaitType::Wait`] keeps reading until the buffer is full or EOF.
    /// The loop is bounded to `2 * buffer.len()` iterations (C++ parity).
    /// Requires mode == [`Mode::OpenRead`] else [`Status::InvalidMode`].
    pub fn read(&mut self, buffer: &mut [u8], size: &mut FwSizeType, wait: WaitType) -> Status {
        if self.mode == Mode::OpenNoMode {
            *size = 0;
            return Status::NotOpened;
        }
        if self.mode != Mode::OpenRead {
            *size = 0;
            return Status::InvalidMode;
        }
        let fd = match &mut self.fd {
            None => {
                *size = 0;
                return Status::NotOpened;
            }
            Some(fd) => fd,
        };
        let request = buffer.len();
        let bound = request.saturating_mul(2);
        let mut accumulated = 0usize;
        let mut iteration = 0usize;
        while accumulated < request && iteration < bound {
            iteration += 1;
            match fd.read(&mut buffer[accumulated..]) {
                // EOF: break out now.
                Ok(0) => break,
                Ok(read_size) => {
                    accumulated += read_size;
                    // NO_WAIT: break after the first successful read.
                    if wait == WaitType::NoWait {
                        break;
                    }
                }
                // EINTR retries (counts against the bound, C++ parity).
                Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(error) => {
                    *size = accumulated as FwSizeType;
                    return map_io_error(&error);
                }
            }
        }
        *size = accumulated as FwSizeType;
        Status::OpOk
    }

    /// Write `buffer` (port of `write`). `size` is an output: bytes actually
    /// written (equals `buffer.len()` on success).
    ///
    /// C++ parity: the write loops to completion REGARDLESS of `wait`
    /// (bounded to `2 * buffer.len()` iterations); [`WaitType::Wait`] only
    /// adds an fsync afterwards. [`Mode::OpenSyncWrite`] also fsyncs (the
    /// O_SYNC approximation). Requires an open, non-read-mode file.
    pub fn write(&mut self, buffer: &[u8], size: &mut FwSizeType, wait: WaitType) -> Status {
        if self.mode == Mode::OpenNoMode {
            *size = 0;
            return Status::NotOpened;
        }
        if self.mode == Mode::OpenRead {
            *size = 0;
            return Status::InvalidMode;
        }
        let sync_mode = self.mode == Mode::OpenSyncWrite;
        let fd = match &mut self.fd {
            None => {
                *size = 0;
                return Status::NotOpened;
            }
            Some(fd) => fd,
        };
        let request = buffer.len();
        let bound = request.saturating_mul(2);
        let mut written = 0usize;
        let mut iteration = 0usize;
        while written < request && iteration < bound {
            iteration += 1;
            match fd.write(&buffer[written..]) {
                Ok(0) => break,
                Ok(write_size) => written += write_size,
                Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(error) => {
                    *size = written as FwSizeType;
                    return map_io_error(&error);
                }
            }
        }
        *size = written as FwSizeType;
        if wait == WaitType::Wait || sync_mode {
            if let Err(error) = fd.sync_all() {
                return map_io_error(&error);
            }
        }
        Status::OpOk
    }

    /// Read one newline-terminated line (port of `readline`,
    /// Os/File.cpp). `size` is an output: bytes read INCLUDING the newline.
    ///
    /// Contract (C++ parity):
    /// - a newline within `buffer.len()` bytes: `size` = bytes through the
    ///   newline, position left just after it, [`Status::OpOk`];
    /// - EOF before any newline: [`Status::OpOk`] with what was read;
    /// - no newline found within the buffer, or any error: `size` = 0, the
    ///   file position is seeked back to where it was, and
    ///   [`Status::OtherError`] (or the underlying error) is returned.
    pub fn readline(&mut self, buffer: &mut [u8], size: &mut FwSizeType, wait: WaitType) -> Status {
        if self.mode == Mode::OpenNoMode {
            *size = 0;
            return Status::NotOpened;
        }
        if self.mode != Mode::OpenRead {
            *size = 0;
            return Status::InvalidMode;
        }
        let mut original_location: FwSizeType = 0;
        let status = self.position(&mut original_location);
        if status != Status::OpOk {
            *size = 0;
            return status;
        }
        let requested_size = buffer.len();
        let mut position = 0usize;
        while position < requested_size {
            // Read in chunks to match the C++ scan (chunk size 512).
            let chunk = (requested_size - position).min(FW_FILE_CHUNK_SIZE);
            let mut read_size: FwSizeType = 0;
            let status = self.read(
                &mut buffer[position..position + chunk],
                &mut read_size,
                wait,
            );
            if status != Status::OpOk {
                // Contract: on error, seek back to the original location.
                *size = 0;
                let _ = self.seek_absolute(original_location);
                return status;
            }
            let read_size = read_size as usize;
            // EOF: return what was read so far.
            if read_size == 0 {
                *size = position as FwSizeType;
                return Status::OpOk;
            }
            // Scan the fresh chunk for '\n'.
            if let Some(found) = buffer[position..position + read_size]
                .iter()
                .position(|&byte| byte == b'\n')
            {
                let scan = position + found;
                *size = (scan + 1) as FwSizeType;
                let _ = self.seek_absolute(original_location + (scan as FwSizeType) + 1);
                return Status::OpOk;
            }
            position += read_size;
        }
        // Failed to find a newline within the available buffer.
        // Contract: seek back to the original location.
        *size = 0;
        let _ = self.seek_absolute(original_location);
        Status::OtherError
    }

    /// Feed one chunk of the file into the running CRC (port of
    /// `incrementalCrc`). `size` is in/out: in = requested chunk size
    /// (FW_ASSERT <= [`FW_FILE_CHUNK_SIZE`]), out = bytes actually read
    /// (a NO_WAIT read, C++ parity). Requires [`Mode::OpenRead`].
    pub fn incremental_crc(&mut self, size: &mut FwSizeType) -> Status {
        fw_assert!(*size <= FW_FILE_CHUNK_SIZE as FwSizeType);
        if self.mode == Mode::OpenNoMode {
            return Status::NotOpened;
        }
        if self.mode != Mode::OpenRead {
            return Status::InvalidMode;
        }
        let request = *size as usize;
        let mut staging = self.crc_buffer;
        let mut read_size: FwSizeType = 0;
        let status = self.read(&mut staging[..request], &mut read_size, WaitType::NoWait);
        if status == Status::OpOk {
            self.crc_buffer = staging;
            self.crc = crc32_update(self.crc, &staging[..read_size as usize]);
            *size = read_size;
        }
        status
    }

    /// Finalize the running CRC (port of `finalizeCrc`).
    ///
    /// C++ parity gotcha: the historical F Prime file CRC OMITS the
    /// standard final 1's complement — this returns the raw CRC32 register
    /// (`!standard_crc32`). The register is re-initialized afterwards.
    pub fn finalize_crc(&mut self, crc: &mut u32) -> Status {
        *crc = self.crc;
        self.crc = INITIAL_CRC;
        Status::OpOk
    }

    /// CRC the whole file from the current position (port of
    /// `calculateCrc`): chunks of [`FW_FILE_CHUNK_SIZE`] until a short read,
    /// then finalize. `crc` is 0 on error (C++ parity).
    pub fn calculate_crc(&mut self, crc: &mut u32) -> Status {
        *crc = 0;
        let mut status;
        loop {
            let mut size = FW_FILE_CHUNK_SIZE as FwSizeType;
            status = self.incremental_crc(&mut size);
            // Break on EOF (short read) or error.
            if size != FW_FILE_CHUNK_SIZE as FwSizeType || status != Status::OpOk {
                break;
            }
        }
        if status == Status::OpOk {
            status = self.finalize_crc(crc);
        }
        status
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::temp_dir;

    fn write_file(path: &std::path::Path, contents: &[u8]) {
        std::fs::write(path, contents).expect("test file write");
    }

    #[test]
    fn errno_table_has_no_erofs_or_edquot_cases() {
        use std::io::{Error, ErrorKind};
        // The C++ errno_to_file_status table (unlike the filesystem table)
        // has no EROFS or EDQUOT cases — both hit default OTHER_ERROR.
        assert_eq!(
            map_io_error(&Error::from(ErrorKind::ReadOnlyFilesystem)),
            Status::OtherError
        );
        assert_eq!(
            map_io_error(&Error::from(ErrorKind::QuotaExceeded)),
            Status::OtherError
        );
        // EPERM/EACCES and ENOSPC/EFBIG remain mapped.
        assert_eq!(
            map_io_error(&Error::from(ErrorKind::PermissionDenied)),
            Status::NoPermission
        );
        assert_eq!(
            map_io_error(&Error::from(ErrorKind::StorageFull)),
            Status::NoSpace
        );
    }

    #[test]
    fn open_read_on_missing_file_is_doesnt_exist() {
        let dir = temp_dir("file_missing");
        let mut file = File::new();
        assert_eq!(
            file.open(dir.join("nope").to_str().unwrap(), Mode::OpenRead),
            Status::DoesntExist
        );
        assert!(!file.is_open());
    }

    // Gotcha: open on an already-open file is INVALID_MODE, not an assert.
    #[test]
    fn open_while_open_is_invalid_mode() {
        let dir = temp_dir("file_double_open");
        let path = dir.join("f.bin");
        let path_str = path.to_str().unwrap();
        let mut file = File::new();
        assert_eq!(file.open(path_str, Mode::OpenWrite), Status::OpOk);
        assert_eq!(file.open(path_str, Mode::OpenRead), Status::InvalidMode);
        file.close();
        assert_eq!(file.open(path_str, Mode::OpenRead), Status::OpOk);
    }

    // Gotcha: OPEN_CREATE + default NO_OVERWRITE maps to O_EXCL.
    #[test]
    fn create_no_overwrite_on_existing_is_file_exists() {
        let dir = temp_dir("file_excl");
        let path = dir.join("f.bin");
        let path_str = path.to_str().unwrap();
        write_file(&path, b"already here");
        let mut file = File::new();
        assert_eq!(file.open(path_str, Mode::OpenCreate), Status::FileExists);
        // Explicit OVERWRITE truncates and succeeds.
        assert_eq!(
            file.open_with_overwrite(path_str, Mode::OpenCreate, OverwriteType::Overwrite),
            Status::OpOk
        );
        file.close();
        assert_eq!(std::fs::read(&path).unwrap(), b"");
    }

    #[test]
    fn read_requires_read_mode_and_write_rejects_read_mode() {
        let dir = temp_dir("file_gating");
        let path = dir.join("f.bin");
        let path_str = path.to_str().unwrap();
        write_file(&path, b"data");

        let mut file = File::new();
        let mut buffer = [0u8; 4];
        let mut size: FwSizeType = 99;
        // Closed file: NOT_OPENED with size zeroed.
        assert_eq!(
            file.read(&mut buffer, &mut size, WaitType::Wait),
            Status::NotOpened
        );
        assert_eq!(size, 0);

        assert_eq!(file.open(path_str, Mode::OpenWrite), Status::OpOk);
        size = 99;
        assert_eq!(
            file.read(&mut buffer, &mut size, WaitType::Wait),
            Status::InvalidMode
        );
        assert_eq!(size, 0);
        file.close();

        assert_eq!(file.open(path_str, Mode::OpenRead), Status::OpOk);
        size = 99;
        assert_eq!(
            file.write(b"nope", &mut size, WaitType::Wait),
            Status::InvalidMode
        );
        assert_eq!(size, 0);
        assert_eq!(file.flush(), Status::InvalidMode);
    }

    #[test]
    fn write_then_read_round_trip_with_append() {
        let dir = temp_dir("file_round_trip");
        let path = dir.join("f.bin");
        let path_str = path.to_str().unwrap();

        let mut file = File::new();
        assert_eq!(file.open(path_str, Mode::OpenWrite), Status::OpOk);
        let mut size: FwSizeType = 0;
        assert_eq!(
            file.write(b"hello ", &mut size, WaitType::Wait),
            Status::OpOk
        );
        assert_eq!(size, 6);
        file.close();

        assert_eq!(file.open(path_str, Mode::OpenAppend), Status::OpOk);
        assert_eq!(
            file.write(b"world", &mut size, WaitType::NoWait),
            Status::OpOk
        );
        assert_eq!(size, 5);
        file.close();

        assert_eq!(file.open(path_str, Mode::OpenRead), Status::OpOk);
        let mut file_size: FwSizeType = 0;
        assert_eq!(file.size(&mut file_size), Status::OpOk);
        assert_eq!(file_size, 11);
        let mut buffer = [0u8; 16];
        assert_eq!(
            file.read(&mut buffer, &mut size, WaitType::Wait),
            Status::OpOk
        );
        assert_eq!(size, 11);
        assert_eq!(&buffer[..11], b"hello world");
        // At EOF a further read returns 0 bytes, OP_OK.
        assert_eq!(
            file.read(&mut buffer, &mut size, WaitType::Wait),
            Status::OpOk
        );
        assert_eq!(size, 0);
    }

    // OPEN_WRITE does not truncate (O_WRONLY|O_CREAT, no O_TRUNC).
    #[test]
    fn open_write_does_not_truncate() {
        let dir = temp_dir("file_write_no_trunc");
        let path = dir.join("f.bin");
        let path_str = path.to_str().unwrap();
        write_file(&path, b"0123456789");
        let mut file = File::new();
        assert_eq!(file.open(path_str, Mode::OpenWrite), Status::OpOk);
        let mut size: FwSizeType = 0;
        assert_eq!(file.write(b"AB", &mut size, WaitType::Wait), Status::OpOk);
        file.close();
        assert_eq!(std::fs::read(&path).unwrap(), b"AB23456789");
    }

    #[test]
    fn seek_and_position() {
        let dir = temp_dir("file_seek");
        let path = dir.join("f.bin");
        let path_str = path.to_str().unwrap();
        write_file(&path, b"abcdefgh");
        let mut file = File::new();

        // Gates: seek/size/position on a closed file are NOT_OPENED.
        assert_eq!(file.seek(0, SeekType::Absolute), Status::NotOpened);
        let mut out: FwSizeType = 0;
        assert_eq!(file.size(&mut out), Status::NotOpened);
        assert_eq!(file.position(&mut out), Status::NotOpened);

        assert_eq!(file.open(path_str, Mode::OpenRead), Status::OpOk);
        assert_eq!(file.seek(4, SeekType::Absolute), Status::OpOk);
        assert_eq!(file.position(&mut out), Status::OpOk);
        assert_eq!(out, 4);
        assert_eq!(file.seek(-2, SeekType::Relative), Status::OpOk);
        assert_eq!(file.position(&mut out), Status::OpOk);
        assert_eq!(out, 2);
        assert_eq!(file.seek_absolute(7), Status::OpOk);
        let mut buffer = [0u8; 1];
        let mut size: FwSizeType = 0;
        assert_eq!(
            file.read(&mut buffer, &mut size, WaitType::Wait),
            Status::OpOk
        );
        assert_eq!(buffer[0], b'h');
    }

    #[test]
    #[should_panic]
    fn absolute_seek_with_negative_offset_asserts() {
        let mut file = File::new();
        let _ = file.seek(-1, SeekType::Absolute);
    }

    #[test]
    fn readline_contract() {
        let dir = temp_dir("file_readline");
        let path = dir.join("f.txt");
        let path_str = path.to_str().unwrap();
        write_file(&path, b"line1\nsecond line\ntail");
        let mut file = File::new();
        assert_eq!(file.open(path_str, Mode::OpenRead), Status::OpOk);

        let mut buffer = [0u8; 64];
        let mut size: FwSizeType = 0;
        // First line: size includes the newline; position lands after it.
        assert_eq!(
            file.readline(&mut buffer, &mut size, WaitType::Wait),
            Status::OpOk
        );
        assert_eq!(size, 6);
        assert_eq!(&buffer[..6], b"line1\n");
        let mut position: FwSizeType = 0;
        assert_eq!(file.position(&mut position), Status::OpOk);
        assert_eq!(position, 6);

        // Second line.
        assert_eq!(
            file.readline(&mut buffer, &mut size, WaitType::Wait),
            Status::OpOk
        );
        assert_eq!(&buffer[..size as usize], b"second line\n");

        // EOF without newline: OP_OK with what was read.
        assert_eq!(
            file.readline(&mut buffer, &mut size, WaitType::Wait),
            Status::OpOk
        );
        assert_eq!(size, 4);
        assert_eq!(&buffer[..4], b"tail");
    }

    // Gotcha: newline not found within the caller's buffer => size 0,
    // position seeked BACK to the original location, OTHER_ERROR.
    #[test]
    fn readline_seeks_back_when_no_newline_fits() {
        let dir = temp_dir("file_readline_back");
        let path = dir.join("f.txt");
        let path_str = path.to_str().unwrap();
        write_file(&path, b"a very long line without break\nrest");
        let mut file = File::new();
        assert_eq!(file.open(path_str, Mode::OpenRead), Status::OpOk);
        assert_eq!(file.seek(2, SeekType::Absolute), Status::OpOk);

        let mut small = [0u8; 8];
        let mut size: FwSizeType = 99;
        assert_eq!(
            file.readline(&mut small, &mut size, WaitType::Wait),
            Status::OtherError
        );
        assert_eq!(size, 0);
        let mut position: FwSizeType = 0;
        assert_eq!(file.position(&mut position), Status::OpOk);
        assert_eq!(position, 2);
    }

    // Gotcha: finalizeCrc returns the UN-complemented register:
    // !crc32(b"123456789") where standard crc32 = 0xCBF43926.
    #[test]
    fn calculate_crc_matches_historical_quirk_vector() {
        let dir = temp_dir("file_crc");
        let path = dir.join("f.bin");
        let path_str = path.to_str().unwrap();
        write_file(&path, b"123456789");
        let mut file = File::new();
        assert_eq!(file.open(path_str, Mode::OpenRead), Status::OpOk);
        let mut crc: u32 = 0;
        assert_eq!(file.calculate_crc(&mut crc), Status::OpOk);
        assert_eq!(crc, !0xCBF4_3926u32);
        assert_eq!(crc, 0x340B_C6D9);
    }

    // CRC over data larger than one 512-byte chunk exercises the chunk loop.
    #[test]
    fn calculate_crc_multi_chunk() {
        let dir = temp_dir("file_crc_chunks");
        let path = dir.join("f.bin");
        let path_str = path.to_str().unwrap();
        let data: Vec<u8> = (0..1500u32).map(|i| (i % 251) as u8).collect();
        write_file(&path, &data);

        let mut file = File::new();
        assert_eq!(file.open(path_str, Mode::OpenRead), Status::OpOk);
        let mut crc: u32 = 0;
        assert_eq!(file.calculate_crc(&mut crc), Status::OpOk);

        // Reference: single-shot register computation.
        let expected = crc32_update(INITIAL_CRC, &data);
        assert_eq!(crc, expected);
    }

    #[test]
    fn incremental_crc_gating() {
        let dir = temp_dir("file_crc_gate");
        let path = dir.join("f.bin");
        let path_str = path.to_str().unwrap();
        let mut file = File::new();
        let mut size: FwSizeType = 16;
        assert_eq!(file.incremental_crc(&mut size), Status::NotOpened);
        assert_eq!(file.open(path_str, Mode::OpenWrite), Status::OpOk);
        size = 16;
        assert_eq!(file.incremental_crc(&mut size), Status::InvalidMode);
    }
}
