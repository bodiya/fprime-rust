//! # LogFile — port of `Svc::LogFile`
//!
//! C++ sources: `Svc/ActiveTextLogger/LogFile.{cpp,hpp}`.
//! Analysis: `docs/cpp-analysis/svc-misc.md` (ActiveTextLogger + LogFile).
//!
//! The optional size-capped text file behind
//! [`ActiveTextLogger`](super::ActiveTextLogger). Despite the roadmap's
//! wording there is no rotation: upstream stops writing for good.
//!
//! - [`LogFile::set_log_file`] closes any open file, then probes
//!   `name`, `name0`, `name1`, … `name<max_backups - 1>` with
//!   `Os::FileSystem::getFileSize`: the first probe that does NOT return
//!   `OP_OK` names the file to use. If every probe succeeds (all
//!   `max_backups + 1` names exist), the ORIGINAL name is reused. The chosen
//!   file is opened `OPEN_CREATE` + `OVERWRITE`, so a reused name is
//!   truncated. A name (or suffixed name) longer than `FileNameString`
//!   fails the call; so does a failed open. Every failure leaves no file
//!   open.
//! - [`LogFile::write_to_log`] writes one line with `WAIT` (fsync). If the
//!   line would take the file past its maximum size (or the size
//!   arithmetic would overflow), the file is closed PERMANENTLY and the
//!   line dropped; only another `set_log_file` reopens. A write error
//!   leaves the file open and still adds the bytes actually written.

use fprime_config::FwSizeType;
use fprime_fw::{FileNameString, fw_assert};
use fprime_os::file::{self, File, Mode, OverwriteType, WaitType};
use fprime_os::filesystem;
use std::fmt;

/// Counts the bytes a format would produce, so a name can be checked
/// against `FileNameString` before it is formatted (the C++
/// `Fw::FormatStatus::OVERFLOWED` check) without a heap allocation.
struct LenCounter(usize);

impl fmt::Write for LenCounter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.0 += s.len();
        Ok(())
    }
}

/// C++ `searchFilename.format(...)` with its status: `false` (and `out`
/// untouched) when the result would not fit a `FileNameString`.
fn format_name(out: &mut FileNameString, args: fmt::Arguments<'_>) -> bool {
    let mut counter = LenCounter(0);
    // LenCounter never errors.
    let _ = fmt::Write::write_fmt(&mut counter, args);
    if counter.0 > FileNameString::max_length() {
        return false;
    }
    out.format(args);
    true
}

/// `Svc::LogFile` — a size-capped log file.
///
/// DEVIATION: the C++ type is a `struct` with public members "for ease of
/// use in object composition"; here the fields are private (they must stay
/// consistent with the open [`File`]) and read through accessors.
pub struct LogFile {
    /// C++ `m_fileName`: the name actually opened (possibly suffixed).
    file_name: FileNameString,
    /// C++ `m_file`.
    file: File,
    /// C++ `m_maxFileSize`.
    max_file_size: FwSizeType,
    /// C++ `m_openFile`: a file is open and accepting lines.
    open_file: bool,
    /// C++ `m_currentFileSize`: bytes written to the open file.
    current_file_size: FwSizeType,
}

impl Default for LogFile {
    fn default() -> Self {
        Self::new()
    }
}

impl LogFile {
    /// The C++ default for `maxBackups` (`set_log_file(..., maxBackups = 10)`).
    pub const DEFAULT_MAX_BACKUPS: FwSizeType = 10;

    /// C++ `LogFile()`: no file open.
    pub fn new() -> Self {
        Self {
            file_name: FileNameString::new(),
            file: File::new(),
            max_file_size: 0,
            open_file: false,
            current_file_size: 0,
        }
    }

    /// Whether a file is open and accepting lines (C++ `m_openFile`).
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.open_file
    }

    /// The name of the file last opened (C++ `m_fileName`). Kept after the
    /// file is closed by the size cap; empty until a `set_log_file`
    /// succeeds.
    #[must_use]
    pub fn file_name(&self) -> &FileNameString {
        &self.file_name
    }

    /// Bytes written to the current file (C++ `m_currentFileSize`).
    #[must_use]
    pub fn current_file_size(&self) -> FwSizeType {
        self.current_file_size
    }

    /// The size cap of the current file (C++ `m_maxFileSize`).
    #[must_use]
    pub fn max_file_size(&self) -> FwSizeType {
        self.max_file_size
    }

    /// C++ `write_to_log(buf, size)`: append `buf` if a file is open.
    ///
    /// Returns `true` when nothing had to be written (no open file, or an
    /// empty `buf`) or the whole buffer was written; `false` when the line
    /// would exceed the maximum size (the file is then closed for good) or
    /// the write failed or was short.
    pub fn write_to_log(&mut self, buf: &[u8]) -> bool {
        // The C++ FW_ASSERT(buf != nullptr) is unrepresentable for a slice.
        let size = buf.len() as FwSizeType;
        let mut status = true;

        if self.open_file && size > 0 {
            // C++ parity: `projectedSize > m_maxFileSize` OR the addition
            // overflowing both close the file and drop the line.
            match self.current_file_size.checked_add(size) {
                Some(projected) if projected <= self.max_file_size => {
                    let mut write_size = size;
                    let stat = self.file.write(buf, &mut write_size, WaitType::Wait);
                    // C++ parity: writing to a file never opened is a
                    // programmer error.
                    fw_assert!(stat != file::Status::NotOpened, stat as i32);
                    status = stat == file::Status::OpOk && write_size == size;
                    // C++ parity: counted even on a failed or short write
                    // (`write_size` is what actually reached the file).
                    self.current_file_size += write_size;
                }
                _ => {
                    status = false;
                    self.open_file = false;
                    self.file.close();
                }
            }
        }

        status
    }

    /// C++ `set_log_file(fileName, maxSize, maxBackups)`: open a new log
    /// file capped at `max_size` bytes, probing up to `max_backups`
    /// suffixed names when `file_name` exists (see the module docs).
    /// Returns `true` when a file was opened.
    ///
    /// C++ `%s` stops at a NUL, so `file_name` is used up to its first NUL
    /// byte.
    ///
    /// DEVIATION: the C++ probe loop counts with a `U32` against the
    /// `FwSizeType` bound, so a `max_backups` above `u32::MAX` loops forever
    /// when every name exists; the Rust loop counts in `FwSizeType` and
    /// terminates. ([`ActiveTextLogger::set_log_file`] takes a `u32`, as in
    /// C++, so this is reachable only through `LogFile` directly.)
    ///
    /// [`ActiveTextLogger::set_log_file`]: super::ActiveTextLogger::set_log_file
    pub fn set_log_file(
        &mut self,
        file_name: &str,
        max_size: FwSizeType,
        max_backups: FwSizeType,
    ) -> bool {
        let file_name = file_name.split('\0').next().unwrap_or_default();

        // If there is already a previously open file then close it.
        if self.open_file {
            self.open_file = false;
            self.file.close();
        }

        // A file name too large for FileNameString is a failure.
        let mut search_filename = FileNameString::new();
        if !format_name(&mut search_filename, format_args!("{file_name}")) {
            return false;
        }

        // Probe the original name, then the suffixed names, for one that
        // does not exist: `max_backups + 1` probes in all.
        let mut failed_suffix = true;
        let mut file_size: FwSizeType = 0;
        for suffix in 0..=max_backups {
            // C++ parity: ANY getFileSize status other than OP_OK means the
            // name is free.
            let probe = search_filename.as_str().unwrap_or_default();
            if filesystem::get_file_size(probe, &mut file_size) != filesystem::Status::OpOk {
                failed_suffix = false;
                break;
            }
            if suffix < max_backups
                && !format_name(&mut search_filename, format_args!("{file_name}{suffix}"))
            {
                return false;
            }
        }

        // Every name exists: reuse (and truncate) the original.
        if failed_suffix {
            search_filename.set(file_name);
        }

        // OPEN_CREATE + OVERWRITE truncates an existing file.
        let stat = self.file.open_with_overwrite(
            search_filename.as_str().unwrap_or_default(),
            Mode::OpenCreate,
            OverwriteType::Overwrite,
        );
        if stat != file::Status::OpOk {
            return false;
        }

        self.current_file_size = 0;
        self.max_file_size = max_size;
        self.file_name = search_filename;
        self.open_file = true;
        true
    }
}

// C++ `~LogFile()` closes an open file; dropping the `File` closes it here.

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    /// A per-test scratch directory, removed on drop.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(tag: &str) -> Self {
            let mut path = std::env::temp_dir();
            path.push(format!(
                "fprime_rust_logfile_{tag}_{}_{}",
                std::process::id(),
                COUNTER.fetch_add(1, Ordering::SeqCst)
            ));
            let _ = std::fs::remove_dir_all(&path);
            std::fs::create_dir_all(&path).unwrap();
            Self(path)
        }

        fn file(&self, name: &str) -> String {
            self.0.join(name).to_str().unwrap().to_string()
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn write_without_an_open_file_or_with_no_bytes_succeeds_and_does_nothing() {
        let mut log = LogFile::new();
        assert!(!log.is_open());
        assert!(log.write_to_log(b"dropped\n"));
        assert_eq!(log.current_file_size(), 0);

        let dir = TempDir::new("empty");
        let name = dir.file("log");
        assert!(log.set_log_file(&name, 100, LogFile::DEFAULT_MAX_BACKUPS));
        assert!(log.write_to_log(b""));
        assert_eq!(log.current_file_size(), 0);
        assert!(log.is_open());
    }

    #[test]
    fn a_line_that_fits_exactly_is_written_and_the_next_closes_the_file() {
        let dir = TempDir::new("exact");
        let name = dir.file("log");
        let mut log = LogFile::new();
        assert!(log.set_log_file(&name, 8, LogFile::DEFAULT_MAX_BACKUPS));
        assert!(log.write_to_log(b"1234"));
        assert!(log.write_to_log(b"5678"));
        assert_eq!(log.current_file_size(), 8);
        assert!(log.is_open());
        assert!(!log.write_to_log(b"9"));
        assert!(!log.is_open());
        assert_eq!(log.current_file_size(), 8);
        // Permanently closed: later lines are ignored and report success.
        assert!(log.write_to_log(b"9"));
        assert_eq!(std::fs::read(&name).unwrap(), b"12345678");
        // The name of the closed file is kept.
        assert_eq!(log.file_name().as_str(), Some(name.as_str()));
    }

    #[test]
    fn size_overflow_closes_the_file() {
        let dir = TempDir::new("overflow");
        let mut log = LogFile::new();
        assert!(log.set_log_file(&dir.file("log"), FwSizeType::MAX, 0));
        log.current_file_size = FwSizeType::MAX - 1;
        assert!(!log.write_to_log(b"ab"));
        assert!(!log.is_open());
        assert_eq!(log.current_file_size(), FwSizeType::MAX - 1);
    }

    #[test]
    fn zero_backups_reuses_and_truncates_an_existing_name() {
        let dir = TempDir::new("zero");
        let name = dir.file("log");
        std::fs::write(&name, b"old contents").unwrap();
        let mut log = LogFile::new();
        assert!(log.set_log_file(&name, 100, 0));
        assert_eq!(log.file_name().as_str(), Some(name.as_str()));
        assert_eq!(std::fs::read(&name).unwrap(), b"");
        assert!(!std::path::Path::new(&dir.file("log0")).exists());
    }

    #[test]
    fn suffix_search_takes_the_first_free_name() {
        let dir = TempDir::new("gap");
        let name = dir.file("log");
        std::fs::write(&name, b"").unwrap();
        std::fs::write(dir.file("log0"), b"").unwrap();
        // log1 free, log2 exists: log1 is chosen.
        std::fs::write(dir.file("log2"), b"").unwrap();
        let mut log = LogFile::new();
        assert!(log.set_log_file(&name, 100, 3));
        assert_eq!(log.file_name().as_str(), Some(dir.file("log1").as_str()));
    }

    #[test]
    fn set_log_file_closes_the_previous_file_even_when_it_then_fails() {
        let dir = TempDir::new("reopen");
        let mut log = LogFile::new();
        assert!(log.set_log_file(&dir.file("first"), 100, 0));
        assert!(log.write_to_log(b"a"));

        // Open failure (missing parent directory): false, nothing open.
        assert!(!log.set_log_file(&dir.file("missing/log"), 100, 0));
        assert!(!log.is_open());
        assert!(log.write_to_log(b"b"));
        assert_eq!(std::fs::read(dir.file("first")).unwrap(), b"a");
    }

    #[test]
    fn name_is_cut_at_the_first_nul() {
        let dir = TempDir::new("nul");
        let name = dir.file("log");
        let mut log = LogFile::new();
        assert!(log.set_log_file(&format!("{name}\0ignored"), 100, 0));
        assert_eq!(log.file_name().as_str(), Some(name.as_str()));
        assert!(std::path::Path::new(&name).exists());
    }

    #[test]
    fn a_write_error_keeps_the_file_open() {
        // /dev/full accepts the open and fails every write with ENOSPC.
        if !std::path::Path::new("/dev/full").exists() {
            return;
        }
        let mut log = LogFile::new();
        // /dev/full "exists", so with no backups the original name is used.
        assert!(log.set_log_file("/dev/full", 100, 0));
        assert!(!log.write_to_log(b"line\n"));
        assert!(log.is_open());
        assert_eq!(log.current_file_size(), 0);
    }
}
