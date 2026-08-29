//! The F Prime directory facility.
//!
//! Port of `Os::Directory` (Os/Directory.hpp, Os/Posix/Directory.cpp) over
//! `std::fs::read_dir` — see `docs/cpp-analysis/os.md`.
//!
//! Semantics kept:
//! - open modes: [`OpenMode::Read`] (must exist),
//!   [`OpenMode::CreateIfMissing`] (mkdir, EEXIST tolerated),
//!   [`OpenMode::CreateExclusive`] (mkdir, EEXIST =>
//!   [`Status::AlreadyExists`]); any other mkdir error aborts before the
//!   directory stream opens;
//! - [`Directory::read`] skips `.` and `..` (std does this natively),
//!   silently truncates long names into the caller's fixed string (C++
//!   `string_copy` parity), and returns [`Status::NoMoreFiles`] at the end
//!   of the stream;
//! - `read_directory`/`get_file_count` rewind before AND after.

use fprime_config::FwSizeType;
use fprime_fw::FileNameString;
use fprime_fw::fw_assert;
use std::path::PathBuf;

/// Port of `Os::DirectoryInterface::Status` (Os/Directory.hpp) — exact C++
/// discriminants.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Operation was successful.
    OpOk = 0,
    /// Directory doesn't exist.
    DoesntExist = 1,
    /// No permission.
    NoPermission = 2,
    /// Directory hasn't been opened yet.
    NotOpened = 3,
    /// Path is not a directory.
    NotDir = 4,
    /// Directory stream has no more files.
    NoMoreFiles = 5,
    /// Too many files or links.
    FileLimit = 6,
    /// Directory stream descriptor is invalid.
    BadDescriptor = 7,
    /// Directory already exists.
    AlreadyExists = 8,
    /// Operation not supported.
    NotSupported = 9,
    /// All other errors.
    OtherError = 10,
}

/// Port of `Os::DirectoryInterface::OpenMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OpenMode {
    /// Open an existing directory for reading.
    Read = 0,
    /// Create the directory if missing, then open it.
    CreateIfMissing = 1,
    /// Create the directory; fail with [`Status::AlreadyExists`] if it
    /// exists.
    CreateExclusive = 2,
}

/// Map a std I/O error to the C++ Posix directory errno table.
fn map_io_error(error: &std::io::Error) -> Status {
    use std::io::ErrorKind;
    match error.kind() {
        ErrorKind::NotFound => Status::DoesntExist,
        ErrorKind::PermissionDenied => Status::NoPermission,
        ErrorKind::NotADirectory => Status::NotDir,
        ErrorKind::AlreadyExists => Status::AlreadyExists,
        _ => Status::OtherError,
    }
}

struct DirInner {
    path: PathBuf,
    stream: std::fs::ReadDir,
}

/// The F Prime directory (see module docs). Closes on drop.
#[derive(Default)]
pub struct Directory {
    inner: Option<DirInner>,
}

impl Directory {
    /// Construct a closed directory object.
    pub fn new() -> Self {
        Self { inner: None }
    }

    /// Open (and for the create modes, first create) the directory. See
    /// [`OpenMode`]. An already-open `Directory` is closed first.
    pub fn open(&mut self, path: &str, mode: OpenMode) -> Status {
        self.close();
        match mode {
            OpenMode::Read => {}
            OpenMode::CreateIfMissing | OpenMode::CreateExclusive => {
                if let Err(error) = std::fs::create_dir(path) {
                    let status = map_io_error(&error);
                    // C++ parity: EEXIST tolerated only for
                    // CREATE_IF_MISSING; any other mkdir error aborts
                    // before opendir.
                    if !(status == Status::AlreadyExists && mode == OpenMode::CreateIfMissing) {
                        return status;
                    }
                }
            }
        }
        match std::fs::read_dir(path) {
            Ok(stream) => {
                self.inner = Some(DirInner {
                    path: PathBuf::from(path),
                    stream,
                });
                Status::OpOk
            }
            Err(error) => map_io_error(&error),
        }
    }

    /// Whether the directory is open.
    pub fn is_open(&self) -> bool {
        self.inner.is_some()
    }

    /// Reset the read stream to the beginning (port of `rewind`; C++
    /// rewinddir always succeeds — a re-open failure here maps to
    /// [`Status::OtherError`]).
    pub fn rewind(&mut self) -> Status {
        let inner = match &mut self.inner {
            None => return Status::NotOpened,
            Some(inner) => inner,
        };
        match std::fs::read_dir(&inner.path) {
            Ok(stream) => {
                inner.stream = stream;
                Status::OpOk
            }
            Err(_) => Status::OtherError,
        }
    }

    /// Read the next entry name (port of `read`). Skips `.` and `..`;
    /// silently truncates names longer than the destination capacity (C++
    /// `string_copy` parity); [`Status::NoMoreFiles`] at end of stream.
    pub fn read(&mut self, filename: &mut FileNameString) -> Status {
        let inner = match &mut self.inner {
            None => return Status::NotOpened,
            Some(inner) => inner,
        };
        match inner.stream.next() {
            None => Status::NoMoreFiles,
            Some(Ok(entry)) => {
                let name = entry.file_name();
                filename.set(&name.to_string_lossy());
                Status::OpOk
            }
            // readdir with errno set: BAD_DESCRIPTOR (C++ parity).
            Some(Err(_)) => Status::BadDescriptor,
        }
    }

    /// Count the directory's entries (port of `getFileCount`): rewinds
    /// before and after.
    pub fn get_file_count(&mut self, file_count: &mut FwSizeType) -> Status {
        if !self.is_open() {
            return Status::NotOpened;
        }
        if self.rewind() != Status::OpOk {
            return Status::OtherError;
        }
        let mut count: FwSizeType = 0;
        let mut scratch = FileNameString::new();
        loop {
            match self.read(&mut scratch) {
                Status::NoMoreFiles => break,
                Status::OpOk => count += 1,
                _ => return Status::OtherError,
            }
        }
        if self.rewind() != Status::OpOk {
            return Status::OtherError;
        }
        *file_count = count;
        Status::OpOk
    }

    /// Fill `filenames` with entry names (port of `readDirectory`):
    /// rewinds before and after; `filename_count` receives the number of
    /// entries stored.
    pub fn read_directory(
        &mut self,
        filenames: &mut [FileNameString],
        filename_count: &mut FwSizeType,
    ) -> Status {
        fw_assert!(!filenames.is_empty());
        if !self.is_open() {
            return Status::NotOpened;
        }
        if self.rewind() != Status::OpOk {
            return Status::OtherError;
        }
        *filename_count = 0;
        let mut return_status = Status::OpOk;
        for slot in filenames.iter_mut() {
            match self.read(slot) {
                Status::OpOk => *filename_count += 1,
                Status::NoMoreFiles => break,
                other => {
                    return_status = other;
                    break;
                }
            }
        }
        if self.rewind() != Status::OpOk {
            return Status::OtherError;
        }
        return_status
    }

    /// Close the directory (idempotent).
    pub fn close(&mut self) {
        self.inner = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::temp_dir;

    #[test]
    fn open_read_missing_is_doesnt_exist() {
        let dir = temp_dir("dir_missing");
        let mut directory = Directory::new();
        assert_eq!(
            directory.open(dir.join("nope").to_str().unwrap(), OpenMode::Read),
            Status::DoesntExist
        );
        assert!(!directory.is_open());
        // Operations on a closed directory are NOT_OPENED.
        let mut name = FileNameString::new();
        assert_eq!(directory.read(&mut name), Status::NotOpened);
        assert_eq!(directory.rewind(), Status::NotOpened);
        let mut count = 0;
        assert_eq!(directory.get_file_count(&mut count), Status::NotOpened);
    }

    #[test]
    fn create_modes() {
        let dir = temp_dir("dir_create");
        let sub = dir.join("sub");
        let sub_str = sub.to_str().unwrap();
        let mut directory = Directory::new();
        assert_eq!(
            directory.open(sub_str, OpenMode::CreateExclusive),
            Status::OpOk
        );
        directory.close();
        // Exclusive on an existing directory fails.
        assert_eq!(
            directory.open(sub_str, OpenMode::CreateExclusive),
            Status::AlreadyExists
        );
        // If-missing tolerates EEXIST.
        assert_eq!(
            directory.open(sub_str, OpenMode::CreateIfMissing),
            Status::OpOk
        );
        assert!(directory.is_open());
    }

    #[test]
    fn read_entries_then_no_more_files() {
        let dir = temp_dir("dir_read");
        for name in ["b.txt", "a.txt", "c.txt"] {
            std::fs::write(dir.join(name), b"x").unwrap();
        }
        let mut directory = Directory::new();
        assert_eq!(
            directory.open(dir.to_str().unwrap(), OpenMode::Read),
            Status::OpOk
        );
        let mut names = Vec::new();
        let mut entry = FileNameString::new();
        loop {
            match directory.read(&mut entry) {
                Status::OpOk => names.push(entry.as_str().unwrap().to_string()),
                Status::NoMoreFiles => break,
                other => panic!("unexpected status {other:?}"),
            }
        }
        names.sort();
        // "." and ".." never appear.
        assert_eq!(names, ["a.txt", "b.txt", "c.txt"]);
    }

    #[test]
    fn file_count_and_rewind() {
        let dir = temp_dir("dir_count");
        for name in ["one", "two"] {
            std::fs::write(dir.join(name), b"x").unwrap();
        }
        let mut directory = Directory::new();
        assert_eq!(
            directory.open(dir.to_str().unwrap(), OpenMode::Read),
            Status::OpOk
        );
        let mut count: FwSizeType = 0;
        assert_eq!(directory.get_file_count(&mut count), Status::OpOk);
        assert_eq!(count, 2);
        // Counting rewound the stream: a full read pass still sees both.
        let mut entry = FileNameString::new();
        assert_eq!(directory.read(&mut entry), Status::OpOk);
        assert_eq!(directory.read(&mut entry), Status::OpOk);
        assert_eq!(directory.read(&mut entry), Status::NoMoreFiles);
    }

    #[test]
    fn read_directory_fills_array() {
        let dir = temp_dir("dir_bulk");
        for name in ["x", "y", "z"] {
            std::fs::write(dir.join(name), b"x").unwrap();
        }
        let mut directory = Directory::new();
        assert_eq!(
            directory.open(dir.to_str().unwrap(), OpenMode::Read),
            Status::OpOk
        );
        // Array larger than the entry count.
        let mut names = [const { FileNameString::new() }; 8];
        let mut count: FwSizeType = 0;
        assert_eq!(
            directory.read_directory(&mut names, &mut count),
            Status::OpOk
        );
        assert_eq!(count, 3);

        // Array smaller than the entry count: fills what fits.
        let mut two = [const { FileNameString::new() }; 2];
        assert_eq!(directory.read_directory(&mut two, &mut count), Status::OpOk);
        assert_eq!(count, 2);
    }
}
