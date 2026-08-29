//! The F Prime file-system facility.
//!
//! Port of `Os::FileSystem` (Os/FileSystem.{hpp,cpp}, Os/Posix/FileSystem.cpp)
//! — see `docs/cpp-analysis/os.md`. The C++ singleton + static wrappers
//! collapse to free functions in this module.
//!
//! Composite algorithms kept byte-for-byte in behavior (components like
//! FileManager and PrmDb depend on the exact status outcomes):
//! - `create_directory` goes through [`crate::Directory`] open modes;
//! - `touch` = `File` open OPEN_WRITE + close;
//! - `exists` treats ANY path-type error (including permission errors) as
//!   not-existing (C++ parity gotcha);
//! - `copy_file`/`append_file` copy in 512-byte chunks with WAIT
//!   reads/writes, a `2 * size` loop bound, and short-write =>
//!   [`Status::OtherError`];
//! - `move_file` renames, falling back to copy+remove ONLY on
//!   [`Status::ExdevError`] (cross-device) — other rename failures
//!   propagate;
//! - `handle_file_error` collapses most `File` statuses to
//!   [`Status::OtherError`] (only NO_SPACE / NO_PERMISSION / DOESNT_EXIST
//!   survive).

use crate::Directory;
use crate::directory;
use crate::file::{self, File, Mode, WaitType};
use fprime_config::FwSizeType;
use fprime_fw::FileNameString;
use fprime_fw::fw_assert;

/// C++ `FILE_SYSTEM_FILE_CHUNK_SIZE`: the chunked-copy buffer size.
pub const FILE_SYSTEM_FILE_CHUNK_SIZE: usize = file::FW_FILE_CHUNK_SIZE;

/// Port of `Os::FileSystemInterface::Status` (Os/FileSystem.hpp) — exact
/// C++ discriminants.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Operation was successful.
    OpOk = 0,
    /// File already exists.
    AlreadyExists = 1,
    /// No space left on device.
    NoSpace = 2,
    /// No permission.
    NoPermission = 3,
    /// Path is not a directory.
    NotDir = 4,
    /// Path is a directory.
    IsDir = 5,
    /// Directory is not empty.
    NotEmpty = 6,
    /// Path is invalid.
    InvalidPath = 7,
    /// Path doesn't exist.
    DoesntExist = 8,
    /// Too many files or links.
    FileLimit = 9,
    /// Resource is busy.
    Busy = 10,
    /// Directory stream has no more files.
    NoMoreFiles = 11,
    /// Buffer is too small.
    BufferTooSmall = 12,
    /// Cross-device rename error.
    ExdevError = 13,
    /// Arithmetic overflow in the operation.
    OverflowError = 14,
    /// Operation is not supported.
    NotSupported = 15,
    /// All other errors.
    OtherError = 16,
}

/// Port of `Os::FileSystemInterface::PathType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathType {
    /// Path is a regular file.
    File = 0,
    /// Path is a directory.
    Directory = 1,
    /// Path is something else (device, socket, symlink to nowhere, ...).
    Other = 2,
    /// Path does not exist.
    NotExist = 3,
}

/// Map a std I/O error to the C++ `errno_to_filesystem_status` table
/// (Os/Posix/error.cpp).
fn map_io_error(error: &std::io::Error) -> Status {
    use std::io::ErrorKind;
    match error.kind() {
        // ELOOP | ENOENT => DOESNT_EXIST (ELOOP has no stable ErrorKind and
        // lands in OtherError — noted divergence).
        ErrorKind::NotFound => Status::DoesntExist,
        // EPERM | EACCES | EROFS | EFAULT => NO_PERMISSION
        ErrorKind::PermissionDenied | ErrorKind::ReadOnlyFilesystem => Status::NoPermission,
        ErrorKind::AlreadyExists => Status::AlreadyExists,
        ErrorKind::NotADirectory => Status::NotDir,
        ErrorKind::IsADirectory => Status::IsDir,
        ErrorKind::DirectoryNotEmpty => Status::NotEmpty,
        // ENAMETOOLONG => INVALID_PATH in C++; std has no stable ErrorKind
        // for it at the 1.85 floor, so it lands in OtherError (noted
        // divergence).
        // EDQUOT | ENOSPC | EFBIG => NO_SPACE
        ErrorKind::StorageFull | ErrorKind::QuotaExceeded | ErrorKind::FileTooLarge => {
            Status::NoSpace
        }
        // EMLINK => FILE_LIMIT
        ErrorKind::TooManyLinks => Status::FileLimit,
        ErrorKind::ResourceBusy => Status::Busy,
        // EXDEV => EXDEV_ERROR
        ErrorKind::CrossesDevices => Status::ExdevError,
        ErrorKind::Unsupported => Status::NotSupported,
        _ => Status::OtherError,
    }
}

/// C++ `FileSystem::handleFileError`: only NO_SPACE / NO_PERMISSION /
/// DOESNT_EXIST survive; everything else collapses to OTHER_ERROR.
fn handle_file_error(file_status: file::Status) -> Status {
    match file_status {
        file::Status::NoSpace => Status::NoSpace,
        file::Status::NoPermission => Status::NoPermission,
        file::Status::DoesntExist => Status::DoesntExist,
        _ => Status::OtherError,
    }
}

/// C++ `FileSystem::handleDirectoryError`.
fn handle_directory_error(dir_status: directory::Status) -> Status {
    match dir_status {
        directory::Status::DoesntExist => Status::DoesntExist,
        directory::Status::NoPermission => Status::NoPermission,
        directory::Status::AlreadyExists => Status::AlreadyExists,
        directory::Status::NotSupported => Status::NotSupported,
        _ => Status::OtherError,
    }
}

/// Remove a directory (port of `removeDirectory`; the directory must be
/// empty).
pub fn remove_directory(path: &str) -> Status {
    match std::fs::remove_dir(path) {
        Ok(()) => Status::OpOk,
        Err(error) => map_io_error(&error),
    }
}

/// Remove a file (port of `removeFile`).
pub fn remove_file(path: &str) -> Status {
    match std::fs::remove_file(path) {
        Ok(()) => Status::OpOk,
        Err(error) => map_io_error(&error),
    }
}

/// Rename a file or directory (port of `rename`). A cross-device rename
/// returns [`Status::ExdevError`]; see [`move_file`] for the fallback.
pub fn rename(source_path: &str, dest_path: &str) -> Status {
    match std::fs::rename(source_path, dest_path) {
        Ok(()) => Status::OpOk,
        Err(error) => map_io_error(&error),
    }
}

/// Classify a path (port of `getPathType`). C++ parity: uses lstat
/// semantics (`symlink_metadata` — a symlink itself is Other) and ANY error
/// maps to [`PathType::NotExist`].
pub fn get_path_type(path: &str) -> PathType {
    match std::fs::symlink_metadata(path) {
        Ok(metadata) => {
            let file_type = metadata.file_type();
            if file_type.is_file() {
                PathType::File
            } else if file_type.is_dir() {
                PathType::Directory
            } else {
                PathType::Other
            }
        }
        Err(_) => PathType::NotExist,
    }
}

/// Whether the path exists (port of `exists`). C++ parity gotcha: any
/// path-type error — including a permission error — reads as "does not
/// exist".
pub fn exists(path: &str) -> bool {
    get_path_type(path) != PathType::NotExist
}

/// Create a directory (port of `createDirectory`): opens a
/// [`Directory`] with CREATE_EXCLUSIVE when `error_if_already_exists`,
/// CREATE_IF_MISSING otherwise, mapping directory statuses through the C++
/// table.
pub fn create_directory(path: &str, error_if_already_exists: bool) -> Status {
    let mut dir = Directory::new();
    let mode = if error_if_already_exists {
        directory::OpenMode::CreateExclusive
    } else {
        directory::OpenMode::CreateIfMissing
    };
    let dir_status = dir.open(path, mode);
    dir.close();
    if dir_status != directory::Status::OpOk {
        return handle_directory_error(dir_status);
    }
    Status::OpOk
}

/// Create an empty file, or update an existing one (port of `touch`):
/// `File` open OPEN_WRITE + close.
pub fn touch(path: &str) -> Status {
    let mut file = File::new();
    let file_status = file.open(path, Mode::OpenWrite);
    file.close();
    if file_status != file::Status::OpOk {
        return handle_file_error(file_status);
    }
    Status::OpOk
}

/// C++ `FileSystem::copyFileData`: chunked copy with a 512-byte buffer,
/// WAIT reads/writes, loop bounded to `2 * size`, short-write and
/// short-copy => OTHER_ERROR.
fn copy_file_data(source: &mut File, destination: &mut File, size: FwSizeType) -> Status {
    let mut buffer = [0u8; FILE_SYSTEM_FILE_CHUNK_SIZE];
    let maximum = size.saturating_mul(2);
    let mut copied: FwSizeType = 0;
    let mut iteration: FwSizeType = 0;
    while copied < size && iteration < maximum {
        iteration += 1;
        let chunk = (size - copied).min(FILE_SYSTEM_FILE_CHUNK_SIZE as FwSizeType) as usize;
        let mut read_size: FwSizeType = 0;
        let file_status = source.read(&mut buffer[..chunk], &mut read_size, WaitType::Wait);
        if file_status != file::Status::OpOk {
            return handle_file_error(file_status);
        }
        // Zero-byte read: the source ended before the expected size.
        if read_size == 0 {
            break;
        }
        let mut write_size: FwSizeType = 0;
        let file_status = destination.write(
            &buffer[..read_size as usize],
            &mut write_size,
            WaitType::Wait,
        );
        if file_status != file::Status::OpOk {
            return handle_file_error(file_status);
        }
        // Short write: the destination did not take all the data.
        if write_size != read_size {
            return Status::OtherError;
        }
        copied += write_size;
    }
    // A copy that did not transfer the full expected size is not a success.
    if copied != size {
        return Status::OtherError;
    }
    Status::OpOk
}

/// Copy a file (port of `copyFile`): source OPEN_READ, destination
/// OPEN_WRITE, then chunked copy of the source's size.
pub fn copy_file(source_path: &str, dest_path: &str) -> Status {
    let mut source = File::new();
    let mut destination = File::new();
    let file_status = source.open(source_path, Mode::OpenRead);
    if file_status != file::Status::OpOk {
        return handle_file_error(file_status);
    }
    let file_status = destination.open(dest_path, Mode::OpenWrite);
    if file_status != file::Status::OpOk {
        return handle_file_error(file_status);
    }
    let mut source_size: FwSizeType = 0;
    let fs_status = get_file_size(source_path, &mut source_size);
    if fs_status != Status::OpOk {
        return fs_status;
    }
    copy_file_data(&mut source, &mut destination, source_size)
}

/// Append `source_path` onto `dest_path` (port of `appendFile`). When
/// `create_missing_dest` is false and the destination does not exist,
/// returns [`Status::DoesntExist`] without touching either file.
pub fn append_file(source_path: &str, dest_path: &str, create_missing_dest: bool) -> Status {
    let dest_exists = exists(dest_path);
    if !create_missing_dest && !dest_exists {
        return Status::DoesntExist;
    }
    let mut source = File::new();
    let mut destination = File::new();
    let file_status = source.open(source_path, Mode::OpenRead);
    if file_status != file::Status::OpOk {
        return handle_file_error(file_status);
    }
    let file_status = destination.open(dest_path, Mode::OpenAppend);
    if file_status != file::Status::OpOk {
        return handle_file_error(file_status);
    }
    let mut source_size: FwSizeType = 0;
    let fs_status = get_file_size(source_path, &mut source_size);
    if fs_status != Status::OpOk {
        return fs_status;
    }
    copy_file_data(&mut source, &mut destination, source_size)
}

/// Move a file (port of `moveFile`): rename, falling back to copy + remove
/// ONLY on the cross-device error (C++ parity — any other rename failure
/// propagates).
pub fn move_file(source_path: &str, dest_path: &str) -> Status {
    let mut status = rename(source_path, dest_path);
    if status == Status::ExdevError {
        status = copy_file(source_path, dest_path);
        if status != Status::OpOk {
            return status;
        }
        status = remove_file(source_path);
    }
    status
}

/// Get the size of a file (port of `getFileSize`): open OPEN_READ + size.
pub fn get_file_size(path: &str, size: &mut FwSizeType) -> Status {
    let mut file = File::new();
    let file_status = file.open(path, Mode::OpenRead);
    if file_status != file::Status::OpOk {
        return handle_file_error(file_status);
    }
    let file_status = file.size(size);
    if file_status != file::Status::OpOk {
        return handle_file_error(file_status);
    }
    Status::OpOk
}

/// Get total and free space in bytes for the file system holding `path`.
///
/// DEVIATION: the C++ implementation uses `statvfs`, which has no
/// zero-dependency `std` equivalent — this port returns
/// [`Status::NotSupported`]. (`_getFreeSpace` overflow semantics are
/// therefore not reachable.)
pub fn get_free_space(
    path: &str,
    total_bytes: &mut FwSizeType,
    free_bytes: &mut FwSizeType,
) -> Status {
    let _ = (path, total_bytes, free_bytes);
    Status::NotSupported
}

/// Get the current working directory (port of `getWorkingDirectory`).
/// C++ parity: FW_ASSERTs a non-empty destination capacity; a path longer
/// than the destination returns [`Status::BufferTooSmall`] (ERANGE).
pub fn get_working_directory(path: &mut FileNameString) -> Status {
    fw_assert!(FileNameString::max_length() > 0);
    match std::env::current_dir() {
        Ok(cwd) => {
            let cwd = cwd.to_string_lossy();
            if cwd.len() > FileNameString::max_length() {
                return Status::BufferTooSmall;
            }
            path.set(&cwd);
            Status::OpOk
        }
        Err(error) => map_io_error(&error),
    }
}

/// Change the current working directory (port of
/// `changeWorkingDirectory`).
pub fn change_working_directory(path: &str) -> Status {
    match std::env::set_current_dir(path) {
        Ok(()) => Status::OpOk,
        Err(error) => map_io_error(&error),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::temp_dir;

    #[test]
    fn create_directory_exclusive_and_if_missing() {
        let dir = temp_dir("fs_mkdir");
        let sub = dir.join("sub");
        let sub_str = sub.to_str().unwrap();
        assert_eq!(create_directory(sub_str, true), Status::OpOk);
        assert!(sub.is_dir());
        // Exclusive create over an existing directory fails.
        assert_eq!(create_directory(sub_str, true), Status::AlreadyExists);
        // If-missing tolerates it.
        assert_eq!(create_directory(sub_str, false), Status::OpOk);
    }

    #[test]
    fn touch_exists_and_path_types() {
        let dir = temp_dir("fs_touch");
        let path = dir.join("touched.txt");
        let path_str = path.to_str().unwrap();
        assert!(!exists(path_str));
        assert_eq!(get_path_type(path_str), PathType::NotExist);
        assert_eq!(touch(path_str), Status::OpOk);
        assert!(exists(path_str));
        assert_eq!(get_path_type(path_str), PathType::File);
        assert_eq!(get_path_type(dir.to_str().unwrap()), PathType::Directory);
    }

    #[test]
    fn copy_file_multi_chunk_content_and_size() {
        let dir = temp_dir("fs_copy");
        let source = dir.join("src.bin");
        let dest = dir.join("dst.bin");
        // > 2 chunks to exercise the 512-byte loop plus a partial tail.
        let data: Vec<u8> = (0..1300u32).map(|i| (i % 253) as u8).collect();
        std::fs::write(&source, &data).unwrap();

        assert_eq!(
            copy_file(source.to_str().unwrap(), dest.to_str().unwrap()),
            Status::OpOk
        );
        assert_eq!(std::fs::read(&dest).unwrap(), data);

        let mut size: FwSizeType = 0;
        assert_eq!(
            get_file_size(dest.to_str().unwrap(), &mut size),
            Status::OpOk
        );
        assert_eq!(size, 1300);
    }

    #[test]
    fn copy_file_missing_source_is_doesnt_exist() {
        let dir = temp_dir("fs_copy_missing");
        assert_eq!(
            copy_file(
                dir.join("nope").to_str().unwrap(),
                dir.join("dst").to_str().unwrap()
            ),
            Status::DoesntExist
        );
    }

    #[test]
    fn append_file_composite() {
        let dir = temp_dir("fs_append");
        let source = dir.join("src.txt");
        let dest = dir.join("dst.txt");
        std::fs::write(&source, b" world").unwrap();
        std::fs::write(&dest, b"hello").unwrap();
        assert_eq!(
            append_file(source.to_str().unwrap(), dest.to_str().unwrap(), false),
            Status::OpOk
        );
        assert_eq!(std::fs::read(&dest).unwrap(), b"hello world");

        // Missing destination without create_missing_dest: DOESNT_EXIST.
        let missing = dir.join("missing.txt");
        assert_eq!(
            append_file(source.to_str().unwrap(), missing.to_str().unwrap(), false),
            Status::DoesntExist
        );
        // With create_missing_dest: created and filled.
        assert_eq!(
            append_file(source.to_str().unwrap(), missing.to_str().unwrap(), true),
            Status::OpOk
        );
        assert_eq!(std::fs::read(&missing).unwrap(), b" world");
    }

    #[test]
    fn move_file_renames_within_filesystem() {
        let dir = temp_dir("fs_move");
        let source = dir.join("src.txt");
        let dest = dir.join("dst.txt");
        std::fs::write(&source, b"payload").unwrap();
        assert_eq!(
            move_file(source.to_str().unwrap(), dest.to_str().unwrap()),
            Status::OpOk
        );
        assert!(!source.exists());
        assert_eq!(std::fs::read(&dest).unwrap(), b"payload");

        // Moving a missing file propagates the rename failure (no copy
        // fallback for non-EXDEV errors).
        assert_eq!(
            move_file(source.to_str().unwrap(), dest.to_str().unwrap()),
            Status::DoesntExist
        );
    }

    #[test]
    fn remove_file_and_directory() {
        let dir = temp_dir("fs_remove");
        let sub = dir.join("sub");
        let file = sub.join("f.txt");
        std::fs::create_dir(&sub).unwrap();
        std::fs::write(&file, b"x").unwrap();

        // Non-empty directory: NOT_EMPTY.
        assert_eq!(remove_directory(sub.to_str().unwrap()), Status::NotEmpty);
        assert_eq!(remove_file(file.to_str().unwrap()), Status::OpOk);
        assert_eq!(remove_directory(sub.to_str().unwrap()), Status::OpOk);
        assert_eq!(remove_file(file.to_str().unwrap()), Status::DoesntExist);
    }

    #[test]
    fn get_free_space_is_not_supported_on_std_backend() {
        let mut total = 0;
        let mut free = 0;
        assert_eq!(
            get_free_space("/", &mut total, &mut free),
            Status::NotSupported
        );
    }

    #[test]
    fn working_directory_round_trip() {
        let mut cwd = FileNameString::new();
        assert_eq!(get_working_directory(&mut cwd), Status::OpOk);
        assert!(!cwd.is_empty());
    }
}
