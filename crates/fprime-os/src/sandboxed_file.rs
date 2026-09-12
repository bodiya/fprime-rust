//! `Os::SandboxedFile` — a [`File`] whose `open` is restricted to a
//! directory subtree.
//!
//! Port of `Os/SandboxedFile.{hpp,cpp}`. Every open resolves the requested
//! path with [`resolve_from_cwd`], checks it against the configured
//! directory with [`check_containment`] and opens the RESOLVED path; any
//! resolution or containment failure is reported as
//! [`FileStatus::OutsideSandbox`], never as a more specific error. Formerly
//! lived inline in `fprime-svc::file_uplink` while this crate belonged to
//! an earlier porting wave.

use crate::file::{File, Mode, SeekType, Status as FileStatus, WaitType};
use crate::file_path_utils::{MAX_PATH_LENGTH, PathStatus, check_containment, resolve_from_cwd};
use fprime_config::FwSizeType;
use fprime_fw::{FileNameString, fw_assert};

/// `Os::SandboxedFile` — an [`File`] whose `open` is restricted to a
/// directory subtree.
///
/// **FAIL-OPEN by default** (C++ parity): a default-constructed instance is
/// already "configured" with `/` as its allowed directory, and the stock
/// `FileHandling` subtopology never calls [`configure`](Self::configure).
/// Path resolution is purely textual, so a symlink inside the sandbox still
/// escapes it — do NOT "improve" this with `std::fs::canonicalize`, which
/// would change both the semantics and the error codes.
pub struct SandboxedFile {
    file: File,
    allowed_directory: FileNameString,
    configured: bool,
}

impl Default for SandboxedFile {
    fn default() -> Self {
        Self::new()
    }
}

impl SandboxedFile {
    /// A fail-open sandboxed file (allowed directory `/`, configured).
    #[must_use]
    pub fn new() -> Self {
        Self {
            file: File::new(),
            allowed_directory: FileNameString::from("/"),
            configured: true,
        }
    }

    /// C++ `SandboxedFile::configure`: resolve `directory` from the CWD and
    /// store it with a trailing `/`. Asserts (as C++ does) that the file is
    /// closed and that the directory resolves.
    pub fn configure(&mut self, directory: &str) {
        fw_assert!(!self.file.is_open());
        let mut resolved = FileNameString::new();
        let status = resolve_from_cwd(directory, &mut resolved);
        fw_assert!(status == PathStatus::Valid, status as i32);
        let len = resolved.len();
        fw_assert!(len > 0);
        fw_assert!(len + 2 <= MAX_PATH_LENGTH, len as i32);
        if resolved.as_bytes()[len - 1] != b'/' {
            resolved.append("/");
        }
        self.allowed_directory = resolved;
        self.configured = true;
    }

    /// The configured sandbox directory (always ends with `/`).
    #[must_use]
    pub fn sandbox_directory(&self) -> &FileNameString {
        &self.allowed_directory
    }

    /// True when a sandbox is configured (always true — fail-open).
    #[must_use]
    pub fn is_configured(&self) -> bool {
        self.configured
    }

    /// C++ `SandboxedFile::open`: resolve, check containment, then open the
    /// RESOLVED path. Any resolution or containment failure is
    /// [`FileStatus::OutsideSandbox`].
    pub fn open(&mut self, path: &str, mode: Mode) -> FileStatus {
        if !self.configured {
            return FileStatus::OutsideSandbox;
        }
        let mut resolved = FileNameString::new();
        if resolve_from_cwd(path, &mut resolved) != PathStatus::Valid {
            return FileStatus::OutsideSandbox;
        }
        let (resolved_str, allowed_str) = match (resolved.as_str(), self.allowed_directory.as_str())
        {
            (Some(r), Some(a)) => (r, a),
            _ => return FileStatus::OutsideSandbox,
        };
        if check_containment(resolved_str, allowed_str) != PathStatus::Valid {
            return FileStatus::OutsideSandbox;
        }
        self.file.open(resolved_str, mode)
    }

    /// Close the underlying file (idempotent).
    pub fn close(&mut self) {
        self.file.close();
    }

    /// True when the underlying file is open.
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.file.is_open()
    }

    /// Forwarded to [`File::size`].
    pub fn size(&mut self, size_result: &mut FwSizeType) -> FileStatus {
        self.file.size(size_result)
    }

    /// Forwarded to [`File::seek`].
    pub fn seek(&mut self, offset: i64, seek_type: SeekType) -> FileStatus {
        self.file.seek(offset, seek_type)
    }

    /// Forwarded to [`File::read`].
    pub fn read(&mut self, buffer: &mut [u8], size: &mut FwSizeType, wait: WaitType) -> FileStatus {
        self.file.read(buffer, size, wait)
    }

    /// Forwarded to [`File::write`].
    pub fn write(&mut self, buffer: &[u8], size: &mut FwSizeType, wait: WaitType) -> FileStatus {
        self.file.write(buffer, size, wait)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    static COUNTER: AtomicU32 = AtomicU32::new(0);

    fn temp_dir(tag: &str) -> std::path::PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "fpsb{}_{}_{tag}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::SeqCst)
        ));
        std::fs::create_dir_all(&path).unwrap();
        path
    }

    #[test]
    fn sandboxed_file_is_fail_open_by_default() {
        let sandbox = SandboxedFile::new();
        assert!(sandbox.is_configured());
        assert_eq!(sandbox.sandbox_directory().as_str(), Some("/"));
    }

    #[test]
    fn sandbox_configure_appends_a_trailing_slash() {
        let dir = temp_dir("cfg");
        let mut sandbox = SandboxedFile::new();
        sandbox.configure(dir.to_str().unwrap());
        let stored = sandbox.sandbox_directory().as_str().unwrap().to_string();
        assert!(stored.ends_with('/'), "{stored}");
        assert_eq!(stored, format!("{}/", dir.to_str().unwrap()));
    }

    #[test]
    fn open_inside_the_sandbox_succeeds_and_outside_is_outside_sandbox() {
        let dir = temp_dir("open");
        let inside = dir.join("in.bin");
        std::fs::write(&inside, b"abc").unwrap();
        let outside = temp_dir("other").join("out.bin");
        std::fs::write(&outside, b"xyz").unwrap();

        let mut sandbox = SandboxedFile::new();
        sandbox.configure(dir.to_str().unwrap());

        assert_eq!(
            sandbox.open(inside.to_str().unwrap(), Mode::OpenRead),
            FileStatus::OpOk
        );
        assert!(sandbox.is_open());
        let mut size = 0;
        assert_eq!(sandbox.size(&mut size), FileStatus::OpOk);
        assert_eq!(size, 3);
        sandbox.close();
        assert!(!sandbox.is_open());

        // A traversal that lexically escapes the directory is rejected
        // before the OS is consulted.
        let escape = format!(
            "{}/../{}",
            dir.to_str().unwrap(),
            outside.to_str().unwrap().trim_start_matches('/')
        );
        assert_eq!(
            sandbox.open(&escape, Mode::OpenRead),
            FileStatus::OutsideSandbox
        );
        assert_eq!(
            sandbox.open(outside.to_str().unwrap(), Mode::OpenRead),
            FileStatus::OutsideSandbox
        );
        assert!(!sandbox.is_open());
    }

    #[test]
    fn a_dotdot_that_stays_inside_the_sandbox_is_allowed() {
        let dir = temp_dir("dotdot");
        std::fs::create_dir_all(dir.join("sub")).unwrap();
        std::fs::write(dir.join("f.bin"), b"1").unwrap();
        let mut sandbox = SandboxedFile::new();
        sandbox.configure(dir.to_str().unwrap());
        let path = format!("{}/sub/../f.bin", dir.to_str().unwrap());
        assert_eq!(sandbox.open(&path, Mode::OpenRead), FileStatus::OpOk);
    }
}
