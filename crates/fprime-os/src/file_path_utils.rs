//! `Os::FilePathUtils` — lexical path resolution and sandbox containment.
//!
//! Port of `Os/FilePathUtils.{hpp,cpp}`. Resolution is purely textual: `.`,
//! `..` and `//` are collapsed in a single in-place pass with no `realpath`
//! and no symlink following, so a symlink inside a sandbox still escapes it
//! (C++ parity — do NOT "improve" this with `std::fs::canonicalize`, which
//! would change both the semantics and the error codes). Containment is a
//! prefix check against a canonical directory that ends in `/`.
//!
//! Consumers: [`crate::SandboxedFile`], `Svc::FileUplink`/`FileDownlink`
//! and `Svc::PrmDb`. These helpers formerly lived inline in
//! `fprime-svc::file_uplink` (with a second copy in `prm_db`) while this
//! crate belonged to an earlier porting wave.

use crate::filesystem;
use fprime_config::FILE_NAME_STRING_SIZE;
use fprime_fw::{FileNameString, fw_assert};

/// `Os::FilePathUtils::MAX_PATH_LENGTH` = `FileNameStringSize` = 240.
pub const MAX_PATH_LENGTH: usize = FILE_NAME_STRING_SIZE;

/// `Os::FilePathUtils::Status`.
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathStatus {
    /// Path is valid and within the allowed directory.
    Valid = 0,
    /// Resolved path falls outside the allowed directory.
    OutsideSandbox = 1,
    /// Path is malformed or cannot be resolved.
    InvalidPath = 2,
    /// Combined path length exceeds the output buffer.
    TooLong = 3,
}

/// C++ `FilePathUtils::buildAbsolutePath`: copy the raw (unresolved)
/// absolute path into `out`, prepending `base_dir` for a relative path.
fn build_absolute_path(path: &str, base_dir: &str, out: &mut [u8; MAX_PATH_LENGTH]) -> PathStatus {
    let path_bytes = path.as_bytes();
    if !path.starts_with('/') {
        let base = base_dir.as_bytes();
        // C++ isValidCString: string_length caps at MAX_PATH_LENGTH, and a
        // measured length equal to the cap means "no terminator found".
        if base.len() >= MAX_PATH_LENGTH {
            return PathStatus::InvalidPath;
        }
        if base.is_empty() || base[0] != b'/' {
            return PathStatus::InvalidPath;
        }
        if path_bytes.len() >= MAX_PATH_LENGTH {
            return PathStatus::InvalidPath;
        }
        let needs_slash = usize::from(base[base.len() - 1] != b'/');
        if base.len() + needs_slash + path_bytes.len() + 1 > MAX_PATH_LENGTH {
            return PathStatus::TooLong;
        }
        out[..base.len()].copy_from_slice(base);
        let mut pos = base.len();
        if needs_slash == 1 {
            out[pos] = b'/';
            pos += 1;
        }
        out[pos..pos + path_bytes.len()].copy_from_slice(path_bytes);
        pos += path_bytes.len();
        out[pos] = 0;
    } else {
        if path_bytes.len() >= MAX_PATH_LENGTH {
            return PathStatus::InvalidPath;
        }
        if path_bytes.len() + 1 > MAX_PATH_LENGTH {
            return PathStatus::TooLong;
        }
        out[..path_bytes.len()].copy_from_slice(path_bytes);
        out[path_bytes.len()] = 0;
    }
    PathStatus::Valid
}

/// The NUL-terminated length of `buf`, or `None` when there is no terminator
/// (the C++ `isValidCString` failure).
fn c_len(buf: &[u8]) -> Option<usize> {
    buf.iter().position(|b| *b == 0)
}

/// C++ `FilePathUtils::resolveInPlace`: collapse `.`, `..` and `//` with a
/// single read/write pass. `write_pos <= read_pos` always holds, so the
/// overlapping moves are safe (`copy_within` here, `memmove` in C++).
fn resolve_in_place(buf: &mut [u8; MAX_PATH_LENGTH]) -> PathStatus {
    fw_assert!(buf[0] == b'/');
    let path_length = match c_len(buf) {
        Some(len) => len,
        None => return PathStatus::InvalidPath,
    };

    let mut write_pos: usize = 1; // past the root '/'
    let mut read_pos: usize = 1;
    while read_pos <= path_length {
        let seg_start = read_pos;
        while read_pos < path_length && buf[read_pos] != b'/' {
            read_pos += 1;
        }
        let seg_len = read_pos - seg_start;
        if read_pos < path_length {
            read_pos += 1;
        } else {
            read_pos = path_length + 1;
        }

        if seg_len == 0 {
            continue; // "//"
        }
        if seg_len == 1 && buf[seg_start] == b'.' {
            continue; // "."
        }
        if seg_len == 2 && buf[seg_start] == b'.' && buf[seg_start + 1] == b'.' {
            if write_pos > 1 {
                write_pos -= 1;
                while write_pos > 1 && buf[write_pos - 1] != b'/' {
                    write_pos -= 1;
                }
            }
            continue;
        }
        buf.copy_within(seg_start..seg_start + seg_len, write_pos);
        write_pos += seg_len;
        buf[write_pos] = b'/';
        write_pos += 1;
    }

    // Drop the trailing '/' unless the result is exactly "/".
    if write_pos > 1 {
        write_pos -= 1;
    }
    buf[write_pos] = 0;
    PathStatus::Valid
}

/// C++ `FilePathUtils::resolvePath`: purely textual resolution — no
/// `realpath`, no symlink following. A relative `path` is resolved against
/// `base_dir`, which must itself be absolute.
pub fn resolve_path(path: &str, base_dir: &str, resolved: &mut FileNameString) -> PathStatus {
    if path.is_empty() {
        return PathStatus::InvalidPath;
    }
    let mut buf = [0u8; MAX_PATH_LENGTH];
    let status = build_absolute_path(path, base_dir, &mut buf);
    if status != PathStatus::Valid {
        return status;
    }
    let status = resolve_in_place(&mut buf);
    if status != PathStatus::Valid {
        return status;
    }
    let len = c_len(&buf).unwrap_or(0);
    resolved.set_bytes(&buf[..len]);
    PathStatus::Valid
}

/// C++ `FilePathUtils::resolveFromCwd`: relative paths resolve against the
/// process working directory, absolute paths against `/`.
pub fn resolve_from_cwd(path: &str, resolved: &mut FileNameString) -> PathStatus {
    if !path.starts_with('/') {
        let mut cwd = FileNameString::new();
        if filesystem::get_working_directory(&mut cwd) != filesystem::Status::OpOk {
            return PathStatus::InvalidPath;
        }
        return match cwd.as_str() {
            Some(base) => resolve_path(path, base, resolved),
            None => PathStatus::InvalidPath,
        };
    }
    resolve_path(path, "/", resolved)
}

/// C++ `FilePathUtils::checkContainment`: both arguments must already be
/// canonical; `allowed_directory` must end with `/`. A path equal to the
/// sandbox directory itself (without the trailing slash) is contained.
pub fn check_containment(resolved_path: &str, allowed_directory: &str) -> PathStatus {
    let allowed = allowed_directory.as_bytes();
    let path = resolved_path.as_bytes();
    if allowed.len() >= MAX_PATH_LENGTH || path.len() >= MAX_PATH_LENGTH {
        return PathStatus::InvalidPath;
    }
    if allowed.is_empty() || path.is_empty() {
        return PathStatus::OutsideSandbox;
    }
    if allowed[allowed.len() - 1] != b'/' {
        return PathStatus::OutsideSandbox;
    }
    if path.len() == allowed.len() - 1 && path == &allowed[..path.len()] {
        return PathStatus::Valid;
    }
    if path.len() < allowed.len() {
        return PathStatus::OutsideSandbox;
    }
    if &path[..allowed.len()] != allowed {
        return PathStatus::OutsideSandbox;
    }
    PathStatus::Valid
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolve_path_collapses_dot_and_dotdot() {
        let cases = [
            ("/a/b/../c", "/", "/a/c"),
            ("/a/./b//c/", "/", "/a/b/c"),
            ("/../..", "/", "/"),
            ("/a/b/../../../c", "/", "/c"),
            ("rel/x", "/base", "/base/rel/x"),
            ("rel/x", "/base/", "/base/rel/x"),
            ("/", "/", "/"),
        ];
        for (path, base, expected) in cases {
            let mut out = FileNameString::new();
            assert_eq!(resolve_path(path, base, &mut out), PathStatus::Valid);
            assert_eq!(out.as_str(), Some(expected), "{path} against {base}");
        }
    }

    #[test]
    fn resolve_path_rejects_empty_and_relative_without_absolute_base() {
        let mut out = FileNameString::new();
        assert_eq!(resolve_path("", "/", &mut out), PathStatus::InvalidPath);
        assert_eq!(
            resolve_path("rel", "notabsolute", &mut out),
            PathStatus::InvalidPath
        );
        assert_eq!(resolve_path("rel", "", &mut out), PathStatus::InvalidPath);
    }

    #[test]
    fn resolve_path_reports_too_long() {
        let mut out = FileNameString::new();
        let long = format!("/{}", "a".repeat(MAX_PATH_LENGTH));
        assert_eq!(resolve_path(&long, "/", &mut out), PathStatus::InvalidPath);
        let base = format!("/{}", "b".repeat(MAX_PATH_LENGTH - 10));
        assert_eq!(
            resolve_path("relative/path/that/is/long", &base, &mut out),
            PathStatus::TooLong
        );
    }

    #[test]
    fn resolve_from_cwd_uses_the_working_directory_for_relative_paths() {
        let mut out = FileNameString::new();
        assert_eq!(resolve_from_cwd("/x/../y", &mut out), PathStatus::Valid);
        assert_eq!(out.as_str(), Some("/y"));
        let cwd = std::env::current_dir().unwrap();
        let mut expected = FileNameString::new();
        assert_eq!(
            resolve_path("sub/f.bin", cwd.to_str().unwrap(), &mut expected),
            PathStatus::Valid
        );
        assert_eq!(resolve_from_cwd("sub/f.bin", &mut out), PathStatus::Valid);
        assert_eq!(out.as_str(), expected.as_str());
    }

    #[test]
    fn check_containment_matches_the_cpp_rules() {
        assert_eq!(
            check_containment("/data/uplink/f.bin", "/data/uplink/"),
            PathStatus::Valid
        );
        // The path IS the sandbox directory (no trailing slash).
        assert_eq!(
            check_containment("/data/uplink", "/data/uplink/"),
            PathStatus::Valid
        );
        assert_eq!(
            check_containment("/data/uplinkx/f", "/data/uplink/"),
            PathStatus::OutsideSandbox
        );
        assert_eq!(
            check_containment("/other", "/data/"),
            PathStatus::OutsideSandbox
        );
        // An allowed directory without a trailing slash never matches.
        assert_eq!(
            check_containment("/data/f", "/data"),
            PathStatus::OutsideSandbox
        );
        assert_eq!(check_containment("", "/data/"), PathStatus::OutsideSandbox);
    }

    #[test]
    fn path_status_discriminants_match_cpp() {
        assert_eq!(PathStatus::Valid as i32, 0);
        assert_eq!(PathStatus::OutsideSandbox as i32, 1);
        assert_eq!(PathStatus::InvalidPath as i32, 2);
        assert_eq!(PathStatus::TooLong as i32, 3);
    }
}
