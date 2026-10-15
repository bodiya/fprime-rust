//! Port of `Os::Linux::Memory::LinuxMemory` (Os/Linux/Memory.{hpp,cpp}).
//!
//! The C++ backend calls `sysinfo(2)` and reports
//! `total = totalram * mem_unit`, `used = (totalram - freeram) * mem_unit`.
//! The kernel fills `totalram`/`freeram` from the same counters it prints as
//! `MemTotal` / `MemFree` in `/proc/meminfo` (in KiB), so this backend reads
//! those two lines with `mem_unit = 1024`. Note `MemFree`, NOT
//! `MemAvailable`: page cache counts as used, exactly as with `sysinfo`.
//!
//! Deviation: the C++ code `FW_ASSERT`s that `sysinfo` succeeds (it can
//! only fail on a bad pointer). Reading a file can fail at runtime (procfs
//! not mounted, sandboxing), which is not a programmer error, so it is
//! reported as [`Status::Error`] with `memory_usage` left untouched.

use super::string_to_number;
use crate::file::{self, File, Mode, WaitType};
use crate::memory::{MemoryInterface, Status, Usage};
use fprime_config::FwSizeType;

/// The procfs memory statistics file.
pub const PROC_MEMINFO_PATH: &str = "/proc/meminfo";

/// The `sysinfo` `mem_unit` equivalent: `/proc/meminfo` values are KiB.
const MEM_UNIT: FwSizeType = 1024;

/// Line buffer for `/proc/meminfo` (lines are ~30 bytes).
const LINE_SIZE: usize = 255;

/// Bound on the `/proc/meminfo` scan. `MemTotal` and `MemFree` are the
/// first two lines on every kernel since 2.6; the bound only guards a
/// malformed file.
const MAX_MEMINFO_LINES: usize = 64;

/// `MemTotal:` key (sysinfo `totalram`).
const MEM_TOTAL_KEY: &[u8] = b"MemTotal:";
/// `MemFree:` key (sysinfo `freeram`).
const MEM_FREE_KEY: &[u8] = b"MemFree:";

/// The Linux memory backend (stateless; see the module docs).
#[derive(Debug, Clone, Copy, Default)]
pub struct LinuxMemory;

impl LinuxMemory {
    /// Construct the backend (`const` so it can back the static singleton).
    pub const fn new() -> Self {
        Self
    }
}

impl MemoryInterface for LinuxMemory {
    fn get_usage(&self, memory_usage: &mut Usage) -> Status {
        get_usage_from(PROC_MEMINFO_PATH, memory_usage)
    }
}

/// `_getUsage` over the meminfo file at `path`.
fn get_usage_from(path: &str, memory_usage: &mut Usage) -> Status {
    let Some((total_ram, free_ram)) = read_meminfo(path) else {
        return Status::Error;
    };
    usage_from_units(total_ram, free_ram, MEM_UNIT, memory_usage)
}

/// The arithmetic half of `_getUsage`, including its overflow guard: a
/// count that cannot be scaled to bytes reports `1/1` and
/// [`Status::Error`].
fn usage_from_units(
    total_ram: FwSizeType,
    free_ram: FwSizeType,
    mem_unit: FwSizeType,
    memory_usage: &mut Usage,
) -> Status {
    // C++: FW_ASSERT(info.mem_unit > 0) — a constant here.
    fprime_fw::fw_assert!(mem_unit > 0);
    let max_measurable_ram_units = FwSizeType::MAX / mem_unit;
    if max_measurable_ram_units < total_ram || max_measurable_ram_units < free_ram {
        memory_usage.total = 1;
        memory_usage.used = 1;
        return Status::Error;
    }
    memory_usage.total = total_ram * mem_unit;
    // Unsigned C++ arithmetic: wraps if the kernel ever reported
    // free > total.
    memory_usage.used = total_ram.wrapping_sub(free_ram).wrapping_mul(mem_unit);
    Status::OpOk
}

/// Read `(MemTotal, MemFree)` in KiB from the meminfo file at `path`.
fn read_meminfo(path: &str) -> Option<(FwSizeType, FwSizeType)> {
    let mut file = File::new();
    if file.open(path, Mode::OpenRead) != file::Status::OpOk {
        return None;
    }
    let mut line = [0u8; LINE_SIZE];
    let mut total: Option<FwSizeType> = None;
    let mut free: Option<FwSizeType> = None;
    for _ in 0..MAX_MEMINFO_LINES {
        let mut read_size: FwSizeType = 0;
        if file.readline(&mut line, &mut read_size, WaitType::NoWait) != file::Status::OpOk {
            return None;
        }
        let text = &line[..read_size as usize];
        if text.is_empty() {
            // EOF.
            break;
        }
        if let Some(rest) = text.strip_prefix(MEM_TOTAL_KEY) {
            total = Some(string_to_number(rest)?.0);
        } else if let Some(rest) = text.strip_prefix(MEM_FREE_KEY) {
            free = Some(string_to_number(rest)?.0);
        }
        if let (Some(total), Some(free)) = (total, free) {
            return Some((total, free));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::temp_dir;

    /// A literal `/proc/meminfo` excerpt.
    const MEMINFO: &str = "\
MemTotal:       16316000 kB
MemFree:         2624936 kB
MemAvailable:   10624936 kB
Buffers:          412345 kB
";

    fn write_file(tag: &str, contents: &str) -> String {
        let dir = temp_dir(tag);
        let path = dir.join("meminfo");
        std::fs::write(&path, contents).unwrap();
        path.to_str().unwrap().to_string()
    }

    #[test]
    fn usage_is_memtotal_minus_memfree_in_bytes() {
        let path = write_file("mem_ok", MEMINFO);
        let mut usage = Usage::default();
        assert_eq!(get_usage_from(&path, &mut usage), Status::OpOk);
        assert_eq!(usage.total, 16_316_000 * 1024);
        // MemFree, not MemAvailable.
        assert_eq!(usage.used, (16_316_000 - 2_624_936) * 1024);
    }

    #[test]
    fn key_order_does_not_matter() {
        let path = write_file(
            "mem_order",
            "Foo: 1 kB\nMemFree: 10 kB\nBar: 2 kB\nMemTotal: 30 kB\n",
        );
        let mut usage = Usage::default();
        assert_eq!(get_usage_from(&path, &mut usage), Status::OpOk);
        assert_eq!(
            usage,
            Usage {
                used: 20 * 1024,
                total: 30 * 1024
            }
        );
    }

    #[test]
    fn missing_or_malformed_file_is_an_error_leaving_usage_untouched() {
        let untouched = Usage { used: 3, total: 4 };
        for contents in [
            "MemTotal: 30 kB\n",               // no MemFree
            "MemTotal: x kB\nMemFree: 1 kB\n", // non-numeric
            "",                                // empty
        ] {
            let path = write_file("mem_bad", contents);
            let mut usage = untouched;
            assert_eq!(get_usage_from(&path, &mut usage), Status::Error);
            assert_eq!(usage, untouched);
        }
        let missing = temp_dir("mem_missing").join("nope");
        let mut usage = untouched;
        assert_eq!(
            get_usage_from(missing.to_str().unwrap(), &mut usage),
            Status::Error
        );
        assert_eq!(usage, untouched);
    }

    #[test]
    fn overflow_guard_reports_one_of_one() {
        let max_units = FwSizeType::MAX / MEM_UNIT;
        let mut usage = Usage::default();
        assert_eq!(
            usage_from_units(max_units + 1, 0, MEM_UNIT, &mut usage),
            Status::Error
        );
        assert_eq!(usage, Usage { used: 1, total: 1 });
        let mut usage = Usage::default();
        assert_eq!(
            usage_from_units(0, max_units + 1, MEM_UNIT, &mut usage),
            Status::Error
        );
        assert_eq!(usage, Usage { used: 1, total: 1 });
        // Exactly at the limit is still measurable.
        let mut usage = Usage::default();
        assert_eq!(
            usage_from_units(max_units, max_units, MEM_UNIT, &mut usage),
            Status::OpOk
        );
        assert_eq!(usage.total, max_units * MEM_UNIT);
        assert_eq!(usage.used, 0);
    }

    #[test]
    fn linux_backend_reads_this_host() {
        if read_meminfo(PROC_MEMINFO_PATH).is_none() {
            return; // no procfs on this host
        }
        let mut usage = Usage::default();
        assert_eq!(LinuxMemory::new().get_usage(&mut usage), Status::OpOk);
        assert!(usage.total >= usage.used);
        assert!(usage.used > 0);
    }
}
