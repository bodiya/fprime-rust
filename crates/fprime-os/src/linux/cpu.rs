//! Port of `Os::Linux::Cpu::LinuxCpu` (Os/Linux/Cpu.{hpp,cpp}).
//!
//! - [`LinuxCpu::get_count`] is `sysconf(_SC_NPROCESSORS_ONLN)`. Without
//!   libc it reproduces what glibc's implementation reads: the online-CPU
//!   list in `/sys/devices/system/cpu/online`, falling back to counting the
//!   `cpuN` lines of `/proc/stat`, then to the affinity mask
//!   (`std::thread::available_parallelism`).
//! - [`LinuxCpu::get_ticks`] is a line-for-line port of `getCpuData`: read
//!   `/proc/stat` with `Os::File::readline` into a 255-byte line buffer,
//!   reject any line among the first `cpu_index + 2` that does not start
//!   with `"cpu"`, parse the 8 leading numeric fields after the `"cpu"`
//!   prefix, require field 0 (the CPU number) to equal `cpu_index`, and
//!   report `used = USER + NICE + SYSTEM`, `total = used + IDLE`.

use super::{string_to_number, trim_end};
use crate::cpu::{CpuInterface, Status, Ticks};
use crate::file::{self, File, Mode, WaitType};
use fprime_config::FwSizeType;

/// The procfs CPU statistics file.
pub const PROC_STAT_PATH: &str = "/proc/stat";
/// The sysfs online-CPU list (`cpulist` format, e.g. `0-3,8-11`).
pub const SYS_CPU_ONLINE_PATH: &str = "/sys/devices/system/cpu/online";

// C++ `ProcCpuMeasures`: field indices within a `/proc/stat` cpu line
// (counted after the "cpu" prefix; kernel < 2.5 compatible).
/// The CPU-number field.
const CPU_NUMBER: usize = 0;
/// The USER field.
const USER: usize = 1;
/// The NICE field.
const NICE: usize = 2;
/// The SYSTEM field.
const SYSTEM: usize = 3;
/// The IDLE field.
const IDLE: usize = 4;
/// Number of leading fields parsed from a line.
const MAX_CPU_TICK_TYPES: usize = 8;

/// C++ `ProcCpuData`.
type ProcCpuData = [FwSizeType; MAX_CPU_TICK_TYPES];

/// C++ `LINE_SIZE`: log10(max U64) * 11 fields (kernel 2.6.33) = 220,
/// rounded to 256 - 1 (the C++ buffer reserves one byte for the NUL, which
/// a Rust slice does not need).
const LINE_SIZE: usize = 255;

/// The `"cpu"` prefix every line of the CPU section starts with.
const CPU_PREFIX: &[u8] = b"cpu";

/// Bound on the `/proc/stat` line-count fallback of [`online_cpu_count`]:
/// the aggregate line plus the kernel's `CONFIG_NR_CPUS` ceiling (8192).
const MAX_PROC_STAT_CPU_LINES: usize = 8192 + 1;

/// The Linux CPU backend (stateless; see the module docs).
#[derive(Debug, Clone, Copy, Default)]
pub struct LinuxCpu;

impl LinuxCpu {
    /// Construct the backend (`const` so it can back the static singleton).
    pub const fn new() -> Self {
        Self
    }
}

impl CpuInterface for LinuxCpu {
    /// C++ parity: a positive online count is [`Status::OpOk`]; anything
    /// else writes 0 and returns [`Status::Error`].
    fn get_count(&self, cpu_count: &mut FwSizeType) -> Status {
        match online_cpu_count() {
            Some(cpus) if cpus > 0 && cpus < FwSizeType::MAX => {
                *cpu_count = cpus;
                Status::OpOk
            }
            _ => {
                *cpu_count = 0;
                Status::Error
            }
        }
    }

    /// C++ parity: re-check the count and reject `cpu_index >= count`
    /// before touching the file. Note the C++ code fills `ticks` from the
    /// (zero-initialized, possibly partially parsed) field array even when
    /// `getCpuData` fails; that is reproduced.
    fn get_ticks(&self, ticks: &mut Ticks, cpu_index: FwSizeType) -> Status {
        let mut count: FwSizeType = 0;
        let status = self.get_count(&mut count);
        if status != Status::OpOk {
            return status;
        } else if cpu_index >= count {
            return Status::Error;
        }
        let mut cpu_data: ProcCpuData = [0; MAX_CPU_TICK_TYPES];
        let status = get_cpu_data(PROC_STAT_PATH, cpu_index, &mut cpu_data);
        fill_ticks(ticks, &cpu_data);
        status
    }
}

/// `used = USER + NICE + SYSTEM`, `total = used + IDLE` (unsigned C++
/// arithmetic, so wrapping).
fn fill_ticks(ticks: &mut Ticks, cpu_data: &ProcCpuData) {
    ticks.used = cpu_data[USER]
        .wrapping_add(cpu_data[NICE])
        .wrapping_add(cpu_data[SYSTEM]);
    ticks.total = ticks.used.wrapping_add(cpu_data[IDLE]);
}

/// Port of `getCpuData`: read line `cpu_index + 2` of the stat file at
/// `path` and parse its leading fields into `data`.
fn get_cpu_data(path: &str, cpu_index: FwSizeType, data: &mut ProcCpuData) -> Status {
    let mut file = File::new();
    if file.open(path, Mode::OpenRead) != file::Status::OpOk {
        return Status::Error;
    }
    let mut proc_stat_line = [0u8; LINE_SIZE];
    let mut read_size: FwSizeType = 0;
    // The file starts with the aggregate cpu line, then one per CPU. The
    // loop is bounded by the caller's `cpu_index < count` check.
    let line_count = cpu_index.saturating_add(2);
    for _ in 0..line_count {
        read_size = 0;
        if file.readline(&mut proc_stat_line, &mut read_size, WaitType::NoWait)
            != file::Status::OpOk
        {
            return Status::Error;
        }
        // Make sure we have not strayed past the cpu section.
        if !proc_stat_line[..read_size as usize].starts_with(CPU_PREFIX) {
            return Status::Error;
        }
    }
    parse_cpu_line(&proc_stat_line[..read_size as usize], cpu_index, data)
}

/// The field-parsing half of `getCpuData`: the 8 numbers after the `"cpu"`
/// prefix of `line`, field 0 required to equal `cpu_index`. Fields parsed
/// before a failure stay in `data` (C++ parity).
fn parse_cpu_line(line: &[u8], cpu_index: FwSizeType, data: &mut ProcCpuData) -> Status {
    if !line.starts_with(CPU_PREFIX) {
        return Status::Error;
    }
    let mut cursor = CPU_PREFIX.len();
    for (i, slot) in data.iter_mut().enumerate() {
        let Some((token, consumed)) = string_to_number(&line[cursor..]) else {
            return Status::Error;
        };
        cursor += consumed;
        *slot = token;
        // Check the cpu index is correct.
        if i == CPU_NUMBER && token != cpu_index {
            return Status::Error;
        }
    }
    Status::OpOk
}

/// `sysconf(_SC_NPROCESSORS_ONLN)` as glibc computes it (see module docs).
fn online_cpu_count() -> Option<FwSizeType> {
    count_from_online_list(SYS_CPU_ONLINE_PATH)
        .or_else(|| count_from_proc_stat(PROC_STAT_PATH))
        .or_else(|| {
            std::thread::available_parallelism()
                .ok()
                .map(|cpus| cpus.get() as FwSizeType)
        })
}

/// Count the CPUs in a sysfs `cpulist` file (`0-3,8,10-11`).
fn count_from_online_list(path: &str) -> Option<FwSizeType> {
    let mut file = File::new();
    if file.open(path, Mode::OpenRead) != file::Status::OpOk {
        return None;
    }
    let mut line = [0u8; LINE_SIZE];
    let mut read_size: FwSizeType = 0;
    if file.readline(&mut line, &mut read_size, WaitType::NoWait) != file::Status::OpOk {
        return None;
    }
    parse_cpu_list(trim_end(&line[..read_size as usize]))
}

/// Count the CPUs named by a `cpulist` string; `None` when malformed.
fn parse_cpu_list(list: &[u8]) -> Option<FwSizeType> {
    if list.is_empty() {
        return None;
    }
    let mut count: FwSizeType = 0;
    for range in list.split(|&byte| byte == b',') {
        let (first, consumed) = string_to_number(range)?;
        let last = match &range[consumed..] {
            [] => first,
            [b'-', rest @ ..] => match string_to_number(rest)? {
                (last, used) if used == rest.len() && last >= first => last,
                _ => return None,
            },
            _ => return None,
        };
        count = count.checked_add(last - first + 1)?;
    }
    Some(count)
}

/// glibc's fallback: count the `cpuN` lines at the top of `/proc/stat`.
fn count_from_proc_stat(path: &str) -> Option<FwSizeType> {
    let mut file = File::new();
    if file.open(path, Mode::OpenRead) != file::Status::OpOk {
        return None;
    }
    let mut line = [0u8; LINE_SIZE];
    let mut count: FwSizeType = 0;
    for _ in 0..MAX_PROC_STAT_CPU_LINES {
        let mut read_size: FwSizeType = 0;
        if file.readline(&mut line, &mut read_size, WaitType::NoWait) != file::Status::OpOk {
            return None;
        }
        match line[..read_size as usize].strip_prefix(CPU_PREFIX) {
            // The aggregate "cpu " line is not a CPU.
            Some([digit, ..]) if digit.is_ascii_digit() => count += 1,
            Some(_) => {}
            // End of the cpu section (or EOF).
            None => break,
        }
    }
    Some(count)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::temp_dir;

    /// A literal `/proc/stat` excerpt (kernel 6.x layout, 10 tick fields).
    const STAT: &str = "\
cpu  270288 1598 88660 8470416 15185 9775 2990 867 0 0
cpu0 67462 108 22104 2118172 3779 2540 648 287 0 0
cpu1 74237 650 24059 2109680 2995 2447 667 93 0 0
cpu2 69388 630 22033 2115177 4428 2424 649 378 0 0
intr 123456 1 2 3
ctxt 99
";

    fn write_file(tag: &str, contents: &str) -> String {
        let dir = temp_dir(tag);
        let path = dir.join("stat");
        std::fs::write(&path, contents).unwrap();
        path.to_str().unwrap().to_string()
    }

    #[test]
    fn get_cpu_data_reads_the_requested_cpu_line() {
        let path = write_file("cpu_data_ok", STAT);
        let mut data: ProcCpuData = [0; MAX_CPU_TICK_TYPES];
        assert_eq!(get_cpu_data(&path, 1, &mut data), Status::OpOk);
        assert_eq!(data, [1, 74237, 650, 24059, 2109680, 2995, 2447, 667]);

        let mut ticks = Ticks::default();
        fill_ticks(&mut ticks, &data);
        assert_eq!(ticks.used, 74237 + 650 + 24059);
        assert_eq!(ticks.total, 74237 + 650 + 24059 + 2109680);
    }

    #[test]
    fn get_cpu_data_rejects_lines_past_the_cpu_section() {
        // Index 3 lands on the "intr" line.
        let path = write_file("cpu_data_past", STAT);
        let mut data: ProcCpuData = [0; MAX_CPU_TICK_TYPES];
        assert_eq!(get_cpu_data(&path, 3, &mut data), Status::Error);
        // Way past EOF: readline returns an empty line, which is not "cpu".
        assert_eq!(get_cpu_data(&path, 40, &mut data), Status::Error);
    }

    #[test]
    fn get_cpu_data_requires_the_cpu_number_to_match_the_index() {
        // An offline cpu1 shifts cpu2 onto the line for index 1.
        let path = write_file(
            "cpu_data_gap",
            "cpu  1 1 1 1 1 1 1 1\ncpu0 1 1 1 1 1 1 1 1\ncpu2 2 2 2 2 2 2 2 2\n",
        );
        let mut data: ProcCpuData = [0; MAX_CPU_TICK_TYPES];
        assert_eq!(get_cpu_data(&path, 1, &mut data), Status::Error);
        // The mismatching CPU number was still stored (C++ parity).
        assert_eq!(data[CPU_NUMBER], 2);
    }

    #[test]
    fn get_cpu_data_fails_on_a_missing_file() {
        let mut data: ProcCpuData = [0; MAX_CPU_TICK_TYPES];
        let missing = temp_dir("cpu_data_missing").join("nope");
        assert_eq!(
            get_cpu_data(missing.to_str().unwrap(), 0, &mut data),
            Status::Error
        );
    }

    #[test]
    fn get_cpu_data_fails_on_an_overlong_line() {
        // readline cannot find a newline within LINE_SIZE bytes.
        let long = format!("cpu  1 1 1 1 1 1 1 1{}\n", " 1".repeat(200));
        let path = write_file("cpu_data_long", &long);
        let mut data: ProcCpuData = [0; MAX_CPU_TICK_TYPES];
        assert_eq!(get_cpu_data(&path, 0, &mut data), Status::Error);
    }

    #[test]
    fn parse_cpu_line_needs_eight_numeric_fields() {
        let mut data: ProcCpuData = [0; MAX_CPU_TICK_TYPES];
        assert_eq!(
            parse_cpu_line(b"cpu0 1 2 3 4 5 6 7\n", 0, &mut data),
            Status::OpOk
        );
        assert_eq!(data, [0, 1, 2, 3, 4, 5, 6, 7]);
        // Only 7 fields: the 8th conversion runs into the newline.
        let mut data: ProcCpuData = [0; MAX_CPU_TICK_TYPES];
        assert_eq!(
            parse_cpu_line(b"cpu0 1 2 3 4 5 6\n", 0, &mut data),
            Status::Error
        );
        assert_eq!(data, [0, 1, 2, 3, 4, 5, 6, 0], "partial parse kept");
        // A non-numeric field.
        assert_eq!(
            parse_cpu_line(b"cpu0 1 x 3 4 5 6 7\n", 0, &mut data),
            Status::Error
        );
        // An out-of-range field (INVALID_RANGE).
        assert_eq!(
            parse_cpu_line(b"cpu0 1 99999999999999999999 3 4 5 6 7\n", 0, &mut data),
            Status::Error
        );
        // Not a cpu line.
        assert_eq!(
            parse_cpu_line(b"intr 0 1 2 3 4 5 6 7\n", 0, &mut data),
            Status::Error
        );
    }

    #[test]
    fn ticks_arithmetic_wraps_like_unsigned_cpp() {
        let data: ProcCpuData = [0, u64::MAX, 1, 0, 5, 0, 0, 0];
        let mut ticks = Ticks::default();
        fill_ticks(&mut ticks, &data);
        assert_eq!(ticks.used, 0);
        assert_eq!(ticks.total, 5);
    }

    #[test]
    fn cpu_list_counts_ranges_and_singletons() {
        assert_eq!(parse_cpu_list(b"0"), Some(1));
        assert_eq!(parse_cpu_list(b"0-7"), Some(8));
        assert_eq!(parse_cpu_list(b"0-3,8,10-11"), Some(7));
        assert_eq!(parse_cpu_list(b""), None);
        assert_eq!(parse_cpu_list(b"3-1"), None);
        assert_eq!(parse_cpu_list(b"0-"), None);
        assert_eq!(parse_cpu_list(b"0-3x"), None);
        assert_eq!(parse_cpu_list(b"a"), None);
    }

    #[test]
    fn online_list_file_is_read_and_trimmed() {
        let path = write_file("cpu_online", "0-5,7\n");
        assert_eq!(count_from_online_list(&path), Some(7));
        let missing = temp_dir("cpu_online_missing").join("nope");
        assert_eq!(count_from_online_list(missing.to_str().unwrap()), None);
    }

    #[test]
    fn proc_stat_fallback_counts_per_cpu_lines() {
        let path = write_file("cpu_stat_count", STAT);
        assert_eq!(count_from_proc_stat(&path), Some(3));
    }

    #[test]
    fn linux_backend_agrees_with_proc_stat_on_this_host() {
        // Only meaningful where procfs exists; elsewhere the count may come
        // from the affinity fallback and ticks fail, which is fine.
        if count_from_proc_stat(PROC_STAT_PATH).is_none() {
            return;
        }
        let cpu = LinuxCpu::new();
        let mut count: FwSizeType = 0;
        assert_eq!(cpu.get_count(&mut count), Status::OpOk);
        let mut ticks = Ticks::default();
        assert_eq!(cpu.get_ticks(&mut ticks, count), Status::Error);
        assert_eq!(cpu.get_ticks(&mut ticks, 0), Status::OpOk);
        assert!(ticks.total >= ticks.used);
    }
}
