//! Linux OSAL backends (port of `Os/Linux/`).
//!
//! The C++ Linux backends call `sysconf(3)` and `sysinfo(2)`; with no
//! `unsafe` and no third-party crates those are reached through the procfs /
//! sysfs files the C library itself reads, using [`crate::File`] with fixed
//! stack buffers (no heap allocation per sample). The parsing is plain std
//! code, so the module is compiled on every host (the tests feed it crafted
//! files); only the default-backend selection in [`crate::cpu`] /
//! [`crate::memory`] is `cfg(all(feature = "std", target_os = "linux"))`.

pub mod cpu;
pub mod memory;

use fprime_config::FwSizeType;

/// The C `isspace` set, which `strtoull` skips before a number.
fn is_c_space(byte: u8) -> bool {
    matches!(byte, b' ' | b'\t' | b'\n' | 0x0B | 0x0C | b'\r')
}

/// Parse one unsigned decimal number from the start of `input`, skipping
/// leading C whitespace — the subset of `Fw::StringUtils::string_to_number`
/// (a `strtoull(input, &next, 0)` wrapper) that procfs needs.
///
/// Returns the value and the number of bytes consumed (whitespace
/// included), i.e. the C++ `next` pointer as an offset. `None` stands for
/// every non-`SUCCESSFUL_CONVERSION` status: no digits (`INVALID_NUMBER`)
/// or a value above `FwSizeType::MAX` (`INVALID_RANGE`).
///
/// Deviation: `strtoull`'s base-0 prefix detection (`0x…` hex, leading-`0`
/// octal) and its optional `+`/`-` sign are not reproduced — the kernel
/// prints these fields as plain unsigned decimal without leading zeros, so
/// the results are identical for every well-formed procfs line.
pub(crate) fn string_to_number(input: &[u8]) -> Option<(FwSizeType, usize)> {
    let mut cursor = 0usize;
    while cursor < input.len() && is_c_space(input[cursor]) {
        cursor += 1;
    }
    let digits_start = cursor;
    let mut value: FwSizeType = 0;
    while cursor < input.len() && input[cursor].is_ascii_digit() {
        let digit = FwSizeType::from(input[cursor] - b'0');
        value = value.checked_mul(10)?.checked_add(digit)?;
        cursor += 1;
    }
    if cursor == digits_start {
        return None;
    }
    Some((value, cursor))
}

/// Strip trailing C whitespace (the newline `readline` keeps).
pub(crate) fn trim_end(mut line: &[u8]) -> &[u8] {
    while let [rest @ .., last] = line {
        if !is_c_space(*last) {
            break;
        }
        line = rest;
    }
    line
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn string_to_number_skips_leading_whitespace_and_reports_next() {
        assert_eq!(string_to_number(b"  270288 1598"), Some((270_288, 8)));
        assert_eq!(string_to_number(b"0 1"), Some((0, 1)));
        assert_eq!(string_to_number(b"\t\n42\n"), Some((42, 4)));
    }

    #[test]
    fn string_to_number_rejects_missing_digits_and_overflow() {
        assert_eq!(string_to_number(b""), None);
        assert_eq!(string_to_number(b"   \n"), None);
        assert_eq!(string_to_number(b" abc"), None);
        assert_eq!(
            string_to_number(b"18446744073709551615"),
            Some((u64::MAX, 20))
        );
        assert_eq!(string_to_number(b"18446744073709551616"), None);
    }

    #[test]
    fn trim_end_strips_the_newline() {
        assert_eq!(trim_end(b"0-7\n"), b"0-7");
        assert_eq!(trim_end(b" \r\n"), b"");
        assert_eq!(trim_end(b"x"), b"x");
    }
}
