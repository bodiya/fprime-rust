//! The F Prime assert machinery.
//!
//! Port of `Fw/Types/Assert.{hpp,cpp}` at the default `FW_FILENAME_ASSERT`
//! level: on failure the message `Assert: "file:line" arg1 ... argN` is
//! built (truncated to `FW_ASSERT_TEXT_SIZE`), then dispatched to a
//! registered [`AssertHook`] if any, else printed to stderr followed by a
//! panic (the C++ `assert(false)`).

use fprime_config::{FW_ASSERT_TEXT_SIZE, FwAssertArgType};
use std::sync::RwLock;

/// Swappable assert hook (port of `Fw::AssertHook`). Components like an
/// `AssertFatalAdapter` register one to emit FATAL events instead of the
/// default stderr-print + panic.
///
/// C++ parity: if `do_assert` returns (a hook that neither panics nor
/// aborts), execution continues past the failed `fw_assert!` — exactly as a
/// C++ hook overriding `doAssert` without aborting would.
pub trait AssertHook: Send + Sync {
    /// Report a failed assertion. The default builds the standard message and
    /// forwards it to [`AssertHook::print_assert`].
    fn report_assert(&self, file: &str, line: u32, args: &[FwAssertArgType]) {
        let msg = format_assert_msg(file, line, args);
        self.print_assert(&msg);
    }

    /// Print a formatted assert message. Default: stderr.
    fn print_assert(&self, msg: &str) {
        eprintln!("{msg}");
    }

    /// Take the assert action. Default: panic (the C++ `assert(false)`).
    fn do_assert(&self) {
        panic!("FW_ASSERT failed");
    }
}

// Global hook, installed before threads start (install-before-threads is the
// C++ contract; the RwLock makes late swaps merely slow, not unsound).
static ASSERT_HOOK: RwLock<Option<Box<dyn AssertHook>>> = RwLock::new(None);

/// Install the global assert hook, returning any previously installed hook
/// (the C++ `registerHook` chain, made explicit).
pub fn register_assert_hook(hook: Box<dyn AssertHook>) -> Option<Box<dyn AssertHook>> {
    let mut guard = ASSERT_HOOK
        .write()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    guard.replace(hook)
}

/// Remove and return the global assert hook (`deregisterHook`).
pub fn deregister_assert_hook() -> Option<Box<dyn AssertHook>> {
    let mut guard = ASSERT_HOOK
        .write()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    guard.take()
}

/// Build the standard assert message: `Assert: "file:line" a1 ... aN`,
/// truncated to `FW_ASSERT_TEXT_SIZE - 1` bytes like the C++
/// `CHAR[FW_ASSERT_TEXT_SIZE]` stack buffer.
pub fn format_assert_msg(file: &str, line: u32, args: &[FwAssertArgType]) -> String {
    let mut msg = format!("Assert: \"{file}:{line}\"");
    for arg in args {
        msg.push_str(&format!(" {arg}"));
    }
    if msg.len() >= FW_ASSERT_TEXT_SIZE {
        // truncate on a char boundary at or below the C++ buffer limit
        let mut cut = FW_ASSERT_TEXT_SIZE - 1;
        while !msg.is_char_boundary(cut) {
            cut -= 1;
        }
        msg.truncate(cut);
    }
    msg
}

/// Dispatch a failed assertion (the `Fw::SwAssert` path). Called by
/// [`fw_assert!`](crate::fw_assert); do not call directly except from
/// custom assert plumbing.
pub fn assert_failure(file: &str, line: u32, args: &[FwAssertArgType]) {
    let guard = ASSERT_HOOK
        .read()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    match guard.as_ref() {
        Some(hook) => {
            hook.report_assert(file, line, args);
            hook.do_assert();
        }
        None => {
            let msg = format_assert_msg(file, line, args);
            eprintln!("{msg}");
            panic!("{msg}");
        }
    }
}

/// The F Prime `FW_ASSERT` macro: asserts programmer-error invariants with
/// 0..=6 `FwAssertArgType` (i32) arguments.
///
/// ```should_panic
/// # use fprime_fw::fw_assert;
/// let port = 7i32;
/// fw_assert!(port < 5, port);
/// ```
#[macro_export]
macro_rules! fw_assert {
    ($cond:expr $(,)?) => {{
        // bind first so `!(a >= b)`-style conditions don't trip float lints
        let cond: bool = $cond;
        if !cond {
            $crate::assert::assert_failure(file!(), line!(), &[]);
        }
    }};
    ($cond:expr, $($arg:expr),+ $(,)?) => {{
        let cond: bool = $cond;
        if !cond {
            $crate::assert::assert_failure(
                file!(),
                line!(),
                &[$($arg as $crate::config::FwAssertArgType),+],
            );
        }
    }};
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    // Serializes the tests that mutate the global hook against each other.
    static HOOK_TEST_LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn format_matches_cpp_layout() {
        assert_eq!(format_assert_msg("f.rs", 42, &[]), "Assert: \"f.rs:42\"");
        assert_eq!(
            format_assert_msg("dir/f.rs", 7, &[1, -2, 3]),
            "Assert: \"dir/f.rs:7\" 1 -2 3"
        );
    }

    #[test]
    fn format_truncates_at_text_size() {
        let long_file = "x".repeat(400);
        let msg = format_assert_msg(&long_file, 1, &[123456]);
        assert_eq!(msg.len(), FW_ASSERT_TEXT_SIZE - 1);
    }

    #[test]
    fn default_path_panics_with_message() {
        let _guard = HOOK_TEST_LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let result = std::panic::catch_unwind(|| {
            fw_assert!(1 == 2, 9, 8);
        });
        let err = result.expect_err("fw_assert must panic without a hook");
        let text = err.downcast_ref::<String>().cloned().unwrap_or_else(|| {
            err.downcast_ref::<&str>()
                .map(|s| (*s).to_string())
                .unwrap()
        });
        assert!(text.contains("Assert: \""), "got: {text}");
        assert!(text.contains(" 9 8"), "got: {text}");
    }

    type SeenAsserts = Arc<Mutex<Vec<(String, u32, Vec<i32>)>>>;

    struct CaptureHook {
        seen: SeenAsserts,
    }
    impl AssertHook for CaptureHook {
        fn report_assert(&self, file: &str, line: u32, args: &[FwAssertArgType]) {
            self.seen
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .push((file.to_string(), line, args.to_vec()));
        }
        // keep the default panicking do_assert so concurrent should_panic
        // tests still observe a panic while this hook is installed
    }

    #[test]
    fn hook_receives_file_line_and_args() {
        let _guard = HOOK_TEST_LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let seen = Arc::new(Mutex::new(Vec::new()));
        let prev = register_assert_hook(Box::new(CaptureHook { seen: seen.clone() }));
        assert!(prev.is_none());

        let result = std::panic::catch_unwind(|| {
            fw_assert!(false, 1, 2, 3, 4, 5, 6);
        });
        assert!(result.is_err(), "default do_assert still panics");

        let hook = deregister_assert_hook();
        assert!(hook.is_some());

        // Other tests' should_panic asserts may fire while the hook is
        // installed (they still panic via the default do_assert), so check
        // containment of our record rather than exclusivity.
        let seen = seen
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let ours: Vec<_> = seen
            .iter()
            .filter(|(file, _, args)| {
                file.ends_with("assert.rs") && args == &vec![1, 2, 3, 4, 5, 6]
            })
            .collect();
        assert_eq!(
            ours.len(),
            1,
            "hook must have captured our assert: {seen:?}"
        );
    }
}
