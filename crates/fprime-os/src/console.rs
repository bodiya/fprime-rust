//! The F Prime console.
//!
//! Port of `Os::Console` (Os/Console.{hpp,cpp}, Os/Posix/Console.cpp) —
//! see `docs/cpp-analysis/os.md`. The Posix implementation is `fwrite` +
//! `fflush` to stdout (switchable to stderr); the C++ singleton registers
//! itself as the global `Fw::Logger` — here [`init`] (called by
//! [`crate::init`]) registers the static [`CONSOLE`] as the global
//! [`fprime_fw::FwLogger`].
//!
//! C++ parity gotcha: log output emitted before registration is silently
//! dropped, so porting order matters — call `os::init()` early.

use fprime_fw::FwLogger;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};

/// Output stream selection (C++ `setOutputStream`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConsoleStream {
    /// Write to standard output (the C++ default).
    #[default]
    Stdout,
    /// Write to standard error.
    Stderr,
}

/// The F Prime console writer (see module docs).
pub struct Console {
    use_stderr: AtomicBool,
}

impl Console {
    /// Construct a console writing to stdout.
    pub const fn new() -> Self {
        Self {
            use_stderr: AtomicBool::new(false),
        }
    }

    /// Select the output stream (C++ `setOutputStream`).
    pub fn set_output_stream(&self, stream: ConsoleStream) {
        self.use_stderr
            .store(stream == ConsoleStream::Stderr, Ordering::Relaxed);
    }

    /// Write one message and flush (C++ `writeMessage`: fwrite + fflush;
    /// errors are ignored).
    pub fn write(&self, message: &str) {
        if self.use_stderr.load(Ordering::Relaxed) {
            let mut out = std::io::stderr().lock();
            let _ = out.write_all(message.as_bytes());
            let _ = out.flush();
        } else {
            let mut out = std::io::stdout().lock();
            let _ = out.write_all(message.as_bytes());
            let _ = out.flush();
        }
    }
}

impl Default for Console {
    fn default() -> Self {
        Self::new()
    }
}

impl FwLogger for Console {
    fn write_message(&self, message: &str) {
        self.write(message);
    }
}

/// The process-wide console (the C++ singleton).
pub static CONSOLE: Console = Console::new();

/// Register [`CONSOLE`] as the global framework logger (the side effect of
/// the C++ `Console::getSingleton()` first call; invoked by
/// [`crate::init`]).
pub fn init() {
    fprime_fw::logger::register_logger(&CONSOLE);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_to_both_streams_does_not_panic() {
        let console = Console::new();
        console.write("console stdout test\n");
        console.set_output_stream(ConsoleStream::Stderr);
        console.write("console stderr test\n");
        console.set_output_stream(ConsoleStream::Stdout);
    }

    #[test]
    fn init_registers_the_global_logger() {
        // Registration is process-global; just exercise the path and route
        // one message through the fw logger front door.
        crate::init();
        fprime_fw::logger::log_message("os::init logger smoke test\n");
        fprime_fw::logger::deregister_logger();
    }
}
