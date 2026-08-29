//! The F Prime diagnostic text logger.
//!
//! Port of `Fw/Logger/Logger.{hpp,cpp}`: a global registered writer for
//! framework diagnostic text. Messages are silently dropped while no logger
//! is registered. This is distinct from the event (`Fw.Log`) path.

use fprime_config::FW_FIXED_LENGTH_STRING_SIZE;
use std::sync::RwLock;

/// A diagnostic log writer (port of the `Fw::Logger` virtual interface).
pub trait FwLogger: Send + Sync {
    /// Write one formatted diagnostic message.
    fn write_message(&self, message: &str);
}

static LOGGER: RwLock<Option<&'static dyn FwLogger>> = RwLock::new(None);

/// Register the global diagnostic logger (`Fw::Logger::registerLogger`).
/// Expected at init, before tasks start.
pub fn register_logger(logger: &'static dyn FwLogger) {
    let mut guard = LOGGER
        .write()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    *guard = Some(logger);
}

/// Deregister the global diagnostic logger (subsequent messages are dropped).
pub fn deregister_logger() {
    let mut guard = LOGGER
        .write()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    *guard = None;
}

/// Log one diagnostic message; silently dropped when no logger is
/// registered. C++ parity: the message is formatted through a
/// `Fw::String` (256 chars), so it is truncated to
/// `FW_FIXED_LENGTH_STRING_SIZE` bytes here as well.
pub fn log_message(message: &str) {
    let guard = LOGGER
        .read()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    if let Some(logger) = *guard {
        if message.len() > FW_FIXED_LENGTH_STRING_SIZE {
            let mut cut = FW_FIXED_LENGTH_STRING_SIZE;
            while !message.is_char_boundary(cut) {
                cut -= 1;
            }
            logger.write_message(&message[..cut]);
        } else {
            logger.write_message(message);
        }
    }
}

/// Printf-style diagnostic logging (`Fw::Logger::log`), formatted via
/// `format!`. Silently dropped while no logger is registered.
#[macro_export]
macro_rules! fw_log {
    ($($t:tt)*) => {
        $crate::logger::log_message(&format!($($t)*))
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    struct TestLogger {
        messages: Mutex<Vec<String>>,
    }
    impl FwLogger for TestLogger {
        fn write_message(&self, message: &str) {
            self.messages
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .push(message.to_string());
        }
    }

    // Single test: the logger registry is process-global state, so ordering
    // (drop-while-unregistered, then capture) must run as one sequence.
    #[test]
    fn logger_lifecycle() {
        static SINK: TestLogger = TestLogger {
            messages: Mutex::new(Vec::new()),
        };

        // unregistered: silently dropped
        fw_log!("dropped {}", 1);

        register_logger(&SINK);
        fw_log!("value = {}", 42);
        log_message(&"y".repeat(300)); // truncated to 256

        deregister_logger();
        fw_log!("dropped {}", 2);

        let messages = SINK
            .messages
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        assert_eq!(messages.len(), 2);
        assert_eq!(messages[0], "value = 42");
        assert_eq!(messages[1].len(), FW_FIXED_LENGTH_STRING_SIZE);
    }
}
