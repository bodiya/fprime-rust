//! A `std::thread` / `SystemTime` implementation of the `no_std` OSAL seam
//! (`fprime_os::platform::Platform`) for this crate's tests (ROADMAP item
//! 13). The tests run on a hosted target even when the crate is built
//! without `std` (`--no-default-features --features spin`); there,
//! [`install`] registers this platform so active components start their
//! task through the seam. In the default `std` build it does nothing.
//!
//! Shared by `src/lib.rs` (unit tests, via `#[path]`) and the integration
//! tests (`mod no_std_platform;`), so it names only `std::` and
//! `fprime_os::` paths.

/// Register the test platform (idempotent; a no-op with `std`).
pub fn install() {
    #[cfg(not(feature = "std"))]
    let _ = fprime_os::register_platform(&imp::StdThreads);
}

#[cfg(not(feature = "std"))]
mod imp {
    use fprime_fw::TimeInterval;
    use fprime_os::platform::{Platform, TaskHandle};
    use fprime_os::rawtime::{self, RawTime};
    use fprime_os::task::{self, Arguments};
    use std::boxed::Box;

    pub struct StdThreads;

    struct Handle(std::thread::JoinHandle<()>);

    impl TaskHandle for Handle {
        fn join(self: Box<Self>) -> task::Status {
            match self.0.join() {
                Ok(()) => task::Status::OpOk,
                Err(_) => task::Status::JoinError,
            }
        }
    }

    impl Platform for StdThreads {
        fn start_task(&self, arguments: Arguments) -> Result<Box<dyn TaskHandle>, task::Status> {
            std::thread::Builder::new()
                .name(arguments.name)
                .spawn(arguments.routine)
                .map(|handle| Box::new(Handle(handle)) as Box<dyn TaskHandle>)
                .map_err(|_| task::Status::ErrorResources)
        }

        fn delay(&self, interval: TimeInterval) -> task::Status {
            std::thread::sleep(std::time::Duration::new(
                u64::from(interval.get_seconds()),
                interval.get_useconds() * 1000,
            ));
            task::Status::OpOk
        }

        fn raw_time_now(&self) -> Result<RawTime, rawtime::Status> {
            let now = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_err(|_| rawtime::Status::OtherError)?;
            Ok(RawTime::from_parts(now.as_secs(), now.subsec_nanos()))
        }

        fn queue_wait(&self) {
            std::thread::yield_now();
        }
    }
}
