//! # LinuxGpioDriver — port of `Drv::LinuxGpioDriver` (+ `Drv/Interfaces/Gpio`,
//! `Drv/Ports/GpioDriverPorts`)
//!
//! C++ sources: `Drv/LinuxGpioDriver/LinuxGpioDriver.{fpp,hpp,cpp}`,
//! `LinuxGpioDriverCommon.cpp`, `LinuxGpioDriverStub.cpp`,
//! `Drv/Ports/GpioDriverPorts.fpp`, `Drv/Interfaces/Gpio.fpp`.
//! Analysis: `docs/cpp-analysis/linux-drivers.md`.
//!
//! PASSIVE component implementing the `Drv.Gpio` interface: sync `gpioWrite`
//! and `gpioRead` inputs, an `Svc.Cycle` `gpioInterrupt` output driven by a
//! dedicated poll thread, plus the `Log`/`LogText`/`Time` special ports (no
//! telemetry, no commands — exactly as the FPP declares).
//!
//! ## Deviation: no character-device (`/dev/gpiochip*`) backend
//!
//! **Every** operation of the C++ driver goes through `ioctl(2)`
//! (`GPIO_V2_GET_LINE_IOCTL`, `GPIO_V2_LINE_GET/SET_VALUES_IOCTL`, the v1
//! `GPIOHANDLE_*`/`GPIOEVENT_*` requests) and its interrupt wait uses
//! `poll(2)`. Neither is reachable from safe, zero-dependency `std` Rust —
//! they need `libc`/`nix` or an `unsafe extern` shim, both excluded by this
//! workspace's rules (`#![forbid(unsafe_code)]`, no third-party crates).
//!
//! The component surface is therefore ported in full and parameterised over a
//! [`GpioBackend`] trait, so a downstream project can drop in a real
//! character-device implementation without forking this component. Two
//! backends ship here:
//!
//! - [`SysfsGpioBackend`] — the legacy `/sys/class/gpio` interface (plain file
//!   I/O: `export`, `gpio<N>/direction`, `gpio<N>/value`, `gpio<N>/edge`).
//!   Input and output are faithful. **Interrupts are LEVEL SAMPLED, not
//!   kernel edge interrupts** — see the type's documentation for the
//!   consequences (missed pulses, latency, no hardware timestamp).
//! - [`StubGpioBackend`] — mirrors `LinuxGpioDriverStub.cpp`
//!   (`open` → `NOT_SUPPORTED`, both handlers → `UNKNOWN_ERROR`, poll loop
//!   just delays). It is the default on non-Linux targets.
//!
//! ## Other documented divergences
//!
//! - The C++ `ApiVersion` member (v2 vs deprecated v1 uAPI selection) has no
//!   meaning without ioctl and is not ported.
//! - The kernel consumer label (object name truncated to 32 bytes) is passed
//!   to [`GpioBackend::open`] for backends that can use it; sysfs has no
//!   consumer concept and ignores it.
//! - The C++ stub's `open()` returns `NOT_SUPPORTED` *without* logging;
//!   here the event path lives in the component, so [`StubGpioBackend`]
//!   causes a `OpenChipError(device, NOT_SUPPORTED)` event. The returned
//!   status is identical.

use fprime_comp::{CyclePort, EventGlue, OutputPort, PassiveBase};
use fprime_config::{FwEventIdType, FwIdType, FwIndexType, FwSizeType, FwTaskPriorityType};
use fprime_fw::{LogSeverity, LogStringArg, SerBuf, fpp_enum, fw_try};
use fprime_os::file::Status as FileStatus;
use fprime_os::rawtime::Status as RawTimeStatus;
use fprime_os::task::{Arguments, Status as TaskStatus};
use fprime_os::{RawTime, Task};
use std::io;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Event string arguments in `LinuxGpioDriver.fpp` are declared as bare
/// `string`, which is FPP's default size of 80.
const EVENT_STRING_SIZE: usize = 80;

fpp_enum! {
    /// `Fw::Logic` (`Fw/Types/Types.fpp`) — a GPIO line level.
    ///
    /// Declared here rather than in `fprime-fw` because this port's
    /// `fprime-fw` crate predates the driver work; it is byte-identical to
    /// the framework enum (U8 representation, `LOW = 0`, `HIGH = 1`) and can
    /// be moved without a wire change.
    pub enum Logic : u8 {
        /// Logic low (0 V / de-asserted).
        Low = 0,
        /// Logic high (asserted).
        High = 1,
    }
    default Low
}

fpp_enum! {
    /// `Drv::GpioStatus` — FPP enum, `repr U8`
    /// (`Drv/Ports/GpioDriverPorts.fpp`, implicit sequential values).
    pub enum GpioStatus : u8 {
        /// Operation succeeded.
        OpOk = 0,
        /// Pin was never opened.
        ///
        /// Gotcha (C++ parity): only ever produced by
        /// [`errno_to_gpio_status`] on `EBADF`. A driver that was never
        /// opened returns [`GpioStatus::InvalidMode`] instead, because the
        /// configuration gate fires first.
        NotOpened = 1,
        /// Operation not permitted with the current configuration.
        InvalidMode = 2,
        /// An unknown error occurred.
        UnknownError = 3,
    }
    default OpOk
}

/// `Drv.GpioWrite` port: `GpioWrite(state: Fw.Logic) -> GpioStatus`.
pub trait GpioWritePort: Send + Sync {
    /// Drive the line to `state`.
    fn invoke(&self, port_num: FwIndexType, state: Logic) -> GpioStatus;
}

/// `Drv.GpioRead` port: `GpioRead(ref state: Fw.Logic) -> GpioStatus`.
///
/// `state` is a C++ `ref` out-parameter and stays one here, so the port shape
/// matches the FPP signature exactly.
pub trait GpioReadPort: Send + Sync {
    /// Sample the line into `state`.
    fn invoke(&self, port_num: FwIndexType, state: &mut Logic) -> GpioStatus;
}

/// C++ `LinuxGpioDriver::GpioConfiguration`: the pin mode chosen at
/// [`LinuxGpioDriver::open`]. Only one mode may be selected at a time.
///
/// The C++ `MAX_GPIO_CONFIGURATION` sentinel (also the "never opened"
/// default) is represented by `None` in the component state — the enum here
/// carries only real modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GpioConfiguration {
    /// Output pin for direct writing (`gpioWrite` only).
    GpioOutput,
    /// Input pin for direct reading (`gpioRead` only).
    GpioInput,
    /// Input pin firing `gpioInterrupt` on a low → high transition.
    GpioInterruptRisingEdge,
    /// Input pin firing `gpioInterrupt` on a high → low transition.
    GpioInterruptFallingEdge,
    /// Input pin firing `gpioInterrupt` on both transitions.
    GpioInterruptBothRisingAndFallingEdges,
}

impl GpioConfiguration {
    /// True for the three interrupt modes — the C++
    /// `GPIO_INTERRUPT_RISING_EDGE <= m_configuration < MAX` test that gates
    /// [`LinuxGpioDriver::start`].
    #[must_use]
    pub const fn is_interrupt(self) -> bool {
        matches!(
            self,
            Self::GpioInterruptRisingEdge
                | Self::GpioInterruptFallingEdge
                | Self::GpioInterruptBothRisingAndFallingEdges
        )
    }

    /// Does a `from` → `to` transition fire an interrupt in this mode?
    /// Non-interrupt modes never do.
    #[must_use]
    pub const fn edge_fires(self, from: Logic, to: Logic) -> bool {
        match self {
            Self::GpioInterruptRisingEdge => matches!((from, to), (Logic::Low, Logic::High)),
            Self::GpioInterruptFallingEdge => matches!((from, to), (Logic::High, Logic::Low)),
            Self::GpioInterruptBothRisingAndFallingEdges => !matches!(
                (from, to),
                (Logic::Low, Logic::Low) | (Logic::High, Logic::High)
            ),
            Self::GpioOutput | Self::GpioInput => false,
        }
    }

    /// The `/sys/class/gpio/gpio<N>/edge` keyword for this mode
    /// (`"none"` for the non-interrupt modes).
    #[must_use]
    pub const fn sysfs_edge(self) -> &'static str {
        match self {
            Self::GpioOutput | Self::GpioInput => "none",
            Self::GpioInterruptRisingEdge => "rising",
            Self::GpioInterruptFallingEdge => "falling",
            Self::GpioInterruptBothRisingAndFallingEdges => "both",
        }
    }
}

// ---------------------------------------------------------------------------
// errno mapping (C++ errno_to_file_status / errno_to_gpio_status)
// ---------------------------------------------------------------------------

/// C++ `errno_to_file_status`: `0 → OP_OK`, `EBADF(9) → NOT_OPENED`,
/// `EINVAL(22) → INVALID_ARGUMENT`, `ENODEV(19) → DOESNT_EXIST`,
/// `ENOMEM(12) → NO_SPACE`, `EPERM(1) → NO_PERMISSION`,
/// `ENXIO(6) → INVALID_MODE`, everything else → `OTHER_ERROR`.
///
/// `std::io::Error::raw_os_error()` exposes the real errno, so the table
/// ports exactly; an error without an errno (a synthetic `io::Error`) maps to
/// `OTHER_ERROR`.
pub fn errno_to_file_status(error: &io::Error) -> FileStatus {
    match error.raw_os_error() {
        Some(0) => FileStatus::OpOk,
        Some(9) => FileStatus::NotOpened,
        Some(22) => FileStatus::InvalidArgument,
        Some(19) => FileStatus::DoesntExist,
        Some(12) => FileStatus::NoSpace,
        Some(1) => FileStatus::NoPermission,
        Some(6) => FileStatus::InvalidMode,
        _ => FileStatus::OtherError,
    }
}

/// C++ `errno_to_gpio_status`: `EBADF(9) → NOT_OPENED`,
/// `ENXIO(6) → INVALID_MODE`, everything else → `UNKNOWN_ERROR`.
#[must_use]
pub fn errno_to_gpio_status(error: &io::Error) -> GpioStatus {
    match error.raw_os_error() {
        Some(9) => GpioStatus::NotOpened,
        Some(6) => GpioStatus::InvalidMode,
        _ => GpioStatus::UnknownError,
    }
}

// ---------------------------------------------------------------------------
// Backend seam
// ---------------------------------------------------------------------------

/// Chip/pin identification returned by a successful [`GpioBackend::open`],
/// feeding the DIAGNOSTIC `OpenChip` event.
///
/// Mirrors the C++ `struct gpiochip_info` fields (`name`, `label`) plus the
/// line-info derived `pinMessage` (`"Unknown"` when the backend cannot
/// report one, exactly like the C++ default).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpioChipInfo {
    /// Chip name (C++ `gpiochip_info.name`, e.g. `"gpiochip0"`).
    pub name: String,
    /// Chip label (C++ `gpiochip_info.label`).
    pub label: String,
    /// Line description (C++ `pin_message`; `"Unknown"` when unavailable).
    pub pin_message: String,
}

impl GpioChipInfo {
    /// Build chip info with the C++ default pin message (`"Unknown"`).
    #[must_use]
    pub fn new(name: &str, label: &str) -> Self {
        Self {
            name: name.to_string(),
            label: label.to_string(),
            pin_message: "Unknown".to_string(),
        }
    }
}

/// Why [`GpioBackend::open`] failed — selects which of the two C++ open
/// events the component emits.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GpioOpenError {
    /// The chip itself could not be opened/queried:
    /// `OpenChipError(device, status)` (WARNING_HI).
    Chip(FileStatus),
    /// The chip opened but the line could not be requested:
    /// `OpenPinError(device, pin, pin_message, status)` (WARNING_HI).
    Pin {
        /// Line description for the event (C++ `pin_message`).
        pin_message: String,
        /// Failure status.
        status: FileStatus,
    },
}

impl GpioOpenError {
    /// The `Os::File::Status` the component returns from `open()`.
    pub const fn status(&self) -> FileStatus {
        match self {
            Self::Chip(status) => *status,
            Self::Pin { status, .. } => *status,
        }
    }
}

/// One iteration of the interrupt wait — the safe-Rust stand-in for the C++
/// `poll(2)` + `read(2)` pair in `pollLoop()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PollOutcome {
    /// A configured transition was observed: the component timestamps it
    /// with a fresh [`RawTime`] and invokes `gpioInterrupt_out(0, ts)`
    /// (C++ discards the kernel event payload too).
    Interrupt,
    /// The wait expired with nothing to report (C++ `poll()` returning 0).
    NoInterrupt,
    /// The event read came up short (C++ `read()` != `sizeof(event)`):
    /// `InterruptReadError(expected, got)`.
    ///
    /// C++ casts a `ssize_t` of `-1` through `FwSizeType` and then to `U32`,
    /// so a hard read error reports `got == 0xFFFF_FFFF`.
    ReadError {
        /// Bytes the driver expected to read.
        expected: u32,
        /// Bytes actually read.
        got: u32,
    },
    /// The wait itself failed (C++ `poll()` returning < 0):
    /// `PollingError(errno)`.
    PollError(i32),
}

/// Pluggable GPIO hardware access.
///
/// Implement this (with `libc`/`nix` or an `unsafe` FFI shim, in a crate that
/// permits it) to give [`LinuxGpioDriver`] real character-device access; the
/// component, its ports, events and threading are unchanged.
pub trait GpioBackend: Send + Sync {
    /// C++ `open()` minus the event emission: acquire `gpio` on `device` in
    /// `configuration`, driving outputs to `default_state`.
    ///
    /// `consumer` is the kernel consumer label (the component's object name;
    /// the C++ driver truncates it to 32 bytes). Backends without a consumer
    /// concept ignore it.
    fn open(
        &self,
        device: &str,
        gpio: u32,
        configuration: GpioConfiguration,
        default_state: Logic,
        consumer: &str,
    ) -> Result<GpioChipInfo, GpioOpenError>;

    /// C++ `gpioRead_handler` device access (the mode gate lives in the
    /// component).
    fn read(&self) -> Result<Logic, GpioStatus>;

    /// C++ `gpioWrite_handler` device access (the mode gate lives in the
    /// component).
    fn write(&self, state: Logic) -> GpioStatus;

    /// One interrupt wait of at most `timeout`. Implementations MUST return
    /// within roughly `timeout` so [`LinuxGpioDriver::stop`] bounds shutdown
    /// latency, exactly like the C++ 500 ms `poll()` timeout.
    fn poll(&self, timeout: Duration) -> PollOutcome;

    /// C++ destructor's `close(m_fd)`. Default: nothing.
    fn close(&self) {}
}

// ---------------------------------------------------------------------------
// Stub backend (LinuxGpioDriverStub.cpp)
// ---------------------------------------------------------------------------

/// Port of `LinuxGpioDriverStub.cpp`: `open()` → `NOT_SUPPORTED`, both
/// handlers → `UNKNOWN_ERROR`, poll loop just delays. The default backend on
/// non-Linux targets.
#[derive(Debug, Default, Clone, Copy)]
pub struct StubGpioBackend;

impl StubGpioBackend {
    /// Construct the stub backend.
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

impl GpioBackend for StubGpioBackend {
    fn open(
        &self,
        _device: &str,
        _gpio: u32,
        _configuration: GpioConfiguration,
        _default_state: Logic,
        _consumer: &str,
    ) -> Result<GpioChipInfo, GpioOpenError> {
        Err(GpioOpenError::Chip(FileStatus::NotSupported))
    }

    fn read(&self) -> Result<Logic, GpioStatus> {
        Err(GpioStatus::UnknownError)
    }

    fn write(&self, _state: Logic) -> GpioStatus {
        GpioStatus::UnknownError
    }

    fn poll(&self, timeout: Duration) -> PollOutcome {
        // C++ stub: Os::Task::delay(GPIO_POLL_TIMEOUT) per iteration.
        std::thread::sleep(timeout);
        PollOutcome::NoInterrupt
    }
}

// ---------------------------------------------------------------------------
// sysfs backend
// ---------------------------------------------------------------------------

/// Default sampling period of [`SysfsGpioBackend`]'s pseudo-interrupt thread.
pub const DEFAULT_SAMPLE_INTERVAL: Duration = Duration::from_millis(10);

/// Root of the legacy sysfs GPIO interface.
pub const SYSFS_GPIO_ROOT: &str = "/sys/class/gpio";

/// Legacy `/sys/class/gpio` backend — pure `std::fs`, no ioctl.
///
/// # What is faithful
///
/// `GpioOutput` and `GpioInput` behave exactly as the character-device driver
/// does: `open()` exports the line, sets `direction` (`"low"`/`"high"` for an
/// output, which applies `default_state` atomically with the direction
/// change, `"in"` for an input), and `read`/`write` go through
/// `gpio<N>/value`. The value file is re-opened for every access, so the
/// "sysfs value files do not auto-rewind" trap cannot bite.
///
/// # What is NOT: interrupts are SAMPLED, not edge-triggered
///
/// Kernel edge interrupts on sysfs require `poll(2)` on `gpio<N>/value` with
/// `POLLPRI`, and `std` exposes no `poll`/`select`/`epoll`. This backend
/// instead **re-reads the value file every sample interval** (default
/// [`DEFAULT_SAMPLE_INTERVAL`]) and reports a transition when the sampled
/// level differs from the previous sample in the configured direction.
/// Consequences, all of them real:
///
/// - **Pulses shorter than the sample interval are missed entirely.**
/// - Interrupt latency is bounded by the sample interval, not by the
///   hardware; jitter is whatever the scheduler gives.
/// - Two edges inside one interval collapse into at most one report (or
///   none, if the level returns to where it started).
/// - The timestamp handed to `gpioInterrupt` is the sampling thread's
///   [`RawTime::now`], never a hardware timestamp. (The C++ driver also
///   discards the kernel's `timestamp_ns`, so this part matches.)
/// - The `edge` attribute is still written (best effort, failures ignored)
///   so the sysfs state reflects the requested configuration, but nothing
///   here depends on it.
///
/// Do not use this for anything where a missed or late edge matters. Supply
/// a real [`GpioBackend`] instead.
///
/// # Chip resolution
///
/// The component's API is the character-device one (`/dev/gpiochipN` + a line
/// offset). sysfs numbers lines globally, so `open()` maps `gpiochipN` to the
/// N-th `<root>/gpiochip*` directory ordered by its `base` value, then uses
/// `base + line`. `ngpio` bounds the line offset. If the root or the chip
/// directory is missing (no `CONFIG_GPIO_SYSFS`, non-Linux), `open()` returns
/// [`FileStatus::NotSupported`]. The index-by-base mapping is a heuristic:
/// it holds on the usual kernel where chips are numbered in registration
/// order, and is the only mapping available from plain file reads.
pub struct SysfsGpioBackend {
    root: PathBuf,
    sample_interval: Duration,
    state: Mutex<SysfsState>,
}

#[derive(Default)]
struct SysfsState {
    /// Global sysfs line number (`base + offset`), once exported.
    line: Option<u32>,
    /// Mode captured at open (drives edge detection).
    configuration: Option<GpioConfiguration>,
    /// Previous sample; `None` until the first poll iteration.
    last_sample: Option<Logic>,
}

impl Default for SysfsGpioBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl SysfsGpioBackend {
    /// Backend rooted at [`SYSFS_GPIO_ROOT`] with the default sample period.
    #[must_use]
    pub fn new() -> Self {
        Self::with_root(SYSFS_GPIO_ROOT)
    }

    /// Backend rooted at an arbitrary directory laid out like
    /// `/sys/class/gpio` (used by tests and by unusual sysfs mounts).
    #[must_use]
    pub fn with_root<P: AsRef<Path>>(root: P) -> Self {
        Self {
            root: root.as_ref().to_path_buf(),
            sample_interval: DEFAULT_SAMPLE_INTERVAL,
            state: Mutex::new(SysfsState::default()),
        }
    }

    /// Set the pseudo-interrupt sampling period. Shorter = fewer missed
    /// pulses and lower latency, at a linear cost in syscalls.
    #[must_use]
    pub fn with_sample_interval(mut self, interval: Duration) -> Self {
        self.sample_interval = interval;
        self
    }

    /// The sampling period in use.
    #[must_use]
    pub fn sample_interval(&self) -> Duration {
        self.sample_interval
    }

    /// The global sysfs line number this backend exported, if open.
    #[must_use]
    pub fn line(&self) -> Option<u32> {
        self.state.lock().unwrap().line
    }

    fn line_dir(&self, line: u32) -> PathBuf {
        self.root.join(format!("gpio{line}"))
    }

    /// Read a small sysfs attribute file, trimmed.
    fn read_attribute(path: &Path) -> io::Result<String> {
        Ok(std::fs::read_to_string(path)?.trim().to_string())
    }

    /// Resolve `/dev/gpiochipN` + `offset` to a global sysfs line number,
    /// returning the chip directory name and label alongside it.
    fn resolve_line(
        &self,
        device: &str,
        offset: u32,
    ) -> Result<(u32, String, String), GpioOpenError> {
        let chip_name = Path::new(device)
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("");
        let index: u32 = match chip_name.strip_prefix("gpiochip").map(str::parse::<u32>) {
            Some(Ok(index)) => index,
            // Not a /dev/gpiochipN path: the C++ driver would fail its
            // chip-info ioctl on such a device.
            _ => return Err(GpioOpenError::Chip(FileStatus::InvalidArgument)),
        };

        let entries = match std::fs::read_dir(&self.root) {
            Ok(entries) => entries,
            // No sysfs GPIO interface at all (non-Linux, or the kernel was
            // built without CONFIG_GPIO_SYSFS).
            Err(_) => return Err(GpioOpenError::Chip(FileStatus::NotSupported)),
        };

        // Collect (base, ngpio, dir name, label) for every chip, ordered by
        // base — the only ordering plain file reads can establish.
        let mut chips: Vec<(u32, u32, String, String)> = Vec::new();
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().to_string();
            if !name.starts_with("gpiochip") {
                continue;
            }
            let dir = entry.path();
            let base = Self::read_attribute(&dir.join("base"))
                .ok()
                .and_then(|v| v.parse::<u32>().ok());
            let ngpio = Self::read_attribute(&dir.join("ngpio"))
                .ok()
                .and_then(|v| v.parse::<u32>().ok());
            let label = Self::read_attribute(&dir.join("label")).unwrap_or_default();
            if let (Some(base), Some(ngpio)) = (base, ngpio) {
                chips.push((base, ngpio, name, label));
            }
        }
        chips.sort_by_key(|chip| chip.0);

        let chip = match chips.get(index as usize) {
            Some(chip) => chip,
            None => return Err(GpioOpenError::Chip(FileStatus::NotSupported)),
        };
        if offset >= chip.1 {
            // C++ parity: a line beyond the chip's line count is
            // DOESNT_EXIST with pin message "Does Not Exist" (the SDD claims
            // OP_OK here; the code does not).
            return Err(GpioOpenError::Pin {
                pin_message: "Does Not Exist".to_string(),
                status: FileStatus::DoesntExist,
            });
        }
        Ok((chip.0 + offset, chip.2.clone(), chip.3.clone()))
    }

    fn export(&self, line: u32) -> Result<(), GpioOpenError> {
        let dir = self.line_dir(line);
        if dir.exists() {
            // Already exported (EBUSY on a second export) — the C++ driver
            // likewise tolerates a line that is already usable.
            return Ok(());
        }
        match std::fs::write(self.root.join("export"), format!("{line}")) {
            Ok(()) => Ok(()),
            Err(error) => Err(GpioOpenError::Pin {
                pin_message: "export".to_string(),
                status: errno_to_file_status(&error),
            }),
        }
    }

    fn write_line_attribute(&self, line: u32, attribute: &str, value: &str) -> io::Result<()> {
        std::fs::write(self.line_dir(line).join(attribute), value)
    }

    fn read_value(&self, line: u32) -> Result<Logic, GpioStatus> {
        let path = self.line_dir(line).join("value");
        // Re-open per read: sysfs value files do not rewind on their own.
        match Self::read_attribute(&path) {
            Ok(text) => match text.as_str() {
                "0" => Ok(Logic::Low),
                "1" => Ok(Logic::High),
                _ => Err(GpioStatus::UnknownError),
            },
            Err(error) => Err(errno_to_gpio_status(&error)),
        }
    }
}

impl GpioBackend for SysfsGpioBackend {
    fn open(
        &self,
        device: &str,
        gpio: u32,
        configuration: GpioConfiguration,
        default_state: Logic,
        _consumer: &str,
    ) -> Result<GpioChipInfo, GpioOpenError> {
        let (line, chip_name, chip_label) = self.resolve_line(device, gpio)?;
        self.export(line)?;

        // "low"/"high" set direction AND initial value in one write, which is
        // how sysfs applies the C++ default_state without a glitch.
        let direction = match (configuration, default_state) {
            (GpioConfiguration::GpioOutput, Logic::Low) => "low",
            (GpioConfiguration::GpioOutput, Logic::High) => "high",
            _ => "in",
        };
        if let Err(error) = self.write_line_attribute(line, "direction", direction) {
            return Err(GpioOpenError::Pin {
                pin_message: "direction".to_string(),
                status: errno_to_file_status(&error),
            });
        }
        // Best effort: the sampling poller does not need it, and lines whose
        // controller cannot do edges would otherwise fail to open.
        let _ = self.write_line_attribute(line, "edge", configuration.sysfs_edge());

        let mut state = self.state.lock().unwrap();
        state.line = Some(line);
        state.configuration = Some(configuration);
        state.last_sample = None;
        Ok(GpioChipInfo::new(&chip_name, &chip_label))
    }

    fn read(&self) -> Result<Logic, GpioStatus> {
        let line = self.state.lock().unwrap().line;
        match line {
            Some(line) => self.read_value(line),
            None => Err(GpioStatus::NotOpened),
        }
    }

    fn write(&self, state: Logic) -> GpioStatus {
        let line = self.state.lock().unwrap().line;
        let Some(line) = line else {
            return GpioStatus::NotOpened;
        };
        let value = match state {
            Logic::Low => "0",
            Logic::High => "1",
        };
        match self.write_line_attribute(line, "value", value) {
            Ok(()) => GpioStatus::OpOk,
            Err(error) => errno_to_gpio_status(&error),
        }
    }

    fn poll(&self, timeout: Duration) -> PollOutcome {
        // Sleep first, exactly like poll(2) blocks first: bounded by the
        // caller's timeout so stop()/join() stay responsive.
        std::thread::sleep(self.sample_interval.min(timeout));
        let (line, configuration, last) = {
            let state = self.state.lock().unwrap();
            (state.line, state.configuration, state.last_sample)
        };
        let (Some(line), Some(configuration)) = (line, configuration) else {
            return PollOutcome::NoInterrupt;
        };
        let sample = match self.read_value(line) {
            Ok(sample) => sample,
            // A failed sample is the sampling analogue of poll() < 0; report
            // the errno the way the C++ PollingError event does.
            Err(_) => {
                let errno = std::fs::File::open(self.line_dir(line).join("value"))
                    .err()
                    .and_then(|e| e.raw_os_error())
                    .unwrap_or(0);
                return PollOutcome::PollError(errno);
            }
        };
        self.state.lock().unwrap().last_sample = Some(sample);
        match last {
            Some(previous) if configuration.edge_fires(previous, sample) => PollOutcome::Interrupt,
            _ => PollOutcome::NoInterrupt,
        }
    }

    fn close(&self) {
        let line = self.state.lock().unwrap().line.take();
        if let Some(line) = line {
            let _ = std::fs::write(self.root.join("unexport"), format!("{line}"));
        }
    }
}

impl Drop for SysfsGpioBackend {
    fn drop(&mut self) {
        // C++ parity: the destructor releases the line.
        self.close();
    }
}

/// The default backend for this build: [`SysfsGpioBackend`] on Linux,
/// [`StubGpioBackend`] everywhere else (mirroring the C++ CMake selection of
/// `LinuxGpioDriverStub.cpp` on non-Linux platforms).
#[must_use]
pub fn default_backend() -> Box<dyn GpioBackend> {
    #[cfg(target_os = "linux")]
    {
        Box::new(SysfsGpioBackend::new())
    }
    #[cfg(not(target_os = "linux"))]
    {
        Box::new(StubGpioBackend::new())
    }
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

/// Guarded component state (C++ `m_configuration` + `m_running` under
/// `m_lock`; the fd lives in the backend).
#[derive(Default)]
struct GpioState {
    /// `None` == C++ `MAX_GPIO_CONFIGURATION` (never opened).
    configuration: Option<GpioConfiguration>,
    /// C++ `m_running`, read by the poll loop through `getRunning()`.
    running: bool,
}

/// `Drv::LinuxGpioDriver` — passive GPIO driver over a swappable
/// [`GpioBackend`].
pub struct LinuxGpioDriver {
    /// Passive core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (`Log`/`LogText`) + `Time`.
    pub evt: EventGlue,
    /// `gpioInterrupt` — `Svc.Cycle` output, invoked from the poll thread.
    /// Wire it to an ASYNC input (e.g. `Svc::ActiveRateGroup::CycleIn`) or a
    /// thread-safe handler: it does NOT run on the caller's thread.
    pub gpio_interrupt_out: OutputPort<dyn CyclePort>,
    backend: Box<dyn GpioBackend>,
    state: Mutex<GpioState>,
    poller: Task,
}

impl LinuxGpioDriver {
    /// `OpenChip(chip: string, chipLabel: string, pin: U32, pinMessage: string)`
    /// — DIAGNOSTIC, id 0.
    pub const EVENTID_OPEN_CHIP: FwEventIdType = 0;
    /// `OpenChipError(chip: string, status: Os.FileStatus)` — WARNING_HI, id 1.
    pub const EVENTID_OPEN_CHIP_ERROR: FwEventIdType = 1;
    /// `OpenPinError(chip: string, pin: U32, pinMessage: string, status: Os.FileStatus)`
    /// — WARNING_HI, id 2.
    pub const EVENTID_OPEN_PIN_ERROR: FwEventIdType = 2;
    /// `InterruptReadError(expected: U32, got: U32)` — WARNING_HI, id 3.
    /// (Upstream's format string really does say "byes".)
    pub const EVENTID_INTERRUPT_READ_ERROR: FwEventIdType = 3;
    /// `PollingError(error_number: I32)` — WARNING_HI, id 4.
    pub const EVENTID_POLLING_ERROR: FwEventIdType = 4;
    /// `InterruptTimeError(status: Os.RawTimeStatus)` — WARNING_HI, id 5.
    pub const EVENTID_INTERRUPT_TIME_ERROR: FwEventIdType = 5;

    /// C++ `GPIO_POLL_TIMEOUT` (ms): the interrupt wait timeout, and hence
    /// the upper bound on shutdown latency after [`LinuxGpioDriver::stop`].
    pub const GPIO_POLL_TIMEOUT: u64 = 500;

    /// The kernel consumer label is the object name truncated to 32 bytes
    /// (C++ `consumer_label[32]`).
    pub const CONSUMER_LABEL_SIZE: usize = 32;

    /// Construct with the platform default backend
    /// ([`default_backend`]).
    pub fn new(name: &str) -> Arc<Self> {
        Self::with_backend(name, default_backend())
    }

    /// Construct with an explicit backend (a real character-device driver, a
    /// differently-rooted [`SysfsGpioBackend`], or a test fake).
    pub fn with_backend(name: &str, backend: Box<dyn GpioBackend>) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            gpio_interrupt_out: OutputPort::new(),
            backend,
            state: Mutex::new(GpioState::default()),
            poller: Task::new(),
        })
    }

    fn id_base(&self) -> FwIdType {
        self.base.get_id_base()
    }

    /// The consumer label handed to the backend: object name truncated to
    /// [`Self::CONSUMER_LABEL_SIZE`] bytes (C++ parity).
    fn consumer_label(&self) -> String {
        let name = self.base.get_obj_name();
        let text = name.as_str().unwrap_or_default();
        let mut end = Self::CONSUMER_LABEL_SIZE.min(text.len());
        while end > 0 && !text.is_char_boundary(end) {
            end -= 1;
        }
        text[..end].to_string()
    }

    /// C++ `open(device, gpio, configuration, default_state)`.
    ///
    /// Emits `OpenChipError` / `OpenPinError` (WARNING_HI) on failure and
    /// `OpenChip` (DIAGNOSTIC) on success, and commits the configuration
    /// only when the backend succeeded — a failed open leaves the driver in
    /// its "never opened" state, so `gpioRead`/`gpioWrite` answer
    /// [`GpioStatus::InvalidMode`].
    pub fn open(
        &self,
        device: &str,
        gpio: u32,
        configuration: GpioConfiguration,
        default_state: Logic,
    ) -> FileStatus {
        let consumer = self.consumer_label();
        match self
            .backend
            .open(device, gpio, configuration, default_state, &consumer)
        {
            Ok(info) => {
                self.state.lock().unwrap().configuration = Some(configuration);
                let chip = LogStringArg::from(info.name.as_str());
                let label = LogStringArg::from(info.label.as_str());
                let pin_message = LogStringArg::from(info.pin_message.as_str());
                self.evt.log_event(
                    self.id_base(),
                    Self::EVENTID_OPEN_CHIP,
                    LogSeverity::Diagnostic,
                    &format!(
                        "Opened GPIO chip {}[{}] pin {}[{}]",
                        info.name, info.label, gpio, info.pin_message
                    ),
                    |buf| {
                        fw_try!(chip.serialize_to_truncated(
                            buf,
                            EVENT_STRING_SIZE,
                            fprime_fw::Endianness::Big
                        ));
                        fw_try!(label.serialize_to_truncated(
                            buf,
                            EVENT_STRING_SIZE,
                            fprime_fw::Endianness::Big
                        ));
                        fw_try!(buf.serialize_u32_be(gpio));
                        pin_message.serialize_to_truncated(
                            buf,
                            EVENT_STRING_SIZE,
                            fprime_fw::Endianness::Big,
                        )
                    },
                );
                FileStatus::OpOk
            }
            Err(GpioOpenError::Chip(status)) => {
                let chip = LogStringArg::from(device);
                self.evt.log_event(
                    self.id_base(),
                    Self::EVENTID_OPEN_CHIP_ERROR,
                    LogSeverity::WarningHi,
                    &format!("Failed to open GPIO chip {device}: {status:?}"),
                    |buf| {
                        fw_try!(chip.serialize_to_truncated(
                            buf,
                            EVENT_STRING_SIZE,
                            fprime_fw::Endianness::Big
                        ));
                        // Os.FileStatus is a U8 FPP enum; fprime-os models it
                        // as repr(i32) with the same numeric values.
                        buf.serialize_u8_be(status as u8)
                    },
                );
                status
            }
            Err(GpioOpenError::Pin {
                pin_message,
                status,
            }) => {
                let chip = LogStringArg::from(device);
                let message = LogStringArg::from(pin_message.as_str());
                self.evt.log_event(
                    self.id_base(),
                    Self::EVENTID_OPEN_PIN_ERROR,
                    LogSeverity::WarningHi,
                    &format!(
                        "Failed to open GPIO chip {device} pin {gpio} [{pin_message}]: {status:?}"
                    ),
                    |buf| {
                        fw_try!(chip.serialize_to_truncated(
                            buf,
                            EVENT_STRING_SIZE,
                            fprime_fw::Endianness::Big
                        ));
                        fw_try!(buf.serialize_u32_be(gpio));
                        fw_try!(message.serialize_to_truncated(
                            buf,
                            EVENT_STRING_SIZE,
                            fprime_fw::Endianness::Big
                        ));
                        buf.serialize_u8_be(status as u8)
                    },
                );
                status
            }
        }
    }

    /// C++ destructor equivalent: release the line in the backend and drop
    /// back to the "never opened" state.
    pub fn close(&self) {
        self.backend.close();
        self.state.lock().unwrap().configuration = None;
    }

    /// The configuration committed by a successful [`Self::open`].
    #[must_use]
    pub fn configuration(&self) -> Option<GpioConfiguration> {
        self.state.lock().unwrap().configuration
    }

    /// C++ `start()`: launch the interrupt thread, named
    /// `"<objName>.interrupt"`.
    ///
    /// Returns [`GpioStatus::InvalidMode`] and starts nothing unless the pin
    /// was opened in one of the three interrupt configurations, and
    /// [`GpioStatus::UnknownError`] if the task fails to start.
    /// The C++ `identifier` argument is dropped (`fprime-os` defaults it).
    pub fn start(
        self: &Arc<Self>,
        priority: FwTaskPriorityType,
        stack_size: FwSizeType,
        cpu_affinity: FwSizeType,
    ) -> GpioStatus {
        {
            let mut state = self.state.lock().unwrap();
            match state.configuration {
                Some(configuration) if configuration.is_interrupt() => {}
                _ => return GpioStatus::InvalidMode,
            }
            state.running = true;
        }
        let name = self.base.get_obj_name();
        let task_name = format!("{}.interrupt", name.as_str().unwrap_or_default());
        let component = self.clone();
        let mut arguments = Arguments::new(&task_name, Box::new(move || component.poll_loop()));
        arguments.priority = priority;
        arguments.stack_size = stack_size;
        arguments.cpu_affinity = cpu_affinity;
        if self.poller.start(arguments) == TaskStatus::OpOk {
            GpioStatus::OpOk
        } else {
            self.state.lock().unwrap().running = false;
            GpioStatus::UnknownError
        }
    }

    /// C++ `stop()`: clear the running flag. Must precede [`Self::join`], or
    /// the join never returns.
    pub fn stop(&self) {
        self.state.lock().unwrap().running = false;
    }

    /// C++ `join()`: best-effort join of the interrupt thread.
    pub fn join(&self) -> TaskStatus {
        self.poller.join()
    }

    /// C++ `getRunning()`.
    fn get_running(&self) -> bool {
        self.state.lock().unwrap().running
    }

    /// C++ `pollLoop()`, with [`GpioBackend::poll`] standing in for
    /// `poll(2)` + `read(2)`.
    ///
    /// Note the deliberate C++ behavior preserved here: when the timestamp
    /// read fails, the error is logged and `gpioInterrupt_out` is invoked
    /// **anyway**, with whatever the [`RawTime`] holds.
    fn poll_loop(&self) {
        let timeout = Duration::from_millis(Self::GPIO_POLL_TIMEOUT);
        while self.get_running() {
            match self.backend.poll(timeout) {
                PollOutcome::Interrupt => {
                    let mut timestamp = RawTime::new();
                    let status = timestamp.now();
                    if status != RawTimeStatus::OpOk {
                        self.evt.log_event(
                            self.id_base(),
                            Self::EVENTID_INTERRUPT_TIME_ERROR,
                            LogSeverity::WarningHi,
                            &format!("Failed to read interrupt timestamp: {status:?}"),
                            // Os.RawTimeStatus is a U8 FPP enum.
                            |buf| buf.serialize_u8_be(status as u8),
                        );
                    }
                    let p = self.gpio_interrupt_out.get();
                    p.target.invoke(p.port_num, &timestamp);
                }
                PollOutcome::NoInterrupt => {}
                PollOutcome::ReadError { expected, got } => {
                    self.evt.log_event(
                        self.id_base(),
                        Self::EVENTID_INTERRUPT_READ_ERROR,
                        LogSeverity::WarningHi,
                        // Upstream typo "byes" preserved.
                        &format!("Interrupt data read expected {expected} byes and got {got}"),
                        |buf| {
                            fw_try!(buf.serialize_u32_be(expected));
                            buf.serialize_u32_be(got)
                        },
                    );
                }
                PollOutcome::PollError(errno) => {
                    self.evt.log_event(
                        self.id_base(),
                        Self::EVENTID_POLLING_ERROR,
                        LogSeverity::WarningHi,
                        &format!("Interrupt polling returned errno: {errno}"),
                        |buf| buf.serialize_i32_be(errno),
                    );
                }
            }
        }
    }

    // -- Handlers (caller thread) -------------------------------------------

    /// C++ `gpioRead_handler`: `INVALID_MODE` unless the pin is configured as
    /// `GPIO_INPUT`. The interrupt modes deliberately reject reads — a line
    /// that must be both polled and interrupt-driven needs two instances.
    fn gpio_read_handler(&self, _port_num: FwIndexType, state: &mut Logic) -> GpioStatus {
        if self.state.lock().unwrap().configuration != Some(GpioConfiguration::GpioInput) {
            return GpioStatus::InvalidMode;
        }
        match self.backend.read() {
            Ok(value) => {
                *state = value;
                GpioStatus::OpOk
            }
            Err(status) => status,
        }
    }

    /// C++ `gpioWrite_handler`: `INVALID_MODE` unless the pin is configured
    /// as `GPIO_OUTPUT`.
    fn gpio_write_handler(&self, _port_num: FwIndexType, state: Logic) -> GpioStatus {
        if self.state.lock().unwrap().configuration != Some(GpioConfiguration::GpioOutput) {
            return GpioStatus::InvalidMode;
        }
        self.backend.write(state)
    }
}

fprime_comp::input_port_adapter! {
    /// `gpioRead` — SYNC `Drv.GpioRead` input (caller's thread).
    component: LinuxGpioDriver;
    adapter: GpioReadAdapter;
    port: GpioReadPort;
    input: pub gpio_read_in;
    handler: gpio_read_handler;
    returns: GpioStatus;
    args { mut state: Logic }
}

fprime_comp::input_port_adapter! {
    /// `gpioWrite` — SYNC `Drv.GpioWrite` input (caller's thread).
    component: LinuxGpioDriver;
    adapter: GpioWriteAdapter;
    port: GpioWritePort;
    input: pub gpio_write_in;
    handler: gpio_write_handler;
    returns: GpioStatus;
    args { val state: Logic }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::{
        Deserialize, Endianness, LinearBuffer, LogBuffer, Serialize, SerializeStatus, Time,
    };
    use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

    // -- test scaffolding ---------------------------------------------------

    /// Minimal temp directory with cleanup on drop (no third-party crates).
    struct TempDir {
        path: PathBuf,
    }

    impl TempDir {
        fn new(tag: &str) -> Self {
            static COUNTER: AtomicU32 = AtomicU32::new(0);
            let unique = COUNTER.fetch_add(1, Ordering::SeqCst);
            let path = std::env::temp_dir()
                .join(format!("fprime-gpio-{tag}-{}-{unique}", std::process::id()));
            let _ = std::fs::remove_dir_all(&path);
            std::fs::create_dir_all(&path).unwrap();
            Self { path }
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.path);
        }
    }

    /// Collected event: (id, severity, serialized args).
    #[derive(Default)]
    struct EventCollector {
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
    }

    impl fprime_comp::LogPort for EventCollector {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            self.events
                .lock()
                .unwrap()
                .push((id, severity, args.as_slice().to_vec()));
        }
    }

    impl EventCollector {
        fn ids(&self) -> Vec<FwEventIdType> {
            self.events.lock().unwrap().iter().map(|e| e.0).collect()
        }
        fn last(&self) -> (FwEventIdType, LogSeverity, Vec<u8>) {
            self.events.lock().unwrap().last().cloned().unwrap()
        }
    }

    /// Counts `gpioInterrupt` invocations.
    #[derive(Default)]
    struct InterruptSink {
        count: AtomicUsize,
    }

    impl CyclePort for InterruptSink {
        fn invoke(&self, _port_num: FwIndexType, _cycle_start: &RawTime) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    /// What a fake `open()` was called with.
    type OpenRecord = (String, u32, GpioConfiguration, Logic, String);

    /// Deterministic in-test backend.
    #[derive(Default)]
    struct FakeBackend {
        open_result: Mutex<Option<Result<GpioChipInfo, GpioOpenError>>>,
        value: Mutex<Logic>,
        write_status: Mutex<GpioStatus>,
        read_status: Mutex<Option<GpioStatus>>,
        /// Scripted poll outcomes, consumed front to back; `NoInterrupt`
        /// afterwards.
        poll_script: Mutex<Vec<PollOutcome>>,
        opened_with: Mutex<Option<OpenRecord>>,
        closed: AtomicUsize,
    }

    impl FakeBackend {
        fn ok() -> Self {
            Self {
                open_result: Mutex::new(Some(Ok(GpioChipInfo::new("gpiochip0", "fake-chip")))),
                ..Self::default()
            }
        }
        fn with_open(result: Result<GpioChipInfo, GpioOpenError>) -> Self {
            Self {
                open_result: Mutex::new(Some(result)),
                ..Self::default()
            }
        }
    }

    impl GpioBackend for Arc<FakeBackend> {
        fn open(
            &self,
            device: &str,
            gpio: u32,
            configuration: GpioConfiguration,
            default_state: Logic,
            consumer: &str,
        ) -> Result<GpioChipInfo, GpioOpenError> {
            *self.opened_with.lock().unwrap() = Some((
                device.to_string(),
                gpio,
                configuration,
                default_state,
                consumer.to_string(),
            ));
            self.open_result
                .lock()
                .unwrap()
                .clone()
                .unwrap_or(Err(GpioOpenError::Chip(FileStatus::NotSupported)))
        }

        fn read(&self) -> Result<Logic, GpioStatus> {
            match *self.read_status.lock().unwrap() {
                Some(status) => Err(status),
                None => Ok(*self.value.lock().unwrap()),
            }
        }

        fn write(&self, state: Logic) -> GpioStatus {
            let status = *self.write_status.lock().unwrap();
            if status == GpioStatus::OpOk {
                *self.value.lock().unwrap() = state;
            }
            status
        }

        fn poll(&self, _timeout: Duration) -> PollOutcome {
            let mut script = self.poll_script.lock().unwrap();
            if script.is_empty() {
                drop(script);
                std::thread::sleep(Duration::from_millis(1));
                return PollOutcome::NoInterrupt;
            }
            script.remove(0)
        }

        fn close(&self) {
            self.closed.fetch_add(1, Ordering::SeqCst);
        }
    }

    fn driver_with(fake: &Arc<FakeBackend>) -> (Arc<LinuxGpioDriver>, Arc<EventCollector>) {
        let driver = LinuxGpioDriver::with_backend("gpioDrv", Box::new(fake.clone()));
        let collector = Arc::new(EventCollector::default());
        driver.evt.log_out.connect(collector.clone(), 0);
        (driver, collector)
    }

    // -- enums / wire format -------------------------------------------------

    #[test]
    fn gpio_status_discriminants_match_fpp() {
        assert_eq!(GpioStatus::OpOk as u8, 0);
        assert_eq!(GpioStatus::NotOpened as u8, 1);
        assert_eq!(GpioStatus::InvalidMode as u8, 2);
        assert_eq!(GpioStatus::UnknownError as u8, 3);
        assert_eq!(GpioStatus::NUM_CONSTANTS, 4);
    }

    #[test]
    fn logic_discriminants_and_wire_width_match_fw_types() {
        assert_eq!(Logic::Low as u8, 0);
        assert_eq!(Logic::High as u8, 1);
        let mut buf = LinearBuffer::<4>::new();
        assert!(Logic::High.serialize_to(&mut buf, Endianness::Big).is_ok());
        assert_eq!(buf.as_slice(), &[1]);
    }

    #[test]
    fn gpio_status_decode_rejects_undeclared_value() {
        let mut buf = LinearBuffer::<4>::new();
        assert!(buf.serialize_u8_be(9).is_ok());
        let mut out = GpioStatus::InvalidMode;
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserFormatError
        );
        assert_eq!(out, GpioStatus::InvalidMode);
    }

    #[test]
    fn errno_tables_match_cpp() {
        let map = |errno: i32| errno_to_file_status(&io::Error::from_raw_os_error(errno));
        assert_eq!(map(0), FileStatus::OpOk);
        assert_eq!(map(9), FileStatus::NotOpened);
        assert_eq!(map(22), FileStatus::InvalidArgument);
        assert_eq!(map(19), FileStatus::DoesntExist);
        assert_eq!(map(12), FileStatus::NoSpace);
        assert_eq!(map(1), FileStatus::NoPermission);
        assert_eq!(map(6), FileStatus::InvalidMode);
        assert_eq!(map(5), FileStatus::OtherError);

        let gmap = |errno: i32| errno_to_gpio_status(&io::Error::from_raw_os_error(errno));
        assert_eq!(gmap(9), GpioStatus::NotOpened);
        assert_eq!(gmap(6), GpioStatus::InvalidMode);
        assert_eq!(gmap(22), GpioStatus::UnknownError);
        assert_eq!(
            errno_to_gpio_status(&io::Error::other("no errno")),
            GpioStatus::UnknownError
        );
    }

    #[test]
    fn edge_detection_matches_configured_mode() {
        use GpioConfiguration::*;
        assert!(GpioInterruptRisingEdge.edge_fires(Logic::Low, Logic::High));
        assert!(!GpioInterruptRisingEdge.edge_fires(Logic::High, Logic::Low));
        assert!(GpioInterruptFallingEdge.edge_fires(Logic::High, Logic::Low));
        assert!(!GpioInterruptFallingEdge.edge_fires(Logic::Low, Logic::High));
        assert!(GpioInterruptBothRisingAndFallingEdges.edge_fires(Logic::Low, Logic::High));
        assert!(GpioInterruptBothRisingAndFallingEdges.edge_fires(Logic::High, Logic::Low));
        assert!(!GpioInterruptBothRisingAndFallingEdges.edge_fires(Logic::Low, Logic::Low));
        assert!(!GpioInput.edge_fires(Logic::Low, Logic::High));
        assert!(!GpioOutput.edge_fires(Logic::Low, Logic::High));
        assert!(GpioInterruptRisingEdge.is_interrupt());
        assert!(!GpioInput.is_interrupt());
        assert!(!GpioOutput.is_interrupt());
    }

    // -- component: open / events -------------------------------------------

    #[test]
    fn open_success_emits_diagnostic_open_chip_with_exact_bytes() {
        let fake = Arc::new(FakeBackend::ok());
        let (driver, events) = driver_with(&fake);
        let status = driver.open(
            "/dev/gpiochip0",
            17,
            GpioConfiguration::GpioOutput,
            Logic::High,
        );
        assert_eq!(status, FileStatus::OpOk);
        assert_eq!(driver.configuration(), Some(GpioConfiguration::GpioOutput));

        let (id, severity, bytes) = events.last();
        assert_eq!(id, LinuxGpioDriver::EVENTID_OPEN_CHIP);
        assert_eq!(severity, LogSeverity::Diagnostic);
        // [u16 len]"gpiochip0" [u16 len]"fake-chip" [u32 pin] [u16 len]"Unknown"
        let mut expected = Vec::new();
        expected.extend_from_slice(&[0, 9]);
        expected.extend_from_slice(b"gpiochip0");
        expected.extend_from_slice(&[0, 9]);
        expected.extend_from_slice(b"fake-chip");
        expected.extend_from_slice(&[0, 0, 0, 17]);
        expected.extend_from_slice(&[0, 7]);
        expected.extend_from_slice(b"Unknown");
        assert_eq!(bytes, expected);

        // The consumer label is the object name, truncated to 32 bytes.
        let opened = fake.opened_with.lock().unwrap().clone().unwrap();
        assert_eq!(opened.0, "/dev/gpiochip0");
        assert_eq!(opened.1, 17);
        assert_eq!(opened.2, GpioConfiguration::GpioOutput);
        assert_eq!(opened.3, Logic::High);
        assert_eq!(opened.4, "gpioDrv");
    }

    #[test]
    fn open_chip_failure_emits_open_chip_error_and_leaves_driver_closed() {
        let fake = Arc::new(FakeBackend::with_open(Err(GpioOpenError::Chip(
            FileStatus::NotSupported,
        ))));
        let (driver, events) = driver_with(&fake);
        let status = driver.open(
            "/dev/gpiochip0",
            3,
            GpioConfiguration::GpioInput,
            Logic::Low,
        );
        assert_eq!(status, FileStatus::NotSupported);
        assert_eq!(driver.configuration(), None);

        let (id, severity, bytes) = events.last();
        assert_eq!(id, LinuxGpioDriver::EVENTID_OPEN_CHIP_ERROR);
        assert_eq!(severity, LogSeverity::WarningHi);
        let mut expected = vec![0, 14];
        expected.extend_from_slice(b"/dev/gpiochip0");
        expected.push(FileStatus::NotSupported as u8);
        assert_eq!(bytes, expected);
        assert_eq!(FileStatus::NotSupported as u8, 7);
    }

    #[test]
    fn open_pin_failure_emits_open_pin_error_with_exact_bytes() {
        let fake = Arc::new(FakeBackend::with_open(Err(GpioOpenError::Pin {
            pin_message: "Does Not Exist".to_string(),
            status: FileStatus::DoesntExist,
        })));
        let (driver, events) = driver_with(&fake);
        let status = driver.open(
            "/dev/gpiochip0",
            99,
            GpioConfiguration::GpioInput,
            Logic::Low,
        );
        assert_eq!(status, FileStatus::DoesntExist);

        let (id, severity, bytes) = events.last();
        assert_eq!(id, LinuxGpioDriver::EVENTID_OPEN_PIN_ERROR);
        assert_eq!(severity, LogSeverity::WarningHi);
        let mut expected = vec![0, 14];
        expected.extend_from_slice(b"/dev/gpiochip0");
        expected.extend_from_slice(&[0, 0, 0, 99]);
        expected.extend_from_slice(&[0, 14]);
        expected.extend_from_slice(b"Does Not Exist");
        expected.push(FileStatus::DoesntExist as u8);
        assert_eq!(bytes, expected);
    }

    // -- component: mode gating ---------------------------------------------

    #[test]
    fn read_and_write_are_invalid_mode_before_open() {
        let fake = Arc::new(FakeBackend::ok());
        let (driver, _events) = driver_with(&fake);
        let mut state = Logic::Low;
        let read = driver.gpio_read_in(0);
        let write = driver.gpio_write_in(0);
        assert_eq!(
            read.target.invoke(read.port_num, &mut state),
            GpioStatus::InvalidMode
        );
        assert_eq!(
            write.target.invoke(write.port_num, Logic::High),
            GpioStatus::InvalidMode
        );
    }

    #[test]
    fn input_mode_allows_read_and_rejects_write() {
        let fake = Arc::new(FakeBackend::ok());
        let (driver, _events) = driver_with(&fake);
        assert_eq!(
            driver.open(
                "/dev/gpiochip0",
                1,
                GpioConfiguration::GpioInput,
                Logic::Low
            ),
            FileStatus::OpOk
        );
        *fake.value.lock().unwrap() = Logic::High;

        let mut state = Logic::Low;
        let read = driver.gpio_read_in(0);
        assert_eq!(
            read.target.invoke(read.port_num, &mut state),
            GpioStatus::OpOk
        );
        assert_eq!(state, Logic::High);

        let write = driver.gpio_write_in(0);
        assert_eq!(
            write.target.invoke(write.port_num, Logic::Low),
            GpioStatus::InvalidMode
        );
    }

    #[test]
    fn output_mode_allows_write_and_rejects_read() {
        let fake = Arc::new(FakeBackend::ok());
        let (driver, _events) = driver_with(&fake);
        assert_eq!(
            driver.open(
                "/dev/gpiochip0",
                1,
                GpioConfiguration::GpioOutput,
                Logic::Low
            ),
            FileStatus::OpOk
        );
        let write = driver.gpio_write_in(0);
        assert_eq!(
            write.target.invoke(write.port_num, Logic::High),
            GpioStatus::OpOk
        );
        assert_eq!(*fake.value.lock().unwrap(), Logic::High);

        let mut state = Logic::Low;
        let read = driver.gpio_read_in(0);
        assert_eq!(
            read.target.invoke(read.port_num, &mut state),
            GpioStatus::InvalidMode
        );
    }

    #[test]
    fn interrupt_mode_rejects_read_and_write() {
        let fake = Arc::new(FakeBackend::ok());
        let (driver, _events) = driver_with(&fake);
        assert_eq!(
            driver.open(
                "/dev/gpiochip0",
                1,
                GpioConfiguration::GpioInterruptBothRisingAndFallingEdges,
                Logic::Low
            ),
            FileStatus::OpOk
        );
        let mut state = Logic::Low;
        let read = driver.gpio_read_in(0);
        let write = driver.gpio_write_in(0);
        assert_eq!(
            read.target.invoke(read.port_num, &mut state),
            GpioStatus::InvalidMode
        );
        assert_eq!(
            write.target.invoke(write.port_num, Logic::High),
            GpioStatus::InvalidMode
        );
    }

    #[test]
    fn read_propagates_backend_status() {
        let fake = Arc::new(FakeBackend::ok());
        let (driver, _events) = driver_with(&fake);
        assert_eq!(
            driver.open(
                "/dev/gpiochip0",
                1,
                GpioConfiguration::GpioInput,
                Logic::Low
            ),
            FileStatus::OpOk
        );
        *fake.read_status.lock().unwrap() = Some(GpioStatus::NotOpened);
        let mut state = Logic::Low;
        let read = driver.gpio_read_in(0);
        assert_eq!(
            read.target.invoke(read.port_num, &mut state),
            GpioStatus::NotOpened
        );
    }

    #[test]
    fn write_propagates_backend_status() {
        let fake = Arc::new(FakeBackend::ok());
        let (driver, _events) = driver_with(&fake);
        assert_eq!(
            driver.open(
                "/dev/gpiochip0",
                1,
                GpioConfiguration::GpioOutput,
                Logic::Low
            ),
            FileStatus::OpOk
        );
        *fake.write_status.lock().unwrap() = GpioStatus::UnknownError;
        let write = driver.gpio_write_in(0);
        assert_eq!(
            write.target.invoke(write.port_num, Logic::High),
            GpioStatus::UnknownError
        );
    }

    #[test]
    fn close_releases_the_backend_and_resets_the_mode() {
        let fake = Arc::new(FakeBackend::ok());
        let (driver, _events) = driver_with(&fake);
        assert_eq!(
            driver.open(
                "/dev/gpiochip0",
                1,
                GpioConfiguration::GpioInput,
                Logic::Low
            ),
            FileStatus::OpOk
        );
        driver.close();
        assert_eq!(fake.closed.load(Ordering::SeqCst), 1);
        assert_eq!(driver.configuration(), None);
    }

    // -- component: interrupt thread ----------------------------------------

    #[test]
    fn start_rejects_non_interrupt_configurations() {
        let fake = Arc::new(FakeBackend::ok());
        let (driver, _events) = driver_with(&fake);
        assert_eq!(driver.start(100, 0, 0), GpioStatus::InvalidMode);
        assert_eq!(
            driver.open(
                "/dev/gpiochip0",
                1,
                GpioConfiguration::GpioInput,
                Logic::Low
            ),
            FileStatus::OpOk
        );
        assert_eq!(driver.start(100, 0, 0), GpioStatus::InvalidMode);
        assert_eq!(
            driver.open(
                "/dev/gpiochip0",
                1,
                GpioConfiguration::GpioOutput,
                Logic::Low
            ),
            FileStatus::OpOk
        );
        assert_eq!(driver.start(100, 0, 0), GpioStatus::InvalidMode);
    }

    #[test]
    fn poll_thread_forwards_interrupts_and_logs_error_outcomes() {
        let fake = Arc::new(FakeBackend::ok());
        fake.poll_script.lock().unwrap().extend_from_slice(&[
            PollOutcome::NoInterrupt,
            PollOutcome::Interrupt,
            PollOutcome::ReadError {
                expected: 48,
                got: 0xFFFF_FFFF,
            },
            PollOutcome::PollError(4),
            PollOutcome::Interrupt,
        ]);
        let (driver, events) = driver_with(&fake);
        let sink = Arc::new(InterruptSink::default());
        driver.gpio_interrupt_out.connect(sink.clone(), 0);
        assert_eq!(
            driver.open(
                "/dev/gpiochip0",
                4,
                GpioConfiguration::GpioInterruptRisingEdge,
                Logic::Low
            ),
            FileStatus::OpOk
        );
        assert_eq!(driver.start(100, 0, 0), GpioStatus::OpOk);

        // Wait for the script to drain (bounded).
        for _ in 0..500 {
            if fake.poll_script.lock().unwrap().is_empty() && sink.count.load(Ordering::SeqCst) >= 2
            {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        driver.stop();
        assert_eq!(driver.join(), TaskStatus::OpOk);

        assert_eq!(sink.count.load(Ordering::SeqCst), 2);
        let ids = events.ids();
        assert!(ids.contains(&LinuxGpioDriver::EVENTID_INTERRUPT_READ_ERROR));
        assert!(ids.contains(&LinuxGpioDriver::EVENTID_POLLING_ERROR));

        // InterruptReadError args: [expected u32][got u32], the got=-1 cast
        // reported as 0xFFFFFFFF exactly like C++.
        let read_error = events
            .events
            .lock()
            .unwrap()
            .iter()
            .find(|e| e.0 == LinuxGpioDriver::EVENTID_INTERRUPT_READ_ERROR)
            .cloned()
            .unwrap();
        assert_eq!(read_error.1, LogSeverity::WarningHi);
        assert_eq!(read_error.2, vec![0, 0, 0, 48, 0xFF, 0xFF, 0xFF, 0xFF]);

        let poll_error = events
            .events
            .lock()
            .unwrap()
            .iter()
            .find(|e| e.0 == LinuxGpioDriver::EVENTID_POLLING_ERROR)
            .cloned()
            .unwrap();
        assert_eq!(poll_error.2, vec![0, 0, 0, 4]);
    }

    // -- stub backend --------------------------------------------------------

    #[test]
    fn stub_backend_mirrors_the_cpp_stub() {
        let stub = StubGpioBackend::new();
        assert_eq!(
            stub.open(
                "/dev/gpiochip0",
                0,
                GpioConfiguration::GpioInput,
                Logic::Low,
                "x"
            ),
            Err(GpioOpenError::Chip(FileStatus::NotSupported))
        );
        assert_eq!(stub.read(), Err(GpioStatus::UnknownError));
        assert_eq!(stub.write(Logic::High), GpioStatus::UnknownError);
        assert_eq!(
            stub.poll(Duration::from_millis(1)),
            PollOutcome::NoInterrupt
        );
    }

    // -- sysfs backend -------------------------------------------------------

    fn make_sysfs(dir: &Path, base: u32, ngpio: u32, label: &str) {
        let chip = dir.join(format!("gpiochip{base}"));
        std::fs::create_dir_all(&chip).unwrap();
        std::fs::write(chip.join("base"), format!("{base}\n")).unwrap();
        std::fs::write(chip.join("ngpio"), format!("{ngpio}\n")).unwrap();
        std::fs::write(chip.join("label"), format!("{label}\n")).unwrap();
    }

    fn make_line(dir: &Path, line: u32, value: &str) {
        let line_dir = dir.join(format!("gpio{line}"));
        std::fs::create_dir_all(&line_dir).unwrap();
        std::fs::write(line_dir.join("value"), value).unwrap();
        std::fs::write(line_dir.join("direction"), "in").unwrap();
        std::fs::write(line_dir.join("edge"), "none").unwrap();
    }

    #[test]
    fn sysfs_open_output_writes_export_and_direction_files() {
        let temp = TempDir::new("out");
        make_sysfs(&temp.path, 0, 32, "pinctrl-test");
        // The line dir appears when the kernel handles `export`; pre-create
        // it here so the write targets exist.
        make_line(&temp.path, 17, "0");

        let backend = SysfsGpioBackend::with_root(&temp.path);
        let info = backend
            .open(
                "/dev/gpiochip0",
                17,
                GpioConfiguration::GpioOutput,
                Logic::High,
                "gpioDrv",
            )
            .expect("open");
        assert_eq!(info.name, "gpiochip0");
        assert_eq!(info.label, "pinctrl-test");
        assert_eq!(info.pin_message, "Unknown");
        assert_eq!(backend.line(), Some(17));

        // Exact file contents (this IS the wire format of sysfs GPIO).
        assert_eq!(
            std::fs::read(temp.path.join("gpio17/direction")).unwrap(),
            b"high"
        );
        assert_eq!(
            std::fs::read(temp.path.join("gpio17/edge")).unwrap(),
            b"none"
        );

        assert_eq!(backend.write(Logic::High), GpioStatus::OpOk);
        assert_eq!(std::fs::read(temp.path.join("gpio17/value")).unwrap(), b"1");
        assert_eq!(backend.write(Logic::Low), GpioStatus::OpOk);
        assert_eq!(std::fs::read(temp.path.join("gpio17/value")).unwrap(), b"0");

        backend.close();
        assert_eq!(std::fs::read(temp.path.join("unexport")).unwrap(), b"17");
        assert_eq!(backend.line(), None);
    }

    #[test]
    fn sysfs_export_is_written_when_the_line_directory_is_absent() {
        let temp = TempDir::new("export");
        make_sysfs(&temp.path, 0, 32, "chip");
        let backend = SysfsGpioBackend::with_root(&temp.path);
        // No gpio5 directory yet: export is written, then the direction
        // write fails (the kernel would have created the directory).
        let error = backend
            .open(
                "/dev/gpiochip0",
                5,
                GpioConfiguration::GpioInput,
                Logic::Low,
                "c",
            )
            .unwrap_err();
        assert_eq!(std::fs::read(temp.path.join("export")).unwrap(), b"5");
        assert!(matches!(error, GpioOpenError::Pin { .. }));
    }

    #[test]
    fn sysfs_input_open_writes_in_and_reads_the_value_file() {
        let temp = TempDir::new("in");
        make_sysfs(&temp.path, 0, 32, "chip");
        make_line(&temp.path, 3, "1\n");
        let backend = SysfsGpioBackend::with_root(&temp.path);
        backend
            .open(
                "/dev/gpiochip0",
                3,
                GpioConfiguration::GpioInput,
                Logic::High,
                "c",
            )
            .expect("open");
        // default_state is ignored for inputs (C++ parity).
        assert_eq!(
            std::fs::read(temp.path.join("gpio3/direction")).unwrap(),
            b"in"
        );
        assert_eq!(backend.read(), Ok(Logic::High));
        std::fs::write(temp.path.join("gpio3/value"), "0\n").unwrap();
        assert_eq!(backend.read(), Ok(Logic::Low));
        std::fs::write(temp.path.join("gpio3/value"), "x").unwrap();
        assert_eq!(backend.read(), Err(GpioStatus::UnknownError));
    }

    #[test]
    fn sysfs_line_beyond_ngpio_is_doesnt_exist() {
        let temp = TempDir::new("range");
        make_sysfs(&temp.path, 0, 8, "chip");
        let backend = SysfsGpioBackend::with_root(&temp.path);
        let error = backend
            .open(
                "/dev/gpiochip0",
                8,
                GpioConfiguration::GpioInput,
                Logic::Low,
                "c",
            )
            .unwrap_err();
        assert_eq!(error.status(), FileStatus::DoesntExist);
        assert_eq!(
            error,
            GpioOpenError::Pin {
                pin_message: "Does Not Exist".to_string(),
                status: FileStatus::DoesntExist,
            }
        );
    }

    #[test]
    fn sysfs_missing_root_or_chip_is_not_supported() {
        let temp = TempDir::new("missing");
        let backend = SysfsGpioBackend::with_root(temp.path.join("nope"));
        assert_eq!(
            backend
                .open(
                    "/dev/gpiochip0",
                    0,
                    GpioConfiguration::GpioInput,
                    Logic::Low,
                    "c"
                )
                .unwrap_err()
                .status(),
            FileStatus::NotSupported
        );

        // Root exists but has no chips.
        let backend = SysfsGpioBackend::with_root(&temp.path);
        assert_eq!(
            backend
                .open(
                    "/dev/gpiochip0",
                    0,
                    GpioConfiguration::GpioInput,
                    Logic::Low,
                    "c"
                )
                .unwrap_err()
                .status(),
            FileStatus::NotSupported
        );
    }

    #[test]
    fn sysfs_non_gpiochip_device_path_is_invalid_argument() {
        let temp = TempDir::new("badpath");
        make_sysfs(&temp.path, 0, 8, "chip");
        let backend = SysfsGpioBackend::with_root(&temp.path);
        assert_eq!(
            backend
                .open(
                    "/dev/ttyUSB0",
                    0,
                    GpioConfiguration::GpioInput,
                    Logic::Low,
                    "c"
                )
                .unwrap_err()
                .status(),
            FileStatus::InvalidArgument
        );
    }

    #[test]
    fn sysfs_chip_index_resolves_by_ascending_base() {
        let temp = TempDir::new("chips");
        make_sysfs(&temp.path, 100, 4, "chip-b");
        make_sysfs(&temp.path, 0, 4, "chip-a");
        make_line(&temp.path, 101, "0");
        let backend = SysfsGpioBackend::with_root(&temp.path);
        let info = backend
            .open(
                "/dev/gpiochip1",
                1,
                GpioConfiguration::GpioInput,
                Logic::Low,
                "c",
            )
            .expect("open");
        assert_eq!(info.label, "chip-b");
        assert_eq!(backend.line(), Some(101));
    }

    #[test]
    fn sysfs_sampling_poller_reports_only_configured_transitions() {
        let temp = TempDir::new("sample");
        make_sysfs(&temp.path, 0, 8, "chip");
        make_line(&temp.path, 2, "0");
        let backend =
            SysfsGpioBackend::with_root(&temp.path).with_sample_interval(Duration::from_millis(1));
        backend
            .open(
                "/dev/gpiochip0",
                2,
                GpioConfiguration::GpioInterruptRisingEdge,
                Logic::Low,
                "c",
            )
            .expect("open");
        assert_eq!(
            std::fs::read(temp.path.join("gpio2/edge")).unwrap(),
            b"rising"
        );

        let timeout = Duration::from_millis(50);
        // First sample only primes the previous-level state.
        assert_eq!(backend.poll(timeout), PollOutcome::NoInterrupt);
        std::fs::write(temp.path.join("gpio2/value"), "1").unwrap();
        assert_eq!(backend.poll(timeout), PollOutcome::Interrupt);
        // Steady high: nothing more.
        assert_eq!(backend.poll(timeout), PollOutcome::NoInterrupt);
        // Falling edge is not configured.
        std::fs::write(temp.path.join("gpio2/value"), "0").unwrap();
        assert_eq!(backend.poll(timeout), PollOutcome::NoInterrupt);
    }

    #[test]
    fn sysfs_sampling_poller_reports_read_failures_as_poll_errors() {
        let temp = TempDir::new("pollerr");
        make_sysfs(&temp.path, 0, 8, "chip");
        make_line(&temp.path, 2, "0");
        let backend =
            SysfsGpioBackend::with_root(&temp.path).with_sample_interval(Duration::from_millis(1));
        backend
            .open(
                "/dev/gpiochip0",
                2,
                GpioConfiguration::GpioInterruptBothRisingAndFallingEdges,
                Logic::Low,
                "c",
            )
            .expect("open");
        std::fs::remove_file(temp.path.join("gpio2/value")).unwrap();
        // ENOENT (2) is not in the errno table, so it is reported verbatim.
        assert_eq!(
            backend.poll(Duration::from_millis(5)),
            PollOutcome::PollError(2)
        );
    }

    #[test]
    fn sysfs_poll_without_open_is_no_interrupt() {
        let temp = TempDir::new("noopen");
        let backend = SysfsGpioBackend::with_root(&temp.path);
        assert_eq!(
            backend.poll(Duration::from_millis(1)),
            PollOutcome::NoInterrupt
        );
        assert_eq!(backend.read(), Err(GpioStatus::NotOpened));
        assert_eq!(backend.write(Logic::High), GpioStatus::NotOpened);
    }

    #[test]
    fn consumer_label_truncates_to_thirty_two_bytes() {
        let fake = Arc::new(FakeBackend::ok());
        let long = "a".repeat(40);
        let driver = LinuxGpioDriver::with_backend(&long, Box::new(fake.clone()));
        let _ = driver.open(
            "/dev/gpiochip0",
            0,
            GpioConfiguration::GpioInput,
            Logic::Low,
        );
        let opened = fake.opened_with.lock().unwrap().clone().unwrap();
        assert_eq!(opened.4.len(), LinuxGpioDriver::CONSUMER_LABEL_SIZE);
    }

    #[test]
    fn driver_used_with_the_sysfs_backend_end_to_end() {
        let temp = TempDir::new("e2e");
        make_sysfs(&temp.path, 0, 8, "chip");
        make_line(&temp.path, 4, "0");
        let driver = LinuxGpioDriver::with_backend(
            "gpio",
            Box::new(SysfsGpioBackend::with_root(&temp.path)),
        );
        let events = Arc::new(EventCollector::default());
        driver.evt.log_out.connect(events.clone(), 0);
        assert_eq!(
            driver.open(
                "/dev/gpiochip0",
                4,
                GpioConfiguration::GpioOutput,
                Logic::Low
            ),
            FileStatus::OpOk
        );
        assert_eq!(
            std::fs::read(temp.path.join("gpio4/direction")).unwrap(),
            b"low"
        );
        let write = driver.gpio_write_in(0);
        assert_eq!(
            write.target.invoke(write.port_num, Logic::High),
            GpioStatus::OpOk
        );
        assert_eq!(std::fs::read(temp.path.join("gpio4/value")).unwrap(), b"1");
        assert_eq!(events.ids(), vec![LinuxGpioDriver::EVENTID_OPEN_CHIP]);
    }
}
