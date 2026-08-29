//! # LinuxUartDriver — port of `Drv::LinuxUartDriver`
//!
//! C++ sources: `Drv/LinuxUartDriver/LinuxUartDriver.{fpp,hpp,cpp}`,
//! `Events.fppi`, `Telemetry.fppi`.
//! Analysis: `docs/cpp-analysis/linux-drivers.md`.
//!
//! PASSIVE component implementing the `Drv.ByteStreamDriver` interface over a
//! serial device, with a dedicated read thread (C++ `"SerReader"`):
//!
//! - `send` (guarded input, [`ByteStreamSendPort`]) — writes the caller's
//!   buffer on the caller's thread;
//! - `recvReturnIn` (guarded input, `Fw.BufferSend`) — forwards to
//!   `deallocate_out`;
//! - `run` (sync input, `Svc.Sched`) — writes the two byte counters;
//! - outputs `allocate` (`Fw.BufferGet`), `deallocate` (`Fw.BufferSend`),
//!   `recv` ([`ByteStreamDataPort`]), `ready` ([`ByteStreamReadyPort`]).
//!
//! ## Deviation: no termios configuration
//!
//! The C++ `open()` configures the port with `tcgetattr`/`tcsetattr`,
//! `cfsetispeed`/`cfsetospeed` and `tcflush` — every one of them an
//! `ioctl(2)` under the hood, and unreachable from safe, zero-dependency
//! `std` Rust. Baud rate, parity, `CS8|CLOCAL|CREAD`, `CRTSCTS` and the
//! `VMIN`/`VTIME` read timeout therefore CANNOT be applied here, and
//! `O_NOCTTY` cannot be requested either (so a process with no controlling
//! terminal may acquire the tty as one — a documented divergence).
//!
//! Rather than run at whatever settings the port happens to carry, this port
//! makes configuration an explicit choice — [`UartConfigPolicy`]:
//!
//! - [`UartConfigPolicy::Reject`] (the default) — [`LinuxUartDriver::open`]
//!   emits the `ConfigError` event (FPP id 1, declared but never emitted
//!   upstream — finally put to work) and returns `false` without touching
//!   the device. Nothing silently runs at the wrong baud.
//! - [`UartConfigPolicy::Stty`] — the requested settings are applied by
//!   shelling out to `stty(1)` via [`std::process::Command`] (safe and
//!   dependency-free, but an external process and a Linux/BSD userland
//!   dependency). A failing `stty` is a `ConfigError` and a failed open.
//! - [`UartConfigPolicy::TrustExternal`] — the caller asserts the device was
//!   already configured externally (e.g.
//!   `stty -F /dev/ttyUSB0 115200 raw -echo -crtscts min 0 time 10`);
//!   `open()` proceeds without applying anything.
//!
//! [`LinuxUartDriver::open_preconfigured`] is the same as `TrustExternal`
//! but does not ask for settings at all.
//!
//! Data transfer itself is faithful: [`SerialBackend`] is plain `read`/
//! `write`, and the read thread reproduces the C++ loop shape exactly
//! (allocate → spin while the read returns 0 and quit is not requested →
//! `set_size(0)` → classify → `recv_out` with the possibly-zero-sized
//! buffer). Because `VMIN`/`VTIME` may not be applied, a read that returns 0
//! immediately would hot-spin, so the retry is paced by
//! [`LinuxUartDriver::IDLE_READ_RETRY_US`] (a documented addition — the C++
//! loop relies on the 1 s `VTIME` timeout for the same effect).
//!
//! Unlike C++, the device string is OWNED by the component (C++ stores the
//! caller's `const char*` with no copy — a latent dangling pointer).

use crate::byte_stream::{
    ByteStreamDataPort, ByteStreamReadyPort, ByteStreamSendPort, ByteStreamStatus,
};
use fprime_comp::{
    BufferGetPort, BufferSendPort, EventGlue, EventThrottle, OutputPort, PassiveBase, SchedPort,
    TlmGlue,
};
use fprime_config::{
    FwChanIdType, FwEventIdType, FwIdType, FwIndexType, FwSizeType, FwTaskPriorityType,
};
use fprime_fw::{
    Buffer, Endianness, LogSeverity, LogStringArg, SerBuf, TimeInterval, fw_assert, fw_try,
};
use fprime_os::Task;
use fprime_os::task::{Arguments, Status as TaskStatus};
use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// UART event string arguments are declared `string size 40`.
const EVENT_STRING_SIZE: usize = 40;

/// C++ `LinuxUartDriver::UartBaudRate` — the constants ARE the baud numbers.
///
/// The conditionally-compiled high rates (`B460800`..`B4000000`) are all
/// present here; whether the underlying tty supports one is discovered when
/// the configuration is applied (or not applied at all — see the module
/// documentation).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum UartBaudRate {
    /// 9600 baud.
    Baud9600 = 9600,
    /// 19200 baud.
    Baud19200 = 19200,
    /// 38400 baud.
    Baud38400 = 38400,
    /// 57600 baud.
    Baud57600 = 57600,
    /// 115200 baud (C++ `BAUD_115K`).
    Baud115K = 115_200,
    /// 230400 baud (C++ `BAUD_230K`).
    Baud230K = 230_400,
    /// 460800 baud (C++ `BAUD_460K`).
    Baud460K = 460_800,
    /// 921600 baud (C++ `BAUD_921K`).
    Baud921K = 921_600,
    /// 1000000 baud (C++ `BAUD_1000K`).
    Baud1000K = 1_000_000,
    /// 1152000 baud (C++ `BAUD_1152K`).
    Baud1152K = 1_152_000,
    /// 1500000 baud (C++ `BAUD_1500K`).
    Baud1500K = 1_500_000,
    /// 2000000 baud (C++ `BAUD_2000K`).
    Baud2000K = 2_000_000,
    /// 2500000 baud (C++ `BAUD_2500K`).
    Baud2500K = 2_500_000,
    /// 3000000 baud (C++ `BAUD_3000K`).
    Baud3000K = 3_000_000,
    /// 3500000 baud (C++ `BAUD_3500K`).
    Baud3500K = 3_500_000,
    /// 4000000 baud (C++ `BAUD_4000K`).
    Baud4000K = 4_000_000,
}

impl UartBaudRate {
    /// The baud rate as a number (the enum's own discriminant).
    #[must_use]
    pub const fn as_u32(self) -> u32 {
        self as u32
    }
}

/// C++ `LinuxUartDriver::UartFlowControl`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum UartFlowControl {
    /// No hardware flow control (`-crtscts`).
    #[default]
    NoFlow = 0,
    /// RTS/CTS hardware flow control (`CRTSCTS`).
    HwFlow = 1,
}

/// C++ `LinuxUartDriver::UartParity`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum UartParity {
    /// No parity bit (`-parenb`).
    #[default]
    ParityNone = 0,
    /// Odd parity (`PARENB | PARODD`).
    ParityOdd = 1,
    /// Even parity (`PARENB`).
    ParityEven = 2,
}

/// How [`LinuxUartDriver::open`] deals with the settings it cannot apply
/// through `termios` (see the module documentation).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UartConfigPolicy {
    /// Default: refuse to open. `open()` emits `ConfigError` and returns
    /// `false`, so nothing runs at an unverified baud rate.
    #[default]
    Reject,
    /// Apply the requested settings by running `stty(1)`
    /// ([`stty_arguments`] builds the command line).
    Stty,
    /// The caller guarantees the device is already configured to match;
    /// `open()` applies nothing and proceeds.
    TrustExternal,
}

/// Build the `stty(1)` argument vector for a requested configuration.
///
/// Exposed (and unit-tested) so a deployment can log or reproduce exactly
/// what [`UartConfigPolicy::Stty`] runs:
///
/// ```text
/// stty -F <device> <baud> cs8 clocal cread raw -echo min 0 time 10 \
///      {crtscts|-crtscts} {-parenb | parenb parodd | parenb -parodd}
/// ```
///
/// `min 0 time 10` reproduces the C++ `c_cc[VMIN]=0`, `c_cc[VTIME]=10`
/// (1 s no-data timeout); `raw -echo` reproduces `c_oflag = 0`,
/// `c_lflag = 0`.
#[must_use]
pub fn stty_arguments(
    device: &str,
    baud: UartBaudRate,
    flow_control: UartFlowControl,
    parity: UartParity,
) -> Vec<String> {
    let mut args = vec![
        "-F".to_string(),
        device.to_string(),
        baud.as_u32().to_string(),
        "cs8".to_string(),
        "clocal".to_string(),
        "cread".to_string(),
        "raw".to_string(),
        "-echo".to_string(),
        "min".to_string(),
        "0".to_string(),
        "time".to_string(),
        "10".to_string(),
    ];
    args.push(
        match flow_control {
            UartFlowControl::HwFlow => "crtscts",
            UartFlowControl::NoFlow => "-crtscts",
        }
        .to_string(),
    );
    match parity {
        UartParity::ParityNone => args.push("-parenb".to_string()),
        UartParity::ParityOdd => {
            args.push("parenb".to_string());
            args.push("parodd".to_string());
        }
        UartParity::ParityEven => {
            args.push("parenb".to_string());
            args.push("-parodd".to_string());
        }
    }
    args
}

// ---------------------------------------------------------------------------
// Backend seam
// ---------------------------------------------------------------------------

/// Pluggable serial data transfer — the two operations the C++ driver does
/// with `::read`/`::write` on its file descriptor.
///
/// A downstream project that needs real `termios` control implements this
/// (or keeps [`FileSerialBackend`] and adds configuration elsewhere); the
/// component, its ports, events, telemetry and read thread do not change.
pub trait SerialBackend: Send + Sync {
    /// Open the device for reading and writing (C++
    /// `::open(device, O_RDWR | O_NOCTTY)`; `O_NOCTTY` is unavailable here).
    fn open(&self, device: &str) -> io::Result<()>;

    /// Read up to `dest.len()` bytes. `Ok(0)` is the C++ `VTIME` timeout
    /// case: the read thread retries.
    fn read(&self, dest: &mut [u8]) -> io::Result<usize>;

    /// Write `data`; the returned count is compared against `data.len()`
    /// exactly as the C++ short-write check does.
    fn write(&self, data: &[u8]) -> io::Result<usize>;

    /// Is the device open? (C++ `m_fd != -1`.)
    fn is_open(&self) -> bool;

    /// Close the device (C++ destructor). Default: nothing.
    fn close(&self) {}
}

/// Default backend: an already-configured character device opened with
/// [`std::fs::OpenOptions`] (read + write).
///
/// Reads and writes go straight to the file, so the blocking behavior is
/// whatever the tty's current `VMIN`/`VTIME` provide. On a port that was
/// never put into raw mode, `read` can return 0 immediately and repeatedly —
/// the component paces that with [`LinuxUartDriver::IDLE_READ_RETRY_US`].
#[derive(Default)]
pub struct FileSerialBackend {
    /// `Arc` so a blocking read can happen outside the mutex while `send`
    /// writes concurrently — the C++ driver likewise reads and writes the
    /// same fd from two threads.
    file: Mutex<Option<Arc<std::fs::File>>>,
}

impl FileSerialBackend {
    /// Construct a closed backend.
    #[must_use]
    pub fn new() -> Self {
        Self {
            file: Mutex::new(None),
        }
    }

    fn handle(&self) -> Option<Arc<std::fs::File>> {
        self.file.lock().unwrap().clone()
    }
}

impl SerialBackend for FileSerialBackend {
    fn open(&self, device: &str) -> io::Result<()> {
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(device)?;
        *self.file.lock().unwrap() = Some(Arc::new(file));
        Ok(())
    }

    fn read(&self, dest: &mut [u8]) -> io::Result<usize> {
        match self.handle() {
            // `impl Read for &File` — no &mut File needed, so the read runs
            // without holding the backend mutex.
            Some(file) => (&*file).read(dest),
            None => Err(io::Error::from_raw_os_error(9)), // EBADF
        }
    }

    fn write(&self, data: &[u8]) -> io::Result<usize> {
        match self.handle() {
            Some(file) => (&*file).write(data),
            None => Err(io::Error::from_raw_os_error(9)), // EBADF
        }
    }

    fn is_open(&self) -> bool {
        self.file.lock().unwrap().is_some()
    }

    fn close(&self) {
        *self.file.lock().unwrap() = None;
    }
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

/// Component state protected by the state mutex (C++ plain members set at
/// `open()` before the read thread starts).
struct UartState {
    /// Owned device path (C++ keeps the caller's pointer).
    device: String,
    /// Receive allocation size (C++ `m_allocationSize`).
    allocation_size: FwSizeType,
    /// How to deal with settings that need `termios`.
    config_policy: UartConfigPolicy,
}

impl Default for UartState {
    fn default() -> Self {
        Self {
            // C++ initialises m_device to the literal "NOT_EXIST".
            device: "NOT_EXIST".to_string(),
            allocation_size: 0,
            config_policy: UartConfigPolicy::default(),
        }
    }
}

/// `Drv::LinuxUartDriver` — passive byte-stream driver over a serial device.
pub struct LinuxUartDriver {
    /// Passive core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (`Log`/`LogText`) + `Time`.
    pub evt: EventGlue,
    /// Telemetry port (`Tlm`).
    pub tlm: TlmGlue,
    /// `allocate` — receive buffers (→ BufferManager), called on the read
    /// thread.
    pub allocate_out: OutputPort<dyn BufferGetPort>,
    /// `deallocate` — return receive buffers (→ BufferManager).
    pub deallocate_out: OutputPort<dyn BufferSendPort>,
    /// `recv` — deliver received data upstream.
    pub recv_out: OutputPort<dyn ByteStreamDataPort>,
    /// `ready` — fired once after a successful open, only when connected
    /// (C++ `isConnected_ready_OutputPort(0)`).
    pub ready_out: OutputPort<dyn ByteStreamReadyPort>,
    backend: Box<dyn SerialBackend>,
    state: Mutex<UartState>,
    /// The component (guarded-port) mutex: `send` and `recvReturnIn`.
    guard: Mutex<()>,
    bytes_sent: AtomicU64,
    bytes_received: AtomicU64,
    quit_read_thread: AtomicBool,
    read_task: Task,
    write_error_throttle: EventThrottle,
    read_error_throttle: EventThrottle,
    no_buffers_throttle: EventThrottle,
}

impl LinuxUartDriver {
    /// `OpenError(device: string size 40, error: I32, name: string size 40)`
    /// — WARNING_HI, id 0.
    ///
    /// C++ parity note: the `error` field carries the *file descriptor*, not
    /// `errno` (`-1` on the open failure, and a VALID fd on the later
    /// `tcgetattr`/`tcsetattr` failures). This port has no termios stage, so
    /// it always reports `-1`; the OS message lives in the `name` field, as
    /// upstream.
    pub const EVENTID_OPEN_ERROR: FwEventIdType = 0;
    /// `ConfigError(device: string size 40, error: I32)` — WARNING_HI, id 1.
    ///
    /// Never emitted by the C++ driver. Emitted HERE when a configuration is
    /// requested that this port cannot apply (see [`UartConfigPolicy`]).
    pub const EVENTID_CONFIG_ERROR: FwEventIdType = 1;
    /// `WriteError(device: string size 40, error: I32)` — WARNING_HI, id 2,
    /// `throttle 5`.
    pub const EVENTID_WRITE_ERROR: FwEventIdType = 2;
    /// `ReadError(device: string size 40, error: I32)` — WARNING_HI, id 3,
    /// `throttle 5`.
    pub const EVENTID_READ_ERROR: FwEventIdType = 3;
    /// `PortOpened(device: string size 40)` — ACTIVITY_HI, id 4.
    pub const EVENTID_PORT_OPENED: FwEventIdType = 4;
    /// `NoBuffers(device: string size 40)` — WARNING_HI, id 5, `throttle 20`.
    pub const EVENTID_NO_BUFFERS: FwEventIdType = 5;
    /// `BufferTooSmall(device: string size 40, size: U32, needed: U32)` —
    /// WARNING_HI, id 6. Declared but never emitted (C++ parity: the id
    /// space is preserved).
    pub const EVENTID_BUFFER_TOO_SMALL: FwEventIdType = 6;

    /// FPP `throttle 5` on `WriteError` and `ReadError`.
    pub const ERROR_EVENT_THROTTLE: u32 = 5;
    /// FPP `throttle 20` on `NoBuffers`.
    pub const NO_BUFFERS_THROTTLE: u32 = 20;

    /// `BytesSent: FwSizeType` — id 0, written every `run` (NOT on change).
    pub const CHANID_BYTES_SENT: FwChanIdType = 0;
    /// `BytesRecv: FwSizeType` — id 1, written every `run` (NOT on change).
    pub const CHANID_BYTES_RECV: FwChanIdType = 1;

    /// C++ read-thread back-off after a failed buffer allocation: 50 ms.
    pub const NO_BUFFER_RETRY_US: u32 = 50_000;

    /// Pacing for the "read returned 0" retry (µs). The C++ loop spins on
    /// `stat == 0`, relying on the 1 s `VTIME` timeout to keep that cheap;
    /// with no way to set `VTIME` from safe Rust, an unconfigured port could
    /// hot-spin, so the retry sleeps this long instead. Documented
    /// divergence.
    pub const IDLE_READ_RETRY_US: u32 = 50_000;

    /// Construct with the default [`FileSerialBackend`].
    pub fn new(name: &str) -> Arc<Self> {
        Self::with_backend(name, Box::new(FileSerialBackend::new()))
    }

    /// Construct with an explicit [`SerialBackend`] (a termios-capable
    /// implementation, or a test fake).
    pub fn with_backend(name: &str, backend: Box<dyn SerialBackend>) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            allocate_out: OutputPort::new(),
            deallocate_out: OutputPort::new(),
            recv_out: OutputPort::new(),
            ready_out: OutputPort::new(),
            backend,
            state: Mutex::new(UartState::default()),
            guard: Mutex::new(()),
            bytes_sent: AtomicU64::new(0),
            bytes_received: AtomicU64::new(0),
            quit_read_thread: AtomicBool::new(false),
            read_task: Task::new(),
            write_error_throttle: EventThrottle::new(Self::ERROR_EVENT_THROTTLE),
            read_error_throttle: EventThrottle::new(Self::ERROR_EVENT_THROTTLE),
            no_buffers_throttle: EventThrottle::new(Self::NO_BUFFERS_THROTTLE),
        })
    }

    fn id_base(&self) -> FwIdType {
        self.base.get_id_base()
    }

    fn device(&self) -> String {
        self.state.lock().unwrap().device.clone()
    }

    /// Choose how [`Self::open`] deals with settings needing `termios`.
    /// Call before `open()`.
    pub fn set_config_policy(&self, policy: UartConfigPolicy) {
        self.state.lock().unwrap().config_policy = policy;
    }

    /// The active configuration policy.
    #[must_use]
    pub fn config_policy(&self) -> UartConfigPolicy {
        self.state.lock().unwrap().config_policy
    }

    /// C++ `open(device, baud, fc, parity, allocationSize)`.
    ///
    /// Behavior depends on [`UartConfigPolicy`] (see the module docs): the
    /// default policy emits `ConfigError` and returns `false` WITHOUT
    /// opening the device, because the requested baud/parity/flow-control
    /// cannot be applied from safe Rust. On success the `PortOpened`
    /// (ACTIVITY_HI) event is emitted and `ready_out` is invoked when
    /// connected.
    pub fn open(
        &self,
        device: &str,
        baud: UartBaudRate,
        flow_control: UartFlowControl,
        parity: UartParity,
        allocation_size: FwSizeType,
    ) -> bool {
        let policy = {
            let mut state = self.state.lock().unwrap();
            state.device = device.to_string();
            state.allocation_size = allocation_size;
            state.config_policy
        };
        match policy {
            UartConfigPolicy::Reject => {
                // The one place upstream's declared-but-unused ConfigError
                // earns its id: refuse rather than run at the wrong baud.
                self.log_config_error(device, 0);
                false
            }
            UartConfigPolicy::TrustExternal => self.open_device(device),
            UartConfigPolicy::Stty => match Self::run_stty(device, baud, flow_control, parity) {
                Ok(()) => self.open_device(device),
                Err(code) => {
                    self.log_config_error(device, code);
                    false
                }
            },
        }
    }

    /// Open an ALREADY-CONFIGURED device without asking for any settings.
    ///
    /// Precondition (documented, not checkable from here): the port is
    /// already in raw mode at the right baud, e.g.
    /// `stty -F /dev/ttyUSB0 115200 raw -echo -crtscts min 0 time 10`.
    pub fn open_preconfigured(&self, device: &str, allocation_size: FwSizeType) -> bool {
        {
            let mut state = self.state.lock().unwrap();
            state.device = device.to_string();
            state.allocation_size = allocation_size;
        }
        self.open_device(device)
    }

    /// Run `stty(1)`; `Err(code)` carries the process exit code (or `-1`
    /// when `stty` could not be run or was killed by a signal).
    fn run_stty(
        device: &str,
        baud: UartBaudRate,
        flow_control: UartFlowControl,
        parity: UartParity,
    ) -> Result<(), i32> {
        let args = stty_arguments(device, baud, flow_control, parity);
        match std::process::Command::new("stty").args(&args).status() {
            Ok(status) if status.success() => Ok(()),
            Ok(status) => Err(status.code().unwrap_or(-1)),
            Err(_) => Err(-1),
        }
    }

    /// Shared open tail: open the device, emit `PortOpened`, signal `ready`.
    fn open_device(&self, device: &str) -> bool {
        match self.backend.open(device) {
            Ok(()) => {
                let arg = LogStringArg::from(device);
                self.evt.log_event(
                    self.id_base(),
                    Self::EVENTID_PORT_OPENED,
                    LogSeverity::ActivityHi,
                    &format!("UART Device {device} configured"),
                    |buf| arg.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big),
                );
                // C++ parity: an unconnected ready port must not assert.
                if let Some(p) = self.ready_out.try_get() {
                    p.target.invoke(p.port_num);
                }
                true
            }
            Err(error) => {
                let arg = LogStringArg::from(device);
                let message = LogStringArg::from(error.to_string().as_str());
                self.evt.log_event(
                    self.id_base(),
                    Self::EVENTID_OPEN_ERROR,
                    LogSeverity::WarningHi,
                    &format!("Error opening UART device {device}: -1 {error}"),
                    |buf| {
                        fw_try!(arg.serialize_to_truncated(
                            buf,
                            EVENT_STRING_SIZE,
                            Endianness::Big
                        ));
                        // C++ passes the file descriptor here, which is -1
                        // for an open failure.
                        fw_try!(buf.serialize_i32_be(-1));
                        message.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big)
                    },
                );
                false
            }
        }
    }

    fn log_config_error(&self, device: &str, error: i32) {
        let arg = LogStringArg::from(device);
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_CONFIG_ERROR,
            LogSeverity::WarningHi,
            &format!("Error configuring UART device {device}: {error}"),
            |buf| {
                fw_try!(arg.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                buf.serialize_i32_be(error)
            },
        );
    }

    /// C++ `start(priority, stackSize, cpuAffinity)`: launch the `"SerReader"`
    /// thread. A task-start failure is a `fw_assert!` (C++ `FW_ASSERT`).
    pub fn start(
        self: &Arc<Self>,
        priority: FwTaskPriorityType,
        stack_size: FwSizeType,
        cpu_affinity: FwSizeType,
    ) {
        self.quit_read_thread.store(false, Ordering::SeqCst);
        let component = self.clone();
        // C++ names the task with the literal "SerReader".
        let mut arguments = Arguments::new(
            "SerReader",
            Box::new(move || component.serial_read_task_entry()),
        );
        arguments.priority = priority;
        arguments.stack_size = stack_size;
        arguments.cpu_affinity = cpu_affinity;
        let status = self.read_task.start(arguments);
        fw_assert!(status == TaskStatus::OpOk, status as i32);
    }

    /// C++ `quitReadThread()`.
    ///
    /// The flag is only re-checked between reads: a backend blocked in a
    /// long read finishes it first (the C++ driver relies on `VTIME` to
    /// bound that at 1 s; the same caveat applies to any backend here).
    pub fn quit_read_thread(&self) {
        self.quit_read_thread.store(true, Ordering::SeqCst);
    }

    /// C++ `join()`.
    pub fn join(&self) -> TaskStatus {
        self.read_task.join()
    }

    /// C++ destructor's `close(m_fd)`.
    pub fn close(&self) {
        self.backend.close();
    }

    /// Bytes written by `send` so far (C++ `m_bytesSent`).
    #[must_use]
    pub fn bytes_sent(&self) -> FwSizeType {
        self.bytes_sent.load(Ordering::Relaxed)
    }

    /// Bytes delivered by the read thread so far (C++ `m_bytesReceived`).
    #[must_use]
    pub fn bytes_received(&self) -> FwSizeType {
        self.bytes_received.load(Ordering::Relaxed)
    }

    /// C++ `serialReadTaskEntry`, loop shape preserved exactly.
    fn serial_read_task_entry(&self) {
        while !self.quit_read_thread.load(Ordering::SeqCst) {
            let allocation_size = self.state.lock().unwrap().allocation_size;
            let p = self.allocate_out.get();
            let mut buffer = p.target.invoke(p.port_num, allocation_size);

            // C++ checks buff.getData() == nullptr.
            if !buffer.is_valid() {
                if self.no_buffers_throttle.ok_to_emit() {
                    let device = self.device();
                    let arg = LogStringArg::from(device.as_str());
                    self.evt.log_event(
                        self.id_base(),
                        Self::EVENTID_NO_BUFFERS,
                        LogSeverity::WarningHi,
                        &format!("UART Device {device} ran out of buffers"),
                        |buf| arg.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big),
                    );
                }
                let p = self.recv_out.get();
                p.target
                    .invoke(p.port_num, buffer, ByteStreamStatus::OtherError);
                // C++: avoid spinning, wait 50 ms.
                let _ = Task::delay(TimeInterval::new(0, Self::NO_BUFFER_RETRY_US));
                continue;
            }

            // Read until something arrives, an error occurs, or quit is
            // requested (C++ `while ((stat == 0) && !m_quitReadThread)`).
            let mut result: io::Result<usize> = Ok(0);
            while matches!(result, Ok(0)) && !self.quit_read_thread.load(Ordering::SeqCst) {
                result = self.backend.read(buffer.data_mut());
                if matches!(result, Ok(0)) && !self.quit_read_thread.load(Ordering::SeqCst) {
                    // Documented divergence: pace the retry, since VMIN/VTIME
                    // cannot be applied from safe Rust.
                    let _ = Task::delay(TimeInterval::new(0, Self::IDLE_READ_RETRY_US));
                }
            }
            buffer.set_size(0);

            let status = match result {
                Err(_) => {
                    if self.read_error_throttle.ok_to_emit() {
                        let device = self.device();
                        let arg = LogStringArg::from(device.as_str());
                        self.evt.log_event(
                            self.id_base(),
                            Self::EVENTID_READ_ERROR,
                            LogSeverity::WarningHi,
                            &format!("Error reading UART device {device}: -1"),
                            |buf| {
                                fw_try!(arg.serialize_to_truncated(
                                    buf,
                                    EVENT_STRING_SIZE,
                                    Endianness::Big
                                ));
                                // C++ passes the ssize_t read() result: -1.
                                buf.serialize_i32_be(-1)
                            },
                        );
                    }
                    ByteStreamStatus::OtherError
                }
                Ok(0) => {
                    // Quit requested: OTHER_ERROR simply returns the buffer.
                    ByteStreamStatus::OtherError
                }
                Ok(count) => {
                    buffer.set_size(count);
                    self.bytes_received
                        .fetch_add(count as FwSizeType, Ordering::Relaxed);
                    ByteStreamStatus::OpOk
                }
            };
            let p = self.recv_out.get();
            p.target.invoke(p.port_num, buffer, status);
        }
    }

    // -- Handlers ------------------------------------------------------------

    /// C++ `send_handler`: a closed device, an invalid buffer or a zero-size
    /// buffer is `OTHER_ERROR` with NO event; a failed or short write is a
    /// throttled `WriteError` plus `OTHER_ERROR`. Buffer ownership stays
    /// with the caller.
    fn send_handler(&self, _port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus {
        let _guard = self.guard.lock().unwrap();
        if !self.backend.is_open() || !buffer.is_valid() || buffer.size() == 0 {
            return ByteStreamStatus::OtherError;
        }
        let requested = buffer.size();
        let (status, reported) = match self.backend.write(buffer.data()) {
            Ok(count) if count == requested => {
                self.bytes_sent
                    .fetch_add(count as FwSizeType, Ordering::Relaxed);
                (ByteStreamStatus::OpOk, 0)
            }
            // C++ reports the ssize_t result: the short count, or -1.
            Ok(count) => (ByteStreamStatus::OtherError, count as i32),
            Err(_) => (ByteStreamStatus::OtherError, -1),
        };
        if status != ByteStreamStatus::OpOk && self.write_error_throttle.ok_to_emit() {
            let device = self.device();
            let arg = LogStringArg::from(device.as_str());
            self.evt.log_event(
                self.id_base(),
                Self::EVENTID_WRITE_ERROR,
                LogSeverity::WarningHi,
                &format!("Error writing UART device {device}: {reported}"),
                |buf| {
                    fw_try!(arg.serialize_to_truncated(buf, EVENT_STRING_SIZE, Endianness::Big));
                    buf.serialize_i32_be(reported)
                },
            );
        }
        status
    }

    /// C++ `recvReturnIn_handler`: unconditionally forward to
    /// `deallocate_out(0, ...)`.
    fn recv_return_in_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        let _guard = self.guard.lock().unwrap();
        let p = self.deallocate_out.get();
        p.target.invoke(p.port_num, buffer);
    }

    /// C++ `run_handler`: write both counters unconditionally (no on-change
    /// suppression), `BytesSent` first.
    fn run_handler(&self, _port_num: FwIndexType, _context: u32) {
        let time_tag = self.evt.time_get();
        let sent = self.bytes_sent.load(Ordering::Relaxed);
        let received = self.bytes_received.load(Ordering::Relaxed);
        self.tlm
            .tlm_write(self.id_base(), Self::CHANID_BYTES_SENT, &sent, time_tag);
        self.tlm
            .tlm_write(self.id_base(), Self::CHANID_BYTES_RECV, &received, time_tag);
    }
}

fprime_comp::input_port_adapter! {
    /// `send` — GUARDED `Drv.ByteStreamSend` input (caller's thread, under
    /// the component mutex).
    component: LinuxUartDriver;
    adapter: SendAdapter;
    port: ByteStreamSendPort;
    input: pub send_in;
    handler: send_handler;
    returns: ByteStreamStatus;
    args { mut buffer: Buffer }
}

fprime_comp::input_port_adapter! {
    /// `recvReturnIn` — GUARDED `Fw.BufferSend` input: receive buffers come
    /// back here and are forwarded to `deallocate_out`.
    component: LinuxUartDriver;
    adapter: RecvReturnAdapter;
    port: BufferSendPort;
    input: pub recv_return_in;
    handler: recv_return_in_handler;
    args { val buffer: Buffer }
}

fprime_comp::input_port_adapter! {
    /// `run` — SYNC `Svc.Sched` input: telemetry only.
    component: LinuxUartDriver;
    adapter: RunAdapter;
    port: SchedPort;
    input: pub run_in;
    handler: run_handler;
    args { val context: u32 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{LogPort, TlmPort};
    use fprime_fw::{LogBuffer, Time, TlmBuffer};
    use std::sync::atomic::AtomicU32;
    use std::time::Duration;

    // -- scaffolding ---------------------------------------------------------

    struct TempPath {
        path: std::path::PathBuf,
    }

    impl TempPath {
        fn new(tag: &str) -> Self {
            static COUNTER: AtomicU32 = AtomicU32::new(0);
            let unique = COUNTER.fetch_add(1, Ordering::SeqCst);
            let path = std::env::temp_dir()
                .join(format!("fprime-uart-{tag}-{}-{unique}", std::process::id()));
            let _ = std::fs::remove_file(&path);
            Self { path }
        }
        fn as_str(&self) -> &str {
            self.path.to_str().unwrap()
        }
    }

    impl Drop for TempPath {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.path);
        }
    }

    #[derive(Default)]
    struct EventCollector {
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
    }

    impl LogPort for EventCollector {
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
        fn count_of(&self, id: FwEventIdType) -> usize {
            self.events
                .lock()
                .unwrap()
                .iter()
                .filter(|e| e.0 == id)
                .count()
        }
        fn find(&self, id: FwEventIdType) -> Option<(FwEventIdType, LogSeverity, Vec<u8>)> {
            self.events
                .lock()
                .unwrap()
                .iter()
                .find(|e| e.0 == id)
                .cloned()
        }
    }

    #[derive(Default)]
    struct TlmCollector {
        writes: Mutex<Vec<(FwChanIdType, Vec<u8>)>>,
    }

    impl TlmPort for TlmCollector {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwChanIdType,
            _time_tag: &mut Time,
            val: &mut TlmBuffer,
        ) {
            self.writes
                .lock()
                .unwrap()
                .push((id, val.as_slice().to_vec()));
        }
    }

    /// Allocator stub: hands out real buffers, or an invalid one when
    /// `fail` is set; counts deallocations.
    #[derive(Default)]
    struct BufferStub {
        fail: AtomicBool,
        allocated: AtomicU32,
        deallocated: AtomicU32,
        last_size: AtomicU64,
    }

    impl BufferGetPort for BufferStub {
        fn invoke(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
            self.last_size.store(size, Ordering::SeqCst);
            if self.fail.load(Ordering::SeqCst) {
                return Buffer::empty();
            }
            self.allocated.fetch_add(1, Ordering::SeqCst);
            Buffer::allocate(size as usize)
        }
    }

    impl BufferSendPort for BufferStub {
        fn invoke(&self, _port_num: FwIndexType, _buffer: Buffer) {
            self.deallocated.fetch_add(1, Ordering::SeqCst);
        }
    }

    /// Collects everything delivered on `recv`.
    #[derive(Default)]
    struct RecvSink {
        received: Mutex<Vec<(Vec<u8>, ByteStreamStatus)>>,
    }

    impl ByteStreamDataPort for RecvSink {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer, status: ByteStreamStatus) {
            self.received
                .lock()
                .unwrap()
                .push((buffer.data().to_vec(), status));
        }
    }

    #[derive(Default)]
    struct ReadySink {
        count: AtomicU32,
    }

    impl ByteStreamReadyPort for ReadySink {
        fn invoke(&self, _port_num: FwIndexType) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    /// Scriptable serial backend.
    #[derive(Default)]
    struct FakeSerial {
        open_error: Mutex<Option<i32>>,
        opened: AtomicBool,
        opened_device: Mutex<String>,
        /// Scripted read results, consumed front to back; `Ok(0)`
        /// afterwards.
        reads: Mutex<Vec<io::Result<Vec<u8>>>>,
        written: Mutex<Vec<u8>>,
        /// `Some(Ok(n))` forces a short write, `Some(Err(e))` a failure.
        write_override: Mutex<Option<io::Result<usize>>>,
    }

    impl SerialBackend for Arc<FakeSerial> {
        fn open(&self, device: &str) -> io::Result<()> {
            if let Some(errno) = *self.open_error.lock().unwrap() {
                return Err(io::Error::from_raw_os_error(errno));
            }
            *self.opened_device.lock().unwrap() = device.to_string();
            self.opened.store(true, Ordering::SeqCst);
            Ok(())
        }

        fn read(&self, dest: &mut [u8]) -> io::Result<usize> {
            let mut reads = self.reads.lock().unwrap();
            if reads.is_empty() {
                return Ok(0);
            }
            match reads.remove(0) {
                Ok(data) => {
                    let count = data.len().min(dest.len());
                    dest[..count].copy_from_slice(&data[..count]);
                    Ok(count)
                }
                Err(error) => Err(error),
            }
        }

        fn write(&self, data: &[u8]) -> io::Result<usize> {
            if let Some(result) = self.write_override.lock().unwrap().take() {
                return result;
            }
            self.written.lock().unwrap().extend_from_slice(data);
            Ok(data.len())
        }

        fn is_open(&self) -> bool {
            self.opened.load(Ordering::SeqCst)
        }

        fn close(&self) {
            self.opened.store(false, Ordering::SeqCst);
        }
    }

    struct Harness {
        driver: Arc<LinuxUartDriver>,
        fake: Arc<FakeSerial>,
        events: Arc<EventCollector>,
        tlm: Arc<TlmCollector>,
        buffers: Arc<BufferStub>,
        recv: Arc<RecvSink>,
        ready: Arc<ReadySink>,
    }

    fn harness() -> Harness {
        let fake = Arc::new(FakeSerial::default());
        let driver = LinuxUartDriver::with_backend("uartDrv", Box::new(fake.clone()));
        let events = Arc::new(EventCollector::default());
        let tlm = Arc::new(TlmCollector::default());
        let buffers = Arc::new(BufferStub::default());
        let recv = Arc::new(RecvSink::default());
        let ready = Arc::new(ReadySink::default());
        driver.evt.log_out.connect(events.clone(), 0);
        driver.tlm.tlm_out.connect(tlm.clone(), 0);
        driver.allocate_out.connect(buffers.clone(), 0);
        driver.deallocate_out.connect(buffers.clone(), 0);
        driver.recv_out.connect(recv.clone(), 0);
        driver.ready_out.connect(ready.clone(), 0);
        Harness {
            driver,
            fake,
            events,
            tlm,
            buffers,
            recv,
            ready,
        }
    }

    // -- configuration policy -------------------------------------------------

    #[test]
    fn stty_arguments_are_exact() {
        assert_eq!(
            stty_arguments(
                "/dev/ttyUSB0",
                UartBaudRate::Baud115K,
                UartFlowControl::NoFlow,
                UartParity::ParityNone
            ),
            vec![
                "-F",
                "/dev/ttyUSB0",
                "115200",
                "cs8",
                "clocal",
                "cread",
                "raw",
                "-echo",
                "min",
                "0",
                "time",
                "10",
                "-crtscts",
                "-parenb",
            ]
        );
        let odd = stty_arguments(
            "/dev/ttyS1",
            UartBaudRate::Baud9600,
            UartFlowControl::HwFlow,
            UartParity::ParityOdd,
        );
        assert_eq!(&odd[2], "9600");
        assert_eq!(&odd[odd.len() - 3..], ["crtscts", "parenb", "parodd"]);
        let even = stty_arguments(
            "/dev/ttyS1",
            UartBaudRate::Baud230K,
            UartFlowControl::NoFlow,
            UartParity::ParityEven,
        );
        assert_eq!(&even[even.len() - 2..], ["parenb", "-parodd"]);
    }

    #[test]
    fn baud_enum_values_are_the_baud_numbers() {
        assert_eq!(UartBaudRate::Baud9600.as_u32(), 9600);
        assert_eq!(UartBaudRate::Baud115K.as_u32(), 115_200);
        assert_eq!(UartBaudRate::Baud230K.as_u32(), 230_400);
        assert_eq!(UartBaudRate::Baud4000K.as_u32(), 4_000_000);
        assert_eq!(UartFlowControl::NoFlow as u8, 0);
        assert_eq!(UartFlowControl::HwFlow as u8, 1);
        assert_eq!(UartParity::ParityNone as u8, 0);
        assert_eq!(UartParity::ParityOdd as u8, 1);
        assert_eq!(UartParity::ParityEven as u8, 2);
    }

    #[test]
    fn default_policy_rejects_configuration_and_does_not_open() {
        let h = harness();
        assert_eq!(h.driver.config_policy(), UartConfigPolicy::Reject);
        assert!(!h.driver.open(
            "/dev/ttyUSB0",
            UartBaudRate::Baud115K,
            UartFlowControl::NoFlow,
            UartParity::ParityNone,
            512
        ));
        assert!(!h.fake.opened.load(Ordering::SeqCst));
        let (_, severity, bytes) = h
            .events
            .find(LinuxUartDriver::EVENTID_CONFIG_ERROR)
            .expect("ConfigError");
        assert_eq!(severity, LogSeverity::WarningHi);
        let mut expected = vec![0, 12];
        expected.extend_from_slice(b"/dev/ttyUSB0");
        expected.extend_from_slice(&[0, 0, 0, 0]);
        assert_eq!(bytes, expected);
    }

    #[test]
    fn trust_external_policy_opens_and_emits_port_opened() {
        let h = harness();
        h.driver.set_config_policy(UartConfigPolicy::TrustExternal);
        assert!(h.driver.open(
            "/dev/ttyUSB0",
            UartBaudRate::Baud115K,
            UartFlowControl::HwFlow,
            UartParity::ParityEven,
            256
        ));
        assert_eq!(*h.fake.opened_device.lock().unwrap(), "/dev/ttyUSB0");
        let (_, severity, bytes) = h
            .events
            .find(LinuxUartDriver::EVENTID_PORT_OPENED)
            .expect("PortOpened");
        assert_eq!(severity, LogSeverity::ActivityHi);
        let mut expected = vec![0, 12];
        expected.extend_from_slice(b"/dev/ttyUSB0");
        assert_eq!(bytes, expected);
        assert_eq!(h.ready.count.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn open_failure_emits_open_error_with_minus_one_and_the_os_message() {
        let h = harness();
        *h.fake.open_error.lock().unwrap() = Some(2); // ENOENT
        assert!(!h.driver.open_preconfigured("/dev/nope", 64));
        let (_, severity, bytes) = h
            .events
            .find(LinuxUartDriver::EVENTID_OPEN_ERROR)
            .expect("OpenError");
        assert_eq!(severity, LogSeverity::WarningHi);
        // [u16 len]"/dev/nope" [i32 -1] [u16 len]<os message>
        assert_eq!(&bytes[..2], &[0, 9]);
        assert_eq!(&bytes[2..11], b"/dev/nope");
        assert_eq!(&bytes[11..15], &[0xFF, 0xFF, 0xFF, 0xFF]);
        let message_len = u16::from_be_bytes([bytes[15], bytes[16]]) as usize;
        assert_eq!(bytes.len(), 17 + message_len);
        assert_eq!(h.ready.count.load(Ordering::SeqCst), 0);
    }

    #[test]
    fn unconnected_ready_port_does_not_assert() {
        let fake = Arc::new(FakeSerial::default());
        let driver = LinuxUartDriver::with_backend("noReady", Box::new(fake.clone()));
        // ready_out deliberately left unconnected (C++
        // isConnected_ready_OutputPort guard).
        assert!(driver.open_preconfigured("/dev/ttyUSB0", 32));
    }

    // -- send ----------------------------------------------------------------

    #[test]
    fn send_writes_the_buffer_and_counts_bytes() {
        let h = harness();
        assert!(h.driver.open_preconfigured("/dev/ttyUSB0", 64));
        let mut buffer = Buffer::allocate(4);
        buffer.data_mut().copy_from_slice(b"ping");
        let port = h.driver.send_in(0);
        assert_eq!(
            port.target.invoke(port.port_num, &mut buffer),
            ByteStreamStatus::OpOk
        );
        assert_eq!(&*h.fake.written.lock().unwrap(), b"ping");
        assert_eq!(h.driver.bytes_sent(), 4);
        assert!(
            !h.events
                .ids()
                .contains(&LinuxUartDriver::EVENTID_WRITE_ERROR)
        );
    }

    #[test]
    fn send_on_closed_device_or_empty_buffer_is_other_error_without_event() {
        let h = harness();
        let mut buffer = Buffer::allocate(4);
        let port = h.driver.send_in(0);
        // Not open.
        assert_eq!(
            port.target.invoke(port.port_num, &mut buffer),
            ByteStreamStatus::OtherError
        );
        assert!(h.driver.open_preconfigured("/dev/ttyUSB0", 64));
        // Zero-size buffer.
        buffer.set_size(0);
        assert_eq!(
            port.target.invoke(port.port_num, &mut buffer),
            ByteStreamStatus::OtherError
        );
        // Invalid buffer.
        let mut invalid = Buffer::empty();
        assert_eq!(
            port.target.invoke(port.port_num, &mut invalid),
            ByteStreamStatus::OtherError
        );
        assert_eq!(h.events.count_of(LinuxUartDriver::EVENTID_WRITE_ERROR), 0);
        assert_eq!(h.driver.bytes_sent(), 0);
    }

    #[test]
    fn short_write_reports_the_written_count_in_write_error() {
        let h = harness();
        assert!(h.driver.open_preconfigured("/dev/ttyUSB0", 64));
        *h.fake.write_override.lock().unwrap() = Some(Ok(2));
        let mut buffer = Buffer::allocate(4);
        buffer.data_mut().copy_from_slice(b"ping");
        let port = h.driver.send_in(0);
        assert_eq!(
            port.target.invoke(port.port_num, &mut buffer),
            ByteStreamStatus::OtherError
        );
        let (_, _, bytes) = h
            .events
            .find(LinuxUartDriver::EVENTID_WRITE_ERROR)
            .expect("WriteError");
        let mut expected = vec![0, 12];
        expected.extend_from_slice(b"/dev/ttyUSB0");
        expected.extend_from_slice(&2i32.to_be_bytes());
        assert_eq!(bytes, expected);
        assert_eq!(h.driver.bytes_sent(), 0);
    }

    #[test]
    fn failed_write_reports_minus_one_and_throttles_at_five() {
        let h = harness();
        assert!(h.driver.open_preconfigured("/dev/ttyUSB0", 64));
        let mut buffer = Buffer::allocate(4);
        buffer.data_mut().copy_from_slice(b"ping");
        let port = h.driver.send_in(0);
        for _ in 0..8 {
            *h.fake.write_override.lock().unwrap() = Some(Err(io::Error::from_raw_os_error(5)));
            assert_eq!(
                port.target.invoke(port.port_num, &mut buffer),
                ByteStreamStatus::OtherError
            );
        }
        // FPP `throttle 5`.
        assert_eq!(
            h.events.count_of(LinuxUartDriver::EVENTID_WRITE_ERROR),
            LinuxUartDriver::ERROR_EVENT_THROTTLE as usize
        );
        let (_, _, bytes) = h.events.find(LinuxUartDriver::EVENTID_WRITE_ERROR).unwrap();
        assert_eq!(&bytes[bytes.len() - 4..], &[0xFF, 0xFF, 0xFF, 0xFF]);
    }

    // -- recvReturnIn / run ---------------------------------------------------

    #[test]
    fn recv_return_forwards_to_deallocate() {
        let h = harness();
        let port = h.driver.recv_return_in(0);
        port.target.invoke(port.port_num, Buffer::allocate(8));
        assert_eq!(h.buffers.deallocated.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn run_writes_both_counters_unconditionally_as_u64() {
        let h = harness();
        let port = h.driver.run_in(0);
        port.target.invoke(port.port_num, 0);
        port.target.invoke(port.port_num, 0);
        let writes = h.tlm.writes.lock().unwrap().clone();
        assert_eq!(writes.len(), 4);
        assert_eq!(writes[0].0, LinuxUartDriver::CHANID_BYTES_SENT);
        assert_eq!(writes[0].1, vec![0, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(writes[1].0, LinuxUartDriver::CHANID_BYTES_RECV);
        // Not on-change: the same values are written again.
        assert_eq!(writes[2].0, LinuxUartDriver::CHANID_BYTES_SENT);
        assert_eq!(writes[3].0, LinuxUartDriver::CHANID_BYTES_RECV);
    }

    #[test]
    fn run_reports_the_accumulated_counters() {
        let h = harness();
        assert!(h.driver.open_preconfigured("/dev/ttyUSB0", 64));
        let mut buffer = Buffer::allocate(3);
        buffer.data_mut().copy_from_slice(b"abc");
        let send = h.driver.send_in(0);
        assert_eq!(
            send.target.invoke(send.port_num, &mut buffer),
            ByteStreamStatus::OpOk
        );
        let run = h.driver.run_in(0);
        run.target.invoke(run.port_num, 0);
        let writes = h.tlm.writes.lock().unwrap().clone();
        assert_eq!(writes[0].1, 3u64.to_be_bytes().to_vec());
    }

    // -- read thread ----------------------------------------------------------

    fn wait_for<F: Fn() -> bool>(condition: F) -> bool {
        for _ in 0..500 {
            if condition() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        false
    }

    #[test]
    fn read_thread_delivers_data_with_op_ok() {
        let h = harness();
        h.fake.reads.lock().unwrap().push(Ok(b"hello".to_vec()));
        assert!(h.driver.open_preconfigured("/dev/ttyUSB0", 32));
        h.driver.start(100, 0, 0);
        assert!(wait_for(|| !h.recv.received.lock().unwrap().is_empty()));
        h.driver.quit_read_thread();
        assert_eq!(h.driver.join(), TaskStatus::OpOk);

        let received = h.recv.received.lock().unwrap().clone();
        assert_eq!(received[0].0, b"hello".to_vec());
        assert_eq!(received[0].1, ByteStreamStatus::OpOk);
        assert_eq!(h.driver.bytes_received(), 5);
        assert_eq!(h.buffers.last_size.load(Ordering::SeqCst), 32);
    }

    #[test]
    fn read_error_delivers_a_zero_size_buffer_with_other_error() {
        let h = harness();
        h.fake
            .reads
            .lock()
            .unwrap()
            .push(Err(io::Error::from_raw_os_error(5)));
        assert!(h.driver.open_preconfigured("/dev/ttyUSB0", 16));
        h.driver.start(100, 0, 0);
        assert!(wait_for(|| !h.recv.received.lock().unwrap().is_empty()));
        h.driver.quit_read_thread();
        assert_eq!(h.driver.join(), TaskStatus::OpOk);

        let received = h.recv.received.lock().unwrap().clone();
        assert!(received[0].0.is_empty());
        assert_eq!(received[0].1, ByteStreamStatus::OtherError);
        let (_, severity, bytes) = h
            .events
            .find(LinuxUartDriver::EVENTID_READ_ERROR)
            .expect("ReadError");
        assert_eq!(severity, LogSeverity::WarningHi);
        let mut expected = vec![0, 12];
        expected.extend_from_slice(b"/dev/ttyUSB0");
        expected.extend_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF]);
        assert_eq!(bytes, expected);
        assert_eq!(h.driver.bytes_received(), 0);
    }

    #[test]
    fn failed_allocation_emits_no_buffers_and_returns_the_invalid_buffer() {
        let h = harness();
        h.buffers.fail.store(true, Ordering::SeqCst);
        assert!(h.driver.open_preconfigured("/dev/ttyUSB0", 16));
        h.driver.start(100, 0, 0);
        assert!(wait_for(|| !h.recv.received.lock().unwrap().is_empty()));
        h.driver.quit_read_thread();
        assert_eq!(h.driver.join(), TaskStatus::OpOk);

        let received = h.recv.received.lock().unwrap().clone();
        assert_eq!(received[0].1, ByteStreamStatus::OtherError);
        assert!(received[0].0.is_empty());
        let (_, severity, bytes) = h
            .events
            .find(LinuxUartDriver::EVENTID_NO_BUFFERS)
            .expect("NoBuffers");
        assert_eq!(severity, LogSeverity::WarningHi);
        let mut expected = vec![0, 12];
        expected.extend_from_slice(b"/dev/ttyUSB0");
        assert_eq!(bytes, expected);
        // FPP `throttle 20` — never more than 20 however long it spins.
        assert!(h.events.count_of(LinuxUartDriver::EVENTID_NO_BUFFERS) <= 20);
    }

    #[test]
    fn quit_during_the_idle_retry_returns_the_buffer_with_other_error() {
        let h = harness();
        // No scripted reads: every read returns Ok(0), the C++ timeout case.
        assert!(h.driver.open_preconfigured("/dev/ttyUSB0", 8));
        h.driver.start(100, 0, 0);
        std::thread::sleep(Duration::from_millis(20));
        h.driver.quit_read_thread();
        assert_eq!(h.driver.join(), TaskStatus::OpOk);
        let received = h.recv.received.lock().unwrap().clone();
        assert_eq!(received.len(), 1);
        assert!(received[0].0.is_empty());
        assert_eq!(received[0].1, ByteStreamStatus::OtherError);
    }

    // -- file backend ---------------------------------------------------------

    #[test]
    fn file_backend_round_trips_through_a_real_file() {
        let temp = TempPath::new("io");
        std::fs::write(&temp.path, b"").unwrap();
        let backend = FileSerialBackend::new();
        assert!(!backend.is_open());
        backend.open(temp.as_str()).expect("open");
        assert!(backend.is_open());
        assert_eq!(backend.write(b"0123456789").unwrap(), 10);

        // Re-open to read from the start (the write cursor is at EOF).
        let reader = FileSerialBackend::new();
        reader.open(temp.as_str()).expect("open");
        let mut dest = [0u8; 4];
        assert_eq!(reader.read(&mut dest).unwrap(), 4);
        assert_eq!(&dest, b"0123");
        assert_eq!(reader.read(&mut dest).unwrap(), 4);
        assert_eq!(&dest, b"4567");
        // EOF behaves like the VTIME timeout: a zero-length read.
        let mut rest = [0u8; 8];
        assert_eq!(reader.read(&mut rest).unwrap(), 2);
        assert_eq!(reader.read(&mut rest).unwrap(), 0);

        backend.close();
        assert!(!backend.is_open());
        // A closed backend reports EBADF, exactly like the C++ fd == -1 path.
        assert_eq!(backend.write(b"x").unwrap_err().raw_os_error(), Some(9));
        assert_eq!(backend.read(&mut dest).unwrap_err().raw_os_error(), Some(9));
    }

    #[test]
    fn file_backend_open_failure_is_reported() {
        let temp = TempPath::new("missing");
        let backend = FileSerialBackend::new();
        assert!(backend.open(temp.as_str()).is_err());
        assert!(!backend.is_open());
    }

    #[test]
    fn driver_over_the_file_backend_sends_bytes_to_the_device() {
        let temp = TempPath::new("driver");
        std::fs::write(&temp.path, b"").unwrap();
        let driver = LinuxUartDriver::with_backend("uart", Box::new(FileSerialBackend::new()));
        assert!(driver.open_preconfigured(temp.as_str(), 64));
        let mut buffer = Buffer::allocate(5);
        buffer.data_mut().copy_from_slice(b"frame");
        let port = driver.send_in(0);
        assert_eq!(
            port.target.invoke(port.port_num, &mut buffer),
            ByteStreamStatus::OpOk
        );
        driver.close();
        assert_eq!(std::fs::read(&temp.path).unwrap(), b"frame");
    }
}
