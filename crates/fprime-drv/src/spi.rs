//! # LinuxSpiDriver — port of `Drv::LinuxSpiDriverComponentImpl`
//! (+ `Drv/Interfaces/Spi`, `Drv/Ports/SpiDriverPorts`)
//!
//! C++ sources: `Drv/LinuxSpiDriver/LinuxSpiDriver.fpp`,
//! `LinuxSpiDriverComponentImpl.{hpp,cpp}`,
//! `LinuxSpiDriverComponentImplCommon.cpp`,
//! `LinuxSpiDriverComponentImplStub.cpp`, `Events.fppi`, `Telemetry.fppi`,
//! `Drv/Ports/SpiDriverPorts.fpp`, `Drv/Interfaces/Spi.fpp`.
//! Analysis: `docs/cpp-analysis/linux-drivers.md`.
//!
//! PASSIVE component implementing the `Drv.Spi` interface — a GUARDED
//! `SpiWriteRead` input returning [`SpiStatus`], and the DEPRECATED sync
//! `SpiReadWrite` input with no return value — plus the
//! `Log`/`LogText`/`Tlm`/`Time` special ports.
//!
//! ## UNSUPPORTED: no full-duplex backend, and no configuration
//!
//! A full-duplex SPI transfer is `ioctl(fd, SPI_IOC_MESSAGE(1), &spi_ioc_transfer)`,
//! and mode / bits-per-word / max-speed are the `SPI_IOC_WR_*` and
//! `SPI_IOC_RD_*` ioctls. `std` issues no ioctls, so a real backend needs an
//! `unsafe extern` FFI shim or a `libc`/`nix` dependency — both excluded by
//! this workspace. The component surface is ported in full over the
//! [`SpiBackend`] seam, and the DEFAULT backend is [`StubSpiBackend`]
//! (mirroring `LinuxSpiDriverComponentImplStub.cpp`).
//!
//! [`HalfDuplexSpidevBackend`] exists as an explicitly-named, opt-in
//! alternative — constructed only through
//! [`HalfDuplexSpidevBackend::new_write_then_read_not_full_duplex`], because
//! it is **NOT** the operation `SpiWriteRead` promises: it `write()`s the
//! transmit buffer (receive data discarded) and then `read()`s the receive
//! buffer (transmit zeros), two separate bus transactions with the chip
//! select released in between. Devices that latch data on the same clock
//! edges as the command — most of them — will not work. Nor can it set or
//! read back mode/speed/bits (they come from the device tree), so
//! `SPI_ConfigError` and `SPI_ConfigMismatch` are unreachable with it.
//!
//! ## Documented divergences
//!
//! - C++ `SpiReadWrite` is a `sync` (UNGUARDED) port that calls straight into
//!   the guarded handler's body, so concurrent use of both ports races on
//!   `m_fd`/`m_bytes`. Here both ports go through the same mutex-locked
//!   handler — a deliberate behavior improvement.
//! - The C++ stub returns `SPI_OK` from `SpiWriteRead` even though its
//!   `open()` returns `false`. Here the component's real open-gate applies,
//!   so a driver backed by [`StubSpiBackend`] answers
//!   [`SpiStatus::SpiOpenErr`]; the stub *backend's* transfer still reports
//!   success, matching upstream at the seam.
//! - `SPI_PortOpened` (event id 4) is declared and NEVER emitted, exactly as
//!   upstream; the id is reserved.

use fprime_comp::{EventGlue, EventThrottle, PassiveBase, TlmGlue};
use fprime_config::{FwChanIdType, FwEventIdType, FwIdType, FwIndexType, FwSizeType};
use fprime_fw::{
    Buffer, Endianness, LogSeverity, LogStringArg, SerBuf, fpp_enum, fw_assert, fw_try,
};
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

/// `SPI_ConfigMismatch`'s `parameter` argument is a bare FPP `string`, which
/// defaults to size 80.
const EVENT_STRING_SIZE: usize = 80;

fpp_enum! {
    /// `Drv::SpiStatus` — FPP enum, `repr U8`
    /// (`Drv/Ports/SpiDriverPorts.fpp`, explicit values).
    ///
    /// Gotcha (C++ parity): only `SPI_OK`, `SPI_OPEN_ERR` and `SPI_OTHER_ERR`
    /// are ever returned by any code path. `SpiConfigErr`, `SpiMismatchErr`
    /// and `SpiWriteErr` are dead values kept for wire compatibility.
    pub enum SpiStatus : u8 {
        /// Transaction okay.
        SpiOk = 0,
        /// SPI driver failed to open the device.
        SpiOpenErr = 1,
        /// Configuration failure (never returned).
        SpiConfigErr = 2,
        /// Configuration read-back mismatch (never returned).
        SpiMismatchErr = 3,
        /// Write failure (never returned).
        SpiWriteErr = 4,
        /// Other errors that do not fit — what a failed transfer returns.
        SpiOtherErr = 5,
    }
    default SpiOk
}

/// C++ `Drv::SpiFrequency` (a plain C++ enum, not FPP): the clock rate in Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum SpiFrequency {
    /// 1 MHz.
    SpiFrequency1Mhz = 1_000_000,
    /// 5 MHz.
    SpiFrequency5Mhz = 5_000_000,
    /// 10 MHz.
    SpiFrequency10Mhz = 10_000_000,
    /// 15 MHz.
    SpiFrequency15Mhz = 15_000_000,
    /// 20 MHz.
    SpiFrequency20Mhz = 20_000_000,
}

impl SpiFrequency {
    /// The clock rate in Hz (the enum's own discriminant).
    #[must_use]
    pub const fn as_hz(self) -> u32 {
        self as u32
    }
}

/// C++ `Drv::SpiMode` — clock polarity/phase, mapping onto the kernel
/// `SPI_MODE_0..3` bits (`SPI_CPHA = 0x1`, `SPI_CPOL = 0x2`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(u8)]
pub enum SpiMode {
    /// CPOL = 0, CPHA = 0 (kernel `SPI_MODE_0`).
    #[default]
    SpiModeCpolLowCphaLow = 0,
    /// CPOL = 0, CPHA = 1 (kernel `SPI_MODE_1`).
    SpiModeCpolLowCphaHigh = 1,
    /// CPOL = 1, CPHA = 0 (kernel `SPI_MODE_2`).
    SpiModeCpolHighCphaLow = 2,
    /// CPOL = 1, CPHA = 1 (kernel `SPI_MODE_3`).
    SpiModeCpolHighCphaHigh = 3,
}

impl SpiMode {
    /// The kernel mode bits written with `SPI_IOC_WR_MODE`.
    #[must_use]
    pub const fn as_kernel_mode(self) -> u8 {
        self as u8
    }
}

/// `Drv.SpiWriteRead` port:
/// `SpiWriteRead(ref writeBuffer, ref readBuffer) -> SpiStatus`.
///
/// Both buffers are caller-owned C++ `ref`s and MUST be the same size — a
/// full-duplex transfer clocks out and in simultaneously.
pub trait SpiWriteReadPort: Send + Sync {
    /// Perform one full-duplex transfer.
    fn invoke(
        &self,
        port_num: FwIndexType,
        write_buffer: &mut Buffer,
        read_buffer: &mut Buffer,
    ) -> SpiStatus;
}

/// `Drv.SpiReadWrite` port — **DEPRECATED** upstream: the same operation
/// without a return value. Use [`SpiWriteReadPort`].
pub trait SpiReadWritePort: Send + Sync {
    /// Perform one full-duplex transfer, discarding the status.
    fn invoke(&self, port_num: FwIndexType, write_buffer: &mut Buffer, read_buffer: &mut Buffer);
}

// ---------------------------------------------------------------------------
// Backend seam
// ---------------------------------------------------------------------------

/// One `SPI_IOC_WR_*` / `SPI_IOC_RD_*` read-back disagreement, reported by
/// [`SpiBackend::open`] and turned into a WARNING_LO `SPI_ConfigMismatch`
/// event. A mismatch does NOT abort the open (C++ parity: the device is used
/// with whatever the kernel accepted).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpiConfigMismatch {
    /// The parameter name, exactly as upstream spells it: `"MODE"`,
    /// `"BITS_PER_WORD"` or `"MAX_SPEED_HZ"`.
    pub parameter: String,
    /// Value written.
    pub write_value: u32,
    /// Value read back.
    pub read_value: u32,
}

/// Why [`SpiBackend::open`] failed — selects which event the component
/// emits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiOpenError {
    /// `::open()` failed: `SPI_OpenError(device, select, error)`.
    ///
    /// C++ passes the file descriptor (always `-1`) as the error code, not
    /// `errno`; backends here should do the same for parity.
    Open(i32),
    /// A configuration ioctl failed: `SPI_ConfigError(device, select, error)`.
    /// The C++ driver closes the fd and leaves the driver unopened.
    Config(i32),
}

/// Pluggable SPI bus access.
///
/// Implement this in a crate that may use `libc`/`unsafe` to give
/// [`LinuxSpiDriver`] a real full-duplex `SPI_IOC_MESSAGE(1)` transfer; the
/// component, its ports, events, telemetry and assertions do not change.
pub trait SpiBackend: Send + Sync {
    /// C++ `open(device, select, clock, spiMode)` minus the event emission:
    /// open `/dev/spidev<device>.<select>` and apply the configuration.
    ///
    /// Returns the read-back mismatches to report (empty when everything
    /// matched, or when the backend cannot read the settings back).
    fn open(
        &self,
        device: FwIndexType,
        select: FwIndexType,
        clock: SpiFrequency,
        mode: SpiMode,
    ) -> Result<Vec<SpiConfigMismatch>, SpiOpenError>;

    /// One transfer: `write.len() == read.len()` is guaranteed by the
    /// component's assertions. `Err` carries the value reported in
    /// `SPI_WriteError` (C++ passes the `ioctl` return, `< 1` on failure).
    fn write_read(&self, write: &[u8], read: &mut [u8]) -> Result<(), i32>;

    /// C++ destructor's `close(m_fd)`. Default: nothing.
    fn close(&self) {}
}

/// Port of `LinuxSpiDriverComponentImplStub.cpp`: `open()` fails, a transfer
/// "succeeds" without touching hardware. The DEFAULT backend.
///
/// Because the component keeps the real driver's open-gate, a
/// `StubSpiBackend`-backed driver returns [`SpiStatus::SpiOpenErr`] from
/// every transfer (upstream's stub returns `SPI_OK` from a closed driver —
/// a stub artifact, see the module docs).
#[derive(Debug, Default, Clone, Copy)]
pub struct StubSpiBackend;

impl StubSpiBackend {
    /// Construct the stub backend.
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

impl SpiBackend for StubSpiBackend {
    fn open(
        &self,
        _device: FwIndexType,
        _select: FwIndexType,
        _clock: SpiFrequency,
        _mode: SpiMode,
    ) -> Result<Vec<SpiConfigMismatch>, SpiOpenError> {
        // C++ stub: open() returns false. The C++ event carries the fd (-1).
        Err(SpiOpenError::Open(-1))
    }

    fn write_read(&self, _write: &[u8], _read: &mut [u8]) -> Result<(), i32> {
        // C++ stub: SpiWriteRead_handler returns SPI_OK.
        Ok(())
    }
}

/// Default root holding the spidev character devices.
pub const SPIDEV_ROOT: &str = "/dev";

/// **NOT full duplex.** Opt-in `spidev` backend using plain `File` writes and
/// reads.
///
/// `spidev`'s `file_operations` implement `write()` (transmit only, received
/// data discarded) and `read()` (receive only, zeros transmitted). This
/// backend therefore performs a write transaction followed by a SEPARATE
/// read transaction — chip select is released in between and the two halves
/// are not clocked together. It is NOT equivalent to the
/// `SPI_IOC_MESSAGE(1)` transfer `SpiWriteRead` names, and `SpiStatus::SpiOk`
/// from it does not mean a full-duplex exchange happened.
///
/// It also cannot set or read back mode, bits-per-word or max speed — those
/// come from the device tree — so `open()` never reports a
/// [`SpiConfigMismatch`] and never fails with [`SpiOpenError::Config`].
///
/// Constructed ONLY through
/// [`Self::new_write_then_read_not_full_duplex`], so nobody reaches it by
/// accident.
pub struct HalfDuplexSpidevBackend {
    root: PathBuf,
    file: Mutex<Option<std::fs::File>>,
}

impl HalfDuplexSpidevBackend {
    /// Construct the half-duplex backend rooted at [`SPIDEV_ROOT`].
    ///
    /// The name is the warning: this is write-then-read, NOT the full-duplex
    /// transfer `SpiWriteRead` promises. See the type documentation.
    #[must_use]
    pub fn new_write_then_read_not_full_duplex() -> Self {
        Self {
            root: PathBuf::from(SPIDEV_ROOT),
            file: Mutex::new(None),
        }
    }

    /// Same backend rooted elsewhere (tests, unusual device layouts).
    #[must_use]
    pub fn with_device_root<P: Into<PathBuf>>(mut self, root: P) -> Self {
        self.root = root.into();
        self
    }

    /// The device path for a `device.select` pair — C++
    /// `"/dev/spidev%d.%d"`.
    #[must_use]
    pub fn device_path(&self, device: FwIndexType, select: FwIndexType) -> PathBuf {
        self.root.join(format!("spidev{device}.{select}"))
    }
}

impl SpiBackend for HalfDuplexSpidevBackend {
    fn open(
        &self,
        device: FwIndexType,
        select: FwIndexType,
        _clock: SpiFrequency,
        _mode: SpiMode,
    ) -> Result<Vec<SpiConfigMismatch>, SpiOpenError> {
        let path = self.device_path(device, select);
        match std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
        {
            Ok(file) => {
                *self.file.lock().unwrap() = Some(file);
                // Configuration is not applicable: nothing to mismatch.
                Ok(Vec::new())
            }
            // C++ parity: the event's error code is the fd, i.e. -1.
            Err(_) => Err(SpiOpenError::Open(-1)),
        }
    }

    fn write_read(&self, write: &[u8], read: &mut [u8]) -> Result<(), i32> {
        let mut guard = self.file.lock().unwrap();
        let Some(file) = guard.as_mut() else {
            return Err(-1);
        };
        // Transaction 1: transmit (received bytes are discarded by spidev).
        if file.write(write).map_err(|_| -1)? != write.len() {
            return Err(-1);
        }
        // Transaction 2: receive (zeros transmitted) — a DIFFERENT bus
        // transaction, which is exactly why this is not full duplex.
        let mut filled = 0;
        while filled < read.len() {
            match file.read(&mut read[filled..]) {
                Ok(0) => return Err(-1),
                Ok(count) => filled += count,
                Err(_) => return Err(-1),
            }
        }
        Ok(())
    }

    fn close(&self) {
        *self.file.lock().unwrap() = None;
    }
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

/// Guarded state (C++ `m_fd`, `m_device`, `m_select`, `m_bytes`).
struct SpiState {
    open: bool,
    device: FwIndexType,
    select: FwIndexType,
    bytes: FwSizeType,
}

impl Default for SpiState {
    fn default() -> Self {
        Self {
            open: false,
            // C++ initialises m_device/m_select to -1.
            device: -1,
            select: -1,
            bytes: 0,
        }
    }
}

/// `Drv::LinuxSpiDriver` — passive SPI driver over a swappable
/// [`SpiBackend`].
pub struct LinuxSpiDriver {
    /// Passive core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (`Log`/`LogText`) + `Time`.
    pub evt: EventGlue,
    /// Telemetry port (`Tlm`).
    pub tlm: TlmGlue,
    backend: Box<dyn SpiBackend>,
    /// Component (guarded-port) mutex; also the bus lock.
    state: Mutex<SpiState>,
    write_error_throttle: EventThrottle,
}

impl LinuxSpiDriver {
    /// `SPI_OpenError(device: I32, select: I32, error: I32)` — WARNING_HI,
    /// id 0. The `error` field carries the file descriptor (`-1`), not
    /// `errno` (C++ parity).
    pub const EVENTID_SPI_OPEN_ERROR: FwEventIdType = 0;
    /// `SPI_ConfigError(device: I32, select: I32, error: I32)` — WARNING_HI,
    /// id 1.
    pub const EVENTID_SPI_CONFIG_ERROR: FwEventIdType = 1;
    /// `SPI_WriteError(device: I32, select: I32, error: I32)` — WARNING_HI,
    /// id 2, `throttle 5`.
    pub const EVENTID_SPI_WRITE_ERROR: FwEventIdType = 2;
    /// `SPI_ConfigMismatch(device: I32, select: I32, parameter: string,
    /// write_value: U32, read_value: U32)` — WARNING_LO, id 3. Does NOT
    /// abort the open.
    pub const EVENTID_SPI_CONFIG_MISMATCH: FwEventIdType = 3;
    /// `SPI_PortOpened(device: I32, select: I32)` — ACTIVITY_HI, id 4.
    /// Declared and NEVER emitted (C++ parity: the id is reserved).
    pub const EVENTID_SPI_PORT_OPENED: FwEventIdType = 4;

    /// FPP `throttle 5` on `SPI_WriteError`.
    pub const WRITE_ERROR_THROTTLE: u32 = 5;

    /// `SPI_Bytes: FwSizeType` — id 0, cumulative, written on every
    /// successful transfer.
    pub const CHANID_SPI_BYTES: FwChanIdType = 0;

    /// Construct with the default [`StubSpiBackend`].
    pub fn new(name: &str) -> Arc<Self> {
        Self::with_backend(name, Box::new(StubSpiBackend::new()))
    }

    /// Construct with an explicit [`SpiBackend`] — a real full-duplex
    /// implementation supplied downstream, the opt-in
    /// [`HalfDuplexSpidevBackend`], or a test fake.
    pub fn with_backend(name: &str, backend: Box<dyn SpiBackend>) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            tlm: TlmGlue::new(),
            backend,
            state: Mutex::new(SpiState::default()),
            write_error_throttle: EventThrottle::new(Self::WRITE_ERROR_THROTTLE),
        })
    }

    fn id_base(&self) -> FwIdType {
        self.base.get_id_base()
    }

    /// C++ `open(device, select, clock, spiMode)`.
    ///
    /// `device` and `select` must be non-negative (`FW_ASSERT`). A failed
    /// open or configuration emits `SPI_OpenError` / `SPI_ConfigError` and
    /// leaves the driver CLOSED (C++ writes `m_fd` only after every ioctl
    /// succeeded); read-back mismatches emit WARNING_LO `SPI_ConfigMismatch`
    /// but do not abort. `SPI_PortOpened` is never emitted, as upstream.
    pub fn open(
        &self,
        device: FwIndexType,
        select: FwIndexType,
        clock: SpiFrequency,
        mode: SpiMode,
    ) -> bool {
        fw_assert!(device >= 0, device);
        fw_assert!(select >= 0, select);
        match self.backend.open(device, select, clock, mode) {
            Ok(mismatches) => {
                {
                    let mut state = self.state.lock().unwrap();
                    state.open = true;
                    state.device = device;
                    state.select = select;
                }
                for mismatch in &mismatches {
                    self.log_config_mismatch(device, select, mismatch);
                }
                true
            }
            Err(error) => {
                let (id, code) = match error {
                    SpiOpenError::Open(code) => (Self::EVENTID_SPI_OPEN_ERROR, code),
                    SpiOpenError::Config(code) => (Self::EVENTID_SPI_CONFIG_ERROR, code),
                };
                let text = if id == Self::EVENTID_SPI_OPEN_ERROR {
                    format!("Error opening SPI device {device}.{select}: {code}")
                } else {
                    format!("Error configuring SPI device {device}.{select}: {code}")
                };
                self.evt
                    .log_event(self.id_base(), id, LogSeverity::WarningHi, &text, |buf| {
                        fw_try!(buf.serialize_i32_be(i32::from(device)));
                        fw_try!(buf.serialize_i32_be(i32::from(select)));
                        buf.serialize_i32_be(code)
                    });
                false
            }
        }
    }

    fn log_config_mismatch(
        &self,
        device: FwIndexType,
        select: FwIndexType,
        mismatch: &SpiConfigMismatch,
    ) {
        let parameter = LogStringArg::from(mismatch.parameter.as_str());
        let write_value = mismatch.write_value;
        let read_value = mismatch.read_value;
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_SPI_CONFIG_MISMATCH,
            LogSeverity::WarningLo,
            &format!(
                "SPI device {device}.{select} configuration mismatch for {}: wrote {write_value}, read {read_value}",
                mismatch.parameter
            ),
            |buf| {
                fw_try!(buf.serialize_i32_be(i32::from(device)));
                fw_try!(buf.serialize_i32_be(i32::from(select)));
                fw_try!(parameter.serialize_to_truncated(
                    buf,
                    EVENT_STRING_SIZE,
                    Endianness::Big
                ));
                fw_try!(buf.serialize_u32_be(write_value));
                buf.serialize_u32_be(read_value)
            },
        );
    }

    /// Is the driver open (C++ `m_fd != -1`)?
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.state.lock().unwrap().open
    }

    /// Cumulative bytes transferred (C++ `m_bytes`, the `SPI_Bytes` channel).
    #[must_use]
    pub fn bytes(&self) -> FwSizeType {
        self.state.lock().unwrap().bytes
    }

    /// C++ destructor equivalent (which calls `close(m_fd)`
    /// unconditionally, including `close(-1)` when never opened).
    pub fn close(&self) {
        self.backend.close();
        self.state.lock().unwrap().open = false;
    }

    // -- Handlers (caller thread, under the component mutex) ----------------

    /// C++ `SpiWriteRead_handler`.
    ///
    /// `FW_ASSERT`s (programmer errors, never status returns): `portNum >= 0`,
    /// both buffers valid, and equal sizes. A closed driver is
    /// [`SpiStatus::SpiOpenErr`] with NO event. A failed transfer is a
    /// throttled `SPI_WriteError` plus [`SpiStatus::SpiOtherErr`]; success
    /// adds the read size to `m_bytes` and writes the `SPI_Bytes` channel.
    fn spi_write_read_handler(
        &self,
        port_num: FwIndexType,
        write_buffer: &mut Buffer,
        read_buffer: &mut Buffer,
    ) -> SpiStatus {
        fw_assert!(port_num >= 0, port_num);
        fw_assert!(write_buffer.is_valid());
        fw_assert!(read_buffer.is_valid());
        fw_assert!(
            write_buffer.size() == read_buffer.size(),
            write_buffer.size(),
            read_buffer.size()
        );

        let outcome = {
            let mut state = self.state.lock().unwrap();
            if !state.open {
                return SpiStatus::SpiOpenErr;
            }
            let device = state.device;
            let select = state.select;
            let transferred = read_buffer.size() as FwSizeType;
            match self
                .backend
                .write_read(write_buffer.data(), read_buffer.data_mut())
            {
                Ok(()) => {
                    state.bytes += transferred;
                    Ok(state.bytes)
                }
                Err(code) => Err((device, select, code)),
            }
        };
        match outcome {
            Ok(bytes) => {
                let time_tag = self.evt.time_get();
                self.tlm
                    .tlm_write(self.id_base(), Self::CHANID_SPI_BYTES, &bytes, time_tag);
                SpiStatus::SpiOk
            }
            Err((device, select, code)) => {
                if self.write_error_throttle.ok_to_emit() {
                    self.evt.log_event(
                        self.id_base(),
                        Self::EVENTID_SPI_WRITE_ERROR,
                        LogSeverity::WarningHi,
                        &format!("Error writing/reading SPI device {device}.{select}: {code}"),
                        |buf| {
                            fw_try!(buf.serialize_i32_be(i32::from(device)));
                            fw_try!(buf.serialize_i32_be(i32::from(select)));
                            buf.serialize_i32_be(code)
                        },
                    );
                }
                SpiStatus::SpiOtherErr
            }
        }
    }

    /// C++ `SpiReadWrite_handler` (DEPRECATED port): same asserts, then the
    /// write/read handler with the status discarded.
    ///
    /// Divergence: routed through the same mutex-locked handler, so it
    /// cannot race the guarded port (C++ leaves this port unguarded).
    fn spi_read_write_handler(
        &self,
        port_num: FwIndexType,
        write_buffer: &mut Buffer,
        read_buffer: &mut Buffer,
    ) {
        let _ = self.spi_write_read_handler(port_num, write_buffer, read_buffer);
    }
}

fprime_comp::input_port_adapter! {
    /// `SpiWriteRead` — GUARDED `Drv.SpiWriteRead` input.
    component: LinuxSpiDriver;
    adapter: SpiWriteReadAdapter;
    port: SpiWriteReadPort;
    input: pub spi_write_read_in;
    handler: spi_write_read_handler;
    returns: SpiStatus;
    args { mut write_buffer: Buffer, mut read_buffer: Buffer }
}

fprime_comp::input_port_adapter! {
    /// `SpiReadWrite` — DEPRECATED sync `Drv.SpiReadWrite` input.
    component: LinuxSpiDriver;
    adapter: SpiReadWriteAdapter;
    port: SpiReadWritePort;
    input: pub spi_read_write_in;
    handler: spi_read_write_handler;
    args { mut write_buffer: Buffer, mut read_buffer: Buffer }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_comp::{LogPort, TlmPort};
    use fprime_fw::{
        Deserialize, LinearBuffer, LogBuffer, Serialize, SerializeStatus, Time, TlmBuffer,
    };
    use std::sync::atomic::{AtomicU32, Ordering};

    // -- scaffolding ---------------------------------------------------------

    struct TempDir {
        path: PathBuf,
    }

    impl TempDir {
        fn new(tag: &str) -> Self {
            static COUNTER: AtomicU32 = AtomicU32::new(0);
            let unique = COUNTER.fetch_add(1, Ordering::SeqCst);
            let path = std::env::temp_dir()
                .join(format!("fprime-spi-{tag}-{}-{unique}", std::process::id()));
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

    #[derive(Default)]
    struct FakeSpi {
        open_result: Mutex<Option<Result<Vec<SpiConfigMismatch>, SpiOpenError>>>,
        transfer_error: Mutex<Option<i32>>,
        /// Bytes handed to the transmit half.
        written: Mutex<Vec<Vec<u8>>>,
        /// Payload copied into the receive half.
        read_payload: Mutex<Vec<u8>>,
        closed: AtomicU32,
    }

    impl FakeSpi {
        fn opening() -> Arc<Self> {
            Arc::new(Self {
                open_result: Mutex::new(Some(Ok(Vec::new()))),
                ..Self::default()
            })
        }
    }

    impl SpiBackend for Arc<FakeSpi> {
        fn open(
            &self,
            _device: FwIndexType,
            _select: FwIndexType,
            _clock: SpiFrequency,
            _mode: SpiMode,
        ) -> Result<Vec<SpiConfigMismatch>, SpiOpenError> {
            self.open_result
                .lock()
                .unwrap()
                .clone()
                .unwrap_or(Err(SpiOpenError::Open(-1)))
        }

        fn write_read(&self, write: &[u8], read: &mut [u8]) -> Result<(), i32> {
            self.written.lock().unwrap().push(write.to_vec());
            if let Some(code) = *self.transfer_error.lock().unwrap() {
                return Err(code);
            }
            let payload = self.read_payload.lock().unwrap();
            let count = payload.len().min(read.len());
            read[..count].copy_from_slice(&payload[..count]);
            Ok(())
        }

        fn close(&self) {
            self.closed.fetch_add(1, Ordering::SeqCst);
        }
    }

    struct Harness {
        driver: Arc<LinuxSpiDriver>,
        fake: Arc<FakeSpi>,
        events: Arc<EventCollector>,
        tlm: Arc<TlmCollector>,
    }

    fn harness(fake: Arc<FakeSpi>) -> Harness {
        let driver = LinuxSpiDriver::with_backend("spiDrv", Box::new(fake.clone()));
        let events = Arc::new(EventCollector::default());
        let tlm = Arc::new(TlmCollector::default());
        driver.evt.log_out.connect(events.clone(), 0);
        driver.tlm.tlm_out.connect(tlm.clone(), 0);
        Harness {
            driver,
            fake,
            events,
            tlm,
        }
    }

    fn buffer_with(data: &[u8]) -> Buffer {
        let mut buffer = Buffer::allocate(data.len());
        buffer.data_mut().copy_from_slice(data);
        buffer
    }

    // -- enums ---------------------------------------------------------------

    #[test]
    fn spi_status_discriminants_match_fpp() {
        assert_eq!(SpiStatus::SpiOk as u8, 0);
        assert_eq!(SpiStatus::SpiOpenErr as u8, 1);
        assert_eq!(SpiStatus::SpiConfigErr as u8, 2);
        assert_eq!(SpiStatus::SpiMismatchErr as u8, 3);
        assert_eq!(SpiStatus::SpiWriteErr as u8, 4);
        assert_eq!(SpiStatus::SpiOtherErr as u8, 5);
        assert_eq!(SpiStatus::NUM_CONSTANTS, 6);
    }

    #[test]
    fn spi_status_serializes_at_u8_width_and_rejects_undeclared_values() {
        let mut buf = LinearBuffer::<4>::new();
        assert!(
            SpiStatus::SpiOtherErr
                .serialize_to(&mut buf, Endianness::Big)
                .is_ok()
        );
        assert_eq!(buf.as_slice(), &[5]);

        let mut buf = LinearBuffer::<4>::new();
        assert!(buf.serialize_u8_be(6).is_ok());
        let mut out = SpiStatus::SpiOk;
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserFormatError
        );
        assert_eq!(out, SpiStatus::SpiOk);
    }

    #[test]
    fn frequency_and_mode_values_match_cpp() {
        assert_eq!(SpiFrequency::SpiFrequency1Mhz.as_hz(), 1_000_000);
        assert_eq!(SpiFrequency::SpiFrequency5Mhz.as_hz(), 5_000_000);
        assert_eq!(SpiFrequency::SpiFrequency10Mhz.as_hz(), 10_000_000);
        assert_eq!(SpiFrequency::SpiFrequency15Mhz.as_hz(), 15_000_000);
        assert_eq!(SpiFrequency::SpiFrequency20Mhz.as_hz(), 20_000_000);
        assert_eq!(SpiMode::SpiModeCpolLowCphaLow.as_kernel_mode(), 0);
        assert_eq!(SpiMode::SpiModeCpolLowCphaHigh.as_kernel_mode(), 1);
        assert_eq!(SpiMode::SpiModeCpolHighCphaLow.as_kernel_mode(), 2);
        assert_eq!(SpiMode::SpiModeCpolHighCphaHigh.as_kernel_mode(), 3);
    }

    // -- open ----------------------------------------------------------------

    #[test]
    fn open_failure_emits_spi_open_error_with_fd_as_the_error_code() {
        let fake = Arc::new(FakeSpi {
            open_result: Mutex::new(Some(Err(SpiOpenError::Open(-1)))),
            ..FakeSpi::default()
        });
        let h = harness(fake);
        assert!(!h.driver.open(
            1,
            0,
            SpiFrequency::SpiFrequency1Mhz,
            SpiMode::SpiModeCpolLowCphaLow
        ));
        assert!(!h.driver.is_open());
        let (_, severity, bytes) = h
            .events
            .find(LinuxSpiDriver::EVENTID_SPI_OPEN_ERROR)
            .expect("SPI_OpenError");
        assert_eq!(severity, LogSeverity::WarningHi);
        let mut expected = Vec::new();
        expected.extend_from_slice(&1i32.to_be_bytes());
        expected.extend_from_slice(&0i32.to_be_bytes());
        expected.extend_from_slice(&(-1i32).to_be_bytes());
        assert_eq!(bytes, expected);
    }

    #[test]
    fn config_failure_emits_spi_config_error_and_leaves_the_driver_closed() {
        let fake = Arc::new(FakeSpi {
            open_result: Mutex::new(Some(Err(SpiOpenError::Config(-7)))),
            ..FakeSpi::default()
        });
        let h = harness(fake);
        assert!(!h.driver.open(
            0,
            1,
            SpiFrequency::SpiFrequency10Mhz,
            SpiMode::SpiModeCpolHighCphaHigh
        ));
        assert!(!h.driver.is_open());
        let (_, severity, bytes) = h
            .events
            .find(LinuxSpiDriver::EVENTID_SPI_CONFIG_ERROR)
            .expect("SPI_ConfigError");
        assert_eq!(severity, LogSeverity::WarningHi);
        let mut expected = Vec::new();
        expected.extend_from_slice(&0i32.to_be_bytes());
        expected.extend_from_slice(&1i32.to_be_bytes());
        expected.extend_from_slice(&(-7i32).to_be_bytes());
        assert_eq!(bytes, expected);
    }

    #[test]
    fn config_mismatch_is_warning_low_and_does_not_abort_the_open() {
        let fake = Arc::new(FakeSpi {
            open_result: Mutex::new(Some(Ok(vec![SpiConfigMismatch {
                parameter: "MAX_SPEED_HZ".to_string(),
                write_value: 10_000_000,
                read_value: 8_000_000,
            }]))),
            ..FakeSpi::default()
        });
        let h = harness(fake);
        assert!(h.driver.open(
            2,
            3,
            SpiFrequency::SpiFrequency10Mhz,
            SpiMode::SpiModeCpolLowCphaLow
        ));
        assert!(h.driver.is_open());
        let (_, severity, bytes) = h
            .events
            .find(LinuxSpiDriver::EVENTID_SPI_CONFIG_MISMATCH)
            .expect("SPI_ConfigMismatch");
        assert_eq!(severity, LogSeverity::WarningLo);
        let mut expected = Vec::new();
        expected.extend_from_slice(&2i32.to_be_bytes());
        expected.extend_from_slice(&3i32.to_be_bytes());
        expected.extend_from_slice(&[0, 12]);
        expected.extend_from_slice(b"MAX_SPEED_HZ");
        expected.extend_from_slice(&10_000_000u32.to_be_bytes());
        expected.extend_from_slice(&8_000_000u32.to_be_bytes());
        assert_eq!(bytes, expected);
    }

    #[test]
    fn port_opened_is_never_emitted() {
        let h = harness(FakeSpi::opening());
        assert!(h.driver.open(
            0,
            0,
            SpiFrequency::SpiFrequency1Mhz,
            SpiMode::SpiModeCpolLowCphaLow
        ));
        assert!(
            !h.events
                .ids()
                .contains(&LinuxSpiDriver::EVENTID_SPI_PORT_OPENED)
        );
        assert!(h.events.ids().is_empty());
    }

    #[test]
    #[should_panic]
    fn open_asserts_on_a_negative_device() {
        let h = harness(FakeSpi::opening());
        let _ = h.driver.open(
            -1,
            0,
            SpiFrequency::SpiFrequency1Mhz,
            SpiMode::SpiModeCpolLowCphaLow,
        );
    }

    #[test]
    #[should_panic]
    fn open_asserts_on_a_negative_select() {
        let h = harness(FakeSpi::opening());
        let _ = h.driver.open(
            0,
            -1,
            SpiFrequency::SpiFrequency1Mhz,
            SpiMode::SpiModeCpolLowCphaLow,
        );
    }

    // -- transfers ------------------------------------------------------------

    #[test]
    fn closed_driver_returns_open_err_without_an_event() {
        let h = harness(FakeSpi::opening());
        let mut write = buffer_with(&[1, 2]);
        let mut read = Buffer::allocate(2);
        let port = h.driver.spi_write_read_in(0);
        assert_eq!(
            port.target.invoke(port.port_num, &mut write, &mut read),
            SpiStatus::SpiOpenErr
        );
        assert!(h.events.ids().is_empty());
        assert!(h.fake.written.lock().unwrap().is_empty());
    }

    #[test]
    fn stub_backed_driver_cannot_open_so_transfers_are_open_err() {
        // Documented divergence from the C++ stub, which returns SPI_OK.
        let driver = LinuxSpiDriver::new("stubSpi");
        let events = Arc::new(EventCollector::default());
        driver.evt.log_out.connect(events.clone(), 0);
        assert!(!driver.open(
            0,
            0,
            SpiFrequency::SpiFrequency1Mhz,
            SpiMode::SpiModeCpolLowCphaLow
        ));
        assert_eq!(events.count_of(LinuxSpiDriver::EVENTID_SPI_OPEN_ERROR), 1);
        let mut write = buffer_with(&[1]);
        let mut read = Buffer::allocate(1);
        let port = driver.spi_write_read_in(0);
        assert_eq!(
            port.target.invoke(port.port_num, &mut write, &mut read),
            SpiStatus::SpiOpenErr
        );
        // The stub BACKEND itself still reports a successful transfer,
        // matching LinuxSpiDriverComponentImplStub.cpp at the seam.
        let mut sink = [0u8; 1];
        assert_eq!(StubSpiBackend::new().write_read(&[1], &mut sink), Ok(()));
    }

    #[test]
    fn successful_transfer_fills_the_read_buffer_and_writes_spi_bytes() {
        let fake = FakeSpi::opening();
        *fake.read_payload.lock().unwrap() = vec![0xA0, 0xA1, 0xA2];
        let h = harness(fake);
        assert!(h.driver.open(
            0,
            0,
            SpiFrequency::SpiFrequency1Mhz,
            SpiMode::SpiModeCpolLowCphaLow
        ));
        let mut write = buffer_with(&[1, 2, 3]);
        let mut read = Buffer::allocate(3);
        let port = h.driver.spi_write_read_in(0);
        assert_eq!(
            port.target.invoke(port.port_num, &mut write, &mut read),
            SpiStatus::SpiOk
        );
        assert_eq!(read.data(), &[0xA0, 0xA1, 0xA2]);
        assert_eq!(h.fake.written.lock().unwrap().clone(), vec![vec![1, 2, 3]]);
        assert_eq!(h.driver.bytes(), 3);

        // SPI_Bytes is cumulative and FwSizeType (u64) wide.
        let writes = h.tlm.writes.lock().unwrap().clone();
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0].0, LinuxSpiDriver::CHANID_SPI_BYTES);
        assert_eq!(writes[0].1, 3u64.to_be_bytes().to_vec());

        assert_eq!(
            port.target.invoke(port.port_num, &mut write, &mut read),
            SpiStatus::SpiOk
        );
        let writes = h.tlm.writes.lock().unwrap().clone();
        assert_eq!(writes[1].1, 6u64.to_be_bytes().to_vec());
    }

    #[test]
    fn failed_transfer_emits_throttled_write_error_and_other_err() {
        let fake = FakeSpi::opening();
        *fake.transfer_error.lock().unwrap() = Some(-1);
        let h = harness(fake);
        assert!(h.driver.open(
            3,
            2,
            SpiFrequency::SpiFrequency5Mhz,
            SpiMode::SpiModeCpolLowCphaLow
        ));
        let mut write = buffer_with(&[7]);
        let mut read = Buffer::allocate(1);
        let port = h.driver.spi_write_read_in(0);
        for _ in 0..8 {
            assert_eq!(
                port.target.invoke(port.port_num, &mut write, &mut read),
                SpiStatus::SpiOtherErr
            );
        }
        assert_eq!(
            h.events.count_of(LinuxSpiDriver::EVENTID_SPI_WRITE_ERROR),
            LinuxSpiDriver::WRITE_ERROR_THROTTLE as usize
        );
        let (_, severity, bytes) = h
            .events
            .find(LinuxSpiDriver::EVENTID_SPI_WRITE_ERROR)
            .unwrap();
        assert_eq!(severity, LogSeverity::WarningHi);
        let mut expected = Vec::new();
        expected.extend_from_slice(&3i32.to_be_bytes());
        expected.extend_from_slice(&2i32.to_be_bytes());
        expected.extend_from_slice(&(-1i32).to_be_bytes());
        assert_eq!(bytes, expected);
        // No telemetry on failure.
        assert!(h.tlm.writes.lock().unwrap().is_empty());
        assert_eq!(h.driver.bytes(), 0);
    }

    #[test]
    fn deprecated_read_write_port_performs_the_transfer_and_drops_the_status() {
        let fake = FakeSpi::opening();
        *fake.read_payload.lock().unwrap() = vec![0x5A, 0x5B];
        let h = harness(fake);
        assert!(h.driver.open(
            0,
            0,
            SpiFrequency::SpiFrequency1Mhz,
            SpiMode::SpiModeCpolLowCphaLow
        ));
        let mut write = buffer_with(&[9, 9]);
        let mut read = Buffer::allocate(2);
        let port = h.driver.spi_read_write_in(0);
        port.target.invoke(port.port_num, &mut write, &mut read);
        assert_eq!(read.data(), &[0x5A, 0x5B]);
        assert_eq!(h.driver.bytes(), 2);
    }

    #[test]
    #[should_panic]
    fn transfer_asserts_on_mismatched_buffer_sizes() {
        let h = harness(FakeSpi::opening());
        assert!(h.driver.open(
            0,
            0,
            SpiFrequency::SpiFrequency1Mhz,
            SpiMode::SpiModeCpolLowCphaLow
        ));
        let mut write = buffer_with(&[1, 2, 3]);
        let mut read = Buffer::allocate(2);
        let port = h.driver.spi_write_read_in(0);
        let _ = port.target.invoke(port.port_num, &mut write, &mut read);
    }

    #[test]
    #[should_panic]
    fn transfer_asserts_on_an_invalid_buffer() {
        let h = harness(FakeSpi::opening());
        assert!(h.driver.open(
            0,
            0,
            SpiFrequency::SpiFrequency1Mhz,
            SpiMode::SpiModeCpolLowCphaLow
        ));
        let mut write = Buffer::empty();
        let mut read = Buffer::empty();
        let port = h.driver.spi_write_read_in(0);
        let _ = port.target.invoke(port.port_num, &mut write, &mut read);
    }

    #[test]
    fn close_reverts_to_the_open_error_path() {
        let h = harness(FakeSpi::opening());
        assert!(h.driver.open(
            0,
            0,
            SpiFrequency::SpiFrequency1Mhz,
            SpiMode::SpiModeCpolLowCphaLow
        ));
        h.driver.close();
        assert_eq!(h.fake.closed.load(Ordering::SeqCst), 1);
        let mut write = buffer_with(&[1]);
        let mut read = Buffer::allocate(1);
        let port = h.driver.spi_write_read_in(0);
        assert_eq!(
            port.target.invoke(port.port_num, &mut write, &mut read),
            SpiStatus::SpiOpenErr
        );
    }

    // -- half-duplex opt-in backend -------------------------------------------

    #[test]
    fn half_duplex_backend_paths_match_the_cpp_format() {
        let backend = HalfDuplexSpidevBackend::new_write_then_read_not_full_duplex();
        assert_eq!(backend.device_path(1, 0), PathBuf::from("/dev/spidev1.0"));
        assert_eq!(backend.device_path(0, 2), PathBuf::from("/dev/spidev0.2"));
    }

    #[test]
    fn half_duplex_backend_writes_then_reads_two_separate_transactions() {
        let temp = TempDir::new("halfduplex");
        let device = temp.path.join("spidev0.0");
        // A regular file stands in for the character device; seed the bytes
        // the "read" transaction will pick up after the write moved the
        // cursor (a real spidev is not positional — this is a test artifact).
        std::fs::write(&device, b"..CD").unwrap();
        let backend = HalfDuplexSpidevBackend::new_write_then_read_not_full_duplex()
            .with_device_root(&temp.path);
        assert_eq!(
            backend.open(
                0,
                0,
                SpiFrequency::SpiFrequency1Mhz,
                SpiMode::SpiModeCpolLowCphaLow
            ),
            // No configuration is applicable, so no mismatches are reported.
            Ok(Vec::new())
        );
        let mut read = [0u8; 2];
        // Two SEPARATE transactions: the write lands first, then a distinct
        // read follows — never one simultaneous exchange.
        assert_eq!(backend.write_read(b"ab", &mut read), Ok(()));
        assert_eq!(&read, b"CD");
        assert_eq!(std::fs::read(&device).unwrap(), b"abCD");
        backend.close();
        assert_eq!(backend.write_read(b"ab", &mut read), Err(-1));
    }

    #[test]
    fn half_duplex_backend_reports_open_failure_with_minus_one() {
        let temp = TempDir::new("halfmissing");
        let backend = HalfDuplexSpidevBackend::new_write_then_read_not_full_duplex()
            .with_device_root(&temp.path);
        assert_eq!(
            backend.open(
                9,
                9,
                SpiFrequency::SpiFrequency1Mhz,
                SpiMode::SpiModeCpolLowCphaLow
            ),
            Err(SpiOpenError::Open(-1))
        );
    }
}
