//! # LinuxI2cDriver — port of `Drv::LinuxI2cDriver` (+ `Drv/Interfaces/I2c`,
//! `AsyncI2c`, `AsyncGuardedI2c`, `Drv/Ports/I2cDriverPorts`)
//!
//! C++ sources: `Drv/LinuxI2cDriver/LinuxI2cDriver.{fpp,hpp,cpp}`,
//! `LinuxI2cDriverStub.cpp`, `Drv/Ports/I2cDriverPorts.fpp`,
//! `Drv/Interfaces/{I2c,AsyncI2c,AsyncGuardedI2c}.fpp`.
//! Analysis: `docs/cpp-analysis/linux-drivers.md`.
//!
//! PASSIVE component with three GUARDED input ports (`write`, `read`,
//! `writeRead`) and nothing else — the FPP declares no `Log`, `Tlm`, `Time`
//! or command ports, so this driver has NO events, NO telemetry and NO
//! commands. Buffers are caller-owned throughout: the driver never allocates
//! or deallocates.
//!
//! ## UNSUPPORTED: there is no real backend here, and there cannot be one
//!
//! Every I2C transaction needs `ioctl(2)`:
//!
//! - `ioctl(fd, I2C_SLAVE /* 0x0703 */, addr)` must select the slave before
//!   any `read()`/`write()` on `/dev/i2c-N`. A plain `read`/`write` on a
//!   freshly-opened `/dev/i2c-N` addresses nobody and fails with `EINVAL`.
//! - `ioctl(fd, I2C_RDWR /* 0x0707 */, &i2c_rdwr_ioctl_data)` is the ONLY
//!   way to issue the repeated-START combined write-then-read that
//!   `writeRead` promises.
//!
//! `std` exposes no `ioctl`, so a working backend requires either an
//! `unsafe extern` FFI shim or a `libc`/`nix` dependency — both excluded by
//! this workspace (`#![forbid(unsafe_code)]`, zero third-party
//! dependencies). **`LinuxI2cDriver` is therefore ported as a component
//! shell over the [`I2cBackend`] seam, shipping only [`StubI2cBackend`].**
//! A downstream project that may use `libc` implements [`I2cBackend`] and
//! passes it to [`LinuxI2cDriver::with_backend`]; ports, statuses, guard
//! semantics and assertions all stay as they are.
//!
//! [`StubI2cBackend`] reproduces `LinuxI2cDriverStub.cpp` exactly: `open()`
//! returns `true` and all three handlers return `I2C_OK` (the SDD's claim
//! that the stub reports open failures is wrong — follow the code).

use fprime_config::{FwIndexType, FwSizeType};
use fprime_fw::{Buffer, fpp_enum, fw_assert};
use std::sync::{Arc, Mutex};

/// `Drv.AsyncI2cCfg.I2cDriverPorts` (`default/config/AsyncI2cCfg.fpp`): the
/// array size of every port in the `AsyncI2c` / `AsyncGuardedI2c`
/// interfaces.
pub const I2C_DRIVER_PORTS: usize = 10;

fpp_enum! {
    /// `Drv::I2cStatus` — FPP enum, `repr U8`
    /// (`Drv/Ports/I2cDriverPorts.fpp`, explicit values).
    pub enum I2cStatus : u8 {
        /// Transaction okay.
        I2cOk = 0,
        /// I2C address invalid.
        I2cAddressErr = 1,
        /// I2C write failed.
        I2cWriteErr = 2,
        /// I2C read failed.
        I2cReadErr = 3,
        /// I2C driver failed to open the device.
        I2cOpenErr = 4,
        /// Other errors that do not fit.
        ///
        /// Gotcha (C++ parity): `writeRead` collapses EVERY failure into
        /// this value, because `ioctl(I2C_RDWR)` reports a single result —
        /// never map a combined-transfer failure to ADDRESS/WRITE/READ.
        I2cOtherErr = 5,
    }
    default I2cOk
}

/// `Drv.I2c` port: `I2c(addr: U32, ref serBuffer: Fw.Buffer) -> I2cStatus`.
///
/// The buffer is a C++ `ref` — caller-owned — so it stays a `&mut Buffer`
/// here. On `read` the driver fills the buffer's data window; the caller
/// sizes it beforehand.
pub trait I2cPort: Send + Sync {
    /// Perform the transfer against slave `addr`.
    fn invoke(&self, port_num: FwIndexType, addr: u32, ser_buffer: &mut Buffer) -> I2cStatus;
}

/// `Drv.I2cWriteRead` port:
/// `I2cWriteRead(addr: U32, ref writeBuffer, ref readBuffer) -> I2cStatus`.
///
/// The caller MUST size `read_buffer` before the call — its size is the
/// number of bytes read back in the combined transfer.
pub trait I2cWriteReadPort: Send + Sync {
    /// Write then (repeated START) read against slave `addr`.
    fn invoke(
        &self,
        port_num: FwIndexType,
        addr: u32,
        write_buffer: &mut Buffer,
        read_buffer: &mut Buffer,
    ) -> I2cStatus;
}

/// `Drv.I2cRequest` port (async request, no return value) from
/// `Drv/Ports/I2cDriverPorts.fpp`.
///
/// Port TYPE only: no in-tree driver implements `AsyncI2c` /
/// `AsyncGuardedI2c`, and neither does this port. It is declared so a
/// deployment can wire components that use the async interface. A future
/// async driver carrying an owned `Fw::Buffer` across its queue would need
/// the `BufferEscrow` mechanism (see `fprime_comp::escrow`).
pub trait I2cRequestPort: Send + Sync {
    /// Request a write (or read) transaction against slave `addr`.
    fn invoke(&self, port_num: FwIndexType, addr: u32, buffer: &mut Buffer);
}

/// `Drv.I2cWriteReadRequest` port (async request, no return value).
/// Port type only — see [`I2cRequestPort`].
pub trait I2cWriteReadRequestPort: Send + Sync {
    /// Request a combined write-then-read transaction against `addr`.
    fn invoke(
        &self,
        port_num: FwIndexType,
        addr: u32,
        write_buffer: &mut Buffer,
        read_buffer: &mut Buffer,
    );
}

/// `Drv.I2cCallback` port: completion of an async write or read.
/// Port type only — see [`I2cRequestPort`].
pub trait I2cCallbackPort: Send + Sync {
    /// Report the transaction result (the buffer holds read data for reads).
    fn invoke(&self, port_num: FwIndexType, buffer: &mut Buffer, status: I2cStatus);
}

/// `Drv.I2cWriteReadCallback` port: completion of an async combined
/// transfer. Port type only — see [`I2cRequestPort`].
pub trait I2cWriteReadCallbackPort: Send + Sync {
    /// Report the combined-transfer result.
    fn invoke(
        &self,
        port_num: FwIndexType,
        write_buffer: &mut Buffer,
        read_buffer: &mut Buffer,
        status: I2cStatus,
    );
}

// ---------------------------------------------------------------------------
// Backend seam
// ---------------------------------------------------------------------------

/// Pluggable I2C bus access.
///
/// Implement this in a crate that may use `libc`/`unsafe` to give
/// [`LinuxI2cDriver`] real hardware access (see the module documentation for
/// the exact `ioctl`s required). Implementations are responsible for the
/// `I2C_SLAVE` selection and the following transfer being atomic — which is
/// why every driver port is `guarded`.
pub trait I2cBackend: Send + Sync {
    /// C++ `open()`: `::open(device, O_RDWR)`; `true` when the device
    /// opened. Typical device: `/dev/i2c-1`.
    fn open(&self, device: &str) -> bool;

    /// `write_handler` transfer: select `addr`, write `data`.
    fn write(&self, addr: u32, data: &[u8]) -> I2cStatus;

    /// `read_handler` transfer: select `addr`, read exactly `dest.len()`
    /// bytes into `dest`.
    fn read(&self, addr: u32, dest: &mut [u8]) -> I2cStatus;

    /// `writeRead_handler` transfer: one `I2C_RDWR` combined transaction
    /// (write, repeated START, read) against `addr`.
    ///
    /// Contract (C++ parity): report EVERY failure as
    /// [`I2cStatus::I2cOtherErr`]. `ioctl(I2C_RDWR)` returns a single
    /// result, so a combined transfer cannot honestly be classified as an
    /// address, write or read error. The component propagates whatever this
    /// returns, so the honesty lives here.
    fn write_read(&self, addr: u32, write: &[u8], read: &mut [u8]) -> I2cStatus;

    /// C++ destructor equivalent. Default: nothing.
    fn close(&self) {}
}

/// Port of `LinuxI2cDriverStub.cpp`: `open()` → `true`, every transfer →
/// [`I2cStatus::I2cOk`] with no bus activity. The only backend shipped here.
///
/// Buffers are left untouched, so a `read` through this backend yields
/// whatever the caller's buffer already held — it is a wiring/topology aid,
/// never a data source.
#[derive(Debug, Default, Clone, Copy)]
pub struct StubI2cBackend;

impl StubI2cBackend {
    /// Construct the stub backend.
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

impl I2cBackend for StubI2cBackend {
    fn open(&self, _device: &str) -> bool {
        true
    }
    fn write(&self, _addr: u32, _data: &[u8]) -> I2cStatus {
        I2cStatus::I2cOk
    }
    fn read(&self, _addr: u32, _dest: &mut [u8]) -> I2cStatus {
        I2cStatus::I2cOk
    }
    fn write_read(&self, _addr: u32, _write: &[u8], _read: &mut [u8]) -> I2cStatus {
        I2cStatus::I2cOk
    }
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

/// Guarded state: the C++ component's single `int m_fd = -1` member, plus
/// the device string (owned here).
#[derive(Default)]
struct I2cState {
    open: bool,
    device: String,
}

/// `Drv::LinuxI2cDriver` — passive I2C driver over a swappable
/// [`I2cBackend`].
///
/// All three ports are `guarded`, so they serialize on the component mutex —
/// mandatory, because address selection and the transfer that follows must
/// be atomic on a shared bus.
pub struct LinuxI2cDriver {
    /// Passive core (name / id_base / instance).
    pub base: fprime_comp::PassiveBase,
    backend: Box<dyn I2cBackend>,
    /// Component (guarded-port) mutex, also holding the open state.
    state: Mutex<I2cState>,
}

impl LinuxI2cDriver {
    /// Construct with the only shipped backend, [`StubI2cBackend`].
    pub fn new(name: &str) -> Arc<Self> {
        Self::with_backend(name, Box::new(StubI2cBackend::new()))
    }

    /// Construct with an explicit [`I2cBackend`] (a real `libc`-based
    /// implementation supplied downstream, or a test fake).
    pub fn with_backend(name: &str, backend: Box<dyn I2cBackend>) -> Arc<Self> {
        Arc::new(Self {
            base: fprime_comp::PassiveBase::new(name),
            backend,
            state: Mutex::new(I2cState::default()),
        })
    }

    /// C++ `open(device)`: `true` when the device opened. The C++
    /// `FW_ASSERT(device != nullptr)` is structural here.
    pub fn open(&self, device: &str) -> bool {
        let opened = self.backend.open(device);
        let mut state = self.state.lock().unwrap();
        state.open = opened;
        state.device = device.to_string();
        opened
    }

    /// Is the driver open (C++ `m_fd != -1`)?
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.state.lock().unwrap().open
    }

    /// The device path passed to the last [`Self::open`].
    #[must_use]
    pub fn device(&self) -> String {
        self.state.lock().unwrap().device.clone()
    }

    /// C++ destructor equivalent.
    pub fn close(&self) {
        self.backend.close();
        self.state.lock().unwrap().open = false;
    }

    // -- Handlers (caller thread, under the component mutex) ----------------

    /// C++ `write_handler`: closed → `I2C_OPEN_ERR`; an invalid buffer is a
    /// programmer error (`FW_ASSERT(data != nullptr)`); otherwise the
    /// backend's status. The address is passed through unchecked, exactly
    /// like the C++ `ioctl(I2C_SLAVE, addr)` call.
    fn write_handler(
        &self,
        _port_num: FwIndexType,
        addr: u32,
        ser_buffer: &mut Buffer,
    ) -> I2cStatus {
        let state = self.state.lock().unwrap();
        if !state.open {
            return I2cStatus::I2cOpenErr;
        }
        fw_assert!(ser_buffer.is_valid());
        self.backend.write(addr, ser_buffer.data())
    }

    /// C++ `read_handler`: identical to `write_handler` but reading into the
    /// caller-sized buffer window; a short read is `I2C_READ_ERR`.
    fn read_handler(
        &self,
        _port_num: FwIndexType,
        addr: u32,
        ser_buffer: &mut Buffer,
    ) -> I2cStatus {
        let state = self.state.lock().unwrap();
        if !state.open {
            return I2cStatus::I2cOpenErr;
        }
        fw_assert!(ser_buffer.is_valid());
        self.backend.read(addr, ser_buffer.data_mut())
    }

    /// C++ `writeRead_handler`: closed → `I2C_OPEN_ERR`; both buffers must
    /// be valid and the address and both sizes must fit `U16`
    /// (`FW_ASSERT_NO_OVERFLOW`) — those are programmer errors, not status
    /// returns. Every failure of the combined transfer is
    /// [`I2cStatus::I2cOtherErr`].
    fn write_read_handler(
        &self,
        _port_num: FwIndexType,
        addr: u32,
        write_buffer: &mut Buffer,
        read_buffer: &mut Buffer,
    ) -> I2cStatus {
        let state = self.state.lock().unwrap();
        if !state.open {
            return I2cStatus::I2cOpenErr;
        }
        fw_assert!(write_buffer.is_valid());
        fw_assert!(read_buffer.is_valid());
        // C++ FW_ASSERT_NO_OVERFLOW(addr, U16) and the same for both sizes
        // (i2c_msg carries U16 addr/len).
        fw_assert!(addr <= u32::from(u16::MAX), addr);
        fw_assert!(
            write_buffer.size() as FwSizeType <= FwSizeType::from(u16::MAX),
            write_buffer.size()
        );
        fw_assert!(
            read_buffer.size() as FwSizeType <= FwSizeType::from(u16::MAX),
            read_buffer.size()
        );
        // Distinct buffers: the write window is borrowed immutably and the
        // read window mutably, with no copy (no steady-state allocation).
        self.backend
            .write_read(addr, write_buffer.data(), read_buffer.data_mut())
    }
}

fprime_comp::input_port_adapter! {
    /// `write` — GUARDED `Drv.I2c` input.
    component: LinuxI2cDriver;
    adapter: I2cWriteAdapter;
    port: I2cPort;
    input: pub write_in;
    handler: write_handler;
    returns: I2cStatus;
    args { val addr: u32, mut ser_buffer: Buffer }
}

fprime_comp::input_port_adapter! {
    /// `read` — GUARDED `Drv.I2c` input.
    component: LinuxI2cDriver;
    adapter: I2cReadAdapter;
    port: I2cPort;
    input: pub read_in;
    handler: read_handler;
    returns: I2cStatus;
    args { val addr: u32, mut ser_buffer: Buffer }
}

fprime_comp::input_port_adapter! {
    /// `writeRead` — GUARDED `Drv.I2cWriteRead` input.
    component: LinuxI2cDriver;
    adapter: I2cWriteReadAdapter;
    port: I2cWriteReadPort;
    input: pub write_read_in;
    handler: write_read_handler;
    returns: I2cStatus;
    args { val addr: u32, mut write_buffer: Buffer, mut read_buffer: Buffer }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fprime_fw::{Deserialize, Endianness, LinearBuffer, SerBuf, Serialize, SerializeStatus};

    /// Scriptable in-test backend.
    #[derive(Default)]
    struct FakeI2c {
        open_result: Mutex<bool>,
        write_status: Mutex<I2cStatus>,
        read_status: Mutex<I2cStatus>,
        write_read_status: Mutex<I2cStatus>,
        /// Bytes handed to `write` / the write half of `write_read`.
        written: Mutex<Vec<(u32, Vec<u8>)>>,
        /// Payload copied into every read window.
        read_payload: Mutex<Vec<u8>>,
    }

    impl FakeI2c {
        fn opened() -> Arc<Self> {
            Arc::new(Self {
                open_result: Mutex::new(true),
                ..Self::default()
            })
        }
    }

    impl I2cBackend for Arc<FakeI2c> {
        fn open(&self, _device: &str) -> bool {
            *self.open_result.lock().unwrap()
        }
        fn write(&self, addr: u32, data: &[u8]) -> I2cStatus {
            self.written.lock().unwrap().push((addr, data.to_vec()));
            *self.write_status.lock().unwrap()
        }
        fn read(&self, _addr: u32, dest: &mut [u8]) -> I2cStatus {
            let payload = self.read_payload.lock().unwrap();
            let count = payload.len().min(dest.len());
            dest[..count].copy_from_slice(&payload[..count]);
            *self.read_status.lock().unwrap()
        }
        fn write_read(&self, addr: u32, write: &[u8], read: &mut [u8]) -> I2cStatus {
            self.written.lock().unwrap().push((addr, write.to_vec()));
            let payload = self.read_payload.lock().unwrap();
            let count = payload.len().min(read.len());
            read[..count].copy_from_slice(&payload[..count]);
            *self.write_read_status.lock().unwrap()
        }
    }

    fn buffer_with(data: &[u8]) -> Buffer {
        let mut buffer = Buffer::allocate(data.len());
        buffer.data_mut().copy_from_slice(data);
        buffer
    }

    // -- enum / wire format --------------------------------------------------

    #[test]
    fn i2c_status_discriminants_match_fpp() {
        assert_eq!(I2cStatus::I2cOk as u8, 0);
        assert_eq!(I2cStatus::I2cAddressErr as u8, 1);
        assert_eq!(I2cStatus::I2cWriteErr as u8, 2);
        assert_eq!(I2cStatus::I2cReadErr as u8, 3);
        assert_eq!(I2cStatus::I2cOpenErr as u8, 4);
        assert_eq!(I2cStatus::I2cOtherErr as u8, 5);
        assert_eq!(I2cStatus::NUM_CONSTANTS, 6);
        assert_eq!(I2C_DRIVER_PORTS, 10);
    }

    #[test]
    fn i2c_status_serializes_at_u8_width_and_rejects_undeclared_values() {
        let mut buf = LinearBuffer::<4>::new();
        assert!(
            I2cStatus::I2cOpenErr
                .serialize_to(&mut buf, Endianness::Big)
                .is_ok()
        );
        assert_eq!(buf.as_slice(), &[4]);

        let mut buf = LinearBuffer::<4>::new();
        assert!(buf.serialize_u8_be(6).is_ok());
        let mut out = I2cStatus::I2cReadErr;
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserFormatError
        );
        assert_eq!(out, I2cStatus::I2cReadErr);
    }

    // -- stub backend (upstream stub parity) ---------------------------------

    #[test]
    fn stub_backend_matches_the_cpp_stub() {
        let driver = LinuxI2cDriver::new("i2cDrv");
        assert!(driver.open("/dev/i2c-1"));
        assert!(driver.is_open());
        assert_eq!(driver.device(), "/dev/i2c-1");

        let mut buffer = buffer_with(&[1, 2, 3]);
        let write = driver.write_in(0);
        assert_eq!(
            write.target.invoke(write.port_num, 0x48, &mut buffer),
            I2cStatus::I2cOk
        );
        let read = driver.read_in(0);
        assert_eq!(
            read.target.invoke(read.port_num, 0x48, &mut buffer),
            I2cStatus::I2cOk
        );
        let mut read_buffer = Buffer::allocate(2);
        let write_read = driver.write_read_in(0);
        assert_eq!(
            write_read
                .target
                .invoke(write_read.port_num, 0x48, &mut buffer, &mut read_buffer),
            I2cStatus::I2cOk
        );
    }

    // -- open gating ----------------------------------------------------------

    #[test]
    fn all_three_ports_report_open_error_before_open() {
        let fake = FakeI2c::opened();
        let driver = LinuxI2cDriver::with_backend("i2c", Box::new(fake.clone()));
        let mut buffer = buffer_with(&[9]);
        let mut read_buffer = Buffer::allocate(1);
        let write = driver.write_in(0);
        let read = driver.read_in(0);
        let write_read = driver.write_read_in(0);
        assert_eq!(
            write.target.invoke(write.port_num, 1, &mut buffer),
            I2cStatus::I2cOpenErr
        );
        assert_eq!(
            read.target.invoke(read.port_num, 1, &mut buffer),
            I2cStatus::I2cOpenErr
        );
        assert_eq!(
            write_read
                .target
                .invoke(write_read.port_num, 1, &mut buffer, &mut read_buffer),
            I2cStatus::I2cOpenErr
        );
        // Nothing reached the bus.
        assert!(fake.written.lock().unwrap().is_empty());
    }

    #[test]
    fn failed_open_leaves_the_driver_closed() {
        let fake = Arc::new(FakeI2c::default());
        let driver = LinuxI2cDriver::with_backend("i2c", Box::new(fake.clone()));
        assert!(!driver.open("/dev/i2c-9"));
        assert!(!driver.is_open());
    }

    #[test]
    fn close_reverts_to_the_open_error_path() {
        let fake = FakeI2c::opened();
        let driver = LinuxI2cDriver::with_backend("i2c", Box::new(fake.clone()));
        assert!(driver.open("/dev/i2c-1"));
        driver.close();
        let mut buffer = buffer_with(&[1]);
        let write = driver.write_in(0);
        assert_eq!(
            write.target.invoke(write.port_num, 1, &mut buffer),
            I2cStatus::I2cOpenErr
        );
    }

    // -- transfers ------------------------------------------------------------

    #[test]
    fn write_forwards_the_address_and_data_window() {
        let fake = FakeI2c::opened();
        let driver = LinuxI2cDriver::with_backend("i2c", Box::new(fake.clone()));
        assert!(driver.open("/dev/i2c-1"));
        let mut buffer = buffer_with(&[0xDE, 0xAD, 0xBE, 0xEF]);
        let write = driver.write_in(0);
        assert_eq!(
            write.target.invoke(write.port_num, 0x2A, &mut buffer),
            I2cStatus::I2cOk
        );
        assert_eq!(
            fake.written.lock().unwrap().clone(),
            vec![(0x2A, vec![0xDE, 0xAD, 0xBE, 0xEF])]
        );
    }

    #[test]
    fn read_fills_the_caller_sized_window() {
        let fake = FakeI2c::opened();
        *fake.read_payload.lock().unwrap() = vec![0x11, 0x22, 0x33, 0x44];
        let driver = LinuxI2cDriver::with_backend("i2c", Box::new(fake.clone()));
        assert!(driver.open("/dev/i2c-1"));
        let mut buffer = Buffer::allocate(2);
        let read = driver.read_in(0);
        assert_eq!(
            read.target.invoke(read.port_num, 0x2A, &mut buffer),
            I2cStatus::I2cOk
        );
        assert_eq!(buffer.data(), &[0x11, 0x22]);
    }

    #[test]
    fn every_backend_status_is_propagated_verbatim() {
        let fake = FakeI2c::opened();
        let driver = LinuxI2cDriver::with_backend("i2c", Box::new(fake.clone()));
        assert!(driver.open("/dev/i2c-1"));
        let mut buffer = buffer_with(&[1]);
        let mut read_buffer = Buffer::allocate(1);

        for status in [
            I2cStatus::I2cOk,
            I2cStatus::I2cAddressErr,
            I2cStatus::I2cWriteErr,
            I2cStatus::I2cReadErr,
            I2cStatus::I2cOtherErr,
        ] {
            *fake.write_status.lock().unwrap() = status;
            *fake.read_status.lock().unwrap() = status;
            let write = driver.write_in(0);
            let read = driver.read_in(0);
            assert_eq!(write.target.invoke(write.port_num, 1, &mut buffer), status);
            assert_eq!(read.target.invoke(read.port_num, 1, &mut buffer), status);
        }

        // The combined transfer only ever reports OK or OTHER_ERR — one
        // ioctl, one result (the documented backend contract).
        for status in [I2cStatus::I2cOk, I2cStatus::I2cOtherErr] {
            *fake.write_read_status.lock().unwrap() = status;
            let write_read = driver.write_read_in(0);
            assert_eq!(
                write_read
                    .target
                    .invoke(write_read.port_num, 1, &mut buffer, &mut read_buffer),
                status
            );
        }
    }

    #[test]
    fn write_read_performs_the_combined_transfer_in_one_call() {
        let fake = FakeI2c::opened();
        *fake.read_payload.lock().unwrap() = vec![0xAA, 0xBB];
        let driver = LinuxI2cDriver::with_backend("i2c", Box::new(fake.clone()));
        assert!(driver.open("/dev/i2c-1"));
        let mut write_buffer = buffer_with(&[0x01]);
        let mut read_buffer = Buffer::allocate(2);
        let port = driver.write_read_in(0);
        assert_eq!(
            port.target
                .invoke(port.port_num, 0x50, &mut write_buffer, &mut read_buffer),
            I2cStatus::I2cOk
        );
        // Exactly ONE bus transaction (repeated START, no STOP between).
        assert_eq!(
            fake.written.lock().unwrap().clone(),
            vec![(0x50, vec![0x01])]
        );
        assert_eq!(read_buffer.data(), &[0xAA, 0xBB]);
    }

    // -- programmer-error assertions -----------------------------------------

    #[test]
    #[should_panic]
    fn write_asserts_on_an_invalid_buffer() {
        let fake = FakeI2c::opened();
        let driver = LinuxI2cDriver::with_backend("i2c", Box::new(fake));
        assert!(driver.open("/dev/i2c-1"));
        let mut invalid = Buffer::empty();
        let write = driver.write_in(0);
        let _ = write.target.invoke(write.port_num, 1, &mut invalid);
    }

    #[test]
    #[should_panic]
    fn write_read_asserts_on_an_address_that_does_not_fit_u16() {
        let fake = FakeI2c::opened();
        let driver = LinuxI2cDriver::with_backend("i2c", Box::new(fake));
        assert!(driver.open("/dev/i2c-1"));
        let mut write_buffer = buffer_with(&[1]);
        let mut read_buffer = Buffer::allocate(1);
        let port = driver.write_read_in(0);
        let _ = port
            .target
            .invoke(port.port_num, 0x1_0000, &mut write_buffer, &mut read_buffer);
    }

    #[test]
    fn write_and_read_pass_oversized_addresses_through_unchecked() {
        // C++ parity: only writeRead range-checks the address; write/read
        // hand the raw U32 to ioctl(I2C_SLAVE).
        let fake = FakeI2c::opened();
        let driver = LinuxI2cDriver::with_backend("i2c", Box::new(fake.clone()));
        assert!(driver.open("/dev/i2c-1"));
        let mut buffer = buffer_with(&[1]);
        let write = driver.write_in(0);
        assert_eq!(
            write
                .target
                .invoke(write.port_num, 0xFFFF_FFFF, &mut buffer),
            I2cStatus::I2cOk
        );
        assert_eq!(fake.written.lock().unwrap()[0].0, 0xFFFF_FFFF);
    }
}
