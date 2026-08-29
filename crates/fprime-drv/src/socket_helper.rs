//! # SocketHelper — port of `Drv::SocketComponentHelper` (+ `Drv/Ip/IpSocket` core)
//!
//! C++ sources: `Drv/Ip/SocketComponentHelper.{cpp,hpp}`, `Drv/Ip/IpSocket.cpp`,
//! `default/config/IpCfg.hpp`.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (Drv/Ip sockets +
//! SocketComponentHelper).
//!
//! Runs TWO `std` threads per driver, exactly as C++ runs two `Os::Task`s:
//!
//! - a **read loop** — blocking `recv` into a buffer obtained from the
//!   component (`allocate_out`), delivered via the component's `recv_out`;
//! - a **reconnect loop** — the ONLY place sockets are opened; polls every
//!   `reconnect_check_interval` (50 ms default) for reconnect requests, and
//!   fires the component's `connected()` (→ `ready_out`) on success.
//!
//! Requesters (`send`, the read loop) never open sockets themselves: they
//! `request_reconnect()` and wait in `reconnect_wait_interval` (10 ms) steps
//! up to `reconnect_wait_timeout` (1 s default).
//!
//! ## Socket-handle semantics (Rust divergence, behavior-preserving)
//!
//! C++ copies the raw `fd` out of the lock and does blocking calls on the
//! copy; `close()`/`shutdown()` on the shared fd break a blocking `recv`.
//! Rust `TcpStream` is owned, so each operation clones a handle
//! (`try_clone` = `dup(2)`) out of the lock, and `stop()`/`close()` call
//! `TcpStream::shutdown(Both)` on the stored stream — which breaks blocking
//! reads on every clone of the same underlying socket, exactly like the C++
//! `shutdown()`. As belt-and-braces (some platforms/wrappers do not wake a
//! blocked reader on shutdown), an optional read timeout
//! ([`Timing::read_timeout`], default 500 ms) bounds each blocking `recv`;
//! a timeout maps to `SOCK_NO_DATA_AVAILABLE` — the same status C++ produces
//! for `EAGAIN`, and one the read loop treats as benign.

use crate::byte_stream::ByteStreamStatus;
use fprime_fw::{Buffer, fw_assert, fw_log};
use fprime_os::Task;
use fprime_os::task::{Arguments, State as TaskState, Status as TaskStatus};
use std::io::{ErrorKind, Read, Write};
use std::net::{Shutdown, TcpStream};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Port of `Drv::SocketIpStatus` (`Drv/Ip/IpSocket.hpp`). Exact C++
/// discriminants; all values are negative except `Success = 0`, and `-12` is
/// unused (C++ parity — do not "fix").
#[must_use]
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SocketIpStatus {
    /// Socket operation successful.
    Success = 0,
    /// Socket open failed.
    FailedToGetSocket = -1,
    /// Host IP lookup failed.
    FailedToGetHostIp = -2,
    /// Bad IP address supplied.
    InvalidIpAddress = -3,
    /// Failed to connect socket.
    FailedToConnect = -4,
    /// Failed to configure socket.
    FailedToSetSocketOptions = -5,
    /// Interrupted status for retries.
    InterruptedTryAgain = -6,
    /// Failed to read socket.
    ReadError = -7,
    /// Failed to read socket with disconnect.
    Disconnected = -8,
    /// Failed to bind to socket.
    FailedToBind = -9,
    /// Failed to listen on socket.
    FailedToListen = -10,
    /// Failed to accept connection.
    FailedToAccept = -11,
    // -12 intentionally unused (C++ parity).
    /// Failed to send after configured retries.
    SendError = -13,
    /// Socket has not been started.
    NotStarted = -14,
    /// Failed to read back port from connection.
    FailedToReadBackPort = -15,
    /// No data available or read operation would block.
    NoDataAvailable = -16,
    /// Another thread is opening.
    AnotherThreadOpening = -17,
    /// Automatic connections are disabled.
    AutoConnectDisabled = -18,
    /// Operation is invalid.
    InvalidCall = -19,
}

/// `IpCfg::SOCKET_MAX_ITERATIONS` — maximum send/recv attempts before an
/// error is returned.
pub const SOCKET_MAX_ITERATIONS: usize = 0xFFFF;

/// Driver timing knobs. Defaults are the C++ values
/// (`SocketComponentHelper.hpp` intervals, `IpCfg.hpp` timeouts); tests
/// shrink them via [`SocketHelper::set_timing`].
#[derive(Debug, Clone, Copy)]
pub struct Timing {
    /// Reconnect-loop poll interval (C++ `m_reconnectCheckInterval`, 50 ms).
    pub reconnect_check_interval: Duration,
    /// Requester wait step (C++ `m_reconnectWaitInterval`, 10 ms).
    pub reconnect_wait_interval: Duration,
    /// Requester total wait (C++ `waitForReconnect` default timeout, 1 s).
    pub reconnect_wait_timeout: Duration,
    /// Retry delay after failed open / failed buffer allocation
    /// (`IpCfg` `SOCKET_RETRY_INTERVAL`, 1 s).
    pub retry_interval: Duration,
    /// Per-`recv` read timeout (Rust belt-and-braces; see module docs).
    /// `None` = fully blocking, exact C++ behavior.
    pub read_timeout: Option<Duration>,
    /// Send timeout (`SO_SNDTIMEO`; `IpCfg` `SOCKET_SEND_TIMEOUT_*`, 1 s).
    pub write_timeout: Option<Duration>,
    /// TCP client connect timeout (C++ blocks in `connect(2)`; a bound keeps
    /// the reconnect loop responsive to `stop()`).
    pub connect_timeout: Duration,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            reconnect_check_interval: Duration::from_millis(50),
            reconnect_wait_interval: Duration::from_millis(10),
            reconnect_wait_timeout: Duration::from_secs(1),
            retry_interval: Duration::from_secs(1),
            read_timeout: Some(Duration::from_millis(500)),
            write_timeout: Some(Duration::from_secs(1)),
            connect_timeout: Duration::from_secs(1),
        }
    }
}

/// C++ `SocketComponentHelper::OpenState` (`SKIP` is a local-only value in
/// C++ and not represented in the shared state).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OpenState {
    NotOpen,
    Opening,
    Open,
}

/// C++ `SocketComponentHelper::ReconnectState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReconnectState {
    NotReconnecting,
    RequestReconnect,
    ReconnectInProgress,
}

/// Socket + read-loop shared state (C++ fields under `m_lock`).
struct SocketState {
    stream: Option<TcpStream>,
    open: OpenState,
    /// Stops the read loop when true (C++ `m_stop`, initially true).
    stop: bool,
    /// Automatic (re)open enabled (C++ `m_reopen`, default true).
    reopen: bool,
}

/// Reconnect-loop shared state (C++ fields under `m_reconnectLock`).
struct ReconnectShared {
    state: ReconnectState,
    stop: bool,
}

/// The component callbacks the helper's loops need — the C++ pure-virtual
/// surface of `SocketComponentHelper` (`getSocketHandler` collapses into
/// [`SocketWorker::open_protocol`], since only the reconnect thread opens).
pub trait SocketWorker: Send + Sync + 'static {
    /// The helper embedded in the component.
    fn helper(&self) -> &SocketHelper;

    /// Open the protocol-level connection (client: `connect`; server:
    /// `accept`). Called ONLY from the reconnect thread (via
    /// `SocketHelper::open`). Returns the connected stream or the C++
    /// `SocketIpStatus` failure.
    fn open_protocol(&self) -> Result<TcpStream, SocketIpStatus>;

    /// C++ `getBuffer()` — obtain a receive buffer (component `allocate_out`).
    fn get_buffer(&self) -> Buffer;

    /// C++ `sendBuffer()` — deliver a filled (or failed) receive buffer
    /// (component `recv_out` with the mapped [`ByteStreamStatus`]).
    fn send_buffer(&self, buffer: Buffer, status: SocketIpStatus);

    /// C++ `connected()` — fired on every successful open (`ready_out`).
    fn connected(&self);

    /// The read-thread body. Default = the generic
    /// `SocketComponentHelper::readLoop`; `TcpServer` overrides it to retry
    /// `startup()` first and `terminate()` after (C++ parity).
    fn read_loop(&self) {
        self.helper().read_loop_body(self);
    }
}

/// Port of `Drv::SocketComponentHelper`: owns the two driver threads and the
/// shared socket state. Embedded by [`crate::TcpClient`] / [`crate::TcpServer`].
pub struct SocketHelper {
    state: Mutex<SocketState>,
    reconnect: Mutex<ReconnectShared>,
    timing: Mutex<Timing>,
    read_task: Task,
    reconnect_task: Task,
}

impl Default for SocketHelper {
    fn default() -> Self {
        Self::new()
    }
}

impl SocketHelper {
    /// Construct with C++ initial state: stopped, not open, auto-open on.
    pub fn new() -> Self {
        Self {
            state: Mutex::new(SocketState {
                stream: None,
                open: OpenState::NotOpen,
                stop: true,
                reopen: true,
            }),
            reconnect: Mutex::new(ReconnectShared {
                state: ReconnectState::NotReconnecting,
                stop: true,
            }),
            timing: Mutex::new(Timing::default()),
            read_task: Task::new(),
            reconnect_task: Task::new(),
        }
    }

    /// Replace the timing knobs. Call before [`SocketHelper::start`].
    pub fn set_timing(&self, timing: Timing) {
        *self.timing.lock().unwrap() = timing;
    }

    /// Current timing knobs (copied).
    pub fn timing(&self) -> Timing {
        *self.timing.lock().unwrap()
    }

    /// C++ `start()`: spawn the reconnect thread, then the read thread.
    /// Starting twice is a coding error (C++ FW_ASSERT on task state).
    pub fn start<C: SocketWorker>(&self, component: &Arc<C>, name: &str) {
        // C++ parity: the helper being started must be the component's own.
        fw_assert!(std::ptr::eq(component.helper(), self));

        // Reconnect thread first (C++ order).
        fw_assert!(self.reconnect_task.get_state() == TaskState::NotStarted);
        self.reconnect.lock().unwrap().stop = false;
        let comp = component.clone();
        let status = self.reconnect_task.start(Arguments::new(
            &format!("{name}_reconnect"),
            Box::new(move || comp.helper().reconnect_loop_body(&*comp)),
        ));
        fw_assert!(status == TaskStatus::OpOk, status as i32);

        // Read thread.
        fw_assert!(self.read_task.get_state() == TaskState::NotStarted);
        self.state.lock().unwrap().stop = false;
        let comp = component.clone();
        let status = self
            .read_task
            .start(Arguments::new(name, Box::new(move || comp.read_loop())));
        fw_assert!(status == TaskStatus::OpOk, status as i32);
    }

    /// C++ `open()`: transition NOT_OPEN → OPENING → OPEN with the socket
    /// opened OUTSIDE the lock; any concurrent caller gets
    /// `AnotherThreadOpening`. Fires `connected()` on success, outside the
    /// lock (C++ parity).
    pub fn open<C: SocketWorker + ?Sized>(&self, component: &C) -> SocketIpStatus {
        let opening = {
            let mut state = self.state.lock().unwrap();
            if state.open == OpenState::NotOpen {
                state.open = OpenState::Opening;
                // C++ parity: FW_ASSERT(descriptor.fd == -1) — never open
                // over an existing socket.
                fw_assert!(state.stream.is_none());
                true
            } else {
                false
            }
        };
        if !opening {
            return SocketIpStatus::AnotherThreadOpening;
        }
        match component.open_protocol() {
            Ok(stream) => {
                {
                    let mut state = self.state.lock().unwrap();
                    state.stream = Some(stream);
                    state.open = OpenState::Open;
                }
                // Notify connection on success outside the locked scope.
                component.connected();
                SocketIpStatus::Success
            }
            Err(status) => {
                let mut state = self.state.lock().unwrap();
                state.open = OpenState::NotOpen;
                state.stream = None;
                status
            }
        }
    }

    /// C++ `isOpened()`.
    pub fn is_opened(&self) -> bool {
        self.state.lock().unwrap().open == OpenState::Open
    }

    /// C++ `setAutomaticOpen()`.
    pub fn set_automatic_open(&self, auto_open: bool) {
        self.state.lock().unwrap().reopen = auto_open;
    }

    /// C++ `getAutomaticOpen()`.
    pub fn get_automatic_open(&self) -> bool {
        self.state.lock().unwrap().reopen
    }

    /// C++ `send()`: when the socket is not open, request a reconnect and
    /// wait for the reconnect thread (bounded by
    /// `Timing::reconnect_wait_timeout`); on success send the whole slice
    /// with the bounded-retry accumulation loop. `Disconnected` closes the
    /// socket.
    pub fn send(&self, data: &[u8]) -> SocketIpStatus {
        let mut stream = self.clone_stream();
        // Prevent transmission before connection, or after a disconnect.
        if stream.is_none() {
            self.request_reconnect();
            let reconnect_status = self.wait_for_reconnect();
            if reconnect_status == SocketIpStatus::Success {
                // Refresh the local copy after reopen.
                stream = self.clone_stream();
                if stream.is_none() {
                    return SocketIpStatus::Disconnected;
                }
            } else {
                return reconnect_status;
            }
        }
        let status = socket_send(&stream.unwrap(), data);
        if status == SocketIpStatus::Disconnected {
            self.close();
        }
        status
    }

    /// C++ `recv()`: fills `data`, sets `*size` to the bytes read (0 on any
    /// non-success). `Disconnected` closes the socket.
    pub fn recv(&self, data: &mut [u8], size: &mut usize) -> SocketIpStatus {
        // Check for a previously disconnected socket.
        let Some(stream) = self.clone_stream() else {
            *size = 0;
            return SocketIpStatus::Disconnected;
        };
        let status = socket_recv(&stream, data, size);
        if status == SocketIpStatus::Disconnected {
            self.close();
        }
        status
    }

    /// C++ `shutdown()`: begin closing communications — breaks blocking
    /// reads on every clone of the stream.
    pub fn shutdown(&self) {
        let state = self.state.lock().unwrap();
        if let Some(stream) = state.stream.as_ref() {
            let _ = stream.shutdown(Shutdown::Both);
        }
    }

    /// C++ `close()`: shut down and drop the stream, mark NOT_OPEN.
    /// (Rust: `shutdown` before drop so clones held by the read thread are
    /// woken — dropping our dup alone would not close the connection.)
    pub fn close(&self) {
        let mut state = self.state.lock().unwrap();
        if let Some(stream) = state.stream.take() {
            let _ = stream.shutdown(Shutdown::Both);
        }
        state.open = OpenState::NotOpen;
    }

    /// C++ `stop()`: flag both loops to stop and shut the socket down to
    /// break any blocking recv. Stopping before start is permitted.
    pub fn stop(&self) {
        self.state.lock().unwrap().stop = true;
        self.stop_reconnect();
        self.shutdown();
    }

    /// C++ `running()` — read loop keeps going.
    pub fn running(&self) -> bool {
        !self.state.lock().unwrap().stop
    }

    /// C++ `runningReconnect()`.
    pub fn running_reconnect(&self) -> bool {
        !self.reconnect.lock().unwrap().stop
    }

    /// C++ `stopReconnect()`.
    pub fn stop_reconnect(&self) {
        let mut rec = self.reconnect.lock().unwrap();
        rec.state = ReconnectState::NotReconnecting;
        rec.stop = true;
    }

    /// C++ `join()`: join the read task, then the reconnect task; the read
    /// task's status wins when it is not OK.
    pub fn join(&self) -> TaskStatus {
        let status = self.read_task.join();
        let reconnect_status = self.reconnect_task.join();
        if status == TaskStatus::OpOk {
            return reconnect_status;
        }
        status
    }

    /// C++ `requestReconnect()`: only flips NOT_RECONNECTING →
    /// REQUEST_RECONNECT (an in-progress reconnect absorbs the request).
    pub fn request_reconnect(&self) {
        let mut rec = self.reconnect.lock().unwrap();
        if rec.state == ReconnectState::NotReconnecting {
            rec.state = ReconnectState::RequestReconnect;
        }
    }

    /// C++ `waitForReconnect()`: poll in `reconnect_wait_interval` steps up
    /// to `reconnect_wait_timeout` for the reconnect thread to finish, then
    /// report `Success` / `AutoConnectDisabled` / `Disconnected`.
    pub fn wait_for_reconnect(&self) -> SocketIpStatus {
        // Do not wait at all when automatic open is disabled.
        if !self.get_automatic_open() {
            return SocketIpStatus::AutoConnectDisabled;
        }
        let timing = self.timing();
        let deadline = Instant::now() + timing.reconnect_wait_timeout;
        while Instant::now() < deadline {
            {
                let rec = self.reconnect.lock().unwrap();
                // Done waiting when the reconnect thread is idle or stopped.
                if rec.state == ReconnectState::NotReconnecting || rec.stop {
                    break;
                }
            }
            std::thread::sleep(timing.reconnect_wait_interval);
        }
        if self.is_opened() {
            return SocketIpStatus::Success;
        }
        // Check one more time whether auto-open got disabled during the wait.
        if !self.get_automatic_open() {
            return SocketIpStatus::AutoConnectDisabled;
        }
        SocketIpStatus::Disconnected // another reopen is needed
    }

    /// C++ `readLoop()` — the generic read-thread body (see module docs).
    pub fn read_loop_body<C: SocketWorker + ?Sized>(&self, component: &C) {
        loop {
            // Prevent reception before connection, or after a disconnect.
            if !self.is_opened() && self.running() {
                self.request_reconnect();
                let status = self.wait_for_reconnect();
                // When auto-open is disabled this is the loop exit condition.
                if status == SocketIpStatus::AutoConnectDisabled {
                    break;
                }
            }
            // If the network connection is open, read from it.
            if self.is_opened() && self.running() {
                let mut buffer = component.get_buffer();
                if buffer.is_valid() {
                    let mut size = buffer.size();
                    // recv blocks, so it may have been a while since the
                    // last isOpened check.
                    let status = self.recv(buffer.data_mut(), &mut size);
                    if status != SocketIpStatus::Success
                        && status != SocketIpStatus::InterruptedTryAgain
                        && status != SocketIpStatus::NoDataAvailable
                    {
                        fw_log!(
                            "[WARNING] socket read loop failed to recv with status {}",
                            status as i32
                        );
                        self.close();
                        buffer.set_size(0);
                    } else {
                        buffer.set_size(size);
                    }
                    component.send_buffer(buffer, status);
                } else {
                    fw_log!("[WARNING] socket read loop failed to get buffer for recv");
                    std::thread::sleep(self.timing().retry_interval);
                }
            }
            // C++ do { ... } while (running()).
            if !self.running() {
                break;
            }
        }
        // Close the port entirely.
        self.close();
    }

    /// C++ `reconnectLoop()` — the reconnect-thread body: the ONLY place
    /// sockets are opened.
    pub fn reconnect_loop_body<C: SocketWorker + ?Sized>(&self, component: &C) {
        while self.running_reconnect() {
            let reconnect = {
                let mut rec = self.reconnect.lock().unwrap();
                match rec.state {
                    ReconnectState::RequestReconnect => {
                        rec.state = ReconnectState::ReconnectInProgress;
                        true
                    }
                    ReconnectState::ReconnectInProgress => true,
                    ReconnectState::NotReconnecting => false,
                }
            };
            if reconnect {
                let status = self.reopen(component);
                if status == SocketIpStatus::AutoConnectDisabled
                    || status == SocketIpStatus::Success
                {
                    // Done (or told not to try): stop reconnecting.
                    self.reconnect.lock().unwrap().state = ReconnectState::NotReconnecting;
                } else {
                    // Keep trying — NO reconnect state change (C++ parity).
                    fw_log!(
                        "[WARNING] socket reconnect failed to open with status {}",
                        status as i32
                    );
                    std::thread::sleep(self.timing().retry_interval);
                }
            } else {
                std::thread::sleep(self.timing().reconnect_check_interval);
            }
        }
    }

    /// C++ private `reopen()`.
    fn reopen<C: SocketWorker + ?Sized>(&self, component: &C) -> SocketIpStatus {
        if self.is_opened() {
            return SocketIpStatus::Success;
        }
        if !self.get_automatic_open() {
            return SocketIpStatus::AutoConnectDisabled;
        }
        let status = self.open(component);
        // C++ parity: a concurrent open in progress counts as success here.
        if status == SocketIpStatus::AnotherThreadOpening {
            return SocketIpStatus::Success;
        }
        status
    }

    /// Duplicate the current stream handle (C++ "copy the fd out of the
    /// lock"). A `try_clone` failure is treated like a closed socket.
    fn clone_stream(&self) -> Option<TcpStream> {
        let state = self.state.lock().unwrap();
        state.stream.as_ref().and_then(|s| s.try_clone().ok())
    }
}

/// C++ `IpSocket::send()` over an `std` stream: zero-size sends are a no-op;
/// bounded accumulation loop; `EINTR`/zero-write retries; `ECONNRESET` (and
/// kin) → `Disconnected`; any other error → `SendError`; incomplete after
/// `SOCKET_MAX_ITERATIONS` → `InterruptedTryAgain`.
fn socket_send(stream: &TcpStream, data: &[u8]) -> SocketIpStatus {
    if data.is_empty() {
        return SocketIpStatus::Success;
    }
    let mut writer: &TcpStream = stream;
    let mut total: usize = 0;
    for _ in 0..SOCKET_MAX_ITERATIONS {
        if total >= data.len() {
            break;
        }
        match writer.write(&data[total..]) {
            // Zero-byte write or EINTR: just try again (C++ parity).
            Ok(0) => continue,
            Ok(sent) => total += sent,
            Err(e) => match e.kind() {
                ErrorKind::Interrupted => continue,
                // C++: EBADF/ECONNRESET → SOCK_DISCONNECTED (EBADF is
                // unrepresentable through std; ConnectionAborted covers the
                // reset family).
                ErrorKind::ConnectionReset | ErrorKind::ConnectionAborted => {
                    return SocketIpStatus::Disconnected;
                }
                // Everything else (incl. EPIPE/BrokenPipe and SO_SNDTIMEO
                // expiry) → SOCK_SEND_ERROR, exactly as C++.
                _ => return SocketIpStatus::SendError,
            },
        }
    }
    if total < data.len() {
        return SocketIpStatus::InterruptedTryAgain;
    }
    SocketIpStatus::Success
}

/// C++ `IpSocket::recv()` over an `std` stream. `*size` in: capacity used;
/// out: bytes read (0 on non-success). TCP zero-read ⇒ orderly peer shutdown
/// ⇒ `Disconnected` (C++ `handleZeroReturn`).
fn socket_recv(stream: &TcpStream, data: &mut [u8], size: &mut usize) -> SocketIpStatus {
    let mut reader: &TcpStream = stream;
    // Loop primarily for EINTR; other conditions exit earlier (C++ parity).
    for _ in 0..SOCKET_MAX_ITERATIONS {
        match reader.read(data) {
            Ok(n) if n > 0 => {
                *size = n;
                return SocketIpStatus::Success;
            }
            Ok(_) => {
                *size = 0;
                return SocketIpStatus::Disconnected;
            }
            Err(e) => match e.kind() {
                // EAGAIN/EWOULDBLOCK, incl. SO_RCVTIMEO expiry (reported as
                // either kind depending on platform).
                ErrorKind::WouldBlock | ErrorKind::TimedOut => {
                    *size = 0;
                    return SocketIpStatus::NoDataAvailable;
                }
                ErrorKind::ConnectionReset | ErrorKind::ConnectionAborted => {
                    *size = 0;
                    return SocketIpStatus::Disconnected;
                }
                ErrorKind::Interrupted => continue,
                _ => {
                    *size = 0;
                    return SocketIpStatus::ReadError;
                }
            },
        }
    }
    // SOCKET_MAX_ITERATIONS of EINTR occurred.
    *size = 0;
    SocketIpStatus::InterruptedTryAgain
}

/// C++ `TcpClient/TcpServer::sendBuffer()` receive-status mapping:
/// `SOCK_SUCCESS → OP_OK`, `SOCK_NO_DATA_AVAILABLE → RECV_NO_DATA`, all
/// else → `OTHER_ERROR`.
pub fn byte_stream_recv_status(status: SocketIpStatus) -> ByteStreamStatus {
    match status {
        SocketIpStatus::Success => ByteStreamStatus::OpOk,
        SocketIpStatus::NoDataAvailable => ByteStreamStatus::RecvNoData,
        _ => ByteStreamStatus::OtherError,
    }
}

/// C++ `TcpClient/TcpServer::send_handler()` status mapping: ONLY
/// `SOCK_INTERRUPTED_TRY_AGAIN → SEND_RETRY`; `SOCK_SUCCESS → OP_OK`; all
/// else → `OTHER_ERROR`.
pub fn byte_stream_send_status(status: SocketIpStatus) -> ByteStreamStatus {
    match status {
        SocketIpStatus::InterruptedTryAgain => ByteStreamStatus::SendRetry,
        SocketIpStatus::Success => ByteStreamStatus::OpOk,
        _ => ByteStreamStatus::OtherError,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn socket_ip_status_discriminants_match_cpp() {
        assert_eq!(SocketIpStatus::Success as i32, 0);
        assert_eq!(SocketIpStatus::FailedToGetSocket as i32, -1);
        assert_eq!(SocketIpStatus::InterruptedTryAgain as i32, -6);
        assert_eq!(SocketIpStatus::Disconnected as i32, -8);
        assert_eq!(SocketIpStatus::FailedToAccept as i32, -11);
        // -12 is skipped in C++ — SEND_ERROR is -13.
        assert_eq!(SocketIpStatus::SendError as i32, -13);
        assert_eq!(SocketIpStatus::NotStarted as i32, -14);
        assert_eq!(SocketIpStatus::NoDataAvailable as i32, -16);
        assert_eq!(SocketIpStatus::AutoConnectDisabled as i32, -18);
        assert_eq!(SocketIpStatus::InvalidCall as i32, -19);
    }

    #[test]
    fn recv_status_mapping_matches_cpp_send_buffer() {
        assert_eq!(
            byte_stream_recv_status(SocketIpStatus::Success),
            ByteStreamStatus::OpOk
        );
        assert_eq!(
            byte_stream_recv_status(SocketIpStatus::NoDataAvailable),
            ByteStreamStatus::RecvNoData
        );
        // Everything else, including retry statuses, is OTHER_ERROR.
        for other in [
            SocketIpStatus::Disconnected,
            SocketIpStatus::ReadError,
            SocketIpStatus::InterruptedTryAgain,
            SocketIpStatus::SendError,
            SocketIpStatus::AutoConnectDisabled,
        ] {
            assert_eq!(byte_stream_recv_status(other), ByteStreamStatus::OtherError);
        }
    }

    #[test]
    fn send_status_mapping_matches_cpp_send_handler() {
        assert_eq!(
            byte_stream_send_status(SocketIpStatus::Success),
            ByteStreamStatus::OpOk
        );
        assert_eq!(
            byte_stream_send_status(SocketIpStatus::InterruptedTryAgain),
            ByteStreamStatus::SendRetry
        );
        for other in [
            SocketIpStatus::Disconnected,
            SocketIpStatus::SendError,
            SocketIpStatus::ReadError,
            SocketIpStatus::NoDataAvailable,
            SocketIpStatus::AutoConnectDisabled,
        ] {
            assert_eq!(byte_stream_send_status(other), ByteStreamStatus::OtherError);
        }
    }

    #[test]
    fn recv_on_closed_helper_is_disconnected() {
        let helper = SocketHelper::new();
        let mut data = [0u8; 8];
        let mut size = data.len();
        let status = helper.recv(&mut data, &mut size);
        assert_eq!(status, SocketIpStatus::Disconnected);
        assert_eq!(size, 0);
        assert!(!helper.is_opened());
    }

    #[test]
    fn wait_for_reconnect_with_auto_open_disabled_returns_immediately() {
        let helper = SocketHelper::new();
        helper.set_automatic_open(false);
        let start = Instant::now();
        let status = helper.wait_for_reconnect();
        assert_eq!(status, SocketIpStatus::AutoConnectDisabled);
        assert!(start.elapsed() < Duration::from_millis(100));
    }

    #[test]
    fn send_when_not_open_with_auto_open_disabled_fails_fast() {
        let helper = SocketHelper::new();
        helper.set_automatic_open(false);
        let status = helper.send(b"data");
        assert_eq!(status, SocketIpStatus::AutoConnectDisabled);
        assert_eq!(
            byte_stream_send_status(status),
            ByteStreamStatus::OtherError
        );
    }

    #[test]
    fn send_when_not_open_with_no_reconnect_thread_times_out_disconnected() {
        let helper = SocketHelper::new();
        helper.set_timing(Timing {
            reconnect_wait_timeout: Duration::from_millis(30),
            reconnect_wait_interval: Duration::from_millis(2),
            ..Timing::default()
        });
        // No reconnect thread is running, so the request is never serviced.
        let status = helper.send(b"data");
        assert_eq!(status, SocketIpStatus::Disconnected);
    }

    #[test]
    fn request_reconnect_does_not_override_in_progress() {
        let helper = SocketHelper::new();
        helper.request_reconnect();
        assert_eq!(
            helper.reconnect.lock().unwrap().state,
            ReconnectState::RequestReconnect
        );
        helper.reconnect.lock().unwrap().state = ReconnectState::ReconnectInProgress;
        helper.request_reconnect();
        assert_eq!(
            helper.reconnect.lock().unwrap().state,
            ReconnectState::ReconnectInProgress
        );
    }

    #[test]
    fn stop_before_start_is_permitted() {
        let helper = SocketHelper::new();
        helper.stop(); // C++ contract: only shuts the (absent) socket down.
        assert!(!helper.running());
        assert!(!helper.running_reconnect());
    }
}
