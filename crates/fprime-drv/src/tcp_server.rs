//! # TcpServer — port of `Drv::TcpServerComponentImpl` (+ `Drv/Ip/TcpServerSocket`)
//!
//! C++ sources: `Drv/TcpServer/TcpServerComponentImpl.{cpp,hpp}`,
//! `Drv/Ip/TcpServerSocket.cpp`.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (SocketComponentHelper + Tcp
//! components; gotcha: "TcpServer startup binds+listens at configure()
//! time; listen backlog is 1; getsockname reads back the ephemeral port
//! when port 0 is configured").
//!
//! Same shape as [`crate::TcpClient`], plus the listening socket:
//!
//! - `configure()` calls `startup()` (bind + listen + read back the
//!   ephemeral port) and returns its status — C++ parity;
//! - the read loop retries `startup()` until it succeeds, then runs the
//!   generic loop, then `terminate()`s (C++ `readLoop` override);
//! - `open_protocol` (reconnect thread only) accepts a SINGLE client.
//!
//! ## Divergences (documented, behavior-preserving)
//!
//! - C++ listens with backlog 1 to prevent queuing of clients; `std`'s
//!   `TcpListener::bind` fixes the backlog (128 on most platforms). Extra
//!   queued connections are still never *accepted* concurrently — a single
//!   client is served at a time, as in C++.
//! - C++ breaks a blocking `accept` by `shutdown()` on the server fd;
//!   `std` cannot shut down a `TcpListener`, so the listener is
//!   non-blocking and `open_protocol` polls `accept` at
//!   `reconnect_wait_interval`, checking the stop flags — same
//!   responsiveness, no busy accept.
//! - C++ sets `SO_REUSEADDR` (IpCfg); `std` exposes no portable knob, so a
//!   rebind during `TIME_WAIT` may fail with `FailedToBind` (the retry loop
//!   handles it).

use crate::byte_stream::{
    ByteStreamDataPort, ByteStreamReadyPort, ByteStreamSendPort, ByteStreamStatus,
};
use crate::socket_helper::{
    SocketHelper, SocketIpStatus, SocketWorker, byte_stream_recv_status, byte_stream_send_status,
};
use fprime_comp::{BufferGetPort, BufferSendPort, OutputPort, PassiveBase, PortRef};
use fprime_config::{FwIndexType, FwSizeType};
use fprime_fw::{Buffer, fw_assert, fw_log};
use std::io::ErrorKind;
use std::net::{Ipv4Addr, SocketAddrV4, TcpListener, TcpStream};
use std::str::FromStr;
use std::sync::{Arc, Mutex};

/// Server configuration captured at `configure`. `port` is updated by
/// `startup()` with the read-back ephemeral port (C++ `getsockname`).
#[derive(Default)]
struct ServerConfig {
    address: String,
    port: u16,
    allocation_size: FwSizeType,
    configured: bool,
}

/// `Drv::TcpServer` — passive byte-stream driver serving a single TCP
/// client.
pub struct TcpServer {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `allocate` — obtain receive buffers (→ BufferManager).
    pub allocate_out: OutputPort<dyn BufferGetPort>,
    /// `deallocate` — return receive buffers (→ BufferManager).
    pub deallocate_out: OutputPort<dyn BufferSendPort>,
    /// `recv` — deliver received data upstream (→ ComStub `drvReceiveIn`).
    pub recv_out: OutputPort<dyn ByteStreamDataPort>,
    /// `ready` — client-accepted signal (→ ComStub `drvConnected`).
    pub ready_out: OutputPort<dyn ByteStreamReadyPort>,
    helper: SocketHelper,
    config: Mutex<ServerConfig>,
    /// The listening socket (C++ `m_descriptor.serverFd`).
    listener: Mutex<Option<TcpListener>>,
    /// The component (guarded-port) mutex for `send` / `recvReturnIn`.
    guard: Mutex<()>,
}

impl TcpServer {
    /// Construct the component (unconfigured, not listening).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            allocate_out: OutputPort::new(),
            deallocate_out: OutputPort::new(),
            recv_out: OutputPort::new(),
            ready_out: OutputPort::new(),
            helper: SocketHelper::new(),
            config: Mutex::new(ServerConfig::default()),
            listener: Mutex::new(None),
            guard: Mutex::new(()),
        })
    }

    /// C++ `configure()`: store the endpoint / allocation size, then
    /// `startup()` (bind + listen) and return ITS status. Port 0 is valid
    /// for a server — the bound ephemeral port is read back
    /// ([`TcpServer::get_listen_port`]). `reconnect` maps to the helper's
    /// automatic-open flag.
    pub fn configure(
        &self,
        hostname: &str,
        port: u16,
        buffer_size: FwSizeType,
        reconnect: bool,
    ) -> SocketIpStatus {
        {
            let mut config = self.config.lock().unwrap();
            config.address = hostname.to_string();
            config.port = port;
            config.allocation_size = buffer_size;
            config.configured = true;
        }
        self.helper.set_automatic_open(reconnect);
        self.startup()
    }

    /// C++ `startup()` / `TcpServerSocket::startup()`: bind + listen once
    /// (multiple calls are a success no-op while listening) and read back
    /// the actual bound port.
    pub fn startup(&self) -> SocketIpStatus {
        let mut listener_slot = self.listener.lock().unwrap();
        // Prevent multiple startup attempts (C++ serverFd != -1 check).
        if listener_slot.is_some() {
            return SocketIpStatus::Success;
        }
        let (address, port) = {
            let config = self.config.lock().unwrap();
            fw_assert!(config.configured);
            (config.address.clone(), config.port)
        };
        let Ok(ip) = Ipv4Addr::from_str(&address) else {
            return SocketIpStatus::InvalidIpAddress;
        };
        // std bind() covers socket+bind+listen; bind-family failures land in
        // FailedToBind (C++ distinguishes FAILED_TO_LISTEN — unreachable
        // through std).
        let Ok(listener) = TcpListener::bind(SocketAddrV4::new(ip, port)) else {
            return SocketIpStatus::FailedToBind;
        };
        // Read back the (possibly ephemeral) port — C++ getsockname.
        let Ok(local) = listener.local_addr() else {
            return SocketIpStatus::FailedToReadBackPort;
        };
        // Non-blocking so open_protocol can poll accept responsively (see
        // module docs on the C++ shutdown-breaks-accept divergence).
        if listener.set_nonblocking(true).is_err() {
            return SocketIpStatus::FailedToSetSocketOptions;
        }
        self.config.lock().unwrap().port = local.port();
        *listener_slot = Some(listener);
        fw_log!(
            "Listening for single client at {}:{}",
            address,
            local.port()
        );
        SocketIpStatus::Success
    }

    /// C++ `getListenPort()`: the actual bound port (ephemeral read-back
    /// included).
    pub fn get_listen_port(&self) -> u16 {
        self.config.lock().unwrap().port
    }

    /// C++ `isStarted()`.
    pub fn is_started(&self) -> bool {
        self.listener.lock().unwrap().is_some()
    }

    /// C++ `terminate()`: stop everything and close the listening socket.
    pub fn terminate(&self) {
        self.stop();
        *self.listener.lock().unwrap() = None;
    }

    /// Start the read + reconnect threads (C++ `SocketComponentHelper::start`).
    pub fn start(self: &Arc<Self>) {
        fw_assert!(self.config.lock().unwrap().configured);
        let name = self.base.get_obj_name();
        self.helper
            .start(self, name.as_str().unwrap_or("TcpServer"));
    }

    /// Stop both threads and shut the client socket down (C++ `stop()`).
    pub fn stop(&self) {
        self.helper.stop();
    }

    /// Join both threads (C++ `join()`).
    pub fn join(&self) -> fprime_os::task::Status {
        self.helper.join()
    }

    /// Direct access to the embedded helper (timing configuration, state
    /// inspection in tests / deployments).
    pub fn socket_helper(&self) -> &SocketHelper {
        &self.helper
    }

    // -- Input-port factories (topology wiring surface) ----------------------

    /// `send` — GUARDED `Drv.ByteStreamSend` input.
    pub fn send_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ByteStreamSendPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// `recvReturnIn` — GUARDED `Fw.BufferSend` input → `deallocate_out`.
    pub fn recv_return_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn BufferSendPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Handlers ------------------------------------------------------------

    /// C++ `send_handler` (identical mapping to TcpClient's).
    fn send_handler(&self, _port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus {
        byte_stream_send_status(self.helper.send(buffer.data()))
    }

    /// C++ `recvReturnIn_handler`: forward to `deallocate_out(0, ...)`.
    fn recv_return_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        let p = self.deallocate_out.get();
        p.target.invoke(p.port_num, buffer);
    }
}

impl ByteStreamSendPort for TcpServer {
    fn invoke(&self, port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus {
        let _guard = self.guard.lock().unwrap();
        self.send_handler(port_num, buffer)
    }
}

impl BufferSendPort for TcpServer {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        let _guard = self.guard.lock().unwrap();
        self.recv_return_handler(port_num, buffer);
    }
}

impl SocketWorker for TcpServer {
    fn helper(&self) -> &SocketHelper {
        &self.helper
    }

    /// C++ `TcpServerSocket::openProtocol`: accept a single client on the
    /// listening socket (reconnect thread only). Polls the non-blocking
    /// listener, aborting when either loop is stopped (the C++ equivalent
    /// is `terminate()` shutting the server fd down under a blocking
    /// `accept`).
    fn open_protocol(&self) -> Result<TcpStream, SocketIpStatus> {
        // Duplicate the listener handle out of the lock (poll without
        // blocking startup/terminate).
        let listener = {
            let slot = self.listener.lock().unwrap();
            match slot.as_ref() {
                // May be true during start-up reconnect attempts (C++
                // SOCK_NOT_STARTED).
                None => return Err(SocketIpStatus::NotStarted),
                Some(l) => l.try_clone().map_err(|_| SocketIpStatus::NotStarted)?,
            }
        };
        let timing = self.helper.timing();
        loop {
            if !(self.helper.running() && self.helper.running_reconnect()) {
                return Err(SocketIpStatus::FailedToAccept);
            }
            match listener.accept() {
                Ok((stream, _peer)) => {
                    // Accepted sockets can inherit non-blocking mode on some
                    // platforms — force blocking before applying timeouts.
                    if stream.set_nonblocking(false).is_err() {
                        return Err(SocketIpStatus::FailedToSetSocketOptions);
                    }
                    if stream.set_write_timeout(timing.write_timeout).is_err()
                        || stream.set_read_timeout(timing.read_timeout).is_err()
                    {
                        return Err(SocketIpStatus::FailedToSetSocketOptions);
                    }
                    let (address, port) = {
                        let config = self.config.lock().unwrap();
                        (config.address.clone(), config.port)
                    };
                    fw_log!("Accepted client at {}:{}", address, port);
                    return Ok(stream);
                }
                Err(e) => match e.kind() {
                    ErrorKind::WouldBlock => {
                        std::thread::sleep(timing.reconnect_wait_interval);
                    }
                    ErrorKind::Interrupted => {}
                    _ => return Err(SocketIpStatus::FailedToAccept),
                },
            }
        }
    }

    /// C++ `getBuffer()`: `allocate_out(0, m_allocation_size)`.
    fn get_buffer(&self) -> Buffer {
        let size = self.config.lock().unwrap().allocation_size;
        let p = self.allocate_out.get();
        p.target.invoke(p.port_num, size)
    }

    /// C++ `sendBuffer()`: map the socket status and deliver via `recv_out`.
    fn send_buffer(&self, buffer: Buffer, status: SocketIpStatus) {
        let p = self.recv_out.get();
        p.target
            .invoke(p.port_num, buffer, byte_stream_recv_status(status));
    }

    /// C++ `connected()`: fire `ready_out` when a client is accepted.
    fn connected(&self) {
        if let Some(p) = self.ready_out.try_get() {
            p.target.invoke(p.port_num);
        }
    }

    /// C++ `TcpServerComponentImpl::readLoop` override: retry `startup()`
    /// until it succeeds (or stop / auto-open-off), run the generic read
    /// loop, then `terminate()`.
    fn read_loop(&self) {
        let mut status;
        loop {
            status = self.startup();
            if status != SocketIpStatus::Success {
                fw_log!(
                    "[WARNING] Failed to listen on port {} with status {}",
                    self.get_listen_port(),
                    status as i32
                );
                std::thread::sleep(self.helper.timing().retry_interval);
            }
            // C++ do/while condition.
            if !(self.helper.running()
                && status != SocketIpStatus::Success
                && self.helper.get_automatic_open())
            {
                break;
            }
        }
        // If start up was successful then perform normal operations.
        if self.helper.running() && status == SocketIpStatus::Success {
            self.helper.read_loop_body(self);
        }
        // Terminate the server.
        self.terminate();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configure_with_port_zero_reads_back_ephemeral_port() {
        let server = TcpServer::new("ephemeral");
        let status = server.configure("127.0.0.1", 0, 128, true);
        assert_eq!(status, SocketIpStatus::Success);
        assert_ne!(server.get_listen_port(), 0);
        assert!(server.is_started());
        server.terminate();
        assert!(!server.is_started());
    }

    #[test]
    fn startup_twice_is_a_success_noop() {
        let server = TcpServer::new("double");
        let status = server.configure("127.0.0.1", 0, 128, true);
        assert_eq!(status, SocketIpStatus::Success);
        let port = server.get_listen_port();
        // C++ parity: second startup while listening returns SUCCESS and
        // does not rebind.
        assert_eq!(server.startup(), SocketIpStatus::Success);
        assert_eq!(server.get_listen_port(), port);
        server.terminate();
    }

    #[test]
    fn configure_with_invalid_address_fails_startup() {
        let server = TcpServer::new("badAddr");
        let status = server.configure("not-an-ip", 0, 128, true);
        assert_eq!(status, SocketIpStatus::InvalidIpAddress);
        assert!(!server.is_started());
    }

    #[test]
    fn open_protocol_before_startup_is_not_started() {
        let server = TcpServer::new("notStarted");
        // Configured but never bound (invalid address path keeps listener
        // empty); call open_protocol directly.
        let _ = server.configure("not-an-ip", 0, 128, true);
        assert_eq!(
            server.open_protocol().err(),
            Some(SocketIpStatus::NotStarted)
        );
    }

    #[test]
    fn open_protocol_aborts_when_stopped() {
        // Listener bound, no client, helper stopped (initial state):
        // the accept poll must exit immediately with FailedToAccept
        // rather than spinning.
        let server = TcpServer::new("stopped");
        let status = server.configure("127.0.0.1", 0, 128, true);
        assert_eq!(status, SocketIpStatus::Success);
        assert_eq!(
            server.open_protocol().err(),
            Some(SocketIpStatus::FailedToAccept)
        );
        server.terminate();
    }
}
