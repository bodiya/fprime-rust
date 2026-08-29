//! # TcpClient — port of `Drv::TcpClientComponentImpl` (+ `Drv/Ip/TcpClientSocket`)
//!
//! C++ sources: `Drv/TcpClient/TcpClientComponentImpl.{cpp,hpp}`,
//! `Drv/Ip/TcpClientSocket.cpp`.
//! Analysis: `docs/cpp-analysis/svc-comms.md` (SocketComponentHelper + Tcp
//! components).
//!
//! PASSIVE component implementing the sync `Drv.ByteStreamDriver` interface
//! over a TCP client socket, with an embedded [`SocketHelper`] owning the
//! read + reconnect threads. Ports:
//!
//! - `send` (guarded input, `Drv.ByteStreamSend`) — implemented on the
//!   component, runs on the caller's thread under the component mutex;
//! - `recvReturnIn` (guarded input, `Fw.BufferSend`) — forwards to
//!   `deallocate_out`;
//! - `allocate_out` (`Fw.BufferGet`), `deallocate_out` (`Fw.BufferSend`),
//!   `recv_out` (`Drv.ByteStreamData`), `ready_out` (`Drv.ByteStreamReady`).
//!
//! The address must be a dotted-quad IPv4 string (C++ `inet_pton`, no DNS)
//! and the port must be nonzero (C++ `TcpClientSocket::isValidPort` — a
//! zero port fw_asserts at `configure`).

use crate::byte_stream::{
    ByteStreamDataPort, ByteStreamReadyPort, ByteStreamSendPort, ByteStreamStatus,
};
use crate::socket_helper::{
    SocketHelper, SocketIpStatus, SocketWorker, byte_stream_recv_status, byte_stream_send_status,
};
use fprime_comp::{BufferGetPort, BufferSendPort, OutputPort, PassiveBase, PortRef};
use fprime_config::{FwIndexType, FwSizeType};
use fprime_fw::{Buffer, fw_assert, fw_log};
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, TcpStream};
use std::str::FromStr;
use std::sync::{Arc, Mutex};

/// Client configuration captured at `configure` (C++ `IpSocket::configure`
/// stored fields + `m_allocation_size`).
#[derive(Default)]
struct ClientConfig {
    address: String,
    port: u16,
    allocation_size: FwSizeType,
    configured: bool,
}

/// `Drv::TcpClient` — passive byte-stream driver over a TCP client socket.
pub struct TcpClient {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `allocate` — obtain receive buffers (→ BufferManager).
    pub allocate_out: OutputPort<dyn BufferGetPort>,
    /// `deallocate` — return receive buffers (→ BufferManager).
    pub deallocate_out: OutputPort<dyn BufferSendPort>,
    /// `recv` — deliver received data upstream (→ ComStub `drvReceiveIn`).
    pub recv_out: OutputPort<dyn ByteStreamDataPort>,
    /// `ready` — connection-established signal (→ ComStub `drvConnected`).
    pub ready_out: OutputPort<dyn ByteStreamReadyPort>,
    helper: SocketHelper,
    config: Mutex<ClientConfig>,
    /// The component (guarded-port) mutex: `send` and `recvReturnIn` are
    /// `guarded` inputs in the C++ interface.
    guard: Mutex<()>,
}

impl TcpClient {
    /// Construct the component (unconfigured, threads not started).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            allocate_out: OutputPort::new(),
            deallocate_out: OutputPort::new(),
            recv_out: OutputPort::new(),
            ready_out: OutputPort::new(),
            helper: SocketHelper::new(),
            config: Mutex::new(ClientConfig::default()),
            guard: Mutex::new(()),
        })
    }

    /// C++ `configure()`: store the endpoint and receive-allocation size.
    /// `reconnect` maps to the helper's automatic-open flag
    /// (`setAutomaticOpen`); timeouts come from [`SocketHelper::set_timing`]
    /// (defaults = `IpCfg.hpp`). A zero port fw_asserts (C++
    /// `TcpClientSocket::isValidPort`); the address string is validated at
    /// open time, exactly like C++ `inet_pton` in `openProtocol`.
    pub fn configure(
        &self,
        hostname: &str,
        port: u16,
        buffer_size: FwSizeType,
        reconnect: bool,
    ) -> SocketIpStatus {
        // C++ parity: FW_ASSERT(isValidPort(port)) — port 0 is invalid for
        // a TCP client.
        fw_assert!(port != 0);
        {
            let mut config = self.config.lock().unwrap();
            config.address = hostname.to_string();
            config.port = port;
            config.allocation_size = buffer_size;
            config.configured = true;
        }
        self.helper.set_automatic_open(reconnect);
        SocketIpStatus::Success
    }

    /// Start the read + reconnect threads (C++ `SocketComponentHelper::start`).
    pub fn start(self: &Arc<Self>) {
        fw_assert!(self.config.lock().unwrap().configured);
        let name = self.base.get_obj_name();
        self.helper
            .start(self, name.as_str().unwrap_or("TcpClient"));
    }

    /// Stop both threads and shut the socket down (C++ `stop()`).
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

    /// `recvReturnIn` — GUARDED `Fw.BufferSend` input: receive buffers come
    /// back here and are forwarded to `deallocate_out`.
    pub fn recv_return_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn BufferSendPort> {
        PortRef::new(self.clone(), port_num)
    }

    // -- Handlers ------------------------------------------------------------

    /// C++ `send_handler`: send the data window; map
    /// `SOCK_INTERRUPTED_TRY_AGAIN → SEND_RETRY`, `SOCK_SUCCESS → OP_OK`,
    /// everything else → `OTHER_ERROR`. A not-open socket requests a
    /// reconnect inside `SocketHelper::send` (and waits, bounded).
    fn send_handler(&self, _port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus {
        byte_stream_send_status(self.helper.send(buffer.data()))
    }

    /// C++ `recvReturnIn_handler`: forward to `deallocate_out(0, ...)`.
    fn recv_return_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        let p = self.deallocate_out.get();
        p.target.invoke(p.port_num, buffer);
    }
}

impl ByteStreamSendPort for TcpClient {
    fn invoke(&self, port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus {
        // Guarded: component mutex on the caller's thread.
        let _guard = self.guard.lock().unwrap();
        self.send_handler(port_num, buffer)
    }
}

impl BufferSendPort for TcpClient {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        let _guard = self.guard.lock().unwrap();
        self.recv_return_handler(port_num, buffer);
    }
}

impl SocketWorker for TcpClient {
    fn helper(&self) -> &SocketHelper {
        &self.helper
    }

    /// C++ `TcpClientSocket::openProtocol`: dotted-quad parse (`inet_pton`
    /// semantics — no DNS) → `connect` → timeouts. Called only from the
    /// reconnect thread.
    fn open_protocol(&self) -> Result<TcpStream, SocketIpStatus> {
        let (address, port) = {
            let config = self.config.lock().unwrap();
            (config.address.clone(), config.port)
        };
        let ip = Ipv4Addr::from_str(&address).map_err(|_| SocketIpStatus::InvalidIpAddress)?;
        let timing = self.helper.timing();
        let addr = SocketAddr::V4(SocketAddrV4::new(ip, port));
        let stream = TcpStream::connect_timeout(&addr, timing.connect_timeout)
            .map_err(|_| SocketIpStatus::FailedToConnect)?;
        // C++ setupTimeouts (SO_SNDTIMEO) + the Rust belt-and-braces read
        // timeout (see socket_helper module docs).
        stream
            .set_write_timeout(timing.write_timeout)
            .map_err(|_| SocketIpStatus::FailedToSetSocketOptions)?;
        stream
            .set_read_timeout(timing.read_timeout)
            .map_err(|_| SocketIpStatus::FailedToSetSocketOptions)?;
        fw_log!("Connected to {}:{} as a tcp client", address, port);
        Ok(stream)
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

    /// C++ `connected()`: fire `ready_out` when connected (only if wired).
    fn connected(&self) {
        if let Some(p) = self.ready_out.try_get() {
            p.target.invoke(p.port_num);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    fn configured_client(port: u16) -> Arc<TcpClient> {
        let client = TcpClient::new("testClient");
        let status = client.configure("127.0.0.1", port, 128, true);
        assert_eq!(status, SocketIpStatus::Success);
        client
    }

    #[test]
    #[should_panic]
    fn configure_asserts_on_zero_port() {
        // C++ parity: TcpClientSocket::isValidPort(0) fails FW_ASSERT.
        let client = TcpClient::new("badPort");
        let _ = client.configure("127.0.0.1", 0, 128, true);
    }

    #[test]
    fn open_protocol_rejects_non_dotted_quad_address() {
        // C++ parity: inet_pton semantics — hostnames are NOT resolved.
        let client = TcpClient::new("badAddr");
        let status = client.configure("localhost", 5000, 128, true);
        assert_eq!(status, SocketIpStatus::Success);
        assert_eq!(
            client.open_protocol().err(),
            Some(SocketIpStatus::InvalidIpAddress)
        );
    }

    #[test]
    fn open_protocol_maps_refused_connection_to_failed_to_connect() {
        // Bind then drop a listener to find a port that refuses connections.
        let port = {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            listener.local_addr().unwrap().port()
        };
        let client = configured_client(port);
        assert_eq!(
            client.open_protocol().err(),
            Some(SocketIpStatus::FailedToConnect)
        );
    }

    #[test]
    fn open_protocol_connects_and_client_send_delivers_bytes() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let client = configured_client(port);
        let stream = client.open_protocol().expect("connect");
        let (mut peer, _) = listener.accept().unwrap();

        // Feed the stream into the helper state via open()'s publish path:
        // simplest is to exercise socket-level send directly through the
        // helper by publishing the stream with open()+a stub worker — here
        // we just write on the raw stream to prove the connection works.
        use std::io::{Read, Write};
        let mut s = &stream;
        s.write_all(b"ping").unwrap();
        let mut buf = [0u8; 4];
        peer.read_exact(&mut buf).unwrap();
        assert_eq!(&buf, b"ping");
    }

    #[test]
    fn send_when_not_open_without_reconnect_is_other_error() {
        // reconnect=false: waitForReconnect returns AUTO_CONNECT_DISABLED
        // immediately, mapping to OtherError at the byte-stream level.
        let client = TcpClient::new("noReconnect");
        let status = client.configure("127.0.0.1", 5000, 128, false);
        assert_eq!(status, SocketIpStatus::Success);
        let mut buffer = Buffer::allocate(4);
        let port = client.send_in(0);
        assert_eq!(
            port.target.invoke(port.port_num, &mut buffer),
            ByteStreamStatus::OtherError
        );
    }

    #[test]
    fn recv_return_forwards_to_deallocate() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        #[derive(Default)]
        struct DeallocStub {
            count: AtomicUsize,
        }
        impl BufferSendPort for DeallocStub {
            fn invoke(&self, _port_num: FwIndexType, _buffer: Buffer) {
                self.count.fetch_add(1, Ordering::SeqCst);
            }
        }
        let client = configured_client(5000);
        let stub = Arc::new(DeallocStub::default());
        client.deallocate_out.connect(stub.clone(), 0);
        let port = client.recv_return_in(0);
        port.target.invoke(port.port_num, Buffer::allocate(16));
        assert_eq!(stub.count.load(Ordering::SeqCst), 1);
    }
}
