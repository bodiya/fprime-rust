//! In-process loopback integration tests for `TcpServer` + `TcpClient`
//! (`std::net` on 127.0.0.1, ephemeral ports).
//!
//! Covers the driver contract from `docs/cpp-analysis/svc-comms.md`:
//! connect handshake fires `ready`, send → recv delivery with `OpOk` and
//! byte-exact payloads, disconnection → the reconnect loop re-establishes,
//! and clean `stop()`/`join()`. All waits are bounded polling loops with
//! deadlines — no unbounded sleeps.

use fprime_config::{FwIndexType, FwSizeType};
use fprime_drv::{
    ByteStreamDataPort, ByteStreamReadyPort, ByteStreamStatus, SocketIpStatus, TcpClient,
    TcpServer, Timing,
};
use fprime_fw::Buffer;
use fprime_os::task::Status as TaskStatus;
use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// Harness: recorder stubs for the driver's output ports.
// ---------------------------------------------------------------------------

/// Records every recv delivery, allocation and deallocation; allocates real
/// buffers on demand (stand-in for BufferManager).
#[derive(Default)]
struct Harness {
    /// (status, delivered bytes) per `recv_out` invocation.
    recvs: Mutex<Vec<(ByteStreamStatus, Vec<u8>)>>,
    ready_count: AtomicUsize,
    allocs: AtomicUsize,
    deallocs: AtomicUsize,
}

impl Harness {
    fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// Concatenated payload of all `OpOk` deliveries (TCP may fragment).
    fn ok_bytes(&self) -> Vec<u8> {
        self.recvs
            .lock()
            .unwrap()
            .iter()
            .filter(|(status, _)| *status == ByteStreamStatus::OpOk)
            .flat_map(|(_, bytes)| bytes.iter().copied())
            .collect()
    }

    /// Number of `OtherError` deliveries (disconnect indications).
    fn error_count(&self) -> usize {
        self.recvs
            .lock()
            .unwrap()
            .iter()
            .filter(|(status, _)| *status == ByteStreamStatus::OtherError)
            .count()
    }

    fn ready(&self) -> usize {
        self.ready_count.load(Ordering::SeqCst)
    }
}

impl fprime_comp::BufferGetPort for Harness {
    fn invoke(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
        self.allocs.fetch_add(1, Ordering::SeqCst);
        Buffer::allocate(size as usize)
    }
}

impl fprime_comp::BufferSendPort for Harness {
    fn invoke(&self, _port_num: FwIndexType, _buffer: Buffer) {
        self.deallocs.fetch_add(1, Ordering::SeqCst);
    }
}

impl ByteStreamDataPort for Harness {
    fn invoke(&self, _port_num: FwIndexType, buffer: Buffer, status: ByteStreamStatus) {
        self.recvs
            .lock()
            .unwrap()
            .push((status, buffer.data().to_vec()));
        // Real topology: return via recvReturnIn; the harness just drops
        // (the buffer is owned, nothing leaks).
    }
}

impl ByteStreamReadyPort for Harness {
    fn invoke(&self, _port_num: FwIndexType) {
        self.ready_count.fetch_add(1, Ordering::SeqCst);
    }
}

// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

/// Fast timing for tests: short polls, bounded waits, 50 ms read timeout.
fn fast_timing() -> Timing {
    Timing {
        reconnect_check_interval: Duration::from_millis(5),
        reconnect_wait_interval: Duration::from_millis(2),
        reconnect_wait_timeout: Duration::from_millis(500),
        retry_interval: Duration::from_millis(20),
        read_timeout: Some(Duration::from_millis(50)),
        write_timeout: Some(Duration::from_millis(500)),
        connect_timeout: Duration::from_millis(500),
    }
}

/// Bounded busy-wait: polls `pred` every 2 ms until `timeout`, panicking
/// with `what` on expiry.
fn wait_until(what: &str, timeout: Duration, mut pred: impl FnMut() -> bool) {
    let deadline = Instant::now() + timeout;
    loop {
        if pred() {
            return;
        }
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(2));
    }
}

const WAIT: Duration = Duration::from_secs(5);

fn data_buffer(bytes: &[u8]) -> Buffer {
    let mut buffer = Buffer::allocate(bytes.len());
    buffer.data_mut().copy_from_slice(bytes);
    buffer
}

fn wired_server(harness: &Arc<Harness>) -> Arc<TcpServer> {
    let server = TcpServer::new("loopServer");
    server.allocate_out.connect(harness.clone(), 0);
    server.deallocate_out.connect(harness.clone(), 0);
    server.recv_out.connect(harness.clone(), 0);
    server.ready_out.connect(harness.clone(), 0);
    server.socket_helper().set_timing(fast_timing());
    server
}

fn wired_client(harness: &Arc<Harness>) -> Arc<TcpClient> {
    let client = TcpClient::new("loopClient");
    client.allocate_out.connect(harness.clone(), 0);
    client.deallocate_out.connect(harness.clone(), 0);
    client.recv_out.connect(harness.clone(), 0);
    client.ready_out.connect(harness.clone(), 0);
    client.socket_helper().set_timing(fast_timing());
    client
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

/// Full loopback: server (port 0 → ephemeral) + client; both `ready` on
/// connect; data flows byte-exact both directions with `OpOk`; clean
/// stop/join.
#[test]
fn loopback_handshake_and_bidirectional_data() {
    let server_side = Harness::new();
    let client_side = Harness::new();
    let server = wired_server(&server_side);
    let client = wired_client(&client_side);

    assert_eq!(
        server.configure("127.0.0.1", 0, 1024, true),
        SocketIpStatus::Success
    );
    let port = server.get_listen_port();
    assert_ne!(port, 0);
    assert_eq!(
        client.configure("127.0.0.1", port, 1024, true),
        SocketIpStatus::Success
    );

    server.start();
    client.start();

    // Connect handshake: ready fires on BOTH sides exactly once.
    wait_until("server ready", WAIT, || server_side.ready() >= 1);
    wait_until("client ready", WAIT, || client_side.ready() >= 1);
    assert_eq!(server_side.ready(), 1);
    assert_eq!(client_side.ready(), 1);

    // Client → server.
    let send_port = client.send_in(0);
    let mut buffer = data_buffer(b"uplink: hello fprime");
    assert_eq!(
        send_port.target.invoke(send_port.port_num, &mut buffer),
        ByteStreamStatus::OpOk
    );
    wait_until("server recv", WAIT, || {
        server_side.ok_bytes() == b"uplink: hello fprime"
    });

    // Server → client.
    let send_port = server.send_in(0);
    let mut buffer = data_buffer(b"downlink: ack");
    assert_eq!(
        send_port.target.invoke(send_port.port_num, &mut buffer),
        ByteStreamStatus::OpOk
    );
    wait_until("client recv", WAIT, || {
        client_side.ok_bytes() == b"downlink: ack"
    });

    // The read loops allocated their receive buffers through allocate_out.
    assert!(server_side.allocs.load(Ordering::SeqCst) >= 1);
    assert!(client_side.allocs.load(Ordering::SeqCst) >= 1);

    // Clean teardown: stop + join both, bounded by the read timeout.
    client.stop();
    server.stop();
    assert_eq!(client.join(), TaskStatus::OpOk);
    assert_eq!(server.join(), TaskStatus::OpOk);
}

/// Server re-accepts after its client disconnects: the read loop detects the
/// orderly shutdown (`OtherError` delivery), closes, and the reconnect loop
/// accepts a new client (second `ready`).
#[test]
fn server_reaccepts_after_client_disconnect() {
    let harness = Harness::new();
    let server = wired_server(&harness);
    assert_eq!(
        server.configure("127.0.0.1", 0, 1024, true),
        SocketIpStatus::Success
    );
    let port = server.get_listen_port();
    server.start();

    // Raw client #1 connects, then disconnects.
    let first = TcpStream::connect(("127.0.0.1", port)).expect("first connect");
    wait_until("first accept", WAIT, || harness.ready() >= 1);
    drop(first);

    // The disconnect surfaces as a non-OK delivery and triggers a reconnect.
    wait_until("disconnect delivery", WAIT, || harness.error_count() >= 1);

    // Raw client #2 is accepted by the reconnect loop and can send data.
    let mut second = TcpStream::connect(("127.0.0.1", port)).expect("second connect");
    wait_until("re-accept", WAIT, || harness.ready() >= 2);
    second.write_all(b"back again").expect("write");
    wait_until("second recv", WAIT, || harness.ok_bytes() == b"back again");

    server.stop();
    assert_eq!(server.join(), TaskStatus::OpOk);
}

/// Client reconnects after the peer drops the connection: disconnect is
/// detected, the reconnect loop re-establishes (second `ready`), and data
/// flows on the new connection.
#[test]
fn client_reconnects_after_peer_drop() {
    let harness = Harness::new();
    let client = wired_client(&harness);

    // Raw listener harness (blocking accept is fine on the test thread —
    // the client's reconnect loop connects promptly).
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let port = listener.local_addr().unwrap().port();
    assert_eq!(
        client.configure("127.0.0.1", port, 1024, true),
        SocketIpStatus::Success
    );
    client.start();

    let (first, _) = listener.accept().expect("first accept");
    wait_until("first connect", WAIT, || harness.ready() >= 1);
    drop(first); // peer drops → client recv sees disconnect

    wait_until("disconnect delivery", WAIT, || harness.error_count() >= 1);

    // The reconnect loop re-establishes on the same listener.
    let (mut second, _) = listener.accept().expect("second accept");
    wait_until("reconnect", WAIT, || harness.ready() >= 2);

    second.write_all(b"fresh connection").expect("write");
    wait_until("recv on new connection", WAIT, || {
        harness.ok_bytes() == b"fresh connection"
    });

    client.stop();
    assert_eq!(client.join(), TaskStatus::OpOk);
}

/// C++ `send()` semantics: a send on a not-yet-open socket requests a
/// reconnect and WAITS; because the server is already listening, the
/// reconnect thread connects within the wait window and the send succeeds.
#[test]
fn send_before_connect_waits_for_reconnect_and_succeeds() {
    let server_side = Harness::new();
    let client_side = Harness::new();
    let server = wired_server(&server_side);
    let client = wired_client(&client_side);

    assert_eq!(
        server.configure("127.0.0.1", 0, 1024, true),
        SocketIpStatus::Success
    );
    assert_eq!(
        client.configure("127.0.0.1", server.get_listen_port(), 1024, true),
        SocketIpStatus::Success
    );
    server.start();
    client.start();

    // Immediately send — may run before the reconnect loop has connected;
    // the guarded handler must block until the connection exists, then
    // succeed (C++ requestReconnect + waitForReconnect path).
    let send_port = client.send_in(0);
    let mut buffer = data_buffer(b"early bird");
    assert_eq!(
        send_port.target.invoke(send_port.port_num, &mut buffer),
        ByteStreamStatus::OpOk
    );
    wait_until("server recv", WAIT, || {
        server_side.ok_bytes() == b"early bird"
    });

    client.stop();
    server.stop();
    assert_eq!(client.join(), TaskStatus::OpOk);
    assert_eq!(server.join(), TaskStatus::OpOk);
}

/// A stopped driver joins cleanly even when it never connected (client
/// pointed at a dead port keeps retrying until stop).
#[test]
fn stop_and_join_while_never_connected() {
    let harness = Harness::new();
    let client = wired_client(&harness);
    // Find a port that refuses connections.
    let dead_port = {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        listener.local_addr().unwrap().port()
    };
    assert_eq!(
        client.configure("127.0.0.1", dead_port, 1024, true),
        SocketIpStatus::Success
    );
    client.start();
    // Let the reconnect loop attempt (and fail) at least once.
    std::thread::sleep(Duration::from_millis(30));
    client.stop();
    assert_eq!(client.join(), TaskStatus::OpOk);
    assert_eq!(harness.ready(), 0);
}

/// recvReturnIn → deallocate_out (buffer return path through a full
/// component wiring).
#[test]
fn recv_return_path_deallocates() {
    let harness = Harness::new();
    let server = wired_server(&harness);
    assert_eq!(
        server.configure("127.0.0.1", 0, 1024, true),
        SocketIpStatus::Success
    );
    let port = server.recv_return_in(0);
    port.target.invoke(port.port_num, Buffer::allocate(64));
    assert_eq!(harness.deallocs.load(Ordering::SeqCst), 1);
    server.terminate();
}
