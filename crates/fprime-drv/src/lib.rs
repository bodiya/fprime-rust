//! # fprime-drv — F Prime Rust port: byte-stream drivers
//!
//! Port of `Drv/ByteStreamDriverModel`, `Drv/Interfaces`, `Drv/Ip`
//! (`SocketComponentHelper` + TCP sockets), `Drv/TcpClient` and
//! `Drv/TcpServer`. Analysis: `docs/cpp-analysis/svc-comms.md`.
//!
//! The byte-stream port traits defined in [`byte_stream`] are the CANONICAL
//! ones for the workspace; `fprime-svc`'s `com_stub` module carries
//! structurally-identical local copies because the two crates must not
//! depend on each other (see the seam note in `byte_stream`). The reference
//! deployment bridges the families with tiny adapter shims.

pub mod byte_stream;
pub mod gpio;
pub mod i2c;
pub mod socket_helper;
pub mod spi;
pub mod tcp_client;
pub mod tcp_server;
pub mod uart;

pub use byte_stream::{
    BufferSendPort, ByteStreamDataPort, ByteStreamReadyPort, ByteStreamSendPort, ByteStreamStatus,
};
pub use socket_helper::{
    SOCKET_MAX_ITERATIONS, SocketHelper, SocketIpStatus, SocketWorker, Timing,
    byte_stream_recv_status, byte_stream_send_status,
};
pub use tcp_client::TcpClient;
pub use tcp_server::TcpServer;
