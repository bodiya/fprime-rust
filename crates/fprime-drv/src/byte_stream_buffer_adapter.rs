//! # ByteStreamBufferAdapter — port of `Drv::ByteStreamBufferAdapter`
//!
//! C++ sources:
//! `Drv/ByteStreamBufferAdapter/ByteStreamBufferAdapter.{fpp,hpp,cpp}`,
//! `Drv/Interfaces/{ByteStreamDriver,PassiveByteStreamDriverClient,
//! PassiveByteStreamDriverClientReadyRecv,PassiveBufferDriver}.fpp`,
//! `Fw/Buffer/Buffer.fpp` (`Fw.PassiveBufferIn` / `Fw.PassiveBufferOut`).
//! Upstream SDD: `Drv/ByteStreamBufferAdapter/docs/sdd.md`.
//!
//! PASSIVE component that turns a synchronous byte-stream driver
//! (`Drv.ByteStreamDriver`: [`TcpClient`](crate::TcpClient),
//! [`TcpServer`](crate::TcpServer), the UART driver) into a passive buffer
//! driver (`Drv.PassiveBufferDriver`) — e.g. the transport behind
//! `Svc::GenericHub`:
//!
//! ```text
//! ByteStreamDriver <--> ByteStreamBufferAdapter <--> PassiveBufferDriverClient
//! ```
//!
//! It is the synchronous sibling of
//! [`AsyncByteStreamBufferAdapter`](crate::AsyncByteStreamBufferAdapter):
//! the same events and receive path, but `toByteStreamDriver` is a
//! `Drv.ByteStreamSend` call that returns the send status, so there is no
//! `toByteStreamDriverReturn` input.
//!
//! Ports (all inputs are `sync`, so every handler runs on the caller's
//! thread with no component mutex, as upstream):
//!
//! | FPP port                    | direction | type                  | Rust                                |
//! |-----------------------------|-----------|-----------------------|-------------------------------------|
//! | `bufferIn`                  | sync in   | `Fw.BufferSend`       | [`ByteStreamBufferAdapter::buffer_in`] |
//! | `bufferInReturn`            | out       | `Fw.BufferSend`       | `buffer_in_return_out`              |
//! | `bufferOut`                 | out       | `Fw.BufferSend`       | `buffer_out`                        |
//! | `bufferOutReturn`           | sync in   | `Fw.BufferSend`       | [`ByteStreamBufferAdapter::buffer_out_return_in`] |
//! | `byteStreamDriverReady`     | sync in   | `Drv.ByteStreamReady` | [`ByteStreamBufferAdapter::byte_stream_driver_ready_in`] |
//! | `fromByteStreamDriver`      | sync in   | `Drv.ByteStreamData`  | [`ByteStreamBufferAdapter::from_byte_stream_driver_in`] |
//! | `fromByteStreamDriverReturn`| out       | `Fw.BufferSend`       | `from_byte_stream_driver_return_out` |
//! | `toByteStreamDriver`        | out       | `Drv.ByteStreamSend`  | `to_byte_stream_driver_out`         |
//!
//! plus `Log` / `LogText` / `Time` through [`EventGlue`] (no telemetry, no
//! commands).
//!
//! ## Buffer ownership
//!
//! Every `Fw.BufferSend` buffer MOVES (the workspace's `Fw::Buffer`
//! convention); the adapter never holds one past a handler:
//!
//! - send: `bufferIn` → `toByteStreamDriver` (by `&mut`, the driver only
//!   borrows it: `Drv.ByteStreamSend` leaves ownership with the caller)
//!   while the driver is ready, with `DataSendError` on any non-`OP_OK`
//!   status; before the driver is ready, `DriverNotReady` and no send. The
//!   buffer then goes back on `bufferInReturn` in EVERY case, right away
//!   (the driver is synchronous).
//! - receive: `fromByteStreamDriver` with `OP_OK` → `bufferOut`; the client
//!   returns it on `bufferOutReturn`, which forwards it on
//!   `fromByteStreamDriverReturn` (to the driver's `recvReturnIn`). Any other
//!   status returns it to the driver at once with `DataReceiveError`.
//!
//! The adapter does not interpret `SEND_RETRY`: like any non-`OP_OK` send
//! status it is reported and the buffer goes back to the client (C++
//! parity).
//!
//! C++ parity quirk kept: the ready flag is set by `byteStreamDriverReady`
//! and never cleared (there is no "not ready" signal in the interface).
//!
//! Deviation: C++ forwards the input port number as the output port index
//! (`toByteStreamDriver_out(portNum, ..)`). Every port here is a single
//! port, so that is always 0; the Rust `OutputPort` invokes its
//! connection's own target port number instead.

use crate::byte_stream::{
    ByteStreamDataPort, ByteStreamReadyPort, ByteStreamSendPort, ByteStreamStatus,
};
use fprime_comp::{BufferSendPort, EventGlue, OutputPort, PassiveBase};
use fprime_config::{FwEventIdType, FwIdType, FwIndexType};
use fprime_fw::{Buffer, LogSeverity, SerBuf, SerializeStatus};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// `Drv::ByteStreamBufferAdapter` — passive adapter from a synchronous
/// byte-stream driver to the passive buffer-driver interface.
pub struct ByteStreamBufferAdapter {
    /// Passive core (name / id_base / instance).
    pub base: PassiveBase,
    /// Event ports (`Log` / `LogText`) + `Time`.
    pub evt: EventGlue,
    /// `bufferInReturn` — return buffers received on `bufferIn` to the
    /// client (`Fw.PassiveBufferIn`).
    pub buffer_in_return_out: OutputPort<dyn BufferSendPort>,
    /// `bufferOut` — deliver received data to the client
    /// (`Fw.PassiveBufferOut`).
    pub buffer_out: OutputPort<dyn BufferSendPort>,
    /// `fromByteStreamDriverReturn` — return received buffers to the driver
    /// (sample connection: `adapter.fromByteStreamDriverReturn ->
    /// driver.recvReturnIn`).
    pub from_byte_stream_driver_return_out: OutputPort<dyn BufferSendPort>,
    /// `toByteStreamDriver` — synchronous send to the driver (sample
    /// connection: `adapter.toByteStreamDriver -> driver.$send`).
    pub to_byte_stream_driver_out: OutputPort<dyn ByteStreamSendPort>,
    /// C++ `std::atomic<bool> m_driverIsReady = {false}`.
    driver_is_ready: AtomicBool,
}

impl ByteStreamBufferAdapter {
    /// `DriverNotReady` — WARNING_LO, id 0, no arguments:
    /// "Buffer Driver not ready".
    pub const EVENTID_DRIVER_NOT_READY: FwEventIdType = 0;
    /// `DataSendError(stat: I32)` — WARNING_LO, id 1:
    /// "Data send error status: {}".
    pub const EVENTID_DATA_SEND_ERROR: FwEventIdType = 1;
    /// `DataReceiveError(stat: I32)` — WARNING_LO, id 2:
    /// "Data reception error status: {}".
    pub const EVENTID_DATA_RECEIVE_ERROR: FwEventIdType = 2;

    /// Construct the component (driver not ready).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            buffer_in_return_out: OutputPort::new(),
            buffer_out: OutputPort::new(),
            from_byte_stream_driver_return_out: OutputPort::new(),
            to_byte_stream_driver_out: OutputPort::new(),
            driver_is_ready: AtomicBool::new(false),
        })
    }

    /// Has the driver signalled `ready`? (C++ `m_driverIsReady`.)
    #[must_use]
    pub fn driver_is_ready(&self) -> bool {
        self.driver_is_ready.load(Ordering::SeqCst)
    }

    fn id_base(&self) -> FwIdType {
        self.base.get_id_base()
    }

    // -- Events ----------------------------------------------------------------

    fn log_warning_lo_driver_not_ready(&self) {
        self.evt.log_event(
            self.id_base(),
            Self::EVENTID_DRIVER_NOT_READY,
            LogSeverity::WarningLo,
            "Buffer Driver not ready",
            |_| SerializeStatus::Ok,
        );
    }

    /// `DataSendError` / `DataReceiveError`: the `I32` argument is the
    /// `ByteStreamStatus` value (C++ passes the enum to the `I32` parameter).
    fn log_status_event(&self, id: FwEventIdType, text: &str, status: ByteStreamStatus) {
        let stat = status as i32;
        self.evt.log_event(
            self.id_base(),
            id,
            LogSeverity::WarningLo,
            &format!("{text}{stat}"),
            |buf| buf.serialize_i32_be(stat),
        );
    }

    // -- Handlers --------------------------------------------------------------

    /// C++ `bufferIn_handler`: send to the driver when ready (logging a
    /// failed send), otherwise log `DriverNotReady`; then return the buffer
    /// at once either way ("ByteStreamDriver is Sync").
    ///
    /// Here and in every handler below, C++ reuses the input `portNum` as the
    /// output port index; each port is a single port, so that is always 0,
    /// and the Rust `OutputPort` invokes its connection's target port number
    /// instead (documented deviation, no observable difference).
    fn buffer_in_handler(&self, _port_num: FwIndexType, mut buffer: Buffer) {
        if self.driver_is_ready.load(Ordering::SeqCst) {
            let p = self.to_byte_stream_driver_out.get();
            let status = p.target.invoke(p.port_num, &mut buffer);
            if status != ByteStreamStatus::OpOk {
                self.log_status_event(
                    Self::EVENTID_DATA_SEND_ERROR,
                    "Data send error status: ",
                    status,
                );
            }
        } else {
            self.log_warning_lo_driver_not_ready();
        }
        let p = self.buffer_in_return_out.get();
        p.target.invoke(p.port_num, buffer);
    }

    /// C++ `bufferOutReturn_handler`: hand the received buffer back to the
    /// driver.
    fn buffer_out_return_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        let p = self.from_byte_stream_driver_return_out.get();
        p.target.invoke(p.port_num, buffer);
    }

    /// C++ `fromByteStreamDriver_handler`: `OP_OK` data goes to the client
    /// (it returns through `bufferOutReturn`); anything else is logged and
    /// returned to the driver at once.
    // The name is the FPP port name (`fromByteStreamDriver`), not a `from_*`
    // conversion.
    #[allow(clippy::wrong_self_convention)]
    fn from_byte_stream_driver_handler(
        &self,
        _port_num: FwIndexType,
        buffer: Buffer,
        status: ByteStreamStatus,
    ) {
        if status == ByteStreamStatus::OpOk {
            let p = self.buffer_out.get();
            p.target.invoke(p.port_num, buffer);
        } else {
            self.log_status_event(
                Self::EVENTID_DATA_RECEIVE_ERROR,
                "Data reception error status: ",
                status,
            );
            let p = self.from_byte_stream_driver_return_out.get();
            p.target.invoke(p.port_num, buffer);
        }
    }

    /// C++ `byteStreamDriverReady_handler`: latch the ready flag.
    fn byte_stream_driver_ready_handler(&self, _port_num: FwIndexType) {
        self.driver_is_ready.store(true, Ordering::SeqCst);
    }
}

fprime_comp::input_port_adapter! {
    /// `bufferIn` — SYNC `Fw.BufferSend` input: data from the client.
    component: ByteStreamBufferAdapter;
    adapter: BufferInAdapter;
    port: BufferSendPort;
    input: pub buffer_in;
    handler: buffer_in_handler;
    args { val buffer: Buffer }
}

fprime_comp::input_port_adapter! {
    /// `bufferOutReturn` — SYNC `Fw.BufferSend` input: the client returns
    /// buffers sent on `bufferOut`.
    component: ByteStreamBufferAdapter;
    adapter: BufferOutReturnAdapter;
    port: BufferSendPort;
    input: pub buffer_out_return_in;
    handler: buffer_out_return_handler;
    args { val buffer: Buffer }
}

fprime_comp::input_port_adapter! {
    /// `byteStreamDriverReady` — SYNC `Drv.ByteStreamReady` input (sample
    /// connection: `driver.ready -> adapter.byteStreamDriverReady`).
    component: ByteStreamBufferAdapter;
    adapter: ByteStreamDriverReadyAdapter;
    port: ByteStreamReadyPort;
    input: pub byte_stream_driver_ready_in;
    handler: byte_stream_driver_ready_handler;
    args {}
}

fprime_comp::input_port_adapter! {
    /// `fromByteStreamDriver` — SYNC `Drv.ByteStreamData` input: received
    /// data (sample connection: `driver.$recv -> adapter.fromByteStreamDriver`).
    component: ByteStreamBufferAdapter;
    adapter: FromByteStreamDriverAdapter;
    port: ByteStreamDataPort;
    input: pub from_byte_stream_driver_in;
    handler: from_byte_stream_driver_handler;
    args { val buffer: Buffer, val status: ByteStreamStatus }
}

#[cfg(test)]
mod tests {
    //! Port of `Drv/ByteStreamBufferAdapter/test/ut` (`Test_Stream_out`,
    //! `Test_Stream_in`, `Test_Stream_Return`, `Test_Driver_Ready_State`)
    //! plus literal-byte event checks. Upstream's `STest::Pick` random fill
    //! becomes a fixed-seed generator.
    use super::*;
    use fprime_comp::{LogPort, LogTextPort};
    use fprime_fw::{LogBuffer, TextLogString, Time};
    use std::sync::Mutex;

    // Upstream: FW_COM_BUFFER_MAX_SIZE * 10 + sizeof(U32) + sizeof(U32) +
    // sizeof(FwBuffSizeType).
    const DATA_SIZE: usize = fprime_config::FW_COM_BUFFER_MAX_SIZE * 10 + 4 + 4 + 2;
    const ID_BASE: FwIdType = 0x100;

    #[derive(Default)]
    struct EventCollector {
        events: Mutex<Vec<(FwEventIdType, LogSeverity, Vec<u8>)>>,
        texts: Mutex<Vec<String>>,
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

    impl LogTextPort for EventCollector {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            _id: FwEventIdType,
            _time_tag: &mut Time,
            _severity: LogSeverity,
            text: &mut TextLogString,
        ) {
            self.texts
                .lock()
                .unwrap()
                .push(text.as_str().unwrap_or_default().to_string());
        }
    }

    impl EventCollector {
        fn count_of(&self, local_id: FwEventIdType) -> usize {
            self.events
                .lock()
                .unwrap()
                .iter()
                .filter(|e| e.0 == ID_BASE + local_id)
                .count()
        }
        fn all(&self) -> Vec<(FwEventIdType, LogSeverity, Vec<u8>)> {
            self.events.lock().unwrap().clone()
        }
        fn clear(&self) {
            self.events.lock().unwrap().clear();
            self.texts.lock().unwrap().clear();
        }
    }

    /// One buffer output history (`from_<port>` in the C++ tester): data
    /// address, size and contents of each buffer received.
    #[derive(Default)]
    struct BufferHistory {
        entries: Mutex<Vec<(usize, usize, Vec<u8>)>>,
    }

    impl BufferSendPort for BufferHistory {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.entries.lock().unwrap().push(snapshot(&buffer));
        }
    }

    impl BufferHistory {
        fn size(&self) -> usize {
            self.entries.lock().unwrap().len()
        }
        fn get(&self, index: usize) -> (usize, usize, Vec<u8>) {
            self.entries.lock().unwrap()[index].clone()
        }
        fn clear(&self) {
            self.entries.lock().unwrap().clear();
        }
    }

    /// Stand-in synchronous driver on `toByteStreamDriver`: records the
    /// buffer and returns `OP_OK`, or `status` when set (C++
    /// `from_toByteStreamDriver_handler` with `m_byte_stream_driver_fail`
    /// returning `OTHER_ERROR`).
    #[derive(Default)]
    struct FakeDriver {
        history: BufferHistory,
        status: Mutex<Option<ByteStreamStatus>>,
    }

    impl ByteStreamSendPort for FakeDriver {
        fn invoke(&self, _port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus {
            self.history.entries.lock().unwrap().push(snapshot(buffer));
            self.status
                .lock()
                .unwrap()
                .unwrap_or(ByteStreamStatus::OpOk)
        }
    }

    impl FakeDriver {
        fn fail_with(&self, status: Option<ByteStreamStatus>) {
            *self.status.lock().unwrap() = status;
        }
    }

    struct Tester {
        component: Arc<ByteStreamBufferAdapter>,
        events: Arc<EventCollector>,
        driver: Arc<FakeDriver>,
        buffer_in_return: Arc<BufferHistory>,
        buffer_out: Arc<BufferHistory>,
        from_byte_stream_driver_return: Arc<BufferHistory>,
        rng: u64,
    }

    impl Tester {
        fn new() -> Self {
            let component = ByteStreamBufferAdapter::new("ByteStreamBufferAdapter");
            component.base.set_id_base(ID_BASE);
            let events = Arc::new(EventCollector::default());
            let driver = Arc::new(FakeDriver::default());
            let buffer_in_return = Arc::new(BufferHistory::default());
            let buffer_out = Arc::new(BufferHistory::default());
            let from_byte_stream_driver_return = Arc::new(BufferHistory::default());
            component.evt.log_out.connect(events.clone(), 0);
            component.evt.text_log_out.connect(events.clone(), 0);
            component
                .buffer_in_return_out
                .connect(buffer_in_return.clone(), 0);
            component.buffer_out.connect(buffer_out.clone(), 0);
            component
                .from_byte_stream_driver_return_out
                .connect(from_byte_stream_driver_return.clone(), 0);
            component
                .to_byte_stream_driver_out
                .connect(driver.clone(), 0);
            Self {
                component,
                events,
                driver,
                buffer_in_return,
                buffer_out,
                from_byte_stream_driver_return,
                rng: 0x5eed_0000_c0ff_ee02,
            }
        }

        /// `clearFromPortHistory()` + `clearHistory()`.
        fn clear_histories(&self) {
            self.driver.history.clear();
            self.buffer_in_return.clear();
            self.buffer_out.clear();
            self.from_byte_stream_driver_return.clear();
            self.events.clear();
        }

        fn next_random(&mut self) -> u64 {
            // xorshift64: deterministic stand-in for STest::Pick.
            self.rng ^= self.rng << 13;
            self.rng ^= self.rng >> 7;
            self.rng ^= self.rng << 17;
            self.rng
        }

        /// A `DATA_SIZE` buffer with a random-length random fill, sized to
        /// the fill (upstream `m_buffer.set(..)` + `random_fill` + `setSize`).
        fn random_buffer(&mut self) -> Buffer {
            let max = (DATA_SIZE - (4 + 4 + 2)) as u64;
            let size = (self.next_random() % (max + 1)) as usize;
            let mut buffer = Buffer::allocate(DATA_SIZE);
            assert!(buffer.size() >= size);
            for i in 0..size {
                buffer.data_mut()[i] = self.next_random() as u8;
            }
            buffer.set_size(size);
            buffer
        }

        fn buffer_in(&self, buffer: Buffer) {
            self.component.buffer_in(0).target.invoke(0, buffer);
        }

        fn ready(&self) {
            self.component
                .byte_stream_driver_ready_in(0)
                .target
                .invoke(0);
        }
    }

    fn snapshot(buffer: &Buffer) -> (usize, usize, Vec<u8>) {
        (
            buffer.data().as_ptr() as usize,
            buffer.size(),
            buffer.data().to_vec(),
        )
    }

    /// Upstream `Nominal.Test_Stream_out`.
    #[test]
    fn send_path_gates_on_ready_and_returns_every_buffer() {
        let mut t = Tester::new();
        t.clear_histories();

        // Driver not ready by default: no data flow, buffer returned at once.
        let buffer = t.random_buffer();
        let sent = snapshot(&buffer);
        t.buffer_in(buffer);
        assert_eq!(t.driver.history.size(), 0);
        assert_eq!(
            t.events
                .count_of(ByteStreamBufferAdapter::EVENTID_DRIVER_NOT_READY),
            1
        );
        assert_eq!(t.buffer_in_return.size(), 1);
        assert_eq!(t.buffer_in_return.get(0), sent);

        // Driver ready: data flows.
        t.ready();
        t.clear_histories();
        let buffer = t.random_buffer();
        let sent = snapshot(&buffer);
        t.buffer_in(buffer);
        assert_eq!(t.driver.history.size(), 1);
        assert_eq!(t.buffer_in_return.size(), 1);
        assert_eq!(t.driver.history.get(0), sent);
        // The same buffer comes back once the synchronous send returns.
        assert_eq!(t.buffer_in_return.get(0), sent);
        assert_eq!(
            t.events
                .count_of(ByteStreamBufferAdapter::EVENTID_DRIVER_NOT_READY),
            0
        );

        // Forced driver failure.
        t.clear_histories();
        t.driver.fail_with(Some(ByteStreamStatus::OtherError));
        let buffer = t.random_buffer();
        let sent = snapshot(&buffer);
        t.buffer_in(buffer);
        assert_eq!(
            t.events
                .count_of(ByteStreamBufferAdapter::EVENTID_DATA_SEND_ERROR),
            1
        );
        // ASSERT_EVENTS_DataSendError(0, 3): OTHER_ERROR as an I32.
        assert_eq!(
            t.events.all()[0],
            (
                ID_BASE + ByteStreamBufferAdapter::EVENTID_DATA_SEND_ERROR,
                LogSeverity::WarningLo,
                vec![0, 0, 0, 3]
            )
        );
        assert_eq!(
            t.events
                .count_of(ByteStreamBufferAdapter::EVENTID_DRIVER_NOT_READY),
            0
        );
        assert_eq!(t.driver.history.size(), 1);
        assert_eq!(t.buffer_in_return.size(), 1);
        assert_eq!(t.buffer_in_return.get(0), sent);
        t.driver.fail_with(None);
    }

    /// Upstream `Nominal.Test_Stream_in`.
    #[test]
    fn receive_path_delivers_ok_data_and_returns_errors_to_the_driver() {
        let mut t = Tester::new();
        t.clear_histories();
        t.ready();

        // Faulty data from the driver: nothing out, buffer returned.
        let buffer = t.random_buffer();
        let received = snapshot(&buffer);
        t.component.from_byte_stream_driver_in(0).target.invoke(
            0,
            buffer,
            ByteStreamStatus::RecvNoData,
        );
        assert_eq!(t.buffer_out.size(), 0);
        assert_eq!(
            t.events
                .count_of(ByteStreamBufferAdapter::EVENTID_DATA_RECEIVE_ERROR),
            1
        );
        assert_eq!(t.from_byte_stream_driver_return.size(), 1);
        assert_eq!(t.from_byte_stream_driver_return.get(0), received);

        // Good data: delivered, NOT returned to the driver.
        t.clear_histories();
        let buffer = t.random_buffer();
        let received = snapshot(&buffer);
        t.component
            .from_byte_stream_driver_in(0)
            .target
            .invoke(0, buffer, ByteStreamStatus::OpOk);
        assert_eq!(
            t.events
                .count_of(ByteStreamBufferAdapter::EVENTID_DATA_RECEIVE_ERROR),
            0
        );
        assert_eq!(t.buffer_out.size(), 1);
        assert_eq!(t.buffer_out.get(0), received);
        assert_eq!(t.from_byte_stream_driver_return.size(), 0);
    }

    /// Upstream `Nominal.Test_Stream_Return`.
    #[test]
    fn buffer_out_return_goes_back_to_the_driver() {
        let mut t = Tester::new();
        t.clear_histories();

        let buffer = t.random_buffer();
        let sent = snapshot(&buffer);
        t.component.buffer_out_return_in(0).target.invoke(0, buffer);

        // Must return the buffer regardless of the driver ready state.
        assert!(!t.component.driver_is_ready());
        assert_eq!(t.from_byte_stream_driver_return.size(), 1);
        assert_eq!(t.from_byte_stream_driver_return.get(0), sent);
    }

    /// Upstream `Nominal.Test_Driver_Ready_State`.
    #[test]
    fn driver_ready_state_gates_the_send() {
        let mut t = Tester::new();
        t.clear_histories();

        // Initial state: not ready.
        let buffer = t.random_buffer();
        let sent = snapshot(&buffer);
        t.buffer_in(buffer);
        assert_eq!(t.driver.history.size(), 0);
        assert_eq!(
            t.events
                .count_of(ByteStreamBufferAdapter::EVENTID_DRIVER_NOT_READY),
            1
        );
        assert_eq!(t.buffer_in_return.size(), 1);
        assert_eq!(t.buffer_in_return.get(0), sent);

        t.clear_histories();
        t.ready();
        let buffer = t.random_buffer();
        let sent = snapshot(&buffer);
        t.buffer_in(buffer);
        assert_eq!(t.driver.history.size(), 1);
        assert_eq!(
            t.events
                .count_of(ByteStreamBufferAdapter::EVENTID_DRIVER_NOT_READY),
            0
        );
        assert_eq!(t.buffer_in_return.size(), 1);
        assert_eq!(t.buffer_in_return.get(0), sent);
    }

    // -- Beyond the upstream suite ---------------------------------------------

    #[test]
    fn event_ids_severities_and_payloads_match_the_fpp() {
        let t = Tester::new();
        t.buffer_in(Buffer::allocate(4));
        t.component.from_byte_stream_driver_in(0).target.invoke(
            0,
            Buffer::allocate(4),
            ByteStreamStatus::OtherError,
        );
        t.ready();
        t.driver.fail_with(Some(ByteStreamStatus::SendRetry));
        t.buffer_in(Buffer::allocate(4));
        assert_eq!(
            t.events.all(),
            vec![
                (ID_BASE, LogSeverity::WarningLo, vec![]),
                (ID_BASE + 2, LogSeverity::WarningLo, vec![0, 0, 0, 3]),
                (ID_BASE + 1, LogSeverity::WarningLo, vec![0, 0, 0, 1]),
            ]
        );
        assert_eq!(
            *t.events.texts.lock().unwrap(),
            [
                "Buffer Driver not ready",
                "Data reception error status: 3",
                "Data send error status: 1",
            ]
        );
    }

    #[test]
    fn every_non_ok_send_status_is_reported_and_returned_to_the_client() {
        let t = Tester::new();
        t.ready();
        for (i, status) in [
            ByteStreamStatus::SendRetry,
            ByteStreamStatus::RecvNoData,
            ByteStreamStatus::OtherError,
        ]
        .into_iter()
        .enumerate()
        {
            t.driver.fail_with(Some(status));
            t.buffer_in(Buffer::allocate(8));
            assert_eq!(t.driver.history.size(), i + 1);
            assert_eq!(t.buffer_in_return.size(), i + 1);
            let (_, _, payload) = t.events.all()[i].clone();
            assert_eq!(payload, (status as i32).to_be_bytes());
        }
        // OP_OK: returned silently.
        t.driver.fail_with(None);
        t.buffer_in(Buffer::allocate(8));
        assert_eq!(t.buffer_in_return.size(), 4);
        assert_eq!(t.events.all().len(), 3);
        assert_eq!(t.buffer_out.size(), 0);
        assert_eq!(t.from_byte_stream_driver_return.size(), 0);
    }

    #[test]
    fn every_non_ok_receive_status_is_returned_to_the_driver() {
        let t = Tester::new();
        for status in [
            ByteStreamStatus::SendRetry,
            ByteStreamStatus::RecvNoData,
            ByteStreamStatus::OtherError,
        ] {
            t.component
                .from_byte_stream_driver_in(0)
                .target
                .invoke(0, Buffer::allocate(8), status);
        }
        assert_eq!(t.buffer_out.size(), 0);
        assert_eq!(t.from_byte_stream_driver_return.size(), 3);
        assert_eq!(
            t.events
                .count_of(ByteStreamBufferAdapter::EVENTID_DATA_RECEIVE_ERROR),
            3
        );
        // Receive does not depend on the ready flag (C++ parity).
        assert!(!t.component.driver_is_ready());
    }

    #[test]
    fn ready_is_latched_and_never_cleared() {
        let t = Tester::new();
        assert!(!t.component.driver_is_ready());
        t.ready();
        t.ready();
        assert!(t.component.driver_is_ready());
        // A failed send does not clear it.
        t.driver.fail_with(Some(ByteStreamStatus::OtherError));
        t.buffer_in(Buffer::allocate(4));
        assert!(t.component.driver_is_ready());
        t.driver.fail_with(None);
        t.buffer_in(Buffer::allocate(4));
        assert_eq!(t.driver.history.size(), 2);
        assert_eq!(
            t.events
                .count_of(ByteStreamBufferAdapter::EVENTID_DRIVER_NOT_READY),
            0
        );
    }

    #[test]
    fn connects_to_a_sync_driver_without_shims() {
        // GenericHub's sample transport: a TCP driver behind this adapter.
        let adapter = ByteStreamBufferAdapter::new("adapter");
        let client = crate::TcpClient::new("client");
        adapter
            .to_byte_stream_driver_out
            .connect_to(client.send_in(0));
        adapter
            .from_byte_stream_driver_return_out
            .connect_to(client.recv_return_in(0));
        client
            .ready_out
            .connect_to(adapter.byte_stream_driver_ready_in(0));
        client
            .recv_out
            .connect_to(adapter.from_byte_stream_driver_in(0));
        assert!(adapter.to_byte_stream_driver_out.is_connected());
        assert!(adapter.from_byte_stream_driver_return_out.is_connected());
    }

    #[test]
    fn unconnected_event_ports_are_optional() {
        let component = ByteStreamBufferAdapter::new("bare");
        let returned = Arc::new(BufferHistory::default());
        component.buffer_in_return_out.connect(returned.clone(), 0);
        component.buffer_in(0).target.invoke(0, Buffer::allocate(4));
        assert_eq!(returned.size(), 1);
    }

    #[test]
    #[should_panic]
    fn unconnected_driver_send_asserts_once_ready() {
        let component = ByteStreamBufferAdapter::new("bare");
        let returned = Arc::new(BufferHistory::default());
        component.buffer_in_return_out.connect(returned, 0);
        component.byte_stream_driver_ready_in(0).target.invoke(0);
        component.buffer_in(0).target.invoke(0, Buffer::allocate(4));
    }

    #[test]
    #[should_panic]
    fn unconnected_buffer_out_asserts() {
        let component = ByteStreamBufferAdapter::new("bare");
        component.from_byte_stream_driver_in(0).target.invoke(
            0,
            Buffer::allocate(4),
            ByteStreamStatus::OpOk,
        );
    }
}
