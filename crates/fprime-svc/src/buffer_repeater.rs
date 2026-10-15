//! # BufferRepeater — port of `Svc::BufferRepeater` (passive)
//!
//! C++ sources: `Svc/BufferRepeater/BufferRepeater.{cpp,hpp,fpp}`,
//! `Svc/BufferRepeater/docs/sdd.md`.
//!
//! Repeats one `Fw.BufferSend` call to every connected `portOut`: for each
//! connected index it allocates a buffer the size of the incoming one,
//! copies the bytes in and sends the copy out; then it deallocates the
//! incoming buffer. A failed allocation (no storage, or smaller than the
//! incoming buffer) skips that index and is reported as configured with
//! [`BufferRepeater::configure`]: not at all, as a WARNING_HI or as a FATAL
//! event. An allocation that has storage but is too small goes back on
//! `deallocate`.
//!
//! C++ parity kept: the component is unusable until configured — `portIn`
//! asserts while the response is still
//! [`BufferRepeaterFailureOption::NumBufferRepeaterFailureOptions`] (the
//! constructor's "uninitialized" marker); allocations always use
//! `allocate[0]` and both the copies and the original return through
//! `deallocate[0]`; and the events' format strings name the port first
//! and the size second ("Failed to allocate {port} byte buffer for port
//! {size}", as the FPP format substitutes the arguments in declaration
//! order).
//!
//! Rust ownership: `Fw::Buffer` moves (see `fprime_comp::port`), so the
//! original moves into `deallocate` and each copy into its `portOut`. "No
//! storage" (C++ `getData() == nullptr`) is a buffer with zero capacity.

use fprime_comp::{BufferGetPort, BufferSendPort, EventGlue, OutputPort, PassiveBase, PortRef};
pub use fprime_config::BUFFER_REPEATER_OUTPUT_PORTS;
use fprime_config::{FwEventIdType, FwIndexType, FwSizeType};
use fprime_fw::sync::{Arc, AtomicU8, Ordering};
use fprime_fw::{Buffer, LogSeverity, SerBuf, SerializeStatus, fw_assert, fw_try};

/// C++ `BufferRepeater::BufferRepeaterFailureOption`: the response to a
/// failed allocation. Discriminants are the C++ values.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferRepeaterFailureOption {
    /// Continue regardless of allocation failures.
    NoResponseOnOutOfMemory = 0,
    /// Emit `AllocationSoftFailure` (WARNING_HI) on allocation failures.
    WarningOnOutOfMemory = 1,
    /// Emit `AllocationHardFailure` (FATAL) on allocation failures.
    FatalOnOutOfMemory = 2,
    /// Number of options; the constructor's "not configured" marker.
    NumBufferRepeaterFailureOptions = 3,
}

impl BufferRepeaterFailureOption {
    fn from_u8(raw: u8) -> Self {
        match raw {
            0 => Self::NoResponseOnOutOfMemory,
            1 => Self::WarningOnOutOfMemory,
            2 => Self::FatalOnOutOfMemory,
            _ => Self::NumBufferRepeaterFailureOptions,
        }
    }
}

/// `Svc::BufferRepeater`.
pub struct BufferRepeater {
    /// Component core (name / id_base / instance).
    pub base: PassiveBase,
    /// `Log` / `LogText` / `Time` ports.
    pub evt: EventGlue,
    /// `portOut` — the duplicated outputs (`Fw.BufferSend`).
    pub port_out: [OutputPort<dyn BufferSendPort>; BUFFER_REPEATER_OUTPUT_PORTS],
    /// `allocate` — memory for each copy (`Fw.BufferGet`).
    pub allocate: OutputPort<dyn BufferGetPort>,
    /// `deallocate` — returns the original buffer (and unusable
    /// allocations) (`Fw.BufferSend`).
    pub deallocate: OutputPort<dyn BufferSendPort>,
    /// `m_allocation_failure_response`, set by `configure` at setup (a
    /// plain member in C++; a relaxed atomic here).
    allocation_failure_response: AtomicU8,
}

impl BufferRepeater {
    /// `AllocationSoftFailure` event id (WARNING_HI).
    pub const EVENTID_ALLOCATION_SOFT_FAILURE: FwEventIdType = 0;
    /// `AllocationHardFailure` event id (FATAL).
    pub const EVENTID_ALLOCATION_HARD_FAILURE: FwEventIdType = 1;

    /// Construct, not yet configured (C++ constructor: the response is
    /// `NUM_BUFFER_REPEATER_FAILURE_OPTIONS`, so `portIn` asserts until
    /// [`configure`](Self::configure) is called).
    pub fn new(name: &str) -> Arc<Self> {
        Arc::new(Self {
            base: PassiveBase::new(name),
            evt: EventGlue::new(),
            port_out: core::array::from_fn(|_| OutputPort::new()),
            allocate: OutputPort::new(),
            deallocate: OutputPort::new(),
            allocation_failure_response: AtomicU8::new(
                BufferRepeaterFailureOption::NumBufferRepeaterFailureOptions as u8,
            ),
        })
    }

    /// C++ `configure(allocation_failure_response)`.
    pub fn configure(&self, allocation_failure_response: BufferRepeaterFailureOption) {
        self.allocation_failure_response
            .store(allocation_failure_response as u8, Ordering::Relaxed);
    }

    /// The configured response.
    pub fn allocation_failure_response(&self) -> BufferRepeaterFailureOption {
        BufferRepeaterFailureOption::from_u8(
            self.allocation_failure_response.load(Ordering::Relaxed),
        )
    }

    /// `portIn` — SYNC `Fw.BufferSend` input.
    pub fn port_in(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn BufferSendPort> {
        PortRef::new(self.clone(), port_num)
    }

    /// C++ `check_allocation`: is `new_allocation` usable for a copy of
    /// `incoming`? Reports a failure as configured.
    fn check_allocation(&self, index: usize, new_allocation: &Buffer, incoming: &Buffer) -> bool {
        fw_assert!(index < BUFFER_REPEATER_OUTPUT_PORTS, index as i32);
        let is_valid = new_allocation.capacity() > 0 && new_allocation.size() >= incoming.size();
        if !is_valid {
            match self.allocation_failure_response() {
                BufferRepeaterFailureOption::NoResponseOnOutOfMemory => {}
                BufferRepeaterFailureOption::WarningOnOutOfMemory => {
                    self.log_allocation_failure(
                        Self::EVENTID_ALLOCATION_SOFT_FAILURE,
                        LogSeverity::WarningHi,
                        index as i32,
                        incoming.size() as FwSizeType,
                    );
                }
                BufferRepeaterFailureOption::FatalOnOutOfMemory => {
                    self.log_allocation_failure(
                        Self::EVENTID_ALLOCATION_HARD_FAILURE,
                        LogSeverity::Fatal,
                        index as i32,
                        incoming.size() as FwSizeType,
                    );
                }
                BufferRepeaterFailureOption::NumBufferRepeaterFailureOptions => {
                    fw_assert!(false);
                }
            }
        }
        is_valid
    }

    /// `log_WARNING_HI_AllocationSoftFailure` /
    /// `log_FATAL_AllocationHardFailure`: `(port: I32, size: FwSizeType)`.
    fn log_allocation_failure(
        &self,
        id: FwEventIdType,
        severity: LogSeverity,
        port: i32,
        size: FwSizeType,
    ) {
        // C++ parity: the FPP format substitutes in declaration order, so
        // the port lands where the text says "byte".
        self.evt.log_event(
            self.base.get_id_base(),
            id,
            severity,
            &alloc::format!("Failed to allocate {port} byte buffer for port {size}"),
            |buf| -> SerializeStatus {
                fw_try!(buf.serialize_i32_be(port));
                buf.serialize_u64_be(size)
            },
        );
    }

    /// `portIn_handler`.
    fn port_in_handler(&self, _port_num: FwIndexType, buffer: Buffer) {
        let response = self.allocation_failure_response();
        fw_assert!(
            response != BufferRepeaterFailureOption::NumBufferRepeaterFailureOptions,
            response as i32
        );
        for (i, out) in self.port_out.iter().enumerate() {
            let Some(p_out) = out.try_get() else {
                continue;
            };
            let p = self.allocate.get();
            let mut new_allocation = p.target.invoke(p.port_num, buffer.size() as FwSizeType);
            if self.check_allocation(i, &new_allocation, &buffer) {
                let n = buffer.size();
                new_allocation.data_mut()[..n].copy_from_slice(buffer.data());
                new_allocation.set_size(n);
                p_out.target.invoke(p_out.port_num, new_allocation);
            } else if new_allocation.capacity() > 0 {
                let p = self.deallocate.get();
                p.target.invoke(p.port_num, new_allocation);
            }
        }
        let p = self.deallocate.get();
        p.target.invoke(p.port_num, buffer);
    }
}

impl BufferSendPort for BufferRepeater {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        self.port_in_handler(port_num, buffer);
    }
}

#[cfg(test)]
mod tests {
    //! Port of `Svc/BufferRepeater/test/ut` (`BufferRepeaterTester`), plus
    //! the paths upstream leaves untested (a too-small allocation, partial
    //! wiring, the unconfigured assert).
    use super::*;
    use fprime_comp::LogPort;
    use fprime_config::FwEventIdType;
    use fprime_fw::{LogBuffer, Time};
    use std::sync::Mutex;
    use std::vec::Vec;

    #[derive(Clone, Copy, PartialEq)]
    enum AllocMode {
        Ok,
        Fail,
        TooSmall,
    }

    struct Tester {
        mode: Mutex<AllocMode>,
        allocations: Mutex<Vec<FwSizeType>>,
        deallocated: Mutex<Vec<Vec<u8>>>,
        out: Mutex<Vec<(FwIndexType, Vec<u8>)>>,
        events: Mutex<Vec<(FwEventIdType, LogSeverity, i32, u64)>>,
    }

    impl Tester {
        fn new(mode: AllocMode) -> Arc<Self> {
            Arc::new(Self {
                mode: Mutex::new(mode),
                allocations: Mutex::new(Vec::new()),
                deallocated: Mutex::new(Vec::new()),
                out: Mutex::new(Vec::new()),
                events: Mutex::new(Vec::new()),
            })
        }
    }

    /// `from_allocate_handler`.
    impl BufferGetPort for Tester {
        fn invoke(&self, _port_num: FwIndexType, size: FwSizeType) -> Buffer {
            self.allocations.lock().unwrap().push(size);
            match *self.mode.lock().unwrap() {
                AllocMode::Ok => Buffer::allocate(size as usize),
                AllocMode::Fail => Buffer::empty(),
                AllocMode::TooSmall => Buffer::allocate(size as usize - 1),
            }
        }
    }

    /// `from_deallocate_handler`.
    struct Dealloc(Arc<Tester>);
    impl BufferSendPort for Dealloc {
        fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
            self.0
                .deallocated
                .lock()
                .unwrap()
                .push(buffer.data().to_vec());
        }
    }

    /// `from_portOut_handler`.
    impl BufferSendPort for Tester {
        fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
            assert!(buffer.capacity() > 0, "Passed invalid buffer out port");
            self.out
                .lock()
                .unwrap()
                .push((port_num, buffer.data().to_vec()));
        }
    }

    impl LogPort for Tester {
        fn invoke(
            &self,
            _port_num: FwIndexType,
            id: FwEventIdType,
            _time_tag: &mut Time,
            severity: LogSeverity,
            args: &mut LogBuffer,
        ) {
            let mut port = 0i32;
            let mut size = 0u64;
            assert!(args.deserialize_i32_be(&mut port).is_ok());
            assert!(args.deserialize_u64_be(&mut size).is_ok());
            assert_eq!(args.deserialize_size_left(), 0);
            self.events.lock().unwrap().push((id, severity, port, size));
        }
    }

    fn harness(mode: AllocMode, connected: usize) -> (Arc<BufferRepeater>, Arc<Tester>) {
        let comp = BufferRepeater::new("BufferRepeater");
        let tester = Tester::new(mode);
        comp.evt.log_out.connect(tester.clone(), 0);
        comp.allocate.connect(tester.clone(), 0);
        comp.deallocate
            .connect(Arc::new(Dealloc(tester.clone())), 0);
        for i in 0..connected {
            comp.port_out[i].connect(tester.clone(), i as FwIndexType);
        }
        (comp, tester)
    }

    fn initial_buffer() -> (Buffer, Vec<u8>) {
        let mut buffer = Buffer::allocate(1024);
        for (i, byte) in buffer.data_mut().iter_mut().enumerate() {
            *byte = i as u8;
        }
        let bytes = buffer.data().to_vec();
        (buffer, bytes)
    }

    fn send(comp: &Arc<BufferRepeater>, buffer: Buffer) {
        let p = comp.port_in(0);
        p.target.invoke(p.port_num, buffer);
    }

    #[test]
    fn repeater_copies_to_every_port_and_deallocates_the_original() {
        // TEST(Nominal, TestRepeater)
        let (comp, tester) = harness(AllocMode::Ok, BUFFER_REPEATER_OUTPUT_PORTS);
        comp.configure(BufferRepeaterFailureOption::FatalOnOutOfMemory);
        let (buffer, bytes) = initial_buffer();
        send(&comp, buffer);
        assert!(tester.events.lock().unwrap().is_empty());
        let out = tester.out.lock().unwrap();
        assert_eq!(out.len(), BUFFER_REPEATER_OUTPUT_PORTS);
        for (i, (port, data)) in out.iter().enumerate() {
            assert_eq!(*port as usize, i);
            assert_eq!(*data, bytes, "data not copied correctly to port {i}");
        }
        assert_eq!(
            *tester.allocations.lock().unwrap(),
            vec![1024; BUFFER_REPEATER_OUTPUT_PORTS]
        );
        assert_eq!(*tester.deallocated.lock().unwrap(), vec![bytes]);
    }

    fn failure_case(option: BufferRepeaterFailureOption) -> Arc<Tester> {
        let (comp, tester) = harness(AllocMode::Fail, BUFFER_REPEATER_OUTPUT_PORTS);
        comp.configure(option);
        let (buffer, bytes) = initial_buffer();
        send(&comp, buffer);
        assert!(tester.out.lock().unwrap().is_empty());
        assert_eq!(*tester.deallocated.lock().unwrap(), vec![bytes]);
        tester
    }

    #[test]
    fn no_memory_response() {
        // TEST(OffNominal, NoMemoryResponse)
        let tester = failure_case(BufferRepeaterFailureOption::NoResponseOnOutOfMemory);
        assert!(tester.events.lock().unwrap().is_empty());
    }

    #[test]
    fn warning_memory_response() {
        // TEST(OffNominal, WarningMemoryResponse)
        let tester = failure_case(BufferRepeaterFailureOption::WarningOnOutOfMemory);
        let expected: Vec<_> = (0..BUFFER_REPEATER_OUTPUT_PORTS as i32)
            .map(|i| {
                (
                    BufferRepeater::EVENTID_ALLOCATION_SOFT_FAILURE,
                    LogSeverity::WarningHi,
                    i,
                    1024,
                )
            })
            .collect();
        assert_eq!(*tester.events.lock().unwrap(), expected);
    }

    #[test]
    fn fatal_memory_response() {
        // TEST(OffNominal, FatalMemoryResponse)
        let tester = failure_case(BufferRepeaterFailureOption::FatalOnOutOfMemory);
        let events = tester.events.lock().unwrap();
        assert_eq!(events.len(), BUFFER_REPEATER_OUTPUT_PORTS);
        assert!(
            events
                .iter()
                .all(|e| e.0 == BufferRepeater::EVENTID_ALLOCATION_HARD_FAILURE
                    && e.1 == LogSeverity::Fatal)
        );
    }

    #[test]
    fn too_small_allocations_are_deallocated() {
        let (comp, tester) = harness(AllocMode::TooSmall, 2);
        comp.configure(BufferRepeaterFailureOption::WarningOnOutOfMemory);
        let (buffer, bytes) = initial_buffer();
        send(&comp, buffer);
        assert!(tester.out.lock().unwrap().is_empty());
        let deallocated = tester.deallocated.lock().unwrap();
        // the two unusable allocations, then the original
        assert_eq!(deallocated.len(), 3);
        assert_eq!(deallocated[0].len(), 1023);
        assert_eq!(deallocated[1].len(), 1023);
        assert_eq!(deallocated[2], bytes);
        assert_eq!(tester.events.lock().unwrap().len(), 2);
    }

    #[test]
    fn only_connected_ports_get_copies() {
        let (comp, tester) = harness(AllocMode::Ok, 3);
        comp.configure(BufferRepeaterFailureOption::NoResponseOnOutOfMemory);
        let (buffer, bytes) = initial_buffer();
        send(&comp, buffer);
        let ports: Vec<_> = tester.out.lock().unwrap().iter().map(|e| e.0).collect();
        assert_eq!(ports, vec![0, 1, 2]);
        assert_eq!(tester.allocations.lock().unwrap().len(), 3);
        assert_eq!(*tester.deallocated.lock().unwrap(), vec![bytes]);
    }

    #[test]
    fn a_larger_allocation_is_trimmed_to_the_incoming_size() {
        let (comp, tester) = harness(AllocMode::Ok, 1);
        comp.configure(BufferRepeaterFailureOption::FatalOnOutOfMemory);
        let mut buffer = Buffer::allocate(16);
        buffer.data_mut().copy_from_slice(&[7; 16]);
        buffer.set_size(5);
        send(&comp, buffer);
        assert_eq!(*tester.out.lock().unwrap(), vec![(0, vec![7; 5])]);
        assert_eq!(*tester.allocations.lock().unwrap(), vec![5]);
    }

    #[test]
    fn event_text_names_the_port_where_upstream_does() {
        use fprime_comp::LogTextPort;
        use fprime_fw::TextLogString;
        struct Text(Mutex<Vec<String>>);
        impl LogTextPort for Text {
            fn invoke(
                &self,
                _port_num: FwIndexType,
                _id: FwEventIdType,
                _time_tag: &mut Time,
                _severity: LogSeverity,
                text: &mut TextLogString,
            ) {
                self.0.lock().unwrap().push(text.to_string());
            }
        }
        let (comp, _tester) = harness(AllocMode::Fail, 2);
        let text = Arc::new(Text(Mutex::new(Vec::new())));
        comp.evt.text_log_out.connect(text.clone(), 0);
        comp.configure(BufferRepeaterFailureOption::WarningOnOutOfMemory);
        send(&comp, Buffer::allocate(64));
        assert_eq!(
            *text.0.lock().unwrap(),
            vec![
                "Failed to allocate 0 byte buffer for port 64",
                "Failed to allocate 1 byte buffer for port 64"
            ]
        );
    }

    #[test]
    #[should_panic]
    fn unconfigured_component_asserts() {
        let (comp, _tester) = harness(AllocMode::Ok, 1);
        send(&comp, Buffer::allocate(4));
    }

    #[test]
    fn failure_option_discriminants_match_cpp() {
        assert_eq!(
            BufferRepeaterFailureOption::NoResponseOnOutOfMemory as u8,
            0
        );
        assert_eq!(BufferRepeaterFailureOption::WarningOnOutOfMemory as u8, 1);
        assert_eq!(BufferRepeaterFailureOption::FatalOnOutOfMemory as u8, 2);
        assert_eq!(
            BufferRepeaterFailureOption::NumBufferRepeaterFailureOptions as u8,
            3
        );
    }
}
